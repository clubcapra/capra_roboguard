#!/usr/bin/env python3
"""Mirror authoritative physics and stream alpha-aware Livox scans over UDP.

Both LiDARs share one static Embree BVH and local BVHs for moving robot links.
Textures, UVs and coverage thresholds are shared with the GPU renderer.
The legacy Bullet raycaster remains available with --raycaster bullet.
"""

import argparse
import json
from pathlib import Path
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p

from rove_sim.world.render_sync import (
    apply_robot_state,
    read_robot_state,
    DEFAULT_STATE_FILE,
)
from rove_sim.robot.occluder import sync_occluders
from rove_sim.sensors.lidar import LidarUdpPublisher, HaloLivoxPublisher
from rove_sim.transport.ports import PortMap
from rove_sim.transport.udp import DEFAULT_PORTS


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--metrics", default=None, help="Write measured scan-rate JSON")
    ap.add_argument("--profile", default="standard")
    ap.add_argument(
        "--world-config", default=None, help="World YAML (terrain and datum)"
    )
    ap.add_argument(
        "--scene",
        default=None,
        help="Saved scene JSON (objects, friction and initial pose)",
    )
    ap.add_argument(
        "--terrain",
        nargs="?",
        const="../free_dirt_road_through_forest.glb",
        default=None,
    )
    ap.add_argument("--no-texture", action="store_true")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument(
        "--lidars",
        default="livox_top,livox_bottom",
        help="comma list of Livox sensor names this worker raycasts",
    )
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument(
        "--motion-hz",
        type=float,
        default=60.0,
        help="Pose acquisition batches per second; divides the nominal scan ray budget",
    )
    ap.add_argument(
        "--pose-delay-ms",
        type=float,
        default=50.0,
        help="Buffered physics interpolation latency",
    )
    ap.add_argument("--rays", type=int, default=4000, help="rays/scan per Livox")
    ap.add_argument(
        "--threads",
        type=int,
        default=0,
        help="rayTestBatch threads (0=all cores; cap to leave cores "
        "for physics when running parallel lidar workers)",
    )
    ap.add_argument(
        "--raycaster",
        choices=("embree", "bullet"),
        default="embree",
        help="texture-aware Embree terrain rays, or legacy cutout-mesh Bullet rays",
    )
    ap.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    ap.add_argument(
        "--rsa-backend",
        action="store_true",
        help="Feed HALo's Livox inputs as LVXR instead of standalone LVX2 outputs",
    )
    ap.add_argument("--ports", default=None, help="Shared HALo/simulator ports.toml")
    from rove_sim.world.render_sync import apply_world_state

    args = ap.parse_args()
    if args.hz <= 0 or args.rays <= 0 or args.motion_hz <= 0 or args.pose_delay_ms < 0:
        ap.error("--hz and --rays must be positive")
    port_map = PortMap.load(args.ports) if args.rsa_backend else None
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    from rove_sim.core.util import die_with_parent

    die_with_parent()  # no orphans if the fleet launcher is killed

    from rove_sim.world.scene import build_configured_sim

    sim = build_configured_sim(
        args.profile,
        world_config=args.world_config,
        scene_path=args.scene,
        terrain=args.terrain,
        texture=not args.no_texture,
        egl=False,
        purpose="raycast" if args.raycaster == "embree" else "full",
    )
    p.setGravity(0, 0, 0)

    # Same obstacles as sim_server, so the lidar RAYCASTS against them (this worker
    # is a separate sim process). Static -> the synced robot pose is enough.
    from rove_sim.world.scene import spawn_obstacles_from_env

    spawn_obstacles_from_env(sim.world)

    want = [n.strip() for n in args.lidars.split(",") if n.strip()]
    livox = [s for s in sim.sensors if s.name in want]
    if not livox:
        print(f"[lidar_worker] no Livox matched {want}", file=sys.stderr)
        sys.exit(2)
    acquisition_hz = max(args.hz, args.motion_hz)
    batch_rays = max(1, round(args.rays * args.hz / acquisition_hz))
    for s in livox:
        if hasattr(s, "set_rays") and args.rays > 0:
            s.set_rays(min(batch_rays, s.rays_per_scan))
        if args.threads > 0 and hasattr(s, "set_ray_threads"):
            s.set_ray_threads(args.threads)
    if args.raycaster == "embree" and sim.world.terrain_manifest:
        from rove_sim.world.mesh_assets import load_terrain, AlphaRaycaster

        assets = load_terrain(
            sim.world.terrain_manifest, sim.world.spec["terrain"].get("scale", 1)
        )
        raycaster = AlphaRaycaster(assets)
        from rove_sim.world.moving_rays import MovingRaycaster

        moving = MovingRaycaster(sim.robot, sim.world)
        for s in livox:
            s.world_raycaster = raycaster
            s.moving_raycaster = moving
    pubs = {
        s.name: (
            HaloLivoxPublisher(
                args.host,
                port_map.backend_telemetry_ports()[
                    "livox_41" if "top" in s.name else "livox_40"
                ],
                41 if "top" in s.name else 40,
            )
            if port_map
            else LidarUdpPublisher(DEFAULT_PORTS.get(s.name, 5022))
        )
        for s in livox
    }
    print(
        f"[lidar_worker] {', '.join(s.name for s in livox)} @ {acquisition_hz:g}Hz acquisition "
        f"{batch_rays} rays/batch -> UDP {', '.join(str(pubs[s.name].port) for s in livox)} "
        f"({'HALo LVXR inputs' if port_map else 'standalone LVX2 subscription'})"
    )

    period = 1.0 / acquisition_hz
    rt0 = time.monotonic()
    n = 0
    started = rt0
    total = 0
    last_acquisition_ns = 0
    last_points = {}
    cloud_report = 0
    from rove_sim.render.layers import cloud_path, publish_cloud

    view_cloud = cloud_path(args.state_file)
    try:
        while True:
            t0 = time.monotonic()
            from rove_sim.world.motion import interpolate_state

            source = read_robot_state(args.state_file + ".history")
            state = interpolate_state(
                source, time.time_ns() - int(args.pose_delay_ms * 1e6)
            )
            if not state or time.time_ns() - state.get("timestamp_ns", 0) > 250_000_000:
                time.sleep(period)
                continue
            if state["timestamp_ns"] <= last_acquisition_ns:
                time.sleep(period)
                continue
            last_acquisition_ns = state["timestamp_ns"]
            apply_robot_state(sim.robot, state)
            apply_world_state(sim.world, state)
            if sim.occluders:  # pole self-occluder follows robot
                sync_occluders(sim.robot, sim.occluders)
            if livox[0].moving_raycaster is not None:
                livox[0].moving_raycaster.update()
            else:
                p.performCollisionDetection()
            scans = []
            for s in livox:
                scan = s.sample()
                scan.timestamp_ns = state["timestamp_ns"]
                scan.t = scan.timestamp_ns / 1e9
                scans.append(scan)
                last_points[s.name] = len(scan.points)
                pubs[s.name].publish(scan)
            if time.monotonic() - cloud_report >= 1.0 / 30:
                publish_cloud(view_cloud, scans)
                cloud_report = time.monotonic()
            n += 1
            total += 1
            now = time.monotonic()
            if now - rt0 >= 3.0:
                info = {
                    "hz_per_lidar": n / (now - rt0),
                    "scans_per_lidar": total,
                    "measured_seconds": now - started,
                    "rays_per_scan": batch_rays,
                    "nominal_scan_hz": args.hz,
                    "target_acquisition_hz": acquisition_hz,
                    "pose_delay_ms": args.pose_delay_ms,
                    "pose_age_ms": (time.time_ns() - state["timestamp_ns"]) / 1e6,
                    "points_last_scan": last_points,
                    "raycaster": args.raycaster,
                }
                print("[lidar_worker] " + json.dumps(info), flush=True)
                if args.metrics:
                    target = Path(args.metrics)
                    temp = target.with_name(target.name + f".{os.getpid()}.tmp")
                    temp.write_text(json.dumps(info))
                    os.replace(temp, target)
                rt0 = now
                n = 0
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        for lp in pubs.values():
            lp.close()
        sim.disconnect()


if __name__ == "__main__":
    main()
