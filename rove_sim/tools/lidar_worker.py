#!/usr/bin/env python3
"""lidar_worker: a decoupled Livox raycast process (same idea as cam_worker).

pybullet's rayTestBatch is GIL-bound and ~half a wall-second of work for two
Mid-360s at 10 Hz -- which drags the single-process sim below realtime. So the
authoritative sim publishes its robot pose+joints to a shared file, and THIS
process (its own GIL) mirrors that robot, raycasts its assigned Livox(es) and
publishes the point clouds over UDP -- in parallel with the physics + camera
workers, so the sim stays realtime.

    tools/lidar_worker.py --terrain --lidars livox_top,livox_bottom --hz 10
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p

from rove_sim import runtime
from rove_sim.world.render_sync import (apply_robot_state, read_robot_state,
                                        DEFAULT_STATE_FILE)
from rove_sim.robot.occluder import sync_occluders
from rove_sim.sensors.lidar import LidarUdpPublisher
from rove_sim.transport.udp import DEFAULT_PORTS


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--terrain", nargs="?",
                    const="../free_dirt_road_through_forest.glb", default=None)
    ap.add_argument("--no-texture", action="store_true")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--lidars", default="livox_top,livox_bottom",
                    help="comma list of Livox sensor names this worker raycasts")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--rays", type=int, default=10000, help="rays/scan per Livox")
    ap.add_argument("--threads", type=int, default=0,
                    help="rayTestBatch threads (0=all cores; cap to leave cores "
                         "for physics when running parallel lidar workers)")
    ap.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    from rove_sim.core.util import die_with_parent
    die_with_parent()                    # no orphans if the fleet launcher is killed

    overrides = {"friction": {"origin": (-25., -25.), "extent": (50., 50.), "cell": 0.25}}
    if args.terrain:
        overrides["terrain"] = {"source": args.terrain, "texture": not args.no_texture}
    # render-only build: gives us the world + robot + the SAME Livox setup (ray_mask,
    # collision groups, self-occluders) as the real sim, so self-occlusion matches.
    # rayTestBatch is CPU; this process never renders -> egl=False, no GPU/VRAM.
    sim = runtime.build(args.profile, mode="headless", world="mock",
                        world_overrides=overrides, egl=False)
    p.setGravity(0, 0, 0)

    want = [n.strip() for n in args.lidars.split(",") if n.strip()]
    livox = [s for s in sim.sensors if s.name in want]
    if not livox:
        print(f"[lidar_worker] no Livox matched {want}", file=sys.stderr)
        sys.exit(2)
    for s in livox:
        if hasattr(s, "set_rays") and args.rays > 0:
            s.set_rays(min(args.rays, s.rays_per_scan))
        if args.threads > 0 and hasattr(s, "set_ray_threads"):
            s.set_ray_threads(args.threads)
    # Livox-style subscribable streams: bind the port, push to whoever registers.
    pubs = {s.name: LidarUdpPublisher(DEFAULT_PORTS.get(s.name, 5022))
            for s in livox}
    print(f"[lidar_worker] {', '.join(s.name for s in livox)} @ {args.hz:g}Hz "
          f"{args.rays} rays -> UDP {', '.join(str(DEFAULT_PORTS.get(s.name)) for s in livox)} "
          f"(subscribe to receive)")

    period = 1.0 / args.hz
    rt0 = time.time(); n = 0
    try:
        while True:
            t0 = time.time()
            apply_robot_state(sim.robot, read_robot_state(args.state_file))
            if sim.occluders:                       # pole self-occluder follows robot
                sync_occluders(sim.robot, sim.occluders)
            p.performCollisionDetection()           # refresh world after the resets
            for s in livox:
                pubs[s.name].publish(s.sample())
            n += 1
            if time.time() - rt0 >= 3.0:
                print(f"[lidar_worker] {n / (time.time() - rt0):.1f} Hz "
                      f"({len(livox)} Livox)")
                rt0 = time.time(); n = 0
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        for lp in pubs.values():
            lp.close()
        sim.disconnect()


if __name__ == "__main__":
    main()
