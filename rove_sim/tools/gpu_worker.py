#!/usr/bin/env python3
"""Shared GPU camera renderer: original alpha, one scene copy, local live view."""

import argparse
import io
import json
import os
from pathlib import Path
import sys
import time
import subprocess
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import pybullet as p
from PIL import Image
from rove_sim.world.scene import build_configured_sim
from rove_sim.world.mesh_assets import load_terrain
from rove_sim.world.render_sync import (
    apply_robot_state,
    apply_world_state,
    read_robot_state,
    DEFAULT_STATE_FILE,
)
from rove_sim.render.gpu import GpuScene, equivalent_view
from rove_sim.render.layers import (
    LAYER_DEFAULTS,
    cloud_path,
    read_cloud,
    camera_frustum,
    radiation_geometry,
)
from rove_sim.sensors.rtsp import RtspCameraFeeds


def atomic(path, data):
    target = Path(path)
    temp = target.with_name(target.name + f".{os.getpid()}.tmp")
    temp.write_bytes(data)
    os.replace(temp, target)


def publish_view(path, rgb):
    stream = io.BytesIO()
    Image.fromarray(rgb).save(stream, format="JPEG", quality=85)
    atomic(path, stream.getvalue())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--world-config", default=None)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--terrain", default=None)
    ap.add_argument(
        "--cameras", default="cam_front,cam_rear,cam_left,cam_right,cam_arm"
    )
    ap.add_argument("--fps", type=float, default=30)
    ap.add_argument("--res", default="1920x1080")
    ap.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    ap.add_argument("--port", type=int, default=8554)
    ap.add_argument("--shared-server", action="store_true")
    ap.add_argument(
        "--encoder", choices=("auto", "libx264", "h264_nvenc"), default="auto"
    )
    ap.add_argument("--no-rtsp", action="store_true")
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--metrics", default=None)
    ap.add_argument("--view-image", default=None)
    ap.add_argument("--view-control", default=None)
    ap.add_argument(
        "--shadow-size",
        type=int,
        default=4096,
        help="Cached static shadow map size; 0 disables forest shadows",
    )
    args = ap.parse_args()
    if args.fps <= 0:
        ap.error("--fps must be positive")
    if not 0 <= args.shadow_size <= 8192:
        ap.error("--shadow-size must be between 0 and 8192")
    from rove_sim.core.util import die_with_parent

    die_with_parent()
    sim = build_configured_sim(
        args.profile,
        world_config=args.world_config,
        scene_path=args.scene,
        terrain=args.terrain,
        purpose="render",
        egl=False,
    )
    if not sim.world.terrain_manifest:
        raise ValueError("GPU forest rendering requires a configured terrain")
    assets = load_terrain(
        sim.world.terrain_manifest,
        sim.world.spec["terrain"].get("scale", 1),
        material_maps=True,
    )
    environment = dict(sim.world.spec.get("environment", {}))
    if environment.get("hdri"):
        path = Path(environment["hdri"])
        if not path.is_absolute() and args.world_config:
            path = Path(args.world_config).resolve().parent / path
        environment["hdri"] = str(path)
    gpu = GpuScene(
        assets, sim.robot, shadow_size=args.shadow_size, environment=environment or None
    )
    del assets
    w, h = map(int, args.res.split("x"))
    wanted = args.cameras.split(",")
    cams = [sim.sensor(name) for name in wanted]
    if any(cam is None for cam in cams):
        raise ValueError("Unknown camera in --cameras")
    if w <= 0 or h <= 0:
        ap.error("--res must have positive dimensions")
    for cam in cams:
        cam.width = w
        cam.height = h
        aspect = w / h
        vfov = np.degrees(2 * np.arctan(np.tan(np.radians(cam.hfov) / 2) / aspect))
        cam._proj = p.computeProjectionMatrixFOV(vfov, aspect, cam.near, cam.far)
    feeds = None
    if not args.no_rtsp:
        if args.encoder == "auto":
            probe = subprocess.run(
                [
                    "ffmpeg",
                    "-v",
                    "error",
                    "-f",
                    "lavfi",
                    "-i",
                    "color=size=64x64",
                    "-frames:v",
                    "1",
                    "-c:v",
                    "h264_nvenc",
                    "-f",
                    "null",
                    "-",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=15,
            )
            args.encoder = "h264_nvenc" if probe.returncode == 0 else "libx264"
        feeds = RtspCameraFeeds(
            cams,
            fps=args.fps,
            port=args.port,
            encoder=args.encoder,
            bitrate="8M" if w >= 1920 else "3M",
            manage_server=not args.shared_server,
            input_format="yuv420p",
        ).start()
    print(
        f"[gpu_worker] {len(cams)} cameras {w}x{h} @ {args.fps:g} fps, shared GPU scene",
        flush=True,
    )
    radiation_sources = sim.world.spec.get("radiation_sources", [])
    radiation_lines = radiation_geometry(radiation_sources)
    layer_info = {}
    frames = 0
    view_writer = ThreadPoolExecutor(max_workers=1) if args.view_image else None
    view_pending = None
    start = time.monotonic()
    report = start
    times = []
    stage_times = []
    seq = 0
    try:
        while not args.duration or time.monotonic() - start < args.duration:
            t0 = time.monotonic()
            state = read_robot_state(args.state_file)
            apply_robot_state(sim.robot, state)
            apply_world_state(sim.world, state)
            gpu.update_robot()
            gpu.update_sensors(sim.sensors)
            gpu.update_objects(sim.world)
            sync_done = time.monotonic()
            pending_frames = []
            for cam in cams:
                *_, view = cam._view()
                rgb = gpu.render(
                    view,
                    cam._proj,
                    w,
                    h,
                    stream=cam.name,
                    output="yuv420p" if feeds else "rgb",
                    defer=True,
                )
                encoder = feeds.streams[cam.name] if feeds else None
                if encoder:
                    if encoder.proc.poll() is not None:
                        raise RuntimeError(f"Encoder stopped for {cam.name}")
                pending_frames.append((encoder, rgb))
            submit_done = time.monotonic()
            if args.view_image:
                control = {"yaw": 50, "pitch": -25, "distance": 4}
                if args.view_control:
                    try:
                        control.update(json.loads(Path(args.view_control).read_text()))
                    except (OSError, ValueError):
                        pass
                target = np.asarray(state["pos"] if state else [0, 0, 0]) + [0, 0, 0.4]
                view = p.computeViewMatrixFromYawPitchRoll(
                    target.tolist(),
                    control["distance"],
                    control["yaw"],
                    control["pitch"],
                    0,
                    2,
                )
                proj = p.computeProjectionMatrixFOV(55, 4 / 3, 0.05, 100)
                layers = dict(LAYER_DEFAULTS, **control.get("layers", {}))
                points, cloud_age = (
                    read_cloud(cloud_path(args.state_file))
                    if layers["lidar"]
                    else (None, None)
                )
                line_groups = []
                if layers["cameras"]:
                    line_groups.extend(
                        camera_frustum(cam._view()[-1], cam._proj) for cam in cams
                    )
                if layers["radiation"]:
                    line_groups.append(radiation_lines)
                lines = np.concatenate(line_groups) if line_groups else None
                rgb = gpu.render(
                    view,
                    proj,
                    800,
                    600,
                    stream="observer",
                    scene=layers["scene"],
                    points=points,
                    lines=lines,
                )
                layer_info = dict(
                    enabled=layers,
                    lidar_points=len(points) if points is not None else 0,
                    lidar_age_seconds=cloud_age,
                    radiation_sources=radiation_sources,
                    camera_frustums=len(cams) if layers["cameras"] else 0,
                    radiation_readings=[
                        sensor.sample().to_dict()
                        for sensor in sim.sensors
                        if hasattr(sensor, "detector_pose")
                    ],
                )
                # Encode the browser image alongside GPU work. Keep at most one
                # queued frame so a slow filesystem cannot accumulate latency.
                if view_pending is None or view_pending.done():
                    if view_pending is not None:
                        view_pending.result()
                    view_pending = view_writer.submit(
                        publish_view, args.view_image, rgb
                    )
            observer_done = time.monotonic()
            for encoder, collect in pending_frames:
                frame = collect()
                if encoder:
                    encoder.push_bytes(frame)
            elapsed = time.monotonic() - t0
            stage_times.append([(sync_done-t0)*1000, (submit_done-sync_done)*1000, (observer_done-submit_done)*1000, (time.monotonic()-observer_done)*1000])
            times.append(elapsed)
            frames += 1
            seq += 1
            time.sleep(max(0, 1 / args.fps - elapsed))
            now = time.monotonic()
            if now - report >= 3:
                info = {
                    "fps_per_camera": frames / (now - report),
                    "stage_ms": dict(zip(("sync", "camera_submit", "observer", "camera_collect"), np.mean(stage_times, axis=0).tolist())),
                    "render_ms_mean": float(np.mean(times) * 1000),
                    "render_ms_p95": float(np.percentile(times, 95) * 1000),
                    "width": w,
                    "height": h,
                    "cameras": wanted,
                    "frames_total": seq,
                    "measured_seconds": now - start,
                    "static_cache_shift": gpu.cache_diagnostics,
                    "static_cache_hits": gpu.static_cache_hits,
                    "static_cache_misses": gpu.static_cache_misses,
                    "layers": layer_info,
                    "encoder": args.encoder,
                }
                print("[gpu_worker] " + json.dumps(info), flush=True)
                if args.metrics:
                    atomic(args.metrics, json.dumps(info).encode())
                frames = 0
                report = now
                times = []
                stage_times = []
    finally:
        if view_writer:
            view_writer.shutdown(wait=True)
            if view_pending is not None:
                view_pending.result()
        if feeds:
            feeds.stop()
        gpu.close()
        sim.disconnect()


if __name__ == "__main__":
    main()
