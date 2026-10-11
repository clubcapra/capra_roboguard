#!/usr/bin/env python3
"""Measure fresh source-frame and scan rates from a running optimized fleet."""

import argparse
import json
from pathlib import Path
import statistics
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-metrics", required=True, type=Path)
    parser.add_argument("--lidar-metrics", required=True, type=Path)
    parser.add_argument("--state-file", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    samples = []
    seen = None
    started = time.monotonic()
    while time.monotonic() - started < args.seconds:
        now = time.time()
        if any(
            now - file.stat().st_mtime > 6
            for file in (args.camera_metrics, args.lidar_metrics)
        ):
            raise RuntimeError("Worker metrics are stale")
        camera = json.loads(args.camera_metrics.read_text())
        lidar = json.loads(args.lidar_metrics.read_text())
        if camera["frames_total"] != seen:
            seen = camera["frames_total"]
            sample = {
                "seconds": time.monotonic() - started,
                "camera": camera,
                "lidar": lidar,
                "physics_state_age_seconds": now - args.state_file.stat().st_mtime,
            }
            if sample["physics_state_age_seconds"] > 1:
                raise RuntimeError("Physics state is stale")
            try:
                sample["gpu"] = subprocess.check_output(
                    [
                        "nvidia-smi",
                        "--query-gpu=name,memory.used,utilization.gpu",
                        "--format=csv,noheader",
                    ],
                    text=True,
                ).strip()
            except (FileNotFoundError, subprocess.CalledProcessError):
                pass
            samples.append(sample)
        time.sleep(1)
    if len(samples) < 2:
        raise RuntimeError("Not enough fresh samples")
    fps = [s["camera"]["fps_per_camera"] for s in samples]
    hz = [s["lidar"]["hz_per_lidar"] for s in samples]
    result = {
        "duration_seconds": time.monotonic() - started,
        "minimum_camera_fps": min(fps),
        "mean_camera_fps": statistics.mean(fps),
        "minimum_lidar_hz": min(hz),
        "mean_lidar_hz": statistics.mean(hz),
        "samples": samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "samples"}, indent=2))
    if min(fps) < 24:
        raise SystemExit("FAIL: a measured camera-rate window fell below 24 fps")


if __name__ == "__main__":
    main()
