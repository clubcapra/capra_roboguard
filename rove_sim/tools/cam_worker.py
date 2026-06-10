#!/usr/bin/env python3
"""cam_worker: a decoupled GPU render process for the sim's cameras.

PyBullet renders behind the GIL and returns pixels as a Python tuple, so a single
process can't stream many cameras at a useful fps while also stepping physics. So
the authoritative sim (sim_server / live.py) publishes its robot's pose+joints on
the `robot_state` UDP channel, and N of THESE workers -- each its own process, GIL
and EGL context -- mirror that robot (no physics) and stream a SUBSET of the
cameras over RTSP. Run ~one worker per 2-3 cameras (256x192 ~ 14 ms/render -> ~72
renders/s/worker), so the cores render in parallel and the sim stays realtime.

    tools/cam_worker.py --terrain --cameras cam_front,cam_rear,cam_left --fps 24
    tools/cam_worker.py --terrain --cameras cam_right,cam_arm --fps 24
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
from rove_sim.sensors.rtsp import RtspCameraFeeds


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--terrain", nargs="?",
                    const="../free_dirt_road_through_forest.glb", default=None)
    ap.add_argument("--no-texture", action="store_true")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--cameras", required=True,
                    help="comma list of camera names this worker renders")
    ap.add_argument("--fps", type=float, default=24.0, help="per-camera target fps")
    ap.add_argument("--res", default="256x192", help="render resolution WxH")
    ap.add_argument("--encoder", default="auto")
    ap.add_argument("--state-file", default=DEFAULT_STATE_FILE,
                    help="shared robot-state file written by the sim")
    ap.add_argument("--port", type=int, default=8554, help="RTSP server port")
    ap.add_argument("--shared-server", action="store_true",
                    help="push to an already-running mediamtx on --port instead of "
                         "starting one (many workers share one server)")
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
    # render-only: build the world+robot but we never step_control (the authoritative
    # sim owns physics; we just mirror its pose+joints and render).
    sim = runtime.build(args.profile, mode="headless", world="mock",
                        world_overrides=overrides)
    p.setGravity(0, 0, 0)                          # no drift; we reset state anyway

    want = [n.strip() for n in args.cameras.split(",") if n.strip()]
    cams = [s for s in sim.sensors if s.name in want]
    if not cams:
        print(f"[cam_worker] no cameras matched {want}", file=sys.stderr)
        sys.exit(2)
    w, h = (int(v) for v in args.res.split("x"))
    for c in cams:
        c.width, c.height = w, h

    enc = args.encoder
    if enc == "auto":
        import subprocess as _sp
        try:
            out = _sp.run(["ffmpeg", "-hide_banner", "-encoders"],
                          capture_output=True, text=True, timeout=5).stdout
            enc = "h264_nvenc" if "h264_nvenc" in out else "libx264"
        except Exception:
            enc = "libx264"
    feeds = RtspCameraFeeds(cams, fps=args.fps, port=args.port, encoder=enc,
                            manage_server=not args.shared_server).start()
    streams = feeds.streams
    if not streams:
        print("[cam_worker] RTSP unavailable (need mediamtx + ffmpeg)", file=sys.stderr)
        sys.exit(3)
    print(f"[cam_worker] {len(cams)} cam(s) @ {w}x{h} {args.fps:g}fps [{enc}]: "
          + "  ".join(feeds.urls()))

    period = 1.0 / args.fps
    rt0 = time.time(); frames = 0
    try:
        while True:
            t0 = time.time()
            apply_robot_state(sim.robot, read_robot_state(args.state_file))
            for c in cams:                          # render ALL my cameras this tick
                s = streams.get(c.name)
                if s is not None:
                    s.push(c.rgb_frame())
            frames += 1
            if time.time() - rt0 >= 3.0:            # achieved per-camera render fps
                fps = frames / (time.time() - rt0)
                print(f"[cam_worker:{cams[0].name}+] {fps:.1f} fps/cam "
                      f"({fps * len(cams):.0f} renders/s)")
                rt0 = time.time(); frames = 0
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        feeds.stop()
        sim.disconnect()


if __name__ == "__main__":
    main()
