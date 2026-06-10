#!/usr/bin/env python3
"""sim_server: the headless sim exposed exactly like the real robot.

Runs the mock-physics sim and speaks the robot's wire seams so the autonomy stack
+ vision model hook up unchanged -- in production the ONLY difference is the
transport endpoint (real hardware vs this). It serves, all at once:

  * rove_sensor_api TELEMETRY out (UDP)  -- vectornav/kinova/robotiq/odrive_3x/pmic
  * rove_control_bridge CONTROL in (UDP) -- drive/flippers/arm/gripper from autonomy
  * camera RTSP feeds                    -- rtsp://host:8554/<cam>  (vision model)
  * Livox point clouds (binary UDP)      -- livox_top/bottom  (SLAM / mapping)
  * ground-truth pose (UDP)              -- scoring only, never consumed by autonomy

    tools/sim_server.py --profile standard --terrain
    # (on the host, for GPU render + ffmpeg/mediamtx)  tools/sim_host.sh ...

Pull control with rove_sim.api.control_bridge.ControlPublisher; read telemetry
with state.RoveSensorApiStateSource; read clouds with sensors.lidar.decode_cloud.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim import runtime
from rove_sim.core.util import die_with_parent
from rove_sim.api.sim_sensor_api import SimSensorApi
from rove_sim.api.control_bridge import RoveControlBridge
from rove_sim.transport import build_transport
from rove_sim.transport.udp import DEFAULT_PORTS
from rove_sim.transport.ports import PortMap
from rove_sim.sensors.lidar import LidarUdpPublisher, LivoxImuUdpPublisher
from rove_sim.sensors.rtsp import RtspCameraFeeds
from rove_sim.world.render_sync import publish_robot_state, DEFAULT_STATE_FILE


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--terrain", nargs="?",
                    const="../free_dirt_road_through_forest.glb", default=None)
    ap.add_argument("--host", default="127.0.0.1", help="bind/target host for UDP+RTSP")
    ap.add_argument("--sensor-host", default=None,
                    help="destination host for PUSHED sensor streams (lidar / Livox IMU / "
                         "ground-truth) when autonomy runs on a different machine than the sim. "
                         "Defaults to --host. Backend telemetry + control stay on --host, local "
                         "to the rove_sensor_api binary, so the rsa mocks keep receiving them.")
    ap.add_argument("--no-texture", action="store_true")
    ap.add_argument("--no-rtsp", action="store_true", help="skip the camera RTSP feeds")
    ap.add_argument("--no-lidar", action="store_true",
                    help="skip the Livox raycast (delegate to lidar_worker for realtime)")
    ap.add_argument("--rtsp-fps", type=float, default=15.0,
                    help="output (CFR) fps per stream")
    ap.add_argument("--cam-render-fps", type=float, default=12.0,
                    help="TOTAL camera renders/sec, round-robin across cameras "
                         "(each cam = this/N; RTSP still emits --rtsp-fps CFR)")
    ap.add_argument("--rtsp-scale", default="none",
                    help="WxH to downscale RTSP, or 'none' (cameras already render small)")
    ap.add_argument("--rtsp-encoder", default="auto",
                    help="auto|libx264|h264_nvenc (auto prefers GPU nvenc)")
    ap.add_argument("--rsa-backend", action="store_true",
                    help="back the real rove_sensor_api Rust binary: publish telemetry "
                         "on the shared backend ports (6000+) its mock drivers subscribe "
                         "to, and emit the native Livox IMU stream")
    ap.add_argument("--ports", default=None,
                    help="path to the shared ports.toml (default: rove_sensor_api/config/ports.toml)")
    ap.add_argument("--telemetry-hz", type=float, default=50.0)
    ap.add_argument("--imu-hz", type=float, default=200.0,
                    help="Livox IMU publish rate (rsa-backend mode)")
    ap.add_argument("--lidar-hz", type=float, default=10.0)
    ap.add_argument("--lidar-rays", type=int, default=10000,
                    help="rays/scan per Livox (cloud is decimated to 4k anyway)")
    ap.add_argument("--state-file", default=DEFAULT_STATE_FILE,
                    help="shared file for cam_worker robot-state sync")
    ap.add_argument("--tiny", action="store_true",
                    help="low-spec mode (2GB GPU / weak CPU): CPU-only physics, "
                         "fewer solver iters")
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    die_with_parent()                    # no orphans if the fleet launcher is killed

    overrides = {"friction": {"origin": (-25., -25.), "extent": (50., 50.), "cell": 0.25}}
    if args.terrain:
        overrides["terrain"] = {"source": args.terrain, "texture": not args.no_texture}
    # No GPU unless we render cameras inline (--no-rtsp -> cameras offloaded to the
    # pyrender fleet -> physics runs CPU-only, freeing all VRAM for the renderer).
    sim = runtime.build(args.profile, mode="headless", world="mock",
                        world_overrides=overrides, egl=not args.no_rtsp,
                        solver_iterations=8 if args.tiny else None)

    # --- wire seams ---------------------------------------------------------
    # In rsa-backend mode telemetry moves to the shared backend ports (6000+) the
    # rove_sensor_api Rust mock drivers subscribe to; control stays on its port.
    port_map = PortMap.load(args.ports) if args.rsa_backend else None
    ctrl_port = port_map.control_port if port_map else DEFAULT_PORTS["control"]
    tele_spec = {"mode": "udp", "host": args.host, "role": "pub"}
    if port_map:
        tele_spec["ports"] = port_map.backend_telemetry_ports()
    tele = build_transport(tele_spec)
    tele.start()
    api = SimSensorApi(sim.robot, sim.robot.profile, transport=tele,
                       actuators=sim.actuators)
    ctrl_spec = {"mode": "udp", "host": args.host, "role": "sub",
                 "subscribe": ["control"]}
    if port_map:
        ctrl_spec["ports"] = {"control": ctrl_port}
    ctrl_t = build_transport(ctrl_spec)
    bridge = RoveControlBridge(transport=ctrl_t); bridge.start()

    all_livox = [s for s in sim.sensors if s.name.startswith("livox")]
    # The heavy part is the point-cloud RAYCAST -- --no-lidar offloads it to
    # lidar_worker (so the fleet's pyrender cameras stay realtime).
    livox = [] if args.no_lidar else all_livox
    for s in livox:                      # lighter ray budget for realtime serving
        if hasattr(s, "set_rays") and args.lidar_rays > 0:
            s.set_rays(min(args.lidar_rays, s.rays_per_scan))
    lidar_pubs = {s.name: LidarUdpPublisher(args.host, DEFAULT_PORTS.get(s.name, 5022))
                  for s in livox}
    # The Livox Mid-360 built-in IMU is part of the LIDAR subsystem -- completely
    # separate from the rove_sensor_api (which is only Pi-board hardware). It is
    # CHEAP (one getLinkState) and needs the sim's REAL physics velocities, so the
    # authoritative sim always emits it on its own native Livox UDP stream -- even
    # when the point-cloud raycast is delegated to lidar_worker (--no-lidar).
    imu_pub = None
    imu_sensors = []
    if all_livox:
        imu_sensors = sorted(all_livox, key=lambda s: 0 if "top" in s.name else 1)
        imu_port = port_map.livox_imu_port if port_map else 56401
        imu_pub = LivoxImuUdpPublisher(args.host, imu_port)
    gt = next((d for d in api.devices if getattr(d, "channel", "") == "vectornav"), None)

    feeds = None
    if not args.no_rtsp:
        cams = [s for s in sim.sensors if s.name.startswith("cam_")]
        scale = None if args.rtsp_scale == "none" else tuple(
            int(v) for v in args.rtsp_scale.split("x"))
        enc = args.rtsp_encoder
        if enc == "auto":                # prefer GPU nvenc; fall back to libx264
            import subprocess as _sp
            try:
                out = _sp.run(["ffmpeg", "-hide_banner", "-encoders"],
                              capture_output=True, text=True, timeout=5).stdout
                enc = "h264_nvenc" if "h264_nvenc" in out else "libx264"
            except Exception:
                enc = "libx264"
        feeds = RtspCameraFeeds(cams, fps=args.rtsp_fps, scale=scale, encoder=enc,
                                render_fps=args.cam_render_fps).start()

    tele_ports = (port_map.backend_telemetry_ports() if port_map else DEFAULT_PORTS)
    print(f"[sim_server] up on {args.host}"
          f"{'  [RSA-BACKEND: feeding the rove_sensor_api Rust binary]' if port_map else ''}")
    print(f"  telemetry UDP : {', '.join(f'{c}:{tele_ports[c]}' for c in tele_ports if c not in ('control','ground_truth','robot_state') and not c.startswith('livox'))}")
    print(f"  control  UDP  : control:{ctrl_port}")
    print(f"  lidar    UDP  : {', '.join(f'{n}:{DEFAULT_PORTS.get(n)}' for n in lidar_pubs)}")
    if imu_pub and imu_sensors:
        print(f"  livox IMU UDP : {imu_sensors[0].name}:{imu_pub.port} (native Livox @ {args.imu_hz:.0f} Hz, separate from the API)")
    print(f"  ground-truth  : ground_truth:{DEFAULT_PORTS['ground_truth']}")
    if feeds:
        print(f"  camera RTSP   : {', '.join(feeds.urls())}")

    dt = 1.0 / sim.control_hz
    acc_t = acc_l = acc_i = 0.0
    rt_t0 = time.time(); rt_ticks = 0               # realtime-factor counter
    try:
        while True:
            t0 = time.time()
            sim.set_intent(bridge.poll())              # autonomy command (or hold)
            sim.step_control(1)
            rt_ticks += 1
            publish_robot_state(sim.robot, args.state_file)  # for cam_workers
            if time.time() - rt_t0 >= 5.0:             # report achieved rate
                rate = rt_ticks / (time.time() - rt_t0)
                print(f"[sim_server] {rate:.0f} Hz control "
                      f"({rate / sim.control_hz:.2f}x realtime)")
                rt_t0 = time.time(); rt_ticks = 0
            acc_t += dt; acc_l += dt; acc_i += dt
            if imu_pub and imu_sensors and acc_i >= 1.0 / args.imu_hz:  # Livox IMU
                acc_i = 0.0
                gyro, accel = imu_sensors[0].imu_sample()
                imu_pub.publish(gyro, accel, time.time_ns())
            if acc_t >= 1.0 / args.telemetry_hz:        # rove_sensor_api telemetry
                acc_t = 0.0
                api.publish()
                if gt is not None:                      # true pose for scoring
                    import pybullet as p
                    pos, orn = p.getBasePositionAndOrientation(sim.robot.body_id)
                    tele.publish("ground_truth", {"pos": list(pos), "orn": list(orn),
                                                  "t": sim.clock.now()})
            if livox and acc_l >= 1.0 / args.lidar_hz:  # point clouds
                acc_l = 0.0
                for s in livox:
                    lidar_pubs[s.name].publish(s.sample())
            if feeds:
                feeds.publish(dt)                       # camera RTSP (self rate-gated)
            time.sleep(max(0.0, dt - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if feeds:
            feeds.stop()
        for lp in lidar_pubs.values():
            lp.close()
        if imu_pub:
            imu_pub.close()
        bridge.stop(); tele.stop(); sim.disconnect()
        print("\n[sim_server] stopped")


if __name__ == "__main__":
    main()
