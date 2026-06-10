"""M3/M4: bilateral rove_sensor_api seam -- full physics telemetry, control in,
point-cloud codec, GNSS modes, IMU error model.

    ../rove_sim_venv/bin/python -m pytest tests/test_m3_api.py -q
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim import runtime
from rove_sim.api.sim_sensor_api import SimSensorApi
from rove_sim.api.control_bridge import (RoveControlBridge, ControlPublisher,
                                         encode_control, decode_control)
from rove_sim.transport import InProcessTransport
from rove_sim.sensors.lidar import encode_cloud, decode_cloud, CloudReassembler
from rove_sim.control import RoveControl, Tracks, Gripper


def test_full_telemetry_channels_and_physics_fields():
    sim = runtime.build("standard", mode="headless", world="mock")
    try:
        bus = InProcessTransport(bus="t_m3")
        api = SimSensorApi(sim.robot, sim.robot.profile, transport=bus,
                           actuators=sim.actuators)
        api.publish()
        chans = {d.channel for d in api.devices}
        assert {"vectornav", "kinova", "robotiq", "pmic",
                "odrive_31", "odrive_34"} <= chans
        od = bus.latest("odrive_31")
        # real OdriveNodeState schema keys (sim is a verbatim passthrough)
        assert {"iq_measured", "bus_current", "fet_temp", "vel_estimate",
                "node_id", "axis_state"} <= set(od)
        pm = bus.latest("pmic")
        assert 0.0 <= pm["soc"] <= 1.0 and pm["bus_voltage"] > 30.0
    finally:
        sim.disconnect()


def test_control_bridge_roundtrip():
    rc = RoveControl(tracks=Tracks(0.6, -0.4), gripper=Gripper(200))
    assert decode_control(encode_control(rc)).tracks.left_vel == 0.6
    bridge = RoveControlBridge(transport=InProcessTransport(bus="t_ctrl"))
    ControlPublisher(InProcessTransport(bus="t_ctrl")).send(rc)
    got = bridge.poll()
    assert got.tracks.right_vel == -0.4 and got.gripper.position == 200


def test_pointcloud_codec_roundtrip_and_udp_size():
    sim = runtime.build("standard", mode="headless", world="mock")
    try:
        scan = sim.sensor("livox_bottom").sample()
        pkts = encode_cloud(scan, frame_id=7, max_points_per_packet=4000)
        # full cloud is fragmented across N datagrams, each one UDP-sized
        assert len(pkts) == max(1, -(-len(scan.points) // 4000))
        for buf in pkts:
            assert len(buf) < 60000
            _, _, _, fid, _, n_pkts, total = decode_cloud(buf)
            assert fid == 7 and n_pkts == len(pkts) and total == len(scan.points)
        # reassembling every packet yields the WHOLE cloud, no decimation
        ra = CloudReassembler()
        out = None
        for buf in pkts:
            out = ra.feed(buf)
        assert out is not None
        pts, t, pose = out
        assert pts.shape[1] == 3 and len(pts) == len(scan.points)
        assert np.allclose(pts, scan.points)
    finally:
        sim.disconnect()


def test_gnss_modes():
    sim = runtime.build("standard", mode="headless", world="mock")
    try:
        bus = InProcessTransport(bus="t_gnss")
        api = SimSensorApi(sim.robot, sim.robot.profile, transport=bus,
                           actuators=sim.actuators)
        vn = next(d for d in api.devices if d.channel == "vectornav")
        api.publish(); assert bus.latest("vectornav")["gnss_fix"] is True
        vn.gnss_mode = "denied"
        api.publish(); assert bus.latest("vectornav")["gnss_fix"] is False
        vn.gnss_mode = "spoofed"
        a = api.publish() or bus.latest("vectornav")
        for _ in range(20):
            api.publish()
        b = bus.latest("vectornav")
        assert b["gnss_mode"] == "spoofed"            # offset creeps (adversarial)
    finally:
        sim.disconnect()


def test_imu_error_model_adds_noise():
    sim = runtime.build("standard", mode="headless", world="mock")
    try:
        imu = sim.sensor("vn300")
        imu.errors = False
        clean = np.array(imu.sample().angular_velocity)
        imu.errors = True
        noisy = np.array(imu.sample().angular_velocity)
        assert not np.allclose(clean, noisy)          # bias+noise applied
    finally:
        sim.disconnect()
