"""M2 telemetry loop: mock SimSensorApi -> wire -> RoveSensorApiStateSource.

Closes the loop in one process WITHOUT two pybullet instances: build one mock
sim, publish its physics state as rove_sensor_api DATA frames, then decode them
back through the matching device codecs and assert the reconstructed RobotState
matches the sim's joints. This exercises the BBH+JSON codec and the encode/decode
symmetry that real-mode sync depends on.

    ../rove_sim_venv/bin/python -m pytest tests/test_m2_telemetry.py -q
"""
import math
import os
import sys

import pybullet as p
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim import runtime
from rove_sim.api.devices import build_devices
from rove_sim.api.sim_sensor_api import SimSensorApi
from rove_sim.state.rove_sensor_api import RoveSensorApiStateSource
from rove_sim.transport import InProcessTransport
from rove_sim.transport.packet import decode, encode, MSG_DATA


def test_packet_codec_roundtrip():
    payload = {"joint_1_pos": 12.5, "position": 200, "timestamp_ns": 7}
    mt, seq, got = decode(encode(MSG_DATA, 42, payload))
    assert mt == MSG_DATA and seq == 42 and got == payload


def test_udp_transport_roundtrip():
    """The real-robot wire: publish a DATA frame over UDP, subscriber decodes it."""
    import time
    from rove_sim.transport.udp import UdpTransport

    ports = {"kinova": 5402}                     # avoid the default 5000s in case in use
    pub = UdpTransport(host="127.0.0.1", ports=ports, role="pub")
    sub = UdpTransport(host="127.0.0.1", ports=ports, role="sub")
    sub.start(); pub.start()
    try:
        deadline = time.time() + 2.0
        got = None
        while time.time() < deadline and got is None:
            pub.publish("kinova", {"joint_1_pos": 33.0})
            time.sleep(0.02)
            got = sub.latest("kinova")
        assert got is not None and got["joint_1_pos"] == 33.0
    finally:
        pub.stop(); sub.stop()


def test_telemetry_loop_reconstructs_joints():
    sim = runtime.build("standard", mode="headless", world="mock")
    try:
        robot = sim.robot
        # pose the arm + gripper to non-trivial values so the loop has signal
        arm_link = "ASection"
        arm_idx = robot.movable_joint[arm_link]
        p.resetJointState(robot.body_id, arm_idx, 0.9)
        finger = robot.joint_index["finger_joint"]
        p.resetJointState(robot.body_id, finger, 0.35)

        bus = InProcessTransport(bus="test_loop")
        api = SimSensorApi(robot, robot.profile, transport=bus)
        # subscriber shares the same in-process bus, its own device codecs
        src = RoveSensorApiStateSource(profile=robot.profile, robot=robot,
                                       transport=InProcessTransport(bus="test_loop"))
        api.publish()                       # one DATA frame per channel
        state = src.read()

        # arm joint round-trips through degrees/JSON within float tolerance
        idx_to_name = {v: k for k, v in robot.joint_index.items()}
        assert math.isclose(state.joints[idx_to_name[arm_idx]], 0.9, abs_tol=1e-4)
        # gripper byte-quantised: 0.35 rad / 0.7 closed * 255 = 127 -> back ~0.349
        assert math.isclose(state.joints["finger_joint"], 0.35, abs_tol=0.01)
        # mimic joints were expanded so a synced twin actually closes
        assert "right_inner_finger_joint" in state.joints
        # vectornav carried a base pose
        assert state.base_pose is not None
    finally:
        sim.disconnect()
