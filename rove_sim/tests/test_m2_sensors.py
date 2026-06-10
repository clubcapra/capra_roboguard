"""Sensors: the profile's camera / livox / vn300 instantiate on their mount links
and produce sane readings (M2 camera + lidar raycast + basic IMU).

    ../rove_sim_venv/bin/python -m pytest tests/test_m2_sensors.py -q
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim import runtime


def test_profile_sensors_build_and_sample():
    sim = runtime.build("standard", mode="headless", world="mock")
    try:
        names = {s.name for s in sim.sensors}
        assert {"cam_front", "cam_arm", "livox_top", "vn300"} <= names

        cam = sim.sensor("cam_front")
        f = cam.sample()
        assert f.rgb.shape == (f.height, f.width, 3)
        assert f.rgb.dtype == np.uint8
        assert f.depth.shape == (f.height, f.width)
        assert f.depth.min() > 0.0                    # metric depth, positive

        liv = sim.sensor("livox_top")
        sc = liv.sample()
        assert sc.n_rays > 1000
        assert sc.points.shape[1] == 3
        assert len(sc.points) > 0                      # rays hit the ground plane
        assert sc.ranges.min() >= 0.0

        imu = sim.sensor("vn300").sample()
        a = np.linalg.norm(imu.linear_acceleration)
        assert 9.0 < a < 10.5                          # ~1 g at rest (specific force)
    finally:
        sim.disconnect()


def test_sensor_rate_gating():
    """A 10 Hz lidar fires once over 0.1 s of control, not every control tick."""
    sim = runtime.build("standard", mode="headless", world="mock",
                        step_sensors=True)
    try:
        liv = sim.sensor("livox_top")
        calls = [0]
        orig = liv._sample
        liv._sample = lambda: (calls.__setitem__(0, calls[0] + 1), orig())[1]
        sim.step_control(5)              # 0.1 s at 50 Hz control -> one 10 Hz tick
        assert calls[0] == 1
    finally:
        sim.disconnect()
