"""M0 gate: both robot profiles load headless and rest on the ground stably.

    ../rove_sim_venv/bin/python -m pytest tests/test_m0_load.py -q
"""
import os
import sys

import numpy as np
import pybullet as p
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim.core.engine import Engine, EngineConfig
from rove_sim.robot import loader
from rove_sim.robot.profile import load_profile
from rove_sim.world.mock import MockWorld
from rove_sim import capabilities

PROFILES = os.path.join(os.path.dirname(__file__), "..", "profiles")


def _run(profile_name):
    prof = load_profile(os.path.join(PROFILES, f"{profile_name}.yaml"))
    eng = Engine(EngineConfig(mode="headless")).connect()
    try:
        # gravity + ground plane moved out of the engine into the World seam
        MockWorld(eng, {}).build()
        robot = loader.load(eng, prof)
        for _ in range(480):                 # 2 s @ 240 Hz
            eng.step()
        lin, ang = p.getBaseVelocity(robot.body_id)
        return robot, prof, np.linalg.norm(lin), np.linalg.norm(ang)
    finally:
        eng.disconnect()


@pytest.mark.parametrize("name", ["standard", "caged"])
def test_loads_and_rests(name):
    robot, prof, lin, ang = _run(name)
    assert robot.body_id >= 0
    assert len(robot.link_index) > 10
    # drum + flipper links resolve to a movable joint by their semantic name
    for link in ("DrumFL", "DrumFR", "FlipperFL", "FlipperFR"):
        assert link in robot.movable_joint, f"{link} has no driving joint"
    # comes to rest
    assert lin < 0.05, f"{name} still translating: {lin:.3f} m/s"
    assert ang < 0.5, f"{name} still rotating: {ang:.3f} rad/s"


def test_capabilities_differ():
    std = capabilities.derive(load_profile(os.path.join(PROFILES, "standard.yaml")))
    cag = capabilities.derive(load_profile(os.path.join(PROFILES, "caged.yaml")))
    assert std.has_arm and not cag.has_arm        # arm only on standard
    assert std.has_gnss and cag.has_gnss          # both carry VN300 GNSS
    assert not std.has_gripper and not cag.has_gripper  # no finger joint yet
