"""M2 seam: mock vs real worlds + the kinematic SyncDriver.

    ../rove_sim_venv/bin/python -m pytest tests/test_m2_modes.py -q
"""
import os
import sys

import numpy as np
import pybullet as p
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim import runtime
from rove_sim.state.manual import ManualStateSource


def test_mock_world_has_ground():
    sim = runtime.build("standard", mode="headless", world="mock")
    try:
        assert sim.world.ground_id is not None          # plane.urdf loaded
        z = p.getBasePositionAndOrientation(sim.robot.body_id)[0][2]
        assert z > 0.0                                  # rests on the ground
    finally:
        sim.disconnect()


def test_real_world_has_no_ground_and_no_physics():
    """Real mode is a kinematic world model: no ground plane, and stepping does
    NOT integrate the robot (it would free-fall in a physics world with no floor)."""
    src = ManualStateSource()
    sim = runtime.build("standard", mode="headless", world="real", state_source=src)
    try:
        assert sim.world.ground_id is None              # no ground body
        before = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])
        sim.step_control(50)                            # 1 s of control ticks
        after = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])
        assert np.linalg.norm(after - before) < 1e-6    # base did not move/fall
        lin, _ = p.getBaseVelocity(sim.robot.body_id)
        assert np.linalg.norm(lin) < 1e-6               # no velocity integrated
    finally:
        sim.disconnect()


def test_sync_driver_moves_url_kinematically():
    """Push a joint value through the source; the URDF link must follow without
    any physics (the whole point of real/sync mode)."""
    src = ManualStateSource()
    sim = runtime.build("standard", mode="headless", world="real", state_source=src)
    try:
        robot = sim.robot
        idx_to_name = {v: k for k, v in robot.joint_index.items()}
        # the joint that actually drives the shoulder link (past the fixed _offset)
        arm_joint_idx = robot.movable_joint["ASection"]
        arm_joint_name = idx_to_name[arm_joint_idx]
        tip = robot.link_index["JointGripper_pivot"]

        ee_before = np.array(p.getLinkState(robot.body_id, tip,
                                            computeForwardKinematics=True)[4])
        src.set_joint(arm_joint_name, 1.2)              # rad
        sim.step_control(1)

        # the commanded joint took the exact value (kinematic, no servo lag)
        assert abs(p.getJointState(robot.body_id, arm_joint_idx)[0] - 1.2) < 1e-6
        # and the tip moved as a result
        ee_after = np.array(p.getLinkState(robot.body_id, tip,
                                           computeForwardKinematics=True)[4])
        assert np.linalg.norm(ee_after - ee_before) > 0.05
        # still no physics: base stays put
        lin, _ = p.getBaseVelocity(robot.body_id)
        assert np.linalg.norm(lin) < 1e-6
    finally:
        sim.disconnect()
