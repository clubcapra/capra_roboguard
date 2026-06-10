"""M1 gate: scripted RoveControl drives the robot, articulates flippers, moves
the arm. Turning is exercised via a driving arc (point-turns work but are slow
and high-slip by design -- the real robot also 'forces a lot' to spin, and the
narrow-track/long-wheelbase geometry makes yaw rate variable, so the test asserts
forward + arc + flipper + arm, which are stable).

    ../rove_sim_venv/bin/python -m pytest tests/test_m1_actuation.py -q
"""
import os
import sys

import numpy as np
import pybullet as p
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim.runtime import build
from rove_sim.control import RoveControl, Tracks, Flippers, Ovis


def _xy(r):
    return np.array(p.getBasePositionAndOrientation(r.body_id)[0])[:2]


def _yaw(r):
    return p.getEulerFromQuaternion(p.getBasePositionAndOrientation(r.body_id)[1])[2]


@pytest.fixture()
def std():
    sim = build("standard", mode="headless")
    yield sim
    sim.engine.disconnect()


def test_drives_forward(std):
    p0 = _xy(std.robot)
    std.run_for(2.0, RoveControl(tracks=Tracks(0.15, 0.15)))
    assert np.linalg.norm(_xy(std.robot) - p0) > 0.8, "robot did not drive forward"


def test_point_turns(std):
    # narrow-track skid-steer: turns are slow and high-slip by design (the real
    # robot also forces hard to spin), so assert a modest heading change over 3 s.
    y0 = _yaw(std.robot)
    std.run_for(3.0, RoveControl(tracks=Tracks(-1.0, 1.0)))
    assert abs(_yaw(std.robot) - y0) > np.radians(8), "robot did not point-turn"


def test_flipper_articulates(std):
    # flippers are worm-geared and slow (~10 deg/s = 0.175 rad/s), so 2 s of
    # command gives ~0.3 rad. Just assert it lifts meaningfully and stays put.
    r = std.robot
    j = r.movable_joint["FlipperFL"]
    a0 = p.getJointState(r.body_id, j)[0]
    std.run_for(2.0, RoveControl(flippers=Flippers(fl=1)))
    assert abs(p.getJointState(r.body_id, j)[0] - a0) > 0.1, "flipper did not move"


def test_arm_ik_moves_ee(std):
    assert std.caps.has_arm
    # the REAL end-effector is JointGripper_pivot (JointGripper is a URDF artifact
    # brought back to the body origin); check the actual tip rises on a vz+ (up) cmd
    ee = std.robot.link_index["JointGripper_pivot"]
    z0 = p.getLinkState(std.robot.body_id, ee)[4][2]
    std.run_for(2.0, RoveControl(ovis=Ovis(vz=1.0)))
    assert p.getLinkState(std.robot.body_id, ee)[4][2] - z0 > 0.03, "arm EE did not rise"


def test_belt_injected(std):
    # each track's ground patch is a dense row of 16 tread rollers
    assert len(std.robot.track_wheels["left"]) == 16
    assert len(std.robot.track_wheels["right"]) == 16
    # brush tracks actuator tags every belt-contact link with its side: 16+16
    # treads + the flipper-belt links (paddle + pivot, both sides)
    tracks = next(a for a in std.actuators if a.intent_field == "tracks")
    n_left = sum(1 for s in tracks.side_of.values() if s == "L")
    n_right = sum(1 for s in tracks.side_of.values() if s == "R")
    assert n_left >= 16 and n_right >= 16
    assert std.robot.belt_links            # flipper belts mapped
    assert std.robot.total_mass == pytest.approx(100.0, abs=0.5)


def test_no_lean_when_driving(std):
    # the drive-sign fix: driving straight must not roll the robot onto one side
    std.run_for(1.5, RoveControl(tracks=Tracks(0.3, 0.3)))
    roll, _, _ = p.getEulerFromQuaternion(
        p.getBasePositionAndOrientation(std.robot.body_id)[1])
    assert abs(np.degrees(roll)) < 10, "robot leaned while driving straight"
