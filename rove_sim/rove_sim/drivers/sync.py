"""SyncDriver: kinematic robot, driven by telemetry (real mode).

The robot is NOT physically integrated -- each tick we read the latest RobotState
from the source and slam the URDF to it with resetJointState /
resetBasePositionAndOrientation, then refresh the world model. There is NO
engine.step(): nothing about the robot's motion is simulated, it MIRRORS the real
robot. PyBullet here is a geometric world model for IK / collision / spatial
reasoning, and getLinkState(...computeForwardKinematics=True) (used by the
SelfCollisionGuard) is valid immediately after resetJointState with no step.

Joint telemetry is keyed by URDF joint NAME and resolved through
`robot.joint_index` -- the same name->index handle rove_ik.py maps off.
"""
from __future__ import annotations

import pybullet as p

from .base import Driver, register


@register("real")
class SyncDriver(Driver):
    def step_control(self, ticks: int, intent) -> None:
        for _ in range(ticks):
            if self.source is not None:
                state = self.source.read()
                bid = self.robot.body_id
                for jname, val in state.joints.items():
                    idx = self.robot.joint_index.get(jname)
                    if idx is not None:
                        p.resetJointState(bid, idx, float(val))
                if state.base_pose is not None:
                    pos, orn = state.base_pose
                    p.resetBasePositionAndOrientation(bid, list(pos), list(orn))
            self.world.update(1.0 / self.control_hz)
