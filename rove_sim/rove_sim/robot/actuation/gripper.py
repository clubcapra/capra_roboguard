"""Mimic gripper actuator -- drives a Robotiq 2F-140 (or any single-DOF gripper).

A 2F-140 has ONE actuated joint (`finger_joint`); the rest of the 4-bar linkage
are <mimic> joints (target = multiplier * driven angle). PyBullet ignores mimic
tags, so every finger joint loads free and we enforce the coupling here each
control tick. Intent `gripper.position` is 0..255 (0 = open) -> driven joint
0..closed_rad, mirrored to the mimic joints. Bound entirely by config (driven
joint name + mimic map), so a different gripper is just a different param block.
"""
from __future__ import annotations

import pybullet as p

from .base import Actuator, register


@register("mimic_gripper")
@register("robotiq_2f140")              # alias
class MimicGripper(Actuator):
    intent_field = "gripper"

    def _resolve_joints(self):
        self.closed = float(self.params.get("closed_rad", 0.7))
        self.force = float(self.params.get("force", 60.0))
        self.max_vel = float(self.params.get("max_vel_rad_s", 2.0))
        ji = self.robot.joint_index
        self.driven = ji.get(self.params.get("driven_joint", "finger_joint"))
        self.mimic = {ji[j]: float(m)
                      for j, m in (self.params.get("mimic") or {}).items()
                      if j in ji}

    def apply(self, intent):
        if self.driven is None:
            return
        frac = max(0.0, min(1.0, intent.gripper.position / 255.0))
        ang = frac * self.closed
        self._drive(self.driven, ang)
        for j, mult in self.mimic.items():
            self._drive(j, mult * ang)

    def _drive(self, joint, target):
        p.setJointMotorControl2(self.robot.body_id, joint, p.POSITION_CONTROL,
                                targetPosition=target, force=self.force,
                                maxVelocity=self.max_vel)
