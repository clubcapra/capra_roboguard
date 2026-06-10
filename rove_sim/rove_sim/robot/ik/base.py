"""IK resolver seam (S7).

`IKResolver.solve(ee_pos, ee_orn, current)` returns target angles for the arm's
movable joints. PyBulletIK (DLS, default) is the in-sim solver; a SidecarIK
(Robotics Toolbox) can be swapped in for exact arm parity with the real engine.
Selected per profile; arm motion is not bit-identical sim-vs-real unless sidecar.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Sequence

from ...registry import Registry

IK_REGISTRY = Registry("ik")
register = IK_REGISTRY.register


class IKResolver(ABC):
    def __init__(self, robot, ee_link: str, arm_links: Sequence[str], **params):
        self.robot = robot
        self.ee_link = ee_link
        self.arm_links = list(arm_links)
        self.params = params

    @abstractmethod
    def solve(self, ee_pos, ee_orn: Optional[Sequence[float]],
              current: Dict[int, float]) -> Dict[int, float]:
        """Return {joint_index: target_angle} for the arm's movable joints."""
