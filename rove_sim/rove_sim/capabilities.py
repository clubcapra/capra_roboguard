"""Capability set derived from a profile.

The point of the composition model: autonomy asks "do we have GNSS?", never
"are we the standard robot?". A capability is present iff some declared
component provides it. No lat/lon hardware in the manifest => has_gnss is
False => PositionService runs SLAM-only, with no robot-specific branch anywhere.

This object is the contract autonomy reads. It is intentionally derived, never
hand-written per robot.
"""
from __future__ import annotations

from dataclasses import dataclass

from .robot.profile import Profile


@dataclass(frozen=True)
class Capabilities:
    provided: frozenset[str]

    # -- generic query -------------------------------------------------------
    def has(self, cap: str) -> bool:
        return cap in self.provided

    # -- named conveniences (autonomy-facing) --------------------------------
    @property
    def has_gnss(self) -> bool:
        return "gnss" in self.provided

    @property
    def has_imu(self) -> bool:
        return "imu" in self.provided

    @property
    def has_lidar(self) -> bool:
        return "pointcloud" in self.provided

    @property
    def has_arm(self) -> bool:
        return "arm_ik" in self.provided

    @property
    def has_gripper(self) -> bool:
        return "gripper" in self.provided

    @property
    def has_camera(self) -> bool:
        return "camera" in self.provided or "image" in self.provided

    def __repr__(self) -> str:
        return f"Capabilities({sorted(self.provided)})"


def derive(profile: Profile) -> Capabilities:
    return Capabilities(provided=frozenset(profile.provided_capabilities))
