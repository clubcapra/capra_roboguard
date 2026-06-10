"""Hardware-free state sources: hand-driven and replay.

ManualStateSource lets a test or tool set joint values directly and watch the
URDF follow kinematically -- the backbone of the SyncDriver test and of driving
a real-mode twin from a script. ReplayStateSource plays a recorded list of
RobotState frames (e.g. captured telemetry) one per read().
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

from .base import RobotState, RobotStateSource, register


@register("manual")
class ManualStateSource(RobotStateSource):
    def __init__(self, profile=None, **_):
        self._state = RobotState()

    # -- producer API (a test/tool pushes state) ---------------------------
    def set_joint(self, name: str, value: float) -> None:
        self._state.joints[name] = float(value)

    def set_joints(self, joints: Dict[str, float]) -> None:
        self._state.joints.update({k: float(v) for k, v in joints.items()})

    def set_base_pose(self, pos: Sequence[float], orn: Sequence[float]) -> None:
        self._state.base_pose = (list(pos), list(orn))

    # -- consumer API (the SyncDriver reads) -------------------------------
    def read(self) -> RobotState:
        return RobotState(dict(self._state.joints), self._state.base_pose,
                          self._state.stamp_us)


@register("replay")
class ReplayStateSource(RobotStateSource):
    def __init__(self, profile=None, frames: Optional[List[RobotState]] = None,
                 loop: bool = True, **_):
        self._frames = list(frames or [])
        self._loop = loop
        self._i = 0

    def read(self) -> RobotState:
        if not self._frames:
            return RobotState()
        frame = self._frames[min(self._i, len(self._frames) - 1)]
        if self._i < len(self._frames) - 1:
            self._i += 1
        elif self._loop:
            self._i = 0
        return frame
