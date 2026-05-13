"""Shared engine state. Single-threaded asyncio writes; transports may
swap-in a new Ovis at any await point but never tear it.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from forgebot.core.model import Project

from .proto import Ovis


@dataclass
class EngineState:
    project: Project
    joint_values: dict[str, float] = field(default_factory=dict)
    joint_velocities: dict[str, float] = field(default_factory=dict)
    latest_ovis: Ovis | None = None
    start_time: float = field(default_factory=time.monotonic)
    last_tip: str = ""  # most recent Ovis.target, kept after Ovis goes None

    def elapsed(self) -> float:
        return time.monotonic() - self.start_time

    def set_ovis(self, ovis: Ovis | None) -> None:
        self.latest_ovis = ovis
        if ovis is not None and ovis.target:
            self.last_tip = ovis.target

    def take_ovis(self) -> Ovis | None:
        """Read-and-clear. Each Ovis is consumed by one tick; subsequent ticks
        with no fresh input hold the robot still (qdot = 0)."""
        o = self.latest_ovis
        self.latest_ovis = None
        return o
