"""Shared engine state. Single-threaded asyncio writes; transports may
swap-in a new Ovis at any await point but never tear it.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

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
    # Per-link TCP (centroid) offset in the link's local frame. Populated at
    # engine startup from mesh geometry; used as the rotation pivot / IK
    # position target when an Ovis arrives with no tcp_offset_local set.
    tcp_offsets: dict[str, np.ndarray] = field(default_factory=dict)
    # Latest kinova_arm state pushed by rove_sensor_api. Ordered by kinova
    # actuator index (1..N). None until the first frame arrives.
    latest_kinova_positions: list[float] | None = None
    latest_kinova_t: float = 0.0
    # Per-joint offset (radians) captured at sync time:
    #     offset[id] = kinova_q_at_sync - model_q_at_sync
    # After sync, mapping kinova readings into the model frame is:
    #     model_q = kinova_q - offset
    # At sync time itself this evaluates to the model's pre-sync value, so
    # the model doesn't visually jump when the user clicks Sync.
    kinova_offsets: dict[str, float] = field(default_factory=dict)
    # Ordered list of joint entity ids matching kinova actuator index 1..N.
    # Captured at sync time so the per-tick mirror loop doesn't have to
    # re-resolve the chain.
    kinova_chain_joint_ids: list[str] = field(default_factory=list)

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
