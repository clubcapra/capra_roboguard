"""Normalized control intent -- the smallest, most stable contract (S3, S6).

Mirror of `RoveControl` from telemetry.proto. The sim sits *below* the control
bridge: it consumes this intent and does its own intent->actuator solving. Every
actuator reads only its own slice; nothing outside hands per-joint setpoints.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Tracks:
    left_vel: float = 0.0     # [-1, 1]
    right_vel: float = 0.0    # [-1, 1]


@dataclass
class Flippers:
    # per-flipper step command in {-1, 0, +1}; accumulated by the actuator
    fl: int = 0
    fr: int = 0
    rl: int = 0               # rear-left  -> FlipperBL
    rr: int = 0               # rear-right -> FlipperBR


@dataclass
class Ovis:
    # 6-DOF end-effector twist intent, each [-1, 1]: linear then angular
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0


@dataclass
class Gripper:
    position: int = 0         # 0 = open .. 255 = closed


@dataclass
class RoveControl:
    tracks: Tracks = field(default_factory=Tracks)
    flippers: Flippers = field(default_factory=Flippers)
    ovis: Ovis = field(default_factory=Ovis)
    gripper: Gripper = field(default_factory=Gripper)
    timestamp_us: int = 0

    @staticmethod
    def stop() -> "RoveControl":
        return RoveControl()
