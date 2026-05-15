"""Typed config dataclasses — one section per YAML block in default.yaml."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ListenConfig:
    """UDP port the bridge binds to for incoming RoveControl packets."""
    host: str = "0.0.0.0"
    port: int = 5005


@dataclass
class SensorApiConfig:
    """How to reach rove_sensor_api for /discover + command UDP."""
    host: str = "127.0.0.1"
    http_port: int = 8080


@dataclass
class TracksConfig:
    """How left/right stick values map onto ODrive node commands."""
    strategy: str = "velocity"   # "velocity" | "torque" | "mixed"
    left_node_ids: list[int] = field(default_factory=lambda: [31, 34])
    right_node_ids: list[int] = field(default_factory=lambda: [32, 33])
    max_velocity: float = 10.0
    max_torque: float = 5.0
    invert_left: bool = False
    invert_right: bool = True
    curve_expo: float = 0.0      # 0=linear, 1=cubic


@dataclass
class FlipperNodeIds:
    fl: Optional[int] = None
    fr: Optional[int] = None
    rl: Optional[int] = None
    rr: Optional[int] = None


@dataclass
class FlippersConfig:
    enabled: bool = False
    strategy: str = "velocity"
    node_ids: FlipperNodeIds = field(default_factory=FlipperNodeIds)
    max_velocity: float = 2.0


@dataclass
class OvisConfig:
    """Forward arm-twist frames to rove_ik_engine for IK + collision sim."""
    enabled: bool = False
    engine_host: str = "127.0.0.1"
    engine_port: int = 9100
    # Entity id the engine should drive toward. Fetch from the engine's
    # startup scene log or read it out of the bundled .forgebot.
    target_entity_id: str = ""


@dataclass
class GripperConfig:
    enabled: bool = True
    sensor_id: str = "robotiq_gripper"
    speed: int = 255   # 0-255, sent with every position command
    force: int = 128   # 0-255, sent with every position command


@dataclass
class BridgeConfig:
    """Root config; one instance is built per bridge process."""
    listen: ListenConfig = field(default_factory=ListenConfig)
    sensor_api: SensorApiConfig = field(default_factory=SensorApiConfig)
    tracks: TracksConfig = field(default_factory=TracksConfig)
    flippers: FlippersConfig = field(default_factory=FlippersConfig)
    ovis: OvisConfig = field(default_factory=OvisConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    discover_timeout_s: float = 10.0
    idle_timeout_s: float = 0.5    # ODrives → Idle after this much silence
    verbose: bool = False
