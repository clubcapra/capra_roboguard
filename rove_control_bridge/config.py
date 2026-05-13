"""Config loading and dataclass definitions for rove_control_bridge."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


# ---------------------------------------------------------------------------
# Dataclasses — typed view of the YAML config
# ---------------------------------------------------------------------------

@dataclass
class ListenConfig:
    host: str = "0.0.0.0"
    port: int = 5005


@dataclass
class SensorApiConfig:
    host: str = "127.0.0.1"
    http_port: int = 8080


@dataclass
class TracksConfig:
    strategy: str = "velocity"       # "velocity" | "torque"
    left_node_ids: list[int] = field(default_factory=lambda: [31, 34])
    right_node_ids: list[int] = field(default_factory=lambda: [32, 33])
    max_velocity: float = 10.0
    max_torque: float = 5.0
    invert_left: bool = False
    invert_right: bool = True   # right-side motors are physically mirrored
    curve_expo: float = 0.0     # torque strategy input shaping: 0=linear, 1=cubic


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
    enabled: bool = False
    # rove_mvp_engine endpoint. The bridge forwards every incoming
    # RoveControl.Ovis to this UDP socket (re-wrapped as the engine's own
    # Ovis proto with `target` filled in from `target_entity_id`).
    engine_host: str = "127.0.0.1"
    engine_port: int = 9100
    # Entity id the engine should drive toward. Currently hardcoded to the
    # jointgripper — fetch from the engine's startup scene log
    # (`scene contents — send Ovis.target = ...`) or read it out of the
    # bundled .forgebot.
    target_entity_id: str = ""


@dataclass
class GripperConfig:
    enabled: bool = True
    sensor_id: str = "robotiq_gripper"
    speed: int = 255    # 0–255, sent with every position command
    force: int = 128    # 0–255, sent with every position command


@dataclass
class BridgeConfig:
    listen: ListenConfig = field(default_factory=ListenConfig)
    sensor_api: SensorApiConfig = field(default_factory=SensorApiConfig)
    tracks: TracksConfig = field(default_factory=TracksConfig)
    flippers: FlippersConfig = field(default_factory=FlippersConfig)
    ovis: OvisConfig = field(default_factory=OvisConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    discover_timeout_s: float = 10.0
    # Seconds of silence before ODrives are put back to Idle state.
    idle_timeout_s: float = 0.5
    verbose: bool = False


# ---------------------------------------------------------------------------
# Loader
# ---------------------------------------------------------------------------

def load(path: Path) -> BridgeConfig:
    """Load a YAML config file and return a fully-populated BridgeConfig."""
    with open(path) as fh:
        raw = yaml.safe_load(fh) or {}

    cfg = BridgeConfig()

    if "listen" in raw:
        s = raw["listen"]
        cfg.listen = ListenConfig(
            host=s.get("host", cfg.listen.host),
            port=int(s.get("port", cfg.listen.port)),
        )

    if "sensor_api" in raw:
        s = raw["sensor_api"]
        cfg.sensor_api = SensorApiConfig(
            host=s.get("host", cfg.sensor_api.host),
            http_port=int(s.get("http_port", cfg.sensor_api.http_port)),
        )

    if "tracks" in raw:
        s = raw["tracks"]
        cfg.tracks = TracksConfig(
            strategy=s.get("strategy", cfg.tracks.strategy),
            left_node_ids=list(s.get("left_node_ids", cfg.tracks.left_node_ids)),
            right_node_ids=list(s.get("right_node_ids", cfg.tracks.right_node_ids)),
            max_velocity=float(s.get("max_velocity", cfg.tracks.max_velocity)),
            max_torque=float(s.get("max_torque", cfg.tracks.max_torque)),
            invert_left=bool(s.get("invert_left", cfg.tracks.invert_left)),
            invert_right=bool(s.get("invert_right", cfg.tracks.invert_right)),
            curve_expo=float(s.get("curve_expo", cfg.tracks.curve_expo)),
        )

    if "flippers" in raw:
        s = raw["flippers"]
        nids_raw = s.get("node_ids", {})
        cfg.flippers = FlippersConfig(
            enabled=bool(s.get("enabled", cfg.flippers.enabled)),
            strategy=s.get("strategy", cfg.flippers.strategy),
            node_ids=FlipperNodeIds(
                fl=nids_raw.get("fl"),
                fr=nids_raw.get("fr"),
                rl=nids_raw.get("rl"),
                rr=nids_raw.get("rr"),
            ),
            max_velocity=float(s.get("max_velocity", cfg.flippers.max_velocity)),
        )

    if "ovis" in raw:
        s = raw["ovis"]
        cfg.ovis = OvisConfig(
            enabled=bool(s.get("enabled", cfg.ovis.enabled)),
            engine_host=str(s.get("engine_host", cfg.ovis.engine_host)),
            engine_port=int(s.get("engine_port", cfg.ovis.engine_port)),
            target_entity_id=str(s.get("target_entity_id", cfg.ovis.target_entity_id)),
        )

    if "gripper" in raw:
        s = raw["gripper"]
        cfg.gripper = GripperConfig(
            enabled=bool(s.get("enabled", cfg.gripper.enabled)),
            sensor_id=str(s.get("sensor_id", cfg.gripper.sensor_id)),
            speed=int(s.get("speed", cfg.gripper.speed)),
            force=int(s.get("force", cfg.gripper.force)),
        )

    cfg.discover_timeout_s = float(raw.get("discover_timeout_s", cfg.discover_timeout_s))
    cfg.idle_timeout_s = float(raw.get("idle_timeout_s", cfg.idle_timeout_s))
    cfg.verbose = bool(raw.get("verbose", cfg.verbose))

    return cfg
