"""YAML → BridgeConfig loader.

Every section is optional: missing keys fall back to the dataclass defaults
in `schema.py`. Use this for files written by operators; programmatic
callers can construct BridgeConfig directly.
"""
from __future__ import annotations

from pathlib import Path

import yaml

from .schema import (
    BridgeConfig,
    FlipperNodeIds,
    FlippersConfig,
    GripperConfig,
    ListenConfig,
    OvisConfig,
    SensorApiConfig,
    TracksConfig,
)


def load(path: Path) -> BridgeConfig:
    """Read a YAML file and return a fully-populated BridgeConfig."""
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
        nids = s.get("node_ids", {}) or {}
        cfg.flippers = FlippersConfig(
            enabled=bool(s.get("enabled", cfg.flippers.enabled)),
            strategy=s.get("strategy", cfg.flippers.strategy),
            node_ids=FlipperNodeIds(
                fl=nids.get("fl"),
                fr=nids.get("fr"),
                rl=nids.get("rl"),
                rr=nids.get("rr"),
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
