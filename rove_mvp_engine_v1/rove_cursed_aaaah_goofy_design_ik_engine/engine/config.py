"""Parse engine.toml into typed dataclasses."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class RobotConfig:
    forgebot: str = "data/scene.forgebot"
    urdf: str = "data/robot.urdf"


@dataclass
class IKConfig:
    collision_aware: bool = True
    twist_frame: str = "world"  # "world" | "target"
    max_lin_vel: float = 0.25
    max_ang_vel: float = 1.0
    rate_hz: float = 30.0
    debug: bool = False


@dataclass
class InputConfig:
    udp_enabled: bool = True
    udp_bind: str = "0.0.0.0:9100"
    ws_enabled: bool = True
    ws_bind: str = "0.0.0.0:9101"
    ws_path: str = "/ovis"


@dataclass
class OutputConfig:
    udp_enabled: bool = False
    udp_target: str = "127.0.0.1:9200"
    ws_enabled: bool = True
    ws_path: str = "/state"
    stdout_enabled: bool = False


@dataclass
class EngineConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    ik: IKConfig = field(default_factory=IKConfig)
    input: InputConfig = field(default_factory=InputConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    root: Path = field(default_factory=lambda: Path.cwd())


def load(path: Path) -> EngineConfig:
    data = tomllib.loads(path.read_text())
    cfg = EngineConfig(root=path.parent.resolve())
    if "robot" in data:
        cfg.robot = RobotConfig(**data["robot"])
    if "ik" in data:
        cfg.ik = IKConfig(**data["ik"])
        if cfg.ik.twist_frame not in ("world", "target"):
            raise ValueError(
                f"[ik].twist_frame must be 'world' or 'target', got "
                f"{cfg.ik.twist_frame!r}"
            )
    if "input" in data:
        cfg.input = InputConfig(**data["input"])
    if "output" in data:
        cfg.output = OutputConfig(**data["output"])
    return cfg


def resolve(cfg: EngineConfig, rel: str) -> Path:
    return (cfg.root / rel).resolve()


def parse_bind(bind: str) -> tuple[str, int]:
    host, _, port = bind.rpartition(":")
    if not host or not port:
        raise ValueError(f"expected 'host:port', got {bind!r}")
    return host, int(port)
