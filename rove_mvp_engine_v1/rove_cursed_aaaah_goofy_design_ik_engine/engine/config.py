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
class HardwareConfig:
    """Optional bridge to rove_sensor_api's `kinova_arm` sensor.

    When configured, the engine binds `state_listen_port` and waits for the
    sensor_api to push UDP state datagrams to it. The frontend's "Sync"
    button reads the latest received frame and snaps the model's joint
    values to match the real arm.

    Two ways to map kinova actuator index -> engine joint entity:

    1. CHAIN MODE (preferred when joints share names like "joint_revolute"):
       Set `arm_base_entity_id` and `arm_tip_entity_id`. The engine walks
       the kinematic chain between them and assigns the N joints to kinova
       actuators 1..N (base -> tip).

    2. NAME MODE: Set `joint_names` to a list ordered by kinova actuator
       index. The engine looks up `Entity.name` (case-insensitive).
       Useful when joint names are unique.
    """

    enabled: bool = False
    state_listen_port: int = 9300

    # Chain mode (preferred). When both are set, takes priority over joint_names.
    arm_base_entity_id: str = ""
    arm_tip_entity_id: str = ""

    # Name mode (fallback). Joint names in kinova actuator-index order.
    joint_names: list[str] = field(default_factory=list)


@dataclass
class EngineConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    ik: IKConfig = field(default_factory=IKConfig)
    input: InputConfig = field(default_factory=InputConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    hardware: HardwareConfig = field(default_factory=HardwareConfig)
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
    if "hardware" in data:
        cfg.hardware = HardwareConfig(**data["hardware"])
    return cfg


def resolve(cfg: EngineConfig, rel: str) -> Path:
    return (cfg.root / rel).resolve()


def parse_bind(bind: str) -> tuple[str, int]:
    host, _, port = bind.rpartition(":")
    if not host or not port:
        raise ValueError(f"expected 'host:port', got {bind!r}")
    return host, int(port)
