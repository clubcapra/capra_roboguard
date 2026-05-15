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
    # Adds a per-link offset (in the link's LOCAL frame, metres) to the
    # auto-computed mesh-centroid TCP offset. Keys are entity ids OR link
    # names (case-insensitive). Use this to push the IK pivot past the
    # gripper centroid -- e.g. 0.127 m (5") along the gripper's forward
    # axis so the tool's tip becomes the IK reference point.
    tcp_offset_extra: dict[str, list[float]] = field(default_factory=dict)


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

    rove_sensor_api uses a subscribe-push model: the engine sends a
    SUBSCRIBE packet to (sensor_api_host, kinova_data_port) and the sensor
    pushes DATA frames back to our ephemeral port at `subscribe_interval_ms`.

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
    # rove_sensor_api endpoint. Defaults match the standard kinova driver:
    # data_port = 5002 (UDP), reachable from the engine host.
    sensor_api_host: str = "127.0.0.1"
    kinova_data_port: int = 5002
    subscribe_interval_ms: int = 100   # 10 Hz push, plenty for sync

    # Chain mode (preferred). When both are set, takes priority over joint_names.
    arm_base_entity_id: str = ""
    arm_tip_entity_id: str = ""

    # Name mode (fallback). Joint names in kinova actuator-index order.
    joint_names: list[str] = field(default_factory=list)

    # Kinova actuator indices (1..N) whose rotation axis is inverted
    # relative to the model's URDF axis. After Sync the engine multiplies
    # readings for these joints by -1 so the mirror direction matches.
    inverted_joints: list[int] = field(default_factory=list)

    # ---- velocity output to kinova_arm ----
    # SAFETY: leave disabled until mirror direction is verified for every
    # joint by physically moving the arm and watching the model. When on,
    # the engine sends per-tick MSG_COMMAND packets with joint_N_vel (deg/s)
    # to (sensor_api_host, kinova_cmd_port) whenever any IK-derived qdot is
    # non-zero. Silence -> kinova's own 300 ms velocity-hold timeout halts.
    vel_output_enabled: bool = False
    kinova_cmd_port: int = 5003
    max_kinova_vel_deg_s: float = 20.0
    # Velocities below this magnitude are treated as zero (no packet sent).
    # Keeps IK floating-point dust from continuously poking the arm.
    min_vel_deg_s: float = 0.05


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
