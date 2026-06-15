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
    # Drive teleop (flipper steps + drum velocities) from the control bridge,
    # JSON over UDP. Mirror of the Ovis (arm) input on its own port.
    drive_udp_enabled: bool = True
    drive_udp_bind: str = "0.0.0.0:9102"


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
class FlipperNode:
    """One drive: an ODrive node feeding one revolute joint entity."""

    node_id: int                 # ODrive node id (31-34 drums, 41-44 flippers)
    joint: str                   # model joint entity NAME (e.g. "FlipperFL")
    data_port: int               # rove_sensor_api served data port for this node
    sign: float = 1.0            # +1/-1 to align motor direction with model axis
    # Per-node override of the bank gear ratio (motor revs per joint rev).
    # None -> use FlippersConfig.gear_ratio.
    gear_ratio: float | None = None
    # ---- position output (commanding) ----
    # Whether this node may receive position commands. Drums stay False; only
    # flippers opt in. Still gated by FlippersConfig.output_enabled (master).
    output: bool = False
    cmd_port: int = 0            # rove_sensor_api served COMMAND port for this node
    # Command mode: "position" (flippers — input_pos rev, control_mode 3) or
    # "velocity" (drums — input_vel rev/s, control_mode 2). Drums are continuous
    # drive wheels and must be velocity-commanded.
    mode: str = "position"
    max_vel_rev_s: float = 10.0  # velocity mode: motor rev/s at command = 1.0
    min_deg: float | None = None  # position mode soft travel limit (model joint deg)
    max_deg: float | None = None


@dataclass
class FlippersConfig:
    """Read-only mirror of the flipper ODrives (41..44) into the model.

    Same subscribe-push transport as the kinova bridge, but one subscriber
    per ODrive node (each flipper is an independent drive). Each node reports
    `pos_estimate` in MOTOR revolutions; the model joint angle is
    `sign * (2*pi / gear_ratio) * pos`. The gear ratio is a hardware value
    that is not yet known precisely, so it's a tunable here; verify the
    mirror direction/scale by physically moving a flipper and watching the
    model before trusting it.

    Only 41/42 are wired on the current robot (43/44 are fried) — list just
    the live nodes; a missing node is simply never mirrored.
    """

    enabled: bool = False
    sensor_api_host: str = "127.0.0.1"
    subscribe_interval_ms: int = 100
    # Resolve each node's data/cmd ports from the robot's GET /discover at
    # startup (ports are assigned by boot order and drift between restarts).
    # Strongly recommended ON — a stale COMMAND port could drive the wrong
    # motor. Falls back to the configured ports if discovery fails.
    discover: bool = True
    discover_http_port: int = 8080
    # Flippers lose their encoder zero on power-cycle but can't physically move
    # (worm gear). When True, restore the persisted physical angle and re-anchor
    # the offset from the first frame, so a synced flipper survives a reboot.
    reanchor_on_boot: bool = True
    # Default motor revs per joint revolution. TUNABLE — real gearbox ratio TBD.
    gear_ratio: float = 1.0
    nodes: list[FlipperNode] = field(default_factory=list)
    # ---- position output (commanding) ----
    # MASTER SAFETY GATE for sending flipper position commands. Leave False
    # (the engine computes targets + moves the model, but emits NO packets).
    # Set True only against a target you intend to actuate.
    output_enabled: bool = False
    # How fast a held +1/-1 step ramps a flipper (model joint deg per second).
    step_rate_deg_s: float = 20.0
    # Drive watchdog: if no drive frame (drums/flipper steps) arrives within this
    # many seconds, the engine self-stops the drives — zero drum velocity and halt
    # flipper ramps. Clones the old rove_control_bridge_py idle watchdog (0.5 s).
    # Make it comfortably longer than the worst-case inter-packet gap on the
    # (low-bandwidth) operator link, or it will stop-start during driving.
    drive_idle_timeout_s: float = 0.5


@dataclass
class EngineConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    ik: IKConfig = field(default_factory=IKConfig)
    input: InputConfig = field(default_factory=InputConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    flippers: FlippersConfig = field(default_factory=FlippersConfig)
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
    # The drive-mirror bank covers all position-fed ODrives (drums + flippers).
    # Preferred key is [drives]; [flippers] is still accepted for compatibility.
    drive_key = "drives" if "drives" in data else ("flippers" if "flippers" in data else None)
    if drive_key is not None:
        fdata = dict(data[drive_key])
        # Nested [[drives.node]] / [[flippers.node]] tables arrive under "node".
        node_dicts = fdata.pop("node", []) or fdata.pop("nodes", [])
        cfg.flippers = FlippersConfig(**fdata)
        cfg.flippers.nodes = [FlipperNode(**n) for n in node_dicts]
    return cfg


def resolve(cfg: EngineConfig, rel: str) -> Path:
    return (cfg.root / rel).resolve()


def parse_bind(bind: str) -> tuple[str, int]:
    host, _, port = bind.rpartition(":")
    if not host or not port:
        raise ValueError(f"expected 'host:port', got {bind!r}")
    return host, int(port)
