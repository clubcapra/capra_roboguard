"""Project: top-level container — manifest, scene, systems, simulation, assets."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from .bindings import Bindings
from .scene import Scene
from .units import Units


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class Metadata(BaseModel):
    model_config = ConfigDict(extra="allow")

    name: str = "Untitled"
    author: str = ""
    description: str = ""
    created: datetime = Field(default_factory=_utcnow)
    modified: datetime = Field(default_factory=_utcnow)
    tags: list[str] = Field(default_factory=list)


class Storage(BaseModel):
    """How `scene` is encoded inside the .forgebot archive."""

    model_config = ConfigDict(extra="forbid")

    scene_format: str = "toml"  # "toml" or "msgpack"


class Manifest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    forgebot_version: str = "1.0.0"
    metadata: Metadata = Field(default_factory=Metadata)
    units: Units = Field(default_factory=Units)
    storage: Storage = Field(default_factory=Storage)


class Material(BaseModel):
    model_config = ConfigDict(extra="forbid")

    color: tuple[float, float, float, float] = (0.8, 0.8, 0.8, 1.0)
    metallic: float = 0.0
    roughness: float = 0.5
    texture: str | None = None  # asset stem name


class KinematicChainSpec(BaseModel):
    """Declared kinematic chain: base link, tip link, joints in order."""

    model_config = ConfigDict(extra="forbid")

    base: str
    tip: str
    joints: list[str] = Field(default_factory=list)


class IKProfile(BaseModel):
    """Tuned IK parameters for one chain (keyed in `Project.ik_profiles` by
    base link entity id). Produced by the IK tuner; consumed by `IKGizmo`.

    `mode` selects the task formulation:
      - "pose_locked": position+orientation in the primary task (legacy).
      - "pos_primary": position primary, orientation as null-space secondary
        (better for non-spherical-wrist arms — orientation tracks where the
        redundancy allows, doesn't force large joint motion at singularities).
    """

    model_config = ConfigDict(extra="forbid")

    mode: str = "pose_locked"
    damping: float = 0.05
    rest_pose_gain: float = 0.3
    max_iter: int = 60
    orientation_weight: float = 5.0
    joint_weight_strength: float = 0.0
    max_dq_step: float = 0.05
    max_pos_step: float = 0.05
    # Caps |q_final - q_initial|_inf per IK call. None = no cap. Set to a
    # small number (e.g. 0.10 rad ≈ 6°) to bound how much the chain can move
    # in a single call — turns pose_locked from "explodes from singularity"
    # into "smooth catch-up" on non-spherical-wrist arms.
    max_total_dq_step: float | None = 0.10
    # Only used in pos_primary mode; null-space orientation pull strength.
    orientation_secondary_gain: float = 0.5
    # Score the tuner achieved (lower is better) and per-component breakdown.
    score: float = 0.0
    pos_err_max_mm: float = 0.0
    rot_drift_deg: float = 0.0
    max_jump_rad: float = 0.0
    total_motion_rad: float = 0.0
    saturated_joints: int = 0
    new_collision_pairs: int = 0
    # ISO-8601 timestamp (string for TOML friendliness).
    tuned_at: str = ""


class Systems(BaseModel):
    """Cross-cutting: chains, signals, layout, flow paths.

    Signal graph and layout are stored as raw dicts in phase 1; they get
    proper Pydantic models in phase 3 when their UI lands.
    """

    model_config = ConfigDict(extra="forbid")

    kinematic_chains: dict[str, KinematicChainSpec] = Field(default_factory=dict)
    signal_graph: dict[str, Any] = Field(default_factory=dict)
    layout: dict[str, Any] = Field(default_factory=dict)


class PhysicsConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    engine: str = "default"
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.81)
    timestep: float = 0.001


class Simulation(BaseModel):
    model_config = ConfigDict(extra="forbid")

    physics: PhysicsConfig = Field(default_factory=PhysicsConfig)
    controllers: dict[str, Any] = Field(default_factory=dict)


class MeshAsset(BaseModel):
    """A mesh's binary content + suffix. Suffix is needed to dispatch loaders
    on save (we have to write `.stl`/`.obj`/`.glb` etc) and on the frontend.
    """

    model_config = ConfigDict(extra="forbid")

    suffix: str = ".stl"
    data: bytes = b""  # never serialized through TOML — only ZIP


class Assets(BaseModel):
    """Materials live in TOML; meshes/textures live as binary blobs in memory
    and as binary files in the .forgebot ZIP.

    `mesh_data` is the canonical source of mesh bytes (keyed by stem name).
    `mesh_files` is a fallback map of stem -> external file path, used by
    importers to point at on-disk meshes that haven't been read into memory yet.
    Exporters prefer `mesh_data` when available.
    """

    model_config = ConfigDict(extra="forbid")

    materials: dict[str, Material] = Field(default_factory=dict)
    mesh_data: dict[str, MeshAsset] = Field(default_factory=dict)
    mesh_files: dict[str, str] = Field(default_factory=dict)  # stem -> external path
    texture_data: dict[str, MeshAsset] = Field(default_factory=dict)
    texture_files: dict[str, str] = Field(default_factory=dict)


class Project(BaseModel):
    model_config = ConfigDict(extra="forbid", validate_assignment=True)

    manifest: Manifest = Field(default_factory=Manifest)
    scene: Scene = Field(default_factory=Scene)
    systems: Systems = Field(default_factory=Systems)
    simulation: Simulation = Field(default_factory=Simulation)
    assets: Assets = Field(default_factory=Assets)
    # Named "home" pose: snapshot of joint slider values the user can recall.
    # Keys are joint entity ids; values are slider-space (so don't include
    # JointComponent.offset — apply offset only inside FK).
    home_pose: dict[str, float] = Field(default_factory=dict)
    # Per-chain tuned IK params; key is the chain base link entity id.
    ik_profiles: dict[str, IKProfile] = Field(default_factory=dict)
    # Mapping from external proto fields / sensor stream IDs to entities
    # in this project. Lets Simulate / Map / robot runtime drive joints
    # from telemetry, push control back, and place lidar / camera frames
    # in the world without hardcoded IDs. See `core/model/bindings.py`.
    bindings: Bindings = Field(default_factory=lambda: Bindings())

    def touch(self) -> None:
        self.manifest.metadata.modified = _utcnow()
