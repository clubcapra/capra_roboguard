"""Binding manifest — maps robot proto fields to entities in the project.

Embedded in `.forgebot` archives as `bindings.toml`. Optional. Lets the
Simulate and Map pages drive joint values from telemetry, push control
back, and identify which lidar/camera in the world frame each stream
corresponds to — without hardcoding entity IDs in the runtime.

Two halves: `telemetry` (proto field → entity field) and `control`
(proto field → entity action). Schema deliberately permissive so robot
authors can add new field paths without touching code.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class TelemetryBinding(BaseModel):
    """Drive an entity field from a proto telemetry path."""

    model_config = ConfigDict(extra="forbid")

    entity: str
    field: str = "motor_pos"
    scale: float = 1.0
    offset: float = 0.0


ControlMode = Literal[
    "increment",
    "ik_target",
    "open_close",
    "velocity",
    "absolute",
]


class ControlBinding(BaseModel):
    """Map a proto control field to an entity action."""

    model_config = ConfigDict(extra="forbid")

    entity: str
    mode: ControlMode = "absolute"
    scale: float = 1.0
    normalize: str | None = None


class CameraIntrinsics(BaseModel):
    """Pinhole intrinsics for an RTSP camera. The Simulate panel only
    edits H-FOV + aspect today; fx/fy/cx/cy/distortion are reserved for
    a calibration pass (e.g. checkerboard target → solvePnP) so the
    lidar-color projection used by the splat pipeline can plug them in
    without a schema change."""

    model_config = ConfigDict(extra="forbid")

    image_width: int = 1920
    image_height: int = 1080
    # Horizontal FOV in degrees. Vertical FOV is derived from H-FOV +
    # the resolution aspect when fy is not explicitly set.
    fov_h_deg: float = 90.0
    # Pixel-space pinhole parameters. None = derive from fov_h_deg + image
    # size, assuming square pixels and centered principal point. A real
    # calibration overrides all four.
    fx: float | None = None
    fy: float | None = None
    cx: float | None = None
    cy: float | None = None
    # Brown-Conrady distortion. Zero = ideal pinhole.
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    p1: float = 0.0
    p2: float = 0.0


class SensorBinding(BaseModel):
    """Identify an external sensor stream by name and bind it to an
    entity in the world. The entity's transform tells us the sensor's
    position; mount_rpy_deg is the sensor's broadcast-frame rotation
    (independent of the URDF link rotation, which usually encodes
    visual-mesh placement, not the sensor's coordinate frame). Default
    [0,0,0] = sensor's local +Z aligned with world +Z (upright).

    For kind='camera', `intrinsics` carries the pinhole parameters used
    by the splat-color pipeline to project lidar points into image space."""

    model_config = ConfigDict(extra="forbid")

    entity: str
    stream_id: str
    kind: Literal["lidar", "camera", "imu"]
    mount_rpy_deg: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    intrinsics: CameraIntrinsics | None = None


class Bindings(BaseModel):
    model_config = ConfigDict(extra="forbid")

    telemetry: dict[str, TelemetryBinding] = Field(default_factory=dict)
    control: dict[str, ControlBinding] = Field(default_factory=dict)
    sensors: dict[str, SensorBinding] = Field(default_factory=dict)
