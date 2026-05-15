"""Link component: rigid body with mass, inertia, visual and collision geometry."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from .base import BaseComponent, register_component

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


class InertiaTensor(BaseModel):
    """6-component symmetric inertia tensor (kg·m²)."""

    model_config = ConfigDict(extra="forbid")

    ixx: float = 0.0
    iyy: float = 0.0
    izz: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0


class Inertial(BaseModel):
    """Mass and inertia, plus origin in the link frame."""

    model_config = ConfigDict(extra="forbid")

    mass: float = 0.0
    origin: Vec3 = (0.0, 0.0, 0.0)
    origin_rotation: Quat = (0.0, 0.0, 0.0, 1.0)
    inertia: InertiaTensor = Field(default_factory=InertiaTensor)


class Geometry(BaseModel):
    """One visual or collision shape attached to a link.

    `mesh` is the asset stem name (resolved against the .forgebot archive's
    assets/meshes/ directory, or any directory the loader knows about).
    `primitive` is an alternative for parametric shapes.
    """

    model_config = ConfigDict(extra="forbid")

    mesh: str | None = None
    primitive: str | None = None  # "box" | "sphere" | "cylinder" | None
    primitive_params: dict[str, float] = Field(default_factory=dict)
    material: str | None = None
    origin: Vec3 = (0.0, 0.0, 0.0)
    origin_rotation: Quat = (0.0, 0.0, 0.0, 1.0)
    scale: Vec3 = (1.0, 1.0, 1.0)


@register_component("link")
class LinkComponent(BaseComponent):
    inertial: Inertial = Field(default_factory=Inertial)
    visuals: list[Geometry] = Field(default_factory=list)
    collisions: list[Geometry] = Field(default_factory=list)
