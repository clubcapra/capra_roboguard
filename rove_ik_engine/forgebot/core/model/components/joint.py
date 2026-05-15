"""Joint component: kinematic connection between two links."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from .base import BaseComponent, register_component

Vec3 = tuple[float, float, float]

JointType = Literal[
    "revolute",
    "continuous",
    "prismatic",
    "fixed",
    "floating",
    "planar",
    "ball",
]


class JointLimits(BaseModel):
    model_config = ConfigDict(extra="forbid")

    lower: float = 0.0
    upper: float = 0.0
    effort: float = 0.0      # N·m or N
    velocity: float = 0.0    # rad/s or m/s


class JointDynamics(BaseModel):
    model_config = ConfigDict(extra="forbid")

    damping: float = 0.0
    friction: float = 0.0


@register_component("joint")
class JointComponent(BaseComponent):
    type: JointType = "fixed"
    axis: Vec3 = (0.0, 0.0, 1.0)
    parent_link: str = ""
    child_link: str = ""
    limits: JointLimits | None = None
    dynamics: JointDynamics | None = None
    mimic: str | None = None  # entity id of the joint this one mimics
    mimic_multiplier: float = 1.0
    mimic_offset: float = 0.0
    # `inverted` flips the rotation/translation direction. Used by the
    # editor FK (so you can verify the flip visually) and exported in
    # chain.json so the morpher knows whether to negate the velocity
    # the IK engine emits before sending it to the real arm.
    inverted: bool = False

    # Deprecated. Was once a "real-encoder-at-home" calibration constant
    # but it caused FK to deform the model and broke IK targets. Kept on
    # the model so older .forgebot files with this field still load
    # (Pydantic `extra="forbid"` would otherwise reject them), but
    # nothing in the codebase reads it.
    offset: float = 0.0
