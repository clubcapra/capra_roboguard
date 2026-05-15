"""Controller component: PID/trajectory/PLC reference attached to an entity."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from .base import BaseComponent, register_component


ControllerKind = Literal[
    "joint_trajectory",
    "joint_position",
    "joint_velocity",
    "cartesian",
    "plc",
    "custom",
]


class ControllerGains(BaseModel):
    model_config = ConfigDict(extra="forbid")

    p: float = 0.0
    i: float = 0.0
    d: float = 0.0


@register_component("controller")
class ControllerComponent(BaseComponent):
    kind: ControllerKind = "joint_position"
    target: str = ""  # chain id or entity id depending on kind
    gains: ControllerGains = Field(default_factory=ControllerGains)
