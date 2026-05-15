"""End effector: gripper, tool, flange."""

from __future__ import annotations

from typing import Literal

from .base import BaseComponent, register_component


EndEffectorKind = Literal[
    "parallel_jaw",
    "suction",
    "magnetic",
    "flange",
    "welding",
    "spray",
    "custom",
]


@register_component("end_effector")
class EndEffectorComponent(BaseComponent):
    kind: EndEffectorKind = "flange"
    payload: float = 0.0       # kg
    grasp_width_min: float = 0.0
    grasp_width_max: float = 0.0
    grasp_force: float = 0.0   # N
    tool_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
