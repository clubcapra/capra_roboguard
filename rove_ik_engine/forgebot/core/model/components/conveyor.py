"""Conveyor component: belt/roller/turntable for material transport."""

from __future__ import annotations

from typing import Literal

from .base import BaseComponent, register_component


ConveyorKind = Literal["belt", "roller", "turntable", "buffer"]


@register_component("conveyor")
class ConveyorComponent(BaseComponent):
    kind: ConveyorKind = "belt"
    speed: float = 0.5     # m/s (or rad/s for turntable)
    width: float = 0.4     # m
    length: float = 1.0    # m
    direction: tuple[float, float, float] = (1.0, 0.0, 0.0)
    bidirectional: bool = False
