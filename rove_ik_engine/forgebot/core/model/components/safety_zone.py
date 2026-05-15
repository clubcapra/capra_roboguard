"""Safety zone: bounded region with classification and behavior policy."""

from __future__ import annotations

from typing import Literal

from pydantic import Field

from .base import BaseComponent, register_component


SafetyClass = Literal["fence", "cell", "collaborative", "exclusion", "storage", "custom"]


@register_component("safety_zone")
class SafetyZoneComponent(BaseComponent):
    safety_class: SafetyClass = "cell"
    boundary: list[tuple[float, float]] = Field(default_factory=list)  # 2D polygon (x, y)
    height: float = 2.5  # m, extruded vertically
    speed_limit: float = 0.0  # m/s; 0 = no limit
    on_breach: str = "stop"   # "stop" | "slow" | "warn" | "ignore"
