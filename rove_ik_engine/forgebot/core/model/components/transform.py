"""Transform component: position, rotation (quaternion xyzw), scale."""

from __future__ import annotations

from pydantic import Field, field_validator

from .base import BaseComponent, register_component

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # (x, y, z, w)


@register_component("transform")
class TransformComponent(BaseComponent):
    position: Vec3 = (0.0, 0.0, 0.0)
    rotation: Quat = (0.0, 0.0, 0.0, 1.0)
    scale: Vec3 = (1.0, 1.0, 1.0)

    @field_validator("rotation")
    @classmethod
    def _quat_must_be_finite(cls, q: Quat) -> Quat:
        # Allow non-unit quats on input; the FK pipeline normalizes when used.
        # We just guard against NaN/inf which would silently corrupt math later.
        for c in q:
            if not (c == c) or c in (float("inf"), float("-inf")):
                raise ValueError("quaternion contains non-finite component")
        return q
