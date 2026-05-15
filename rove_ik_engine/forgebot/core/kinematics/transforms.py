"""4x4 rigid transform helpers built on numpy.

Avoiding spatialmath as a hard dep keeps installs slim. The math here is
the minimum needed for FK: quat <-> matrix, compose, transform points.
"""

from __future__ import annotations

import numpy as np

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


def identity() -> np.ndarray:
    return np.eye(4)


def from_position_quat(p: Vec3, q: Quat) -> np.ndarray:
    """Build a 4x4 transform from position and quaternion (x, y, z, w)."""
    x, y, z, w = q
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-12:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / n, y / n, z / n, w / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    m = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), p[0]],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), p[1]],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), p[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    return m


def axis_angle_to_quat(axis: Vec3, angle: float) -> Quat:
    n = (axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2) ** 0.5
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = np.sin(angle * 0.5) / n
    c = float(np.cos(angle * 0.5))
    return (axis[0] * s, axis[1] * s, axis[2] * s, c)


def joint_offset_transform(joint_type: str, axis: Vec3, value: float) -> np.ndarray:
    """The variable transform a joint applies given its current value.

    For revolute/continuous: rotation about axis by `value` radians.
    For prismatic: translation along axis by `value` meters.
    For fixed: identity.
    """
    if joint_type in ("revolute", "continuous"):
        q = axis_angle_to_quat(axis, value)
        return from_position_quat((0.0, 0.0, 0.0), q)
    if joint_type == "prismatic":
        n = (axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2) ** 0.5 or 1.0
        return from_position_quat(
            (axis[0] / n * value, axis[1] / n * value, axis[2] / n * value),
            (0.0, 0.0, 0.0, 1.0),
        )
    return identity()
