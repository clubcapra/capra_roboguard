"""Forward kinematics: compute world transforms for every entity in a scene.

The model itself stores only local (parent-relative) transforms. FK walks
the scene tree and accumulates them. Joint values are passed in as a dict
{joint_entity_id: value (radians or meters)}.
"""

from __future__ import annotations

from typing import cast

import numpy as np

from ..model import JointComponent, Project, TransformComponent
from .transforms import from_position_quat, identity, joint_offset_transform


def world_transforms(
    project: Project,
    joint_values: dict[str, float] | None = None,
) -> dict[str, np.ndarray]:
    """Compute world-frame 4x4 transforms for every entity.

    Roots get their local transform (or identity if they don't have one).
    Children get parent_world @ local @ joint_offset (joint_offset only
    applies if the entity carries a `joint` component).
    """
    jv = joint_values or {}
    scene = project.scene
    out: dict[str, np.ndarray] = {}

    def visit(eid: str, parent_world: np.ndarray) -> None:
        e = scene.entities[eid]
        local = _local_transform(e)
        joint = cast(JointComponent | None, e.get("joint"))
        if joint is not None and joint.type != "fixed":
            # FK applies *only* the direction flip — never the offset. Offset
            # is a morpher-side constant (see JointComponent docstring); if
            # we added it here, setting calibration would visually deform
            # the model and the IK targets stored at slider-zero would
            # become unreachable. Keep editor FK clean: sign * slider.
            sign = -1.0 if getattr(joint, "inverted", False) else 1.0
            actual = sign * jv.get(eid, 0.0)
            local = local @ joint_offset_transform(joint.type, joint.axis, actual)
        world = parent_world @ local
        out[eid] = world
        for child_id in e.children:
            if child_id in scene.entities:
                visit(child_id, world)

    for root_id in scene.roots:
        if root_id in scene.entities:
            visit(root_id, identity())
    return out


def _local_transform(entity) -> np.ndarray:
    t = cast(TransformComponent | None, entity.get("transform"))
    if t is None:
        return identity()
    return from_position_quat(t.position, t.rotation)
