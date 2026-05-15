"""High-level commands that operate on the kinematic topology."""

from __future__ import annotations

from typing import cast

import numpy as np

from ..kinematics import world_transforms
from ..kinematics.transforms import from_position_quat
from ..model import (
    Entity,
    JointComponent,
    Project,
    TransformComponent,
    new_entity_id,
)
from ..model.components.joint import JointLimits
from .base import Command


Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


def _decompose(matrix: np.ndarray) -> tuple[Vec3, Quat]:
    """Pull translation + quaternion (xyzw) out of a 4x4 transform."""
    t = (float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3]))
    # Rotation matrix -> quaternion. Standard formula.
    m = matrix[:3, :3]
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / float(np.sqrt(trace + 1.0))
        w = 0.25 / s
        x = float((m[2, 1] - m[1, 2]) * s)
        y = float((m[0, 2] - m[2, 0]) * s)
        z = float((m[1, 0] - m[0, 1]) * s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * float(np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]))
        w = float((m[2, 1] - m[1, 2]) / s)
        x = 0.25 * s
        y = float((m[0, 1] + m[1, 0]) / s)
        z = float((m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * float(np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]))
        w = float((m[0, 2] - m[2, 0]) / s)
        x = float((m[0, 1] + m[1, 0]) / s)
        y = 0.25 * s
        z = float((m[1, 2] + m[2, 1]) / s)
    else:
        s = 2.0 * float(np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]))
        w = float((m[1, 0] - m[0, 1]) / s)
        x = float((m[0, 2] + m[2, 0]) / s)
        y = float((m[1, 2] + m[2, 1]) / s)
        z = 0.25 * s
    return t, (x, y, z, float(w))


class ConnectJointCommand(Command):
    """Insert a new joint between two existing link entities.

    Topology change:
        before:  parent_link's parent → ?
                                          (parent_link, child_link separate)

        after:   parent_link → joint (new) → child_link

    If `preserve_child_world_pose` is True (the default), the child's local
    transform is rewritten so its world pose is unchanged after the reparent —
    which is what users want when picking surface mates.
    """

    def __init__(
        self,
        *,
        parent_link_id: str,
        child_link_id: str,
        joint_type: str = "revolute",
        axis: Vec3 = (0.0, 0.0, 1.0),
        name: str | None = None,
        position: Vec3 = (0.0, 0.0, 0.0),
        limits: tuple[float, float] | None = (-3.14159, 3.14159),
        preserve_child_world_pose: bool = True,
    ) -> None:
        self._parent_link_id = parent_link_id
        self._child_link_id = child_link_id
        self._joint_type = joint_type
        self._axis = axis
        self._name = name or f"joint_{joint_type}"
        self._position = position
        self._limits = limits
        self._preserve = preserve_child_world_pose

    def execute(self, project: Project) -> None:
        scene = project.scene
        if self._parent_link_id not in scene:
            raise KeyError(f"parent link {self._parent_link_id!r} not in scene")
        if self._child_link_id not in scene:
            raise KeyError(f"child link {self._child_link_id!r} not in scene")

        old_parent = scene.entities[self._child_link_id].parent
        was_root = self._child_link_id in scene.roots

        # Capture the child's existing local + world transform (FK at zero pose)
        # so we can preserve its world pose after reparenting.
        old_child_local: TransformComponent | None = None
        child_world_before: np.ndarray | None = None
        if self._preserve:
            existing = cast(
                TransformComponent | None,
                scene.entities[self._child_link_id].get("transform"),
            )
            old_child_local = (
                TransformComponent(**existing.model_dump(exclude_none=True))
                if existing is not None
                else None
            )
            world = world_transforms(project)
            child_world_before = world.get(self._child_link_id)

        joint_eid = new_entity_id("joint")
        joint_entity = Entity(id=joint_eid, name=self._name)
        joint_entity.attach(TransformComponent(position=self._position))

        limits_obj = None
        if self._limits is not None and self._joint_type in ("revolute", "prismatic"):
            lo, hi = self._limits
            limits_obj = JointLimits(lower=lo, upper=hi, effort=10.0, velocity=1.0)

        joint_entity.attach(
            JointComponent(
                type=self._joint_type,  # type: ignore[arg-type]
                axis=self._axis,
                parent_link=self._parent_link_id,
                child_link=self._child_link_id,
                limits=limits_obj,
            )
        )

        # Insert: parent_link → joint → child_link.
        scene.add(joint_entity, parent=self._parent_link_id)
        scene.reparent(self._child_link_id, joint_eid)

        # Restore child's world pose by rewriting its local transform:
        #   new_local = (joint_world)^-1 * old_child_world
        if self._preserve and child_world_before is not None:
            world_after = world_transforms(project)
            joint_world = world_after.get(joint_eid)
            if joint_world is not None:
                new_local = np.linalg.inv(joint_world) @ child_world_before
                pos, quat = _decompose(new_local)
                child = scene.entities[self._child_link_id]
                existing_t = cast(TransformComponent | None, child.get("transform"))
                scale = existing_t.scale if existing_t is not None else (1.0, 1.0, 1.0)
                child.attach(TransformComponent(position=pos, rotation=quat, scale=scale))

        self._undo_state = {
            "joint_id": joint_eid,
            "old_parent": old_parent,
            "was_root": was_root,
            "old_child_local": (
                old_child_local.model_dump(exclude_none=True) if old_child_local else None
            ),
        }

    def undo(self, project: Project) -> None:
        if self._undo_state is None:
            return
        scene = project.scene
        joint_id = self._undo_state["joint_id"]
        old_parent = self._undo_state["old_parent"]
        old_child_local = self._undo_state.get("old_child_local")

        if self._child_link_id in scene:
            scene.reparent(self._child_link_id, old_parent)
            # Restore the child's original local transform so its world pose
            # matches what it was before this command.
            if old_child_local is not None:
                child = scene.entities[self._child_link_id]
                child.attach(TransformComponent(**old_child_local))
        if joint_id in scene:
            scene.remove(joint_id)

    @property
    def description(self) -> str:
        return f"Connect '{self._name}' ({self._joint_type})"


# Re-export for convenience; some callers import via from_position_quat directly.
__all__ = ["ConnectJointCommand", "from_position_quat"]
