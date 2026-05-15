"""Convenience commands for the most common transform edits."""

from __future__ import annotations

from typing import cast

from ..model import Project, TransformComponent
from .base import Command


Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


class MoveEntityCommand(Command):
    """Set absolute position on an entity's transform."""

    def __init__(self, entity_id: str, new_position: Vec3) -> None:
        self._entity_id = entity_id
        self._new = new_position

    def execute(self, project: Project) -> None:
        e = project.scene.get(self._entity_id)
        if e is None:
            self._undo_state = {"missing": True}
            return
        t = cast(TransformComponent | None, e.get("transform"))
        if t is None:
            t = TransformComponent()
            e.attach(t)
            self._undo_state = {"missing": False, "added": True, "old_position": (0.0, 0.0, 0.0)}
        else:
            self._undo_state = {"missing": False, "added": False, "old_position": t.position}
        e.attach(TransformComponent(position=self._new, rotation=t.rotation, scale=t.scale))

    def undo(self, project: Project) -> None:
        if self._undo_state is None or self._undo_state.get("missing"):
            return
        e = project.scene.get(self._entity_id)
        if e is None:
            return
        if self._undo_state["added"]:
            e.detach("transform")
            return
        t = cast(TransformComponent, e.get("transform"))
        e.attach(TransformComponent(
            position=tuple(self._undo_state["old_position"]),  # type: ignore[arg-type]
            rotation=t.rotation,
            scale=t.scale,
        ))

    @property
    def description(self) -> str:
        return f"Move {self._entity_id} to {self._new}"


class RotateEntityCommand(Command):
    def __init__(self, entity_id: str, new_rotation: Quat) -> None:
        self._entity_id = entity_id
        self._new = new_rotation

    def execute(self, project: Project) -> None:
        e = project.scene.get(self._entity_id)
        if e is None:
            self._undo_state = {"missing": True}
            return
        t = cast(TransformComponent | None, e.get("transform"))
        if t is None:
            self._undo_state = {"missing": False, "added": True, "old_rotation": (0.0, 0.0, 0.0, 1.0)}
            e.attach(TransformComponent(rotation=self._new))
            return
        self._undo_state = {"missing": False, "added": False, "old_rotation": t.rotation}
        e.attach(TransformComponent(position=t.position, rotation=self._new, scale=t.scale))

    def undo(self, project: Project) -> None:
        if self._undo_state is None or self._undo_state.get("missing"):
            return
        e = project.scene.get(self._entity_id)
        if e is None:
            return
        if self._undo_state["added"]:
            e.detach("transform")
            return
        t = cast(TransformComponent, e.get("transform"))
        e.attach(TransformComponent(
            position=t.position,
            rotation=tuple(self._undo_state["old_rotation"]),  # type: ignore[arg-type]
            scale=t.scale,
        ))

    @property
    def description(self) -> str:
        return f"Rotate {self._entity_id}"
