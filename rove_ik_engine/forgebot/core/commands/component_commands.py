"""Commands that attach, detach, or update components on an entity."""

from __future__ import annotations

from typing import Any

from ..model import BaseComponent, Project
from ..model.components import parse_component
from .base import Command


class AttachComponentCommand(Command):
    def __init__(self, entity_id: str, component: BaseComponent) -> None:
        self._entity_id = entity_id
        self._component = component

    def execute(self, project: Project) -> None:
        e = project.scene.get(self._entity_id)
        if e is None:
            self._undo_state = {"missing": True}
            return
        key = self._component.component_key
        prev = e.components.get(key)
        self._undo_state = {
            "key": key,
            "had_previous": prev is not None,
            "previous": prev.model_dump(exclude_none=True) if prev else None,
            "missing": False,
        }
        e.attach(self._component)

    def undo(self, project: Project) -> None:
        if self._undo_state is None or self._undo_state.get("missing"):
            return
        e = project.scene.get(self._entity_id)
        if e is None:
            return
        key = self._undo_state["key"]
        if self._undo_state["had_previous"]:
            e.attach(parse_component(key, self._undo_state["previous"]))
        else:
            e.detach(key)

    @property
    def description(self) -> str:
        return f"Attach {self._component.component_key} to {self._entity_id}"


class DetachComponentCommand(Command):
    def __init__(self, entity_id: str, component_key: str) -> None:
        self._entity_id = entity_id
        self._key = component_key

    def execute(self, project: Project) -> None:
        e = project.scene.get(self._entity_id)
        if e is None:
            self._undo_state = {"missing": True}
            return
        prev = e.components.get(self._key)
        if prev is None:
            self._undo_state = {"missing": False, "had_previous": False}
            return
        self._undo_state = {
            "missing": False,
            "had_previous": True,
            "previous": prev.model_dump(exclude_none=True),
        }
        e.detach(self._key)

    def undo(self, project: Project) -> None:
        if self._undo_state is None or self._undo_state.get("missing"):
            return
        if not self._undo_state.get("had_previous"):
            return
        e = project.scene.get(self._entity_id)
        if e is None:
            return
        e.attach(parse_component(self._key, self._undo_state["previous"]))

    @property
    def description(self) -> str:
        return f"Detach {self._key} from {self._entity_id}"


class UpdateComponentCommand(Command):
    """Apply a partial update (a dict of field overrides) to one component."""

    def __init__(self, entity_id: str, component_key: str, updates: dict[str, Any]) -> None:
        self._entity_id = entity_id
        self._key = component_key
        self._updates = updates

    def execute(self, project: Project) -> None:
        e = project.scene.get(self._entity_id)
        if e is None or self._key not in e.components:
            self._undo_state = {"missing": True}
            return
        prev = e.components[self._key]
        self._undo_state = {
            "missing": False,
            "previous": prev.model_dump(exclude_none=True),
        }
        merged = {**self._undo_state["previous"], **self._updates}
        e.attach(parse_component(self._key, merged))

    def undo(self, project: Project) -> None:
        if self._undo_state is None or self._undo_state.get("missing"):
            return
        e = project.scene.get(self._entity_id)
        if e is None:
            return
        e.attach(parse_component(self._key, self._undo_state["previous"]))

    @property
    def description(self) -> str:
        return f"Update {self._key}.{','.join(self._updates)} on {self._entity_id}"
