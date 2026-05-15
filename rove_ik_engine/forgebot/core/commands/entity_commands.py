"""Commands that add, remove, and reparent entities."""

from __future__ import annotations

from typing import cast

import numpy as np

from ..kinematics import world_transforms
from ..model import BaseComponent, Entity, Project, TransformComponent
from ..model.components import parse_component
from .base import Command
from .joint_commands import _decompose


class AddEntityCommand(Command):
    def __init__(
        self,
        entity: Entity,
        parent: str | None = None,
    ) -> None:
        self._entity = entity
        self._parent = parent

    def execute(self, project: Project) -> None:
        # If somehow already present, snapshot it so undo can restore.
        if self._entity.id in project.scene.entities:
            existing = project.scene.entities[self._entity.id]
            self._undo_state = {"existed": True, "snapshot": _snapshot(existing)}
            return
        project.scene.add(self._entity, parent=self._parent)
        self._undo_state = {"existed": False}

    def undo(self, project: Project) -> None:
        if self._undo_state is None:
            return
        if not self._undo_state["existed"]:
            project.scene.remove(self._entity.id)

    @property
    def description(self) -> str:
        name = self._entity.name or self._entity.id
        return f"Add entity '{name}'"


class RemoveEntityCommand(Command):
    """Remove an entity and its descendants. Undo restores the whole subtree."""

    def __init__(self, entity_id: str) -> None:
        self._entity_id = entity_id

    def execute(self, project: Project) -> None:
        scene = project.scene
        e = scene.get(self._entity_id)
        if e is None:
            self._undo_state = {"snapshots": [], "parent": None, "roots": list(scene.roots)}
            return
        # Capture every descendant + the original parent so we can rebuild.
        snapshots: list[dict] = []
        for desc in scene.iter_dfs(self._entity_id):
            snapshots.append(_snapshot(desc))
        self._undo_state = {
            "snapshots": snapshots,
            "parent": e.parent,
            "roots": list(scene.roots),
        }
        scene.remove(self._entity_id)

    def undo(self, project: Project) -> None:
        if self._undo_state is None:
            return
        snapshots: list[dict] = self._undo_state["snapshots"]
        if not snapshots:
            return
        # Restore root of the removed subtree first, then descendants.
        # Snapshots are in DFS order so processing them in order works.
        scene = project.scene
        for snap in snapshots:
            entity = _from_snapshot(snap)
            scene.entities[entity.id] = entity
        # Restore the root's parent linkage.
        root_snap = snapshots[0]
        root_id = root_snap["id"]
        root = scene.entities[root_id]
        parent_id = self._undo_state["parent"]
        if parent_id is None:
            if root_id not in scene.roots:
                scene.roots.append(root_id)
        else:
            parent = scene.get(parent_id)
            if parent is not None and root_id not in parent.children:
                parent.children.append(root_id)
            root.parent = parent_id

    @property
    def description(self) -> str:
        return f"Remove entity {self._entity_id}"


class RemoveEntityKeepChildrenCommand(Command):
    """Remove only the named entity. Its children are reparented to its
    grandparent (or become roots), and each child's local transform is
    rewritten so its world pose is unchanged.

    Use this for "delete this joint, but keep both links it connected" and
    similar surgical edits. For a recursive subtree delete, use
    `RemoveEntityCommand`.
    """

    def __init__(self, entity_id: str) -> None:
        self._entity_id = entity_id

    def execute(self, project: Project) -> None:
        scene = project.scene
        e = scene.get(self._entity_id)
        if e is None:
            self._undo_state = {"missing": True}
            return

        old_parent = e.parent
        was_root = self._entity_id in scene.roots
        child_ids = list(e.children)

        # Snapshot the entity itself + every child's pre-move local transform.
        self_snap = _snapshot(e)
        old_child_locals: dict[str, dict | None] = {}
        for cid in child_ids:
            child = scene.entities.get(cid)
            if child is None:
                continue
            t = cast(TransformComponent | None, child.get("transform"))
            old_child_locals[cid] = (
                t.model_dump(exclude_none=True) if t is not None else None
            )

        # World transforms BEFORE we mutate the tree — used to preserve each
        # child's world pose after reparent.
        worlds_before = world_transforms(project)
        new_parent_world = worlds_before.get(old_parent) if old_parent else None
        new_parent_inv = (
            np.linalg.inv(new_parent_world) if new_parent_world is not None else np.eye(4)
        )

        # Reparent each child to the grandparent (or root) and rewrite its
        # local transform so its world pose is preserved.
        for cid in child_ids:
            child = scene.entities.get(cid)
            if child is None:
                continue
            child_world = worlds_before.get(cid)
            if child_world is not None:
                new_local = new_parent_inv @ child_world
                pos, quat = _decompose(new_local)
                existing_t = cast(TransformComponent | None, child.get("transform"))
                scale = existing_t.scale if existing_t is not None else (1.0, 1.0, 1.0)
                child.attach(TransformComponent(position=pos, rotation=quat, scale=scale))
            scene.reparent(cid, old_parent)

        # By now `e.children` is empty (every child was reparented elsewhere)
        # so scene.remove only removes the one entity.
        scene.remove(self._entity_id)

        self._undo_state = {
            "missing": False,
            "self_snapshot": self_snap,
            "old_parent": old_parent,
            "was_root": was_root,
            "old_child_locals": old_child_locals,
            "child_order": child_ids,
        }

    def undo(self, project: Project) -> None:
        state = self._undo_state
        if state is None or state.get("missing"):
            return
        scene = project.scene

        # Re-create the entity from snapshot. Force `children=[]` initially —
        # we'll reparent below, which re-populates it via scene.reparent.
        snap = state["self_snapshot"]
        entity = _from_snapshot(snap)
        entity.children = []
        scene.entities[entity.id] = entity

        old_parent = state["old_parent"]
        if old_parent is None:
            if entity.id not in scene.roots:
                scene.roots.append(entity.id)
        else:
            parent_obj = scene.entities.get(old_parent)
            if parent_obj is not None and entity.id not in parent_obj.children:
                parent_obj.children.append(entity.id)
            entity.parent = old_parent

        # Reparent the original children back to this entity, preserving the
        # original child order; restore each one's pre-execute local transform.
        for cid in state["child_order"]:
            child = scene.entities.get(cid)
            if child is None:
                continue
            scene.reparent(cid, entity.id)
            old_local = state["old_child_locals"].get(cid)
            if old_local is not None:
                child.attach(TransformComponent(**old_local))
            else:
                child.detach("transform")

    @property
    def description(self) -> str:
        return f"Remove entity {self._entity_id} (keep children)"


class ReparentCommand(Command):
    def __init__(self, entity_id: str, new_parent: str | None) -> None:
        self._entity_id = entity_id
        self._new_parent = new_parent

    def execute(self, project: Project) -> None:
        e = project.scene.get(self._entity_id)
        if e is None:
            self._undo_state = {"old_parent": None, "missing": True}
            return
        self._undo_state = {"old_parent": e.parent, "missing": False}
        project.scene.reparent(self._entity_id, self._new_parent)

    def undo(self, project: Project) -> None:
        if self._undo_state is None or self._undo_state.get("missing"):
            return
        project.scene.reparent(self._entity_id, self._undo_state["old_parent"])

    @property
    def description(self) -> str:
        return f"Reparent {self._entity_id} -> {self._new_parent or '<root>'}"


def _snapshot(e: Entity) -> dict:
    """Produce a JSON-able snapshot of an entity (for undo)."""
    return {
        "id": e.id,
        "name": e.name,
        "parent": e.parent,
        "children": list(e.children),
        "components": {
            key: {
                "key": key,
                "data": comp.model_dump(exclude_none=True),
            }
            for key, comp in e.components.items()
        },
    }


def _from_snapshot(snap: dict) -> Entity:
    components: dict[str, BaseComponent] = {}
    for key, payload in snap["components"].items():
        components[key] = parse_component(key, payload["data"])
    return Entity(
        id=snap["id"],
        name=snap["name"],
        parent=snap["parent"],
        children=list(snap["children"]),
        components=components,
    )
