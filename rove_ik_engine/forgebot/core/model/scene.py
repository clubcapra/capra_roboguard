"""Scene: a tree of entities with O(1) ID lookup."""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from .components import BaseComponent, parse_component
from .entity import Entity, new_entity_id


class Scene(BaseModel):
    """A flat dict of entities keyed by ID, plus a list of root IDs.

    Parent/child links live on the entities themselves. The scene exposes
    helpers for traversal, ancestry, and structural edits.
    """

    model_config = ConfigDict(extra="forbid", validate_assignment=True)

    entities: dict[str, Entity] = Field(default_factory=dict)
    roots: list[str] = Field(default_factory=list)

    # ----- queries -----

    def __contains__(self, entity_id: str) -> bool:
        return entity_id in self.entities

    def __len__(self) -> int:
        return len(self.entities)

    def get(self, entity_id: str) -> Entity | None:
        return self.entities.get(entity_id)

    def require(self, entity_id: str) -> Entity:
        e = self.entities.get(entity_id)
        if e is None:
            raise KeyError(f"entity not found: {entity_id}")
        return e

    def find_by_name(self, name: str) -> Entity | None:
        for e in self.entities.values():
            if e.name == name:
                return e
        return None

    def iter_dfs(self, start: str | None = None) -> Iterator[Entity]:
        """Depth-first traversal. If `start` is None, walk every root."""
        starts = [start] if start is not None else list(self.roots)
        stack: list[str] = list(starts)
        while stack:
            eid = stack.pop()
            e = self.entities.get(eid)
            if e is None:
                continue
            yield e
            stack.extend(reversed(e.children))

    def ancestors(self, entity_id: str) -> list[str]:
        out: list[str] = []
        cur = self.entities.get(entity_id)
        while cur is not None and cur.parent is not None:
            out.append(cur.parent)
            cur = self.entities.get(cur.parent)
        return out

    # ----- edits -----

    def add(self, entity: Entity, parent: str | None = None) -> None:
        if entity.id in self.entities:
            raise ValueError(f"duplicate entity id: {entity.id}")
        self.entities[entity.id] = entity
        if parent is None:
            entity.parent = None
            if entity.id not in self.roots:
                self.roots.append(entity.id)
        else:
            p = self.require(parent)
            entity.parent = parent
            if entity.id not in p.children:
                p.children.append(entity.id)

    def remove(self, entity_id: str) -> None:
        """Remove an entity and all its descendants."""
        e = self.entities.get(entity_id)
        if e is None:
            return
        for child_id in list(e.children):
            self.remove(child_id)
        if e.parent is not None:
            parent = self.entities.get(e.parent)
            if parent is not None and entity_id in parent.children:
                parent.children.remove(entity_id)
        if entity_id in self.roots:
            self.roots.remove(entity_id)
        del self.entities[entity_id]

    def reparent(self, entity_id: str, new_parent: str | None) -> None:
        e = self.require(entity_id)
        if e.parent is not None:
            old = self.entities.get(e.parent)
            if old is not None and entity_id in old.children:
                old.children.remove(entity_id)
        elif entity_id in self.roots:
            self.roots.remove(entity_id)

        if new_parent is None:
            e.parent = None
            if entity_id not in self.roots:
                self.roots.append(entity_id)
        else:
            np = self.require(new_parent)
            e.parent = new_parent
            if entity_id not in np.children:
                np.children.append(entity_id)

    def create(
        self,
        *,
        name: str = "",
        entity_type: str = "misc",
        parent: str | None = None,
        components: list[BaseComponent] | None = None,
    ) -> Entity:
        e = Entity(id=new_entity_id(entity_type), name=name)
        for c in components or []:
            e.attach(c)
        self.add(e, parent=parent)
        return e

    # ----- TOML round-trip -----

    def to_toml_dict(self) -> dict[str, Any]:
        return {"entities": {eid: e.to_toml_dict() for eid, e in self.entities.items()}}

    @classmethod
    def from_toml_dict(cls, data: dict[str, Any]) -> Scene:
        scene = cls()
        ent_table: dict[str, Any] = data.get("entities", {}) or {}
        for eid, raw in ent_table.items():
            comps_raw: dict[str, Any] = raw.get("components", {}) or {}
            components: dict[str, BaseComponent] = {}
            for key, payload in comps_raw.items():
                components[key] = parse_component(key, payload)
            entity = Entity(
                id=eid,
                name=raw.get("name", ""),
                parent=raw.get("parent"),
                children=list(raw.get("children", [])),
                components=components,
            )
            scene.entities[eid] = entity
        # Anything without a parent becomes a root.
        scene.roots = [eid for eid, e in scene.entities.items() if e.parent is None]
        return scene
