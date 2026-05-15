"""Entity: an ID, a name, parent/child links, and a bag of components."""

from __future__ import annotations

import secrets
from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from .components import BaseComponent


_VALID_TYPES = {
    "link",
    "joint",
    "sensor",
    "actuator",
    "tool",
    "conveyor",
    "fixture",
    "zone",
    "controller",
    "group",
    "misc",
}


def new_entity_id(entity_type: str = "misc") -> str:
    """Generate an ID like `ent_link_a3f2b1c0`.

    `entity_type` is normalized to a known bucket — unknown types fall back
    to `misc` so the format stays predictable.
    """
    bucket = entity_type if entity_type in _VALID_TYPES else "misc"
    return f"ent_{bucket}_{secrets.token_hex(4)}"


class Entity(BaseModel):
    """One node in the scene graph."""

    model_config = ConfigDict(extra="forbid", validate_assignment=True)

    id: str
    name: str = ""
    parent: str | None = None
    children: list[str] = Field(default_factory=list)
    components: dict[str, BaseComponent] = Field(default_factory=dict)

    def has(self, key: str) -> bool:
        return key in self.components

    def get(self, key: str) -> BaseComponent | None:
        return self.components.get(key)

    def attach(self, component: BaseComponent) -> None:
        self.components[component.component_key] = component

    def detach(self, key: str) -> BaseComponent | None:
        return self.components.pop(key, None)

    def to_toml_dict(self) -> dict[str, Any]:
        """Produce a TOML-friendly dict (no Python-only types)."""
        out: dict[str, Any] = {"name": self.name}
        if self.parent is not None:
            out["parent"] = self.parent
        if self.children:
            out["children"] = list(self.children)
        if self.components:
            out["components"] = {
                key: comp.model_dump(exclude_none=True, exclude_defaults=False)
                for key, comp in self.components.items()
            }
        return out
