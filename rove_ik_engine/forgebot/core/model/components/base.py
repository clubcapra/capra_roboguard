"""Base class and registry for all components.

Components are Pydantic models attached to entities. The registry maps a
TOML key (e.g. "joint") to its component class. New components are added
by creating a subclass of `BaseComponent` and decorating it with
`@register_component("my_key")`.
"""

from __future__ import annotations

from typing import Any, ClassVar

from pydantic import BaseModel, ConfigDict


class BaseComponent(BaseModel):
    """All components extend this.

    The `_source` field is reserved for round-trip metadata: importers
    write the original element name/attributes here so re-exporting can
    reproduce them.
    """

    model_config = ConfigDict(extra="forbid", validate_assignment=True)

    # Reserved field; used by importers/exporters for fidelity, not by core.
    source: dict[str, Any] | None = None

    # Subclasses override this via @register_component(...).
    component_key: ClassVar[str] = ""


_REGISTRY: dict[str, type[BaseComponent]] = {}


def register_component(key: str):
    """Decorator: register a component class under a TOML key."""

    def wrap(cls: type[BaseComponent]) -> type[BaseComponent]:
        if key in _REGISTRY and _REGISTRY[key] is not cls:
            raise ValueError(f"component key '{key}' already registered to {_REGISTRY[key]}")
        cls.component_key = key
        _REGISTRY[key] = cls
        return cls

    return wrap


def get_component_class(key: str) -> type[BaseComponent] | None:
    return _REGISTRY.get(key)


def all_component_keys() -> list[str]:
    return sorted(_REGISTRY.keys())


def parse_component(key: str, data: dict[str, Any]) -> BaseComponent:
    """Construct a component from its TOML key and a raw dict.

    Unknown keys raise `KeyError`; pydantic raises `ValidationError` on bad data.
    """
    cls = _REGISTRY.get(key)
    if cls is None:
        raise KeyError(f"unknown component key: {key!r}")
    return cls.model_validate(data)
