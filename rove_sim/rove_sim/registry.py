"""Tiny decorator-based component registry.

Every pluggable library in rove_sim (sensors, actuators, API-seam adapters,
transports) is a registry: a component declares a string key with @register,
and a profile manifest names that key to instantiate it. Adding a new piece of
hardware support is therefore "write a class + register it", never editing a
switch statement.

    REGISTRY = Registry("sensor")

    @REGISTRY.register("livox_mid360")
    class LivoxMid360(Sensor): ...

    sensor = REGISTRY.build("livox_mid360", **kwargs)
"""
from __future__ import annotations

from typing import Callable, Dict, Type


class Registry:
    def __init__(self, kind: str):
        self.kind = kind
        self._items: Dict[str, type] = {}

    def register(self, key: str) -> Callable[[type], type]:
        def deco(cls: type) -> type:
            if key in self._items:
                raise KeyError(f"{self.kind} {key!r} already registered "
                               f"by {self._items[key].__name__}")
            self._items[key] = cls
            cls.registry_key = key
            return cls
        return deco

    def get(self, key: str) -> type:
        try:
            return self._items[key]
        except KeyError:
            raise KeyError(
                f"unknown {self.kind} {key!r}; "
                f"available: {sorted(self._items)}") from None

    def build(self, key: str, *args, **kwargs):
        return self.get(key)(*args, **kwargs)

    def keys(self):
        return sorted(self._items)

    def __contains__(self, key: str) -> bool:
        return key in self._items
