"""Bridge configuration: typed dataclasses + YAML loader.

Layout
------
- ``schema.py``   — dataclasses, one per YAML section
- ``loader.py``   — ``load(path) -> BridgeConfig``
- ``default.yaml``— shipped operator defaults
"""
from .loader import load
from .schema import (
    BridgeConfig,
    FlipperNodeIds,
    FlippersConfig,
    GripperConfig,
    ListenConfig,
    OvisConfig,
    SensorApiConfig,
    TracksConfig,
)

__all__ = [
    "BridgeConfig",
    "FlipperNodeIds",
    "FlippersConfig",
    "GripperConfig",
    "ListenConfig",
    "OvisConfig",
    "SensorApiConfig",
    "TracksConfig",
    "load",
]
