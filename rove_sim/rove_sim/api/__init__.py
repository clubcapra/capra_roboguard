"""API-seam package. Importing it registers the built-in wire adapters."""
from .base import (SensorApi, ControlBridge, SENSOR_API_REGISTRY,
                   CONTROL_BRIDGE_REGISTRY)
from . import sim_sensor_api          # noqa: F401  (register on import)
from . import control_bridge          # noqa: F401  (register on import)

__all__ = ["SensorApi", "ControlBridge", "SENSOR_API_REGISTRY",
           "CONTROL_BRIDGE_REGISTRY"]
