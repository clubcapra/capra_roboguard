"""API-seam adapter contract (S1, S2): the swappable wire seam.

The seam is the API, not an abstraction layer. A SensorApi adapter serializes
sim telemetry into a specific robot's wire format (e.g. rove_sensor_api); a
ControlBridge adapter accepts that robot's control/estop/mission messages and
hands decoded RoveControl intent to the actuators. A different robot with a
different proto = a new adapter registered here and named in the profile's
`api:` block -- no change to physics, sensors, or autonomy.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from ..registry import Registry

SENSOR_API_REGISTRY = Registry("sensor_api")
CONTROL_BRIDGE_REGISTRY = Registry("control_bridge")
register_sensor_api = SENSOR_API_REGISTRY.register
register_control_bridge = CONTROL_BRIDGE_REGISTRY.register


class SensorApi(ABC):
    """Robot-facing telemetry publisher (sim implements the real wire format)."""

    @abstractmethod
    def publish(self, telemetry: dict) -> None:
        """Serialize + emit one telemetry frame on the sensors plane."""


class ControlBridge(ABC):
    """Robot-facing command sink: decode wire control -> RoveControl intent."""

    @abstractmethod
    def poll(self):
        """Return the latest decoded RoveControl intent (or None)."""

    @abstractmethod
    def apply(self, actuators) -> None:
        """Dispatch the latest intent across the actuator list."""
