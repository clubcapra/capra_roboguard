"""Signal port: I/O interface on an entity (input/output, protocol, data type)."""

from __future__ import annotations

from typing import Literal

from .base import BaseComponent, register_component

PortDirection = Literal["in", "out", "inout"]
PortDataType = Literal["bool", "int", "float", "vec3", "image", "string", "bytes", "custom"]


@register_component("signal_port")
class SignalPortComponent(BaseComponent):
    direction: PortDirection = "out"
    data_type: PortDataType = "float"
    protocol: str = "internal"  # "ethercat", "modbus", "ros2", "internal", ...
    name: str = ""              # local port name on the entity
    description: str = ""
