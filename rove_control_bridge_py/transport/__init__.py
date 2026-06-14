"""Outbound transport: how the bridge talks to other services.

- ``SensorApiUdpClient``        — JSON-over-UDP commands to rove_sensor_api
- ``discover_odrive_ports``     — resolve ODrive node_id → command port
- ``discover_sensor_command_port`` — resolve any named sensor's command port
- ``OvisForwarder``             — re-wrap + UDP arm twist to rove_ik_engine
"""
from .discovery import discover_odrive_ports, discover_sensor_command_port
from .ovis_forwarder import OvisForwarder
from .sensor_api_client import SensorApiUdpClient

__all__ = [
    "OvisForwarder",
    "SensorApiUdpClient",
    "discover_odrive_ports",
    "discover_sensor_command_port",
]
