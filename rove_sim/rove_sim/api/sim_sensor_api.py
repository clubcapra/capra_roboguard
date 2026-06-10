"""SimSensorApi: the mock sim implementing rove_sensor_api's robot-facing side.

Each control tick (or sensor tick) it samples its per-driver devices from the
physics state and publishes one DATA frame per channel over a Transport, in the
real wire format. That is what makes the offline mock->real loop possible: a
real-mode RoveSensorApiStateSource subscribes to these exact frames -- the same
ones the real robot would emit -- so the synced twin is driven by telemetry, not
by reaching into the physics sim.
"""
from __future__ import annotations

from .base import SensorApi, register_sensor_api
from .devices import build_devices
from ..transport import build_transport


@register_sensor_api("rove_sensor_api")
class SimSensorApi(SensorApi):
    def __init__(self, robot, profile, transport=None, devices=None, actuators=None):
        self.robot = robot
        self.profile = profile
        self.devices = devices if devices is not None \
            else build_devices(profile, robot, actuators)
        self.transport = transport if transport is not None \
            else build_transport(profile.api.transport)

    def start(self) -> None:
        self.transport.start()

    def stop(self) -> None:
        self.transport.stop()

    def publish(self, telemetry: dict | None = None) -> None:
        """Sample every device and emit one DATA frame per channel."""
        for d in self.devices:
            self.transport.publish(d.channel, d.sample())
