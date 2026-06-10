"""RoveSensorApiStateSource: real-mode robot state from rove_sensor_api telemetry.

Subscribes (in-process to the mock SimSensorApi, or over UDP to the real robot)
and decodes each driver's latest DATA frame into a RobotState via the SAME device
codecs the publisher uses (api/devices.py), so encode and decode can never drift.
The SyncDriver then slams the URDF to that state kinematically.
"""
from __future__ import annotations

from .base import RobotState, RobotStateSource, register
from ..api.devices import build_devices
from ..transport import build_transport


@register("rove_sensor_api")
class RoveSensorApiStateSource(RobotStateSource):
    def __init__(self, profile=None, robot=None, transport=None,
                 devices=None, **kwargs):
        if robot is None:
            raise ValueError("RoveSensorApiStateSource needs the loaded robot "
                             "(joint maps); runtime.build passes it).")
        self.profile = profile
        self.robot = robot
        self.devices = devices if devices is not None else build_devices(profile, robot)
        if transport is not None:
            self.transport = transport
        else:
            spec = dict((profile.api.transport if profile else {}) or {})
            spec.setdefault("role", "sub")        # this side subscribes
            self.transport = build_transport(spec)

    def start(self) -> None:
        self.transport.start()

    def stop(self) -> None:
        self.transport.stop()

    def read(self) -> RobotState:
        state = RobotState()
        for d in self.devices:
            payload = self.transport.latest(d.channel)
            if payload:
                d.apply(payload, state)
        return state
