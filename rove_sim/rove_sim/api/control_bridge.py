"""RoveControlBridge: the robot-facing COMMAND sink (autonomy -> sim).

The sim publishes telemetry in the rove_sensor_api format AND accepts control in
the same wire convention, so the autonomy stack drives the sim exactly as it
drives the robot -- the only thing that changes in production is the transport
endpoint. A control frame is the JSON image of RoveControl on the `control`
channel; the bridge keeps the latest decoded intent for the loop to apply.
`ControlPublisher` is the autonomy-side helper to send one.
"""
from __future__ import annotations

from .base import ControlBridge, register_control_bridge
from ..transport import build_transport
from ..control import RoveControl, Tracks, Flippers, Ovis, Gripper

CONTROL_CHANNEL = "control"


def encode_control(rc: RoveControl) -> dict:
    return {"tracks": {"left": rc.tracks.left_vel, "right": rc.tracks.right_vel},
            "flippers": {"fl": rc.flippers.fl, "fr": rc.flippers.fr,
                         "rl": rc.flippers.rl, "rr": rc.flippers.rr},
            "ovis": {"vx": rc.ovis.vx, "vy": rc.ovis.vy, "vz": rc.ovis.vz,
                     "wx": rc.ovis.wx, "wy": rc.ovis.wy, "wz": rc.ovis.wz},
            "gripper": {"position": rc.gripper.position},
            "timestamp_us": rc.timestamp_us}


def decode_control(d: dict) -> RoveControl:
    t = d.get("tracks", {}); f = d.get("flippers", {})
    o = d.get("ovis", {}); g = d.get("gripper", {})
    return RoveControl(
        tracks=Tracks(float(t.get("left", 0.0)), float(t.get("right", 0.0))),
        flippers=Flippers(int(f.get("fl", 0)), int(f.get("fr", 0)),
                          int(f.get("rl", 0)), int(f.get("rr", 0))),
        ovis=Ovis(float(o.get("vx", 0.0)), float(o.get("vy", 0.0)),
                  float(o.get("vz", 0.0)), float(o.get("wx", 0.0)),
                  float(o.get("wy", 0.0)), float(o.get("wz", 0.0))),
        gripper=Gripper(int(g.get("position", 0))),
        timestamp_us=int(d.get("timestamp_us", 0)))


@register_control_bridge("rove_control_bridge")
class RoveControlBridge(ControlBridge):
    channel = CONTROL_CHANNEL

    def __init__(self, robot=None, profile=None, transport=None):
        self.transport = transport if transport is not None \
            else build_transport({"mode": "inprocess"})
        self._last = RoveControl()

    def start(self) -> None:
        self.transport.start()

    def stop(self) -> None:
        self.transport.stop()

    def poll(self) -> RoveControl:
        """Latest decoded intent (holds the last command if none new arrived)."""
        d = self.transport.latest(self.channel)
        if d:
            self._last = decode_control(d)
        return self._last

    def apply(self, actuators) -> None:
        for a in actuators:
            a.apply(self._last)


class ControlPublisher:
    """Autonomy-side: push one RoveControl frame to the sim over a pub transport."""

    def __init__(self, transport):
        self.transport = transport

    def start(self) -> None:
        self.transport.start()

    def stop(self) -> None:
        self.transport.stop()

    def send(self, rc: RoveControl) -> None:
        self.transport.publish(CONTROL_CHANNEL, encode_control(rc))
