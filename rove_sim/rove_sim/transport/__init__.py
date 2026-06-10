"""Transport seam for the telemetry plane: publish/latest over a named channel.

Two implementations, same wire codec (transport/packet.py):
  * InProcessTransport -- a shared in-memory bus keyed by name; single-process
    dev + tests. Still round-trips through encode/decode so the codec is on the
    hot path everywhere.
  * UdpTransport -- the real wire: DATA frames over UDP per channel:port. The
    mock SimSensorApi publishes, a real-mode RobotStateSource subscribes -- the
    same path the real robot uses.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, Optional

from .packet import MSG_DATA, decode, encode


class Transport(ABC):
    @abstractmethod
    def publish(self, channel: str, payload: dict) -> None: ...

    @abstractmethod
    def latest(self, channel: str) -> Optional[dict]: ...

    def start(self) -> None: ...
    def stop(self) -> None: ...


class InProcessTransport(Transport):
    """Process-wide named buses. A publisher and a subscriber that name the same
    `bus` are connected; the payload round-trips through the packet codec."""
    _buses: Dict[str, Dict[str, bytes]] = {}

    def __init__(self, bus: str = "default", **_):
        self.bus = InProcessTransport._buses.setdefault(bus, {})
        self._seq = 0

    def publish(self, channel: str, payload: dict) -> None:
        self._seq += 1
        self.bus[channel] = encode(MSG_DATA, self._seq, payload)

    def latest(self, channel: str) -> Optional[dict]:
        raw = self.bus.get(channel)
        return None if raw is None else decode(raw)[2]


def build_transport(spec: Optional[dict]) -> Transport:
    spec = dict(spec or {})
    mode = spec.get("mode", "inprocess")
    if mode in ("inprocess", "inproc"):
        return InProcessTransport(bus=spec.get("bus", "default"))
    if mode in ("udp", "network"):
        from .udp import UdpTransport
        return UdpTransport(**{k: v for k, v in spec.items() if k != "mode"})
    raise ValueError(f"unknown transport mode {mode!r}")
