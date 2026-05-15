"""UDP input and output transports.

Wire format: a single protobuf-serialised message per datagram. No
framing or sequence numbers — Ovis is a velocity, so dropped packets
self-correct on the next one.
"""

from __future__ import annotations

import asyncio
import logging

from ..config import parse_bind
from ..proto import Ovis
from ..state import EngineState
from .bus import StateBus

_log = logging.getLogger(__name__)


class _OvisInProtocol(asyncio.DatagramProtocol):
    def __init__(self, state: EngineState) -> None:
        self.state = state
        self._last_decode_warn_t = 0.0
        self._last_err_warn_t = 0.0

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        try:
            msg = Ovis()
            msg.ParseFromString(data)
        except Exception as e:  # noqa: BLE001
            # Rate-limited so a spammy bad sender can't fill journald.
            import time as _t
            now = _t.monotonic()
            if now - self._last_decode_warn_t > 30.0:
                _log.warning("dropped malformed Ovis from %s: %s", addr, e)
                self._last_decode_warn_t = now
            return
        self.state.set_ovis(msg)

    def error_received(self, exc: Exception) -> None:
        # UDP input is unconnected so ICMP is unusual here, but log it
        # anyway. Rate-limited.
        import time as _t
        now = _t.monotonic()
        if now - self._last_err_warn_t > 30.0:
            _log.warning("Ovis UDP input error: %s", exc)
            self._last_err_warn_t = now


class UdpInput:
    def __init__(self, state: EngineState, bind: str) -> None:
        self.state = state
        self.bind = bind
        self._transport: asyncio.DatagramTransport | None = None

    async def start(self) -> None:
        host, port = parse_bind(self.bind)
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: _OvisInProtocol(self.state),
            local_addr=(host, port),
        )
        _log.info("UDP input listening on %s:%d", host, port)

    async def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()


class _OvisOutProtocol(asyncio.DatagramProtocol):
    """Surfaces ICMP / connection_lost so UdpOutput can log the breakage
    instead of silently sending into the void after the consumer dies."""

    def __init__(self) -> None:
        self._last_err_warn_t = 0.0

    def error_received(self, exc: Exception) -> None:
        import time as _t
        now = _t.monotonic()
        if now - self._last_err_warn_t > 30.0:
            _log.warning("UDP output transport error: %s", exc)
            self._last_err_warn_t = now

    def connection_lost(self, exc: Exception | None) -> None:
        if exc is not None:
            _log.warning("UDP output transport closed: %s", exc)


class UdpOutput:
    """Per-tick `sendto` to a configured target. Re-resolves on each send so
    DHCP / DNS churn doesn't wedge the engine."""

    def __init__(self, bus: StateBus, target: str) -> None:
        self.bus = bus
        self.target = target
        self._transport: asyncio.DatagramTransport | None = None
        self._addr: tuple[str, int] | None = None
        self._last_send_warn_t = 0.0

    async def start(self) -> None:
        host, port = parse_bind(self.target)
        self._addr = (host, port)
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            _OvisOutProtocol,
            remote_addr=(host, port),
        )
        self.bus.subscribe(self._send)
        _log.info("UDP output streaming to %s:%d", host, port)

    async def _send(self, frame: bytes) -> None:
        if self._transport is None or self._addr is None:
            return
        try:
            self._transport.sendto(frame)
        except Exception as exc:  # noqa: BLE001
            import time as _t
            now = _t.monotonic()
            if now - self._last_send_warn_t > 30.0:
                _log.warning("UDP output send failed: %s", exc)
                self._last_send_warn_t = now

    async def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()
