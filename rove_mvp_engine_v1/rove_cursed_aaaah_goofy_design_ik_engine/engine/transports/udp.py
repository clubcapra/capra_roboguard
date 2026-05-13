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

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        try:
            msg = Ovis()
            msg.ParseFromString(data)
        except Exception as e:  # noqa: BLE001
            _log.debug("dropped malformed Ovis from %s: %s", addr, e)
            return
        self.state.set_ovis(msg)


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


class UdpOutput:
    """Per-tick `sendto` to a configured target. Re-resolves on each send so
    DHCP / DNS churn doesn't wedge the engine."""

    def __init__(self, bus: StateBus, target: str) -> None:
        self.bus = bus
        self.target = target
        self._transport: asyncio.DatagramTransport | None = None
        self._addr: tuple[str, int] | None = None

    async def start(self) -> None:
        host, port = parse_bind(self.target)
        self._addr = (host, port)
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            asyncio.DatagramProtocol,
            remote_addr=(host, port),
        )
        self.bus.subscribe(self._send)
        _log.info("UDP output streaming to %s:%d", host, port)

    async def _send(self, frame: bytes) -> None:
        if self._transport is None or self._addr is None:
            return
        # send (datagram transports buffer internally; no await needed)
        self._transport.sendto(frame)

    async def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()
