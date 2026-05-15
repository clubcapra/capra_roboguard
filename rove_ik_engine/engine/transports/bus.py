"""Publish-subscribe for outgoing StateUpdate frames.

The IK loop emits one StateUpdate per tick. Output transports register a
subscriber callback; each receives the serialised frame and is responsible
for delivering it (sending UDP, broadcasting WS, etc.). Failures in one
subscriber don't stop others.
"""

from __future__ import annotations

import logging
from typing import Awaitable, Callable

_log = logging.getLogger(__name__)

Subscriber = Callable[[bytes], Awaitable[None]]


class StateBus:
    def __init__(self) -> None:
        self._subs: list[Subscriber] = []

    def subscribe(self, sub: Subscriber) -> None:
        self._subs.append(sub)

    async def publish(self, frame: bytes) -> None:
        for sub in self._subs:
            try:
                await sub(frame)
            except Exception as e:  # noqa: BLE001
                _log.warning("output subscriber raised: %s", e)
