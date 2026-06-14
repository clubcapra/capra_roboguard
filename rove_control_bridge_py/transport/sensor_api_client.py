"""UDP client for rove_sensor_api's command protocol.

Wire format (4-byte header, little-endian):
    byte 0   — protocol version (always 0x01)
    byte 1   — message type
    byte 2-3 — sequence number (u16 LE)
    byte 4.. — JSON payload

Message types:
    0x10 Command    — client → sensor (JSON command dict)
    0x11 CommandAck — sensor → client (ignored in stream mode)
"""
from __future__ import annotations

import json
import logging
import socket
import struct
from typing import Any

log = logging.getLogger(__name__)

_PROTOCOL_VERSION = 0x01
_MSG_COMMAND = 0x10
_HEADER_FMT = "<BBH"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)


class SensorApiUdpClient:
    """Non-blocking UDP command sender to rove_sensor_api sensor ports.

    Fire-and-forget: dropped frames are logged but don't stall the control
    loop, since the next tick will overwrite the lost setpoint anyway.
    """

    def __init__(self, host: str) -> None:
        self._host = host
        self._seq = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def send_command(self, cmd_port: int, payload: dict[str, Any]) -> bool:
        """Send a JSON command to the sensor listening on cmd_port.

        Returns True on success, False if the OS buffer was full or the
        payload couldn't be serialized.
        """
        self._seq = (self._seq + 1) & 0xFFFF
        header = struct.pack(_HEADER_FMT, _PROTOCOL_VERSION, _MSG_COMMAND, self._seq)
        try:
            body = json.dumps(payload).encode()
        except (TypeError, ValueError) as exc:
            log.error("Failed to serialize command payload: %s", exc)
            return False

        try:
            self._sock.sendto(header + body, (self._host, cmd_port))
            return True
        except BlockingIOError:
            log.debug("UDP send would block, dropping command frame")
            return False
        except OSError as exc:
            log.warning("UDP command send failed: %s", exc)
            return False

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> "SensorApiUdpClient":
        return self

    def __exit__(self, *_) -> None:
        self.close()
