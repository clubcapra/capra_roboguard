"""UDP client for the rove_sensor_api sensor protocol.

Wire format (4-byte header, little-endian):
    byte 0  — protocol version (always 0x01)
    byte 1  — message type
    byte 2-3 — sequence number (u16 LE)
    byte 4.. — JSON payload

Message types used here:
    0x10 Command    — client → sensor (JSON payload = command dict)
    0x11 CommandAck — sensor → client (JSON result, ignored in stream mode)
"""
from __future__ import annotations

import json
import logging
import socket
import struct
import time
from typing import Any

log = logging.getLogger(__name__)

_PROTOCOL_VERSION: int = 0x01
_MSG_COMMAND: int = 0x10
_HEADER_FMT: str = "<BBH"   # version, msg_type, seq_num
_HEADER_SIZE: int = struct.calcsize(_HEADER_FMT)


class SensorApiUdpClient:
    """Non-blocking UDP client for sending commands to rove_sensor_api sensors.

    A single socket is reused for all send calls.  Each call is fire-and-forget
    (no ACK waiting) which is correct for stream-mode ODrive nodes: we want to
    keep the control loop running even if a packet gets lost.
    """

    def __init__(self, host: str) -> None:
        self._host = host
        self._seq: int = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def send_command(self, cmd_port: int, payload: dict[str, Any]) -> bool:
        """Encode and send a Command packet to *cmd_port*.

        Returns True on success; False if the OS buffer was full (frame dropped)
        or any other transient error occurred.
        """
        self._seq = (self._seq + 1) & 0xFFFF
        header = struct.pack(_HEADER_FMT, _PROTOCOL_VERSION, _MSG_COMMAND, self._seq)
        try:
            body = json.dumps(payload).encode()
        except (TypeError, ValueError) as exc:
            log.error("Failed to serialize command payload: %s", exc)
            return False

        packet = header + body
        try:
            self._sock.sendto(packet, (self._host, cmd_port))
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


def discover_sensor_command_port(
    host: str, http_port: int, sensor_id: str, timeout_s: float = 10.0
) -> int | None:
    """Return the command port for a named sensor, or None if not found.

    Makes a single /discover call (no retry — caller may want to proceed
    without the sensor if it's optional).
    """
    import urllib.error
    import urllib.request

    url = f"http://{host}:{http_port}/discover"
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as resp:
            sensors = json.loads(resp.read())
    except Exception as exc:
        log.warning("discover_sensor_command_port: /discover failed: %s", exc)
        return None

    sensor_list = sensors.get("sensors", sensors) if isinstance(sensors, dict) else sensors
    for sensor in sensor_list:
        if sensor.get("id") == sensor_id:
            port = sensor.get("command_port")
            if port is not None:
                log.info("Discovered %s → cmd_port %d", sensor_id, int(port))
                return int(port)
    log.warning("Sensor %r not found in /discover response", sensor_id)
    return None


def discover_odrive_ports(
    host: str, http_port: int, timeout_s: float = 10.0
) -> dict[int, int]:
    """Query ``GET /discover`` and return a ``{node_id: cmd_port}`` mapping.

    Retries until *timeout_s* seconds have elapsed.  Raises ``RuntimeError``
    if the sensor API never responds or no ODrive sensors are found.
    """
    import urllib.error
    import urllib.request

    url = f"http://{host}:{http_port}/discover"
    deadline = time.monotonic() + timeout_s
    delay = 1.0

    while True:
        try:
            with urllib.request.urlopen(url, timeout=3.0) as resp:
                sensors: list[dict] = json.loads(resp.read())
            break
        except Exception as exc:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    f"Cannot reach rove_sensor_api at {url} after {timeout_s}s: {exc}"
                ) from exc
            log.info("Waiting for sensor_api (%s) … retrying in %.0fs", exc, delay)
            time.sleep(min(delay, remaining))
            delay = min(delay * 2, 5.0)

    # Response shape: {"sensors": [{id, command_port, endpoints, …}, …]}
    sensor_list: list[dict] = sensors.get("sensors", sensors) if isinstance(sensors, dict) else sensors

    port_map: dict[int, int] = {}
    for sensor in sensor_list:
        sid: str = sensor.get("id", "")
        if not sid.startswith("odrive_"):
            continue
        try:
            node_id = int(sid.removeprefix("odrive_"))
        except ValueError:
            continue
        cmd_port = sensor.get("command_port")
        if cmd_port is not None:
            port_map[node_id] = int(cmd_port)
            log.info("Discovered odrive_%d → cmd_port %d", node_id, cmd_port)

    if not port_map:
        raise RuntimeError(
            f"No ODrive sensors found in /discover response from {url}. "
            "Ensure rove_sensor_api is running and ODrive nodes are on the CAN bus."
        )

    return port_map
