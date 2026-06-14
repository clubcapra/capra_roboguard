"""Resolve sensor command ports via rove_sensor_api's /discover endpoint.

The bridge needs to know which UDP port to send each ODrive / gripper
command to. rove_sensor_api owns those ports and advertises them via
HTTP /discover; the helpers here turn that response into the per-sensor
mappings the bridge dispatches against.
"""
from __future__ import annotations

import json
import logging
import time
import urllib.request

log = logging.getLogger(__name__)


def _fetch_sensor_list(url: str, timeout_s: float) -> list[dict]:
    with urllib.request.urlopen(url, timeout=timeout_s) as resp:
        data = json.loads(resp.read())
    if isinstance(data, dict):
        return data.get("sensors", [])
    return data


def discover_sensor_command_port(
    host: str, http_port: int, sensor_id: str, timeout_s: float = 10.0
) -> int | None:
    """Return the command port for a named sensor, or None if not found.

    Single-shot — no retry. Caller decides whether the sensor is required.
    """
    url = f"http://{host}:{http_port}/discover"
    try:
        sensors = _fetch_sensor_list(url, timeout_s)
    except Exception as exc:
        log.warning("discover_sensor_command_port: /discover failed: %s", exc)
        return None

    for sensor in sensors:
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
    """Return ``{node_id: cmd_port}`` for every ODrive sensor on the bus.

    Retries with exponential backoff until *timeout_s* has elapsed.
    Raises RuntimeError if rove_sensor_api never responds or no ODrive
    sensors are advertised — both are unrecoverable for the bridge.
    """
    url = f"http://{host}:{http_port}/discover"
    deadline = time.monotonic() + timeout_s
    delay = 1.0

    while True:
        try:
            sensors = _fetch_sensor_list(url, timeout_s=3.0)
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

    port_map: dict[int, int] = {}
    for sensor in sensors:
        sid = sensor.get("id", "")
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
