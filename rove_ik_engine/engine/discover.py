"""Resolve rove_sensor_api served ports at startup.

rove_sensor_api assigns served UDP ports (data = base+2i, cmd = base+2i+1) by
sensor REGISTRATION ORDER, which varies between boots — so a port hardcoded in
config can silently point at the wrong sensor after a restart. For READS that's
just missing data; for COMMANDS it's dangerous (you could drive the wrong
motor). So before opening any socket we query GET /discover and resolve each
sensor's ports by id.

Synchronous (stdlib urllib) one-shot at startup — blocking is fine there, and it
avoids pulling the async client into the boot path.
"""

from __future__ import annotations

import json
import logging
import urllib.request

_log = logging.getLogger(__name__)


def fetch_ports(host: str, http_port: int = 8080, timeout: float = 3.0) -> dict[str, tuple[int, int]]:
    """Return {sensor_id: (data_port, command_port)} from the robot's /discover.

    Empty dict on any failure — callers then keep their configured ports."""
    url = f"http://{host}:{http_port}/discover"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:  # noqa: S310 (trusted LAN host)
            data = json.loads(r.read().decode())
    except Exception as exc:  # noqa: BLE001
        _log.warning("port discovery failed at %s (%s) — keeping configured ports", url, exc)
        return {}
    out: dict[str, tuple[int, int]] = {}
    for s in data.get("sensors", []):
        sid = s.get("id")
        dp, cp = s.get("data_port"), s.get("command_port")
        if sid is not None and dp is not None:
            out[sid] = (int(dp), int(cp) if cp is not None else 0)
    _log.info("discovered %d sensor ports from %s", len(out), url)
    return out
