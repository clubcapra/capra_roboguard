#!/usr/bin/env python3
"""Arm-ping e-stop + device-health watchdog.

Stopgap until the robot exposes a real e-stop status feed. The Kinova arm
(192.168.2.50) is powered through the e-stop circuit, so:

    arm reachable  -> e-stop released
    arm unreachable -> e-stopped

Every PING_INTERVAL seconds this pings the arm plus a list of other networked
devices (in parallel) and drives the tower light by priority:

    RED     arm down (e-stopped)                       -- highest priority
    YELLOW  arm up, but a comms host (steamdeck / station) unreachable
    ORANGE  arm up, comms ok, but >=1 other device unreachable
    GREEN   arm up and every device reachable

On an arm down->up transition (e-stop recovery) it POSTs /reload to the sensor
API so every driver re-runs its connect path -- the documented fix for the arm
not answering after a power-cycle. /reload is tied to arm recovery only.

Runs on the Jetson. Tower API is local; the sensor API is on the Pi.
Stdlib only -- no third-party deps.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor

# ── Config (env overrides) ──────────────────────────────────────────────────
ARM_IP = os.environ.get("ARM_IP", "192.168.2.50")

# Health set -> drives ORANGE. .36 is currently down for testing but stays in
# the list (it's a real device, just offline right now).
_DEFAULT_DEVICES = (
    "192.168.2.3,192.168.2.12,"
    "192.168.2.40,192.168.2.41,"
    "192.168.2.31,192.168.2.33,192.168.2.34,192.168.2.35,"
    "10.10.61.22"
)
DEVICE_IPS = [ip.strip() for ip in os.environ.get("DEVICE_IPS", _DEFAULT_DEVICES).split(",") if ip.strip()]

# Comms hosts -> drive YELLOW (takes precedence over ORANGE). Losing the
# steamdeck or the station side of the link is more urgent than a single sensor
# being down, so it gets its own color.
_DEFAULT_COMMS = "192.168.2.4,10.10.62.21"  # steamdeck, station side
COMMS_IPS = [ip.strip() for ip in os.environ.get("COMMS_IPS", _DEFAULT_COMMS).split(",") if ip.strip()]

TOWER_URL = os.environ.get("TOWER_URL", "http://192.168.2.3:3000").rstrip("/")
SENSOR_API_URL = os.environ.get("SENSOR_API_URL", "http://192.168.2.2:8080").rstrip("/")

PING_INTERVAL = float(os.environ.get("PING_INTERVAL", "5"))
PING_TIMEOUT = int(float(os.environ.get("PING_TIMEOUT", "1")))
HTTP_TIMEOUT = float(os.environ.get("HTTP_TIMEOUT", "3"))

# Tower colors. We drive the tower with POST /{color}/on: red/orange/green are
# physical channels; "yellow" is a virtual channel that lights red+orange+green
# together and sets the firmware yellow flag. The virtual yellow ONLY activates
# via /yellow/on -- turning the three physical channels on individually (e.g.
# via /set) does not, so we must use the per-channel /on route.
RED = "red"
YELLOW = "yellow"
ORANGE = "orange"
GREEN = "green"
COLORS = (RED, YELLOW, ORANGE, GREEN)

log = logging.getLogger("arm_estop_watchdog")

_running = True


def _handle_signal(signum, _frame):
    global _running
    log.info("received signal %s, shutting down", signum)
    _running = False


def ping(ip: str) -> bool:
    """Return True if *ip* answers a single ICMP echo within PING_TIMEOUT."""
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", str(PING_TIMEOUT), ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=PING_TIMEOUT + 2,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, OSError) as exc:
        log.warning("ping %s raised: %s", ip, exc)
        return False


def _post(url: str, payload: dict | None = None) -> bool:
    """POST JSON (or empty body) to *url*. Returns True on HTTP success."""
    data = json.dumps(payload).encode() if payload is not None else b""
    headers = {"Content-Type": "application/json"} if payload is not None else {}
    req = urllib.request.Request(url, data=data, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT) as resp:
            resp.read()
        return True
    except (urllib.error.URLError, OSError) as exc:
        log.error("POST %s failed: %s", url, exc)
        return False


def set_led(color: str) -> bool:
    """Clear the tower, then turn on the channel for *color*.

    Uses POST /{color}/on (red/orange/green physical; yellow virtual = all three
    + the yellow flag). /clear first cancels any prior color/blink so exactly one
    color shows. Returns True only if the /{color}/on succeeds.
    """
    _post(f"{TOWER_URL}/clear")  # best-effort; the /on below is what matters
    return _post(f"{TOWER_URL}/{color}/on")


def reload_sensors() -> bool:
    """POST /reload to the sensor API on the Pi. Returns True on success."""
    if _post(f"{SENSOR_API_URL}/reload"):
        log.info("POST /reload -> sensor API accepted (process bounce scheduled)")
        return True
    return False


def sweep(pool: ThreadPoolExecutor) -> tuple[bool, list[str], list[str]]:
    """Ping arm + devices + comms hosts in parallel.

    Returns (arm_up, [down device ips], [down comms ips]).
    """
    arm_future = pool.submit(ping, ARM_IP)
    device_futures = {ip: pool.submit(ping, ip) for ip in DEVICE_IPS}
    comms_futures = {ip: pool.submit(ping, ip) for ip in COMMS_IPS}
    arm_up = arm_future.result()
    down = [ip for ip, fut in device_futures.items() if not fut.result()]
    comms_down = [ip for ip, fut in comms_futures.items() if not fut.result()]
    return arm_up, down, comms_down


def decide_color(arm_up: bool, down: list[str], comms_down: list[str] = ()) -> str:
    """Pure color-priority rule: RED > YELLOW > ORANGE > GREEN."""
    if not arm_up:
        return RED
    if comms_down:
        return YELLOW
    if down:
        return ORANGE
    return GREEN


class Watchdog:
    """Holds transition state and performs one sweep->act cycle.

    `act()` is split from the ping I/O so the full state machine (including the
    /reload-on-recovery edge and LED-retry-on-failure) can be driven directly in
    tests without touching the network.
    """

    def __init__(self) -> None:
        self.last_color: str | None = None
        self.last_arm_up: bool | None = None

    def tick(self, pool: ThreadPoolExecutor) -> None:
        arm_up, down, comms_down = sweep(pool)
        self.act(arm_up, down, comms_down)

    def act(self, arm_up: bool, down: list[str], comms_down: list[str] = ()) -> None:
        desired = decide_color(arm_up, down, comms_down)

        # E-stop recovery: arm came back after being down. None->True
        # (first run) does NOT count.
        if arm_up and self.last_arm_up is False:
            log.info("arm recovered (e-stop released) -- reloading sensors")
            reload_sensors()
        self.last_arm_up = arm_up

        if desired != self.last_color:
            detail = ""
            if desired == YELLOW:
                detail = f" (comms down: {', '.join(comms_down)})"
            elif desired == ORANGE:
                detail = f" (down: {', '.join(down)})"
            elif desired == RED:
                detail = " (arm unreachable / e-stopped)"
            log.info("state -> %s%s", desired.upper(), detail)
            if set_led(desired):
                self.last_color = desired
            else:
                self.last_color = None  # retry next tick


def main() -> int:
    logging.basicConfig(
        level=os.environ.get("LOG_LEVEL", "INFO").upper(),
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stdout,
    )
    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    log.info(
        "starting: arm=%s devices=%d comms=%d tower=%s sensor_api=%s interval=%ss",
        ARM_IP, len(DEVICE_IPS), len(COMMS_IPS), TOWER_URL, SENSOR_API_URL, PING_INTERVAL,
    )
    if "192.168.2.X" in SENSOR_API_URL:
        log.warning("SENSOR_API_URL still has placeholder IP (192.168.2.X) -- /reload will fail")

    wd = Watchdog()
    with ThreadPoolExecutor(max_workers=len(DEVICE_IPS) + len(COMMS_IPS) + 1) as pool:
        while _running:
            cycle_start = time.monotonic()
            try:
                wd.tick(pool)
            except Exception:  # noqa: BLE001 -- loop must never die
                log.exception("unexpected error in sweep loop")
                wd.last_color = None

            elapsed = time.monotonic() - cycle_start
            time.sleep(max(0.0, PING_INTERVAL - elapsed))

    log.info("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
