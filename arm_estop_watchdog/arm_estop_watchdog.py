#!/usr/bin/env python3
"""Arm-ping e-stop + device-health watchdog.

Stopgap until the robot exposes a real e-stop status feed. The Kinova arm
(192.168.2.50) is powered through the e-stop circuit, so:

    arm reachable  -> e-stop released
    arm unreachable -> e-stopped

Every PING_INTERVAL seconds this pings the arm + a few hosts (in parallel) and
drives the tower light by priority, most -> least important:

    solid RED       arm down (e-stopped)
    blinking ORANGE local comm down
    blinking YELLOW station comm down
    solid YELLOW    steamdeck down
    solid ORANGE    any local device down
    solid GREEN     everything up

The buzzer beeps once on every state change. On an arm down->up transition
(e-stop recovery) it POSTs /reload to BOTH the sensor API (so every driver
re-runs its connect path -- the documented fix for the arm not answering after a
power-cycle) AND the IK engine (so it re-discovers ports + re-anchors the
power-cycled flippers, instead of needing a reinstall). Reloads are tied to arm
recovery only.

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
from collections import namedtuple
from concurrent.futures import ThreadPoolExecutor

# ── Config (env overrides) ──────────────────────────────────────────────────
ARM_IP = os.environ.get("ARM_IP", "192.168.2.50")

# Single-host watchdogs, each with its own priority + color (see decide_state).
STEAMDECK_IP = os.environ.get("STEAMDECK_IP", "192.168.2.4")        # solid yellow
STATION_COMM_IP = os.environ.get("STATION_COMM_IP", "10.10.61.21")  # blinking yellow
LOCAL_COMM_IP = os.environ.get("LOCAL_COMM_IP", "10.10.61.22")      # blinking orange

# Local device health set -> solid ORANGE if any are down (lowest-priority fault).
_DEFAULT_DEVICES = (
    "192.168.2.3,192.168.2.12,"
    "192.168.2.40,192.168.2.41,"
    "192.168.2.31,192.168.2.33,192.168.2.34,192.168.2.35,"
    "10.10.61.22"
)
DEVICE_IPS = [ip.strip() for ip in os.environ.get("DEVICE_IPS", _DEFAULT_DEVICES).split(",") if ip.strip()]

TOWER_URL = os.environ.get("TOWER_URL", "http://192.168.2.3:3000").rstrip("/")
SENSOR_API_URL = os.environ.get("SENSOR_API_URL", "http://192.168.2.2:8080").rstrip("/")
# The IK engine self-heals (re-discovers ports + re-anchors flippers) on this
# POST instead of needing a reinstall after an e-stop. Runs on the Jetson with
# the watchdog by default. Set empty to disable the engine reload.
IK_ENGINE_URL = os.environ.get("IK_ENGINE_URL", "http://127.0.0.1:9101").rstrip("/")

PING_INTERVAL = float(os.environ.get("PING_INTERVAL", "5"))
PING_TIMEOUT = int(float(os.environ.get("PING_TIMEOUT", "1")))
HTTP_TIMEOUT = float(os.environ.get("HTTP_TIMEOUT", "3"))

# Tower colors. Solid color = POST /{color}/on; blinking = POST /{color}/blink.
# red/orange/green are physical channels; "yellow" is a virtual channel (lights
# red+orange+green together and sets the yellow flag). Yellow activates ONLY via
# its own /yellow/on or /yellow/blink route (hardware blink is unsupported for
# yellow, so we always use software /{color}/blink).
RED = "red"
YELLOW = "yellow"
ORANGE = "orange"
GREEN = "green"

BLINK_ON_MS = int(os.environ.get("BLINK_ON_MS", "400"))
BLINK_OFF_MS = int(os.environ.get("BLINK_OFF_MS", "400"))

# An LED state is a (color, blink) pair. A sweep's observations -> Obs.
LedState = namedtuple("LedState", "color blink")
Obs = namedtuple("Obs", "arm_up local_comm_up station_comm_up steamdeck_up device_down")

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


def set_led(color: str, blink: bool = False) -> bool:
    """Clear the tower, then show *color* (solid via /on, blinking via /blink).

    /clear first cancels any prior color/blink so exactly one color shows.
    Software blink is used for both physical channels and the virtual yellow
    (hardware blink isn't supported for yellow). Returns True if the color
    command succeeds.
    """
    _post(f"{TOWER_URL}/clear")  # best-effort; the command below is what matters
    if blink:
        return _post(f"{TOWER_URL}/{color}/blink", {"on_ms": BLINK_ON_MS, "off_ms": BLINK_OFF_MS})
    return _post(f"{TOWER_URL}/{color}/on")


def beep() -> bool:
    """Single short buzzer pulse to mark a state change."""
    return _post(f"{TOWER_URL}/buzzer/pulse", {"count": 1, "on_ms": 150, "off_ms": 0})


def reload_sensors() -> bool:
    """POST /reload to the sensor API on the Pi. Returns True on success."""
    if _post(f"{SENSOR_API_URL}/reload"):
        log.info("POST /reload -> sensor API accepted (process bounce scheduled)")
        return True
    return False


def reload_ik_engine() -> bool:
    """POST /api/v1/reload to the IK engine so it re-discovers ports + re-anchors
    the flippers after the e-stop power-cycle, instead of needing a reinstall.
    No-op (returns True) when IK_ENGINE_URL is blank."""
    if not IK_ENGINE_URL:
        return True
    if _post(f"{IK_ENGINE_URL}/api/v1/reload"):
        log.info("POST /api/v1/reload -> IK engine accepted (re-discover + re-anchor)")
        return True
    return False


def sweep(pool: ThreadPoolExecutor) -> Obs:
    """Ping the arm, the three single-host watchdogs, and the device set.

    Returns an Obs (all up/down flags + the list of down device IPs).
    """
    arm_f = pool.submit(ping, ARM_IP)
    local_f = pool.submit(ping, LOCAL_COMM_IP)
    station_f = pool.submit(ping, STATION_COMM_IP)
    steamdeck_f = pool.submit(ping, STEAMDECK_IP)
    device_futures = {ip: pool.submit(ping, ip) for ip in DEVICE_IPS}
    device_down = [ip for ip, fut in device_futures.items() if not fut.result()]
    return Obs(
        arm_up=arm_f.result(),
        local_comm_up=local_f.result(),
        station_comm_up=station_f.result(),
        steamdeck_up=steamdeck_f.result(),
        device_down=device_down,
    )


def decide_state(obs: Obs) -> LedState:
    """Priority ladder, most → least important.

    e-stop (arm down)          -> solid red
    local comm down            -> blinking orange
    station comm down          -> blinking yellow
    steamdeck down             -> solid yellow
    any local device down      -> solid orange
    else                       -> solid green
    """
    if not obs.arm_up:
        return LedState(RED, False)
    if not obs.local_comm_up:
        return LedState(ORANGE, True)
    if not obs.station_comm_up:
        return LedState(YELLOW, True)
    if not obs.steamdeck_up:
        return LedState(YELLOW, False)
    if obs.device_down:
        return LedState(ORANGE, False)
    return LedState(GREEN, False)


def _describe(obs: Obs) -> str:
    """Human-readable reason for the current state (for logging)."""
    if not obs.arm_up:
        return "arm unreachable / e-stopped"
    if not obs.local_comm_up:
        return f"local comm down ({LOCAL_COMM_IP})"
    if not obs.station_comm_up:
        return f"station comm down ({STATION_COMM_IP})"
    if not obs.steamdeck_up:
        return f"steamdeck down ({STEAMDECK_IP})"
    if obs.device_down:
        return f"device(s) down: {', '.join(obs.device_down)}"
    return "all up"


class Watchdog:
    """Holds transition state and performs one sweep->act cycle.

    `act()` is split from the ping I/O so the full state machine (the
    /reload-on-recovery edge, beep-on-change, and LED-retry-on-failure) can be
    driven directly in tests without touching the network.
    """

    def __init__(self) -> None:
        self.last_state: LedState | None = None
        self.last_arm_up: bool | None = None

    def tick(self, pool: ThreadPoolExecutor) -> None:
        self.act(sweep(pool))

    def act(self, obs: Obs) -> None:
        desired = decide_state(obs)

        # E-stop recovery: arm came back after being down. None->True
        # (first run) does NOT count.
        if obs.arm_up and self.last_arm_up is False:
            log.info("arm recovered (e-stop released) -- reloading sensors + IK engine")
            #reload_sensors()
            #reload_ik_engine()
        self.last_arm_up = obs.arm_up

        if desired != self.last_state:
            first = self.last_state is None
            log.info(
                "state -> %s%s -- %s",
                desired.color.upper(), " (blink)" if desired.blink else "", _describe(obs),
            )
            if set_led(desired.color, desired.blink):
                self.last_state = desired
                if not first:  # beep on real transitions, not the initial establish / restart
                    beep()
            else:
                self.last_state = None  # retry next tick


def main() -> int:
    logging.basicConfig(
        level=os.environ.get("LOG_LEVEL", "INFO").upper(),
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stdout,
    )
    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    log.info(
        "starting: arm=%s local_comm=%s station_comm=%s steamdeck=%s devices=%d "
        "tower=%s sensor_api=%s ik_engine=%s interval=%ss",
        ARM_IP, LOCAL_COMM_IP, STATION_COMM_IP, STEAMDECK_IP, len(DEVICE_IPS),
        TOWER_URL, SENSOR_API_URL, IK_ENGINE_URL or "(disabled)", PING_INTERVAL,
    )
    if "192.168.2.X" in SENSOR_API_URL:
        log.warning("SENSOR_API_URL still has placeholder IP (192.168.2.X) -- /reload will fail")

    wd = Watchdog()
    with ThreadPoolExecutor(max_workers=len(DEVICE_IPS) + 5) as pool:
        while _running:
            cycle_start = time.monotonic()
            try:
                wd.tick(pool)
            except Exception:  # noqa: BLE001 -- loop must never die
                log.exception("unexpected error in sweep loop")
                wd.last_state = None

            elapsed = time.monotonic() - cycle_start
            time.sleep(max(0.0, PING_INTERVAL - elapsed))

    log.info("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
