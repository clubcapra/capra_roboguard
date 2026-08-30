#!/usr/bin/env python3
"""Local tests for the watchdog -- no Jetson / robot network required.

Run:  python3 -m unittest -v test_arm_estop_watchdog

Layers:
  * TestPriority    -- decide_state() priority ladder (pure, no I/O).
  * TestStateMachine-- Watchdog.act() with fake set_led/reload/beep: reload edge,
                       beep-on-change, no-beep-on-first, LED retry-on-failure.
  * TestHttpRoundTrip-- real stand-in HTTP server; exercises the actual urllib
                       calls / routes for solid, blink, beep, reload.
  * TestSweep       -- fakes ping() to confirm parallel ping aggregation -> Obs.
"""

import json
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, HTTPServer

import arm_estop_watchdog as awd


def obs(arm=True, local_comm=True, station_comm=True, steamdeck=True, device_down=()):
    """Build an Obs with all-up defaults; override what you want down."""
    return awd.Obs(arm, local_comm, station_comm, steamdeck, list(device_down))


class _Recorder(BaseHTTPRequestHandler):
    calls: list[tuple[str, str]] = []

    def do_POST(self):  # noqa: N802
        length = int(self.headers.get("Content-Length", 0) or 0)
        body = self.rfile.read(length).decode() if length else ""
        _Recorder.calls.append((self.path, body))
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b"{}")

    def log_message(self, *args):  # silence stderr noise
        pass


class TestPriority(unittest.TestCase):
    """The 6-state priority ladder, most -> least important."""

    def test_estop_solid_red(self):
        self.assertEqual(awd.decide_state(obs(arm=False)), awd.LedState(awd.RED, False))

    def test_local_comm_blinking_orange(self):
        self.assertEqual(awd.decide_state(obs(local_comm=False)), awd.LedState(awd.ORANGE, True))

    def test_station_comm_blinking_yellow(self):
        self.assertEqual(awd.decide_state(obs(station_comm=False)), awd.LedState(awd.YELLOW, True))

    def test_steamdeck_solid_yellow(self):
        self.assertEqual(awd.decide_state(obs(steamdeck=False)), awd.LedState(awd.YELLOW, False))

    def test_device_solid_orange(self):
        self.assertEqual(awd.decide_state(obs(device_down=["192.168.2.31"])), awd.LedState(awd.ORANGE, False))

    def test_all_up_solid_green(self):
        self.assertEqual(awd.decide_state(obs()), awd.LedState(awd.GREEN, False))

    def test_priority_order(self):
        # everything down at once -> e-stop (red) wins
        everything_down = obs(arm=False, local_comm=False, station_comm=False,
                              steamdeck=False, device_down=["x"])
        self.assertEqual(awd.decide_state(everything_down), awd.LedState(awd.RED, False))
        # arm up, the rest down -> local comm wins (blinking orange)
        self.assertEqual(
            awd.decide_state(obs(local_comm=False, station_comm=False, steamdeck=False, device_down=["x"])),
            awd.LedState(awd.ORANGE, True),
        )
        # local comm restored -> station comm wins (blinking yellow)
        self.assertEqual(
            awd.decide_state(obs(station_comm=False, steamdeck=False, device_down=["x"])),
            awd.LedState(awd.YELLOW, True),
        )
        # station restored -> steamdeck wins (solid yellow) over device-down
        self.assertEqual(
            awd.decide_state(obs(steamdeck=False, device_down=["x"])),
            awd.LedState(awd.YELLOW, False),
        )


class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.led_calls: list[awd.LedState] = []
        self.beeps = 0
        self.reloads = 0
        self._led_ok = True
        self._orig = (awd.set_led, awd.reload_sensors, awd.beep)
        awd.set_led = lambda color, blink=False: (
            self.led_calls.append(awd.LedState(color, blink)), self._led_ok)[1]
        awd.reload_sensors = lambda: (setattr(self, "reloads", self.reloads + 1), True)[1]
        awd.beep = lambda: (setattr(self, "beeps", self.beeps + 1), True)[1]

    def tearDown(self):
        awd.set_led, awd.reload_sensors, awd.beep = self._orig

    def test_first_establish_sets_led_but_no_beep_no_reload(self):
        wd = awd.Watchdog()
        wd.act(obs())  # all up -> GREEN
        self.assertEqual(self.led_calls, [awd.LedState(awd.GREEN, False)])
        self.assertEqual(self.beeps, 0)    # first establish does not beep
        self.assertEqual(self.reloads, 0)  # first up != recovery

    def test_beep_on_each_transition(self):
        wd = awd.Watchdog()
        wd.act(obs())                       # GREEN  (first, no beep)
        wd.act(obs(steamdeck=False))        # solid YELLOW -> beep
        wd.act(obs(local_comm=False))       # blinking ORANGE -> beep
        self.assertEqual(self.beeps, 2)
        self.assertEqual(self.led_calls, [
            awd.LedState(awd.GREEN, False),
            awd.LedState(awd.YELLOW, False),
            awd.LedState(awd.ORANGE, True),
        ])

    def test_no_duplicate_led_or_beep_while_steady(self):
        wd = awd.Watchdog()
        wd.act(obs())
        wd.act(obs())
        wd.act(obs())
        self.assertEqual(self.led_calls, [awd.LedState(awd.GREEN, False)])
        self.assertEqual(self.beeps, 0)

    def test_reload_fires_once_on_recovery_only(self):
        wd = awd.Watchdog()
        wd.act(obs())                 # GREEN, no reload (first run)
        wd.act(obs(arm=False))        # RED (e-stop)
        wd.act(obs(device_down=["192.168.2.31"]))  # recovery -> reload, solid ORANGE
        wd.act(obs(device_down=["192.168.2.31"]))  # steady -> no reload, no new LED
        self.assertEqual(self.reloads, 1)

    def test_led_failure_triggers_retry_next_tick(self):
        wd = awd.Watchdog()
        self._led_ok = False
        wd.act(obs())   # set_led fails -> last_state stays None
        self._led_ok = True
        wd.act(obs())   # retries the same state
        self.assertEqual(self.led_calls, [awd.LedState(awd.GREEN, False), awd.LedState(awd.GREEN, False)])
        self.assertEqual(self.beeps, 0)  # retry after failure is not a "transition"


class TestHttpRoundTrip(unittest.TestCase):
    """Exercise the real urllib calls + routes against a stand-in server."""

    @classmethod
    def setUpClass(cls):
        _Recorder.calls = []
        cls.server = HTTPServer(("127.0.0.1", 0), _Recorder)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        host, port = cls.server.server_address
        base = f"http://{host}:{port}"
        cls._orig = (awd.TOWER_URL, awd.SENSOR_API_URL)
        awd.TOWER_URL = base
        awd.SENSOR_API_URL = base

    @classmethod
    def tearDownClass(cls):
        awd.TOWER_URL, awd.SENSOR_API_URL = cls._orig
        cls.server.shutdown()

    def setUp(self):
        _Recorder.calls.clear()

    def test_solid_color_clears_then_on(self):
        self.assertTrue(awd.set_led(awd.RED))
        self.assertEqual([p for p, _ in _Recorder.calls], ["/clear", "/red/on"])

    def test_blink_color_clears_then_blink_with_body(self):
        self.assertTrue(awd.set_led(awd.YELLOW, blink=True))
        paths = [p for p, _ in _Recorder.calls]
        self.assertEqual(paths, ["/clear", "/yellow/blink"])
        body = json.loads(_Recorder.calls[1][1])
        self.assertEqual(body, {"on_ms": awd.BLINK_ON_MS, "off_ms": awd.BLINK_OFF_MS})

    def test_beep_pulses_buzzer(self):
        self.assertTrue(awd.beep())
        self.assertEqual(_Recorder.calls[0][0], "/buzzer/pulse")
        self.assertEqual(json.loads(_Recorder.calls[0][1]), {"count": 1, "on_ms": 150, "off_ms": 0})

    def test_reload_hits_endpoint(self):
        self.assertTrue(awd.reload_sensors())
        self.assertEqual(_Recorder.calls[0][0], "/reload")


class TestSweep(unittest.TestCase):
    def test_parallel_ping_aggregation(self):
        orig = (awd.ping, awd.ARM_IP, awd.LOCAL_COMM_IP, awd.STATION_COMM_IP,
                awd.STEAMDECK_IP, awd.DEVICE_IPS)
        # arm + station + steamdeck up; local_comm down; one device down
        reachable = {"10.0.0.50", "10.0.0.21", "10.0.0.4", "10.0.0.1"}
        awd.ping = lambda ip: ip in reachable
        awd.ARM_IP = "10.0.0.50"
        awd.LOCAL_COMM_IP = "10.0.0.22"   # down
        awd.STATION_COMM_IP = "10.0.0.21"
        awd.STEAMDECK_IP = "10.0.0.4"
        awd.DEVICE_IPS = ["10.0.0.1", "10.0.0.2"]  # .2 down
        try:
            with ThreadPoolExecutor(max_workers=8) as pool:
                o = awd.sweep(pool)
            self.assertTrue(o.arm_up)
            self.assertFalse(o.local_comm_up)
            self.assertTrue(o.station_comm_up)
            self.assertTrue(o.steamdeck_up)
            self.assertEqual(o.device_down, ["10.0.0.2"])
            # local comm down -> blinking orange
            self.assertEqual(awd.decide_state(o), awd.LedState(awd.ORANGE, True))
        finally:
            (awd.ping, awd.ARM_IP, awd.LOCAL_COMM_IP, awd.STATION_COMM_IP,
             awd.STEAMDECK_IP, awd.DEVICE_IPS) = orig


if __name__ == "__main__":
    unittest.main(verbosity=2)
