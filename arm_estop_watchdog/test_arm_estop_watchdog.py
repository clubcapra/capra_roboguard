#!/usr/bin/env python3
"""Local tests for the watchdog -- no Jetson / robot network required.

Run:  python3 -m unittest -v test_arm_estop_watchdog

Two layers:
  * TestStateMachine -- drives Watchdog.act() with scripted (arm_up, down)
    values and a fake set_led/reload to assert color priority, the
    /reload-only-on-recovery edge, and LED-retry-on-failure.
  * TestHttpRoundTrip -- runs a real stand-in HTTP server and points the
    watchdog's TOWER_URL / SENSOR_API_URL at it, so the actual urllib calls and
    JSON payloads are exercised end-to-end.
  * TestSweep -- fakes ping() to confirm parallel ping aggregation.
"""

import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, HTTPServer

import arm_estop_watchdog as awd


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


class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.led_calls: list[str] = []
        self.reloads = 0
        self._led_ok = True
        # Patch side-effects to in-memory recorders.
        self._orig_led = awd.set_led
        self._orig_reload = awd.reload_sensors
        awd.set_led = lambda c: (self.led_calls.append(c), self._led_ok)[1]
        awd.reload_sensors = lambda: (setattr(self, "reloads", self.reloads + 1), True)[1]

    def tearDown(self):
        awd.set_led = self._orig_led
        awd.reload_sensors = self._orig_reload

    def test_priority_and_first_run_no_reload(self):
        wd = awd.Watchdog()
        wd.act(arm_up=True, down=[])           # all good
        self.assertEqual(self.led_calls, [awd.GREEN])
        self.assertEqual(self.reloads, 0)      # first-run up != recovery

    def test_orange_when_device_down(self):
        wd = awd.Watchdog()
        wd.act(arm_up=True, down=["192.168.2.36"])
        self.assertEqual(self.led_calls, [awd.ORANGE])

    def test_red_takes_priority_over_devices(self):
        wd = awd.Watchdog()
        wd.act(arm_up=False, down=["192.168.2.36"])  # arm down wins
        self.assertEqual(self.led_calls, [awd.RED])

    def test_yellow_when_comms_down(self):
        wd = awd.Watchdog()
        wd.act(arm_up=True, down=[], comms_down=["192.168.2.4"])
        self.assertEqual(self.led_calls, [awd.YELLOW])

    def test_yellow_takes_priority_over_orange(self):
        wd = awd.Watchdog()
        wd.act(arm_up=True, down=["192.168.2.33"], comms_down=["10.10.62.21"])
        self.assertEqual(self.led_calls, [awd.YELLOW])

    def test_red_takes_priority_over_comms(self):
        wd = awd.Watchdog()
        wd.act(arm_up=False, down=[], comms_down=["192.168.2.4"])  # e-stop still wins
        self.assertEqual(self.led_calls, [awd.RED])

    def test_reload_fires_once_on_recovery_only(self):
        wd = awd.Watchdog()
        wd.act(arm_up=True, down=[])     # GREEN, no reload (first run)
        wd.act(arm_up=False, down=[])    # RED (e-stop)
        wd.act(arm_up=True, down=["192.168.2.36"])  # recovery -> reload, ORANGE
        wd.act(arm_up=True, down=["192.168.2.36"])  # steady -> no reload, no new LED
        self.assertEqual(self.reloads, 1)
        self.assertEqual(self.led_calls, [awd.GREEN, awd.RED, awd.ORANGE])

    def test_no_duplicate_led_while_steady(self):
        wd = awd.Watchdog()
        wd.act(arm_up=True, down=[])
        wd.act(arm_up=True, down=[])
        wd.act(arm_up=True, down=[])
        self.assertEqual(self.led_calls, [awd.GREEN])  # only the transition

    def test_led_failure_triggers_retry_next_tick(self):
        wd = awd.Watchdog()
        self._led_ok = False
        wd.act(arm_up=True, down=[])   # set_led fails -> last_color stays None
        self._led_ok = True
        wd.act(arm_up=True, down=[])   # retries the same color
        self.assertEqual(self.led_calls, [awd.GREEN, awd.GREEN])


class TestHttpRoundTrip(unittest.TestCase):
    """Exercise the real urllib calls + JSON payloads against a stand-in server."""

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

    def test_set_led_clears_then_turns_on(self):
        self.assertTrue(awd.set_led(awd.RED))
        # each color change is /clear then /{color}/on
        self.assertEqual([p for p, _ in _Recorder.calls], ["/clear", "/red/on"])

    def test_set_led_paths(self):
        for c in (awd.RED, awd.YELLOW, awd.ORANGE, awd.GREEN):
            self.assertTrue(awd.set_led(c))
        on_calls = [p for p, _ in _Recorder.calls if p.endswith("/on")]
        # yellow is the virtual channel (lights all three + sets yellow flag)
        self.assertEqual(on_calls, ["/red/on", "/yellow/on", "/orange/on", "/green/on"])

    def test_reload_hits_endpoint(self):
        self.assertTrue(awd.reload_sensors())
        self.assertEqual(_Recorder.calls[0][0], "/reload")

    def test_full_recovery_cycle_over_http(self):
        wd = awd.Watchdog()
        wd.act(arm_up=True, down=[])                  # GREEN
        wd.act(arm_up=False, down=[])                 # RED
        wd.act(arm_up=True, down=["192.168.2.36"])    # /reload + ORANGE
        paths = [p for p, _ in _Recorder.calls]
        self.assertEqual(paths.count("/reload"), 1)
        on_calls = [p for p in paths if p.endswith("/on")]
        self.assertEqual(on_calls, ["/green/on", "/red/on", "/orange/on"])
        # /reload must precede the ORANGE /on that follows it
        self.assertLess(paths.index("/reload"), len(paths) - 1)


class TestSweep(unittest.TestCase):
    def test_parallel_ping_aggregation(self):
        orig = (awd.ping, awd.ARM_IP, awd.DEVICE_IPS, awd.COMMS_IPS)
        reachable = {"10.0.0.50", "10.0.0.1", "10.0.0.9"}  # arm + one device + one comms up
        awd.ping = lambda ip: ip in reachable
        awd.ARM_IP = "10.0.0.50"
        awd.DEVICE_IPS = ["10.0.0.1", "10.0.0.2"]  # .2 down
        awd.COMMS_IPS = ["10.0.0.9", "10.0.0.8"]   # .8 down
        try:
            with ThreadPoolExecutor(max_workers=6) as pool:
                arm_up, down, comms_down = awd.sweep(pool)
            self.assertTrue(arm_up)
            self.assertEqual(down, ["10.0.0.2"])
            self.assertEqual(comms_down, ["10.0.0.8"])
        finally:
            awd.ping, awd.ARM_IP, awd.DEVICE_IPS, awd.COMMS_IPS = orig


if __name__ == "__main__":
    unittest.main(verbosity=2)
