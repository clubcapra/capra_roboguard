#!/usr/bin/env python3
"""Exercise HALo with the real headless simulator; optional offline browser checks.
Run using the simulator's Python environment after `cargo build`.
"""
import argparse
import http.cookiejar
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

HALO = Path(__file__).resolve().parents[1]
SIM = HALO.parent / "rove_sim"
PASSWORD = "HALo-test-password-2026"


def wait_for(check, timeout=20):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = check()
            if last:
                return last
        except (urllib.error.URLError, ConnectionError, KeyError):
            pass
        time.sleep(.1)
    raise AssertionError(f"condition timed out; last result: {last}")


class Client:
    def __init__(self, port):
        self.url = f"http://127.0.0.1:{port}"
        self.cookies = http.cookiejar.CookieJar()
        self.open = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(self.cookies))

    def request(self, path, method="GET", body=None, header=True, expected=200):
        headers = {"X-HALo-Request": "1"} if header else {}
        data = None
        if body is not None:
            data = json.dumps(body).encode()
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(self.url + path, data=data, headers=headers, method=method)
        try:
            response = self.open.open(req, timeout=5)
        except urllib.error.HTTPError as error:
            response = error
        raw = response.read()
        assert response.code == expected, (path, response.code, expected, raw.decode())
        return json.loads(raw) if raw else None

    def manifest(self):
        return {s["id"]: s for s in self.request("/discover")["sensors"]}

    def retry_ready(self, driver_id):
        def status():
            return next(d for d in self.request("/admin/state")["drivers"] if d["id"] == driver_id)
        previous = status()["last_ready_ns"]
        self.request(f"/admin/drivers/{driver_id}/retry", "POST")
        # A scheduled retry may not have detached the previous session yet.
        # Require completion of a new readiness transition, not its old manifest.
        return wait_for(lambda: (s := status())["state"] in ["online", "degraded"] and s["last_ready_ns"] != previous)

    def login(self, password=PASSWORD):
        self.request("/auth/login", "POST", {"password": password})


def free_port(kind=socket.SOCK_DGRAM):
    with socket.socket(socket.AF_INET, kind) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def udp_data(sock, timeout=5):
    deadline = time.monotonic() + timeout
    sock.settimeout(.3)
    while time.monotonic() < deadline:
        try:
            packet, _ = sock.recvfrom(65535)
            if packet[1] == 3:
                assert packet[0] == 1
                return json.loads(packet[4:])
        except socket.timeout:
            pass
    raise AssertionError("UDP telemetry did not resume")


def browser_checks(client, screenshot):
    from playwright.sync_api import sync_playwright
    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=True, args=["--no-sandbox"])
        page = browser.new_page(viewport={"width": 1440, "height": 1000})
        errors, external = [], []
        page.on("pageerror", lambda e: errors.append(str(e)))
        def route(request):
            if not request.request.url.startswith(client.url + "/"):
                external.append(request.request.url)
                request.abort()
            else:
                request.continue_()
        page.route("**/*", route)
        page.goto(client.url)
        page.locator("#password").fill(PASSWORD)
        page.locator("#login-form button").click()
        page.locator("#console").wait_for(state="visible")
        wait_for(lambda: page.locator("#ready-count").inner_text() == "13")
        assert page.locator("#drivers tr").count() == 12
        page.screenshot(path=str(screenshot), full_page=True)
        page.locator('[data-view="connections"]').click()
        page.locator("#config-buttons button").filter(has_text="kinova").click()
        page.locator("#config-dialog").wait_for(state="visible")
        assert page.locator("#config-text").is_disabled()
        page.locator("#policy-max_attempts").fill("4")
        page.locator("#save-policy").click()
        wait_for(lambda: page.locator("#config-error").inner_text() == "Recovery policy saved.")
        page.locator("#close-config").click()
        page.locator('[data-view="api"]').click()
        page.locator("#operations details").first.wait_for()
        page.locator("#request-form button").click()
        wait_for(lambda: "sensors" in page.locator("#request-result").inner_text())
        page.locator('[data-view="logs"]').click()
        page.locator("#logs details summary").first.click()
        page.locator("#logs a").first.wait_for()
        page.locator('[data-view="commands"]').click()
        page.get_by_role('tab', name='kinova_arm', exact=True).click()
        page.on("dialog", lambda d: d.accept())
        page.locator('#sensor-estop').wait_for(state='visible')
        page.locator('#sensor-estop').click()
        wait_for(lambda: client.request('/kinova_arm/data')['limp_mode'] is True)
        wait_for(lambda: 'Emergency stop acknowledged.' in page.locator('#sensor-command-hint').inner_text())
        page.get_by_label('limp_mode', exact=True).select_option('false')
        page.locator('#sensor-command-send').click()
        wait_for(lambda: client.request('/kinova_arm/data')['limp_mode'] is False)
        page.get_by_role('tab', name='livox_40', exact=True).click()
        wait_for(lambda: 'declares no commands' in page.locator('#sensor-command-fields').inner_text())
        assert page.locator('#sensor-command-send').is_hidden()
        page.set_viewport_size({"width": 390, "height": 844})
        assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
        page.locator('[data-view="settings"]').click()
        page.locator('#current-password').fill(PASSWORD)
        new_password = "HALo-rotated-password-2026"
        page.locator('#new-password').fill(new_password)
        page.locator('#confirm-password').fill("a-mismatched-password")
        page.locator('#password-form button').click()
        wait_for(lambda: 'match' in page.locator('#password-error').inner_text())
        assert page.locator('#console').is_visible()
        page.locator('#confirm-password').fill(new_password)
        page.locator('#password-form button').click()
        page.locator('#login-panel').wait_for(state='visible')
        assert 'Password changed' in page.locator('#login-error').inner_text()
        # Every existing session is revoked by a successful rotation.
        client.request('/admin/state', expected=401)
        time.sleep(2.1)
        page.locator('#password').fill(new_password)
        page.locator('#login-form button').click()
        page.locator('#console').wait_for(state='visible')
        page.locator("#logout").click()
        page.locator("#login-panel").wait_for(state="visible")
        assert not errors, errors
        assert not external, external
        browser.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=HALO / "target/debug/halo")
    parser.add_argument("--split-lidar", action="store_true", help="Use the fleet-style separate raycast worker")
    parser.add_argument("--browser", action="store_true")
    parser.add_argument("--world-config", type=Path, help="Use a simulator world YAML")
    parser.add_argument("--scene", type=Path, help="Use a saved simulator scene JSON")
    parser.add_argument("--screenshot", type=Path, default=Path("/tmp/halo-console.png"))
    args = parser.parse_args()
    world_args = []
    for flag, path in [("--world-config", args.world_config), ("--scene", args.scene)]:
        if path: world_args.extend([flag, str(path.resolve())])
    processes, files = [], []
    with tempfile.TemporaryDirectory(prefix="halo-e2e-") as folder:
        root = Path(folder)
        config = root / "config"
        config.mkdir()
        http_port = free_port(socket.SOCK_STREAM)
        # Reserve every served/backend pair explicitly so parallel previews cannot collide.
        ids = ["vectornav_sim", "kinova_arm", "robotiq_gripper"] + [f"odrive_{n}" for n in (31,32,33,34,41,42,43,44)] + ["livox_40", "livox_41"]
        ports = "served_base = 28000\nbackend_base = 29000\n"
        ports += f"control_port = {free_port()}\nlivox_imu_port = {free_port()}\n"
        for sensor in ids:
            ports += f"\n[overrides.{sensor}]\nserved_data = {free_port()}\nserved_cmd = {free_port()}\nbackend = {free_port()}\n"
        (config / "ports.toml").write_text(ports)
        (config / "server.toml").write_text(f"http_port = {http_port}\ndefault_push_interval_ms = 50\n")
        (config / "sim.toml").write_text('host = "127.0.0.1"\nports_file = "ports.toml"\n')
        (config / "recovery.toml").write_text('max_attempts = 2\nattempt_timeout_ms = 3000\nretry_delay_ms = 200\nstale_after_ms = 1000\n')
        env = dict(os.environ, HALO_CONFIG_DIR=str(config), LOG_DIR=str(root / "logs"))
        env.pop("HALO_SIM_BACKEND", None)
        env.pop("ROVE_SIM_BACKEND", None)
        subprocess.run([str(args.binary), "--set-password"], input=PASSWORD+"\n", text=True, env=env, check=True, capture_output=True)
        password_hash = (config / "password.hash").read_text()
        assert password_hash.startswith("$argon2id$") and PASSWORD not in password_hash
        assert (config / "password.hash").stat().st_mode & 0o777 == 0o600
        def spawn(command, name, cwd):
            log = open(root / name, "w+")
            files.append(log)
            process = subprocess.Popen(command, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
            processes.append(process)
            return process
        def start_halo():
            return spawn([str(args.binary)], "halo.log", HALO)
        try:
            halo = start_halo()
            client = Client(http_port)
            wait_for(lambda: client.request("/auth/status")["configured"])
            assert client.manifest() == {}
            client.request("/admin/state", expected=401)
            client.request("/auth/login", "POST", {"password": PASSWORD}, header=False, expected=403)
            client.request("/auth/login", "POST", {"password": "incorrect"}, expected=401)
            client.request("/auth/login", "POST", {"password": PASSWORD}, expected=429)
            time.sleep(2.1)
            client.login()
            wait_for(lambda: client.request("/kinova_arm/health")["controller_connected"])
            assert not client.request("/kinova_arm/health")["hardware_ready"]
            assert not client.request("/robotiq_gripper/health")["hardware_ready"]
            client.request("/admin/drivers/kinova/retry", "POST", header=False, expected=403)
            wait_for(lambda: all(d["state"] == "failed" for d in client.request("/admin/state")["drivers"]))
            saved_ports = client.request("/admin/state")["ports"]
            spawn([sys.executable, str(SIM / "tools/sim_server.py"), "--profile", "standard", *world_args, "--no-texture", "--no-rtsp", *(["--no-lidar"] if args.split_lidar else ["--lidar-rays", "256"]), "--rsa-backend", "--ports", str(config / "ports.toml"), "--state-file", str(root / "state.json")], "simulator.log", SIM)
            wait_for(lambda: (root / "state.json").exists())
            if args.split_lidar:
                spawn([sys.executable, str(SIM / "tools/lidar_worker.py"), "--profile", "standard", *world_args, "--no-texture", "--rsa-backend", "--ports", str(config / "ports.toml"), "--state-file", str(root / "state.json"), "--rays", "256", "--threads", "1"], "lidar_worker.log", SIM)
                # Terrain/scene startup can take longer than the short retry policy.
                # Wait for this source to finish loading before exercising recovery.
                wait_for(lambda: '[lidar_worker]' in (root / 'lidar_worker.log').read_text(), timeout=40)
            for driver in client.request("/admin/state")["drivers"]:
                client.request(f'/admin/drivers/{driver["id"]}/retry', "POST")
            wait_for(lambda: len(client.manifest()) == 13)
            assert saved_ports == client.request("/admin/state")["ports"]
            # Multiple clients use the same standard Subscribe / DATA contract.
            lidar = client.manifest()["livox_40"]
            assert lidar["data_delivery"] == "Updates"
            listeners = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
            try:
                for listener in listeners:
                    listener.bind(("127.0.0.1", 0)); listener.settimeout(.5)
                    listener.sendto(struct.pack("<BBH",1,1,55), ("127.0.0.1",lidar["data_port"]))
                def check_subscribers():
                    received = [dict(), dict()]
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        for i, listener in enumerate(listeners):
                            try: packet = listener.recv(65535)
                            except socket.timeout: continue
                            if packet[1] != 3: continue
                            assert packet[0] == 1
                            data = json.loads(packet[4:]); kind = data['packet_type']
                            key = (kind, data['points_timestamp_ns'] if kind == 'points' else data['imu_timestamp_ns'], data['point_packet_index'])
                            received[i][key] = data
                        common = received[0].keys() & received[1].keys()
                        if {key[0] for key in common} >= {'points','imu'}:
                            for key in common: assert received[0][key] == received[1][key]
                            return
                    raise AssertionError('Both subscribers did not receive matching point and IMU DATA')
                check_subscribers()
                client.retry_ready('livox')
                check_subscribers() # Stable subscription sockets survive SDK/feed replacement.
                listeners[0].sendto(struct.pack('<BBH',1,2,56),('127.0.0.1',lidar['data_port']))
                wait_for(lambda: listeners[0].recv(65535)[1] == 4)
                listeners[0].settimeout(.15)
                try: listeners[0].recv(65535); raise AssertionError('Unsubscribed client still receives data')
                except socket.timeout: pass
                assert udp_data(listeners[1])['lidar_id'] == 40
            finally:
                for listener in listeners: listener.close()
            sensor = client.manifest()["odrive_31"]
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                udp.bind(("127.0.0.1", 0))
                udp.sendto(struct.pack("<BBH",1,1,42)+b'{"interval_ms":50}', ("127.0.0.1", sensor["data_port"]))
                assert "axis_state" in udp_data(udp)
                client.retry_ready("kinova")
                wait_for(lambda: "kinova_arm" in client.manifest())
                assert "axis_state" in udp_data(udp)
            client.request("/kinova_arm/command", "POST", {"limp_mode":"true"}, expected=400)
            before = client.request("/kinova_arm/data")
            assert client.request('/kinova_arm/info')['has_estop'] is True
            assert client.manifest()['kinova_arm']['endpoints']['estop'] == '/kinova_arm/estop'
            client.request('/admin/sensors/kinova_arm/estop', 'POST', header=False, expected=403)
            client.request('/admin/sensors/kinova_arm/estop', 'POST')
            wait_for(lambda: client.request("/kinova_arm/data").get("limp_mode") is True)
            time.sleep(.5)
            after = client.request("/kinova_arm/data")
            assert any(abs(after[f"joint_{i}_pos"]-before[f"joint_{i}_pos"]) > .5 for i in range(1,7)), "sim arm did not settle under gravity"
            assert sum(after[f"joint_{i}_current"] for i in range(1,7)) < .01, f"sim motors still apply torque: {[after[f'joint_{i}_current'] for i in range(1,7)]}"
            client.request("/kinova_arm/command", "POST", {"move_home":True}, expected=503)
            client.request("/admin/sensors/kinova_arm/command", "POST", {"limp_mode":False})
            wait_for(lambda: client.request("/kinova_arm/data").get("limp_mode") is False)
            client.request("/odrive_31/estop", "POST")
            wait_for(lambda: client.request("/odrive_31/data")["active_errors"] != 0)
            client.request("/admin/drivers/odrive_31/retry", "POST")
            wait_for(lambda: any(d["id"] == "odrive_31" and d["state"] == "connecting" for d in client.request("/admin/state")["drivers"]))
            wait_for(lambda: client.request("/odrive_31/data")["timestamp_ns"] != 0)
            assert client.request("/odrive_31/data")["active_errors"] != 0, "retry erased controller fault"
            wait_for(lambda: any('Emergency stop requested' in error for error in client.request('/odrive_31/health')['telemetry']['errors']))
            client.request("/odrive_31/command", "POST", {"clear_errors":True})
            wait_for(lambda: "odrive_31" in client.manifest())
            new_pair = [free_port(), free_port()]
            client.request("/admin/sensors/kinova_arm/ports", "POST", {"data_port":new_pair[0],"command_port":new_pair[1]})
            assert client.manifest()["kinova_arm"]["data_port"] == new_pair[0]
            ports_after = client.request("/admin/state")["ports"]
            spec = client.request("/openapi.json")
            assert "/kinova_arm/health" in spec["paths"] and "CommandMode" in spec["components"]["schemas"]
            lidar_schema = spec["paths"]["/livox_40/data"]["get"]["responses"]["200"]["content"]["application/json"]["schema"]
            if "$ref" in lidar_schema:
                lidar_schema = spec["components"]["schemas"][lidar_schema["$ref"].rsplit("/",1)[1]]
            lidar_schema = lidar_schema["properties"]
            assert lidar_schema["points"]["type"] == "array"
            assert lidar_schema["points"]["items"]["minItems"] == 3
            assert lidar_schema["points"]["items"]["items"]["type"] == "number"
            assert any(item.get("type") == "null" for item in lidar_schema["gyro"]["anyOf"])

            assert any(f["name"] == "limp_mode" for f in client.request("/kinova_arm/info")["command_schema"])
            assert client.request("/health")["status"] == "ready"
            client.request("/admin/reload", "POST")
            assert halo.wait(timeout=5) == 1
            start_halo()
            client = Client(http_port)
            wait_for(lambda: len(client.manifest()) == 13)
            client.login()
            assert client.request("/admin/state")["ports"] == ports_after
            assert any(d["previous_state"] for d in client.request("/admin/state")["drivers"])
            assert client.request("/logs")["days"]
            client.request('/auth/password', 'POST', {'current_password': PASSWORD, 'new_password': 'short'}, expected=400)
            client.request('/auth/password', 'POST', {'current_password': PASSWORD, 'new_password': PASSWORD}, header=False, expected=403)
            time.sleep(2.1)
            client.request('/auth/password', 'POST', {'current_password': 'incorrect', 'new_password': PASSWORD}, expected=400)
            assert client.request('/auth/status')['authenticated'] is True
            time.sleep(2.1)
            # Rotate to the same text: even this must produce a fresh salt and revoke sessions.
            client.request('/auth/password', 'POST', {'current_password': PASSWORD, 'new_password': PASSWORD})
            rotated_hash = (config / 'password.hash').read_text()
            assert rotated_hash != password_hash and rotated_hash.startswith('$argon2id$')
            assert (config / 'password.hash').stat().st_mode & 0o777 == 0o600
            client.request('/admin/state', expected=401)
            time.sleep(2.1)
            client.login()
            if args.browser:
                time.sleep(2.1)
                browser_checks(client, args.screenshot)
                # Restart proves the Settings password is persisted, not just cached.
                halo = processes[-1]
                halo.terminate(); halo.wait(timeout=5)
                start_halo()
                client = Client(http_port)
                wait_for(lambda: len(client.manifest()) == 13)
                client.request('/auth/login', 'POST', {'password': PASSWORD}, expected=401)
                time.sleep(2.1)
                client.login('HALo-rotated-password-2026')
                assert client.request('/admin/state')['ports'] == ports_after
            print("PASS: simulator readiness, authentication, targeted recovery, UDP continuity, estop/limp physics, readable latched faults, password rotation, port persistence, full bounce, discovery/OpenAPI" + ", both lidars and standard UDP multi-subscriber delivery" + (", offline browser" if args.browser else ""))
        except Exception:
            try:
                print('HALo health at failure:', json.dumps(client.request('/health')), file=sys.stderr)
            except Exception:
                pass
            for log in files:
                log.flush(); log.seek(0)
                print(log.name, log.read()[-4000:], file=sys.stderr)
            raise
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    process.terminate()
                    try: process.wait(timeout=5)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
            for log in files:
                log.close()


if __name__ == "__main__":
    main()
