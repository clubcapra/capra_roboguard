#!/usr/bin/env python3
"""Authenticated static viewer for GPU frames; with optional simulator teleoperation through the control bridge."""

import argparse
import hashlib
import hmac
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--image", required=True)
    parser.add_argument("--control", required=True)
    parser.add_argument("--metrics", required=True)
    parser.add_argument("--password-env", default="SIM_VIEW_PASSWORD")
    parser.add_argument(
        "--teleop-port",
        type=int,
        help="Enable simulator WASD via the local bridge UDP port",
    )
    args = parser.parse_args()
    if args.teleop_port is not None and not 1 <= args.teleop_port <= 65535:
        parser.error("Invalid teleop port")
    from web_teleop import Teleop

    teleop = Teleop(args.teleop_port) if args.teleop_port else None
    password = os.environ.get(args.password_env)
    if not password:
        parser.error(f"Set {args.password_env} before starting the viewer")
    # Temporary viewer credentials: retain only a salted slow hash in memory.
    salt = secrets.token_bytes(16)
    expected = hashlib.scrypt(password.encode(), salt=salt, n=16384, r=8, p=1)
    del password
    sessions = {}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def authenticated(self):
            cookie = SimpleCookie()
            try:
                cookie.load(self.headers.get("Cookie", ""))
                token = cookie["sim_session"].value
            except (KeyError, ValueError):
                return False
            return sessions.get(token, 0) > time.monotonic()

        def reply(self, code, data, kind="application/json", cookie=None):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if cookie:
                self.send_header("Set-Cookie", cookie)
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in (
                "/",
                "/vnc_lite.html",
                "/app.js",
                "/style.css",
                "/control",
                "/control.js",
            ):
                name, mime = {
                    "/": ("index.html", "text/html"),
                    "/control": ("control.html", "text/html"),
                    "/control.js": ("control.js", "text/javascript"),
                    "/vnc_lite.html": ("index.html", "text/html"),
                    "/app.js": ("app.js", "text/javascript"),
                    "/style.css": ("style.css", "text/css"),
                }[path]
                self.reply(200, (ROOT / "ui" / name).read_bytes(), mime)
                return
            if not self.authenticated():
                self.reply(401, b"Login required", "text/plain")
                return
            if path == "/view":
                try:
                    self.reply(200, Path(args.control).read_bytes())
                except FileNotFoundError:
                    self.reply(200, b'{"yaw":50,"pitch":-25,"distance":4}')
            elif path == "/status":
                try:
                    metrics = Path(args.metrics)
                    if time.time() - metrics.stat().st_mtime > 6:
                        raise OSError("Renderer metrics are stale")
                    self.reply(200, metrics.read_bytes())
                except OSError:
                    self.reply(503, b"Renderer starting", "text/plain")
            elif path.startswith("/maps/") and path[6:] in (
                "forest_livox_top.png",
                "forest_livox_bottom.png",
            ):
                target = ROOT / "media" / "lidar" / path[6:]
                if target.exists():
                    self.reply(200, target.read_bytes(), "image/png")
                else:
                    self.reply(404, b"Map unavailable", "text/plain")
            elif path == "/stream.mjpg":
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                self.close_connection = True
                stamp = 0
                try:
                    while self.authenticated():
                        target = Path(args.image)
                        try:
                            current = target.stat().st_mtime_ns
                            if current != stamp:
                                data = target.read_bytes()
                                self.wfile.write(
                                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                    + str(len(data)).encode()
                                    + b"\r\n\r\n"
                                    + data
                                    + b"\r\n"
                                )
                                self.wfile.flush()
                                stamp = current
                        except FileNotFoundError:
                            pass
                        time.sleep(0.02)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.reply(404, b"Not found", "text/plain")

        def do_POST(self):
            # Custom header prevents cross-site form requests over this LAN UI.
            if self.headers.get("X-Sim-Request") != "1":
                self.reply(403, b"Invalid request", "text/plain")
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size > 4096 or size < 0:
                    raise ValueError("Invalid request size")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("Expected an object")
                if self.path == "/login":
                    supplied = hashlib.scrypt(
                        str(data.get("password", "")).encode(),
                        salt=salt,
                        n=16384,
                        r=8,
                        p=1,
                    )
                    if not hmac.compare_digest(supplied, expected):
                        self.reply(401, b"Incorrect password", "text/plain")
                        return
                    now = time.monotonic()
                    for token in list(sessions):
                        if sessions[token] <= now:
                            del sessions[token]
                    token = secrets.token_urlsafe(32)
                    sessions[token] = now + 8 * 3600
                    self.reply(
                        200,
                        b"{}",
                        cookie=f"sim_session={token}; HttpOnly; SameSite=Strict; Path=/; Max-Age=28800",
                    )
                    return
                if not self.authenticated():
                    self.reply(401, b"Login required", "text/plain")
                    return
                if self.path == "/logout":
                    cookie = SimpleCookie(self.headers.get("Cookie", ""))
                    token = cookie["sim_session"].value
                    if teleop:
                        teleop.logout(token)
                    sessions.pop(token, None)
                    self.reply(
                        200,
                        b"{}",
                        cookie="sim_session=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0",
                    )
                elif self.path == "/drive":
                    if not teleop:
                        self.reply(404, b"Teleoperation disabled", "text/plain")
                        return
                    cookie = SimpleCookie(self.headers.get("Cookie", ""))
                    teleop.update(cookie["sim_session"].value, data)
                    self.reply(200, b"{}")
                elif self.path == "/view":
                    control = {
                        key: float(data[key]) for key in ("yaw", "pitch", "distance")
                    }
                    if not all(math.isfinite(v) for v in control.values()):
                        raise ValueError("View values must be finite")
                    control["yaw"] %= 360
                    control["pitch"] = max(-85, min(-5, control["pitch"]))
                    control["distance"] = max(1, min(30, control["distance"]))
                    layers = data.get("layers", {})
                    if not isinstance(layers, dict) or any(
                        key not in ("scene", "lidar", "cameras", "radiation")
                        or type(value) is not bool
                        for key, value in layers.items()
                    ):
                        raise ValueError("Unknown layer or invalid toggle")
                    target = Path(args.control)
                    temp = target.with_name(target.name + ".tmp")
                    # Concurrent requests serialize publication using the server lock.
                    with self.server.control_lock:
                        try:
                            saved = json.loads(target.read_text()).get("layers", {})
                        except (OSError, ValueError):
                            saved = {}
                        control["layers"] = {
                            **dict(
                                scene=True, lidar=False, cameras=False, radiation=False
                            ),
                            **saved,
                            **layers,
                        }
                        temp.write_text(json.dumps(control))
                        os.replace(temp, target)
                    self.reply(200, b"{}")
                else:
                    self.reply(404, b"Not found", "text/plain")
            except (ValueError, KeyError, TypeError) as exc:
                self.reply(400, str(exc).encode(), "text/plain")

        def log_message(self, *_):
            pass

    import threading

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.control_lock = threading.Lock()
    print(f"Simulator viewer http://{args.host}:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if teleop:
            teleop.close()
        server.server_close()


if __name__ == "__main__":
    main()
