"""Optional simulator-only RoveControl front door for the authenticated viewer.

Track-only protobuf encoding follows rove_control_bridge/proto/RoveControl.proto:
RoveControl.tracks (field 1), Tracks.left_vel/right_vel (float fields 1/2).
No HALo commands or hardware ports are used here.
"""

import math
import socket
import struct
import threading
import time
import urllib.request


def track_packet(left, right):
    tracks = b"\x0d" + struct.pack("<f", left) + b"\x15" + struct.pack("<f", right)
    return b"\x0a\x0a" + tracks


class Teleop:
    def __init__(self, port, timeout=0.3):
        self.target = ("127.0.0.1", port)
        self.timeout = timeout
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lock = threading.Lock()
        self.owner = None
        self.sequence = {}
        self.last = 0
        self.health_checked = 0
        self.bridge_available = False
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self._watchdog, daemon=True)
        self.thread.start()

    def update(self, session, data):
        left, right = data["left"], data["right"]
        client, sequence = data["client"], data["sequence"]
        if any(
            type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 0.25
            for v in (left, right)
        ):
            raise ValueError("Track commands must be finite and within ±0.25")
        if not isinstance(client, str) or not 1 <= len(client) <= 64:
            raise ValueError("Invalid controller identifier")
        if type(sequence) is not int or not 0 <= sequence <= 2**53:
            raise ValueError("Invalid command sequence")
        owner = (session, client)
        with self.lock:
            now = time.monotonic()
            if now - self.health_checked > 0.5:
                try:
                    with urllib.request.urlopen(
                        "http://127.0.0.1:8088/healthz", timeout=0.2
                    ) as response:
                        self.bridge_available = response.status == 200
                except OSError:
                    self.bridge_available = False
                self.health_checked = now
            if not self.bridge_available:
                self._stop()
                raise ValueError(
                    "Control bridge unavailable — simulator control stopped"
                )
            if self.owner and now - self.last > self.timeout:
                self._stop()
            if self.owner not in (None, owner):
                raise ValueError("Another browser is controlling the simulator")
            if sequence <= self.sequence.get(owner, (-1, 0))[0]:
                raise ValueError("Out-of-order command")
            # Expire inactive client records, keeping the active lease bounded.
            self.sequence = {
                k: v for k, v in self.sequence.items() if now - v[1] < 28800
            }
            if owner not in self.sequence and len(self.sequence) >= 1024:
                raise ValueError("Too many controller sessions")
            self.sequence[owner] = (sequence, now)
            self.socket.sendto(track_packet(left, right), self.target)
            self.owner = owner if left or right else None
            self.last = now

    def _stop(self):
        self.socket.sendto(track_packet(0, 0), self.target)
        self.owner = None

    def logout(self, session):
        with self.lock:
            if self.owner and self.owner[0] == session:
                self._stop()

    def _watchdog(self):
        while not self.closed.wait(0.05):
            with self.lock:
                if self.owner and time.monotonic() - self.last > self.timeout:
                    self._stop()

    def close(self):
        self.closed.set()
        self.thread.join()
        with self.lock:
            self._stop()
            self.socket.close()
