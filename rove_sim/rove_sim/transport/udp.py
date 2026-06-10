"""UDP telemetry transport -- the real wire path.

A publisher (mock SimSensorApi) sends DATA frames to a per-channel UDP port; a
subscriber (real-mode RobotStateSource) binds those ports and keeps the latest
decoded payload per channel from a background rx thread. Channel->port mapping
defaults to the rove_sensor_api auto-assignment (data ports 5000, 5002, ...),
overridable per channel.

This is intentionally lightweight: no SUBSCRIBE handshake (the publisher pushes
to a known host:port). Against the real robot, point a subscriber at the real
data ports instead -- same decode path.
"""
from __future__ import annotations

import socket
import threading
from typing import Dict, Optional

from . import Transport
from .packet import MSG_DATA, decode, encode

# rove_sensor_api auto-assigns data ports from 5000 upward (data,cmd,data,cmd...)
DEFAULT_PORTS = {"vectornav": 5000, "kinova": 5002, "robotiq": 5004,
                 "odrive_31": 5006, "odrive_32": 5008, "odrive_33": 5010,
                 "odrive_34": 5012, "pmic": 5014,
                 # sim additions: control in, lidar point clouds out, ground truth
                 "control": 5020, "livox_top": 5022, "livox_bottom": 5024,
                 "ground_truth": 5030,
                 # robot pose+joint state for decoupled render workers (cam_worker)
                 "robot_state": 5032}


class UdpTransport(Transport):
    def __init__(self, host: str = "127.0.0.1", ports: Optional[dict] = None,
                 role: str = "pub", subscribe: Optional[list] = None, **_):
        self.host = host
        self.ports = dict(DEFAULT_PORTS, **(ports or {}))
        self.role = role
        # a subscriber binds ONLY these channels (default all) -- so e.g. the
        # control sub doesn't also grab the telemetry ports an external consumer
        # wants. Publishers ignore it.
        self.subscribe = list(subscribe) if subscribe is not None else None
        self._seq = 0
        self._tx: Optional[socket.socket] = None
        self._rx: Dict[str, socket.socket] = {}
        self._latest: Dict[str, dict] = {}
        self._threads: list = []
        self._stop = threading.Event()

    def start(self) -> None:
        if self.role == "pub":
            self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            return
        for channel, port in self.ports.items():
            if self.subscribe is not None and channel not in self.subscribe:
                continue
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, port))
            s.settimeout(0.2)
            self._rx[channel] = s
            t = threading.Thread(target=self._rx_loop, args=(channel, s),
                                 daemon=True)
            t.start()
            self._threads.append(t)

    def _rx_loop(self, channel: str, sock: socket.socket) -> None:
        while not self._stop.is_set():
            try:
                data, _ = sock.recvfrom(65535)
                mt, _, payload = decode(data)
                if mt == MSG_DATA and payload is not None:
                    self._latest[channel] = payload
            except socket.timeout:
                continue
            except OSError:
                break

    def publish(self, channel: str, payload: dict) -> None:
        if self._tx is None:
            return
        port = self.ports.get(channel)
        if port is None:
            return
        self._seq += 1
        self._tx.sendto(encode(MSG_DATA, self._seq, payload), (self.host, port))

    def latest(self, channel: str) -> Optional[dict]:
        return self._latest.get(channel)

    def stop(self) -> None:
        self._stop.set()
        for s in self._rx.values():
            try:
                s.close()
            except OSError:
                pass
        if self._tx is not None:
            self._tx.close()
