"""The bridge runtime: UDP receive loop, IDLE/ACTIVE state machine, dispatch.

State machine
─────────────
                first packet arrives
    IDLE ──────────────────────────────────► ACTIVE
     ▲   convert() includes axis_state=8       │
     │                                         │
     │   no packet for idle_timeout_s          │
     └─────────────────────────────────────────┘
          zero setpoints, axis_state=1

Data flow per received packet
─────────────────────────────
    UDP RoveControl ──► ConversionStrategy.convert()
                              │
                              ▼
                       list[NodeCommand]
                              │
                              ▼
                    SensorApiUdpClient.send_command()  (one per node)
                              │
              ┌───────────────┴────────────────┐
              ▼                                ▼
     gripper.position → robotiq      ovis → rove_ik_engine
     command port                    (OvisForwarder)
"""
from __future__ import annotations

import logging
import socket
import threading
import time

from ..config import BridgeConfig
from ..strategies.base import ConversionStrategy, NodeCommand
from ..transport import OvisForwarder, SensorApiUdpClient

log = logging.getLogger(__name__)

_AXIS_IDLE = 1
_AXIS_CLOSED_LOOP = 8


class RoveControlBridge:
    """Receive RoveControl protos, convert, and forward to rove_sensor_api."""

    def __init__(
        self,
        cfg: BridgeConfig,
        strategy: ConversionStrategy,
        port_map: dict[int, int],
        gripper_port: int | None = None,
        ovis_forwarder: OvisForwarder | None = None,
    ) -> None:
        self._cfg = cfg
        self._strategy = strategy
        self._port_map = port_map           # {node_id: cmd_port}
        self._gripper_port = gripper_port   # None = gripper disabled
        self._last_gripper_pos: int = -1    # sentinel: force first send
        self._ovis_forwarder = ovis_forwarder
        self._client = SensorApiUdpClient(cfg.sensor_api.host)

        self._lock = threading.Lock()
        self._running = False
        self._active = False                # IDLE/ACTIVE state machine flag
        self._last_rx_t = 0.0               # monotonic time of last packet

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Start the bridge. Blocks until stop() is called."""
        self._running = True
        self._last_rx_t = time.monotonic()

        init_cmds = self._strategy.initialize()
        if init_cmds:
            log.info("Sending %d control-mode init commands …", len(init_cmds))
            for cmd in init_cmds:
                self._dispatch(cmd)

        log.info(
            "ODrives ready in Idle — will activate on first RoveControl packet "
            "(idle timeout %.2f s)",
            self._cfg.idle_timeout_s,
        )

        watchdog = threading.Thread(
            target=self._idle_watchdog_loop, name="idle-watchdog", daemon=True
        )
        watchdog.start()

        self._receive_loop()

        if self._active:
            self._set_idle(reason="bridge stopped")

        watchdog.join(timeout=2.0)
        self._client.close()
        if self._ovis_forwarder is not None:
            self._ovis_forwarder.close()
        log.info("Bridge stopped.")

    def stop(self) -> None:
        self._running = False

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _set_active(self) -> None:
        """IDLE → ACTIVE: flip the flag; convert() carries axis_state=8."""
        with self._lock:
            if self._active:
                return
            self._active = True
        log.info("ODrives → ClosedLoopControl (via stream)")

    def _set_idle(self, reason: str = "") -> None:
        """ACTIVE → IDLE: zero setpoints, then disarm."""
        with self._lock:
            if not self._active:
                return
            self._active = False

        log.info("ODrives → Idle (axis_state=%d)%s",
                 _AXIS_IDLE, f" ({reason})" if reason else "")
        for cmd in self._strategy.estop():
            self._dispatch(cmd)
        self._send_axis_state(_AXIS_IDLE)

    def _send_axis_state(self, state: int) -> None:
        for node_id in self._port_map:
            self._dispatch(NodeCommand(node_id=node_id, payload={"axis_state": state}))

    # ------------------------------------------------------------------
    # Internal loops
    # ------------------------------------------------------------------

    def _receive_loop(self) -> None:
        """Listen for RoveControl datagrams and dispatch conversions."""
        # Lazy import so build_protos.py failures surface as a clear runtime
        # error rather than a module-load crash.
        try:
            from ..proto.core import RoveControl_pb2
        except ImportError as exc:
            raise RuntimeError(
                "Protobuf generated files not found. "
                "Run build_protos.py before starting the bridge."
            ) from exc

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.bind((self._cfg.listen.host, self._cfg.listen.port))
        sock.settimeout(0.25)

        log.info(
            "Listening for RoveControl on %s:%d",
            self._cfg.listen.host, self._cfg.listen.port,
        )

        last_log_t = 0.0
        rx_count = 0
        warn_t = time.monotonic()
        window_start = time.monotonic()
        window_count = 0
        LOG_HEARTBEAT = 2.0

        while self._running:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                now = time.monotonic()
                if not self._active and (now - warn_t) >= 5.0:
                    log.warning(
                        "No RoveControl packets in %.0fs — "
                        "is the steamdeck sending to %s:%d?",
                        now - self._last_rx_t,
                        self._cfg.listen.host,
                        self._cfg.listen.port,
                    )
                    warn_t = now
                continue
            except OSError as exc:
                if self._running:
                    log.warning("UDP recv error: %s", exc)
                continue

            try:
                msg = RoveControl_pb2.RoveControl()
                msg.ParseFromString(data)
            except Exception as exc:
                log.debug("Failed to parse RoveControl from %s: %s", addr, exc)
                continue

            rx_count += 1
            warn_t = time.monotonic()

            if not self._active:
                self._last_rx_t = time.monotonic()
                self._set_active()
            else:
                self._last_rx_t = time.monotonic()

            cmds = self._strategy.convert(msg)
            for cmd in cmds:
                self._dispatch(cmd)

            self._send_gripper(msg.gripper.position)

            if self._ovis_forwarder is not None:
                self._ovis_forwarder.forward(msg.ovis)

            window_count += 1

            # DEBUG every packet; INFO only on the 2 s heartbeat. Don't log
            # at INFO every packet — at 100 Hz Python's logging is slow
            # enough to block the receive loop ~100-500 ms and trigger the
            # idle watchdog during normal driving.
            t = msg.tracks
            f = msg.flippers
            log.debug(
                "[#%05d] rx L=%+.3f R=%+.3f  fl/fr/rl/rr=%+d/%+d/%+d/%+d",
                rx_count, t.left_vel, t.right_vel, f.fl, f.fr, f.rl, f.rr,
            )

            now = time.monotonic()
            if (now - last_log_t) >= LOG_HEARTBEAT:
                elapsed = now - window_start
                hz = window_count / elapsed if elapsed > 0 else 0.0
                cmd_summary = "  ".join(
                    f"node{c.node_id}={list(c.payload.values())[0]:+.2f}"
                    for c in cmds
                ) if cmds else "—"
                log.info(
                    "[#%05d %5.1fHz] rx L=%+.3f R=%+.3f  fl/fr/rl/rr=%+d/%+d/%+d/%+d  → %s",
                    rx_count, hz,
                    t.left_vel, t.right_vel, f.fl, f.fr, f.rl, f.rr,
                    cmd_summary,
                )
                last_log_t = now
                window_start = now
                window_count = 0

        sock.close()

    def _idle_watchdog_loop(self) -> None:
        """Transition to IDLE when no packet has arrived for idle_timeout_s."""
        check_interval = min(0.05, self._cfg.idle_timeout_s / 4)
        while self._running:
            time.sleep(check_interval)
            with self._lock:
                is_active = self._active
            if is_active:
                silence = time.monotonic() - self._last_rx_t
                if silence >= self._cfg.idle_timeout_s:
                    self._set_idle(reason=f"no packets for {silence:.3f}s")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _send_gripper(self, position: int) -> None:
        """Send a gripper command only when the position has changed."""
        if self._gripper_port is None:
            return
        if position == self._last_gripper_pos:
            return
        payload = {
            "position": position,
            "speed": self._cfg.gripper.speed,
            "force": self._cfg.gripper.force,
            "activate": True,
            "goto": True,
        }
        ok = self._client.send_command(self._gripper_port, payload)
        log.debug("→ gripper :%d  pos=%d  [%s]",
                  self._gripper_port, position, "ok" if ok else "DROP")
        if ok:
            self._last_gripper_pos = position

    def _dispatch(self, cmd: NodeCommand) -> None:
        port = self._port_map.get(cmd.node_id)
        if port is None:
            log.warning(
                "No command port for ODrive node %d — not in /discover result.",
                cmd.node_id,
            )
            return
        ok = self._client.send_command(port, cmd.payload)
        log.debug("→ node%d :%d  %s  [%s]",
                  cmd.node_id, port, cmd.payload, "ok" if ok else "DROP")
