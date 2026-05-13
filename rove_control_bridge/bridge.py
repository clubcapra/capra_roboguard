"""Main conversion bridge: receive RoveControl → convert → stream to rove_sensor_api.

State machine
─────────────
                    first packet arrives
    IDLE ──────────────────────────────────► ACTIVE
     ▲   set control_mode + axis_state=8       │
     │                                         │  keepalive loop runs,
     │   no packet for idle_timeout_s          │  commands forwarded
     └─────────────────────────────────────────┘
          zero setpoints, axis_state=1

Architecture
────────────
    ┌──────────────────────────────────────┐
    │  UDP listener (:9101)                │
    │  RoveControl.proto datagram          │
    └───────────────┬──────────────────────┘
                    │ parse protobuf
                    ▼
    ┌──────────────────────────────────────┐
    │  ConversionStrategy                  │
    │  (TracksVelocity / TracksTorque …)   │
    └───────────────┬──────────────────────┘
                    │ list[NodeCommand]
                    ▼
    ┌──────────────────────────────────────┐
    │  SensorApiUdpClient                  │
    │  Command packets → rove_sensor_api   │
    │  ODrive node command ports           │
    └──────────────────────────────────────┘
"""
from __future__ import annotations

import logging
import socket
import threading
import time

from .config import BridgeConfig
from .ovis_forwarder import OvisForwarder
from .sensor_api_client import (
    SensorApiUdpClient,
    discover_odrive_ports,
    discover_sensor_command_port,
)
from .strategies.base import ConversionStrategy, NodeCommand

log = logging.getLogger(__name__)

_AXIS_IDLE         = 1
_AXIS_CLOSED_LOOP  = 8


class RoveControlBridge:
    """Receive RoveControl protos, convert, and forward to rove_sensor_api."""

    def __init__(
        self,
        cfg: BridgeConfig,
        strategy: ConversionStrategy,
        port_map: dict[int, int],
        gripper_port: int | None = None,
        ovis_forwarder: "OvisForwarder | None" = None,
    ) -> None:
        self._cfg = cfg
        self._strategy = strategy
        self._port_map = port_map          # {node_id: cmd_port}
        self._gripper_port = gripper_port  # None = gripper not present / disabled
        self._last_gripper_pos: int = -1   # sentinel: force first send
        self._ovis_forwarder = ovis_forwarder
        self._client = SensorApiUdpClient(cfg.sensor_api.host)

        self._lock = threading.Lock()
        self._running = False

        # State machine
        self._active = False               # True = ACTIVE, False = IDLE
        self._last_rx_t = 0.0             # monotonic time of last received packet

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Start the bridge.  Blocks until stop() is called."""
        self._running = True
        self._last_rx_t = time.monotonic()

        # Send control_mode (velocity or torque) to all nodes at startup
        # so the ODrive knows what kind of setpoints to expect.
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

        # Idle watchdog: sends zero-vel + axis_state=1 after silence.
        # No separate keepalive loop — rove_sensor_api's own CAN watchdog
        # handles the ODrive-level heartbeat; we must not inject zero-velocity
        # packets between real ones or the robot shakes.
        watchdog = threading.Thread(
            target=self._idle_watchdog_loop, name="idle-watchdog", daemon=True
        )
        watchdog.start()

        # Proto receive loop (blocking — exits when self._running is False).
        self._receive_loop()

        # Graceful shutdown: zero motion and go idle.
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
        """IDLE → ACTIVE: flip the flag and let the stream carry axis_state=8.

        Every convert() payload already includes axis_state=8, so the very first
        forwarded packet arms the ODrives.  Sending a separate axis_state frame
        here would only add latency before the first motion command.
        """
        with self._lock:
            if self._active:
                return
            self._active = True
        log.info("ODrives → ClosedLoopControl (via stream)")

    def _set_idle(self, reason: str = "") -> None:
        """ACTIVE → IDLE: zero setpoints then put drives in Idle state."""
        with self._lock:
            if not self._active:
                return
            self._active = False

        label = f" ({reason})" if reason else ""
        log.info("ODrives → Idle (axis_state=%d)%s", _AXIS_IDLE, label)
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
        import os
        import sys
        _pkg_dir = os.path.dirname(os.path.abspath(__file__))
        if _pkg_dir not in sys.path:
            sys.path.insert(0, _pkg_dir)

        try:
            from .proto.core import RoveControl_pb2
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
            self._cfg.listen.host,
            self._cfg.listen.port,
        )

        _last_snap: tuple | None = None
        _last_log_t = 0.0
        _LOG_HEARTBEAT = 2.0
        _rx_count = 0
        _warn_t = time.monotonic()
        _window_start = time.monotonic()
        _window_count = 0

        while self._running:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                now = time.monotonic()
                if not self._active and (now - _warn_t) >= 5.0:
                    log.warning(
                        "No RoveControl packets in %.0fs — "
                        "is the steamdeck sending to %s:%d?",
                        now - self._last_rx_t,
                        self._cfg.listen.host,
                        self._cfg.listen.port,
                    )
                    _warn_t = now
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

            _rx_count += 1
            _warn_t = time.monotonic()

            # Transition IDLE → ACTIVE on first packet after silence.
            if not self._active:
                self._last_rx_t = time.monotonic()
                self._set_active()
            else:
                self._last_rx_t = time.monotonic()

            cmds = self._strategy.convert(msg)
            with self._lock:
                self._last_commands = cmds
            for cmd in cmds:
                self._dispatch(cmd)

            self._send_gripper(msg.gripper.position)

            # Forward Ovis twist to the IK engine (input half only -- the
            # bundled engine UI shows the resulting chain motion). The
            # output half (engine StateUpdate -> ODrive NodeCommands) is
            # deferred; until then the arm is driven visually, not mechanically.
            if self._ovis_forwarder is not None:
                self._ovis_forwarder.forward(msg.ovis)

            _window_count += 1

            # Logging: DEBUG for every packet, INFO only at the 2 s heartbeat.
            # Never log at INFO on every packet — at 100 Hz Python's logging
            # is slow enough to cause 100–500 ms of receive-loop blocking,
            # which triggers the idle watchdog during normal driving.
            t = msg.tracks
            f = msg.flippers

            log.debug(
                "[#%05d] rx L=%+.3f R=%+.3f  fl/fr/rl/rr=%+d/%+d/%+d/%+d",
                _rx_count, t.left_vel, t.right_vel,
                f.fl, f.fr, f.rl, f.rr,
            )

            now = time.monotonic()
            if (now - _last_log_t) >= _LOG_HEARTBEAT:
                elapsed = now - _window_start
                hz = _window_count / elapsed if elapsed > 0 else 0.0
                cmd_summary = "  ".join(
                    f"node{c.node_id}={list(c.payload.values())[0]:+.2f}"
                    for c in cmds
                ) if cmds else "—"
                log.info(
                    "[#%05d %5.1fHz] rx L=%+.3f R=%+.3f  fl/fr/rl/rr=%+d/%+d/%+d/%+d"
                    "  → %s",
                    _rx_count, hz,
                    t.left_vel, t.right_vel,
                    f.fl, f.fr, f.rl, f.rr,
                    cmd_summary,
                )
                _last_snap = (round(t.left_vel, 3), round(t.right_vel, 3),
                              f.fl, f.fr, f.rl, f.rr)
                _last_log_t = now
                _window_start = now
                _window_count = 0

        sock.close()

    def _idle_watchdog_loop(self) -> None:
        """Transition to IDLE when no packet has arrived for idle_timeout_s.

        Sends zero-velocity before axis_state=1 so the robot decelerates cleanly
        rather than being cut mid-motion.
        """
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
        """Send a gripper position command when the value has changed."""
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
        log.debug("→ gripper :%d  pos=%d  [%s]", self._gripper_port, position, "ok" if ok else "DROP")
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
        log.debug(
            "→ node%d :%d  %s  [%s]",
            cmd.node_id, port, cmd.payload, "ok" if ok else "DROP",
        )


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def build_strategy(cfg: BridgeConfig) -> ConversionStrategy:
    from .strategies.tracks_torque import TracksTorqueStrategy
    from .strategies.tracks_velocity import TracksVelocityStrategy

    track_strat_name = cfg.tracks.strategy.lower()
    if track_strat_name == "velocity":
        tracks = TracksVelocityStrategy(cfg.tracks)
    elif track_strat_name == "torque":
        tracks = TracksTorqueStrategy(cfg.tracks)
    else:
        raise ValueError(
            f"Unknown tracks strategy {cfg.tracks.strategy!r}. "
            "Valid options: 'velocity', 'torque'."
        )

    if cfg.flippers.enabled:
        log.warning(
            "flippers.enabled=true but no FlipperStrategy implemented yet — ignored."
        )
    # Ovis is handled outside the strategy abstraction for now: it forwards
    # to the engine UDP and returns no NodeCommands until the StateUpdate
    # -> ODrive morph is implemented. See OvisForwarder in start().

    return tracks


def start(cfg: BridgeConfig) -> None:
    """Resolve ODrive ports, build strategy, and run the bridge (blocking)."""
    log.info(
        "Discovering ODrive ports via http://%s:%d/discover …",
        cfg.sensor_api.host,
        cfg.sensor_api.http_port,
    )
    port_map = discover_odrive_ports(
        cfg.sensor_api.host,
        cfg.sensor_api.http_port,
        cfg.discover_timeout_s,
    )

    gripper_port: int | None = None
    if cfg.gripper.enabled:
        gripper_port = discover_sensor_command_port(
            cfg.sensor_api.host,
            cfg.sensor_api.http_port,
            cfg.gripper.sensor_id,
        )
        if gripper_port is None:
            log.warning(
                "Gripper sensor %r not found in /discover — gripper commands disabled.",
                cfg.gripper.sensor_id,
            )

    strategy = build_strategy(cfg)

    ovis_forwarder: OvisForwarder | None = None
    if cfg.ovis.enabled:
        if not cfg.ovis.target_entity_id:
            log.warning(
                "[ovis].enabled=true but target_entity_id is empty — "
                "ovis forwarding disabled."
            )
        else:
            ovis_forwarder = OvisForwarder(
                cfg.ovis.engine_host,
                cfg.ovis.engine_port,
                cfg.ovis.target_entity_id,
            )

    log.info(
        "Bridge starting: tracks strategy=%s  nodes=%s  gripper=%s  ovis=%s  idle_timeout=%.1fs",
        strategy.name,
        sorted(port_map.keys()),
        f"port {gripper_port}" if gripper_port else "disabled",
        f"{cfg.ovis.engine_host}:{cfg.ovis.engine_port} target={cfg.ovis.target_entity_id}"
        if ovis_forwarder
        else "disabled",
        cfg.idle_timeout_s,
    )

    bridge = RoveControlBridge(
        cfg, strategy, port_map, gripper_port=gripper_port, ovis_forwarder=ovis_forwarder
    )

    import signal

    def _handle_stop(signum, frame):  # noqa: ARG001
        log.info("Signal %d received, stopping bridge …", signum)
        bridge.stop()

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    bridge.run()
