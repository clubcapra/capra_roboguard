"""Startup orchestration: discover ports, build the strategy, run the bridge.

``start(cfg)`` is the public entry point invoked by ``__main__``. It does
everything between "I have a BridgeConfig" and "the receive loop is running":

    1. /discover ODrive command ports from rove_sensor_api
    2. /discover the gripper command port (if enabled)
    3. Pick a tracks strategy based on cfg.tracks.strategy
    4. Open the OvisForwarder if cfg.ovis.enabled
    5. Wire SIGINT/SIGTERM to a clean stop
    6. Call RoveControlBridge.run()
"""
from __future__ import annotations

import logging
import signal

from ..config import BridgeConfig
from ..strategies.base import ConversionStrategy
from ..transport import (
    OvisForwarder,
    discover_odrive_ports,
    discover_sensor_command_port,
)
from .core import RoveControlBridge

log = logging.getLogger(__name__)


def build_strategy(cfg: BridgeConfig) -> ConversionStrategy:
    """Pick the tracks ConversionStrategy implementation from config."""
    # Local imports keep the strategy modules out of the import chain when
    # callers only need the config / transport pieces.
    from ..strategies.tracks_mixed import TracksMixedStrategy
    from ..strategies.tracks_torque import TracksTorqueStrategy
    from ..strategies.tracks_velocity import TracksVelocityStrategy

    name = cfg.tracks.strategy.lower()
    if name == "velocity":
        tracks = TracksVelocityStrategy(cfg.tracks)
    elif name == "torque":
        tracks = TracksTorqueStrategy(cfg.tracks)
    elif name == "mixed":
        tracks = TracksMixedStrategy(cfg.tracks)
    else:
        raise ValueError(
            f"Unknown tracks strategy {cfg.tracks.strategy!r}. "
            "Valid options: 'velocity', 'torque', 'mixed'."
        )

    if cfg.flippers.enabled:
        log.warning(
            "flippers.enabled=true but no FlipperStrategy implemented yet — ignored."
        )
    # Ovis is handled outside the strategy abstraction for now: it forwards
    # to rove_ik_engine and emits no NodeCommands. When the engine becomes
    # the core conversion layer, tracks + flippers will join arm there too.

    return tracks


def start(cfg: BridgeConfig) -> None:
    """Resolve ports, build the strategy, and run the bridge (blocking)."""
    log.info(
        "Discovering ODrive ports via http://%s:%d/discover …",
        cfg.sensor_api.host, cfg.sensor_api.http_port,
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
                "ovis.enabled=true but target_entity_id is empty — "
                "ovis forwarding disabled."
            )
        else:
            ovis_forwarder = OvisForwarder(
                cfg.ovis.engine_host,
                cfg.ovis.engine_port,
                cfg.ovis.target_entity_id,
            )

    log.info(
        "Bridge starting: tracks=%s  nodes=%s  gripper=%s  ovis=%s  idle_timeout=%.1fs",
        strategy.name,
        sorted(port_map.keys()),
        f"port {gripper_port}" if gripper_port else "disabled",
        f"{cfg.ovis.engine_host}:{cfg.ovis.engine_port} target={cfg.ovis.target_entity_id}"
        if ovis_forwarder else "disabled",
        cfg.idle_timeout_s,
    )

    bridge = RoveControlBridge(
        cfg, strategy, port_map,
        gripper_port=gripper_port,
        ovis_forwarder=ovis_forwarder,
    )

    def _handle_stop(signum, _frame):
        log.info("Signal %d received, stopping bridge …", signum)
        bridge.stop()

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    bridge.run()
