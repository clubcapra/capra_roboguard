"""Engine orchestrator: load robot, wire transports, drive the IK loop."""

from __future__ import annotations

import asyncio
import logging
import signal
import sys
import time
from pathlib import Path

from . import config as engine_config
from . import ik_loop
from .loader import load_robot
from .hardware import KinovaCommandSender, KinovaStateListener
from .state import EngineState
from .tcp import compute_tcp_offsets
from .transports import HttpWsServer, StateBus, UdpInput, UdpOutput

_log = logging.getLogger("forgebot.engine")


async def run(config_path: Path) -> None:
    cfg = engine_config.load(config_path)
    project = load_robot(cfg)
    state = EngineState(project=project)
    ik_loop.initialise_joint_values(state)
    state.tcp_offsets = compute_tcp_offsets(project)
    if state.tcp_offsets:
        _log.info(
            "computed TCP offsets for %d links (centroid pivot for clients "
            "without their own tcp_offset_local)",
            len(state.tcp_offsets),
        )
    bus = StateBus()

    stopping = asyncio.Event()
    tasks: list[asyncio.Task] = []

    udp_in: UdpInput | None = None
    udp_out: UdpOutput | None = None
    http_server: HttpWsServer | None = None
    kinova_listener: KinovaStateListener | None = None

    if cfg.input.udp_enabled:
        udp_in = UdpInput(state, cfg.input.udp_bind)
        await udp_in.start()
    if cfg.output.udp_enabled:
        udp_out = UdpOutput(bus, cfg.output.udp_target)
        await udp_out.start()

    if cfg.hardware.enabled:
        # Best-effort joint-count guess from whichever mapping is set, so
        # the listener knows how many joint_N_pos fields to extract.
        if cfg.hardware.arm_base_entity_id and cfg.hardware.arm_tip_entity_id:
            try:
                from forgebot.core.kinematics import extract_chain
                _chain = extract_chain(
                    project,
                    cfg.hardware.arm_base_entity_id,
                    cfg.hardware.arm_tip_entity_id,
                )
                expected_n = len(_chain.joints)
            except Exception:  # noqa: BLE001
                expected_n = 0
        else:
            expected_n = len(cfg.hardware.joint_names)

        kinova_listener = KinovaStateListener(
            state,
            host=cfg.hardware.sensor_api_host,
            data_port=cfg.hardware.kinova_data_port,
            subscribe_interval_ms=cfg.hardware.subscribe_interval_ms,
            expected_joint_count=expected_n,
        )
        await kinova_listener.start()

    kinova_sender: KinovaCommandSender | None = None
    if cfg.hardware.enabled and cfg.hardware.vel_output_enabled:
        kinova_sender = KinovaCommandSender(
            state,
            host=cfg.hardware.sensor_api_host,
            cmd_port=cfg.hardware.kinova_cmd_port,
            max_vel_deg_s=cfg.hardware.max_kinova_vel_deg_s,
            min_vel_deg_s=cfg.hardware.min_vel_deg_s,
        )
        await kinova_sender.start()
        _log.warning(
            "VEL OUTPUT ENABLED: every tick the engine will push joint "
            "velocities to %s:%d. Sync must be done first; verify mirror "
            "direction before allowing motion.",
            cfg.hardware.sensor_api_host,
            cfg.hardware.kinova_cmd_port,
        )

    # The HTTP server runs whenever WS in/out is on OR a UI dist is bundled —
    # the UI needs scene + mesh HTTP routes even if it only uses WS for telemetry.
    ui_dir = config_path.parent / "ui"
    data_dir = config_path.parent / "data"
    if cfg.input.ws_enabled or cfg.output.ws_enabled or ui_dir.exists():
        http_server = HttpWsServer(
            state,
            bus,
            cfg.input.ws_bind,
            input_enabled=cfg.input.ws_enabled,
            input_path=cfg.input.ws_path,
            output_enabled=cfg.output.ws_enabled,
            output_path=cfg.output.ws_path,
            ui_dir=ui_dir,
            data_dir=data_dir,
            hardware=cfg.hardware,
        )
        await http_server.start()

    if cfg.output.stdout_enabled:
        bus.subscribe(_stdout_sink())

    _log.info(
        "engine up: %d joints, collision_aware=%s, twist_frame=%s, rate=%.1fHz",
        len(state.joint_values),
        cfg.ik.collision_aware,
        cfg.ik.twist_frame,
        cfg.ik.rate_hz,
    )
    _log_scene(state)

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stopping.set)
        except NotImplementedError:
            # Windows: signal handlers via asyncio aren't supported. Caller
            # can KeyboardInterrupt instead.
            pass

    tasks.append(
        asyncio.create_task(_tick_loop(cfg, state, bus, stopping, kinova_sender))
    )

    await stopping.wait()
    _log.info("shutting down")
    for t in tasks:
        t.cancel()
    for t in tasks:
        try:
            await t
        except asyncio.CancelledError:
            pass
    if http_server is not None:
        await http_server.stop()
    if udp_in is not None:
        await udp_in.stop()
    if udp_out is not None:
        await udp_out.stop()
    if kinova_listener is not None:
        await kinova_listener.stop()
    if kinova_sender is not None:
        await kinova_sender.stop()


async def _tick_loop(
    cfg: engine_config.EngineConfig,
    state: EngineState,
    bus: StateBus,
    stopping: asyncio.Event,
    kinova_sender: KinovaCommandSender | None = None,
) -> None:
    period = 1.0 / max(1e-3, cfg.ik.rate_hz)
    last = time.monotonic()
    while not stopping.is_set():
        now = time.monotonic()
        dt = now - last
        last = now
        try:
            update = ik_loop.tick(state, cfg.ik, dt)
            # Close the loop: push IK velocities to kinova_arm before
            # broadcasting telemetry. Kinova moves -> mirror updates next
            # tick -> state.joint_values reflects the real arm.
            if kinova_sender is not None:
                kinova_sender.maybe_send()
            await bus.publish(update.SerializeToString())
        except Exception as e:  # noqa: BLE001
            _log.exception("tick failed: %s", e)
        # Sleep for the remainder of the period — accounting for time taken.
        elapsed = time.monotonic() - now
        await asyncio.sleep(max(0.0, period - elapsed))


def _log_scene(state: EngineState) -> None:
    """Print every link and movable joint with its id + name, so operators
    can pick targets to drive without grepping the .forgebot file."""
    scene = state.project.scene
    _log.info("scene contents — send Ovis.target = any of these entity ids:")
    for eid, ent in scene.entities.items():
        link = ent.get("link")
        joint = ent.get("joint")
        if link is not None:
            _log.info("  link  %s  %s", eid, ent.name or "")
        elif joint is not None and joint.type != "fixed":
            _log.info(
                "  joint %s  %s  (%s, axis=%s)",
                eid,
                ent.name or "",
                joint.type,
                joint.axis,
            )


def _stdout_sink():
    # Length-prefixed frames: 4-byte big-endian length, then payload.
    async def _send(frame: bytes) -> None:
        sys.stdout.buffer.write(len(frame).to_bytes(4, "big"))
        sys.stdout.buffer.write(frame)
        sys.stdout.buffer.flush()
    return _send
