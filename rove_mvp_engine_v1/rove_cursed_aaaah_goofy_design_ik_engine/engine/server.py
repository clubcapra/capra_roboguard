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
from .state import EngineState
from .transports import HttpWsServer, StateBus, UdpInput, UdpOutput

_log = logging.getLogger("forgebot.engine")


async def run(config_path: Path) -> None:
    cfg = engine_config.load(config_path)
    project = load_robot(cfg)
    state = EngineState(project=project)
    ik_loop.initialise_joint_values(state)
    bus = StateBus()

    stopping = asyncio.Event()
    tasks: list[asyncio.Task] = []

    udp_in: UdpInput | None = None
    udp_out: UdpOutput | None = None
    http_server: HttpWsServer | None = None

    if cfg.input.udp_enabled:
        udp_in = UdpInput(state, cfg.input.udp_bind)
        await udp_in.start()
    if cfg.output.udp_enabled:
        udp_out = UdpOutput(bus, cfg.output.udp_target)
        await udp_out.start()

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

    tasks.append(asyncio.create_task(_tick_loop(cfg, state, bus, stopping)))

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


async def _tick_loop(
    cfg: engine_config.EngineConfig,
    state: EngineState,
    bus: StateBus,
    stopping: asyncio.Event,
) -> None:
    period = 1.0 / max(1e-3, cfg.ik.rate_hz)
    last = time.monotonic()
    while not stopping.is_set():
        now = time.monotonic()
        dt = now - last
        last = now
        try:
            update = ik_loop.tick(state, cfg.ik, dt)
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
