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
from .flippers import FlipperBank, FlipperCommandSender, apply_drive_watchdog
from .calibration import OFFSETS_FILENAME, load_offsets, save_offsets
from .state import EngineState
from .tcp import compute_tcp_offsets
from .transports import DriveUdpInput, HttpWsServer, StateBus, UdpInput, UdpOutput

_log = logging.getLogger("forgebot.engine")


def _apply_drive_ports(cfg: engine_config.EngineConfig, ports: dict) -> None:
    """Apply discovered ports to the drive nodes + kinova. A node ABSENT from
    /discover is DISABLED (ports zeroed) — never read, never commanded. We never
    fall back to a configured port: a wrong port could drive the wrong motor (a
    flipper's old fallback port was a drum's cmd port)."""
    for n in cfg.flippers.nodes:
        hit = ports.get(f"odrive_{n.node_id}")
        if hit:
            if (n.data_port, n.cmd_port) != hit:
                _log.info("drive %d ports -> %s (discover)", n.node_id, hit)
            n.data_port, n.cmd_port = hit
        else:
            if n.data_port or n.cmd_port:
                _log.warning("drive node %d absent from /discover — disabled "
                             "(won't read or command)", n.node_id)
            n.data_port = 0
            n.cmd_port = 0
    km = ports.get("kinova_arm")
    if km and cfg.hardware.enabled:
        cfg.hardware.kinova_data_port, cfg.hardware.kinova_cmd_port = km


def _post_sensor_reload(host: str, http_port: int) -> None:
    """POST /reload to rove_sensor_api to kick it when /discover stays down."""
    import urllib.request
    url = f"http://{host}:{http_port}/reload"
    try:
        req = urllib.request.Request(url, data=b"", method="POST")
        with urllib.request.urlopen(req, timeout=3.0) as r:  # noqa: S310 (trusted LAN)
            r.read()
        _log.info("POST %s -> sensor_api reload requested", url)
    except Exception as exc:  # noqa: BLE001
        _log.warning("sensor_api reload POST to %s failed: %s", url, exc)


async def _await_sensor_api(cfg: engine_config.EngineConfig) -> dict:
    """Block until rove_sensor_api's GET /discover responds, returning its port
    map. The drivetrain waits for autodiscover rather than ever using a fallback
    port. Retries every few seconds; if /discover is still down after ~60s, POSTs
    /reload to kick sensor_api (the documented recovery), then keeps trying.

    Returns immediately on the first success — so when sensor_api is already up
    (the normal case) there is no startup delay."""
    from . import discover as _discover
    loop = asyncio.get_running_loop()
    host, port = cfg.flippers.sensor_api_host, cfg.flippers.discover_http_port
    attempt = 0
    while True:
        ports = await loop.run_in_executor(None, _discover.fetch_ports, host, port)
        if ports:
            if attempt:
                _log.info("sensor_api /discover up (%d sensors) after %d attempt(s)",
                          len(ports), attempt)
            return ports
        attempt += 1
        # Every ~60s (20 * 3s) of silence, kick sensor_api with a /reload.
        if attempt % 20 == 0:
            _log.warning("sensor_api /discover still down after ~%ds — POSTing /reload",
                         attempt * 3)
            await loop.run_in_executor(None, _post_sensor_reload, host, port)
        elif attempt == 1 or attempt % 5 == 0:
            _log.warning("waiting for sensor_api /discover at %s:%d (attempt %d)…",
                         host, port, attempt)
        await asyncio.sleep(3.0)


def _offsets_filename(config_path: Path) -> str:
    """Per-config sync-offsets filename. `engine.standard.toml` keeps the legacy
    `sync_offsets.json` (the arm build's data); the caged default and any other
    config get a `sync_offsets.<name>.json` of their own."""
    name = config_path.name
    if name == "engine.standard.toml":
        return OFFSETS_FILENAME            # legacy arm offsets
    if name == "engine.toml":
        return "sync_offsets.caged.json"   # the default (caged) build
    return f"sync_offsets.{config_path.stem}.json"


async def run(config_path: Path) -> None:
    cfg = engine_config.load(config_path)
    project = load_robot(cfg)
    state = EngineState(project=project)
    # Seed live runtime flags from config so they can be flipped via HTTP
    # without restarting the engine.
    state.collision_aware = cfg.ik.collision_aware
    ik_loop.initialise_joint_values(state)
    # Restore sync offsets captured in a previous session so a synced robot
    # stays synced across restarts (mirrors resume once live frames arrive).
    # Offsets are keyed by entity id, which differs per robot model, so each
    # config gets its own file — a sync of one build must never clobber another's
    # (and the caged ids would just be dropped against the arm's anyway). The
    # legacy sync_offsets.json holds the arm/standard build's offsets.
    offsets_path = config_path.parent / _offsets_filename(config_path)
    load_offsets(offsets_path, state, drive_gear_ratio=cfg.flippers.gear_ratio)
    # Resolve served ports from the robot's GET /discover. There are NO fallback
    # ports: a stale/guessed port could map to a DIFFERENT motor, so the drivetrain
    # WAITS for /discover to come up (retrying, with a /reload kick after ~60s)
    # before commanding, and any node /discover doesn't list is left disabled.
    if cfg.flippers.enabled and cfg.flippers.discover:
        ports = await _await_sensor_api(cfg)
        _apply_drive_ports(cfg, ports)
    state.tcp_offsets = compute_tcp_offsets(project)
    _apply_tcp_extras(state, cfg.ik.tcp_offset_extra)
    if state.tcp_offsets:
        _log.info(
            "TCP offsets (per link, local frame metres) after centroid + "
            "config overrides:",
        )
        for eid, off in state.tcp_offsets.items():
            ent = project.scene.entities.get(eid)
            name = (ent.name if ent else "?") or "?"
            _log.info(
                "  %-25s %-30s (%+.4f, %+.4f, %+.4f)",
                name, eid, off[0], off[1], off[2],
            )

    # Warm the collision BVH cache now (synchronously, before any listener or the
    # tick loop runs) so the first pose-move collision check is fast (~ms) rather
    # than a multi-second BVH build mid-operation — and so it can't race the
    # per-waypoint checks that share the cache.
    if cfg.ik.collision_aware:
        try:
            from forgebot.core.validation.collision import check_collisions
            t_warm = time.monotonic()
            n_pairs = len(check_collisions(project, joint_values=dict(state.joint_values)))
            _log.info("collision BVH cache warmed in %.1fs (%d baseline pairs at home)",
                      time.monotonic() - t_warm, n_pairs)
        except Exception as exc:  # noqa: BLE001
            _log.warning("collision warm-up failed (checks will lazy-build on first use): %s", exc)

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
    drive_in: DriveUdpInput | None = None
    if cfg.input.drive_udp_enabled:
        drive_in = DriveUdpInput(state, cfg.input.drive_udp_bind)
        await drive_in.start()
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

    flipper_bank: FlipperBank | None = None
    if cfg.flippers.enabled and cfg.flippers.nodes:
        flipper_bank = FlipperBank(state, cfg.flippers)
        await flipper_bank.start()
        # Power-cycle recovery: the flipper ODrives boot with pos_estimate=0 at
        # the (unchanged, worm-gear-locked) physical angle, so the persisted
        # OFFSET is stale. Discard it, show the persisted physical angle now, and
        # queue a re-anchor that re-derives the offset from the first frame.
        if cfg.flippers.reanchor_on_boot and state.flipper_phys_persisted:
            for eid, ang in state.flipper_phys_persisted.items():
                if eid in state.flipper_joint_to_node:
                    state.joint_values[eid] = ang
                    state.flipper_offsets.pop(eid, None)
                    state.flipper_reanchor.add(eid)
            _log.info("flipper re-anchor queued for %d joint(s) from persisted positions",
                      len(state.flipper_reanchor))

    # GATED flipper/drum position output. Only created when the master gate is
    # on, so with the default config no command socket even opens.
    flipper_sender: FlipperCommandSender | None = None
    if cfg.flippers.enabled and cfg.flippers.output_enabled:
        flipper_sender = FlipperCommandSender(state, cfg.flippers)
        flipper_sender.resolve_outputs()
        await flipper_sender.start()

    kinova_sender: KinovaCommandSender | None = None
    if cfg.hardware.enabled and cfg.hardware.vel_output_enabled:
        kinova_sender = KinovaCommandSender(
            state,
            host=cfg.hardware.sensor_api_host,
            cmd_port=cfg.hardware.kinova_cmd_port,
            max_vel_deg_s=cfg.hardware.max_kinova_vel_deg_s,
            min_vel_deg_s=cfg.hardware.min_vel_deg_s,
            debug=cfg.ik.debug,   # reuse the [ik] debug flag
        )
        await kinova_sender.start()
        _log.warning(
            "VEL OUTPUT ENABLED: every tick the engine will push joint "
            "velocities to %s:%d. Sync must be done first; verify mirror "
            "direction before allowing motion.",
            cfg.hardware.sensor_api_host,
            cfg.hardware.kinova_cmd_port,
        )

    async def reheal() -> dict:
        """Recover from a rove_sensor_api reload / e-stop recovery WITHOUT
        restarting the engine — the fix for "uninstall + reinstall after e-stop".

        1. Re-discover served ports (boot order shuffles them) and re-point the
           live listeners + command senders, so reads/commands keep hitting the
           right sensor and never a stale COMMAND port (which could drive the
           wrong motor).
        2. Re-anchor every synced flipper to its live model angle. The ODrives
           lose their encoder zero on a power-cycle (pos_estimate -> 0) but the
           worm gear means the flipper hasn't physically moved — so we keep the
           live angle and re-derive the offset from the first fresh frame. No
           operator re-sync, and the model never lurches. Idempotent if the
           encoder did NOT reset (it re-derives the same offset)."""
        out: dict = {"rediscovered": False, "reanchored": 0}
        if cfg.flippers.enabled and cfg.flippers.discover:
            from . import discover as _discover
            ports = _discover.fetch_ports(
                cfg.flippers.sensor_api_host, cfg.flippers.discover_http_port
            )
            if ports:
                out["rediscovered"] = True
                # Same apply path as startup: discovered -> ports, absent -> disabled.
                _apply_drive_ports(cfg, ports)
                # Push the resolved ports into the live transports + reopen.
                if flipper_bank is not None:
                    flipper_bank.update_ports(cfg.flippers)
                if flipper_sender is not None:
                    flipper_sender.update_ports(cfg.flippers)
                if kinova_listener is not None:
                    kinova_listener.set_data_port(cfg.hardware.kinova_data_port)
                if kinova_sender is not None:
                    kinova_sender.set_cmd_port(cfg.hardware.kinova_cmd_port)
        # Re-anchor synced flippers to their live angle (handles the encoder reset).
        for eid in list(state.flipper_offsets.keys()):
            if eid in state.flipper_joint_to_node:
                state.flipper_phys_persisted[eid] = state.joint_values.get(eid, 0.0)
                state.flipper_offsets.pop(eid, None)
                state.flipper_reanchor.add(eid)
                out["reanchored"] += 1
        _log.info(
            "reheal: rediscovered=%s port_changes=%d reanchored=%d",
            out["rediscovered"], len(out["port_changes"]), out["reanchored"],
        )
        return out

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
            flippers=cfg.flippers,
            offsets_path=offsets_path,
            reheal=reheal,
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
        asyncio.create_task(_tick_loop(cfg, state, bus, stopping, kinova_sender, flipper_sender))
    )
    # Periodically persist physical flipper angles (they change when commanded)
    # so a power-cycle recovers the latest position, not just the last Sync.
    if cfg.flippers.enabled:
        tasks.append(asyncio.create_task(_persist_loop(state, offsets_path, stopping)))

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
    if drive_in is not None:
        await drive_in.stop()
    if udp_out is not None:
        await udp_out.stop()
    if kinova_listener is not None:
        await kinova_listener.stop()
    if kinova_sender is not None:
        await kinova_sender.stop()
    if flipper_bank is not None:
        await flipper_bank.stop()
    if flipper_sender is not None:
        await flipper_sender.stop()


async def _persist_loop(state: EngineState, offsets_path, stopping: asyncio.Event) -> None:
    """Save offsets + physical flipper angles every few seconds, but only when a
    synced flipper's angle has actually moved (avoids churning the file)."""
    last: dict[str, float] = {}
    while not stopping.is_set():
        try:
            await asyncio.sleep(5.0)
            if stopping.is_set():
                return
            cur = {eid: round(state.joint_values.get(eid, 0.0), 4) for eid in state.flipper_offsets}
            if cur and cur != last:
                save_offsets(offsets_path, state)
                last = cur
        except asyncio.CancelledError:
            return
        except Exception as exc:  # noqa: BLE001
            _log.warning("persist loop tick failed: %s", exc)


async def _tick_loop(
    cfg: engine_config.EngineConfig,
    state: EngineState,
    bus: StateBus,
    stopping: asyncio.Event,
    kinova_sender: KinovaCommandSender | None = None,
    flipper_sender: FlipperCommandSender | None = None,
) -> None:
    # If the solver throws this many ticks in a row, bail so systemd's
    # Restart=on-failure can give us a clean process. Swallowing forever
    # makes the service silently broken; exiting recovers.
    _CONSECUTIVE_FAIL_LIMIT = 30
    _FAIL_LOG_EVERY = 5   # spam guard while we're failing repeatedly

    period = 1.0 / max(1e-3, cfg.ik.rate_hz)
    last = time.monotonic()
    consecutive_fails = 0
    while not stopping.is_set():
        now = time.monotonic()
        dt = now - last
        last = now
        try:
            # Drive watchdog FIRST: zero drum velocity + halt flipper ramps if no
            # drive frame arrived within drive_idle_timeout_s. Runs before tick()
            # so the halted flipper steps take effect this tick's ramp.
            apply_drive_watchdog(state, cfg.flippers.drive_idle_timeout_s)
            update = ik_loop.tick(state, cfg.ik, dt)
            # Close the loop: push IK velocities to kinova_arm before
            # broadcasting telemetry. Kinova moves -> mirror updates next
            # tick -> state.joint_values reflects the real arm.
            if kinova_sender is not None:
                kinova_sender.maybe_send()
            if flipper_sender is not None:
                flipper_sender.maybe_send()
            await bus.publish(update.SerializeToString())
            consecutive_fails = 0
        except Exception as e:  # noqa: BLE001
            consecutive_fails += 1
            if consecutive_fails == 1 or consecutive_fails % _FAIL_LOG_EVERY == 0:
                _log.exception(
                    "tick failed (%d consecutive): %s", consecutive_fails, e,
                )
            if consecutive_fails >= _CONSECUTIVE_FAIL_LIMIT:
                _log.error(
                    "tick failed %d ticks in a row — exiting so systemd "
                    "Restart=on-failure can give us a fresh process",
                    consecutive_fails,
                )
                stopping.set()
                # Non-zero exit so the service restarts.
                sys.exit(1)
        # Sleep for the remainder of the period — accounting for time taken.
        elapsed = time.monotonic() - now
        await asyncio.sleep(max(0.0, period - elapsed))


def _apply_tcp_extras(
    state: EngineState, extras: dict[str, list[float]]
) -> None:
    """Merge user-specified TCP offset deltas (link's local frame, metres)
    on top of the auto-computed centroid offsets. Keys are entity ids OR
    link names (case-insensitive). Unknown keys are logged and skipped."""
    if not extras:
        return
    import numpy as _np

    scene = state.project.scene
    name_lookup = {
        (ent.name or "").strip().lower(): eid
        for eid, ent in scene.entities.items()
        if (ent.name or "").strip()
    }
    for key, vec in extras.items():
        if key in scene.entities:
            eid = key
        else:
            eid = name_lookup.get(key.strip().lower())
            if eid is None:
                _log.warning(
                    "tcp_offset_extra: no entity matches %r (tried id + name).",
                    key,
                )
                continue
        if not isinstance(vec, (list, tuple)) or len(vec) != 3:
            _log.warning(
                "tcp_offset_extra[%s]: expected 3-element list, got %r", key, vec
            )
            continue
        extra = _np.array([float(v) for v in vec], dtype=float)
        if eid in state.tcp_offsets:
            state.tcp_offsets[eid] = state.tcp_offsets[eid] + extra
        else:
            state.tcp_offsets[eid] = extra


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
