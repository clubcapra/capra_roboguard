"""Flipper ODrive mirror — read-only sync of the track flippers into the model.

Each flipper is an independent ODrive node (41=FL, 42=FR, 43=BL, 44=BR) served
by rove_sensor_api on its own data port. Unlike the kinova arm (one stream of
`joint_N_pos`), here we open ONE subscriber per node and read its `pos_estimate`
(motor revolutions). The model joint angle is

    q_model = sign * scale * pos        scale = 2*pi / gear_ratio

captured against an offset at Sync time so the model doesn't jump:

    offset = sign * scale * pos_at_sync - q_model_at_sync
    q_model = sign * scale * pos_now    - offset

This file mirrors the structure of `hardware.py` (same subscribe-push wire
format, reusing its `_encode`/`_decode`), kept separate because the data shape
and per-node fan-out differ. Output is intentionally absent: this is read-only,
the same posture as the kinova bridge before `vel_output_enabled`.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from .config import FlippersConfig
from .hardware import (
    _MSG_COMMAND,
    _MSG_DATA,
    _MSG_SUBSCRIBE,
    _MSG_UNSUBSCRIBE,
    _decode,
    _encode,
)
from .state import EngineState

_log = logging.getLogger(__name__)

_STALENESS_THRESHOLD_S = 2.0
_WATCHDOG_TICK_S = 1.0
# How stale a flipper frame can be before the mirror stops trusting it.
_FLIPPER_FRESH_S = 0.5


def resolve_flipper_joint(state: EngineState, name: str) -> str | None:
    """Resolve a flipper joint entity id from a config `joint` value.

    The flipper *links* carry the human name ("FlipperFL"); the revolute
    *joint* that moves them is the link's parent and has no name. So we accept
    either: a joint entity named `name`, or a link named `name` whose parent
    is a movable joint (the common case for the track flippers)."""
    target = name.strip().lower()
    if not target:
        return None
    scene = state.project.scene
    for eid, ent in scene.entities.items():
        if (ent.name or "").strip().lower() != target:
            continue
        # Direct hit: the named entity is itself a joint.
        if ent.get("joint") is not None:
            return eid
        # Named link -> its parent joint.
        parent_id = ent.parent
        if parent_id is None:
            continue
        parent = scene.entities.get(parent_id)
        if parent is not None and parent.get("joint") is not None:
            return parent_id
    return None


def _scale_rad_per_rev(gear_ratio: float) -> float:
    """rad per motor-rev for a given gearbox ratio (motor revs per joint rev)."""
    if gear_ratio == 0.0:
        return 2.0 * math.pi
    return (2.0 * math.pi) / gear_ratio


class _FlipperNodeProtocol(asyncio.DatagramProtocol):
    """Decodes one flipper ODrive's DATA frames -> latest_flipper_positions."""

    def __init__(self, state: EngineState, node_id: int, on_error) -> None:
        self.state = state
        self.node_id = node_id
        self.on_error = on_error
        self._last_warn_t = 0.0

    def datagram_received(self, data: bytes, addr) -> None:
        try:
            mt, _, body = _decode(data)
        except Exception as exc:  # noqa: BLE001
            now = time.monotonic()
            if now - self._last_warn_t > 30.0:
                _log.warning("flipper %d frame decode failed: %s", self.node_id, exc)
                self._last_warn_t = now
            return
        if mt != _MSG_DATA or not isinstance(body, dict):
            return
        pos = body.get("pos_estimate")
        if not isinstance(pos, (int, float)):
            return
        self.state.latest_flipper_positions[self.node_id] = float(pos)
        self.state.latest_flipper_t[self.node_id] = time.monotonic()

    def error_received(self, exc: Exception) -> None:
        _log.warning("flipper %d UDP transport error: %s", self.node_id, exc)
        self.on_error()

    def connection_lost(self, exc: Exception | None) -> None:
        if exc is not None:
            _log.warning("flipper %d UDP transport closed: %s", self.node_id, exc)
        self.on_error()


class _FlipperNodeListener:
    """SUBSCRIBE-and-keep-alive for one flipper ODrive node (see KinovaStateListener)."""

    def __init__(self, state: EngineState, host: str, node_id: int,
                 data_port: int, interval_ms: int) -> None:
        self.state = state
        self.host = host
        self.node_id = node_id
        self.data_port = data_port
        self.interval_ms = interval_ms
        self._transport: asyncio.DatagramTransport | None = None
        self._broken = False
        self._last_subscribe_t = 0.0

    async def _open(self) -> bool:
        loop = asyncio.get_running_loop()
        try:
            self._transport, _ = await loop.create_datagram_endpoint(
                lambda: _FlipperNodeProtocol(self.state, self.node_id, self._mark_broken),
                local_addr=("0.0.0.0", 0),
                remote_addr=(self.host, self.data_port),
            )
        except Exception as exc:  # noqa: BLE001
            _log.warning("flipper %d: open to %s:%d failed: %s",
                         self.node_id, self.host, self.data_port, exc)
            self._transport = None
            return False
        self._broken = False
        try:
            self._transport.sendto(
                _encode(_MSG_SUBSCRIBE, 0, {"interval_ms": self.interval_ms})
            )
            self._last_subscribe_t = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            _log.warning("flipper %d SUBSCRIBE failed: %s", self.node_id, exc)
            self._broken = True
            return False
        _log.info("flipper %d: SUBSCRIBE -> %s:%d", self.node_id, self.host, self.data_port)
        return True

    def _mark_broken(self) -> None:
        self._broken = True

    async def _close(self) -> None:
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:  # noqa: BLE001
                pass
            self._transport = None

    async def tick(self) -> None:
        """One watchdog pass: reopen if broken, re-subscribe if stale."""
        now = time.monotonic()
        if self._broken or self._transport is None:
            await self._close()
            await self._open()
            return
        last_t = self.state.latest_flipper_t.get(self.node_id, 0.0)
        age = now - last_t if last_t > 0.0 else float("inf")
        if age > _STALENESS_THRESHOLD_S and now - self._last_subscribe_t > 2.0:
            try:
                self._transport.sendto(
                    _encode(_MSG_SUBSCRIBE, 0, {"interval_ms": self.interval_ms})
                )
                self._last_subscribe_t = now
            except Exception:  # noqa: BLE001
                self._broken = True

    async def stop(self) -> None:
        if self._transport is not None:
            try:
                self._transport.sendto(_encode(_MSG_UNSUBSCRIBE, 0, None))
            except Exception:  # noqa: BLE001
                pass
        await self._close()


class FlipperBank:
    """Owns one listener per configured flipper node + a shared watchdog."""

    def __init__(self, state: EngineState, cfg: FlippersConfig) -> None:
        self.state = state
        self.cfg = cfg
        self._listeners: list[_FlipperNodeListener] = []
        self._watchdog: asyncio.Task | None = None
        self._stopping = False

    def resolve(self) -> list[str]:
        """Resolve joint names -> entity ids, populate per-joint sign/scale and
        the joint->node map. Returns the list of unresolved joint names."""
        errors: list[str] = []
        self.state.flipper_step_rate_rad_s = math.radians(self.cfg.step_rate_deg_s)
        self.state.flipper_gear_ratio = float(self.cfg.gear_ratio)
        for n in self.cfg.nodes:
            eid = resolve_flipper_joint(self.state, n.joint)
            if eid is None:
                errors.append(n.joint)
                continue
            ratio = n.gear_ratio if n.gear_ratio is not None else self.cfg.gear_ratio
            self.state.flipper_joint_to_node[eid] = n.node_id
            # A runtime-tuned sign (persisted) wins over the config default.
            self.state.flipper_signs[eid] = float(
                self.state.flipper_signs_persisted.get(eid, n.sign))
            self.state.flipper_scales[eid] = _scale_rad_per_rev(ratio)
            if n.min_deg is not None and n.max_deg is not None:
                self.state.flipper_limits[eid] = (math.radians(n.min_deg), math.radians(n.max_deg))
        return errors

    async def start(self) -> None:
        unresolved = self.resolve()
        for name in unresolved:
            _log.warning("flippers: no movable joint named %r in scene — skipped", name)
        for n in self.cfg.nodes:
            lis = _FlipperNodeListener(
                self.state, self.cfg.sensor_api_host, n.node_id,
                n.data_port, self.cfg.subscribe_interval_ms,
            )
            await lis._open()
            self._listeners.append(lis)
        self._watchdog = asyncio.create_task(self._watchdog_loop(), name="flipper-watchdog")
        _log.info("flipper bank up: %d node(s), gear_ratio=%.3f (host %s)",
                  len(self._listeners), self.cfg.gear_ratio, self.cfg.sensor_api_host)

    async def _watchdog_loop(self) -> None:
        while not self._stopping:
            try:
                await asyncio.sleep(_WATCHDOG_TICK_S)
                if self._stopping:
                    return
                for lis in self._listeners:
                    await lis.tick()
            except asyncio.CancelledError:
                return
            except Exception as exc:  # noqa: BLE001
                _log.exception("flipper watchdog raised: %s", exc)

    async def stop(self) -> None:
        self._stopping = True
        if self._watchdog is not None:
            self._watchdog.cancel()
            try:
                await self._watchdog
            except asyncio.CancelledError:
                pass
        for lis in self._listeners:
            await lis.stop()


# ---- sync + manual jog -----------------------------------------------------


def snap_model_to_flippers(state: EngineState) -> dict:
    """Capture the model<->flipper frame offset for every resolved flipper joint
    that currently has a fresh reading. The model pose is NOT overwritten — the
    mirror tracks the real flippers from here using sign/scale/offset.

    Returns a JSON-able summary (captured count, per-joint offsets in deg,
    and which nodes had no fresh frame)."""
    captured: dict[str, float] = {}
    missing: list[int] = []
    for eid, node_id in state.flipper_joint_to_node.items():
        pos = state.latest_flipper_positions.get(node_id)
        last_t = state.latest_flipper_t.get(node_id, 0.0)
        if pos is None or last_t == 0.0:
            missing.append(node_id)
            continue
        sign = state.flipper_signs.get(eid, 1.0)
        scale = state.flipper_scales.get(eid, 2.0 * math.pi)
        model_q = float(state.joint_values.get(eid, 0.0))
        offset = sign * scale * float(pos) - model_q
        state.flipper_offsets[eid] = offset
        captured[eid] = offset
    return {
        "ok": len(captured) > 0,
        "captured": len(captured),
        "missing_nodes": missing,
        "offsets": captured,
        "offsets_deg": {k: v * 180.0 / math.pi for k, v in captured.items()},
    }


def _reanchor_flippers(state: EngineState, now: float) -> None:
    """Re-derive the offset for flippers awaiting it, from the first fresh frame
    after boot. The ODrive encoder reset to ~0 at the (unchanged) physical angle
    θ, so: offset = sign*scale*pos_now - θ_persisted. This restores a power-
    cycled flipper to its true position without the operator re-syncing."""
    if not state.flipper_reanchor:
        return
    for eid in list(state.flipper_reanchor):
        node = state.flipper_joint_to_node.get(eid)
        if node is None:
            state.flipper_reanchor.discard(eid)
            continue
        pos = state.latest_flipper_positions.get(node)
        last_t = state.latest_flipper_t.get(node, 0.0)
        if pos is None or now - last_t > _FLIPPER_FRESH_S:
            continue  # wait for a real frame
        sign = state.flipper_signs.get(eid, 1.0)
        scale = state.flipper_scales.get(eid, 2.0 * math.pi)
        theta = state.flipper_phys_persisted.get(eid, 0.0)
        state.flipper_offsets[eid] = sign * scale * float(pos) - theta
        state.flipper_reanchor.discard(eid)
        _log.info("flipper %s re-anchored to persisted %.1f deg (encoder reset, pos=%.4f rev)",
                  node, math.degrees(theta), pos)


def apply_flipper_mirror(state: EngineState) -> None:
    """Per-tick: drive each synced flipper joint from its live ODrive reading.
    No-ops for joints with no offset (not synced) or a stale frame, so a manual
    jog set before Sync is preserved."""
    _reanchor_flippers(state, time.monotonic())
    if not state.flipper_offsets:
        return
    from .motion import motion_joint_set
    skip = motion_joint_set(state)
    now = time.monotonic()
    for eid, offset in state.flipper_offsets.items():
        if eid in skip:
            continue  # owned by a planned pose-to-pose move
        # NOTE: commanded flippers are NOT skipped any more. We drive the real
        # flipper in velocity toward flipper_targets (see _send_position), so the
        # model must keep mirroring the live encoder — that IS "the engine
        # following the flipper". flipper_targets is the goal, not the display.
        node_id = state.flipper_joint_to_node.get(eid)
        if node_id is None:
            continue
        pos = state.latest_flipper_positions.get(node_id)
        last_t = state.latest_flipper_t.get(node_id, 0.0)
        if pos is None or now - last_t > _FLIPPER_FRESH_S:
            continue
        sign = state.flipper_signs.get(eid, 1.0)
        scale = state.flipper_scales.get(eid, 2.0 * math.pi)
        state.joint_values[eid] = sign * scale * float(pos) - offset


# ---- commanding (normalised +1/0/-1 step -> incremented target) ------------


def set_flipper_step(state: EngineState, *, joint: str = "", node_id: int | None = None,
                     step: int = 0) -> dict:
    """Set the held step ({-1,0,+1}) for one flipper. A non-zero step ramps that
    flipper's target each tick (held until changed); 0 holds at the current
    target. Resolves the flipper by joint name OR ODrive node id. The model
    moves immediately; a real packet only leaves if output is enabled."""
    eid: str | None = None
    if joint:
        eid = resolve_flipper_joint(state, joint)
    elif node_id is not None:
        for j, n in state.flipper_joint_to_node.items():
            if n == node_id:
                eid = j
                break
    if eid is None:
        return {"ok": False, "error": "specify a known 'joint' name or 'node' id"}
    node = state.flipper_joint_to_node.get(eid)
    s = 1 if step > 0 else (-1 if step < 0 else 0)
    if node is not None:
        state.flipper_cmd_steps[node] = s
    # Enter commanded mode (seed the ramp target from the current model angle)
    # ONLY on a non-zero step, or if this flipper is already commanded. A step==0
    # on an un-commanded flipper is a NO-OP — it must NOT pull the flipper out of
    # read-only mirror mode. Otherwise a client that streams [fl,fr,rl,rr] every
    # frame (the control bridge forwarding teleop drive) seizes every flipper into
    # commanded mode the instant any drive packet arrives, dropping them to their
    # seeded target instead of tracking the real flipper.
    if s != 0 or eid in state.flipper_targets:
        state.flipper_targets.setdefault(eid, float(state.joint_values.get(eid, 0.0)))
    tgt = state.flipper_targets.get(eid, float(state.joint_values.get(eid, 0.0)))
    return {"ok": True, "joint": eid, "node": node, "step": s,
            "target_deg": round(tgt * 180.0 / math.pi, 1)}


def release_flipper(state: EngineState, *, joint: str = "", node_id: int | None = None) -> dict:
    """Drop a flipper's command (target + step), handing it back to the mirror."""
    eid: str | None = None
    if joint:
        eid = resolve_flipper_joint(state, joint)
    elif node_id is not None:
        eid = next((j for j, n in state.flipper_joint_to_node.items() if n == node_id), None)
    if eid is None:
        return {"ok": False, "error": "unknown flipper"}
    node = state.flipper_joint_to_node.get(eid)
    state.flipper_targets.pop(eid, None)
    if node is not None:
        state.flipper_cmd_steps.pop(node, None)
    return {"ok": True, "joint": eid, "released": True}


def set_flipper_sign(state: EngineState, *, joint: str = "", node_id: int | None = None,
                     sign: float | None = None) -> dict:
    """Flip (or set) a flipper's motor->model sign. For a mirrored flipper the
    whole reading is inverted (axis points the other way), so flipping NEGATES
    the current model angle AND reverses future motion — i.e. -0.92 -> +0.92.
    `sign=None` toggles.

    Maths: model_q = sign*scale*pos - offset. Flipping sign and negating the
    offset gives -(sign*scale*pos - offset) = -model_q, so the displayed value
    mirrors and future motion direction flips. The caller persists the sign
    (state.flipper_signs_persisted) so it survives restart over the config sign."""
    eid: str | None = None
    if joint:
        eid = resolve_flipper_joint(state, joint)
    elif node_id is not None:
        eid = next((j for j, n in state.flipper_joint_to_node.items() if n == node_id), None)
    if eid is None:
        return {"ok": False, "error": "unknown flipper"}
    cur = state.flipper_signs.get(eid, 1.0)
    new = (sign if sign is not None else -cur)
    new = 1.0 if new >= 0 else -1.0
    flipped = abs(new + cur) < 1e-9  # new == -cur
    state.flipper_signs[eid] = new
    state.flipper_signs_persisted[eid] = new
    negated = False
    if flipped and eid in state.flipper_offsets:
        # Negate the offset so the synced reading mirrors (and direction flips).
        state.flipper_offsets[eid] = -state.flipper_offsets[eid]
        negated = True
    node = state.flipper_joint_to_node.get(eid)
    return {"ok": True, "joint": eid, "node": node, "sign": new, "negated": negated}


def apply_flipper_command(state: EngineState, dt: float) -> None:
    """Per-tick: ramp each commanded flipper's TARGET by its held step. The target
    (model rad, clamped to the joint limits) is the goal position the velocity
    servo in `_send_position` drives the real flipper to. We do NOT write the
    model here — the mirror keeps the model on the live encoder so it tracks the
    real flipper. Pure target update — no packets."""
    if not state.flipper_targets:
        return
    rate = state.flipper_step_rate_rad_s
    for eid, target in list(state.flipper_targets.items()):
        node = state.flipper_joint_to_node.get(eid)
        step = state.flipper_cmd_steps.get(node, 0) if node is not None else 0
        if step != 0:
            target = target + step * rate * dt
            lim = state.flipper_limits.get(eid)
            if lim is not None:
                target = max(lim[0], min(lim[1], target))
            state.flipper_targets[eid] = target


def apply_drive_watchdog(state: EngineState, timeout_s: float) -> None:
    """Self-stop the drives on input silence — clone of the old
    rove_control_bridge_py idle watchdog, run per tick.

    `drive_vel_cmd` (drums) and `flipper_cmd_steps` (the held flipper ramp step)
    PERSIST — unlike consume-once Ovis (which is why the arm halts on its own).
    So if drive frames stop arriving (operator releases, or the low-bandwidth
    link drops out) the drums keep their last velocity and the flippers keep
    ramping the last step forever. When no drive frame has arrived within
    `timeout_s`, zero the drum velocities (they brake to a stop on the next send)
    and halt the flipper ramps. The flipper HOLDS its current target (not
    released), so it stays put rather than going limp under gravity."""
    if timeout_s <= 0.0:
        return
    if time.monotonic() - state.latest_drive_t <= timeout_s:
        return
    for n in state.drive_vel_cmd:
        state.drive_vel_cmd[n] = 0.0
    for n in state.flipper_cmd_steps:
        state.flipper_cmd_steps[n] = 0


_CONTROL_MODE_VELOCITY = 2
_CONTROL_MODE_POSITION = 3
_INPUT_MODE_PASSTHROUGH = 1
_AXIS_CLOSED_LOOP = 8
_AXIS_IDLE = 1

# Flipper position->velocity servo (motor-rev space). The engine commands a
# velocity proportional to the target-vs-encoder error, saturated at the node's
# max_vel_rev_s. Working in motor revs (not model rad) makes convergence
# sign-independent (commanding +vel always increases the encoder). Tunables:
_FLIP_SERVO_KP = 15.0          # rev/s of command per rev of error (saturates fast)
_FLIP_SERVO_DEADBAND_REV = 0.2  # |error| below this -> command 0 (hold, no hunting)


class FlipperCommandSender:
    """GATED ODrive command sender for output-enabled drives, following the
    protocol in tools/odrive_test.py:

      * ARM ONCE per node — send {axis_state:8, control_mode, input_mode:1} a
        single time (re-sending it every tick resets the controller -> shake).
      * POSITION mode (flippers): seed input_pos from the live pos_estimate so
        the axis doesn't lurch, then stream input_pos ONLY when it changes
        (re-sending the same setpoint piles redundant trajectory entries).
      * VELOCITY mode (drums): stream input_vel while non-zero; send a single
        0 to stop. Continuous wheels are velocity-commanded.

    SAFETY: only sends when output_enabled is True AND the node has output=true.
    Senders idle (axis_state 1) any node that stops being commanded."""

    def __init__(self, state: EngineState, cfg: FlippersConfig) -> None:
        self.state = state
        self.cfg = cfg
        # eid -> {cmd_port, node_id, mode, max_vel, sign, scale}
        self._out: dict[str, dict] = {}
        self._transport: asyncio.DatagramTransport | None = None
        self._seq = 0
        self._last_warn = 0.0
        self._armed: dict[str, int] = {}      # eid -> control_mode it was armed in
        self._last_pos: dict[str, float] = {}  # eid -> last input_pos sent
        self._vel_zeroed: dict[str, bool] = {}  # eid -> already sent the stopping 0

    def resolve_outputs(self) -> None:
        for n in self.cfg.nodes:
            if not n.output or not n.cmd_port:
                continue
            eid = resolve_flipper_joint(self.state, n.joint)
            if eid is not None:
                self._out[eid] = {
                    "cmd_port": n.cmd_port, "node_id": n.node_id,
                    "mode": n.mode, "max_vel": n.max_vel_rev_s,
                }

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        try:
            self._transport, _ = await loop.create_datagram_endpoint(
                lambda: asyncio.DatagramProtocol(), local_addr=("0.0.0.0", 0))
        except Exception as exc:  # noqa: BLE001
            _log.warning("drive command sender: socket open failed: %s", exc)
            self._transport = None
        modes = {eid: o["mode"] for eid, o in self._out.items()}
        _log.warning("DRIVE OUTPUT ENABLED: commands will be sent to %d node(s) %s. "
                     "Verify direction/scale before trusting this.", len(self._out), modes)

    def _send(self, payload: dict, cmd_port: int) -> None:
        self._seq = (self._seq + 1) & 0xFFFF
        try:
            self._transport.sendto(_encode(_MSG_COMMAND, self._seq, payload),
                                   (self.cfg.sensor_api_host, cmd_port))
        except Exception:  # noqa: BLE001
            pass

    def maybe_send(self) -> None:
        if not self.cfg.output_enabled or self._transport is None or not self._out:
            return
        from .motion import motion_joint_set
        owned = motion_joint_set(self.state)
        for eid, o in self._out.items():
            if o["mode"] == "velocity":
                self._send_velocity(eid, o)
            else:
                self._send_position(eid, o, owned)

    def _idle(self, eid: str, cmd_port: int) -> None:
        if eid in self._armed:
            self._send({"axis_state": _AXIS_IDLE}, cmd_port)
            self._armed.pop(eid, None)
            self._last_pos.pop(eid, None)

    def _send_position(self, eid: str, o: dict, owned) -> None:
        """Velocity servo to the flipper's target — drives the ODrive in VELOCITY
        mode and NEVER sends input_pos. The IK side increments flipper_targets[eid]
        (model rad) from the held step; here we plan a velocity proportional to the
        target-vs-encoder error (saturated at the node's max_vel) and command
        input_vel every tick (keepalive). The model mirrors the live encoder, so it
        follows the real flipper; the servo holds actively against drift."""
        cmd_port, node_id = o["cmd_port"], o["node_id"]
        target_model = self.state.flipper_targets.get(eid)
        if target_model is None and eid in owned:
            # A planned pose-to-pose move owns the joint: servo to its trajectory.
            target_model = float(self.state.joint_values.get(eid, 0.0))
        if target_model is None:
            self._idle(eid, cmd_port)        # not commanded -> release (worm gear holds)
            return
        offset = self.state.flipper_offsets.get(eid)
        if offset is None:
            now = time.monotonic()
            if now - self._last_warn > 5.0:
                _log.warning("flipper %s commanded but not synced — not sending", node_id)
                self._last_warn = now
            return
        cur_rev = self.state.latest_flipper_positions.get(node_id)
        if cur_rev is None:
            return  # no encoder reading yet -> nothing to servo against
        sign = self.state.flipper_signs.get(eid, 1.0)
        scale = self.state.flipper_scales.get(eid, 2.0 * math.pi)
        # Goal in MOTOR revs (inverse of the mirror map), then error vs the encoder.
        # Working in motor revs makes it sign-independent: +input_vel always raises
        # the encoder, so the loop converges regardless of the model sign.
        goal_rev = (float(target_model) + offset) / (sign * scale)
        err = goal_rev - float(cur_rev)
        max_vel = float(o["max_vel"])
        if abs(err) < _FLIP_SERVO_DEADBAND_REV:
            vel = 0.0
        else:
            vel = max(-max_vel, min(max_vel, _FLIP_SERVO_KP * err))
        # Arm VELOCITY mode once, then re-assert closed-loop + setpoint every tick
        # (keepalive — re-arms a faulted ODrive, same as the drums).
        if self._armed.get(eid) != _CONTROL_MODE_VELOCITY:
            self._send({"axis_state": _AXIS_CLOSED_LOOP, "control_mode": _CONTROL_MODE_VELOCITY,
                        "input_mode": _INPUT_MODE_PASSTHROUGH}, cmd_port)
            self._armed[eid] = _CONTROL_MODE_VELOCITY
        self._send({"axis_state": _AXIS_CLOSED_LOOP, "input_vel": float(vel)}, cmd_port)

    def _send_velocity(self, eid: str, o: dict) -> None:
        cmd_port, node_id = o["cmd_port"], o["node_id"]
        cmd = self.state.drive_vel_cmd.get(node_id, 0.0)
        sign = self.state.flipper_signs.get(eid, 1.0)
        vel = sign * max(-1.0, min(1.0, cmd)) * o["max_vel"]
        if abs(vel) > 1e-6:
            if self._armed.get(eid) != _CONTROL_MODE_VELOCITY:
                self._send({"axis_state": _AXIS_CLOSED_LOOP, "control_mode": _CONTROL_MODE_VELOCITY,
                            "input_mode": _INPUT_MODE_PASSTHROUGH}, cmd_port)
                self._armed[eid] = _CONTROL_MODE_VELOCITY
            # Re-assert ClosedLoopControl on EVERY tick alongside the setpoint
            # (clone of the old rove_control_bridge_py keepalive). If the ODrive
            # faults / resets / hits its own watchdog mid-drive, this re-arms it
            # next tick instead of leaving it stuck — which otherwise shows up as
            # the drum juddering/vibrating.
            self._send({"axis_state": _AXIS_CLOSED_LOOP, "input_vel": float(vel)}, cmd_port)
            self._vel_zeroed[eid] = False
        elif eid in self._armed and not self._vel_zeroed.get(eid, False):
            self._send({"input_vel": 0.0}, cmd_port)   # one stop, then idle
            self._vel_zeroed[eid] = True

    async def stop(self) -> None:
        # Release every axis we armed before closing the socket.
        for eid, o in list(self._out.items()):
            self._idle(eid, o["cmd_port"])
        if self._transport is not None:
            self._transport.close()


def jog_flipper(state: EngineState, *, joint: str = "", node_id: int | None = None,
                angle_deg: float | None = None, delta_deg: float | None = None) -> dict:
    """Manually rotate a flipper joint in the model. Resolves the target joint
    by name OR by ODrive node id, then sets it absolutely (`angle_deg`) or
    relatively (`delta_deg`). Used to align the model flipper to the real one
    BEFORE Sync (mirror no-ops with no offset, so the jog sticks).

    Note: once that joint is synced, the per-tick mirror overwrites a jog — jog
    is a pre-sync alignment aid (and a way to pose unsynced/fried flippers)."""
    eid: str | None = None
    if joint:
        eid = resolve_flipper_joint(state, joint)
        if eid is None:
            return {"ok": False, "error": f"no flipper joint for {joint!r}"}
    elif node_id is not None:
        for jid, nid in state.flipper_joint_to_node.items():
            if nid == node_id:
                eid = jid
                break
        if eid is None:
            return {"ok": False, "error": f"no flipper joint mapped to node {node_id}"}
    else:
        return {"ok": False, "error": "specify 'joint' name or 'node' id"}

    if angle_deg is not None:
        new_q = float(angle_deg) * math.pi / 180.0
    elif delta_deg is not None:
        new_q = state.joint_values.get(eid, 0.0) + float(delta_deg) * math.pi / 180.0
    else:
        return {"ok": False, "error": "specify 'angle_deg' (absolute) or 'delta_deg' (relative)"}

    state.joint_values[eid] = new_q
    synced = eid in state.flipper_offsets
    return {
        "ok": True,
        "joint": eid,
        "angle_deg": new_q * 180.0 / math.pi,
        "synced": synced,
        "note": "joint is synced — mirror will overwrite this next tick" if synced else "",
    }
