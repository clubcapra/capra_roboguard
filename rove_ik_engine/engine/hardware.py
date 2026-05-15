"""Kinova arm state subscriber.

rove_sensor_api uses a subscribe-push UDP protocol:

    4-byte header: version(0x01) | msg_type | seq (u16 LE)
    body:          JSON

Message types we use here:
    0x01 SUBSCRIBE   client -> sensor   (we send this once at startup)
    0x02 UNSUBSCRIBE client -> sensor   (sent on shutdown)
    0x03 DATA        sensor -> client   (the stream we read)

For the kinova_arm sensor the data port defaults to 5002 (see
rove_sensor_api/config/kinova.toml). Body shape on DATA frames:

    {
        "joint_1_pos": <degrees>,
        "joint_1_vel": <deg/s>,
        ...
        "joint_6_pos": <degrees>,
        "control_enabled": true,
        ...
    }

The engine works in radians, so positions are converted on arrival.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import struct
import time
from typing import Callable, Iterable

from .state import EngineState

_log = logging.getLogger(__name__)

_PROTOCOL_VERSION = 0x01
_MSG_SUBSCRIBE = 0x01
_MSG_UNSUBSCRIBE = 0x02
_MSG_DATA = 0x03
_HEADER_FMT = "<BBH"
_HEADER_SIZE = 4


def _encode(msg_type: int, seq: int, payload: dict | None) -> bytes:
    body = json.dumps(payload).encode() if payload is not None else b""
    return struct.pack(_HEADER_FMT, _PROTOCOL_VERSION, msg_type, seq & 0xFFFF) + body


def _decode(data: bytes) -> tuple[int, int, dict | None]:
    if len(data) < _HEADER_SIZE:
        raise ValueError("packet shorter than 4-byte header")
    ver, mt, seq = struct.unpack(_HEADER_FMT, data[:_HEADER_SIZE])
    if ver != _PROTOCOL_VERSION:
        raise ValueError(f"protocol version {ver}, expected {_PROTOCOL_VERSION}")
    body = data[_HEADER_SIZE:]
    return mt, seq, (json.loads(body) if body else None)


class _KinovaStateProtocol(asyncio.DatagramProtocol):
    def __init__(
        self,
        state: EngineState,
        expected_n: int,
        on_error: "Callable[[], None] | None" = None,
    ) -> None:
        self.state = state
        self.expected_n = expected_n
        self.on_error = on_error
        self._last_decode_warn_t = 0.0
        self._last_keys_warn_t = 0.0

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        try:
            mt, _, body = _decode(data)
        except Exception as exc:
            # Rate-limit malformed-frame warnings to once per 30s so a
            # rogue sender can't flood journald.
            now = time.monotonic()
            if now - self._last_decode_warn_t > 30.0:
                _log.warning("kinova frame decode failed from %s: %s", addr, exc)
                self._last_decode_warn_t = now
            return
        if mt != _MSG_DATA or not isinstance(body, dict):
            return
        positions_deg = _extract_positions(body, self.expected_n)
        if positions_deg is None:
            now = time.monotonic()
            if now - self._last_keys_warn_t > 30.0:
                _log.warning(
                    "kinova DATA frame had no joint_N_pos fields; keys=%s",
                    list(body.keys())[:8],
                )
                self._last_keys_warn_t = now
            return
        # Convert degrees -> radians: engine kinematics expects radians.
        self.state.latest_kinova_positions = [
            v * math.pi / 180.0 for v in positions_deg
        ]
        self.state.latest_kinova_t = time.monotonic()

    def error_received(self, exc: Exception) -> None:
        # Fires on ICMP "port unreachable" — typically rove_sensor_api died.
        # Signal the watchdog so it'll tear down + reconnect.
        _log.warning("kinova UDP transport error: %s", exc)
        if self.on_error is not None:
            self.on_error()

    def connection_lost(self, exc: Exception | None) -> None:
        if exc is not None:
            _log.warning("kinova UDP transport closed: %s", exc)
        if self.on_error is not None:
            self.on_error()


def _extract_positions(body: dict, expected_n: int) -> list[float] | None:
    """Pull `joint_1_pos` .. `joint_N_pos` out of a DATA body. Returns None if
    none are present (frame may be an unrelated DATA message)."""
    out: list[float] = []
    # Walk indexed keys 1..expected_n; if expected_n is 0 we discover by
    # probing up to 12 joints.
    upper = expected_n if expected_n > 0 else 12
    for i in range(1, upper + 1):
        v = body.get(f"joint_{i}_pos")
        if not isinstance(v, (int, float)):
            break
        out.append(float(v))
    return out or None


_MSG_COMMAND = 0x10


class _CmdSenderProtocol(asyncio.DatagramProtocol):
    """Datagram protocol for the kinova command socket — surfaces ICMP /
    connection_lost so the sender can flag itself broken and reopen."""

    def __init__(self, on_error: Callable[[], None]) -> None:
        self.on_error = on_error

    def error_received(self, exc: Exception) -> None:
        _log.warning("kinova command transport error: %s", exc)
        self.on_error()

    def connection_lost(self, exc: Exception | None) -> None:
        if exc is not None:
            _log.warning("kinova command transport closed: %s", exc)
        self.on_error()


class KinovaCommandSender:
    """Per-tick velocity sender to rove_sensor_api's kinova_arm cmd port.

    Reads state.joint_velocities (radians/s from IK), converts to deg/s,
    applies the same per-joint sign captured at Sync, clamps to a config
    cap, and pushes a single MSG_COMMAND UDP packet to the arm. Pure
    streaming model: silence triggers kinova's 300 ms velocity-hold
    timeout and the arm halts on its own."""

    def __init__(
        self,
        state: EngineState,
        host: str,
        cmd_port: int,
        max_vel_deg_s: float,
        min_vel_deg_s: float,
        debug: bool = False,
    ) -> None:
        self.state = state
        self.host = host
        self.cmd_port = cmd_port
        self.max_vel_deg_s = max_vel_deg_s
        self.min_vel_deg_s = min_vel_deg_s
        self.debug = debug
        self._seq = 0
        self._transport: asyncio.DatagramTransport | None = None
        self._broken = False
        self._last_send_warn_t = 0.0
        self._last_reopen_t = 0.0

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        try:
            self._transport, _ = await loop.create_datagram_endpoint(
                lambda: _CmdSenderProtocol(self._mark_broken),
                remote_addr=(self.host, self.cmd_port),
            )
        except Exception as exc:  # noqa: BLE001
            # Don't crash the engine if the cmd port can't be opened at boot —
            # maybe_send() will keep no-op'ing until reopen succeeds.
            _log.warning(
                "kinova command sender: socket open to %s:%d failed: %s "
                "(will retry on next send)",
                self.host, self.cmd_port, exc,
            )
            self._transport = None
            self._broken = True
            return
        self._broken = False
        _log.info(
            "kinova command sender ready: -> %s:%d  cap=%.1f deg/s",
            self.host, self.cmd_port, self.max_vel_deg_s,
        )

    def _mark_broken(self) -> None:
        self._broken = True

    def maybe_send(self) -> None:
        """Build the COMMAND packet for this tick. No-op when not synced or
        when every joint velocity rounds to zero.

        Velocities are scaled UNIFORMLY (not independently clamped) when the
        cap kicks in: IK produces a coordinated vel vector and the ratio
        between joints is what makes the EE actually translate. Clipping
        each joint independently breaks that ratio and the arm twists
        instead of moving in the requested direction.
        """
        # If a previous send saw an error, attempt to reopen at most once
        # every 2s so we don't hammer the OS with failed connect attempts.
        if (
            (self._transport is None or self._broken)
            and time.monotonic() - self._last_reopen_t > 2.0
        ):
            self._last_reopen_t = time.monotonic()
            self._broken = False
            try:
                loop = asyncio.get_running_loop()
                # create_datagram_endpoint must be awaited; schedule it as
                # a task and bail this tick — the next tick will use the
                # fresh transport if it succeeded.
                asyncio.create_task(self._reopen())
            except RuntimeError:
                # No running loop (called outside async ctx) — just bail.
                pass
            return
        if self._transport is None or self._broken:
            return
        if not self.state.kinova_chain_joint_ids:
            return  # not synced yet
        if not self.state.kinova_offsets:
            return

        # First pass: compute signed deg/s for every joint, no clamp.
        raw: list[float] = []
        for i, eid in enumerate(self.state.kinova_chain_joint_ids):
            if i >= 6:
                break
            sign = self.state.kinova_signs.get(eid, 1.0)
            qdot_rad = self.state.joint_velocities.get(eid, 0.0)
            raw.append(sign * qdot_rad * 180.0 / math.pi)

        # Uniform scale-down if any joint exceeds the cap.
        max_mag = max((abs(v) for v in raw), default=0.0)
        if max_mag > self.max_vel_deg_s:
            scale = self.max_vel_deg_s / max_mag
            raw = [v * scale for v in raw]
            max_mag = self.max_vel_deg_s

        if max_mag < self.min_vel_deg_s:
            # All joints below noise threshold: send nothing, let kinova's
            # velocity-hold timeout halt the arm.
            return

        payload: dict[str, float] = {
            f"joint_{i + 1}_vel": v for i, v in enumerate(raw)
        }

        self._seq = (self._seq + 1) & 0xFFFF
        body = json.dumps(payload).encode()
        header = struct.pack(_HEADER_FMT, _PROTOCOL_VERSION, _MSG_COMMAND, self._seq)
        try:
            self._transport.sendto(header + body)
        except Exception as exc:  # noqa: BLE001
            # Rate-limited WARN (was silent DEBUG before, so silent failures
            # like ICMP-unreachable looked like "arm not moving" with no log).
            now = time.monotonic()
            if now - self._last_send_warn_t > 5.0:
                _log.warning("kinova command send failed: %s", exc)
                self._last_send_warn_t = now
            self._broken = True
            return
        if self.debug:
            # Per-tick trace, only when debug=true. DEBUG level so even with
            # the flag on, raising the engine's log level filters it out
            # without code changes.
            _log.debug(
                "kinova cmd seq=%d  %s",
                self._seq,
                " ".join(f"j{i + 1}={v:+6.2f}" for i, v in enumerate(raw)),
            )

    async def _reopen(self) -> None:
        """Close any existing transport and re-open it. Quiet on success."""
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:  # noqa: BLE001
                pass
            self._transport = None
        try:
            loop = asyncio.get_running_loop()
            self._transport, _ = await loop.create_datagram_endpoint(
                lambda: _CmdSenderProtocol(self._mark_broken),
                remote_addr=(self.host, self.cmd_port),
            )
            self._broken = False
            _log.info(
                "kinova command sender: reconnected to %s:%d",
                self.host, self.cmd_port,
            )
        except Exception as exc:  # noqa: BLE001
            _log.debug("kinova command sender reopen failed: %s", exc)
            self._broken = True

    async def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()


# Staleness threshold: if no frames in this many seconds, the watchdog
# tears down the transport and re-subscribes. Tuned at ~10x the default
# subscribe_interval_ms so a single dropped packet doesn't trigger it.
_STALENESS_THRESHOLD_S = 2.0
_WATCHDOG_TICK_S = 1.0


class KinovaStateListener:
    """Subscribe to rove_sensor_api kinova_arm and keep the subscription alive.

    One UDP socket: SUBSCRIBE goes out, DATA frames come back to the same
    ephemeral port. A watchdog task wakes once per second and:

    * if no frames have arrived in _STALENESS_THRESHOLD_S, re-sends SUBSCRIBE
      (covers rove_sensor_api restart and lost initial SUBSCRIBE);
    * if the transport itself errored (ICMP unreachable, ``connection_lost``),
      tears down and rebuilds it before re-subscribing.

    First-time connection failures are also handled here — ``start()`` no
    longer raises on initial connect failure; the watchdog keeps retrying
    so the engine boots even if rove_sensor_api isn't up yet.
    """

    def __init__(
        self,
        state: EngineState,
        host: str,
        data_port: int,
        subscribe_interval_ms: int,
        expected_joint_count: int,
    ) -> None:
        self.state = state
        self.host = host
        self.data_port = data_port
        self.interval_ms = subscribe_interval_ms
        self.expected_n = expected_joint_count
        self._transport: asyncio.DatagramTransport | None = None
        self._watchdog_task: asyncio.Task | None = None
        self._stopping = False
        self._transport_broken = False
        self._last_subscribe_t = 0.0
        # State: "connecting" / "live" / "stale". Logged on transitions only,
        # so journald sees one line per state change instead of one per tick.
        self._link_state = "connecting"

    async def start(self) -> None:
        self._stopping = False
        # Best-effort initial connect. Failures here are not fatal — the
        # watchdog will retry.
        await self._open_transport()
        self._watchdog_task = asyncio.create_task(
            self._watchdog_loop(), name="kinova-state-watchdog"
        )

    async def _open_transport(self) -> bool:
        """Open the UDP socket and send SUBSCRIBE. Returns True on success."""
        loop = asyncio.get_running_loop()
        try:
            self._transport, _ = await loop.create_datagram_endpoint(
                lambda: _KinovaStateProtocol(
                    self.state, self.expected_n, on_error=self._mark_broken,
                ),
                local_addr=("0.0.0.0", 0),
                remote_addr=(self.host, self.data_port),
            )
        except Exception as exc:  # noqa: BLE001
            _log.warning(
                "kinova subscriber: socket open to %s:%d failed: %s",
                self.host, self.data_port, exc,
            )
            self._transport = None
            return False

        self._transport_broken = False
        try:
            self._transport.sendto(
                _encode(_MSG_SUBSCRIBE, 0, {"interval_ms": self.interval_ms})
            )
            self._last_subscribe_t = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            _log.warning("kinova SUBSCRIBE send failed: %s", exc)
            self._transport_broken = True
            return False

        _log.info(
            "kinova subscriber: SUBSCRIBE -> %s:%d (interval=%dms)",
            self.host, self.data_port, self.interval_ms,
        )
        return True

    def _mark_broken(self) -> None:
        """Called from the protocol when ICMP/connection_lost fires."""
        self._transport_broken = True

    async def _close_transport(self) -> None:
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:  # noqa: BLE001
                pass
            self._transport = None

    async def _watchdog_loop(self) -> None:
        """Detect staleness / transport breakage and reopen as needed."""
        while not self._stopping:
            try:
                await asyncio.sleep(_WATCHDOG_TICK_S)
                if self._stopping:
                    return

                now = time.monotonic()
                have_frame = self.state.latest_kinova_t > 0.0
                age = (
                    now - self.state.latest_kinova_t
                    if have_frame
                    else float("inf")
                )

                # Case 1: transport itself died — tear down and rebuild.
                if self._transport_broken or self._transport is None:
                    if self._link_state != "connecting":
                        _log.warning(
                            "kinova link DOWN (transport broken) — reconnecting"
                        )
                        self._link_state = "connecting"
                    await self._close_transport()
                    if await self._open_transport():
                        # Don't flip to "live" yet — wait for a real frame.
                        pass
                    continue

                # Case 2: socket is up but frames stopped arriving.
                if have_frame and age > _STALENESS_THRESHOLD_S:
                    if self._link_state == "live":
                        _log.warning(
                            "kinova frames stale (last %.1fs ago) — "
                            "re-sending SUBSCRIBE",
                            age,
                        )
                        self._link_state = "stale"
                    # Re-send SUBSCRIBE, no more than once per 2s, so
                    # rove_sensor_api restarting fully picks us up.
                    if now - self._last_subscribe_t > 2.0:
                        try:
                            self._transport.sendto(
                                _encode(
                                    _MSG_SUBSCRIBE,
                                    0,
                                    {"interval_ms": self.interval_ms},
                                )
                            )
                            self._last_subscribe_t = now
                        except Exception as exc:  # noqa: BLE001
                            _log.warning("kinova re-SUBSCRIBE failed: %s", exc)
                            self._transport_broken = True
                    continue

                # Case 3: frames flowing — transition to live if needed.
                if have_frame and age <= _STALENESS_THRESHOLD_S:
                    if self._link_state != "live":
                        _log.info(
                            "kinova link LIVE (first frame %.2fs after start)",
                            age,
                        )
                        self._link_state = "live"

            except asyncio.CancelledError:
                return
            except Exception as exc:  # noqa: BLE001
                _log.exception("kinova watchdog tick raised: %s", exc)
                # Don't let a bug here kill the loop entirely.
                await asyncio.sleep(_WATCHDOG_TICK_S)

    async def stop(self) -> None:
        self._stopping = True
        if self._watchdog_task is not None:
            self._watchdog_task.cancel()
            try:
                await self._watchdog_task
            except asyncio.CancelledError:
                pass
            self._watchdog_task = None
        if self._transport is not None:
            try:
                self._transport.sendto(_encode(_MSG_UNSUBSCRIBE, 0, None))
            except Exception:  # noqa: BLE001
                pass
            await self._close_transport()


# ---- chain / name resolution + sync ---------------------------------------


def resolve_joint_name_to_entity_id(state: EngineState, name: str) -> str | None:
    target = name.strip().lower()
    if not target:
        return None
    for eid, ent in state.project.scene.entities.items():
        if (ent.name or "").strip().lower() == target:
            if ent.get("joint") is not None:
                return eid
    return None


def resolve_arm_joint_ids(
    state: EngineState,
    *,
    arm_base_entity_id: str = "",
    arm_tip_entity_id: str = "",
    joint_names: list[str] | None = None,
) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    if arm_base_entity_id and arm_tip_entity_id:
        from forgebot.core.kinematics import extract_chain
        try:
            chain = extract_chain(
                state.project, arm_base_entity_id, arm_tip_entity_id
            )
        except (KeyError, ValueError) as exc:
            errors.append(f"chain extraction failed: {exc}")
            return [], errors
        return list(chain.joints), errors

    if not joint_names:
        errors.append(
            "no arm mapping configured — set arm_base_entity_id + "
            "arm_tip_entity_id, OR populate joint_names in engine.toml"
        )
        return [], errors

    ids: list[str] = []
    for name in joint_names:
        eid = resolve_joint_name_to_entity_id(state, name)
        if eid is None:
            errors.append(f"no movable joint named {name!r} in scene")
            continue
        ids.append(eid)
    return ids, errors


def snap_model_to_kinova(
    state: EngineState,
    *,
    arm_base_entity_id: str = "",
    arm_tip_entity_id: str = "",
    joint_names: list[str] | None = None,
    inverted_joints: Iterable[int] | None = None,
) -> tuple[int, list[str], list[str], dict[str, float]]:
    """Calibrate the kinova<->model frame offset.

    At sync time the user has placed the real arm at the same physical pose
    as the 3D model (typically home). We capture::

        offset[i] = kinova_q[i] - model_q[i]

    The model is NOT overwritten -- it stays at its current pose visually.
    After sync, future kinova reads map into the model frame as
    `kinova_q - offset`, so when the real arm moves the model can mirror it
    without inheriting kinova's 180-degree-zero convention.

    Returns `(captured_count, errors, resolved_joint_ids, offsets_dict)`."""
    joint_ids, errors = resolve_arm_joint_ids(
        state,
        arm_base_entity_id=arm_base_entity_id,
        arm_tip_entity_id=arm_tip_entity_id,
        joint_names=joint_names,
    )
    if state.latest_kinova_positions is None:
        errors.append("no kinova state received yet (sensor_api not pushing?)")
        return 0, errors, joint_ids, {}

    positions = state.latest_kinova_positions
    if len(positions) < len(joint_ids):
        errors.append(
            f"kinova frame has {len(positions)} positions but chain has "
            f"{len(joint_ids)} joints — only the first {len(positions)} will be calibrated"
        )
    if len(positions) > len(joint_ids):
        errors.append(
            f"kinova frame has {len(positions)} positions but chain only "
            f"has {len(joint_ids)} joints — extra values ignored"
        )

    inverted = set(inverted_joints or ())

    n = min(len(positions), len(joint_ids))
    captured: dict[str, float] = {}
    state.kinova_signs.clear()
    for i in range(n):
        eid = joint_ids[i]
        kinova_idx = i + 1  # 1-based kinova actuator index
        sign = -1.0 if kinova_idx in inverted else 1.0
        state.kinova_signs[eid] = sign
        signed_kinova_q = sign * float(positions[i])
        model_q = float(state.joint_values.get(eid, 0.0))
        offset = signed_kinova_q - model_q
        state.kinova_offsets[eid] = offset
        captured[eid] = offset
        # Intentional no-op on joint_values: model_q stays where it was.
        # The mirror loop reads sign + offset to keep the model in sync
        # with whatever kinova reports going forward.
    state.kinova_chain_joint_ids = joint_ids[:n]
    return n, errors, joint_ids, captured
