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
from typing import Iterable

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
    def __init__(self, state: EngineState, expected_n: int) -> None:
        self.state = state
        self.expected_n = expected_n
        self._warned = False

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        try:
            mt, _, body = _decode(data)
        except Exception as exc:
            if not self._warned:
                _log.warning("kinova frame decode failed from %s: %s", addr, exc)
                self._warned = True
            return
        if mt != _MSG_DATA or not isinstance(body, dict):
            return
        positions_deg = _extract_positions(body, self.expected_n)
        if positions_deg is None:
            if not self._warned:
                _log.warning(
                    "kinova DATA frame had no joint_N_pos fields; keys=%s",
                    list(body.keys())[:8],
                )
                self._warned = True
            return
        # Convert degrees -> radians: engine kinematics expects radians.
        self.state.latest_kinova_positions = [
            v * math.pi / 180.0 for v in positions_deg
        ]
        self.state.latest_kinova_t = time.monotonic()


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


class KinovaStateListener:
    """Subscribe to rove_sensor_api kinova_arm and stash incoming positions in
    EngineState. One UDP socket: SUBSCRIBE goes out, DATA comes back to the
    same (ephemeral) port the kernel assigned us."""

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

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: _KinovaStateProtocol(self.state, self.expected_n),
            local_addr=("0.0.0.0", 0),
            remote_addr=(self.host, self.data_port),
        )
        # Send SUBSCRIBE. The server pushes DATA back to our sender on the
        # same ephemeral port.
        self._transport.sendto(
            _encode(_MSG_SUBSCRIBE, 0, {"interval_ms": self.interval_ms})
        )
        _log.info(
            "kinova subscriber: sent SUBSCRIBE to %s:%d (interval=%dms)",
            self.host,
            self.data_port,
            self.interval_ms,
        )

    async def stop(self) -> None:
        if self._transport is None:
            return
        try:
            self._transport.sendto(_encode(_MSG_UNSUBSCRIBE, 0, None))
        except Exception:  # noqa: BLE001
            pass
        self._transport.close()


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

    n = min(len(positions), len(joint_ids))
    captured: dict[str, float] = {}
    for i in range(n):
        eid = joint_ids[i]
        kinova_q = float(positions[i])
        model_q = float(state.joint_values.get(eid, 0.0))
        offset = kinova_q - model_q
        state.kinova_offsets[eid] = offset
        captured[eid] = offset
        # Intentional no-op on joint_values: model_q stays where it was.
        # The mirror loop in ik_loop.tick() reads the offsets back to keep
        # the model in sync with whatever kinova reports going forward.
    # Cache chain order so the per-tick mirror doesn't re-resolve every time.
    state.kinova_chain_joint_ids = joint_ids[:n]
    return n, errors, joint_ids, captured
