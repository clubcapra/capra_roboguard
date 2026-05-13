"""Kinova arm state listener.

The rove_sensor_api streams kinova_arm joint positions over UDP. This module
binds the configured port, parses each datagram, and stashes the latest
positions in EngineState so the frontend's "Sync" button can snap the model
to the real arm.

Wire format ASSUMPTION (rove_sensor_api convention -- adjust `_parse_frame`
if your stream uses a different schema):

    bytes 0..3   header: version(0x01) + msg_type + seq (u16 LE)
    bytes 4..    JSON body, e.g. `{"positions": [p1, p2, ..., pN]}` (radians)

If the actual stream uses raw JSON with no header, or a binary positions
array, edit `_parse_frame` -- everything else is format-agnostic.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Iterable

from .state import EngineState

_log = logging.getLogger(__name__)


class _KinovaStateProtocol(asyncio.DatagramProtocol):
    def __init__(self, state: EngineState, expected_n: int) -> None:
        self.state = state
        self.expected_n = expected_n
        self._warned_shape = False

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        positions = _parse_frame(data)
        if positions is None:
            if not self._warned_shape:
                _log.warning(
                    "kinova frame from %s did not match expected shape "
                    "(header + JSON {positions: [...]}). First %d bytes: %r",
                    addr,
                    min(32, len(data)),
                    data[:32],
                )
                self._warned_shape = True
            return
        if self.expected_n and len(positions) != self.expected_n:
            _log.warning(
                "kinova frame has %d positions, expected %d (joint_names mismatch?)",
                len(positions),
                self.expected_n,
            )
        self.state.latest_kinova_positions = positions
        self.state.latest_kinova_t = time.monotonic()


def _parse_frame(data: bytes) -> list[float] | None:
    """Extract the positions list from one UDP frame. Returns None on
    parse failure (logged once by the protocol)."""
    # Try header-prefixed JSON first (matches the bridge's command wire
    # format: 4-byte header followed by JSON body).
    body: bytes
    if len(data) >= 4 and data[0] == 0x01:
        body = data[4:]
    else:
        body = data
    try:
        obj = json.loads(body)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    # Accept a few common shapes.
    if isinstance(obj, dict):
        for key in ("positions", "pos", "joint_positions", "q"):
            v = obj.get(key)
            if isinstance(v, list) and all(isinstance(x, (int, float)) for x in v):
                return [float(x) for x in v]
    if isinstance(obj, list) and all(isinstance(x, (int, float)) for x in obj):
        return [float(x) for x in obj]
    return None


class KinovaStateListener:
    def __init__(
        self,
        state: EngineState,
        port: int,
        joint_names: Iterable[str],
    ) -> None:
        self.state = state
        self.port = port
        self.joint_names = list(joint_names)
        self._transport: asyncio.DatagramTransport | None = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: _KinovaStateProtocol(self.state, expected_n=len(self.joint_names)),
            local_addr=("0.0.0.0", self.port),
        )
        _log.info(
            "kinova state listener bound on UDP 0.0.0.0:%d  joints=%s",
            self.port,
            self.joint_names,
        )

    async def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()


def resolve_joint_name_to_entity_id(state: EngineState, name: str) -> str | None:
    """Look up a movable joint entity by its `Entity.name`. Case-insensitive."""
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
    """Return `(joint_ids_in_order, errors)`. Chain mode is preferred when
    both base/tip are set; name mode is the fallback."""
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
) -> tuple[int, list[str], list[str]]:
    """Apply `state.latest_kinova_positions` to the engine's joint_values.

    Returns `(updated, errors, resolved_joint_ids)`. The third element is the
    ordered list of joint entity ids that *should* have been updated, useful
    for the UI to display what the mapping looks like."""
    joint_ids, errors = resolve_arm_joint_ids(
        state,
        arm_base_entity_id=arm_base_entity_id,
        arm_tip_entity_id=arm_tip_entity_id,
        joint_names=joint_names,
    )
    if state.latest_kinova_positions is None:
        errors.append("no kinova state received yet (sensor_api not streaming?)")
        return 0, errors, joint_ids

    positions = state.latest_kinova_positions
    if len(positions) < len(joint_ids):
        errors.append(
            f"kinova frame has {len(positions)} positions but chain has "
            f"{len(joint_ids)} joints — only the first {len(positions)} "
            "will be snapped"
        )
    if len(positions) > len(joint_ids):
        errors.append(
            f"kinova frame has {len(positions)} positions but chain only "
            f"has {len(joint_ids)} joints — extra values ignored"
        )

    n = min(len(positions), len(joint_ids))
    for i in range(n):
        state.joint_values[joint_ids[i]] = float(positions[i])
        state.joint_velocities[joint_ids[i]] = 0.0
    return n, errors, joint_ids
