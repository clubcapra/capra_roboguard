"""Track conversion: full torque ODrive control.

Every drum on both sides runs in torque mode — stick deflection scales
directly to Nm via max_torque.

Stick input is shaped through a configurable expo curve before scaling, so
near-centre deflections give proportional response instead of a step.
curve_expo=0 is linear; curve_expo=1 is cubic (x^3 feel).
"""
from __future__ import annotations

from ..config import TracksConfig
from .base import ConversionStrategy, NodeCommand

_CONTROL_MODE_TORQUE = 1
_INPUT_MODE_PASSTHROUGH = 1


def _shape(x: float, expo: float) -> float:
    """Expo curve: expo=0 → linear, expo=1 → cubic."""
    return x * (expo * x * x + (1.0 - expo))


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


class TracksTorqueStrategy(ConversionStrategy):
    """Convert normalized track velocities [-1, 1] to torque commands on every drum."""

    name = "torque"

    def __init__(self, cfg: TracksConfig) -> None:
        self._cfg = cfg
        self._left_sign = -1.0 if cfg.invert_left else 1.0
        self._right_sign = -1.0 if cfg.invert_right else 1.0

    def initialize(self) -> list[NodeCommand]:
        """Set per-node control modes. axis_state is managed by the bridge."""
        cmds: list[NodeCommand] = []
        for side_ids in (self._cfg.left_node_ids, self._cfg.right_node_ids):
            for nid in side_ids:
                cmds.append(NodeCommand(
                    node_id=nid,
                    payload={
                        "control_mode": _CONTROL_MODE_TORQUE,
                        "input_mode": _INPUT_MODE_PASSTHROUGH,
                        "input_torque": 0.0,
                    },
                ))
        return cmds

    def convert(self, msg) -> list[NodeCommand]:
        expo = self._cfg.curve_expo
        left_shaped = _shape(msg.tracks.left_vel, expo) * self._left_sign
        right_shaped = _shape(msg.tracks.right_vel, expo) * self._right_sign

        left_torque = _clamp(left_shaped * self._cfg.max_torque, self._cfg.max_torque)
        right_torque = _clamp(right_shaped * self._cfg.max_torque, self._cfg.max_torque)

        cmds: list[NodeCommand] = []
        for side_ids, torque in (
            (self._cfg.left_node_ids, left_torque),
            (self._cfg.right_node_ids, right_torque),
        ):
            for nid in side_ids:
                cmds.append(NodeCommand(
                    node_id=nid,
                    payload={"axis_state": 8, "input_torque": torque},
                ))
        return cmds

    def zero_commands(self) -> list[NodeCommand]:
        cmds: list[NodeCommand] = []
        for side_ids in (self._cfg.left_node_ids, self._cfg.right_node_ids):
            for nid in side_ids:
                cmds.append(NodeCommand(node_id=nid, payload={"axis_state": 8, "input_torque": 0.0}))
        return cmds

    def estop(self) -> list[NodeCommand]:
        cmds: list[NodeCommand] = []
        for side_ids in (self._cfg.left_node_ids, self._cfg.right_node_ids):
            for nid in side_ids:
                cmds.append(NodeCommand(node_id=nid, payload={"input_torque": 0.0}))
        return cmds
