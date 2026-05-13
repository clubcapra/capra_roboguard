"""Track conversion: left_vel/right_vel → ODrive velocity (rev/s)."""
from __future__ import annotations

from ..config import TracksConfig
from .base import ConversionStrategy, NodeCommand

# ODrive control_mode=2 (Velocity), input_mode=1 (Passthrough)
_CONTROL_MODE_VELOCITY = 2
_INPUT_MODE_PASSTHROUGH = 1


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


class TracksVelocityStrategy(ConversionStrategy):
    """Convert normalized track velocities [-1, 1] to ODrive rev/s commands.

    Each ODrive node on a track receives the same velocity setpoint.  Nodes
    on the left track receive ``left_vel * max_velocity`` (with optional
    inversion), right-track nodes receive ``right_vel * max_velocity``.
    """

    name = "velocity"

    def __init__(self, cfg: TracksConfig) -> None:
        self._cfg = cfg
        self._left_sign = -1.0 if cfg.invert_left else 1.0
        self._right_sign = -1.0 if cfg.invert_right else 1.0

    def initialize(self) -> list[NodeCommand]:
        """Set control mode on all track nodes. axis_state is managed by the bridge."""
        cmds: list[NodeCommand] = []
        all_ids = list(self._cfg.left_node_ids) + list(self._cfg.right_node_ids)
        for nid in all_ids:
            cmds.append(NodeCommand(
                node_id=nid,
                payload={
                    "control_mode": _CONTROL_MODE_VELOCITY,
                    "input_mode": _INPUT_MODE_PASSTHROUGH,
                    "input_vel": 0.0,
                },
            ))
        return cmds

    def convert(self, msg) -> list[NodeCommand]:
        left_vel = _clamp(
            msg.tracks.left_vel * self._cfg.max_velocity * self._left_sign,
            self._cfg.max_velocity,
        )
        right_vel = _clamp(
            msg.tracks.right_vel * self._cfg.max_velocity * self._right_sign,
            self._cfg.max_velocity,
        )

        # axis_state is included in every packet so the ODrive stays in
        # ClosedLoopControl even if it faults and resets during operation.
        cmds: list[NodeCommand] = []
        for nid in self._cfg.left_node_ids:
            cmds.append(NodeCommand(
                node_id=nid,
                payload={"axis_state": 8, "input_vel": left_vel},
            ))
        for nid in self._cfg.right_node_ids:
            cmds.append(NodeCommand(
                node_id=nid,
                payload={"axis_state": 8, "input_vel": right_vel},
            ))
        return cmds

    def zero_commands(self) -> list[NodeCommand]:
        all_ids = list(self._cfg.left_node_ids) + list(self._cfg.right_node_ids)
        return [
            NodeCommand(node_id=nid, payload={"axis_state": 8, "input_vel": 0.0})
            for nid in all_ids
        ]

    def estop(self) -> list[NodeCommand]:
        all_ids = list(self._cfg.left_node_ids) + list(self._cfg.right_node_ids)
        return [NodeCommand(node_id=nid, payload={"input_vel": 0.0}) for nid in all_ids]
