"""Abstract strategy interface for RoveControl → rove_sensor_api conversion."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any


@dataclass
class NodeCommand:
    """A command destined for a single ODrive node."""
    node_id: int
    payload: dict[str, Any]


class ConversionStrategy(ABC):
    """Base class for all conversion strategies.

    Implementations translate a parsed ``RoveControl`` protobuf message into
    a list of ``NodeCommand`` objects that are then forwarded to
    ``rove_sensor_api`` via the UDP command protocol.
    """

    #: Short human-readable name used in logs and the config file.
    name: str = "unnamed"

    @abstractmethod
    def convert(self, msg: Any) -> list[NodeCommand]:
        """Return per-node commands for this control frame.

        Args:
            msg: A ``RoveControl`` protobuf message.

        Returns:
            List of ``NodeCommand`` — may be empty (e.g. when the subsystem
            is disabled or all setpoints are zero).
        """

    def initialize(self) -> list[NodeCommand]:
        """One-shot commands sent at bridge startup (e.g. set control mode).

        Override to return the axis-state and control-mode commands that must
        be sent before streaming begins.  Default: empty list.
        """
        return []

    def zero_commands(self) -> list[NodeCommand]:
        """Zero-setpoint commands with axis_state=8 for keepalive when no packets arrive.

        The keepalive uses these instead of the last received commands so the
        robot stops immediately when the operator releases the stick, while
        still keeping the ODrives armed in ClosedLoopControl.
        """
        return []

    def estop(self) -> list[NodeCommand]:
        """Commands to send on an emergency stop (zero motion, safe state)."""
        return []
