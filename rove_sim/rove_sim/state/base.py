"""RobotStateSource: where the SyncDriver gets the robot's pose in real mode.

In real mode the robot is NOT simulated -- its joint angles and base pose come
from live telemetry and are mapped onto the URDF kinematically. A RobotStateSource
is the seam that produces that state, keyed by URDF joint NAME so the SyncDriver
can resolve it through `robot.joint_index` (the same name-keyed handle the loader
and rove_ik.py use).

Concrete sources:
  * ManualStateSource / ReplayStateSource (state/manual.py) -- no hardware, for
    tests and canned trajectories.
  * RoveSensorApiStateSource (state/rove_sensor_api.py) -- subscribes to the real
    `rove_sensor_api` wire format (kinova/odrive/robotiq/vectornav), which the
    mock SimSensorApi ALSO speaks, so the same source drives an offline mock->real
    loop or the real robot.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

from ..registry import Registry

STATE_SOURCE_REGISTRY = Registry("state_source")
register = STATE_SOURCE_REGISTRY.register

# (position xyz, orientation xyzw) in the world frame.
BasePose = Tuple[Sequence[float], Sequence[float]]


@dataclass
class RobotState:
    joints: Dict[str, float] = field(default_factory=dict)   # URDF joint name -> rad
    base_pose: Optional[BasePose] = None                     # or None to leave base put
    stamp_us: int = 0


class RobotStateSource(ABC):
    @abstractmethod
    def read(self) -> RobotState:
        """Return the latest robot state (non-blocking snapshot)."""

    def start(self) -> None:
        """Spin up any rx threads / sockets (default no-op)."""

    def stop(self) -> None:
        """Tear down (default no-op)."""


def build_state_source(profile, key: Optional[str] = None,
                       **kwargs) -> RobotStateSource:
    """Resolve a state source. `key` overrides the profile's `state.use`
    (default 'manual' -- an empty hand-driven source for offline real mode)."""
    spec = dict(profile.raw.get("state", {}))
    key = key or spec.get("use", "manual")
    params = {k: v for k, v in spec.items() if k != "use"}
    params.update(kwargs)
    return STATE_SOURCE_REGISTRY.build(key, profile=profile, **params)
