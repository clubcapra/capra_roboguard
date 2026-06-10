"""Driver seam: how robot + world state advance each control tick.

The driver is the other half of the mock/real split (the World owns the
environment; the Driver owns the stepping). It pairs 1:1 with the world:

  * MockDriver (drivers/mock.py)  -- physics: actuators.apply/step + engine.step.
  * SyncDriver (drivers/sync.py)  -- kinematic: resetJointState from a
                                     RobotStateSource; NO physics on the robot.

`build_driver(key, ...)` keys off the same word as the world ('mock' | 'real'),
so one `world=` argument selects the matched (World, Driver) pair.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from ..registry import Registry

DRIVER_REGISTRY = Registry("driver")
register = DRIVER_REGISTRY.register


class Driver(ABC):
    def __init__(self, engine, robot, actuators, world, control_hz=50.0,
                 source=None, guard=None, **params):
        self.engine = engine
        self.robot = robot
        self.actuators = actuators
        self.world = world
        self.control_hz = control_hz
        self.source = source
        self.guard = guard
        self.params = params

    @property
    def _phys_per_control(self) -> int:
        return max(1, round((1.0 / self.control_hz) / self.engine.cfg.timestep))

    @abstractmethod
    def step_control(self, ticks: int, intent) -> None:
        """Advance `ticks` control periods given the current intent."""

    def settle(self, seconds: float) -> None:
        """Initial settle before control (default no-op: kinematic worlds)."""


def build_driver(key: str, **kwargs) -> Driver:
    return DRIVER_REGISTRY.build(key, **kwargs)
