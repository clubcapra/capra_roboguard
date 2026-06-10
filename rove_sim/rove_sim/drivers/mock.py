"""MockDriver: physics stepping (the original Sim.step_control loop).

Extracted verbatim from runtime.Sim so mock mode is byte-identical to before.
apply() runs once per control tick (setpoints persist); force-based actuators
implement step(), which must run EVERY physics step, so we interleave per-step
actuation with single physics steps. The world is static in mock mode, so
world.update() is a cheap no-op called once per control tick.
"""
from __future__ import annotations

from .base import Driver, register


@register("mock")
class MockDriver(Driver):
    def step_control(self, ticks: int, intent) -> None:
        for _ in range(ticks):
            for a in self.actuators:
                a.apply(intent)
            for _ in range(self._phys_per_control):
                for a in self.actuators:
                    a.step(intent)
                self.engine.step(1)
            self.world.update(self.engine.cfg.timestep * self._phys_per_control)

    def settle(self, seconds: float) -> None:
        self.engine.step(int(seconds / self.engine.cfg.timestep))
