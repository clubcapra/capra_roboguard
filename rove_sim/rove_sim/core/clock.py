"""SimClock: the sim's notion of time, advanced by the control loop.

Sensors timestamp their readings against this, not wall-clock, so a headless run
stamps the simulated time it represents (and a paused/stepped sim stamps
consistently). The runtime ticks it once per control period.
"""
from __future__ import annotations


class SimClock:
    def __init__(self, t0: float = 0.0):
        self.t = float(t0)

    def tick(self, dt: float) -> float:
        self.t += float(dt)
        return self.t

    def now(self) -> float:
        return self.t
