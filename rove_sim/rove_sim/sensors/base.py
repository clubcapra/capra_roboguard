"""Sensor library contract (S5.4): template method + registry.

A sensor subclass implements only _sample() (clean physics reading) and,
optionally, _apply_errors(). update() is the template -- it owns rate-gating
and pushes every reading through the error model, so imperfection is in the
pipeline, never optional (S5.5). Add a sensor TYPE = subclass + @register;
add an INSTANCE = a block in a profile manifest.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, List

from ..registry import Registry

SENSOR_REGISTRY = Registry("sensor")
register = SENSOR_REGISTRY.register


class Sensor(ABC):
    def __init__(self, name: str, robot, link: str, clock,
                 rate_hz: float = 50.0, transport=None, engine=None, **params):
        self.name = name
        self.robot = robot
        self.link = link
        self.clock = clock
        self.rate_hz = float(rate_hz)
        self.transport = transport
        self.engine = engine
        self.params = params
        self._period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.0
        self._accum = 0.0
        self.last = None              # most recent reading (for overlay / polling)

    # -- mount pose ---------------------------------------------------------
    def link_pose(self):
        """World (pos, orn-xyzw) of the mount link. Base link uses the body pose."""
        import pybullet as p
        idx = self.robot.link_index.get(self.link, -1)
        if idx < 0:
            return p.getBasePositionAndOrientation(self.robot.body_id)
        st = p.getLinkState(self.robot.body_id, idx, computeForwardKinematics=True)
        return st[4], st[5]           # worldLinkFramePosition, ...Orientation

    # -- template method: DO NOT override -----------------------------------
    def update(self, dt: float) -> None:
        if self._period <= 0.0:
            self._emit(self._apply_errors(self._sample()))
            return
        self._accum += dt
        while self._accum >= self._period:
            self._accum -= self._period
            self._emit(self._apply_errors(self._sample()))

    def sample(self):
        """Force one reading now (ignores rate gate); also updates `last`."""
        r = self._apply_errors(self._sample())
        self._emit(r)
        return r

    def _emit(self, reading) -> None:
        self.last = reading
        self._publish(reading)

    # -- subclass hooks ------------------------------------------------------
    @abstractmethod
    def _sample(self) -> Any:
        """Return a clean reading from physics. Subclass-specific."""

    def _apply_errors(self, reading: Any) -> Any:
        """Default identity; sensors with error models override/compose."""
        return reading

    def _publish(self, reading: Any) -> None:
        if self.transport is not None and reading is not None:
            self.transport.publish(self.name, reading)


def build_sensors(specs, robot, clock, transports=None, engine=None) -> List[Sensor]:
    """Instantiate sensor instances from profile ComponentSpec list."""
    out: List[Sensor] = []
    for spec in specs:
        tp = (transports or {}).get(spec.name)
        out.append(SENSOR_REGISTRY.build(
            spec.use, name=spec.name, robot=robot, link=spec.bind,
            clock=clock, transport=tp, engine=engine, **spec.params))
    return out
