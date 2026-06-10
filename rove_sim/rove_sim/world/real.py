"""RealWorld: a geometric world model, not a physics sim.

In real mode the robot is synced from telemetry (see drivers/sync.py) and is NOT
integrated by physics. The world here exists only as geometry for IK / collision
/ spatial reasoning: no ground plane (the real robot rests on the real ground,
which the sim never simulates), zero gravity (so any perceived bodies inserted
as the world model don't free-fall), and dynamic object bodies upserted each
tick from a PerceptionSource.
"""
from __future__ import annotations

from typing import Dict

import pybullet as p

from .base import World, register
from .perception import build_perception


@register("real")
class RealWorld(World):
    def __init__(self, engine, spec=None, profile=None, **params):
        super().__init__(engine, spec, profile=profile, **params)
        self.perception = build_perception(self.spec.get("perception"))
        self._bodies: Dict[str, int] = {}        # detection id -> pybullet body

    def build(self) -> "RealWorld":
        # World model, not physics: keep a clock for sensor rate-gating but do
        # not integrate the robot (the SyncDriver never calls engine.step()).
        p.setTimeStep(float(self.spec.get("timestep", self.engine.cfg.timestep)))
        p.setGravity(0, 0, 0)
        # No ground plane: the real robot's support is the real world.
        self.perception.start()
        return self

    def update(self, dt: float) -> None:
        seen = set()
        for det in self.perception.poll():
            seen.add(det.id)
            self._upsert(det)
        for stale in [k for k in self._bodies if k not in seen]:
            p.removeBody(self._bodies.pop(stale))

    def _upsert(self, det) -> None:
        half = [e / 2.0 for e in det.extents]
        if det.id not in self._bodies:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
            self._bodies[det.id] = p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=col,
                basePosition=list(det.pose), baseOrientation=list(det.orn))
        else:
            p.resetBasePositionAndOrientation(
                self._bodies[det.id], list(det.pose), list(det.orn))

    def reset(self) -> None:
        for bid in self._bodies.values():
            p.removeBody(bid)
        self._bodies.clear()
        self.perception.stop()
