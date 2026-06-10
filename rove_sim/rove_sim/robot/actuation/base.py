"""Actuator library contract (S3): registry of intent->joint resolvers.

Each actuator consumes one field of the normalized RoveControl intent and drives
its bound joints in PyBullet. The control bridge calls apply() with the decoded
intent; actuators never see per-joint setpoints from outside (S3 principle 6).
Bound by child-link name -- the stable cross-robot handle (S-recon).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from ...registry import Registry

ACTUATOR_REGISTRY = Registry("actuator")
register = ACTUATOR_REGISTRY.register


class Actuator(ABC):
    #: which RoveControl field this actuator consumes ("tracks", "flippers", ...)
    intent_field: str = ""

    def __init__(self, robot, bind, **params):
        self.robot = robot
        self.bind = bind
        self.params = params
        self._resolve_joints()

    @abstractmethod
    def _resolve_joints(self) -> None:
        """Map self.bind (link names) -> pybullet joint indices once."""

    @abstractmethod
    def apply(self, intent) -> None:
        """Drive bound joints from the relevant slice of RoveControl intent.

        Called once per CONTROL tick. Position/velocity setpoints persist across
        physics steps, so most actuators only need apply().
        """

    def step(self, intent) -> None:
        """Per-PHYSICS-step hook (default no-op).

        Force-based actuators (the brush-friction tracks) must re-apply their
        forces every physics step -- applyExternalForce lasts one step only --
        so the runtime calls this on every actuator each physics step.
        """


def build_actuators(specs, robot) -> List[Actuator]:
    return [ACTUATOR_REGISTRY.build(s.use, robot=robot, bind=s.bind, **s.params)
            for s in specs]
