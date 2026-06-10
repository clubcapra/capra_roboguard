"""World seam: everything in the PyBullet scene that is NOT the robot.

A World owns the simulation's *environment*: gravity, the ground/terrain, static
obstacles and (in real mode) the geometry inserted from perception. It is the
half of the old `Engine.connect()` that was world setup masquerading as
connection setup -- pulling it out here is what lets one engine connection serve
both deployment modes:

  * MockWorld  -- physics world: gravity on, a ground plane, terrain + friction
                  (the robot is driven by physics in mock mode).
  * RealWorld  -- a geometric world model: no ground physics, objects inserted
                  from a PerceptionSource (the robot is synced kinematically).

The mode axis (mock|real) is orthogonal to the engine connection mode
(gui|headless). A profile declares its world with an optional top-level `world:`
block (same `profile.raw` convention as `safety:`/`gripper:`); the CLI/`build()`
`world=` argument overrides `world.use`.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict

from ..registry import Registry

WORLD_REGISTRY = Registry("world")
register = WORLD_REGISTRY.register


class World(ABC):
    def __init__(self, engine, spec: Dict[str, Any] | None = None,
                 profile=None, **params):
        self.engine = engine
        self.spec = dict(spec or {})
        self.profile = profile
        self.params = params
        self.ground_id: int | None = None

    @abstractmethod
    def build(self) -> "World":
        """Create all non-robot bodies and set global physics state.

        Sets gravity + timestep on the engine's client and loads ground/terrain
        (mock) or nothing (real). Returns self so callers can `World(...).build()`.
        """

    def update(self, dt: float) -> None:
        """Advance dynamic world content. Default no-op (mock world is static --
        physics moves the robot, not the world). RealWorld overrides to upsert
        bodies from its PerceptionSource."""

    def reset(self) -> None:
        """Tear down dynamic content (default no-op)."""


def build_world(profile, engine, override: str | None = None,
                overrides: Dict[str, Any] | None = None) -> World:
    """Resolve and build the world for a profile.

    `override` (the build()/CLI `world=` argument) wins over the profile's
    `world.use`; both default to 'mock' so an unannotated profile is a physics
    world exactly as before. `overrides` is a dict merged into the world spec
    (e.g. inject `terrain` for a demo without editing the profile).
    """
    spec = dict(profile.raw.get("world", {}))
    if overrides:
        spec.update(overrides)
    key = override or spec.get("use", "mock")
    spec["use"] = key
    return WORLD_REGISTRY.build(key, engine=engine, spec=spec,
                               profile=profile).build()
