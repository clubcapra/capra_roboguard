"""Minimal sim runtime: assemble engine + robot + actuators and step.

Control intent is applied at a fixed control rate (default 50 Hz) while physics
runs at the engine timestep, so a flipper "step" is one increment per control
tick, not per physics tick. Later milestones add sensors, the API seam and a
proper scheduler around this same assembly.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List

import pybullet as p

from .core.engine import Engine, EngineConfig
from .core.clock import SimClock
from .robot import loader
from .robot.profile import load_profile
from .robot.actuation import build_actuators
from .world import build_world
from .drivers import build_driver
from .state import build_state_source
from . import capabilities, sensors  # noqa: F401  (sensors import registers types)
from .sensors.base import build_sensors
from .control import RoveControl

PROFILE_DIR = os.path.join(os.path.dirname(__file__), "..", "profiles")


def resolve_profile(name_or_path: str) -> str:
    if os.path.exists(name_or_path):
        return name_or_path
    cand = os.path.join(PROFILE_DIR, f"{name_or_path}.yaml")
    if os.path.exists(cand):
        return cand
    raise FileNotFoundError(f"no profile {name_or_path!r}")


@dataclass
class Sim:
    engine: Engine
    robot: loader.Robot
    actuators: list
    caps: capabilities.Capabilities
    control_hz: float = 50.0
    intent: RoveControl = field(default_factory=RoveControl)
    guard: object = None        # SelfCollisionGuard (or None)
    world: object = None        # World (mock physics | real world-model)
    driver: object = None       # Driver (mock physics | sync kinematic)
    source: object = None       # RobotStateSource (real mode only)
    sensors: list = field(default_factory=list)   # Sensor instances (rate-gated)
    occluders: list = field(default_factory=list)  # synced concave self-occluders
    clock: SimClock = field(default_factory=SimClock)
    # Continuous sensor stepping is OPT-IN: cameras (EGL renders) are expensive, so
    # by default sensors are only sampled ON DEMAND (sim.sensor(name).sample()).
    # An operator/autonomy run sets this True to publish feeds continuously.
    step_sensors: bool = False

    def set_intent(self, intent: RoveControl) -> None:
        self.intent = intent

    def step_control(self, ticks: int = 1) -> None:
        """Advance `ticks` control periods via the active driver (mock physics or
        real kinematic sync), ticking the clock per tick and -- when enabled --
        the rate-gated sensors."""
        dt = 1.0 / self.control_hz
        for _ in range(ticks):
            self.driver.step_control(1, self.intent)
            if self.occluders:                 # keep the mast occluder on the robot
                from .robot.occluder import sync_occluders
                sync_occluders(self.robot, self.occluders)
            self.clock.tick(dt)
            if self.step_sensors:
                for s in self.sensors:
                    s.update(dt)

    def sensor(self, name: str):
        """The sensor instance by its profile `as:` name (or None)."""
        return next((s for s in self.sensors if s.name == name), None)

    def run_for(self, seconds: float, intent: RoveControl | None = None) -> None:
        if intent is not None:
            self.set_intent(intent)
        self.step_control(round(seconds * self.control_hz))

    def disconnect(self) -> None:
        """Tear down the state source (real mode) then the engine."""
        if self.source is not None:
            self.source.stop()
        self.engine.disconnect()

    # -- arm pose store / path planning (delegates to the arm actuator) ------
    def _arm(self):
        return next((a for a in self.actuators if a.intent_field == "ovis"), None)

    def store_arm_pose(self, name):
        a = self._arm()
        return a.store_pose(name) if a else None

    def arm_goto_pose(self, name, speed=None):
        """Start a planned joint-space move to a stored pose. Returns False if no
        arm or unknown pose. Call step_control() until arm_planning() is False."""
        a = self._arm()
        return bool(a and a.goto_pose(name, speed))

    def arm_planning(self) -> bool:
        a = self._arm()
        return bool(a and a.planning)

    def arm_poses(self):
        a = self._arm()
        return list(a.poses) if a else []


def build(profile: str, mode: str = "headless", world: str = "mock",
          control_hz: float = 50.0, safety: bool = True,
          state_source=None, world_overrides=None, step_sensors: bool = False,
          egl: bool = True, solver_iterations: int | None = None,
          timestep: float | None = None) -> Sim:
    prof = load_profile(resolve_profile(profile))
    # egl=False -> CPU-only pybullet (no GPU/VRAM): for processes that never call
    # getCameraImage (physics server, lidar raycast worker, and the pyrender camera
    # worker whose GPU work is done by pyrender, not pybullet). On a tiny 2 GB GPU
    # this keeps the ONLY GPU context the single pyrender renderer.
    ecfg = EngineConfig(mode=mode, egl=egl)
    if solver_iterations is not None:
        ecfg.solver_iterations = int(solver_iterations)
    if timestep is not None:
        ecfg.timestep = float(timestep)
    engine = Engine(ecfg).connect()
    # World seam owns gravity/ground (mock physics) or the geometric world model
    # (real). `world=` overrides the profile's `world.use` (default 'mock');
    # `world_overrides` injects extra world spec (e.g. terrain) without editing
    # the profile.
    world_obj = build_world(prof, engine, override=world, overrides=world_overrides)
    robot = loader.load(engine, prof)
    # drop the robot onto terrain if a terrain mesh replaced the ground plane
    if world == "mock" and getattr(world_obj, "terrain_id", None) is not None:
        z = world_obj.drop_point(0.0, 0.0)
        if z is not None:
            pos, orn = p.getBasePositionAndOrientation(robot.body_id)
            p.resetBasePositionAndOrientation(
                robot.body_id, [pos[0], pos[1], z + 0.45], orn)
    # Soft foliage colliders are LIDAR-ONLY: disable their physics contact with the
    # robot so it drives THROUGH leaves while the lidar still returns on them.
    for fid in getattr(world_obj, "foliage_ids", []):
        p.setCollisionFilterPair(fid, robot.body_id, -1, -1, 0)
        for j in range(p.getNumJoints(robot.body_id)):
            p.setCollisionFilterPair(fid, robot.body_id, -1, j, 0)

    actuators = build_actuators(prof.actuators, robot)
    caps = capabilities.derive(prof)

    # Self-collision guard: keep the flippers (worm gear) and arm (collaborative)
    # from driving within a keep-out margin of any other part, checked on the real
    # link meshes. Config-gated (profile `safety:`); on by default. Attached to any
    # actuator exposing a `guard` slot.
    guard = None
    scfg = prof.raw.get("safety", {})
    if safety and scfg.get("enabled", True):
        from .robot.safety import SelfCollisionGuard
        mesh_dir = os.path.join(os.path.dirname(prof.model.path), "meshes")
        guard = SelfCollisionGuard(
            robot, mesh_dir,
            margin=float(scfg.get("self_collision_margin_m", 0.0254)))
        if guard.enabled:
            for a in actuators:
                if hasattr(a, "guard"):
                    a.guard = guard

    # Painted ground-friction raster (mock world): the brush-track actuator reads
    # per-contact mu from it. Attached to any actuator exposing the slot.
    if getattr(world_obj, "friction", None) is not None:
        for a in actuators:
            if hasattr(a, "friction_field"):
                a.friction_field = world_obj.friction

    # Real mode is driven by telemetry, not actuators: build a RobotStateSource
    # (default ManualStateSource -- offline, hand/replay driven) and the kinematic
    # SyncDriver. Mock mode keeps the physics MockDriver and no source.
    source = None
    if world == "real":
        source = state_source if state_source is not None \
            else build_state_source(prof, robot=robot)
        source.start()
    driver = build_driver(world, engine=engine, robot=robot, actuators=actuators,
                          world=world_obj, control_hz=control_hz,
                          source=source, guard=guard)

    # Self-occluders: concave colliders (the real sensor-mast mesh) that block the
    # robot's own ray sensors so the lidar doesn't shoot through its tower. Built
    # from the profile's `sensor_occluders` link list, synced each control tick.
    occluders = []
    occ_links = prof.raw.get("sensor_occluders", [])
    if occ_links:
        from .robot.occluder import build_occluders
        mesh_dir = os.path.join(os.path.dirname(prof.model.path), "meshes")
        cache = os.path.join(os.path.dirname(__file__), "..", "assets", "occluders")
        occluders = build_occluders(robot, mesh_dir, occ_links, cache)

    # Sensors: instantiate the profile's declared sensors against their mount links
    # (camera / livox / vn300). They rate-gate themselves; Sim.step_control ticks
    # them. Transports default to none (readings still computed + cached on .last).
    clock = SimClock()
    sensor_list = build_sensors(prof.sensors, robot, clock, engine=engine)
    occ_ids = {o.body for o in occluders}
    for s in sensor_list:                        # cameras: never render the occluder
        if hasattr(s, "mask_ids"):               # proxy (it'd be a box at the lens)
            s.mask_ids |= occ_ids
    bit = 0x10                                   # per-Livox own-housing group bit
    for s in sensor_list:                        # lidars: occlude on the mast mesh
        if not hasattr(s, "occluder_ids"):
            continue
        s.occluder_ids |= occ_ids
        # "rays cast from the sensor centroid, cannot collide with the sensor's own
        # geometry": put each Livox housing in its own collision group and mask it
        # out of that Livox's rays -- so the ray passes through itself but still
        # hits the cage / the other Livox / the world.
        hidx = robot.link_index.get(s.link)
        if hidx is not None and hidx >= 0:
            p.setCollisionFilterGroupMask(robot.body_id, hidx, bit, -1)
            s.ray_mask = ~bit                    # all groups except our own housing
            bit <<= 1

    sim = Sim(engine=engine, robot=robot, actuators=actuators, caps=caps,
              control_hz=control_hz, guard=guard, world=world_obj,
              driver=driver, source=source, sensors=sensor_list, clock=clock,
              step_sensors=step_sensors, occluders=occluders)
    driver.settle(0.5)             # physics settle in mock; no-op kinematic in real
    if occluders:                  # settle moved the robot -> realign the occluders
        from .robot.occluder import sync_occluders
        sync_occluders(robot, occluders)
    return sim
