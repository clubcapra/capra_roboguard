"""Scene save / load / sync: serialize the WORLD (not the robot model) so an
operator view and the world model can share state, and a run can be reproduced.

A Scene is the environment-and-state half of a sim instance:

  * terrain      -- the `terrain:` spec (a REFERENCE to the GLB + flags), not the
                    mesh; reloaded through the normal converter cache on build.
  * friction     -- the painted FrictionField, inline (origin/cell/mu raster).
  * robot        -- base pose + every movable joint value (so a synced twin or a
                    reload lands in the exact same configuration).
  * objects      -- inserted SceneObjects (static obstacles / SAR targets): the
                    mock analogue of RealWorld's perceived Detections.

`capture_scene(sim)` reads the live state into a Scene; `apply_scene(sim, scene)`
slams a built sim to it (idempotent -- objects upsert by id, so the same call is
both "load" and "sync tick"). The terrain ref drives `build()` (it must exist
before the bodies do), so `Scene.build_overrides()` feeds runtime.build and
`load_scene_sim()` ties the two together. Persistence is plain JSON, matching the
FrictionField convention; the same dict is what you'd ship over a transport to
sync a second process.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import pybullet as p

SCENE_VERSION = 1


@dataclass
class SceneObject:
    """A placed body: static obstacle, prop, or SAR target/victim. The mock-world
    twin of perception.Detection (same pose/extents/cls fields) so a perceived
    object and an authored one round-trip through the same path."""
    id: str
    pose: Sequence[float] = (0.0, 0.0, 0.0)        # world xyz
    orn: Sequence[float] = (0.0, 0.0, 0.0, 1.0)    # world quaternion xyzw
    shape: str = "box"                             # box | sphere | cylinder | mesh
    extents: Sequence[float] = (0.3, 0.3, 0.3)     # full extents (m); sphere/cyl use [0],[2]
    rgba: Sequence[float] = (0.85, 0.2, 0.2, 1.0)
    mass: float = 0.0                              # 0 = static
    cls: str = "object"                            # label (e.g. "victim", "barrel")
    mesh: Optional[str] = None                     # file path when shape == "mesh"
    meta: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {"id": self.id, "pose": list(self.pose), "orn": list(self.orn),
                "shape": self.shape, "extents": list(self.extents),
                "rgba": list(self.rgba), "mass": self.mass, "cls": self.cls,
                "mesh": self.mesh, "meta": self.meta}

    @classmethod
    def from_dict(cls, d: dict) -> "SceneObject":
        return cls(id=d["id"], pose=d.get("pose", (0, 0, 0)),
                   orn=d.get("orn", (0, 0, 0, 1)), shape=d.get("shape", "box"),
                   extents=d.get("extents", (0.3, 0.3, 0.3)),
                   rgba=d.get("rgba", (0.85, 0.2, 0.2, 1.0)),
                   mass=float(d.get("mass", 0.0)), cls=d.get("cls", "object"),
                   mesh=d.get("mesh"), meta=dict(d.get("meta", {})))

    @classmethod
    def from_detection(cls, det) -> "SceneObject":
        """Bridge a perception.Detection into a SceneObject (so RealWorld
        detections and authored objects share the scene format)."""
        return cls(id=det.id, pose=det.pose, orn=det.orn, shape="box",
                   extents=det.extents, cls=det.cls, meta=dict(det.meta))


def make_shapes(obj: SceneObject) -> Tuple[int, int]:
    """Create (collision, visual) pybullet shape ids for a SceneObject. Either may
    be -1 (no collision for a massless marker is fine; visual always made)."""
    e = list(obj.extents)
    rgba = list(obj.rgba)
    if obj.shape == "sphere":
        r = e[0] / 2.0
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=rgba)
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=r)
    elif obj.shape == "cylinder":
        r, h = e[0] / 2.0, e[2]
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=rgba)
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h)
    elif obj.shape == "mesh" and obj.mesh:
        vis = p.createVisualShape(p.GEOM_MESH, fileName=obj.mesh, meshScale=e, rgbaColor=rgba)
        col = p.createCollisionShape(p.GEOM_MESH, fileName=obj.mesh, meshScale=e)
    else:  # box (default)
        half = [c / 2.0 for c in e]
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    return col, vis


@dataclass
class Scene:
    terrain: Optional[Dict[str, Any]] = None       # the `terrain:` build spec
    friction: Optional[Dict[str, Any]] = None      # FrictionField.to_dict()
    robot_pose: Optional[Tuple[Sequence[float], Sequence[float]]] = None  # (xyz, xyzw)
    robot_joints: Dict[str, float] = field(default_factory=dict)
    objects: List[SceneObject] = field(default_factory=list)
    meta: Dict[str, Any] = field(default_factory=dict)

    # -- (de)serialization --------------------------------------------------
    def to_dict(self) -> dict:
        return {"version": SCENE_VERSION, "terrain": self.terrain,
                "friction": self.friction,
                "robot_pose": [list(self.robot_pose[0]), list(self.robot_pose[1])]
                if self.robot_pose else None,
                "robot_joints": self.robot_joints,
                "objects": [o.to_dict() for o in self.objects], "meta": self.meta}

    @classmethod
    def from_dict(cls, d: dict) -> "Scene":
        rp = d.get("robot_pose")
        return cls(terrain=d.get("terrain"), friction=d.get("friction"),
                   robot_pose=(tuple(rp[0]), tuple(rp[1])) if rp else None,
                   robot_joints=dict(d.get("robot_joints", {})),
                   objects=[SceneObject.from_dict(o) for o in d.get("objects", [])],
                   meta=dict(d.get("meta", {})))

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    @classmethod
    def from_json(cls, s: str) -> "Scene":
        return cls.from_dict(json.loads(s))

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path: str) -> "Scene":
        with open(path) as f:
            return cls.from_dict(json.load(f))

    def build_overrides(self) -> Dict[str, Any]:
        """world_overrides for runtime.build() so a fresh sim comes up with this
        scene's terrain + friction grid before apply_scene() sets the dynamics."""
        ov: Dict[str, Any] = {}
        if self.terrain:
            ov["terrain"] = dict(self.terrain)
        if self.friction:
            ov["friction"] = {"origin": self.friction["origin"],
                              "cell": self.friction["cell"],
                              "extent": [self.friction["nx"] * self.friction["cell"],
                                         self.friction["ny"] * self.friction["cell"]],
                              "default": self.friction["default"]}
        return ov


def capture_scene(sim, meta: Optional[dict] = None) -> Scene:
    """Read the live sim state into a Scene."""
    world = sim.world
    pose = None
    joints: Dict[str, float] = {}
    if getattr(sim.robot, "body_id", None) is not None:
        pos, orn = p.getBasePositionAndOrientation(sim.robot.body_id)
        pose = (tuple(pos), tuple(orn))
        for jname, idx in getattr(sim.robot, "joint_index", {}).items():
            joints[jname] = float(p.getJointState(sim.robot.body_id, idx)[0])
    fric = world.friction.to_dict() if getattr(world, "friction", None) else None
    terrain = dict(world.spec.get("terrain")) if world.spec.get("terrain") else None
    objs: List[SceneObject] = list(getattr(world, "scene_objects", []))
    return Scene(terrain=terrain, friction=fric, robot_pose=pose,
                 robot_joints=joints, objects=objs, meta=dict(meta or {}))


def apply_scene(sim, scene: Scene) -> None:
    """Slam a built sim to `scene`. Idempotent: objects upsert by id, so this is
    both the LOAD path and a SYNC tick. Terrain is assumed already built (via
    Scene.build_overrides); only dynamic state is applied here."""
    world = sim.world
    bid = getattr(sim.robot, "body_id", None)
    if scene.robot_pose and bid is not None:
        pos, orn = scene.robot_pose
        p.resetBasePositionAndOrientation(bid, list(pos), list(orn))
    if bid is not None:
        for jname, val in scene.robot_joints.items():
            idx = getattr(sim.robot, "joint_index", {}).get(jname)
            if idx is not None:
                p.resetJointState(bid, idx, float(val))
    if scene.friction and getattr(world, "friction", None) is not None:
        world.friction.load_dict(scene.friction)
    # objects (mock world only -- it owns spawn_object / scene_objects)
    if hasattr(world, "spawn_object"):
        want = {o.id for o in scene.objects}
        for oid in list(getattr(world, "objects", {})):
            if oid not in want:
                world.remove_object(oid)
        for o in scene.objects:
            world.spawn_object(o)
        world.scene_objects = list(scene.objects)


def load_scene_sim(path_or_scene, profile: str, mode: str = "headless",
                   **build_kwargs):
    """Build a sim from a saved Scene and apply its dynamic state. Returns the Sim.
    `profile` is the robot profile (the scene stores the world, not the robot)."""
    from .. import runtime
    scene = (path_or_scene if isinstance(path_or_scene, Scene)
             else Scene.load(path_or_scene))
    sim = runtime.build(profile, mode=mode, world="mock",
                        world_overrides=scene.build_overrides(), **build_kwargs)
    apply_scene(sim, scene)
    return sim


# Static trunk obstacles on the road, so autonomy can drive at them and the lidar
# forward-hazard reflex stops in time. Spawned in EVERY sim instance that raycasts/
# collides against them (sim_server physics AND each lidar_worker). Default demo
# positions; override with env ROVE_OBSTACLES="x,y;x,y;..." to place them anywhere
# on the drivable road.
# ON the East road centerline (y≈-6, where GoTo East drives on solid ground) -- NOT
# south of spawn (that's off the road / the no-collision void). Robot drives East
# into them and the lidar reflex stops before the first trunk.
DEMO_TREES = [(10.0, -6.0), (11.5, -6.0)]


def parse_obstacles_env(s):
    """Parse ROVE_OBSTACLES "x,y;x,y;..." -> [(x,y), ...] (bad entries skipped)."""
    out = []
    for part in (s or "").split(";"):
        part = part.strip()
        if not part:
            continue
        try:
            x, y = part.split(",")
            out.append((float(x), float(y)))
        except ValueError:
            pass
    return out


def spawn_obstacles(world, positions=None, radius=0.25, height=4.0) -> int:
    """Upsert cylinder trunks at `positions` (default DEMO_TREES) into `world`."""
    positions = DEMO_TREES if positions is None else positions
    for i, (tx, ty) in enumerate(positions):
        world.spawn_object(SceneObject(
            id=f"obs_{i}", pose=(tx, ty, height / 2.0), shape="cylinder",
            extents=(radius * 2, radius * 2, height),
            rgba=(0.45, 0.30, 0.15, 1.0), mass=0.0, cls="tree"))
    return len(positions)


def spawn_obstacles_from_env(world) -> int:
    """ROVE_OBSTACLES="x,y;..." (explicit) wins, else ROVE_SPAWN_TREES=1 spawns the
    default demo trunks. Returns the count (0 if neither set)."""
    import os
    env = os.environ.get("ROVE_OBSTACLES")
    if env:
        pos = parse_obstacles_env(env)
        return spawn_obstacles(world, pos) if pos else 0
    if os.environ.get("ROVE_SPAWN_TREES"):
        return spawn_obstacles(world)
    return 0


def spawn_demo_trees(world) -> int:           # back-compat alias
    return spawn_obstacles(world)
