"""MockWorld: the physics environment (gravity + ground + terrain + friction).

This is the world setup that used to live inside `Engine.connect()`, moved here
verbatim so mock mode behaves byte-identically to before. Terrain and the raster
friction field attach behind `build()` -- purely additive content, no
driver/runtime changes.
"""
from __future__ import annotations

import os

import pybullet as p

from .base import World, register


@register("mock")
class MockWorld(World):
    def build(self) -> "MockWorld":
        cfg = self.engine.cfg
        spec = self.spec
        gravity = float(spec.get("gravity", cfg.gravity))
        timestep = float(spec.get("timestep", cfg.timestep))
        p.setGravity(0, 0, gravity)
        p.setTimeStep(timestep)
        p.setRealTimeSimulation(1 if cfg.real_time else 0)
        # Solver-iteration tuning (engine perf, see EngineConfig.solver_iterations).
        # Profile may override via `world.solver_iterations`.
        p.setPhysicsEngineParameter(
            numSolverIterations=int(spec.get("solver_iterations",
                                             getattr(cfg, "solver_iterations", 10))))

        self.terrain_id: int | None = None       # terrain COLLISION body (mesh mode)
        self.terrain_vis_id: int | None = None    # terrain visual body
        self.friction = None                      # FrictionField or None
        self.objects: dict = {}                    # scene-object id -> body_id
        self.scene_objects: list = []              # SceneObject specs (for capture)
        self._build_terrain()                     # may set terrain ids
        self._build_friction(cfg)                 # may set self.friction

        # Ground plane: the robust drivable surface. Kept unless the terrain
        # provides its own collision mesh (collision: mesh). Explicit
        # `ground_plane` in the spec always wins.
        want_ground = spec.get("ground_plane",
                               cfg.ground_plane and self.terrain_id is None)
        if want_ground:
            floor_friction = float(spec.get("floor_friction", cfg.floor_friction))
            self.ground_id = p.loadURDF("plane.urdf")
            p.changeDynamics(self.ground_id, -1, lateralFriction=floor_friction)
        return self

    # -- friction field -----------------------------------------------------
    def _build_friction(self, cfg) -> None:
        """Build a paintable ground-friction raster from the `friction:` block.

        `friction.load` => load a saved field; otherwise an empty field at the
        nominal floor friction (paint it via the GUI or FrictionField.paint)."""
        fspec = self.spec.get("friction")
        if not fspec:
            return
        from .friction import FrictionField
        load = fspec.get("load")
        if load:
            if not os.path.isabs(load):
                root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
                load = os.path.join(root, load)
            self.friction = FrictionField.load(load)
        else:
            self.friction = FrictionField(
                origin=fspec.get("origin", (-25.0, -25.0)),
                extent=fspec.get("extent", (50.0, 50.0)),
                cell=float(fspec.get("cell", 0.25)),
                default=float(fspec.get("default", cfg.floor_friction)))

    # -- terrain ------------------------------------------------------------
    def _resolve_source(self, src: str) -> str | None:
        if os.path.isabs(src):
            return src if os.path.exists(src) else None
        bases = []
        if self.profile is not None:
            bases.append(os.path.dirname(self.profile.model.path))       # URDF dir
        root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        bases += [root, os.getcwd()]
        for b in bases:
            cand = os.path.normpath(os.path.join(b, src))
            if os.path.exists(cand):
                return cand
        return None

    def _build_terrain(self) -> None:
        """Load the terrain GLB as the FULL scene (mesh-first): a ground-only
        concave collider the robot drives on + colour-grouped visual bodies for
        the whole environment (ground, grass, rock, trunks, foliage). The terrain
        is intentionally rough/hilly; the converter recenters a flat patch to the
        origin so the robot spawns on solid ground. Set `terrain.texture: true` to
        also GPU-texture the visual bodies (perf permitting)."""
        import json
        from . import terrain

        tspec = self.spec.get("terrain")
        if not tspec:
            return
        src = self._resolve_source(tspec.get("source", ""))
        if not src:
            return
        root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        out_dir = os.path.join(root, "assets", "terrain")
        manifest = terrain.build_assets(src, out_dir,
                                        int(tspec.get("collision_faces", 20000)))
        data = json.load(open(manifest))
        # The manifest stores ABSOLUTE asset paths; if the stack was copied from
        # another machine those point at the old root. Rebase every cached path
        # onto the real cache dir by its stable suffix (everything lives under
        # .../assets/terrain/<name>/), so a copied cache works without a rebuild.
        cache_dir = os.path.dirname(manifest)
        cache_name = os.path.basename(cache_dir)
        def _local(pth):
            if not pth:
                return pth
            parts = pth.replace("\\", "/").split("/" + cache_name + "/", 1)
            return os.path.join(cache_dir, parts[1]) if len(parts) == 2 else pth
        data["collision"] = _local(data["collision"])
        for v in data.get("visual", []):
            v["obj"] = _local(v.get("obj"))
            v["tex"] = _local(v.get("tex"))
            v["collision_obj"] = _local(v.get("collision_obj"))
        scale = float(tspec.get("scale", 1.0))
        want_tex = bool(tspec.get("texture", False))

        # ground-only collision (invisible); the visual comes from the bodies below
        col = p.createCollisionShape(p.GEOM_MESH, fileName=data["collision"],
                                     meshScale=[scale] * 3,
                                     flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        self.terrain_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                            basePosition=[0, 0, 0])
        p.changeDynamics(self.terrain_id, -1,
                         lateralFriction=float(tspec.get("friction", 1.0)))
        lo = p.getAABB(self.terrain_id)[0][2]
        self.ground_id = p.loadURDF("plane.urdf", [0, 0, lo - 5.0])   # deep catch
        p.changeVisualShape(self.ground_id, -1, rgbaColor=[0, 0, 0, 0])

        # full-scene visual: one body per source material. With texture on, the
        # body is white (rgba [1,1,1,1]) so the UV-mapped texture isn't tinted;
        # with texture off it's the material's flat fallback colour.
        #
        # Z-STAGGER to kill z-fighting: the GLB layers several near-coplanar ground
        # materials (dirt road + dry grass + fallen leaves + mud) at the same
        # height. As separate bodies they fight for depth -> jagged shimmer (very
        # visible once textured, e.g. yellow dry-grass tearing through grey road).
        # We lift each layer a few mm by a category render-order so the decals sit
        # JUST above the base ground instead of fighting. Visual only -- the robot
        # drives on the collision mesh, which is untouched.
        _Z_LAYER = {"ground": 0, "rock": 1, "grass": 2, "foliage": 3}
        self.terrain_vis_ids = []
        for i, v in enumerate(data["visual"]):
            textured = want_tex and v.get("tex")
            rgba = [1, 1, 1, 1] if textured else v["color"]
            vis = p.createVisualShape(p.GEOM_MESH, fileName=v["obj"],
                                      meshScale=[scale] * 3, rgbaColor=rgba)
            # base category tier (0/1/2/3 -> 0/4/8/12 mm) + a sub-mm per-body
            # epsilon so two bodies in the same tier still separate.
            z = _Z_LAYER.get(v.get("category"), 0) * 0.004 + i * 0.0003
            bid = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                                    basePosition=[0, 0, z])
            if textured:
                try:
                    p.changeVisualShape(bid, -1, textureUniqueId=p.loadTexture(v["tex"]))
                except Exception:
                    pass
            self.terrain_vis_ids.append(bid)
        self.terrain_vis_id = self.terrain_vis_ids[0] if self.terrain_vis_ids else None

        # Collision promotion: the terrain collider is GROUND-ONLY, so by default a
        # lidar/robot passes through everything above it. We promote selected
        # categories to static concave colliders, in two flavours:
        #   * collide_categories  (HARD, default TRUNKS) -- robot AND lidar collide:
        #     solid obstacles the robot can't drive through, lidar returns on them.
        #   * lidar_categories    (SOFT, default FOLIAGE) -- LIDAR collides, robot
        #     PASSES THROUGH: the cutout canopy/leaves return lidar points through
        #     their gaps (the transparent-gap / leaf-hit model) but are not a wall
        #     to drive into (leaves are soft). Robot contact is disabled in runtime.
        self.obstacle_ids = []          # hard (robot + lidar)
        self.foliage_ids = []           # soft (lidar only; robot pass-through)
        hard = set(tspec.get("collide_categories", ("trunk",)))
        soft = set(tspec.get("lidar_categories", ("foliage",)))
        for v in data["visual"]:
            cat = v.get("category")
            if cat not in hard and cat not in soft:
                continue
            # foliage collides on its alpha-CUTOUT mesh (rays pass through the leaf
            # gaps) even though the VISUAL body is the opaque card; trunks use their
            # own mesh. Falls back to the visual obj if no cutout was baked.
            cobj = v.get("collision_obj") or v["obj"]
            oc = p.createCollisionShape(p.GEOM_MESH, fileName=cobj,
                                        meshScale=[scale] * 3,
                                        flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
            ob = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=oc,
                                   basePosition=[0, 0, 0])
            p.changeDynamics(ob, -1, lateralFriction=0.8)
            (self.obstacle_ids if cat in hard else self.foliage_ids).append(ob)

    # -- scene objects ------------------------------------------------------
    def spawn_object(self, obj) -> int:
        """Materialise a SceneObject (static obstacle / SAR target) as a body and
        track it by id. Re-spawning the same id replaces the old body (upsert) --
        which is exactly what a scene SYNC needs. Returns the pybullet body id."""
        from .scene import make_shapes
        if obj.id in self.objects:
            self.remove_object(obj.id)
        col, vis = make_shapes(obj)
        bid = p.createMultiBody(
            baseMass=float(obj.mass),
            baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=list(obj.pose), baseOrientation=list(obj.orn))
        self.objects[obj.id] = bid
        self.scene_objects = [o for o in self.scene_objects if o.id != obj.id]
        self.scene_objects.append(obj)
        return bid

    def remove_object(self, oid: str) -> None:
        bid = self.objects.pop(oid, None)
        if bid is not None:
            p.removeBody(bid)
        self.scene_objects = [o for o in self.scene_objects if o.id != oid]

    def clear_objects(self) -> None:
        for oid in list(self.objects):
            self.remove_object(oid)

    # -- helpers ------------------------------------------------------------
    def drop_point(self, x: float, y: float, top: float = 80.0) -> float | None:
        """Surface height at (x,y) via a downward ray (for placing the robot on
        terrain). Returns z of the first hit, or None."""
        body = self.terrain_id if self.terrain_id is not None else self.ground_id
        if body is None:
            return None
        hit = p.rayTest([x, y, top], [x, y, -top])[0]
        return None if hit[0] < 0 else hit[3][2]
