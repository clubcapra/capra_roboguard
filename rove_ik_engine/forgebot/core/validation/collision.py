"""Mesh-based collision detection via trimesh + python-fcl.

For each link, we build a single trimesh.Trimesh in the link's local frame
that bakes together every collision geometry (primitives become small
triangulated meshes, external meshes are loaded from project.assets). The
resulting world-space mesh per link is registered with trimesh's
CollisionManager (FCL under the hood) which does broad-phase BVH + narrow-
phase GJK and reports exact penetration depth.

Pairs that are adjacent in the kinematic tree (parent/child, or share a
joint) are skipped — they are intentionally touching at the joint.

Falls back to an empty result if trimesh / python-fcl / scipy are missing
(the editor still works, you just don't get collision feedback).
"""

from __future__ import annotations

import io
from dataclasses import dataclass, field
from typing import Any, cast

import numpy as np

from ..kinematics import world_transforms
from ..model import LinkComponent, Project
from ..model.components.link import Geometry


@dataclass(frozen=True)
class CollisionPair:
    a: str  # entity id
    b: str  # entity id
    # The pair-only FCL query is ~20× faster than the contact-extracting
    # one, so we skip narrow-phase. distance / penetration are kept for
    # API compat (callers expect the fields) but report 0 — the boolean
    # presence of the pair is the signal.
    distance: float
    penetration: float


def check_collisions(
    project: Project,
    *,
    joint_values: dict[str, float] | None = None,
    skip_adjacent: bool = True,
) -> list[CollisionPair]:
    """Two-tier mesh collision: convex-hull broad phase, full-mesh narrow.

    Why the split:

      1. The convex hull of each link is small (~50–200 verts) so FCL's
         broad-phase + GJK over the whole scene takes single-digit ms.
      2. Hulls are *over-conservative* for concave parts — a chassis with
         a recess will report touching parts that fit in the recess as
         colliding. To filter those out we re-test each hull-candidate
         pair against the full mesh BVH (early-out, no contact data).

    Most pairs never reach the second tier (their hulls don't overlap).
    Only the handful that do pay the per-pair cost of full-mesh narrow
    phase, and even then it's bounded — FCL early-outs on first hit.

    All BVHs (hull + full mesh) are cached across calls; only transforms
    are pushed each tick.
    """
    cache = _get_or_build_manager(project)
    if cache is None:
        return []
    hull_mgr = cache.manager
    mesh_mgr = cache.mesh_manager
    registered = cache.entity_ids

    worlds = world_transforms(project, joint_values or {})

    # Sync: add new collidable entities, drop removed ones, push transforms.
    current: set[str] = set()
    for eid, e in project.scene.entities.items():
        link = cast(LinkComponent | None, e.get("link"))
        if link is None or not link.collisions:
            continue
        world_mat = worlds.get(eid)
        if world_mat is None:
            continue
        if eid in registered:
            try:
                hull_mgr.set_transform(eid, world_mat)
                mesh_mgr.set_transform(eid, world_mat)
            except KeyError:
                registered.discard(eid)
                continue
        else:
            full = _link_full_mesh(link, project)
            if full is None or len(getattr(full, "vertices", [])) == 0:
                continue
            hull = _to_convex_hull(full)
            hull_mgr.add_object(eid, hull, transform=world_mat)
            mesh_mgr.add_object(eid, full, transform=world_mat)
            registered.add(eid)
        current.add(eid)

    for stale in registered - current:
        for mgr in (hull_mgr, mesh_mgr):
            try:
                mgr.remove_object(stale)
            except Exception:
                pass
        registered.discard(stale)

    if len(registered) < 2:
        return []

    # Tier 1: hull broad-phase. Pair-only, no contact data.
    is_collision, names = hull_mgr.in_collision_internal(return_names=True)
    if not is_collision:
        return []

    # Tier 2: full-mesh confirmation. Skip pairs whose hulls overlap but
    # whose actual triangle meshes don't.
    out: list[CollisionPair] = []
    seen: set[tuple[str, str]] = set()
    for pair in names:
        a, b = sorted(pair)
        if a == b:
            # FCL's broad-phase can fire the callback with an object paired
            # against itself when the BVT has been updated mid-traversal —
            # not a real collision.
            continue
        key = (a, b)
        if key in seen:
            continue
        if skip_adjacent and _is_adjacent(project, a, b):
            continue
        seen.add(key)
        if not _meshes_collide(mesh_mgr, a, b):
            continue
        out.append(CollisionPair(a=a, b=b, distance=0.0, penetration=0.0))
    return out


# ---- per-link mesh assembly ----


def _link_full_mesh(link: LinkComponent, project: Project) -> Any | None:
    """Concatenate every collision geometry of `link` into one Trimesh in
    the link's local frame. The hull is derived from this — see
    `_to_convex_hull`."""
    import trimesh  # type: ignore[import-untyped]

    parts: list[Any] = []
    for geom in link.collisions:
        m = _geom_to_mesh(geom, project)
        if m is None or len(m.vertices) == 0:
            continue
        parts.append(m)
    if not parts:
        return None
    return parts[0] if len(parts) == 1 else trimesh.util.concatenate(parts)


def _meshes_collide(mesh_mgr: Any, a: str, b: str) -> bool:
    """Direct fcl.collide on the full meshes of `a` and `b`. Early-out:
    one contact, no contact data — we just want yes/no."""
    try:
        import fcl  # type: ignore[import-untyped]
    except ImportError:
        return True  # If FCL is missing somehow, conservatively keep the pair.
    obj_a = mesh_mgr._objs.get(a, {}).get("obj")
    obj_b = mesh_mgr._objs.get(b, {}).get("obj")
    if obj_a is None or obj_b is None:
        return True  # Treat as collision if we can't verify.
    request = fcl.CollisionRequest(num_max_contacts=1, enable_contact=False)
    result = fcl.CollisionResult()
    fcl.collide(obj_a, obj_b, request, result)
    return bool(result.is_collision)


def _to_convex_hull(mesh: Any) -> Any:
    """Reduce a mesh to its convex hull. Falls back to the original mesh on
    failure (e.g., colinear/coplanar vertices that can't produce a hull)."""
    try:
        hull = mesh.convex_hull
    except Exception:
        return mesh
    if hull is None or len(getattr(hull, "vertices", [])) < 4:
        return mesh
    return hull


def _geom_to_mesh(geom: Geometry, project: Project) -> Any | None:
    """Build a Trimesh for one collision geometry, with `geom.origin`
    transform applied."""
    import trimesh  # type: ignore[import-untyped]

    p = geom.primitive_params or {}
    mesh: Any | None = None
    if geom.primitive == "box":
        mesh = trimesh.creation.box(
            extents=[
                float(p.get("x", 1.0)),
                float(p.get("y", 1.0)),
                float(p.get("z", 1.0)),
            ]
        )
    elif geom.primitive == "sphere":
        mesh = trimesh.creation.icosphere(
            subdivisions=2, radius=float(p.get("radius", 1.0))
        )
    elif geom.primitive == "cylinder":
        mesh = trimesh.creation.cylinder(
            radius=float(p.get("radius", 0.5)),
            height=float(p.get("length", 1.0)),
            sections=24,
        )
    elif geom.mesh:
        mesh = _load_mesh_cached(geom.mesh, project)

    if mesh is None:
        return None

    transform = _origin_matrix(geom)
    if transform is not None:
        mesh = mesh.copy()
        mesh.apply_transform(transform)
    return mesh


def _origin_matrix(geom: Geometry) -> np.ndarray | None:
    """4x4 from geom.origin (translation) + geom.origin_rotation (xyzw quat).
    Returns None if the transform is identity."""
    origin = np.array(geom.origin or (0.0, 0.0, 0.0), dtype=float)
    quat = geom.origin_rotation or (0.0, 0.0, 0.0, 1.0)
    qx, qy, qz, qw = (float(q) for q in quat)
    if (
        np.allclose(origin, 0.0)
        and abs(qw - 1.0) < 1e-9
        and abs(qx) < 1e-9
        and abs(qy) < 1e-9
        and abs(qz) < 1e-9
    ):
        return None
    # Quaternion (xyzw) -> rotation matrix.
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        rot = np.eye(3)
    else:
        s = 2.0 / n
        rot = np.array(
            [
                [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
                [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
                [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
            ]
        )
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = origin
    return T


# ---- caches ----

_MESH_DATA_CACHE: dict[str, Any] = {}  # stem -> trimesh.Trimesh | None


@dataclass
class _CachedManager:
    manager: Any        # CollisionManager over convex hulls (broad-phase)
    mesh_manager: Any   # CollisionManager over full meshes (narrow-phase)
    entity_ids: set[str] = field(default_factory=set)


# Keyed by id(project) so swapping projects (App.open) gives a fresh cache.
# `clear_collision_cache` is called on project load to wipe both maps.
_CM_CACHE: dict[int, _CachedManager] = {}


def clear_collision_cache() -> None:
    """Reset the mesh + manager caches; call when a project loads."""
    _MESH_DATA_CACHE.clear()
    _CM_CACHE.clear()


def _get_or_build_manager(project: Project) -> _CachedManager | None:
    cache = _CM_CACHE.get(id(project))
    if cache is not None:
        return cache
    cm_cls = _collision_manager_cls()
    if cm_cls is None:
        return None
    cache = _CachedManager(manager=cm_cls(), mesh_manager=cm_cls())
    _CM_CACHE[id(project)] = cache
    return cache


def _load_mesh_cached(stem: str, project: Project) -> Any | None:
    if stem in _MESH_DATA_CACHE:
        return _MESH_DATA_CACHE[stem]
    asset = project.assets.mesh_data.get(stem)
    if asset is None:
        _MESH_DATA_CACHE[stem] = None
        return None
    mesh = _parse_mesh(asset.data, asset.suffix)
    _MESH_DATA_CACHE[stem] = mesh
    return mesh


def _parse_mesh(data: bytes, suffix: str) -> Any | None:
    """Load bytes into a single Trimesh. Scenes are flattened into one mesh."""
    try:
        import trimesh  # type: ignore[import-untyped]
    except ImportError:
        return None
    sfx = suffix.lower().lstrip(".")
    try:
        loaded = trimesh.load(io.BytesIO(data), file_type=sfx, process=False, force="mesh")
    except Exception:
        return None
    if loaded is None or not hasattr(loaded, "vertices"):
        return None
    if len(loaded.vertices) == 0 or len(getattr(loaded, "faces", [])) == 0:
        return None
    return loaded


def _collision_manager_cls() -> Any | None:
    """Return trimesh.collision.CollisionManager if all native deps load,
    else None. We test by instantiating + adding a tiny mesh, since FCL/scipy
    failures only surface there."""
    try:
        import trimesh  # type: ignore[import-untyped]
        from trimesh.collision import CollisionManager  # type: ignore[import-untyped]
    except ImportError:
        return None
    try:
        probe = CollisionManager()
        b = trimesh.creation.box(extents=[1, 1, 1])
        probe.add_object("__probe__", b, transform=np.eye(4))
    except Exception:
        return None
    return CollisionManager


# ---- adjacency (unchanged) ----


def _is_adjacent(project: Project, a: str, b: str) -> bool:
    """True if a and b are directly connected (parent/child or share a joint)."""
    ea = project.scene.entities.get(a)
    eb = project.scene.entities.get(b)
    if ea is None or eb is None:
        return False
    if ea.parent == b or eb.parent == a:
        return True
    if ea.parent and project.scene.entities.get(ea.parent) is not None:
        joint_a = project.scene.entities[ea.parent]
        if joint_a.has("joint") and joint_a.parent == b:
            return True
    if eb.parent and project.scene.entities.get(eb.parent) is not None:
        joint_b = project.scene.entities[eb.parent]
        if joint_b.has("joint") and joint_b.parent == a:
            return True
    return False
