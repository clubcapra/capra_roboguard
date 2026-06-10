"""Self-occluders: static concave colliders that block the robot's own sensors.

PyBullet won't put a concave mesh on a DYNAMIC link, and a convex hull of the
sensor mast ("pole") is a solid box that over-occludes the lidar. So for links
that must occlude a ray-based sensor with their real shape (the open frame of the
mast, so the laser casts proper post-shadows through the gaps), we build a
SEPARATE static body from the link's real GLB mesh with GEOM_FORCE_CONCAVE_TRIMESH,
disable its physics contact with the robot (it would otherwise shove it), and
sync its pose to the link every control tick. The lidar excludes these body ids
from its cloud, so they OCCLUDE (stop the ray) without ever appearing as points.

Declared per-profile: `sensor_occluders: [pole]`.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List

import pybullet as p

from tools.convert_meshes import convert_glb


@dataclass
class Occluder:
    body: int          # static concave collision body
    link: int          # robot link index it mirrors


def _world_pose(body_id: int, occ: "Occluder"):
    # The collision mesh (convert_glb output) is already in the URDF LINK frame
    # (local z 0..h), exactly how the loader places the link's collision -- so the
    # occluder just mirrors the link's world frame. (Do NOT add the visual-shape
    # offset; PyBullet bakes a separate render transform into the visual that does
    # not apply to collision and would shove the mesh ~0.5 m off the sensors.)
    st = p.getLinkState(body_id, occ.link, computeForwardKinematics=True)
    return st[4], st[5]


def build_occluders(robot, mesh_dir: str, link_names, cache_dir: str) -> List[Occluder]:
    """Create a synced concave occluder for each named robot link that has a GLB
    mesh. Returns the list (empty if none)."""
    out: List[Occluder] = []
    for name in (link_names or []):
        li = robot.link_index.get(name)
        if li is None:
            continue
        glb = os.path.join(mesh_dir, name + ".glb")
        if not os.path.exists(glb):
            continue
        obj = os.path.join(cache_dir, f"occluder_{name}.obj")
        if not os.path.exists(obj):
            os.makedirs(cache_dir, exist_ok=True)
            convert_glb(glb, obj)                       # faithful (concave) mesh
        col = p.createCollisionShape(p.GEOM_MESH, fileName=obj,
                                     flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        # a zero-size visual so PyBullet does NOT draw the collision mesh as a
        # fallback -- the occluder is a raycast-only proxy and must be invisible
        # (else it renders as a box right in front of every camera).
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=1e-5,
                                  rgbaColor=[0, 0, 0, 0])
        occ = Occluder(body=-1, link=li)
        wp, wo = _world_pose(robot.body_id, occ)
        occ.body = p.createMultiBody(0, baseCollisionShapeIndex=col,
                                     baseVisualShapeIndex=vis,
                                     basePosition=wp, baseOrientation=wo)
        # raycastable, but NO physics contact with the robot (it overlaps it)
        p.setCollisionFilterPair(occ.body, robot.body_id, -1, -1, 0)
        for j in range(p.getNumJoints(robot.body_id)):
            p.setCollisionFilterPair(occ.body, robot.body_id, -1, j, 0)
        out.append(occ)
    return out


def sync_occluders(robot, occluders: List[Occluder]) -> None:
    """Mirror each occluder body onto its robot link's current world pose."""
    for o in occluders:
        wp, wo = _world_pose(robot.body_id, o)
        p.resetBasePositionAndOrientation(o.body, wp, wo)
