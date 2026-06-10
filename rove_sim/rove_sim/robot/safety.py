"""Self-collision clearance guard using the ACTUAL link meshes (FCL).

Real-robot rationale: the flippers are on **worm gears** (non-backdrivable, high
torque -- they'll shear the gear or crush whatever they drive into) and the arm
is a **collaborative** manipulator (it can fold onto itself / the body and damage
itself). The real firmware enforces a keep-out margin; we mirror it so autonomy
developed against the sim never commands a self-destructive pose.

Design:
  * Distance is computed on each link's REAL collision MESH (not the convex hull
    / box the physics uses) via FCL, driven by PyBullet forward kinematics. The
    check is independent of the physics collision shapes, which stay tuned for
    ground contact (treads only, raised Core box).
  * An Allowed-Collision-Matrix is built once at the rest pose: any pair already
    within the margin at rest is a structural adjacency (a joint mount) and is
    ignored; every other pair is checked. Standard MoveIt-style ACM.
  * `blocks(joint, links, sign)` returns True iff a guarded link is within the
    margin AND advancing that joint in `sign` would close the gap further
    (finite-difference probe via resetJointState -> FK -> FCL, then restored).
    Retreat is always allowed, so a guarded joint can back out of a near-miss.

Meshes are decimated for the proximity query (collision distance needs shape, not
detail) so the per-step cost stays Jetson-friendly; the guard is only consulted
for joints actually being commanded to move.
"""
from __future__ import annotations

import os
import numpy as np
import pybullet as p

try:
    import fcl
    import trimesh
    _HAVE = True
except Exception:                      # pragma: no cover
    _HAVE = False


def _bvh(mesh):
    m = fcl.BVHModel()
    m.beginModel(len(mesh.vertices), len(mesh.faces))
    m.addSubModel(np.asarray(mesh.vertices, float), np.asarray(mesh.faces, np.int64))
    m.endModel()
    return m


class SelfCollisionGuard:
    def __init__(self, robot, mesh_dir, margin=0.0254, max_faces=400):
        self.robot = robot
        self.body = robot.body_id
        self.margin = float(margin)
        self.enabled = _HAVE
        self.objs = {}                 # link_name -> [fcl obj, link_idx]
        if not self.enabled:
            return
        for name, idx in robot.link_index.items():
            mesh = self._load(mesh_dir, name)
            if mesh is None:
                continue
            if max_faces and len(mesh.faces) > max_faces:
                try:
                    mesh = mesh.simplify_quadric_decimation(max_faces)
                except Exception:
                    pass
            if len(mesh.faces) == 0:
                continue
            self.objs[name] = [fcl.CollisionObject(_bvh(mesh), fcl.Transform()), idx]
        self.names = list(self.objs)
        self._sync()
        self._build_acm()

    # -- mesh / transforms ---------------------------------------------------
    @staticmethod
    def _load(mesh_dir, name):
        for ext in (".obj", ".glb"):
            f = os.path.join(mesh_dir, name + ext)
            if os.path.exists(f):
                try:
                    m = trimesh.load(f, force="mesh")
                    if m is not None and len(m.faces):
                        return m
                except Exception:
                    return None
        return None

    def _pose(self, idx):
        if idx == -1:
            pos, orn = p.getBasePositionAndOrientation(self.body)
        else:
            ls = p.getLinkState(self.body, idx, computeForwardKinematics=True)
            pos, orn = ls[4], ls[5]       # URDF link frame (not the COM frame)
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        return fcl.Transform(R, np.array(pos))

    def _sync(self):
        for obj, idx in self.objs.values():
            obj.setTransform(self._pose(idx))

    def _dist(self, a, b):
        return fcl.distance(self.objs[a][0], self.objs[b][0],
                            fcl.DistanceRequest(), fcl.DistanceResult())

    def _build_acm(self):
        self.ignore = set()
        for i in range(len(self.names)):
            for j in range(i + 1, len(self.names)):
                a, b = self.names[i], self.names[j]
                if self._dist(a, b) < self.margin:     # structural adjacency
                    self.ignore.add(frozenset((a, b)))

    # -- queries -------------------------------------------------------------
    def min_clearance(self, link_name):
        if link_name not in self.objs:
            return float("inf")
        d = float("inf")
        for o in self.names:
            if o == link_name or frozenset((o, link_name)) in self.ignore:
                continue
            d = min(d, self._dist(link_name, o))
        return d

    def blocks(self, joint_idx, link_names, vel_sign, probe=0.06):
        """True iff a guarded link is within margin and moving `joint_idx` in
        `vel_sign` closes the gap. Restores the joint exactly (no dynamics)."""
        names = [n for n in link_names if n in self.objs]
        if not names or not self.enabled:
            return False
        self._sync()
        d0 = min(self.min_clearance(n) for n in names)
        if d0 >= self.margin:
            return False
        st, vel = p.getJointState(self.body, joint_idx)[:2]
        p.resetJointState(self.body, joint_idx, st + vel_sign * probe, 0.0)
        self._sync()
        d1 = min(self.min_clearance(n) for n in names)
        p.resetJointState(self.body, joint_idx, st, vel)   # restore pos+vel
        self._sync()
        return d1 < d0 - 1e-5
