"""Local-space BVHs for moving meshes, shared by all LiDARs in a worker.

Ray geometry follows rendered robot links, including openings in the mast.
The requesting sensor's housing is omitted; other robot surfaces still occlude.
Broad-phase ray/AABB tests avoid visiting links outside each ray's segment.
"""

import numpy as np
import pybullet as p
import trimesh
from numba import njit
from embreex.rtcore_scene import EmbreeScene
from embreex.mesh_construction import TriangleMesh
from .mesh_assets import load_mesh
from rove_sim.robot.frames import base_link_pose


@njit(cache=True)
def ray_box_candidates(origins, directions, low, high, distance):
    """Compiled slab loop: no per-link temporary arrays or row reductions."""
    result = np.empty(len(origins), np.int64)
    count = 0
    for ray in range(len(origins)):
        near, far = 0.0, distance[ray]
        for axis in range(3):
            origin, direction = origins[ray, axis], directions[ray, axis]
            if abs(direction) < 1e-10:
                if origin < low[axis] or origin > high[axis]:
                    far = -1.0
                    break
            else:
                a, b = (low[axis] - origin) / direction, (
                    high[axis] - origin
                ) / direction
                near, far = max(near, min(a, b)), min(far, max(a, b))
                if far < near:
                    break
        if far >= near:
            result[count] = ray
            count += 1
    return result[:count]


def primitive(kind, dimensions):
    if kind == p.GEOM_BOX:
        return trimesh.creation.box(extents=dimensions)
    if kind == p.GEOM_SPHERE:
        return trimesh.creation.icosphere(subdivisions=2, radius=dimensions[0])
    if kind in (p.GEOM_CYLINDER, p.GEOM_CAPSULE):
        if kind == p.GEOM_CAPSULE:
            return trimesh.creation.capsule(
                height=dimensions[0], radius=dimensions[1], count=[12, 12]
            )
        return trimesh.creation.cylinder(
            height=dimensions[0], radius=dimensions[1], sections=24
        )
    return None


class MovingRaycaster:
    def __init__(self, robot, world):
        self.robot = robot
        self.world = world
        self.draws = []
        self.objects = {}
        for visual in p.getVisualShapeData(robot.body_id):
            link, kind, dims, path, pos, orn = visual[1:7]
            if kind == p.GEOM_MESH and path:
                mesh = load_mesh(path.decode() if isinstance(path, bytes) else path)
                vertices, faces = mesh["vertices"] * np.asarray(dims), mesh["faces"]
            else:
                mesh = primitive(kind, dims)
                if mesh is None or max(dims) < 1e-4:
                    continue
                vertices, faces = mesh.vertices, mesh.faces
            rotation = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            vertices = vertices @ rotation.T + pos
            self.draws.append(self._bvh(vertices, faces, link))

    @staticmethod
    def _bvh(vertices, faces, link):
        scene = EmbreeScene()
        geometry = TriangleMesh(
            scene, np.ascontiguousarray(vertices[faces], np.float32)
        )
        return {
            "scene": scene,
            "geometry": geometry,
            "link": link,
            "low": np.min(vertices, axis=0),
            "high": np.max(vertices, axis=0),
        }

    def update(self):
        frames = {-1: base_link_pose(self.robot.body_id)}
        links = sorted({draw["link"] for draw in self.draws if draw["link"] >= 0})
        if links:
            frames.update(
                {
                    link: (state[4], state[5])
                    for link, state in zip(
                        links,
                        p.getLinkStates(
                            self.robot.body_id, links, computeForwardKinematics=True
                        ),
                    )
                }
            )
        for draw in self.draws:
            self._pose(draw, *frames[draw["link"]])
        for obj in self.world.scene_objects:
            body = self.world.objects[obj.id]
            if obj.id not in self.objects or self.objects[obj.id]["spec"] != obj:
                if obj.shape == "mesh" and obj.mesh:
                    data = load_mesh(obj.mesh)
                    mesh = trimesh.Trimesh(
                        vertices=data["vertices"] * np.asarray(obj.extents),
                        faces=data["faces"],
                        process=False,
                    )
                elif obj.shape == "sphere":
                    mesh = trimesh.creation.icosphere(
                        subdivisions=2, radius=obj.extents[0] / 2
                    )
                elif obj.shape == "cylinder":
                    mesh = trimesh.creation.cylinder(
                        radius=obj.extents[0] / 2, height=obj.extents[2], sections=24
                    )
                else:
                    mesh = trimesh.creation.box(extents=obj.extents)
                draw = self._bvh(mesh.vertices, mesh.faces, -1)
                draw["spec"] = obj
                self.objects[obj.id] = draw
            draw = self.objects[obj.id]
            draw["body"] = body
            self._pose(draw, *p.getBasePositionAndOrientation(body))
        for oid in set(self.objects) - set(self.world.objects):
            del self.objects[oid]

    @staticmethod
    def _pose(draw, pos, orn):
        draw["pos"] = np.asarray(pos)
        draw["rotation"] = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        centre = (draw["high"] + draw["low"]) / 2
        half = np.abs(draw["rotation"]) @ ((draw["high"] - draw["low"]) / 2)
        centre = draw["rotation"] @ centre + draw["pos"]
        draw["world_low"], draw["world_high"] = centre - half, centre + half

    def cast(self, origins, directions, max_range, omit_link):
        distance = np.full(len(origins), max_range, np.float32)
        bodies = np.full(len(origins), -1, np.int32)
        for draw in [*self.draws, *self.objects.values()]:
            is_robot = "body" not in draw
            if is_robot and draw["link"] == omit_link:
                continue
            candidates = ray_box_candidates(
                origins, directions, draw["world_low"], draw["world_high"], distance
            )
            if not len(candidates):
                continue
            starts = (origins[candidates] - draw["pos"]) @ draw["rotation"]
            dirs = directions[candidates] @ draw["rotation"]
            hit = draw["scene"].run(
                np.ascontiguousarray(starts, np.float32),
                np.ascontiguousarray(dirs, np.float32),
                output=1,
            )
            valid = (hit["primID"] >= 0) & (hit["tfar"] <= distance[candidates])
            rays = candidates[valid]
            distance[rays] = hit["tfar"][valid]
            bodies[rays] = self.robot.body_id if is_robot else draw["body"]
        return origins + directions * distance[:, None], distance, bodies
