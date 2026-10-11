"""Shared, portable terrain meshes and original texture coverage.

Render and ray workers consume the same indexed mesh/UV/material representation.
The on-disk numpy cache avoids repeatedly parsing OBJ or copying Bullet's large
cutout collision meshes. Source textures remain RGBA and are never flattened.
"""

from dataclasses import dataclass
from pathlib import Path
import json
import os
import hashlib
import io
import numpy as np
from PIL import Image
from numba import njit


@dataclass
class MeshAsset:
    vertices: np.ndarray
    faces: np.ndarray
    uv: np.ndarray
    rgba: np.ndarray
    cutoff: float
    material: str
    normal: np.ndarray | None = None
    roughness: np.ndarray | None = None


def asset_path(manifest, path):
    root = Path(manifest).parent
    local = root / path
    if local.is_file():
        return local
    suffix = str(path).replace("\\", "/").split("/" + root.name + "/", 1)
    return root / suffix[-1] if len(suffix) == 2 else Path(path)


def load_mesh(path):
    path = Path(path)
    cache = path.parent / ".runtime" / (path.name + ".npz")
    if cache.is_file() and cache.stat().st_mtime_ns >= path.stat().st_mtime_ns:
        with np.load(cache) as data:
            return {key: data[key] for key in ("vertices", "faces", "uv")}
    import trimesh

    mesh = trimesh.load(path, force="mesh", process=False, maintain_order=True)
    uv = getattr(mesh.visual, "uv", None)
    data = {
        "vertices": np.asarray(mesh.vertices, dtype=np.float32),
        "faces": np.asarray(mesh.faces, dtype=np.int32),
        "uv": (
            np.asarray(uv, dtype=np.float32)
            if uv is not None
            else np.zeros((len(mesh.vertices), 2), np.float32)
        ),
    }
    cache.parent.mkdir(exist_ok=True)
    temp = cache.with_name(cache.name + f".{os.getpid()}.tmp")
    with temp.open("wb") as f:
        np.savez(f, **data)
    os.replace(temp, cache)
    return data


def load_terrain(manifest, scale=1, material_maps=False):
    data = json.loads(Path(manifest).read_text())
    out = []
    for entry in data["visual"]:
        mesh = load_mesh(asset_path(manifest, entry["obj"]))
        vertices = mesh["vertices"] * float(scale)
        # Small layer offsets prevent coplanar road/leaf-litter decals fighting.
        layer = {"ground": 0, "rock": 1, "grass": 2, "foliage": 3}.get(
            entry.get("category"), 0
        )
        vertices = vertices.copy()
        vertices[:, 2] += layer * 0.004 + len(out) * 0.0003
        tex = entry.get("tex")
        if tex:
            payload = asset_path(manifest, tex).read_bytes()
            expected = entry.get("texture_sha256")
            if expected and hashlib.sha256(payload).hexdigest() != expected:
                raise ValueError(f"Terrain texture checksum mismatch: {tex}")
            with Image.open(io.BytesIO(payload)) as image:
                rgba = np.asarray(image.convert("RGBA")).copy()
        else:
            rgba = (
                (np.array(entry.get("color", [0.5, 0.5, 0.5, 1])) * 255)
                .astype(np.uint8)
                .reshape(1, 1, 4)
            )
        mode = entry.get("alpha_mode", "MASK" if rgba[..., 3].min() < 255 else "OPAQUE")
        cutoff = float(entry.get("alpha_cutoff", 0.5)) if mode != "OPAQUE" else 0
        maps = {}
        if material_maps:
            for channel in ("normal", "roughness"):
                if entry.get(channel + "_tex"):
                    payload = asset_path(manifest, entry[channel + "_tex"]).read_bytes()
                    expected = entry.get(channel + "_sha256")
                    if expected and hashlib.sha256(payload).hexdigest() != expected:
                        raise ValueError(f"Terrain {channel} checksum mismatch")
                    with Image.open(io.BytesIO(payload)) as image:
                        maps[channel] = np.asarray(image.convert("RGB" if channel == "normal" else "L")).copy()
        out.append(
            MeshAsset(
                vertices,
                mesh["faces"],
                mesh["uv"],
                rgba,
                cutoff,
                entry.get("material", ""),
                **maps,
            )
        )
    return out


def texture_alpha(rgba, uv):
    """Bilinear, repeat-wrapped base-level sampling, matching the GPU sampler."""
    h, w = rgba.shape[:2]
    x = np.mod(uv[:, 0], 1) * w - 0.5
    y = (1 - np.mod(uv[:, 1], 1)) * h - 0.5
    x0 = np.floor(x).astype(np.int64)
    y0 = np.floor(y).astype(np.int64)
    fx = x - x0
    fy = y - y0
    a = rgba[..., 3]
    return (
        (1 - fx) * (1 - fy) * a[y0 % h, x0 % w]
        + fx * (1 - fy) * a[y0 % h, (x0 + 1) % w]
        + (1 - fx) * fy * a[(y0 + 1) % h, x0 % w]
        + fx * fy * a[(y0 + 1) % h, (x0 + 1) % w]
    ) / 255


@njit(cache=True)
def covered(texcoord, material, alpha, offsets, widths, heights, cutoffs):
    result = np.ones(len(material), np.bool_)
    for i in range(len(material)):
        mid = material[i]
        if cutoffs[mid] == 0:
            continue
        w, h, offset = widths[mid], heights[mid], offsets[mid]
        x = (texcoord[i, 0] % 1) * w - 0.5
        y = (1 - texcoord[i, 1] % 1) * h - 0.5
        x0, y0 = int(np.floor(x)), int(np.floor(y))
        fx, fy = x - x0, y - y0
        value = (
            (1 - fx) * (1 - fy) * alpha[offset + (y0 % h) * w + x0 % w]
            + fx * (1 - fy) * alpha[offset + (y0 % h) * w + (x0 + 1) % w]
            + (1 - fx) * fy * alpha[offset + ((y0 + 1) % h) * w + x0 % w]
            + fx * fy * alpha[offset + ((y0 + 1) % h) * w + (x0 + 1) % w]
        )
        result[i] = value >= cutoffs[mid] * 255
    return result


class AlphaRaycaster:
    """Exact triangle/UV coverage evaluated inside Embree traversal.

    Opaque texels stop the ray; transparent texels leave traversal running so
    geometry behind the gap remains visible. GPU and LiDAR share alphaCutoff,
    repeat wrapping and bilinear base-level sampling. The optional Python
    traversal is a reference implementation used by regression tests.
    """

    def __init__(self, assets, native=True):
        from embreex.rtcore_scene import EmbreeScene
        from embreex.mesh_construction import TriangleMesh

        self.assets = assets
        self.widths = np.array([a.rgba.shape[1] for a in assets], np.int64)
        self.heights = np.array([a.rgba.shape[0] for a in assets], np.int64)
        self.offsets = np.r_[0, np.cumsum(self.widths * self.heights)[:-1]]
        self.alpha = np.concatenate([a.rgba[..., 3].ravel() for a in assets])
        self.cutoffs = np.array([a.cutoff for a in assets])
        self.scene = None
        self.native = None
        triangles = []
        uvs = []
        materials = []
        for i, mesh in enumerate(assets):
            triangles.append(mesh.vertices[mesh.faces])
            uvs.append(mesh.uv[mesh.faces])
            materials.append(np.full(len(mesh.faces), i, np.int32))
        self.uv = np.concatenate(uvs)
        self.material = np.concatenate(materials)
        triangles = np.ascontiguousarray(np.concatenate(triangles), np.float32)
        if native:
            from .native_rays import NativeTerrain

            self.native = NativeTerrain(
                triangles,
                self.uv,
                self.material,
                self.alpha,
                self.offsets,
                self.widths,
                self.heights,
                self.cutoffs,
            )
        else:
            self.scene = EmbreeScene()
            self.geometry = TriangleMesh(self.scene, triangles)

    def cast(self, origins, directions, max_range=70, max_layers=64):
        if self.native is not None:
            return self.native.cast(origins, directions, max_range)
        origins = np.ascontiguousarray(origins, np.float32)
        directions = np.ascontiguousarray(directions, np.float32)
        starts = origins.copy()
        live = np.arange(len(origins))
        points = np.zeros_like(origins)
        distance = np.full(len(origins), np.inf, np.float32)
        face = np.full(len(origins), -1, np.int32)
        for _ in range(max_layers):
            if not len(live):
                break
            hit = self.scene.run(starts[live], directions[live], output=1)
            valid = hit["primID"] >= 0
            rays = live[valid]
            if not len(rays):
                break
            tri = hit["primID"][valid]
            pos = starts[rays] + directions[rays] * hit["tfar"][valid, None]
            total = np.linalg.norm(pos - origins[rays], axis=1)
            u = hit["u"][valid]
            v = hit["v"][valid]
            uv = self.uv[tri]
            texcoord = (
                uv[:, 0] * (1 - u - v)[:, None]
                + uv[:, 1] * u[:, None]
                + uv[:, 2] * v[:, None]
            )
            accepted = total <= max_range
            material = self.material[tri]
            accepted &= covered(
                texcoord,
                material,
                self.alpha,
                self.offsets,
                self.widths,
                self.heights,
                self.cutoffs,
            )
            good = rays[accepted]
            points[good] = pos[accepted]
            distance[good] = total[accepted]
            face[good] = tri[accepted]
            retry = (~accepted) & (total < max_range)
            live = rays[retry]
            starts[live] = pos[retry] + directions[live] * 1e-4
        return points, distance, face
