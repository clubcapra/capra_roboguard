"""GLB -> OBJ conversion + URDF rewrite (S5.1).

PyBullet renders OBJ, not GLB. For each ``<mesh filename="meshes/X.glb"/>`` in a
URDF we export an OBJ (+MTL) and rewrite the reference, producing a sibling
``<model>.obj.urdf`` that PyBullet can load. Conversion is cached: a GLB is
re-exported only when its mtime is newer than the OBJ.

Idempotent and standalone:

    python -m tools.convert_meshes /path/to/rove_standard.urdf
"""
from __future__ import annotations

import os
import re
import sys
import xml.etree.ElementTree as ET

import trimesh

_MESH_RE = re.compile(r'(meshes/[^"\']+)\.glb', re.IGNORECASE)


def convert_glb(src: str, dst: str) -> bool:
    """Export one GLB to OBJ. Returns True if (re)written, False if cached."""
    if os.path.exists(dst) and os.path.getmtime(dst) >= os.path.getmtime(src):
        return False
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    scene = trimesh.load(src, force="scene")
    # flatten a multi-geometry scene into one mesh in its baked world transform
    mesh = scene.to_geometry() if hasattr(scene, "to_geometry") else scene
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(
            [g.copy().apply_transform(scene.graph.get(n)[0])
             for n, g in scene.geometry.items()])
    mesh.export(dst)
    return True


def convert_urdf(urdf_path: str, out_path: str | None = None) -> str:
    """Convert every GLB referenced by a URDF and emit an OBJ-based URDF."""
    urdf_path = os.path.abspath(urdf_path)
    base = os.path.dirname(urdf_path)
    if out_path is None:
        out_path = urdf_path[:-5] + ".obj.urdf" if urdf_path.endswith(".urdf") \
            else urdf_path + ".obj.urdf"

    text = open(urdf_path).read()
    rels = sorted(set(m.group(1) for m in _MESH_RE.finditer(text)))
    n_conv = 0
    for rel in rels:
        src = os.path.join(base, rel + ".glb")
        dst = os.path.join(base, rel + ".obj")
        if not os.path.exists(src):
            raise FileNotFoundError(f"URDF references missing mesh: {src}")
        n_conv += convert_glb(src, dst)

    new_text = _MESH_RE.sub(r"\1.obj", text)
    with open(out_path, "w") as f:
        f.write(new_text)
    # sanity: the rewritten URDF must still parse as XML
    ET.fromstring(new_text)
    print(f"[convert] {os.path.basename(urdf_path)}: "
          f"{len(rels)} meshes ({n_conv} (re)converted) -> "
          f"{os.path.basename(out_path)}")
    return out_path


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("usage: python -m tools.convert_meshes <urdf> [out.urdf]")
    convert_urdf(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
