#!/usr/bin/env python3
"""voxelize: turn the live lidar map into a 3D VOXEL model.

Subscribes to BOTH Livox (top + bottom), accumulates the world-frame points, bins
them into 3D voxels coloured by traversability height (green=ground, yellow/orange
=low structure, red=walls/trees), and:
  * exports a **GLB** (open in any 3D viewer / drag onto https://gltf-viewer ...),
  * renders an isometric preview PNG (no GPU needed).

    tools/voxelize.py --host 192.168.2.4 --pitch 0.35 --out /tmp/voxmap

The same height field the cost-map planner reasons over -- just shown as voxels.
"""
import argparse
import os
import socket
import sys
import time

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rove_sim.sensors.lidar import CloudReassembler


def grab(host, port, secs):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 0)); s.settimeout(1.0)
    s.sendto(b"LVXSUB", (host, port))
    ra = CloudReassembler(); pts = []; sensor = (0.0, 0.0, 0.0); t0 = time.time(); last = 0.0
    while time.time() - t0 < secs:
        if time.time() - last > 1.0:
            s.sendto(b"LVXSUB", (host, port)); last = time.time()
        try:
            buf, _ = s.recvfrom(65535)
        except socket.timeout:
            continue
        g = ra.feed(buf)
        if g:
            pts.append(g[0]); sensor = g[2][0]
    s.close()
    return (np.concatenate(pts) if pts else np.zeros((0, 3))), sensor


def height_color(z, zlo, zhi):
    """green (ground) -> yellow -> orange -> red (high), by height."""
    t = np.clip((z - zlo) / max(1e-3, zhi - zlo), 0, 1)
    r = np.clip(t * 2.0, 0, 1)
    g = np.clip(1.6 - t * 1.6, 0, 1)
    b = np.zeros_like(t)
    return np.stack([r, g, b, np.ones_like(t)], 1)


def voxelize(pts, sensor, half, pitch):
    cx, cy, cz = sensor
    m = (np.abs(pts[:, 0] - cx) < half) & (np.abs(pts[:, 1] - cy) < half) & \
        (pts[:, 2] - (cz - 0.85) > -1.0) & (pts[:, 2] - (cz - 0.85) < 8.0)
    P = pts[m]
    if len(P) == 0:
        return np.zeros((0, 3)), np.zeros((0, 4))
    keys = np.floor(P / pitch).astype(np.int64)
    uniq = np.unique(keys, axis=0)
    centers = (uniq + 0.5) * pitch
    ground = cz - 0.85
    colors = height_color(centers[:, 2], ground - 0.2, ground + 3.0)
    return centers, colors


def export_glb(centers, colors, pitch, out):
    import trimesh
    occ = {}
    base = centers.min(0) - pitch
    idx = np.floor((centers - base) / pitch).astype(int)
    dim = idx.max(0) + 2
    grid = np.zeros(tuple(dim), bool)
    grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    from trimesh.voxel import VoxelGrid
    from trimesh.voxel.encoding import DenseEncoding
    T = np.eye(4); T[0, 0] = T[1, 1] = T[2, 2] = pitch; T[:3, 3] = base
    vg = VoxelGrid(DenseEncoding(grid), T)
    mesh = vg.as_boxes(colors=(colors[0] * 255).astype(np.uint8) if len(colors) else None)
    # recolor per-voxel by height (as_boxes single colour fallback above)
    try:
        vc = np.repeat((colors * 255).astype(np.uint8), 12 * 3, axis=0)  # 12 tris * 3 verts/box
        if len(vc) == len(mesh.vertices):
            mesh.visual.vertex_colors = vc
    except Exception:
        pass
    mesh.export(out)
    return len(centers)


def render_iso(centers, colors, sensor, out, px=720):
    """Cheap isometric voxel render (painter's algorithm), no GPU."""
    if len(centers) == 0:
        return
    c = centers - np.array(sensor)
    # isometric: screen x = (X - Y), screen y = (X + Y)/2 - Z*1.3 (z up)
    sx = (c[:, 0] - c[:, 1])
    sy = (c[:, 0] + c[:, 1]) * 0.5 - c[:, 2] * 1.4
    depth = c[:, 0] + c[:, 1] - c[:, 2]  # far first
    order = np.argsort(-depth)
    sx, sy, col = sx[order], sy[order], colors[order]
    lo = np.array([sx.min(), sy.min()]); hi = np.array([sx.max(), sy.max()])
    span = max(hi[0] - lo[0], hi[1] - lo[1]) + 1e-6
    scale = (px - 40) / span
    img = Image.new("RGB", (px, px), (12, 12, 16))
    from PIL import ImageDraw
    d = ImageDraw.Draw(img)
    s = max(3, int(scale * 0.42))  # voxel square size
    for k in range(len(sx)):
        X = int(20 + (sx[k] - lo[0]) * scale)
        Y = int(px - 20 - (sy[k] - lo[1]) * scale)
        rgb = tuple(int(v * 255) for v in col[k, :3])
        d.rectangle([X - s, Y - s, X + s, Y + s], fill=rgb, outline=(0, 0, 0))
    img.save(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="192.168.2.4")
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--half", type=float, default=16.0)
    ap.add_argument("--pitch", type=float, default=0.35)
    ap.add_argument("--out", default="/tmp/voxmap")
    a = ap.parse_args()
    pts = []
    sensor = (0, 0, 0)
    for port in (5024, 5022):  # bottom (near) + top (far) for full 3D
        p, sensor = grab(a.host, port, a.secs / 2)
        if len(p):
            pts.append(p)
    pts = np.concatenate(pts) if pts else np.zeros((0, 3))
    centers, colors = voxelize(pts, sensor, a.half, a.pitch)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    render_iso(centers, colors, sensor, a.out + ".png")
    try:
        export_glb(centers, colors, a.pitch, a.out + ".glb")
        glb = a.out + ".glb"
    except Exception as e:
        glb = f"(glb export skipped: {e})"
    print(f"[voxelize] {len(pts)} pts -> {len(centers)} voxels @ {a.pitch} m | "
          f"sensor {tuple(round(v,2) for v in sensor)} | {a.out}.png | {glb}")


if __name__ == "__main__":
    main()
