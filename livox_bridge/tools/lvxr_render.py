#!/usr/bin/env python3
"""Render the LIVE Mid-360 lidar + traversability cost map from the livox_bridge
LVXR stream (no sim, no rove_control_bridge needed).

Captures a few seconds of the sidecar's points + IMU, then renders, per lidar, a
top-down height-coloured point cloud and the cost map — using the SAME classify +
colours as rove_sim/tools/costmap_snapshot.py and rove_control_bridge's
perception/costmap.rs. Saves a montage PNG.

Points are in the lidar SENSOR frame (sensor at origin, Z up). This snapshot does
NOT yet apply the URDF extrinsics or IMU pole-shake levelling (that lands in the
bridge) — it reports the IMU tilt so we can see how level the pole is.

  # start the sidecar first:  livox_bridge/build/livox_bridge livox_bridge/config/mid360.json
  ./lvxr_render.py --out /tmp/lvxr.png
"""
import argparse
import os
import select
import socket
import struct
import time

import numpy as np
from PIL import Image

# repo_root/rove_control_bridge/media/lidar — same place the sim lidar renders live
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUT = os.path.join(_REPO, "rove_control_bridge", "media", "lidar", "live_costmap.png")

# classify thresholds — match costmap_snapshot.py / perception/costmap.rs
STEP_CLIMB = 0.45
WALL_HEIGHT = 0.6
FLAT_SLOPE = 8.0
HILL_SLOPE = 22.0
STEEP_SLOPE = 38.0
CLIFF_DROP = 0.6

HDR = 20
MAGIC = b"LVXR"


def capture(pts_port, imu_port, secs):
    ps = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ps.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ps.bind(("", pts_port))
    isk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    isk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    isk.bind(("", imu_port))
    pts, imu = {}, {}
    t0 = time.time()
    while time.time() - t0 < secs:
        r, _, _ = select.select([ps, isk], [], [], 0.5)
        for s in r:
            d, _ = s.recvfrom(65535)
            if len(d) < HDR or d[:4] != MAGIC:
                continue
            typ, lid = d[5], d[6]
            cnt = struct.unpack_from("<H", d, 8)[0]
            if typ == 1 and cnt:
                arr = np.frombuffer(d, dtype="<f4", count=cnt * 3, offset=HDR).reshape(-1, 3)
                pts.setdefault(lid, []).append(arr.copy())
            elif typ == 2:
                v = np.frombuffer(d, dtype="<f4", count=6, offset=HDR)
                imu.setdefault(lid, []).append(v.copy())
    ps.close()
    isk.close()
    return {lid: (np.concatenate(ch), np.array(imu.get(lid, []))) for lid, ch in pts.items()}


def build_costmap(pts, half, cell):
    n = int(2 * half / cell)
    gi = ((pts[:, 0] + half) / cell).astype(int)
    gj = ((pts[:, 1] + half) / cell).astype(int)
    m = (gi >= 0) & (gi < n) & (gj >= 0) & (gj < n)
    gi, gj, z = gi[m], gj[m], pts[m, 2]
    ground = np.full((n, n), np.nan)
    top = np.full((n, n), np.nan)
    cnt = np.zeros((n, n), int)
    if len(z) == 0:
        return ground, top, cnt, n
    order = np.lexsort((z, gj, gi))
    gi, gj, z = gi[order], gj[order], z[order]
    flat = gi * n + gj
    uniq, start = np.unique(flat, return_index=True)
    for k, u in enumerate(uniq):
        s0 = start[k]
        s1 = start[k + 1] if k + 1 < len(uniq) else len(z)
        zz = z[s0:s1]
        i, j = u // n, u % n
        ground[i, j] = np.percentile(zz, 15)
        top[i, j] = zz.max()
        cnt[i, j] = s1 - s0
    return ground, top, cnt, n


def classify(ground, top, cnt, n, cell):
    img = np.zeros((n, n, 3), np.uint8)
    img[:] = (35, 35, 40)  # unknown grey
    if np.all(np.isnan(ground)):
        return img
    g0 = np.nanmedian(ground)
    for i in range(n):
        for j in range(n):
            if cnt[i, j] == 0:
                continue
            gr = ground[i, j]
            obstacle_h = top[i, j] - gr
            ns = []
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = i + di, j + dj
                if 0 <= a < n and 0 <= b < n and not np.isnan(ground[a, b]):
                    ns.append(ground[a, b])
            if ns:
                dz = max(abs(gr - x) for x in ns)
                slope = np.degrees(np.arctan2(dz, cell))
                drop = gr - min(ns)
            else:
                slope, drop = 0.0, 0.0
            if drop > CLIFF_DROP and gr < g0 - 0.3:
                img[i, j] = (40, 90, 220)  # cliff -> blue
            elif obstacle_h > WALL_HEIGHT and slope > STEEP_SLOPE:
                img[i, j] = (220, 40, 40)  # wall/tree -> red
            elif slope > STEEP_SLOPE or (STEP_CLIMB * 0.4 < obstacle_h <= WALL_HEIGHT):
                img[i, j] = (240, 150, 30)  # step/steep -> orange
            elif slope > HILL_SLOPE:
                img[i, j] = (235, 215, 40)  # steep hill -> yellow
            elif slope > FLAT_SLOPE:
                img[i, j] = (170, 210, 60)  # gentle hill -> light green
            else:
                img[i, j] = (40, 170, 70)  # flat -> green
    return img


def height_cmap(t):
    # blue -> cyan -> green -> yellow -> red ramp for t in [0,1]
    stops = np.array([[40, 60, 200], [40, 200, 200], [60, 200, 70],
                      [235, 215, 40], [230, 50, 40]], float)
    x = np.clip(t, 0, 1) * (len(stops) - 1)
    lo = np.floor(x).astype(int)
    hi = np.clip(lo + 1, 0, len(stops) - 1)
    f = (x - lo)[:, None]
    return (stops[lo] * (1 - f) + stops[hi] * f).astype(np.uint8)


def render_cloud(pts, half, cell):
    n = int(2 * half / cell)
    img = np.full((n, n, 3), (18, 18, 22), np.uint8)
    gi = ((pts[:, 0] + half) / cell).astype(int)
    gj = ((pts[:, 1] + half) / cell).astype(int)
    m = (gi >= 0) & (gi < n) & (gj >= 0) & (gj < n)
    gi, gj, z = gi[m], gj[m], pts[m, 2]
    if len(z):
        zmin, zmax = np.percentile(z, 2), np.percentile(z, 98)
        col = height_cmap((z - zmin) / max(1e-3, zmax - zmin))
        order = np.argsort(z)  # higher points overwrite lower
        img[gi[order], gj[order]] = col[order]
    return img


def orient(img):
    # north/forward up, robot at centre with a white cross
    n = img.shape[0]
    img = np.flipud(img.transpose(1, 0, 2)).copy()
    c = n // 2
    img[c - 3:c + 4, c] = 255
    img[c, c - 3:c + 4] = 255
    return img


def panel(img, size=420):
    return np.array(Image.fromarray(orient(img)).resize((size, size), Image.NEAREST))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pts-port", type=int, default=7020)
    ap.add_argument("--imu-port", type=int, default=7021)
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--half", type=float, default=10.0)
    ap.add_argument("--cloud-cell", type=float, default=0.12)
    ap.add_argument("--map-cell", type=float, default=0.4)
    ap.add_argument("--out", default=DEFAULT_OUT)
    a = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)

    data = capture(a.pts_port, a.imu_port, a.secs)
    if not data:
        print("no LVXR packets — is livox_bridge running and streaming to these ports?")
        return
    rows = []
    for lid in sorted(data):
        pts, imu = data[lid]
        tilt = float("nan")
        if len(imu):
            g = imu[:, 3:6].mean(axis=0)  # mean accel (g)
            gn = g / (np.linalg.norm(g) + 1e-9)
            tilt = np.degrees(np.arccos(abs(gn[2])))  # tilt of sensor Z from vertical
        gnd, top, cnt, n = build_costmap(pts, a.half, a.map_cell)
        cmap = classify(gnd, top, cnt, n, a.map_cell)
        cloud = render_cloud(pts, a.half, a.cloud_cell)
        rows.append(np.hstack([panel(cloud), panel(cmap)]))
        occ = int((cnt > 0).sum())
        print(f"lidar {lid}: {len(pts):>7} pts, {occ:>4} map cells, "
              f"z[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]m, IMU tilt {tilt:.1f} deg")
    montage = np.vstack(rows) if len(rows) > 1 else rows[0]
    Image.fromarray(montage).save(a.out)
    print(f"-> {a.out}  (left: height-coloured cloud | right: cost map; "
          f"rows = lidar id. green=flat yellow=hill orange=step red=wall blue=cliff grey=unknown)")


if __name__ == "__main__":
    main()
