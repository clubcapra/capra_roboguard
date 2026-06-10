#!/usr/bin/env python3
"""costmap_snapshot: build + render a 3D traversability cost map from the lidar.

The first slice of the autonomy cost map: subscribe to the bottom Livox, bin the
points into a 2.5-D world grid, and classify each cell by TRAVERSABILITY COST
(not a binary obstacle) so the planner can prefer flat, accept a hill/stairs when
needed, and refuse walls/cliffs:

  flat        green   (low cost)
  hill        yellow  (slope, costlier)
  steep/step  orange  (stairs/steep -- climbable for this flipper robot, high cost)
  wall/tree   red     (blocked -- too tall+steep to climb)
  cliff/edge  blue    (blocked -- ground drops away / no floor)
  unknown     grey    (no returns)

    tools/costmap_snapshot.py --host 192.168.2.4 --out /tmp/costmap.png
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

# climb envelope for the tracked + flipper robot (tune to the real platform)
STEP_CLIMB = 0.45      # m: a single step up to here is climbable (stairs)
WALL_HEIGHT = 0.6      # m: taller than this above local ground = wall/obstacle
FLAT_SLOPE = 8.0       # deg
HILL_SLOPE = 22.0      # deg: up to here = drivable hill
STEEP_SLOPE = 38.0     # deg: up to here = steep/stairs (climbable, costly); above = wall
CLIFF_DROP = 0.6       # m: neighbour ground this much LOWER = edge/cliff


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


def build_costmap(pts, sensor, half=15.0, cell=0.3):
    n = int(2 * half / cell)
    cx, cy, cz = sensor
    # bin points
    gi = ((pts[:, 0] - (cx - half)) / cell).astype(int)
    gj = ((pts[:, 1] - (cy - half)) / cell).astype(int)
    m = (gi >= 0) & (gi < n) & (gj >= 0) & (gj < n)
    gi, gj, z = gi[m], gj[m], pts[m, 2]
    ground = np.full((n, n), np.nan)   # low surface
    top = np.full((n, n), np.nan)      # high surface
    cnt = np.zeros((n, n), int)
    # per-cell low (ground) and high (top) via sorting
    order = np.lexsort((z, gj, gi))
    gi, gj, z = gi[order], gj[order], z[order]
    flat_idx = gi * n + gj
    uniq, start = np.unique(flat_idx, return_index=True)
    for k, u in enumerate(uniq):
        s0 = start[k]; s1 = start[k + 1] if k + 1 < len(uniq) else len(z)
        zz = z[s0:s1]
        i, j = u // n, u % n
        ground[i, j] = np.percentile(zz, 15)
        top[i, j] = zz.max()
        cnt[i, j] = s1 - s0
    return ground, top, cnt, n, cell


def classify(ground, top, cnt, n, cell):
    """Return an RGB image (n,n,3) of traversability cost colours."""
    img = np.zeros((n, n, 3), np.uint8)
    img[:] = (35, 35, 40)  # unknown grey
    g0 = np.nanmedian(ground)  # nominal ground level
    for i in range(n):
        for j in range(n):
            if cnt[i, j] == 0:
                continue
            gr = ground[i, j]
            obstacle_h = top[i, j] - gr
            # neighbour ground for slope + edge
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
            # classify (priority: cliff > wall > step/steep > hill > flat)
            if drop > CLIFF_DROP and gr < g0 - 0.3:
                img[i, j] = (40, 90, 220)            # cliff/edge -> blue
            elif obstacle_h > WALL_HEIGHT and slope > STEEP_SLOPE:
                img[i, j] = (220, 40, 40)            # wall/tree -> red (blocked)
            elif slope > STEEP_SLOPE or (STEP_CLIMB * 0.4 < obstacle_h <= WALL_HEIGHT):
                img[i, j] = (240, 150, 30)           # steep/step (climbable, costly) -> orange
            elif slope > HILL_SLOPE:
                img[i, j] = (235, 215, 40)           # steep hill -> yellow
            elif slope > FLAT_SLOPE:
                img[i, j] = (170, 210, 60)           # gentle hill -> light green
            else:
                img[i, j] = (40, 170, 70)            # flat -> green
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="192.168.2.4")
    ap.add_argument("--port", type=int, default=5024)
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--half", type=float, default=15.0)
    ap.add_argument("--cell", type=float, default=0.3)
    ap.add_argument("--out", default="/tmp/costmap.png")
    a = ap.parse_args()
    pts, sensor = grab(a.host, a.port, a.secs)
    if len(pts) == 0:
        print("no points"); return
    ground, top, cnt, n, cell = build_costmap(pts, sensor, a.half, a.cell)
    img = classify(ground, top, cnt, n, cell)
    # north up, robot at centre
    img = np.flipud(img.transpose(1, 0, 2)).copy()
    c = n // 2
    img[c - 3:c + 4, c] = 255; img[c, c - 3:c + 4] = 255
    big = Image.fromarray(img).resize((n * 6, n * 6), Image.NEAREST)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    big.save(a.out)
    occ = int((cnt > 0).sum())
    print(f"[costmap] {len(pts)} pts, {occ} cells, sensor {tuple(round(v,2) for v in sensor)} "
          f"-> {a.out} (green=flat, yellow=hill, orange=step/stairs, red=wall, blue=cliff)")


if __name__ == "__main__":
    main()
