#!/usr/bin/env python3
"""Lidar alignment check — overlay the two Mid-360 clouds in a common body frame.

Captures both lidars from the livox_bridge LVXR stream, IMU-LEVELS each one
(rotates its measured gravity to straight down — absorbs pole tilt/shake and the
mount roll/pitch), then places the TOP lidar relative to the BOTTOM via the
caged-URDF mount (default), and renders an OVERLAY so you can visually confirm
alignment:

  top-down (X-Y):  bottom = cyan, top = magenta, OVERLAP = white-ish
  side    (X-Z):   same colours — check both ground planes are flat + coincident

Tune the top-vs-bottom transform until common structures (walls, ground) overlap:

  ./lidar_align.py --yaw-deg 0 --dz 0.07            # defaults from the URDF
  ./lidar_align.py --yaw-deg 12 --dz 0.07 --secs 6  # nudge the top yaw, re-render

Start the sidecar first:
  livox_bridge/build/livox_bridge livox_bridge/config/mid360.json
"""
import argparse
import os
import select
import socket
import struct
import time

import numpy as np
from PIL import Image

HDR = 20
MAGIC = b"LVXR"
BOTTOM_ID = 40
TOP_ID = 41
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUT = os.path.join(_REPO, "rove_control_bridge", "media", "lidar", "lidar_align.png")


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
                a = np.frombuffer(d, dtype="<f4", count=cnt * 3, offset=HDR).reshape(-1, 3)
                pts.setdefault(lid, []).append(a.copy())
            elif typ == 2:
                v = np.frombuffer(d, dtype="<f4", count=6, offset=HDR)
                imu.setdefault(lid, []).append(v.copy())
    ps.close()
    isk.close()
    return {lid: (np.concatenate(ch), np.array(imu.get(lid, []))) for lid, ch in pts.items()}


def rot_a_to_b(a, b):
    """Rotation matrix taking unit vector a -> unit vector b (Rodrigues)."""
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    s = np.linalg.norm(v)
    c = float(np.dot(a, b))
    if s < 1e-8:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


def Rz(deg):
    r = np.radians(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def level(pts, imu):
    """Rotate so the lidar's measured gravity points straight down (z-)."""
    if len(imu) == 0:
        return pts, 0.0
    g = imu[:, 3:6].mean(axis=0)
    tilt = np.degrees(np.arccos(abs(g[2] / (np.linalg.norm(g) + 1e-9))))
    R = rot_a_to_b(g, np.array([0.0, 0.0, -1.0]))
    return pts @ R.T, tilt


def paint(img, pts, half, cell, rgb):
    n = img.shape[0]
    gi = ((pts[:, 0] + half) / cell).astype(int)
    gj = ((pts[:, 1] + half) / cell).astype(int)
    m = (gi >= 0) & (gi < n) & (gj >= 0) & (gj < n)
    np.add.at(img, (gi[m], gj[m]), rgb)


def paint_xz(img, pts, halfx, halfz, cell, rgb):
    n = img.shape[0]
    gi = ((pts[:, 0] + halfx) / cell).astype(int)            # X (forward) -> rows
    gj = ((pts[:, 2] + halfz) / cell).astype(int)            # Z (up)      -> cols
    m = (gi >= 0) & (gi < n) & (gj >= 0) & (gj < n)
    np.add.at(img, (gi[m], gj[m]), rgb)


def finish(img):
    img = np.clip(img, 0, 255).astype(np.uint8)
    img = np.flipud(img.transpose(1, 0, 2)).copy()           # forward up
    return Image.fromarray(img).resize((520, 520), Image.NEAREST)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pts-port", type=int, default=7020)
    ap.add_argument("--imu-port", type=int, default=7021)
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--half", type=float, default=8.0, help="top-down half-extent (m)")
    ap.add_argument("--cell", type=float, default=0.06)
    # top-vs-bottom transform (defaults from the caged URDF: ~same yaw, +0.07 m z)
    ap.add_argument("--yaw-deg", type=float, default=0.0, help="rotate TOP about z")
    ap.add_argument("--dx", type=float, default=0.0)
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dz", type=float, default=0.07)
    ap.add_argument("--out", default=DEFAULT_OUT)
    a = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)

    data = capture(a.pts_port, a.imu_port, a.secs)
    if BOTTOM_ID not in data or TOP_ID not in data:
        print(f"need both lidars; got {sorted(data)} — is the sidecar streaming?")
        return
    bp, bi = data[BOTTOM_ID]
    tp, ti = data[TOP_ID]
    bp, btilt = level(bp, bi)                                # bottom = body reference
    tp, ttilt = level(tp, ti)
    tp = tp @ Rz(a.yaw_deg).T + np.array([a.dx, a.dy, a.dz])  # place TOP rel. to bottom

    # top-down overlay
    n = int(2 * a.half / a.cell)
    top_xy = np.full((n, n, 3), 12.0)
    paint(top_xy, bp, a.half, a.cell, np.array([0.0, 70.0, 70.0]))    # bottom cyan
    paint(top_xy, tp, a.half, a.cell, np.array([80.0, 0.0, 70.0]))    # top magenta
    # side overlay (X forward vs Z up)
    halfz = 3.0
    nz = int(2 * a.half / a.cell)
    side = np.full((nz, nz, 3), 12.0)
    paint_xz(side, bp, a.half, halfz, a.cell, np.array([0.0, 70.0, 70.0]))
    paint_xz(side, tp, a.half, halfz, a.cell, np.array([80.0, 0.0, 70.0]))

    montage = np.hstack([np.array(finish(top_xy)), np.array(finish(side))])
    Image.fromarray(montage).save(a.out)
    print(f"bottom(.{BOTTOM_ID}) {len(bp)} pts tilt {btilt:.1f}deg | "
          f"top(.{TOP_ID}) {len(tp)} pts tilt {ttilt:.1f}deg")
    print(f"top transform: yaw {a.yaw_deg} deg, offset ({a.dx},{a.dy},{a.dz}) m")
    print(f"-> {a.out}  (LEFT top-down X-Y, RIGHT side X-Z; cyan=bottom magenta=top, "
          f"overlap=light. Tune --yaw-deg/--dz until common features overlap.)")


if __name__ == "__main__":
    main()
