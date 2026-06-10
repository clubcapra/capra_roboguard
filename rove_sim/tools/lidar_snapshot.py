#!/usr/bin/env python3
"""lidar_snapshot: subscribe to a live Livox cloud and render it top-down to PNG.

A quick way to SEE what the robot sees -- the road, the trees, and (crucially) the
drop-offs, which show up as gaps with no returns. Subscribes the Livox-style way
(registration), so it works from any machine without soft-locking the stream.

    tools/lidar_snapshot.py --host 192.168.2.4 --port 5022 --out /tmp/lidar.png

Colour = height (blue low -> green ~ground -> red high/trees). White cross = the
sensor (robot). North is up. Dim rings every 5 m.
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
    ra = CloudReassembler(); pts = []; sensor = (0.0, 0.0, 0.0)
    t0 = time.time(); last = 0.0
    while time.time() - t0 < secs:
        if time.time() - last > 1.0:                 # keepalive within the TTL
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


def render(pts, sensor, out, window):
    W = 700; M = float(window); sc = W / (2 * M)
    img = np.zeros((W, W, 3), np.uint8)
    if len(pts):
        dx = pts[:, 0] - sensor[0]; dy = pts[:, 1] - sensor[1]; z = pts[:, 2]
        px = ((dx + M) * sc).astype(int); py = ((M - dy) * sc).astype(int)  # north up
        m = (px >= 0) & (px < W) & (py >= 0) & (py < W)
        px, py, zz = px[m], py[m], z[m]
        zc = np.clip((zz - sensor[2] + 3) / 6.0, 0, 1)
        img[py, px, 0] = (zc * 255).astype(np.uint8)
        img[py, px, 1] = ((1 - np.abs(zc - 0.5) * 2) * 255).clip(0, 255).astype(np.uint8)
        img[py, px, 2] = ((1 - zc) * 255).astype(np.uint8)
    c = W // 2
    img[c - 4:c + 5, c] = 255; img[c, c - 4:c + 5] = 255          # sensor cross
    for rr in (5, 10, 15, 20):                                     # range rings
        a = np.linspace(0, 2 * np.pi, 720)
        gx = (c + rr * sc * np.cos(a)).astype(int); gy = (c - rr * sc * np.sin(a)).astype(int)
        mm = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < W)
        img[gy[mm], gx[mm]] = np.maximum(img[gy[mm], gx[mm]], 40)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    Image.fromarray(img).save(out)
    return len(pts)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="192.168.2.4")
    ap.add_argument("--port", type=int, default=5022, help="5022 top / 5024 bottom Livox")
    ap.add_argument("--secs", type=float, default=4.0, help="accumulate this long")
    ap.add_argument("--window", type=float, default=25.0, help="+/- metres shown")
    ap.add_argument("--out", default="/tmp/lidar.png")
    a = ap.parse_args()
    pts, sensor = grab(a.host, a.port, a.secs)
    n = render(pts, sensor, a.out, a.window)
    print(f"[lidar_snapshot] {n} pts; sensor world {tuple(round(v,2) for v in sensor)} "
          f"-> {a.out} (+/-{a.window:g} m, N up)")


if __name__ == "__main__":
    main()
