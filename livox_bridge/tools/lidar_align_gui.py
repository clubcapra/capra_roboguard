#!/usr/bin/env python3
"""Browser-based Mid-360 alignment tool.

The robot is headless (no X display), so this serves a small web UI instead of a
native window. It continuously captures both lidars from the livox_bridge LVXR
stream, IMU-levels each (gravity -> down, absorbing pole tilt/shake + mount
roll/pitch), and overlays them in a common body frame. Drag the sliders to align
the TOP lidar onto the BOTTOM (the reference), then Save — it writes the extrinsic
config the bridge reads.

  livox_bridge/build/livox_bridge livox_bridge/config/mid360.json &   # sidecar
  python3 livox_bridge/tools/lidar_align_gui.py                        # then open the URL

Open http://192.168.2.2:8099/  (VSCode forwards the port automatically; cyan =
bottom .40, magenta = top .41, white = overlap — align common features).
"""
import argparse
import io
import json
import os
import select
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np
from PIL import Image

HDR = 20
MAGIC = b"LVXR"
BOTTOM_ID = 40
TOP_ID = 41
MAX_PTS = 60000  # subsample per lidar so renders stay snappy
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_CFG = os.path.join(_REPO, "rove_control_bridge", "config", "lidar_extrinsics.json")

LOCK = threading.Lock()
STATE = {"bottom": np.zeros((0, 3)), "top": np.zeros((0, 3)), "btilt": 0.0, "ttilt": 0.0}


# ---------- geometry ----------
def rot_a_to_b(a, b):
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
    if len(imu) == 0:
        return pts, 0.0
    g = imu[:, 3:6].mean(axis=0)
    tilt = float(np.degrees(np.arccos(abs(g[2] / (np.linalg.norm(g) + 1e-9)))))
    return pts @ rot_a_to_b(g, np.array([0.0, 0.0, -1.0])).T, tilt


# ---------- live capture (background thread) ----------
def capture_loop(pts_port, imu_port, window_s):
    ps = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ps.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ps.bind(("", pts_port))
    isk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    isk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    isk.bind(("", imu_port))
    while True:
        pts, imu = {}, {}
        t0 = time.time()
        while time.time() - t0 < window_s:
            r, _, _ = select.select([ps, isk], [], [], 0.3)
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
                    imu.setdefault(lid, []).append(
                        np.frombuffer(d, dtype="<f4", count=6, offset=HDR).copy())
        if BOTTOM_ID in pts and TOP_ID in pts:
            bp, bt = level(np.concatenate(pts[BOTTOM_ID]), np.array(imu.get(BOTTOM_ID, [])))
            tp, tt = level(np.concatenate(pts[TOP_ID]), np.array(imu.get(TOP_ID, [])))
            if len(bp) > MAX_PTS:
                bp = bp[:: len(bp) // MAX_PTS]
            if len(tp) > MAX_PTS:
                tp = tp[:: len(tp) // MAX_PTS]
            with LOCK:
                STATE.update(bottom=bp, top=tp, btilt=bt, ttilt=tt)


# ---------- render ----------
def paint(img, pts, ai, aj, halfa, halfb, cell, rgb):
    n = img.shape[0]
    gi = ((pts[:, ai] + halfa) / cell).astype(int)
    gj = ((pts[:, aj] + halfb) / cell).astype(int)
    m = (gi >= 0) & (gi < n) & (gj >= 0) & (gj < n)
    np.add.at(img, (gi[m], gj[m]), rgb)


def fin(img):
    img = np.clip(img, 0, 255).astype(np.uint8)
    return Image.fromarray(np.flipud(img.transpose(1, 0, 2)).copy()).resize((480, 480), Image.NEAREST)


def render_png(yaw, dx, dy, dz, half, cell):
    with LOCK:
        bp = STATE["bottom"]
        tp = STATE["top"]
    tp = tp @ Rz(yaw).T + np.array([dx, dy, dz]) if len(tp) else tp
    cyan = np.array([0.0, 70.0, 70.0])
    mag = np.array([80.0, 0.0, 70.0])
    n = int(2 * half / cell)
    xy = np.full((n, n, 3), 12.0)
    paint(xy, bp, 0, 1, half, half, cell, cyan)
    paint(xy, tp, 0, 1, half, half, cell, mag)
    halfz = 3.0
    sd = np.full((n, n, 3), 12.0)
    paint(sd, bp, 0, 2, half, halfz, cell, cyan)
    paint(sd, tp, 0, 2, half, halfz, cell, mag)
    montage = Image.new("RGB", (970, 480))
    montage.paste(fin(xy), (0, 0))
    montage.paste(fin(sd), (490, 0))
    buf = io.BytesIO()
    montage.save(buf, format="PNG")
    return buf.getvalue()


PAGE = """<!doctype html><html><head><meta charset=utf-8><title>Mid-360 align</title>
<style>body{background:#111;color:#ddd;font-family:system-ui,sans-serif;margin:16px}
.row{margin:8px 0}label{display:inline-block;width:60px}input[type=range]{width:360px;vertical-align:middle}
.v{display:inline-block;width:70px;text-align:right;font-variant-numeric:tabular-nums}
button{padding:8px 16px;font-size:15px;margin-right:10px}#msg{margin-left:10px;color:#6f6}
img{margin-top:12px;border:1px solid #333}.hint{color:#888;font-size:13px}</style></head>
<body><h2>Mid-360 alignment <span class=hint>cyan = bottom(.40) ref &nbsp; magenta = top(.41) &nbsp; white = overlap</span></h2>
<div id=stat class=hint>…</div>
<div class=row><label>yaw</label><input type=range id=yaw min=-180 max=180 step=0.5 value=0><span class=v id=yawv></span> deg</div>
<div class=row><label>dx</label><input type=range id=dx min=-2 max=2 step=0.01 value=0><span class=v id=dxv></span> m (fwd)</div>
<div class=row><label>dy</label><input type=range id=dy min=-2 max=2 step=0.01 value=0><span class=v id=dyv></span> m (left)</div>
<div class=row><label>dz</label><input type=range id=dz min=-1 max=1 step=0.01 value=0.07><span class=v id=dzv></span> m (up)</div>
<div class=row><button onclick=save()>Save config</button>
<label style=width:auto><input type=checkbox id=live checked> live</label><span id=msg></span></div>
<div class=hint>LEFT: top-down (X fwd up, Y left). RIGHT: side (X fwd, Z up) — check both ground planes coincide.</div>
<img id=img src=""><br>
<script>
const ids=['yaw','dx','dy','dz'];
function q(){return ids.map(i=>i+'='+document.getElementById(i).value).join('&');}
function refresh(){document.getElementById('img').src='/render?'+q()+'&t='+Date.now();}
ids.forEach(i=>{const e=document.getElementById(i),v=document.getElementById(i+'v');
  const u=()=>{v.textContent=(+e.value).toFixed(2);refresh();};e.addEventListener('input',u);u();});
function save(){fetch('/save?'+q()).then(r=>r.json()).then(d=>{
  document.getElementById('msg').textContent='saved -> '+d.path;});}
async function stat(){try{const d=await (await fetch('/status')).json();
  document.getElementById('stat').textContent=
   `bottom ${d.bottom} pts tilt ${d.btilt.toFixed(1)}deg | top ${d.top} pts tilt ${d.ttilt.toFixed(1)}deg`;
  if(document.getElementById('live').checked)refresh();}catch(e){}}
setInterval(stat,1500);stat();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        g = lambda k, d=0.0: float(q.get(k, [d])[0])
        if u.path == "/":
            self._send(200, "text/html; charset=utf-8", PAGE.encode())
        elif u.path == "/status":
            with LOCK:
                s = {"bottom": int(len(STATE["bottom"])), "top": int(len(STATE["top"])),
                     "btilt": STATE["btilt"], "ttilt": STATE["ttilt"]}
            self._send(200, "application/json", json.dumps(s).encode())
        elif u.path == "/render":
            png = render_png(g("yaw"), g("dx"), g("dy"), g("dz"), g("half", 8.0), g("cell", 0.06))
            self._send(200, "image/png", png)
        elif u.path == "/save":
            cfg = {"reference": "bottom_40",
                   "top_41": {"yaw_deg": g("yaw"), "dx": g("dx"), "dy": g("dy"), "dz": g("dz")},
                   "note": "top(.41) transform relative to the IMU-leveled bottom(.40) body frame"}
            os.makedirs(os.path.dirname(DEFAULT_CFG), exist_ok=True)
            with open(DEFAULT_CFG, "w") as f:
                json.dump(cfg, f, indent=2)
            print(f"[align] saved {cfg['top_41']} -> {DEFAULT_CFG}")
            self._send(200, "application/json", json.dumps({"ok": True, "path": DEFAULT_CFG}).encode())
        else:
            self._send(404, "text/plain", b"not found")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pts-port", type=int, default=7020)
    ap.add_argument("--imu-port", type=int, default=7021)
    ap.add_argument("--http-port", type=int, default=8099)
    ap.add_argument("--window", type=float, default=0.8, help="capture window per refresh (s)")
    a = ap.parse_args()
    threading.Thread(target=capture_loop, args=(a.pts_port, a.imu_port, a.window), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.http_port), H)
    print(f"[align] open http://192.168.2.2:{a.http_port}/  (Ctrl-C to stop)  "
          f"saves -> {DEFAULT_CFG}")
    srv.serve_forever()


if __name__ == "__main__":
    main()
