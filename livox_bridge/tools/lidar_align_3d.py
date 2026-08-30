#!/usr/bin/env python3
"""Browser-based 3D Mid-360 alignment tool (WebGL / Three.js).

Headless robot -> serves a 3D point-cloud viewer you orbit in the browser. Both
lidars are captured from the livox_bridge LVXR stream and IMU-leveled (gravity ->
down) as a STARTING point; note the top (.41) points up and the bottom (.40)
points down, so they image opposite hemispheres and the true relative orientation
includes a flip. Use the full yaw/pitch/roll + dx/dy/dz controls on the TOP cloud
to align it onto the BOTTOM (the reference), orbit to verify in 3D, then Save —
writes the extrinsic config the bridge reads.

  livox_bridge/build/livox_bridge livox_bridge/config/mid360.json &   # sidecar
  python3 livox_bridge/tools/lidar_align_3d.py                        # then open the URL
  # http://192.168.2.2:8099/   (VSCode forwards the port)
"""
import argparse
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

HDR = 20
MAGIC = b"LVXR"
BOTTOM_ID = 40
TOP_ID = 41
MAX_PTS = 150000  # subsample per lidar for the wire/GPU (WebGL handles this easily)
_TOOLS = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_TOOLS))
DEFAULT_CFG = os.path.join(_REPO, "rove_control_bridge", "config", "lidar_extrinsics.json")

LOCK = threading.Lock()
STATE = {
    "bottom": np.zeros((0, 3), np.float32), "top": np.zeros((0, 3), np.float32),
    "bgz": 0.0, "tgz": 0.0, "btilt": 0.0, "ttilt": 0.0,
}


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


def level(pts, imu):
    """Rotate so measured gravity -> straight down. Returns (pts, gravity_z, tilt)."""
    if len(imu) == 0:
        return pts, 0.0, 0.0
    g = imu[:, 3:6].mean(axis=0)
    gn = g / (np.linalg.norm(g) + 1e-9)
    tilt = float(np.degrees(np.arccos(abs(gn[2]))))
    # Livox IMU accel is specific force (points UP at rest), so level by mapping it
    # to +Z (up). A down-pointing unit (accel ≈ -Z, e.g. the bottom lidar) then gets
    # the ~180° flip it needs to read world-up. gn[2] sign => which way +Z faces.
    return (pts @ rot_a_to_b(g, np.array([0.0, 0.0, 1.0])).T).astype(np.float32), float(gn[2]), tilt


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
                    pts.setdefault(lid, []).append(
                        np.frombuffer(d, dtype="<f4", count=cnt * 3, offset=HDR).reshape(-1, 3))
                elif typ == 2:
                    imu.setdefault(lid, []).append(
                        np.frombuffer(d, dtype="<f4", count=6, offset=HDR))
        if BOTTOM_ID in pts and TOP_ID in pts:
            bp, bgz, bt = level(np.concatenate(pts[BOTTOM_ID]), np.array(imu.get(BOTTOM_ID, [])))
            tp, tgz, tt = level(np.concatenate(pts[TOP_ID]), np.array(imu.get(TOP_ID, [])))
            if len(bp) > MAX_PTS:
                bp = bp[:: len(bp) // MAX_PTS]
            if len(tp) > MAX_PTS:
                tp = tp[:: len(tp) // MAX_PTS]
            with LOCK:
                STATE.update(bottom=np.ascontiguousarray(bp), top=np.ascontiguousarray(tp),
                             bgz=bgz, tgz=tgz, btilt=bt, ttilt=tt)


PAGE = """<!doctype html><html><head><meta charset=utf-8><title>Mid-360 3D align</title>
<style>
body{margin:0;background:#0a0a0f;color:#ddd;font-family:system-ui,sans-serif;overflow:hidden}
#ui{position:fixed;top:8px;left:8px;background:#000c;padding:11px;border-radius:8px;width:312px;z-index:10}
.row{margin:5px 0}label{display:inline-block;width:42px}input[type=range]{width:178px;vertical-align:middle}
.v{display:inline-block;width:52px;text-align:right;font-variant-numeric:tabular-nums}
#stat{font-size:12px;color:#9ad;margin-bottom:4px}button{padding:6px 12px;margin:6px 6px 0 0}
.hint{font-size:12px;color:#888;margin:4px 0}#msg{color:#6f6;font-size:12px}
</style>
<script type="importmap">{"imports":{"three":"/vendor/three.module.min.js"}}</script>
</head><body>
<div id=ui>
<div id=stat>connecting…</div>
<div class=hint>cyan = bottom(.40) ref · magenta = top(.41). Drag = orbit, scroll = zoom, right-drag = pan.</div>
<div class=row><label>yaw</label><input type=range id=yaw min=-180 max=180 step=0.5 value=0><span class=v id=yawv></span>&deg;</div>
<div class=row><label>pitch</label><input type=range id=pitch min=-180 max=180 step=0.5 value=0><span class=v id=pitchv></span>&deg;</div>
<div class=row><label>roll</label><input type=range id=roll min=-180 max=180 step=0.5 value=0><span class=v id=rollv></span>&deg;</div>
<div class=row><label>dx</label><input type=range id=dx min=-3 max=3 step=0.01 value=0><span class=v id=dxv></span>m</div>
<div class=row><label>dy</label><input type=range id=dy min=-3 max=3 step=0.01 value=0><span class=v id=dyv></span>m</div>
<div class=row><label>dz</label><input type=range id=dz min=-2 max=2 step=0.01 value=0><span class=v id=dzv></span>m</div>
<div class=row>
<label style=width:auto><input type=checkbox id=live checked> live</label>
<label style=width:auto><input type=checkbox id=big> big pts</label></div>
<div class=row><button onclick=reset_()>reset</button><button onclick=save()>Save config</button><span id=msg></span></div>
</div>
<script type=module>
import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x0a0a0f);
const cam=new THREE.PerspectiveCamera(60,innerWidth/innerHeight,0.05,2000);
cam.up.set(0,0,1); cam.position.set(8,8,5);
const rend=new THREE.WebGLRenderer({antialias:true}); rend.setSize(innerWidth,innerHeight);
document.body.appendChild(rend.domElement);
const ctrl=new OrbitControls(cam,rend.domElement); ctrl.target.set(0,0,0);
const grid=new THREE.GridHelper(40,40,0x335,0x224); grid.rotation.x=Math.PI/2; scene.add(grid);
scene.add(new THREE.AxesHelper(2));
function mk(c){const g=new THREE.BufferGeometry();
  g.setAttribute('position',new THREE.BufferAttribute(new Float32Array(0),3));
  return new THREE.Points(g,new THREE.PointsMaterial({color:c,size:0.04,sizeAttenuation:true}));}
const bottom=mk(0x00d8d8); scene.add(bottom);
const topGrp=new THREE.Group(); const top=mk(0xe000c0); topGrp.add(top); scene.add(topGrp);
async function loadCloud(which,pts){const r=await fetch('/cloud?which='+which+'&t='+Date.now());
  const a=new Float32Array(await r.arrayBuffer());
  pts.geometry.setAttribute('position',new THREE.BufferAttribute(a,3));
  pts.geometry.computeBoundingSphere();}
function refresh(){return Promise.all([loadCloud('bottom',bottom),loadCloud('top',top)]);}
const ids=['yaw','pitch','roll','dx','dy','dz'];
const D=THREE.MathUtils.degToRad, gv=id=>+document.getElementById(id).value;
function apply(){topGrp.rotation.set(D(gv('roll')),D(gv('pitch')),D(gv('yaw')),'ZYX');
  topGrp.position.set(gv('dx'),gv('dy'),gv('dz'));}
ids.forEach(i=>{const e=document.getElementById(i),v=document.getElementById(i+'v');
  const u=()=>{v.textContent=(+e.value).toFixed(2);apply();};e.addEventListener('input',u);u();});
document.getElementById('big').addEventListener('change',e=>{
  const s=e.target.checked?0.1:0.04;bottom.material.size=s;top.material.size=s;});
window.reset_=()=>{ids.forEach(i=>{document.getElementById(i).value=0;
  document.getElementById(i).dispatchEvent(new Event('input'));});};
window.save=()=>fetch('/save?'+ids.map(i=>i+'='+gv(i)).join('&')).then(r=>r.json())
  .then(d=>document.getElementById('msg').textContent='saved');
async function stat(){try{const d=await(await fetch('/status')).json();
  document.getElementById('stat').textContent=
   `bottom ${d.bottom} pts (${d.bgz<0?'points down':'points up'}, tilt ${d.btilt.toFixed(1)}°) | `+
   `top ${d.top} pts (${d.tgz<0?'points down':'points up'}, tilt ${d.ttilt.toFixed(1)}°)`;
  if(document.getElementById('live').checked) await refresh();}catch(e){}}
setInterval(stat,2000);
addEventListener('resize',()=>{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();
  rend.setSize(innerWidth,innerHeight);});
refresh(); stat();
(function loop(){requestAnimationFrame(loop);ctrl.update();rend.render(scene,cam);})();
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
            self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        elif u.path.startswith("/vendor/"):
            fn = os.path.join(_TOOLS, "vendor", os.path.basename(u.path))
            if os.path.isfile(fn):
                with open(fn, "rb") as f:
                    self._send(200, "application/javascript", f.read())
            else:
                self._send(404, "text/plain", b"no vendor file")
        elif u.path == "/cloud":
            with LOCK:
                arr = STATE["top"] if q.get("which", [""])[0] == "top" else STATE["bottom"]
            self._send(200, "application/octet-stream", arr.tobytes())
        elif u.path == "/status":
            with LOCK:
                s = {"bottom": int(len(STATE["bottom"])), "top": int(len(STATE["top"])),
                     "bgz": STATE["bgz"], "tgz": STATE["tgz"],
                     "btilt": STATE["btilt"], "ttilt": STATE["ttilt"]}
            self._send(200, "application/json", json.dumps(s).encode())
        elif u.path == "/save":
            cfg = {"reference": "bottom_40",
                   "top_41": {"yaw_deg": g("yaw"), "pitch_deg": g("pitch"), "roll_deg": g("roll"),
                              "dx": g("dx"), "dy": g("dy"), "dz": g("dz")},
                   "note": "top(.41) transform relative to the IMU-leveled bottom(.40) body frame"}
            os.makedirs(os.path.dirname(DEFAULT_CFG), exist_ok=True)
            with open(DEFAULT_CFG, "w") as f:
                json.dump(cfg, f, indent=2)
            print(f"[align3d] saved {cfg['top_41']} -> {DEFAULT_CFG}")
            self._send(200, "application/json", json.dumps({"ok": True, "path": DEFAULT_CFG}).encode())
        else:
            self._send(404, "text/plain", b"not found")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pts-port", type=int, default=7020)
    ap.add_argument("--imu-port", type=int, default=7021)
    ap.add_argument("--http-port", type=int, default=8099)
    ap.add_argument("--window", type=float, default=0.8)
    a = ap.parse_args()
    threading.Thread(target=capture_loop, args=(a.pts_port, a.imu_port, a.window), daemon=True).start()
    print(f"[align3d] open http://192.168.2.2:{a.http_port}/  (Ctrl-C to stop)  saves -> {DEFAULT_CFG}")
    ThreadingHTTPServer(("0.0.0.0", a.http_port), H).serve_forever()


if __name__ == "__main__":
    main()
