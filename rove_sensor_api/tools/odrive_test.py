#!/usr/bin/env python3
"""ODrive multi-node test UI — Flask web app.

Auto-discovers every `odrive_*` sensor exposed by `rove_sensor_api`, then for
each node opens a UDP command stream and a UDP telemetry subscription.
Per-node sliders stream `input_vel` (rev/s), `input_pos` (rev) or
`input_torque` (Nm) over the node's command port. A per-node "custom field"
box streams or one-shots any other driver command field (e.g. `control_mode`),
so you're not limited to the predefined sliders. Buttons arm/disarm the axis
(axis_state 8/1) and clear errors. Headless-friendly — open the browser from
any machine.

A second page at `/config` (linked from the header) proxies the drive API's
HTTP config surface: upload `flat_endpoints.json`, read the drive config into
an editable table, write changed values back, run a calibration sequence, and
save the configuration to non-volatile memory.

Wire format matches src/protocol/packet.rs and the ODrive command surface in
src/drivers/odrive/node.rs (`input_vel`, `input_pos`, `axis_state`,
`control_mode`, `clear_errors`, ...).

Setup:
    pip install flask requests

Run:
    ./odrive_test.py                                       # localhost
    ./odrive_test.py --target 192.168.2.37 --ui-host 0.0.0.0 --ui-port 8091
"""

import argparse
import collections
import json
import socket
import struct
import sys
import threading
import time

try:
    import requests
except ImportError:
    sys.exit("requests not installed. Run: pip install requests")

try:
    from flask import Flask, jsonify, render_template_string, request
except ImportError:
    sys.exit("Flask not installed. Run: pip install flask")

PROTOCOL_VERSION = 0x01
MSG_SUBSCRIBE = 0x01
MSG_UNSUBSCRIBE = 0x02
MSG_DATA = 0x03
MSG_COMMAND = 0x10
MSG_ERROR = 0xFF

# ODrive control_mode values (see src/drivers/odrive/protocol.rs).
CONTROL_MODE_TORQUE = 1
CONTROL_MODE_VELOCITY = 2
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1

# ODrive axis_state values.
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8


def encode(mt, seq, payload):
    body = json.dumps(payload).encode() if payload is not None else b""
    return struct.pack("<BBH", PROTOCOL_VERSION, mt, seq & 0xFFFF) + body


def decode(data):
    if len(data) < 4:
        raise ValueError("short")
    ver, mt, seq = struct.unpack("<BBH", data[:4])
    if ver != PROTOCOL_VERSION:
        raise ValueError(f"bad version {ver}")
    body = data[4:]
    return mt, seq, json.loads(body) if body else None


class NodeState:
    def __init__(self, node_id: int, data_port: int, cmd_port: int, display: str):
        self.node_id = node_id
        self.data_port = data_port
        self.cmd_port = cmd_port
        self.display = display
        self.lock = threading.Lock()
        # Control mode — only one of vel / pos / torque is streamed at a time.
        # "idle" means stop sending setpoints entirely; the driver watchdog keeps
        # the last input_pos refreshed on its own.
        self.mode = "idle"  # "idle" | "velocity" | "position" | "torque"
        self.vel = 0.0    # rev/s
        self.pos = 0.0    # rev
        self.torque = 0.0  # Nm
        # Custom fields streamed into EVERY outgoing command packet. Lets the
        # operator stream arbitrary driver command fields continuously — e.g.
        # hold an `input_torque`, or push any field the driver understands.
        # field name -> JSON-typed value. See /node/<id>/custom.
        self.custom: dict = {}
        # One-shot extras to merge into the next outgoing command (axis_state,
        # clear_errors, control_mode...). Cleared after being sent once.
        self.extra: dict | None = None
        self.telem: dict = {}
        self.sent = 0
        self.errors = 0
        self.last_error: str | None = None
        self.send_times = collections.deque(maxlen=200)
        self.telem_times = collections.deque(maxlen=200)
        self.recent_errors: collections.deque = collections.deque(maxlen=20)


def discover_nodes(base_url: str, timeout: float = 3.0) -> list[NodeState]:
    """Hit /discover and pick out every `odrive_<id>` sensor."""
    r = requests.get(f"{base_url}/discover", timeout=timeout)
    r.raise_for_status()
    nodes: list[NodeState] = []
    for s in r.json().get("sensors", []):
        sid = s.get("id", "")
        if not sid.startswith("odrive_"):
            continue
        try:
            node_id = int(sid.split("_", 1)[1])
        except ValueError:
            continue
        nodes.append(NodeState(
            node_id=node_id,
            data_port=int(s["data_port"]),
            cmd_port=int(s["command_port"]),
            display=s.get("display_name", sid),
        ))
    nodes.sort(key=lambda n: n.node_id)
    return nodes


def stream_thread(host: str, rate_hz: float, n: NodeState, stop: threading.Event):
    """Tight UDP-stream loop, mode-aware (mirrors kinova_test.py).

    - **idle**: send nothing; the driver's watchdog refreshes the last
      input_pos. Use this between sessions.
    - **velocity**: stream `input_vel` while the slider is non-zero. When
      the operator zeros it we stop sending — pure streaming model.
    - **position**: only send `input_pos` when the target changes. Repeating
      the same position every tick can pile redundant entries onto an
      input-mode-2 trajectory queue if one's enabled.
    - **torque**: stream `input_torque` while the slider is non-zero (same
      pure-streaming model as velocity).

    `custom` (persistent custom fields) is merged into EVERY packet — this is
    what lets you stream arbitrary fields (hold an `input_torque`, push any
    endpoint field) regardless of slider mode. `extra` (one-shot fields like
    axis_state / clear_errors / control_mode) is merged into the next packet
    and cleared. If the only field to send is one-shot or custom, the packet
    still goes out.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    addr = (host, n.cmd_port)
    interval = 1.0 / rate_hz
    seq = 0
    next_tick = time.monotonic()
    last_mode = None
    last_pos_sent: float | None = None

    while not stop.is_set():
        with n.lock:
            mode = n.mode
            vel = n.vel
            pos = n.pos
            torque = n.torque
            custom = dict(n.custom)
            extra = n.extra
            n.extra = None

        if mode != last_mode:
            last_pos_sent = None
            last_mode = mode

        payload: dict = {}
        if mode == "velocity":
            if vel != 0.0:
                payload["input_vel"] = vel
        elif mode == "position":
            if pos != last_pos_sent:
                payload["input_pos"] = pos
                last_pos_sent = pos
        elif mode == "torque":
            if torque != 0.0:
                payload["input_torque"] = torque
        # Custom streamed fields are merged into every tick. This is what makes
        # the tool versatile: stream `input_torque`, or push any other field
        # (e.g. control_mode) continuously alongside / instead of the slider.
        if custom:
            payload.update(custom)
        if extra:
            payload.update(extra)

        if payload:
            try:
                sock.sendto(encode(MSG_COMMAND, seq, payload), addr)
                now_mono = time.monotonic()
                with n.lock:
                    n.sent += 1
                    n.send_times.append(now_mono)
                seq = (seq + 1) & 0xFFFF
            except OSError as e:
                with n.lock:
                    n.errors += 1
                    n.last_error = f"send: {e}"
                    n.recent_errors.append((time.time(), f"send: {e}"))

        # Drain acks non-blockingly to surface driver errors.
        while True:
            try:
                ack, _ = sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as e:
                with n.lock:
                    n.last_error = f"recv: {e}"
                    n.recent_errors.append((time.time(), f"recv: {e}"))
                break
            try:
                mt, _, body = decode(ack)
                if mt == MSG_ERROR and isinstance(body, dict):
                    msg = f"driver: {body.get('error', body)}"
                    with n.lock:
                        n.errors += 1
                        n.last_error = msg
                        n.recent_errors.append((time.time(), msg))
            except Exception as e:
                with n.lock:
                    n.last_error = f"decode: {e}"
                    n.recent_errors.append((time.time(), f"decode: {e}"))

        next_tick += interval
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tick = time.monotonic()

    sock.close()


def telem_thread(host: str, interval_ms: int, n: NodeState, stop: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    addr = (host, n.data_port)
    sock.sendto(encode(MSG_SUBSCRIBE, 0, {"interval_ms": interval_ms}), addr)
    while not stop.is_set():
        try:
            pkt, _ = sock.recvfrom(8192)
        except socket.timeout:
            continue
        try:
            mt, _, body = decode(pkt)
        except Exception:
            continue
        if mt == MSG_DATA and isinstance(body, dict):
            now_mono = time.monotonic()
            with n.lock:
                n.telem = body
                n.telem_times.append(now_mono)
    try:
        sock.sendto(encode(MSG_UNSUBSCRIBE, 0, None), addr)
    except OSError:
        pass
    sock.close()


INDEX = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ODrive multi-node test</title>
<style>
:root{--bg:#111;--fg:#eee;--muted:#888;--accent:#4af;--danger:#cc1e25;--panel:#1c1c1c;--border:#2a2a2a;--ok:#1a4}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font-family:-apple-system,system-ui,sans-serif}
h1{margin:0 0 12px;font-size:1.2em}
a.navlink{font-size:.62em;font-weight:normal;margin-left:10px;padding:3px 10px;border:1px solid var(--border);border-radius:4px;color:var(--accent);text-decoration:none;vertical-align:middle}
a.navlink:hover{background:#2a2a2a}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:6px;padding:12px;margin-bottom:12px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
button{background:#2a2a2a;color:var(--fg);border:1px solid var(--border);padding:6px 12px;border-radius:4px;cursor:pointer;font:inherit;font-size:.9em}
button:hover{background:#333}
button.estop{background:var(--danger);border-color:var(--danger);color:#fff;font-weight:bold;margin-left:auto;padding:8px 24px}
button.estop:hover{background:#a51820}
button.arm{background:var(--ok);border-color:var(--ok);color:#fff}
button.arm:hover{background:#176}
.node-card{margin-bottom:14px}
.node-head{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:8px}
.node-title{font-weight:bold;font-size:1.05em}
.mode-row{display:flex;gap:8px;align-items:center;margin:6px 0 8px}
.mode-row label{cursor:pointer;color:var(--muted);padding:3px 10px;border-radius:4px;border:1px solid var(--border);font-size:.9em}
.mode-row label.active{color:var(--fg);background:#2a2a2a;border-color:var(--accent)}
.mode-row input[type=radio]{display:none}
.slider-row{display:grid;grid-template-columns:80px 60px 1fr 60px 90px 40px;gap:8px;align-items:center;margin:4px 0}
.slider-row label{color:var(--muted);font-size:.85em}
.slider-row .val{font-family:ui-monospace,monospace;text-align:right;color:var(--accent)}
input[type=range]{width:100%}
input[type=number]{background:#0c0c0c;color:var(--fg);border:1px solid var(--border);border-radius:3px;padding:2px 4px;width:64px;font:inherit;font-size:.85em}
table.t{width:100%;font-family:ui-monospace,monospace;font-size:.8em;border-collapse:collapse}
table.t td{padding:2px 6px;border-bottom:1px solid var(--border)}
table.t td:first-child{color:var(--muted);width:18ch}
.status{font-family:ui-monospace,monospace;font-size:.8em;color:var(--muted)}
.status.err{color:#f66}.status.ok{color:#6c6}
.pill{display:inline-block;padding:1px 7px;border-radius:10px;font-size:.78em;font-family:ui-monospace,monospace}
.pill.armed{background:var(--ok);color:#fff}
.pill.idle{background:#555;color:#ccc}
.pill.err{background:var(--danger);color:#fff}
.pill.warn{background:#a60;color:#fff}
.errlog{max-height:120px;overflow-y:auto;font-family:ui-monospace,monospace;font-size:.75em;color:#f99;background:#0a0a0a;border:1px solid var(--border);border-radius:4px;padding:6px;margin-top:6px}
.custom-row{display:flex;gap:6px;align-items:center;flex-wrap:wrap;margin:6px 0}
.custom-row input[type=text]{background:#0c0c0c;color:var(--fg);border:1px solid var(--border);border-radius:3px;padding:3px 6px;font:inherit;font-size:.85em}
.cf-active{font-family:ui-monospace,monospace;font-size:.78em;color:var(--muted)}
.cf-chip{display:inline-block;background:#0a2a0a;border:1px solid var(--ok);border-radius:10px;padding:1px 8px;margin:2px}
.cf-chip a{color:#f66;cursor:pointer;font-weight:bold;text-decoration:none;margin-left:2px}
</style></head><body>

<h1>ODrive multi-node test — {{target}} <a href="/config" class="navlink">⚙ Config / calibrate →</a></h1>

<div class="panel"><div class="row">
  <button onclick="zeroAll()">Zero all sliders</button>
  <button onclick="idleAll()">Idle all axes (state=1)</button>
  <button onclick="armAll()" class="arm">Arm all axes (state=8)</button>
  <button onclick="clearErrorsAll()">Clear errors all</button>
  <button onclick="estopAll()" class="estop">⚠ ESTOP ALL</button>
</div></div>

<div id="nodes"></div>

<script>
const NODES = {{nodes_json | safe}};
const HZ = {{rate_hz}};
const DEFAULT_VEL_LIMIT = {{max_vel}};
const DEFAULT_POS_LIMIT = {{max_pos}};
const DEFAULT_TORQUE_LIMIT = {{max_torque}};

// Per-node slider state mirror — used when constructing /cmd POSTs.
const nodeMode = {};      // node_id -> "idle"|"velocity"|"position"|"torque"
const nodeVel = {};       // node_id -> number (rev/s)
const nodePos = {};       // node_id -> number (rev)
const nodeTorque = {};    // node_id -> number (Nm)
const nodeVelMax = {};    // node_id -> slider range
const nodePosMax = {};    // node_id -> slider range
const nodeTorqueMax = {}; // node_id -> slider range

function el(html){const t=document.createElement('template');t.innerHTML=html.trim();return t.content.firstChild;}
function escapeHtml(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}

function buildNode(n){
  const id=n.node_id;
  nodeMode[id]='idle';nodeVel[id]=0;nodePos[id]=0;nodeTorque[id]=0;
  nodeVelMax[id]=DEFAULT_VEL_LIMIT;nodePosMax[id]=DEFAULT_POS_LIMIT;nodeTorqueMax[id]=DEFAULT_TORQUE_LIMIT;
  const card=el(`<div class="panel node-card" data-node="${id}">
    <div class="node-head">
      <span class="node-title">${escapeHtml(n.display)} (cmd:${n.cmd_port} data:${n.data_port})</span>
      <span class="pill idle" id="state-${id}">state ?</span>
      <span class="pill warn" id="err-pill-${id}" style="display:none">errors</span>
      <button onclick="armOne(${id})" class="arm">Arm (8)</button>
      <button onclick="idleOne(${id})">Idle (1)</button>
      <button onclick="clearErrorsOne(${id})">Clear errors</button>
      <button onclick="estopOne(${id})" class="estop" style="margin-left:auto;padding:4px 14px">ESTOP</button>
    </div>
    <div class="mode-row">
      <span style="color:var(--muted);font-size:.85em">Mode:</span>
      <label class="active" data-mode="idle"><input type="radio" name="mode-${id}" value="idle" checked>idle</label>
      <label data-mode="velocity"><input type="radio" name="mode-${id}" value="velocity">velocity (rev/s)</label>
      <label data-mode="position"><input type="radio" name="mode-${id}" value="position">position (rev)</label>
      <label data-mode="torque"><input type="radio" name="mode-${id}" value="torque">torque (Nm)</label>
      <span style="margin-left:14px;color:var(--muted);font-size:.8em">vel ±</span>
      <input type="number" id="vmax-${id}" value="${DEFAULT_VEL_LIMIT}" step="0.5" min="0.1" style="width:60px">
      <span style="color:var(--muted);font-size:.8em">pos ±</span>
      <input type="number" id="pmax-${id}" value="${DEFAULT_POS_LIMIT}" step="0.5" min="0.1" style="width:60px">
      <span style="color:var(--muted);font-size:.8em">tq ±</span>
      <input type="number" id="tmax-${id}" value="${DEFAULT_TORQUE_LIMIT}" step="0.1" min="0.01" style="width:60px">
    </div>
    <div class="slider-row" id="vel-row-${id}" style="display:none">
      <label>input_vel</label>
      <span style="text-align:right;color:var(--muted);font-size:.8em" id="vmin-lbl-${id}">−${DEFAULT_VEL_LIMIT}</span>
      <input type="range" id="vel-${id}" min="${-DEFAULT_VEL_LIMIT}" max="${DEFAULT_VEL_LIMIT}" step="0.05" value="0">
      <span style="color:var(--muted);font-size:.8em" id="vmax-lbl-${id}">+${DEFAULT_VEL_LIMIT}</span>
      <span class="val" id="vel-val-${id}">0.00 rev/s</span>
      <button onclick="zeroVel(${id})">0</button>
    </div>
    <div class="slider-row" id="pos-row-${id}" style="display:none">
      <label>input_pos</label>
      <span style="text-align:right;color:var(--muted);font-size:.8em" id="pmin-lbl-${id}">−${DEFAULT_POS_LIMIT}</span>
      <input type="range" id="pos-${id}" min="${-DEFAULT_POS_LIMIT}" max="${DEFAULT_POS_LIMIT}" step="0.01" value="0">
      <span style="color:var(--muted);font-size:.8em" id="pmax-lbl-${id}">+${DEFAULT_POS_LIMIT}</span>
      <span class="val" id="pos-val-${id}">0.00 rev</span>
      <button onclick="zeroPos(${id})">0</button>
    </div>
    <div class="slider-row" id="tq-row-${id}" style="display:none">
      <label>input_torque</label>
      <span style="text-align:right;color:var(--muted);font-size:.8em" id="tmin-lbl-${id}">−${DEFAULT_TORQUE_LIMIT}</span>
      <input type="range" id="tq-${id}" min="${-DEFAULT_TORQUE_LIMIT}" max="${DEFAULT_TORQUE_LIMIT}" step="0.01" value="0">
      <span style="color:var(--muted);font-size:.8em" id="tmax-lbl-${id}">+${DEFAULT_TORQUE_LIMIT}</span>
      <span class="val" id="tq-val-${id}">0.00 Nm</span>
      <button onclick="zeroTorque(${id})">0</button>
    </div>
    <div class="custom-row">
      <span style="color:var(--muted);font-size:.85em">custom field:</span>
      <input type="text" id="cf-name-${id}" placeholder="field (e.g. input_torque)" style="width:170px">
      <input type="text" id="cf-val-${id}" placeholder="value (e.g. 0.5 or 15)" style="width:130px">
      <button onclick="customSend(${id},false)" title="Merge into the next packet only">Send once</button>
      <button onclick="customSend(${id},true)" class="arm" title="Stream in every packet until removed">Stream +</button>
      <button onclick="customClear(${id})">Clear streamed</button>
      <span class="cf-active" id="cf-active-${id}"></span>
    </div>
    <table class="t"><tbody id="telem-${id}"></tbody></table>
    <div class="status" id="status-${id}">connecting…</div>
    <div class="errlog" id="errlog-${id}" style="display:none">no errors yet</div>
  </div>`);

  document.getElementById('nodes').appendChild(card);

  card.querySelectorAll(`input[name=mode-${id}]`).forEach(r=>{
    r.addEventListener('change',e=>setMode(id,e.target.value));
  });
  document.getElementById(`vel-${id}`).addEventListener('input',e=>{
    nodeVel[id]=parseFloat(e.target.value);
    document.getElementById(`vel-val-${id}`).textContent=nodeVel[id].toFixed(2)+' rev/s';
    pushVel(id);
  });
  document.getElementById(`pos-${id}`).addEventListener('input',e=>{
    nodePos[id]=parseFloat(e.target.value);
    document.getElementById(`pos-val-${id}`).textContent=nodePos[id].toFixed(2)+' rev';
    pushPos(id);
  });
  document.getElementById(`tq-${id}`).addEventListener('input',e=>{
    nodeTorque[id]=parseFloat(e.target.value);
    document.getElementById(`tq-val-${id}`).textContent=nodeTorque[id].toFixed(2)+' Nm';
    pushTorque(id);
  });
  document.getElementById(`vmax-${id}`).addEventListener('change',e=>{
    const v=Math.max(0.1,parseFloat(e.target.value)||DEFAULT_VEL_LIMIT);
    nodeVelMax[id]=v;
    const s=document.getElementById(`vel-${id}`);
    s.min=-v;s.max=v;
    document.getElementById(`vmin-lbl-${id}`).textContent=`−${v}`;
    document.getElementById(`vmax-lbl-${id}`).textContent=`+${v}`;
  });
  document.getElementById(`pmax-${id}`).addEventListener('change',e=>{
    const v=Math.max(0.1,parseFloat(e.target.value)||DEFAULT_POS_LIMIT);
    nodePosMax[id]=v;
    const s=document.getElementById(`pos-${id}`);
    s.min=-v;s.max=v;
    document.getElementById(`pmin-lbl-${id}`).textContent=`−${v}`;
    document.getElementById(`pmax-lbl-${id}`).textContent=`+${v}`;
  });
  document.getElementById(`tmax-${id}`).addEventListener('change',e=>{
    const v=Math.max(0.01,parseFloat(e.target.value)||DEFAULT_TORQUE_LIMIT);
    nodeTorqueMax[id]=v;
    const s=document.getElementById(`tq-${id}`);
    s.min=-v;s.max=v;
    document.getElementById(`tmin-lbl-${id}`).textContent=`−${v}`;
    document.getElementById(`tmax-lbl-${id}`).textContent=`+${v}`;
  });
}

// Latest-wins POST per node so sliders can't queue up old values.
const pushAbort={};
async function postCmd(id, body){
  if(pushAbort[id]){pushAbort[id].abort();}
  pushAbort[id]=new AbortController();
  try{
    await fetch(`/node/${id}/cmd`,{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify(body),signal:pushAbort[id].signal});
  }catch(e){/* aborted by next push or network blip */}
}

function pushVel(id){postCmd(id,{vel:nodeVel[id]});}
function pushPos(id){postCmd(id,{pos:nodePos[id]});}
function pushTorque(id){postCmd(id,{torque:nodeTorque[id]});}
function postAction(id, extra){
  return fetch(`/node/${id}/action`,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(extra)});
}

// Parse a free-text custom value into a JSON-typed value: bool, number, else string.
function parseCustomVal(s){
  s=String(s).trim();
  if(s==='true')return true;
  if(s==='false')return false;
  if(s!==''){const n=Number(s);if(!isNaN(n))return n;}
  return s;
}
function customSend(id, stream){
  const field=document.getElementById(`cf-name-${id}`).value.trim();
  if(!field){alert('Enter a field name (e.g. input_torque or control_mode)');return;}
  const value=parseCustomVal(document.getElementById(`cf-val-${id}`).value);
  fetch(`/node/${id}/custom`,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({action:'add',field,value,stream})})
    .then(r=>r.json()).then(j=>renderCustom(id,j.custom||{})).catch(e=>{});
}
function customClear(id){
  fetch(`/node/${id}/custom`,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({action:'clear'})})
    .then(r=>r.json()).then(j=>renderCustom(id,j.custom||{})).catch(e=>{});
}
function customRemove(id, field){
  fetch(`/node/${id}/custom`,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({action:'remove',field})})
    .then(r=>r.json()).then(j=>renderCustom(id,j.custom||{})).catch(e=>{});
}
const customShown={};  // node_id -> JSON string of last-rendered custom, to avoid needless DOM churn
function renderCustom(id, custom){
  const sig=JSON.stringify(custom);
  if(customShown[id]===sig)return;
  customShown[id]=sig;
  const elc=document.getElementById(`cf-active-${id}`);
  if(!elc)return;
  const keys=Object.keys(custom);
  if(!keys.length){elc.innerHTML='';return;}
  elc.innerHTML='streaming: '+keys.map(k=>
    `<span class="cf-chip">${escapeHtml(k)}=${escapeHtml(String(custom[k]))}`
    +` <a onclick="customRemove(${id},'${escapeHtml(k)}')">×</a></span>`).join(' ');
}

async function setMode(id, newMode){
  // Switching modes always passes through "idle" (zero pending setpoints,
  // tell the server to stop streaming) before the new mode comes online.
  // For position mode we seed `input_pos` with the live `pos_estimate` so
  // the axis doesn't jump on the first slider tick.
  document.querySelectorAll(`#nodes [data-node="${id}"] .mode-row label`).forEach(l=>{
    l.classList.toggle('active', l.dataset.mode===newMode);
  });
  document.getElementById(`vel-row-${id}`).style.display = (newMode==='velocity'?'grid':'none');
  document.getElementById(`pos-row-${id}`).style.display = (newMode==='position'?'grid':'none');
  document.getElementById(`tq-row-${id}`).style.display = (newMode==='torque'?'grid':'none');

  // Always reset the velocity & torque sliders to zero when (re)entering any
  // mode — never start streaming a stale non-zero setpoint.
  document.getElementById(`vel-${id}`).value=0;nodeVel[id]=0;
  document.getElementById(`vel-val-${id}`).textContent='0.00 rev/s';
  document.getElementById(`tq-${id}`).value=0;nodeTorque[id]=0;
  document.getElementById(`tq-val-${id}`).textContent='0.00 Nm';

  if(newMode==='position'){
    // Seed slider from telemetry pos_estimate so the axis doesn't lurch.
    let seed=0;
    try{
      const j=await(await fetch('/state')).json();
      const t=(j.nodes[id]||{}).telem||{};
      if(typeof t.pos_estimate==='number')seed=t.pos_estimate;
    }catch(e){}
    nodePos[id]=seed;
    const s=document.getElementById(`pos-${id}`);
    // Make sure the slider range covers the seed.
    if(Math.abs(seed)>nodePosMax[id]){
      const v=Math.ceil(Math.abs(seed)*1.5);
      nodePosMax[id]=v;document.getElementById(`pmax-${id}`).value=v;
      s.min=-v;s.max=v;
      document.getElementById(`pmin-lbl-${id}`).textContent=`−${v}`;
      document.getElementById(`pmax-lbl-${id}`).textContent=`+${v}`;
    }
    s.value=seed;
    document.getElementById(`pos-val-${id}`).textContent=seed.toFixed(2)+' rev';
  }
  nodeMode[id]=newMode;
  await fetch(`/node/${id}/mode`,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode:newMode, seed_pos:nodePos[id]})});
}

function zeroVel(id){
  document.getElementById(`vel-${id}`).value=0;nodeVel[id]=0;
  document.getElementById(`vel-val-${id}`).textContent='0.00 rev/s';pushVel(id);
}
function zeroPos(id){
  document.getElementById(`pos-${id}`).value=0;nodePos[id]=0;
  document.getElementById(`pos-val-${id}`).textContent='0.00 rev';pushPos(id);
}
function zeroTorque(id){
  document.getElementById(`tq-${id}`).value=0;nodeTorque[id]=0;
  document.getElementById(`tq-val-${id}`).textContent='0.00 Nm';pushTorque(id);
}
function zeroAll(){NODES.forEach(n=>{zeroVel(n.node_id);zeroPos(n.node_id);zeroTorque(n.node_id);});}

// Arm with the control_mode that matches the active slider mode:
// torque→1 (TORQUE_CONTROL), position→3 (POSITION_CONTROL), else→2 (VELOCITY_CONTROL).
function armOne(id){
  const m=nodeMode[id];
  const cm=m==='torque'?1:m==='position'?3:2;
  postAction(id,{axis_state:8,control_mode:cm,input_mode:1});
}
function idleOne(id){postAction(id,{axis_state:1});}
function clearErrorsOne(id){postAction(id,{clear_errors:true});}
async function estopOne(id){
  // ESTOP via the dedicated /estop HTTP endpoint — the rove_sensor_api
  // exposes that out of band of the UDP command port.
  try{await fetch(`/node/${id}/estop`,{method:'POST'});}catch(e){}
}
function armAll(){NODES.forEach(n=>armOne(n.node_id));}
function idleAll(){NODES.forEach(n=>idleOne(n.node_id));}
function clearErrorsAll(){NODES.forEach(n=>clearErrorsOne(n.node_id));}
function estopAll(){if(confirm('Send ESTOP to ALL ODrive nodes?'))NODES.forEach(n=>estopOne(n.node_id));}

function fmt(v,unit){return (typeof v==='number'?v.toFixed(3):'—')+(unit?' '+unit:'');}
// Render an error bitfield together with the decoded flag names the API now
// returns in the matching *_words field.
function errCell(bits, words){
  const hex='0x'+((bits>>>0)||0).toString(16);
  return (words&&words.length)?hex+'  '+escapeHtml(words.join(', ')):hex;
}
async function poll(){
  let j;try{j=await(await fetch('/state')).json();}catch(e){return;}
  for(const n of NODES){
    const id=n.node_id;
    const ns=j.nodes[id];if(!ns)continue;
    const t=ns.telem||{};
    const rows=[
      ['axis_state',     `${t.axis_state??'—'} ${axisName(t.axis_state)}`],
      ['axis_error',     errCell(t.axis_error, t.axis_error_words)],
      ['active_errors',  errCell(t.active_errors, t.active_errors_words)],
      ['pos_estimate',   fmt(t.pos_estimate,'rev')],
      ['vel_estimate',   fmt(t.vel_estimate,'rev/s')],
      ['iq_measured',    fmt(t.iq_measured,'A')],
      ['torque_estimate',fmt(t.torque_estimate,'Nm')],
      ['bus_voltage',    fmt(t.bus_voltage,'V')],
      ['fet_temp',       fmt(t.fet_temp,'°C')],
      ['rates',          `cmd ${(ns.send_hz||0).toFixed(1)} Hz | telem ${(ns.telem_hz||0).toFixed(1)} Hz | last ${ns.last_telem_age_ms!=null?ns.last_telem_age_ms.toFixed(0)+' ms':'—'}`],
    ];
    document.getElementById(`telem-${id}`).innerHTML=rows.map(([k,v])=>`<tr><td>${k}</td><td>${v}</td></tr>`).join('');

    const pill=document.getElementById(`state-${id}`);
    const st=t.axis_state;
    pill.textContent='state '+(st??'?');
    pill.className='pill '+(st===8?'armed':st===1?'idle':'warn');

    const errPill=document.getElementById(`err-pill-${id}`);
    const hasErr=(t.axis_error&&t.axis_error!==0)||(t.active_errors&&t.active_errors!==0);
    errPill.style.display=hasErr?'inline-block':'none';
    if(hasErr)errPill.className='pill err';

    const status=document.getElementById(`status-${id}`);
    let s=`mode=${nodeMode[id]} | sent=${ns.sent} errors=${ns.errors}`;
    if(ns.last_error)s+=`  |  last: ${ns.last_error}`;
    status.textContent=s;status.className='status '+(ns.last_error?'err':'ok');

    renderCustom(id, ns.custom||{});

    const errs=ns.recent_errors||[];
    const log=document.getElementById(`errlog-${id}`);
    if(errs.length===0){log.style.display='none';}
    else{
      log.style.display='block';
      log.innerHTML=errs.slice().reverse().map(([ts,msg])=>{
        const dt=new Date(ts*1000);
        const hh=String(dt.getHours()).padStart(2,'0');
        const mm=String(dt.getMinutes()).padStart(2,'0');
        const ss=String(dt.getSeconds()).padStart(2,'0');
        return `<div>${hh}:${mm}:${ss}  ${escapeHtml(msg)}</div>`;
      }).join('');
    }
  }
}
function axisName(s){
  return ({1:'(idle)',3:'(full_calib)',4:'(motor_calib)',6:'(enc_idx)',7:'(enc_off)',8:'(closed_loop)'}[s])||'';
}

NODES.forEach(buildNode);
setInterval(poll,200);
poll();
</script></body></html>"""


CONFIG = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ODrive config — {{target}}</title>
<style>
:root{--bg:#111;--fg:#eee;--muted:#888;--accent:#4af;--danger:#cc1e25;--panel:#1c1c1c;--border:#2a2a2a;--ok:#1a4}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font-family:-apple-system,system-ui,sans-serif}
h1{margin:0 0 12px;font-size:1.2em}
h2{font-size:1em;margin:0 0 8px}
a.navlink{font-size:.62em;font-weight:normal;margin-left:10px;padding:3px 10px;border:1px solid var(--border);border-radius:4px;color:var(--accent);text-decoration:none;vertical-align:middle}
a.navlink:hover{background:#2a2a2a}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:6px;padding:12px;margin-bottom:12px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
button{background:#2a2a2a;color:var(--fg);border:1px solid var(--border);padding:6px 12px;border-radius:4px;cursor:pointer;font:inherit;font-size:.9em}
button:hover{background:#333}
button:disabled{opacity:.5;cursor:default}
button.primary{background:var(--ok);border-color:var(--ok);color:#fff}
button.danger{background:var(--danger);border-color:var(--danger);color:#fff}
select,input[type=text],input[type=file],textarea{background:#0c0c0c;color:var(--fg);border:1px solid var(--border);border-radius:4px;padding:5px 8px;font:inherit;font-size:.88em}
textarea{width:100%;min-height:80px;font-family:ui-monospace,monospace}
.muted{color:var(--muted);font-size:.82em}
.status{font-family:ui-monospace,monospace;font-size:.82em;margin-top:6px;word-break:break-word}
.status.ok{color:#6c6}.status.err{color:#f66}.status.busy{color:var(--accent)}
.cfgtable{max-height:460px;overflow-y:auto;margin-top:8px;border:1px solid var(--border);border-radius:4px}
table.cfg{width:100%;border-collapse:collapse;font-family:ui-monospace,monospace;font-size:.8em}
table.cfg td{padding:2px 6px;border-bottom:1px solid var(--border);vertical-align:top}
table.cfg td.k{color:var(--muted);width:62%;word-break:break-word}
table.cfg input{width:100%;background:#0c0c0c;color:var(--accent);border:1px solid var(--border);border-radius:3px;padding:2px 4px;font:inherit;font-size:.95em}
table.cfg input.dirty{border-color:var(--ok);color:#9f9}
.cfgerr{color:#f66}
.sep{flex:1}
</style></head><body>

<h1>ODrive config / calibrate — {{target}} <a href="/" class="navlink">← control</a></h1>

<div class="panel">
  <h2>1 · Upload endpoint map (flat_endpoints.json)</h2>
  <div class="muted">Required before read / write / save can work. Loads the endpoint id↔path map into the drive API
    (applies to <b>all</b> nodes). Download the file matching your hw + fw from docs.odriverobotics.com, or run with
    <code>ODRIVE_HW_VERSION</code>+<code>ODRIVE_FW_VERSION</code> on the API and skip this.</div>
  <div class="row" style="margin-top:8px">
    <input type="file" id="epfile" accept=".json,application/json">
    <button id="epupload" onclick="uploadEndpoints()">Upload to drive API</button>
  </div>
  <div class="status" id="ep-status"></div>
</div>

<div class="panel">
  <div class="row">
    <h2 style="margin:0">Node</h2>
    <select id="node"></select>
    <span class="muted">all actions below target the selected node</span>
  </div>
</div>

<div class="panel">
  <h2>2 · Read / write drive config</h2>
  <div class="row">
    <button id="readbtn" onclick="readConfig()">Read config from drive</button>
    <input type="text" id="filter" placeholder="filter paths…" oninput="applyFilter()" style="width:220px">
    <span class="sep"></span>
    <button class="primary" id="writebtn" onclick="writeChanged()" disabled>Write changed values</button>
  </div>
  <div class="status" id="cfg-status"></div>
  <div class="cfgtable"><table class="cfg"><tbody id="cfg-body"></tbody></table></div>
  <details style="margin-top:10px">
    <summary class="muted" style="cursor:pointer">Advanced: write raw JSON (full flat-endpoint paths)</summary>
    <textarea id="rawjson" placeholder='{"axis0.controller.config.vel_limit": 20.0, "axis0.config.motor.pole_pairs": 7}'></textarea>
    <div class="row"><button onclick="writeRaw()">Write raw JSON</button></div>
    <div class="status" id="raw-status"></div>
  </details>
</div>

<div class="panel">
  <h2>3 · Calibrate</h2>
  <div class="muted">Drive must be in <b>Idle</b>. Runs asynchronously — watch <code>axis_state</code> return to 1 on the control page.</div>
  <div class="row" style="margin-top:8px">
    <select id="caltype">
      <option value="full">full — FullCalibrationSequence (3)</option>
      <option value="motor">motor — MotorCalibration (4)</option>
      <option value="encoder_index">encoder_index — EncoderIndexSearch (6)</option>
      <option value="encoder_offset">encoder_offset — EncoderOffsetCalibration (7)</option>
      <option value="harmonic">harmonic — HarmonicCalibration (15)</option>
      <option value="harmonic_commutation">harmonic_commutation — HarmonicCalibrationCommutation (16)</option>
    </select>
    <button onclick="calibrate()">Start calibration</button>
  </div>
  <div class="status" id="cal-status"></div>
</div>

<div class="panel">
  <h2>4 · Save configuration to drive</h2>
  <div class="muted">Persists the current config + calibration to non-volatile memory (ODrive <code>save_configuration()</code>).
    Drive must be <b>Idle</b>; it may briefly drop off CAN while writing flash.</div>
  <div class="row" style="margin-top:8px">
    <button class="primary" onclick="saveConfig()">Save config to drive</button>
  </div>
  <div class="status" id="save-status"></div>
</div>

<script>
const NODES = {{nodes_json | safe}};

function escapeHtml(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
function nodeId(){return parseInt(document.getElementById('node').value,10);}
function setStatus(id, msg, cls){const e=document.getElementById(id);e.textContent=msg;e.className='status '+(cls||'');}
// Parse a value-input string into a JSON-typed value (bool / number / string).
function parseVal(s){
  s=String(s).trim();
  if(s==='true')return true; if(s==='false')return false;
  if(s!==''&&!isNaN(Number(s)))return Number(s);
  return s;  // includes "inf"/"-inf"/"nan" — left as-is (drive will reject if non-numeric for that type)
}

// Populate node dropdown.
(function(){
  const sel=document.getElementById('node');
  for(const n of NODES){
    const o=document.createElement('option');
    o.value=n.node_id; o.textContent=`${n.display} (node ${n.node_id})`;
    sel.appendChild(o);
  }
  if(!NODES.length){sel.innerHTML='<option>no odrive nodes discovered</option>';sel.disabled=true;}
})();

async function uploadEndpoints(){
  const f=document.getElementById('epfile').files[0];
  if(!f){setStatus('ep-status','choose a flat_endpoints.json file first','err');return;}
  setStatus('ep-status',`uploading ${f.name}…`,'busy');
  const text=await f.text();
  try{
    const r=await fetch('/upload_endpoints',{method:'POST',headers:{'Content-Type':'application/json'},body:text});
    const j=await r.json();
    if(r.ok)setStatus('ep-status',`loaded ${j.loaded ?? '?'} endpoints — ${j.status ?? 'ok'}`,'ok');
    else setStatus('ep-status',`error: ${j.error ?? JSON.stringify(j)}`,'err');
  }catch(e){setStatus('ep-status',`upload failed: ${e}`,'err');}
}

let cfgRows=[];  // {path, input?, errCell?}
async function readConfig(){
  const id=nodeId(); if(isNaN(id))return;
  const btn=document.getElementById('readbtn'); btn.disabled=true;
  setStatus('cfg-status','reading config from drive (this can take 10–30 s)…','busy');
  document.getElementById('cfg-body').innerHTML='';
  try{
    const r=await fetch(`/node/${id}/config`);
    const j=await r.json();
    if(!r.ok){setStatus('cfg-status',`error: ${j.error ?? JSON.stringify(j)}`,'err');return;}
    renderConfig(j);
  }catch(e){setStatus('cfg-status',`read failed: ${e}`,'err');}
  finally{btn.disabled=false;}
}

function renderConfig(obj){
  const body=document.getElementById('cfg-body'); body.innerHTML=''; cfgRows=[];
  let ok=0, errs=0;
  const keys=Object.keys(obj).filter(k=>k!=='node_id').sort();
  for(const path of keys){
    const v=obj[path];
    const tr=document.createElement('tr');
    const tdK=document.createElement('td'); tdK.className='k'; tdK.textContent=path;
    const tdV=document.createElement('td');
    if(v&&typeof v==='object'&&'error' in v){
      tdV.innerHTML=`<span class="cfgerr">err: ${escapeHtml(v.error)}</span>`;
      errs++;
      cfgRows.push({path, errCell:true});
    }else{
      const inp=document.createElement('input');
      inp.type='text';
      inp.value=(typeof v==='boolean')?String(v):String(v);
      inp.dataset.path=path; inp.dataset.orig=inp.value;
      inp.addEventListener('input',()=>{inp.classList.toggle('dirty',inp.value!==inp.dataset.orig);});
      tdV.appendChild(inp);
      cfgRows.push({path, input:inp});
      ok++;
    }
    tr.appendChild(tdK); tr.appendChild(tdV); body.appendChild(tr);
  }
  document.getElementById('writebtn').disabled = ok===0;
  setStatus('cfg-status',`read ${ok} values${errs?`, ${errs} unreadable`:''}. Edit fields then “Write changed values”.`,'ok');
  applyFilter();
}

function applyFilter(){
  const q=document.getElementById('filter').value.toLowerCase();
  for(const tr of document.getElementById('cfg-body').children){
    const k=tr.firstChild.textContent.toLowerCase();
    tr.style.display = (!q||k.includes(q))?'':'none';
  }
}

async function writeChanged(){
  const id=nodeId(); if(isNaN(id))return;
  const changes={};
  for(const r of cfgRows){
    if(r.input && r.input.value!==r.input.dataset.orig){
      changes[r.path]=parseVal(r.input.value);
    }
  }
  if(!Object.keys(changes).length){setStatus('cfg-status','no changed values to write','err');return;}
  setStatus('cfg-status',`writing ${Object.keys(changes).length} value(s)…`,'busy');
  try{
    const r=await fetch(`/node/${id}/config`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(changes)});
    const j=await r.json();
    if(!r.ok){setStatus('cfg-status',`error: ${j.error ?? JSON.stringify(j)}`,'err');return;}
    const res=j.result||j;
    const wrote=(res.written||[]).length;
    const errc=Object.keys(res.errors||{}).length;
    // Mark written rows clean.
    for(const r2 of cfgRows){
      if(r2.input && (res.written||[]).includes(r2.path)){
        r2.input.dataset.orig=r2.input.value; r2.input.classList.remove('dirty');
      }
    }
    setStatus('cfg-status',`wrote ${wrote} value(s)${errc?`, ${errc} error(s): `+escapeHtml(JSON.stringify(res.errors)):''}`, errc?'err':'ok');
  }catch(e){setStatus('cfg-status',`write failed: ${e}`,'err');}
}

async function writeRaw(){
  const id=nodeId(); if(isNaN(id))return;
  let payload;
  try{payload=JSON.parse(document.getElementById('rawjson').value);}
  catch(e){setStatus('raw-status',`invalid JSON: ${e}`,'err');return;}
  setStatus('raw-status','writing…','busy');
  try{
    const r=await fetch(`/node/${id}/config`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
    const j=await r.json();
    setStatus('raw-status', r.ok?`done: ${JSON.stringify(j.result||j)}`:`error: ${j.error??JSON.stringify(j)}`, r.ok?'ok':'err');
  }catch(e){setStatus('raw-status',`write failed: ${e}`,'err');}
}

async function calibrate(){
  const id=nodeId(); if(isNaN(id))return;
  const type=document.getElementById('caltype').value;
  if(!confirm(`Start "${type}" calibration on node ${id}? The motor may move. Drive must be Idle.`))return;
  setStatus('cal-status',`starting ${type} calibration…`,'busy');
  try{
    const r=await fetch(`/node/${id}/calibrate`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({type})});
    const j=await r.json();
    setStatus('cal-status', r.ok?`${type}: ${JSON.stringify(j.result||j)}`:`error: ${j.error??JSON.stringify(j)}`, r.ok?'ok':'err');
  }catch(e){setStatus('cal-status',`calibrate failed: ${e}`,'err');}
}

async function saveConfig(){
  const id=nodeId(); if(isNaN(id))return;
  if(!confirm(`Save configuration to non-volatile memory on node ${id}? Drive must be Idle and may briefly drop off CAN.`))return;
  setStatus('save-status','saving configuration…','busy');
  try{
    const r=await fetch(`/node/${id}/save_config`,{method:'POST'});
    const j=await r.json();
    setStatus('save-status', r.ok?`saved: ${JSON.stringify(j.result||j)}`:`error: ${j.error??JSON.stringify(j)}`, r.ok?'ok':'err');
  }catch(e){setStatus('save-status',`save failed: ${e}`,'err');}
}
</script></body></html>"""


def make_app(host: str, nodes: list[NodeState], rate_hz: float, max_vel: float, max_pos: float,
             max_torque: float):
    app = Flask(__name__)
    by_id = {n.node_id: n for n in nodes}

    def find(node_id: int) -> NodeState | None:
        return by_id.get(node_id)

    @app.get("/")
    def index():
        nodes_json = json.dumps([
            {"node_id": n.node_id, "display": n.display,
             "data_port": n.data_port, "cmd_port": n.cmd_port}
            for n in nodes
        ])
        target = f"{host} ({len(nodes)} node{'s' if len(nodes) != 1 else ''})"
        return render_template_string(
            INDEX, target=target, nodes_json=nodes_json,
            rate_hz=rate_hz, max_vel=max_vel, max_pos=max_pos, max_torque=max_torque,
        )

    # ── Config / calibrate page (proxies to the rove_sensor_api HTTP API) ──────

    def _api(path: str) -> str:
        return f"http://{host}:{api_http_port}{path}"

    def _proxy_json(method: str, path: str, **kw):
        """Forward a request to the rove_sensor_api and relay its JSON + status."""
        try:
            r = requests.request(method, _api(path), **kw)
        except Exception as e:
            return jsonify({"error": f"cannot reach drive API at {host}:{api_http_port}: {e}"}), 502
        try:
            body = r.json()
        except ValueError:
            body = {"raw": r.text}
        return jsonify(body), r.status_code

    @app.get("/config")
    def config_page():
        nodes_json = json.dumps([
            {"node_id": n.node_id, "display": n.display,
             "data_port": n.data_port, "cmd_port": n.cmd_port}
            for n in nodes
        ])
        target = f"{host} ({len(nodes)} node{'s' if len(nodes) != 1 else ''})"
        return render_template_string(CONFIG, target=target, nodes_json=nodes_json)

    @app.post("/upload_endpoints")
    def upload_endpoints():
        """Upload flat_endpoints.json to the drive API (global, all nodes)."""
        data = request.get_data()  # raw file bytes
        if not data:
            return jsonify({"error": "empty body — choose a flat_endpoints.json file"}), 400
        return _proxy_json("POST", "/odrive/endpoints", data=data,
                           headers={"Content-Type": "application/json"}, timeout=20)

    @app.get("/node/<int:nid>/config")
    def node_read_config(nid):
        if find(nid) is None:
            return jsonify({"error": "unknown node"}), 404
        # Reading every config endpoint over SDO is sequential and can be slow.
        return _proxy_json("GET", f"/odrive_{nid}/config", timeout=90)

    @app.post("/node/<int:nid>/config")
    def node_write_config(nid):
        if find(nid) is None:
            return jsonify({"error": "unknown node"}), 404
        payload = request.get_json(force=True, silent=True) or {}
        return _proxy_json("POST", f"/odrive_{nid}/config", json=payload, timeout=30)

    @app.post("/node/<int:nid>/calibrate")
    def node_calibrate(nid):
        if find(nid) is None:
            return jsonify({"error": "unknown node"}), 404
        payload = request.get_json(force=True, silent=True) or {}
        return _proxy_json("POST", f"/odrive_{nid}/calibrate", json=payload, timeout=15)

    @app.post("/node/<int:nid>/save_config")
    def node_save_config(nid):
        if find(nid) is None:
            return jsonify({"error": "unknown node"}), 404
        return _proxy_json("POST", f"/odrive_{nid}/save_config", timeout=20)

    @app.post("/node/<int:nid>/cmd")
    def post_cmd(nid):
        n = find(nid)
        if n is None:
            return jsonify({"error": "unknown node"}), 404
        body = request.get_json(force=True, silent=True) or {}
        with n.lock:
            if "vel" in body:
                n.vel = float(body["vel"])
            if "pos" in body:
                n.pos = float(body["pos"])
            if "torque" in body:
                n.torque = float(body["torque"])
        return jsonify({"ok": True})

    @app.post("/node/<int:nid>/mode")
    def post_mode(nid):
        n = find(nid)
        if n is None:
            return jsonify({"error": "unknown node"}), 404
        body = request.get_json(force=True, silent=True) or {}
        new_mode = body.get("mode")
        if new_mode not in ("idle", "velocity", "position", "torque"):
            return jsonify({"error": "mode must be idle|velocity|position|torque"}), 400
        with n.lock:
            n.mode = new_mode
            if new_mode == "velocity":
                n.vel = 0.0
            elif new_mode == "torque":
                n.torque = 0.0
            elif new_mode == "position":
                # Seed pos to where the slider thinks it is so the first
                # tick after entering position mode doesn't snap to 0.
                seed = body.get("seed_pos")
                if isinstance(seed, (int, float)):
                    n.pos = float(seed)
        return jsonify({"mode": new_mode})

    @app.post("/node/<int:nid>/action")
    def post_action(nid):
        """Merge one-shot fields into the next outgoing UDP packet.

        Accepts any of the ODrive command fields (`axis_state`,
        `control_mode`, `input_mode`, `clear_errors`, `velocity_limit`,
        `current_limit`, `pos_gain`, `vel_gain`, `vel_integrator_gain`,
        ...) — see src/drivers/odrive/node.rs `execute_command`.
        """
        n = find(nid)
        if n is None:
            return jsonify({"error": "unknown node"}), 404
        body = request.get_json(force=True, silent=True) or {}
        if not body:
            return jsonify({"error": "empty body"}), 400
        with n.lock:
            n.extra = (n.extra or {}) | dict(body)
        return jsonify({"queued": body})

    @app.post("/node/<int:nid>/custom")
    def post_custom(nid):
        """Manage custom streamed command fields for a node.

        Body:
          - `{"action":"add", "field":"input_torque", "value":0.5, "stream":true}`
            Stream `field=value` in every outgoing packet until removed. With
            `stream:false` (or omitted) it's merged into the next packet only
            (one-shot), same as /action.
          - `{"action":"remove", "field":"input_torque"}` — stop streaming one.
          - `{"action":"clear"}` — stop streaming all custom fields.

        `value` should already be JSON-typed (number/bool/string). This is how
        you "control in torque" (stream `input_torque`) or "send 15 to
        control_mode" (one-shot `control_mode=15`).
        """
        n = find(nid)
        if n is None:
            return jsonify({"error": "unknown node"}), 404
        body = request.get_json(force=True, silent=True) or {}
        action = body.get("action", "add")

        if action == "clear":
            with n.lock:
                n.custom = {}
            return jsonify({"custom": {}})

        field = body.get("field")
        if not field or not isinstance(field, str):
            return jsonify({"error": "field must be a non-empty string"}), 400

        if action == "remove":
            with n.lock:
                n.custom.pop(field, None)
                custom = dict(n.custom)
            return jsonify({"custom": custom})

        if action == "add":
            value = body.get("value")
            if bool(body.get("stream", False)):
                with n.lock:
                    n.custom[field] = value
                    custom = dict(n.custom)
                return jsonify({"streamed": {field: value}, "custom": custom})
            # one-shot
            with n.lock:
                n.extra = (n.extra or {}) | {field: value}
                custom = dict(n.custom)
            return jsonify({"queued": {field: value}, "custom": custom})

        return jsonify({"error": f"unknown action '{action}'"}), 400

    @app.post("/node/<int:nid>/estop")
    def post_estop(nid):
        """Hit the rove_sensor_api HTTP /<id>/estop endpoint out-of-band.

        The driver exposes ESTOP at the HTTP layer, not as a UDP command
        field. We piggy-back on the same target host and the default 8080.
        """
        n = find(nid)
        if n is None:
            return jsonify({"error": "unknown node"}), 404
        try:
            r = requests.post(
                f"http://{host}:{api_http_port}/odrive_{n.node_id}/estop",
                timeout=2.0,
            )
            return jsonify({"status": r.status_code, "body": r.text})
        except Exception as e:
            with n.lock:
                n.last_error = f"estop: {e}"
                n.recent_errors.append((time.time(), f"estop: {e}"))
            return jsonify({"error": str(e)}), 502

    @app.get("/state")
    def get_state():
        now = time.monotonic()
        window = 2.0
        out: dict = {}
        for n in nodes:
            with n.lock:
                send_times = list(n.send_times)
                telem_times = list(n.telem_times)
                telem = n.telem
                sent = n.sent
                errors = n.errors
                last_error = n.last_error
                recent_errors = list(n.recent_errors)
                custom = dict(n.custom)
            send_hz = sum(1 for t in send_times if now - t <= window) / window
            telem_hz = sum(1 for t in telem_times if now - t <= window) / window
            last_age = (now - telem_times[-1]) * 1000.0 if telem_times else None
            out[n.node_id] = {
                "telem": telem,
                "sent": sent,
                "errors": errors,
                "last_error": last_error,
                "send_hz": send_hz,
                "telem_hz": telem_hz,
                "last_telem_age_ms": last_age,
                "recent_errors": recent_errors,
                "custom": custom,
            }
        return jsonify({"nodes": out})

    return app


# Module-level — set by main() so /node/<id>/estop knows which HTTP port to call.
api_http_port: int = 8080


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--target", default="127.0.0.1", help="rove_sensor_api host")
    p.add_argument("--api-http-port", type=int, default=8080,
                   help="rove_sensor_api HTTP port (for /discover and /<id>/estop)")
    p.add_argument("--ui-host", default="0.0.0.0")
    p.add_argument("--ui-port", type=int, default=8091)
    p.add_argument("--max-vel", type=float, default=10.0,
                   help="default velocity slider range ±rev/s")
    p.add_argument("--max-pos", type=float, default=5.0,
                   help="default position slider range ±rev")
    p.add_argument("--max-torque", type=float, default=1.0,
                   help="default torque slider range ±Nm")
    p.add_argument("--rate", type=float, default=50.0,
                   help="UDP stream rate Hz to each ODrive command port")
    p.add_argument("--telem-interval-ms", type=int, default=50,
                   help="telemetry push interval requested via Subscribe")
    args = p.parse_args()

    global api_http_port
    api_http_port = args.api_http_port
    base_url = f"http://{args.target}:{args.api_http_port}"

    print(f"Discovering ODrive nodes via {base_url}/discover ...", file=sys.stderr)
    try:
        nodes = discover_nodes(base_url)
    except Exception as e:
        sys.exit(f"discover failed: {e}\n"
                 f"Is rove_sensor_api running at {base_url}?")
    if not nodes:
        sys.exit(f"no odrive_* sensors found at {base_url}/discover")
    for n in nodes:
        print(f"  - odrive_{n.node_id}  data:{n.data_port}  cmd:{n.cmd_port}  ({n.display})",
              file=sys.stderr)

    stop = threading.Event()
    for n in nodes:
        threading.Thread(
            target=stream_thread, args=(args.target, args.rate, n, stop),
            daemon=True, name=f"odrive-stream-{n.node_id}",
        ).start()
        threading.Thread(
            target=telem_thread,
            args=(args.target, args.telem_interval_ms, n, stop),
            daemon=True, name=f"odrive-telem-{n.node_id}",
        ).start()

    app = make_app(args.target, nodes, args.rate, args.max_vel, args.max_pos, args.max_torque)
    print(f"ODrive test UI: http://{args.ui_host}:{args.ui_port}/", file=sys.stderr)
    try:
        app.run(host=args.ui_host, port=args.ui_port,
                debug=False, use_reloader=False, threaded=True)
    finally:
        stop.set()


if __name__ == "__main__":
    main()
