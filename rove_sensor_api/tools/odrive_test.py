#!/usr/bin/env python3
"""ODrive multi-node test UI — Flask web app.

Auto-discovers every `odrive_*` sensor exposed by `rove_sensor_api`, then for
each node opens a UDP command stream and a UDP telemetry subscription.
Per-node sliders stream `input_vel` (rev/s) or `input_pos` (rev) over the
node's command port. Buttons arm/disarm the axis (axis_state 8/1) and clear
errors. Headless-friendly — open the browser from any machine.

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
        # Control mode — only one of vel / pos is streamed at a time. "idle"
        # means stop sending setpoints entirely; the driver watchdog keeps
        # the last input_pos refreshed on its own.
        self.mode = "idle"  # "idle" | "velocity" | "position"
        self.vel = 0.0   # rev/s
        self.pos = 0.0   # rev
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

    `extra` (one-shot fields like axis_state / clear_errors / control_mode)
    is merged into the next packet and cleared. If the only field to send
    is one-shot, the packet still goes out.
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
</style></head><body>

<h1>ODrive multi-node test — {{target}}</h1>

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

// Per-node slider state mirror — used when constructing /cmd POSTs.
const nodeMode = {};      // node_id -> "idle"|"velocity"|"position"
const nodeVel = {};       // node_id -> number (rev/s)
const nodePos = {};       // node_id -> number (rev)
const nodeVelMax = {};    // node_id -> slider range
const nodePosMax = {};    // node_id -> slider range

function el(html){const t=document.createElement('template');t.innerHTML=html.trim();return t.content.firstChild;}
function escapeHtml(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}

function buildNode(n){
  const id=n.node_id;
  nodeMode[id]='idle';nodeVel[id]=0;nodePos[id]=0;
  nodeVelMax[id]=DEFAULT_VEL_LIMIT;nodePosMax[id]=DEFAULT_POS_LIMIT;
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
      <span style="margin-left:14px;color:var(--muted);font-size:.8em">vel ±</span>
      <input type="number" id="vmax-${id}" value="${DEFAULT_VEL_LIMIT}" step="0.5" min="0.1" style="width:60px">
      <span style="color:var(--muted);font-size:.8em">pos ±</span>
      <input type="number" id="pmax-${id}" value="${DEFAULT_POS_LIMIT}" step="0.5" min="0.1" style="width:60px">
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
function postAction(id, extra){
  return fetch(`/node/${id}/action`,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(extra)});
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
  }else{
    document.getElementById(`vel-${id}`).value=0;
    nodeVel[id]=0;
    document.getElementById(`vel-val-${id}`).textContent='0.00 rev/s';
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
function zeroAll(){NODES.forEach(n=>{zeroVel(n.node_id);zeroPos(n.node_id);});}

function armOne(id){postAction(id,{axis_state:8,control_mode:nodeMode[id]==='position'?3:2,input_mode:1});}
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
async function poll(){
  let j;try{j=await(await fetch('/state')).json();}catch(e){return;}
  for(const n of NODES){
    const id=n.node_id;
    const ns=j.nodes[id];if(!ns)continue;
    const t=ns.telem||{};
    const rows=[
      ['axis_state',     `${t.axis_state??'—'} ${axisName(t.axis_state)}`],
      ['axis_error',     `0x${(t.axis_error??0).toString(16)}`],
      ['active_errors',  `0x${(t.active_errors??0).toString(16)}`],
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


def make_app(host: str, nodes: list[NodeState], rate_hz: float, max_vel: float, max_pos: float):
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
            rate_hz=rate_hz, max_vel=max_vel, max_pos=max_pos,
        )

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
        return jsonify({"ok": True})

    @app.post("/node/<int:nid>/mode")
    def post_mode(nid):
        n = find(nid)
        if n is None:
            return jsonify({"error": "unknown node"}), 404
        body = request.get_json(force=True, silent=True) or {}
        new_mode = body.get("mode")
        if new_mode not in ("idle", "velocity", "position"):
            return jsonify({"error": "mode must be idle|velocity|position"}), 400
        with n.lock:
            n.mode = new_mode
            if new_mode == "velocity":
                n.vel = 0.0
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

    app = make_app(args.target, nodes, args.rate, args.max_vel, args.max_pos)
    print(f"ODrive test UI: http://{args.ui_host}:{args.ui_port}/", file=sys.stderr)
    try:
        app.run(host=args.ui_host, port=args.ui_port,
                debug=False, use_reloader=False, threaded=True)
    finally:
        stop.set()


if __name__ == "__main__":
    main()
