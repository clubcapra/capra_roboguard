#!/usr/bin/env python3
"""Kinova arm test UI — Flask web app.

Slider-based browser UI that streams joint velocities to the Kinova driver
over UDP, and reads telemetry back via UDP subscribe. Headless-friendly
(open the browser from any machine).

Setup:
    pip install flask           # only dep

Run:
    ./kinova_test.py                                       # localhost
    ./kinova_test.py --target 192.168.2.37 --ui-host 0.0.0.0 --ui-port 8090

Wire format matches src/protocol/packet.rs.
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
    from flask import Flask, jsonify, render_template_string, request
except ImportError:
    sys.exit("Flask not installed. Run: pip install flask")

PROTOCOL_VERSION = 0x01
MSG_SUBSCRIBE = 0x01
MSG_UNSUBSCRIBE = 0x02
MSG_DATA = 0x03
MSG_COMMAND = 0x10
MSG_ERROR = 0xFF

NUM_JOINTS = 6


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


class State:
    def __init__(self):
        self.lock = threading.Lock()
        # Control mode — only one of `vels` or `positions` is streamed at a time.
        self.mode = "velocity"  # "velocity" | "position"
        self.vels = [0.0] * NUM_JOINTS
        self.positions = [0.0] * NUM_JOINTS
        self.extra = None
        self.telem = {}
        self.sent = 0
        self.errors = 0
        self.last_error = None
        # Diagnostics — bounded ring buffers so an idle session doesn't grow
        # memory unboundedly. Used by the UI's Diagnostics panel to compute
        # actual send/telem rates and surface recent driver errors.
        self.send_times = collections.deque(maxlen=200)   # monotonic seconds
        self.telem_times = collections.deque(maxlen=200)  # monotonic seconds
        # (timestamp, message) tuples. Newest first when read for display.
        self.recent_errors = collections.deque(maxlen=20)


def stream_thread(host, cmd_port, rate_hz, st: State, stop):
    """Tight UDP-stream loop, mode-aware.

    - **Velocity mode**: streams `joint_*_vel` at `rate_hz` *only while at
      least one joint is non-zero*. Pure streaming semantics — when the
      operator zeros the sliders we simply stop sending; the rove_sensor_api
      worker's velocity hold timeout (~300 ms) lapses and the arm halts on
      its own. Sending continuous zeros would be redundant.
    - **Position mode**: sends `joint_*_pos` *only when the target changes*.
      Position is FIFO-driven — repeating the same target every tick would
      pile redundant entries onto the arm's 2000-entry trajectory queue.

    Drift-free monotonic-clock scheduling, non-blocking ack drain.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    addr = (host, cmd_port)
    interval = 1.0 / rate_hz
    seq = 0
    next_tick = time.monotonic()
    last_mode = None
    last_pos_sent: list | None = None

    while not stop.is_set():
        with st.lock:
            mode = st.mode
            vels = list(st.vels)
            positions = list(st.positions)
            extra = st.extra
            st.extra = None

        # Mode transition: reset the position-change tracker so the first
        # packet after entering position mode actually sends.
        if mode != last_mode:
            last_pos_sent = None
            last_mode = mode

        payload: dict = {}
        if mode == "velocity":
            # Pure streaming: only send while at least one joint is non-zero.
            # When the operator zeros the sliders we stop sending; the
            # rove_sensor_api worker's velocity hold timeout halts the arm.
            # Continuously streaming zeros was causing instability (the SDK
            # appears to handle a stream of zero setpoints differently than
            # an absence of stream).
            if any(v != 0.0 for v in vels):
                payload = {f"joint_{i + 1}_vel": vels[i] for i in range(NUM_JOINTS)}
        else:  # position
            if positions != last_pos_sent:
                payload = {f"joint_{i + 1}_pos": positions[i] for i in range(NUM_JOINTS)}
                last_pos_sent = list(positions)
        if extra:
            payload.update(extra)

        if payload:
            try:
                sock.sendto(encode(MSG_COMMAND, seq, payload), addr)
                now_mono = time.monotonic()
                with st.lock:
                    st.sent += 1
                    st.send_times.append(now_mono)
                seq = (seq + 1) & 0xFFFF
            except OSError as e:
                with st.lock:
                    st.errors += 1
                    st.last_error = f"send: {e}"
                    st.recent_errors.append((time.time(), f"send: {e}"))

        # Always drain acks non-blockingly to surface driver errors.
        while True:
            try:
                ack, _ = sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as e:
                with st.lock:
                    st.last_error = f"recv: {e}"
                    st.recent_errors.append((time.time(), f"recv: {e}"))
                break
            try:
                mt, _, body = decode(ack)
                if mt == MSG_ERROR and isinstance(body, dict):
                    msg = f"driver: {body.get('error', body)}"
                    with st.lock:
                        st.errors += 1
                        st.last_error = msg
                        st.recent_errors.append((time.time(), msg))
            except Exception as e:
                with st.lock:
                    st.last_error = f"decode: {e}"
                    st.recent_errors.append((time.time(), f"decode: {e}"))

        next_tick += interval
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            # Fell behind — skip ahead so we don't burst-catch-up.
            next_tick = time.monotonic()

    # No final zero packet — pure streaming model. The worker's velocity
    # hold timeout (~300 ms) fires once the stream stops and halts the arm.
    sock.close()


def telem_thread(host, data_port, interval_ms, st: State, stop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    addr = (host, data_port)
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
            with st.lock:
                st.telem = body
                st.telem_times.append(now_mono)
    try:
        sock.sendto(encode(MSG_UNSUBSCRIBE, 0, None), addr)
    except OSError:
        pass
    sock.close()


INDEX = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Kinova arm test</title>
<style>
:root{--bg:#111;--fg:#eee;--muted:#888;--accent:#4af;--danger:#cc1e25;--panel:#1c1c1c;--border:#2a2a2a}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font-family:-apple-system,system-ui,sans-serif}
h1{margin:0 0 12px;font-size:1.2em}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:6px;padding:12px;margin-bottom:12px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
button{background:#2a2a2a;color:var(--fg);border:1px solid var(--border);padding:8px 14px;border-radius:4px;cursor:pointer;font:inherit;font-size:.95em}
button:hover{background:#333}
button.estop{background:var(--danger);border-color:var(--danger);color:#fff;font-weight:bold;margin-left:auto;padding:8px 24px}
button.estop:hover{background:#a51820}
.joint-row{display:grid;grid-template-columns:70px 60px 1fr 60px 70px 40px 90px;gap:8px;align-items:center;margin:6px 0}
.mode-row{display:flex;gap:16px;align-items:center;margin-bottom:8px}
.mode-row label{cursor:pointer;color:var(--muted);padding:4px 12px;border-radius:4px;border:1px solid var(--border)}
.mode-row label.active{color:var(--fg);background:#2a2a2a;border-color:var(--accent)}
.mode-row input[type=radio]{display:none}
button.zero-here{padding:4px 8px;font-size:.75em;background:#1a4}
button.zero-here:hover{background:#176}
.joint-row label{color:var(--muted);font-size:.9em}
.joint-row .val{font-family:ui-monospace,monospace;text-align:right;color:var(--accent)}
input[type=range]{width:100%}
.joint-row button{padding:4px 8px;font-size:.8em}
table.t{width:100%;font-family:ui-monospace,monospace;font-size:.85em;border-collapse:collapse}
table.t td{padding:3px 6px;border-bottom:1px solid var(--border)}
table.t td:first-child{color:var(--muted);width:18ch}
.status{font-family:ui-monospace,monospace;font-size:.85em;color:var(--muted)}
.status.err{color:#f66}.status.ok{color:#6c6}
.pill{display:inline-block;padding:1px 7px;border-radius:10px;font-size:.8em;font-family:ui-monospace,monospace}
.pill.on{background:#1a4;color:#fff}.pill.off{background:#555;color:#ccc}
.pill.estop{background:var(--danger);color:#fff}
</style></head><body>

<h1>Kinova arm test — {{target}}</h1>

<div class="panel"><div class="row">
  <button onclick="zeroAll()">Zero sliders</button>
  <button onclick="action('move_home', {move_home: true})">Move home</button>
  <button onclick="halt()">Halt motion</button>
</div></div>

<div class="panel">
  <div class="mode-row">
    <span style="color:var(--muted)">Control mode:</span>
    <label id="m-vel" class="active"><input type="radio" name="mode" value="velocity" checked> Velocity (deg/s, streaming)</label>
    <label id="m-pos"><input type="radio" name="mode" value="position"> Position (deg, FIFO)</label>
    <button onclick="setAllZeroHere()" title="Persist current arm pose as the new zero on every joint (writes to actuator flash). Each joint will read ~180° afterward — that's the Kinova zero reference." style="margin-left:auto">⊘ Set all joint zeros here</button>
  </div>
  <div id="sliders"></div>
</div>

<div class="panel">
  <div style="font-weight:bold;margin-bottom:8px">Telemetry</div>
  <table class="t"><tbody id="telem"></tbody></table>
</div>

<div class="panel">
  <div style="font-weight:bold;margin-bottom:8px">Diagnostics — stutter debugging</div>
  <table class="t"><tbody id="diag"></tbody></table>
  <div style="margin-top:10px;font-weight:bold;color:var(--muted);font-size:.85em">Recent driver errors (most recent first)</div>
  <div id="errlog" style="max-height:160px;overflow-y:auto;font-family:ui-monospace,monospace;font-size:.78em;color:#f99;background:#0a0a0a;border:1px solid var(--border);border-radius:4px;padding:6px;margin-top:4px">no errors yet</div>
</div>

<div class="panel" id="control-warn" style="display:none;background:#3a1010;border-color:var(--danger);color:#fff">
  <strong>⚠ Arm reports control is OFF.</strong> The driver no longer manages control state — if this banner persists, restart the rove_sensor_api process so its connect() re-runs StartControlAPI.
</div>
<div class="panel"><div class="status" id="status">connecting…</div></div>

<script>
const N = 6;
const MAX_VEL = {{max_vel}};
const MAX_POS = {{max_pos}};
const HZ = {{rate_hz}};

let mode = 'velocity';
const sliders=[],vals=[];
const wrap=document.getElementById('sliders');

function modeMax(){return mode==='velocity'?MAX_VEL:MAX_POS;}
function modeStep(){return mode==='velocity'?0.1:0.5;}
function modeUnit(){return mode==='velocity'?'°/s':'°';}

function rebuildSliders(initial){
  wrap.innerHTML='';sliders.length=0;vals.length=0;
  const max=modeMax(),step=modeStep(),unit=modeUnit();
  for(let i=0;i<N;i++){
    const initVal = initial?.[i] ?? 0;
    const r=document.createElement('div');
    r.className='joint-row';
    r.innerHTML=`<label>Joint ${i+1}</label>
      <span style="text-align:right;color:var(--muted);font-size:.85em">−${max.toFixed(0)}</span>
      <input type="range" min="${-max}" max="${max}" step="${step}" value="${initVal}">
      <span style="color:var(--muted);font-size:.85em">+${max.toFixed(0)}</span>
      <span class="val">${initVal.toFixed(2)} ${unit}</span>
      <button title="Reset slider to 0">0</button>
      <button class="zero-here" title="Persist this joint's current physical position as its new zero (writes to actuator flash). Reads as ~180° afterward — Kinova zero reference.">⊘ zero here</button>`;
    wrap.appendChild(r);
    const cells=r.querySelectorAll('input, .val, button');
    const s=cells[0], v=r.querySelector('.val'), bReset=cells[2], bZero=cells[3];
    sliders.push(s);vals.push(v);
    s.addEventListener('input',()=>{v.textContent=parseFloat(s.value).toFixed(2)+' '+unit;push();});
    bReset.addEventListener('click',()=>{s.value=0;v.textContent='0.00 '+unit;push();});
    bZero.addEventListener('click',()=>setJointZeroHere(i+1));
  }
}

// Latest-wins HTTP POST. The Flask dev server is slow enough on a Pi that
// stacked-up in-flight fetches let the browser queue the *latest* slider
// value behind older ones — observed as multi-100 ms latency on slider
// changes. AbortController cancels any in-flight POST as soon as a newer
// slider value exists, so only the most recent ever lands at Flask.
let pushAbort=null;
async function push(){
  if(pushAbort){pushAbort.abort();}
  pushAbort=new AbortController();
  const sig=pushAbort.signal;
  const endpoint=mode==='velocity'?'/vel':'/pos';
  const key=mode==='velocity'?'vels':'positions';
  const body={};body[key]=sliders.map(s=>parseFloat(s.value));
  try{
    await fetch(endpoint,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:sig});
  }catch(e){/* aborted or network blip — next push will retry */}
}
async function action(name, extra){
  const body={action:name, ...(extra||{})};
  try{
    const r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const j=await r.json().catch(()=>({}));
    if(!r.ok){setStatus(`action ${name} rejected: ${j.error||r.status}`,'err');return false;}
    return true;
  }catch(e){setStatus(`action ${name} failed: ${e}`,'err');return false;}
}
function zeroAll(){sliders.forEach((s,i)=>{s.value=0;vals[i].textContent='0.00 '+modeUnit();});push();}
async function halt(){zeroAll();await action('halt');setStatus('Halt — trajectories erased, sliders zeroed','err');}
// `poll()` runs every 200 ms and overwrites the status bar with the current
// stream rate / error count. Important messages (zero-set progress, action
// rejections, mode switches) need to outlive that — `flashUntil` blocks
// poll's status update for ~5 s after a sticky message.
let flashUntil=0;
function setStatus(t,c,sticky){
  const e=document.getElementById('status');
  e.textContent=t;e.className='status '+(c||'');
  if(sticky)flashUntil=Date.now()+5000;
}

async function setMode(newMode){
  if(newMode===mode)return;
  // Halt first by sending a zero-velocity packet via /vel — that *also*
  // pins Python's mode to "velocity" until we explicitly flip it.
  await fetch('/vel',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({vels:Array(N).fill(0)})});

  if(newMode==='position'){
    // CRITICAL ordering: read the arm's *current* joint angles from
    // telemetry first, then atomically flip Python to position mode with
    // those values via /pos. If we flipped mode first, the stream thread
    // could pick up `mode=position` while `state.positions` is still
    // zeros from initialization, and drive the arm to (0,0,0,0,0,0)
    // before we got around to seeding the seed positions.
    let initial = Array(N).fill(0);
    try{
      const j=await(await fetch('/telem')).json();
      const d=j.telemetry||{};
      initial = Array.from({length:N},(_,i)=>d[`joint_${i+1}_pos`]??0);
    }catch(e){
      setStatus('telemetry unreachable — staying in velocity mode for safety','err');
      // Re-check the velocity radio to keep the UI honest.
      document.querySelector('input[name=mode][value=velocity]').checked=true;
      return;
    }
    // /pos sets state.positions AND state.mode atomically. Stream thread's
    // first position tick will send these values, which equal where the
    // arm already is — no motion.
    await fetch('/pos',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({positions:initial})});
    mode='position';
    rebuildSliders(initial);
  }else{
    // Switching to velocity: the /vel POST above already set state.vels
    // to zeros and state.mode to velocity. Just refresh the UI.
    mode='velocity';
    rebuildSliders(Array(N).fill(0));
  }
  document.getElementById('m-vel').classList.toggle('active', mode==='velocity');
  document.getElementById('m-pos').classList.toggle('active', mode==='position');
  setStatus(`switched to ${mode} mode`,'ok');
}

document.querySelectorAll('input[name=mode]').forEach(r=>{
  r.addEventListener('change', e=>setMode(e.target.value));
});

// Per-joint commanded velocity, kept in sync with the slider state so the
// Diagnostics panel can render commanded-vs-actual side-by-side.
function commandedVels(){
  return mode==='velocity' ? sliders.map(s=>parseFloat(s.value)) : Array(N).fill(0);
}

// SetJointZero writes the current physical joint position into the
// actuator's flash as its new zero reference. The driver no longer
// manages control state, so all we need to do is zero the sliders (so the
// streaming worker doesn't fight the calibration with a stale velocity)
// and let the driver issue the SDK call. After a successful set_zero the
// joint reads as ~180° because Kinova uses 180° as its zero reference.
async function quiescentBeforeCalibration(){
  // Zero the sliders, send one explicit halt, briefly settle so the
  // actuator's encoder reading is stable before flashing it.
  sliders.forEach((s,i)=>{s.value=0;vals[i].textContent='0.00 '+modeUnit();});
  await fetch('/vel',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({vels:Array(N).fill(0)})});
  await action('halt',{erase_trajectories:true});
  await new Promise(r=>setTimeout(r,400));
}
// IMPORTANT empirical finding: SetJointZero writes the new zero to actuator
// flash, but the running firmware does NOT pick it up — it keeps using the
// previous zero until the arm is electrically power-cycled. So after a
// successful set_joint_zero the telemetry will still report the OLD
// reference frame; only after a reboot will the joints read from the new
// zero. The status messages and confirms below are explicit about that.
async function setJointZeroHere(joint){
  if(!confirm(`Set joint ${joint}'s current physical position as its new zero?\n\nWrites to actuator flash. IMPORTANT: the new zero will NOT take effect until the arm is electrically power-cycled — the running firmware keeps using the old zero until reboot.`))return;
  setStatus(`Halting before zero-set for joint ${joint}…`,'',true);
  await quiescentBeforeCalibration();
  if(await action('set_joint_zero',{joint:joint})){
    setStatus(`Joint ${joint} zero written to flash. ⚠ POWER-CYCLE THE ARM to activate it — telemetry still reflects the old zero until reboot.`,'ok',true);
  }
}
async function setAllZeroHere(){
  if(!confirm(`Set ALL six joints' current positions as their new zeros?\n\nWrites to all six actuators' flash. IMPORTANT: the new zeros will NOT take effect until the arm is electrically power-cycled — the running firmware keeps using the previous zeros until reboot.`))return;
  setStatus('Halting before zero-set for all joints…','',true);
  await quiescentBeforeCalibration();
  for(let j=1;j<=N;j++){
    setStatus(`Setting zero on joint ${j}/${N}…`,'',true);
    if(!await action('set_joint_zero',{joint:j})){
      setStatus(`Aborted at joint ${j}`,'err',true);
      return;
    }
    // Each SetJointZero writes to actuator flash — give the actuator time
    // to complete the write before issuing the next one. ~1 s is what
    // proved reliable in earlier testing.
    await new Promise(r=>setTimeout(r,1000));
  }
  setStatus('All six joint zeros written to flash. ⚠ POWER-CYCLE THE ARM to activate them — telemetry still reflects the previous zeros until reboot.','ok',true);
}

function fmtJ(d,suf,fn){return Array.from({length:N},(_,i)=>{const v=d[`joint_${i+1}_${suf}`];return typeof v==='number'?fn(v):'  —  ';}).join(' ');}
function escapeHtml(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
async function poll(){try{
  const j=await(await fetch('/telem')).json();
  const d=j.telemetry||{};
  // Update the telemetry table regardless, but skip the status-bar overwrite
  // while a sticky message is still showing.
  const skipStatus = Date.now() < flashUntil;
  const f=v=>v.toFixed(2).padStart(7);
  const enabled=d.control_enabled?'<span class="pill on">on</span>':'<span class="pill off">off</span>';
  const rows=[
    ['joint_*_pos (deg)',     fmtJ(d,'pos',f)],
    ['joint_*_vel (deg/s)',   fmtJ(d,'vel',f)],
    ['joint_*_torque (Nm)',   fmtJ(d,'torque',f)],
    ['joint_*_current (A)',   fmtJ(d,'current',f)],
    ['joint_*_temp (°C)',     fmtJ(d,'temp',f)],
    ['bus', `${(d.bus_voltage||0).toFixed(2)} V   ${(d.bus_current||0).toFixed(2)} A`],
    ['status', `control_enabled ${enabled}  retract=${d.retract_state}  torque_sensors=${d.torque_sensors_available}`],
    ['timestamp_ns', String(d.timestamp_ns??'—')],
  ];
  document.getElementById('telem').innerHTML=rows.map(([k,v])=>`<tr><td>${k}</td><td>${v}</td></tr>`).join('');

  // Diagnostics panel — actual rates (not the configured target), telemetry
  // freshness, and per-joint tracking gap (commanded velocity − actual). A
  // persistent gap when sliders are held nonzero is the clearest signal of
  // a stutter/lag problem; a missing or stale telemetry rate is the next.
  const diag=j.diagnostics||{};
  const sendHz=(diag.send_hz??0).toFixed(1);
  const telemHz=(diag.telem_hz??0).toFixed(1);
  const ageMs=(diag.last_telem_age_ms??0).toFixed(0);
  const cmdVels=mode==='velocity' ? sliders.map(s=>parseFloat(s.value)) : Array(N).fill(0);
  const trackingGap=Array.from({length:N},(_,i)=>{
    const actual=d[`joint_${i+1}_vel`];
    if(typeof actual!=='number')return '  —  ';
    return (cmdVels[i]-actual).toFixed(2).padStart(7);
  }).join(' ');
  const diagRows=[
    ['actual send rate',     `${sendHz} Hz   (target: ${HZ.toFixed(0)} Hz)`],
    ['actual telem rate',    `${telemHz} Hz   (last packet ${ageMs} ms ago)`],
    ['cmd − actual vel',     trackingGap + '   (deg/s, persistent ≠0 ⇒ tracking lag)'],
    ['error count',          String(diag.errors??0)],
  ];
  document.getElementById('diag').innerHTML=diagRows.map(([k,v])=>`<tr><td>${k}</td><td>${v}</td></tr>`).join('');

  const errs=diag.recent_errors||[];
  const errlogEl=document.getElementById('errlog');
  if(errs.length===0){errlogEl.textContent='no errors yet';}
  else{
    errlogEl.innerHTML=errs.slice().reverse().map(([ts,msg])=>{
      const dt=new Date(ts*1000);
      const hh=String(dt.getHours()).padStart(2,'0');
      const mm=String(dt.getMinutes()).padStart(2,'0');
      const ss=String(dt.getSeconds()).padStart(2,'0');
      const ms=String(dt.getMilliseconds()).padStart(3,'0');
      return `<div>${hh}:${mm}:${ss}.${ms}  ${escapeHtml(msg)}</div>`;
    }).join('');
  }

  // Banner: only fires now on the arm's own control_enabled telemetry going
  // false — driver no longer mutates an estopped flag. If it stays on, the
  // arm needs a power cycle / process restart, not a button click.
  const cw=document.getElementById('control-warn');
  cw.style.display = (d.control_enabled===false) ? 'block' : 'none';
  if(!skipStatus){
    const s=j.status||{};
    let t=`streaming @ ${sendHz} Hz (target ${HZ.toFixed(0)}) | sent=${s.sent} errors=${s.errors}`;
    if(s.last_error)t+=`  |  last: ${s.last_error}`;
    setStatus(t,s.last_error?'err':'ok');
  }
}catch(e){setStatus('UI server unreachable: '+e,'err');}}

// Initial render: velocity mode, zeroed sliders.
rebuildSliders();
push();
setInterval(poll, 200);
poll();
</script></body></html>"""


def make_app(
    state: State, target_label: str, max_vel: float, max_pos: float, rate_hz: float
):
    app = Flask(__name__)

    @app.get("/")
    def index():
        return render_template_string(
            INDEX,
            target=target_label,
            max_vel=max_vel,
            max_pos=max_pos,
            rate_hz=rate_hz,
        )

    @app.post("/vel")
    def post_vel():
        body = request.get_json(force=True, silent=True) or {}
        vels = body.get("vels", [])
        if not (isinstance(vels, list) and len(vels) == NUM_JOINTS):
            return jsonify({"error": "vels must be a 6-element array"}), 400
        with state.lock:
            state.vels = [float(x) for x in vels]
            state.mode = "velocity"
        return jsonify({"ok": True})

    @app.post("/pos")
    def post_pos():
        body = request.get_json(force=True, silent=True) or {}
        positions = body.get("positions", [])
        if not (isinstance(positions, list) and len(positions) == NUM_JOINTS):
            return jsonify({"error": "positions must be a 6-element array"}), 400
        with state.lock:
            state.positions = [float(x) for x in positions]
            state.mode = "position"
        return jsonify({"ok": True})

    @app.post("/mode")
    def post_mode():
        body = request.get_json(force=True, silent=True) or {}
        new_mode = body.get("mode")
        if new_mode not in ("velocity", "position"):
            return jsonify({"error": "mode must be 'velocity' or 'position'"}), 400
        with state.lock:
            state.mode = new_mode
            if new_mode == "velocity":
                # Hard-zero velocity sliders on every entry into velocity mode
                # so the arm doesn't lurch with a stale value.
                state.vels = [0.0] * NUM_JOINTS
        return jsonify({"mode": new_mode})

    @app.post("/action")
    def post_action():
        body = request.get_json(force=True, silent=True) or {}
        # The driver no longer manages control state (no clear_errors /
        # start_control / hard_stop) — those would just return 400 from
        # the sensor command surface and are removed here too.
        mapping = {
            "move_home": {"move_home": True},
            "halt": {"erase_trajectories": True},
        }
        name = body.get("action")
        # set_joint_zero takes a joint argument (1..6); persists current
        # position as the new zero in actuator flash. No mapping entry —
        # constructed from the request body directly.
        if name == "set_joint_zero":
            joint = body.get("joint")
            if not isinstance(joint, int) or not (1 <= joint <= NUM_JOINTS):
                return jsonify({"error": "joint must be an int 1..6"}), 400
            extra = {"set_joint_zero": joint}
        elif name in mapping:
            extra = mapping[name]
        else:
            return jsonify({"error": f"unknown action {name}"}), 400
        with state.lock:
            state.extra = (state.extra or {}) | extra
        return jsonify({"queued": name, **extra})

    @app.get("/telem")
    def get_telem():
        # Compute rates over a 2-second sliding window — long enough that an
        # idle browser tab doesn't see the rate snap to 0 between polls,
        # short enough that a stutter shows up immediately. We snapshot
        # under the lock and compute outside to keep the lock window small.
        now_mono = time.monotonic()
        window = 2.0
        with state.lock:
            send_times = list(state.send_times)
            telem_times = list(state.telem_times)
            recent_errors = list(state.recent_errors)
            telem = state.telem
            sent = state.sent
            errors = state.errors
            last_error = state.last_error
        send_in_window = sum(1 for t in send_times if now_mono - t <= window)
        telem_in_window = sum(1 for t in telem_times if now_mono - t <= window)
        send_hz = send_in_window / window
        telem_hz = telem_in_window / window
        last_telem_age_ms = (now_mono - telem_times[-1]) * 1000.0 if telem_times else None
        return jsonify({
            "telemetry": telem,
            "status": {
                "sent": sent,
                "errors": errors,
                "last_error": last_error,
            },
            "diagnostics": {
                "send_hz": send_hz,
                "telem_hz": telem_hz,
                "last_telem_age_ms": last_telem_age_ms,
                "errors": errors,
                "recent_errors": recent_errors,
            },
        })

    return app


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--target", default="127.0.0.1", help="rove_sensor_api host")
    p.add_argument("--cmd-port", type=int, default=5003)
    p.add_argument("--data-port", type=int, default=5002)
    p.add_argument("--ui-host", default="0.0.0.0")
    p.add_argument("--ui-port", type=int, default=8090)
    p.add_argument(
        "--max-vel",
        type=float,
        default=40.0,
        help="velocity slider range ±deg/s (Jaco2 maxes ~36 base / ~48 wrist)",
    )
    p.add_argument(
        "--max-pos",
        type=float,
        default=180.0,
        help="position slider range ±deg",
    )
    p.add_argument(
        "--rate",
        type=float,
        default=75.0,
        help="UDP stream rate Hz to the rove_sensor_api. The worker on that side already re-sends the held velocity to the arm at 100 Hz internally, so this rate only governs how quickly slider changes propagate from the browser to the worker. 30–100 Hz is fine; below ~10 Hz the velocity hold (300 ms) will lapse between user inputs.",
    )
    args = p.parse_args()

    state = State()
    stop = threading.Event()

    threading.Thread(
        target=stream_thread, args=(args.target, args.cmd_port, args.rate, state, stop),
        daemon=True, name="kinova-stream",
    ).start()
    threading.Thread(
        target=telem_thread, args=(args.target, args.data_port, 100, state, stop),
        daemon=True, name="kinova-telem",
    ).start()

    app = make_app(
        state,
        f"{args.target}:{args.cmd_port}",
        args.max_vel,
        args.max_pos,
        args.rate,
    )
    print(f"Kinova test UI: http://{args.ui_host}:{args.ui_port}/", file=sys.stderr)
    print(f"Streaming → {args.target}:{args.cmd_port}  /  reading ← {args.target}:{args.data_port}", file=sys.stderr)
    try:
        # Disable Flask reloader (would re-run threads in a child process).
        app.run(host=args.ui_host, port=args.ui_port, debug=False, use_reloader=False, threaded=True)
    finally:
        stop.set()


if __name__ == "__main__":
    main()
