#!/usr/bin/env python3
"""Overall sensor dashboard — Flask web UI.

Hits the rove_sensor_api `/discover` endpoint, then for every sensor it finds:
  1. fetches `/<id>/info` to get the field schema (units, descriptions)
  2. opens a UDP socket and sends a Subscribe (0x01) to the sensor's data port
  3. dumps incoming Data (0x03) packets into the in-memory state for that sensor

The browser side renders one card per sensor — built dynamically from the
schema returned by the API, no per-sensor templates. Cards refresh from
`/state` ~5 Hz and show: live field values, packet rate, packet count,
last-packet age, last error.

A second page at `/graph` lets you bind any numeric field from any sensor and
plot its value over time on a canvas chart, with a selectable window (1/5/10/30
min or "All retained"). History is recorded for every numeric field as packets
arrive — downsampled to `--history-hz` and capped to `--history-seconds` of
retention — so it spans the full life of the process up to that window.

Setup:
    pip install flask requests

Run:
    ./sensor_dashboard.py                                       # localhost:8080
    ./sensor_dashboard.py --target 192.168.2.37 --http-port 8080 --ui-host 0.0.0.0
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
MSG_ERROR = 0xFF


def encode(mt: int, seq: int, payload):
    body = json.dumps(payload).encode() if payload is not None else b""
    return struct.pack("<BBH", PROTOCOL_VERSION, mt, seq & 0xFFFF) + body


def decode(data: bytes):
    if len(data) < 4:
        raise ValueError("short")
    ver, mt, seq = struct.unpack("<BBH", data[:4])
    if ver != PROTOCOL_VERSION:
        raise ValueError(f"bad version {ver}")
    body = data[4:]
    return mt, seq, json.loads(body) if body else None


class SensorState:
    """Per-sensor live state populated by the UDP subscriber thread."""

    def __init__(self, summary: dict, info: dict | None,
                 history_seconds: float = 1800.0, history_hz: float = 4.0):
        self.id: str = summary["id"]
        self.display_name: str = summary.get("display_name", self.id)
        self.data_port: int = int(summary["data_port"])
        self.command_port: int = int(summary.get("command_port", 0))
        self.command_mode = summary.get("command_mode")
        # `data_schema` is `[{name, type_name, unit, description}, ...]`.
        self.data_schema: list[dict] = (info or {}).get("data_schema") or []
        self.lock = threading.Lock()
        self.latest: dict = {}
        self.packets = 0
        self.last_packet_mono: float | None = None
        self.last_error: str | None = None
        self.recv_times = collections.deque(maxlen=200)

        # --- Per-field time-series history for the /graph page. ---
        # field name -> deque[(wall_time, float_value)]. Populated by the
        # subscriber for every numeric field, downsampled to `history_hz` and
        # capped to `history_seconds` of retention (the "full life" window).
        self.history: dict[str, collections.deque] = {}
        self.hist_min_interval = (1.0 / history_hz) if history_hz > 0 else 0.0
        self.hist_maxlen = max(2, int(history_seconds * max(history_hz, 0.1)) + 2)
        self.last_hist_mono = 0.0


def discover(base_url: str, timeout: float = 3.0) -> list[dict]:
    r = requests.get(f"{base_url}/discover", timeout=timeout)
    r.raise_for_status()
    return r.json().get("sensors", [])


def fetch_info(base_url: str, sensor_id: str, timeout: float = 3.0) -> dict | None:
    try:
        r = requests.get(f"{base_url}/{sensor_id}/info", timeout=timeout)
        r.raise_for_status()
        return r.json()
    except Exception:
        return None


def subscriber_thread(host: str, st: SensorState, interval_ms: int | None, stop: threading.Event):
    """One socket per sensor. Subscribe, drain Data packets, unsubscribe on exit.

    `interval_ms=None` subscribes with an empty payload, deferring to the
    server's configured `default_push_interval_ms` (config/server.toml).
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    addr = (host, st.data_port)
    try:
        payload = {"interval_ms": interval_ms} if interval_ms is not None else None
        sock.sendto(encode(MSG_SUBSCRIBE, 0, payload), addr)
    except OSError as e:
        with st.lock:
            st.last_error = f"subscribe: {e}"
    while not stop.is_set():
        try:
            pkt, _ = sock.recvfrom(8192)
        except socket.timeout:
            continue
        except OSError as e:
            with st.lock:
                st.last_error = f"recv: {e}"
            time.sleep(0.2)
            continue
        try:
            mt, _, body = decode(pkt)
        except Exception as e:
            with st.lock:
                st.last_error = f"decode: {e}"
            continue
        if mt == MSG_DATA and isinstance(body, dict):
            now = time.monotonic()
            wall = time.time()
            with st.lock:
                st.latest = body
                st.packets += 1
                st.last_packet_mono = now
                st.recv_times.append(now)
                # Downsampled history capture: at most one sample per
                # hist_min_interval, recording every numeric scalar field.
                if now - st.last_hist_mono >= st.hist_min_interval:
                    st.last_hist_mono = now
                    for k, v in body.items():
                        if isinstance(v, bool):
                            fv = 1.0 if v else 0.0
                        elif isinstance(v, (int, float)):
                            fv = float(v)
                        else:
                            continue  # skip strings, arrays (e.g. *_words), objects
                        dq = st.history.get(k)
                        if dq is None:
                            dq = collections.deque(maxlen=st.hist_maxlen)
                            st.history[k] = dq
                        dq.append((wall, fv))
        elif mt == MSG_ERROR and isinstance(body, dict):
            with st.lock:
                st.last_error = f"driver: {body.get('error', body)}"
    try:
        sock.sendto(encode(MSG_UNSUBSCRIBE, 0, None), addr)
    except OSError:
        pass
    sock.close()


INDEX = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rove sensor dashboard — {{target}}</title>
<style>
:root{--bg:#111;--fg:#eee;--muted:#888;--accent:#4af;--ok:#6c6;--err:#f66;--panel:#1c1c1c;--border:#2a2a2a}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font-family:-apple-system,system-ui,sans-serif}
h1{margin:0 0 4px;font-size:1.2em}
.sub{color:var(--muted);font-size:.85em;margin-bottom:14px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(360px,1fr));gap:12px}
.card{background:var(--panel);border:1px solid var(--border);border-radius:6px;padding:12px;display:flex;flex-direction:column}
.card h2{margin:0;font-size:1em;display:flex;align-items:center;gap:8px}
.card h2 .id{color:var(--muted);font-family:ui-monospace,monospace;font-size:.85em;font-weight:normal}
.meta{color:var(--muted);font-size:.78em;margin:4px 0 8px;font-family:ui-monospace,monospace}
.pill{display:inline-block;padding:1px 7px;border-radius:10px;font-size:.78em;font-family:ui-monospace,monospace}
.pill.ok{background:#1a4;color:#fff}.pill.stale{background:#a40;color:#fff}.pill.dead{background:#a22;color:#fff}
table.t{width:100%;font-family:ui-monospace,monospace;font-size:.82em;border-collapse:collapse}
table.t td{padding:2px 6px;border-bottom:1px solid var(--border);vertical-align:top}
table.t td:first-child{color:var(--muted);max-width:55%;word-break:break-word}
table.t td:last-child{text-align:right;color:var(--accent);font-variant-numeric:tabular-nums}
.err{color:var(--err);font-size:.78em;margin-top:6px;font-family:ui-monospace,monospace;word-break:break-word}
.empty{color:var(--muted);font-style:italic;font-size:.85em}
.navlink{font-size:.6em;font-weight:normal;margin-left:10px;padding:3px 10px;border:1px solid var(--border);border-radius:4px;color:var(--accent);text-decoration:none;vertical-align:middle}
.navlink:hover{background:var(--panel)}
</style></head><body>

<h1>Rove sensor dashboard <a href="/graph" class="navlink">📈 Field graph →</a></h1>
<div class="sub">target <code>{{target}}</code> — discovering sensors…</div>

<div id="grid" class="grid"></div>

<script>
const TARGET = {{target_json|safe}};

function fmtVal(v, unit){
  if(v===null||v===undefined)return '—';
  if(typeof v==='number'){
    const s=(Math.abs(v)>=1000||(Math.abs(v)<0.01&&v!==0))?v.toExponential(3):v.toFixed(3);
    return unit?`${s} ${unit}`:s;
  }
  if(typeof v==='boolean')return v?'true':'false';
  if(Array.isArray(v))return '['+v.map(x=>fmtVal(x,'')).join(', ')+']';
  if(typeof v==='object')return JSON.stringify(v);
  return String(v);
}
function escapeHtml(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}

function freshnessPill(ageMs, hz){
  if(ageMs===null||ageMs===undefined)return '<span class="pill dead">no data</span>';
  if(ageMs>2000)return `<span class="pill dead">stale ${(ageMs/1000).toFixed(1)}s</span>`;
  if(ageMs>500)return `<span class="pill stale">${ageMs.toFixed(0)} ms · ${hz.toFixed(1)} Hz</span>`;
  return `<span class="pill ok">${ageMs.toFixed(0)} ms · ${hz.toFixed(1)} Hz</span>`;
}

// Render the schema once, then update only values on each poll. The
// schema rarely changes; rebuilding the whole DOM 5×/sec causes noticeable
// flicker on Pi-class machines.
const cards = new Map();  // id -> {root, valueCells, ageEl, errEl}

function buildCard(s){
  const card = document.createElement('div');
  card.className = 'card';
  // Header: display name + id, + freshness pill (updated every poll).
  const header = document.createElement('h2');
  header.innerHTML = `${escapeHtml(s.display_name)} <span class="id">${escapeHtml(s.id)}</span>`;
  card.appendChild(header);

  const meta = document.createElement('div');
  meta.className = 'meta';
  const cmd = s.command_mode ? (typeof s.command_mode==='string'?s.command_mode:JSON.stringify(s.command_mode)) : '—';
  meta.textContent = `data_port=${s.data_port}  cmd_port=${s.command_port}  mode=${cmd}`;
  card.appendChild(meta);

  const ageEl = document.createElement('div');
  ageEl.className = 'meta';
  ageEl.innerHTML = freshnessPill(null,0)+` · packets=0`;
  card.appendChild(ageEl);

  // Build one row per schema field. If schema is empty, fall back to
  // whatever keys appear in the data payload.
  const table = document.createElement('table');
  table.className = 't';
  const tbody = document.createElement('tbody');
  table.appendChild(tbody);
  card.appendChild(table);

  const valueCells = new Map();
  if(s.data_schema && s.data_schema.length){
    for(const f of s.data_schema){
      const tr = document.createElement('tr');
      const tdK = document.createElement('td');
      const unit = f.unit?` (${f.unit})`:'';
      tdK.textContent = f.name + unit;
      if(f.description) tdK.title = f.description;
      const tdV = document.createElement('td');
      tdV.textContent = '—';
      tr.appendChild(tdK); tr.appendChild(tdV);
      tbody.appendChild(tr);
      valueCells.set(f.name, {td:tdV, unit:f.unit||''});
    }
  } else {
    // Schema-less fallback — body filled in dynamically by update().
    const tr = document.createElement('tr');
    const td = document.createElement('td');
    td.colSpan = 2;
    td.className = 'empty';
    td.textContent = 'no schema, awaiting data…';
    tr.appendChild(td);
    tbody.appendChild(tr);
  }

  const errEl = document.createElement('div');
  errEl.className = 'err';
  card.appendChild(errEl);

  return {root:card, ageEl, tbody, valueCells, errEl, hasSchema: s.data_schema && s.data_schema.length>0};
}

function updateCard(c, snap){
  c.ageEl.innerHTML = freshnessPill(snap.last_packet_age_ms, snap.hz)+` · packets=${snap.packets}`;
  const data = snap.latest || {};
  if(c.hasSchema){
    for(const [name, cell] of c.valueCells){
      cell.td.textContent = fmtVal(data[name], cell.unit);
    }
  } else {
    // Rebuild dynamic rows when keys change. Cheap because schema-less
    // sensors are rare and usually small.
    const keys = Object.keys(data);
    if(keys.length === 0){
      c.tbody.innerHTML = '<tr><td colspan="2" class="empty">no schema, awaiting data…</td></tr>';
    } else {
      const sig = keys.join('|');
      if(c.tbody.dataset.sig !== sig){
        c.tbody.innerHTML = '';
        c.tbody.dataset.sig = sig;
        c.valueCells.clear();
        for(const k of keys){
          const tr = document.createElement('tr');
          const tdK = document.createElement('td'); tdK.textContent = k;
          const tdV = document.createElement('td'); tdV.textContent = '—';
          tr.appendChild(tdK); tr.appendChild(tdV);
          c.tbody.appendChild(tr);
          c.valueCells.set(k, {td:tdV, unit:''});
        }
      }
      for(const [k, cell] of c.valueCells){
        cell.td.textContent = fmtVal(data[k], '');
      }
    }
  }
  c.errEl.textContent = snap.last_error ? `last error: ${snap.last_error}` : '';
}

async function init(){
  let resp;
  try{
    resp = await fetch('/sensors').then(r=>r.json());
  }catch(e){
    document.getElementById('grid').innerHTML =
      `<div class="card err">discovery failed: ${escapeHtml(e.toString())}</div>`;
    return;
  }
  const grid = document.getElementById('grid');
  grid.innerHTML = '';
  document.querySelector('.sub').textContent = `target ${TARGET} — ${resp.sensors.length} sensor(s) discovered, subscribed to UDP data ports`;

  if(resp.sensors.length === 0){
    grid.innerHTML = '<div class="card empty">no sensors registered</div>';
    return;
  }
  for(const s of resp.sensors){
    const c = buildCard(s);
    grid.appendChild(c.root);
    cards.set(s.id, c);
  }
  poll();
  setInterval(poll, 200);
}

async function poll(){
  let st;
  try{ st = await fetch('/state').then(r=>r.json()); }
  catch(e){ return; }
  for(const [id, snap] of Object.entries(st.sensors||{})){
    const c = cards.get(id);
    if(c) updateCard(c, snap);
  }
}

init();
</script></body></html>"""


GRAPH = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rove field graph — {{target}}</title>
<style>
:root{--bg:#111;--fg:#eee;--muted:#888;--accent:#4af;--ok:#6c6;--err:#f66;--panel:#1c1c1c;--border:#2a2a2a;--grid:#222}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font-family:-apple-system,system-ui,sans-serif}
h1{margin:0 0 10px;font-size:1.2em}
a.navlink{font-size:.6em;font-weight:normal;margin-left:10px;padding:3px 10px;border:1px solid var(--border);border-radius:4px;color:var(--accent);text-decoration:none;vertical-align:middle}
a.navlink:hover{background:var(--panel)}
.controls{display:flex;gap:8px;align-items:center;flex-wrap:wrap;background:var(--panel);border:1px solid var(--border);border-radius:6px;padding:10px;margin-bottom:10px}
select,button,input{background:#0c0c0c;color:var(--fg);border:1px solid var(--border);border-radius:4px;padding:5px 8px;font:inherit;font-size:.88em}
button{cursor:pointer;background:#2a2a2a}
button:hover{background:#333}
button.add{background:var(--ok);border-color:var(--ok);color:#062}
button.add:hover{filter:brightness(1.1)}
.controls label{color:var(--muted);font-size:.85em;display:flex;align-items:center;gap:4px}
.sep{width:1px;align-self:stretch;background:var(--border);margin:0 4px}
.retnote{color:var(--muted);font-size:.78em;margin-left:auto;font-family:ui-monospace,monospace}
.legend{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px;min-height:24px}
.chip{display:inline-flex;align-items:center;gap:6px;background:var(--panel);border:1px solid var(--border);border-radius:14px;padding:3px 10px;font-family:ui-monospace,monospace;font-size:.8em}
.chip .sw{width:11px;height:11px;border-radius:2px;flex:none}
.chip .val{color:var(--accent)}
.chip a{color:var(--err);cursor:pointer;font-weight:bold;text-decoration:none;margin-left:2px}
.chartwrap{position:relative;background:var(--panel);border:1px solid var(--border);border-radius:6px;padding:8px}
canvas{display:block;width:100%;height:480px;cursor:crosshair}
.tip{position:absolute;pointer-events:none;display:none;z-index:5;background:rgba(8,8,8,.94);border:1px solid var(--border);border-radius:5px;padding:6px 8px;font-family:ui-monospace,monospace;font-size:.78em;min-width:120px;box-shadow:0 2px 10px #000a}
.tip .tt-time{color:var(--muted);margin-bottom:3px}
.tip .tt-row{display:flex;align-items:center;gap:6px;white-space:nowrap}
.tip .tt-row .sw{width:9px;height:9px;border-radius:2px;flex:none}
.tip .tt-f{color:var(--fg)}
.tip .tt-v{color:var(--accent);margin-left:auto;padding-left:10px}
.hint{color:var(--muted);font-size:.78em;margin-top:8px}
.empty{color:var(--muted);font-style:italic}
</style></head><body>

<h1>Rove field graph <a href="/" class="navlink">← dashboard</a></h1>

<div class="controls">
  <select id="sensor" title="sensor"></select>
  <select id="field" title="field"></select>
  <button id="add" class="add">Bind field +</button>
  <span class="sep"></span>
  <label>window
    <select id="win">
      <option value="60">1 min</option>
      <option value="300">5 min</option>
      <option value="600" selected>10 min</option>
      <option value="1800">30 min</option>
      <option value="all">All (retained)</option>
    </select>
  </label>
  <label><input type="checkbox" id="norm"> normalize</label>
  <label><input type="checkbox" id="pause"> pause</label>
  <button id="clear">Clear all</button>
  <span class="retnote" id="retnote"></span>
</div>

<div id="legend" class="legend"></div>
<div class="chartwrap"><canvas id="chart"></canvas><div id="tip" class="tip"></div></div>
<div class="hint">Hover the chart to read each series' exact value at that time. Bound fields persist in this browser (localStorage). History is recorded from the moment this dashboard process started, at {{history_hz}} Hz, capped to {{retention_seconds}} s of retention — so <b>All</b> shows up to that window.</div>

<script>
const TARGET = {{target_json|safe}};
const RETENTION = {{retention_seconds}};
const HIST_HZ = {{history_hz}};
const PALETTE = ['#4af','#6c6','#fb4','#f66','#b8f','#4dd','#f9a','#9d5','#88f','#fd7'];

let FIELDS = [];                  // [{sensor, sensor_name, field, unit}]
let bound = [];                   // [{sensor, field}] — colors derived from index
let lastSeries = [];              // last /series response, aligned to `bound`
let lastNow = null;

const cv = document.getElementById('chart');
const ctx = cv.getContext('2d');
let hoverX = null;                // cursor x in canvas-buffer px, or null
let mCssX = 0, mCssY = 0;         // cursor position in CSS px within the canvas
let lastPlot = null;             // mapping from the last draw(), for the hover overlay
let rafPending = false;

function escapeHtml(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
function colorOf(i){return PALETTE[i % PALETTE.length];}
function keyOf(b){return b.sensor+':'+b.field;}
function unitOf(sensor, field){const f=FIELDS.find(x=>x.sensor===sensor&&x.field===field);return f?(f.unit||''):'';}
function fmtNum(v){
  if(v===null||v===undefined||isNaN(v))return '—';
  const a=Math.abs(v);
  return (a>=1000||(a<0.01&&v!==0))?v.toExponential(2):v.toFixed(3);
}
function fmtClock(t){const d=new Date(t*1000);const p=n=>String(n).padStart(2,'0');return p(d.getHours())+':'+p(d.getMinutes())+':'+p(d.getSeconds());}
function minMax(arr){let mn=Infinity,mx=-Infinity;for(let i=0;i<arr.length;i++){const x=arr[i];if(x<mn)mn=x;if(x>mx)mx=x;}return [mn,mx];}

function saveBound(){try{localStorage.setItem('rove_graph_bound', JSON.stringify(bound));}catch(e){}}
function loadBound(){try{const s=localStorage.getItem('rove_graph_bound');if(s)bound=JSON.parse(s)||[];}catch(e){bound=[];}}

// --- Field/sensor pickers ---
function populateSensors(){
  const sel=document.getElementById('sensor');
  const seen=new Set(); sel.innerHTML='';
  for(const f of FIELDS){
    if(seen.has(f.sensor))continue; seen.add(f.sensor);
    const o=document.createElement('option');
    o.value=f.sensor; o.textContent=`${f.sensor_name} (${f.sensor})`;
    sel.appendChild(o);
  }
  populateFields();
}
function populateFields(){
  const sid=document.getElementById('sensor').value;
  const sel=document.getElementById('field'); sel.innerHTML='';
  for(const f of FIELDS.filter(x=>x.sensor===sid)){
    const o=document.createElement('option');
    o.value=f.field; o.textContent=f.field+(f.unit?` (${f.unit})`:'');
    sel.appendChild(o);
  }
}

function addBound(){
  const sensor=document.getElementById('sensor').value;
  const field=document.getElementById('field').value;
  if(!sensor||!field)return;
  if(bound.some(b=>b.sensor===sensor&&b.field===field))return; // no dup
  bound.push({sensor, field});
  saveBound(); renderLegend(); refresh();
}
function removeBound(sensor, field){
  bound=bound.filter(b=>!(b.sensor===sensor&&b.field===field));
  saveBound(); renderLegend(); refresh();
}
function clearBound(){bound=[];saveBound();renderLegend();refresh();}

function renderLegend(){
  const leg=document.getElementById('legend');
  if(!bound.length){leg.innerHTML='<span class="empty">No fields bound — pick a sensor + field and hit “Bind field +”.</span>';return;}
  leg.innerHTML=bound.map((b,i)=>{
    const s=lastSeries[i];
    const latest=(s&&s.v&&s.v.length)?fmtNum(s.v[s.v.length-1]):'—';
    const unit=unitOf(b.sensor,b.field);
    return `<span class="chip"><span class="sw" style="background:${colorOf(i)}"></span>`
      +`${escapeHtml(b.sensor)}:${escapeHtml(b.field)} <span class="val">${latest}${unit?' '+escapeHtml(unit):''}</span>`
      +` <a title="remove" onclick="removeBound('${escapeHtml(b.sensor)}','${escapeHtml(b.field)}')">×</a></span>`;
  }).join('');
}

// --- Data + drawing ---
async function refresh(){
  if(!bound.length){lastSeries=[];lastNow=null;draw();renderLegend();return;}
  const win=document.getElementById('win').value;
  const q=bound.map(keyOf).join(',');
  try{
    const r=await fetch(`/series?q=${encodeURIComponent(q)}&window=${encodeURIComponent(win)}`);
    const j=await r.json();
    // Re-align response to current bound order by key (defensive).
    const byKey={}; for(const s of (j.series||[])) byKey[s.sensor+':'+s.field]=s;
    lastSeries=bound.map(b=>byKey[keyOf(b)]||{sensor:b.sensor,field:b.field,t:[],v:[]});
    lastNow=j.now;
  }catch(e){/* keep last frame */}
  draw(); renderLegend();
}

function sizeCanvas(){
  const w=cv.parentElement.clientWidth-16;       // minus padding
  cv.width=Math.max(320,w); cv.height=480;
}

function draw(){
  const W=cv.width, H=cv.height;
  const padL=62,padR=14,padT=14,padB=30;
  const plotW=W-padL-padR, plotH=H-padT-padB;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#151515'; ctx.fillRect(padL,padT,plotW,plotH);

  const haveData=lastSeries.some(s=>s&&s.v&&s.v.length);
  if(!bound.length||!haveData){
    ctx.fillStyle='#888'; ctx.font='13px system-ui'; ctx.textAlign='center';
    ctx.fillText(bound.length?'waiting for data…':'bind a field to start graphing', W/2, H/2);
    lastPlot=null; hideTip();
    return;
  }
  const norm=document.getElementById('norm').checked;
  const win=document.getElementById('win').value;

  // Time axis range.
  let tMin=Infinity,tMax=-Infinity;
  for(const s of lastSeries){if(!s)continue;for(const t of s.t){if(t<tMin)tMin=t;if(t>tMax)tMax=t;}}
  if(win!=='all'&&lastNow){tMax=lastNow;tMin=lastNow-Number(win);}
  if(!isFinite(tMin)||!isFinite(tMax)||tMin>=tMax){tMax=lastNow||Date.now()/1000;tMin=tMax-1;}

  // Value axis range.
  let vMin,vMax;
  if(norm){vMin=0;vMax=1;}
  else{
    vMin=Infinity;vMax=-Infinity;
    for(const s of lastSeries){if(!s)continue;for(const v of s.v){if(v<vMin)vMin=v;if(v>vMax)vMax=v;}}
    if(!isFinite(vMin)){vMin=0;vMax=1;}
    if(vMin===vMax){vMin-=1;vMax+=1;}
    const pad=(vMax-vMin)*0.08; vMin-=pad; vMax+=pad;
  }

  const X=t=>padL+(t-tMin)/(tMax-tMin)*plotW;
  const Y=v=>padT+(1-(v-vMin)/(vMax-vMin))*plotH;

  // Grid + Y labels.
  ctx.strokeStyle='#262626'; ctx.fillStyle='#888'; ctx.lineWidth=1;
  ctx.font='11px ui-monospace,monospace'; ctx.textAlign='right'; ctx.textBaseline='middle';
  for(let i=0;i<=4;i++){
    const yv=vMin+(vMax-vMin)*i/4; const y=Y(yv);
    ctx.beginPath();ctx.moveTo(padL,y);ctx.lineTo(padL+plotW,y);ctx.stroke();
    ctx.fillText(norm?(i/4).toFixed(2):fmtNum(yv), padL-6, y);
  }
  // X time labels.
  ctx.textAlign='center'; ctx.textBaseline='top';
  for(let i=0;i<=4;i++){
    const tv=tMin+(tMax-tMin)*i/4; const x=X(tv);
    ctx.strokeStyle='#1e1e1e';ctx.beginPath();ctx.moveTo(x,padT);ctx.lineTo(x,padT+plotH);ctx.stroke();
    ctx.fillStyle='#888';ctx.fillText(fmtClock(tv), x, padT+plotH+6);
  }

  // Series polylines.
  ctx.lineWidth=1.6;
  const seriesNorm=[];
  lastSeries.forEach((s,idx)=>{
    if(!s||!s.t.length){seriesNorm[idx]=null;return;}
    let smin=0,smax=1;
    if(norm){[smin,smax]=minMax(s.v); if(smin===smax){smin-=1;smax+=1;}}
    seriesNorm[idx]=norm?[smin,smax]:null;
    ctx.strokeStyle=colorOf(idx); ctx.beginPath();
    for(let j=0;j<s.t.length;j++){
      let vv=s.v[j];
      if(norm)vv=(vv-smin)/(smax-smin);
      const x=X(s.t[j]), y=Y(vv);
      if(j===0)ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  });

  // Remember the mapping so the hover overlay can place the crosshair/markers.
  lastPlot={tMin,tMax,padL,padT,plotW,plotH,norm,seriesNorm,X,Y};
  if(hoverX!==null) drawHover();
}

// Nearest index in an ascending time array (binary search).
function nearestIdx(arr, target){
  if(target<=arr[0])return 0;
  const last=arr.length-1;
  if(target>=arr[last])return last;
  let lo=0,hi=last;
  while(lo<=hi){const mid=(lo+hi)>>1; if(arr[mid]<target)lo=mid+1; else hi=mid-1;}
  return (Math.abs(arr[lo-1]-target)<=Math.abs(arr[lo]-target))?lo-1:lo;
}

// Draw the crosshair + per-series marker at the hovered time and fill the tooltip.
function drawHover(){
  if(!lastPlot){hideTip();return;}
  const {tMin,tMax,padL,padT,plotW,plotH,norm,seriesNorm,X,Y}=lastPlot;
  if(hoverX<padL||hoverX>padL+plotW){hideTip();return;}
  const ct=tMin+(hoverX-padL)/plotW*(tMax-tMin);  // cursor time

  ctx.save();
  ctx.strokeStyle='rgba(255,255,255,.28)'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(hoverX,padT); ctx.lineTo(hoverX,padT+plotH); ctx.stroke();

  const rows=[];
  lastSeries.forEach((s,idx)=>{
    const b=bound[idx];
    if(!b||!s||!s.t.length)return;
    const i=nearestIdx(s.t, ct);
    const rt=s.t[i], rv=s.v[i];
    let pv=rv;
    if(norm&&seriesNorm[idx]){let[mn,mx]=seriesNorm[idx]; pv=(rv-mn)/(mx-mn);}
    const x=X(rt), y=Y(pv);
    ctx.fillStyle=colorOf(idx); ctx.beginPath(); ctx.arc(x,y,3.5,0,Math.PI*2); ctx.fill();
    ctx.lineWidth=1; ctx.strokeStyle='#000'; ctx.stroke();
    rows.push({idx, field:b.field, sensor:b.sensor, rv, rt});
  });
  ctx.restore();
  showTip(ct, rows);
}

function showTip(ct, rows){
  const tip=document.getElementById('tip');
  if(!rows.length){tip.style.display='none';return;}
  let html=`<div class="tt-time">${fmtClock(ct)}</div>`;
  for(const r of rows){
    const unit=unitOf(r.sensor,r.field);
    html+=`<div class="tt-row"><span class="sw" style="background:${colorOf(r.idx)}"></span>`
      +`<span class="tt-f">${escapeHtml(r.field)}</span>`
      +`<span class="tt-v">${fmtNum(r.rv)}${unit?' '+escapeHtml(unit):''}</span></div>`;
  }
  tip.innerHTML=html;
  tip.style.display='block';
  const wrap=tip.parentElement;
  const tw=tip.offsetWidth, th=tip.offsetHeight;
  let left=cv.offsetLeft+mCssX+14, top=cv.offsetTop+mCssY+14;
  if(left+tw>wrap.clientWidth) left=cv.offsetLeft+mCssX-tw-14;   // flip left near right edge
  if(left<0) left=4;
  if(top+th>wrap.clientHeight) top=wrap.clientHeight-th-4;
  if(top<0) top=4;
  tip.style.left=left+'px'; tip.style.top=top+'px';
}
function hideTip(){const t=document.getElementById('tip'); if(t)t.style.display='none';}

function onMove(e){
  const rect=cv.getBoundingClientRect();
  mCssX=e.clientX-rect.left; mCssY=e.clientY-rect.top;
  hoverX=mCssX*(cv.width/rect.width);   // CSS px -> canvas buffer px
  if(rafPending)return;
  rafPending=true;
  requestAnimationFrame(()=>{rafPending=false; draw();});
}
function onLeave(){hoverX=null; hideTip(); draw();}

async function init(){
  loadBound();
  try{
    const j=await fetch('/fields').then(r=>r.json());
    FIELDS=j.fields||[];
    document.getElementById('retnote').textContent=
      `${FIELDS.length} graphable fields · retention ${RETENTION}s @ ${HIST_HZ}Hz`;
  }catch(e){
    document.getElementById('retnote').textContent='failed to load /fields';
  }
  populateSensors();
  document.getElementById('sensor').addEventListener('change',populateFields);
  document.getElementById('add').addEventListener('click',addBound);
  document.getElementById('clear').addEventListener('click',clearBound);
  document.getElementById('win').addEventListener('change',refresh);
  document.getElementById('norm').addEventListener('change',draw);
  window.addEventListener('resize',()=>{sizeCanvas();draw();});
  cv.addEventListener('mousemove', onMove);
  cv.addEventListener('mouseleave', onLeave);
  sizeCanvas(); renderLegend(); draw();
  refresh();
  setInterval(()=>{ if(!document.getElementById('pause').checked) refresh(); }, 1000);
}
init();
</script></body></html>"""


def make_app(target_label: str, sensors: list[SensorState],
             history_seconds: float = 1800.0, history_hz: float = 4.0):
    app = Flask(__name__)
    sensors_by_id = {s.id: s for s in sensors}

    @app.get("/")
    def index():
        return render_template_string(
            INDEX,
            target=target_label,
            target_json=json.dumps(target_label),
        )

    @app.get("/sensors")
    def list_sensors():
        return jsonify({
            "sensors": [
                {
                    "id": s.id,
                    "display_name": s.display_name,
                    "data_port": s.data_port,
                    "command_port": s.command_port,
                    "command_mode": s.command_mode,
                    "data_schema": s.data_schema,
                }
                for s in sensors
            ],
        })

    @app.get("/state")
    def state():
        now = time.monotonic()
        window = 2.0
        out = {}
        for s in sensors:
            with s.lock:
                latest = s.latest
                packets = s.packets
                last = s.last_packet_mono
                last_error = s.last_error
                recv = list(s.recv_times)
            age_ms = (now - last) * 1000.0 if last is not None else None
            hz = sum(1 for t in recv if now - t <= window) / window
            out[s.id] = {
                "latest": latest,
                "packets": packets,
                "last_packet_age_ms": age_ms,
                "hz": hz,
                "last_error": last_error,
            }
        return jsonify({"sensors": out})

    # ── Graph page ────────────────────────────────────────────────────────────

    @app.get("/graph")
    def graph():
        return render_template_string(
            GRAPH,
            target=target_label,
            target_json=json.dumps(target_label),
            retention_seconds=int(history_seconds),
            history_hz=history_hz,
        )

    # Numeric schema types eligible for graphing.
    _NUMERIC_TYPES = {
        "f16", "f32", "f64", "float", "double", "number",
        "u8", "u16", "u32", "u64", "usize",
        "i8", "i16", "i32", "i64", "isize", "int", "integer",
        "bool", "boolean",
    }

    @app.get("/fields")
    def fields():
        """List every numeric field that can be bound on the graph page.

        Built from each sensor's schema (numeric types) plus any numeric keys
        actually observed in the history buffer (covers schema-less sensors and
        derived fields).
        """
        out = []
        for s in sensors:
            seen = set()
            for f in s.data_schema:
                if str(f.get("type_name", "")).lower() in _NUMERIC_TYPES:
                    out.append({
                        "sensor": s.id, "sensor_name": s.display_name,
                        "field": f["name"], "unit": f.get("unit", ""),
                    })
                    seen.add(f["name"])
            with s.lock:
                hist_keys = list(s.history.keys())
            for k in hist_keys:
                if k not in seen:
                    out.append({
                        "sensor": s.id, "sensor_name": s.display_name,
                        "field": k, "unit": "",
                    })
                    seen.add(k)
        return jsonify({
            "fields": out,
            "retention_seconds": int(history_seconds),
            "history_hz": history_hz,
        })

    @app.get("/series")
    def series():
        """Return recorded time-series for one or more `sensor:field` pairs.

        Query params:
          - `q=sensor:field,sensor:field`  (or `sensor=...&field=...`)
          - `window=<seconds>` or `window=all`
        Response: `{now, series:[{sensor, field, unit, t:[wall...], v:[...]}]}`.
        """
        q = request.args.get("q")
        pairs: list[tuple[str, str]] = []
        if q:
            for part in q.split(","):
                if ":" in part:
                    sid, fld = part.split(":", 1)
                    sid, fld = sid.strip(), fld.strip()
                    if sid and fld:
                        pairs.append((sid, fld))
        else:
            sid = request.args.get("sensor")
            fld = request.args.get("field")
            if sid and fld:
                pairs.append((sid, fld))

        win = request.args.get("window", "600")
        now = time.time()
        cutoff = None
        if win != "all":
            try:
                cutoff = now - float(win)
            except (TypeError, ValueError):
                cutoff = None

        result = []
        for sid, fld in pairs:
            st = sensors_by_id.get(sid)
            if st is None:
                result.append({"sensor": sid, "field": fld, "unit": "",
                               "t": [], "v": [], "missing": True})
                continue
            with st.lock:
                points = list(st.history.get(fld, ()))
            if cutoff is not None:
                points = [p for p in points if p[0] >= cutoff]
            unit = next((f.get("unit", "") for f in st.data_schema
                         if f.get("name") == fld), "")
            result.append({
                "sensor": sid, "field": fld, "unit": unit,
                "t": [p[0] for p in points],
                "v": [p[1] for p in points],
            })
        return jsonify({"now": now, "series": result})

    return app


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--target", default="127.0.0.1", help="rove_sensor_api host")
    p.add_argument("--http-port", type=int, default=8080, help="rove_sensor_api HTTP port")
    p.add_argument("--ui-host", default="0.0.0.0")
    p.add_argument("--ui-port", type=int, default=8092)
    p.add_argument(
        "--interval-ms",
        type=int,
        default=None,
        help=(
            "Subscribe interval_ms to request from each sensor. Default: omitted, "
            "so the server's `default_push_interval_ms` (config/server.toml) applies. "
            "Pass an explicit value to override the server-side default for this dashboard."
        ),
    )
    p.add_argument("--history-seconds", type=float, default=1800.0,
                   help="graph history retention window per field (the /graph 'All' span). Default: 1800 (30 min)")
    p.add_argument("--history-hz", type=float, default=4.0,
                   help="graph history sample rate per field (downsamples high-rate sensors). Default: 4")
    args = p.parse_args()

    base_url = f"http://{args.target}:{args.http_port}"
    print(f"discovering sensors at {base_url}/discover …", file=sys.stderr)
    try:
        summaries = discover(base_url)
    except Exception as e:
        sys.exit(f"discover failed: {e}")

    sensors: list[SensorState] = []
    for s in summaries:
        info = fetch_info(base_url, s["id"])
        sensors.append(SensorState(s, info,
                                   history_seconds=args.history_seconds,
                                   history_hz=args.history_hz))
        schema_n = len((info or {}).get("data_schema") or [])
        print(
            f"  • {s['id']:<14} '{s.get('display_name','')}'  "
            f"data_port={s['data_port']}  cmd_port={s.get('command_port','?')}  "
            f"schema_fields={schema_n}",
            file=sys.stderr,
        )

    if not sensors:
        print("no sensors registered — UI will show empty dashboard", file=sys.stderr)

    stop = threading.Event()
    for st in sensors:
        threading.Thread(
            target=subscriber_thread,
            args=(args.target, st, args.interval_ms, stop),
            daemon=True,
            name=f"sub-{st.id}",
        ).start()

    rate_note = (
        f"forced interval_ms={args.interval_ms}"
        if args.interval_ms is not None
        else "using server's default_push_interval_ms (config/server.toml)"
    )
    print(f"\nSubscribe rate: {rate_note}", file=sys.stderr)

    app = make_app(f"{args.target}:{args.http_port}", sensors,
                   history_seconds=args.history_seconds, history_hz=args.history_hz)
    print(f"\nDashboard: http://{args.ui_host}:{args.ui_port}/  (graph: /graph)", file=sys.stderr)
    try:
        app.run(host=args.ui_host, port=args.ui_port, debug=False, use_reloader=False, threaded=True)
    finally:
        stop.set()


if __name__ == "__main__":
    main()
