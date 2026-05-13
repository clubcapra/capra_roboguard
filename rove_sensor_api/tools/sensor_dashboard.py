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
    from flask import Flask, jsonify, render_template_string
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

    def __init__(self, summary: dict, info: dict | None):
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
            with st.lock:
                st.latest = body
                st.packets += 1
                st.last_packet_mono = now
                st.recv_times.append(now)
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
</style></head><body>

<h1>Rove sensor dashboard</h1>
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


def make_app(target_label: str, sensors: list[SensorState]):
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

    return app


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--target", default="127.0.0.1", help="rove_sensor_api host")
    p.add_argument("--http-port", type=int, default=8080, help="rove_sensor_api HTTP port")
    p.add_argument("--ui-host", default="0.0.0.0")
    p.add_argument("--ui-port", type=int, default=8091)
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
        sensors.append(SensorState(s, info))
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

    app = make_app(f"{args.target}:{args.http_port}", sensors)
    print(f"\nDashboard: http://{args.ui_host}:{args.ui_port}/", file=sys.stderr)
    try:
        app.run(host=args.ui_host, port=args.ui_port, debug=False, use_reloader=False, threaded=True)
    finally:
        stop.set()


if __name__ == "__main__":
    main()
