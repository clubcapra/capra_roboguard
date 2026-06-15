import express from "express";
import { WebSocketServer } from "ws";
import http from "node:http";
import { listSensors, readRange, LOG_ROOT } from "./csvReader.ts";
import { LiveHub } from "./liveBridge.ts";

const PORT = Number(process.env.LOG_VIEWER_API_PORT ?? 8765);
const API_BASE = process.env.ROVE_API_BASE ?? "http://127.0.0.1:8080";

const app = express();
const hub = new LiveHub(API_BASE);

app.get("/api/health", (_req, res) => {
  res.json({ ok: true, log_root: LOG_ROOT, api_base: API_BASE });
});

app.get("/api/sensors", async (_req, res) => {
  // Merge historical (from disk) with live (from /discover). Schema preference:
  // historical CSV header is the source of truth for plotting fields, since
  // that's what we can graph. We attach live discovery info if present.
  const historical = listSensors();
  let live: Awaited<ReturnType<LiveHub["refreshDiscovery"]>> = [];
  try {
    live = await hub.refreshDiscovery();
  } catch {
    // API not running — fine, we just won't offer live mode.
  }
  const byId = new Map(historical.map((s) => [s.id, { ...s, live: false, display_name: s.id }]));
  for (const l of live) {
    const prev = byId.get(l.id);
    if (prev) {
      byId.set(l.id, { ...prev, live: true, display_name: l.display_name });
    } else {
      byId.set(l.id, {
        id: l.id,
        dates: [],
        fields: [],
        live: true,
        display_name: l.display_name,
      });
    }
  }
  res.json({ sensors: Array.from(byId.values()).sort((a, b) => a.id.localeCompare(b.id)) });
});

app.get("/api/log/range", async (req, res) => {
  const sensor = String(req.query.sensor ?? "");
  const start_ms = Number(req.query.start_ms);
  const end_ms = Number(req.query.end_ms);
  const maxPoints = req.query.max_points ? Number(req.query.max_points) : undefined;
  const fields = req.query.fields ? String(req.query.fields).split(",").filter(Boolean) : undefined;
  if (!sensor || !Number.isFinite(start_ms) || !Number.isFinite(end_ms)) {
    return res.status(400).json({ error: "sensor, start_ms, end_ms required" });
  }
  try {
    const data = await readRange({ sensor, start_ms, end_ms, fields, maxPoints });
    res.json(data);
  } catch (err) {
    res.status(500).json({ error: String(err) });
  }
});

const server = http.createServer(app);
const wss = new WebSocketServer({ server, path: "/ws/live" });

wss.on("connection", (ws) => {
  ws.on("message", (raw) => {
    let msg: unknown;
    try {
      msg = JSON.parse(raw.toString());
    } catch {
      return;
    }
    if (!msg || typeof msg !== "object") return;
    const m = msg as { type?: string; sensors?: string[]; interval_ms?: number };
    if (m.type === "subscribe" && Array.isArray(m.sensors)) {
      hub.subscribe(ws, m.sensors, m.interval_ms);
    } else if (m.type === "unsubscribe" && Array.isArray(m.sensors)) {
      hub.unsubscribe(ws, m.sensors);
    }
  });
  ws.on("close", () => hub.detach(ws));
  ws.on("error", () => hub.detach(ws));
});

server.listen(PORT, () => {
  console.log(`log_viewer API listening on :${PORT}`);
  console.log(`  LOG_ROOT=${LOG_ROOT}`);
  console.log(`  ROVE_API_BASE=${API_BASE}`);
});
