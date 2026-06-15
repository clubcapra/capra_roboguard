import dgram from "node:dgram";
import type { WebSocket } from "ws";

// Matches sensor_dashboard.py's binary header.
const PROTOCOL_VERSION = 0x01;
const MSG_SUBSCRIBE = 0x01;
const MSG_UNSUBSCRIBE = 0x02;
const MSG_DATA = 0x03;

function encode(mt: number, seq: number, payload: unknown): Buffer {
  const body = payload === null || payload === undefined ? Buffer.alloc(0) : Buffer.from(JSON.stringify(payload));
  const header = Buffer.alloc(4);
  header.writeUInt8(PROTOCOL_VERSION, 0);
  header.writeUInt8(mt, 1);
  header.writeUInt16LE(seq & 0xffff, 2);
  return Buffer.concat([header, body]);
}

function decode(buf: Buffer): { mt: number; body: unknown } | null {
  if (buf.length < 4) return null;
  if (buf.readUInt8(0) !== PROTOCOL_VERSION) return null;
  const mt = buf.readUInt8(1);
  const rest = buf.slice(4);
  try {
    return { mt, body: rest.length ? JSON.parse(rest.toString("utf8")) : null };
  } catch {
    return null;
  }
}

/** One UDP socket per (sensor_id) shared by all WS clients subscribed to it. */
class SensorTap {
  readonly id: string;
  readonly socket: dgram.Socket;
  readonly subscribers = new Set<WebSocket>();
  private readonly addr: { host: string; port: number };

  constructor(id: string, host: string, port: number) {
    this.id = id;
    this.addr = { host, port };
    this.socket = dgram.createSocket("udp4");
    this.socket.on("message", (msg) => this.onMessage(msg));
    this.socket.on("error", () => {
      // Silent — UDP errors are usually transient (port closed during shutdown).
    });
  }

  start(interval_ms: number | undefined) {
    const payload = interval_ms !== undefined ? { interval_ms } : null;
    this.socket.send(encode(MSG_SUBSCRIBE, 0, payload), this.addr.port, this.addr.host);
  }

  stop() {
    try {
      this.socket.send(encode(MSG_UNSUBSCRIBE, 0, null), this.addr.port, this.addr.host);
    } catch {
      /* socket likely already closed */
    }
    this.socket.close();
  }

  private onMessage(buf: Buffer) {
    const decoded = decode(buf);
    if (!decoded || decoded.mt !== MSG_DATA || typeof decoded.body !== "object" || decoded.body === null) return;
    const fields = decoded.body as Record<string, unknown>;
    const payload = JSON.stringify({
      type: "data",
      sensor: this.id,
      t_ms: Date.now(),
      fields,
    });
    for (const ws of this.subscribers) {
      if (ws.readyState === ws.OPEN) ws.send(payload);
    }
  }
}

export class LiveHub {
  private taps = new Map<string, SensorTap>();
  private sensorPorts = new Map<string, { host: string; port: number }>();

  constructor(private readonly apiBase: string) {}

  /** Fetch `/discover` and remember each sensor's data port. Re-callable to refresh. */
  async refreshDiscovery(): Promise<{ id: string; display_name: string; data_port: number }[]> {
    const res = await fetch(`${this.apiBase}/discover`);
    const json = (await res.json()) as { sensors: { id: string; display_name?: string; data_port: number }[] };
    const host = new URL(this.apiBase).hostname;
    this.sensorPorts.clear();
    for (const s of json.sensors) {
      this.sensorPorts.set(s.id, { host, port: s.data_port });
    }
    return json.sensors.map((s) => ({
      id: s.id,
      display_name: s.display_name ?? s.id,
      data_port: s.data_port,
    }));
  }

  /** Best-effort schema lookup for one sensor. */
  async fetchSchema(id: string): Promise<{ name: string; type_name?: string; unit?: string; description?: string }[]> {
    try {
      const res = await fetch(`${this.apiBase}/${id}/info`);
      if (!res.ok) return [];
      const json = (await res.json()) as { data_schema?: { name: string; type_name?: string; unit?: string; description?: string }[] };
      return json.data_schema ?? [];
    } catch {
      return [];
    }
  }

  subscribe(ws: WebSocket, sensors: string[], interval_ms?: number) {
    for (const id of sensors) {
      const addr = this.sensorPorts.get(id);
      if (!addr) continue;
      let tap = this.taps.get(id);
      if (!tap) {
        tap = new SensorTap(id, addr.host, addr.port);
        tap.start(interval_ms);
        this.taps.set(id, tap);
      }
      tap.subscribers.add(ws);
    }
  }

  unsubscribe(ws: WebSocket, sensors: string[]) {
    for (const id of sensors) {
      const tap = this.taps.get(id);
      if (!tap) continue;
      tap.subscribers.delete(ws);
      if (tap.subscribers.size === 0) {
        tap.stop();
        this.taps.delete(id);
      }
    }
  }

  /** Called when a client disconnects — drops it from every tap. */
  detach(ws: WebSocket) {
    for (const [id, tap] of this.taps) {
      tap.subscribers.delete(ws);
      if (tap.subscribers.size === 0) {
        tap.stop();
        this.taps.delete(id);
      }
    }
  }
}
