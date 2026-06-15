import type { RangeResult, SensorInfo } from "./types";

export async function fetchSensors(): Promise<SensorInfo[]> {
  const res = await fetch("/api/sensors");
  if (!res.ok) throw new Error(`/api/sensors ${res.status}`);
  const json = (await res.json()) as { sensors: SensorInfo[] };
  return json.sensors;
}

export async function fetchRange(args: {
  sensor: string;
  start_ms: number;
  end_ms: number;
  fields?: string[];
  maxPoints?: number;
  signal?: AbortSignal;
}): Promise<RangeResult> {
  const params = new URLSearchParams({
    sensor: args.sensor,
    start_ms: String(Math.floor(args.start_ms)),
    end_ms: String(Math.ceil(args.end_ms)),
  });
  if (args.fields?.length) params.set("fields", args.fields.join(","));
  if (args.maxPoints) params.set("max_points", String(args.maxPoints));
  const res = await fetch(`/api/log/range?${params}`, { signal: args.signal });
  if (!res.ok) throw new Error(`/api/log/range ${res.status}`);
  return (await res.json()) as RangeResult;
}

export type LiveMessage =
  | { type: "data"; sensor: string; t_ms: number; fields: Record<string, number | boolean | string | null> };

/** Single WS connection shared across the app — caller adds/removes listeners. */
export class LiveSocket {
  private ws: WebSocket | null = null;
  private listeners = new Set<(msg: LiveMessage) => void>();
  private subscriptions = new Map<string, number>(); // sensor → ref count
  private reconnectTimer: number | null = null;

  connect() {
    if (this.ws && this.ws.readyState <= 1) return;
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    this.ws = new WebSocket(`${proto}//${location.host}/ws/live`);
    this.ws.addEventListener("open", () => {
      // Re-send subscriptions so a reconnect resumes the stream.
      if (this.subscriptions.size) {
        this.ws?.send(
          JSON.stringify({ type: "subscribe", sensors: Array.from(this.subscriptions.keys()) }),
        );
      }
    });
    this.ws.addEventListener("message", (e) => {
      try {
        const msg = JSON.parse(e.data) as LiveMessage;
        for (const l of this.listeners) l(msg);
      } catch {
        /* ignore */
      }
    });
    this.ws.addEventListener("close", () => {
      if (this.reconnectTimer != null) return;
      this.reconnectTimer = window.setTimeout(() => {
        this.reconnectTimer = null;
        this.connect();
      }, 1000);
    });
  }

  addListener(cb: (msg: LiveMessage) => void): () => void {
    this.listeners.add(cb);
    return () => this.listeners.delete(cb);
  }

  /** Reference-counted subscribe. The actual WS subscribe is sent on the 0→1 transition. */
  subscribe(sensor: string) {
    const prev = this.subscriptions.get(sensor) ?? 0;
    this.subscriptions.set(sensor, prev + 1);
    if (prev === 0 && this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ type: "subscribe", sensors: [sensor] }));
    }
  }

  unsubscribe(sensor: string) {
    const prev = this.subscriptions.get(sensor) ?? 0;
    if (prev <= 1) {
      this.subscriptions.delete(sensor);
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify({ type: "unsubscribe", sensors: [sensor] }));
      }
    } else {
      this.subscriptions.set(sensor, prev - 1);
    }
  }
}

export const liveSocket = new LiveSocket();
