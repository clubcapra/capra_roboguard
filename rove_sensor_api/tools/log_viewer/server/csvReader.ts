import fs from "node:fs";
import path from "node:path";
import readline from "node:readline";

// Default mirrors the LOG_DIR set by scripts/rove-sensor-api.service.
export const LOG_ROOT = process.env.LOG_DIR ?? "/var/log/rove-sensor-api";

export type SensorMeta = {
  id: string;
  /** Sorted list of `YYYY-MM-DD` dates that have any data for this sensor. */
  dates: string[];
  /** Header row from the most recent CSV — used as the field list. */
  fields: string[];
};

export type RangePoint = {
  t_ms: number;
  values: (number | null)[];
};

export type RangeResult = {
  sensor: string;
  fields: string[];
  points: RangePoint[];
  /** True if the result was downsampled to maxPoints. */
  downsampled: boolean;
};

function listDir(p: string): string[] {
  try {
    return fs.readdirSync(p).sort();
  } catch {
    return [];
  }
}

/** Scan LOG_ROOT for sensors. A sensor is any `*.csv` filename appearing under any hour bucket. */
export function listSensors(): SensorMeta[] {
  const acc = new Map<string, { dates: Set<string>; latestFile: string }>();
  for (const date of listDir(LOG_ROOT)) {
    const dateDir = path.join(LOG_ROOT, date);
    if (!fs.statSync(dateDir, { throwIfNoEntry: false })?.isDirectory()) continue;
    for (const hour of listDir(dateDir)) {
      const hourDir = path.join(dateDir, hour);
      for (const file of listDir(hourDir)) {
        if (!file.endsWith(".csv")) continue;
        const id = file.slice(0, -4);
        const entry = acc.get(id) ?? { dates: new Set<string>(), latestFile: "" };
        entry.dates.add(date);
        // Later iterations win → ends up as the most-recent file.
        entry.latestFile = path.join(hourDir, file);
        acc.set(id, entry);
      }
    }
  }
  const out: SensorMeta[] = [];
  for (const [id, { dates, latestFile }] of acc) {
    out.push({
      id,
      dates: Array.from(dates).sort(),
      fields: readHeader(latestFile),
    });
  }
  return out.sort((a, b) => a.id.localeCompare(b.id));
}

function readHeader(file: string): string[] {
  try {
    const fd = fs.openSync(file, "r");
    const buf = Buffer.alloc(8192);
    const n = fs.readSync(fd, buf, 0, 8192, 0);
    fs.closeSync(fd);
    const nl = buf.indexOf(0x0a, 0);
    const end = nl >= 0 ? nl : n;
    return buf
      .slice(0, end)
      .toString("utf8")
      .split(",")
      .map((s) => s.trim());
  } catch {
    return [];
  }
}

/** Iterate the hour buckets [start_ms, end_ms] in order, returning paths to <sensor>.csv files that exist. */
function hourFilesFor(sensor: string, start_ms: number, end_ms: number): string[] {
  const files: string[] = [];
  // Walk hour-by-hour. Bound the loop to avoid runaway ranges.
  const startHour = Math.floor(start_ms / 3_600_000) * 3_600_000;
  const endHour = Math.floor(end_ms / 3_600_000) * 3_600_000;
  for (let h = startHour; h <= endHour; h += 3_600_000) {
    const d = new Date(h);
    const date = d.toISOString().slice(0, 10);
    const hour = d.toISOString().slice(11, 13);
    const file = path.join(LOG_ROOT, date, hour, `${sensor}.csv`);
    if (fs.existsSync(file)) files.push(file);
  }
  return files;
}

function parseValue(s: string): number | null {
  if (s === "" || s === "NaN") return null;
  if (s === "true") return 1;
  if (s === "false") return 0;
  const n = Number(s);
  return Number.isFinite(n) ? n : null;
}

/**
 * Read a time range for one sensor. Filters columns to `fields` (or all if undefined)
 * and stride-samples to ≤ maxPoints rows.
 *
 * Timestamps in the CSVs are nanoseconds since epoch; we convert to ms for the wire format.
 */
export async function readRange(args: {
  sensor: string;
  start_ms: number;
  end_ms: number;
  fields?: string[];
  maxPoints?: number;
}): Promise<RangeResult> {
  const { sensor, start_ms, end_ms } = args;
  const maxPoints = args.maxPoints ?? 5000;
  const files = hourFilesFor(sensor, start_ms, end_ms);

  if (files.length === 0) {
    return { sensor, fields: args.fields ?? [], points: [], downsampled: false };
  }

  // First pass: count rows to derive a stride. Cheap because we only iterate
  // bytes, not parse CSV.
  let totalRows = 0;
  for (const f of files) totalRows += await countLines(f);
  totalRows = Math.max(0, totalRows - files.length); // subtract one header per file
  const stride = Math.max(1, Math.ceil(totalRows / maxPoints));

  // Second pass: parse with stride. We resolve the field index from each file's
  // own header in case schemas drift across hours.
  let header: string[] = [];
  let fieldIdxs: number[] = [];
  let tsIdx = 0;
  const points: RangePoint[] = [];
  let rowCursor = 0;

  for (const file of files) {
    const stream = fs.createReadStream(file, { encoding: "utf8" });
    const rl = readline.createInterface({ input: stream, crlfDelay: Infinity });
    let isHeader = true;

    for await (const line of rl) {
      if (!line) continue;
      if (isHeader) {
        header = line.split(",");
        tsIdx = header.indexOf("timestamp_ns");
        // If caller asked for specific fields we use those; otherwise everything except timestamp.
        const wanted =
          args.fields && args.fields.length
            ? args.fields
            : header.filter((h) => h !== "timestamp_ns");
        fieldIdxs = wanted.map((f) => header.indexOf(f));
        isHeader = false;
        continue;
      }

      if (rowCursor++ % stride !== 0) continue;

      // Splitting once per row is the hot path; avoid String.prototype.split's
      // regex form. The CSVs never quote values, so a simple split is safe.
      const cols = line.split(",");
      const t_ns = Number(cols[tsIdx]);
      if (!Number.isFinite(t_ns)) continue;
      const t_ms = t_ns / 1e6;
      if (t_ms < start_ms || t_ms > end_ms) continue;

      const values = fieldIdxs.map((i) => (i < 0 ? null : parseValue(cols[i] ?? "")));
      points.push({ t_ms, values });
    }
  }

  const fields =
    args.fields && args.fields.length
      ? args.fields
      : header.filter((h) => h !== "timestamp_ns");

  return { sensor, fields, points, downsampled: stride > 1 };
}

function countLines(file: string): Promise<number> {
  return new Promise((resolve) => {
    let n = 0;
    fs.createReadStream(file)
      .on("data", (chunk) => {
        const buf = Buffer.isBuffer(chunk) ? chunk : Buffer.from(chunk);
        for (let i = 0; i < buf.length; i++) if (buf[i] === 0x0a) n++;
      })
      .on("end", () => resolve(n))
      .on("error", () => resolve(0));
  });
}
