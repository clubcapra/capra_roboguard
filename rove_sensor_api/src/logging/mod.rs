//! Time-series CSV logging for sensor data and command inputs.
//!
//! Layout under `LOG_DIR` (default `./logs`):
//! ```text
//! logs/
//!   2026-05-02/
//!     14/
//!       inputs.csv
//!       kinova_arm.csv
//!       odrive_node_1.csv
//!       ...
//! ```
//!
//! Files are split per UTC hour. Writers are opened in append mode so a session
//! that crosses an hour boundary, or a restart mid-hour, never drops rows: the
//! existing file is reopened and rows are appended after the existing content.
//! Every row is flushed before the lock is released, so an unclean shutdown
//! loses at most the row currently being formatted.

use std::collections::HashMap;
use std::fs::{self, File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use chrono::{DateTime, Datelike, Timelike, Utc};
use serde_json::Value;

use crate::core::registry::SensorRegistry;

/// One open writer for a single CSV file. The slot remembers the absolute path
/// so we can detect when the hour rolls over and we need to swap files.
struct Writer {
    path: PathBuf,
    sink: BufWriter<File>,
}

impl Writer {
    fn open(path: PathBuf, header: &[String]) -> std::io::Result<Self> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let needs_header = !path.exists() || fs::metadata(&path).map(|m| m.len() == 0).unwrap_or(true);

        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)?;
        let mut sink = BufWriter::new(file);

        if needs_header {
            write_csv_row(&mut sink, header)?;
            sink.flush()?;
        }

        Ok(Self { path, sink })
    }

    fn write_row(&mut self, row: &[String]) -> std::io::Result<()> {
        write_csv_row(&mut self.sink, row)?;
        // Flush every row — favors durability over throughput. Sensor logs
        // top out at a few hundred rows/sec across all drivers, well within
        // what a buffered+flushed file can sustain.
        self.sink.flush()
    }
}

/// Write a single CSV row, RFC-4180-ish: fields containing comma, quote, CR or
/// LF are wrapped in double quotes with internal quotes doubled.
fn write_csv_row<W: Write>(w: &mut W, row: &[String]) -> std::io::Result<()> {
    let mut first = true;
    for field in row {
        if !first {
            w.write_all(b",")?;
        }
        first = false;
        if field
            .as_bytes()
            .iter()
            .any(|&b| b == b',' || b == b'"' || b == b'\n' || b == b'\r')
        {
            w.write_all(b"\"")?;
            for b in field.as_bytes() {
                if *b == b'"' {
                    w.write_all(b"\"\"")?;
                } else {
                    w.write_all(std::slice::from_ref(b))?;
                }
            }
            w.write_all(b"\"")?;
        } else {
            w.write_all(field.as_bytes())?;
        }
    }
    w.write_all(b"\n")
}

struct Inner {
    /// Per-sensor data writers, keyed by sensor id.
    sensors: HashMap<String, Writer>,
    /// Cached header columns per sensor so we can format rows in a stable order
    /// even if `data_schema()` returns a different ordering between calls.
    headers: HashMap<String, Vec<String>>,
    /// Inputs writer (shared across sensors).
    inputs: Option<Writer>,
}

pub struct LogManager {
    log_dir: PathBuf,
    inner: Mutex<Inner>,
}

const INPUTS_HEADER: &[&str] = &[
    "timestamp_ns",
    "sensor_id",
    "source",
    "client",
    "user_agent",
    "kind",
    "payload",
    "result",
    "error",
];

impl LogManager {
    /// Create a manager rooted at `log_dir`. The directory is created if it
    /// does not exist.
    pub fn new(log_dir: PathBuf) -> std::io::Result<Self> {
        fs::create_dir_all(&log_dir)?;
        Ok(Self {
            log_dir,
            inner: Mutex::new(Inner {
                sensors: HashMap::new(),
                headers: HashMap::new(),
                inputs: None,
            }),
        })
    }

    pub fn log_dir(&self) -> &Path {
        &self.log_dir
    }

    /// Build `<log_dir>/<YYYY-MM-DD>/<HH>/<file>`.
    fn path_for(&self, ts: DateTime<Utc>, file: &str) -> PathBuf {
        let date = format!("{:04}-{:02}-{:02}", ts.year(), ts.month(), ts.day());
        let hour = format!("{:02}", ts.hour());
        self.log_dir.join(date).join(hour).join(file)
    }

    /// Log one snapshot for a sensor. `header` defines the column order — the
    /// first call for a given sensor freezes the column set.
    fn log_sensor_row(
        &self,
        sensor_id: &str,
        header: Vec<String>,
        row: Vec<String>,
        now: DateTime<Utc>,
    ) {
        let mut inner = self.inner.lock().unwrap();
        let target = self.path_for(now, &format!("{sensor_id}.csv"));

        // Freeze header on first row so column order is stable across hour
        // rotations (data_schema() should be stable, but don't rely on it).
        let header_ref: Vec<String> = inner
            .headers
            .entry(sensor_id.to_string())
            .or_insert_with(|| header)
            .clone();

        // Rotate if the path changed (hour boundary crossed) or this is the
        // first write for this sensor.
        let needs_open = inner
            .sensors
            .get(sensor_id)
            .map(|w| w.path != target)
            .unwrap_or(true);

        if needs_open {
            // Drop the previous writer first so its BufWriter flushes on Drop.
            inner.sensors.remove(sensor_id);
            match Writer::open(target.clone(), &header_ref) {
                Ok(w) => {
                    inner.sensors.insert(sensor_id.to_string(), w);
                }
                Err(e) => {
                    tracing::warn!(error = %e, sensor = sensor_id, path = %target.display(), "log open failed");
                    return;
                }
            }
        }

        if let Some(w) = inner.sensors.get_mut(sensor_id) {
            if let Err(e) = w.write_row(&row) {
                tracing::warn!(error = %e, sensor = sensor_id, "log write failed");
            }
        }
    }

    /// Log one command/input event.
    ///
    /// `client` is the originating peer in `host:port` form (UDP datagram source
    /// address, or HTTP TCP peer). `user_agent` is the HTTP `User-Agent` header
    /// when available, empty otherwise.
    pub fn log_input(
        &self,
        sensor_id: &str,
        source: &str,
        client: &str,
        user_agent: &str,
        kind: &str,
        payload: &Value,
        result: Result<&Value, &str>,
    ) {
        let now = Utc::now();
        let mut inner = self.inner.lock().unwrap();
        let target = self.path_for(now, "inputs.csv");

        let needs_open = inner.inputs.as_ref().map(|w| w.path != target).unwrap_or(true);

        if needs_open {
            inner.inputs = None;
            let header: Vec<String> = INPUTS_HEADER.iter().map(|s| s.to_string()).collect();
            match Writer::open(target.clone(), &header) {
                Ok(w) => inner.inputs = Some(w),
                Err(e) => {
                    tracing::warn!(error = %e, path = %target.display(), "inputs log open failed");
                    return;
                }
            }
        }

        let (result_str, error_str) = match result {
            Ok(v) => (v.to_string(), String::new()),
            Err(e) => (String::new(), e.to_string()),
        };
        let row = vec![
            ts_ns(now).to_string(),
            sensor_id.to_string(),
            source.to_string(),
            client.to_string(),
            user_agent.to_string(),
            kind.to_string(),
            payload.to_string(),
            result_str,
            error_str,
        ];

        if let Some(w) = inner.inputs.as_mut() {
            if let Err(e) = w.write_row(&row) {
                tracing::warn!(error = %e, "inputs log write failed");
            }
        }
    }

    /// Spawn the polling task. `period` is the per-tick interval; every
    /// registered driver is polled once per tick.
    pub fn spawn_polling(self: Arc<Self>, registry: Arc<SensorRegistry>, period: Duration) {
        tokio::spawn(async move {
            let mut ticker = tokio::time::interval(period);
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                ticker.tick().await;
                let now = Utc::now();
                let now_ns = ts_ns(now);
                for (id, driver) in registry.iter_drivers() {
                    let data = match driver.read_data() {
                        Ok(v) => v,
                        Err(e) => {
                            tracing::trace!(error = %e, sensor = %id, "skip log row: read_data failed");
                            continue;
                        }
                    };
                    let schema = driver.data_schema();
                    let mut header = Vec::with_capacity(schema.len() + 1);
                    header.push("timestamp_ns".to_string());
                    for f in &schema {
                        header.push(f.name.clone());
                    }
                    let mut row = Vec::with_capacity(header.len());
                    row.push(now_ns.to_string());
                    for f in &schema {
                        let cell = data
                            .get(&f.name)
                            .map(value_to_csv)
                            .unwrap_or_default();
                        row.push(cell);
                    }
                    self.log_sensor_row(&id, header, row, now);
                }
            }
        });
    }
}

fn ts_ns(now: DateTime<Utc>) -> i128 {
    // chrono::DateTime<Utc> → unix nanos. Fall back to SystemTime if chrono's
    // nanosecond conversion ever fails (it shouldn't for present-day clocks).
    if let Some(ns) = now.timestamp_nanos_opt() {
        ns as i128
    } else {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as i128)
            .unwrap_or(0)
    }
}

fn value_to_csv(v: &Value) -> String {
    match v {
        Value::Null => String::new(),
        Value::Bool(b) => b.to_string(),
        Value::Number(n) => n.to_string(),
        Value::String(s) => s.clone(),
        // Arrays/objects are flattened to compact JSON so a single-cell column
        // round-trips through `csv`-aware tooling.
        other => other.to_string(),
    }
}
