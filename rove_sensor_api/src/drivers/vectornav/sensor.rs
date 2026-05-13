//! `SensorDriver` implementation for the VectorNav VN-300.
//!
//! The VN-300 is read-mostly: the device pushes async ASCII messages
//! (`$VNINS`, etc.) at a configured rate, the read loop parses them into
//! shared state, and `read_data()` returns a snapshot of that state.
//!
//! Commands are REST-mode (one-shot). Supported actions:
//!   - `tare`                          — `$VNTAR` (set current orientation as zero)
//!   - `reset`                         — `$VNRST` (soft reboot)
//!   - `restore_factory_settings`      — `$VNRFS`
//!   - `write_settings`                — `$VNWNV` (persist current registers to flash)
//!   - `set_initial_heading {heading}` — `$VNSIH,<deg>`
//!   - `set_async_type {ador}`         — write register 6 (ADOR)
//!   - `set_async_freq {freq}`         — write register 7 (ADOF, Hz)
//!   - `raw {command}`                 — send arbitrary VN ASCII (checksum auto-appended)
//!
//! Multiple actions in one payload are sent in the order listed above.

use std::sync::{Arc, RwLock};

use serde_json::Value;
use tokio_util::sync::CancellationToken;

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;

use super::protocol::format_command;
use super::serial::{send_command, SerialWriter};
use super::state::VectorNavState;

pub struct VectorNavSensor {
    id_str: String,
    display_name: String,
    state: Arc<RwLock<VectorNavState>>,
    writer: SerialWriter,
    /// Cancels the background read task on drop. Held for lifetime parity
    /// with `OdriveNode::_watchdog_cancel`.
    _read_cancel: CancellationToken,
    /// Original port name, surfaced in the display name for multi-VN setups.
    port_name: String,
}

impl VectorNavSensor {
    pub fn new(
        port_name: String,
        state: Arc<RwLock<VectorNavState>>,
        writer: SerialWriter,
        read_cancel: CancellationToken,
    ) -> Self {
        // Strip a leading /dev/ for a tidier sensor id.
        let short = port_name
            .rsplit('/')
            .next()
            .unwrap_or(&port_name)
            .to_string();
        let id_str = format!("vectornav_{short}");
        let display_name = format!("VectorNav VN-300 ({port_name})");
        Self {
            id_str,
            display_name,
            state,
            writer,
            _read_cancel: read_cancel,
            port_name,
        }
    }

    /// Send a fully-framed VN command from a sync context, blocking until the
    /// bytes hit the serial port. Mirrors the pattern used by `OdriveNode`.
    fn send_blocking(&self, framed: String) -> Result<(), DriverError> {
        let writer = self.writer.clone();
        tokio::task::block_in_place(|| {
            tokio::runtime::Handle::current()
                .block_on(async move { send_command(&writer, &framed).await })
        })
        .map_err(|e| DriverError::CommandFailed(e.to_string()))
    }

    /// Build and send a VN command body (without `$` prefix or `*XX` suffix).
    fn send_body(&self, body: &str) -> Result<(), DriverError> {
        self.send_blocking(format_command(body))
    }
}

impl SensorDriver for VectorNavSensor {
    fn id(&self) -> &str {
        &self.id_str
    }

    fn display_name(&self) -> &str {
        &self.display_name
    }

    fn command_mode(&self) -> CommandMode {
        // VN-300 commands are one-shot; the device pushes data on its own schedule.
        CommandMode::Rest
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("port",                 "Serial device path",                                 "String"),
            FieldDescriptor::new("gps_tow",              "GPS time of week",                "f64").with_unit("s"),
            FieldDescriptor::new("gps_week",             "GPS week number",                                    "u16"),
            FieldDescriptor::new("ins_status_raw",       "INS status bitfield (raw)",                          "u16"),
            FieldDescriptor::new("ins_mode",             "INS mode (0=NotTracking, 1=Aligning, 2=Tracking, 3=LossOfGNSS)", "u8"),
            FieldDescriptor::new("ins_error",            "INS sensor error code (0 = none)",                   "u8"),
            FieldDescriptor::new("gnss_fix",             "GNSS receiver has a valid fix",                      "bool"),
            FieldDescriptor::new("gnss_compass_active",  "GNSS compass operational",                           "bool"),
            FieldDescriptor::new("gnss_heading_aiding",  "INS heading currently aided by GNSS compass",        "bool"),
            FieldDescriptor::new("yaw",                  "Yaw / true heading",              "f32").with_unit("deg"),
            FieldDescriptor::new("pitch",                "Pitch",                           "f32").with_unit("deg"),
            FieldDescriptor::new("roll",                 "Roll",                            "f32").with_unit("deg"),
            FieldDescriptor::new("latitude",             "WGS84 latitude",                  "f64").with_unit("deg"),
            FieldDescriptor::new("longitude",            "WGS84 longitude",                 "f64").with_unit("deg"),
            FieldDescriptor::new("altitude",             "Altitude above WGS84 ellipsoid",  "f64").with_unit("m"),
            FieldDescriptor::new("vel_north",            "Velocity North (NED)",            "f32").with_unit("m/s"),
            FieldDescriptor::new("vel_east",             "Velocity East (NED)",             "f32").with_unit("m/s"),
            FieldDescriptor::new("vel_down",             "Velocity Down (NED)",             "f32").with_unit("m/s"),
            FieldDescriptor::new("att_uncertainty",      "Attitude uncertainty (1σ)",       "f32").with_unit("deg"),
            FieldDescriptor::new("pos_uncertainty",      "Position uncertainty (1σ)",       "f32").with_unit("m"),
            FieldDescriptor::new("vel_uncertainty",      "Velocity uncertainty (1σ)",       "f32").with_unit("m/s"),
            FieldDescriptor::new("mag_x",                "Magnetic field X (body)",         "f32").with_unit("Gauss"),
            FieldDescriptor::new("mag_y",                "Magnetic field Y (body)",         "f32").with_unit("Gauss"),
            FieldDescriptor::new("mag_z",                "Magnetic field Z (body)",         "f32").with_unit("Gauss"),
            FieldDescriptor::new("accel_x",              "Acceleration X (body)",           "f32").with_unit("m/s²"),
            FieldDescriptor::new("accel_y",              "Acceleration Y (body)",           "f32").with_unit("m/s²"),
            FieldDescriptor::new("accel_z",              "Acceleration Z (body)",           "f32").with_unit("m/s²"),
            FieldDescriptor::new("gyro_x",               "Angular rate X (body)",           "f32").with_unit("rad/s"),
            FieldDescriptor::new("gyro_y",               "Angular rate Y (body)",           "f32").with_unit("rad/s"),
            FieldDescriptor::new("gyro_z",               "Angular rate Z (body)",           "f32").with_unit("rad/s"),
            FieldDescriptor::new("gnss_num_sats",        "GNSS satellites used",                               "u8"),
            FieldDescriptor::new("gnss_fix_type",        "GNSS fix type (0/1/2/3)",                            "u8"),
            FieldDescriptor::new("temperature",          "IMU temperature",                 "f32").with_unit("°C"),
            FieldDescriptor::new("pressure",             "Barometric pressure",             "f32").with_unit("kPa"),
            FieldDescriptor::new("last_async_header",    "Header of most recent parsed message",               "String"),
            FieldDescriptor::new("messages_parsed",      "Total async messages parsed",                        "u64"),
            FieldDescriptor::new("messages_dropped",     "Total malformed lines dropped",                      "u64"),
            FieldDescriptor::new("timestamp_ns",         "Unix timestamp of most recent message",              "i64").with_unit("ns"),
        ]
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("tare",                     "Set current orientation as zero ($VNTAR)", "bool"),
            FieldDescriptor::new("reset",                    "Soft-reboot the unit ($VNRST)",            "bool"),
            FieldDescriptor::new("restore_factory_settings", "Restore factory defaults ($VNRFS)",        "bool"),
            FieldDescriptor::new("write_settings",           "Persist current registers to flash ($VNWNV)", "bool"),
            FieldDescriptor::new("set_initial_heading",      "Provide initial heading hint",        "f32").with_unit("deg"),
            FieldDescriptor::new("set_async_type",           "ADOR (register 6) — async output type id",  "u32"),
            FieldDescriptor::new("set_async_freq",           "ADOF (register 7) — async output rate",     "u32").with_unit("Hz"),
            FieldDescriptor::new("raw",                      "Raw VN command body, e.g. \"VNRRG,1\" — checksum auto-appended", "String"),
        ]
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        let s = self.state.read().unwrap();
        Ok(serde_json::json!({
            "port":                 self.port_name,
            "gps_tow":              s.gps_tow,
            "gps_week":             s.gps_week,
            "ins_status_raw":       s.ins_status_raw,
            "ins_mode":             s.ins_mode,
            "ins_error":            s.ins_error,
            "gnss_fix":             s.gnss_fix,
            "gnss_compass_active":  s.gnss_compass_active,
            "gnss_heading_aiding":  s.gnss_heading_aiding,
            "yaw":                  s.yaw,
            "pitch":                s.pitch,
            "roll":                 s.roll,
            "latitude":             s.latitude,
            "longitude":            s.longitude,
            "altitude":             s.altitude,
            "vel_north":            s.vel_north,
            "vel_east":             s.vel_east,
            "vel_down":             s.vel_down,
            "att_uncertainty":      s.att_uncertainty,
            "pos_uncertainty":      s.pos_uncertainty,
            "vel_uncertainty":      s.vel_uncertainty,
            "mag_x":                s.mag_x,
            "mag_y":                s.mag_y,
            "mag_z":                s.mag_z,
            "accel_x":              s.accel_x,
            "accel_y":              s.accel_y,
            "accel_z":              s.accel_z,
            "gyro_x":               s.gyro_x,
            "gyro_y":               s.gyro_y,
            "gyro_z":               s.gyro_z,
            "gnss_num_sats":        s.gnss_num_sats,
            "gnss_fix_type":        s.gnss_fix_type,
            "temperature":          s.temperature,
            "pressure":             s.pressure,
            "last_async_header":    s.last_async_header,
            "messages_parsed":      s.messages_parsed,
            "messages_dropped":     s.messages_dropped,
            "timestamp_ns":         s.timestamp_ns,
        }))
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        let mut sent = Vec::<&str>::new();

        // ── In-place actions, ordered so a save-then-reset payload behaves sensibly. ──

        if payload.get("tare").and_then(Value::as_bool).unwrap_or(false) {
            self.send_body("VNTAR")?;
            sent.push("tare");
        }

        if let Some(h) = payload.get("set_initial_heading").and_then(Value::as_f64) {
            // $VNSIH expects the heading in degrees as a signed decimal.
            self.send_body(&format!("VNSIH,{:+.4}", h as f32))?;
            sent.push("set_initial_heading");
        }

        if let Some(ador) = payload.get("set_async_type").and_then(Value::as_u64) {
            // Register 6 (ADOR).
            self.send_body(&format!("VNWRG,06,{ador}"))?;
            sent.push("set_async_type");
        }

        if let Some(adof) = payload.get("set_async_freq").and_then(Value::as_u64) {
            // Register 7 (ADOF).
            self.send_body(&format!("VNWRG,07,{adof}"))?;
            sent.push("set_async_freq");
        }

        if let Some(raw) = payload.get("raw").and_then(Value::as_str) {
            // Strip a leading "$" so callers can paste lines verbatim from the manual.
            let body = raw.trim_start_matches('$');
            // Drop anything from "*" onward — we add the checksum.
            let body = body.split('*').next().unwrap_or(body);
            self.send_body(body.trim())?;
            sent.push("raw");
        }

        // ── Persistence-affecting actions, last so any newly-set register is
        //    in place before save/reset. ──

        if payload
            .get("write_settings")
            .and_then(Value::as_bool)
            .unwrap_or(false)
        {
            self.send_body("VNWNV")?;
            sent.push("write_settings");
        }

        if payload
            .get("restore_factory_settings")
            .and_then(Value::as_bool)
            .unwrap_or(false)
        {
            self.send_body("VNRFS")?;
            sent.push("restore_factory_settings");
        }

        if payload.get("reset").and_then(Value::as_bool).unwrap_or(false) {
            self.send_body("VNRST")?;
            sent.push("reset");
        }

        if sent.is_empty() {
            return Err(DriverError::CommandFailed(
                "no recognised command fields in payload".into(),
            ));
        }

        Ok(serde_json::json!({ "sent": sent }))
    }
}
