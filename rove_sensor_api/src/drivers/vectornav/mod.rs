//! VectorNav VN-300 dual-antenna GNSS/INS driver.
//!
//! Talks to the sensor over a serial port (typically `/dev/ttyUSB_VN300` via
//! the udev rule shipped in `README.md`). The device pushes async ASCII
//! messages — by default `$VNINS` at 40 Hz — which a background task parses
//! into a shared snapshot. Commands such as `tare`, `reset`, and raw
//! register writes are sent over the same port via `execute_command`.
//!
//! See `quick start guide.pdf` for sensor-side configuration (antenna offset,
//! compass baseline, reference frame rotation). This driver does **not**
//! reconfigure the sensor on connect — it parses whatever async messages the
//! device is already configured to emit.

pub mod binary;
pub mod protocol;
pub mod sensor;
pub mod serial;
pub mod state;

use std::io;
use std::sync::{Arc, RwLock};

use tokio_util::sync::CancellationToken;

use sensor::VectorNavSensor;
use state::VectorNavState;

/// IMU rate divider for Binary Output 1. VN-300 IMU samples internally at
/// 800 Hz, so a divider of 20 yields a 40 Hz binary stream — same effective
/// rate as the previous `$VNINS` ASCII configuration.
const BINARY_RATE_DIVIDER: u16 = 20;

/// Connect to a VectorNav VN-300 over the given serial port at the specified
/// baudrate (factory default: 115200). On success, spawns a background task
/// that reads async messages and updates the cached state, and returns a
/// `VectorNavSensor` ready to be registered with the `SensorRegistry`.
///
/// After the read loop is alive we push a Binary Output 1 configuration
/// (Common group: TimeGps, YPR, AngularRate, Position, Velocity, Accel,
/// MagPres, InsStatus) and disable ADOR so the only async traffic is the
/// binary frame. Failures to send config are logged but non-fatal: the read
/// loop is permissive and will parse whatever the device happens to emit.
///
/// Configuration is *not* persisted to flash (`$VNWNV`). The driver re-applies
/// it on each connect, so a power cycle leaves the user's saved settings
/// intact for direct serial-terminal use.
pub async fn connect(port_name: &str, baudrate: u32) -> io::Result<VectorNavSensor> {
    tracing::info!(port = port_name, baudrate, "opening VectorNav serial port");

    let (read_half, writer) = serial::open(port_name, baudrate)?;

    let state = Arc::new(RwLock::new(VectorNavState::default()));
    let cancel = CancellationToken::new();

    tokio::spawn(serial::run_read_loop(
        read_half,
        state.clone(),
        cancel.clone(),
    ));

    tracing::info!(port = port_name, "VectorNav read loop started");

    // Configure binary output and silence ADOR. Field bitmaps are written in
    // hex per the VN ASCII convention; AsyncMode/RateDivider are decimal.
    let cmds = [
        format!(
            "VNWRG,75,1,{},01,{:04X}",
            BINARY_RATE_DIVIDER,
            binary::COMMON_DEFAULT_FIELDS
        ),
        "VNWRG,06,0".to_string(),
    ];
    for body in &cmds {
        let framed = protocol::format_command(body);
        if let Err(e) = serial::send_command(&writer, &framed).await {
            tracing::warn!(error = %e, body = body.as_str(), "VectorNav config send failed");
        } else {
            tracing::debug!(cmd = body.as_str(), "VectorNav config sent");
        }
    }

    Ok(VectorNavSensor::new(
        port_name.to_string(),
        state,
        writer,
        cancel,
    ))
}
