//! Robotiq 2F-140 adaptive gripper driver (Modbus RTU over USB→RS-485).
//!
//! # Overview
//!
//! Talks to the gripper over a serial-attached RS-485 adapter (Capra default
//! `/dev/ttyUSB_gripper`, slave ID 9, 115200 8N1). Exposes a single
//! `SensorDriver` named `robotiq_gripper`.
//!
//! # Configuration
//!
//! Loaded from `config/robotiq.toml` at startup (see `RobotiqConfig`). If the
//! file is missing the driver is skipped silently — same graceful pattern as
//! the other drivers.
//!
//! # Modbus layout (from the Robotiq manual)
//!
//! Output (write `0x03E8..0x03EA`, 3 holding registers):
//!   - reg0 high: ACTION = rACT | rGTO<<3 | rATR<<4 | rARD<<5
//!   - reg1 low : rPR (position 0..255, 0=open / 255=closed)
//!   - reg2 high: rSP (speed 0..255)
//!   - reg2 low : rFR (force 0..255)
//!
//! Input (read `0x07D0..0x07D2`, 3 holding registers):
//!   - reg0 high: gACT | gGTO<<3 | gSTA<<4 | gOBJ<<6
//!   - reg1 high: gFLT (low 4 bits)
//!   - reg1 low : gPR (position request echo)
//!   - reg2 high: gPO (actual position)
//!   - reg2 low : gCU (current, ≈ × 10 mA)

pub mod config;
pub mod gripper;
pub mod state;
pub mod worker;

use std::sync::{Arc, RwLock};
use std::time::Duration;

use tokio_modbus::prelude::*;
use tokio_serial::SerialStream;

use config::RobotiqConfig;
use gripper::RobotiqGripper;
use state::RobotiqState;

#[derive(Debug, thiserror::Error)]
pub enum ConnectError {
    #[error("opening serial port {port}: {source}")]
    OpenSerial {
        port: String,
        #[source]
        source: tokio_serial::Error,
    },
}

/// Open the serial port, attach a Modbus RTU client, spawn the worker task,
/// and return a ready-to-register `RobotiqGripper`.
///
/// The activation handshake (rACT=1 then wait for `gSTA == 3`) runs inside
/// the worker so that this function returns promptly even if the gripper is
/// slow to activate; `RobotiqState::link_up` flips to true as soon as the
/// first status read succeeds.
pub async fn connect(cfg: &RobotiqConfig) -> Result<RobotiqGripper, ConnectError> {
    tracing::info!(
        port = %cfg.port,
        baud = cfg.baudrate,
        slave = cfg.slave_id,
        "opening Robotiq Modbus channel"
    );

    let builder = tokio_serial::new(&cfg.port, cfg.baudrate);
    let serial = SerialStream::open(&builder).map_err(|source| ConnectError::OpenSerial {
        port: cfg.port.clone(),
        source,
    })?;

    let ctx = rtu::attach_slave(serial, Slave(cfg.slave_id));

    let state = Arc::new(RwLock::new(RobotiqState::default()));
    let handle = worker::spawn(
        ctx,
        state.clone(),
        Duration::from_millis(cfg.poll_interval_ms),
        cfg.auto_activate,
        Duration::from_millis(cfg.activation_timeout_ms),
    );

    Ok(RobotiqGripper::new(state, handle.cmd_tx))
}
