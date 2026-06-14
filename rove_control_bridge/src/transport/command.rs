//! Fire-and-forget command sender to rove_sensor_api command ports.
//!
//! Each call wraps a JSON payload in a `Command` (0x10) frame and sends it to
//! `host:command_port`. Mirrors rove_control_bridge's SensorApiUdpClient: dropped
//! datagrams are tolerated (the control loop re-sends at rate). A `--dry-run`
//! sink logs instead of sending so the first live bring-up can't move the robot.

use super::packet::{encode, MSG_COMMAND};
use anyhow::{Context, Result};
use serde_json::Value;
use std::sync::atomic::{AtomicU16, Ordering};
use tokio::net::UdpSocket;

pub struct CommandSink {
    sock: UdpSocket,
    host: String,
    seq: AtomicU16,
    dry_run: bool,
}

impl CommandSink {
    pub fn host(&self) -> String {
        self.host.clone()
    }

    pub async fn new(host: impl Into<String>, dry_run: bool) -> Result<Self> {
        let sock = UdpSocket::bind(("0.0.0.0", 0))
            .await
            .context("binding command socket")?;
        Ok(Self {
            sock,
            host: host.into(),
            seq: AtomicU16::new(0),
            dry_run,
        })
    }

    /// Send `payload` to a sensor command port. Never blocks the control loop.
    pub async fn send(&self, command_port: u16, payload: &Value) -> Result<()> {
        if self.dry_run {
            tracing::info!("[dry-run] -> :{command_port} {payload}");
            return Ok(());
        }
        let seq = self.seq.fetch_add(1, Ordering::Relaxed);
        let frame = encode(MSG_COMMAND, seq, payload);
        self.sock
            .send_to(&frame, (self.host.as_str(), command_port))
            .await
            .with_context(|| format!("sending command to :{command_port}"))?;
        Ok(())
    }
}
