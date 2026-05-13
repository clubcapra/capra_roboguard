//! Robotiq 2F-140 driver configuration loaded from `config/robotiq.toml`.

use serde::Deserialize;

use crate::core::config::{self, ConfigError};

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotiqConfig {
    /// Serial device for the USB→RS-485 adapter, e.g. `/dev/ttyUSB_gripper`.
    pub port: String,

    /// Robotiq factory default is 115200.
    #[serde(default = "default_baudrate")]
    pub baudrate: u32,

    /// Modbus slave ID. Robotiq factory default is 9.
    #[serde(default = "default_slave_id")]
    pub slave_id: u8,

    /// Status read cadence (ms). 50 ms = 20 Hz, plenty for grasp feedback.
    #[serde(default = "default_poll_interval_ms")]
    pub poll_interval_ms: u64,

    /// Send rACT=1 on connect and wait for activation to complete.
    #[serde(default = "default_auto_activate")]
    pub auto_activate: bool,

    /// Max time to wait for `gSTA == 3` after auto-activate before giving up
    /// and continuing anyway. Activation is fast (<1 s) when nothing is
    /// blocking the jaws.
    #[serde(default = "default_activation_timeout_ms")]
    pub activation_timeout_ms: u64,
}

impl RobotiqConfig {
    pub fn load() -> Result<Option<Self>, ConfigError> {
        config::load_optional("robotiq.toml")
    }
}

fn default_baudrate() -> u32 {
    115200
}
fn default_slave_id() -> u8 {
    9
}
fn default_poll_interval_ms() -> u64 {
    50
}
fn default_auto_activate() -> bool {
    true
}
fn default_activation_timeout_ms() -> u64 {
    5000
}
