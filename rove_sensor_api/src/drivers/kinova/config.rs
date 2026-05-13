use std::net::Ipv4Addr;
use std::path::PathBuf;

use serde::Deserialize;

use crate::core::config::{self, ConfigError};

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct KinovaConfig {
    #[serde(default)]
    pub lib_dir: Option<PathBuf>,

    pub local_ip: Ipv4Addr,

    #[serde(default = "default_subnet")]
    pub local_subnet: Ipv4Addr,

    #[serde(default = "default_robot_ip")]
    pub robot_ip: Ipv4Addr,

    #[serde(default = "default_local_cmd_port")]
    pub local_cmd_port: u16,
    #[serde(default = "default_local_bcast_port")]
    pub local_bcast_port: u16,
    #[serde(default = "default_robot_port")]
    pub robot_port: u16,

    #[serde(default = "default_rx_timeout_ms")]
    pub rx_timeout_ms: u32,

    #[serde(default = "default_command_rate_hz")]
    pub command_rate_hz: u32,

    /// Connect via USB instead of Ethernet.  When true, local_ip / robot_ip /
    /// port settings are ignored and the USB command layer is used instead.
    #[serde(default)]
    pub use_usb: bool,

    #[serde(default)]
    pub joint_offsets: [f32; 6],
}

impl KinovaConfig {
    pub fn load() -> Result<Option<Self>, ConfigError> {
        config::load_optional("kinova.toml")
    }
}

fn default_subnet() -> Ipv4Addr {
    Ipv4Addr::new(255, 255, 255, 0)
}
fn default_robot_ip() -> Ipv4Addr {
    Ipv4Addr::new(192, 168, 2, 50)
}
fn default_local_cmd_port() -> u16 {
    25015
}
fn default_local_bcast_port() -> u16 {
    25025
}
fn default_robot_port() -> u16 {
    55000
}
fn default_rx_timeout_ms() -> u32 {
    // Direct Ethernet round-trips are < 5ms; 20ms gives ample headroom while
    // keeping stalls short when the ARM firmware briefly drops a command ACK.
    // Increase this only if operating over a high-latency network path.
    20
}
fn default_command_rate_hz() -> u32 {
    100
}
