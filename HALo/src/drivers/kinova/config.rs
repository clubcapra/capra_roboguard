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
    /// Restore this base-frame gravity vector after limp mode (m/s²).
    #[serde(default = "default_gravity_vector")]
    pub gravity_vector: [f32; 3],
}

impl KinovaConfig {
    pub fn load() -> Result<Option<Self>, ConfigError> {
        config::load_driver_config("kinova.toml")
    }
}

impl config::DriverConfig for KinovaConfig {
    fn validate(&self) -> Result<(), String> {
        if (!self.use_usb
            && (self.local_cmd_port == 0
                || self.local_bcast_port == 0
                || self.local_cmd_port == self.local_bcast_port
                || self.robot_port == 0))
            || self.rx_timeout_ms == 0
            || !(1..=1000).contains(&self.command_rate_hz)
            || !self.joint_offsets.iter().all(|value| value.is_finite())
            || !self.gravity_vector.iter().all(|value| value.is_finite())
            || self.gravity_vector.iter().map(|v| v * v).sum::<f32>() < 1.0
            || self.gravity_vector.iter().map(|v| v * v).sum::<f32>() > 400.0
        {
            return Err("invalid arm transport, rate, joint offsets or gravity vector".into());
        }
        Ok(())
    }
    fn resolve_paths(&mut self, base: &std::path::Path) {
        if let Some(path) = &mut self.lib_dir {
            if path.is_relative() {
                *path = base.join(&path);
            }
        }
    }
    fn reserved_udp_ports(&self) -> Result<Vec<u16>, String> {
        Ok(if self.use_usb {
            vec![]
        } else {
            vec![self.local_cmd_port, self.local_bcast_port]
        })
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

fn default_gravity_vector() -> [f32; 3] {
    [0.0, 0.0, -9.81]
}
