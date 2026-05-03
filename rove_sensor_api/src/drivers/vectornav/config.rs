//! VectorNav VN-300 driver configuration loaded from `config/vectornav.toml`.

use serde::Deserialize;

use crate::core::config::{self, ConfigError};

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VectorNavConfig {
    /// Serial device path. Capra default is `/dev/ttyUSB_VN300` via the udev
    /// rule shipped in `README.md`.
    pub port: String,

    /// Factory default is 115200.
    #[serde(default = "default_baudrate")]
    pub baudrate: u32,
}

impl VectorNavConfig {
    pub fn load() -> Result<Option<Self>, ConfigError> {
        config::load_optional("vectornav.toml")
    }
}

fn default_baudrate() -> u32 {
    115200
}
