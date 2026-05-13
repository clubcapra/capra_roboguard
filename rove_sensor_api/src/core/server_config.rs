//! Server-wide configuration loaded from `config/server.toml`.
//!
//! Per-driver settings live in their own `<driver>.toml` files; this is for
//! settings that apply to the API server itself (UDP push rates, HTTP port,
//! etc.).

use serde::Deserialize;

use crate::core::config;

#[derive(Debug, Clone, Deserialize)]
pub struct ServerConfig {
    /// Default UDP push interval (ms) when a Subscribe packet doesn't carry
    /// its own `interval_ms`. Subscribers can still override per connection.
    #[serde(default = "default_push_interval_ms")]
    pub default_push_interval_ms: u64,
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            default_push_interval_ms: default_push_interval_ms(),
        }
    }
}

fn default_push_interval_ms() -> u64 {
    100
}

/// Load `config/server.toml`. Missing file → defaults; parse error → fatal
/// (caller decides). Unlike per-driver configs, an absent server.toml is
/// expected and silent.
pub fn load() -> Result<ServerConfig, config::ConfigError> {
    Ok(config::load_optional::<ServerConfig>("server.toml")?.unwrap_or_default())
}
