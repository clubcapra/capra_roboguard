//! Sim-backend configuration + the shared, configurable port map.
//!
//! Sim mode is enabled when `config/sim.toml` is present (mirrors the existing
//! per-driver "TOML present ⇒ load it" convention), or via the env override
//! `ROVE_SIM_BACKEND=host:served_base`.
//!
//! Ports come from a single canonical `config/ports.toml` that BOTH this binary
//! and the sim read, so the two sides can't drift into an overlap. Anything
//! unset falls back to the `served_base + 2*idx` / `backend_base + 2*idx` rule,
//! so the common case needs zero per-sensor config.

use std::collections::HashMap;

use serde::Deserialize;

use crate::core::config::load_optional;

/// Per-sensor explicit port pins (to dodge a conflict). All optional.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct PortOverride {
    pub served_data: Option<u16>,
    pub served_cmd: Option<u16>,
    pub backend: Option<u16>,
}

/// `config/ports.toml` — the source of truth shared with the sim.
#[derive(Debug, Clone, Deserialize)]
pub struct PortsFile {
    #[serde(default = "default_served_base")]
    pub served_base: u16,
    #[serde(default = "default_backend_base")]
    pub backend_base: u16,
    #[serde(default = "default_control_port")]
    pub control_port: u16,
    #[serde(default = "default_livox_imu_port")]
    pub livox_imu_port: u16,
    #[serde(default)]
    pub overrides: HashMap<String, PortOverride>,
}

fn default_served_base() -> u16 { 5000 }
fn default_backend_base() -> u16 { 6000 }
fn default_control_port() -> u16 { 5100 }   // clear of the served 5000+ range
fn default_livox_imu_port() -> u16 { 56401 }

impl Default for PortsFile {
    fn default() -> Self {
        Self {
            served_base: default_served_base(),
            backend_base: default_backend_base(),
            control_port: default_control_port(),
            livox_imu_port: default_livox_imu_port(),
            overrides: HashMap::new(),
        }
    }
}

/// `config/sim.toml` — presence toggles sim mode.
#[derive(Debug, Clone, Deserialize)]
struct SimToml {
    #[serde(default = "default_host")]
    host: String,
    #[serde(default = "default_max_vel")]
    odrive_max_vel_rev_s: f64,
    #[serde(default = "default_ports_file")]
    ports_file: String,
}

fn default_host() -> String { "127.0.0.1".to_string() }
fn default_max_vel() -> f64 { 20.0 }
fn default_ports_file() -> String { "ports.toml".to_string() }

/// Resolved sim-backend settings, ready to construct mocks.
#[derive(Debug, Clone)]
pub struct SimConfig {
    pub host: String,
    pub odrive_max_vel_rev_s: f64,
    pub ports: PortsFile,
}

impl SimConfig {
    /// Resolve sim mode from `sim.toml` or the `ROVE_SIM_BACKEND` env var.
    /// Returns `None` when sim mode is off (run against real hardware).
    pub fn resolve() -> Option<Self> {
        // Env override wins: ROVE_SIM_BACKEND=host:served_base
        if let Ok(spec) = std::env::var("ROVE_SIM_BACKEND") {
            let (host, base) = match spec.split_once(':') {
                Some((h, b)) => (h.to_string(), b.parse().unwrap_or(default_served_base())),
                None => (spec, default_served_base()),
            };
            let mut ports = load_ports("ports.toml");
            ports.served_base = base;
            return Some(Self { host, odrive_max_vel_rev_s: default_max_vel(), ports });
        }

        let sim: SimToml = match load_optional::<SimToml>("sim.toml") {
            Ok(Some(s)) => s,
            Ok(None) => return None,
            Err(e) => {
                tracing::warn!(error = %e, "sim.toml load failed — running against hardware");
                return None;
            }
        };
        let ports = load_ports(&sim.ports_file);
        Some(Self { host: sim.host, odrive_max_vel_rev_s: sim.odrive_max_vel_rev_s, ports })
    }

    /// Served data port for the sensor at registration index `idx`
    /// (what autonomy subscribes to). Honors a per-id override.
    pub fn served_data(&self, id: &str, idx: u16) -> u16 {
        self.ports.overrides.get(id).and_then(|o| o.served_data)
            .unwrap_or(self.ports.served_base + idx * 2)
    }

    /// Sim backend port for `id` at index `idx` (what the mock subscribes to).
    pub fn backend(&self, id: &str, idx: u16) -> u16 {
        self.ports.overrides.get(id).and_then(|o| o.backend)
            .unwrap_or(self.ports.backend_base + idx * 2)
    }

    pub fn control_port(&self) -> u16 { self.ports.control_port }
    pub fn livox_imu_port(&self) -> u16 { self.ports.livox_imu_port }
}

fn load_ports(name: &str) -> PortsFile {
    match load_optional::<PortsFile>(name) {
        Ok(Some(p)) => p,
        Ok(None) => {
            tracing::info!(file = name, "no ports file — using default sim port map");
            PortsFile::default()
        }
        Err(e) => {
            tracing::warn!(error = %e, file = name, "ports file parse failed — using defaults");
            PortsFile::default()
        }
    }
}
