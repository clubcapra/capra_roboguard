//! Engine configuration, loaded from `config/autonomy.toml`.

use anyhow::{Context, Result};
use serde::Deserialize;
use std::path::Path;

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub robot_host: String,
    pub http_port: u16,
    pub datum: Datum,
    pub telemetry: Telemetry,
    pub position: Position,
    pub tracks: Tracks,
    pub control: Control,
    pub goto: Goto,
    pub asserv: Asserv,
    pub reflex: Reflex,
    pub perception: Perception,
    pub mission: Mission,
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Datum {
    pub lat: f64,
    pub lon: f64,
    #[allow(dead_code)]
    pub alt: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Telemetry {
    pub vectornav_id: String,
    pub subscribe_ms: u32,
    pub pose_stale_ms: u64,
}

/// PositionService filter — smooths the noisy VN GNSS into a usable pose.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Position {
    /// Per-sample pull of the dead-reckoned position toward raw GNSS [0..1].
    /// Small = smoother (more lag-free dead reckoning), large = trusts GNSS more.
    pub correction_gain: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Tracks {
    pub left_nodes: Vec<u32>,
    pub right_nodes: Vec<u32>,
    pub invert_left: bool,
    pub invert_right: bool,
    pub max_velocity: f64,
    pub slew_per_s: f64, // max change in normalised command per second (ramp)
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Control {
    pub rate_hz: f64,
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Goto {
    pub arrive_tol_m: f64,
    pub k_v: f64,
    pub v_max: f64,
}

/// Inner heading asservissement (IMU vs tracks) — see `control::heading`.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Asserv {
    pub kp: f64,       // per rad of heading error
    pub ki: f64,       // per rad·s of accumulated error
    pub kd: f64,       // per rad/s of measured yaw rate (gyro damping)
    pub i_gate: f64,   // only integrate when |heading_err| < this (rad)
    pub i_clamp: f64,  // anti-windup bound on the integral term
    pub max_turn: f64, // clamp on the normalised differential
    pub gyro_sign: f64, // +1/-1 to align gyro_z with CCW-positive heading rate
}

/// Lidar forward-hazard reflex — stops the robot at obstacles / drop-offs.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Perception {
    pub lidar_port: u16,         // bottom Livox (near sensing) data port
    pub obstacle_stop_m: f64,    // hold if a solid object is within this, ahead
    pub cliff_stop_m: f64,       // hold if the ground edge is within this, ahead
    pub min_ground_per_bin: u32, // ground-return floor per range bin (below => drop-off)
}

/// L0 reflex limits — hard safety bounds that gate all driving (`reflex`).
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Reflex {
    pub geofence_radius_m: f64, // max distance from home before estop
    pub fall_floor_m: f64,      // estop if height drops this far below datum
    pub max_roll_deg: f64,
    pub max_pitch_deg: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Mission {
    pub relative_to_start: bool,
    pub demo_forward_m: f64,
    pub waypoints: Vec<[f64; 2]>,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading config {}", path.display()))?;
        let cfg: Config = toml::from_str(&text)
            .with_context(|| format!("parsing config {}", path.display()))?;
        Ok(cfg)
    }
}
