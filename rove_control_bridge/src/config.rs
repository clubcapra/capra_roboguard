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
    #[serde(default)]
    pub comms: Comms,
}

/// Front-door ports (operator intent from the Steam Deck via udp_multiplexer).
#[derive(Debug, Clone, Deserialize)]
pub struct Comms {
    #[serde(default = "default_teleop_port")]
    pub teleop_port: u16, // RoveControl (teleop) — matches the Python bridge's 5005
    #[serde(default = "default_mission_port")]
    pub mission_port: u16, // Mission proto uploads from the operator
    #[serde(default = "default_estop_port")]
    pub estop_port: u16, // Estop proto (operator e-stop / clear)
    #[serde(default = "default_telemetry_out_port")]
    pub telemetry_out_port: u16, // bridge republishes RoveTelemetry to subscribers here
    #[serde(default = "default_telemetry_out_hz")]
    pub telemetry_out_hz: f64,
    // --- rove_ik_engine (all control-proto motion: tracks + flippers + arm ovis) ---
    #[serde(default = "default_engine_host")]
    pub engine_host: String,
    #[serde(default = "default_engine_port")]
    pub engine_port: u16,
    /// Engine drive-teleop UDP port (flipper steps + drum velocities as JSON).
    #[serde(default = "default_engine_drive_port")]
    pub engine_drive_port: u16,
    /// Tip entity the engine drives for the arm. Empty disables arm forwarding.
    #[serde(default)]
    pub arm_target_entity: String,
}

impl Default for Comms {
    fn default() -> Self {
        Self {
            teleop_port: default_teleop_port(),
            mission_port: default_mission_port(),
            estop_port: default_estop_port(),
            telemetry_out_port: default_telemetry_out_port(),
            telemetry_out_hz: default_telemetry_out_hz(),
            engine_host: default_engine_host(),
            engine_port: default_engine_port(),
            engine_drive_port: default_engine_drive_port(),
            arm_target_entity: String::new(),
        }
    }
}

fn default_telemetry_out_port() -> u16 {
    5053 // clear of rove_sensor_api served block 5000-5017 (5010 = odrive_41 data)
}
fn default_telemetry_out_hz() -> f64 {
    20.0
}

fn default_engine_host() -> String {
    "127.0.0.1".into()
}
fn default_engine_port() -> u16 {
    9100
}
fn default_engine_drive_port() -> u16 {
    9102
}

// Front-door ports stay clear of rove_sensor_api's served block (5000-5017):
// 5005 = gripper cmd, 5006/5007 = odrive_31 data/cmd. Use the 5050+ band.
fn default_teleop_port() -> u16 {
    5050
}
fn default_mission_port() -> u16 {
    5051
}
fn default_estop_port() -> u16 {
    5052
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
    /// Body-frame offset (deg) from VN yaw to the drive-forward axis. The brush
    /// model's forward (track geometry) is ~90deg off the VN yaw on this robot
    /// (calibrated by probe). drive_heading_enu = (90 - yaw_ned) + this.
    pub drive_offset_deg: f64,
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
    /// SAR behaviour: waypoint | goto | patrol | returnhome | orbit | sentinel |
    /// retreat | backtrackcomm | explore. Default "waypoint".
    #[serde(default = "default_behavior")]
    pub behavior: String,
    /// ReturnHome target safe point: origin | last_comms | last_gnss_trusted |
    /// last_stable | last_vision | <operator-label>. Default "origin".
    #[serde(default = "default_return_target")]
    pub return_target: String,
    #[serde(default)]
    pub orbit_center: Option<[f64; 2]>,
    #[serde(default = "default_orbit_radius")]
    pub orbit_radius: f64,
    #[serde(default = "default_orbit_laps")]
    pub orbit_laps: u32,
    #[serde(default = "default_retreat_dist")]
    pub retreat_dist: f64,
    /// Optional explicit Home (SetHome); if absent, origin = first fix.
    #[serde(default)]
    pub home: Option<[f64; 2]>,
}

fn default_behavior() -> String {
    "waypoint".into()
}
fn default_return_target() -> String {
    "origin".into()
}
fn default_orbit_radius() -> f64 {
    4.0
}
fn default_orbit_laps() -> u32 {
    1
}
fn default_retreat_dist() -> f64 {
    5.0
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading config {}", path.display()))?;
        let mut cfg: Config = toml::from_str(&text)
            .with_context(|| format!("parsing config {}", path.display()))?;
        cfg.apply_calibration();
        Ok(cfg)
    }

    /// Overlay calibration.toml (written by the Calibrate mission) over the config.
    /// Per-robot values measured at deploy — re-running the calibrate mission
    /// overwrites the file; this picks it up on the next start.
    fn apply_calibration(&mut self) {
        let Ok(text) = std::fs::read_to_string("calibration.toml") else { return };
        let Ok(v) = text.parse::<toml::Value>() else { return };
        if let Some(o) = v.get("drive_offset_deg").and_then(|x| x.as_float()) {
            // SANITY GATE: the motion-derived drive_offset is still noisy (open-loop
            // curve), and a bad value silently breaks navigation (it once baked a 26°
            // heading error that drove the robot off a cliff). Only override the
            // config if the measurement is within 10° of it; otherwise keep config
            // and shout. (Lift this once the straight-creep calibration lands.)
            let cfg_off = self.goto.drive_offset_deg;
            if (o - cfg_off).abs() <= 10.0 {
                self.goto.drive_offset_deg = o;
                tracing::info!("applied calibration.toml drive_offset_deg={:.1}", o);
            } else {
                tracing::warn!(
                    "IGNORING calibration.toml drive_offset_deg={:.1} (>{:.0}° off config {:.1}) \
                     — calibration looks bad; keeping config. Re-run calibrate or delete the file.",
                    o, 10.0, cfg_off);
            }
        }
        // gyro_bias_z, odom_scale, flipper_zeros are read by their consumers later.
    }
}
