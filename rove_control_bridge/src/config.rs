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
    #[serde(default)]
    pub gnss: Gnss,
    #[serde(default)]
    pub vn: Vn,
}

/// VectorNav mounting. The sim VN is upright (body-up = sensor Z); the real VN is
/// mounted on its side (gravity on +Y) so body-up = sensor Y. This selects the
/// axis used to derive body tilt + yaw rate from the VN accel/gyro.
#[derive(Debug, Clone, Deserialize)]
pub struct Vn {
    /// "x" | "y" | "z" — which VN sensor axis points up on the robot. Default "z".
    #[serde(default = "default_vn_up_axis")]
    pub up_axis: String,
    /// Sign that maps the up-axis gyro to a CCW-positive heading rate, for the
    /// IMU heading integrator. +1 or -1; tune so a CCW turn raises the heading.
    #[serde(default = "default_gyro_yaw_sign")]
    pub gyro_yaw_sign: f64,
}

impl Default for Vn {
    fn default() -> Self {
        Self { up_axis: default_vn_up_axis(), gyro_yaw_sign: default_gyro_yaw_sign() }
    }
}

fn default_vn_up_axis() -> String {
    "z".into()
}
fn default_gyro_yaw_sign() -> f64 {
    1.0
}

/// GNSS source. In the sim the VectorNav frame carries a GNSS fix in-band
/// (`source = "vectornav"`, the default). On the real robot the VN has a working
/// IMU but NO GNSS, so the fix arrives as a UDP JSON broadcast from an external
/// service (`source = "broadcast"`, e.g. `mpu5-gps-restream` → `:7010`); the VN
/// frame is then used for ATTITUDE/IMU only. See `crate::gnss`.
#[derive(Debug, Clone, Deserialize)]
pub struct Gnss {
    /// "vectornav" (fix in the VN telemetry) | "broadcast" (external UDP fix).
    #[serde(default = "default_gnss_source")]
    pub source: String,
    /// UDP port the external GNSS service broadcasts JSON fixes on (broadcast mode).
    #[serde(default = "default_gnss_port")]
    pub port: u16,
    /// A broadcast fix older than this is treated as no-fix (ms).
    #[serde(default = "default_gnss_stale_ms")]
    pub stale_ms: u64,
}

impl Default for Gnss {
    fn default() -> Self {
        Self {
            source: default_gnss_source(),
            port: default_gnss_port(),
            stale_ms: default_gnss_stale_ms(),
        }
    }
}

impl Gnss {
    /// True when position comes from the external UDP broadcast (real robot).
    pub fn is_broadcast(&self) -> bool {
        self.source.eq_ignore_ascii_case("broadcast")
    }
}

fn default_gnss_source() -> String {
    "vectornav".into()
}
fn default_gnss_port() -> u16 {
    7010
}
fn default_gnss_stale_ms() -> u64 {
    1500
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
    /// REST API port — POST /api/v1/goto (missions over HTTP, the primary path).
    #[serde(default = "default_mission_http_port")]
    pub mission_http_port: u16,
    // --- rove_ik_engine (all control-proto motion: tracks + flippers + arm ovis) ---
    #[serde(default = "default_engine_host")]
    pub engine_host: String,
    #[serde(default = "default_engine_port")]
    pub engine_port: u16,
    /// Engine drive-teleop UDP port (flipper steps + drum velocities as JSON).
    #[serde(default = "default_engine_drive_port")]
    pub engine_drive_port: u16,
    /// Engine HTTP port (named pose-goto relay: POST /api/v1/poses/goto).
    #[serde(default = "default_engine_http_port")]
    pub engine_http_port: u16,
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
            mission_http_port: default_mission_http_port(),
            engine_host: default_engine_host(),
            engine_port: default_engine_port(),
            engine_drive_port: default_engine_drive_port(),
            engine_http_port: default_engine_http_port(),
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
fn default_mission_http_port() -> u16 {
    8088 // bridge REST API (clear of rove_sensor_api :8080 and the engine :9101)
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
fn default_engine_http_port() -> u16 {
    9101
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
#[derive(Debug, Clone, Deserialize)]
pub struct Perception {
    pub lidar_port: u16,         // sim LVX2 data port (source = "lvxsub")
    pub obstacle_stop_m: f64,    // hold if a solid object is within this
    pub cliff_stop_m: f64,       // hold if a ground edge/hole is within this
    pub min_ground_per_bin: u32, // ground-return floor per range bin (below => drop-off)
    /// "lvxsub" (sim world-frame LVX2) | "lvxr" (real Mid-360 via the livox_bridge
    /// sidecar — IMU-leveled proximity reflex). Default "lvxsub".
    #[serde(default = "default_perception_source")]
    pub source: String,
    /// livox_bridge LVXR ports (source = "lvxr"): points + IMU.
    #[serde(default = "default_lvxr_pts_port")]
    pub lvxr_pts_port: u16,
    #[serde(default = "default_lvxr_imu_port")]
    pub lvxr_imu_port: u16,
    /// Self-mask radius (m): ignore returns within this of the lidar (the robot's
    /// own cage/chassis). Circular at the cage's max extent. lvxr mode only.
    #[serde(default = "default_self_radius_m")]
    pub self_radius_m: f64,
    /// Forward arc for the reflex (lvxr mode): only stop for obstacles/edges whose
    /// bearing is within `fwd_arc_deg` of `fwd_offset_deg` in the LIDAR leveled
    /// frame. `fwd_arc_deg >= 180` = omnidirectional. `fwd_offset_deg` is the
    /// lidar-frame bearing of the robot's drive-forward (TUNE on the robot).
    #[serde(default = "default_fwd_offset_deg")]
    pub fwd_offset_deg: f64,
    #[serde(default = "default_fwd_arc_deg")]
    pub fwd_arc_deg: f64,
    /// If true (default, SAFE), autonomy holds when there's no fresh lidar (sidecar
    /// down / lidar lost) — never drives blind. Set false to allow blind driving.
    #[serde(default = "default_require_lidar")]
    pub require_lidar: bool,
}

fn default_require_lidar() -> bool {
    true
}

fn default_self_radius_m() -> f64 {
    1.0
}
fn default_fwd_offset_deg() -> f64 {
    0.0
}
fn default_fwd_arc_deg() -> f64 {
    180.0 // omnidirectional until the forward offset is calibrated (safe default)
}

fn default_perception_source() -> String {
    "lvxsub".into()
}
fn default_lvxr_pts_port() -> u16 {
    7020
}
fn default_lvxr_imu_port() -> u16 {
    7021
}

/// L0 reflex limits — hard safety bounds that gate all driving (`reflex`).
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Reflex {
    pub geofence_radius_m: f64, // max distance from home before estop
    pub fall_floor_m: f64,      // estop if height drops this far below datum
    pub max_roll_deg: f64,      // (legacy/logging — raw VN roll is unreliable when mounted rotated)
    pub max_pitch_deg: f64,     // (legacy/logging)
    /// Body tilt from vertical (deg) that trips the attitude reflex. This is the
    /// real check now — frame-honest, from the VN accel (see Pose::tilt_deg).
    #[serde(default = "default_max_tilt_deg")]
    pub max_tilt_deg: f64,
}

fn default_max_tilt_deg() -> f64 {
    35.0
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
