//! PositionService — turns VectorNav telemetry into a local-ENU [`Pose`].
//!
//! Slice 1 is GNSS-nominal: pose comes straight from the INS fix. M8 will fuse
//! SLAM here and hold through GNSS-denied stretches without changing this type.

pub mod geodetic;

use crate::config::Datum;
use std::time::Instant;

/// Robot pose in local ENU about the datum.
///
/// `yaw_ned_deg` is the VectorNav true heading (NED, 0deg = North, 90deg = East,
/// range [-180,180]). Convert to an ENU heading with [`Pose::heading_enu_rad`].
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    /// Height above the datum (m). ~0 on the road; large-negative = fell off.
    pub z: f64,
    pub yaw_ned_deg: f64,
    pub roll_deg: f64,
    pub pitch_deg: f64,
    /// Body yaw rate (VectorNav `gyro_z`, rad/s) — the IMU feedback the heading
    /// asservissement damps on. Positive = CCW (heading-ENU increasing).
    pub yaw_rate: f64,
    pub gnss_fix: bool,
    pub received_at: Instant,
}

impl Pose {
    /// ENU heading in radians (CCW from East), derived from the NED yaw.
    pub fn heading_enu_rad(&self) -> f64 {
        (90.0 - self.yaw_ned_deg).to_radians()
    }
}

/// Parse a VectorNav JSON frame into a [`Pose`]. Returns `None` if the required
/// fields are missing. Marked stale/unfixed via `gnss_fix`.
pub fn pose_from_vectornav(frame: &serde_json::Value, datum: &Datum) -> Option<Pose> {
    let lat = frame.get("latitude")?.as_f64()?;
    let lon = frame.get("longitude")?.as_f64()?;
    let yaw = frame.get("yaw")?.as_f64()?;
    let alt = frame.get("altitude").and_then(|v| v.as_f64()).unwrap_or(datum.alt);
    let roll = frame.get("roll").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let pitch = frame.get("pitch").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let gnss_fix = frame
        .get("gnss_fix")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let yaw_rate = frame.get("gyro_z").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let (x, y) = geodetic::geodetic_to_enu(lat, lon, datum.lat, datum.lon);
    Some(Pose {
        x,
        y,
        z: alt - datum.alt,
        yaw_ned_deg: yaw,
        roll_deg: roll,
        pitch_deg: pitch,
        yaw_rate,
        gnss_fix,
        received_at: Instant::now(),
    })
}
