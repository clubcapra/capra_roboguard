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
    /// SAR position confidence in [0,1]: ~1 with trusted GNSS, decays while
    /// GNSS-denied (dead-reckoning), recovers on re-fix. Gates missions.
    pub position_confidence: f64,
    /// Estimated position drift (m) accumulated since the last GNSS correction.
    pub drift_m: f64,
    pub received_at: Instant,
}

impl Pose {
    /// ENU heading in radians (CCW from East), derived from the NED yaw.
    pub fn heading_enu_rad(&self) -> f64 {
        (90.0 - self.yaw_ned_deg).to_radians()
    }
}

/// A raw VN fix before filtering — ENU position + the NED velocity used to dead
/// reckon, plus attitude/rate carried straight through.
#[derive(Debug, Clone, Copy)]
pub struct RawFix {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vel_e: f64, // ENU east velocity (= VN vel_east), m/s
    pub vel_n: f64, // ENU north velocity (= VN vel_north), m/s
    pub yaw_ned_deg: f64,
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_rate: f64,
    pub gnss_fix: bool,
}

/// Parse a VectorNav JSON frame into a [`RawFix`] (ENU about the datum).
pub fn parse_vectornav(frame: &serde_json::Value, datum: &Datum) -> Option<RawFix> {
    let lat = frame.get("latitude")?.as_f64()?;
    let lon = frame.get("longitude")?.as_f64()?;
    let yaw = frame.get("yaw")?.as_f64()?;
    let alt = frame.get("altitude").and_then(|v| v.as_f64()).unwrap_or(datum.alt);
    let roll = frame.get("roll").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let pitch = frame.get("pitch").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let gnss_fix = frame.get("gnss_fix").and_then(|v| v.as_bool()).unwrap_or(false);
    let yaw_rate = frame.get("gyro_z").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let vel_e = frame.get("vel_east").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let vel_n = frame.get("vel_north").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let (x, y) = geodetic::geodetic_to_enu(lat, lon, datum.lat, datum.lon);
    Some(RawFix {
        x,
        y,
        z: alt - datum.alt,
        vel_e,
        vel_n,
        yaw_ned_deg: yaw,
        roll_deg: roll,
        pitch_deg: pitch,
        yaw_rate,
        gnss_fix,
    })
}

/// Raw (unfiltered) pose — used by tests and as a fallback.
pub fn pose_from_vectornav(frame: &serde_json::Value, datum: &Datum) -> Option<Pose> {
    let r = parse_vectornav(frame, datum)?;
    Some(Pose {
        x: r.x,
        y: r.y,
        z: r.z,
        yaw_ned_deg: r.yaw_ned_deg,
        roll_deg: r.roll_deg,
        pitch_deg: r.pitch_deg,
        yaw_rate: r.yaw_rate,
        gnss_fix: r.gnss_fix,
        position_confidence: if r.gnss_fix { 1.0 } else { 0.3 },
        drift_m: 0.0,
        received_at: Instant::now(),
    })
}

/// Complementary position filter that HOLDS THROUGH GNSS LOSS. With a trusted
/// fix it dead-reckons from VN velocity (lag-free) and pulls slowly toward GNSS
/// (bounds drift + averages out the ±1 m noise). When GNSS drops it keeps
/// dead-reckoning on velocity + IMU heading alone — so the ENU/lat-lon estimate
/// survives a GNSS-denied stretch and the robot can still navigate home — while
/// `confidence` decays and `drift` grows (the SAR position-quality signal). On
/// re-fix it re-converges and resets drift. (Lidar scan-matching odometry against
/// the persistent cost map is the next layer to bound drift harder.)
pub struct PositionFilter {
    gain: f64,
    fx: f64,
    fy: f64,
    last: Option<Instant>,
    confidence: f64,
    drift_m: f64,
}

/// Confidence decay per second of GNSS denial (→ ~0.0 after ~40 s adrift).
const CONF_DECAY_PER_S: f64 = 0.025;
/// Confidence recovery per second once a trusted fix returns.
const CONF_RECOVER_PER_S: f64 = 0.5;
/// Dead-reckon error grows ≈ this fraction of distance travelled while denied.
const DRIFT_FRAC: f64 = 0.06;

impl PositionFilter {
    pub fn new(gain: f64) -> Self {
        Self { gain, fx: 0.0, fy: 0.0, last: None, confidence: 0.0, drift_m: 0.0 }
    }

    /// Returns (x, y, confidence, drift_m).
    fn update(
        &mut self,
        raw_x: f64,
        raw_y: f64,
        vel_e: f64,
        vel_n: f64,
        gnss_fix: bool,
        now: Instant,
    ) -> (f64, f64, f64, f64) {
        match self.last {
            None => {
                self.fx = raw_x;
                self.fy = raw_y;
                self.confidence = if gnss_fix { 1.0 } else { 0.3 };
            }
            Some(prev) => {
                let dt = now.duration_since(prev).as_secs_f64().clamp(0.0, 0.1);
                // always dead-reckon on velocity (works with or without GNSS)
                self.fx += vel_e * dt;
                self.fy += vel_n * dt;
                if gnss_fix {
                    // trusted fix: correct toward GNSS, recover confidence, reset drift
                    self.fx += self.gain * (raw_x - self.fx);
                    self.fy += self.gain * (raw_y - self.fy);
                    self.confidence = (self.confidence + CONF_RECOVER_PER_S * dt).min(1.0);
                    self.drift_m = 0.0;
                } else {
                    // GNSS-denied: pure dead-reckon; confidence decays, drift grows
                    let step = vel_e.hypot(vel_n) * dt;
                    self.drift_m += step * DRIFT_FRAC;
                    self.confidence = (self.confidence - CONF_DECAY_PER_S * dt).max(0.0);
                }
            }
        }
        self.last = Some(now);
        (self.fx, self.fy, self.confidence, self.drift_m)
    }
}

/// PositionService — owns the datum + filter, turns raw VN frames into a smoothed
/// [`Pose`]. (Slice 1: GNSS + filter. M8 fuses wheel-odo + lidar-odo + GNSS here.)
pub struct PositionService {
    datum: Datum,
    filter: PositionFilter,
}

impl PositionService {
    pub fn new(datum: Datum, correction_gain: f64) -> Self {
        Self { datum, filter: PositionFilter::new(correction_gain) }
    }

    /// Filtered pose from a VN frame. `None` if the frame lacks required fields.
    pub fn update(&mut self, frame: &serde_json::Value, now: Instant) -> Option<Pose> {
        let r = parse_vectornav(frame, &self.datum)?;
        let (x, y, conf, drift) =
            self.filter.update(r.x, r.y, r.vel_e, r.vel_n, r.gnss_fix, now);
        Some(Pose {
            x,
            y,
            z: r.z,
            yaw_ned_deg: r.yaw_ned_deg,
            roll_deg: r.roll_deg,
            pitch_deg: r.pitch_deg,
            yaw_rate: r.yaw_rate,
            gnss_fix: r.gnss_fix,
            position_confidence: conf,
            drift_m: drift,
            received_at: now,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filter_averages_out_white_noise() {
        let mut f = PositionFilter::new(0.05);
        let t0 = Instant::now();
        // Stationary truth at (10, 5) with +/-1 m jitter (no velocity).
        f.update(10.0, 5.0, 0.0, 0.0, true, t0); // seed
        let mut fx = 0.0;
        let jit = [0.9, -1.1, 0.8, -0.7, 1.0, -0.9, 0.6, -0.8, 0.7, -0.5];
        let mut t = t0;
        for (i, j) in jit.iter().cycle().take(400).enumerate() {
            t += std::time::Duration::from_millis(20);
            let (x, ..) = f.update(10.0 + j, 5.0 - j, 0.0, 0.0, true, t);
            if i >= 399 {
                fx = x;
            }
        }
        // Filtered estimate sits near the true mean, far inside the +/-1 m jitter.
        assert!((fx - 10.0).abs() < 0.25, "fx={fx}");
    }

    #[test]
    fn filter_tracks_motion_via_velocity() {
        let mut f = PositionFilter::new(0.05);
        let t0 = Instant::now();
        f.update(0.0, 0.0, 0.0, 0.0, true, t0);
        // Truth moves east at 1 m/s; GNSS = truth + alternating jitter; vel = 1.0.
        // The filter should track the moving truth (dead reckon) while smoothing.
        let mut t = t0;
        let mut truth = 0.0;
        let mut x = 0.0;
        let jit = [0.8, -0.9, 0.7, -1.0, 0.9, -0.6];
        for i in 0..100 {
            t += std::time::Duration::from_millis(20);
            truth += 1.0 * 0.02;
            (x, ..) = f.update(truth + jit[i % jit.len()], 0.0, 1.0, 0.0, true, t);
        }
        // After 2 s truth = 2 m; filtered tracks it, not lagging at 0.
        assert!((x - 2.0).abs() < 0.3, "x={x} should track truth ~2.0");
    }

    #[test]
    fn dead_reckons_and_loses_confidence_when_gnss_denied() {
        let mut f = PositionFilter::new(0.05);
        let t0 = Instant::now();
        f.update(0.0, 0.0, 0.0, 0.0, true, t0);
        // Drive east at 1 m/s for 3 s with NO gnss fix: position must still advance
        // (dead reckon) and confidence must decay below 1.0.
        let mut t = t0;
        let (mut x, mut conf, mut drift) = (0.0, 1.0, 0.0);
        for _ in 0..150 {
            t += std::time::Duration::from_millis(20);
            (x, _, conf, drift) = f.update(999.0, 999.0, 1.0, 0.0, false, t); // bogus GNSS, ignored
        }
        assert!((x - 3.0).abs() < 0.2, "dead-reckoned east ~3 m, got {x}");
        assert!(conf < 1.0 && conf > 0.0, "confidence decayed, got {conf}");
        assert!(drift > 0.0, "drift accrued, got {drift}");
    }
}
