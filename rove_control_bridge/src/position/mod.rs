//! PositionService — turns VectorNav telemetry into a local-ENU [`Pose`].
//!
//! Slice 1 is GNSS-nominal: pose comes straight from the INS fix. M8 will fuse
//! SLAM here and hold through GNSS-denied stretches without changing this type.

pub mod geodetic;

use crate::config::Datum;
use crate::gnss::GnssFix;
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
    /// Body tilt from vertical (deg), derived from the VN accelerometer vs the
    /// configured body-up axis. Frame-honest (works even though the VN is mounted
    /// rotated, reading roll≈-90° when level). This — not the raw roll/pitch — is
    /// what the level gate + attitude reflex use.
    pub tilt_deg: f64,
    /// Body yaw rate (rad/s) about the body-up axis — the IMU feedback the heading
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
    /// Compensated linear acceleration (m/s², VN body frame). Used to derive the
    /// body tilt independent of the VN mount orientation.
    pub accel: [f64; 3],
    /// Angular rate (rad/s, VN body frame) — x,y,z.
    pub gyro: [f64; 3],
    pub gnss_fix: bool,
}

/// Body-up axis in the VN sensor frame. The sim VN is `Z`; the real VN is mounted
/// on its side (gravity on +Y) so it's `Y`. Picks the tilt reference + yaw-rate axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpAxis {
    X,
    Y,
    Z,
}

impl UpAxis {
    pub fn parse(s: &str) -> Self {
        match s.trim().to_ascii_lowercase().as_str() {
            "x" => UpAxis::X,
            "y" => UpAxis::Y,
            _ => UpAxis::Z,
        }
    }
    fn idx(self) -> usize {
        match self {
            UpAxis::X => 0,
            UpAxis::Y => 1,
            UpAxis::Z => 2,
        }
    }
}

/// Body tilt from vertical (deg) and yaw rate (rad/s about up) from the VN
/// accel + gyro, given which VN axis is body-up. Tilt = angle between the
/// (gravity-reaction) accel and the up axis; yaw rate = gyro about up.
fn tilt_and_yaw_rate(accel: [f64; 3], gyro: [f64; 3], up: UpAxis) -> (f64, f64) {
    let n = (accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]).sqrt();
    let i = up.idx();
    let tilt = if n > 1e-6 {
        (accel[i] / n).clamp(-1.0, 1.0).acos().to_degrees()
    } else {
        0.0
    };
    (tilt, gyro[i])
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
    let getf = |k: &str| frame.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0);
    let accel = [getf("accel_x"), getf("accel_y"), getf("accel_z")];
    let gyro = [getf("gyro_x"), getf("gyro_y"), getf("gyro_z")];
    let vel_e = getf("vel_east");
    let vel_n = getf("vel_north");
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
        yaw_rate: gyro[2], // overridden per up-axis in PositionService
        accel,
        gyro,
        gnss_fix,
    })
}

/// Raw (unfiltered) pose — used by tests and as a fallback.
pub fn pose_from_vectornav(frame: &serde_json::Value, datum: &Datum) -> Option<Pose> {
    let r = parse_vectornav(frame, datum)?;
    let (tilt_deg, yaw_rate) = tilt_and_yaw_rate(r.accel, r.gyro, UpAxis::Z);
    Some(Pose {
        x: r.x,
        y: r.y,
        z: r.z,
        yaw_ned_deg: r.yaw_ned_deg,
        roll_deg: r.roll_deg,
        pitch_deg: r.pitch_deg,
        tilt_deg,
        yaw_rate,
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
/// Min ground speed (m/s) to trust GNSS course-over-ground as a heading reference.
const COURSE_MIN_SPEED: f64 = 0.4;
/// Per-correction pull of the gyro-integrated heading toward the GNSS course.
const COURSE_GAIN: f64 = 0.1;

fn wrap_pi(a: f64) -> f64 {
    let mut a = a % (2.0 * std::f64::consts::PI);
    if a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    } else if a <= -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

pub struct PositionService {
    datum: Datum,
    filter: PositionFilter,
    /// Which VN axis is body-up (the VN is mounted rotated on the real robot).
    up_axis: UpAxis,
    /// Sign mapping the up-axis gyro to CCW-positive heading rate.
    gyro_yaw_sign: f64,
    /// Fused drive-forward heading (ENU rad): gyro-integrated, GNSS-course-corrected.
    /// The side-mounted VN's own yaw isn't the body heading, so we build it here.
    heading_rad: f64,
    heading_seeded: bool,
    last_heading_t: Option<Instant>,
    /// Last height (m above datum), from the GNSS fix. Held through GNSS gaps so
    /// the fall reflex keeps a value (frozen, not zeroed) while denied.
    last_z: f64,
    /// Set once the first external GNSS fix has seeded the filter (broadcast mode).
    seeded: bool,
}

impl PositionService {
    pub fn new(datum: Datum, correction_gain: f64, up_axis: UpAxis, gyro_yaw_sign: f64) -> Self {
        Self {
            datum,
            filter: PositionFilter::new(correction_gain),
            up_axis,
            gyro_yaw_sign,
            heading_rad: 0.0,
            heading_seeded: false,
            last_heading_t: None,
            last_z: 0.0,
            seeded: false,
        }
    }

    /// Fuse the drive-forward heading: integrate the up-axis gyro, and correct
    /// toward the GNSS course-over-ground when moving fast enough to trust it.
    /// Returns the heading (ENU rad). Mount-independent (doesn't use VN yaw).
    fn fuse_heading(&mut self, yaw_rate: f64, gnss: Option<&GnssFix>, now: Instant) -> f64 {
        let dt = self
            .last_heading_t
            .map(|t| now.duration_since(t).as_secs_f64().clamp(0.0, 0.2))
            .unwrap_or(0.0);
        self.last_heading_t = Some(now);
        self.heading_rad = wrap_pi(self.heading_rad + self.gyro_yaw_sign * yaw_rate * dt);
        if let Some(g) = gnss {
            if g.speed_ms > COURSE_MIN_SPEED {
                let course_enu = (90.0 - g.track_deg).to_radians(); // NED->ENU
                if !self.heading_seeded {
                    self.heading_rad = course_enu;
                    self.heading_seeded = true;
                } else {
                    self.heading_rad =
                        wrap_pi(self.heading_rad + COURSE_GAIN * wrap_pi(course_enu - self.heading_rad));
                }
            }
        }
        self.heading_rad
    }

    /// Filtered pose from a VN frame (sim / `source = "vectornav"`): position,
    /// attitude and GNSS all come from the VN. `None` if the frame lacks fields.
    pub fn update(&mut self, frame: &serde_json::Value, now: Instant) -> Option<Pose> {
        let r = parse_vectornav(frame, &self.datum)?;
        let (tilt_deg, yaw_rate) = tilt_and_yaw_rate(r.accel, r.gyro, self.up_axis);
        let (x, y, conf, drift) =
            self.filter.update(r.x, r.y, r.vel_e, r.vel_n, r.gnss_fix, now);
        Some(Pose {
            x,
            y,
            z: r.z,
            yaw_ned_deg: r.yaw_ned_deg,
            roll_deg: r.roll_deg,
            pitch_deg: r.pitch_deg,
            tilt_deg,
            yaw_rate,
            gnss_fix: r.gnss_fix,
            position_confidence: conf,
            drift_m: drift,
            received_at: now,
        })
    }

    /// Fused pose for the REAL robot (`source = "broadcast"`): ATTITUDE/IMU from
    /// the VN frame (yaw/roll/pitch/gyro_z — the VN's own GNSS fields are ignored)
    /// and POSITION from the external GNSS broadcast (`gnss` = the freshest fix,
    /// or `None` when stale / not yet received).
    ///
    /// With a fresh fix the filter corrects toward it and dead-reckons on the
    /// GNSS-derived velocity (speed + course). Without one it holds: position
    /// freezes (no wheel odometry yet, so velocity is unknown → zero), confidence
    /// decays and drift grows — the SAR position-quality signal. Returns `None`
    /// only if the VN frame lacks attitude.
    pub fn update_with_gnss(
        &mut self,
        frame: &serde_json::Value,
        gnss: Option<&GnssFix>,
        now: Instant,
    ) -> Option<Pose> {
        let r = parse_vectornav(frame, &self.datum)?; // attitude + IMU only
        let (tilt_deg, yaw_rate) = tilt_and_yaw_rate(r.accel, r.gyro, self.up_axis);
        // Fused drive-forward heading (gyro + GNSS course) — the VN's own yaw is
        // unusable (side mount). Injected via yaw_ned_deg so GoTo/perception (which
        // read heading_enu_rad) use it with drive_offset_deg = 0.
        let heading = self.fuse_heading(yaw_rate, gnss, now);
        let yaw_ned_deg = 90.0 - heading.to_degrees();
        let (x, y, conf, drift, fix) = match gnss {
            Some(g) => {
                let (rx, ry) =
                    geodetic::geodetic_to_enu(g.lat, g.lon, self.datum.lat, self.datum.lon);
                // course over ground (deg CW from North) -> ENU velocity
                let track = g.track_deg.to_radians();
                let (vel_e, vel_n) = (g.speed_ms * track.sin(), g.speed_ms * track.cos());
                let (fx, fy, c, d) = self.filter.update(rx, ry, vel_e, vel_n, true, now);
                self.last_z = g.alt_msl - self.datum.alt;
                self.seeded = true;
                (fx, fy, c, d, true)
            }
            None if self.seeded => {
                // No fresh fix: hold. Velocity is unknown without odometry, so
                // dead-reckon with zero velocity — position freezes, confidence
                // decays, drift grows.
                let (fx, fy, c, d) = self.filter.update(0.0, 0.0, 0.0, 0.0, false, now);
                (fx, fy, c, d, false)
            }
            // Attitude-only until the very first fix (don't seed the filter at the
            // origin — that would make the first real fix crawl in over many ticks).
            None => (0.0, 0.0, 0.0, 0.0, false),
        };
        Some(Pose {
            x,
            y,
            z: self.last_z,
            yaw_ned_deg, // fused heading (NED), not the raw VN yaw
            roll_deg: r.roll_deg,
            pitch_deg: r.pitch_deg,
            tilt_deg,
            yaw_rate,
            gnss_fix: fix,
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

    fn vn_frame(yaw: f64, roll: f64, pitch: f64, gyro_z: f64) -> serde_json::Value {
        // A real VN frame: valid attitude/IMU, but its GNSS fields are junk (no
        // GNSS on the real unit) — update_with_gnss must ignore them.
        serde_json::json!({
            "latitude": 0.0, "longitude": 0.0, "altitude": 0.0, "gnss_fix": false,
            "yaw": yaw, "roll": roll, "pitch": pitch,
            "gyro_z": gyro_z, "vel_east": 0.0, "vel_north": 0.0,
        })
    }

    #[test]
    fn fused_pose_takes_attitude_from_vn_and_position_from_gnss() {
        let datum = Datum { lat: 46.7512, lon: 7.6131, alt: 560.0 };
        let mut svc = PositionService::new(datum, 0.05, UpAxis::Z, 1.0);
        let t = Instant::now();

        // Before any fix: attitude flows, position is unknown (no fix). yaw_ned now
        // carries the FUSED heading (not the raw VN yaw), so don't assert passthrough.
        let p0 = svc
            .update_with_gnss(&vn_frame(12.0, 3.0, -4.0, 0.1), None, t)
            .expect("attitude-only pose");
        assert!(!p0.gnss_fix, "no fix yet");
        assert!((p0.roll_deg - 3.0).abs() < 1e-6 && (p0.pitch_deg + 4.0).abs() < 1e-6);

        // First fix at the datum -> ENU ~ (0,0), height = alt_msl - datum.alt.
        let fix = GnssFix {
            lat: 46.7512, lon: 7.6131, alt_msl: 562.0,
            speed_ms: 0.0, track_deg: 0.0, accuracy_m: 1.0,
            received_at: t,
        };
        let p1 = svc
            .update_with_gnss(&vn_frame(12.0, 0.0, 0.0, 0.0), Some(&fix), t)
            .expect("fused pose");
        assert!(p1.gnss_fix, "have a fix");
        assert!(p1.x.abs() < 0.5 && p1.y.abs() < 0.5, "at datum -> ~origin ({},{})", p1.x, p1.y);
        assert!((p1.z - 2.0).abs() < 1e-6, "z = 562 - 560 = 2 m, got {}", p1.z);
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
