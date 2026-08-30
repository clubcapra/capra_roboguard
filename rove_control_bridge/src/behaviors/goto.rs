//! GoTo — the OUTER loop: steer the robot toward an ENU waypoint (GNSS pose).
//!
//! It emits a desired `forward` speed and a `heading_err` (radians); the INNER
//! IMU heading controller (`control::heading`) turns that error into the track
//! differential, servoing on measured yaw rate so a slipping track is corrected
//! immediately (mud rejection). Conservative by design: skid steer on these
//! tracks is slow and slippy (see rove_sim locomotion notes).

use crate::config::Goto;
use crate::position::Pose;
use std::f64::consts::PI;

/// Forward speed is fully allowed within this heading error (rad, ~9deg)...
const PIVOT_FULL: f64 = 0.15;
/// ...and reduced (not zeroed) beyond this (rad, ~46deg). This robot CANNOT pivot
/// in place — a pure pivot stalls in the brush model's static-grip regime (the
/// tracks never scrub). It must keep moving so the forward motion engages the
/// lateral scrub; so we keep a forward FLOOR and let it arc-turn toward the target.
const PIVOT_ZERO: f64 = 0.80;
/// Minimum forward fraction even at large heading error. HIGH on purpose: the
/// robot must keep real longitudinal speed to break the static-grip regime and
/// scrub-turn (a near-stopped pivot stalls). The control loop also arc-clamps the
/// differential so the inner track never deep-reverses into a stalling pivot.
const FWD_FLOOR: f64 = 0.85;

/// A waypoint in local ENU metres.
#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Step {
    /// Keep driving: normalised `forward` speed + signed `heading_err` (rad,
    /// positive = target is CCW of current heading). `dist` is metres remaining.
    Drive {
        forward: f64,
        heading_err: f64,
        dist: f64,
    },
    /// Within tolerance of the waypoint.
    Arrived,
}

/// Wrap an angle (radians) to (-pi, pi].
fn wrap_pi(a: f64) -> f64 {
    let mut a = a % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a <= -PI {
        a += 2.0 * PI;
    }
    a
}

/// Outer-loop drive params toward an arbitrary world target (no arrival check) --
/// used to follow the cost-map planner's local target. Returns (forward, heading_err).
pub fn step_to(pose: &Pose, target: [f64; 2], g: &Goto) -> (f64, f64) {
    let dx = target[0] - pose.x;
    let dy = target[1] - pose.y;
    let dist = dx.hypot(dy);
    let bearing = dy.atan2(dx);
    let drive_heading = pose.heading_enu_rad() + g.drive_offset_deg.to_radians();
    let heading_err = wrap_pi(bearing - drive_heading);
    let align = ((PIVOT_ZERO - heading_err.abs()) / (PIVOT_ZERO - PIVOT_FULL))
        .clamp(0.0, 1.0)
        .max(FWD_FLOOR);
    let forward = (g.k_v * dist).clamp(0.0, g.v_max) * align;
    (forward, heading_err)
}

/// One outer-loop step toward `wp` from `pose`.
pub fn step(pose: &Pose, wp: &Waypoint, g: &Goto) -> Step {
    let dx = wp.x - pose.x;
    let dy = wp.y - pose.y;
    let dist = dx.hypot(dy);
    if dist < g.arrive_tol_m {
        return Step::Arrived;
    }

    let bearing = dy.atan2(dx); // ENU, CCW from East
    // Steer the DRIVE-forward axis (VN yaw + body offset), not the VN yaw itself.
    let drive_heading = pose.heading_enu_rad() + g.drive_offset_deg.to_radians();
    let heading_err = wrap_pi(bearing - drive_heading);

    // Reduce forward as heading error grows (full within PIVOT_FULL), but never
    // below FWD_FLOOR -- this robot must keep moving to scrub-turn (it can't pivot
    // in place). So it arc-turns toward the target instead of stalling.
    let align = ((PIVOT_ZERO - heading_err.abs()) / (PIVOT_ZERO - PIVOT_FULL))
        .clamp(0.0, 1.0)
        .max(FWD_FLOOR);
    let forward = (g.k_v * dist).clamp(0.0, g.v_max) * align;

    Step::Drive {
        forward,
        heading_err,
        dist,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;
    use std::time::Instant;

    fn pose(x: f64, y: f64, yaw_ned: f64) -> Pose {
        Pose {
            x,
            y,
            z: 0.0,
            yaw_ned_deg: yaw_ned,
            roll_deg: 0.0,
            pitch_deg: 0.0,
            tilt_deg: 0.0,
            yaw_rate: 0.0,
            gnss_fix: true,
            position_confidence: 1.0,
            drift_m: 0.0,
            received_at: Instant::now(),
        }
    }

    fn gains() -> Goto {
        Goto {
            arrive_tol_m: 0.6,
            k_v: 0.35,
            v_max: 0.45,
            drive_offset_deg: 0.0, // tests check raw VN-yaw geometry
        }
    }

    #[test]
    fn arrives_within_tolerance() {
        let p = pose(0.0, 0.0, 0.0);
        let wp = Waypoint { x: 0.3, y: 0.0 };
        assert_eq!(step(&p, &wp, &gains()), Step::Arrived);
    }

    #[test]
    fn aligned_drives_straight() {
        // yaw_ned = 90 => ENU heading 0 (facing East). Waypoint due East.
        let p = pose(0.0, 0.0, 90.0);
        let wp = Waypoint { x: 5.0, y: 0.0 };
        match step(&p, &wp, &gains()) {
            Step::Drive {
                forward,
                heading_err,
                ..
            } => {
                assert!(heading_err.abs() < 1e-9, "should be aligned");
                assert!(forward > 0.0, "should move forward");
            }
            _ => panic!("expected Drive"),
        }
    }

    #[test]
    fn heading_error_sign_is_ccw_positive() {
        // Facing East (yaw_ned=90), waypoint due North => target bearing +pi/2,
        // which is CCW of current heading => positive heading_err.
        let p = pose(0.0, 0.0, 90.0);
        let wp = Waypoint { x: 0.0, y: 5.0 };
        match step(&p, &wp, &gains()) {
            Step::Drive { heading_err, .. } => {
                assert!((heading_err - FRAC_PI_2).abs() < 1e-9, "err={heading_err}")
            }
            _ => panic!("expected Drive"),
        }
    }
}
