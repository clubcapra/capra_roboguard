//! Heading asservissement — the INNER closed loop (IMU vs tracks).
//!
//! The outer GoTo loop (GNSS pose) gives a `heading_err`; this PID turns it into
//! a normalised track differential `turn`, servoing on the measured body yaw rate
//! (VectorNav `gyro_z`). The point of the gyro (D) and integral (I) terms is mud
//! rejection: when a track slips the robot starts rotating *before* the slow GNSS
//! loop notices — the D term counters that yaw the instant the IMU sees it, and
//! the I term holds heading against a sustained disturbance (cambered muddy slope).
//!
//! ```text
//! turn = Kp·heading_err + Ki·∫heading_err − Kd·(gyro_sign·yaw_rate)
//! ```
//!
//! `Ki = Kd = 0` degrades gracefully to a plain proportional steerer.

use crate::config::Asserv;

pub struct HeadingController {
    cfg: Asserv,
    integral: f64,
}

impl HeadingController {
    pub fn new(cfg: Asserv) -> Self {
        Self {
            cfg,
            integral: 0.0,
        }
    }

    /// Compute the track differential for this tick.
    /// `heading_err` rad (CCW+), `yaw_rate` rad/s (VectorNav gyro_z), `dt` seconds.
    pub fn update(&mut self, heading_err: f64, yaw_rate: f64, dt: f64) -> f64 {
        // Integrate only near alignment — the I term is for steady slip on a
        // straightaway, not for big turns (where it would wind up and saturate).
        // Beyond the gate, leak it toward zero.
        if heading_err.abs() < self.cfg.i_gate {
            self.integral = (self.integral + heading_err * dt)
                .clamp(-self.cfg.i_clamp, self.cfg.i_clamp);
        } else {
            self.integral *= 0.9;
        }

        let rate = self.cfg.gyro_sign * yaw_rate; // measured CCW+ heading rate
        let turn = self.cfg.kp * heading_err + self.cfg.ki * self.integral - self.cfg.kd * rate;
        turn.clamp(-self.cfg.max_turn, self.cfg.max_turn)
    }

    /// Drop accumulated integral (between waypoints / on hold) to avoid a lurch.
    pub fn reset(&mut self) {
        self.integral = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Asserv {
        Asserv {
            kp: 1.2,
            ki: 0.4,
            kd: 0.3,
            i_gate: 0.20,
            i_clamp: 0.5,
            max_turn: 0.6,
            gyro_sign: 1.0,
        }
    }

    #[test]
    fn proportional_steers_toward_error() {
        let mut hc = HeadingController::new(cfg());
        // Positive heading error (target CCW), no rotation yet => turn CCW (+).
        let turn = hc.update(0.3, 0.0, 0.02);
        assert!(turn > 0.0, "turn={turn}");
    }

    #[test]
    fn gyro_damps_unwanted_rotation() {
        let mut hc = HeadingController::new(cfg());
        // Aligned (err=0) but the robot is yawing CCW (slip): turn should oppose
        // it (negative) to hold heading. This is the mud-rejection term.
        let turn = hc.update(0.0, 0.5, 0.02);
        assert!(turn < 0.0, "turn={turn}");
    }

    #[test]
    fn integral_builds_against_sustained_error() {
        let mut hc = HeadingController::new(cfg());
        let first = hc.update(0.1, 0.0, 0.1);
        let second = hc.update(0.1, 0.0, 0.1);
        assert!(second > first, "integral should grow: {first} -> {second}");
    }

    #[test]
    fn output_is_clamped() {
        let mut hc = HeadingController::new(cfg());
        let turn = hc.update(10.0, 0.0, 0.02);
        assert!((turn - 0.6).abs() < 1e-9, "turn={turn}");
    }
}
