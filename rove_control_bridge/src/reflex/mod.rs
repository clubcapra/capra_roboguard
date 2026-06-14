//! L0 ReflexEngine — the safety layer that gates ALL driving (roadmap M7).
//!
//! It runs every control tick *before* the behaviour and can veto motion. These
//! are hard, fast, behaviour-independent limits — the kind of thing that, had it
//! existed, would have stopped the robot before it spiralled off the map edge.
//!
//! Slice-1 reflexes are pose-derived (cheap, always available):
//!   * **geofence** — never travel more than `geofence_radius_m` from home,
//!   * **fall**     — height below `-fall_floor_m` (drove off the terrain shell),
//!   * **attitude** — |roll|/|pitch| beyond limits (rolled over / on a cliff).
//!
//! Future (need telemetry/lidar): over-current / thermal / stuck (ODrive), and
//! imminent-collision (lidar) — same trip→latch→estop contract.

use crate::config::Reflex as ReflexCfg;
use crate::position::Pose;

/// A latched safety trip. Once tripped the engine stays tripped — recovery is a
/// deliberate human/mission action, never automatic.
#[derive(Debug, Clone)]
pub struct Trip {
    pub reason: String,
}

pub struct ReflexEngine {
    cfg: ReflexCfg,
    home: (f64, f64),
    tripped: Option<Trip>,
}

impl ReflexEngine {
    pub fn new(cfg: ReflexCfg, home: (f64, f64)) -> Self {
        Self {
            cfg,
            home,
            tripped: None,
        }
    }

    /// Already latched?
    #[allow(dead_code)] // status surface for teleop/mission layers (M9)
    pub fn tripped(&self) -> Option<&Trip> {
        self.tripped.as_ref()
    }

    /// Evaluate this tick. Returns `Some(Trip)` the first time a limit is
    /// breached (and on every tick thereafter, since the trip latches).
    pub fn check(&mut self, pose: &Pose) -> Option<&Trip> {
        if self.tripped.is_some() {
            return self.tripped.as_ref();
        }
        if let Some(reason) = self.evaluate(pose) {
            tracing::error!("REFLEX TRIP — {reason} — estop");
            self.tripped = Some(Trip { reason });
        }
        self.tripped.as_ref()
    }

    fn evaluate(&self, pose: &Pose) -> Option<String> {
        if pose.z < -self.cfg.fall_floor_m {
            return Some(format!("fall: height {:.1} m below datum", pose.z));
        }
        let d = (pose.x - self.home.0).hypot(pose.y - self.home.1);
        if d > self.cfg.geofence_radius_m {
            return Some(format!(
                "geofence: {:.1} m from home (limit {:.1})",
                d, self.cfg.geofence_radius_m
            ));
        }
        if pose.roll_deg.abs() > self.cfg.max_roll_deg {
            return Some(format!("attitude: roll {:.0} deg", pose.roll_deg));
        }
        if pose.pitch_deg.abs() > self.cfg.max_pitch_deg {
            return Some(format!("attitude: pitch {:.0} deg", pose.pitch_deg));
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    fn cfg() -> ReflexCfg {
        ReflexCfg {
            geofence_radius_m: 15.0,
            fall_floor_m: 5.0,
            max_roll_deg: 45.0,
            max_pitch_deg: 45.0,
        }
    }

    fn pose_at(x: f64, y: f64, z: f64, roll: f64, pitch: f64) -> Pose {
        Pose {
            x,
            y,
            z,
            yaw_ned_deg: 0.0,
            roll_deg: roll,
            pitch_deg: pitch,
            yaw_rate: 0.0,
            gnss_fix: true,
            position_confidence: 1.0,
            drift_m: 0.0,
            received_at: Instant::now(),
        }
    }

    #[test]
    fn nominal_pose_is_ok() {
        let mut r = ReflexEngine::new(cfg(), (0.0, 0.0));
        assert!(r.check(&pose_at(3.0, 4.0, 0.0, 2.0, 3.0)).is_none());
    }

    #[test]
    fn geofence_trips_and_latches() {
        let mut r = ReflexEngine::new(cfg(), (0.0, 0.0));
        assert!(r.check(&pose_at(20.0, 0.0, 0.0, 0.0, 0.0)).is_some());
        // Latches even after returning inside the fence.
        assert!(r.check(&pose_at(0.0, 0.0, 0.0, 0.0, 0.0)).is_some());
    }

    #[test]
    fn fall_trips() {
        let mut r = ReflexEngine::new(cfg(), (0.0, 0.0));
        assert!(r.check(&pose_at(0.0, 0.0, -50.0, 0.0, 0.0)).is_some());
    }

    #[test]
    fn rollover_trips() {
        let mut r = ReflexEngine::new(cfg(), (0.0, 0.0));
        assert!(r.check(&pose_at(0.0, 0.0, 0.0, 60.0, 0.0)).is_some());
    }
}
