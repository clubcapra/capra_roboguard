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

/// Consecutive ticks below the fall floor before the fall reflex trips. GNSS
/// altitude is very noisy (z swings ±10 m), so a single low sample is NOT a fall;
/// require it sustained (~1 s @ 50 Hz). The real edge protection is the lidar
/// cliff reflex — this is a coarse "drove right off the map" backstop.
const FALL_DEBOUNCE_TICKS: u32 = 50;

pub struct ReflexEngine {
    cfg: ReflexCfg,
    home: (f64, f64),
    tripped: Option<Trip>,
    fall_ticks: u32,
}

impl ReflexEngine {
    pub fn new(cfg: ReflexCfg, home: (f64, f64)) -> Self {
        Self {
            cfg,
            home,
            tripped: None,
            fall_ticks: 0,
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
        // debounce the fall check against noisy GNSS altitude
        if pose.z < -self.cfg.fall_floor_m {
            self.fall_ticks += 1;
        } else {
            self.fall_ticks = 0;
        }
        if let Some(reason) = self.evaluate(pose) {
            tracing::error!("REFLEX TRIP — {reason} — estop");
            self.tripped = Some(Trip { reason });
        }
        self.tripped.as_ref()
    }

    fn evaluate(&self, pose: &Pose) -> Option<String> {
        if self.fall_ticks >= FALL_DEBOUNCE_TICKS {
            return Some(format!(
                "fall: height {:.1} m below datum (sustained {} ticks)",
                pose.z, self.fall_ticks
            ));
        }
        let d = (pose.x - self.home.0).hypot(pose.y - self.home.1);
        if d > self.cfg.geofence_radius_m {
            return Some(format!(
                "geofence: {:.1} m from home (limit {:.1})",
                d, self.cfg.geofence_radius_m
            ));
        }
        // Body tilt from vertical (from the VN accel) — frame-honest, unlike the
        // raw VN roll/pitch which read ~90° because the VN is mounted rotated.
        if pose.tilt_deg > self.cfg.max_tilt_deg {
            return Some(format!(
                "attitude: tilt {:.0} deg (limit {:.0})",
                pose.tilt_deg, self.cfg.max_tilt_deg
            ));
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
            max_tilt_deg: 45.0,
        }
    }

    fn pose_at(x: f64, y: f64, z: f64, tilt: f64) -> Pose {
        Pose {
            x,
            y,
            z,
            yaw_ned_deg: 0.0,
            roll_deg: 0.0,
            pitch_deg: 0.0,
            tilt_deg: tilt,
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
        assert!(r.check(&pose_at(3.0, 4.0, 0.0, 3.0)).is_none());
    }

    #[test]
    fn geofence_trips_and_latches() {
        let mut r = ReflexEngine::new(cfg(), (0.0, 0.0));
        assert!(r.check(&pose_at(20.0, 0.0, 0.0, 0.0)).is_some());
        // Latches even after returning inside the fence.
        assert!(r.check(&pose_at(0.0, 0.0, 0.0, 0.0)).is_some());
    }

    #[test]
    fn fall_trips_only_when_sustained() {
        let mut r = ReflexEngine::new(cfg(), (0.0, 0.0));
        // a single low-z sample is NOT a fall (GNSS-altitude noise)
        assert!(r.check(&pose_at(0.0, 0.0, -50.0, 0.0)).is_none());
        // sustained low z does trip
        for _ in 0..super::FALL_DEBOUNCE_TICKS + 1 {
            r.check(&pose_at(0.0, 0.0, -50.0, 0.0));
        }
        assert!(r.check(&pose_at(0.0, 0.0, -50.0, 0.0)).is_some());
    }

    #[test]
    fn tilt_trips() {
        let mut r = ReflexEngine::new(cfg(), (0.0, 0.0));
        assert!(r.check(&pose_at(0.0, 0.0, 0.0, 60.0)).is_some());
    }
}
