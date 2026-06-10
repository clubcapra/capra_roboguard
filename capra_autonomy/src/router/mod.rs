//! Command Router + mode state machine (the M6 seed).
//!
//! Slice 1 modes: `Idle` (do nothing), `Auto` (run the waypoint mission),
//! `Estop` (latched stop). Future: `Teleop` (operator RoveControl passthrough)
//! and arbitration between them. The router owns the waypoint queue and ticks
//! the GoTo behaviour, emitting a normalised track intent or a terminal verdict.

use crate::behaviors::goto::{self, Waypoint};
use crate::config::Goto;
use crate::position::Pose;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Idle,
    Auto,
    // Estop is latched by the M7 ReflexEngine; pre-wired here, not yet triggered.
    #[allow(dead_code)]
    Estop,
}

/// What the control loop should do this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Action {
    /// Outer-loop intent: drive `forward` (normalised), correcting `heading_err`
    /// (rad) via the inner IMU heading controller. `dist` is metres remaining.
    Drive {
        forward: f64,
        heading_err: f64,
        dist: f64,
    },
    /// Hold position (send idle/zero).
    Hold,
    /// Mission complete — every waypoint reached.
    MissionComplete,
}

pub struct Router {
    mode: Mode,
    waypoints: Vec<Waypoint>,
    idx: usize,
    goto: Goto,
}

impl Router {
    pub fn new(waypoints: Vec<Waypoint>, goto: Goto) -> Self {
        let mode = if waypoints.is_empty() {
            Mode::Idle
        } else {
            Mode::Auto
        };
        Self {
            mode,
            waypoints,
            idx: 0,
            goto,
        }
    }

    #[allow(dead_code)] // consumed by teleop/reflex arbitration (M7/M9)
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Latched emergency stop. Only a fresh mission can leave it (not modelled
    /// in slice 1 — restart the engine). Triggered by the M7 ReflexEngine.
    #[allow(dead_code)]
    pub fn estop(&mut self) {
        self.mode = Mode::Estop;
    }

    /// Current target waypoint, if any (for logging).
    #[allow(dead_code)]
    pub fn current_waypoint(&self) -> Option<Waypoint> {
        self.waypoints.get(self.idx).copied()
    }

    pub fn waypoint_index(&self) -> usize {
        self.idx
    }

    pub fn waypoint_count(&self) -> usize {
        self.waypoints.len()
    }

    /// Advance the state machine one tick against the latest pose.
    pub fn tick(&mut self, pose: &Pose) -> Action {
        match self.mode {
            Mode::Idle | Mode::Estop => Action::Hold,
            Mode::Auto => {
                let Some(wp) = self.waypoints.get(self.idx) else {
                    self.mode = Mode::Idle;
                    return Action::MissionComplete;
                };
                match goto::step(pose, wp, &self.goto) {
                    goto::Step::Drive {
                        forward,
                        heading_err,
                        dist,
                    } => Action::Drive {
                        forward,
                        heading_err,
                        dist,
                    },
                    goto::Step::Arrived => {
                        self.idx += 1;
                        if self.idx >= self.waypoints.len() {
                            self.mode = Mode::Idle;
                            Action::MissionComplete
                        } else {
                            // Pivot toward the next one on the following tick.
                            Action::Hold
                        }
                    }
                }
            }
        }
    }
}
