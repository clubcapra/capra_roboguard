//! Mission layer — SAR Safe Points + behaviours that compose from GoTo.
//!
//! Per the SAR spec the robot keeps a **Safe Point** registry of known-good
//! positions, updated automatically and by the operator:
//!   origin · last_comms · last_gnss_trusted · last_stable · last_vision · operator
//! Each carries a coordinate + position confidence + drift estimate. Fallback
//! behaviours resolve "the best safe point for this trigger" by distance × age ×
//! confidence — ReturnHome doesn't blindly go to origin.
//!
//! (POIs in the SAR are vision-detected OBJECTS — personnel, vehicles, etc. — a
//! separate registry fed by the VisionService; stubbed here until that lands.)
//!
//! The mission BEHAVIOURS are SAR verbs that almost all reduce to "fly a list of
//! GoTo targets", so they compile to a waypoint [`Plan`] the router already flies,
//! plus a terminal disposition (Complete / Hold):
//!   GoTo · Waypoint · ReturnHome(target) · Orbit · Sentinel · Patrol
//!   Retreat(path_history) · BacktrackComm(path_history) · Explore(frontier)

use crate::behaviors::goto::Waypoint;
use std::collections::HashMap;

/// SAR safe-point classes. `Operator` points are labelled (rally/extraction).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum SafeKind {
    Origin,
    LastComms,
    LastGnssTrusted,
    LastStable,
    LastVision,
    Operator(String),
}

impl SafeKind {
    /// Map a ReturnHome `target` string to a key (default origin).
    pub fn key(s: &str) -> SafeKind {
        match s.to_ascii_lowercase().as_str() {
            "last_comms" => SafeKind::LastComms,
            "last_gnss_trusted" => SafeKind::LastGnssTrusted,
            "last_stable" => SafeKind::LastStable,
            "last_vision" => SafeKind::LastVision,
            "origin" => SafeKind::Origin,
            other => SafeKind::Operator(other.to_string()),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SafePoint {
    pub coord: [f64; 2],
    pub confidence: f64, // position confidence when recorded [0,1]
    pub drift_m: f64,    // drift estimate when recorded (m)
}

/// The robot's safe-point registry (SAR).
#[derive(Default)]
pub struct SafePoints {
    pts: HashMap<SafeKind, SafePoint>,
}

impl SafePoints {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set(&mut self, kind: SafeKind, coord: [f64; 2], confidence: f64, drift_m: f64) {
        self.pts.insert(kind, SafePoint { coord, confidence, drift_m });
    }

    /// SetSafePoint(label) — operator marks the current position.
    pub fn set_operator(&mut self, label: &str, coord: [f64; 2], confidence: f64) {
        self.set(SafeKind::Operator(label.to_string()), coord, confidence, 0.0);
    }

    /// ClearSafePoint(label).
    pub fn clear_operator(&mut self, label: &str) {
        self.pts.remove(&SafeKind::Operator(label.to_string()));
    }

    pub fn get(&self, kind: &SafeKind) -> Option<SafePoint> {
        self.pts.get(kind).copied()
    }

    /// Resolve a ReturnHome target. The named point if present; otherwise the
    /// "best" reachable fallback by SAR score (high confidence, near, low drift).
    pub fn resolve(&self, target: &SafeKind, robot: [f64; 2]) -> Option<[f64; 2]> {
        if let Some(sp) = self.pts.get(target) {
            return Some(sp.coord);
        }
        self.best(robot)
    }

    /// SAR fallback resolution: distance × confidence × drift → the safest point.
    pub fn best(&self, robot: [f64; 2]) -> Option<[f64; 2]> {
        self.pts
            .values()
            .map(|sp| {
                let d = ((sp.coord[0] - robot[0]).powi(2) + (sp.coord[1] - robot[1]).powi(2)).sqrt();
                let score = sp.confidence / (1.0 + 0.05 * d + sp.drift_m);
                (score, sp.coord)
            })
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
            .map(|(_, c)| c)
    }

    pub fn len(&self) -> usize {
        self.pts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }
}

/// What to do once the compiled waypoint plan is exhausted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Terminal {
    Complete,
    /// Reached the post — hold + watch (Sentinel / Retreat / BacktrackComm).
    Hold,
}

/// A SAR mission verb.
#[derive(Debug, Clone)]
pub enum Behavior {
    GoTo([f64; 2]),
    Waypoint(Vec<[f64; 2]>),
    Patrol([f64; 2], [f64; 2]),
    ReturnHome(SafeKind),
    Orbit { center: [f64; 2], radius: f64, laps: u32 },
    Sentinel([f64; 2]),
    Retreat { dist: f64 },
    BacktrackComm,
    Explore,
}

/// A compiled mission: a waypoint plan + terminal disposition.
pub struct Plan {
    pub waypoints: Vec<Waypoint>,
    pub terminal: Terminal,
    /// Explore is regenerated live from the frontier; not a static list.
    pub dynamic: bool,
}

const ORBIT_SEGMENTS: usize = 12;

/// Compile a behaviour into a flyable [`Plan`] given the safe points, current
/// position, and the breadcrumb trail travelled so far (oldest → newest).
pub fn compile(
    behavior: &Behavior,
    safe: &SafePoints,
    start: [f64; 2],
    breadcrumbs: &[[f64; 2]],
) -> Plan {
    let wp = |p: [f64; 2]| Waypoint { x: p[0], y: p[1] };
    let plan = |w: Vec<Waypoint>, t: Terminal| Plan { waypoints: w, terminal: t, dynamic: false };
    match behavior {
        Behavior::GoTo(p) => plan(vec![wp(*p)], Terminal::Complete),
        Behavior::Waypoint(list) => plan(list.iter().map(|p| wp(*p)).collect(), Terminal::Complete),
        Behavior::Patrol(a, b) => plan(vec![wp(*a), wp(*b)], Terminal::Complete),
        Behavior::ReturnHome(target) => {
            let h = safe.resolve(target, start).unwrap_or(start);
            plan(vec![wp(h)], Terminal::Complete)
        }
        Behavior::Orbit { center, radius, laps } => {
            let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
            let mut pts = Vec::new();
            for _ in 0..(*laps).max(1) {
                for k in 1..=ORBIT_SEGMENTS {
                    let a = a0 + std::f64::consts::TAU * (k as f64) / (ORBIT_SEGMENTS as f64);
                    pts.push(wp([center[0] + radius * a.cos(), center[1] + radius * a.sin()]));
                }
            }
            plan(pts, Terminal::Complete)
        }
        Behavior::Sentinel(p) => plan(vec![wp(*p)], Terminal::Hold),
        Behavior::Retreat { dist } => {
            let t = breadcrumb_back(breadcrumbs, start, *dist).unwrap_or(start);
            plan(vec![wp(t)], Terminal::Hold)
        }
        Behavior::BacktrackComm => {
            let mut pts: Vec<Waypoint> = breadcrumbs.iter().rev().map(|p| wp(*p)).collect();
            if pts.is_empty() {
                pts.push(wp(start));
            }
            plan(pts, Terminal::Hold)
        }
        Behavior::Explore => Plan { waypoints: vec![], terminal: Terminal::Complete, dynamic: true },
    }
}

/// The breadcrumb roughly `dist` metres back along the trail from the robot.
fn breadcrumb_back(crumbs: &[[f64; 2]], start: [f64; 2], dist: f64) -> Option<[f64; 2]> {
    let mut acc = 0.0;
    let mut prev = start;
    for c in crumbs.iter().rev() {
        acc += ((c[0] - prev[0]).powi(2) + (c[1] - prev[1]).powi(2)).sqrt();
        prev = *c;
        if acc >= dist {
            return Some(*c);
        }
    }
    crumbs.first().copied()
}

/// Build a behaviour from config (name + the params it needs).
pub fn from_config(
    name: &str,
    waypoints: &[[f64; 2]],
    orbit_center: Option<[f64; 2]>,
    orbit_radius: f64,
    orbit_laps: u32,
    retreat_dist: f64,
    return_target: &str,
) -> Behavior {
    let first = || waypoints.first().copied().unwrap_or([0.0, 0.0]);
    match name.to_ascii_lowercase().as_str() {
        "goto" => Behavior::GoTo(first()),
        "patrol" => Behavior::Patrol(
            waypoints.first().copied().unwrap_or([0.0, 0.0]),
            waypoints.get(1).copied().unwrap_or([0.0, 0.0]),
        ),
        "returnhome" | "return_home" => Behavior::ReturnHome(SafeKind::key(return_target)),
        "orbit" => Behavior::Orbit {
            center: orbit_center.or_else(|| waypoints.first().copied()).unwrap_or([0.0, 0.0]),
            radius: orbit_radius,
            laps: orbit_laps,
        },
        "sentinel" => Behavior::Sentinel(first()),
        "retreat" => Behavior::Retreat { dist: retreat_dist },
        "backtrackcomm" | "backtrack" => Behavior::BacktrackComm,
        "explore" => Behavior::Explore,
        _ => Behavior::Waypoint(waypoints.to_vec()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn return_home_resolves_origin() {
        let mut s = SafePoints::new();
        s.set(SafeKind::Origin, [3.0, -6.0], 1.0, 0.0);
        let plan = compile(&Behavior::ReturnHome(SafeKind::Origin), &s, [20.0, 5.0], &[]);
        assert_eq!((plan.waypoints[0].x, plan.waypoints[0].y), (3.0, -6.0));
    }

    #[test]
    fn return_home_falls_back_to_best_when_target_missing() {
        let mut s = SafePoints::new();
        // No last_comms; a high-confidence stable point nearby should win.
        s.set(SafeKind::LastStable, [10.0, 0.0], 0.9, 0.5);
        s.set(SafeKind::Origin, [0.0, 0.0], 1.0, 30.0); // far + huge drift -> worse
        let plan = compile(&Behavior::ReturnHome(SafeKind::LastComms), &s, [11.0, 0.0], &[]);
        assert_eq!((plan.waypoints[0].x, plan.waypoints[0].y), (10.0, 0.0));
    }

    #[test]
    fn operator_safe_point_set_and_clear() {
        let mut s = SafePoints::new();
        s.set_operator("rally", [5.0, 5.0], 0.8);
        assert!(s.get(&SafeKind::Operator("rally".into())).is_some());
        s.clear_operator("rally");
        assert!(s.get(&SafeKind::Operator("rally".into())).is_none());
    }

    #[test]
    fn orbit_makes_a_ring_of_segments() {
        let s = SafePoints::new();
        let plan = compile(
            &Behavior::Orbit { center: [10.0, 0.0], radius: 4.0, laps: 1 },
            &s,
            [14.0, 0.0],
            &[],
        );
        assert_eq!(plan.waypoints.len(), ORBIT_SEGMENTS);
        for w in &plan.waypoints {
            let r = ((w.x - 10.0).powi(2) + w.y.powi(2)).sqrt();
            assert!((r - 4.0).abs() < 1e-6, "r={r}");
        }
    }

    #[test]
    fn retreat_walks_back_the_trail() {
        let s = SafePoints::new();
        let crumbs = [[1.0, 0.0], [2.0, 0.0], [3.0, 0.0], [4.0, 0.0]];
        let plan = compile(&Behavior::Retreat { dist: 2.0 }, &s, [5.0, 0.0], &crumbs);
        assert_eq!(plan.terminal, Terminal::Hold);
        assert!(plan.waypoints[0].x <= 3.5, "should step back down the trail");
    }
}
