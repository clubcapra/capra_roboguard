//! Wire bridge — the udp_multiplexer `Mission` proto → SAR behaviours.
//!
//! A Mission is a sequence of Steps; each Step is a command id + typed params +
//! a transition. We map the geometry commands onto the [`Behavior`] enum (Coords
//! are GEODETIC → converted to local ENU about the datum). Vision-driven commands
//! (Follow / Intercept on a `vision_object`) return `None` here — wired in Phase 5.

use super::{compile, Behavior, SafeKind, SafePoints, Terminal};
use crate::behaviors::goto::Waypoint;
use crate::comms::proto;
use crate::config::Datum;
use crate::position::geodetic;

fn coord_to_enu(c: &proto::Coord, d: &Datum) -> [f64; 2] {
    let (x, y) = geodetic::geodetic_to_enu(c.lat, c.lon, d.lat, d.lon);
    [x, y]
}

fn value<'a>(step: &'a proto::Step, name: &str) -> Option<&'a proto::Value> {
    step.params.iter().find(|p| p.name == name).and_then(|p| p.value.as_ref())
}
fn as_coord(v: &proto::Value) -> Option<&proto::Coord> {
    match &v.kind {
        Some(proto::value::Kind::Coordinate(c)) => Some(c),
        _ => None,
    }
}
fn as_number(v: &proto::Value) -> Option<f64> {
    match &v.kind {
        Some(proto::value::Kind::Number(n)) => Some(*n),
        _ => None,
    }
}
fn as_text(v: &proto::Value) -> Option<&str> {
    match &v.kind {
        Some(proto::value::Kind::Text(s)) | Some(proto::value::Kind::EnumValue(s)) => Some(s),
        _ => None,
    }
}
fn as_route(v: &proto::Value) -> Option<&proto::Route> {
    match &v.kind {
        Some(proto::value::Kind::Route(r)) => Some(r),
        _ => None,
    }
}

/// Map one mission Step to a SAR [`Behavior`]. `None` = unknown or vision-driven.
pub fn behavior_from_step(step: &proto::Step, d: &Datum) -> Option<Behavior> {
    let coord = |names: &[&str]| names.iter().find_map(|n| value(step, n).and_then(as_coord));
    let laps = || {
        step.transition.as_ref().map_or(1, |t| {
            if t.r#type == proto::TransitionType::LoopN as i32 {
                t.loop_count.max(1)
            } else {
                1
            }
        })
    };
    match step.command.to_ascii_lowercase().as_str() {
        "goto" => Some(Behavior::GoTo(coord_to_enu(coord(&["coordinate", "target"])?, d))),
        "sentinel" => Some(Behavior::Sentinel(coord_to_enu(coord(&["coordinate"])?, d))),
        "orbit" => Some(Behavior::Orbit {
            center: coord_to_enu(coord(&["coordinate", "center"])?, d),
            radius: value(step, "radius").and_then(as_number).unwrap_or(4.0),
            laps: laps(),
        }),
        "patrol" => Some(Behavior::Patrol(
            coord_to_enu(coord(&["coordinate_a"])?, d),
            coord_to_enu(coord(&["coordinate_b"])?, d),
        )),
        "waypoint" | "followpath" => {
            let r = value(step, "coordinate_list").or_else(|| value(step, "route")).and_then(as_route)?;
            Some(Behavior::Waypoint(r.points.iter().map(|c| coord_to_enu(c, d)).collect()))
        }
        "returnhome" => Some(Behavior::ReturnHome(SafeKind::key(
            value(step, "target").and_then(as_text).unwrap_or("origin"),
        ))),
        "retreat" => Some(Behavior::Retreat {
            dist: value(step, "distance").and_then(as_number).unwrap_or(5.0),
        }),
        "backtrackcomm" | "backtrack" => Some(Behavior::BacktrackComm),
        "explore" => Some(Behavior::Explore),
        _ => None, // unknown / vision-driven (Follow, Intercept, Track) -> Phase 5
    }
}

/// Compile a whole Mission into a flat waypoint plan + terminal disposition.
/// (Geometry steps flatten into one path; UNTIL_STOPPED -> Hold. Per-step success
/// conditions / blackboard variables are a Router enhancement.) Returns the plan
/// plus how many steps compiled (for the MissionResult reply).
pub fn compile_mission(
    m: &proto::Mission,
    d: &Datum,
    safe: &SafePoints,
    start: [f64; 2],
) -> (Vec<Waypoint>, Terminal, usize) {
    let mut wps = Vec::new();
    let mut terminal = Terminal::Complete;
    let mut compiled = 0usize;
    let mut cursor = start;
    for step in &m.sequence {
        if let Some(b) = behavior_from_step(step, d) {
            let plan = compile(&b, safe, cursor, &[]);
            if let Some(last) = plan.waypoints.last() {
                cursor = [last.x, last.y];
            }
            wps.extend(plan.waypoints);
            compiled += 1;
            if step.transition.as_ref().map(|t| t.r#type) == Some(proto::TransitionType::UntilStopped as i32) {
                terminal = Terminal::Hold;
            }
        }
    }
    (wps, terminal, compiled)
}

/// Whether a mission needs a trustworthy position fix before it may start — the
/// GNSS pre-flight reflex. Every navigation behaviour compiles to ENU waypoints
/// about the datum (and several are relative to the current position), so they
/// all require a fix today. A position-free command (e.g. a future arm-only or
/// `wait` step) opts out by being listed in `POSITION_FREE`.
#[allow(dead_code)] // wired into the mission-start pre-flight (control-loop refactor, in progress)
pub fn requires_position(m: &proto::Mission) -> bool {
    /// Commands that can run with no position fix (none yet).
    const POSITION_FREE: &[&str] = &[];
    m.sequence.iter().any(|s| {
        let cmd = s.command.to_ascii_lowercase();
        !POSITION_FREE.contains(&cmd.as_str())
    })
}
