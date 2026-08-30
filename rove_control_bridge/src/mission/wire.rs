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

/// Pose-goto command names (case-insensitive) that relay a named joint-space pose
/// to the IK engine instead of compiling to a nav waypoint.
const POSE_COMMANDS: &[&str] = &["pose", "gotopose", "goto_pose", "arm_pose"];

/// Extract a pose-goto command from a step: one of [`POSE_COMMANDS`] with a
/// `pose`/`name`/`target` text param and an optional `speed_deg_s` number.
/// Returns `(pose_name, speed_deg_s)` or `None` if it isn't a pose command (or
/// has no name). Pose moves relay to the engine over HTTP — they're position-free
/// (no GNSS fix needed), so the mission listener fires them on receipt.
pub fn pose_command(step: &proto::Step) -> Option<(String, Option<f64>)> {
    if !POSE_COMMANDS.contains(&step.command.to_ascii_lowercase().as_str()) {
        return None;
    }
    let name = ["pose", "name", "target"]
        .iter()
        .find_map(|n| value(step, n).and_then(as_text))
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())?;
    let speed = value(step, "speed_deg_s").and_then(as_number);
    Some((name, speed))
}

/// Every pose-goto command in a mission, in sequence order.
pub fn pose_steps(m: &proto::Mission) -> Vec<(String, Option<f64>)> {
    m.sequence.iter().filter_map(pose_command).collect()
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::comms::proto;

    fn text_param(name: &str, val: &str) -> proto::Param {
        proto::Param {
            name: name.into(),
            value: Some(proto::Value {
                kind: Some(proto::value::Kind::Text(val.into())),
            }),
        }
    }
    fn num_param(name: &str, val: f64) -> proto::Param {
        proto::Param {
            name: name.into(),
            value: Some(proto::Value { kind: Some(proto::value::Kind::Number(val)) }),
        }
    }
    fn step(command: &str, params: Vec<proto::Param>) -> proto::Step {
        proto::Step { command: command.into(), params, binds: String::new(), transition: None }
    }

    #[test]
    fn pose_command_extracts_name_and_speed() {
        let s = step("pose", vec![text_param("name", "home"), num_param("speed_deg_s", 25.0)]);
        assert_eq!(pose_command(&s), Some(("home".to_string(), Some(25.0))));
    }

    #[test]
    fn pose_command_accepts_aliases_and_pose_param() {
        let s = step("GotoPose", vec![text_param("pose", " stow ")]);
        assert_eq!(pose_command(&s), Some(("stow".to_string(), None)));
    }

    #[test]
    fn pose_command_rejects_non_pose_and_empty_name() {
        assert_eq!(pose_command(&step("goto", vec![text_param("name", "home")])), None);
        assert_eq!(pose_command(&step("pose", vec![text_param("name", "  ")])), None);
        assert_eq!(pose_command(&step("pose", vec![])), None);
    }

    #[test]
    fn pose_only_mission_is_position_free() {
        let m = proto::Mission {
            sequence: vec![step("pose", vec![text_param("name", "home")])],
            ..Default::default()
        };
        assert!(!requires_position(&m));
        assert_eq!(pose_steps(&m), vec![("home".to_string(), None)]);
    }

    #[test]
    fn mixed_mission_requires_position() {
        let m = proto::Mission {
            sequence: vec![
                step("pose", vec![text_param("name", "home")]),
                step("goto", vec![]),
            ],
            ..Default::default()
        };
        assert!(requires_position(&m));
    }
}

/// Whether a mission needs a trustworthy position fix before it may start — the
/// GNSS pre-flight reflex. Every navigation behaviour compiles to ENU waypoints
/// about the datum (and several are relative to the current position), so they
/// all require a fix today. A position-free command (e.g. a future arm-only or
/// `wait` step) opts out by being listed in `POSITION_FREE`.
#[allow(dead_code)] // wired into the mission-start pre-flight (control-loop refactor, in progress)
pub fn requires_position(m: &proto::Mission) -> bool {
    m.sequence.iter().any(|s| {
        // Pose-goto is an arm/flipper command relayed to the IK engine — it needs
        // no position fix. Everything else navigates and does.
        pose_command(s).is_none()
    })
}
