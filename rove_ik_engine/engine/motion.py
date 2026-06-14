"""Joint-space pose-to-pose motion — plan and execute a smooth trajectory from
the current model pose to a saved named pose.

This sits on top of the per-tick IK: instead of an instant snap (reset-to-home),
it ramps every joint from its current angle to the target over a duration set by
a max joint speed, with ease-in/ease-out so starts and stops are gentle. While a
motion is active the planned joints are owned by the trajectory — the hardware
mirrors and the Ovis IK leave them alone — so the model shows the planned move.

MODEL-ONLY by default: this drives `state.joint_values`. It does not command the
real robot. Real-arm execution would ride the existing (gated, default-off)
velocity output; this module never enables it.

Collision-awareness: the vendored solver only exposes collision checking *inside*
`solve_position_ik`, not as a standalone "is this config colliding" call, and the
engine's `collision_aware` is off by default — so this planner does NOT yet
collision-check the path. Flagged as a follow-on (wire an FCL per-waypoint check).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from .state import EngineState

# Default joint speed for a planned move (deg/s of the fastest joint). The move
# duration scales so the largest joint delta runs at this rate; gentler joints
# finish proportionally. Conservative on purpose.
DEFAULT_SPEED_DEG_S = 30.0
_MIN_DURATION_S = 0.15


@dataclass
class Motion:
    name: str
    eids: list[str]
    starts: dict[str, float]   # eid -> start angle (rad)
    targets: dict[str, float]  # eid -> target angle (rad)
    t0: float                  # monotonic start
    duration: float            # seconds
    joint_set: frozenset[str] = field(default_factory=frozenset)


def _smoothstep(s: float) -> float:
    """Ease in/out: 0..1 -> 0..1 with zero slope at both ends."""
    s = max(0.0, min(1.0, s))
    return s * s * (3.0 - 2.0 * s)


def plan_to_pose(state: EngineState, name: str, *,
                 speed_deg_s: float = DEFAULT_SPEED_DEG_S) -> dict:
    """Plan a joint-space move from the current pose to saved pose `name`.

    Stores the trajectory in `state.active_motion`; the tick advances it. Returns
    a summary (duration, per-joint deltas). Replaces any in-flight motion."""
    pose = state.poses.get(name)
    if pose is None:
        return {"ok": False, "error": f"no saved pose named {name!r}"}

    scene = state.project.scene
    starts: dict[str, float] = {}
    targets: dict[str, float] = {}
    eids: list[str] = []
    for eid, target in pose.items():
        ent = scene.entities.get(eid)
        joint = ent.get("joint") if ent else None
        if joint is None or joint.type == "fixed":
            continue  # pose may reference a now-fixed/removed joint
        starts[eid] = float(state.joint_values.get(eid, 0.0))
        targets[eid] = float(target)
        eids.append(eid)

    if not eids:
        return {"ok": False, "error": "pose has no movable joints in the current scene"}

    max_delta = max(abs(targets[e] - starts[e]) for e in eids)
    if max_delta < 1e-4:
        # Already there — snap and finish instantly.
        for e in eids:
            state.joint_values[e] = targets[e]
        state.active_motion = None
        return {"ok": True, "name": name, "duration_s": 0.0, "joints": len(eids), "note": "already at pose"}

    speed_rad_s = max(1e-3, math.radians(speed_deg_s))
    duration = max(_MIN_DURATION_S, max_delta / speed_rad_s)
    state.active_motion = Motion(
        name=name, eids=eids, starts=starts, targets=targets,
        t0=time.monotonic(), duration=duration, joint_set=frozenset(eids),
    )
    return {
        "ok": True, "name": name, "duration_s": round(duration, 2), "joints": len(eids),
        "deltas_deg": {e[-8:]: round(math.degrees(targets[e] - starts[e]), 1) for e in eids},
    }


def advance_motion(state: EngineState) -> bool:
    """Step the active motion. Writes planned joints into joint_values and sets
    their velocities (finite difference). Returns True while a motion is running
    (so the tick can skip IK), False if none/just finished."""
    m = state.active_motion
    if not isinstance(m, Motion):
        return False
    now = time.monotonic()
    s = (now - m.t0) / m.duration if m.duration > 0 else 1.0
    ss = _smoothstep(s)
    # Analytic velocity of the smoothstep ramp, so the (gated) arm velocity
    # output follows a planned move: d(ss)/dt = 6 s (1-s) / duration.
    sc = max(0.0, min(1.0, s))
    dss_dt = (6.0 * sc * (1.0 - sc)) / m.duration if m.duration > 0 else 0.0
    for eid in m.eids:
        a, b = m.starts[eid], m.targets[eid]
        state.joint_values[eid] = a + (b - a) * ss
        state.joint_velocities[eid] = (b - a) * dss_dt
    if s >= 1.0:
        for eid in m.eids:                 # land exactly on target
            state.joint_values[eid] = m.targets[eid]
            state.joint_velocities[eid] = 0.0
        state.active_motion = None
        return False
    return True


def stop_motion(state: EngineState) -> dict:
    """Abort the active motion in place (joints hold at current angle)."""
    m = state.active_motion
    if not isinstance(m, Motion):
        return {"ok": True, "stopped": False}
    state.active_motion = None
    for eid in m.eids:
        state.joint_velocities[eid] = 0.0
    return {"ok": True, "stopped": True, "was": m.name}


def motion_status(state: EngineState) -> dict:
    m = state.active_motion
    if not isinstance(m, Motion):
        return {"active": False}
    now = time.monotonic()
    s = max(0.0, min(1.0, (now - m.t0) / m.duration if m.duration > 0 else 1.0))
    return {"active": True, "name": m.name, "progress": round(s, 3),
            "remaining_s": round(max(0.0, m.duration - (now - m.t0)), 2)}


def motion_joint_set(state: EngineState) -> frozenset:
    """Joints currently owned by an active motion (mirrors/IK skip these)."""
    m = state.active_motion
    return m.joint_set if isinstance(m, Motion) else frozenset()
