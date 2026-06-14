"""Direct joint posing — set any movable joint angle in the model.

This is the engine-side primitive behind a "rotate this joint" gizmo: pose the
model to match the real robot's physical pose, then Sync to capture the offset
(the inverse of physically moving the real robot to the model home).

It writes straight into `state.joint_values[eid]`, which the tick loop
broadcasts over `/state`, so the 3D view follows. Pre-sync the kinova/flipper
mirrors no-op, so a posed value persists; once a chain is synced its mirror
takes over (posing it then is pointless — sync first off the posed model).
"""

from __future__ import annotations

import math

from forgebot.core.model import JointComponent

from .state import EngineState


def resolve_joint(state: EngineState, ref: str) -> str | None:
    """Resolve a joint entity id from: a joint entity id, a link entity id
    (-> its parent joint), a joint NAME, or a link NAME (-> its parent joint).

    Lets callers refer to a joint however is convenient — the flipper joints
    have no name (the link carries it), while arm joints are named."""
    ref = ref.strip()
    if not ref:
        return None
    scene = state.project.scene

    def joint_of(eid: str) -> str | None:
        ent = scene.entities.get(eid)
        if ent is None:
            return None
        if ent.get("joint") is not None:
            return eid
        # A link: its parent is usually the joint that moves it.
        parent = scene.entities.get(ent.parent) if ent.parent else None
        if parent is not None and parent.get("joint") is not None:
            return ent.parent
        return None

    # 1. Direct entity-id reference.
    if ref in scene.entities:
        return joint_of(ref)

    # 2. Name reference (joint name, else link name -> parent joint).
    target = ref.lower()
    for eid, ent in scene.entities.items():
        if (ent.name or "").strip().lower() == target:
            j = joint_of(eid)
            if j is not None:
                return j
    return None


def _is_movable(state: EngineState, eid: str) -> bool:
    ent = state.project.scene.entities.get(eid)
    joint = ent.get("joint") if ent else None
    return joint is not None and joint.type != "fixed"


def set_joint(state: EngineState, ref: str, *,
              angle_deg: float | None = None,
              delta_deg: float | None = None) -> dict:
    """Set a movable joint absolutely (`angle_deg`) or relatively (`delta_deg`).

    Returns a JSON-able result. Refuses fixed joints and unknown refs. Warns
    (but still applies) if the joint is already synced — its mirror will
    overwrite the pose on the next tick."""
    eid = resolve_joint(state, ref)
    if eid is None:
        return {"ok": False, "error": f"no joint resolves from {ref!r} (id or name)"}
    if not _is_movable(state, eid):
        return {"ok": False, "error": f"{ref!r} is a fixed joint"}

    if angle_deg is not None:
        new_q = float(angle_deg) * math.pi / 180.0
    elif delta_deg is not None:
        new_q = state.joint_values.get(eid, 0.0) + float(delta_deg) * math.pi / 180.0
    else:
        return {"ok": False, "error": "specify 'angle_deg' (absolute) or 'delta_deg' (relative)"}

    state.joint_values[eid] = new_q
    synced = eid in state.kinova_offsets or eid in state.flipper_offsets
    return {
        "ok": True,
        "joint": eid,
        "angle_deg": new_q * 180.0 / math.pi,
        "synced": synced,
        "note": "joint is synced — its mirror overwrites this next tick" if synced else "",
    }


def home_pose(state: EngineState) -> dict[str, float]:
    """The effective home: the saved "home" pose if set, else the project's."""
    saved = state.poses.get("home")
    if saved is not None:
        return saved
    return dict(state.project.home_pose) if state.project.home_pose else {}


def _capture_pose(state: EngineState) -> dict[str, float]:
    """Current angle of every movable joint (eid -> rad)."""
    out: dict[str, float] = {}
    for eid, ent in state.project.scene.entities.items():
        joint = ent.get("joint")
        if joint is None or joint.type == "fixed":
            continue
        out[eid] = float(state.joint_values.get(eid, 0.0))
    return out


def save_pose(state: EngineState, name: str) -> dict:
    """Capture the current model pose under `name` (persisted by the caller)."""
    name = name.strip()
    if not name:
        return {"ok": False, "error": "pose name required"}
    captured = _capture_pose(state)
    state.poses[name] = captured
    return {"ok": True, "name": name, "captured": len(captured)}


def delete_pose(state: EngineState, name: str) -> dict:
    if name == "home":
        return {"ok": False, "error": "cannot delete the home pose"}
    existed = state.poses.pop(name, None) is not None
    return {"ok": existed, "name": name, "deleted": existed}


def list_poses(state: EngineState) -> list[dict]:
    return [{"name": n, "joints": len(p)} for n, p in sorted(state.poses.items())]


def reset_to_home(state: EngineState) -> dict:
    """Snap every movable joint to the home pose (instant — no path/motion).

    Uses the operator-set home if one exists, else project.home_pose, else 0.
    Use before a from-home Sync (e.g. kinova): that sync captures
    offset = real_q - model_q assuming the model is at home, so posing the arm
    with the panel first would bake a wrong offset. Synced/mirrored joints snap
    back to hardware on the next tick — reset only sticks for free joints."""
    home = home_pose(state)
    n = 0
    for eid, ent in state.project.scene.entities.items():
        joint = ent.get("joint")
        if joint is None or joint.type == "fixed":
            continue
        state.joint_values[eid] = float(home.get(eid, 0.0))
        n += 1
    return {"ok": True, "reset": n, "source": "saved" if "home" in state.poses else "project"}


def set_home(state: EngineState) -> dict:
    """Capture the CURRENT model pose as the home pose. Records angles only —
    it does not move anything. Persisted by the caller."""
    r = save_pose(state, "home")
    return {"ok": True, "captured": r["captured"]}


def list_movable_joints(state: EngineState) -> list[dict]:
    """Every movable joint with id, name, current angle, axis, and whether a
    hardware mirror currently drives it — enough for a UI joint picker."""
    scene = state.project.scene
    out: list[dict] = []
    for eid, ent in scene.entities.items():
        joint = ent.get("joint")
        if joint is None or joint.type == "fixed":
            continue
        # Joints share the generic name "joint_revolute"; the meaningful name
        # lives on the child LINK (FlipperFL, JointA, DrumBL...). Prefer that so
        # the operator sees real names, not 14 identical "joint_revolute" rows.
        label = ""
        for cid, c in scene.entities.items():
            if c.parent == eid and c.get("link") is not None and (c.name or "").strip():
                label = c.name.strip()
                break
        if not label:
            label = (ent.name or "").strip()
        out.append({
            "joint": eid,
            "name": label or None,
            "type": joint.type,
            "axis": list(joint.axis) if getattr(joint, "axis", None) is not None else None,
            "angle_deg": state.joint_values.get(eid, 0.0) * 180.0 / math.pi,
            "mirrored": eid in state.kinova_offsets or eid in state.flipper_offsets,
        })
    return out
