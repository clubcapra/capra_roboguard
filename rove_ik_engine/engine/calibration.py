"""Persist sync offsets across restarts.

The kinova arm and the drive ODrives (drums + flippers) are mirrored into the
model through an offset captured at Sync. Those offsets live in `EngineState`
and were lost on every restart — this saves them to a JSON file next to
engine.toml on each Sync and reloads them at startup, so a synced robot stays
synced across reboots (the editor used to persist these; the runtime didn't).

Keyed by joint entity id. On load we drop any id no longer in the scene, so a
changed model can't apply stale offsets to the wrong joint.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path

from .state import EngineState

_log = logging.getLogger(__name__)

OFFSETS_FILENAME = "sync_offsets.json"
_VERSION = 1


def save_offsets(path: Path, state: EngineState) -> None:
    """Write the current kinova + drive offsets to `path` (atomic replace).

    Both sections are written from live state every time, so loading at startup
    (which populates state) keeps a Sync of one subsystem from clobbering the
    other's persisted offsets."""
    data = {
        "version": _VERSION,
        "kinova": {
            "chain_joint_ids": list(state.kinova_chain_joint_ids),
            "offsets": {k: float(v) for k, v in state.kinova_offsets.items()},
            "signs": {k: float(v) for k, v in state.kinova_signs.items()},
        },
        "drives": {
            # Gear ratio at capture — offsets/positions are scale-dependent, so a
            # changed ratio invalidates them (re-sync required).
            "gear_ratio": float(state.flipper_gear_ratio),
            # Tuned signs (scale-independent ±1) — kept across a ratio change.
            "signs": {k: float(v) for k, v in state.flipper_signs_persisted.items()},
            "offsets": {k: float(v) for k, v in state.flipper_offsets.items()},
            # PHYSICAL angle of each synced flipper (rad). Survives the ODrive
            # encoder reset on power-cycle; the offset above does not.
            "positions": {
                eid: float(state.joint_values.get(eid, 0.0))
                for eid in state.flipper_offsets
            },
        },
        # Named pose library (name -> {joint eid -> angle rad}).
        "poses": {
            name: {k: float(v) for k, v in pose.items()}
            for name, pose in state.poses.items()
        },
    }
    try:
        tmp = path.with_name(path.name + ".tmp")
        tmp.write_text(json.dumps(data, indent=2))
        tmp.replace(path)  # atomic on POSIX
        _log.info("saved sync offsets -> %s (kinova=%d, drives=%d)",
                  path, len(data["kinova"]["offsets"]), len(data["drives"]["offsets"]))
    except Exception as exc:  # noqa: BLE001
        _log.warning("could not persist sync offsets to %s: %s", path, exc)


def load_offsets(path: Path, state: EngineState, drive_gear_ratio: float | None = None) -> dict:
    """Restore offsets from `path` into `state`, ignoring ids not in the scene.

    Restores enough for the mirrors to resume immediately once live frames flow:
    kinova needs offsets + signs + chain_joint_ids; drives need only offsets
    (their sign/scale/joint->node map are rebuilt from config at startup).

    If `drive_gear_ratio` differs from the one the drive sync was captured with,
    the persisted drive offsets/positions are SCALE-INVALID and are dropped
    (forcing a fresh re-sync) — they were computed against a different ratio."""
    if not path.exists():
        return {"loaded": False, "reason": "no file"}
    try:
        data = json.loads(path.read_text())
    except Exception as exc:  # noqa: BLE001
        _log.warning("ignoring unreadable sync offsets %s: %s", path, exc)
        return {"loaded": False, "reason": str(exc)}

    scene_ids = set(state.project.scene.entities)
    d0 = data.get("drives") or data.get("flippers") or {}
    saved_gr = d0.get("gear_ratio")
    drive_stale = (
        drive_gear_ratio is not None and saved_gr is not None
        and abs(float(saved_gr) - float(drive_gear_ratio)) > 1e-6
    )
    if drive_stale:
        _log.warning("drive gear_ratio changed (%.3f -> %.3f) — dropping stale flipper "
                     "sync (offsets+positions); RE-SYNC the flippers.", float(saved_gr), float(drive_gear_ratio))

    k = data.get("kinova", {}) or {}
    state.kinova_offsets = {e: float(v) for e, v in (k.get("offsets") or {}).items() if e in scene_ids}
    state.kinova_signs = {e: float(v) for e, v in (k.get("signs") or {}).items() if e in scene_ids}
    state.kinova_chain_joint_ids = [e for e in (k.get("chain_joint_ids") or []) if e in scene_ids]

    # Tuned signs are ±1 (scale-independent), so restore them even if the gear
    # ratio changed — they survive a re-sync.
    state.flipper_signs_persisted = {
        e: float(v) for e, v in (d0.get("signs") or {}).items() if e in scene_ids
    }
    # Accept "drives" (current) or legacy "flippers" key. Skip the scale-
    # dependent offsets/positions entirely if the gear ratio changed.
    d = {} if drive_stale else d0
    state.flipper_offsets = {e: float(v) for e, v in (d.get("offsets") or {}).items() if e in scene_ids}
    # Persisted physical flipper angles — restored, then used to re-anchor the
    # offset on the first frame after a power-cycle (see server.run).
    state.flipper_phys_persisted = {
        e: float(v) for e, v in (d.get("positions") or {}).items() if e in scene_ids
    }

    # Pose library (current). Drop ids not in the scene.
    poses = data.get("poses")
    if isinstance(poses, dict):
        for name, pose in poses.items():
            if isinstance(pose, dict):
                state.poses[name] = {e: float(v) for e, v in pose.items() if e in scene_ids}
    # Legacy single home_pose -> poses["home"].
    hp = data.get("home_pose")
    if isinstance(hp, dict) and "home" not in state.poses:
        state.poses["home"] = {e: float(v) for e, v in hp.items() if e in scene_ids}

    nk, nd = len(state.kinova_offsets), len(state.flipper_offsets)
    if nk or nd:
        _log.info("restored sync offsets from %s — kinova=%d joints, drives=%d joints "
                  "(mirrors resume once frames arrive)", path, nk, nd)
    return {"loaded": True, "kinova": nk, "drives": nd}
