"""Robot pose+joint state sync for DECOUPLED RENDER WORKERS.

PyBullet's `getCameraImage` holds the GIL and returns pixels as a Python tuple, so
one process can't render many cameras at a useful fps AND step physics. The fix is
to render cameras in SEPARATE processes (each its own GIL/GL context) that mirror
the authoritative sim's robot. The sim writes its robot's base pose + every joint
angle (tiny) to a shared-memory file; each `cam_worker` reads it and slams its own
copy of the robot to it (no physics), then renders its assigned cameras.

A shared `/dev/shm` file (not UDP) is used so ANY number of workers read the same
state with no port contention. Writes are atomic (temp + os.replace).
"""

from __future__ import annotations

import json
import os
import time
from collections import deque

_HISTORY = {}

import pybullet as p

DEFAULT_STATE_FILE = "/dev/shm/rove_robot_state.json"


def robot_state(robot, world=None) -> dict:
    """Snapshot the robot's base pose + all joint angles (index order)."""
    pos, orn = p.getBasePositionAndOrientation(robot.body_id)
    n = p.getNumJoints(robot.body_id)
    q = [s[0] for s in p.getJointStates(robot.body_id, list(range(n)))]
    state = {"pos": list(pos), "orn": list(orn), "q": q}
    if world is not None:
        state["objects"] = {
            oid: {"pos": list(pose[0]), "orn": list(pose[1])}
            for oid, bid in getattr(world, "objects", {}).items()
            for pose in [p.getBasePositionAndOrientation(bid)]
        }
    return state


def publish_robot_state(robot, path: str = DEFAULT_STATE_FILE, world=None) -> None:
    """Write the live robot state to the shared file (atomic)."""
    state = robot_state(robot, world)
    state["timestamp_ns"] = time.time_ns()
    history = _HISTORY.setdefault(path, deque(maxlen=8))
    history.append(state)
    # Camera workers read only the current state; LiDAR reads the history file.
    # Keep history parsing out of the five-camera render loop.
    for target, payload in (
        (path + ".history", {**state, "history": list(history)}),
        (path, state),
    ):
        tmp = f"{target}.{os.getpid()}.tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f)
        os.replace(tmp, target)


def read_robot_state(path: str = DEFAULT_STATE_FILE):
    """Read the latest published state (or None if not available)."""
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


def apply_robot_state(robot, st: dict) -> None:
    """Kinematically slam `robot` to a published state (worker side)."""
    if not st:
        return
    p.resetBasePositionAndOrientation(robot.body_id, st["pos"], st["orn"])
    nj = p.getNumJoints(robot.body_id)
    for i, a in enumerate(st.get("q") or []):
        if i < nj:
            p.resetJointState(robot.body_id, i, a)


def apply_world_state(world, state: dict) -> None:
    """Mirror dynamic scene objects by stable ID alongside the robot state."""
    if not state:
        return
    for oid, pose in state.get("objects", {}).items():
        bid = getattr(world, "objects", {}).get(oid)
        if bid is not None:
            p.resetBasePositionAndOrientation(bid, pose["pos"], pose["orn"])
