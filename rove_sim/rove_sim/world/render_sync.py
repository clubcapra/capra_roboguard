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

import pybullet as p

DEFAULT_STATE_FILE = "/dev/shm/rove_robot_state.json"


def robot_state(robot) -> dict:
    """Snapshot the robot's base pose + all joint angles (index order)."""
    pos, orn = p.getBasePositionAndOrientation(robot.body_id)
    n = p.getNumJoints(robot.body_id)
    q = [s[0] for s in p.getJointStates(robot.body_id, list(range(n)))]
    return {"pos": list(pos), "orn": list(orn), "q": q}


def publish_robot_state(robot, path: str = DEFAULT_STATE_FILE) -> None:
    """Write the live robot state to the shared file (atomic)."""
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w") as f:
        json.dump(robot_state(robot), f)
    os.replace(tmp, path)                          # atomic swap; readers never tear


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
