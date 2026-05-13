"""IK tick: integrate the latest Ovis, run the solver, build a StateUpdate.

Calls into the vendored `forgebot.core.kinematics.inverse.solve_position_ik`
so the math is bit-identical to the editor's IK route.
"""

from __future__ import annotations

import logging
import math
import time
from typing import cast

import numpy as np

from forgebot.core.kinematics import solve_position_ik, world_transforms
from forgebot.core.kinematics.transforms import axis_angle_to_quat
from forgebot.core.model import IKProfile, JointComponent

from .chain import count_movable_joints, find_ik_base
from .config import IKConfig
from .proto import (
    EEPose,
    JointState,
    Orientation,
    Ovis,
    Quaternion,
    SolverDiag,
    StateUpdate,
    Vector3,
)
from .state import EngineState

_log = logging.getLogger(__name__)

# Defaults match frontend/src/components/viewport/IKGizmo.tsx constants so
# the engine's untrained behaviour matches the editor's untrained behaviour.
_DEFAULT_REST_POSE_GAIN = 0.30
_DEFAULT_ORIENT_WEIGHT = 5.0
_DEFAULT_DAMPING = 0.05
_DEFAULT_MAX_ITER = 60
_DEFAULT_MAX_DQ_STEP = 0.05
_DEFAULT_MAX_POS_STEP = 0.05
_DEFAULT_MAX_TOTAL_DQ_STEP = 0.10


def tick(state: EngineState, ik: IKConfig, dt: float) -> StateUpdate:
    _apply_kinova_mirror(state)
    ovis = state.take_ovis()
    t = state.elapsed()

    if ovis is None or not ovis.target:
        # No input this tick: hold still, emit telemetry.
        for jid in state.joint_velocities:
            state.joint_velocities[jid] = 0.0
        return _build_state_update(state, t, diag=None)

    tip = ovis.target
    if tip not in state.project.scene.entities:
        _log.warning("Ovis target %r not in scene; ignoring", tip)
        return _build_state_update(state, t, diag=None)

    tcp_offset = _tcp_offset_from_ovis(ovis)
    if tcp_offset is None:
        # Fall back to the per-link centroid the engine computed at startup
        # (clients without mesh geometry — bridge, joysticks — get
        # centroid-anchored rotation for free).
        cached = state.tcp_offsets.get(tip)
        if cached is not None:
            tcp_offset = cached
    target_world, target_rot = _integrate_ovis(state, ik, ovis, dt, tcp_offset)

    base = find_ik_base(state.project, tip)
    if base == tip:
        # Tip is a root link — nothing to move via IK. Emit current state.
        return _build_state_update(state, t, diag=None)

    profile = state.project.ik_profiles.get(base) if state.project.ik_profiles else None
    use_tcp_offset = count_movable_joints(state.project, tip) >= 3
    is_rotate = _is_rotate_intent(ovis)

    # Mode/OSG mirror the editor's IKGizmo translate/rotate split.
    if is_rotate and use_tcp_offset:
        mode = "pos_primary"
        osg = 4.0
    else:
        mode = (profile.mode if profile else None) or "pose_locked"
        osg = profile.orientation_secondary_gain if profile else 0.5

    q_prev = dict(state.joint_values)

    result = solve_position_ik(
        state.project,
        base=base,
        tip=tip,
        target_world=target_world,
        target_rotation=target_rot,
        initial_joint_values=state.joint_values,
        rest_pose=state.joint_values,
        rest_pose_gain=(profile.rest_pose_gain if profile else _DEFAULT_REST_POSE_GAIN),
        joint_weight_strength=(
            profile.joint_weight_strength if profile else 0.0
        ),
        respect_collisions=ik.collision_aware,
        max_iter=(profile.max_iter if profile else _DEFAULT_MAX_ITER),
        damping=(profile.damping if profile else _DEFAULT_DAMPING),
        orientation_weight=(
            profile.orientation_weight if profile else _DEFAULT_ORIENT_WEIGHT
        ),
        mode=mode,
        orientation_secondary_gain=osg,
        max_dq_step=(profile.max_dq_step if profile else _DEFAULT_MAX_DQ_STEP),
        max_pos_step=(profile.max_pos_step if profile else _DEFAULT_MAX_POS_STEP),
        max_total_dq_step=(
            profile.max_total_dq_step if profile else _DEFAULT_MAX_TOTAL_DQ_STEP
        ),
        # When set, makes the IK position task target `link_pos + link_R @ tcp_offset`
        # (the gripper centroid) instead of the link origin. Long chains with a TCP
        # offset pivot around the visible gripper instead of swinging it on the arm's
        # lever. Engine only forwards this when the client sent a non-zero offset and
        # the chain has enough DOF to satisfy the centroid-anchored task.
        tcp_offset_local=(
            tuple(float(v) for v in tcp_offset)
            if (tcp_offset is not None and use_tcp_offset)
            else None
        ),
    )

    # collision_aware breaks out early if a step introduces a new collision pair.
    # That manifests as iterations < max_iter and converged=False with the
    # joint state unchanged from the previous *successful* iteration. We
    # detect "collision-rejected" by checking whether the final residual is
    # roughly the same as it would have been with no motion. Cheap heuristic:
    # if no joint moved more than 1e-6 and we didn't converge, it was rejected.
    moved = any(
        abs(result.joint_values.get(jid, q_prev.get(jid, 0.0)) - q_prev.get(jid, 0.0))
        > 1e-6
        for jid in result.joint_values
    )
    collision_hit = ik.collision_aware and not moved and not result.converged

    for jid, q in result.joint_values.items():
        prev = q_prev.get(jid, 0.0)
        state.joint_values[jid] = float(q)
        state.joint_velocities[jid] = (float(q) - prev) / dt if dt > 0 else 0.0

    diag = SolverDiag(
        iters=int(result.iterations),
        pos_residual=float(result.pos_residual),
        rot_residual=float(result.rot_residual),
        converged=bool(result.converged),
        collision_hit=bool(collision_hit),
        target=tip,
    )

    if ik.debug:
        # One line per non-idle tick. Decoded enough that you can grep for
        # specific tips or correlate frontend deltas with engine motion.
        ovis_dump = (
            f"ovis pos=({ovis.position.x:+.3f},{ovis.position.y:+.3f},"
            f"{ovis.position.z:+.3f}) "
            f"ang(ypr)=({ovis.orientation.yaw:+.3f},{ovis.orientation.pitch:+.3f},"
            f"{ovis.orientation.roll:+.3f})"
        )
        tcp_dump = (
            f" tcp=({tcp_offset[0]:+.3f},{tcp_offset[1]:+.3f},{tcp_offset[2]:+.3f})"
            if tcp_offset is not None
            else " tcp=-"
        )
        tgt_dump = (
            f"target=({target_world[0]:+.3f},{target_world[1]:+.3f},"
            f"{target_world[2]:+.3f}) rot=({target_rot[0]:+.3f},{target_rot[1]:+.3f},"
            f"{target_rot[2]:+.3f},{target_rot[3]:+.3f})"
        )
        moved_max = max(
            (abs(result.joint_values.get(jid, q_prev.get(jid, 0.0)) - q_prev.get(jid, 0.0))
             for jid in result.joint_values),
            default=0.0,
        )
        _log.info(
            "tick tip=%s dt=%.3f %s%s frame=%s -> %s iters=%d "
            "pos_res=%.4f rot_res=%.4f conv=%s coll=%s moved_max=%.4f",
            tip, dt, ovis_dump, tcp_dump, ik.twist_frame, tgt_dump,
            result.iterations, result.pos_residual, result.rot_residual,
            result.converged, collision_hit, moved_max,
        )

    return _build_state_update(state, t, diag=diag, ee_tip=tip)


def _integrate_ovis(
    state: EngineState,
    ik: IKConfig,
    ovis: Ovis,
    dt: float,
    tcp_offset: np.ndarray | None,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Convert Ovis (normalised, possibly target-frame) into an absolute world
    target pose for `ovis.target` at the next tick. When `tcp_offset` is set,
    integration is anchored at the TCP (centroid) rather than the link origin
    so rotation pivots around the visible gripper, not its lever arm."""
    tip = ovis.target
    worlds = world_transforms(state.project, state.joint_values)
    current_T = worlds[tip]
    cur_link_pos = current_T[:3, 3]
    cur_R = current_T[:3, :3]
    cur_tcp = (
        cur_link_pos + cur_R @ tcp_offset
        if tcp_offset is not None
        else cur_link_pos
    )

    lin_step = np.array(
        [
            float(ovis.position.x) * ik.max_lin_vel * dt,
            float(ovis.position.y) * ik.max_lin_vel * dt,
            float(ovis.position.z) * ik.max_lin_vel * dt,
        ]
    )
    ang_step = (
        float(ovis.orientation.roll) * ik.max_ang_vel * dt,
        float(ovis.orientation.pitch) * ik.max_ang_vel * dt,
        float(ovis.orientation.yaw) * ik.max_ang_vel * dt,
    )
    delta_R = _rpy_to_rotmat(ang_step[2], ang_step[1], ang_step[0])

    if ik.twist_frame == "target":
        # Linear step in TCP's local frame.
        new_tcp = cur_tcp + cur_R @ lin_step
        new_R = cur_R @ delta_R
    else:  # "world"
        new_tcp = cur_tcp + lin_step
        new_R = delta_R @ cur_R

    new_quat = _rotmat_to_quat(new_R)
    # The IK solver consumes `target_world` paired with `tcp_offset_local` —
    # when tcp_offset is set, it interprets target_world as the centroid
    # target. When it's not, the link origin is the target.
    return (float(new_tcp[0]), float(new_tcp[1]), float(new_tcp[2])), new_quat


def _tcp_offset_from_ovis(ovis: Ovis) -> np.ndarray | None:
    """Read the optional TCP offset from an Ovis frame. Returns None when the
    field is unset or all-zero (so legacy clients keep "pivot at link origin")."""
    field = getattr(ovis, "tcp_offset_local", None)
    if field is None:
        return None
    x, y, z = float(field.x), float(field.y), float(field.z)
    if abs(x) < 1e-12 and abs(y) < 1e-12 and abs(z) < 1e-12:
        return None
    return np.array([x, y, z], dtype=float)


def _is_rotate_intent(ovis: Ovis) -> bool:
    """When |angular| >> |linear|, mirror the editor's rotate-gizmo IK mode."""
    lin = abs(ovis.position.x) + abs(ovis.position.y) + abs(ovis.position.z)
    ang = abs(ovis.orientation.yaw) + abs(ovis.orientation.pitch) + abs(ovis.orientation.roll)
    return ang > lin * 1.5


def _rpy_to_rotmat(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return Rz @ Ry @ Rx


def _rotmat_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def _build_state_update(
    state: EngineState,
    t: float,
    diag: SolverDiag | None,
    ee_tip: str = "",
) -> StateUpdate:
    msg = StateUpdate(t=t)
    for jid, q in state.joint_values.items():
        js = msg.joints.add()
        js.id = jid
        js.q = float(q)
        js.qdot = float(state.joint_velocities.get(jid, 0.0))

    tip = ee_tip or state.last_tip
    if tip and tip in state.project.scene.entities:
        worlds = world_transforms(state.project, state.joint_values)
        T = worlds.get(tip)
        if T is not None:
            pos = T[:3, 3]
            quat = _rotmat_to_quat(T[:3, :3])
            ee = msg.ee
            ee.tip = tip
            ee.position.x = float(pos[0])
            ee.position.y = float(pos[1])
            ee.position.z = float(pos[2])
            ee.orientation.x = quat[0]
            ee.orientation.y = quat[1]
            ee.orientation.z = quat[2]
            ee.orientation.w = quat[3]

    if diag is not None:
        msg.diag.iters = diag.iters
        msg.diag.pos_residual = diag.pos_residual
        msg.diag.rot_residual = diag.rot_residual
        msg.diag.converged = diag.converged
        msg.diag.collision_hit = diag.collision_hit
        msg.diag.target = diag.target

    return msg


def initialise_joint_values(state: EngineState) -> None:
    """Seed joint_values from project.home_pose or zero. Called once at startup."""
    project = state.project
    home = dict(project.home_pose) if project.home_pose else {}
    for eid, ent in project.scene.entities.items():
        joint = cast(JointComponent | None, ent.get("joint"))
        if joint is None or joint.type == "fixed":
            continue
        state.joint_values[eid] = float(home.get(eid, 0.0))
        state.joint_velocities[eid] = 0.0


# How stale a kinova frame can be before we stop trusting it. 0.5 s is long
# enough to ride out a brief sensor_api hiccup, short enough that a real
# disconnect doesn't leave the model showing a frozen pose.
_KINOVA_FRESH_S = 0.5


def _apply_kinova_mirror(state: EngineState) -> None:
    """Map fresh kinova reads into the model frame via the calibration
    captured at Sync. Idempotent — silently no-ops when there are no offsets
    (operator hasn't synced yet) or the latest frame is too stale.

    Note: this runs *before* the IK step in `tick`. If the user is dragging
    the gizmo, IK writes new joint values *after* the mirror, so the gizmo
    drag wins for this tick. Next tick, mirror writes again — meaning that
    while no velocity command is reaching kinova, the chain visually snaps
    back to wherever kinova is pointing. Once the velocity-out path is
    wired, kinova will actually move and mirror will track it."""
    if not state.kinova_offsets or not state.kinova_chain_joint_ids:
        return
    if state.latest_kinova_positions is None:
        return
    if time.monotonic() - state.latest_kinova_t > _KINOVA_FRESH_S:
        return

    positions = state.latest_kinova_positions
    for i, eid in enumerate(state.kinova_chain_joint_ids):
        if i >= len(positions):
            break
        offset = state.kinova_offsets.get(eid, 0.0)
        state.joint_values[eid] = float(positions[i]) - offset
