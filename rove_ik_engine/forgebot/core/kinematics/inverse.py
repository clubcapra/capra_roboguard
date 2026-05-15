from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..model import JointComponent, Project
from .chain import extract_chain
from .forward import world_transforms


@dataclass
class IKResult:
    joint_values: dict[str, float]
    iterations: int
    residual: float
    converged: bool
    pos_residual: float = 0.0
    rot_residual: float = 0.0


def solve_position_ik(
    project: Project,
    *,
    base: str,
    tip: str,
    target_world: tuple[float, float, float],
    target_rotation: tuple[float, float, float, float] | None = None,
    initial_joint_values: dict[str, float] | None = None,
    rest_pose: dict[str, float] | None = None,
    rest_pose_gain: float = 0.05,
    joint_limit_avoidance: float = 0.5,
    joint_weight_strength: float = 1.0,
    respect_collisions: bool = False,
    max_iter: int = 100,
    tol: float = 1e-4,
    damping: float = 0.02,
    step: float = 1.0,
    orientation_weight: float = 1.0,
    max_pos_step: float = 0.15,
    max_rot_step: float = 0.30,
    max_dq_step: float = 0.20,
    max_total_dq_step: float | None = None,
    mode: str = "pose_locked",
    orientation_secondary_gain: float = 1.0,
    tcp_offset_local: tuple[float, float, float] | None = None,
) -> IKResult:
    """`mode`:
      - "pose_locked": position+orientation are both in the primary task.
        Hard-locks orientation. Best for spherical-wrist arms.
      - "pos_primary": position is the primary task; orientation enters as a
        null-space secondary. Position always tracks; orientation tracks
        wherever the chain has redundancy. Best for non-spherical-wrist arms
        where pose_locked can stall or jump configurations near singularities.

    `orientation_secondary_gain` only applies in pos_primary mode (gain on the
    null-space orientation pull).

    `max_total_dq_step` caps `||q_final - q_initial||_inf` per IK call. None
    = unlimited. Set to a small number (e.g. 0.05 rad ≈ 3°) to make a single
    call's effect bounded — the chain takes many small calls to escape a
    singularity instead of one big jump. This is the knob that turns
    pose_locked from "explodes from home" into "smooth jog with locked
    orientation" on non-spherical-wrist arms.
    """
    chain = extract_chain(project, base, tip)
    movable = [
        jid
        for jid in chain.joints
        if _movable_joint_axis(project, jid) is not None
    ]
    has_rotation_target = target_rotation is not None
    # In pose_locked mode, orientation lives in the primary Jacobian rows.
    # In pos_primary mode, orientation is a null-space secondary, so the
    # primary task is position-only (3 rows).
    pose_mode = has_rotation_target and mode != "pos_primary"
    target_pos = np.array(target_world, dtype=float)
    target_R = _quat_to_rotmat(target_rotation) if has_rotation_target else None

    initial_joint_values = {
        jid: float(v)
        for jid, v in (initial_joint_values or {}).items()
        if math.isfinite(v)
    }
    if rest_pose is not None:
        rest_pose = {
            jid: float(v) for jid, v in rest_pose.items() if math.isfinite(v)
        } or None

    if not movable:
        worlds = world_transforms(project, initial_joint_values or {})
        residual, pos_r, rot_r = _pose_residuals(
            worlds[tip], target_pos, target_R, orientation_weight
        )
        return IKResult(
            joint_values=dict(initial_joint_values or {}),
            iterations=0,
            residual=residual,
            converged=residual < tol,
            pos_residual=pos_r,
            rot_residual=rot_r,
        )

    external: dict[str, float] = dict(initial_joint_values or {})
    q: dict[str, float] = {jid: float(external.get(jid, 0.0)) for jid in movable}
    q_initial: dict[str, float] = dict(q)

    n = len(movable)
    rows = 6 if pose_mode else 3
    damp_sq = damping * damping
    last_residual = float("inf")
    last_pos_residual = float("inf")
    last_rot_residual = 0.0
    sv_threshold = 0.05

    alpha = max(0.0, min(1.0, joint_weight_strength))
    tapered = np.array(
        [max(1.0, 50.0 * (0.4 ** depth)) for depth in range(n)],
        dtype=float,
    )
    weights = (1.0 - alpha) * np.ones(n) + alpha * tapered
    w_inv = 1.0 / weights
    s_w = np.sqrt(w_inv)

    baseline_collisions: frozenset[tuple[str, str]] = frozenset()
    if respect_collisions:
        from ..validation.collision import check_collisions as _check_coll
        baseline_collisions = frozenset(
            tuple(sorted((p.a, p.b)))
            for p in _check_coll(project, joint_values={**external, **q})
        )

    tcp_off = (
        np.array(tcp_offset_local, dtype=float)
        if tcp_offset_local is not None
        else None
    )

    for it in range(max_iter):
        worlds = world_transforms(project, {**external, **q})
        tip_T = worlds[tip]
        tip_pos = tip_T[:3, 3]
        tip_R = tip_T[:3, :3]
        # Position task targets the TCP (tool center point) when an offset is
        # provided. Lets the gizmo "rotate around the gripper" instead of
        # around the link origin — critical when the link origin is far
        # from the visible mesh (rove_standard's gripper has a ~1.4m gap
        # between link and centroid).
        if tcp_off is not None:
            ee_pos = tip_pos + tip_R @ tcp_off
        else:
            ee_pos = tip_pos

        pos_err = target_pos - ee_pos
        if has_rotation_target:
            assert target_R is not None
            rot_err_raw = _matrix_log_so3(target_R @ tip_R.T)
        else:
            rot_err_raw = np.zeros(3)

        if pose_mode:
            unscaled = np.concatenate([pos_err, rot_err_raw])
        else:
            unscaled = pos_err

        residual = float(np.linalg.norm(unscaled))
        pos_residual = float(np.linalg.norm(pos_err))
        rot_residual = float(np.linalg.norm(rot_err_raw))
        last_residual = residual
        last_pos_residual = pos_residual
        last_rot_residual = rot_residual
        if residual < tol and (mode != "pos_primary" or rot_residual < tol):
            return IKResult(
                joint_values=q,
                iterations=it,
                residual=residual,
                converged=True,
                pos_residual=pos_residual,
                rot_residual=rot_residual,
            )

        pos_norm = float(np.linalg.norm(pos_err))
        if pos_norm > max_pos_step:
            pos_err = pos_err * (max_pos_step / pos_norm)
        rot_norm = float(np.linalg.norm(rot_err_raw))
        if rot_norm > max_rot_step:
            rot_err_raw = rot_err_raw * (max_rot_step / rot_norm)
        if pose_mode:
            err = np.concatenate([pos_err, rot_err_raw * orientation_weight])
        else:
            err = pos_err

        # Always compute the rotation Jacobian rows; pose_locked uses them in
        # the primary task (J), pos_primary uses them as a null-space secondary.
        J = np.zeros((rows, n))
        J_rot = np.zeros((3, n))
        for i, jid in enumerate(movable):
            joint_world = worlds[jid]
            joint_pos = joint_world[:3, 3]
            jcomp = _joint_comp(project, jid)
            axis_local = np.array(jcomp.axis, dtype=float)
            axis_world = joint_world[:3, :3] @ axis_local
            anorm = np.linalg.norm(axis_world)
            if anorm > 1e-12:
                axis_world = axis_world / anorm
            if jcomp.type in ("revolute", "continuous"):
                J[0:3, i] = np.cross(axis_world, ee_pos - joint_pos)
                J_rot[:, i] = axis_world
                if pose_mode:
                    J[3:6, i] = orientation_weight * axis_world
            elif jcomp.type == "prismatic":
                J[0:3, i] = axis_world
                # Prismatic joints don't contribute to rotation; J_rot row stays 0.
            # An inverted joint moves opposite the URDF axis for positive
            # slider, so d(end_effector)/d(slider) flips sign on every row.
            if getattr(jcomp, "inverted", False):
                J[:, i] = -J[:, i]
                J_rot[:, i] = -J_rot[:, i]

        J_w = J * s_w
        try:
            U, sigma, Vt = np.linalg.svd(J_w, full_matrices=False)
        except np.linalg.LinAlgError:
            break
        lam_sq = damp_sq + np.maximum(0.0, sv_threshold * sv_threshold - sigma * sigma)
        sigma_inv = sigma / (sigma * sigma + lam_sq)
        u_primary = Vt.T @ (sigma_inv * (U.T @ err))
        dq = s_w * u_primary

        secondary = np.zeros(n)
        q_vec = np.array([q[jid] for jid in movable])
        if rest_pose and rest_pose_gain > 0.0:
            rest_vec = np.array([float(rest_pose.get(jid, q[jid])) for jid in movable])
            secondary = secondary + rest_pose_gain * (rest_vec - q_vec)
        # Task-priority IK for orientation in pos_primary mode: solve
        # `dq_null = (J_rot @ N_pos)^# @ rot_err` directly, where N_pos is
        # the null-space projector of the position task. This minimizes
        # orientation error *within* the null space of position — gives an
        # exact (damped-LS) Newton step rather than a slow gradient.
        # Computed below after we have the null-space projector from the
        # primary SVD; see post-projection block.
        rot_secondary_pending = (
            mode == "pos_primary"
            and has_rotation_target
            and orientation_secondary_gain > 0.0
        )
        if joint_limit_avoidance > 0.0:
            for i, jid in enumerate(movable):
                jcomp = _joint_comp(project, jid)
                if jcomp.limits is None or jcomp.type not in ("revolute", "prismatic"):
                    continue
                lo, hi = jcomp.limits.lower, jcomp.limits.upper
                if hi <= lo:
                    continue
                center = 0.5 * (lo + hi)
                half = 0.5 * (hi - lo)
                norm = (q_vec[i] - center) / half
                secondary[i] -= joint_limit_avoidance * (norm ** 3) * half
        if np.any(secondary != 0):
            secondary_u = secondary / s_w
            eff = sigma > sv_threshold
            if np.any(eff):
                V_eff_t = Vt[eff]
                proj_u = secondary_u - V_eff_t.T @ (V_eff_t @ secondary_u)
            else:
                proj_u = secondary_u
            dq = dq + s_w * proj_u

        # Task-priority orientation: solve for the joint-space step that
        # reduces orientation error *within* the null space of the position
        # task. This is `dq_rot = (J_rot @ N_pos)^# @ rot_err`. Distinct
        # from the `secondary` block above (which is a Liegeois-style
        # post-projection of arbitrary secondary objectives) because for a
        # Cartesian rotation task the projection-then-solve order matters:
        # solving in null space gives an exact damped Newton step instead
        # of a slow gradient.
        if rot_secondary_pending:
            eff = sigma > sv_threshold
            if np.any(eff):
                V_eff = Vt[eff].T  # (n, k_eff) — basis of row(J_pos)
                # Null-space projector in WEIGHTED coords (the SVD basis).
                N_w = np.eye(n) - V_eff @ V_eff.T
            else:
                N_w = np.eye(n)
            J_rot_w = J_rot * s_w  # (3, n) weighted rot Jacobian
            J_rot_proj = J_rot_w @ N_w
            try:
                U_r, sig_r, Vt_r = np.linalg.svd(J_rot_proj, full_matrices=False)
            except np.linalg.LinAlgError:
                pass
            else:
                lam_r_sq = damp_sq + np.maximum(
                    0.0, sv_threshold * sv_threshold - sig_r * sig_r
                )
                sig_r_inv = sigma_inv_safe = sig_r / (sig_r * sig_r + lam_r_sq)
                u_rot = Vt_r.T @ (sigma_inv_safe * (U_r.T @ rot_err_raw))
                dq_rot = s_w * u_rot
                dq = dq + orientation_secondary_gain * dq_rot

        dq = step * dq

        dq_inf = float(np.max(np.abs(dq))) if dq.size else 0.0
        if dq_inf > max_dq_step:
            dq = dq * (max_dq_step / dq_inf)

        candidate_q = dict(q)
        for i, jid in enumerate(movable):
            jcomp = _joint_comp(project, jid)
            new_val = q[jid] + float(dq[i])
            if jcomp.limits is not None and jcomp.type in ("revolute", "prismatic"):
                lo, hi = jcomp.limits.lower, jcomp.limits.upper
                if hi > lo:
                    new_val = max(lo, min(hi, new_val))
            candidate_q[jid] = new_val

        if respect_collisions:
            from ..validation.collision import check_collisions as _check_coll
            new_pairs = frozenset(
                tuple(sorted((p.a, p.b)))
                for p in _check_coll(project, joint_values={**external, **candidate_q})
            )
            if new_pairs - baseline_collisions:
                break

        # Per-call cap on cumulative joint motion. If `candidate_q` would push
        # us past the cap, scale the diff back to the cap and stop iterating.
        # This is what makes pose_locked mode usable on non-spherical-wrist
        # arms — the chain takes many small calls to escape a singularity
        # instead of one big jump.
        if max_total_dq_step is not None and max_total_dq_step > 0:
            diff_inf = 0.0
            for jid in movable:
                d = abs(candidate_q[jid] - q_initial[jid])
                if d > diff_inf:
                    diff_inf = d
            if diff_inf > max_total_dq_step:
                scale = max_total_dq_step / diff_inf
                for jid in movable:
                    candidate_q[jid] = q_initial[jid] + scale * (candidate_q[jid] - q_initial[jid])
                q = candidate_q
                # Recompute residuals at the clamped pose so the caller's HUD
                # reflects the actual achieved error, not the pre-clamp iter's.
                final_T = world_transforms(project, {**external, **q})[tip]
                pos_err_final = target_pos - final_T[:3, 3]
                pos_r = float(np.linalg.norm(pos_err_final))
                if has_rotation_target:
                    assert target_R is not None
                    rot_err_final = _matrix_log_so3(target_R @ final_T[:3, :3].T)
                    rot_r = float(np.linalg.norm(rot_err_final))
                else:
                    rot_err_final = np.zeros(3)
                    rot_r = 0.0
                if pose_mode:
                    res = float(np.linalg.norm(np.concatenate([pos_err_final, rot_err_final])))
                else:
                    res = pos_r
                return IKResult(
                    joint_values=q,
                    iterations=it + 1,
                    residual=res,
                    converged=False,
                    pos_residual=pos_r,
                    rot_residual=rot_r,
                )

        q = candidate_q

    return IKResult(
        joint_values=q,
        iterations=max_iter,
        residual=last_residual,
        converged=False,
        pos_residual=last_pos_residual,
        rot_residual=last_rot_residual,
    )


def _joint_comp(project: Project, jid: str) -> JointComponent:
    return project.scene.entities[jid].components["joint"]  # type: ignore[return-value]


def _movable_joint_axis(project: Project, jid: str) -> tuple[float, float, float] | None:
    j = _joint_comp(project, jid)
    if j.type not in ("revolute", "continuous", "prismatic"):
        return None
    return j.axis


def _pose_residual(
    tip_T: np.ndarray,
    target_pos: np.ndarray,
    target_R: np.ndarray | None,
    orientation_weight: float,
) -> float:
    return _pose_residuals(tip_T, target_pos, target_R, orientation_weight)[0]


def _pose_residuals(
    tip_T: np.ndarray,
    target_pos: np.ndarray,
    target_R: np.ndarray | None,
    orientation_weight: float,
) -> tuple[float, float, float]:
    pos_err = target_pos - tip_T[:3, 3]
    pos_norm = float(np.linalg.norm(pos_err))
    if target_R is None:
        return pos_norm, pos_norm, 0.0
    rot_err_raw = _matrix_log_so3(target_R @ tip_T[:3, :3].T)
    rot_norm = float(np.linalg.norm(rot_err_raw))
    weighted = float(
        np.linalg.norm(np.concatenate([pos_err, rot_err_raw * orientation_weight]))
    )
    return weighted, pos_norm, rot_norm


def _quat_to_rotmat(q: tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = (float(c) for c in q)
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ]
    )


def _matrix_log_so3(R: np.ndarray) -> np.ndarray:
    cos_angle = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) * 0.5))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    sin_angle = math.sin(angle)
    if sin_angle < 1e-9:
        diag = np.diagonal(R) + 1.0
        i = int(np.argmax(diag))
        if diag[i] <= 0:
            return np.zeros(3)
        col = (R + np.eye(3))[:, i]
        axis = col / math.sqrt(2.0 * diag[i])
        return angle * axis
    axis = (
        np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        / (2.0 * sin_angle)
    )
    return angle * axis
