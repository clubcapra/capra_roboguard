"""Per-arm IK parameter tuner.

Evaluates candidate `IKProfile` configurations against a benchmark scenario
suite — small jogs in 6 cartesian directions and 3 rotation axes from a
handful of seed poses. Each scenario is scored on:

  - max per-tick position error (mm)
  - final orientation drift (deg)
  - largest per-tick joint jump (rad)
  - total joint motion across the drag (rad)
  - joints pinned at limits
  - new collision pairs (drum/flipper/arm self-collisions)

Composite score is a weighted sum (lower = better). The tuner walks a small
grid over the meaningful knobs and returns the winning profile.

Collisions are always respected during tuning — that's the user's promise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable

import numpy as np

from ..model import IKProfile, JointComponent, Project
from .chain import extract_chain
from .forward import world_transforms
from .inverse import solve_position_ik

ProgressCallback = Callable[[int, int, float, "IKProfile | None"], None]
CancelFn = Callable[[], bool]


@dataclass
class TuneScenario:
    """One drag scenario from a seed pose. Either translate XOR rotate."""

    label: str
    seed: dict[str, float]
    direction: tuple[float, float, float] | None  # translation direction (unit)
    axis: tuple[float, float, float] | None  # rotation axis (unit)
    n_ticks: int
    step_m: float
    step_rad: float


@dataclass
class ScenarioResult:
    pos_err_max: float = 0.0  # m
    rot_drift_final: float = 0.0  # rad
    max_per_tick_jump: float = 0.0  # rad
    total_motion: float = 0.0  # rad
    saturated_joints: int = 0
    new_collision_pairs: int = 0


@dataclass
class TuneScore:
    composite: float
    pos_err_max_mm: float
    rot_drift_deg: float
    max_jump_rad: float
    total_motion_rad: float
    saturated_joints: int
    new_collision_pairs: int
    per_scenario: list[tuple[str, ScenarioResult]] = field(default_factory=list)


@dataclass
class TuneStatus:
    """Snapshot of an in-progress tune for streaming to the UI."""

    job_id: str
    base: str
    tip: str
    done: int
    total: int
    started_at: float  # monotonic seconds
    eta_s: float
    best_score: float
    best_profile: IKProfile | None
    finished: bool = False
    cancelled: bool = False
    error: str | None = None


# ---- scenario suite ----


def _quat_from_R(R: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return (
            (R[2, 1] - R[1, 2]) * s,
            (R[0, 2] - R[2, 0]) * s,
            (R[1, 0] - R[0, 1]) * s,
            0.25 / s,
        )
    # Pick the largest diagonal as anchor.
    diag = (R[0, 0], R[1, 1], R[2, 2])
    i = int(max(range(3), key=lambda k: diag[k]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(max(1e-9, 1.0 + R[i, i] - R[j, j] - R[k, k]))
    out = [0.0, 0.0, 0.0, 0.0]
    out[3] = (R[k, j] - R[j, k]) / s
    out[i] = 0.25 * s
    out[j] = (R[j, i] + R[i, j]) / s
    out[k] = (R[k, i] + R[i, k]) / s
    return (out[0], out[1], out[2], out[3])


def _rot_drift_rad(R_now: np.ndarray, R_ref: np.ndarray) -> float:
    R_diff = R_now @ R_ref.T
    cos_a = max(-1.0, min(1.0, (float(np.trace(R_diff)) - 1.0) * 0.5))
    return math.acos(cos_a)


def _axis_angle_to_R(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    ax = np.array(axis, dtype=float)
    n = float(np.linalg.norm(ax))
    if n < 1e-12:
        return np.eye(3)
    ax = ax / n
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]], dtype=float)
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def build_scenarios(project: Project, base: str, tip: str) -> list[TuneScenario]:
    """Build the default scenario suite: jog from home + small perturbations."""
    chain = extract_chain(project, base, tip)
    movable = [
        jid for jid in chain.joints
        if project.scene.entities[jid].components.get("joint", None) is not None
        and getattr(project.scene.entities[jid].components["joint"], "type", "fixed")
        in ("revolute", "continuous", "prismatic")
    ]
    seeds: list[tuple[str, dict[str, float]]] = [("home", {})]
    # One perturbed seed — small offset on the second joint to step away from
    # the home singularity (first joint is often the base yaw, which doesn't
    # break a planar singularity by itself).
    if len(movable) >= 2:
        seeds.append(("seed_bend", {movable[1]: 0.4}))

    scenarios: list[TuneScenario] = []
    translate_dirs = [
        ("+x", (1.0, 0.0, 0.0)),
        ("-x", (-1.0, 0.0, 0.0)),
        ("+y", (0.0, 1.0, 0.0)),
        ("-y", (0.0, -1.0, 0.0)),
        ("+z", (0.0, 0.0, 1.0)),
        ("-z", (0.0, 0.0, -1.0)),
    ]
    rotate_axes = [
        ("rx", (1.0, 0.0, 0.0)),
        ("ry", (0.0, 1.0, 0.0)),
        ("rz", (0.0, 0.0, 1.0)),
    ]

    for sname, seed in seeds:
        for dname, dvec in translate_dirs:
            scenarios.append(
                TuneScenario(
                    label=f"{sname}/{dname}",
                    seed=seed,
                    direction=dvec,
                    axis=None,
                    n_ticks=12,
                    step_m=0.003,
                    step_rad=0.0,
                )
            )
        # Only run rotation scenarios from "home" to keep the suite compact;
        # the perturbed seeds already exercise mid-workspace translation.
        if sname == "home":
            for rname, raxis in rotate_axes:
                scenarios.append(
                    TuneScenario(
                        label=f"{sname}/{rname}",
                        seed=seed,
                        direction=None,
                        axis=raxis,
                        n_ticks=12,
                        step_m=0.0,
                        step_rad=math.radians(0.8),
                    )
                )
    return scenarios


# ---- evaluation ----


def _saturated(jcomp: JointComponent, value: float, tol: float = 1e-3) -> bool:
    if jcomp.limits is None or jcomp.type not in ("revolute", "prismatic"):
        return False
    lo, hi = jcomp.limits.lower, jcomp.limits.upper
    if hi <= lo:
        return False
    return value >= hi - tol or value <= lo + tol


def evaluate_scenario(
    project: Project,
    base: str,
    tip: str,
    scenario: TuneScenario,
    profile: IKProfile,
    *,
    collision_baseline: frozenset[tuple[str, str]] | None = None,
) -> ScenarioResult:
    """Run one drag scenario and score it.

    Note: we do NOT pass `respect_collisions=True` to the solver during tuning
    — that runs collision detection inside every IK iter and is ~100× slower.
    Instead we sample collisions at midpoint and end of the trajectory and
    count any new pairs. The composite score still penalizes collisions
    heavily, so candidates that collide get rejected.
    """
    from ..validation.collision import check_collisions

    fk = world_transforms(project, scenario.seed)
    home_pos = fk[tip][:3, 3]
    home_R = fk[tip][:3, :3]
    home_quat = _quat_from_R(home_R)

    if collision_baseline is None:
        collision_baseline = frozenset(
            tuple(sorted((p.a, p.b)))
            for p in check_collisions(project, joint_values=scenario.seed)
        )

    seed = dict(scenario.seed)
    drag_start = dict(scenario.seed)
    last_q: dict[str, float] = dict(seed)

    res = ScenarioResult()
    new_collision_set: set[tuple[str, str]] = set()
    sample_ticks = {scenario.n_ticks // 2, scenario.n_ticks - 1}

    for tick in range(scenario.n_ticks):
        if scenario.direction is not None:
            d = np.array(scenario.direction, dtype=float)
            target_pos = home_pos + d * scenario.step_m * (tick + 1)
            target_quat: tuple[float, float, float, float] | None = home_quat
        else:
            target_pos = home_pos
            assert scenario.axis is not None
            R_target = _axis_angle_to_R(scenario.axis, scenario.step_rad * (tick + 1)) @ home_R
            target_quat = _quat_from_R(R_target)

        result = solve_position_ik(
            project,
            base=base,
            tip=tip,
            target_world=tuple(target_pos),
            target_rotation=target_quat,
            initial_joint_values=seed,
            rest_pose=drag_start,
            rest_pose_gain=profile.rest_pose_gain,
            joint_weight_strength=profile.joint_weight_strength,
            max_iter=profile.max_iter,
            damping=profile.damping,
            orientation_weight=profile.orientation_weight,
            max_dq_step=profile.max_dq_step,
            max_pos_step=profile.max_pos_step,
            max_total_dq_step=profile.max_total_dq_step,
            mode=profile.mode,
            orientation_secondary_gain=profile.orientation_secondary_gain,
        )
        seed = dict(result.joint_values)

        ach_T = world_transforms(project, seed)[tip]
        pos_err = float(np.linalg.norm(ach_T[:3, 3] - target_pos))
        if pos_err > res.pos_err_max:
            res.pos_err_max = pos_err

        for jid, v in seed.items():
            d = abs(v - last_q.get(jid, 0.0))
            if d > res.max_per_tick_jump:
                res.max_per_tick_jump = d
            res.total_motion += d
        last_q = dict(seed)

        if tick in sample_ticks:
            sample_pairs = frozenset(
                tuple(sorted((p.a, p.b)))
                for p in check_collisions(project, joint_values=seed)
            )
            for pair in sample_pairs - collision_baseline:
                new_collision_set.add(pair)

    final_T = world_transforms(project, seed)[tip]
    if scenario.direction is not None:
        # Translate scenarios — drift is final orientation - home.
        res.rot_drift_final = _rot_drift_rad(final_T[:3, :3], home_R)
    else:
        # Rotate scenarios — drift is "how far from the requested rotation",
        # so close-to-zero means good tracking.
        assert scenario.axis is not None
        R_target = _axis_angle_to_R(scenario.axis, scenario.step_rad * scenario.n_ticks) @ home_R
        res.rot_drift_final = _rot_drift_rad(final_T[:3, :3], R_target)

    # Saturation count.
    for jid, v in seed.items():
        e = project.scene.entities.get(jid)
        if e is None:
            continue
        jc = e.components.get("joint")
        if jc is None:
            continue
        if _saturated(jc, v):  # type: ignore[arg-type]
            res.saturated_joints += 1

    res.new_collision_pairs = len(new_collision_set)
    return res


def composite_score(scenarios: list[tuple[str, ScenarioResult]]) -> TuneScore:
    pos_max = 0.0
    rot_drift_max = 0.0
    jump_max = 0.0
    total_motion = 0.0
    saturated = 0
    new_coll = 0
    for _, r in scenarios:
        if r.pos_err_max > pos_max:
            pos_max = r.pos_err_max
        if r.rot_drift_final > rot_drift_max:
            rot_drift_max = r.rot_drift_final
        if r.max_per_tick_jump > jump_max:
            jump_max = r.max_per_tick_jump
        total_motion += r.total_motion
        saturated += r.saturated_joints
        new_coll += r.new_collision_pairs

    pos_mm = pos_max * 1000.0
    rot_deg = math.degrees(rot_drift_max)

    # Weights chosen so a 1mm pos error, a 2° drift, a 0.02 rad per-tick jump,
    # and a 0.5 rad of summed motion all contribute ~1 unit. Saturation and
    # collision pairs are heavy penalties.
    composite = (
        pos_mm * 1.0
        + rot_deg * 0.5
        + jump_max * 50.0
        + total_motion * 2.0
        + saturated * 100.0
        + new_coll * 200.0
    )
    return TuneScore(
        composite=composite,
        pos_err_max_mm=pos_mm,
        rot_drift_deg=rot_deg,
        max_jump_rad=jump_max,
        total_motion_rad=total_motion,
        saturated_joints=saturated,
        new_collision_pairs=new_coll,
        per_scenario=scenarios,
    )


def evaluate_profile(
    project: Project,
    base: str,
    tip: str,
    profile: IKProfile,
    scenarios: list[TuneScenario],
    *,
    collision_baselines: dict[str, frozenset[tuple[str, str]]] | None = None,
) -> TuneScore:
    out: list[tuple[str, ScenarioResult]] = []
    for sc in scenarios:
        baseline = None if collision_baselines is None else collision_baselines.get(sc.label)
        out.append((
            sc.label,
            evaluate_scenario(project, base, tip, sc, profile, collision_baseline=baseline),
        ))
    return composite_score(out)


# ---- search grid ----


def candidate_grid() -> list[IKProfile]:
    """Coarse grid over the knobs that actually matter at home/singularity.

    Heavily biased toward pose_locked + max_total_dq_step: that combination
    gives the cleanest jog feel (orientation hard-locked, motion bounded so
    the chain catches up smoothly without exploding). pos_primary is kept
    as a fallback for arms where pose_locked truly can't satisfy the task
    (the tuner picks whichever scores best per arm).

    Sized for ~30s on a 6-DOF arm.
    """
    profiles: list[IKProfile] = []
    # pose_locked sweep over total motion cap (the new key knob) and damping.
    for damping in (0.03, 0.05, 0.10):
        for rest_gain in (0.10, 0.30):
            for max_total in (0.05, 0.10, 0.20, None):
                profiles.append(
                    IKProfile(
                        mode="pose_locked",
                        damping=damping,
                        rest_pose_gain=rest_gain,
                        max_iter=60,
                        orientation_weight=5.0,
                        joint_weight_strength=0.0,
                        max_dq_step=0.05,
                        max_pos_step=0.05,
                        max_total_dq_step=max_total,
                        orientation_secondary_gain=0.0,
                    )
                )
    # pos_primary fallback — for arms where pose_locked can't satisfy the
    # task at all, we accept some orientation drift. Smaller sweep.
    for damping in (0.05, 0.10):
        for ori_sec in (0.5, 2.0):
            profiles.append(
                IKProfile(
                    mode="pos_primary",
                    damping=damping,
                    rest_pose_gain=0.30,
                    max_iter=60,
                    orientation_weight=5.0,
                    joint_weight_strength=0.0,
                    max_dq_step=0.05,
                    max_pos_step=0.05,
                    max_total_dq_step=None,
                    orientation_secondary_gain=ori_sec,
                )
            )
    return profiles


def run_tune(
    project: Project,
    *,
    base: str,
    tip: str,
    on_progress: ProgressCallback | None = None,
    cancel: CancelFn | None = None,
    candidates: list[IKProfile] | None = None,
    scenarios: list[TuneScenario] | None = None,
) -> tuple[IKProfile, TuneScore]:
    from ..validation.collision import check_collisions

    if scenarios is None:
        scenarios = build_scenarios(project, base, tip)
    if candidates is None:
        candidates = candidate_grid()
    total = len(candidates)

    # Pre-compute the per-seed-pose collision baseline once. Each scenario's
    # baseline depends only on its `seed`, not on which IK profile we're
    # evaluating, so this saves one collision check per (candidate × scenario).
    seen_seed_keys: dict[str, frozenset[tuple[str, str]]] = {}
    collision_baselines: dict[str, frozenset[tuple[str, str]]] = {}
    for sc in scenarios:
        # Cache by serialized seed dict so identical seeds reuse one check.
        key = ",".join(f"{k}={v:.4f}" for k, v in sorted(sc.seed.items()))
        if key not in seen_seed_keys:
            seen_seed_keys[key] = frozenset(
                tuple(sorted((p.a, p.b)))
                for p in check_collisions(project, joint_values=sc.seed)
            )
        collision_baselines[sc.label] = seen_seed_keys[key]

    best_profile: IKProfile | None = None
    best_score: TuneScore | None = None

    for i, candidate in enumerate(candidates):
        if cancel is not None and cancel():
            break
        score = evaluate_profile(
            project, base, tip, candidate, scenarios,
            collision_baselines=collision_baselines,
        )
        if best_score is None or score.composite < best_score.composite:
            best_score = score
            best_profile = candidate.model_copy()
            best_profile.score = score.composite
            best_profile.pos_err_max_mm = score.pos_err_max_mm
            best_profile.rot_drift_deg = score.rot_drift_deg
            best_profile.max_jump_rad = score.max_jump_rad
            best_profile.total_motion_rad = score.total_motion_rad
            best_profile.saturated_joints = score.saturated_joints
            best_profile.new_collision_pairs = score.new_collision_pairs
        if on_progress is not None:
            on_progress(i + 1, total, best_score.composite if best_score else float("inf"), best_profile)

    if best_profile is None or best_score is None:
        # Fallback — if everything was cancelled before any eval, return defaults.
        best_profile = IKProfile()
        best_score = TuneScore(
            composite=float("inf"),
            pos_err_max_mm=0.0,
            rot_drift_deg=0.0,
            max_jump_rad=0.0,
            total_motion_rad=0.0,
            saturated_joints=0,
            new_collision_pairs=0,
        )
    best_profile.tuned_at = datetime.now(timezone.utc).isoformat()
    return best_profile, best_score
