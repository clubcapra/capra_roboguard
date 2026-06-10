"""Rove IK resolver -- wraps the production `rove_ik_engine` (forgebot) solver.

The user's real arm runs `forgebot.core.kinematics.solve_position_ik`, the same
math the editor and the on-robot engine use; it tracks position in the primary
task with orientation in the null-space (`pos_primary`), so a non-spherical-wrist
arm EXTENDS to a forward/down target instead of folding the way PyBullet's stock
DLS IK does. Selecting `ik: rove` in a profile gives the sim bit-similar arm
behaviour to the real robot.

We import the robot's URDF into a forgebot Project once, map the arm's movable
joints to PyBullet by URDF joint name, and each solve convert the world target
into the robot base frame (= forgebot world / the URDF root), call the solver
with the current joint state as the seed, and map the result back to PyBullet
joint indices. The gripper is a fixed payload branch and simply isn't in the
Base->EE chain, so it never perturbs the solve.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Dict, Optional, Sequence

import pybullet as p

from .base import IKResolver, register

# make the vendored forgebot importable (its own repo sits next to rove_sim)
_ENGINE = os.path.normpath(os.path.join(os.path.dirname(__file__),
                                        "..", "..", "..", "..", "rove_ik_engine"))
if os.path.isdir(_ENGINE) and _ENGINE not in sys.path:
    sys.path.insert(0, _ENGINE)


@register("rove")
class RoveIK(IKResolver):
    def __init__(self, robot, ee_link, arm_links, **params):
        super().__init__(robot, ee_link, arm_links, **params)
        from forgebot.io.importers.urdf_importer import URDFImporter
        from forgebot.core.kinematics import solve_position_ik
        self._solve = solve_position_ik

        res = URDFImporter().import_file(Path(robot.profile.model.path))
        self.proj = res.project
        ents = self.proj.scene.entities
        byname = {getattr(e, "name", None): eid for eid, e in ents.items()}
        self.base_id = byname[params.get("ik_base", arm_links[0])]
        self.tip_id = byname[ee_link]

        # PyBullet arm movable-joint index <-> forgebot joint id, by URDF name
        idx_to_name = {v: k for k, v in robot.joint_index.items()}
        self.fb_of_idx, self.idx_of_fb = {}, {}
        for link in arm_links:
            pj = robot.movable_joint.get(link)
            fb = byname.get(idx_to_name.get(pj))
            if pj is not None and fb is not None:
                self.fb_of_idx[pj] = fb
                self.idx_of_fb[fb] = pj

        # Defaults mirror rove_ik_engine/engine/ik_loop.py (the on-robot + editor
        # solver), which is the tuning the user is happy with. mode is set
        # per-solve by the actuator (pose_locked for translate, pos_primary for
        # rotate); osg pairs with it.
        self.mode = params.get("mode", "pose_locked")
        self.osg = float(params.get("orientation_secondary_gain", 0.5))
        self.max_iter = int(params.get("max_iter", 60))
        self.damping = float(params.get("damping", 0.05))
        self.rest_gain = float(params.get("rest_pose_gain", 0.30))
        self.orient_w = float(params.get("orientation_weight", 5.0))
        self.max_dq_step = float(params.get("max_dq_step", 0.05))
        self.max_pos_step = float(params.get("max_pos_step", 0.05))
        mt = params.get("max_total_dq_step", 0.10)
        self.max_total_dq = None if mt in (None, 0) else float(mt)
        # The engine's `respect_collisions` baselines rest-pose contacts and
        # rejects steps that add new collision pairs. It's OFF here by default:
        # the forgebot Project from a URDF import only has the ARM joints in the
        # solve, so the gripper/flippers default to 0 and it reports SPURIOUS
        # collisions that freeze valid moves (vz "up" started dropping the arm).
        # The sim's own SelfCollisionGuard (real-mesh FCL, full robot state) is the
        # reliable collision-avoider -- it refuses any IK result that would self-
        # collide. Set `respect_collisions: true` to also use the forgebot check.
        self.respect_collisions = bool(params.get("respect_collisions", False))

    def set_mode(self, mode, osg):
        """Per-solve mode swap (the actuator calls this from the twist intent)."""
        self.mode, self.osg = mode, float(osg)

    def solve(self, ee_pos, ee_orn: Optional[Sequence[float]],
              current: Dict[int, float]) -> Dict[int, float]:
        # world target -> robot base frame (forgebot world == URDF root frame)
        bpos, born = p.getBasePositionAndOrientation(self.robot.body_id)
        inv_bp, inv_bo = p.invertTransform(bpos, born)
        tp, to = p.multiplyTransforms(inv_bp, inv_bo, ee_pos,
                                      ee_orn if ee_orn is not None else [0, 0, 0, 1])
        seed = {self.fb_of_idx[i]: v for i, v in current.items()
                if i in self.fb_of_idx}
        r = self._solve(
            self.proj, base=self.base_id, tip=self.tip_id,
            target_world=tuple(tp),
            target_rotation=tuple(to) if ee_orn is not None else None,
            initial_joint_values=seed,
            rest_pose=seed,                  # pull toward current -> no null-space wander
            rest_pose_gain=self.rest_gain,
            joint_weight_strength=0.0,
            mode=self.mode, orientation_secondary_gain=self.osg,
            max_iter=self.max_iter, damping=self.damping,
            orientation_weight=self.orient_w,
            respect_collisions=self.respect_collisions,
            max_dq_step=self.max_dq_step, max_pos_step=self.max_pos_step,
            max_total_dq_step=self.max_total_dq)
        out = {self.idx_of_fb[fb]: v for fb, v in r.joint_values.items()
               if fb in self.idx_of_fb}
        # any arm joint the solver didn't move keeps its current value
        for i, v in current.items():
            out.setdefault(i, v)
        return out
