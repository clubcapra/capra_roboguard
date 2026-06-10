"""PyBullet damped-least-squares IK (default resolver, S7).

calculateInverseKinematics returns angles for *all* movable joints in DOF order
(ascending joint index). We map that vector back to the arm's joints by name so
only the arm chain is commanded.
"""
from __future__ import annotations

from typing import Dict, Optional, Sequence

import pybullet as p

from .base import IKResolver, register


@register("pybullet")
class PyBulletIK(IKResolver):
    def __init__(self, robot, ee_link, arm_links, **params):
        super().__init__(robot, ee_link, arm_links, **params)
        self.ee_index = robot.link_index[ee_link]
        # DOF order: movable (non-fixed) joints, ascending index
        self.dof_joints = sorted(set(robot.movable_joint.values()))
        self.arm_joints = [robot.movable_joint[l] for l in arm_links]
        self.rest = [0.0] * len(self.dof_joints)

    def solve(self, ee_pos, ee_orn: Optional[Sequence[float]],
              current: Dict[int, float]) -> Dict[int, float]:
        # Restrict the solve to the ARM chain: any other movable joint (the
        # gripper's finger linkage -- a fixed payload branching off the EE that
        # can't move it anyway) is pinned to its current angle via tight limits,
        # so the solver leaves it alone instead of diluting the arm solution.
        arm = set(self.arm_joints)
        ll, ul, jr, rp = [], [], [], []
        for j in self.dof_joints:
            info = p.getJointInfo(self.robot.body_id, j)
            lo, hi = info[8], info[9]
            if j in arm:
                if lo >= hi:                     # continuous / unlimited
                    lo, hi = -3.14159, 3.14159
                ll.append(lo); ul.append(hi); jr.append(hi - lo); rp.append(0.0)
            else:                                # pin gripper / other DOF
                q = p.getJointState(self.robot.body_id, j)[0]
                ll.append(q - 1e-4); ul.append(q + 1e-4)
                jr.append(2e-4); rp.append(q)
        kw = dict(maxNumIterations=80, residualThreshold=1e-4,
                  lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp)
        if ee_orn is not None:
            sol = p.calculateInverseKinematics(
                self.robot.body_id, self.ee_index, ee_pos, ee_orn, **kw)
        else:
            sol = p.calculateInverseKinematics(
                self.robot.body_id, self.ee_index, ee_pos, **kw)
        by_joint = {j: sol[i] for i, j in enumerate(self.dof_joints)}
        return {j: by_joint[j] for j in self.arm_joints}
