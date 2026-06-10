"""Arm control: Ovis twist -> EE pose delta -> IK -> joint targets (S3, S5.3).

ovis is intent, never per-actuator setpoints: the 6-DOF twist is integrated into
a target end-effector pose, resolved through the IK seam (S7), and the arm joints
are held with POSITION_CONTROL.

The target is tracked in the ROBOT BASE frame, not world: when the robot drives
or turns, the arm holds its pose relative to the body and moves with it. (A
world-frame target gets left behind as the robot moves, so the IK flails trying
to reach a point the robot already drove away from -- "wobbling in the wind".)

Per-joint holding torque matches the real arm: the base and shoulder motors hold
at ~12 N.m, the wrist motors at ~3.6 N.m. A velocity cap keeps motion smooth.
"""
from __future__ import annotations

import json
import os

import pybullet as p

from .base import Actuator, register
from ..ik.base import IK_REGISTRY
from ..ik import pybullet_ik  # noqa: F401  (registers "pybullet")
try:
    from ..ik import rove_ik  # noqa: F401  (registers "rove" if forgebot present)
except Exception:             # forgebot/pydantic not installed -> "rove" absent
    pass


@register("arm_ik")
class ArmIK(Actuator):
    intent_field = "ovis"

    def _resolve_joints(self):
        self.guard = None              # optional SelfCollisionGuard (set by runtime)
        links = list(self.bind)
        self.arm_link_names = list(links)
        self.ee_link = self.params.get("ee_link", links[-1])
        self.arm_joints = [self.robot.movable_joint[l] for l in links]
        self.rate_hz = float(self.params.get("rate_hz", 75.0))
        self.dt = 1.0 / self.rate_hz
        self.lin_speed = float(self.params.get("lin_speed_mps", 0.15))
        self.ang_speed = float(self.params.get("ang_speed_rps", 0.6))
        self.max_vel = float(self.params.get("max_vel_rad_s", 1.0))
        self.reach = float(self.params.get("workspace_m", 1.2))

        # per-joint holding torque (N.m): base + shoulder strong, wrist weak.
        strong = float(self.params.get("base_torque", 12.0))
        weak = float(self.params.get("wrist_torque", 3.6))
        strong_links = set(self.params.get("strong_links", links[:2]))
        self.force = {self.robot.movable_joint[l]:
                      (strong if l in strong_links else weak) for l in links}

        # forward the profile params to the resolver (ik_base, gains, step caps,
        # ...) -- without this they were silently dropped and ik_base defaulted to
        # the first arm link, excluding the Base yaw joint from the chain.
        ik_params = {k: v for k, v in self.params.items()
                     if k not in ("ik", "ee_link")}
        self.ik = IK_REGISTRY.build(self.params.get("ik", "pybullet"),
                                    robot=self.robot, ee_link=self.ee_link,
                                    arm_links=links, **ik_params)
        # HOME pose: the URDF rest has the arm nearly fully extended (elbow ~11deg
        # -> "we will break it", and no room to deploy forward since forward only
        # extends it further). Fold it to a safe retracted home so commands deploy
        # from there. `home_pose`: {link: angle}.
        for link, ang in (self.params.get("home_pose") or {}).items():
            if link in self.robot.movable_joint:
                p.resetJointState(self.robot.body_id,
                                  self.robot.movable_joint[link], float(ang))

        # CONTROL POINT (TCP) = the GRIPPER PINCH centre: the operator drives by
        # the pinch, so translation moves it and rotation pivots about it (Ovis).
        # `tcp_links` (e.g. the two finger pads) -> their midpoint; else the EE
        # link COM. Stored as `com_local` in the EE-link frame (constant: the pads
        # move symmetrically so their midpoint offset is fixed as the gripper
        # opens/closes). The IK targets the EE link, recovered from this offset.
        ee = self.robot.link_index[self.ee_link]
        st = p.getLinkState(self.robot.body_id, ee, computeForwardKinematics=True)
        frame_pos, frame_orn = st[4], st[5]
        self.tcp_links = [l for l in (self.params.get("tcp_links") or [])
                          if l in self.robot.link_index]
        inv_fp, inv_fo = p.invertTransform(frame_pos, frame_orn)
        self.com_local, _ = p.multiplyTransforms(inv_fp, inv_fo, self._tcp_world(),
                                                 [0, 0, 0, 1])
        # target (the TCP pose) stored RELATIVE to the base, so the arm holds with
        # the body (no wobble/drift when the robot drives or turns).
        self.rel_pos, self.rel_eul = self._current_tcp_rel()
        self.joint_targets = {j: p.getJointState(self.robot.body_id, j)[0]
                              for j in self.arm_joints}

        # named-pose store + joint-space path planning (the "go to pose" feature).
        self._poses_file = self.params.get("poses_file") or os.path.join(
            os.path.dirname(self.robot.profile.model.path),
            f"{self.robot.profile.name}_arm_poses.json")
        self.poses = self._load_poses()
        self.goto_speed = float(self.params.get("goto_speed_rad_s", 0.6))
        self._goto = None        # active plan: (target {joint: q}, speed) or None

    # -- TCP helpers --------------------------------------------------------
    def _tcp_world(self):
        """World position of the control point (gripper pinch centre = midpoint
        of the finger pads; else the EE link COM)."""
        ee = self.robot.link_index[self.ee_link]
        if self.tcp_links:
            pts = [p.getLinkState(self.robot.body_id, self.robot.link_index[l],
                                  computeForwardKinematics=True)[4]
                   for l in self.tcp_links]
            return [sum(c) / len(pts) for c in zip(*pts)]
        return p.getLinkState(self.robot.body_id, ee, computeForwardKinematics=True)[0]

    def _current_tcp_rel(self):
        """The current TCP pose (pos, euler) in the robot base frame."""
        ee = self.robot.link_index[self.ee_link]
        frame_orn = p.getLinkState(self.robot.body_id, ee,
                                   computeForwardKinematics=True)[5]
        bpos, born = p.getBasePositionAndOrientation(self.robot.body_id)
        inv_bp, inv_bo = p.invertTransform(bpos, born)
        rel_p, rel_o = p.multiplyTransforms(inv_bp, inv_bo, self._tcp_world(), frame_orn)
        return list(rel_p), list(p.getEulerFromQuaternion(rel_o))

    # -- pose store + path planning -----------------------------------------
    def store_pose(self, name):
        """Save the current arm joint configuration under `name` (persisted)."""
        self.poses[str(name)] = {l: p.getJointState(
            self.robot.body_id, self.robot.movable_joint[l])[0]
            for l in self.arm_link_names}
        self._save_poses()
        return self.poses[str(name)]

    def goto_pose(self, name, speed=None):
        """Plan + execute a joint-space path to stored pose `name`. Returns False
        if unknown. Motion runs over subsequent control ticks; a manual twist or
        a self-collision stops it."""
        name = str(name)
        if name not in self.poses:
            return False
        target = {self.robot.movable_joint[l]: float(q)
                  for l, q in self.poses[name].items()
                  if l in self.robot.movable_joint}
        self._goto = (target, float(speed) if speed else self.goto_speed)
        return True

    @property
    def planning(self):
        return self._goto is not None

    def _step_goto(self):
        target, speed = self._goto
        step = speed * self.dt
        nxt, done = dict(self.joint_targets), True
        for j, tgt in target.items():
            cur = self.joint_targets.get(j, p.getJointState(self.robot.body_id, j)[0])
            if abs(tgt - cur) > step:
                nxt[j] = cur + step * (1.0 if tgt > cur else -1.0)
                done = False
            else:
                nxt[j] = tgt
        # collaborative arm: never plan THROUGH a self-collision -- stop at the
        # last safe waypoint (a joint-space lerp isn't guaranteed collision-free).
        if self.guard and self._would_self_collide(nxt):
            self._goto = None
            self.rel_pos, self.rel_eul = self._current_tcp_rel()
            return
        self.joint_targets = nxt
        if done:
            self._goto = None
            self.rel_pos, self.rel_eul = self._current_tcp_rel()

    def _load_poses(self):
        try:
            with open(self._poses_file) as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_poses(self):
        try:
            with open(self._poses_file, "w") as f:
                json.dump(self.poses, f, indent=2)
        except Exception:
            pass

    def apply(self, intent):
        o = intent.ovis
        twist = (o.vx, o.vy, o.vz, o.wx, o.wy, o.wz)
        if any(abs(v) > 1e-6 for v in twist):
            self._goto = None          # operator twist overrides a planned move
            self._solve(o)
        elif self._goto is not None:
            self._step_goto()          # advance the planned joint-space path
        # always hold the (possibly updated) joint targets
        for j, q in self.joint_targets.items():
            p.setJointMotorControl2(self.robot.body_id, j, p.POSITION_CONTROL,
                                    targetPosition=q, force=self.force[j],
                                    maxVelocity=self.max_vel)

    def _solve(self, o):
        # integrate the twist in the base frame
        # Robot-frame command axes (REP-103: x fwd, y left, z up). This URDF's
        # front is -X and right is +Y (matches the tracks' derived forward), so
        # vx fwd -> -X, vy left -> -Y, vz up -> +Z. (Was raw +X/+Y -> "forward"
        # drove the arm backward and sideways didn't work.)
        self.rel_pos[0] -= o.vx * self.lin_speed * self.dt    # forward = -X
        self.rel_pos[1] -= o.vy * self.lin_speed * self.dt    # left = -Y
        self.rel_pos[2] += o.vz * self.lin_speed * self.dt    # up = +Z
        self.rel_eul[0] += o.wx * self.ang_speed * self.dt
        self.rel_eul[1] += o.wy * self.ang_speed * self.dt
        self.rel_eul[2] += o.wz * self.ang_speed * self.dt
        # spherical reach cap from the robot base (NOT a per-axis box, whose
        # diagonal over-reaches): keep the EE target inside `reach`, set BELOW
        # full mechanical extension (~1.2 m) so the arm never locks straight
        # ("we will 100% break it") -- the elbow stays bent.
        dist = sum(x * x for x in self.rel_pos) ** 0.5
        if dist > self.reach:
            s = self.reach / dist
            self.rel_pos = [self.rel_pos[i] * s for i in range(3)]

        # base-frame centroid target -> world
        bpos, born = p.getBasePositionAndOrientation(self.robot.body_id)
        rel_o = p.getQuaternionFromEuler(self.rel_eul)
        c_pos, c_orn = p.multiplyTransforms(bpos, born, self.rel_pos, rel_o)
        # centroid target -> link-frame target (subtract the centroid offset):
        # link_frame * com_local = centroid, so link_frame = centroid * com_local^-1
        inv_cl, _ = p.invertTransform(self.com_local, [0, 0, 0, 1])
        link_pos, link_orn = p.multiplyTransforms(c_pos, c_orn, inv_cl,
                                                  [0, 0, 0, 1])

        cur = {j: p.getJointState(self.robot.body_id, j)[0]
               for j in self.arm_joints}
        # editor/engine IKGizmo split: rotation-dominant twist -> pos_primary
        # (orientation in the null space); otherwise pose_locked (hold the
        # gripper orientation while translating). Only the rove resolver supports
        # this; pybullet IK ignores it.
        if hasattr(self.ik, "set_mode"):
            lin = abs(o.vx) + abs(o.vy) + abs(o.vz)
            ang = abs(o.wx) + abs(o.wy) + abs(o.wz)
            if ang > lin * 1.5:
                self.ik.set_mode("pos_primary", 4.0)
            else:
                self.ik.set_mode("pose_locked", 0.5)
        candidate = self.ik.solve(list(link_pos), link_orn, cur)
        # collaborative arm: refuse a solution that folds a link within the
        # self-collision margin (closing). Hold the last safe targets; retreat
        # (a candidate that improves clearance) is always allowed.
        if self.guard and self._would_self_collide(candidate):
            return
        self.joint_targets = candidate

    def _would_self_collide(self, targets) -> bool:
        g = self.guard
        saved = {j: p.getJointState(self.robot.body_id, j)[:2]
                 for j in self.arm_joints}
        g._sync()
        cur_clear = min(g.min_clearance(n) for n in self.arm_link_names)
        for j, q in targets.items():
            p.resetJointState(self.robot.body_id, j, q, 0.0)
        g._sync()
        cand_clear = min(g.min_clearance(n) for n in self.arm_link_names)
        for j, (pos, vel) in saved.items():
            p.resetJointState(self.robot.body_id, j, pos, vel)
        g._sync()
        return cand_clear < g.margin and cand_clear < cur_clear
