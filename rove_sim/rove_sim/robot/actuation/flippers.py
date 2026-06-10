"""Stepped flipper control (S3, S5.3).

flippers.{fl,fr,rl,rr} in {-1,0,+1} are stateful step commands: each non-zero
step nudges the target angle by step_rad, clamped to the joint limit, held with
POSITION_CONTROL. rl/rr map to the rear (BL/BR) links.
"""
from __future__ import annotations

import pybullet as p

from .base import Actuator, register

# RoveControl field -> semantic flipper link
_MAP = {"fl": "FlipperFL", "fr": "FlipperFR",
        "rl": "FlipperBL", "rr": "FlipperBR"}


@register("stepped_flippers")
class SteppedFlippers(Actuator):
    intent_field = "flippers"

    def _resolve_joints(self):
        self.guard = None              # optional SelfCollisionGuard (set by runtime)
        self.link_of = dict(_MAP)      # field -> link name (for the guard)
        self.step_rad = float(self.params.get("step_rad", 0.087))
        # flippers must lift a corner of the 100 kg robot (clearance / stairs),
        # so they are high-torque -- 150 N.m only nudged it; 500 lifts cleanly.
        self.force = float(self.params.get("force", 500.0))
        # worm-geared flippers are slow & non-backdrivable (hold position). The
        # cap also stops the high-torque motor slamming the flipper and
        # catapulting the robot (S-M1). ~15 deg/s (a bit faster than the first
        # 10 deg/s estimate, per the real robot).
        self.max_vel = float(self.params.get("max_vel_rad_s", 0.26))
        self.max_angle = float(self.params.get("max_angle_rad", 1.7))
        # bind may be a list of links; map each to its RoveControl field
        present = set(self.bind) if isinstance(self.bind, (list, tuple)) \
            else set(self.bind.values())
        self.joints = {}      # field -> (joint idx, lo, hi)
        self.target = {}      # field -> commanded angle
        self.sign = {}        # per-flipper axis normalization
        for fld, link in _MAP.items():
            if link not in present or link not in self.robot.movable_joint:
                continue
            j = self.robot.movable_joint[link]
            info = p.getJointInfo(self.robot.body_id, j)
            lo, hi = info[8], info[9]
            if lo >= hi:       # unlimited in URDF
                lo, hi = -3.14159, 3.14159
            lo, hi = max(lo, -self.max_angle), min(hi, self.max_angle)
            # Deploy direction by GEOMETRY, not the joint axis sign (which gave
            # inconsistent results -- the rear flippers rotated up into the cage).
            # Probe the joint +/- and pick the direction that LOWERS the paddle's
            # lowest point toward the ground; set sign so a -1 command deploys
            # every flipper DOWN (tippy-toe) and +1 retracts up, uniformly.
            self.sign[fld] = -self._down_dir(j, self.robot.link_index[link])
            self.joints[fld] = (j, lo, hi)
            self.target[fld] = p.getJointState(self.robot.body_id, j)[0]

    def _down_dir(self, j, paddle_link):
        """+1 if a POSITIVE joint delta PLANTS the paddle (lowest point goes
        toward the ground), else -1 -- this is the tippy-toe deploy direction.

        Two subtleties cost real debugging here:
          * probe the PADDLE link (paddle_link, a separate index), NOT the
            revolute joint's own link, whose frame sits ON the rotation axis and
            never moves;
          * use the paddle's lowest point (getAABB min-z), NOT its COM -- on this
            flipper geometry the COM and the tip move OPPOSITE ways, so the COM
            criterion plants the flippers backwards (folds them up into the cage);
          * getAABB only refreshes on collision detection, so force it after each
            reset or every probe reads the same stale box.
        Correctly distinguishes mirrored flippers (FL/BR vs FR/BL opposite)."""
        body = self.robot.body_id
        st, vel = p.getJointState(body, j)[:2]
        p.resetJointState(body, j, st + 0.5, 0.0)
        p.performCollisionDetection()
        zp = p.getAABB(body, paddle_link)[0][2]
        p.resetJointState(body, j, st - 0.5, 0.0)
        p.performCollisionDetection()
        zn = p.getAABB(body, paddle_link)[0][2]
        p.resetJointState(body, j, st, vel)
        p.performCollisionDetection()
        return 1.0 if zp < zn else -1.0

    def apply(self, intent):
        f = intent.flippers
        for fld, (j, lo, hi) in self.joints.items():
            step = getattr(f, fld, 0)
            if step:
                desired = min(hi, max(lo, self.target[fld]
                              + step * self.step_rad * self.sign[fld]))
                # worm-gear flippers will crush whatever they drive into -- refuse
                # to advance a step that closes within the self-collision margin
                # (the guard allows retreat). Freeze at the current angle instead.
                cur = p.getJointState(self.robot.body_id, j)[0]
                sgn = 1.0 if desired >= cur else -1.0
                if self.guard and self.guard.blocks(j, [self.link_of[fld]], sgn):
                    self.target[fld] = cur
                else:
                    self.target[fld] = desired
            p.setJointMotorControl2(self.robot.body_id, j, p.POSITION_CONTROL,
                                    targetPosition=self.target[fld],
                                    force=self.force, maxVelocity=self.max_vel)
