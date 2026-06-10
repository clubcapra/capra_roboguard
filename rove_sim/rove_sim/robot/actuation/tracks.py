"""Differential tracks via a distributed brush / Contact-Surface-Motion model (S3).

Background. A rubber belt is not a wheel: every contact point along one side
moves at the SAME belt speed, and the belt grips fore-aft while sliding sideways.
Driving the injected tread cylinders with velocity control made them fight each
other and the chassis wobbled; lumping the drive into one force at the COM was
smooth but is a sim-only cheat (no real slip). The literature (Pecka et al.,
"Fast Simulation of Vehicles with Non-deformable Tracks", and Martinez ICR
kinematics) points at the faithful-yet-fast model used here:

  * Treads are PASSIVE contact geometry with zero PyBullet friction -- they give
    the wide, stable, load-bearing patch (and obstacle support), nothing else.
  * Each physics step, for every tread/flipper contact with the ground we apply
    a brush friction force at the contact point:
      - longitudinal: drive the contact toward the commanded BELT SURFACE speed
        of that side (slip = v_contact.fwd - V_side), so traction emerges;
      - lateral: resist side-slip (slip = v_contact.lat).
    Both are stiffness*slip CAPPED at mu*N using PyBullet's ACTUAL per-contact
    normal load N -- the physical friction limit. When demand exceeds mu*N the
    contact slips, exactly like the real track. Distributing this across the
    contact patch reproduces the turning-resistance moment (the long track scrubs
    at its ends), so a point turn stays near-centre instead of needing a fake
    torque -- and the slip the autonomy must cope with is real.

mu_lat << mu_long because paddle/rubber tracks grip fore-aft and slide sideways;
that anisotropy is what lets a long, narrow-gauge track skid-steer at all.

Flipper belts are coupled to their side here too: a left flipper's belt runs at
the left track speed (internal gear on the real robot). They are tagged by link
in robot.belt_links {link_index: 'L'|'R'} and driven along each contact's own
body-forward, so a flipper still drives while articulated.
"""
from __future__ import annotations

import numpy as np
import pybullet as p

from .base import Actuator, register


@register("differential_tracks")
@register("differential_drive")          # alias: same model drives wheels too
class DifferentialTracks(Actuator):
    """Skid-steer drive over ANY per-side ground-contact surfaces.

    Nothing here is specific to this robot. The driven surfaces are discovered
    from config -- bound drive links (`bind.left/right`, e.g. wheels), injected
    track treads (`robot.track_wheels`), and flipper belts (`robot.belt_links`)
    -- and the forward/lateral axes are DERIVED from the robot's geometry (or set
    explicitly via `forward_axis`/`lateral_axis`). So a new tracked, wheeled, or
    flippered robot gets the same behaviour from its profile alone:

      tracks  : declare `track_wheels` (treads injected) + bind the drums.
      wheels  : bind the wheel links left/right, give them collision; no treads.
      flippers: add `flipper_belts: {LinkName: left|right, ...}`.

    Tune live from the GUI or per-profile params: mu_long, mu_lat, max_track_rad_s.
    """
    intent_field = "tracks"

    def _resolve_joints(self):
        b = self.robot.body_id
        self.drum_radius = float(self.params.get("drum_radius", 0.0899))
        self.max_rad_s = float(self.params.get("max_track_rad_s", 46.0))
        # belt SURFACE speed at full command (m/s). 46 rad/s * 0.09 m ~ 4.16 m/s
        # ~ 15 km/h top, the real robot's top speed (ODrive caps the rest).
        self.v_max = self.max_rad_s * self.drum_radius
        # rubber-on-hardwood ~0.6 fore-aft.
        self.mu_long = float(self.params.get("mu_long", 0.6))
        # Lateral friction reconciles arcs and pivots. A drive/arc command (tracks
        # the same sign) must GRIP laterally (mu_lat_static) so the track supplies
        # centripetal force and the robot doesn't wash out sideways; a point-turn
        # command (tracks oppose) must SCRUB (mu_lat, low) so the long track can
        # rotate in place. We blend by how turn-dominated the command is -- this
        # is the static-vs-kinetic regime keyed on the maneuver (a slip-velocity
        # Stribeck stalls: a slow pivot never leaves the static regime to scrub).
        self.mu_lat = float(self.params.get("mu_lat", 0.15))           # scrub
        self.mu_lat_static = float(self.params.get("mu_lat_static", 0.7))  # grip
        # brush stiffness N per (m/s) of slip, before the mu*N cap bites. High =
        # grips almost rigidly until it saturates and slips.
        self.k = float(self.params.get("brush_stiffness", 3000.0))
        # optional painted ground-friction raster (set by runtime from the world);
        # None => uniform mu_long everywhere (original behaviour).
        self.friction_field = None

        # Tag every DRIVE-SURFACE link with its side {link_index: 'L'|'R'}, all
        # from config/loader -- no name parsing, so this works for tracks (treads),
        # wheels (the bound wheel links) or any mix. Sources:
        #   * self.bind {left:[...], right:[...]}  -- bound drive links (wheels/drums)
        #   * robot.track_wheels {left/right:[idx]} -- treads injected for tracks
        #   * robot.belt_links {idx: side}          -- flipper belts
        self.side_of = {}
        self._driven = set()        # surfaces WE own (free-roll); not flippers
        for key, S in (("left", "L"), ("right", "R")):
            for name in (self.bind.get(key, []) if isinstance(self.bind, dict) else []):
                idx = self.robot.link_index.get(name)
                if idx is not None:
                    self.side_of[idx] = S
                    self._driven.add(idx)
            for idx in getattr(self.robot, "track_wheels", {}).get(key, []):
                self.side_of[idx] = S
                self._driven.add(idx)
        # flipper belts: the flipper actuator owns their joint (POSITION_CONTROL),
        # so tag them for driving but do NOT free-roll them.
        self.side_of.update(getattr(self.robot, "belt_links", {}))

        # drive surfaces are CONTACT, not motors: zero PyBullet friction (the brush
        # forces ARE the friction) and free-roll the ones we own.
        for idx, S in self.side_of.items():
            if idx < 0:
                continue
            p.changeDynamics(b, idx, lateralFriction=0.0)
            if idx in self._driven:
                p.setJointMotorControl2(b, idx, p.VELOCITY_CONTROL, force=0)
        self._loc_com = np.array(p.getDynamicsInfo(b, -1)[3])
        self._derive_axes()

        # M3 telemetry hooks: track-force budget -> motor current. The real robot
        # draws ~6 A/motor in a floor pivot; nominal 20 N.m, 7.2 A = 50 N.m peak.
        self.motor_torque = float(self.params.get("motor_torque", 50.0))
        self.motors_per_side = int(self.params.get("motors_per_side", 2))
        self.current_limit_a = float(self.params.get("current_limit_a", 7.2))
        self.last_side_force = {"L": 0.0, "R": 0.0}   # N, signed fore-aft

    def _derive_axes(self):
        """Body-frame forward/lateral unit axes -- derived from the robot's own
        geometry so no +X/+Y assumption is baked in (a different URDF may face
        any way). Lateral = right-group centroid minus left-group centroid;
        forward = front-vs-rear drum centroid if the profile declares front/rear,
        else up x lateral. Either can be overridden with profile params
        `forward_axis` / `lateral_axis` (body-frame 3-vectors)."""
        b = self.robot.body_id
        bp, bo = p.getBasePositionAndOrientation(b)
        R = np.array(p.getMatrixFromQuaternion(bo)).reshape(3, 3)

        def body_centroid(idxs):
            ps = [np.array(p.getLinkState(b, i)[0]) for i in idxs if i >= 0]
            return R.T @ (np.mean(ps, 0) - np.array(bp)) if ps else None

        fa, la = self.params.get("forward_axis"), self.params.get("lateral_axis")
        if fa is not None and la is not None:
            self.fwd_body, self.lat_body = np.array(fa, float), np.array(la, float)
        else:
            lc = body_centroid([i for i, s in self.side_of.items() if s == "L"])
            rc = body_centroid([i for i, s in self.side_of.items() if s == "R"])
            lat = (rc - lc) if (lc is not None and rc is not None) else np.array([0, 1., 0])
            lat[2] = 0.0
            lat /= np.linalg.norm(lat)
            # forward = the horizontal axis perpendicular to lateral.
            fwd = np.cross([0, 0, 1.], lat)
            # fix its SIGN from the declared front/rear drums. Those links are
            # fixed-lumped (their frame is unreliable), so resolve to the real
            # revolute *_pivot link, and use it only to orient (rear->front).
            li = self.robot.link_index
            sides = (self.robot.profile.raw.get("track_wheels") or {}).get("sides", {})

            def resolve(role):
                out = []
                for d in sides.values():
                    n = d.get(role) if isinstance(d, dict) else None
                    for cand in ((n + "_pivot", n) if n else ()):
                        if cand in li:
                            out.append(li[cand]); break
                return out
            fc, rec = body_centroid(resolve("front")), body_centroid(resolve("rear"))
            if fc is not None and rec is not None:
                d = fc - rec; d[2] = 0.0
                if np.linalg.norm(d) > 1e-3 and d @ fwd < 0:
                    fwd = -fwd
            self.fwd_body, self.lat_body = fwd, lat
        self.fwd_body /= np.linalg.norm(self.fwd_body)
        self.lat_body /= np.linalg.norm(self.lat_body)

    # apply() is control-rate; the brush forces must be re-applied every physics
    # step, so the real work happens in step(). We just stash the command.
    def apply(self, intent):
        self._cmd = intent.tracks

    def step(self, intent):
        t = getattr(self, "_cmd", None) or intent.tracks
        b = self.robot.body_id
        V = {"L": t.left_vel * self.v_max, "R": t.right_vel * self.v_max}
        # lateral-grip blend: 0 = pure drive/arc (grip), 1 = pure pivot (scrub).
        fwd_cmd = abs(t.left_vel + t.right_vel) / 2.0
        turn_cmd = abs(t.right_vel - t.left_vel) / 2.0
        blend = turn_cmd / (fwd_cmd + turn_cmd + 1e-6)
        mu_lat_eff = self.mu_lat_static * (1.0 - blend) + self.mu_lat * blend
        bp, bo = p.getBasePositionAndOrientation(b)
        M = np.array(p.getMatrixFromQuaternion(bo)).reshape(3, 3)
        fwd, lat = M @ self.fwd_body, M @ self.lat_body
        lin, ang = (np.asarray(x) for x in p.getBaseVelocity(b))
        com = np.asarray(bp) + M @ self._loc_com

        # Gather the driven ground contacts into arrays so the whole brush model
        # is a handful of vectorised numpy ops instead of a per-contact Python
        # loop (the old loop's per-point np.cross/clip dominated the step). Forces
        # are byte-for-byte identical -- only the arithmetic is batched.
        pts, Ns, sides = [], [], []
        for c in p.getContactPoints(bodyA=b):
            side = self.side_of.get(c[3])
            if side is None or c[9] <= 0:       # not ours / no normal load
                continue
            pts.append(c[5]); Ns.append(c[9]); sides.append(side)
        if not pts:
            self.last_side_force = {"L": 0.0, "R": 0.0}
            return

        pts = np.asarray(pts)                              # (n,3)
        Ns = np.asarray(Ns)                                # (n,)
        is_R = np.fromiter((s == "R" for s in sides), bool, len(sides))
        # contact velocity v = lin + ang x (pt - com), explicit cross (np.cross is
        # ~4 normalize_axis_index calls/contact -- the old hot spot).
        r = pts - com
        v = lin + np.column_stack((
            ang[1] * r[:, 2] - ang[2] * r[:, 1],
            ang[2] * r[:, 0] - ang[0] * r[:, 2],
            ang[0] * r[:, 1] - ang[1] * r[:, 0]))
        v_fwd = v @ fwd
        v_lat = v @ lat
        V = np.where(is_R, t.right_vel, t.left_vel) * self.v_max

        # per-contact surface friction: a painted FrictionField scales the
        # traction cap by the cell mu at the contact's world XY (ice -> slip,
        # gravel -> grip). Unpainted / no field => mu_long everywhere (scale 1).
        if self.friction_field is not None:
            mu_long_c = np.fromiter(
                (self.friction_field.lookup(x, y) for x, y in pts[:, :2]),
                float, len(pts))
            scale = mu_long_c / self.mu_long if self.mu_long > 0 else np.ones_like(mu_long_c)
        else:
            mu_long_c = np.full(len(pts), self.mu_long)
            scale = np.ones(len(pts))

        cap_long = mu_long_c * Ns
        cap_lat = mu_lat_eff * scale * Ns
        f_long = np.clip(-self.k * (v_fwd - V), -cap_long, cap_long)
        f_lat = np.clip(-self.k * v_lat, -cap_lat, cap_lat)
        F = f_long[:, None] * fwd + f_lat[:, None] * lat
        for i in range(len(pts)):
            p.applyExternalForce(b, -1, F[i].tolist(), pts[i].tolist(), p.WORLD_FRAME)
        self.last_side_force = {"L": float(f_long[~is_R].sum()),
                                "R": float(f_long[is_R].sum())}

    # -- M3 telemetry -------------------------------------------------------
    def side_current_a(self):
        """Per-MOTOR current (A) each side, from the fore-aft track force.

        force -> drum torque (F*r) -> split over motors_per_side -> A via the
        nominal N.m/A. Calibrated so a floor pivot lands near the measured 6 A.
        """
        nm_per_a = self.motor_torque / max(self.current_limit_a, 1e-6)
        out = {}
        for s, F in self.last_side_force.items():
            tau = abs(F) * self.drum_radius / max(self.motors_per_side, 1)
            out[s] = min(tau / nm_per_a, self.current_limit_a)
        return out
