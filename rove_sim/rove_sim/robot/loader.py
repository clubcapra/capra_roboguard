"""Robot loader: OBJ visuals + collision-primitive overrides + name maps.

Two stages:

  1. prepare_urdf -- XML surgery on the source URDF (S5.1, S5.2). Visual meshes
     are converted GLB->OBJ; collision geometry is rewritten per link-name rule
     to a primitive (box/cylinder, sized from the mesh's trimesh extents), a
     convex hull (the OBJ, which PyBullet treats as convex for a dynamic body),
     or removed entirely (sensor/pivot links). Dynamic concave meshes are
     unstable in PyBullet, so a profile should never leave a ground-contacting
     link as a raw mesh.

  2. load -- loadURDF with INERTIA_FROM_FILE | MAINTAIN_LINK_ORDER and build the
     link-name / joint-name -> index maps the rest of the sim keys off (joint
     names differ between robots; link names are the stable identifier, S-recon).

Collision override rules come from the profile (first fnmatch wins, default
'none'):

    collision_overrides:
      - {match: "Drum*",          shape: cylinder}
      - {match: "Base|Core|cage", shape: box}
      - {match: "Flipper*",       shape: box}
      - {match: "*",              shape: none}
"""
from __future__ import annotations

import fnmatch
import hashlib
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
import pybullet as p
import trimesh

from ..core.engine import Engine
from ..core.util import suppressed_fds
from .profile import Profile
from tools.convert_meshes import convert_glb

_GEOM_CACHE: Dict[str, tuple] = {}   # glb -> (extents, center) in mesh frame

# Role-based mass defaults (kg). The URDFs carry no inertials, so every link
# would otherwise default to 1 kg -- making the multi-link arm as heavy as the
# chassis and the robot top-heavy (it tips, S-M0). First fnmatch wins; a
# profile may supply its own `masses:` list to override. Pivots are massless
# connectors. These are coarse placeholders, TODO(calibrate from CAD/datasheet).
# Real robot is bottom-heavy: the Core (chassis + batteries) is ~80 of 100 kg
# with a very low COM, which is what keeps it stable through turns and with the
# arm/flippers up. Earlier values spread too much mass high (arm/wheels/sensors)
# and the robot rocked. These are a distribution scaled to total_mass.
_DEFAULT_MASSES = [
    ("DrumW_*",                                            1.0),    # belt cylinders
    ("Core",                                              80.0),    # dominant chassis (80-90 kg, batteries low)
    ("cage*",                                              3.0),
    ("Drum*",                                              2.0),
    ("Flipper*",                                           1.5),
    # arm is light so the real holding torques (12 N.m shoulder, 3.6 N.m wrist)
    # actually hold it: at ~0.25 kg/link the elbow needed ~4.9 N.m > 3.6 and
    # drooped, so links are ~0.13 kg (elbow gravity torque < 3.6).
    ("Base",                                               0.35),
    ("ASection|BSection|JointA|JointB|JointGripper",       0.13),
    ("*_pivot",                                            0.02),
    ("*",                                                  0.15),   # sensors etc
]


def _mesh_geom(glb_path: str) -> tuple:
    """Return (extents, center) of the mesh AABB in the mesh/link frame.

    The center matters: these meshes are not centered on their link origin, so a
    primitive placed at the origin lands in the wrong place (the chassis box
    bellied to the floor while the drums floated). We offset each primitive by
    the AABB center via a collision <origin>.
    """
    if glb_path not in _GEOM_CACHE:
        scene = trimesh.load(glb_path, force="scene")
        lo, hi = scene.bounds
        extents = np.asarray(hi - lo, dtype=float)
        center = np.asarray((hi + lo) / 2.0, dtype=float)
        _GEOM_CACHE[glb_path] = (extents, center)
    return _GEOM_CACHE[glb_path]


def _inject_track_wheels(root: ET.Element, profile: Profile) -> None:
    """Lay each track's ground-contact patch as a dense row of small tread rollers.

    A real track is a continuous belt; its ground contact is the flat *bottom
    run* spanning drum-to-drum, as wide as the drum. We model that the most
    faithful way PyBullet allows: a dense row of small (~1 inch) cylinders along
    the bottom run of each side, dropped from the axle line down to the
    drum-bottom tangent so the little treads -- not the big drums -- are what
    touches the floor. Each tread is as WIDE as the drum (axial length), so the
    contact patch is the full track width, not a thin line -> a broad, stable
    base that kills the point-turn roll/lift. They free-roll (the tracks actuator
    drives the chassis with two per-track surface forces, S-M1); naming them
    'DrumW_<L|R><i>' inherits the low-lateral belt friction + continuous handling.

    Geometry knobs (profile.track_wheels): `radius` = tread radius (~0.013 = 1in),
    `width` = tread axial length (= drum width ~0.089), `drum_radius` = the real
    drum radius used to drop the row to the bottom run, `count` = treads/side.
    """
    cfg = profile.raw.get("track_wheels")
    if not cfg:
        return
    count = int(cfg.get("count", 20))
    radius = float(cfg.get("radius", 0.0127))      # ~1 inch tread roller
    width = float(cfg.get("width", 0.089))         # as wide as the drum
    drum_radius = float(cfg.get("drum_radius", 0.0899))
    # drop the treads from the axle line down to the drum-bottom tangent, so the
    # small rollers reach the floor where the big drum did (the track bottom run).
    drop = drum_radius - radius
    # extend the patch beyond the drums (e.g. toward flipper tips); default off.
    extend = float(cfg.get("extend_x", 0.0))

    origin_of = {}                      # pivot child link -> (xyz, axis)
    for j in root.findall("joint"):
        child = j.find("child")
        if child is None:
            continue
        o = j.find("origin")
        a = j.find("axis")
        if o is not None:
            origin_of[child.get("link")] = (
                [float(v) for v in o.get("xyz", "0 0 0").split()],
                a.get("xyz", "0 0 1") if a is not None else "0 0 1")

    for side, drums in cfg["sides"].items():
        fp = origin_of.get(drums["front"] + "_pivot")
        rp = origin_of.get(drums["rear"] + "_pivot")
        if not (fp and rp):
            continue
        (fx, fy, fz), axis = fp
        (rx, ry, rz), _ = rp
        if extend:                                # reach the flipper tips
            d = 1.0 if rx >= fx else -1.0
            fx, rx = fx - d * extend, rx + d * extend
        tag = "L" if side == "left" else "R"
        for i in range(count):
            # span the FULL bottom run drum-to-drum (inclusive endpoints), the
            # "outer drum to outer drum" patch -- not just the interior.
            t = i / (count - 1) if count > 1 else 0.5
            x = fx + t * (rx - fx)
            y = fy + t * (ry - fy)
            z = fz + t * (rz - fz) - drop          # down to the bottom run
            name = f"DrumW_{tag}{i}"
            link = ET.SubElement(root, "link")
            link.set("name", name)
            col = ET.SubElement(link, "collision")
            geom = ET.SubElement(col, "geometry")
            cyl = ET.SubElement(geom, "cylinder")
            cyl.set("radius", f"{radius:.6f}")
            cyl.set("length", f"{width:.6f}")
            co = ET.SubElement(col, "origin")
            co.set("rpy", "1.570796 0 0")          # cylinder length -> Y axis
            co.set("xyz", "0 0 0")
            joint = ET.SubElement(root, "joint")
            joint.set("name", name + "_joint")
            joint.set("type", "continuous")
            ET.SubElement(joint, "parent").set("link", "Core")
            ET.SubElement(joint, "child").set("link", name)
            jo = ET.SubElement(joint, "origin")
            jo.set("xyz", f"{x:.6f} {y:.6f} {z:.6f}")
            jo.set("rpy", "0 0 0")
            ET.SubElement(joint, "axis").set("xyz", axis)


def _inject_gripper(root: ET.Element, profile: Profile, base: str) -> None:
    """Merge an end-effector gripper URDF (e.g. Robotiq 2F-140) into the robot.

    The gripper is a real, separately-authored URDF; we splice its links/joints
    into the rove body and bolt its base to the arm's EE link with a fixed mount
    joint, so it is one body (IK, the self-collision guard and rendering all see
    it). Injected AFTER the collision-shape pass, so the gripper keeps its own
    mesh collision (PyBullet convex-hulls it -> stable, graspable). PyBullet
    ignores <mimic> tags, so all finger joints load free; the gripper actuator
    enforces the 4-bar mimic coupling. Config: profile.gripper.{urdf, mount_link,
    base_link, mount_xyz, mount_rpy}.
    """
    cfg = profile.raw.get("gripper")
    if not cfg:
        return
    gpath = cfg["urdf"]
    if not os.path.isabs(gpath):
        gpath = os.path.join(base, gpath)
    if not os.path.exists(gpath):
        return
    gripper = ET.parse(gpath).getroot()
    # The gripper URDF may carry ABSOLUTE mesh paths baked on another machine
    # (e.g. the robotiq STLs). Rebase any such filename onto THIS machine's tree
    # by its stable "meshes/..." suffix so a copied stack loads without editing
    # the source URDF; relative/resolvable filenames are left untouched.
    gdir = os.path.dirname(gpath)
    for mesh in gripper.iter("mesh"):
        fn = (mesh.get("filename") or "").replace("\\", "/")
        if "meshes/" in fn and (os.path.isabs(fn)
                                or not os.path.exists(os.path.join(gdir, fn))):
            mesh.set("filename",
                     os.path.join(gdir, "meshes/" + fn.split("meshes/", 1)[1]))
    have_links = {l.get("name") for l in root.findall("link")}
    have_joints = {j.get("name") for j in root.findall("joint")}
    for el in list(gripper):
        if el.tag == "link" and el.get("name") not in have_links:
            root.append(el)
        elif el.tag == "joint" and el.get("name") not in have_joints:
            root.append(el)
    mount = ET.SubElement(root, "joint")
    mount.set("name", cfg.get("mount_joint", "gripper_mount"))
    mount.set("type", "fixed")
    ET.SubElement(mount, "parent").set("link", cfg.get("mount_link", "JointGripper"))
    ET.SubElement(mount, "child").set("link",
                                      cfg.get("base_link", "robotiq_arg2f_base_link"))
    o = ET.SubElement(mount, "origin")
    o.set("xyz", " ".join(str(v) for v in cfg.get("mount_xyz", [0, 0, 0])))
    o.set("rpy", " ".join(str(v) for v in cfg.get("mount_rpy", [0, 0, 0])))


def _inject_flipper_belts(root: ET.Element, profile: Profile, base: str) -> None:
    """Add a driven belt of cylinders along each flipper's contact edge.

    Each flipper is a tracked paddle whose belt rolls around it at main-track
    speed (internal gear), giving drive/traction whether the flipper is flat or
    up on its tip ("tippy-toe"). We inject a row of small continuous-joint
    cylinders along the paddle's bottom edge as children of the FLIPPER link, so
    they move with the flipper as it articulates. Named DrumW_<side><n> so the
    tracks actuator drives them with that side's main belt and they inherit the
    low-lateral belt friction.
    """
    cfg = profile.raw.get("flipper_belts")
    if not cfg:
        return
    count = int(cfg.get("count", 4))
    radius = float(cfg.get("radius", 0.045))
    width = float(cfg.get("width", 0.06))
    flippers = cfg.get("flippers", {})       # link -> "left"|"right"

    # PyBullet lumps fixed-joint children into their parent, so the flipper
    # paddle link is merged into its *_pivot link. Parent the belts to that
    # surviving pivot link and shift by the fixed offset. Build maps from joints:
    fixed_parent = {}   # flipper link -> (pivot link, offset xyz)
    axis_of = {}        # pivot link -> revolute axis
    for j in root.findall("joint"):
        ch = j.find("child")
        par = j.find("parent")
        o = j.find("origin")
        a = j.find("axis")
        if ch is None or par is None:
            continue
        if j.get("type") == "fixed" and o is not None:
            fixed_parent[ch.get("link")] = (
                par.get("link"),
                [float(v) for v in o.get("xyz", "0 0 0").split()])
        if a is not None:
            axis_of[ch.get("link")] = a.get("xyz", "0 -1 0")

    for link_name, side in flippers.items():
        glb = os.path.join(base, "meshes", link_name + ".glb")
        if not os.path.exists(glb) or link_name not in fixed_parent:
            continue
        pivot, off = fixed_parent[link_name]
        ext, ctr = _mesh_geom(glb)
        # paddle bottom edge along X (extension), at the lowest Z; +offset to
        # express in the pivot frame.
        x0, x1 = ctr[0] - ext[0] / 2, ctr[0] + ext[0] / 2
        z_bottom = ctr[2] - ext[2] / 2 + radius
        tag = "L" if side == "left" else "R"
        axis = axis_of.get(pivot, "0 -1 0")
        for i in range(count):
            t = (i + 0.5) / count
            x = x0 + t * (x1 - x0)
            name = f"DrumW_{tag}F{link_name[-2:]}{i}"
            link = ET.SubElement(root, "link")
            link.set("name", name)
            col = ET.SubElement(link, "collision")
            geom = ET.SubElement(col, "geometry")
            cyl = ET.SubElement(geom, "cylinder")
            cyl.set("radius", f"{radius:.6f}")
            cyl.set("length", f"{width:.6f}")
            co = ET.SubElement(col, "origin")
            co.set("rpy", "1.570796 0 0")
            co.set("xyz", "0 0 0")
            joint = ET.SubElement(root, "joint")
            joint.set("name", name + "_joint")
            joint.set("type", "continuous")
            ET.SubElement(joint, "parent").set("link", pivot)
            ET.SubElement(joint, "child").set("link", name)
            jo = ET.SubElement(joint, "origin")
            jo.set("xyz", f"{x + off[0]:.6f} {ctr[1] + off[1]:.6f} "
                          f"{z_bottom + off[2]:.6f}")
            jo.set("rpy", "0 0 0")
            ET.SubElement(joint, "axis").set("xyz", axis)


def _set_inertial(link: ET.Element, com, extents) -> None:
    """Give a link an <inertial> with COM at `com` and a coarse box inertia.

    Sets the COM *position* (the important bit -- fixes the Core tilt); the mass
    value here is a placeholder overridden by changeDynamics post-load, and the
    inertia is a unit-mass box from extents (changeDynamics rescales it with the
    real mass via localInertiaDiagonal).
    """
    for old in link.findall("inertial"):
        link.remove(old)
    inr = ET.SubElement(link, "inertial")
    o = ET.SubElement(inr, "origin")
    o.set("xyz", f"{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}")
    o.set("rpy", "0 0 0")
    ET.SubElement(inr, "mass").set("value", "1.0")
    ex, ey, ez = (max(0.02, float(e)) for e in extents)
    ixx = (ey * ey + ez * ez) / 12.0
    iyy = (ex * ex + ez * ez) / 12.0
    izz = (ex * ex + ey * ey) / 12.0
    it = ET.SubElement(inr, "inertia")
    it.set("ixx", f"{ixx:.6f}"); it.set("iyy", f"{iyy:.6f}"); it.set("izz", f"{izz:.6f}")
    it.set("ixy", "0"); it.set("ixz", "0"); it.set("iyz", "0")


def _matches(link: str, patterns) -> bool:
    for pat in patterns or []:
        for sub in str(pat).split("|"):
            if fnmatch.fnmatch(link, sub.strip()):
                return True
    return False


def _match_rule(link: str, rules: List[dict]) -> dict:
    for rule in rules:
        for pat in str(rule["match"]).split("|"):
            if fnmatch.fnmatch(link, pat.strip()):
                return rule
    return {"shape": "none"}


def _match_mass(link: str, rules: List) -> float:
    for pat, mass in rules:
        for sub in str(pat).split("|"):
            if fnmatch.fnmatch(link, sub.strip()):
                return float(mass)
    return 1.0


def _apply_masses(robot: "Robot") -> None:
    """Assign role-based masses; scale to the profile's total_mass if set.

    The role values are a *distribution*, not absolutes. If the profile gives
    total_mass (the real robot is 100 kg), every link is scaled by
    total_mass / sum(role masses) so the total is exact while the chassis/arm/
    wheel proportions are preserved. Pivot links (~0) are excluded from the
    scaled budget so they stay massless.
    """
    profile_rules = robot.profile.raw.get("masses")
    rules = ([(r["match"], r["mass"]) for r in profile_rules]
             if profile_rules else []) + _DEFAULT_MASSES
    role = {link: _match_mass(link, rules) for link in robot.link_index}

    total = robot.profile.raw.get("total_mass")
    if total:
        # scale only the load-bearing links; keep near-zero pivots as-is
        budget = {l: m for l, m in role.items() if m > 0.05}
        fixed = sum(m for l, m in role.items() if m <= 0.05)
        s = (float(total) - fixed) / sum(budget.values())
        role = {l: (m * s if m > 0.05 else m) for l, m in role.items()}

    for link, idx in robot.link_index.items():
        m = role[link]
        # box inertia from the link's world AABB (coarse but keeps rotational
        # dynamics sane now that we inject our own inertial frames).
        lo, hi = p.getAABB(robot.body_id, idx)
        ex, ey, ez = (max(0.03, hi[i] - lo[i]) for i in range(3))
        diag = [m * (ey*ey + ez*ez) / 12.0,
                m * (ex*ex + ez*ez) / 12.0,
                m * (ex*ex + ey*ey) / 12.0]
        p.changeDynamics(robot.body_id, idx, mass=m, localInertiaDiagonal=diag)
    robot.total_mass = sum(role.values())


# Friction per role (S5.8). The drums are real-world TRACKS: model them with
# anisotropic friction -- strong longitudinal grip, weak lateral so the robot
# can skid-steer (a point-contact cylinder cannot turn). NOT traction truth.
# A profile `friction:` list overrides: {match, value, anisotropic?, anchor?}.
# Tracks are mud-grade rubber + paddles: they GRIP hard (real robot stops fast
# and doesn't drift on ice). Isotropic high friction -- the point-turn is driven
# by motor torque scrubbing the grippy tracks (forceful, ~5 A on carpet), NOT by
# low lateral friction letting it slide (that read as "on ice"). Rubber is also
# compliant+damped (contactStiffness/Damping) and rollingFriction keeps it
# planted -- without these the fast drums bounce off the floor in turns.
_DEFAULT_FRICTION = [
    # belt cylinders: grip longitudinally, SCRUB laterally (low lateral) so the
    # two belts counter-rotating pivot the robot cleanly instead of locking.
    {"match": "DrumW_*", "value": 1.4, "anchor": True,
     "anisotropic": [1.0, 0.3, 1.0], "contact_stiffness": 30000.0,
     "contact_damping": 4000.0},
    {"match": "Drum*", "value": 1.5, "anchor": True,
     "contact_stiffness": 30000.0, "contact_damping": 4000.0, "rolling": 0.02},
    {"match": "Flipper*", "value": 1.3, "rolling": 0.02},
    {"match": "*",        "value": 0.9},
]


def _apply_friction(robot: "Robot") -> None:
    rules = list(robot.profile.raw.get("friction", [])) + _DEFAULT_FRICTION
    for link, idx in robot.link_index.items():
        rule = _match_rule(link, rules)
        kw = dict(lateralFriction=float(rule.get("value", 0.8)),
                  restitution=0.0)
        if rule.get("anchor"):
            kw["frictionAnchor"] = 1
        if rule.get("anisotropic"):
            kw["anisotropicFriction"] = list(rule["anisotropic"])
        if rule.get("rolling"):
            kw["rollingFriction"] = float(rule["rolling"])
        if rule.get("contact_stiffness"):
            kw["contactStiffness"] = float(rule["contact_stiffness"])
            kw["contactDamping"] = float(rule.get("contact_damping", 1000.0))
        p.changeDynamics(robot.body_id, idx, **kw)


def _cylinder_from_extents(ext: np.ndarray):
    """Drum/wheel: axial extent is the smallest, radius from the other two."""
    axis = int(np.argmin(ext))
    length = float(ext[axis])
    radial = [e for i, e in enumerate(ext) if i != axis]
    radius = float(np.mean(radial)) / 2.0
    # pybullet cylinder length axis is Z; rotate Z onto the detected axis
    rpy = {0: (0.0, 1.5707963, 0.0),   # X
           1: (1.5707963, 0.0, 0.0),   # Y
           2: (0.0, 0.0, 0.0)}[axis]   # Z
    return radius, length, rpy


@dataclass
class Robot:
    body_id: int
    profile: Profile
    link_index: Dict[str, int] = field(default_factory=dict)    # link -> idx
    joint_index: Dict[str, int] = field(default_factory=dict)   # joint -> idx
    # semantic link name -> the movable joint that actually drives it. In these
    # URDFs a drum/flipper/arm link X hangs off X_pivot via a fixed _offset
    # joint, so the controlling joint is found by climbing past fixed joints
    # (S-M0). This is the handle actuators bind to.
    movable_joint: Dict[str, int] = field(default_factory=dict)
    track_wheels: Dict[str, list] = field(default_factory=dict)   # side -> [joint idx]
    # belt CONTACT links -> side ('L'|'R'): every link whose ground contacts the
    # brush-friction tracks actuator should drive. Main treads are added by the
    # actuator from their DrumW_<L|R> names; flipper belts are added here from the
    # `flipper_belts` profile map (a left flipper's belt runs at left-track speed).
    belt_links: Dict[int, str] = field(default_factory=dict)
    total_mass: float = 0.0

    def link_id(self, name: str) -> int:
        return self.link_index[name]

    def joint_for(self, link: str) -> int:
        """The movable joint driving a semantic link (e.g. 'DrumFL')."""
        return self.movable_joint[link]


def prepare_urdf(profile: Profile, cache_dir: Optional[str] = None) -> str:
    src = profile.model.path
    base = os.path.dirname(src)
    rules = profile.collision_overrides
    if cache_dir is None:
        cache_dir = os.path.join(base, ".rove_sim_cache")
    os.makedirs(cache_dir, exist_ok=True)

    tree = ET.parse(src)
    root = tree.getroot()

    for link in root.findall("link"):
        name = link.get("name", "")
        # -- visuals: GLB -> OBJ -------------------------------------------
        for mesh in link.findall("./visual/geometry/mesh"):
            fn = mesh.get("filename", "")
            if fn.lower().endswith(".glb"):
                rel = fn[:-4]
                convert_glb(os.path.join(base, rel + ".glb"),
                            os.path.join(base, rel + ".obj"))
                mesh.set("filename", rel + ".obj")

        # -- collisions: rewrite per rule ----------------------------------
        rule = _match_rule(name, rules)
        shape = rule.get("shape", "none")
        for col in link.findall("collision"):
            geom = col.find("geometry")
            mesh = geom.find("mesh") if geom is not None else None
            glb = None
            if mesh is not None and mesh.get("filename", "").lower().endswith(".glb"):
                glb = os.path.join(base, mesh.get("filename"))

            if shape == "none":
                link.remove(col)
                continue
            if shape == "convex":
                if mesh is not None and glb:
                    rel = mesh.get("filename")[:-4]
                    convert_glb(glb, os.path.join(base, rel + ".obj"))
                    mesh.set("filename", rel + ".obj")
                continue
            if glb is None:
                continue  # nothing to size a primitive from; leave as-is
            ext, ctr = _mesh_geom(glb)
            for child in list(geom):
                geom.remove(child)
            origin = col.find("origin")
            if origin is None:
                origin = ET.SubElement(col, "origin")
            cz = ctr[2]
            ez = ext[2]
            if shape == "box":
                # raise_bottom: lift the box's bottom face by N metres (keep the
                # top) so wheels/flippers carry the robot, not the chassis (S-M1,
                # user-directed). Shrinks height by N, shifts centre up by N/2.
                rb = float(rule.get("raise_bottom", 0.0))
                if rb:
                    ez = max(1e-3, ez - rb)
                    cz = cz + rb / 2.0
                b = ET.SubElement(geom, "box")
                b.set("size", f"{ext[0]:.6f} {ext[1]:.6f} {ez:.6f}")
                origin.set("rpy", origin.get("rpy", "0 0 0"))
                origin.set("xyz", f"{ctr[0]:.6f} {ctr[1]:.6f} {cz:.6f}")
            elif shape == "cylinder":
                r, l, rpy = _cylinder_from_extents(ext)
                c = ET.SubElement(geom, "cylinder")
                c.set("radius", f"{r:.6f}")
                c.set("length", f"{l:.6f}")
                origin.set("rpy", f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}")
                origin.set("xyz", f"{ctr[0]:.6f} {ctr[1]:.6f} {ctr[2]:.6f}")
            else:
                raise ValueError(f"unknown collision shape {shape!r} for {name}")

            # COM at the body centroid, not the link origin: the heavy Core has
            # its origin at y=0 but its body (and track support) is at y~=0.21,
            # so a link-origin COM tips the robot onto its side (S-M1). Mass is
            # set post-load; here we fix the inertial *position* and a coarse
            # box inertia. Core gets a low COM (batteries down low).
            cm = [float(ctr[0]), float(ctr[1]), float(ctr[2])]
            if name == "Core":
                cm[2] = float(ctr[2]) - ext[2] * 0.30   # push COM low
            _set_inertial(link, cm, ext)

    # wheel joints: the URDF gives drums revolute limits of +/-pi (stale demo
    # values), so a "wheel" jams after half a turn. Convert joints whose child
    # link matches `continuous_joints` to type=continuous (unlimited). (S-M1)
    cont_pats = profile.raw.get("continuous_joints", [])
    for joint in root.findall("joint"):
        child_el = joint.find("child")
        child = child_el.get("link") if child_el is not None else ""
        if joint.get("type") == "revolute" and _matches(child, cont_pats):
            joint.set("type", "continuous")
            lim = joint.find("limit")
            if lim is not None:
                lim.attrib.pop("lower", None)
                lim.attrib.pop("upper", None)

    _inject_track_wheels(root, profile)
    _inject_flipper_belts(root, profile, base)
    _inject_gripper(root, profile, base)

    digest = hashlib.md5(ET.tostring(root)).hexdigest()[:8]
    out = os.path.join(cache_dir,
                       f"{os.path.basename(src)[:-5]}.{digest}.urdf")
    tree.write(out)
    return out


def load(engine: Engine, profile: Profile,
         cache_dir: Optional[str] = None, quiet: bool = True) -> Robot:
    urdf = prepare_urdf(profile, cache_dir)
    # NOTE: URDF_MAINTAIN_LINK_ORDER segfaults pybullet 3.2.7 on these URDFs;
    # we key everything by link name (Robot.link_index), so link order is
    # irrelevant and the flag is omitted. (Bisected S-M0.)
    flags = p.URDF_USE_INERTIA_FROM_FILE
    if profile.model.self_collision:
        flags |= p.URDF_USE_SELF_COLLISION
    # pybullet prints a per-link "No inertial data" warning to fd 1/2; we set
    # masses below via changeDynamics, so silence the C-level spam (quiet).
    with suppressed_fds(quiet):
        body = p.loadURDF(
            urdf,
            basePosition=profile.model.base_position,
            baseOrientation=profile.model.base_orientation,
            useFixedBase=False,
            flags=flags,
        )

    robot = Robot(body_id=body, profile=profile)
    base_link = p.getBodyInfo(body)[0].decode()
    robot.link_index[base_link] = -1                          # base link

    # gather raw topology: child link -> (joint idx, type, parent link)
    parent_joint: Dict[str, tuple] = {}
    for j in range(p.getNumJoints(body)):
        info = p.getJointInfo(body, j)
        jname = info[1].decode()
        child = info[12].decode()
        parent = p.getJointInfo(body, info[16])[12].decode() if info[16] >= 0 \
            else base_link
        robot.joint_index[jname] = j
        robot.link_index[child] = j
        parent_joint[child] = (j, info[2], parent)

    # for each semantic link, climb past fixed joints to its driving joint
    def driving_joint(link: str):
        seen = set()
        while link in parent_joint and link not in seen:
            seen.add(link)
            j, jtype, parent = parent_joint[link]
            if jtype != p.JOINT_FIXED:
                return j
            link = parent
        return None

    for link in robot.link_index:
        j = driving_joint(link)
        if j is not None:
            robot.movable_joint[link] = j

    # group injected road wheels per side for the tracks actuator
    robot.track_wheels = {"left": [], "right": []}
    for link, j in robot.movable_joint.items():
        if fnmatch.fnmatch(link, "DrumW_L*"):
            robot.track_wheels["left"].append(j)
        elif fnmatch.fnmatch(link, "DrumW_R*"):
            robot.track_wheels["right"].append(j)

    # flipper belts: map each flipper's contact link(s) to its side so the tracks
    # actuator drives them at that side's belt speed. The paddle (FlipperX) is
    # fixed-lumped into its revolute FlipperX_pivot, so a ground contact may be
    # reported on either link index -- tag both.
    for link_name, side in (profile.raw.get("flipper_belts") or {}).items():
        s = "L" if str(side).lower().startswith("l") else "R"
        for nm in (link_name, link_name + "_pivot"):
            if nm in robot.link_index:
                robot.belt_links[robot.link_index[nm]] = s

    _apply_masses(robot)
    _apply_friction(robot)
    return robot
