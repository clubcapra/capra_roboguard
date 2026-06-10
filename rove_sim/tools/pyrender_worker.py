#!/usr/bin/env python3
"""pyrender_worker: a decoupled camera renderer that uses PYRENDER, not pybullet.

pybullet's renderer drops per-texel texture alpha (foliage -> opaque green shards)
and has weak depth (z-fighting "flicker"). This worker renders the SAME cameras
with pyrender: real glTF alpha (transparent leaves), correct textures, proper
depth (no flicker), and a numpy framebuffer (no tuple readback -> faster, higher
res). It mirrors the sim's robot from the shared state file (like cam_worker) and
streams its assigned cameras over RTSP.

    tools/pyrender_worker.py --terrain --cameras cam_front,cam_arm --res 512x384
"""
import argparse
import os
import sys
import time

os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pybullet as p
import trimesh
import pyrender
import pyrender.light as _pr_light
import pyrender.renderer as _pr_renderer

# The directional shadow camera spans the whole ~260 m scene, so a 2048² shadow
# map is ~0.13 m/texel -> dropping to 1024 barely changes the (already coarse)
# shadow but halves the shadow-map fill cost. Patch the names bound in BOTH modules
# (they `from .constants import SHADOW_TEX_SZ`, so patching constants alone is moot).
_SHADOW_SZ = 1024
_pr_light.SHADOW_TEX_SZ = _SHADOW_SZ
_pr_renderer.SHADOW_TEX_SZ = _SHADOW_SZ

from rove_sim import runtime
from rove_sim.world import terrain as terrain_mod
from rove_sim.world.render_sync import (apply_robot_state, read_robot_state,
                                        DEFAULT_STATE_FILE)
from rove_sim.sensors.rtsp import RtspCameraFeeds


_TEXMAX = 512   # cap GLB texture size -> the GPU has limited memory and multiple
                # render workers each load the scene; 512 is plenty (no photoreal).

# Per-part robot colours, matching live.py `_colorize` (the GUI). The robot CAD
# meshes are untextured gray; we colour by LINK NAME here. NOTE: we do NOT read
# the colour back from getVisualShapeData -- pybullet returns the URDF gray
# default there, not a changeVisualShape() override (that's why the feeds were
# gray). Longest keys first so "DrumW" wins over "Drum", etc.
_COL = [("DrumW", (.1, .1, .12)), ("Core", (.25, .27, .3)), ("cage", (.2, .4, .75)),
        ("Drum", (.15, .15, .17)), ("Flipper", (.85, .7, .1)), ("Base", (.8, .45, .1)),
        ("Section", (.8, .45, .1)), ("Joint", (.8, .45, .1)), ("robotiq", (.9, .5, .1)),
        ("knuckle", (.9, .5, .1)), ("finger", (.9, .5, .1)), ("pad", (.95, .55, .15)),
        ("mid360", (.1, .6, .2)), ("livox", (.1, .6, .2)), ("camera", (.35, .35, .38)),
        ("vn300", (.5, .12, .12))]


def _color_for(name: str):
    """Part colour by link-name substring (falls back to a neutral gray-orange)."""
    for key, col in _COL:
        if key.lower() in name.lower():
            return col
    return (0.6, 0.45, 0.3)


def _sun_pose(direction, dist=80.0):
    """Pose for a DirectionalLight shining along `direction` (world-down-ish).
    pyrender's light shines along its local -Z, so build a basis with z=-dir."""
    d = np.asarray(direction, float); d = d / np.linalg.norm(d)
    z = -d
    up = np.array([0., 0., 1.]) if abs(z[2]) < 0.95 else np.array([0., 1., 0.])
    x = np.cross(up, z); x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2] = x, y, z
    pose[:3, 3] = -d * dist
    return pose


def _fix_textures(m):
    mat = getattr(getattr(m, "visual", None), "material", None)
    if mat is None:
        return
    from PIL import Image as _Im
    for a in ("baseColorTexture", "image", "emissiveTexture", "metallicRoughnessTexture"):
        img = getattr(mat, a, None)
        if img is None:
            continue
        if getattr(img, "mode", "RGBA") not in ("RGB", "RGBA"):
            img = img.convert("RGBA")            # 2-channel -> RGBA (alpha kept)
        if max(img.size) > _TEXMAX:              # downscale to fit GPU memory
            s = _TEXMAX / max(img.size)
            img = img.resize((max(1, int(img.width * s)), max(1, int(img.height * s))),
                             _Im.LANCZOS)
        setattr(mat, a, img)


def build_terrain(pr_scene, glb):
    """Replicate the terrain converter's transform on the ORIGINAL GLB (keeping
    its alpha materials) so the pyrender scene aligns with the pybullet world.

    Returns the list of (mesh, aabb_min, aabb_max) for the static terrain meshes,
    so the render loop can frustum-cull the ones outside each camera's view."""
    scene = trimesh.load(glb, force="scene")
    R = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
    items, ground = [], []
    for node in scene.graph.nodes_geometry:
        Tn, gname = scene.graph.get(node)
        m = scene.geometry[gname].copy()
        m.apply_transform(Tn); m.apply_transform(R)
        items.append((gname, m))
        if terrain_mod._is_ground(gname):
            ground.append(m)
    pts = trimesh.util.concatenate(ground or [m for _, m in items]).vertices
    bx, by, bz = terrain_mod._flat_spot(pts)
    off = [-bx, -by, -bz]
    terrain_meshes = []
    for gname, m in items:
        if any(k in terrain_mod._suffix(gname) for k in terrain_mod.EXCLUDE_KEYWORDS):
            continue
        m.apply_translation(off)
        _fix_textures(m)
        mesh = pyrender.Mesh.from_trimesh(m, smooth=False)
        is_foliage = terrain_mod._category(gname) in ("grass", "foliage")
        # set alphaMode on the PYRENDER material (from_trimesh doesn't carry the
        # glTF alphaMode over). MASK = alpha-TEST: transparent leaf texels are
        # DISCARDED (no depth write) so the trees/ground behind show through the
        # gaps -- NOT the sky (that's what BLEND+depth-write does). Ground/rock are
        # forced OPAQUE (the dirt-road layers are BLEND in the GLB -> they'd vanish).
        for prim in mesh.primitives:
            if prim.material is None:
                continue
            # MASK = alpha-cutout (our patched pyrender discards transparent leaf
            # texels): the gaps show what's actually BEHIND via real depth (trees/
            # ground), not the sky bg that BLEND bleeds. Ground/rock OPAQUE.
            if is_foliage:
                prim.material.alphaMode = "MASK"; prim.material.alphaCutoff = 0.5
            else:
                prim.material.alphaMode = "OPAQUE"
        pr_scene.add(mesh)
        b = m.bounds                                  # world AABB (static mesh)
        terrain_meshes.append((mesh, np.array(b[0], float), np.array(b[1], float)))
    return terrain_meshes


def _frustum_planes(clip):
    """6 frustum planes (Gribb–Hartmann) from a row-major clip matrix
    (clip-space = clip @ world_point). Each row is (a,b,c,d); inside = >= 0."""
    m = clip
    return np.array([m[3] + m[0], m[3] - m[0],      # left, right
                     m[3] + m[1], m[3] - m[1],      # bottom, top
                     m[3] + m[2], m[3] - m[2]])     # near, far


def _aabb_visible(planes, mn, mx):
    """AABB-vs-frustum: cull only if the box is fully outside some plane
    (test the box's positive vertex per plane). Conservative -> no false cull."""
    for pl in planes:
        px = mx[0] if pl[0] >= 0 else mn[0]
        py = mx[1] if pl[1] >= 0 else mn[1]
        pz = mx[2] if pl[2] >= 0 else mn[2]
        if pl[0] * px + pl[1] * py + pl[2] * pz + pl[3] < 0.0:
            return False
    return True


def _add_mesh_node(pr_scene, nodes, fn, dims, lpos, lorn, name, link):
    try:
        rm = trimesh.load(fn, force="mesh")
    except Exception as e:
        print(f"[pyrender_worker] FAILED to load {fn}: {e}", file=sys.stderr)
        return
    rm.apply_scale(dims)
    col = _color_for(name)
    material = pyrender.MetallicRoughnessMaterial(
        baseColorFactor=[*col, 1.0], metallicFactor=0.0, roughnessFactor=0.85)
    # smooth=True -> interpolated vertex normals, so curved parts (drums, gripper)
    # and angled panels shade with a gradient instead of one flat tone per face
    # (the "flat colored cutout" look). Pure-diffuse material (metallic 0) so the
    # directional sun actually drives the shading.
    node = pr_scene.add(pyrender.Mesh.from_trimesh(rm, material=material, smooth=True))
    nodes.append((node, link, np.array(lpos), np.array(lorn)))


def build_robot(pr_scene, robot):
    """One pyrender node per robot visual mesh, coloured by link name (matching
    the GUI). returns (node, link, local_pose) for per-frame posing.

    Gripper caveat: the Robotiq 2F-140 is a closed-loop four-bar linkage, which
    pybullet can't represent in a URDF -> it loads the RIGHT-side finger links
    WITHOUT their visual meshes (getVisualShapeData gives them no mesh). Those
    links use the SAME STLs as their LEFT twins, so we fill the missing ones from
    the left counterpart and pose them by the right link's real frame -> both
    fingers show."""
    nodes = []
    idx_to_name = {i: n for n, i in robot.link_index.items()}
    loaded = {}                                       # link name -> (fn, dims)
    have_visual = set()                               # links with ANY visual entry
    n_vis = 0
    for v in p.getVisualShapeData(robot.body_id):
        link, gtype, dims, mesh, lpos, lorn = v[1], v[2], v[3], v[4], v[5], v[6]
        name = idx_to_name.get(link, str(link))
        have_visual.add(link)
        if gtype != p.GEOM_MESH or not mesh:
            continue
        n_vis += 1
        fn = mesh.decode() if isinstance(mesh, bytes) else mesh
        if not os.path.exists(fn):
            print(f"[pyrender_worker] MISSING visual mesh for {name}: {fn}", file=sys.stderr)
            continue
        loaded[name] = (fn, dims)
        _add_mesh_node(pr_scene, nodes, fn, dims, lpos, lorn, name, link)

    # Fill links that have NO mesh visual from their left/right twin's STL (the
    # gripper four-bar links pybullet dropped). Pose by the link's own frame.
    filled = 0
    for link, name in idx_to_name.items():
        if link in have_visual and name in loaded:
            continue                                  # already rendered
        twin = (name.replace("right_", "left_") if "right_" in name
                else name.replace("left_", "right_") if "left_" in name else None)
        if twin and twin in loaded and name not in loaded:
            fn, dims = loaded[twin]
            _add_mesh_node(pr_scene, nodes, fn, dims, [0, 0, 0], [0, 0, 0, 1], name, link)
            filled += 1
    print(f"[pyrender_worker] robot: {len(nodes)} meshes "
          f"({n_vis} from pybullet, {filled} twin-filled e.g. gripper right side)")
    return nodes


def _compose(pos, orn):
    M = np.eye(4)
    M[:3, :3] = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    M[:3, 3] = pos
    return M


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--terrain", nargs="?",
                    const="../free_dirt_road_through_forest.glb", default=None)
    ap.add_argument("--cameras", required=True)
    ap.add_argument("--fps", type=float, default=24.0)
    ap.add_argument("--res", default="512x384")
    ap.add_argument("--encoder", default="auto")
    ap.add_argument("--port", type=int, default=8554)
    ap.add_argument("--shared-server", action="store_true")
    ap.add_argument("--no-shadows", action="store_true",
                    help="disable directional shadows (recovers ~2x fps)")
    ap.add_argument("--texmax", type=int, default=512,
                    help="cap GLB texture size (px). Drop to 128 on a 2GB GPU.")
    ap.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    from rove_sim.core.util import die_with_parent
    die_with_parent()                    # no orphans if the fleet launcher is killed

    global _TEXMAX
    _TEXMAX = int(args.texmax)            # tiny 2GB GPU -> 128

    glb = args.terrain or "../free_dirt_road_through_forest.glb"
    # pybullet loads the terrain GEOMETRY (texture:False -> no GPU textures, which
    # is what blew up GPU memory) so the robot sits at the right height and the
    # camera poses align with the PYRENDER terrain. pyrender does the textured render.
    overrides = {"friction": {"origin": (-25., -25.), "extent": (50., 50.), "cell": 0.25}}
    if args.terrain:
        overrides["terrain"] = {"source": glb, "texture": False}
    # pybullet here only mirrors robot kinematics for camera poses; PYRENDER owns
    # the GPU. egl=False -> pybullet stays CPU-only so the renderer is the single
    # GPU context (critical on a 2 GB card).
    sim = runtime.build(args.profile, mode="headless", world="mock",
                        world_overrides=overrides, egl=False)
    p.setGravity(0, 0, 0)

    # Robot meshes are coloured by link name in build_robot() (the GUI palette) --
    # we do NOT rely on changeVisualShape + getVisualShapeData here (pybullet hands
    # back the URDF gray default, which is what made the feeds gray).

    want = [n.strip() for n in args.cameras.split(",") if n.strip()]
    cams = [s for s in sim.sensors if s.name in want]
    if not cams:
        print(f"[pyrender_worker] no cameras matched {want}", file=sys.stderr); sys.exit(2)
    w, h = (int(v) for v in args.res.split("x"))
    for c in cams:
        c.width, c.height = w, h

    # ---- pyrender scene (built once) ----
    # Lower ambient so the directional-light shadows actually read (high ambient
    # washes them out); bump the sun intensity so lit areas stay bright.
    pr_scene = pyrender.Scene(bg_color=[150, 175, 205], ambient_light=[0.30, 0.30, 0.30])
    terrain_meshes = build_terrain(pr_scene, glb)
    robot_nodes = build_robot(pr_scene, sim.robot)
    renderer = pyrender.OffscreenRenderer(w, h)
    # Angled "afternoon" sun so objects cast oblique, visible shadows AND surfaces
    # facing it are clearly brighter than those facing away (3-D shading). Ambient
    # is kept low enough that the directional contribution isn't washed out.
    sun = pr_scene.add(pyrender.DirectionalLight(color=[1, 1, 1], intensity=6.0),
                       pose=_sun_pose([0.55, 0.4, -1.0]))
    # FIX SHADOWS: pyrender fits the directional shadow ortho to the WHOLE ~260 m
    # scene -> the robot's shadow is ~4 texels (invisible). Override the light's
    # shadow camera to a tight span around the robot's roaming area (±35 m; the
    # friction grid is ±25 m) so the robot + nearby objects cast a CRISP shadow.
    _SHADOW_SPAN_M = 35.0
    sun.light._get_shadow_camera = (                     # bound as an instance attr
        lambda scene_scale: pyrender.OrthographicCamera(
            xmag=_SHADOW_SPAN_M, ymag=_SHADOW_SPAN_M,
            znear=0.05, zfar=3.0 * scene_scale + 2.0 * _SHADOW_SPAN_M))
    cam_node = pr_scene.add(pyrender.PerspectiveCamera(yfov=1.0, znear=0.03, zfar=80))

    enc = args.encoder
    if enc == "auto":
        import subprocess as _sp
        try:
            out = _sp.run(["ffmpeg", "-hide_banner", "-encoders"], capture_output=True,
                          text=True, timeout=5).stdout
            enc = "h264_nvenc" if "h264_nvenc" in out else "libx264"
        except Exception:
            enc = "libx264"
    feeds = RtspCameraFeeds(cams, fps=args.fps, port=args.port, encoder=enc,
                            manage_server=not args.shared_server).start()
    streams = feeds.streams
    if not streams:
        print("[pyrender_worker] RTSP unavailable", file=sys.stderr); sys.exit(3)
    print(f"[pyrender_worker] {len(cams)} cam(s) @ {w}x{h} {args.fps:g}fps [{enc}] "
          f"PYRENDER (alpha+textures): " + "  ".join(feeds.urls()))

    period = 1.0 / args.fps
    render_flags = (pyrender.RenderFlags.NONE if args.no_shadows
                    else pyrender.RenderFlags.SHADOWS_DIRECTIONAL)   # directional sun shadows
    rt0 = time.time(); n = 0
    try:
        while True:
            t0 = time.time()
            apply_robot_state(sim.robot, read_robot_state(args.state_file))
            # pose the robot meshes from the (now mirrored) kinematics
            for node, link, lpos, lorn in robot_nodes:
                if link == -1:
                    pos, orn = p.getBasePositionAndOrientation(sim.robot.body_id)
                else:
                    stt = p.getLinkState(sim.robot.body_id, link, computeForwardKinematics=True)
                    pos, orn = stt[4], stt[5]
                pr_scene.set_pose(node, _compose(pos, orn) @ _compose(lpos, lorn))
            vis_last = 0
            for c in cams:
                _, _, eye, view = c._view()
                view_rm = np.array(view, float).reshape(4, 4).T   # world -> camera
                cam_node.camera.yfov = np.deg2rad(c.hfov) / (c.width / c.height)
                pr_scene.set_pose(cam_node, np.linalg.inv(view_rm))
                # frustum-cull terrain: only draw meshes inside THIS camera's view.
                planes = _frustum_planes(
                    cam_node.camera.get_projection_matrix(c.width, c.height) @ view_rm)
                vis_last = 0
                for mesh, mn, mx in terrain_meshes:
                    mesh.is_visible = _aabb_visible(planes, mn, mx)
                    vis_last += mesh.is_visible
                try:
                    color, _ = renderer.render(pr_scene, flags=render_flags)
                except Exception as e:
                    if render_flags:        # disable shadows once, keep the feed alive
                        print(f"[pyrender_worker] shadows disabled ({e})", file=sys.stderr)
                        render_flags = pyrender.RenderFlags.NONE
                    color, _ = renderer.render(pr_scene, flags=render_flags)
                streams[c.name].push(np.ascontiguousarray(color))
            n += 1
            if time.time() - rt0 >= 3.0:
                print(f"[pyrender_worker:{cams[0].name}+] {n / (time.time() - rt0):.1f} fps/cam "
                      f"({vis_last}/{len(terrain_meshes)} terrain meshes in view)")
                rt0 = time.time(); n = 0
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        feeds.stop(); renderer.delete(); sim.disconnect()


if __name__ == "__main__":
    main()
