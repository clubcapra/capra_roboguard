"""Fast native PyBullet window (no readback) for driving + friction painting.

The Qt panel renders headless and pulls each frame back from the GPU
(getCameraImage), which has a ~40 ms+ readback floor -> laggy. This tool runs
PyBullet in GUI mode: the GPU renders straight to an OpenGL window (XWayland on
Wayland), so it's smooth (60 fps) even with the terrain. Controls are keyboard +
on-window debug sliders; friction is painted by clicking the ground.

    PYTHONPATH=. ../rove_sim_venv/bin/python tools/live.py --profile standard --terrain

Drive (hold):  W/Up,S/Down = fwd/rev   A/Left,D/Right = turn   SPACE = stop
               Q/E = flippers down/up   I/K = arm fwd/back   J/L = arm yaw   U/O = arm up/down
Camera:  PyBullet built-in -- CTRL+left-drag = orbit, scroll = zoom, CTRL+middle-drag = pan.
Paint:   enable "PAINT" slider (>0.5), then LEFT-CLICK/drag the ground to paint friction.
Sensors: --sensors draws the live Livox cloud (height-coloured); P snapshots the cameras.
Scene:   N = spawn object in front, X = delete last, M = save; SAVE/EXPORT/SPAWN/DELETE
         buttons too. Load with --scene <file>.
"""
import argparse
import glob
import os
import sys
import time

import numpy as np
import pybullet as p

from rove_sim import runtime
from rove_sim.control import RoveControl, Tracks, Flippers, Ovis
from rove_sim.world.friction import MATERIALS

_X_HELP = """
Native window needs an X server (PyBullet GUI is X11/GLX; on Wayland that's
XWayland). This shell has no usable DISPLAY.

  * Open a REAL host GNOME Terminal (Activities -> "Terminal"), NOT an IDE /
    Flatpak embedded terminal, then:  echo $DISPLAY    (should be :0 or :1)
    and run this command again from there.
  * Or point it explicitly:           tools/live.py --display :0
    (if it says "Authorization required":  xhost +SI:localuser:$USER )
  * If no X server exists at all, use the Qt panel instead (Wayland-native):
      QT_QPA_PLATFORM=wayland PYTHONPATH=. ../rove_sim_venv/bin/python \\
          tools/gui.py --profile standard --terrain
"""

_COL = [("Core", (.25, .27, .3)), ("cage", (.2, .4, .75)), ("DrumW", (.1, .1, .12)),
        ("Drum", (.15, .15, .17)), ("Flipper", (.85, .7, .1)), ("Base", (.8, .45, .1)),
        ("Section", (.8, .45, .1)), ("Joint", (.8, .45, .1)), ("robotiq", (.9, .5, .1)),
        ("knuckle", (.9, .5, .1)), ("finger", (.9, .5, .1)), ("mid360", (.1, .6, .2)),
        ("livox", (.1, .6, .2)), ("camera", (.4, .4, .42)), ("vn300", (.5, .12, .12))]
_MATS = list(MATERIALS)


def _colorize(robot):
    for n, idx in robot.link_index.items():
        for key, c in _COL:
            if key in n:
                p.changeVisualShape(robot.body_id, idx, rgbaColor=[*c, 1]); break


def _down(keys, *codes):
    return 1.0 if any(keys.get(c, 0) & p.KEY_IS_DOWN for c in codes) else 0.0


def _ray_from_mouse(mx, my):
    """Reconstruct a world-space ray through the mouse pixel from the debug cam."""
    cam = p.getDebugVisualizerCamera()
    w, h = cam[0], cam[1]
    forward, horiz, vert = cam[5], cam[6], cam[7]
    dist, target = cam[10], cam[11]
    cam_pos = [target[i] - dist * forward[i] for i in range(3)]
    far = 1000.0
    ray_fwd = [forward[i] * far for i in range(3)]
    # NDC offsets (origin top-left)
    ox = (mx / w - 0.5)
    oy = (my / h - 0.5)
    ray_to = [cam_pos[i] + ray_fwd[i] + ox * horiz[i] - oy * vert[i] for i in range(3)]
    return cam_pos, ray_to


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--terrain", nargs="?",
                    const="../free_dirt_road_through_forest.glb", default=None)
    ap.add_argument("--display", default=None,
                    help="X display to use (e.g. :0). Overrides $DISPLAY.")
    ap.add_argument("--no-texture", action="store_true",
                    help="flat-colour terrain (a bit faster) instead of textured")
    ap.add_argument("--sensors", action="store_true",
                    help="draw the live Livox point cloud; P snapshots the cameras")
    ap.add_argument("--lidar-rays", type=int, default=4000,
                    help="rays/scan PER Livox for the live preview (default 4000)")
    ap.add_argument("--scene", default=None,
                    help="load a saved scene .json (terrain+friction+objects+pose); "
                         "press M in-window to save the current scene back to it")
    ap.add_argument("--rtsp", action="store_true",
                    help="also stream THIS robot's cameras as RTSP (open in VLC); "
                         "round-robin so it stays drivable")
    ap.add_argument("--publish-state", action="store_true",
                    help="write robot pose+joints to the shared file each frame so "
                         "parallel cam_worker processes can stream the cameras at "
                         "24fps without stalling the GUI (run WORKERS_ONLY=1 "
                         "tools/sim_fleet.sh alongside)")
    ap.add_argument("--cam-render-fps", type=float, default=20.0,
                    help="TOTAL camera renders/sec for --rtsp (per-cam = this/N). "
                         "Higher = smoother feeds but choppier driving (readback).")
    ap.add_argument("--cam-res", default="320x240",
                    help="--rtsp camera preview resolution (lower = faster readback)")
    args = ap.parse_args()
    if args.display:
        os.environ["DISPLAY"] = args.display

    overrides = {"friction": {"origin": (-25.0, -25.0), "extent": (50.0, 50.0),
                              "cell": 0.25}}
    if args.terrain:
        overrides["terrain"] = {"source": args.terrain,
                                "texture": not args.no_texture}
    try:
        if args.scene and os.path.exists(args.scene):
            # the scene carries terrain+friction (via build_overrides) + the robot
            # pose/joints/objects; --profile still picks the ROBOT (standard|caged).
            from rove_sim.world.scene import load_scene_sim
            sim = load_scene_sim(args.scene, args.profile, mode="gui")
            print(f"[scene] loaded {args.scene} with profile {args.profile!r}")
        else:
            sim = runtime.build(args.profile, mode="gui", world="mock",
                                world_overrides=overrides)
    except Exception as e:
        print(f"\n[live.py] could not open a native window: {e}\n{_X_HELP}")
        sys.exit(1)
    _colorize(sim.robot)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)        # faster
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    base = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])
    # Camera is PyBullet's built-in: CTRL+left-drag orbit, scroll zoom, CTRL+
    # middle-drag pan. We just set the initial view. WASD is the robot's (driving).
    p.resetDebugVisualizerCamera(4.0, 50, -30, base.tolist())

    # --- live sensors: draw BOTH Livox clouds in the 3D view; P snapshots cams --
    # The rove carries two Mid-360s (top dome up, bottom dome down) for full
    # spherical coverage; we merge their returns into one height-coloured cloud.
    livoxes = [s for s in sim.sensors if s.name.startswith("livox")] if args.sensors else []
    cams = ([s for s in sim.sensors if s.name.startswith("cam_")]
            if (args.sensors or args.rtsp) else [])

    # Optional RTSP: stream THIS GUI robot's cameras so you can watch the feeds in
    # VLC while you drive. Round-robin (one EGL readback per render tick) at a low
    # budget so the readback doesn't stall driving; the streams still emit CFR.
    feeds = None
    if args.rtsp and cams:
        from rove_sim.sensors.rtsp import RtspCameraFeeds
        for c in cams:                  # small preview res (4:3 kept) so the EGL
            c.width, c.height = 320, 240  # readback doesn't stall driving
        feeds = RtspCameraFeeds(cams, fps=15.0, render_fps=6.0).start()
        if feeds.streams:
            print("[rtsp] open in VLC:  " + "   ".join(feeds.urls()))
        else:
            print("[rtsp] not available (need mediamtx + ffmpeg)")
    pc_item = [-1]                                 # addUserDebugPoints handle (reused)
    last_scans = [None] * len(livoxes)
    if args.sensors:
        # The full Mid-360 is 20k rays/scan; that many debug points redrawn 10x/s
        # chokes the GUI. Cap the LIVE preview (raycast is multithreaded; the
        # headless/autonomy path keeps the full budget) for a smooth cloud.
        for lv in livoxes:
            lv.set_rays(int(args.lidar_rays))
        print(f"[sensors] {len(livoxes)} Livox live cloud(s) @ {args.lidar_rays} rays "
              f"each; P = snapshot {len(cams)} camera(s) to /tmp/rove_cam_*.png")

    field = sim.world.friction
    tr = next((a for a in sim.actuators if a.intent_field == "tracks"), None)
    has_arm = any(a.intent_field == "ovis" for a in sim.actuators)

    s_belt = p.addUserDebugParameter("belt speed (rad/s)", 10, 60,
                                     tr.max_rad_s if tr else 46)
    s_mul = p.addUserDebugParameter("mu longitudinal", 0.05, 1.2,
                                    tr.mu_long if tr else 0.6)
    s_mut = p.addUserDebugParameter("mu lateral", 0.03, 0.8, tr.mu_lat if tr else 0.15)
    s_mat = p.addUserDebugParameter(f"material 0=ice..{len(_MATS)-1}", 0, len(_MATS) - 1, 4)
    s_brush = p.addUserDebugParameter("brush radius (m)", 0.2, 3.0, 0.8)
    s_paint = p.addUserDebugParameter("PAINT mode (>0.5)", 0, 1, 0)

    # --- on-screen robot controls (mouse-only operation; keyboard still works) --
    # Throttle sliders: 0 = stop, slide to drive/turn. They ADD to the keyboard
    # (WASD/arrows), so either input drives the robot.
    s_drive = p.addUserDebugParameter(">> DRIVE  back -1 .. +1 fwd", -1, 1, 0)
    s_steer = p.addUserDebugParameter(">> STEER  left -1 .. +1 right", -1, 1, 0)
    # Buttons (click to act): a pybullet "button" is a param with min>max; its read
    # value increments by 1 each click, so we latch on a change.
    b_fdown = p.addUserDebugParameter("Flippers DOWN (deploy)", 1, 0, 0)
    b_fup = p.addUserDebugParameter("Flippers UP (retract)", 1, 0, 0)
    b_stop = p.addUserDebugParameter("STOP (halt + flippers neutral)", 1, 0, 0)
    # Scene authoring: Save -> args.scene; Export -> a fresh numbered file. Keys
    # N = spawn a marker in front of the robot, X = delete the last spawned one.
    b_save = p.addUserDebugParameter("SAVE scene", 1, 0, 0)
    b_export = p.addUserDebugParameter("EXPORT scene (new file)", 1, 0, 0)
    b_spawn = p.addUserDebugParameter("SPAWN object (front)", 1, 0, 0)
    b_del = p.addUserDebugParameter("DELETE last object", 1, 0, 0)
    btn = {b_fdown: 0.0, b_fup: 0.0, b_stop: 0.0,
           b_save: 0.0, b_export: 0.0, b_spawn: 0.0, b_del: 0.0}  # last-seen clicks
    export_n = [0]
    spawned = []                                        # ids we spawned this session
    flip_latch = [0]                                    # persistent flipper command
    halt = [False]                                      # STOP latch (drive held at 0)
    last_thr = [0.0, 0.0]                               # last drive/steer slider values

    def clicked(b):
        v = p.readUserDebugParameter(b)
        hit = v != btn[b]
        btn[b] = v
        return hit

    def paint_at(x, y):
        a, b = _ray_from_mouse(x, y)
        hit = p.rayTest(a, b)[0]
        if hit[0] >= 0:
            field.paint_material(hit[3][0], hit[3][1],
                                 p.readUserDebugParameter(s_brush),
                                 _MATS[int(round(p.readUserDebugParameter(s_mat)))])

    def spawn_front():
        """Drop a marker object ~1.5 m in front of the robot (front is -X here)."""
        from rove_sim.world.scene import SceneObject
        if not hasattr(sim.world, "spawn_object"):
            print("[scene] this world can't spawn objects"); return
        pos, orn = p.getBasePositionAndOrientation(sim.robot.body_id)
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        wp = np.array(pos) + R @ np.array([-1.5, 0.0, 0.3])
        oid = f"obj_{len(spawned) + 1}"
        sim.world.spawn_object(SceneObject(id=oid, pose=tuple(wp), cls="object"))
        spawned.append(oid)
        print(f"[scene] spawned {oid} at {tuple(round(v,2) for v in wp)}")

    def save_scene(path):
        from rove_sim.world.scene import capture_scene
        capture_scene(sim).save(path)
        print(f"[scene] saved -> {path}")

    dt = 1.0 / sim.control_hz
    UP, DN, LF, RT = p.B3G_UP_ARROW, p.B3G_DOWN_ARROW, p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW
    while p.isConnected():
        t0 = time.time()
        if tr:
            tr.max_rad_s = p.readUserDebugParameter(s_belt)
            tr.v_max = tr.max_rad_s * tr.drum_radius
            tr.mu_long = p.readUserDebugParameter(s_mul)
            tr.mu_lat = p.readUserDebugParameter(s_mut)

        # buttons (mouse): latch flipper state / stop
        if clicked(b_fdown):
            flip_latch[0] = -1            # deploy DOWN (a -1 cmd plants all flippers)
        if clicked(b_fup):
            flip_latch[0] = 1             # retract UP
        if clicked(b_stop):
            flip_latch[0] = 0
            halt[0] = True               # hold drive at 0 until an input changes
        # scene authoring buttons
        if clicked(b_save):
            save_scene(args.scene or "scene.json")
        if clicked(b_export):
            export_n[0] += 1
            save_scene(f"scene_export_{export_n[0]}.json")
        if clicked(b_spawn):
            spawn_front()
        if clicked(b_del) and spawned:
            oid = spawned.pop()
            sim.world.remove_object(oid)
            print(f"[scene] deleted {oid}")

        keys = p.getKeyboardEvents()
        # drive = keyboard (WASD/arrows, hold) + throttle sliders (mouse). Either works.
        # (Camera is PyBullet's built-in: CTRL+drag orbit, scroll zoom, CTRL+mid pan.)
        thr_d = p.readUserDebugParameter(s_drive)
        thr_s = p.readUserDebugParameter(s_steer)
        kb_fwd = _down(keys, UP, ord('w')) - _down(keys, DN, ord('s'))
        kb_turn = _down(keys, RT, ord('d')) - _down(keys, LF, ord('a'))
        # friction painting (mouse): with the PAINT slider on, LEFT-CLICK/drag ground
        if field is not None and p.readUserDebugParameter(s_paint) > 0.5:
            for e in p.getMouseEvents():
                if e[3] == 0 and (e[4] & p.KEY_IS_DOWN or e[4] & p.KEY_WAS_TRIGGERED):
                    paint_at(e[1], e[2])
        # any fresh input (slider moved or a drive key) releases the STOP latch
        if thr_d != last_thr[0] or thr_s != last_thr[1] or kb_fwd or kb_turn:
            halt[0] = False
        last_thr[0], last_thr[1] = thr_d, thr_s
        fwd = kb_fwd + thr_d
        turn = kb_turn + thr_s
        if halt[0] or keys.get(ord(' '), 0) & p.KEY_IS_DOWN:
            fwd = turn = 0.0
        tracks = Tracks(float(np.clip(fwd - turn, -1, 1)),
                        float(np.clip(fwd + turn, -1, 1)))
        # flippers: keyboard q/e momentary overrides the button latch when pressed
        fl_kb = int(_down(keys, ord('e')) - _down(keys, ord('q')))   # e=up, q=down
        fl = fl_kb if fl_kb != 0 else flip_latch[0]
        flippers = Flippers(fl, fl, fl, fl)
        ovis = Ovis()
        if has_arm:
            ovis.vx = _down(keys, ord('i')) - _down(keys, ord('k'))
            ovis.vz = _down(keys, ord('u')) - _down(keys, ord('o'))
            ovis.wz = _down(keys, ord('j')) - _down(keys, ord('l'))

        sim.set_intent(RoveControl(tracks=tracks, flippers=flippers, ovis=ovis))
        if not p.isConnected():                  # window closed mid-loop
            break
        sim.step_control(1)

        # --- live sensors ---------------------------------------------------
        if livoxes:
            changed = False
            for i, lv in enumerate(livoxes):
                lv.update(dt)                        # rate-gated to the Livox rate
                if lv.last is not None and lv.last is not last_scans[i]:
                    last_scans[i] = lv.last; changed = True
            if changed:
                clouds = [s.points for s in last_scans
                          if s is not None and len(s.points)]
                if clouds:
                    pts = np.concatenate(clouds)
                    z = pts[:, 2]                    # height colour (blue->red)
                    t = np.clip((z - z.min()) / (np.ptp(z) + 1e-6), 0, 1)
                    cols = np.stack([t, 0.3 + 0.0 * t, 1.0 - t], 1).tolist()
                    pc_item[0] = p.addUserDebugPoints(
                        pts.tolist(), cols, pointSize=2.0,
                        replaceItemUniqueId=pc_item[0] if pc_item[0] >= 0 else -1)
            if keys.get(ord('p'), 0) & p.KEY_WAS_TRIGGERED:
                from PIL import Image
                for c in cams:
                    Image.fromarray(c.sample().rgb).save(f"/tmp/rove_{c.name}.png")
                print(f"[sensors] saved {len(cams)} camera frame(s) -> /tmp/rove_cam_*.png")
        if keys.get(ord('m'), 0) & p.KEY_WAS_TRIGGERED:   # M: save the scene
            save_scene(args.scene or "scene.json")
        if keys.get(ord('n'), 0) & p.KEY_WAS_TRIGGERED:   # N: spawn object in front
            spawn_front()
        if keys.get(ord('x'), 0) & p.KEY_WAS_TRIGGERED and spawned:  # X: delete last
            oid = spawned.pop(); sim.world.remove_object(oid)
            print(f"[scene] deleted {oid}")
        if feeds is not None:
            feeds.publish(dt)                              # stream cameras (RTSP)
        if args.publish_state:                             # feed parallel cam_workers
            from rove_sim.world.render_sync import publish_robot_state
            publish_robot_state(sim.robot)
        time.sleep(max(0.0, dt - (time.time() - t0)))
    if feeds is not None:
        feeds.stop()
    sim.engine.disconnect()


if __name__ == "__main__":
    main()
