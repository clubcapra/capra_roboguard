"""Live multi-robot locomotion tuner / visualiser.

Spawns 3 robots running fixed behaviours side-by-side -- PIVOT (point-turn),
FORWARD (straight), ARC (gentle turn) -- and lets you see what each locomotion
param does to each behaviour.

    --render : sweep presets and render an MP4 with the params + per-robot
               metrics (ground speed, roll wobble) burned into each frame.
    --gui    : open a window with live sliders (run on your desktop).
    (none)   : headless, print metrics per preset.

Params: belt forward speed, turn speed cap, lateral (scrub) friction, drive
force, fraction of belt cylinders actually driven (rest free-roll), contact k.
"""
import argparse
import os
import time
import subprocess
import numpy as np
import pybullet as p
from PIL import Image, ImageDraw

from rove_sim.core.engine import Engine, EngineConfig
from rove_sim.robot import loader
from rove_sim.robot.profile import load_profile
from rove_sim.world.mock import MockWorld
from rove_sim.control import RoveControl, Tracks
from rove_sim.robot.actuation import build_actuators

BEHAVIORS = [("PIVOT", Tracks(-1.0, 1.0)),
             ("FORWARD", Tracks(1.0, 1.0)),
             ("ARC", Tracks(0.25, 0.7))]
W, H, FPS = 1100, 620, 30


def prof_path(name):
    here = os.path.join(os.path.dirname(__file__), "..", "profiles", f"{name}.yaml")
    return here if os.path.exists(here) else name


def apply_params(rb, tr, P):
    tr.max_rad_s = P["belt"]
    tr.turn_max_rad_s = P["turn"]
    n = len(tr.left)
    k = max(1, int(round(P["driven_frac"] * n)))
    step = max(1, n // k)
    tr._driven = set(range(0, n, step))
    tr._dforce = P["force"] * 2 / max(1, len(tr._driven))
    for L, idx in rb.link_index.items():
        if L.startswith("DrumW"):
            kw = dict(lateralFriction=1.4, frictionAnchor=1,
                      anisotropicFriction=[1.0, P["lat"], 1.0], restitution=0)
            if P["cstiff"] > 0:
                kw["contactStiffness"] = P["cstiff"] * 1000
                kw["contactDamping"] = P["cstiff"] * 130
            p.changeDynamics(rb.body_id, idx, **kw)


def drive(rb, tr, beh):
    turning = beh.left_vel * beh.right_vel < 0
    max_rad = tr.turn_max_rad_s if turning else tr.max_rad_s
    for side, vel, sign in ((tr.left, beh.left_vel, tr.left_sign),
                            (tr.right, beh.right_vel, tr.right_sign)):
        for i, j in enumerate(side):
            if i in tr._driven:
                p.setJointMotorControl2(rb.body_id, j, p.VELOCITY_CONTROL,
                                        targetVelocity=sign * vel * max_rad,
                                        force=tr._dforce)
            else:
                p.setJointMotorControl2(rb.body_id, j, p.VELOCITY_CONTROL, force=0)


def spawn(eng, profile, n=3):
    robots = []
    for i, (name, beh) in enumerate(BEHAVIORS):
        prof = load_profile(prof_path(profile))
        prof.model.base_position = [0.0, i * 3.0, 0.5]
        rb = loader.load(eng, prof)
        tr = next(a for a in build_actuators(prof.actuators, rb)
                  if a.intent_field == "tracks")
        robots.append([name, rb, tr, beh, [i * 3.0]])
    return robots


def reset(rb, y):
    p.resetBasePositionAndOrientation(rb.body_id, [0, y, 0.3], [0, 0, 0, 1])
    p.resetBaseVelocity(rb.body_id, [0, 0, 0], [0, 0, 0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--out", default="media/tune.mp4")
    args = ap.parse_args()

    mode = "gui" if args.gui else "headless"
    eng = Engine(EngineConfig(mode=mode)).connect()
    MockWorld(eng, {}).build()                   # gravity + ground (moved off the engine)
    robots = spawn(eng, args.profile)

    P = dict(belt=53.0, turn=8.0, lat=0.3, force=100.0, driven_frac=1.0, cstiff=30.0)

    if args.gui:
        p.resetDebugVisualizerCamera(7, 50, -32, [0, 3, 0])
        S = {k: p.addUserDebugParameter(lbl, lo, hi, P[k]) for k, (lbl, lo, hi) in {
            "belt": ("belt_speed", 10, 60), "turn": ("turn_speed", 3, 30),
            "lat": ("lateral_friction", 0.05, 1.5), "force": ("drive_force", 10, 150),
            "driven_frac": ("driven_fraction", 0.1, 1.0), "cstiff": ("contact_k", 0, 200),
        }.items()}
        print("[tune] 3 robots: PIVOT / FORWARD / ARC. Tune sliders live; Ctrl-C to exit.")
        try:
            while True:
                for k, sid in S.items():
                    P[k] = p.readUserDebugParameter(sid)
                for name, rb, tr, beh, st in robots:
                    apply_params(rb, tr, P); drive(rb, tr, beh)
                p.stepSimulation(); time.sleep(1 / 240)
        except KeyboardInterrupt:
            pass
        eng.disconnect(); return

    # ---- render / headless preset sweep ----
    presets = [
        dict(P),
        dict(P, belt=35),
        dict(P, driven_frac=0.25),
        dict(P, driven_frac=0.25, belt=35),
        dict(P, lat=0.7),
        dict(P, cstiff=0),
    ]
    ff = None
    if args.render:
        proj = p.computeProjectionMatrixFOV(60, W / H, 0.1, 60)
        ff = subprocess.Popen(
            ["ffmpeg", "-y", "-f", "rawvideo", "-pixel_format", "rgba",
             "-video_size", f"{W}x{H}", "-framerate", str(FPS), "-i", "-",
             "-pix_fmt", "yuv420p", "-loglevel", "error", args.out],
            stdin=subprocess.PIPE)

    for pi, preset in enumerate(presets):
        for name, rb, tr, beh, st in robots:
            reset(rb, BEHAVIORS_idx(name) * 3.0)
            apply_params(rb, tr, preset)
        roll = {n: [] for n, *_ in robots}
        for step in range(int(4.0 * 240)):
            for name, rb, tr, beh, st in robots:
                drive(rb, tr, beh)
                roll[name].append(np.degrees(p.getEulerFromQuaternion(
                    p.getBasePositionAndOrientation(rb.body_id)[1])[0]))
            p.stepSimulation()
            if args.render and step % 8 == 0:
                metrics = {}
                for name, rb, *_ in robots:
                    v, _ = p.getBaseVelocity(rb.body_id)
                    metrics[name] = (np.linalg.norm(v[:2]),
                                     np.ptp(roll[name][-120:]) if roll[name] else 0)
                frame(ff, robots, proj, preset, metrics)
        # log
        out = [f"preset{pi}: belt={preset['belt']:.0f} turn={preset['turn']:.0f} "
               f"lat={preset['lat']:.2f} driven={preset['driven_frac']:.2f} "
               f"k={preset['cstiff']:.0f}"]
        for name, rb, *_ in robots:
            v, _ = p.getBaseVelocity(rb.body_id)
            out.append(f"{name} v={np.linalg.norm(v[:2]):.1f} wob={np.ptp(roll[name][-120:]):.0f}d")
        print("  ".join(out))
    if ff:
        ff.stdin.close(); ff.wait(); print(f"wrote {args.out}")
    eng.disconnect()


def BEHAVIORS_idx(name):
    return [n for n, _ in BEHAVIORS].index(name)


def frame(ff, robots, proj, preset, metrics):
    # camera follows the 3 robots' centroid
    cy = np.mean([p.getBasePositionAndOrientation(rb.body_id)[0][1]
                  for _, rb, *_ in robots])
    cx = np.mean([p.getBasePositionAndOrientation(rb.body_id)[0][0]
                  for _, rb, *_ in robots])
    view = p.computeViewMatrix([cx + 4.5, cy, 4.0], [cx, cy, 0.2], [0, 0, 1])
    _, _, rgb, _, _ = p.getCameraImage(W, H, view, proj, lightDirection=[-1, -1, 2])
    img = Image.fromarray(np.reshape(rgb, (H, W, 4))[:, :, :3].astype(np.uint8))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, W, 78], fill=(20, 20, 30))
    d.text((10, 6), f"belt={preset['belt']:.0f}rad/s  turn={preset['turn']:.0f}  "
           f"lateral_fric={preset['lat']:.2f}  drive_force={preset['force']:.0f}  "
           f"driven={preset['driven_frac']*100:.0f}%  contact_k={preset['cstiff']:.0f}",
           fill=(255, 255, 120))
    x = 10
    for name, *_ in robots:
        v, w = metrics.get(name, (0, 0))
        col = (120, 255, 120) if w < 5 else (255, 200, 80) if w < 15 else (255, 90, 90)
        d.text((x, 44), f"{name}: {v:.1f} m/s  wobble {w:.0f}deg", fill=col)
        x += 360
    ff.stdin.write(np.array(img).astype(np.uint8).tobytes()[:W * H * 3]
                   if False else _rgba(img))


def _rgba(img):
    a = np.array(img)
    rgba = np.dstack([a, np.full(a.shape[:2], 255, np.uint8)])
    return rgba.astype(np.uint8).tobytes()


if __name__ == "__main__":
    main()
