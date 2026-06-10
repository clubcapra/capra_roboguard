"""Render a chase-camera MP4 of a scripted maneuver tour (headless GPU).

Each phase is labelled on-screen; the camera follows the robot.

    PYTHONPATH=. rove_sim_venv/bin/python tools/render_clip.py --profile standard --out media/rove_standard.mp4
"""
import argparse
import subprocess
import numpy as np
import pybullet as p
from PIL import Image, ImageDraw, ImageFont

from rove_sim.runtime import build
from rove_sim.control import RoveControl, Tracks, Flippers, Ovis, Gripper

# FPS == frames-per-second of sim time (one frame per control tick @ 50 Hz) so
# the clip plays at realtime.
W, H, FPS = 854, 480, 50

_FONT = ImageFont.load_default()
for _p in ("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf",
           "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf"):
    try:
        _FONT = ImageFont.truetype(_p, 22)
        break
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--out", default="/tmp/rove.mp4")
    args = ap.parse_args()

    sim = build(args.profile, mode="headless")
    # The GLB->OBJ meshes carry no image textures, so under one hard light they
    # render near-black. Assign per-part material colours + use ambient light so
    # the robot reads on camera.
    _COL = [("Core", (.25, .27, .3)), ("cage", (.2, .4, .75)), ("DrumW", (.1, .1, .12)),
            ("Drum", (.15, .15, .17)), ("Flipper", (.85, .7, .1)), ("Base", (.8, .45, .1)),
            ("Section", (.8, .45, .1)), ("Joint", (.8, .45, .1)), ("robotiq", (.9, .5, .1)),
            ("knuckle", (.9, .5, .1)), ("finger", (.9, .5, .1)), ("mid360", (.1, .6, .2)),
            ("livox", (.1, .6, .2)), ("camera", (.4, .4, .42)), ("vn300", (.5, .12, .12))]
    for n, idx in sim.robot.link_index.items():
        for key, c in _COL:
            if key in n:
                p.changeVisualShape(sim.robot.body_id, idx, rgbaColor=[*c, 1]); break
    proj = p.computeProjectionMatrixFOV(55, W / H, 0.1, 60)
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-f", "rawvideo", "-pixel_format", "rgba",
         "-video_size", f"{W}x{H}", "-framerate", str(FPS), "-i", "-",
         "-pix_fmt", "yuv420p", "-loglevel", "error", args.out],
        stdin=subprocess.PIPE)

    # chase cam: follow the robot's position (tight low-pass), fixed 3/4 offset.
    smooth = [None]

    def frame(label):
        base = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])
        if smooth[0] is None:
            smooth[0] = base.copy()
        smooth[0] = 0.85 * smooth[0] + 0.15 * base       # tighter follow
        tgt = smooth[0]
        view = p.computeViewMatrix(tgt + [2.8, -2.8, 1.8], tgt + [0, 0, 0.25],
                                   [0, 0, 1])
        _, _, rgb, _, _ = p.getCameraImage(
            W, H, view, proj, renderer=sim.engine.camera_renderer_flag,
            lightDirection=[-0.6, -0.8, 1.4], lightAmbientCoeff=0.5,
            lightDiffuseCoeff=0.65, shadow=1)
        img = Image.fromarray(np.reshape(rgb, (H, W, 4))[:, :, :3].astype(np.uint8))
        d = ImageDraw.Draw(img)
        d.rectangle([0, 0, W, 34], fill=(20, 20, 28))
        d.text((12, 6), label, fill=(245, 220, 120), font=_FONT)
        rgba = np.dstack([np.asarray(img), np.full((H, W), 255, np.uint8)])
        ff.stdin.write(rgba.astype(np.uint8).tobytes())

    def run_phase(label, intent, secs):
        sim.set_intent(intent)
        for _ in range(int(secs * sim.control_hz)):
            sim.step_control(1)
            frame(label)

    def run_goto(label, pose, speed=1.2):
        sim.arm_goto_pose(pose, speed)
        guard = 0
        while sim.arm_planning() and guard < int(8 * sim.control_hz):
            sim.step_control(1); frame(label); guard += 1

    sim.store_arm_pose("home")        # the folded home (for the planned return)
    F_DOWN = Flippers(fl=-1, fr=-1, rl=-1, rr=-1)
    F_UP = Flippers(fl=1, fr=1, rl=1, rr=1)

    for label, intent, secs in [
        ("Settle", RoveControl(), 0.8),
        ("Forward  ~15 km/h", RoveControl(tracks=Tracks(1.0, 1.0)), 3.0),
        ("Brake", RoveControl.stop(), 1.0),
        ("Point turn  (slow, narrow gauge)", RoveControl(tracks=Tracks(-1.0, 1.0)), 8.0),
        ("Brake", RoveControl.stop(), 0.6),
        ("Arc turn  (grips the curve)", RoveControl(tracks=Tracks(0.45, 0.12)), 5.0),
        ("Brake", RoveControl.stop(), 0.6),
    ]:
        run_phase(label, intent, secs)

    if sim.caps.has_arm:
        # pinch-TCP, robot-frame axes: vx fwd, vz down -> claw-machine reach
        run_phase("Arm deploy  (pinch forward)", RoveControl(ovis=Ovis(vx=1.0, vz=-0.4)), 2.4)
        run_phase("Gripper  close", RoveControl(gripper=Gripper(position=255)), 1.4)
        run_phase("Gripper  open", RoveControl(gripper=Gripper(position=0)), 1.4)
        run_goto("Path-plan -> stored home pose", "home", 1.2)     # planned joint move

    for label, intent, secs in [
        # worm-gear flippers (~15 deg/s): ~6 s to ~90deg -> chassis lifts ~80 mm.
        ("Flipper tippy-toe  (lift)", RoveControl(flippers=F_DOWN), 6.0),
        ("Flipper hold", RoveControl(), 1.2),
        ("Flipper retract", RoveControl(flippers=F_UP), 6.0),
        ("Done", RoveControl.stop(), 0.8),
    ]:
        run_phase(label, intent, secs)

    ff.stdin.close()
    ff.wait()
    sim.engine.disconnect()
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
