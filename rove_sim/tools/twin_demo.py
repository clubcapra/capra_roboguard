"""Mock vs synced-twin validation clip.

Phase A: drive the MOCK (physics) sim through an arm maneuver; each tick, publish
its state as rove_sensor_api telemetry and decode it back to a RobotState frame
(the full wire loop), recording the rendered image too.
Phase B: build a REAL (kinematic) sim and replay those decoded frames through the
SyncDriver -- no physics, no actuators, just telemetry -> URDF -- and render it.

Stitch mock|twin side by side. The twin must track the physics arm; the printed
per-frame joint error is the encode/decode quantisation, not drift.

    PYTHONPATH=. ../rove_sim_venv/bin/python tools/twin_demo.py --out media/twin.mp4
"""
import argparse
import subprocess

import numpy as np
import pybullet as p
from PIL import Image, ImageDraw, ImageFont

from rove_sim import runtime
from rove_sim.api.sim_sensor_api import SimSensorApi
from rove_sim.state.base import RobotState
from rove_sim.state.manual import ReplayStateSource
from rove_sim.state.rove_sensor_api import RoveSensorApiStateSource
from rove_sim.transport import InProcessTransport
from rove_sim.control import RoveControl, Ovis, Gripper

W, H, FPS = 480, 480, 50
_FONT = ImageFont.load_default()
for _p in ("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf",
           "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf"):
    try:
        _FONT = ImageFont.truetype(_p, 20); break
    except Exception:
        pass

_COL = [("Core", (.25, .27, .3)), ("cage", (.2, .4, .75)), ("DrumW", (.1, .1, .12)),
        ("Drum", (.15, .15, .17)), ("Flipper", (.85, .7, .1)), ("Base", (.8, .45, .1)),
        ("Section", (.8, .45, .1)), ("Joint", (.8, .45, .1)), ("robotiq", (.9, .5, .1)),
        ("knuckle", (.9, .5, .1)), ("finger", (.9, .5, .1))]


def _colorize(sim):
    for n, idx in sim.robot.link_index.items():
        for key, c in _COL:
            if key in n:
                p.changeVisualShape(sim.robot.body_id, idx, rgbaColor=[*c, 1]); break


def _render(sim, proj):
    base = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])
    view = p.computeViewMatrix(base + [1.6, -2.4, 1.3], base + [-0.4, 0, 0.4], [0, 0, 1])
    _, _, rgb, _, _ = p.getCameraImage(
        W, H, view, proj, renderer=sim.engine.camera_renderer_flag,
        lightDirection=[-0.6, -0.8, 1.4], lightAmbientCoeff=0.5,
        lightDiffuseCoeff=0.65, shadow=1)
    return np.reshape(rgb, (H, W, 4))[:, :, :3].astype(np.uint8)


def _label(arr, text, color):
    img = Image.fromarray(arr); d = ImageDraw.Draw(img)
    d.rectangle([0, 0, W, 30], fill=(20, 20, 28))
    d.text((10, 5), text, fill=color, font=_FONT)
    return np.asarray(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/twin.mp4")
    args = ap.parse_args()
    proj = p.computeProjectionMatrixFOV(55, W / H, 0.1, 60)

    # ---- Phase A: mock physics, record telemetry frames + images ----------
    sim = runtime.build("standard", mode="headless", world="mock")
    _colorize(sim)
    bus = "twin_demo"
    api = SimSensorApi(sim.robot, sim.robot.profile, transport=InProcessTransport(bus=bus))
    sub = RoveSensorApiStateSource(profile=sim.robot.profile, robot=sim.robot,
                                   transport=InProcessTransport(bus=bus))
    arm_joints = [j for j in sim.robot.movable_joint
                  if j in ("Base", "ASection", "JointA", "BSection", "JointB", "JointGripper")]
    idx_to_name = {v: k for k, v in sim.robot.joint_index.items()}
    arm_names = [idx_to_name[sim.robot.movable_joint[j]] for j in arm_joints]

    frames, mock_imgs, errs = [], [], []
    script = [(RoveControl(ovis=Ovis(vx=1.0, vz=-0.4)), 2.4),
              (RoveControl(gripper=Gripper(position=255)), 1.2),
              (RoveControl(gripper=Gripper(position=0)), 1.2),
              (RoveControl(ovis=Ovis(vx=-0.6, vz=0.5)), 1.6)]
    for intent, secs in script:
        sim.set_intent(intent)
        for _ in range(int(secs * sim.control_hz)):
            sim.step_control(1)
            api.publish()                       # mock -> telemetry (wire)
            state = sub.read()                  # telemetry -> RobotState (wire)
            frames.append(state)
            mock_imgs.append(_render(sim, proj))
    # ground-truth mock arm joints for the error metric
    mock_q = [np.array([p.getJointState(sim.robot.body_id,
              sim.robot.movable_joint[j])[0] for j in arm_joints])]
    sim.disconnect()

    # ---- Phase B: real/kinematic twin, replay the decoded frames ----------
    twin = runtime.build("standard", mode="headless", world="real",
                         state_source=ReplayStateSource(frames=frames, loop=False))
    _colorize(twin)
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-f", "rawvideo", "-pixel_format", "rgba",
         "-video_size", f"{2*W}x{H}", "-framerate", str(FPS), "-i", "-",
         "-pix_fmt", "yuv420p", "-loglevel", "error", args.out], stdin=subprocess.PIPE)
    for i, frame in enumerate(frames):
        twin.step_control(1)
        timg = _render(twin, proj)
        # joint tracking error: twin URDF vs the telemetry frame it was given
        tq = np.array([p.getJointState(twin.robot.body_id,
                       twin.robot.movable_joint[j])[0] for j in arm_joints])
        fq = np.array([frame.joints.get(n, 0.0) for n in arm_names])
        errs.append(float(np.abs(tq - fq).max()))
        left = _label(mock_imgs[i], "MOCK (physics)", (245, 220, 120))
        right = _label(timg, "TWIN (telemetry sync)", (140, 220, 245))
        comp = np.hstack([left, right])
        rgba = np.dstack([comp, np.full((H, 2 * W), 255, np.uint8)])
        ff.stdin.write(rgba.astype(np.uint8).tobytes())
    ff.stdin.close(); ff.wait()
    twin.disconnect()
    print(f"wrote {args.out}  frames={len(frames)}  "
          f"max joint-sync err={max(errs):.5f} rad  mean={np.mean(errs):.6f} rad")


if __name__ == "__main__":
    main()
