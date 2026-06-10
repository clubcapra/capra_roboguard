"""Scripted RoveControl drive demo (M1). Runs a maneuver sequence and saves a
before/after snapshot pair on the GPU.

    PYTHONPATH=. rove_sim_venv/bin/python tools/drive_demo.py --profile standard
"""
import argparse
import time
import numpy as np
import pybullet as p
from PIL import Image

from rove_sim.runtime import build
from rove_sim.control import RoveControl, Tracks, Flippers, Ovis


def snapshot(sim, path, w=960, h=720):
    pos = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])
    view = p.computeViewMatrix(pos + [2.4, -2.4, 1.8], pos + [0, 0, 0.2], [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(55, w / h, 0.1, 50)
    _, _, rgb, _, _ = p.getCameraImage(w, h, view, proj,
                                       renderer=sim.engine.camera_renderer_flag,
                                       lightDirection=[-1, -1, 2])
    Image.fromarray(np.reshape(rgb, (h, w, 4))[:, :, :3].astype(np.uint8)).save(path)
    print(f"saved {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--mode", default="headless", choices=["headless", "gui"])
    ap.add_argument("--out", default="/tmp/rove_drive")
    args = ap.parse_args()

    sim = build(args.profile, mode=args.mode)
    gui = args.mode == "gui"
    if not gui:
        snapshot(sim, f"{args.out}_0.png")

    def yaw():
        return np.degrees(p.getEulerFromQuaternion(
            p.getBasePositionAndOrientation(sim.robot.body_id)[1])[2])
    x0 = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])

    # in GUI, pace to wall-clock so the maneuver is watchable in real time
    def run(secs, intent):
        if not gui:
            sim.run_for(secs, intent)
            return
        sim.set_intent(intent)
        for _ in range(int(secs * sim.control_hz)):
            sim.step_control(1)
            time.sleep(1.0 / sim.control_hz)

    run(2.5, RoveControl(tracks=Tracks(0.2, 0.2)))      # forward
    y0 = yaw()
    run(3.0, RoveControl(tracks=Tracks(-1.0, 1.0)))     # point turn
    run(1.0, RoveControl(flippers=Flippers(fl=1, fr=1)))  # raise flippers
    if sim.caps.has_arm:
        run(1.5, RoveControl(ovis=Ovis(vz=1.0)))        # raise arm
    run(0.3, RoveControl.stop())

    dx = np.linalg.norm(np.array(p.getBasePositionAndOrientation(
        sim.robot.body_id)[0])[:2] - x0[:2])
    print(f"drove {dx:.2f} m, turned {yaw()-y0:.0f} deg, "
          f"mass={sim.robot.total_mass:.0f} kg, renderer={sim.engine.renderer}")
    if gui:
        print("Drag to orbit; Ctrl-C to exit.")
        try:
            while True:
                sim.step_control(1)
                time.sleep(1.0 / sim.control_hz)
        except KeyboardInterrupt:
            pass
    else:
        snapshot(sim, f"{args.out}_1.png")
    sim.engine.disconnect()


if __name__ == "__main__":
    main()
