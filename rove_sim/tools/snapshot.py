"""Render a robot snapshot on the GPU (EGL) and save a PNG. M2 preview."""
import argparse, os, numpy as np, pybullet as p
from PIL import Image
from rove_sim.core.engine import Engine, EngineConfig
from rove_sim.robot import loader
from rove_sim.robot.profile import load_profile
from rove_sim.world.mock import MockWorld

ap = argparse.ArgumentParser()
ap.add_argument("--profile", default="standard")
ap.add_argument("--out", default="/tmp/rove_snapshot.png")
ap.add_argument("--w", type=int, default=960); ap.add_argument("--h", type=int, default=720)
a = ap.parse_args()

eng = Engine(EngineConfig(mode="headless")).connect()
_prof = load_profile(f"profiles/{a.profile}.yaml")
MockWorld(eng, {}, profile=_prof).build()        # gravity + ground (moved off the engine)
loader.load(eng, _prof)
for _ in range(240): eng.step()
view = p.computeViewMatrix([2.2, -2.2, 1.6], [0, 0, 0.35], [0, 0, 1])
proj = p.computeProjectionMatrixFOV(55, a.w/a.h, 0.1, 50)
w,h,rgb,_,_ = p.getCameraImage(a.w, a.h, view, proj, renderer=eng.camera_renderer_flag,
                               lightDirection=[-1,-1,2])
Image.fromarray(np.reshape(rgb,(h,w,4))[:,:,:3].astype(np.uint8)).save(a.out)
print(f"saved {a.out}  renderer={eng.renderer}")
eng.disconnect()
