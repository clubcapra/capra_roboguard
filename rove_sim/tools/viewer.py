"""Interactive GUI model inspector.

Opens the robot in PyBullet's GUI with a slider per movable joint (articulate the
arm, flippers, drums by hand) and toggles for wireframe / collision-shape view.
Drag to orbit, scroll to zoom, Ctrl-drag to pan. Ctrl-C in the terminal to exit.

    PYTHONPATH=. rove_sim_venv/bin/python tools/viewer.py --profile standard

Requires a display (your desktop). On Wayland, XWayland must be running and
DISPLAY set (the engine defaults it to :0).
"""
import argparse
import time

import pybullet as p

from rove_sim.core.engine import Engine, EngineConfig
from rove_sim.robot import loader
from rove_sim.robot.profile import load_profile
from rove_sim.world.mock import MockWorld
from rove_sim import capabilities


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--physics", action="store_true",
                    help="step physics (let it settle/move); default frozen")
    args = ap.parse_args()

    prof = load_profile(loader_profile_path(args.profile))
    eng = Engine(EngineConfig(mode="gui", ground_plane=True)).connect()
    MockWorld(eng, {}, profile=prof).build()     # gravity + ground (moved off the engine)
    robot = loader.load(eng, prof)
    caps = capabilities.derive(prof)
    print(f"[inspect] {prof.name}: {len(robot.link_index)} links, "
          f"{len(set(robot.movable_joint.values()))} movable joints, "
          f"{robot.total_mass:.0f} kg | {caps}")

    p.resetDebugVisualizerCamera(cameraDistance=1.6, cameraYaw=50,
                                 cameraPitch=-25, cameraTargetPosition=[0, 0.2, 0.2])

    # one slider per movable joint (semantic name -> joint), at its current angle
    sliders = {}
    seen = set()
    for link, j in sorted(robot.movable_joint.items()):
        if j in seen:
            continue
        seen.add(j)
        info = p.getJointInfo(robot.body_id, j)
        lo, hi = info[8], info[9]
        if lo >= hi:
            lo, hi = -3.1416, 3.1416
        q0 = p.getJointState(robot.body_id, j)[0]
        sliders[j] = p.addUserDebugParameter(link, lo, hi, q0)

    wire = p.addUserDebugParameter("wireframe", 0, 1, 0)
    last_wire = 0

    print("[inspect] drag=orbit, scroll=zoom, ctrl-drag=pan; sliders move joints. "
          "Ctrl-C to exit.")
    try:
        while True:
            for j, sid in sliders.items():
                p.setJointMotorControl2(robot.body_id, j, p.POSITION_CONTROL,
                                        targetPosition=p.readUserDebugParameter(sid),
                                        force=2000)
            w = int(p.readUserDebugParameter(wire) > 0.5)
            if w != last_wire:
                p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, w)
                last_wire = w
            if args.physics:
                p.stepSimulation()
            else:
                # hold joints without integrating the falling/contact dynamics
                for _ in range(2):
                    p.stepSimulation()
            time.sleep(1 / 120)
    except KeyboardInterrupt:
        pass
    eng.disconnect()


def loader_profile_path(name):
    import os
    here = os.path.join(os.path.dirname(__file__), "..", "profiles", f"{name}.yaml")
    return here if os.path.exists(here) else name


if __name__ == "__main__":
    main()
