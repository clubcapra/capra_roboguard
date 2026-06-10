"""rove_sim entrypoint.

    python -m rove_sim.main --profile standard --mode headless
    python -m rove_sim.main --profile caged    --mode gui

Assembles engine -> robot from a profile and derives the capability set.
For M0 it loads the robot, lets it settle, and reports stability. Later
milestones add the scheduler, sensors, actuators and API seam on top of this
same assembly.
"""
from __future__ import annotations

import argparse
import os
import time

import numpy as np
import pybullet as p

from . import runtime
from .core.engine import Engine
from .robot import loader


def settle(engine: Engine, robot: loader.Robot, seconds: float = 2.0,
           dt: float = 1 / 240) -> dict:
    """Step physics and measure base drift over the last 0.5 s (M0 gate)."""
    steps = int(seconds / dt)
    tail = []
    for i in range(steps):
        engine.step()
        if i >= steps - int(0.5 / dt):
            pos, _ = p.getBasePositionAndOrientation(robot.body_id)
            tail.append(pos)
    tail = np.asarray(tail)
    lin_v, ang_v = p.getBaseVelocity(robot.body_id)
    pos, orn = p.getBasePositionAndOrientation(robot.body_id)
    return {
        "final_pos": [round(x, 4) for x in pos],
        "settle_drift_m": round(float(np.ptp(tail, axis=0).max()), 5),
        "lin_speed_mps": round(float(np.linalg.norm(lin_v)), 5),
        "ang_speed_rps": round(float(np.linalg.norm(ang_v)), 5),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--mode", default="headless", choices=["headless", "gui"],
                    help="engine connection (window vs offscreen)")
    ap.add_argument("--world", default="mock", choices=["mock", "real"],
                    help="mock = physics sim; real = robot synced from telemetry")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="seconds to settle for the M0 stability check")
    ap.add_argument("--hold", action="store_true",
                    help="keep the GUI window open after settling")
    args = ap.parse_args()

    sim = runtime.build(args.profile, mode=args.mode, world=args.world)
    engine, robot, caps = sim.engine, sim.robot, sim.caps
    print(f"[engine] mode={engine.cfg.mode} renderer={engine.renderer} "
          f"world={args.world}")
    print(f"[robot]  profile={robot.profile.name} body_id={robot.body_id} "
          f"links={len(robot.link_index)} "
          f"movable_joints={len(set(robot.movable_joint.values()))}")
    print(f"[caps]   {caps}")
    print(f"         arm={caps.has_arm} gnss={caps.has_gnss} "
          f"lidar={caps.has_lidar} gripper={caps.has_gripper}")

    if args.world == "mock":
        stats = settle(engine, robot, seconds=args.settle)
        ok = stats["settle_drift_m"] < 0.02 and stats["lin_speed_mps"] < 0.05
        print(f"[M0]     {'PASS' if ok else 'FAIL'} {stats}")
    else:
        print("[real]   kinematic world model (no physics settle)")

    if args.hold and args.mode == "gui":
        print("[hold]   Ctrl-C to exit")
        try:
            while True:
                engine.step()
                time.sleep(1 / 240)
        except KeyboardInterrupt:
            pass
    engine.disconnect()


if __name__ == "__main__":
    main()
