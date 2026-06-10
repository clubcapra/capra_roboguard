"""scene_editor: interactive 3D editor for sim Scene files.

A dedicated PyBullet GUI app to author a mission scene: load (or start fresh),
drive/pose the robot, place / move / delete objects (SAR targets, obstacles), and
save or export to JSON. Builds on the same Scene + MockWorld.spawn_object path the
sim uses, so what you author here loads identically in sim_server / live.py.

    PYTHONPATH=. ../rove_sim_venv/bin/python tools/scene_editor.py --profile standard --terrain
    PYTHONPATH=. ../rove_sim_venv/bin/python tools/scene_editor.py --scene mission.json

Drive (hold):  W/S = fwd/rev   A/D = turn   SPACE = stop
Author (sliders + buttons on the left panel):
  Obj X/Y/Z, Shape, Class  -> SPAWN places one there
  Select  -> picks an existing object; MOVE re-places it at the sliders; DELETE removes it
  SAVE -> --scene file (or scene.json);  EXPORT -> a fresh numbered file
Camera: PyBullet built-in (CTRL+drag orbit, scroll zoom, CTRL+mid pan).
"""
import argparse
import os
import sys
import time

import numpy as np
import pybullet as p

from rove_sim import runtime
from rove_sim.control import RoveControl, Tracks
from rove_sim.world.scene import Scene, SceneObject, capture_scene, load_scene_sim

SHAPES = ["box", "sphere", "cylinder"]
CLASSES = ["object", "victim", "barrel", "debris", "obstacle"]
CLS_RGBA = {"object": (0.85, 0.2, 0.2, 1), "victim": (0.1, 0.8, 0.2, 1),
            "barrel": (0.9, 0.6, 0.1, 1), "debris": (0.5, 0.5, 0.55, 1),
            "obstacle": (0.2, 0.4, 0.8, 1)}


def _down(keys, *codes):
    return 1.0 if any(keys.get(c, 0) & p.KEY_IS_DOWN for c in codes) else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--terrain", nargs="?",
                    const="../free_dirt_road_through_forest.glb", default=None)
    ap.add_argument("--scene", default=None, help="load/save this scene .json")
    ap.add_argument("--no-texture", action="store_true")
    ap.add_argument("--display", default=None)
    args = ap.parse_args()
    if args.display:
        os.environ["DISPLAY"] = args.display

    if args.scene and os.path.exists(args.scene):
        sim = load_scene_sim(args.scene, args.profile, mode="gui")
        print(f"[editor] loaded {args.scene}")
    else:
        overrides = {"friction": {"origin": (-25., -25.), "extent": (50., 50.), "cell": 0.25}}
        if args.terrain:
            overrides["terrain"] = {"source": args.terrain, "texture": not args.no_texture}
        sim = runtime.build(args.profile, mode="gui", world="mock", world_overrides=overrides)

    if not hasattr(sim.world, "spawn_object"):
        sys.exit("[editor] this world can't hold objects (need a MockWorld)")

    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    base = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])
    p.resetDebugVisualizerCamera(5.0, 50, -35, base.tolist())

    # objects already in the scene (from a load) seed the editable list
    objs = list(getattr(sim.world, "scene_objects", []))
    save_path = args.scene or "scene.json"
    export_n = [0]

    s_x = p.addUserDebugParameter("Obj X", -25, 25, float(base[0]))
    s_y = p.addUserDebugParameter("Obj Y", -25, 25, float(base[1]))
    s_z = p.addUserDebugParameter("Obj Z", 0, 5, 0.3)
    s_shape = p.addUserDebugParameter(f"Shape 0=box 1=sph 2=cyl", 0, 2, 0)
    s_cls = p.addUserDebugParameter(f"Class 0..{len(CLASSES)-1}", 0, len(CLASSES) - 1, 0)
    s_size = p.addUserDebugParameter("Size (m)", 0.1, 2.0, 0.4)
    s_sel = p.addUserDebugParameter("Select idx", 0, 64, 0)
    b_spawn = p.addUserDebugParameter("SPAWN", 1, 0, 0)
    b_move = p.addUserDebugParameter("MOVE selected -> sliders", 1, 0, 0)
    b_del = p.addUserDebugParameter("DELETE selected", 1, 0, 0)
    b_save = p.addUserDebugParameter("SAVE", 1, 0, 0)
    b_export = p.addUserDebugParameter("EXPORT (new file)", 1, 0, 0)
    btn = {b_spawn: 0., b_move: 0., b_del: 0., b_save: 0., b_export: 0.}
    hud = [-1]

    def clicked(b):
        v = p.readUserDebugParameter(b); hit = v != btn[b]; btn[b] = v; return hit

    def make_obj(oid):
        cls = CLASSES[int(round(p.readUserDebugParameter(s_cls)))]
        sz = p.readUserDebugParameter(s_size)
        return SceneObject(
            id=oid,
            pose=(p.readUserDebugParameter(s_x), p.readUserDebugParameter(s_y),
                  p.readUserDebugParameter(s_z)),
            shape=SHAPES[int(round(p.readUserDebugParameter(s_shape)))],
            extents=(sz, sz, sz), rgba=CLS_RGBA[cls], cls=cls,
            mass=0.0)

    def respawn_all():
        for o in objs:
            if o.id in getattr(sim.world, "objects", {}):
                sim.world.remove_object(o.id)
            sim.world.spawn_object(o)
        sim.world.scene_objects = list(objs)

    def hud_text():
        sel = int(round(p.readUserDebugParameter(s_sel)))
        sel = max(0, min(sel, max(0, len(objs) - 1)))
        cur = f"sel[{sel}]={objs[sel].id}({objs[sel].cls})" if objs else "no objects"
        hud[0] = p.addUserDebugText(
            f"{len(objs)} objects  {cur}  save->{os.path.basename(save_path)}",
            [base[0], base[1], 2.5], textColorRGB=[1, 1, 0], textSize=1.3,
            replaceItemUniqueId=hud[0] if hud[0] >= 0 else -1)

    respawn_all(); hud_text()
    n_counter = [len(objs)]

    tr = next((a for a in sim.actuators if a.intent_field == "tracks"), None)
    dt = 1.0 / sim.control_hz
    UP, DN, LF, RT = p.B3G_UP_ARROW, p.B3G_DOWN_ARROW, p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW
    while p.isConnected():
        t0 = time.time()
        if clicked(b_spawn):
            n_counter[0] += 1
            o = make_obj(f"obj_{n_counter[0]}")
            objs.append(o); sim.world.spawn_object(o)
            sim.world.scene_objects = list(objs); hud_text()
            print(f"[editor] spawned {o.id} ({o.cls} {o.shape}) at {tuple(round(v,2) for v in o.pose)}")
        if clicked(b_move) and objs:
            sel = max(0, min(int(round(p.readUserDebugParameter(s_sel))), len(objs) - 1))
            o = make_obj(objs[sel].id)        # keep id, take pose/shape/class from sliders
            objs[sel] = o; respawn_all(); hud_text()
            print(f"[editor] moved {o.id} -> {tuple(round(v,2) for v in o.pose)}")
        if clicked(b_del) and objs:
            sel = max(0, min(int(round(p.readUserDebugParameter(s_sel))), len(objs) - 1))
            o = objs.pop(sel)
            if o.id in getattr(sim.world, "objects", {}):
                sim.world.remove_object(o.id)
            sim.world.scene_objects = list(objs); hud_text()
            print(f"[editor] deleted {o.id}")
        if clicked(b_save):
            capture_scene(sim).save(save_path); print(f"[editor] saved -> {save_path}")
        if clicked(b_export):
            export_n[0] += 1
            path = f"scene_export_{export_n[0]}.json"
            capture_scene(sim).save(path); print(f"[editor] exported -> {path}")

        keys = p.getKeyboardEvents()
        fwd = _down(keys, UP, ord('w')) - _down(keys, DN, ord('s'))
        turn = _down(keys, RT, ord('d')) - _down(keys, LF, ord('a'))
        if keys.get(ord(' '), 0) & p.KEY_IS_DOWN:
            fwd = turn = 0.0
        sim.set_intent(RoveControl(tracks=Tracks(
            float(np.clip(fwd - turn, -1, 1)), float(np.clip(fwd + turn, -1, 1)))))
        if not p.isConnected():
            break
        sim.step_control(1)
        time.sleep(max(0.0, dt - (time.time() - t0)))
    sim.engine.disconnect()


if __name__ == "__main__":
    main()
