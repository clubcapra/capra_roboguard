#!/usr/bin/env python3
"""scene_cli: scriptable create / edit / inspect of sim Scene JSON files.

A Scene is the environment + state half of a sim (terrain ref + friction raster +
robot pose/joints + placed SceneObjects). This tool manipulates the JSON directly
(no 3D), for automation/CI and quick authoring; pair it with `scene_editor.py`
(interactive 3D) or `live.py --scene` to view/drive a scene.

    tools/scene_cli.py new mission.json --terrain ../free_dirt_road_through_forest.glb
    tools/scene_cli.py add-object mission.json --id victim1 --pos 4 0 0 --cls victim --shape cylinder
    tools/scene_cli.py add-object mission.json --id barrel --pos 2 1 0 --extents 0.6 0.6 0.9 --mass 5
    tools/scene_cli.py set-robot mission.json --pos 0 0 0.5
    tools/scene_cli.py remove mission.json --id barrel
    tools/scene_cli.py info mission.json
    tools/scene_cli.py validate mission.json
    tools/scene_cli.py merge mission.json --from other.json   # import its objects
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim.world.scene import Scene, SceneObject


def _load(path: str) -> Scene:
    return Scene.load(path)


def cmd_new(a):
    scene = Scene()
    if a.terrain:
        scene.terrain = {"source": a.terrain, "texture": not a.no_texture}
    scene.meta = {"name": a.name or os.path.splitext(os.path.basename(a.scene))[0]}
    scene.save(a.scene)
    print(f"created {a.scene}" + (f" (terrain {a.terrain})" if a.terrain else ""))


def cmd_add_object(a):
    scene = _load(a.scene)
    if any(o.id == a.id for o in scene.objects):
        sys.exit(f"object id '{a.id}' already exists (use remove first)")
    obj = SceneObject(
        id=a.id, pose=a.pos, orn=a.orn or (0.0, 0.0, 0.0, 1.0), shape=a.shape,
        extents=a.extents, rgba=a.rgba, mass=a.mass, cls=a.cls, mesh=a.mesh)
    scene.objects.append(obj)
    scene.save(a.scene)
    print(f"added {a.shape} '{a.id}' ({a.cls}) at {a.pos}  -> {len(scene.objects)} objects")


def cmd_remove(a):
    scene = _load(a.scene)
    n0 = len(scene.objects)
    scene.objects = [o for o in scene.objects if o.id != a.id]
    if len(scene.objects) == n0:
        sys.exit(f"no object with id '{a.id}'")
    scene.save(a.scene)
    print(f"removed '{a.id}'  -> {len(scene.objects)} objects")


def cmd_set_robot(a):
    scene = _load(a.scene)
    orn = a.orn if a.orn else (scene.robot_pose[1] if scene.robot_pose else (0, 0, 0, 1))
    scene.robot_pose = (tuple(a.pos), tuple(orn))
    scene.save(a.scene)
    print(f"robot pose -> pos {a.pos} orn {tuple(orn)}")


def cmd_info(a):
    scene = _load(a.scene)
    t = scene.terrain
    print(f"scene: {a.scene}")
    print(f"  meta     : {scene.meta}")
    print(f"  terrain  : {t['source'] if t else '(none)'}"
          + (f"  textured={t.get('texture', True)}" if t else ""))
    print(f"  friction : {'painted raster' if scene.friction else '(none)'}")
    rp = scene.robot_pose
    print(f"  robot    : pose={tuple(round(v,3) for v in rp[0]) if rp else None}"
          f"  joints={len(scene.robot_joints)}")
    print(f"  objects  : {len(scene.objects)}")
    for o in scene.objects:
        print(f"    - {o.id:14s} {o.cls:10s} {o.shape:8s} pos="
              f"{tuple(round(v,2) for v in o.pose)} mass={o.mass}")


def cmd_validate(a):
    scene = _load(a.scene)
    errs = []
    ids = [o.id for o in scene.objects]
    dups = {i for i in ids if ids.count(i) > 1}
    if dups:
        errs.append(f"duplicate object ids: {sorted(dups)}")
    for o in scene.objects:
        if o.shape not in ("box", "sphere", "cylinder", "mesh"):
            errs.append(f"{o.id}: bad shape '{o.shape}'")
        if o.shape == "mesh" and not o.mesh:
            errs.append(f"{o.id}: shape=mesh but no mesh path")
        if len(o.pose) != 3 or len(o.orn) != 4:
            errs.append(f"{o.id}: pose must be xyz(3), orn xyzw(4)")
    if scene.terrain and not os.path.exists(
            os.path.join(os.path.dirname(os.path.abspath(a.scene)), scene.terrain["source"])) \
            and not os.path.exists(scene.terrain["source"]):
        errs.append(f"terrain source not found: {scene.terrain['source']} (may resolve at build time)")
    if errs:
        print("INVALID:")
        for e in errs:
            print("  -", e)
        sys.exit(1)
    print(f"valid: {len(scene.objects)} objects, terrain "
          f"{'yes' if scene.terrain else 'no'}")


def cmd_merge(a):
    base = _load(a.scene)
    other = _load(a.from_scene)
    have = {o.id for o in base.objects}
    added = 0
    for o in other.objects:
        if o.id in have:
            if not a.overwrite:
                print(f"  skip '{o.id}' (exists; --overwrite to replace)")
                continue
            base.objects = [b for b in base.objects if b.id != o.id]
        base.objects.append(o)
        added += 1
    base.save(a.scene)
    print(f"merged {added} objects from {a.from_scene} -> {len(base.objects)} total")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("new", help="create a new scene")
    p.add_argument("scene")
    p.add_argument("--terrain", default=None, help="terrain GLB path (relative to rove_sim/)")
    p.add_argument("--no-texture", action="store_true")
    p.add_argument("--name", default=None)
    p.set_defaults(func=cmd_new)

    p = sub.add_parser("add-object", help="add a SceneObject")
    p.add_argument("scene")
    p.add_argument("--id", required=True)
    p.add_argument("--pos", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    p.add_argument("--orn", type=float, nargs=4, default=None, metavar=("QX", "QY", "QZ", "QW"))
    p.add_argument("--shape", default="box", choices=["box", "sphere", "cylinder", "mesh"])
    p.add_argument("--extents", type=float, nargs=3, default=[0.3, 0.3, 0.3])
    p.add_argument("--rgba", type=float, nargs=4, default=[0.85, 0.2, 0.2, 1.0])
    p.add_argument("--cls", default="object")
    p.add_argument("--mass", type=float, default=0.0)
    p.add_argument("--mesh", default=None)
    p.set_defaults(func=cmd_add_object, orn=None)

    p = sub.add_parser("remove", help="remove a SceneObject by id")
    p.add_argument("scene")
    p.add_argument("--id", required=True)
    p.set_defaults(func=cmd_remove)

    p = sub.add_parser("set-robot", help="set the robot base pose")
    p.add_argument("scene")
    p.add_argument("--pos", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    p.add_argument("--orn", type=float, nargs=4, default=None, metavar=("QX", "QY", "QZ", "QW"))
    p.set_defaults(func=cmd_set_robot)

    p = sub.add_parser("info", help="print a scene summary")
    p.add_argument("scene")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("validate", help="check a scene for errors")
    p.add_argument("scene")
    p.set_defaults(func=cmd_validate)

    p = sub.add_parser("merge", help="merge objects from another scene")
    p.add_argument("scene")
    p.add_argument("--from", dest="from_scene", required=True)
    p.add_argument("--overwrite", action="store_true", help="replace objects with the same id")
    p.set_defaults(func=cmd_merge)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
