"""CLI wrapper: GLB terrain -> cached PyBullet OBJ. Logic in rove_sim.world.terrain.

    PYTHONPATH=. ../rove_sim_venv/bin/python tools/convert_terrain.py \
        --src ../free_dirt_road_through_forest.glb --faces 60000
"""
import argparse

import numpy as np
import trimesh

from rove_sim.world import terrain


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out-dir", default="assets/terrain")
    ap.add_argument("--faces", type=int, default=60000)
    args = ap.parse_args()
    out = terrain.convert(args.src, args.out_dir, args.faces)
    m = trimesh.load(out, force="mesh")
    print(f"wrote {out}  faces={len(m.faces)}  extents={np.round(m.extents, 1)}")


if __name__ == "__main__":
    main()
