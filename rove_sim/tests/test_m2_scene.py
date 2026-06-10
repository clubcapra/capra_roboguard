"""Scene save / load / sync: a captured world round-trips through JSON into a
fresh sim with identical robot pose, joints, painted friction and objects.

    ../rove_sim_venv/bin/python -m pytest tests/test_m2_scene.py -q
"""
import os
import sys
import tempfile

import numpy as np
import pybullet as p

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim import runtime
from rove_sim.world.scene import (Scene, SceneObject, capture_scene,
                                   apply_scene, load_scene_sim)

FRIC = {"friction": {"origin": (-10.0, -10.0), "extent": (20.0, 20.0),
                     "cell": 0.5, "default": 0.6}}


def test_scene_object_roundtrip_dict():
    o = SceneObject(id="victim_1", pose=(1, 2, 0.3), shape="cylinder",
                    extents=(0.4, 0.4, 1.7), cls="victim", rgba=(0.9, 0.1, 0.1, 1))
    assert SceneObject.from_dict(o.to_dict()).to_dict() == o.to_dict()


def test_capture_save_load_apply_roundtrip():
    sim = runtime.build("standard", mode="headless", world="mock",
                        world_overrides=FRIC)
    try:
        # mutate state: move base, set a joint, paint friction, spawn an object
        bid = sim.robot.body_id
        p.resetBasePositionAndOrientation(bid, [1.5, -0.5, 0.6], [0, 0, 0, 1])
        jname = next(iter(sim.robot.joint_index))
        jidx = sim.robot.joint_index[jname]
        p.resetJointState(bid, jidx, 0.3)
        sim.world.friction.paint_material(1.0, 1.0, 0.6, "ice")
        sim.world.spawn_object(SceneObject(id="barrel", pose=(2.0, 0.0, 0.25),
                                           shape="cylinder", extents=(0.5, 0.5, 0.9),
                                           cls="obstacle"))
        scene = capture_scene(sim, meta={"profile": "standard"})
        path = os.path.join(tempfile.mkdtemp(), "scene.json")
        scene.save(path)
        # capture the values we expect to survive
        ice_mu = sim.world.friction.lookup(1.0, 1.0)
    finally:
        sim.disconnect()

    assert ice_mu < 0.2                                      # ice really painted

    sim2 = load_scene_sim(path, profile="standard", mode="headless")
    try:
        pos = np.array(p.getBasePositionAndOrientation(sim2.robot.body_id)[0])
        assert np.allclose(pos, [1.5, -0.5, 0.6], atol=1e-5)         # base restored
        assert abs(p.getJointState(sim2.robot.body_id,
                                   sim2.robot.joint_index[jname])[0] - 0.3) < 1e-5
        assert abs(sim2.world.friction.lookup(1.0, 1.0) - ice_mu) < 1e-6  # friction
        assert "barrel" in sim2.world.objects                        # object spawned
        bpos = np.array(p.getBasePositionAndOrientation(
            sim2.world.objects["barrel"])[0])
        assert np.allclose(bpos, [2.0, 0.0, 0.25], atol=1e-5)
    finally:
        sim2.disconnect()


def test_sync_upserts_by_id():
    """apply_scene is idempotent: the same id moves (not duplicates), a dropped id
    is removed -- the mechanic a cross-process scene SYNC relies on."""
    sim = runtime.build("standard", mode="headless", world="mock",
                        world_overrides=FRIC)
    try:
        s1 = Scene(objects=[SceneObject(id="a", pose=(1, 0, 0.2)),
                            SceneObject(id="b", pose=(0, 1, 0.2))])
        apply_scene(sim, s1)
        assert set(sim.world.objects) == {"a", "b"}
        # sync a new snapshot: 'a' moved, 'b' gone, 'c' added
        s2 = Scene.from_json(Scene(objects=[
            SceneObject(id="a", pose=(3, 0, 0.2)),
            SceneObject(id="c", pose=(0, 3, 0.2))]).to_json())
        apply_scene(sim, s2)
        assert set(sim.world.objects) == {"a", "c"}
        apos = np.array(p.getBasePositionAndOrientation(sim.world.objects["a"])[0])
        assert np.allclose(apos, [3, 0, 0.2], atol=1e-5)             # moved, not dup
    finally:
        sim.disconnect()
