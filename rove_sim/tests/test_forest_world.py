"""Exercise the deployed forest assets and saved test scene, without source GLB."""

from pathlib import Path

import pybullet as p
import pytest

from rove_sim.world.scene import build_configured_sim
from rove_sim.world.render_sync import robot_state, apply_world_state

ROOT = Path(__file__).resolve().parents[1]


def test_forest_configuration_loads_collision_scene_and_dynamic_sync():
    sim = build_configured_sim(
        "standard",
        world_config=ROOT / "config/world.yaml",
        scene_path=ROOT / "scenes/forest_demo.json",
        texture=False,
        egl=False,
    )
    try:
        assert sim.world.terrain_id is not None
        assert sim.world.obstacle_ids and sim.world.foliage_ids
        assert "t1" in sim.world.objects
        assert sim.world.friction.lookup(0, 0) == pytest.approx(0.6)
        initial = p.getBasePositionAndOrientation(sim.robot.body_id)[0]
        sim.step_control(100)
        settled = p.getBasePositionAndOrientation(sim.robot.body_id)[0]
        assert abs(settled[2] - initial[2]) < 0.1
        snapshot = robot_state(sim.robot, sim.world)
        body = sim.world.objects["t1"]
        p.resetBasePositionAndOrientation(body, [1, 1, 1], [0, 0, 0, 1])
        apply_world_state(sim.world, snapshot)
        assert p.getBasePositionAndOrientation(body)[0] == pytest.approx(
            snapshot["objects"]["t1"]["pos"]
        )
    finally:
        sim.disconnect()


def test_missing_configured_terrain_cannot_silently_become_a_flat_world(tmp_path):
    from rove_sim.world.terrain import build_assets

    with pytest.raises(FileNotFoundError, match="no compatible exported assets"):
        build_assets(str(tmp_path / "missing.glb"), str(tmp_path / "exports"))


@pytest.mark.parametrize("purpose", ["physics", "render", "raycast"])
def test_worker_roles_keep_only_the_geometry_they_use(purpose):
    sim = build_configured_sim(
        "standard",
        world_config=ROOT / "config/world.yaml",
        scene_path=ROOT / "scenes/forest_demo.json",
        purpose=purpose,
        texture=False,
        egl=False,
    )
    try:
        assert sim.world.terrain_manifest
        assert sim.world.terrain_vis_id is None
        assert not sim.world.foliage_ids
        assert "t1" in sim.world.objects
        if purpose == "physics":
            assert sim.world.terrain_id is not None and sim.world.obstacle_ids
            initial = p.getBasePositionAndOrientation(sim.robot.body_id)[0]
            sim.step_control(100)
            settled = p.getBasePositionAndOrientation(sim.robot.body_id)[0]
            assert abs(settled[2] - initial[2]) < 0.1
        else:
            assert sim.world.terrain_id is None and not sim.world.obstacle_ids
    finally:
        sim.disconnect()


def test_raycast_flat_world_keeps_its_ground_plane():
    sim = build_configured_sim("standard", purpose="raycast", egl=False)
    try:
        assert sim.world.ground_id is not None
    finally:
        sim.disconnect()


def test_complete_forest_has_ground_beneath_transparent_road_layers():
    import json
    import numpy as np
    from rove_sim.world.mesh_assets import load_terrain, AlphaRaycaster

    manifest = ROOT / "assets/terrain/free_dirt_road_through_forest/manifest.json"
    metadata = json.loads(manifest.read_text())
    assert metadata["all_visual_layers"]
    assert len(metadata["visual"]) == 26
    assets = load_terrain(manifest)
    x, y = np.meshgrid(np.linspace(-4, 4, 81), np.linspace(-4, 4, 81))
    origins = np.c_[x.ravel(), y.ravel(), np.full(x.size, 2.1)]
    _, distance, _ = AlphaRaycaster(assets).cast(
        origins, np.tile([0, 0, -1.0], (len(origins), 1)), 100
    )
    # Previously hundreds of these rays escaped the road through its opacity gaps.
    assert np.all(np.isfinite(distance))
