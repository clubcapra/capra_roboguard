"""Render and LiDAR must agree on texture coverage, including surfaces behind gaps."""

import numpy as np
import pytest
from rove_sim.world.mesh_assets import MeshAsset, AlphaRaycaster, texture_alpha, covered


def card(z, rgba, cutoff=0.5):
    return MeshAsset(
        np.array([[-1, -1, z], [1, -1, z], [1, 1, z], [-1, 1, z]], np.float32),
        np.array([[0, 1, 2], [0, 2, 3]], np.int32),
        np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32),
        rgba,
        cutoff,
        "test",
    )


def scene():
    rgba = np.zeros((2, 2, 4), np.uint8)
    rgba[:] = [255, 0, 0, 0]
    rgba[:, 0, 3] = 255
    wall = np.full((1, 1, 4), [0, 255, 0, 255], np.uint8)
    return [card(1, rgba), card(0, wall, 0)]


def test_lidar_hits_opaque_texels_and_sees_wall_through_gap():
    ray = AlphaRaycaster(scene())
    origins = np.array([[-0.5, 0, 2], [0.5, 0, 2]], np.float32)
    points, distance, _ = ray.cast(origins, np.array([[0, 0, -1]] * 2, np.float32))
    np.testing.assert_allclose(points[:, 2], [1, 0], atol=1e-5)
    np.testing.assert_allclose(distance, [1, 2], atol=1e-5)
    _, short, _ = ray.cast(
        origins, np.array([[0, 0, -1]] * 2, np.float32), max_range=1.5
    )
    assert short[0] == pytest.approx(1) and np.isinf(short[1])


def test_alpha_wrap_and_compiled_sampler_agree():
    rng = np.random.default_rng(123)
    rgba = rng.integers(0, 256, (8, 16, 4), dtype=np.uint8)
    uv = rng.uniform(-3, 3, (1000, 2))
    reference = texture_alpha(rgba, uv)
    actual = covered(
        uv,
        np.zeros(1000, np.int32),
        rgba[..., 3].ravel(),
        np.array([0]),
        np.array([16]),
        np.array([8]),
        np.array([0.5]),
    )
    np.testing.assert_array_equal(actual, reference >= 0.5)
    np.testing.assert_allclose(
        texture_alpha(rgba, uv), texture_alpha(rgba, uv + [2, -1]), atol=1e-12
    )


def test_gpu_discards_transparent_texels_before_depth_write():
    import pybullet as p
    from rove_sim.render.gpu import GpuScene

    try:
        gpu = GpuScene(scene())
    except Exception as exc:
        import os

        if os.environ.get("SIM_REQUIRE_GPU") == "1":
            raise
        pytest.skip(f"No EGL GPU in this environment: {exc}")
    try:
        view = p.computeViewMatrix([0, 0, 2], [0, 0, 0], [0, 1, 0])
        projection = p.computeProjectionMatrixFOV(90, 1, 0.05, 10)
        rgb = gpu.render(view, projection, 128, 128)
        red = rgb[64, 32]
        green = rgb[64, 80]
        assert red[0] > 100 and red[1] < 5
        assert green[1] > 100 and green[0] < 5
    finally:
        gpu.close()


def test_compiled_ray_box_parallel_axes_and_inside_origin():
    from rove_sim.world.moving_rays import ray_box_candidates

    origins = np.array([[0, 0, 0], [2, 0, 0], [0, 0, 2], [0, 0, 2]], float)
    directions = np.array([[0, 0, 1], [0, 0, 1], [0, 0, -1], [0, 0, 1]], float)
    hits = ray_box_candidates(
        origins,
        directions,
        np.array([-1, -1, -1.0]),
        np.array([1, 1, 1.0]),
        np.full(4, 10.0),
    )
    np.testing.assert_array_equal(hits, [0, 2])


@pytest.mark.parametrize("native", [False, True])
def test_alpha_sampler_orientation_and_multiple_layers(native):
    rgba = np.zeros((2, 2, 4), np.uint8)
    rgba[0, 0] = [255, 255, 255, 255]
    assets = [card(1, rgba), card(0, np.full((1, 1, 4), 255, np.uint8), 0)]
    origins = np.array(
        [[-0.5, 0.5, 2], [-0.5, -0.5, 2], [0.5, 0.5, 2], [0.5, -0.5, 2]], np.float32
    )
    _, distance, _ = AlphaRaycaster(assets, native=native).cast(
        origins, np.array([[0, 0, -1]] * 4, np.float32)
    )
    np.testing.assert_allclose(distance, [1, 2, 2, 2], atol=1e-5)


def test_native_can_traverse_more_than_64_transparent_layers():
    transparent = np.full((1, 1, 4), [0, 0, 0, 0], np.uint8)
    opaque = np.full((1, 1, 4), 255, np.uint8)
    assets = [card(z, transparent) for z in np.linspace(1, 1.8, 80)] + [
        card(0, opaque, 0)
    ]
    _, distance, _ = AlphaRaycaster(assets).cast(
        np.array([[0, 0, 2]], np.float32), np.array([[0, 0, -1]], np.float32)
    )
    np.testing.assert_allclose(distance, [2], atol=1e-5)


def test_double_buffered_camera_readback_preserves_previous_frame():
    import pybullet as p
    from rove_sim.render.gpu import GpuScene

    try:
        gpu = GpuScene(scene())
    except Exception as exc:
        import os

        if os.environ.get("SIM_REQUIRE_GPU") == "1":
            raise
        pytest.skip(f"No EGL GPU: {exc}")
    try:
        projection = p.computeProjectionMatrixFOV(90, 1, 0.05, 10)
        front = p.computeViewMatrix([0, 0, 2], [0, 0, 0], [0, 1, 0])
        away = p.computeViewMatrix([0, 0, 2], [0, 0, 3], [0, 1, 0])
        first = gpu.render(front, projection, 64, 64)
        second = gpu.render(away, projection, 64, 64)
        np.testing.assert_array_equal(
            gpu.render(front, projection, 64, 64, stream="test"), first
        )
        np.testing.assert_array_equal(
            gpu.render(away, projection, 64, 64, stream="test"), first
        )
        np.testing.assert_array_equal(
            gpu.render(away, projection, 64, 64, stream="test"), second
        )
    finally:
        gpu.close()


def test_moving_meshes_omit_own_sensor_and_follow_dynamic_props():
    import pybullet as p
    from rove_sim.world.moving_rays import MovingRaycaster
    from rove_sim.world.scene import SceneObject
    from types import SimpleNamespace

    connection = p.connect(p.DIRECT)
    try:
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
        robot = SimpleNamespace(
            body_id=p.createMultiBody(
                0, baseVisualShapeIndex=visual, basePosition=[0, 0, 1]
            )
        )
        prop = p.createMultiBody(0, basePosition=[0, 0, 2])
        spec = SceneObject(
            id="target", pose=(0, 0, 2), shape="box", extents=(1, 1, 0.2)
        )
        world = SimpleNamespace(scene_objects=[spec], objects={"target": prop})
        ray = MovingRaycaster(robot, world)
        ray.update()
        origins = np.array([[0, 0, 0]], np.float32)
        directions = np.array([[0, 0, 1]], np.float32)
        _, distance, body = ray.cast(origins, directions, 10, omit_link=-2)
        assert body[0] == robot.body_id and distance[0] == pytest.approx(0.9, abs=1e-5)
        _, distance, body = ray.cast(origins, directions, 10, omit_link=-1)
        assert body[0] == prop and distance[0] == pytest.approx(1.9, abs=1e-5)
        p.resetBasePositionAndOrientation(prop, [2, 0, 2], [0, 0, 0, 1])
        ray.update()
        _, _, body = ray.cast(origins, directions, 10, omit_link=-1)
        assert body[0] == -1
    finally:
        p.disconnect(connection)


def test_gpu_minification_filters_colour_without_filling_alpha_gaps():
    import pybullet as p
    from rove_sim.render.gpu import GpuScene

    # Many source texels per camera pixel: unfiltered sampling aliases to stripes.
    rgba = np.full((1024, 1024, 4), 255, np.uint8)
    rgba[::2, :, :3] = 0
    rgba[:, 512:, 3] = 0
    wall = np.full((1, 1, 4), [0, 255, 0, 255], np.uint8)
    try:
        gpu = GpuScene([card(1, rgba), card(0, wall, 0)])
    except Exception as exc:
        import os

        if os.environ.get("SIM_REQUIRE_GPU") == "1":
            raise
        pytest.skip(f"No EGL GPU: {exc}")
    try:
        view = p.computeViewMatrix([0, 0, 2], [0, 0, 0], [0, 1, 0])
        projection = p.computeProjectionMatrixFOV(90, 1, 0.05, 10)
        rgb = gpu.render(view, projection, 64, 64)
        solid = rgb[12:52, 12:24].astype(float)
        assert solid[..., 0].mean() > 70
        assert solid[..., 0].std() < 2
        # Sunlight has a warm tint and ambient light a cool tint. Filtering
        # must remain spatially uniform in every channel, not neutral white.
        assert np.all(solid.std(axis=(0, 1)) < 2)
        gap = rgb[24:40, 36:44]
        assert np.all(gap[..., 1] > 100) and np.all(gap[..., 0] < 5)
    finally:
        gpu.close()


def test_batched_tiles_match_direct_draws_exactly():
    import os
    import pybullet as p
    from rove_sim.render.gpu import GpuScene

    try:
        gpu = GpuScene(scene(), tile_metres=0.25, shadow_size=0)
    except Exception as exc:
        if os.environ.get("SIM_REQUIRE_GPU") == "1":
            raise
        pytest.skip(f"No EGL GPU: {exc}")
    try:
        if not gpu.indirect:
            pytest.skip("OpenGL 4.3 multi-draw unavailable")
        view = p.computeViewMatrix([0, 0, 2], [0, 0, 0], [0, 1, 0])
        projection = p.computeProjectionMatrixFOV(90, 1, 0.05, 10)
        batched = gpu.render(view, projection, 128, 128)
        gpu.indirect.clear()
        direct = gpu.render(view, projection, 128, 128)
        np.testing.assert_array_equal(batched, direct)
    finally:
        gpu.close()


def test_minified_leaf_colours_do_not_include_transparent_black_padding():
    import os
    import pybullet as p
    from rove_sim.render.gpu import GpuScene

    rgba = np.zeros((1024, 1024, 4), np.uint8)
    rgba[7::16, :] = [0, 255, 0, 255]
    rgba[8::16, :] = [0, 255, 0, 255]
    try:
        gpu = GpuScene([card(1, rgba)], shadow_size=0)
    except Exception as exc:
        if os.environ.get("SIM_REQUIRE_GPU") == "1":
            raise
        pytest.skip(f"No EGL GPU: {exc}")
    try:
        view = p.computeViewMatrix([0, 0, 2], [0, 0, 0], [0, 1, 0])
        projection = p.computeProjectionMatrixFOV(90, 1, 0.05, 10)
        rgb = gpu.render(view, projection, 64, 64)
        leaves = rgb[8:56, 8:56]
        assert np.all(leaves[..., 0] < 5)
        assert np.all(leaves[..., 1] > 200)
    finally:
        gpu.close()


def test_cached_shadows_leave_transparent_texels_unshadowed():
    import os
    import pybullet as p
    from rove_sim.render.gpu import GpuScene

    receiver = card(0, np.full((1, 1, 4), [0, 255, 0, 255], np.uint8), 0)
    receiver.vertices[:, :2] *= 3
    opaque = np.full((2, 2, 4), [255, 0, 0, 255], np.uint8)
    masked = opaque.copy()
    masked[:, 1, 3] = 0
    view = p.computeViewMatrix([0, 0, 5], [0, 0, 0], [0, 1, 0])
    projection = p.computeProjectionMatrixFOV(90, 1, 0.05, 10)
    values = []
    for rgba, size in [(opaque, 1024), (masked, 1024), (masked, 0)]:
        try:
            gpu = GpuScene([receiver, card(1, rgba)], shadow_size=size)
        except Exception as exc:
            if os.environ.get("SIM_REQUIRE_GPU") == "1":
                raise
            pytest.skip(f"No EGL GPU: {exc}")
        try:
            rgb = gpu.render(view, projection, 128, 128)
            # Sun casts the card's shadow to +X/+Y, outside the card's
            # camera silhouette. This ray crosses its transparent right half.
            values.append(int(rgb[60, 83, 1]))
            assert rgb[60, 83, 0] < 5  # Receiver, not the red caster.
        finally:
            gpu.close()
    assert values[1] > values[0] + 30
    assert abs(values[1] - values[2]) < 5


@pytest.mark.parametrize("flip_y", [True, False])
def test_texture_import_keeps_opacity_and_colour_orientation(tmp_path, flip_y):
    import hashlib
    from PIL import Image
    from tools.restore_terrain_textures import prepare_texture

    colour = np.zeros((2, 2, 4), np.uint8)
    colour[0] = [255, 0, 0, 255]
    colour[1] = [0, 255, 0, 255]
    Image.fromarray(colour).save(tmp_path / "colour.png")
    Image.fromarray(np.array([[255, 0], [0, 255]], np.uint8)).save(
        tmp_path / "mask.png"
    )
    entry = {
        "material": "test",
        "source_images": {
            "diffuse": "colour.png",
            "opacity": "mask.png",
            "flip_y": flip_y,
        },
        "source_sha256": {
            "colour.png": hashlib.sha256(
                (tmp_path / "colour.png").read_bytes()
            ).hexdigest()
        },
    }
    payload, hashes, masked = prepare_texture(entry, tmp_path)
    import io

    result = np.asarray(Image.open(io.BytesIO(payload)))
    np.testing.assert_array_equal(result[int(flip_y), 0], [255, 0, 0, 255])
    np.testing.assert_array_equal(result[1 - int(flip_y), 0], [0, 255, 0, 0])
    assert masked and hashes["colour.png"] == entry["source_sha256"]["colour.png"]
    (tmp_path / "colour.png").write_bytes(b"replaced")
    with pytest.raises(ValueError, match="checksum mismatch"):
        prepare_texture(entry, tmp_path)


def test_terrain_loader_rejects_replaced_texture(tmp_path):
    import hashlib
    import json
    from PIL import Image
    from rove_sim.world.mesh_assets import load_terrain

    (tmp_path / "mesh.obj").write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    texture = tmp_path / "texture.png"
    Image.new("RGBA", (2, 2), (255, 0, 0, 255)).save(texture)
    entry = {
        "obj": "mesh.obj",
        "tex": "texture.png",
        "texture_sha256": hashlib.sha256(texture.read_bytes()).hexdigest(),
    }
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({"visual": [entry]}))
    assert load_terrain(manifest)[0].rgba[0, 0, 0] == 255
    Image.new("RGBA", (2, 2), (0, 255, 0, 255)).save(texture)
    with pytest.raises(ValueError, match="checksum mismatch"):
        load_terrain(manifest)


def test_offset_inertial_frames_keep_visuals_aligned_with_bullet():
    import pybullet as p
    from types import SimpleNamespace
    from rove_sim.render.gpu import GpuScene

    client = p.connect(p.DIRECT)
    gpu = None
    try:
        visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.3, 0.2, 0.1], rgbaColor=[1, 0, 0, 1]
        )
        body = p.createMultiBody(
            baseMass=1,
            baseVisualShapeIndex=visual,
            basePosition=[0, 0, 0],
            baseInertialFramePosition=[0.4, -0.3, 0.2],
            baseInertialFrameOrientation=p.getQuaternionFromEuler([0.2, 0.1, 0.4]),
            linkMasses=[1],
            linkCollisionShapeIndices=[-1],
            linkVisualShapeIndices=[visual],
            linkPositions=[[0.7, 0, 0.2]],
            linkOrientations=[[0, 0, 0, 1]],
            linkInertialFramePositions=[[0.2, 0.1, 0]],
            linkInertialFrameOrientations=[p.getQuaternionFromEuler([0, 0, 0.3])],
            linkParentIndices=[0],
            linkJointTypes=[p.JOINT_FIXED],
            linkJointAxis=[[0, 0, 1]],
        )
        robot = SimpleNamespace(
            body_id=body, link_index={"fixture_root": -1, "fixture_link": 0}
        )
        try:
            gpu = GpuScene([], robot)
        except Exception as exc:
            import os

            if os.environ.get("SIM_REQUIRE_GPU") == "1":
                raise
            pytest.skip(f"No EGL GPU: {exc}")
        gpu.update_robot()
        view = p.computeViewMatrix([1.8, -2, 1.5], [0.3, 0, 0], [0, 0, 1])
        projection = p.computeProjectionMatrixFOV(55, 1, 0.05, 10)
        rgb = gpu.render(view, projection, 256, 256)
        expected = np.asarray(
            p.getCameraImage(
                256,
                256,
                viewMatrix=view,
                projectionMatrix=projection,
                renderer=p.ER_TINY_RENDERER,
            )[2]
        )
        actual_mask = (rgb[..., 0] > 30) & (rgb[..., 1] < 10)
        expected_mask = (expected[..., 0] > 30) & (expected[..., 1] < 10)
        assert (
            np.sum(actual_mask & expected_mask) / np.sum(actual_mask | expected_mask)
            > 0.95
        )
    finally:
        if gpu is not None:
            gpu.close()
        p.disconnect(client)


def test_robot_base_self_occlusion_uses_link_frame_with_offset_inertia():
    import pybullet as p
    from types import SimpleNamespace
    from rove_sim.robot.frames import base_link_pose
    from rove_sim.world.moving_rays import MovingRaycaster

    client = p.connect(p.DIRECT)
    try:
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
        body = p.createMultiBody(
            baseMass=1,
            baseVisualShapeIndex=visual,
            basePosition=[0, 0, 1],
            baseInertialFramePosition=[0.4, -0.3, 0.2],
            baseInertialFrameOrientation=p.getQuaternionFromEuler([0.2, 0.1, 0.4]),
        )
        np.testing.assert_allclose(base_link_pose(body)[0], [0, 0, 1], atol=1e-6)
        ray = MovingRaycaster(
            SimpleNamespace(body_id=body), SimpleNamespace(scene_objects=[], objects={})
        )
        ray.update()
        _, distance, hit = ray.cast(
            np.array([[0, 0, 0]], np.float32),
            np.array([[0, 0, 1]], np.float32),
            10,
            omit_link=-2,
        )
        assert hit[0] == body and distance[0] == pytest.approx(0.9, abs=1e-5)
    finally:
        p.disconnect(client)
