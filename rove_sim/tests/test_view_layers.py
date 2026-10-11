from types import SimpleNamespace
import time
import numpy as np
import pybullet as p
from rove_sim.render.layers import (
    camera_frustum,
    radiation_geometry,
    publish_cloud,
    read_cloud,
)


def test_cloud_is_the_published_world_scan_and_stale_data_is_hidden(tmp_path):
    path = tmp_path / "cloud.npz"
    points = np.array([[2, 3, 4], [5, 6, 7]], "f4")
    publish_cloud(path, [SimpleNamespace(points=points)])
    vertices, age = read_cloud(path)
    np.testing.assert_array_equal(vertices[:, :3], points)
    assert age is not None and age < 1
    np.savez(path, vertices=vertices, timestamp=time.time() - 2)
    assert len(read_cloud(path)[0]) == 0


def test_frustum_follows_actual_camera_pose_and_radiation_markers():
    view = p.computeViewMatrix([2, 3, 4], [2, 3, 3], [0, 1, 0])
    projection = p.computeProjectionMatrixFOV(90, 1, 0.05, 20)
    vertices = camera_frustum(view, projection)
    assert vertices.shape == (32, 6)
    np.testing.assert_allclose(vertices[24, :3], [2, 3, 4], atol=1e-5)
    np.testing.assert_allclose(vertices[25, 2], 2, atol=1e-5)
    markers = radiation_geometry([dict(id="source", position=[2, 3, 4])])
    np.testing.assert_allclose(markers[:2, :3].mean(axis=0), [2, 3, 4])


def test_gpu_yuv_matches_rgb_orientation_and_bt709_planes():
    from rove_sim.render.gpu import GpuScene
    from .test_texture_coverage import card
    rgba = np.full((4, 4, 4), 255, np.uint8)
    rgba[:2, :, :3] = [255, 0, 0]
    rgba[2:, :, :3] = [0, 0, 255]
    gpu = GpuScene([card(0, rgba, 0)], shadow_size=0)
    try:
        view = p.computeViewMatrix([0, 0, 2], [0, 0, 0], [0, 1, 0])
        projection = p.computeProjectionMatrixFOV(90, 1, .05, 10)
        rgb = gpu.render(view, projection, 64, 64).astype(float) / 255
        data = np.frombuffer(gpu.render(view, projection, 64, 64, output='yuv420p'), np.uint8)
        assert len(data) == 64 * 64 * 3 // 2
        expected_y = 16 + 219 * (rgb @ np.array([.2126, .7152, .0722]))
        small = rgb.reshape(32, 2, 32, 2, 3).mean(axis=(1, 3))
        expected_u = 128 + 224 * (small @ np.array([-.114572, -.385428, .5]))
        expected_v = 128 + 224 * (small @ np.array([.5, -.454153, -.045847]))
        for actual, expected in ((data[:4096].reshape(64, 64), expected_y),
                                 (data[4096:5120].reshape(32, 32), expected_u),
                                 (data[5120:].reshape(32, 32), expected_v)):
            np.testing.assert_allclose(actual, expected, atol=1)
        # Deferred transfers must return the same native-size frame as immediate reads.
        collect = gpu.render(view, projection, 64, 64, stream='deferred', output='yuv420p', defer=True)
        np.testing.assert_array_equal(np.frombuffer(collect(), np.uint8), data)
        blank = gpu.render(view, projection, 64, 64, scene=False)
        assert blank.std(axis=(0, 1)).max() < 1
        point = np.array([[0, 0, 0, 0, 1, 0]], 'f4')
        overlay = gpu.render(view, projection, 64, 64, scene=False, points=point)
        assert np.any(overlay[..., 1] > 200)
        marked = gpu.render(view, projection, 64, 64, scene=True, points=point)
        assert np.any(marked[..., 1] > 200)
    finally:
        gpu.close()
