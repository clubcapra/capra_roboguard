"""HDR sky orientation, linear radiance and diffuse-light precomputation."""

import numpy as np
import pytest
from rove_sim.render.environment import read_hdr, environment_lighting, basis


def write_hdr(path, rows):
    height = len(rows)
    data = bytearray(
        f"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y {height} +X 8\n".encode()
    )
    for row in rows:
        data.extend([2, 2, 0, 8])
        for value in row:
            data.extend([136, value])
    path.write_bytes(data)


def test_hdr_rle_preserves_linear_values_and_rejects_truncation(tmp_path):
    path = tmp_path / "sky.hdr"
    write_hdr(path, [[64, 0, 0, 129], [0, 0, 128, 130]])
    rgb = read_hdr(path)
    np.testing.assert_allclose(rgb[0, 0], [0.5, 0, 0])
    np.testing.assert_allclose(rgb[1, 0], [0, 0, 2])
    path.write_bytes(path.read_bytes()[:-1])
    with pytest.raises(ValueError, match="Truncated"):
        read_hdr(path)


def test_uniform_environment_has_uniform_diffuse_light():
    rgb = np.ones((64, 128, 3), np.float32)
    coefficients, sun, colour, exposure = environment_lighting(rgb)
    normals = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, -1]])
    np.testing.assert_allclose(basis(normals) @ coefficients, 0.55, atol=0.001)
    np.testing.assert_allclose(colour, 0, atol=1e-6)
    assert exposure > 0 and sun[2] > 0


def test_solar_direction_rotates_with_panorama_and_is_not_double_counted():
    rgb = np.ones((64, 128, 3), np.float32)
    rgb[16:18, 20:22] = 1000
    sh, sun, colour, _ = environment_lighting(rgb)
    rotated_sh, rotated_sun, rotated_colour, _ = environment_lighting(rgb, 180)
    np.testing.assert_allclose(rotated_sun, sun * [-1, -1, 1], atol=1e-6)
    np.testing.assert_allclose(rotated_colour, colour, atol=1e-6)
    # The solar peak remains shadowable direct light, outside ambient SH.
    np.testing.assert_allclose(basis(np.array([[0, 0, 1]])) @ sh, 0.55, atol=0.001)
    assert np.all(colour > 0.1) and np.all(np.isfinite(rotated_sh))


def test_gpu_sky_uses_hdr_north_and_south_poles(tmp_path):
    import os
    import pybullet as p
    from rove_sim.render.gpu import GpuScene

    path = tmp_path / "sky.hdr"
    write_hdr(path, [[64, 0, 0, 129]] * 2 + [[0, 0, 64, 129]] * 2)
    try:
        gpu = GpuScene([], shadow_size=0, environment={"hdri": path})
    except Exception as exc:
        if os.environ.get("SIM_REQUIRE_GPU") == "1":
            raise
        pytest.skip(f"No EGL GPU: {exc}")
    try:
        projection = p.computeProjectionMatrixFOV(60, 1, 0.05, 10)
        for direction, channel in [(1, 0), (-1, 2)]:
            view = p.computeViewMatrix([0, 0, 0], [0, 0, direction], [0, 1, 0])
            rgb = gpu.render(view, projection, 32, 32)
            assert rgb[16, 16, channel] > 100
            assert rgb[16, 16, 2 - channel] < 5
    finally:
        gpu.close()
