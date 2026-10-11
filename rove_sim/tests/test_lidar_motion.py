import numpy as np

from rove_sim.world.motion import interpolate_state, quaternion_mix


def state(t, x, angle):
    return {
        "timestamp_ns": t,
        "pos": [x, 0, 0],
        "orn": [0, 0, np.sin(angle / 2), np.cos(angle / 2)],
        "q": [x],
        "objects": {},
    }


def test_continuous_pose_and_shortest_rotation_without_prediction():
    a, b = state(1_000_000_000, 0, 0), state(1_100_000_000, 0.1, np.pi / 2)
    source = {**b, "history": [a, b]}
    poses = [
        interpolate_state(source, 1_000_000_000 + i * 10_000_000) for i in range(11)
    ]
    assert np.allclose([s["pos"][0] for s in poses], np.linspace(0, 0.1, 11))
    assert np.allclose(poses[5]["orn"], [0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8)])
    assert interpolate_state(source, 2_000_000_000) is b
    assert np.allclose(quaternion_mix(a["orn"], -np.asarray(a["orn"]), 0.5), a["orn"])


def test_reset_is_not_interpolated_into_a_false_lidar_sweep():
    a, b = state(1_000_000_000, 0, 0), state(1_100_000_000, 10, 0)
    assert interpolate_state({**b, "history": [a, b]}, 1_050_000_000) is b
