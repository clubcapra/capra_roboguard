"""Dose falloff, superposition and mount pose must agree with the visible detector."""

from types import SimpleNamespace
import numpy as np
import pybullet as p
import pytest
from rove_sim.sensors.radiation import dose_at, RadiationSensor
from rove_sim.core.clock import SimClock


def test_inverse_square_sum_and_near_source_clamp():
    sources = [dict(id="a", position=[0, 0, 0], dose_rate_at_1m_usv_h=4)]
    at_one, _ = dose_at([1, 0, 0], sources, background=0)
    at_two, _ = dose_at([2, 0, 0], sources, background=0)
    assert at_one == pytest.approx(4)
    assert at_two == pytest.approx(at_one / 4)
    coincident, _ = dose_at([0, 0, 0], sources, background=0, minimum_distance=0.1)
    assert coincident == pytest.approx(400)
    total, contributions = dose_at([1, 0, 0], sources * 2, background=0.1)
    assert total == pytest.approx(8.1)
    assert len(contributions) == 2
    with pytest.raises(ValueError):
        dose_at([1, 0, 0], sources, minimum_distance=0)


def test_detector_offset_rotates_and_reading_changes_with_arm_pose():
    connection = p.connect(p.DIRECT)
    try:
        body = p.createMultiBody(baseMass=0)
        robot = SimpleNamespace(body_id=body, link_index={"mount": -1})
        sensor = RadiationSensor(
            name="test",
            robot=robot,
            link="mount",
            clock=SimClock(),
            offset=[1, 0, 0],
            background_usv_h=0,
        )
        sensor.sources = [dict(id="a", position=[0, 0, 0], dose_rate_at_1m_usv_h=4)]
        assert sensor.sample().dose_rate_usv_h == pytest.approx(4)
        p.resetBasePositionAndOrientation(
            body, [0, 1, 0], p.getQuaternionFromEuler([0, 0, np.pi / 2])
        )
        reading = sensor.sample()
        np.testing.assert_allclose(reading.position, [0, 2, 0], atol=1e-6)
        assert reading.dose_rate_usv_h == pytest.approx(1)
        with pytest.raises(ValueError):
            RadiationSensor(name="bad", robot=robot, link="missing", clock=SimClock())
    finally:
        p.disconnect(connection)
