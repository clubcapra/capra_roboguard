"""Point-source radiation sensor: sum inverse-square dose rates, no ray tracing.

Sources specify a simulated dose rate at one metre, rather than activity in Bq:
activity alone cannot determine dose without a radionuclide/energy model.
"""

from dataclasses import dataclass, asdict
import numpy as np
import pybullet as p
from .base import Sensor, register


def dose_at(position, sources, background=0.1, minimum_distance=0.1):
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("Detector position must be finite XYZ")
    if (
        not np.isfinite([background, minimum_distance]).all()
        or background < 0
        or minimum_distance <= 0
    ):
        raise ValueError("Background must be nonnegative and minimum distance positive")
    contributions = []
    for source in sources:
        centre = np.asarray(source["position"], dtype=float)
        strength = float(source["dose_rate_at_1m_usv_h"])
        if (
            centre.shape != (3,)
            or not np.isfinite(centre).all()
            or not np.isfinite(strength)
            or strength < 0
        ):
            raise ValueError("Invalid radiation source position or strength")
        distance = float(np.linalg.norm(position - centre))
        dose = strength / max(distance, minimum_distance) ** 2
        contributions.append(
            dict(id=source["id"], distance_m=distance, dose_rate_usv_h=dose)
        )
    return (
        float(background + sum(c["dose_rate_usv_h"] for c in contributions)),
        contributions,
    )


@dataclass
class RadiationReading:
    position: list
    dose_rate_usv_h: float
    contributions: list
    t: float

    def to_dict(self):
        return asdict(self)


@register("radiation")
class RadiationSensor(Sensor):
    """Profile-configured arm-mounted detector; follows the normal sensor contract."""

    def __init__(self, **kw):
        super().__init__(**kw)
        if self.link not in self.robot.link_index:
            raise ValueError(f"Unknown radiation detector mount: {self.link}")
        self.offset = np.asarray(self.params.get("offset", [0, 0, 0]), dtype=float)
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError("Detector offset must be finite XYZ")
        self.sources = []
        self.background = float(self.params.get("background_usv_h", 0.1))
        self.minimum_distance = float(self.params.get("minimum_distance_m", 0.1))
        self.visual_extents = tuple(
            self.params.get("visual_extents", [0.07, 0.04, 0.025])
        )
        self.visual_rgba = (0.18, 0.8, 0.95, 1)
        dose_at([0, 0, 0], [], self.background, self.minimum_distance)

    def detector_pose(self):
        pos, orn = self.link_pose()
        rotation = np.asarray(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        return np.asarray(pos) + rotation @ self.offset, orn

    def _sample(self):
        pos, _ = self.detector_pose()
        dose, contributions = dose_at(
            pos, self.sources, self.background, self.minimum_distance
        )
        return RadiationReading(pos.tolist(), dose, contributions, float(self.clock.t))
