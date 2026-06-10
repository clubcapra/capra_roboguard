"""VN300 IMU/GNSS sensor (basic; M4 adds the bias/random-walk/GNSS-spoof models).

Reads the mount link's orientation, angular velocity and a finite-difference
linear acceleration (incl. gravity reaction, expressed in the body frame, like a
real accelerometer), plus a world-XY "GNSS" position placeholder. The error model
is the default identity for now -- M4 composes bias/noise/scale into _apply_errors
without touching this clean-physics _sample().
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Tuple

import numpy as np
import pybullet as p

from .base import Sensor, register

_G = np.array([0.0, 0.0, -9.81])


@dataclass
class ImuReading:
    orientation: Tuple[float, float, float, float]    # quat xyzw
    angular_velocity: Tuple[float, float, float]      # rad/s, body frame
    linear_acceleration: Tuple[float, float, float]   # m/s^2, body frame (specific force)
    position: Tuple[float, float, float]              # world xyz ("GNSS" placeholder)
    t: float = 0.0
    meta: Dict[str, Any] = field(default_factory=dict)


@register("vn300")
class VN300(Sensor):
    def __init__(self, **kw):
        kw.setdefault("rate_hz", float(kw.pop("imu_rate_hz", 200.0)))
        kw.pop("imu_rate_hz", None)
        # M4 error model: white noise + slowly random-walking bias on gyro/accel.
        self.gyro_noise = float(kw.pop("gyro_noise", 0.002))      # rad/s
        self.accel_noise = float(kw.pop("accel_noise", 0.02))     # m/s^2
        self.gyro_walk = float(kw.pop("gyro_bias_walk", 1e-5))
        self.accel_walk = float(kw.pop("accel_bias_walk", 1e-4))
        self.errors = bool(kw.pop("errors", True))
        super().__init__(**kw)
        self._rng = np.random.default_rng(int(self.params.get("seed", 0)))
        self._gbias = np.zeros(3)
        self._abias = np.zeros(3)
        self._last_v = np.zeros(3)
        self._last_t = None

    def _apply_errors(self, r):
        """Real IMUs aren't perfect: add a random-walking bias + white noise so
        SLAM/INS must cope with drift (M4). Set errors:false for ground truth."""
        if not self.errors or r is None:
            return r
        self._gbias += self._rng.normal(0, self.gyro_walk, 3)
        self._abias += self._rng.normal(0, self.accel_walk, 3)
        w = np.array(r.angular_velocity) + self._gbias + self._rng.normal(0, self.gyro_noise, 3)
        a = np.array(r.linear_acceleration) + self._abias + self._rng.normal(0, self.accel_noise, 3)
        r.angular_velocity = tuple(w)
        r.linear_acceleration = tuple(a)
        return r

    def _sample(self) -> ImuReading:
        pos, orn = self.link_pose()
        lin, ang = p.getBaseVelocity(self.robot.body_id)
        v = np.array(lin)
        now = self.clock.now() if self.clock else 0.0
        dt = (now - self._last_t) if self._last_t is not None else 0.0
        a_world = (v - self._last_v) / dt if dt > 1e-9 else np.zeros(3)
        self._last_v, self._last_t = v, now
        # specific force an accelerometer measures = a - g, rotated into body frame
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        f_body = R.T @ (a_world - _G)
        w_body = R.T @ np.array(ang)
        return ImuReading(orientation=tuple(orn), angular_velocity=tuple(w_body),
                          linear_acceleration=tuple(f_body), position=tuple(pos),
                          t=now)
