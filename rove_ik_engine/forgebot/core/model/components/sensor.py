"""Sensor components: generic + camera, lidar, IMU, force/torque."""

from __future__ import annotations

from typing import Literal

from pydantic import Field

from .base import BaseComponent, register_component


@register_component("sensor")
class SensorComponent(BaseComponent):
    """Generic sensor when no specific subtype fits."""

    kind: str = "generic"
    update_rate: float = 30.0  # Hz
    noise_stddev: float = 0.0


@register_component("camera")
class CameraComponent(BaseComponent):
    width: int = 640
    height: int = 480
    fov: float = 1.047  # radians (60 deg)
    near: float = 0.01
    far: float = 100.0
    update_rate: float = 30.0
    depth: bool = False


@register_component("lidar")
class LidarComponent(BaseComponent):
    dimensionality: Literal["2d", "3d"] = "2d"
    channels: int = 1
    range_min: float = 0.05
    range_max: float = 30.0
    horizontal_resolution: float = 0.0087  # rad
    horizontal_min: float = -3.14159
    horizontal_max: float = 3.14159
    vertical_min: float = 0.0
    vertical_max: float = 0.0
    update_rate: float = 10.0
    noise_stddev: float = 0.0


@register_component("imu")
class IMUComponent(BaseComponent):
    accel_range: float = 19.6   # m/s²
    gyro_range: float = 8.726   # rad/s
    accel_noise: float = 0.0
    gyro_noise: float = 0.0
    update_rate: float = 100.0


@register_component("force_torque")
class ForceTorqueComponent(BaseComponent):
    force_range: float = 200.0   # N
    torque_range: float = 20.0   # N·m
    noise: float = 0.0
    update_rate: float = 100.0
    axes: list[str] = Field(default_factory=lambda: ["x", "y", "z"])
