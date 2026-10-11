"""Sensor library: importing the package registers every sensor type."""
from . import camera, lidar, imu, radiation  # noqa: F401  (register side-effects)
