"""Sensor library: importing the package registers every sensor type."""
from . import camera, lidar, imu  # noqa: F401  (register side-effects)
