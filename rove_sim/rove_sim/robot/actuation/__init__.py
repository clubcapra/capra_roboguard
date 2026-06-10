"""Actuator library. Importing this package registers all actuator types."""
from .base import Actuator, ACTUATOR_REGISTRY, build_actuators, register  # noqa: F401
from . import tracks      # noqa: F401  registers "differential_tracks"
from . import flippers    # noqa: F401  registers "stepped_flippers"
from . import arm         # noqa: F401  registers "arm_ik"
from . import gripper      # noqa: F401  registers "mimic_gripper"
