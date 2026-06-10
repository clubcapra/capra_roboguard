"""Robot state-source package. Importing it registers the built-in sources.

The live `rove_sensor_api` source is imported lazily inside build_state_source's
registry only when selected, so the package imports cleanly without a network/UDP
stack present.
"""
from .base import (RobotState, RobotStateSource, STATE_SOURCE_REGISTRY,
                   build_state_source)
from . import manual              # noqa: F401  (register on import)
from . import rove_sensor_api     # noqa: F401  (register the live source)

__all__ = ["RobotState", "RobotStateSource", "STATE_SOURCE_REGISTRY",
           "build_state_source"]
