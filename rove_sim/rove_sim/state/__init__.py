"""Robot state-source package. Importing it registers the built-in sources.

The live `HALo` source registers on import. Its device codecs load only when
constructed, allowing API and state modules to import in either order.
"""
from .base import (RobotState, RobotStateSource, STATE_SOURCE_REGISTRY,
                   build_state_source)
from . import manual              # noqa: F401  (register on import)
from . import halo     # noqa: F401  (register the live source)

__all__ = ["RobotState", "RobotStateSource", "STATE_SOURCE_REGISTRY",
           "build_state_source"]
