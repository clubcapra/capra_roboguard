"""World seam package. Importing it registers the built-in world types."""
from .base import World, WORLD_REGISTRY, build_world
from . import mock, real          # noqa: F401  (register on import)

__all__ = ["World", "WORLD_REGISTRY", "build_world"]
