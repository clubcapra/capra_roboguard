"""Bridge runtime: receive loop + startup orchestration.

- ``RoveControlBridge`` — the runtime class (lives in ``core``)
- ``start(cfg)``        — discover ports, build strategy, run the bridge
- ``build_strategy()``  — pick the right ConversionStrategy from config
"""
from .core import RoveControlBridge
from .factory import build_strategy, start

__all__ = ["RoveControlBridge", "build_strategy", "start"]
