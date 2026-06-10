"""Perception source: where RealWorld gets the objects it inserts into PyBullet.

In real mode the sim is a geometric WORLD MODEL: the robot is synced from
telemetry and the surrounding objects come from live lidar/camera detections,
materialised as primitive bodies so PyBullet can answer IK / collision /
spatial-reasoning queries against them. That detection pipeline (vision + lidar
fusion) does not exist yet -- there are no detection protos in the stack -- so
this is a seam with a null default. When the protos land, add a concrete
PerceptionSource (e.g. `livox_detector`, `vision_detector`) behind this same
interface and RealWorld picks it up unchanged.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Sequence

from ..registry import Registry

PERCEPTION_REGISTRY = Registry("perception")
register = PERCEPTION_REGISTRY.register


@dataclass
class Detection:
    """A perceived object to place in the world model."""
    id: str                                       # stable key (track id)
    pose: Sequence[float]                         # world xyz
    extents: Sequence[float] = (0.2, 0.2, 0.2)    # AABB half-... full extents (m)
    orn: Sequence[float] = (0, 0, 0, 1)           # world quaternion xyzw
    cls: str = "object"                           # detected class label
    meta: Dict[str, Any] = field(default_factory=dict)


class PerceptionSource(ABC):
    @abstractmethod
    def poll(self) -> List[Detection]:
        """Return the current set of detections (latest snapshot)."""

    def start(self) -> None: ...
    def stop(self) -> None: ...


@register("null")
class NullPerception(PerceptionSource):
    """No perception -- an empty world model. The default until detection
    protos exist; real mode runs end-to-end with this (robot syncs, world empty)."""

    def __init__(self, **_):
        pass

    def poll(self) -> List[Detection]:
        return []


def build_perception(spec: Dict[str, Any] | None) -> PerceptionSource:
    spec = dict(spec or {})
    key = spec.get("use", "null")
    return PERCEPTION_REGISTRY.build(key, **{k: v for k, v in spec.items()
                                            if k != "use"})
