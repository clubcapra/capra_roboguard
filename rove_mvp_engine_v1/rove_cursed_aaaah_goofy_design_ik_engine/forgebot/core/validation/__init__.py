"""Validation: scene, physics, kinematic rules + Diagnostic types + collision.

Validators are organized as Chain of Responsibility — each rule is a pure
function `(Project) -> list[Diagnostic]` and the validator just sums their
output. Add a new rule = append one function to a module's `RULES` list.
"""

from __future__ import annotations

from ..model import Project
from . import kinematic_validator, physics_validator, scene_validator
from .collision import CollisionPair, check_collisions, clear_collision_cache
from .rules import Diagnostic, Severity, filter_severity, has_errors


def validate_all(project: Project) -> list[Diagnostic]:
    """Run every validator in order. Returns one flat list of diagnostics."""
    out: list[Diagnostic] = []
    out.extend(scene_validator.validate(project))
    out.extend(physics_validator.validate(project))
    out.extend(kinematic_validator.validate(project))
    return out


__all__ = [
    "CollisionPair",
    "Diagnostic",
    "Severity",
    "check_collisions",
    "clear_collision_cache",
    "filter_severity",
    "has_errors",
    "kinematic_validator",
    "physics_validator",
    "scene_validator",
    "validate_all",
]
