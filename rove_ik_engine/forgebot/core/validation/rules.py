"""Diagnostic types: the unit of feedback from validators and importers."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Severity(str, Enum):
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


@dataclass
class Diagnostic:
    severity: Severity
    code: str
    message: str
    entity_id: str | None = None
    location: str | None = None  # free-form: "scene.toml#entities.foo" or "joint_1/limits"
    details: dict = field(default_factory=dict)

    @property
    def is_error(self) -> bool:
        return self.severity == Severity.ERROR


def has_errors(diagnostics: list[Diagnostic]) -> bool:
    return any(d.is_error for d in diagnostics)


def filter_severity(diagnostics: list[Diagnostic], severity: Severity) -> list[Diagnostic]:
    return [d for d in diagnostics if d.severity == severity]
