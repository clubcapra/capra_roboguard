"""Physics validation: mass, inertia tensor sanity."""

from __future__ import annotations

from typing import Callable, cast

from ..model import LinkComponent, Project
from .rules import Diagnostic, Severity

Rule = Callable[[Project], list[Diagnostic]]


def check_mass_nonnegative(project: Project) -> list[Diagnostic]:
    diags: list[Diagnostic] = []
    for e in project.scene.entities.values():
        link = cast(LinkComponent | None, e.get("link"))
        if link is None:
            continue
        if link.inertial.mass < 0:
            diags.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="physics.negative_mass",
                    message=f"link '{e.name}' has negative mass {link.inertial.mass}",
                    entity_id=e.id,
                )
            )
    return diags


def check_inertia_diagonal_positive(project: Project) -> list[Diagnostic]:
    """Diagonal inertia entries must be > 0 if mass > 0; off-diagonals can be anything."""
    diags: list[Diagnostic] = []
    for e in project.scene.entities.values():
        link = cast(LinkComponent | None, e.get("link"))
        if link is None or link.inertial.mass <= 0:
            continue
        i = link.inertial.inertia
        for axis, val in (("ixx", i.ixx), ("iyy", i.iyy), ("izz", i.izz)):
            if val <= 0:
                diags.append(
                    Diagnostic(
                        severity=Severity.WARNING,
                        code="physics.zero_inertia_diagonal",
                        message=f"link '{e.name}' has mass {link.inertial.mass} "
                        f"but {axis}={val} (must be > 0 for a real body)",
                        entity_id=e.id,
                    )
                )
    return diags


def check_inertia_triangle_inequality(project: Project) -> list[Diagnostic]:
    """Principal-axis inertias must satisfy a + b >= c for every permutation.

    We approximate by only checking the diagonal (assuming the tensor is roughly
    diagonal in its given frame). A full check would diagonalize, but that's
    overkill for a quick warning.
    """
    diags: list[Diagnostic] = []
    for e in project.scene.entities.values():
        link = cast(LinkComponent | None, e.get("link"))
        if link is None or link.inertial.mass <= 0:
            continue
        i = link.inertial.inertia
        a, b, c = i.ixx, i.iyy, i.izz
        if a + b < c or a + c < b or b + c < a:
            diags.append(
                Diagnostic(
                    severity=Severity.WARNING,
                    code="physics.inertia_triangle",
                    message=f"link '{e.name}' inertia diagonal "
                    f"({a:g}, {b:g}, {c:g}) violates triangle inequality",
                    entity_id=e.id,
                )
            )
    return diags


RULES: list[Rule] = [
    check_mass_nonnegative,
    check_inertia_diagonal_positive,
    check_inertia_triangle_inequality,
]


def validate(project: Project) -> list[Diagnostic]:
    out: list[Diagnostic] = []
    for rule in RULES:
        out.extend(rule(project))
    return out
