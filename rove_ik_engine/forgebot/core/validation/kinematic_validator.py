"""Kinematic validation: joint references, limits, axis sanity."""

from __future__ import annotations

from typing import Callable, cast

from ..model import JointComponent, Project
from .rules import Diagnostic, Severity

Rule = Callable[[Project], list[Diagnostic]]


def check_joint_links_exist(project: Project) -> list[Diagnostic]:
    diags: list[Diagnostic] = []
    for e in project.scene.entities.values():
        j = cast(JointComponent | None, e.get("joint"))
        if j is None:
            continue
        if j.parent_link not in project.scene.entities:
            diags.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="kinematic.joint_unknown_parent",
                    message=f"joint '{e.name}' parent_link {j.parent_link!r} not in scene",
                    entity_id=e.id,
                )
            )
        if j.child_link not in project.scene.entities:
            diags.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="kinematic.joint_unknown_child",
                    message=f"joint '{e.name}' child_link {j.child_link!r} not in scene",
                    entity_id=e.id,
                )
            )
    return diags


def check_joint_limits_consistent(project: Project) -> list[Diagnostic]:
    diags: list[Diagnostic] = []
    for e in project.scene.entities.values():
        j = cast(JointComponent | None, e.get("joint"))
        if j is None or j.limits is None:
            continue
        if j.limits.upper < j.limits.lower:
            diags.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="kinematic.bad_limits",
                    message=f"joint '{e.name}' has upper {j.limits.upper} < lower {j.limits.lower}",
                    entity_id=e.id,
                )
            )
        if j.type in ("revolute", "prismatic") and j.limits.upper == j.limits.lower:
            diags.append(
                Diagnostic(
                    severity=Severity.WARNING,
                    code="kinematic.zero_range",
                    message=f"joint '{e.name}' has zero range — should it be type=fixed?",
                    entity_id=e.id,
                )
            )
    return diags


def check_axis_nonzero(project: Project) -> list[Diagnostic]:
    diags: list[Diagnostic] = []
    for e in project.scene.entities.values():
        j = cast(JointComponent | None, e.get("joint"))
        if j is None or j.type == "fixed":
            continue
        norm_sq = sum(c * c for c in j.axis)
        if norm_sq < 1e-12:
            diags.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="kinematic.zero_axis",
                    message=f"joint '{e.name}' (type {j.type}) has zero axis vector",
                    entity_id=e.id,
                )
            )
    return diags


RULES: list[Rule] = [check_joint_links_exist, check_joint_limits_consistent, check_axis_nonzero]


def validate(project: Project) -> list[Diagnostic]:
    out: list[Diagnostic] = []
    for rule in RULES:
        out.extend(rule(project))
    return out
