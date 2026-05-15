"""Scene-level validation: tree shape, IDs, parent/child consistency."""

from __future__ import annotations

from typing import Callable

from ..model import Project
from .rules import Diagnostic, Severity

Rule = Callable[[Project], list[Diagnostic]]


def check_unique_ids(project: Project) -> list[Diagnostic]:
    """Pydantic dict can't have dupes, so this catches edits done outside the model."""
    seen: set[str] = set()
    diags: list[Diagnostic] = []
    for eid in project.scene.entities:
        if eid in seen:
            diags.append(Diagnostic(Severity.ERROR, "scene.duplicate_id", f"duplicate entity id {eid}"))
        seen.add(eid)
    return diags


def check_parent_consistency(project: Project) -> list[Diagnostic]:
    diags: list[Diagnostic] = []
    for eid, e in project.scene.entities.items():
        if e.parent is not None:
            parent = project.scene.entities.get(e.parent)
            if parent is None:
                diags.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="scene.dangling_parent",
                        message=f"entity {eid} references unknown parent {e.parent!r}",
                        entity_id=eid,
                    )
                )
            elif eid not in parent.children:
                diags.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="scene.parent_child_mismatch",
                        message=f"entity {eid} thinks {e.parent} is its parent, "
                        "but parent doesn't list it as a child",
                        entity_id=eid,
                    )
                )
        for child_id in e.children:
            child = project.scene.entities.get(child_id)
            if child is None:
                diags.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="scene.dangling_child",
                        message=f"entity {eid} references unknown child {child_id!r}",
                        entity_id=eid,
                    )
                )
            elif child.parent != eid:
                diags.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="scene.parent_child_mismatch",
                        message=f"entity {eid} lists {child_id} as child, "
                        f"but child says its parent is {child.parent!r}",
                        entity_id=eid,
                    )
                )
    return diags


def check_no_cycles(project: Project) -> list[Diagnostic]:
    """Standard cycle detection via DFS coloring."""
    WHITE, GRAY, BLACK = 0, 1, 2
    color: dict[str, int] = {eid: WHITE for eid in project.scene.entities}
    diags: list[Diagnostic] = []

    def visit(eid: str, path: list[str]) -> None:
        if color[eid] == GRAY:
            cycle = " -> ".join(path[path.index(eid):] + [eid])
            diags.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="scene.cycle",
                    message=f"cycle in scene graph: {cycle}",
                    entity_id=eid,
                )
            )
            return
        if color[eid] == BLACK:
            return
        color[eid] = GRAY
        e = project.scene.entities[eid]
        for child_id in e.children:
            if child_id in color:
                visit(child_id, path + [eid])
        color[eid] = BLACK

    for eid in project.scene.entities:
        if color[eid] == WHITE:
            visit(eid, [])
    return diags


def check_roots(project: Project) -> list[Diagnostic]:
    diags: list[Diagnostic] = []
    declared_roots = set(project.scene.roots)
    actual_roots = {eid for eid, e in project.scene.entities.items() if e.parent is None}
    if declared_roots != actual_roots:
        diags.append(
            Diagnostic(
                severity=Severity.WARNING,
                code="scene.root_mismatch",
                message=f"scene.roots {sorted(declared_roots)} disagrees with "
                f"parent-pointers {sorted(actual_roots)}",
            )
        )
    return diags


RULES: list[Rule] = [check_unique_ids, check_parent_consistency, check_no_cycles, check_roots]


def validate(project: Project) -> list[Diagnostic]:
    out: list[Diagnostic] = []
    for rule in RULES:
        out.extend(rule(project))
    return out
