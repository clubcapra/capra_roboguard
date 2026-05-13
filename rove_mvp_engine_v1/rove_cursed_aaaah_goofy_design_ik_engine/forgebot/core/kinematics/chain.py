"""Extract a kinematic chain (ordered list of joints) from the scene graph.

A chain is the sequence of joint entities along the path from `base` link
to `tip` link. The path may pass through intermediate links.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import cast

from ..model import JointComponent, Project


@dataclass
class KinematicChain:
    base: str  # link entity id
    tip: str  # link entity id
    joints: list[str]  # joint entity ids in order from base to tip


def extract_chain(project: Project, base: str, tip: str) -> KinematicChain:
    """Walk up the parent pointers from `tip` until we hit `base`.

    Returns the joints found along the way, in order from base to tip.
    Raises if there's no path.
    """
    scene = project.scene
    if base not in scene.entities:
        raise KeyError(f"base entity not found: {base}")
    if tip not in scene.entities:
        raise KeyError(f"tip entity not found: {tip}")

    # Walk up from tip, collecting ancestor entity ids until we reach base.
    path_up: list[str] = [tip]
    cur = scene.entities[tip].parent
    while cur is not None and cur != base:
        path_up.append(cur)
        cur = scene.entities[cur].parent
    if cur != base:
        raise ValueError(f"no path from {base} to {tip} in scene graph")
    path_up.append(base)

    # path_up is tip, ..., base. Reverse to base, ..., tip and pick out joints.
    path = list(reversed(path_up))
    joints: list[str] = []
    for eid in path:
        e = scene.entities[eid]
        if cast(JointComponent | None, e.get("joint")) is not None:
            joints.append(eid)
    return KinematicChain(base=base, tip=tip, joints=joints)
