"""Chain-walking helpers ported from frontend/src/components/viewport/ikChain.ts.

The editor decides the IK base from the tip alone: walk up parent pointers,
remember every link entity encountered, return the topmost one. The engine
mirrors this exactly so behaviour matches what the user trained against.
"""

from __future__ import annotations

from forgebot.core.model import Project


def find_ik_base(project: Project, tip_id: str) -> str:
    """Topmost link ancestor of `tip_id`. Returns `tip_id` if no ancestor link."""
    scene = project.scene
    if tip_id not in scene.entities:
        return tip_id
    result = tip_id
    cur = scene.entities[tip_id].parent
    for _ in range(64):
        if cur is None:
            break
        ent = scene.entities.get(cur)
        if ent is None:
            break
        if ent.get("link") is not None:
            result = cur
        cur = ent.parent
    return result


def count_movable_joints(project: Project, tip_id: str) -> int:
    """Movable joints between `tip_id` and its IK base."""
    base_id = find_ik_base(project, tip_id)
    count = 0
    scene = project.scene
    cur = scene.entities[tip_id].parent if tip_id in scene.entities else None
    for _ in range(64):
        if cur is None or cur == base_id:
            break
        ent = scene.entities.get(cur)
        if ent is None:
            break
        joint = ent.get("joint")
        if joint is not None and joint.type != "fixed":
            count += 1
        cur = ent.parent
    return count


def has_movable_joint_in_chain(project: Project, tip_id: str) -> bool:
    base_id = find_ik_base(project, tip_id)
    if base_id == tip_id:
        return False
    return count_movable_joints(project, tip_id) > 0
