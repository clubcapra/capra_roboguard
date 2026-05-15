"""Merge a sub-project into a target project.

The sub-project contributes its scene tree (re-IDed to avoid collisions),
its mesh/texture data, and its materials. Returns the list of new root
entity IDs in the target project so callers can navigate to them.
"""

from __future__ import annotations

import secrets

from .components import BaseComponent, parse_component
from .entity import Entity
from .project import Project


def _new_id_like(old_id: str) -> str:
    """Generate a fresh ID with the same `ent_<type>_` prefix as `old_id`.

    Falls back to `ent_misc_` if `old_id` doesn't fit the convention.
    """
    if old_id.startswith("ent_") and old_id.count("_") >= 2:
        prefix = "_".join(old_id.split("_")[:2])  # "ent_link"
        return f"{prefix}_{secrets.token_hex(4)}"
    return f"ent_misc_{secrets.token_hex(4)}"


def merge_subproject(
    target: Project,
    source: Project,
    parent_id: str | None = None,
) -> list[str]:
    """Add `source`'s entities under `parent_id` (or as roots), copy assets.

    Returns the new IDs of entities that were roots in `source`.
    """
    # 1. Build an id remap: old -> new, with collision-free new IDs in target.
    id_map: dict[str, str] = {}
    for old_eid in source.scene.entities:
        new_eid = _new_id_like(old_eid)
        # Defend against (extremely unlikely) collision in the target.
        while new_eid in target.scene.entities or new_eid in id_map.values():
            new_eid = _new_id_like(old_eid)
        id_map[old_eid] = new_eid

    # 2. Clone every entity with remapped IDs and patch component references
    #    (joint.parent_link / joint.child_link reference entity IDs too).
    cloned: dict[str, Entity] = {}
    for old_eid, e in source.scene.entities.items():
        new_eid = id_map[old_eid]
        new_components: dict[str, BaseComponent] = {}
        for key, comp in e.components.items():
            data = comp.model_dump(exclude_none=True)
            # Remap any field that holds an entity ID.
            for ref_field in ("parent_link", "child_link", "target_joint", "mimic"):
                if ref_field in data and isinstance(data[ref_field], str):
                    val = data[ref_field]
                    if val in id_map:
                        data[ref_field] = id_map[val]
            new_components[key] = parse_component(key, data)
        cloned[new_eid] = Entity(
            id=new_eid,
            name=e.name,
            parent=id_map.get(e.parent) if e.parent else None,
            children=[id_map[c] for c in e.children if c in id_map],
            components=new_components,
        )

    # 3. Drop them into the target's entity dict.
    for new_eid, e in cloned.items():
        target.scene.entities[new_eid] = e

    # 4. Wire each former-root under the chosen parent (or as a new target root).
    new_root_ids: list[str] = []
    for old_root_id in source.scene.roots:
        new_root_id = id_map.get(old_root_id)
        if new_root_id is None:
            continue
        new_root_ids.append(new_root_id)
        target.scene.reparent(new_root_id, parent_id)

    # 5. Copy assets. On stem collision, source wins (caller picks recently
    #    imported asset by default; users can rename in the model later).
    target.assets.materials.update(source.assets.materials)
    target.assets.mesh_data.update(source.assets.mesh_data)
    target.assets.mesh_files.update(source.assets.mesh_files)
    target.assets.texture_data.update(source.assets.texture_data)
    target.assets.texture_files.update(source.assets.texture_files)

    return new_root_ids
