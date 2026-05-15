"""Commands that bring external geometry into the project as new link entities."""

from __future__ import annotations

from ..model import (
    Entity,
    LinkComponent,
    MeshAsset,
    Project,
    TransformComponent,
    merge_subproject,
    new_entity_id,
)
from ..model.components.link import Geometry, Inertial, InertiaTensor
from .base import Command


class AddPartCommand(Command):
    """Add a single mesh as a new link entity in the current project.

    On execute: creates the link, registers mesh bytes under `stem` in
    `project.assets.mesh_data`, attaches a visual + collision referencing the
    stem, places the link under `parent_id` (or as a root).

    On undo: removes the link entity and the mesh data we registered.
    """

    def __init__(
        self,
        *,
        name: str,
        mesh_bytes: bytes,
        mesh_suffix: str,
        stem: str | None = None,
        parent_id: str | None = None,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self._name = name
        self._mesh_bytes = mesh_bytes
        self._mesh_suffix = mesh_suffix.lower() if mesh_suffix.startswith(".") else f".{mesh_suffix.lower()}"
        self._stem = stem or name
        self._parent_id = parent_id
        self._position = position

    def execute(self, project: Project) -> None:
        # Pick a unique mesh stem (don't clobber existing mesh of same name).
        stem = self._stem
        suffix = self._mesh_suffix
        n = 1
        while stem in project.assets.mesh_data:
            n += 1
            stem = f"{self._stem}_{n}"
        project.assets.mesh_data[stem] = MeshAsset(suffix=suffix, data=self._mesh_bytes)

        eid = new_entity_id("link")
        entity = Entity(id=eid, name=self._name)
        entity.attach(TransformComponent(position=self._position))
        entity.attach(
            LinkComponent(
                inertial=Inertial(
                    mass=0.0,
                    inertia=InertiaTensor(),
                ),
                visuals=[Geometry(mesh=stem)],
                collisions=[Geometry(mesh=stem)],
            )
        )
        project.scene.add(entity, parent=self._parent_id)
        self._undo_state = {"entity_id": eid, "stem": stem}

    def undo(self, project: Project) -> None:
        if self._undo_state is None:
            return
        eid = self._undo_state["entity_id"]
        stem = self._undo_state["stem"]
        if eid in project.scene:
            project.scene.remove(eid)
        project.assets.mesh_data.pop(stem, None)

    @property
    def description(self) -> str:
        return f"Add part '{self._name}'"


class MergeSubprojectCommand(Command):
    """Merge a sub-project (e.g. a library asset like the Robotiq gripper)
    into the current project under `parent_id`.

    On undo: remove every entity that was added by the merge.
    """

    def __init__(self, source: Project, parent_id: str | None = None) -> None:
        self._source = source
        self._parent_id = parent_id

    def execute(self, project: Project) -> None:
        before_eids = set(project.scene.entities.keys())
        before_meshes = set(project.assets.mesh_data.keys())
        before_materials = set(project.assets.materials.keys())
        new_root_ids = merge_subproject(project, self._source, self._parent_id)
        after_eids = set(project.scene.entities.keys())
        self._undo_state = {
            "added_entities": list(after_eids - before_eids),
            "added_meshes": list(set(project.assets.mesh_data.keys()) - before_meshes),
            "added_materials": list(set(project.assets.materials.keys()) - before_materials),
            "new_roots": new_root_ids,
        }

    def undo(self, project: Project) -> None:
        if self._undo_state is None:
            return
        # Remove top-down: kill new root subtrees (that handles descendants).
        for eid in self._undo_state["new_roots"]:
            if eid in project.scene:
                project.scene.remove(eid)
        # Anything that was added but not under a new root (defensive).
        for eid in self._undo_state["added_entities"]:
            if eid in project.scene:
                project.scene.remove(eid)
        for stem in self._undo_state["added_meshes"]:
            project.assets.mesh_data.pop(stem, None)
        for name in self._undo_state["added_materials"]:
            project.assets.materials.pop(name, None)

    @property
    def description(self) -> str:
        n = len(self._source.scene.entities)
        name = self._source.manifest.metadata.name or "subproject"
        return f"Merge '{name}' ({n} entities)"
