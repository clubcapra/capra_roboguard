"""Read and write `.forgebot` archives.

A `.forgebot` is a standard ZIP holding:
  manifest.toml      — version, metadata, units, storage mode
  scene.toml         — entity tree + components (or scene.msgpack if storage.scene_format = "msgpack")
  systems.toml       — kinematic chains, signal graph, layout
  simulation.toml    — physics, controllers
  assets/materials.toml
  assets/meshes/*    — binary mesh files
  assets/textures/*  — binary texture files
  _source/*          — optional round-trip metadata (per-importer)
"""

from __future__ import annotations

from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

from ...core.model import Assets, MeshAsset, Project
from ...core.model.project import IKProfile

from . import asset_manager as assets_mod
from . import toml_codec as codec
from .versioning import migrate_manifest

ARCHIVE_PATHS = {
    "manifest": "manifest.toml",
    "scene_toml": "scene.toml",
    "scene_msgpack": "scene.msgpack",
    "systems": "systems.toml",
    "simulation": "simulation.toml",
    "materials": "assets/materials.toml",
}


def save(project: Project, output_path: Path | str) -> Path:
    """Write a project to `output_path` (creating or overwriting). Returns the path."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    project.touch()

    with ZipFile(output_path, "w", compression=ZIP_DEFLATED) as zf:
        manifest_dict = codec.manifest_to_dict(project.manifest)
        if project.home_pose:
            # Project-level config that doesn't fit anywhere else; ride along
            # in the manifest. Keys are joint entity ids.
            manifest_dict["home_pose"] = {k: float(v) for k, v in project.home_pose.items()}
        if project.ik_profiles:
            # TOML has no null type — drop None-valued fields so they round-trip
            # to the model's default on load.
            manifest_dict["ik_profiles"] = {
                base: {k: v for k, v in profile.model_dump().items() if v is not None}
                for base, profile in project.ik_profiles.items()
            }
        bindings_dict = project.bindings.model_dump(exclude_defaults=True)
        # Strip empty branches so projects without bindings stay clean.
        bindings_dict = {k: v for k, v in bindings_dict.items() if v}
        if bindings_dict:
            manifest_dict["bindings"] = bindings_dict
        zf.writestr(ARCHIVE_PATHS["manifest"], codec.dumps(manifest_dict))

        scene_format = project.manifest.storage.scene_format
        if scene_format == "msgpack":
            payload = _scene_to_msgpack(project)
            zf.writestr(ARCHIVE_PATHS["scene_msgpack"], payload)
        else:
            zf.writestr(
                ARCHIVE_PATHS["scene_toml"],
                codec.dumps(codec.scene_to_dict(project.scene)),
            )

        zf.writestr(ARCHIVE_PATHS["systems"], codec.dumps(codec.systems_to_dict(project.systems)))
        zf.writestr(
            ARCHIVE_PATHS["simulation"],
            codec.dumps(codec.simulation_to_dict(project.simulation)),
        )
        zf.writestr(
            ARCHIVE_PATHS["materials"],
            codec.dumps(codec.materials_to_dict(project.assets)),
        )

        # Mesh binaries: prefer in-memory data, fall back to disk paths.
        # Tracking which stems have been written prevents duplicate writes when
        # both `mesh_data` and `mesh_files` reference the same stem.
        written_meshes: set[str] = set()
        for stem, asset in project.assets.mesh_data.items():
            assets_mod.add_mesh_bytes(zf, asset.data, stem, asset.suffix)
            written_meshes.add(stem)
        for stem, path_str in project.assets.mesh_files.items():
            if stem in written_meshes:
                continue
            src = Path(path_str)
            if src.is_file():
                assets_mod.add_mesh_from_file(zf, src, stem=stem)

        written_textures: set[str] = set()
        for stem, asset in project.assets.texture_data.items():
            assets_mod.add_texture_bytes(zf, asset.data, stem, asset.suffix)
            written_textures.add(stem)
        for stem, path_str in project.assets.texture_files.items():
            if stem in written_textures:
                continue
            src = Path(path_str)
            if src.is_file():
                assets_mod.add_texture_from_file(zf, src, stem=stem)

    return output_path


def load(input_path: Path | str) -> Project:
    input_path = Path(input_path)
    with ZipFile(input_path, "r") as zf:
        manifest_raw = codec.loads_bytes(zf.read(ARCHIVE_PATHS["manifest"]))
        manifest_raw = migrate_manifest(manifest_raw)
        manifest = codec.manifest_from_dict(manifest_raw)
        home_pose_raw = manifest_raw.get("home_pose", {}) or {}
        home_pose = {str(k): float(v) for k, v in home_pose_raw.items()}
        ik_profiles_raw = manifest_raw.get("ik_profiles", {}) or {}
        bindings_raw = manifest_raw.get("bindings", {}) or {}

        if manifest.storage.scene_format == "msgpack":
            scene_blob = zf.read(ARCHIVE_PATHS["scene_msgpack"])
            scene = _scene_from_msgpack(scene_blob)
        else:
            scene_raw = codec.loads_bytes(zf.read(ARCHIVE_PATHS["scene_toml"]))
            scene = codec.scene_from_dict(scene_raw)

        systems = codec.systems_from_dict(_read_optional_toml(zf, ARCHIVE_PATHS["systems"]))
        simulation = codec.simulation_from_dict(
            _read_optional_toml(zf, ARCHIVE_PATHS["simulation"])
        )
        materials = codec.materials_from_dict(_read_optional_toml(zf, ARCHIVE_PATHS["materials"]))

        mesh_archive_paths = assets_mod.list_meshes(zf)
        texture_archive_paths = assets_mod.list_textures(zf)
        mesh_data = {
            stem: MeshAsset(suffix=Path(arc).suffix.lower() or ".stl", data=zf.read(arc))
            for stem, arc in mesh_archive_paths.items()
        }
        texture_data = {
            stem: MeshAsset(suffix=Path(arc).suffix.lower() or ".png", data=zf.read(arc))
            for stem, arc in texture_archive_paths.items()
        }
        assets = Assets(
            materials=materials,
            mesh_data=mesh_data,
            mesh_files=mesh_archive_paths,
            texture_data=texture_data,
            texture_files=texture_archive_paths,
        )

    ik_profiles: dict[str, IKProfile] = {}
    for base, raw in ik_profiles_raw.items():
        if isinstance(raw, dict):
            try:
                ik_profiles[str(base)] = IKProfile.model_validate(raw)
            except Exception:
                continue
    bindings_kwargs: dict = {}
    if bindings_raw:
        try:
            from ...core.model import Bindings
            bindings_kwargs["bindings"] = Bindings.model_validate(bindings_raw)
        except Exception:
            pass
    return Project(
        manifest=manifest,
        scene=scene,
        systems=systems,
        simulation=simulation,
        assets=assets,
        home_pose=home_pose,
        ik_profiles=ik_profiles,
        **bindings_kwargs,
    )


def _read_optional_toml(zf: ZipFile, name: str) -> dict:
    try:
        return codec.loads_bytes(zf.read(name))
    except KeyError:
        return {}


# ----- msgpack mode (lazy-imported so it's an optional dep) -----


def _scene_to_msgpack(project: Project) -> bytes:
    import msgpack

    return msgpack.packb(codec.scene_to_dict(project.scene), use_bin_type=True)


def _scene_from_msgpack(blob: bytes):
    import msgpack

    return codec.scene_from_dict(msgpack.unpackb(blob, raw=False))
