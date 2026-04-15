"""Load the robot at engine startup.

Tries the .forgebot archive first (lossless: keeps IK profiles, metadata,
units, etc.). Falls back to URDF + sidecar ik_profile.json if the archive
is missing or fails to load.
"""

from __future__ import annotations

import hashlib
import json
import logging
from pathlib import Path

from forgebot.core.model import IKProfile, Project
from forgebot.core.model.components import parse_component
from forgebot.core.model.entity import Entity
from forgebot.io.importers.urdf_importer import URDFImporter
from forgebot.io.serializer import load as load_forgebot

from .config import EngineConfig, resolve

_log = logging.getLogger(__name__)


def load_robot(cfg: EngineConfig) -> Project:
    forgebot_path = resolve(cfg, cfg.robot.forgebot)
    urdf_path = resolve(cfg, cfg.robot.urdf)

    if forgebot_path.exists():
        try:
            project = load_forgebot(forgebot_path)
            _log.info("loaded robot from %s", forgebot_path)
            return project
        except Exception as e:  # noqa: BLE001
            _log.warning(
                "%s failed to load (%s); falling back to URDF", forgebot_path, e
            )

    if not urdf_path.exists():
        raise FileNotFoundError(
            f"no robot found: neither {forgebot_path} nor {urdf_path} exists"
        )

    result = URDFImporter().import_file(urdf_path)
    _log.info("loaded robot from %s", urdf_path)

    # URDFImporter mints entity ids with secrets.token_hex -> they're random on
    # every load. Anything keyed by entity id (persisted sync offsets, saved
    # poses, the reanchor-on-boot map, cached client target ids) would silently
    # stop matching after a restart. Rewrite to deterministic, name-derived ids
    # so the same URDF yields identical ids every load (the .forgebot path
    # already has stable baked ids, so it skips this).
    _stabilise_entity_ids(result.project)

    # When loading URDF, IK profiles aren't in the file. Look for a sidecar
    # data/ik_profile.json that the exporter writes; merge it in.
    profile_path = urdf_path.parent / "ik_profile.json"
    if profile_path.exists():
        _merge_ik_profiles(result.project, profile_path)

    return result.project


def _stabilise_entity_ids(project: Project) -> None:
    """Rewrite imported entity ids to deterministic ids derived from entity
    name + role, so the same robot file yields the same ids on every load.

    Mirrors the id-remap surface in forgebot.core.model.merge (entity id /
    parent / children, scene roots, the component ref fields parent_link /
    child_link / target_joint / mimic, plus project home_pose / ik_profiles
    keys). Collisions (duplicate names) are disambiguated deterministically."""
    scene = project.scene

    def _bucket(eid: str) -> str:
        if eid.startswith("ent_") and eid.count("_") >= 2:
            return eid.split("_")[1]
        return "misc"

    id_map: dict[str, str] = {}
    used: set[str] = set()
    for old_eid, ent in scene.entities.items():
        bucket = _bucket(old_eid)
        basis = (ent.name or "").strip().lower() or old_eid
        n = 0
        while True:
            salt = basis if n == 0 else f"{basis}#{n}"
            new_eid = f"ent_{bucket}_{hashlib.sha1(f'{bucket}:{salt}'.encode()).hexdigest()[:8]}"
            if new_eid not in used:
                break
            n += 1
        used.add(new_eid)
        id_map[old_eid] = new_eid

    _REF_FIELDS = ("parent_link", "child_link", "target_joint", "mimic")
    new_entities: dict[str, Entity] = {}
    for old_eid, e in scene.entities.items():
        new_eid = id_map[old_eid]
        new_components = {}
        for key, comp in e.components.items():
            data = comp.model_dump(exclude_none=True)
            for ref_field in _REF_FIELDS:
                v = data.get(ref_field)
                if isinstance(v, str) and v in id_map:
                    data[ref_field] = id_map[v]
            new_components[key] = parse_component(key, data)
        new_entities[new_eid] = Entity(
            id=new_eid,
            name=e.name,
            parent=id_map.get(e.parent) if e.parent else None,
            children=[id_map[c] for c in e.children if c in id_map],
            components=new_components,
        )
    scene.entities = new_entities
    scene.roots = [id_map.get(r, r) for r in scene.roots]

    if project.home_pose:
        project.home_pose = {id_map.get(k, k): v for k, v in project.home_pose.items()}
    if project.ik_profiles:
        project.ik_profiles = {id_map.get(k, k): v for k, v in project.ik_profiles.items()}


def _merge_ik_profiles(project: Project, path: Path) -> None:
    try:
        data = json.loads(path.read_text())
    except Exception as e:  # noqa: BLE001
        _log.warning("could not read %s: %s", path, e)
        return
    # data may be {base_id: profile_dict} or a single profile.
    if isinstance(data, dict) and any(isinstance(v, dict) for v in data.values()):
        for base_id, prof in data.items():
            if not isinstance(prof, dict):
                continue
            try:
                project.ik_profiles[base_id] = IKProfile(**prof)
            except Exception as e:  # noqa: BLE001
                _log.warning("bad IK profile for %s: %s", base_id, e)
