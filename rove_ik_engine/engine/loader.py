"""Load the robot at engine startup.

Tries the .forgebot archive first (lossless: keeps IK profiles, metadata,
units, etc.). Falls back to URDF + sidecar ik_profile.json if the archive
is missing or fails to load.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path

from forgebot.core.model import IKProfile, Project
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

    # When loading URDF, IK profiles aren't in the file. Look for a sidecar
    # data/ik_profile.json that the exporter writes; merge it in.
    profile_path = urdf_path.parent / "ik_profile.json"
    if profile_path.exists():
        _merge_ik_profiles(result.project, profile_path)

    return result.project


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
