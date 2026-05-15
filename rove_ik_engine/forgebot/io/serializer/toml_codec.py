"""TOML <-> Project conversion.

Read uses stdlib `tomllib` (read-only, Python 3.11+).
Write uses `tomli_w`, with optional `tomlkit` for comment-preserving round-trips
when the caller has the original source text.
"""

from __future__ import annotations

import tomllib
from datetime import datetime
from typing import Any

import tomli_w

from ...core.model import (
    Assets,
    KinematicChainSpec,
    Manifest,
    Material,
    Metadata,
    PhysicsConfig,
    Project,
    Scene,
    Simulation,
    Storage,
    Systems,
    Units,
)


def loads(text: str) -> dict[str, Any]:
    return tomllib.loads(text)


def loads_bytes(data: bytes) -> dict[str, Any]:
    return tomllib.loads(data.decode("utf-8"))


def dumps(data: dict[str, Any]) -> str:
    """Serialize a dict to TOML, scrubbing None and converting tuples to lists.

    `tomli_w` rejects None and tuples; Pydantic happily produces both.
    """
    return tomli_w.dumps(_scrub(data))


def _scrub(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: _scrub(v) for k, v in value.items() if v is not None}
    if isinstance(value, (list, tuple)):
        return [_scrub(v) for v in value if v is not None]
    return value


# ---------- manifest ----------


def manifest_to_dict(m: Manifest) -> dict[str, Any]:
    out: dict[str, Any] = {
        "forgebot_version": m.forgebot_version,
        "metadata": {
            "name": m.metadata.name,
            "author": m.metadata.author,
            "description": m.metadata.description,
            "created": m.metadata.created,
            "modified": m.metadata.modified,
            "tags": list(m.metadata.tags),
        },
        "units": {
            "length": m.units.length,
            "angle": m.units.angle,
            "mass": m.units.mass,
        },
        "storage": {"scene_format": m.storage.scene_format},
    }
    return out


def manifest_from_dict(data: dict[str, Any]) -> Manifest:
    md_raw = data.get("metadata", {}) or {}
    units_raw = data.get("units", {}) or {}
    storage_raw = data.get("storage", {}) or {}
    return Manifest(
        forgebot_version=data.get("forgebot_version", "1.0.0"),
        metadata=Metadata(
            name=md_raw.get("name", "Untitled"),
            author=md_raw.get("author", ""),
            description=md_raw.get("description", ""),
            created=_coerce_dt(md_raw.get("created")),
            modified=_coerce_dt(md_raw.get("modified")),
            tags=list(md_raw.get("tags", [])),
        ),
        units=Units(
            length=units_raw.get("length", "meters"),
            angle=units_raw.get("angle", "radians"),
            mass=units_raw.get("mass", "kilograms"),
        ),
        storage=Storage(scene_format=storage_raw.get("scene_format", "toml")),
    )


def _coerce_dt(v: Any) -> datetime:
    if isinstance(v, datetime):
        return v
    if isinstance(v, str):
        return datetime.fromisoformat(v.replace("Z", "+00:00"))
    return datetime.now().astimezone()


# ---------- systems ----------


def systems_to_dict(s: Systems) -> dict[str, Any]:
    return {
        "kinematic_chains": {
            k: {"base": c.base, "tip": c.tip, "joints": list(c.joints)}
            for k, c in s.kinematic_chains.items()
        },
        "signal_graph": dict(s.signal_graph),
        "layout": dict(s.layout),
    }


def systems_from_dict(data: dict[str, Any]) -> Systems:
    chains_raw: dict[str, Any] = data.get("kinematic_chains", {}) or {}
    chains = {
        k: KinematicChainSpec(base=v["base"], tip=v["tip"], joints=list(v.get("joints", [])))
        for k, v in chains_raw.items()
    }
    return Systems(
        kinematic_chains=chains,
        signal_graph=data.get("signal_graph", {}) or {},
        layout=data.get("layout", {}) or {},
    )


# ---------- simulation ----------


def simulation_to_dict(s: Simulation) -> dict[str, Any]:
    return {
        "physics": {
            "engine": s.physics.engine,
            "gravity": list(s.physics.gravity),
            "timestep": s.physics.timestep,
        },
        "controllers": dict(s.controllers),
    }


def simulation_from_dict(data: dict[str, Any]) -> Simulation:
    phys_raw = data.get("physics", {}) or {}
    return Simulation(
        physics=PhysicsConfig(
            engine=phys_raw.get("engine", "default"),
            gravity=tuple(phys_raw.get("gravity", (0.0, 0.0, -9.81))),  # type: ignore[arg-type]
            timestep=phys_raw.get("timestep", 0.001),
        ),
        controllers=data.get("controllers", {}) or {},
    )


# ---------- assets (materials only — meshes/textures live as binary in ZIP) ----------


def materials_to_dict(assets: Assets) -> dict[str, Any]:
    return {
        name: {
            "color": list(m.color),
            "metallic": m.metallic,
            "roughness": m.roughness,
            **({"texture": m.texture} if m.texture else {}),
        }
        for name, m in assets.materials.items()
    }


def materials_from_dict(data: dict[str, Any]) -> dict[str, Material]:
    out: dict[str, Material] = {}
    for name, raw in data.items():
        out[name] = Material(
            color=tuple(raw.get("color", (0.8, 0.8, 0.8, 1.0))),  # type: ignore[arg-type]
            metallic=raw.get("metallic", 0.0),
            roughness=raw.get("roughness", 0.5),
            texture=raw.get("texture"),
        )
    return out


# ---------- scene ----------


def scene_to_dict(scene: Scene) -> dict[str, Any]:
    return scene.to_toml_dict()


def scene_from_dict(data: dict[str, Any]) -> Scene:
    return Scene.from_toml_dict(data)
