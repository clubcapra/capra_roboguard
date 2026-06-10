"""Profile manifest -> composition spec.

A profile ("project format") is one YAML file per hardware setup. It is the
single composition root: it names the model (URDF/SDF), the collision
overrides, which API-seam adapters to speak, and -- as flat import+bind lists
-- which sensors and actuators from the component libraries to instantiate and
which links to bind them to.

Nothing here touches PyBullet; this is pure data. loader.py consumes the model
+ collision spec, the runtime builders consume the actuator/sensor specs, and
capabilities.py derives the capability set from what was declared.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List

import yaml


@dataclass
class ComponentSpec:
    use: str                                   # registry key, e.g. "vn300"
    name: str                                  # instance name, e.g. "vn300_top"
    bind: Any                                   # link name(s) -- str | list | dict
    provides: List[str] = field(default_factory=list)
    params: Dict[str, Any] = field(default_factory=dict)
    transport: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ModelSpec:
    path: str                                  # absolute URDF/SDF path
    kind: str                                  # "urdf" | "sdf"
    mesh_format: str = "glb"                   # source mesh format to convert
    base_position: List[float] = field(default_factory=lambda: [0, 0, 0.5])
    base_orientation: List[float] = field(default_factory=lambda: [0, 0, 0, 1])
    self_collision: bool = False


@dataclass
class ApiSpec:
    sensors: str = "rove_sensor_api"           # sensor-API adapter key
    control: str = "rove_control_bridge"       # control-bridge adapter key
    transport: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Profile:
    name: str
    model: ModelSpec
    api: ApiSpec
    collision_overrides: List[Dict[str, Any]] = field(default_factory=list)
    actuators: List[ComponentSpec] = field(default_factory=list)
    sensors: List[ComponentSpec] = field(default_factory=list)
    raw: Dict[str, Any] = field(default_factory=dict)

    @property
    def provided_capabilities(self) -> set[str]:
        caps: set[str] = set()
        for c in self.sensors:
            caps.update(c.provides)
        for a in self.actuators:
            caps.add(a.use)            # e.g. "arm_ik", "differential_tracks"
            caps.update(a.provides)
        return caps


def _component(entry: Dict[str, Any], idx: int) -> ComponentSpec:
    if "use" not in entry:
        raise ValueError(f"component #{idx} missing 'use': {entry}")
    if "bind" not in entry:
        raise ValueError(f"component {entry['use']!r} missing 'bind'")
    return ComponentSpec(
        use=entry["use"],
        name=entry.get("as", entry["use"]),
        bind=entry["bind"],
        provides=list(entry.get("provides", [])),
        params=dict(entry.get("params", {})),
        transport=dict(entry.get("transport", {})),
    )


def load_profile(path: str) -> Profile:
    path = os.path.abspath(path)
    base = os.path.dirname(path)
    with open(path) as f:
        doc = yaml.safe_load(f)

    m = doc["model"]
    model_path = m["path"]
    if not os.path.isabs(model_path):
        model_path = os.path.normpath(os.path.join(base, model_path))
    kind = m.get("kind") or ("sdf" if model_path.endswith(".sdf") else "urdf")
    model = ModelSpec(
        path=model_path,
        kind=kind,
        mesh_format=m.get("mesh_format", "glb"),
        base_position=list(m.get("base_position", [0, 0, 0.5])),
        base_orientation=list(m.get("base_orientation", [0, 0, 0, 1])),
        self_collision=bool(m.get("self_collision", False)),
    )

    a = doc.get("api", {})
    api = ApiSpec(
        sensors=a.get("sensors", "rove_sensor_api"),
        control=a.get("control", "rove_control_bridge"),
        transport=dict(a.get("transport", {})),
    )

    return Profile(
        name=doc["name"],
        model=model,
        api=api,
        collision_overrides=list(doc.get("collision_overrides", [])),
        actuators=[_component(e, i) for i, e in enumerate(doc.get("actuators", []))],
        sensors=[_component(e, i) for i, e in enumerate(doc.get("sensors", []))],
        raw=doc,
    )
