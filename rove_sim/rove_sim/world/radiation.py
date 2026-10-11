"""Deterministic radiation source blocks shared by physics and render workers."""

from pathlib import Path
import json
import numpy as np
from .scene import SceneObject
from .mesh_assets import load_mesh, asset_path


def ground_heights(world, xy):
    """Place sources on opaque ground geometry, independent of worker purpose."""
    heights = np.full(len(xy), -np.inf)
    if not world.terrain_manifest:
        return np.zeros(len(xy))
    manifest = world.terrain_manifest
    spec = json.loads(Path(manifest).read_text())
    scale = float(world.spec["terrain"].get("scale", 1))
    for entry in spec["visual"]:
        if entry.get("category") != "ground" or entry.get("alpha_mode") != "OPAQUE":
            continue
        mesh = load_mesh(asset_path(manifest, entry["obj"]))
        triangles = mesh["vertices"][mesh["faces"]] * scale
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        ab, ac = b - a, c - a
        denominator = ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0]
        good = np.abs(denominator) > 1e-8
        a, ab, ac, denominator = a[good], ab[good], ac[good], denominator[good]
        for i, position in enumerate(xy):
            delta = position - a[:, :2]
            u = (delta[:, 0] * ac[:, 1] - delta[:, 1] * ac[:, 0]) / denominator
            v = (ab[:, 0] * delta[:, 1] - ab[:, 1] * delta[:, 0]) / denominator
            inside = (u >= 0) & (v >= 0) & (u + v <= 1)
            if inside.any():
                z = a[:, 2] + u * ab[:, 2] + v * ac[:, 2]
                heights[i] = max(heights[i], float(z[inside].max()))
    return heights


def configure_radiation(sim):
    config = sim.world.spec.get("radiation", {})
    sources = list(sim.world.spec.get("radiation_sources", []))
    count = int(config.get("random_count", 0))
    radius = np.asarray(config.get("radius_m", [2, 10]), float)
    strength = np.asarray(config.get("dose_rate_at_1m_usv_h", [1, 8]), float)
    if (
        not 0 <= count <= 100
        or radius.shape != (2,)
        or strength.shape != (2,)
        or not np.isfinite(np.r_[radius, strength]).all()
        or radius[0] < 0
        or radius[1] <= radius[0]
        or strength[0] < 0
        or strength[1] < strength[0]
    ):
        raise ValueError("Invalid random radiation source configuration")
    if count:
        rng = np.random.default_rng(int(config.get("seed", 26)))
        angle = rng.uniform(0, 2 * np.pi, count * 4)
        distance = np.sqrt(rng.uniform(radius[0] ** 2, radius[1] ** 2, count * 4))
        xy = np.column_stack([np.cos(angle), np.sin(angle)]) * distance[:, None]
        xy += np.asarray(config.get("centre", [0, 0]), float)
        heights = ground_heights(sim.world, xy)
        accepted = np.flatnonzero(np.isfinite(heights))[:count]
        if len(accepted) != count:
            raise ValueError("Cannot place all radiation blocks on configured ground")
        for number, i in enumerate(accepted):
            sources.append(
                dict(
                    id=f"radiation_{number+1}",
                    position=[*xy[i].tolist(), float(heights[i]) + 0.09],
                    dose_rate_at_1m_usv_h=float(rng.uniform(*strength)),
                )
            )
    from ..sensors.radiation import dose_at, RadiationSensor

    dose_at([0, 0, 0], sources)
    ids = [str(source["id"]) for source in sources]
    if len(set(ids)) != len(ids) or any(not name for name in ids):
        raise ValueError("Radiation source ids must be unique and nonempty")
    for source in sources:
        identifier = "radiation_source_" + str(source["id"])
        if identifier in sim.world.objects:
            raise ValueError(
                f"Radiation source id collides with scene object: {identifier}"
            )
        sim.world.spawn_object(
            SceneObject(
                id=identifier,
                pose=tuple(source["position"]),
                shape="box",
                extents=(0.18, 0.18, 0.18),
                rgba=(1, 0.65, 0.05, 1),
                mass=0,
                cls="radiation_source",
            )
        )
    sim.robot.radiation_sources = sources
    sim.world.spec["radiation_sources"] = sources
    for sensor in sim.sensors:
        if isinstance(sensor, RadiationSensor):
            sensor.sources = sources
