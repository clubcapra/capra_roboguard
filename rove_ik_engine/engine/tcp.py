"""Per-link mesh-centroid TCP offsets in each link's local frame.

The bundled UI's gizmo computes its own TCP offset from the rendered mesh
(via Three.js Box3) and sends it on every Ovis. External clients (the
control bridge, joysticks, ROS bridges) don't have the mesh, so they leave
the field zero. When that happens the engine falls back to the offsets
computed here at startup — same centroid, same rotation-around-the-gripper
behaviour, no client-side geometry required.

If trimesh isn't installed, the lookup table stays empty and clients without
their own offset will pivot at the link origin (the legacy behaviour).
"""

from __future__ import annotations

import logging
from io import BytesIO
from typing import cast

import numpy as np

from forgebot.core.model import LinkComponent, Project

_log = logging.getLogger(__name__)


def compute_tcp_offsets(project: Project) -> dict[str, np.ndarray]:
    """Return ``{link_entity_id: centroid_in_link_local_frame}`` for every
    link that has visual meshes the loader can parse. Links without meshes
    (e.g. joints, abstract links, links with only primitives) are absent."""
    try:
        import trimesh
    except ImportError:
        _log.warning(
            "trimesh not installed; Ovis without `tcp_offset_local` will "
            "pivot at the link origin. `pip install trimesh` to enable "
            "auto-centroid pivots."
        )
        return {}

    offsets: dict[str, np.ndarray] = {}
    for eid, ent in project.scene.entities.items():
        link = cast("LinkComponent | None", ent.get("link"))
        if link is None or not link.visuals:
            continue

        bbox_min: np.ndarray | None = None
        bbox_max: np.ndarray | None = None

        for visual in link.visuals:
            if not visual.mesh:
                # primitives have known centroids but we'd need to dispatch
                # per shape; skip for now (most users export meshes).
                continue
            asset = project.assets.mesh_data.get(visual.mesh)
            if asset is None:
                continue

            try:
                loaded = trimesh.load(
                    BytesIO(asset.data),
                    file_type=asset.suffix.lstrip("."),
                    force="mesh",
                )
            except Exception as exc:  # noqa: BLE001
                _log.debug(
                    "could not parse mesh %r for %s: %s",
                    visual.mesh,
                    eid,
                    exc,
                )
                continue
            if loaded is None or not hasattr(loaded, "vertices"):
                continue

            # Transform vertices into the link's local frame:
            #   scale -> rotate (visual.origin_rotation) -> translate (visual.origin)
            verts = np.asarray(loaded.vertices, dtype=float) * np.asarray(
                visual.scale, dtype=float
            )
            R = _quat_to_rotmat(visual.origin_rotation)
            verts = verts @ R.T + np.asarray(visual.origin, dtype=float)

            v_min = verts.min(axis=0)
            v_max = verts.max(axis=0)
            bbox_min = v_min if bbox_min is None else np.minimum(bbox_min, v_min)
            bbox_max = v_max if bbox_max is None else np.maximum(bbox_max, v_max)

        if bbox_min is not None and bbox_max is not None:
            offsets[eid] = (bbox_min + bbox_max) * 0.5

    return offsets


def _quat_to_rotmat(q: tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
