"""Debug geometry in world coordinates; independent of simulated sensor outputs."""

from pathlib import Path
import os
import time
import numpy as np

LAYER_DEFAULTS = dict(scene=True, lidar=False, cameras=False, radiation=False)


def cloud_path(state_file):
    return Path(state_file).with_suffix(".lidar.npz")


def publish_cloud(path, scans):
    """Publish the very scans sent to UDP, without another raycast."""
    path = Path(path)
    points, colours = [], []
    palette = [(0.15, 1, 0.65), (1, 0.4, 0.2), (0.3, 0.65, 1)]
    for i, scan in enumerate(scans):
        xyz = np.asarray(scan.points, dtype="f4")[:, :3]
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        points.append(xyz)
        colours.append(np.tile(palette[i % len(palette)], (len(xyz), 1)))
    vertices = np.concatenate(
        [np.concatenate(points), np.concatenate(colours)], axis=1
    ).astype("f4")
    temp = path.with_name(path.name + f".{os.getpid()}.tmp")
    with temp.open("wb") as f:
        np.savez(f, vertices=vertices, timestamp=time.time())
    os.replace(temp, path)


def read_cloud(path):
    try:
        with np.load(path, allow_pickle=False) as data:
            age = time.time() - float(data["timestamp"])
            vertices = data["vertices"]
            if (
                age < 0
                or age > 1
                or vertices.ndim != 2
                or vertices.shape[1] != 6
                or len(vertices) > 100000
            ):
                return np.empty((0, 6), "f4"), None
            return vertices.astype("f4"), age
    except (OSError, ValueError, KeyError):
        return np.empty((0, 6), "f4"), None


def camera_frustum(view, projection, length=2.0, colour=(0.25, 0.7, 1)):
    """Wireframe near plane and a display plane two metres from the lens.

    This glyph shows the camera's real field of view and pose, not its far clip.
    """
    view = np.asarray(view).reshape(4, 4).T
    projection = np.asarray(projection).reshape(4, 4).T
    inverse_view = np.linalg.inv(view)
    near = projection[2, 3] / (projection[2, 2] - 1)
    corners = []
    for distance in (near, length):
        for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            q = inverse_view @ [
                x * distance / projection[0, 0],
                y * distance / projection[1, 1],
                -distance,
                1,
            ]
            corners.append(q[:3])
    origin = inverse_view[:3, 3]
    edges = (
        [(i, (i + 1) % 4) for i in range(4)]
        + [(i + 4, (i + 1) % 4 + 4) for i in range(4)]
        + [(i, i + 4) for i in range(4)]
    )
    positions = [corners[i] for edge in edges for i in edge]
    positions += [q for corner in corners[4:] for q in (origin, corner)]
    return np.concatenate(
        [positions, np.tile(colour, (len(positions), 1))], axis=1
    ).astype("f4")


def radiation_geometry(sources):
    """Configurable source markers; these do not invent dose/count readings."""
    positions = []
    identifiers = set()
    for source in sources:
        name = str(source["id"])
        centre = np.asarray(source["position"], dtype=float)
        if (
            not name
            or name in identifiers
            or centre.shape != (3,)
            or not np.isfinite(centre).all()
        ):
            raise ValueError(
                "Radiation sources require unique ids and finite XYZ positions"
            )
        identifiers.add(name)
        for axis in range(3):
            delta = np.eye(3)[axis] * 0.2
            positions.extend([centre - delta, centre + delta])
        angles = np.linspace(0, 2 * np.pi, 25)
        ring = (
            centre
            + np.column_stack([np.cos(angles), np.sin(angles), np.zeros(len(angles))])
            * 0.3
        )
        positions.extend(
            [q for i in range(len(ring) - 1) for q in (ring[i], ring[i + 1])]
        )
    if not positions:
        return np.empty((0, 6), "f4")
    return np.concatenate(
        [positions, np.tile((1, 0.75, 0.1), (len(positions), 1))], axis=1
    ).astype("f4")
