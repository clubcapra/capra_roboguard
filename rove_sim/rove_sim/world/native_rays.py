"""Small native Embree coverage filter, compiled locally and cached by source hash.

No downloads occur at runtime. Embreex supplies the Embree 4 shared library;
the matching public headers are vendored with their Apache-2.0 license.
"""

import ctypes
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import weakref
import numpy as np


def library():
    import embreex
    import importlib.metadata

    version = importlib.metadata.version("embreex")
    if not version.startswith("4.4."):
        raise RuntimeError(f"Native ray filter needs embreex 4.4.x (found {version})")
    root = Path(__file__).parent / "native"
    binary_root = Path(embreex.__file__).parent.parent / "embreex.libs"
    binaries = list(binary_root.glob("libembree4*.so*"))
    if len(binaries) != 1:
        raise RuntimeError("Could not locate embreex Embree 4 shared library")
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        if path.is_file():
            digest.update(path.read_bytes())
    digest.update(str(binaries[0]).encode())
    cache = (
        Path(os.environ.get("XDG_CACHE_HOME", tempfile.gettempdir()))
        / "rove_sim_native"
    )
    cache.mkdir(parents=True, exist_ok=True)
    target = cache / (digest.hexdigest()[:20] + ".so")
    if not target.is_file():
        temp = target.with_name(target.name + f".{os.getpid()}.tmp")
        try:
            subprocess.run(
                [
                    "g++",
                    "-O3",
                    "-std=c++17",
                    "-shared",
                    "-fPIC",
                    str(root / "alpha_rays.cpp"),
                    "-I",
                    str(root),
                    str(binaries[0]),
                    "-Wl,-rpath," + str(binary_root),
                    "-o",
                    str(temp),
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            os.replace(temp, target)
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(
                "Native ray filter compilation failed: " + exc.stderr
            ) from exc
        finally:
            temp.unlink(missing_ok=True)
    lib = ctypes.CDLL(str(target))
    pointer = ctypes.c_void_p
    lib.terrain_create.argtypes = [pointer, ctypes.c_size_t] + [pointer] * 7
    lib.terrain_create.restype = pointer
    lib.terrain_trace.argtypes = [
        pointer,
        pointer,
        pointer,
        ctypes.c_size_t,
        ctypes.c_float,
        pointer,
        pointer,
    ]
    lib.terrain_destroy.argtypes = [pointer]
    lib.terrain_destroy.restype = None
    lib.terrain_trace.restype = None
    return lib


class NativeTerrain:
    def __init__(
        self, triangles, uv, material, alpha, offsets, widths, heights, cutoffs
    ):
        self.lib = library()
        # The native filter borrows these immutable buffers for its lifetime.
        self.buffers = [
            np.ascontiguousarray(a)
            for a in (uv, material, alpha, offsets, widths, heights, cutoffs)
        ]
        triangles = np.ascontiguousarray(triangles, np.float32)
        self.handle = self.lib.terrain_create(
            triangles.ctypes.data,
            len(triangles),
            *[a.ctypes.data for a in self.buffers],
        )
        if not self.handle:
            raise RuntimeError("Embree could not build the alpha-filtered scene")
        self._finalizer = weakref.finalize(self, self.lib.terrain_destroy, self.handle)

    def cast(self, origins, directions, max_range):
        origins = np.ascontiguousarray(origins, np.float32)
        directions = np.ascontiguousarray(directions, np.float32)
        distance = np.empty(len(origins), np.float32)
        face = np.empty(len(origins), np.int32)
        self.lib.terrain_trace(
            self.handle,
            origins.ctypes.data,
            directions.ctypes.data,
            len(origins),
            max_range,
            distance.ctypes.data,
            face.ctypes.data,
        )
        points = np.zeros_like(origins)
        valid = face >= 0
        points[valid] = origins[valid] + directions[valid] * distance[valid, None]
        return points, distance, face
