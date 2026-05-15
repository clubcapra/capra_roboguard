"""Asset manager: pack/unpack mesh and texture binaries inside the .forgebot ZIP.

Meshes and textures are stored as raw bytes in `assets/meshes/` and
`assets/textures/`. The `Assets` model holds maps from a stem name (e.g.
`"base_visual"`) to the archive path (`"assets/meshes/base_visual.stl"`).
"""

from __future__ import annotations

from pathlib import Path, PurePosixPath
from zipfile import ZipFile

MESH_DIR = "assets/meshes"
TEXTURE_DIR = "assets/textures"
SUPPORTED_MESH_EXT = {".stl", ".obj", ".glb", ".gltf", ".dae", ".ply"}
SUPPORTED_TEX_EXT = {".png", ".jpg", ".jpeg", ".tga", ".bmp", ".exr"}


def stem_from_archive_path(archive_path: str) -> str:
    return PurePosixPath(archive_path).stem


def add_mesh_from_file(zf: ZipFile, source: Path, stem: str | None = None) -> str:
    """Add a mesh file to the archive. Returns the archive path it was written to."""
    s = stem or source.stem
    target = f"{MESH_DIR}/{s}{source.suffix.lower()}"
    zf.write(source, target)
    return target


def add_mesh_bytes(zf: ZipFile, data: bytes, stem: str, suffix: str) -> str:
    sfx = suffix if suffix.startswith(".") else f".{suffix}"
    target = f"{MESH_DIR}/{stem}{sfx.lower()}"
    zf.writestr(target, data)
    return target


def add_texture_from_file(zf: ZipFile, source: Path, stem: str | None = None) -> str:
    s = stem or source.stem
    target = f"{TEXTURE_DIR}/{s}{source.suffix.lower()}"
    zf.write(source, target)
    return target


def add_texture_bytes(zf: ZipFile, data: bytes, stem: str, suffix: str) -> str:
    sfx = suffix if suffix.startswith(".") else f".{suffix}"
    target = f"{TEXTURE_DIR}/{stem}{sfx.lower()}"
    zf.writestr(target, data)
    return target


def list_meshes(zf: ZipFile) -> dict[str, str]:
    """Return {stem: archive_path} for every mesh in the archive."""
    out: dict[str, str] = {}
    for name in zf.namelist():
        if name.startswith(f"{MESH_DIR}/") and not name.endswith("/"):
            out[stem_from_archive_path(name)] = name
    return out


def list_textures(zf: ZipFile) -> dict[str, str]:
    out: dict[str, str] = {}
    for name in zf.namelist():
        if name.startswith(f"{TEXTURE_DIR}/") and not name.endswith("/"):
            out[stem_from_archive_path(name)] = name
    return out


def read_asset(zf: ZipFile, archive_path: str) -> bytes:
    return zf.read(archive_path)


def extract_asset(zf: ZipFile, archive_path: str, dest_dir: Path) -> Path:
    """Extract one asset to `dest_dir`, preserving its filename. Returns the path."""
    dest_dir.mkdir(parents=True, exist_ok=True)
    name = PurePosixPath(archive_path).name
    out = dest_dir / name
    out.write_bytes(zf.read(archive_path))
    return out
