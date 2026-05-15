"""Importer registry. New format = new module + entry in `IMPORTERS`."""

from __future__ import annotations

from pathlib import Path

from .base import BaseImporter, ImportOptions, ImportResult
from .mjcf_importer import MJCFImporter
from .sdf_importer import SDFImporter
from .urdf_importer import URDFImporter

# Order matters: each importer's can_import() inspects the file content
# (root tag), so URDF/MJCF/SDF disambiguate themselves regardless of extension.
IMPORTERS: list[BaseImporter] = [URDFImporter(), MJCFImporter(), SDFImporter()]


def find_importer(file_path: Path) -> BaseImporter | None:
    for imp in IMPORTERS:
        if imp.can_import(file_path):
            return imp
    return None


def import_file(file_path: Path, options: ImportOptions | None = None) -> ImportResult:
    imp = find_importer(file_path)
    if imp is None:
        raise ValueError(f"no importer for {file_path}")
    return imp.import_file(file_path, options)


__all__ = [
    "BaseImporter",
    "IMPORTERS",
    "ImportOptions",
    "ImportResult",
    "MJCFImporter",
    "SDFImporter",
    "URDFImporter",
    "find_importer",
    "import_file",
]
