"""Base interface every importer implements (Strategy pattern)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path

from ...core.model import Project


@dataclass
class ImportOptions:
    """Per-importer knobs. Importers ignore options they don't understand."""

    mesh_search_paths: list[Path] = field(default_factory=list)
    skip_missing_meshes: bool = False
    extras: dict[str, object] = field(default_factory=dict)


@dataclass
class ImportResult:
    project: Project
    diagnostics: list = field(default_factory=list)


class BaseImporter(ABC):
    @abstractmethod
    def can_import(self, file_path: Path) -> bool: ...

    @abstractmethod
    def import_file(self, file_path: Path, options: ImportOptions | None = None) -> ImportResult: ...

    @abstractmethod
    def supported_extensions(self) -> list[str]: ...
