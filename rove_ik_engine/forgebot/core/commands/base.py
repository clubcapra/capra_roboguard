"""Command pattern: every mutation is a reversible Command object.

The CommandStack provides undo/redo. Commands are constructed up-front
(capturing the *intended* change) and asked to capture state at execute()
time so they can fully reverse themselves. This is "Memento inside Command":
the command stores whatever pre-state it needs in `_undo_state` during
execute(), and undo() reads from it.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from typing import Any

from ..model import Project


class Command(ABC):
    """Reversible mutation. Subclasses must capture undo state during execute()."""

    _undo_state: dict[str, Any] | None = None

    @abstractmethod
    def execute(self, project: Project) -> None: ...

    @abstractmethod
    def undo(self, project: Project) -> None: ...

    @property
    @abstractmethod
    def description(self) -> str: ...


class CommandStack:
    """Undo/redo stack with a bounded history."""

    def __init__(self, project: Project, max_history: int = 200) -> None:
        self._project = project
        self._undo: deque[Command] = deque(maxlen=max_history)
        self._redo: deque[Command] = deque(maxlen=max_history)

    @property
    def project(self) -> Project:
        return self._project

    def execute(self, cmd: Command) -> None:
        cmd.execute(self._project)
        self._undo.append(cmd)
        self._redo.clear()

    def undo(self) -> Command | None:
        if not self._undo:
            return None
        cmd = self._undo.pop()
        cmd.undo(self._project)
        self._redo.append(cmd)
        return cmd

    def redo(self) -> Command | None:
        if not self._redo:
            return None
        cmd = self._redo.pop()
        cmd.execute(self._project)
        self._undo.append(cmd)
        return cmd

    def can_undo(self) -> bool:
        return bool(self._undo)

    def can_redo(self) -> bool:
        return bool(self._redo)

    def history(self) -> list[str]:
        return [c.description for c in self._undo]

    def clear(self) -> None:
        self._undo.clear()
        self._redo.clear()
