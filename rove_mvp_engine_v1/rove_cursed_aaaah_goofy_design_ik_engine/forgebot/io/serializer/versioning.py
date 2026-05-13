"""Schema migration between .forgebot versions.

For phase 1 there's only one version, so this is a stub. The intent: each
new release bumps `CURRENT_VERSION` and adds a migration function from
the previous one. `migrate_manifest()` is the entry point.
"""

from __future__ import annotations

from typing import Any

CURRENT_VERSION = "1.0.0"


_MIGRATIONS: dict[str, Any] = {}  # "from_version" -> callable(dict) -> dict


def migrate_manifest(data: dict[str, Any]) -> dict[str, Any]:
    """Walk the migration chain until the manifest is at CURRENT_VERSION."""
    seen: set[str] = set()
    while True:
        v = data.get("forgebot_version", "0.0.0")
        if v == CURRENT_VERSION:
            return data
        if v in seen:
            raise RuntimeError(f"migration loop detected at version {v}")
        seen.add(v)
        step = _MIGRATIONS.get(v)
        if step is None:
            raise RuntimeError(
                f"no migration registered from version {v} to {CURRENT_VERSION}"
            )
        data = step(data)
