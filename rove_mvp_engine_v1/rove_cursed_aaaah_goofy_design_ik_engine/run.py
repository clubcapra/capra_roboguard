#!/usr/bin/env python
"""Run the exported ForgeBOT IK engine.

Usage:
    python run.py                         # uses ./engine.toml
    python run.py path/to/engine.toml     # custom config
"""

from __future__ import annotations

import asyncio
import logging
import sys
from pathlib import Path


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    here = Path(__file__).resolve().parent
    # Make the vendored `forgebot/` importable.
    sys.path.insert(0, str(here))

    from engine.server import run  # noqa: E402  (after sys.path tweak)

    if len(sys.argv) > 1:
        config_path = Path(sys.argv[1]).resolve()
    else:
        config_path = here / "engine.toml"

    if not config_path.exists():
        print(f"config not found: {config_path}", file=sys.stderr)
        return 1

    try:
        asyncio.run(run(config_path))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
