#!/usr/bin/env python3
"""Run the exported ForgeBOT IK engine.

On first invocation, creates a local `.venv/` and installs `requirements.txt`
into it, then re-execs itself under the venv's python. Subsequent runs reuse
the existing venv.

    python3 run.py                     # uses ./engine.toml
    python3 run.py path/to/cfg.toml    # custom config
    FORGEBOT_NO_BOOTSTRAP=1 python3 run.py    # skip env setup (you manage deps yourself)
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
VENV_DIR = HERE / ".venv"
VENV_PY = VENV_DIR / "bin" / "python"
REQS = HERE / "requirements.txt"


def _have_deps() -> bool:
    try:
        import numpy  # noqa: F401
        import aiohttp  # noqa: F401
        from google import protobuf  # noqa: F401
    except ImportError:
        return False
    return True


def _bootstrap() -> None:
    """Make sure required deps are importable. Creates the venv if missing,
    installs requirements, and re-execs under the venv's python."""
    if os.environ.get("FORGEBOT_NO_BOOTSTRAP"):
        return
    if _have_deps():
        return

    running_in_venv = (
        VENV_PY.exists()
        and Path(sys.executable).resolve() == VENV_PY.resolve()
    )

    if not VENV_DIR.exists():
        print(f"[bootstrap] creating venv at {VENV_DIR}")
        try:
            import venv

            venv.create(VENV_DIR, with_pip=True)
        except Exception as exc:
            print(
                f"[bootstrap] could not create venv: {exc}\n"
                "  On Debian/Ubuntu/Raspberry Pi OS: apt install python3-venv\n"
                "  Or set FORGEBOT_NO_BOOTSTRAP=1 and install requirements.txt yourself.",
                file=sys.stderr,
            )
            sys.exit(2)

    if not VENV_PY.exists():
        print(
            f"[bootstrap] venv created but {VENV_PY} not found "
            "(unusual python layout — set FORGEBOT_NO_BOOTSTRAP=1 to bypass)",
            file=sys.stderr,
        )
        sys.exit(2)

    # When we're already in the venv and imports failed, requirements need
    # installing. When we're not in the venv yet, re-exec first so pip runs
    # under the right interpreter.
    if running_in_venv:
        print(f"[bootstrap] installing {REQS}")
        subprocess.check_call(
            [str(VENV_PY), "-m", "pip", "install", "--upgrade", "pip"]
        )
        subprocess.check_call(
            [str(VENV_PY), "-m", "pip", "install", "-r", str(REQS)]
        )

    print(f"[bootstrap] re-executing under {VENV_PY}")
    os.execv(
        str(VENV_PY),
        [str(VENV_PY), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def main() -> int:
    _bootstrap()

    import asyncio
    import logging

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # Make the vendored `forgebot/` importable.
    sys.path.insert(0, str(HERE))

    from engine.server import run  # noqa: E402  (after sys.path tweak)

    if len(sys.argv) > 1:
        config_path = Path(sys.argv[1]).resolve()
    else:
        config_path = HERE / "engine.toml"

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
