#!/usr/bin/env bash
# Launch the fast native-window sim on the HOST (outside the flatpak/IDE sandbox)
# for true 60 fps. PyBullet GUI mode renders straight to an XWayland window with
# no GPU readback -- smooth even with terrain.
#
# Why this wrapper: rove_sim_venv is a python3.13 venv (its numpy/pybullet are
# cp313 binaries). The host's default `python3` is 3.14, which CANNOT load cp313
# wheels -> "ModuleNotFoundError: No module named 'numpy'". Install the matching
# interpreter once (it's the exact version the venv was built for):
#
#     sudo dnf install python3.13
#
# then run this from a host GNOME terminal:
#
#     tools/live_host.sh --profile standard --terrain
#
# It runs the host python3.13 against the venv's site-packages, leaving the venv
# untouched (so it still works inside the sandbox for tests).
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"                       # .../rove_sim
export PATH="$HERE/tools/bin:$PATH"   # bundled mediamtx/ffmpeg first on PATH
VENV_SP="$HERE/../rove_sim_venv/lib/python3.13/site-packages"
PY=/usr/bin/python3.13
if [ ! -x "$PY" ]; then
  echo "python3.13 not found on the host. Install the interpreter the venv was"
  echo "built for (no recompile needed):"
  echo "    sudo dnf install python3.13"
  exit 1
fi
cd "$HERE"
echo "DISPLAY=$DISPLAY   $("$PY" --version)   (native window, no readback)"
exec env PYTHONPATH="$HERE:$VENV_SP" "$PY" tools/live.py "$@"
