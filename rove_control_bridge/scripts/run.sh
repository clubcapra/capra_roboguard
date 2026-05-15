#!/usr/bin/env bash
# Jetson launcher for rove_control_bridge.
#
# Usage:
#   ./scripts/run.sh                               # uses config/default.yaml
#   ./scripts/run.sh --tracks-strategy torque      # override strategy at runtime
#   TRACKS_STRATEGY=torque ./scripts/run.sh        # override via env var
#
# Any extra args are forwarded verbatim to the Python module.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"             # rove_control_bridge/
PYTHONPATH_ROOT="$(dirname "$PKG_ROOT")"        # parent of rove_control_bridge/ (for -m import)
VENV_DIR="$PKG_ROOT/.venv"
REQ_FILE="$PKG_ROOT/requirements.txt"
STAMP_FILE="$VENV_DIR/.requirements.stamp"
CONFIG_FILE="${CONFIG:-$PKG_ROOT/config/default.yaml}"

# ---------- python3 sanity check ----------
if ! command -v python3 &>/dev/null; then
    echo "[bootstrap] python3 not found — installing"
    sudo apt-get install -y python3
fi

# ---------- venv bootstrap (auto-installs python3-venv if missing) ----------
if [[ ! -d "$VENV_DIR" ]]; then
    echo "[bootstrap] creating venv at $VENV_DIR"
    if ! python3 -m venv "$VENV_DIR" 2>/dev/null; then
        PY_VER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
        echo "[bootstrap] venv unavailable — installing python${PY_VER}-venv"
        sudo apt-get install -y "python${PY_VER}-venv"
        python3 -m venv "$VENV_DIR"
    fi
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

if [[ ! -f "$STAMP_FILE" ]] || [[ "$REQ_FILE" -nt "$STAMP_FILE" ]]; then
    echo "[bootstrap] installing requirements"
    pip install --quiet --upgrade pip
    pip install --quiet -r "$REQ_FILE"
    touch "$STAMP_FILE"
fi

# Recompile protobuf modules when any .proto file is newer than the stamp.
# Also fires after a fresh requirements install (protobuf runtime may change).
PROTO_STAMP="$PKG_ROOT/proto/core/.proto.stamp"
if [[ ! -f "$PROTO_STAMP" ]] || [[ "$STAMP_FILE" -nt "$PROTO_STAMP" ]] || \
   find "$PKG_ROOT/proto" -name "*.proto" -newer "$PROTO_STAMP" | grep -q .; then
    echo "[bootstrap] compiling protobuf definitions"
    python3 "$SCRIPT_DIR/build_protos.py"
    touch "$PROTO_STAMP"
fi

# ---------- kill any stale bridge instance ----------
pkill -f "rove_control_bridge" 2>/dev/null && sleep 0.3 || true

# ---------- strategy override via env ----------
EXTRA_ARGS=()
if [[ -n "${TRACKS_STRATEGY:-}" ]]; then
    EXTRA_ARGS+=("--tracks-strategy" "$TRACKS_STRATEGY")
fi

cd "$PYTHONPATH_ROOT"
exec python3 -m rove_control_bridge \
    --config "$CONFIG_FILE" \
    "${EXTRA_ARGS[@]}" \
    "$@"
