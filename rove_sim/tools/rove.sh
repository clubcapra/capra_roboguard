#!/usr/bin/env bash
# Launch the simulator and optional HALo adapter; stop only our own children.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
HALO_DIR="$(cd "$HERE/../HALo" && pwd)"
BIN="$HALO_DIR/target/debug/halo"
MODE=${1:-}; shift || true
[[ "$MODE" = headless || "$MODE" = gui ]] || {
  echo 'usage: tools/rove.sh {headless|gui} [--api] [--profile P] [--world-config W] [--scene S] [--no-terrain]'
  exit 1
}
PROFILE=${PROFILE:-standard}; SCENE=${SCENE:-}
WORLD_CONFIG=${WORLD_CONFIG:-config/world.yaml}
API=0; TERRAIN=1; PASS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --api) API=1 ;;
    --tiny) : ;; # Compatibility alias: the default fleet already shares one GPU scene.
    --profile) PROFILE="$2"; shift ;;
    --scene) SCENE="$2"; shift ;;
    --world-config) WORLD_CONFIG="$2"; shift ;;
    --no-terrain) TERRAIN=0 ;;
    *) PASS+=("$1") ;;
  esac
  shift
done
cd "$HERE"
API_PID=""; SIM_PID=""
cleanup() {
  trap - EXIT INT TERM
  [[ -z "$SIM_PID" ]] || kill "$SIM_PID" 2>/dev/null || true
  [[ -z "$API_PID" ]] || kill "$API_PID" 2>/dev/null || true
  [[ -z "$SIM_PID" ]] || wait "$SIM_PID" 2>/dev/null || true
  [[ -z "$API_PID" ]] || wait "$API_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
if [[ "$API" = 1 ]]; then
  [[ -x "$BIN" ]] || cargo build --manifest-path "$HALO_DIR/Cargo.toml"
  (cd "$HALO_DIR" && HALO_SIM_BACKEND=127.0.0.1:5000 RUST_LOG=info "$BIN") &
  API_PID=$!
fi
export PROFILE SCENE WORLD_CONFIG
if [[ "$MODE" = headless && "$TERRAIN" = 1 ]]; then
  [[ ${#PASS[@]} = 0 ]] || { echo 'Use fleet environment variables for camera/ray budgets (see README).'; exit 1; }
  RSA_BACKEND="$API" tools/sim_fleet.sh &
  SIM_PID=$!
elif [[ "$MODE" = headless ]]; then
  BACKEND=(); [[ "$API" = 1 ]] && BACKEND=(--rsa-backend)
  "${SIM_PYTHON:-python3}" tools/sim_server.py --profile "$PROFILE" "${BACKEND[@]}" "${PASS[@]}" &
  SIM_PID=$!
else
  ARGS=(--profile "$PROFILE")
  [[ "$TERRAIN" = 0 ]] || ARGS+=(--world-config "$WORLD_CONFIG")
  [[ -z "$SCENE" ]] || ARGS+=(--scene "$SCENE")
  tools/gui_fleet.sh "${ARGS[@]}" "${PASS[@]}" &
  SIM_PID=$!
fi
CHILDREN=("$SIM_PID"); [[ -z "$API_PID" ]] || CHILDREN+=("$API_PID")
wait -n "${CHILDREN[@]}"
