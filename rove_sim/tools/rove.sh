#!/usr/bin/env bash
# rove.sh — one entry point to launch the sim, with or without the real
# rove_sensor_api binary on top. Run this on the HOST desktop (it needs the GPU +
# python3.13; see sim_host.sh / live_host.sh for the why).
#
#   tools/rove.sh headless                 # headless sim + cameras (RTSP) + lidar
#   tools/rove.sh headless --api           # ^ + the REAL rove_sensor_api, sim-backed
#   tools/rove.sh gui                       # drivable GUI window + parallel cam/lidar
#   tools/rove.sh gui --api                 # ^ + the REAL rove_sensor_api, sim-backed
#
#   options:  --profile standard|caged   --scene mission.json   --no-terrain
#
# With --api the genuine rove_sensor_api Rust binary runs in sim-backend mode
# (ROVE_SIM_BACKEND): it serves HTTP :8080 + UDP 5000+ to autonomy, pulling live
# values from the sim. Telemetry then looks byte-identical to the real robot, so
# the autonomy / IK stack connects unchanged.
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
RSA_DIR="$(cd "$HERE/../rove_sensor_api" && pwd)"
BIN="$RSA_DIR/target/debug/capra-rove-interface"

MODE="${1:-}"; shift || true
[ "$MODE" = headless ] || [ "$MODE" = gui ] || {
  echo "usage: tools/rove.sh {headless|gui} [--api] [--tiny] [--profile P] [--scene S] [--no-terrain]"
  echo "  --tiny  low-spec mode for a 2GB GPU / weak CPU (640p cams in 1 render proc,"
  echo "          128px tex, no shadows, CPU-only physics, reduced lidar)"; exit 1; }

PROFILE=standard; SCENE=""; API=0; TERRAIN=1; TINY=0; PASS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --api) API=1 ;;
    --tiny) TINY=1 ;;
    --profile) PROFILE="$2"; shift ;;
    --scene) SCENE="$2"; shift ;;
    --no-terrain) TERRAIN=0 ;;
    *) PASS+=("$1") ;;
  esac
  shift
done

# Kill any stale fleet/API processes (orphans from a previous run that was -9'd
# before its trap could fire). Python workers also self-die when orphaned (see
# core/util.die_with_parent), so this mainly mops up mediamtx + the binary.
kill_fleet() {
  set +e
  pkill -9 -f "capra-rove-interface"     2>/dev/null
  pkill -9 -f "tools/sim_server.py"      2>/dev/null
  pkill -9 -f "tools/lidar_worker.py"    2>/dev/null
  pkill -9 -f "tools/pyrender_worker.py" 2>/dev/null
  pkill -9 -f "tools/cam_worker.py"      2>/dev/null
  pkill -9 -f "rtsp://127.0.0.1"         2>/dev/null   # orphaned ffmpeg pushers
  pkill -9 -x mediamtx                   2>/dev/null
  rm -f /dev/shm/rove_robot_state.json
}

API_PID=""
cleanup() { set +e; [ -n "$API_PID" ] && kill "$API_PID" 2>/dev/null; kill_fleet; }
trap cleanup EXIT INT TERM

echo "[rove] pre-cleaning any stale processes…"
kill_fleet
sleep 1

# --- optionally bring up the real rove_sensor_api, sim-backed --------------
if [ "$API" = 1 ]; then
  if [ ! -x "$BIN" ]; then
    echo "[rove] building rove_sensor_api (first run)…"
    cargo build --manifest-path "$RSA_DIR/Cargo.toml" \
      || { echo "[rove] cargo build failed — install rust or build manually"; exit 1; }
  fi
  echo "[rove] starting rove_sensor_api in SIM-BACKEND mode (HTTP :8080, UDP 5000+)"
  ( cd "$RSA_DIR" && ROVE_SIM_BACKEND=127.0.0.1:5000 RUST_LOG=info "$BIN" ) &
  API_PID=$!
fi

# --- launch the sim --------------------------------------------------------
# NOTE: not `exec` — we keep the shell alive so the cleanup trap can reap the
# rove_sensor_api child when the sim exits / the window closes.
if [ "$MODE" = headless ]; then
  [ -n "$SCENE" ] && echo "[rove] note: --scene is GUI-only (live.py); headless ignores it"
  if [ "$TERRAIN" = 0 ]; then
    # bare mode (no terrain): the simple inline-camera path.
    RSA=(); [ "$API" = 1 ] && RSA=(--rsa-backend)
    "$HERE/tools/sim_host.sh" "${RSA[@]}" --profile "$PROFILE" "${PASS[@]}"
  else
    # REALISTIC parallel fleet: pyrender cameras (transparent foliage, correct
    # textures, ~24 fps) + lidar worker (point clouds) + physics server publishing
    # telemetry/control + the Livox IMU. RSA_BACKEND=1 routes telemetry to the
    # backend ports the rove_sensor_api mock drivers subscribe to.
    RSAENV=(); [ "$API" = 1 ] && RSAENV+=(RSA_BACKEND=1)
    [ "$TINY" = 1 ] && RSAENV+=(TINY=1)
    env PROFILE="$PROFILE" "${RSAENV[@]}" "$HERE/tools/sim_fleet.sh"
  fi
else
  # GUI window + parallel camera/lidar workers (kills them when the window closes).
  SIM_ARGS=(--profile "$PROFILE")
  [ "$TERRAIN" = 1 ] && SIM_ARGS+=(--terrain)
  [ -n "$SCENE" ] && SIM_ARGS+=(--scene "$SCENE")
  SIM_ARGS+=("${PASS[@]}")
  "$HERE/tools/gui_fleet.sh" "${SIM_ARGS[@]}"
fi
