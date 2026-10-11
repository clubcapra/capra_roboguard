#!/usr/bin/env bash
# One physics process, one shared alpha-aware LiDAR BVH, one GPU camera scene.
# SIM_PYTHON=/path/to/venv/bin/python RSA_BACKEND=1 PORTS=../HALo/config/ports.toml tools/sim_fleet.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE"
export PATH="$HERE/tools/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
PY=${SIM_PYTHON:-python3}
PROFILE=${PROFILE:-standard}
WORLD_CONFIG=${WORLD_CONFIG:-config/world.yaml}
SCENE=${SCENE:-scenes/forest_demo.json}
STATE_FILE=${STATE_FILE:-/dev/shm/rove_robot_state.json}
FPS=${FPS:-30}; RES=${RES:-1920x1080}
SHADOW_SIZE=${SHADOW_SIZE:-4096}
LHZ=${LHZ:-30}; LRAYS=${LRAYS:-4000}
RTSP_PORT=${RTSP_PORT:-8554}
METRICS=${METRICS:-/tmp/rove_sim_gpu_metrics.json}
LIDAR_METRICS=${LIDAR_METRICS:-/tmp/rove_sim_lidar_metrics.json}
COMMON=(--profile "$PROFILE" --state-file "$STATE_FILE")
[[ -n "$WORLD_CONFIG" ]] && COMMON+=(--world-config "$WORLD_CONFIG")
[[ -n "$SCENE" ]] && COMMON+=(--scene "$SCENE")
[[ -n "${TERRAIN:-}" ]] && COMMON+=(--terrain "$TERRAIN")
BACKEND=()
[[ "${RSA_BACKEND:-0}" = 1 ]] && BACKEND+=(--rsa-backend)
[[ -n "${PORTS:-}" ]] && BACKEND+=(--ports "$PORTS")
PIDS=(); MTX_CFG=""
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
  [[ -z "$MTX_CFG" ]] || rm -f "$MTX_CFG"
}
trap cleanup EXIT INT TERM
# Warm generated URDF/OBJ files before any parallel readers start.
"$PY" - "$PROFILE" "$WORLD_CONFIG" "$SCENE" "${TERRAIN:-}" <<'PY'
import sys
from rove_sim.world.scene import build_configured_sim
sim=build_configured_sim(sys.argv[1],world_config=sys.argv[2] or None,
                         scene_path=sys.argv[3] or None,terrain=sys.argv[4] or None,
                         purpose='physics',texture=False,egl=False)
sim.disconnect()
PY
if [[ "${WORKERS_ONLY:-0}" != 1 ]]; then
  "$PY" tools/sim_server.py "${COMMON[@]}" "${BACKEND[@]}" --no-texture --no-rtsp --no-lidar &
  PIDS+=($!)
  "$PY" tools/lidar_worker.py "${COMMON[@]}" "${BACKEND[@]}" --no-texture \
    --lidars "${LIDARS:-livox_top,livox_bottom}" --hz "$LHZ" --rays "$LRAYS" --threads 1 --metrics "$LIDAR_METRICS" &
  PIDS+=($!)
fi
if [[ "${RTSP_SHARED:-0}" != 1 ]]; then
  MTX_CFG=$(mktemp --suffix=.yml)
  printf 'rtspAddress: :%s\nrtspTransports: [tcp]\nrtmp: no\nhls: no\nwebrtc: no\nsrt: no\nmoq: no\nlogLevel: error\npaths:\n  all_others:\n' "$RTSP_PORT" > "$MTX_CFG"
  mediamtx "$MTX_CFG" &
  PIDS+=($!)
  sleep 1
fi
VIEW_ARGS=()
if [[ -n "${SIM_VIEW_PASSWORD:-}" ]]; then
  VIEW_IMAGE=${VIEW_IMAGE:-/tmp/rove_sim_view.jpg}
  VIEW_CONTROL=${VIEW_CONTROL:-/tmp/rove_sim_view.json}
  VIEW_ARGS=(--view-image "$VIEW_IMAGE" --view-control "$VIEW_CONTROL")
  "$PY" tools/web_viewer.py --host "${VIEW_HOST:-127.0.0.1}" --port "${VIEW_PORT:-8081}" \
    --image "$VIEW_IMAGE" --control "$VIEW_CONTROL" --metrics "$METRICS" &
  PIDS+=($!)
fi
"$PY" tools/gpu_worker.py "${COMMON[@]}" --cameras "${CAMERAS:-cam_front,cam_rear,cam_left,cam_right,cam_arm}" \
  --res "$RES" --fps "$FPS" --encoder "${ENCODER:-auto}" --shadow-size "$SHADOW_SIZE" --port "$RTSP_PORT" --shared-server --metrics "$METRICS" "${VIEW_ARGS[@]}" &
PIDS+=($!)
echo "[fleet] $RES cameras at $FPS fps; LiDAR $LRAYS rays/scan at $LHZ Hz. Ctrl-C to stop."
# A failed worker stops the fleet, instead of silently leaving stale sensors.
wait -n "${PIDS[@]}"
