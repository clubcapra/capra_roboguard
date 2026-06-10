#!/usr/bin/env bash
# sim_fleet: a realtime physics sim + N PARALLEL camera-render workers.
#
# Why: pybullet renders behind the GIL and returns pixels as a Python tuple, so
# ONE process can't stream many cameras at 24 fps. The fix is parallelism across
# processes: the physics server publishes the robot's pose+joints to a shared file
# and each cam_worker (its own process/GIL/GL context) mirrors it and renders a
# SUBSET of the cameras. ~2 cameras per worker hits a solid 24 fps each.
#
#   tools/sim_fleet.sh                    # realtime server + lidar worker + 3 cam
#                                         #   workers: 5 cams @24fps, 2 Livox @10Hz
#   WORKERS_ONLY=1 tools/sim_fleet.sh     # just the cam workers (pair with
#                                         #   live.py --publish-state for the GUI)
#   PERW=2 RES=320x240 FPS=24 tools/sim_fleet.sh
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
# Bundled binaries (mediamtx, ffmpeg) live in tools/bin -- put them first on PATH
# so the camera workers' bare `ffmpeg` calls resolve here without a system install.
export PATH="$HERE/tools/bin:$PATH"
VENV_SP="$HERE/../rove_sim_venv/lib/python3.13/site-packages"
PY=/usr/bin/python3.13
[ -x "$PY" ] || { echo "need python3.13 (sudo dnf install python3.13)"; exit 1; }

PROFILE=${PROFILE:-standard}
CAMERAS=${CAMERAS:-cam_front,cam_rear,cam_left,cam_right,cam_arm}
NCAMS=$(awk -F, '{print NF}' <<< "$CAMERAS")

# TINY=1: aggressive low-spec mode for a ThinkStation Tiny (16 GB RAM, Quadro P320
# / 2 GB VRAM, weak CPU). The whole point: only ONE process touches the GPU.
#   * ALL cameras in ONE pyrender process (PERW=NCAMS) -> 1 scene copy in VRAM.
#   * 640p cameras (user wants the bump), 128px textures, shadows OFF.
#   * physics CPU-only + fewer solver iters; lidar fewer rays + lower rate.
# Set these defaults FIRST so the regular ${VAR:-...} lines below honour them.
TINY=${TINY:-0}
if [ "$TINY" = "1" ]; then
  # Shadows + transparent foliage are KEPT (required features) -- the optimisation
  # is purely the GPU/CAMERA path: the renderer is the ONLY GPU process, 128px
  # textures. PERW stays low so cameras render in PARALLEL (24 fps/cam needs it);
  # the small 128px-texture scene copies fit several-over in 2 GB (measured).
  # NOTE: TINY does NOT touch the lidar -- raycast is CPU-only (never sees the
  # 2 GB GPU), so it runs at full real-hardware fidelity regardless (see below).
  # RES is just under 640p (576x432, 4:3) -- ~19% fewer pixels than 640x480 frees
  # CPU so the sim stays smooth when the API is taking a heavy command stream.
  : "${RES:=576x432}" "${FPS:=24}" "${TEXMAX:=128}"
fi

FPS=${FPS:-24}
TERRAIN=${TERRAIN:-../free_dirt_road_through_forest.glb}
# Lidar = real Livox Mid-360 at the real 10 Hz, one worker PER sensor (parallel).
# Raycast (pybullet rayTestBatch) is CPU-bound and BURSTY (it casts for ~tens of ms,
# then sleeps to the 10 Hz period). The wall is that 2 lidar bursts + 5 cameras +
# physics all want cores at once. The fix that works is THREAD-CAPPING each lidar
# worker so its burst can't grab every core and stall the physics server.
# Measured FULL load (physics + 5 cameras @640p + 2 lidar) on THIS 8-vCPU host:
#     config (rays/threads)   lidar Hz (both)   physics realtime
#     11k / all-cores            6-7 Hz            0.62-0.73x   (lidar starves physics)
#     4k  / all-cores            10.0 Hz           0.86-0.89x
#     6k  / 3-thread             7.5-9.4 Hz        0.94-0.96x
#     4k  / 3-thread             9.8-10.1 Hz       0.95-0.98x   <- best balance (default)
# So 4000 rays + a 3-thread cap per worker holds the real 10 Hz on BOTH Mid-360s
# while physics stays ~realtime. (The 2 GB GPU is NOT the lidar limit -- raycast
# never touches it; cores are.) Higher LRAYS trades Hz/physics for per-scan density;
# the non-repetitive scan fills coverage in over frames either way. Workers PRINT
# live Hz and sim_server prints its realtime factor -- tune LRAYS/LTHREADS to taste.
LIDARS=${LIDARS:-livox_top,livox_bottom}; LRAYS=${LRAYS:-4000}; LHZ=${LHZ:-10}
# threads PER lidar worker (0 = all cores). Capping each worker so the two bursts +
# physics fit the core count is what keeps physics ~realtime (see table above).
LTHREADS=${LTHREADS:-3}
TEXMAX=${TEXMAX:-512}
WORKERS_ONLY=${WORKERS_ONLY:-0}
TINY_ARG=(); [ "$TINY" = "1" ] && TINY_ARG=(--tiny)
# RENDERER: pyrender (real alpha/transparent foliage, correct textures, no flicker,
# numpy framebuffer -> higher res) or pybullet (the old getCameraImage path).
RENDERER=${RENDERER:-pyrender}
if [ "$RENDERER" = "pyrender" ]; then
  CAM_WORKER=tools/pyrender_worker.py; RES=${RES:-640x480}; PERW=${PERW:-2}
else
  CAM_WORKER=tools/cam_worker.py;      RES=${RES:-256x192}; PERW=${PERW:-2}
fi

cd "$HERE"
RUN=(env PYTHONPATH="$HERE:$VENV_SP" PYOPENGL_PLATFORM=egl "$PY")
PIDS=()
cleanup(){ for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done; }
trap cleanup EXIT INT TERM

# Warm the build cache with ONE process first. sim_server + lidar_worker + the
# cam workers all call runtime.build(), which generates+caches the URDF, robot
# GLB->OBJ meshes and terrain OBJ. If they start cold simultaneously they RACE on
# those files (one reads a half-written mesh -> "Cannot load URDF file" and the
# whole stack quietly comes up with a frozen robot). A single warm build makes
# the cache a pure READ for every worker afterwards -> no race.
echo "[fleet] warming build cache (single process, prevents concurrent-build races)…"
env PYTHONPATH="$HERE:$VENV_SP" PROFILE="$PROFILE" TERRAIN="$TERRAIN" "$PY" - <<'PY'
import os
from rove_sim import runtime
glb = os.environ.get("TERRAIN") or None
ov = {"terrain": {"source": glb, "texture": False}} if glb else {}
runtime.build(os.environ.get("PROFILE", "standard"), mode="headless",
              world="mock", world_overrides=ov).disconnect()
PY
echo "[fleet] cache warm."

if [ "$WORKERS_ONLY" != "1" ]; then               # realtime physics + state ONLY
  # RSA_BACKEND=1 -> publish telemetry on the shared backend ports (6000+) so the
  # real rove_sensor_api binary's mock drivers can subscribe (rove.sh --api).
  RSA_ARGS=()
  [ "${RSA_BACKEND:-0}" = "1" ] && RSA_ARGS=(--rsa-backend)
  # --no-texture on the server + lidar worker: NEITHER renders the cameras (pyrender
  # workers do), so loading the 2048² terrain textures into all of them just burns
  # the 6 GB GPU and OOMs. Geometry-only keeps physics/raycast correct.
  "${RUN[@]}" tools/sim_server.py --profile "$PROFILE" --terrain "$TERRAIN" \
      --no-texture --no-rtsp --no-lidar "${RSA_ARGS[@]}" "${TINY_ARG[@]}" &
  PIDS+=($!)
  echo "[fleet] physics server (telemetry/control, robot_state, Livox IMU; raycast+cams offloaded)"
  # Two Mid-360s, two mappings to processes (see LTHREADS note above):
  #   LIDAR_SPLIT=1 (default): one worker PER Livox so the two sensors raycast in
  #     PARALLEL, each capped to LTHREADS cores -> both hold 10 Hz AND physics keeps
  #     its cores. The right choice once threads are capped.
  #   LIDAR_SPLIT=0 ("bundled"): ONE worker casts both Livox sequentially (one core
  #     pool) -> halves the rate at a given LRAYS. Only for very few cores.
  if [ "${LIDAR_SPLIT:-1}" = "1" ]; then
    IFS=',' read -ra _LIDARS <<< "$LIDARS"
    for _L in "${_LIDARS[@]}"; do
      [ -n "$_L" ] || continue
      "${RUN[@]}" tools/lidar_worker.py --terrain "$TERRAIN" --no-texture \
          --lidars "$_L" --hz "$LHZ" --rays "$LRAYS" --threads "$LTHREADS" &
      PIDS+=($!)
      echo "[fleet] lidar worker  ->  $_L  @ ${LHZ}Hz ${LRAYS} rays, ${LTHREADS} threads (UDP cloud)"
    done
  else
    "${RUN[@]}" tools/lidar_worker.py --terrain "$TERRAIN" --no-texture \
        --lidars "$LIDARS" --hz "$LHZ" --rays "$LRAYS" --threads "$LTHREADS" &
    PIDS+=($!)
    echo "[fleet] lidar worker (bundled) -> $LIDARS @ ${LHZ}Hz ${LRAYS} rays/Livox, ${LTHREADS} threads (UDP clouds)"
  fi
fi

# ONE shared mediamtx for all camera workers (multiple mediamtx clash on their
# UDP RTP/MoQ ports) -> every camera lands on rtsp://127.0.0.1:RTSP_PORT/<cam>.
RTSP_PORT=${RTSP_PORT:-8554}
MTX_CFG=$(mktemp --suffix=.yml)
printf 'rtspAddress: :%s\nrtspTransports: [tcp]\nrtmp: no\nhls: no\nwebrtc: no\nsrt: no\nlogLevel: error\npaths:\n  all_others:\n' "$RTSP_PORT" > "$MTX_CFG"
"$HERE/tools/bin/mediamtx" "$MTX_CFG" &
PIDS+=($!)
sleep 1.5
echo "[fleet] shared RTSP server on :$RTSP_PORT"

# SHADOWS=0 disables directional shadows on the pyrender cameras (recovers ~2x fps;
# the forest is otherwise rendered twice -- once for colour, once for the shadow map).
PYR_ARGS=()
if [ "$RENDERER" = "pyrender" ]; then
  PYR_ARGS+=(--texmax "$TEXMAX")
  [ "${SHADOWS:-1}" = "0" ] && { PYR_ARGS+=(--no-shadows); echo "[fleet] shadows OFF (SHADOWS=0)"; }
fi
echo "[fleet] cameras $RES @${FPS}fps, ${PERW}/worker, tex≤${TEXMAX}px$([ "$TINY" = 1 ] && echo ', TINY')"

IFS=',' read -ra CAMS <<< "$CAMERAS"
i=0
while [ $i -lt ${#CAMS[@]} ]; do
  group=$(IFS=,; echo "${CAMS[*]:$i:$PERW}")
  "${RUN[@]}" "$CAM_WORKER" --terrain "$TERRAIN" --cameras "$group" \
      --res "$RES" --fps "$FPS" --port "$RTSP_PORT" --shared-server "${PYR_ARGS[@]}" &
  PIDS+=($!)
  echo "[fleet] cam worker ($RENDERER) -> $group"
  i=$((i + PERW))
done
echo "[fleet] cameras: $(echo "$CAMERAS" | sed "s#\([^,]*\)#rtsp://127.0.0.1:$RTSP_PORT/\1#g")"
echo "[fleet] up: $((${#PIDS[@]})) processes. Ctrl-C to stop."
wait
