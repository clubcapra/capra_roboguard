#!/usr/bin/env bash
# Launch the GUI sim + the parallel camera render workers as ONE unit, and KILL
# EVERYTHING when the GUI window is closed. Close the window -> the cam workers,
# the shared RTSP server and the state file all go away (no orphans).
#
#   tools/gui_fleet.sh --profile standard --terrain
#   RES=320x240 PERW=1 tools/gui_fleet.sh --profile caged --terrain
#
# The GUI runs in the FOREGROUND; when live.py exits (window closed) we fall
# through to cleanup. --sensors (live lidar cloud) + --publish-state (feed the cam
# workers) are added automatically.
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
export PATH="$HERE/tools/bin:$PATH"   # bundled mediamtx/ffmpeg first on PATH
cd "$HERE"

FLEET_PID=""
cleanup() {
  set +e   # a no-match pkill returns 1 -> must NOT abort the rest of cleanup
  # kill the workers DIRECTLY (don't rely on a child trap -- a bash blocked in
  # sleep defers it) + reap the orphaned ffmpeg pushers and the RTSP server.
  [ -n "$FLEET_PID" ] && kill "$FLEET_PID" 2>/dev/null
  pkill -9 -f "tools/cam_worker.py"      2>/dev/null
  pkill -9 -f "tools/pyrender_worker.py" 2>/dev/null
  pkill -9 -f "tools/lidar_worker.py"    2>/dev/null
  pkill -9 -f "rtsp://127.0.0.1"      2>/dev/null   # orphaned ffmpeg pushers
  pkill -9 -x mediamtx 2>/dev/null
  rm -f /dev/shm/rove_robot_state.json
}
trap cleanup EXIT INT TERM

# parallel camera workers (manage their own shared mediamtx); they idle until the
# GUI starts publishing robot_state, then mirror it.
WORKERS_ONLY=1 tools/sim_fleet.sh &
FLEET_PID=$!

# GUI in the foreground -- blocks until the window is closed, then cleanup fires.
# The GUI is FLAT (--no-texture) by default: the textured pybullet window + the
# heavy pyrender workers together saturate the GPU and the GUI's X connection dies
# ("XIO fatal IO error / Timer expired"). The realistic view IS the camera feeds;
# the GUI is just for driving. Set GUI_TEXTURE=1 to force a textured GUI.
GUITEX=${GUI_TEXTURE:+}; [ -z "$GUI_TEXTURE" ] && GUITEX="--no-texture"
DISPLAY=${DISPLAY:-:0} tools/live_host.sh "$@" --sensors --publish-state $GUITEX
