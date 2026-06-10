#!/usr/bin/env bash
# Launch the headless sim_server on the HOST (outside the flatpak/IDE sandbox) so
# it has the GPU (EGL render for the camera feeds) plus ffmpeg + mediamtx on PATH
# for the RTSP streams. This is the sim the autonomy stack + vision model hook up
# to: rove_sensor_api telemetry/control over UDP, camera RTSP, Livox point clouds.
#
# Same python3.13-vs-3.14 reason as live_host.sh (the venv is a cp313 build):
#     sudo dnf install python3.13      # once
#     tools/sim_host.sh --profile standard --terrain
#
# Endpoints are printed on startup. Read telemetry with state.RoveSensorApiState-
# Source, send control with api.control_bridge.ControlPublisher, read clouds with
# sensors.lidar.decode_cloud, and point the vision model at the rtsp:// URLs.
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
export PATH="$HERE/tools/bin:$PATH"   # bundled mediamtx/ffmpeg first on PATH
VENV_SP="$HERE/../rove_sim_venv/lib/python3.13/site-packages"
PY=/usr/bin/python3.13
if [ ! -x "$PY" ]; then
  echo "python3.13 not found on the host (the venv is a cp313 build):"
  echo "    sudo dnf install python3.13"
  exit 1
fi
cd "$HERE"
exec env PYTHONPATH="$HERE:$VENV_SP" "$PY" -u tools/sim_server.py "$@"
