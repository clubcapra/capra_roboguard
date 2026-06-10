#!/usr/bin/env bash
# Install the rove_sim FULL STACK (physics + cameras + lidars + the real
# rove_sensor_api, sim-backed) as a boot-persistent systemd service.
#
# The service runs:  tools/rove.sh headless --tiny --api
#   * physics server (telemetry/control, robot_state, Livox IMU)
#   * 5 cameras @640p -> RTSP rtsp://127.0.0.1:8554/<cam>   (--tiny: fits the 2 GB GPU)
#   * 2 Livox Mid-360 -> multi-packet UDP point clouds 5022/5024  @ ~10 Hz
#   * rove_sensor_api binary in SIM-BACKEND mode -> HTTP :8080 + UDP 5000+
#
# MUST be run with sudo. Runs as the invoking (non-root) user so EGL/GPU + the
# venv work exactly as they do interactively. Re-run any time to redeploy.
#
#   sudo ./install_service.sh
set -euo pipefail

SERVICE_NAME="rove-sim"
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$HERE/.." && pwd)"          # .../rove_sim
ENTRY="$PROJECT_DIR/tools/rove.sh"
ARGS="headless --tiny --api"
UNIT_DST="/etc/systemd/system/$SERVICE_NAME.service"

if [[ $EUID -ne 0 ]]; then
    echo "error: must run as root (use: sudo $0)" >&2
    exit 1
fi
[ -x "$ENTRY" ] || { echo "error: $ENTRY not found/executable" >&2; exit 1; }

# Which user the sim runs as: explicit arg/env wins, else the sudo caller, else the
# owner of the project tree (the sim needs a normal user for EGL/GPU + the venv).
RUN_USER="${1:-${RUN_USER:-${SUDO_USER:-$(stat -c %U "$PROJECT_DIR")}}}"
if [ -z "$RUN_USER" ] || [ "$RUN_USER" = "root" ]; then
    echo "error: could not determine a non-root user to run as." >&2
    echo "       pass one explicitly:  sudo $0 <username>   (e.g. think2)" >&2
    exit 1
fi
id "$RUN_USER" >/dev/null 2>&1 || { echo "error: user '$RUN_USER' does not exist." >&2; exit 1; }

echo "==> Installing $UNIT_DST (runs as $RUN_USER)"
cat > "$UNIT_DST" <<EOF
[Unit]
Description=Capra Roboguard sim — physics + cameras + lidars + rove_sensor_api (sim-backed)
After=network.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$PROJECT_DIR
ExecStart=$ENTRY $ARGS
Environment=PYOPENGL_PLATFORM=egl
Restart=on-failure
RestartSec=5
# rove.sh's own trap + systemd's cgroup kill reap every worker (python/mediamtx/ffmpeg).
KillMode=control-group
TimeoutStopSec=25

[Install]
WantedBy=multi-user.target
EOF
chmod 0644 "$UNIT_DST"

echo "==> Reloading systemd"
systemctl daemon-reload
echo "==> Enabling + (re)starting $SERVICE_NAME"
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"   # restart (not just start) so a redeploy reloads the new binary
sleep 3
systemctl --no-pager --full status "$SERVICE_NAME" || true

cat <<EOF

Done — $SERVICE_NAME is running and will start on every boot.

  systemctl status   $SERVICE_NAME
  systemctl restart  $SERVICE_NAME
  journalctl -u       $SERVICE_NAME -f          # live logs (lidar Hz, physics realtime)
  curl http://localhost:8080/discover           # API up?  (after ~15 s warm-up)
  ffplay rtsp://127.0.0.1:8554/cam_front         # a camera feed

Remove:  sudo $HERE/uninstall_service.sh
EOF
