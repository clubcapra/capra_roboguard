#!/usr/bin/env bash
# Install arm_estop_watchdog as a systemd service on the Jetson.
#
# - Copies the unit file to /etc/systemd/system/.
# - Enables and (re)starts the service.
#
# IMPORTANT: edit SENSOR_API_URL in arm-estop-watchdog.service to point at the
# Pi running rove_sensor_api (default has a 192.168.2.X placeholder) before, or
# re-run this after editing.
#
# Re-run any time you update the unit file or the script.

set -euo pipefail

SERVICE_NAME="arm-estop-watchdog"
UNIT_SRC="$(cd "$(dirname "$0")" && pwd)/${SERVICE_NAME}.service"
UNIT_DST="/etc/systemd/system/${SERVICE_NAME}.service"

if [[ $EUID -ne 0 ]]; then
    echo "error: must run as root (use sudo)." >&2
    exit 1
fi

if [[ ! -f "$UNIT_SRC" ]]; then
    echo "error: unit file not found at $UNIT_SRC" >&2
    exit 1
fi

if grep -q "192.168.2.X" "$UNIT_SRC"; then
    echo "warning: SENSOR_API_URL in the unit file still has the 192.168.2.X" >&2
    echo "         placeholder -- /reload will fail until you set the Pi's IP." >&2
fi

echo "==> Installing unit file -> $UNIT_DST"
install -m 0644 "$UNIT_SRC" "$UNIT_DST"

echo "==> Reloading systemd"
systemctl daemon-reload

echo "==> Enabling and (re)starting $SERVICE_NAME"
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"

sleep 1
systemctl --no-pager --full status "$SERVICE_NAME" || true

cat <<EOF

Done.

Useful commands:
  systemctl status   $SERVICE_NAME
  systemctl restart  $SERVICE_NAME
  journalctl -u      $SERVICE_NAME -f

To remove the service:
  sudo $(dirname "$0")/uninstall_service.sh
EOF
