#!/usr/bin/env bash
# Stop, disable, and remove the rove_sensor_api systemd unit.
# Leaves the project directory and CSV logs alone.

set -euo pipefail

SERVICE_NAME="rove-sensor-api"
UNIT_DST="/etc/systemd/system/${SERVICE_NAME}.service"

if [[ $EUID -ne 0 ]]; then
    echo "error: must run as root (use sudo)." >&2
    exit 1
fi

if systemctl list-unit-files | grep -q "^${SERVICE_NAME}.service"; then
    echo "==> Stopping $SERVICE_NAME"
    systemctl stop "$SERVICE_NAME" || true
    echo "==> Disabling $SERVICE_NAME"
    systemctl disable "$SERVICE_NAME" || true
else
    echo "==> $SERVICE_NAME not registered with systemd, skipping stop/disable"
fi

if [[ -f "$UNIT_DST" ]]; then
    echo "==> Removing $UNIT_DST"
    rm -f "$UNIT_DST"
fi

echo "==> Reloading systemd"
systemctl daemon-reload
systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true

echo "Done. CSV logs under /var/log/rove-sensor-api were left in place."
