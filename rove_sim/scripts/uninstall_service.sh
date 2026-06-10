#!/usr/bin/env bash
# Stop, disable, and remove the rove-sim systemd service.
#   sudo ./uninstall_service.sh
set -euo pipefail

SERVICE_NAME="rove-sim"
UNIT_DST="/etc/systemd/system/$SERVICE_NAME.service"

if [[ $EUID -ne 0 ]]; then
    echo "error: must run as root (use: sudo $0)" >&2
    exit 1
fi

systemctl disable --now "$SERVICE_NAME" 2>/dev/null || true
rm -f "$UNIT_DST"
systemctl daemon-reload
systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true
echo "Removed $SERVICE_NAME."
