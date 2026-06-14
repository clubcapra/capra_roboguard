#!/usr/bin/env bash
# uninstall.sh — remove the rove-control-bridge systemd service.
#
# Usage:
#   sudo ./uninstall.sh
#
# This stops and disables the service, removes the unit file, and reloads
# systemd. It does NOT delete the bridge source code or its venv.

set -euo pipefail

SERVICE_NAME="rove-control-bridge"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

if [[ "$EUID" -ne 0 ]]; then
    echo "[uninstall] ERROR: run this script with sudo."
    exit 1
fi

if [[ ! -f "$SERVICE_FILE" ]]; then
    echo "[uninstall] Service file not found — nothing to remove."
    exit 0
fi

echo "[uninstall] Stopping ${SERVICE_NAME}..."
systemctl stop "${SERVICE_NAME}.service" 2>/dev/null || true

echo "[uninstall] Disabling ${SERVICE_NAME}..."
systemctl disable "${SERVICE_NAME}.service" 2>/dev/null || true

echo "[uninstall] Removing ${SERVICE_FILE}..."
rm -f "$SERVICE_FILE"

systemctl daemon-reload

echo "[uninstall] Done. ${SERVICE_NAME} has been removed."
