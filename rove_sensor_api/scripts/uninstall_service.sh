#!/usr/bin/env bash
# Stop, disable, and remove the rove_sensor_api systemd unit.
# Leaves the project directory and CSV logs alone.
#
# Idempotent: safe to run even if the service was never installed, is already
# stopped, or was disabled manually.

set -uo pipefail

SERVICE_NAME="rove-sensor-api"
UNIT_NAMES=(
    "/etc/systemd/system/${SERVICE_NAME}.service"
    "/lib/systemd/system/${SERVICE_NAME}.service"
    "/usr/lib/systemd/system/${SERVICE_NAME}.service"
)

if [[ $EUID -ne 0 ]]; then
    echo "error: must run as root (use sudo)." >&2
    exit 1
fi

echo "==> Stopping $SERVICE_NAME (if running)"
systemctl stop "$SERVICE_NAME" 2>/dev/null || true

echo "==> Disabling $SERVICE_NAME (if enabled)"
systemctl disable "$SERVICE_NAME" 2>/dev/null || true

# Remove drop-in overrides too — a leftover override directory can keep the
# unit "loaded" in systemd's view even after the main file is gone.
OVERRIDE_DIR="/etc/systemd/system/${SERVICE_NAME}.service.d"
if [[ -d "$OVERRIDE_DIR" ]]; then
    echo "==> Removing override dir $OVERRIDE_DIR"
    rm -rf "$OVERRIDE_DIR"
fi

removed_any=0
for unit in "${UNIT_NAMES[@]}"; do
    if [[ -f "$unit" ]]; then
        echo "==> Removing $unit"
        rm -f "$unit"
        removed_any=1
    fi
done

# Also catch any stray symlinks systemctl may have created under wants/.
find /etc/systemd/system -type l -name "${SERVICE_NAME}.service" -print -delete 2>/dev/null || true

echo "==> Reloading systemd"
systemctl daemon-reload
systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true

if [[ $removed_any -eq 0 ]]; then
    echo "(no unit file was found on disk — service may have already been uninstalled)"
fi

# Final sanity check.
if systemctl status "$SERVICE_NAME" >/dev/null 2>&1; then
    echo "warning: systemctl still knows about $SERVICE_NAME — run 'systemctl status $SERVICE_NAME' to inspect." >&2
else
    echo "Done. CSV logs under /var/log/rove-sensor-api were left in place."
fi
