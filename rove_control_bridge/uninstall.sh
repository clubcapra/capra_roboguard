#!/usr/bin/env bash
# uninstall.sh — stop, disable, and remove the rove_control_bridge service.
# Leaves the crate dir, build artifacts, and config untouched.
#
# Idempotent: safe to run even if the service was never installed, is already
# stopped, or was disabled manually.
#
# Usage:
#   sudo ./uninstall.sh

set -uo pipefail

SERVICE_NAME="rove-control-bridge"

[[ $EUID -eq 0 ]] || { echo "error: must run as root (use sudo)." >&2; exit 1; }

echo "==> Stopping $SERVICE_NAME (if running)"
systemctl stop "$SERVICE_NAME" 2>/dev/null || true

echo "==> Disabling $SERVICE_NAME (if enabled)"
systemctl disable "$SERVICE_NAME" 2>/dev/null || true

# Drop-in overrides can keep a unit "loaded" even after the main file is gone.
OVERRIDE_DIR="/etc/systemd/system/${SERVICE_NAME}.service.d"
if [[ -d "$OVERRIDE_DIR" ]]; then
    echo "==> Removing override dir $OVERRIDE_DIR"
    rm -rf "$OVERRIDE_DIR"
fi

removed_any=0
for unit in \
    "/etc/systemd/system/${SERVICE_NAME}.service" \
    "/lib/systemd/system/${SERVICE_NAME}.service" \
    "/usr/lib/systemd/system/${SERVICE_NAME}.service"; do
    if [[ -f "$unit" ]]; then
        echo "==> Removing $unit"
        rm -f "$unit"
        removed_any=1
    fi
done

# Sweep any stray enable symlinks under wants/.
find /etc/systemd/system -type l -name "${SERVICE_NAME}.service" -print -delete 2>/dev/null || true

echo "==> Reloading systemd"
systemctl daemon-reload
systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true

[[ $removed_any -eq 0 ]] && echo "(no unit file found — already uninstalled?)"

if systemctl status "$SERVICE_NAME" >/dev/null 2>&1; then
    echo "warning: systemctl still knows about $SERVICE_NAME — inspect with 'systemctl status $SERVICE_NAME'." >&2
else
    echo "Done. The crate dir and config were left untouched."
fi
