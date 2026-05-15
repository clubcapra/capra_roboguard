#!/usr/bin/env bash
# install.sh — register rove_control_bridge as a systemd service.
#
# Usage:
#   sudo ./scripts/install.sh              # defaults: user=capra, config=config/default.yaml
#   sudo ./scripts/install.sh --user bob   # run the service as a different user
#
# The service is installed as rove-control-bridge.service and set to start
# automatically on boot after the network comes up.

set -euo pipefail

SERVICE_NAME="rove-control-bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"             # rove_control_bridge/
PYTHONPATH_ROOT="$(dirname "$PKG_ROOT")"        # parent of rove_control_bridge/
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

# ---------- defaults ----------
RUN_AS_USER="${1:-capra}"
CONFIG_FILE="${CONFIG:-$PKG_ROOT/config/default.yaml}"

# ---------- parse args ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --user) RUN_AS_USER="$2"; shift 2 ;;
        --config) CONFIG_FILE="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ---------- sanity checks ----------
if [[ "$EUID" -ne 0 ]]; then
    echo "[install] ERROR: run this script with sudo."
    exit 1
fi

if ! id "$RUN_AS_USER" &>/dev/null; then
    echo "[install] ERROR: user '$RUN_AS_USER' does not exist."
    exit 1
fi

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "[install] ERROR: config file not found: $CONFIG_FILE"
    exit 1
fi

echo "[install] Installing ${SERVICE_NAME}.service"
echo "[install]   run-as user : $RUN_AS_USER"
echo "[install]   bridge dir  : $PKG_ROOT"
echo "[install]   config file : $CONFIG_FILE"

# ---------- write unit file ----------
# ExecStart invokes scripts/run.sh — same entry point you would use from
# the terminal. Config and strategy overrides ride on the env vars run.sh
# already reads (CONFIG, TRACKS_STRATEGY).
cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=Rove Control Bridge (RoveControl proto → rove_sensor_api)
After=network.target
Wants=network.target

[Service]
Type=simple
User=${RUN_AS_USER}
WorkingDirectory=${PYTHONPATH_ROOT}
Environment=CONFIG=${CONFIG_FILE}
ExecStart=/bin/bash ${SCRIPT_DIR}/run.sh
Restart=on-failure
RestartSec=3
StandardOutput=journal
StandardError=journal
SyslogIdentifier=${SERVICE_NAME}

[Install]
WantedBy=multi-user.target
EOF

# ---------- enable & start ----------
systemctl daemon-reload
systemctl enable "${SERVICE_NAME}.service"
systemctl restart "${SERVICE_NAME}.service"

echo ""
echo "[install] Done. Service status:"
systemctl status "${SERVICE_NAME}.service" --no-pager || true
echo ""
echo "  journalctl -u ${SERVICE_NAME} -f    # follow logs"
echo "  systemctl stop ${SERVICE_NAME}      # stop"
echo "  systemctl restart ${SERVICE_NAME}   # restart"
