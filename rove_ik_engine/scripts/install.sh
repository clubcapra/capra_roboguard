#!/usr/bin/env bash
# install.sh — register rove-ik-engine as a systemd service.
#
# Usage:
#   sudo ./scripts/install.sh              # defaults: user=capra
#   sudo ./scripts/install.sh --user bob   # run the service as a different user
#
# The service is installed as rove-ik-engine.service and set to start
# automatically on boot after the network comes up.

set -euo pipefail

SERVICE_NAME="rove-ik-engine"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"             # rove_ik_engine/
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
VENV_PY="${PKG_ROOT}/.venv/bin/python"

# ---------- defaults ----------
RUN_AS_USER="capra"

# ---------- parse args ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --user) RUN_AS_USER="$2"; shift 2 ;;
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

echo "[install] Installing ${SERVICE_NAME}.service"
echo "[install]   run-as user : $RUN_AS_USER"
echo "[install]   engine dir  : $PKG_ROOT"

# ---------- pre-bootstrap venv ----------
# Build the venv once at install time so the service never needs pip access.
if [[ ! -x "$VENV_PY" ]]; then
    echo "[install] Pre-bootstrapping venv (this may take a minute)..."
    runuser -l "$RUN_AS_USER" -c "
        cd '${PKG_ROOT}' &&
        python3 -m venv .venv &&
        .venv/bin/pip install --upgrade pip --quiet &&
        .venv/bin/pip install -r requirements.txt --quiet
    "
    echo "[install] Venv ready."
else
    echo "[install] Venv already exists — skipping bootstrap."
fi

# ---------- write unit file ----------
cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=Rove IK Engine (ForgeBOT IK solver + simulation layer)
After=network.target
Wants=network.target

[Service]
Type=simple
User=${RUN_AS_USER}
WorkingDirectory=${PKG_ROOT}
Environment=FORGEBOT_NO_BOOTSTRAP=1
ExecStart=${VENV_PY} ${PKG_ROOT}/run.py
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
