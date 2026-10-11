#!/usr/bin/env bash
# install.sh — register rove-ik-engine as a systemd service.
#
# Usage:
#   sudo ./scripts/install.sh                              # defaults: user=capra, engine.standard.toml (arm)
#   sudo ./scripts/install.sh --user bob                   # run the service as a different user
#   sudo ./scripts/install.sh --config engine.toml         # run with an alternate config (e.g. caged)
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
# This robot is the ARM build — default to it so a bare install can never
# silently bring back the caged config. Pass --config engine.toml for caged.
CONFIG="engine.standard.toml"

# ---------- parse args ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --user) RUN_AS_USER="$2"; shift 2 ;;
        --config) CONFIG="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

if [[ -n "$CONFIG" && ! -f "${PKG_ROOT}/${CONFIG}" ]]; then
    echo "[install] ERROR: config '${PKG_ROOT}/${CONFIG}' not found."
    exit 1
fi

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
echo "[install]   config      : ${CONFIG:-engine.toml (default)}"

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
After=network.target halo.service
Wants=network.target
# The engine resolves every drive/arm port from sensor_api's /discover at
# startup, and sensor_api assigns those ports DYNAMICALLY — after a sensor_api
# restart the map shuffles and a stale port can command the WRONG MOTOR (track
# commands once drove a flipper). PartOf propagates sensor_api restarts to this
# unit so the engine always re-discovers. No-op if sensor_api isn't a service
# on this host.
PartOf=halo.service
# Never give up restarting: the engine self-exits after a run of solver failures
# so systemd hands it a fresh process. The default start-rate limit would instead
# park it in a permanent 'failed' state (needing a manual reset / reinstall after
# an e-stop) — disable it so it always comes back.
StartLimitIntervalSec=0

[Service]
Type=simple
User=${RUN_AS_USER}
WorkingDirectory=${PKG_ROOT}
Environment=FORGEBOT_NO_BOOTSTRAP=1
ExecStart=${VENV_PY} ${PKG_ROOT}/run.py${CONFIG:+ ${PKG_ROOT}/${CONFIG}}
# Restart on ANY exit (not just failure) so a crash/clean-exit both recover.
Restart=always
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
