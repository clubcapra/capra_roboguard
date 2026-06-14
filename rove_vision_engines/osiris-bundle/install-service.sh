#!/usr/bin/env bash
# Install this Osiris bundle as a systemd SYSTEM service (starts on boot).
#
# Usage:  sudo ./install-service.sh [service-name]
#   OSIRIS_SERVICE_NAME   service name        (default: osiris-bundle)
#   OSIRIS_SERVICE_USER   user to run as      (default: the sudo caller, else root)
#   OSIRIS_BIND           API bind address    (default: baked into run.sh)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="${1:-${OSIRIS_SERVICE_NAME:-osiris-bundle}}"
RUN_AS="${OSIRIS_SERVICE_USER:-${SUDO_USER:-root}}"
UNIT="/etc/systemd/system/${SERVICE_NAME}.service"

if [ "$(id -u)" -ne 0 ]; then
  echo "Must run as root:  sudo ./install-service.sh [service-name]" >&2
  exit 1
fi

# Ports/bind are configured in config.env (run.sh sources it), so the unit just
# runs run.sh.
cat > "$UNIT" <<EOF
[Unit]
Description=Osiris Streaming API (${SERVICE_NAME})
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_AS}
WorkingDirectory=${HERE}
ExecStart=${HERE}/run.sh
Restart=always
RestartSec=5
# Engines may build their venv / download models on first start — don't time out.
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now "${SERVICE_NAME}.service"

echo "Installed and started ${SERVICE_NAME}.service (runs as ${RUN_AS})"
echo "  status:  systemctl status ${SERVICE_NAME}"
echo "  logs:    journalctl -u ${SERVICE_NAME} -f"
echo "  stop:    sudo systemctl stop ${SERVICE_NAME}"
echo "  remove:  sudo ./uninstall-service.sh ${SERVICE_NAME}"
