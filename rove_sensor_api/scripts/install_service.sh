#!/usr/bin/env bash
# Install rove_sensor_api as a systemd service on the Jetson.
#
# - Builds the release binary as the invoking user (so cargo's target/ cache
#   stays under that user, not root).
# - Copies the unit file to /etc/systemd/system/.
# - Creates LOG_DIR with capra:capra ownership.
# - Enables and starts the service.
#
# Re-run this any time you update the unit file or want to redeploy after a
# code change.

set -euo pipefail

SERVICE_NAME="rove-sensor-api"
UNIT_SRC="$(cd "$(dirname "$0")" && pwd)/${SERVICE_NAME}.service"
UNIT_DST="/etc/systemd/system/${SERVICE_NAME}.service"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RUN_USER="${SUDO_USER:-capra}"
LOG_DIR="/var/log/rove-sensor-api"

if [[ $EUID -ne 0 ]]; then
    echo "error: must run as root (use sudo)." >&2
    exit 1
fi

if [[ ! -f "$UNIT_SRC" ]]; then
    echo "error: unit file not found at $UNIT_SRC" >&2
    exit 1
fi

echo "==> Building release binary as $RUN_USER"
sudo -u "$RUN_USER" bash -lc "cd '$PROJECT_DIR' && cargo build --release"

BINARY="$PROJECT_DIR/target/release/capra-rove-interface"
if [[ ! -x "$BINARY" ]]; then
    echo "error: build did not produce $BINARY" >&2
    exit 1
fi

echo "==> Creating log dir $LOG_DIR"
mkdir -p "$LOG_DIR"
chown "$RUN_USER:$RUN_USER" "$LOG_DIR"

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
  curl http://localhost:8080/discover | jq

To remove the service:
  sudo $(dirname "$0")/uninstall_service.sh
EOF
