#!/usr/bin/env bash
# Remove the Osiris bundle systemd service. Bundle files are left untouched.
#
# Usage:  sudo ./uninstall-service.sh [service-name]
set -euo pipefail
SERVICE_NAME="${1:-${OSIRIS_SERVICE_NAME:-osiris-bundle}}"
UNIT="/etc/systemd/system/${SERVICE_NAME}.service"

if [ "$(id -u)" -ne 0 ]; then
  echo "Must run as root:  sudo ./uninstall-service.sh [service-name]" >&2
  exit 1
fi

systemctl disable --now "${SERVICE_NAME}.service" 2>/dev/null || true
rm -f "$UNIT"
systemctl daemon-reload
echo "Removed ${SERVICE_NAME}.service (bundle files left intact)"
