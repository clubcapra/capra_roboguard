#!/usr/bin/env bash
# Build and install HALo on its Linux host; no fixed robot user/checkout paths.
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
    echo 'error: run this installer with sudo.' >&2
    exit 1
fi
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RUN_USER="${HALO_RUN_USER:-${SUDO_USER:-}}"
if [[ -z "$RUN_USER" || "$RUN_USER" == root ]]; then
    echo 'error: set HALO_RUN_USER to the hardware service account.' >&2
    exit 1
fi
RUN_GROUP="$(id -gn "$RUN_USER")"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
LOG_DIR="${HALO_LOG_DIR:-/var/log/halo}"
CARGO_BIN="${HALO_CARGO_BIN:-$RUN_HOME/.cargo/bin/cargo}"
if [[ ! -x "$CARGO_BIN" ]]; then CARGO_BIN="$(command -v cargo)"; fi
sudo -H -u "$RUN_USER" env "PATH=$(dirname "$CARGO_BIN"):$PATH" \
    "$CARGO_BIN" build --release --manifest-path "$PROJECT_DIR/Cargo.toml" --target-dir "$PROJECT_DIR/target"
if [[ -f "$PROJECT_DIR/config/livox.toml" ]]; then
    sudo -H -u "$RUN_USER" cmake -S "$PROJECT_DIR/src/drivers/livox/native" -B "$PROJECT_DIR/src/drivers/livox/native/build"
    sudo -H -u "$RUN_USER" cmake --build "$PROJECT_DIR/src/drivers/livox/native/build" --parallel
fi
install -d -m 0750 -o "$RUN_USER" -g "$RUN_GROUP" "$LOG_DIR"
# Render with systemd quoting, rather than interpolating paths into shell commands.
python3 - "$PROJECT_DIR" "$RUN_USER" "$RUN_GROUP" "$LOG_DIR" <<'PY'
import pathlib, sys
project, user, group, logs = sys.argv[1:]
text = (pathlib.Path(project) / 'scripts/halo.service').read_text()
for key, value in {'PROJECT_DIR':project, 'RUN_USER':user, 'RUN_GROUP':group, 'LOG_DIR':logs}.items():
    if '\n' in value or '\r' in value: raise ValueError('newline in service configuration')
    value = value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')
    text = text.replace('@'+key+'@', value)
pathlib.Path('/etc/systemd/system/halo.service').write_text(text)
PY
chmod 0644 /etc/systemd/system/halo.service
if systemctl cat rove-sensor-api.service >/dev/null 2>&1; then
    systemctl disable --now rove-sensor-api.service
fi
systemctl daemon-reload
systemctl enable halo.service
systemctl restart halo.service
systemctl --no-pager --full status halo.service || true
printf 'HALo installed. Set its password as %s with target/release/halo --set-password, then restart halo.service.\n' "$RUN_USER"
