#!/usr/bin/env bash
# install.sh — run rove_control_bridge as a systemd service on this Pi.
#
# The bridge is the Roboguard orchestrator. It builds to a release binary and
# runs against the real robot with `--config config/real.toml --no-reset`
# (same defaults as run.sh). It reads its config by a RELATIVE path, so the
# unit pins WorkingDirectory to this crate dir.
#
# - Builds the release binary as the invoking user (keeps cargo's target/ cache
#   under that user, not root).
# - Writes the unit to /etc/systemd/system/ and enables + starts it.
#
# Usage:
#   sudo ./install.sh                              # real-robot defaults
#   sudo ./install.sh --config config/autonomy.toml   # e.g. sim config
#   sudo ./install.sh --user bob                   # run as another user
#   sudo ./install.sh --dry-run                    # service logs intent, never actuates
#
# Re-run any time you change code or the config — it rebuilds and restarts.
# Idempotent. Remove with ./uninstall.sh.

set -euo pipefail

SERVICE_NAME="rove-control-bridge"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DST="/etc/systemd/system/${SERVICE_NAME}.service"
BINARY="$PROJECT_DIR/target/release/rove_control_bridge"

# Defaults for the real-robot deployment (match run.sh).
RUN_USER="${SUDO_USER:-capra}"
CONFIG_REL="config/real.toml"
DRY_RUN=""

err() { echo "error: $*" >&2; exit 1; }

# ---------- parse args ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --user)    RUN_USER="$2"; shift 2 ;;
        --config)  CONFIG_REL="$2"; shift 2 ;;   # relative to the crate dir
        --dry-run) DRY_RUN="--dry-run"; shift ;;
        *) err "unknown option: $1" ;;
    esac
done

# ---------- sanity checks ----------
[[ $EUID -eq 0 ]] || err "must run as root (use sudo)."
id "$RUN_USER" &>/dev/null || err "user '$RUN_USER' does not exist."
[[ -f "$PROJECT_DIR/$CONFIG_REL" ]] || err "config not found: $PROJECT_DIR/$CONFIG_REL"
command -v cargo &>/dev/null || sudo -u "$RUN_USER" bash -lc 'command -v cargo' &>/dev/null \
    || err "cargo not found for $RUN_USER — install Rust first."

# ---------- build ----------
echo "==> Building release binary as $RUN_USER (this can take a while on the Pi)"
sudo -u "$RUN_USER" bash -lc "cd '$PROJECT_DIR' && cargo build --release"
[[ -x "$BINARY" ]] || err "build did not produce $BINARY"

# ---------- write unit ----------
echo "==> Writing unit -> $UNIT_DST"
echo "      run-as user : $RUN_USER"
echo "      working dir : $PROJECT_DIR"
echo "      config      : $CONFIG_REL ${DRY_RUN:+(dry-run)}"

# ExecStart mirrors run.sh's real-robot defaults: pinned config + --no-reset
# (no sim respawn). --dry-run is appended only when requested.
# After= only orders against the sibling services if they also run on this Pi;
# it is a harmless no-op if they live elsewhere (the bridge retries on failure).
cat > "$UNIT_DST" <<EOF
[Unit]
Description=Capra Roboguard control bridge / orchestrator (rove_control_bridge)
Documentation=file://${PROJECT_DIR}/README.md
After=network-online.target rove-sensor-api.service rove-ik-engine.service
Wants=network-online.target
# Always retry: the bridge exits if /discover is down at startup, and it is an
# always-on teleop front door (so even a clean mission-complete exit should come
# back). The default start-rate limit would park it in 'failed' after a few rapid
# retries (e.g. while sensor_api is still coming up) — disable it.
StartLimitIntervalSec=0

[Service]
Type=simple
User=${RUN_USER}
Group=${RUN_USER}
WorkingDirectory=${PROJECT_DIR}

# The bridge WAITS internally for rove_sensor_api /discover (retries, doesn't
# exit), so it no longer crash-loops if sensor_api is slow to come up. A short
# settle for the network is still cheap.
ExecStartPre=/bin/sleep 2
ExecStart=${BINARY} --config ${CONFIG_REL} --no-reset ${DRY_RUN}

Environment=RUST_LOG=info

Restart=always
RestartSec=2s

# Clean stop: the bridge catches SIGTERM and idles/disarms the drives before
# exiting. KillSignal is SIGTERM (systemd's default, set explicitly because the
# safe-shutdown depends on it); give the idle path time before SIGKILL.
KillSignal=SIGTERM
TimeoutStopSec=10

StandardOutput=journal
StandardError=journal
SyslogIdentifier=${SERVICE_NAME}

[Install]
WantedBy=multi-user.target
EOF
chmod 0644 "$UNIT_DST"

# ---------- enable + start ----------
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
  systemctl status  $SERVICE_NAME
  systemctl restart $SERVICE_NAME
  journalctl -u     $SERVICE_NAME -f

To remove the service:
  sudo $PROJECT_DIR/uninstall.sh
EOF
