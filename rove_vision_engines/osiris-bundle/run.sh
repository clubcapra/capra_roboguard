#!/usr/bin/env bash
# Launch the Osiris streaming API bundle.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Load configuration (API bind + gateway ports). Edit config.env to change them.
[ -f "$HERE/config.env" ] && source "$HERE/config.env"

export OSIRIS_ENGINES_DIR="$HERE/engines"
export OSIRIS_BIND="${OSIRIS_BIND:-0.0.0.0:9092}"
RTSP_PORT="${OSIRIS_RTSP_PORT:-8554}"
WEBRTC_PORT="${OSIRIS_WEBRTC_PORT:-8889}"
export OSIRIS_WEBRTC_GATEWAY="${OSIRIS_WEBRTC_GATEWAY:-rtsp://127.0.0.1:${RTSP_PORT}}"

# Free our ports from any previous run (this bundle, another bundle, or a stale
# instance) so we don't hit "Address already in use" on the API or the gateway.
API_PORT="${OSIRIS_BIND##*:}"
if command -v fuser >/dev/null 2>&1; then
  fuser -k "${API_PORT}/tcp" 2>/dev/null || true
elif command -v ss >/dev/null 2>&1; then
  ss -ltnpH "sport = :${API_PORT}" 2>/dev/null | grep -oP 'pid=\K[0-9]+' | xargs -r kill 2>/dev/null || true
fi
# Stop any stale MediaMTX gateway (frees its ports).
pkill -f mediamtx 2>/dev/null || true
sleep 1

# Start the WebRTC (WHIP) -> RTSP gateway if present, on the configured ports.
if [ -x "$HERE/gateway/mediamtx" ]; then
  ( cd "$HERE/gateway" &&     MTX_RTSPADDRESS=":${RTSP_PORT}" MTX_WEBRTCADDRESS=":${WEBRTC_PORT}"     ./mediamtx mediamtx.yml ) &
  GW_PID=$!
  trap 'kill $GW_PID 2>/dev/null || true' EXIT
  echo "Gateway: WHIP http://localhost:${WEBRTC_PORT}/<stream>/whip -> rtsp://127.0.0.1:${RTSP_PORT}/<stream>"
fi

EXPECT_OS="linux"
EXPECT_ARCH="x86_64"
CUR_OS="$(uname -s | tr '[:upper:]' '[:lower:]')"
CUR_ARCH="$(uname -m)"
BIN="$HERE/bin/osiris-api"

if [ -x "$BIN" ] && [ "$CUR_OS" = "$EXPECT_OS" ] && [ "$CUR_ARCH" = "$EXPECT_ARCH" ]; then
  echo "Starting Osiris API on $OSIRIS_BIND (docs at /docs)"
  exec "$BIN"
else
  echo "Prebuilt binary not compatible ($CUR_OS/$CUR_ARCH vs $EXPECT_OS/$EXPECT_ARCH)."
  echo "Building from bundled source..."
  "$HERE/build.sh"
  echo "Starting Osiris API on $OSIRIS_BIND (docs at /docs)"
  exec "$HERE/controllers/target/release/osiris-controller"
fi
