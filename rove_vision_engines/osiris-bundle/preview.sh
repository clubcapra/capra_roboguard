#!/usr/bin/env bash
# Launch the Osiris preview UI (create feeds + watch live video with overlays).
# Needs only python3 + ffmpeg (both already required by the bundle).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$HERE/config.env" ] && source "$HERE/config.env"

API_PORT="${OSIRIS_BIND##*:}"
export OSIRIS_API="${OSIRIS_API:-http://localhost:${API_PORT}}"
export OSIRIS_RTSP="${OSIRIS_RTSP:-rtsp://127.0.0.1:${OSIRIS_RTSP_PORT:-8554}}"
export PREVIEW_BIND="${PREVIEW_BIND:-0.0.0.0:${PREVIEW_PORT:-8080}}"

echo "Preview UI:  http://localhost:${PREVIEW_BIND##*:}"
exec python3 "$HERE/preview/preview.py"
