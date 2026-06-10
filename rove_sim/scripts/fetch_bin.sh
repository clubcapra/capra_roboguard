#!/usr/bin/env bash
# Fetch the runtime binaries the sim needs that are too large to keep in git
# (tools/bin/* is git-ignored). Run once after a fresh clone, on each machine.
#
#   - mediamtx : RTSP server for the camera streams. No system package; ALWAYS
#                fetched into tools/bin (the fleet launches it by absolute path).
#   - ffmpeg   : H.264 push for the camera workers. Only fetched (static build)
#                if there's no `ffmpeg` already on PATH (e.g. system install).
#
#   scripts/fetch_bin.sh
set -euo pipefail

HERE="$(cd "$(dirname "$0")/.." && pwd)"   # .../rove_sim
BIN="$HERE/tools/bin"
mkdir -p "$BIN"

# ── mediamtx (always; no system package) ──────────────────────────────────────
if [ -x "$BIN/mediamtx" ]; then
  echo "mediamtx already present: $("$BIN/mediamtx" --version 2>&1 | head -1)"
else
  echo "fetching latest mediamtx (linux_amd64)…"
  url=$(wget -qO- https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
        | grep -oE '"browser_download_url": *"[^"]*linux_amd64\.tar\.gz"' \
        | grep -oE 'https://[^"]+' | head -1)
  [ -n "$url" ] || { echo "could not resolve mediamtx release URL" >&2; exit 1; }
  echo "  $url"
  wget -qO /tmp/mediamtx.tar.gz "$url"
  tar -C "$BIN" -xzf /tmp/mediamtx.tar.gz mediamtx
  chmod +x "$BIN/mediamtx"
  echo "  installed $("$BIN/mediamtx" --version 2>&1 | head -1)"
fi

# ── ffmpeg (only if not already available on PATH) ────────────────────────────
if command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg already on PATH: $(command -v ffmpeg) ($(ffmpeg -version 2>/dev/null | head -1 | awk '{print $3}'))"
elif [ -x "$BIN/ffmpeg" ]; then
  echo "ffmpeg already present in tools/bin"
else
  echo "fetching static ffmpeg (no system ffmpeg found)…"
  wget -qO /tmp/ffmpeg.tar.xz https://johnvansickle.com/ffmpeg/releases/ffmpeg-release-amd64-static.tar.xz
  d=$(tar -tf /tmp/ffmpeg.tar.xz | head -1)   # top-level dir, e.g. ffmpeg-7.0.2-amd64-static/
  tar -C /tmp -xf /tmp/ffmpeg.tar.xz
  cp "/tmp/${d}ffmpeg" "/tmp/${d}ffprobe" "$BIN/"
  chmod +x "$BIN/ffmpeg" "$BIN/ffprobe"
  echo "  installed static ffmpeg + ffprobe in tools/bin"
fi

echo "tools/bin ready: $(ls "$BIN" | tr '\n' ' ')"
