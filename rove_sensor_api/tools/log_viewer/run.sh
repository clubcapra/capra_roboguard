#!/usr/bin/env bash
# One-shot launcher for the rove log viewer.
#
# - Bootstraps a local Node toolchain into .node/ if `node` isn't on PATH or
#   is older than NODE_MAJOR. No sudo required.
# - Installs npm dependencies into node_modules/ if missing or out of date.
# - Starts the Vite dev server + Express/UDP backend (npm run dev).
#
# Usage:  ./run.sh            (run from anywhere)
#         ./run.sh build      (production build → dist/)
#         ./run.sh clean      (wipe .node/ and node_modules/)

set -euo pipefail

NODE_VERSION="20.14.0"
NODE_MAJOR_MIN=20

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

# ---------------------------------------------------------------------------
# Node bootstrap
# ---------------------------------------------------------------------------
detect_arch() {
    case "$(uname -m)" in
        aarch64|arm64)  echo "linux-arm64" ;;
        x86_64|amd64)   echo "linux-x64" ;;
        armv7l)         echo "linux-armv7l" ;;
        *) echo "unsupported-arch:$(uname -m)" >&2; return 1 ;;
    esac
}

have_usable_node() {
    command -v node >/dev/null 2>&1 || return 1
    local major
    major=$(node -p 'process.versions.node.split(".")[0]' 2>/dev/null) || return 1
    [[ "$major" -ge "$NODE_MAJOR_MIN" ]]
}

bootstrap_node() {
    local arch tarball url cache_dir
    arch=$(detect_arch)
    cache_dir="$HERE/.node"
    local node_dir="$cache_dir/node-v${NODE_VERSION}-${arch}"

    if [[ -x "$node_dir/bin/node" ]]; then
        export PATH="$node_dir/bin:$PATH"
        return
    fi

    tarball="node-v${NODE_VERSION}-${arch}.tar.xz"
    url="https://nodejs.org/dist/v${NODE_VERSION}/${tarball}"

    echo "==> Downloading Node v${NODE_VERSION} (${arch})"
    mkdir -p "$cache_dir"
    cd "$cache_dir"
    if command -v curl >/dev/null 2>&1; then
        curl -fL -o "$tarball" "$url"
    elif command -v wget >/dev/null 2>&1; then
        wget -O "$tarball" "$url"
    else
        echo "error: need curl or wget to bootstrap Node" >&2
        exit 1
    fi
    echo "==> Extracting"
    tar -xJf "$tarball"
    rm -f "$tarball"
    cd "$HERE"
    export PATH="$node_dir/bin:$PATH"
}

if have_usable_node; then
    echo "==> Using system node $(node --version)"
else
    bootstrap_node
    echo "==> Using local node $(node --version)"
fi

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
needs_install=0
if [[ ! -d node_modules ]]; then
    needs_install=1
elif [[ package.json -nt node_modules/.package-lock.json ]]; then
    needs_install=1
fi

if [[ "$needs_install" -eq 1 ]]; then
    echo "==> Installing npm dependencies"
    npm install --no-audit --no-fund --loglevel=error
fi

# ---------------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------------
cmd="${1:-dev}"
case "$cmd" in
    dev)
        echo "==> Starting log viewer"
        echo "    UI:      http://localhost:5173"
        echo "    backend: http://localhost:${LOG_VIEWER_API_PORT:-8765}"
        echo "    LOG_DIR: ${LOG_DIR:-/var/log/rove-sensor-api}"
        exec npm run dev
        ;;
    build)
        exec npm run build
        ;;
    clean)
        rm -rf .node node_modules dist
        echo "cleaned .node/, node_modules/, dist/"
        ;;
    *)
        echo "usage: $0 [dev|build|clean]" >&2
        exit 2
        ;;
esac
