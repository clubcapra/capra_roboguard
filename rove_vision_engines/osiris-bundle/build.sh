#!/usr/bin/env bash
# Build the Osiris API from the bundled source (fallback when the prebuilt
# binary does not match this host).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v cargo >/dev/null 2>&1; then
  [ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
fi
command -v cargo >/dev/null 2>&1 || {
  echo "cargo (Rust toolchain) is required: https://rustup.rs"
  exit 1
}

cd "$HERE/controllers"
cargo build --release
cp -f "$HERE/controllers/target/release/osiris-controller" "$HERE/bin/osiris-api"
echo "Built bin/osiris-api"
