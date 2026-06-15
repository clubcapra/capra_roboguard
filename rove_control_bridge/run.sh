#!/usr/bin/env bash
# Start the rove_control_bridge against the real robot.
#
#   ./run.sh                 # build + run (real robot)
#   ./run.sh --dry-run       # log intended commands, don't actuate
#   ./run.sh --config config/autonomy.toml   # override the config (e.g. sim)
#
# Any extra args are forwarded verbatim to the binary. Ctrl-C stops it cleanly
# (the bridge idles + disarms on shutdown).
#
# Prereqs for motion: rove_sensor_api up on :8080 (the bridge hits /discover at
# startup and exits if it's down) and the rove_ik_engine running (tracks/flippers/
# ovis are forwarded to it; the gripper goes straight to the API).
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

# Default to the real-robot config (host 192.168.2.2, front-door ports 5050-5053)
# and skip the sim respawn. A caller-supplied --config/--dry-run/etc. still wins
# because it appears later on the command line.
ARGS=(--config config/real.toml --no-reset)

cargo build --release
exec ./target/release/rove_control_bridge "${ARGS[@]}" "$@"
