# capra_autonomy

Autonomy engine for the Capra **Roboguard**. It reads the robot's sensors through
`rove_sensor_api` and drives the robot through the same wire seams — so it runs
unchanged against the **sim** (think2, sim-backed `rove_sensor_api`) or the **real
robot** (Jetson). Only `robot_host` in the config changes.

This is the **M6–M9 autonomy layer**. The first slice is a vertical **GoTo**:

```
VectorNav (UDP) ─► PositionService ─► Pose (ENU about Thun datum)
                                         │
   waypoint  ─► Router (mode SM) ─► GoTo ─► track intent
                                         │
                          control/tracks ─► {axis_state:8, input_vel} ─► ODrive cmd ports
                                                                          (rove_sensor_api)
```

## Architecture

| Module | Role |
|---|---|
| `transport/` | rove_sensor_api UDP codec, `/discover`, telemetry subscribe, command send |
| `position/` | VectorNav → local-ENU `Pose` (geodetic conversion about the Thun datum) |
| `router/` | mode state machine (Idle/Auto/Estop) + waypoint queue |
| `behaviors/goto.rs` | heading + distance control law → normalised track intent |
| `control/tracks.rs` | intent → per-ODrive velocity commands + arm/idle (port of `rove_control_bridge`) |
| `validate/` | ground-truth cross-check (scoring only, never in the control path) |

## Run

```sh
# safe first: log intended commands, don't move the robot
cargo run --release -- --dry-run

# live drive against the configured robot_host (default 192.168.2.4 = think2 sim)
cargo run --release

# tests
cargo test
```

Config lives in [config/autonomy.toml](config/autonomy.toml) — robot host, Thun
datum, drive gains, track node map, and the waypoint mission. With no explicit
waypoints, `[mission].demo_forward_m` drives a few metres straight ahead of the
start heading (the safest bring-up).

Stop any time with Ctrl-C — the engine idles and disarms every track ODrive.

## Scope

In: pose, GoTo mobility, track control, convergence validation. Out (next
milestones): L0 reflex engine (M7), GNSS-denied → SLAM fusion (M8), mission
compiler + teleop arbitration (M9), arm/flippers/gripper via Python IK, and the
vision/perception feed (lands with the vision bundle).
