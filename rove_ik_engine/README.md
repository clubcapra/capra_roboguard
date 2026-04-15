# rove_ik_engine

Simulation layer for the rover — the **digital twin** of the robot and (over
time) of the world around it. Today it owns inverse kinematics for the arm.
Soon it will own flippers and drum/track conversion too, and then it will
fuse incoming lidar / camera detections into a live world model that the
autonomy layer reads against.

```
┌──────────────────────────┐        ┌─────────────────────────────┐
│  rove_control_bridge     │  UDP   │  rove_ik_engine             │
│  arm twist (Ovis)        │ ─────► │  - kinematic chain          │
└──────────────────────────┘        │  - IK solver (collision-aware) │
                                    │  - hardware sync (current)  │
                                    │  - flippers + tracks (next) │
                                    │  - lidar + camera fusion    │
                                    │    (planned)                │
                                    └──────────┬──────────────────┘
                                               │ StateUpdate (joint q, ee, diag)
                                               ▼
                                    ┌─────────────────────────────┐
                                    │  rove_sensor_api            │
                                    │  Kinova arm cmd port        │
                                    │  (and later: ODrives, etc.) │
                                    └─────────────────────────────┘
                                               │
                                               ▼
                                    ┌─────────────────────────────┐
                                    │  autonomy layer (planned)   │
                                    │  reads world model + state  │
                                    └─────────────────────────────┘
```

## Quick start

```sh
# Recommended on a Jetson — bootstraps venv, runs the engine
python3 run.py

# Install as a boot-time systemd service
sudo ./scripts/install.sh
```

`run.py` creates `.venv/` on first run, installs `requirements.txt`, then
re-execs under the venv. Skip the bootstrap with `FORGEBOT_NO_BOOTSTRAP=1`
if you manage deps yourself.

The engine listens on:
- UDP `:9100` — `Ovis` twist messages in
- WebSocket `:9101/ovis` — `Ovis` twist messages in (alternative)
- WebSocket `:9101/state` — `StateUpdate` broadcast out

Defaults are in `engine.toml`.

## Configurations (caged vs arm)

Two robot builds share this engine; pick one with the config file:

- **`engine.toml` — CAGED rover (the default the systemd service runs).** Loads
  `data/rove_caged.urdf` (drivetrain + a fixed cage, no Kinova arm), so
  `[hardware]` is off. Collision-aware IK/motion is on. Run: `python3 run.py`.
- **`engine.standard.toml` — arm build.** Loads `data/scene.forgebot` (the
  Kinova arm + drivetrain). Run: `python3 run.py engine.standard.toml`.

Each config keeps its own `sync_offsets.*.json` (offsets are keyed by entity id,
which differs per model). The standard build keeps the legacy `sync_offsets.json`.

**Safety posture.** Real motor output is gated by master switches that ship
**OFF**: `[drives].output_enabled` (flippers/drums) and
`[hardware].vel_output_enabled` (arm). With them off the engine mirrors and
visualises everything but sends no motor packets — flip them on only after
verifying mirror direction. The command paths refuse to drive on stale feedback
(no "ghost" motion on a dropped link), drum and flipper command channels are
fully segmented, and `POST /api/v1/reload` re-discovers ports + re-anchors the
flippers after an e-stop/sensor_api reload (the arm e-stop watchdog calls it on
recovery) so the engine self-heals instead of needing a reinstall.

## Module layout

```
rove_ik_engine/
├── run.py              Bootstrap + entry point (venv self-install)
├── engine.toml         Runtime config (robot, IK, transports, hardware sync)
├── requirements.txt    Pinned runtime deps
├── engine/             The simulation runtime
│   ├── ik_loop.py      Tick: pull latest Ovis, solve IK, emit StateUpdate
│   ├── chain.py        Walk the kinematic tree, derive the IK base/tip
│   ├── state.py        Engine state container (joint q, last Ovis, diag)
│   ├── loader.py       Load .forgebot or .urdf into a kinematic model
│   ├── config.py       Read + validate engine.toml
│   ├── server.py       UDP + WebSocket transports orchestration
│   ├── tcp.py          (legacy) raw-TCP transport
│   ├── hardware.py     Sync joint state to/from rove_sensor_api (Kinova)
│   ├── transports/     UDP / WS handlers
│   └── proto/          Generated protobuf (Ovis, StateUpdate)
├── forgebot/           Vendored copy of forgebot.core + needed forgebot.io
│                       (so the bundle is self-contained and bit-identical
│                        to the editor's IK behaviour)
├── data/               Scene + meshes
│   ├── scene.forgebot  Preferred — lossless export from the editor
│   ├── robot.urdf      Fallback URDF
│   ├── meshes/
│   └── manifest.json   Joint/link ids advertised at build time
├── ui/                 Bundled visualiser (HTML + assets)
└── scripts/            Ops glue
    ├── install.sh      systemd: rove-ik-engine.service
    └── uninstall.sh
```

## Wire format

All messages are protobuf — schema in `engine/proto/messages.proto`.

`Ovis` (client → engine):
- `orientation.{yaw,pitch,roll}` — normalised to `[-1, 1]`
- `position.{x,y,z}` — normalised to `[-1, 1]`
- `target` — entity id of the joint/link the twist drives. The engine walks
  up the kinematic tree from `target` to find the IK base, then solves so
  `target` follows the integrated pose.

`StateUpdate` (engine → consumers, emitted at `rate_hz`):
- `joints[]` — per-joint `{id, q, qdot}`
- `ee` — current world pose of the latest Ovis target
- `diag` — solver iters, residuals, converged, `collision_hit`

## Config (`engine.toml`)

- `[robot]` — paths to `.forgebot` (preferred) or `.urdf` (fallback).
- `[ik]` — `collision_aware` toggle, `twist_frame`, velocity caps, tick rate.
- `[input]` / `[output]` — enable UDP and/or WebSocket, configure bind
  addresses. Multiple outputs can run simultaneously.
- `[hardware]` — sync joint state with the Kinova arm via rove_sensor_api.
  `vel_output_enabled` closes the loop (gizmo → IK → qdot → arm).

## Roadmap

- **Now**: arm IK (collision-aware), live Kinova mirror.
- **Next**: own flippers + drums conversion — `rove_control_bridge` will
  hand those off to the engine, since the engine has the world model the
  bridge doesn't.
- **Then**: ingest lidar (point cloud) and camera detections, maintain a
  live digital twin of the rover's surroundings. The autonomy layer reads
  this twin (joint state + world model) to plan motion.

## What's vendored

`forgebot.core` and the slice of `forgebot.io` needed to load `.forgebot`
and `.urdf`. The IK solver is called directly, so engine behaviour matches
what was tuned in the ForgeBOT editor.
