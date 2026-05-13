# ForgeBOT IK Engine — Exported Bundle

Self-contained, headless. Receives `Ovis` twist messages, runs the same
IK solver the editor uses, streams `StateUpdate` frames out.

## Run

```sh
pip install -r requirements.txt
python run.py
```

The engine listens on UDP `:9100` (Ovis in) and WebSocket `:9101/ovis`
(Ovis in) by default, and broadcasts `StateUpdate` on WebSocket
`:9101/state`. Change anything in `engine.toml`.

## Wire format

All messages are protobuf — schema in `engine/proto/messages.proto`.

`Ovis` (client → engine):
- `orientation.{yaw,pitch,roll}` — normalised to `[-1, 1]`
- `position.{x,y,z}` — normalised to `[-1, 1]`
- `target` — entity id of the joint or link the twist drives. The engine
  walks up the kinematic tree from `target` to find the IK base, then
  solves so `target` follows the integrated pose.

`StateUpdate` (engine → consumers, emitted at `rate_hz`):
- `joints[]` — per-joint `{id, q, qdot}`
- `ee` — current world pose of the latest Ovis target
- `diag` — solver iters, residuals, converged, `collision_hit`

## Config (`engine.toml`)

`[robot]` — path to `.forgebot` (preferred, lossless) or `.urdf` (fallback).

`[ik]` — `collision_aware` toggle, `twist_frame` (`"world"` or `"target"`),
`max_lin_vel` / `max_ang_vel` (scale factors for normalised Ovis), and the
tick rate.

`[input]` / `[output]` — enable UDP and/or WebSocket, configure bind /
target addresses. Multiple outputs can run simultaneously.

## What's vendored

The bundle includes a copy of `forgebot.core` and the bits of
`forgebot.io` needed to load `.forgebot` and `.urdf`. The IK solver is
called directly, so engine behaviour is bit-identical to what was tuned
in the editor.
