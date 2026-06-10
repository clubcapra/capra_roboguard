# rove_sim

PyBullet simulation that **implements the robot's own service APIs**
(`rove_sensor_api`, `rove_control_bridge`) against physics, so the autonomy
stack is written once and switched between sim and the real robot with a single
`mock`/`real` flag. See `SAR_Sim_Build_Plan.md` for the full spec.

## Composition model

The sim is **not** hardcoded around one robot. A reusable **component library**
(sensors, actuators, API-seam adapters) is wired together by a per-hardware
**profile manifest** in `profiles/`. A profile imports components and binds them
to URDF links; capabilities are *derived* from what it declares.

```
profiles/standard.yaml   # mobility + 6-DOF arm + dual Mid-360 + 7 cameras
profiles/caged.yaml      # mobility + cage + dual Mid-360 + 5 cameras, NO arm
```

A new robot = a new profile (and, if its wire protocol differs, a new adapter in
`rove_sim/api/`). Autonomy reads `capabilities.derive(profile)` — e.g. a profile
with no `provides: gnss` sensor yields `has_gnss == False`, so PositionService
runs SLAM-only with no robot-specific branch.

## Run

```bash
# headless (CI / GPU box); EGL camera render, falls back to TinyRenderer
rove_sim_venv/bin/python -m rove_sim.main --profile standard --mode headless
rove_sim_venv/bin/python -m rove_sim.main --profile caged    --mode gui --hold

rove_sim_venv/bin/python -m pytest tests/ -q
```

## Headless GPU rendering (EGL)

Headless camera rendering uses the **EGL surfaceless** path on the GPU
(`renderer=egl`), falling back to CPU TinyRenderer only if no NVIDIA EGL device
is found. `Engine` handles this automatically:

- auto-detects the NVIDIA glvnd `egl_vendor.d` and exports
  `__EGL_VENDOR_LIBRARY_DIRS` (driver-injected sandboxes put it off the default
  search path, so libEGL otherwise sees **0** devices); and
- loads pybullet's plugin with the correct `_eglRendererPlugin` symbol (the
  wrong name loads the `.so` but "couldn't bind functions" → silent CPU
  fallback).

Verified on RTX 2060 Mobile (driver 580.159.03): `GL_RENDERER = NVIDIA GeForce
RTX 2060`, ~31 fps @ 640×480 vs ~10 fps on TinyRenderer.

```bash
PYTHONPATH=. rove_sim_venv/bin/python tools/snapshot.py --profile standard --out /tmp/r.png
```

## Status

- **M0 (done):** both profiles load (GLB→OBJ, primitive collision overrides,
  role-based masses), rest on the ground stably, capabilities derived.
- **Headless GPU (done):** EGL on RTX 2060 confirmed; auto-configured in Engine.
- **M1 (done):** scripted `RoveControl` drives the robot, articulates flippers,
  moves the arm via IK. 100 kg; tracks modeled as injected driven road-wheels
  (`track_wheels`) for stable skid-steer; real ODrive budget (7.2 A ≈ 50 N·m
  peak, 125.7 rad/s). 8 tests pass. `tools/drive_demo.py` for a visual run.
- Next: flipper treads (optional), M2 cameras, M3 physics-derived telemetry
  (current calibrated to ~1.2 A straight / ~5 A turn), M4 imperfect IMU/GNSS.

## Locomotion model notes

The drums are real-world **tracks**, not wheels. Two drum cylinders per side
can't scrub-turn (point contact), so `loader._inject_track_wheels` adds a row of
driven road-wheel cylinders between the front/rear drum of each side
(`DrumW_<L|R>i`, continuous joints), giving a distributed, stable contact patch.
Drum joints are forced `continuous` (the URDF's ±π drum limits are stale).
Friction is anisotropic (low lateral) for skid-steer. Turning is slow and
high-slip **by design** — the real robot also forces hard (~5 A) to point-turn
on carpet; the narrow track / long wheelbase makes yaw geometrically weak.

## Layout

```
profiles/            per-hardware manifests ("project format")
config/              world (terrain + GNSS datum), and later sensors/noise/gnss
rove_sim/
  core/engine.py     PyBullet connect: GUI | headless(DIRECT) + EGL plugin
  robot/loader.py    URDF load, GLB→OBJ, collision overrides, mass + joint maps
  robot/profile.py   manifest -> composition spec
  robot/actuation/   actuator library (registry) -- intent -> joint targets
  sensors/           sensor library (registry) -- template-method + error model
  api/               API-seam adapters (registry) -- swappable wire contracts
  capabilities.py    capability set derived from a profile
tools/convert_meshes.py   GLB->OBJ + URDF rewrite (cached)
```

## Notes / caveats

- `URDF_MAINTAIN_LINK_ORDER` segfaults pybullet 3.2.7 on these URDFs; omitted.
  Everything keys off link **name**, so link order is irrelevant.
- URDFs carry no inertials and stale joint limits (placeholder `±π`, `effort=10`).
  Masses are role-based defaults in the loader; limits/`max_track_rad_s` are
  tunable config, all tagged `TODO(calibrate)`.
- `rove_ik_engine/data/robot.urdf` is a stale demo topology — **not** used here.
```
