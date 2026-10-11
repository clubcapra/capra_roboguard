# rove_sim

PyBullet simulation that **implements the robot's own service APIs**
(`HALo`, `rove_control_bridge`) against physics, so the autonomy
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

## Forest world and saved test scene

The forest world is configured in `config/world.yaml`; `scenes/forest_demo.json`
adds the saved spawn, friction map and demo object. The optimized fleet shares
that configuration across physics, LiDAR and camera workers.

Install Python runtime dependencies from `requirements-runtime.txt`. Linux also
needs the NVIDIA EGL driver, `g++`, FFmpeg and MediaMTX 1.21.2 (`tools/bin/mediamtx`). The
small native alpha filter compiles locally on first use and is cached; runtime
workers never fetch packages or assets. The Embree wheel version is pinned to
match the vendored public headers.

```bash
python -m pip install -r requirements-runtime.txt
SIM_PYTHON=/path/to/sim/venv/bin/python \
RSA_BACKEND=1 PORTS=../HALo/config/ports.toml \
SIM_VIEW_PASSWORD=choose-a-password VIEW_HOST=0.0.0.0 tools/sim_fleet.sh
```

Open `http://<server>:8081/` from the SSH client machine. Login with
`SIM_VIEW_PASSWORD`, drag to orbit and scroll to zoom. The browser receives the
same GPU renderer’s 800×600 live view; no desktop, VNC or second software
renderer is required. Hardware commands go through HALo. Camera streams are
`rtsp://<server>:8554/cam_front`, `cam_rear`, `cam_left`, `cam_right` and `cam_arm`.

`tools/rove.sh headless --api --scene <scene>` uses the same fleet and honors
the scene selection. Launchers stop only their own child processes.

Defaults are five **800×600 cameras at 30 fps**, plus both LiDARs at **30 Hz,
4,000 rays per scan per sensor** (120,000 emitted rays/s each). This is a measured
simulation budget, below a physical Mid-360’s nominal 200,000 points/s; the
nonrepetitive pattern accumulates coverage between scans. `FPS`, `RES`, `LHZ`,
`LRAYS`, `CAMERAS` and `LIDARS` override the budget. `WORLD_CONFIG`, `SCENE`,
`STATE_FILE`, `RTSP_PORT` and `PROFILE` select the instance. `METRICS` and
`LIDAR_METRICS` select the worker measurement files. `RTSP_SHARED=1` uses an
existing RTSP server; `WORKERS_ONLY=1` attaches cameras to existing physics.

LiDAR acquisition uses timestamped physics history with 50 ms of buffering,
linear position/joint interpolation and quaternion SLERP. The worker divides
its nominal scan budget into smaller batches at a 60 Hz target (`--motion-hz`);
30 Hz × 4,000 rays becomes 60 Hz × 2,000 rays per sensor. Actual throughput is
reported in the worker metrics. LVXR packets carry acquisition timestamps,
rather than their publication time. Stale physics pauses acquisition; respawns
are treated as discontinuities. The live overlay refreshes at up to 30 Hz.
Pose history is published separately from camera state to avoid extra parsing
in the camera render loop. `--pose-delay-ms` adjusts the interpolation buffer.

The GPU uploads original geometry/RGBA textures once, culls spatial draw ranges,
and uses double-buffered camera readback, mipmaps, 4× anisotropic colour filtering,
and linear-light environment illumination. `environment.hdri` in the world YAML
selects a local Radiance panorama, with `rotation_deg` and `strength` controls.
The panorama supplies the visible sky, nine precomputed diffuse-light
coefficients, and the direction/colour of the shadow-casting solar contribution.
A filtered
4096×4096 forest shadow map is baked once when the renderer starts and shared
by every camera; transparent foliage gaps also pass sunlight. Subtle distance
haze separates distant trees. Moving robot links and props receive terrain
shadows, but do not cast visual shadows; they are excluded from the static bake
to prevent stale silhouettes. No shadow geometry is redrawn per frame.
`SHADOW_SIZE=0` (or `gpu_worker.py --shadow-size 0`) disables the bake for comparisons.
See [lighting measurements and captures](doc/lighting.md). Texture alpha is
never flattened. Both cameras and LiDAR use repeat-wrapped bilinear coverage at
the material’s alpha cutoff; a transparent texel neither writes depth nor stops
a ray. Native Embree filtering lets rays continue inside the BVH, including
through deeply layered foliage. Moving-link BVHs preserve mast openings and
robot shadows; scene props follow authoritative physics. Physics workers load
only contact geometry, keeping camera/foliage copies out of Bullet.

Exported assets can be deployed without the source GLB. A missing source without
compatible exports fails visibly. The forest asset’s `ATTRIBUTION.md` records its
license. Use original downloaded textures, not web-viewer image derivatives:
those derivatives may have rearranged image blocks.
`tools/restore_terrain_textures.py <manifest> <source-image-directory>` imports
the local diffuse/opacity images recorded in `source_images`, applying the same
orientation to both. The selected orientation matches each mesh's UV convention: the six
restored layers use unflipped source UVs, while the older exports need flipped
images. GPU mipmaps weight RGB by alpha so transparent atlas padding does not
darken distant leaves. Imports check source hashes before writing; workers check
exported texture hashes on load. The original forest images are available in
the original model ZIP under `textures/`, or the author’s linked texture-packed
Blender file. The visual export retains all original materials, including ground
beneath transparent road layers; contact-mesh exclusions are separate. Robot
visuals and self-occlusion use URDF link frames rather than base inertial frames.

```bash
SIM_REQUIRE_GPU=1 python -m pytest tests/test_texture_coverage.py tests/test_forest_world.py -q
# From HALo/: discovery, commands and recovery against this same world.
python tests/test_sim_flow.py --split-lidar --browser \
  --world-config ../rove_sim/config/world.yaml --scene ../rove_sim/scenes/forest_demo.json
```

See [the measured performance report](doc/performance.md) for the P600 results.
The older `cam_worker.py`, `pyrender_worker.py` and desktop viewer remain legacy
entry points. Use `sim_fleet.sh` / `gpu_worker.py` for alpha-correct forest feeds.

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
- `robot_doc/ik_legacy/robot.urdf` is a stale demo topology — **not** used here.
```
