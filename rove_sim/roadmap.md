# ROVE sim — roadmap

PyBullet simulation that implements the robot's own service APIs against physics,
so the autonomy stack is written once and switched between **mock** (sim) and
**real** (live robot) with one flag. Full spec: `SAR_Sim_Build_Plan.md`.
Deadline: ELROB 15–19 June 2026 (Thun).

_Last updated: 2026-06-09._

---

## Architecture at a glance

Two run modes, **orthogonal** to the engine connection mode (`gui|headless`):

| `--world` | Robot state | World | Sensors | Use |
|---|---|---|---|---|
| **mock** | PyBullet **physics** (actuators apply forces) | ground + terrain + friction | synthesized from physics | dev / autonomy testing |
| **real** | **synced kinematically** from `rove_sensor_api` telemetry (no physics) | perceived objects (lidar/vision → bodies) | real feeds | on-robot world model |

Seams (all registry-driven from the profile manifest):
`world/` (MockWorld·RealWorld) · `drivers/` (MockDriver·SyncDriver) ·
`state/` (RobotStateSource) · `api/` (SimSensorApi) · `transport/` (inproc·UDP) ·
`sensors/` · `robot/actuation/` · `robot/ik/`.

**Rendering**: hardware OpenGL via EGL on the NVIDIA GPU (confirmed RTX 2060).
PyBullet physics is CPU-only (no CUDA/DX12 — see `memory/pybullet-no-cuda-physics`).
On the Jetson, **real mode is kinematic** (no physics); tensor cores run perception.

---

## Status

### ✅ Done

- **M0 — load & rest.** Both profiles (`standard`, `caged`) load from URDF (GLB→OBJ
  + collision-primitive overrides), 100 kg, rest stable. Headless GPU (EGL) render.
- **M1 — actuation.** Drive (brush/contact-surface model, 15 km/h, clean point-turn,
  arc), stepped flippers (tippy-toe lift), 6-DOF arm via the production `rove_ik_engine`
  solver, Robotiq 2F-140 gripper, self-collision guard (FCL real meshes), pinch-TCP
  arm control, pose store + joint-space path planning. Scripted `RoveControl`.
- **Mock/real seam (the §1–2 backbone).** World + Driver + RobotStateSource seams;
  `MockDriver` = physics (verbatim), `SyncDriver` = kinematic `resetJointState`.
  Validated: kinematic twin tracks the physics robot with **0.0 rad** joint error.
- **Telemetry loop ("the API is the seam").** Mock `SimSensorApi` publishes per-driver
  telemetry (kinova/odrive/robotiq/vectornav) in the real `rove_sensor_api` wire format
  (`<BBH>`+JSON); `RoveSensorApiStateSource` consumes the same bytes from the mock OR
  the real robot. In-process + UDP transports. Offline mock→real loop works (no hardware).
- **Terrain.** `free_dirt_road_through_forest.glb` loads as the FULL scene, mesh-first:
  ground-only concave collider (robot drives + spawns on a flat road patch) + per-
  **material** visual bodies for the whole environment (ground/grass/rock/trunks/foliage).
  **Textured** (`terrain.texture: true`, default on in `live.py`): converter v5 exports
  each of the 20 kept materials as a textured OBJ+PNG (UVs preserved; the 6 billboard/
  overlay cards — aerial/decal/puddle/far — are dropped); `MockWorld` z-staggers the
  near-coplanar ground layers by a few mm to kill z-fighting (dry-grass tearing through
  road). Flat-colour fallback via `--no-texture`.
- **Friction painting.** `FrictionField` raster; the brush-track model looks up μ per
  ground contact (robot slips on painted ice, grips on gravel). GUI brush-paint
  (top-down, material palette, save/load, overlay). Robot slip validated (ice → 28% traction).
- **Tooling.** GUI panel (Qt/Wayland, drive + live μ sliders + arm/flippers + friction
  paint + `--terrain`, renders at panel resolution), `render_clip`, `twin_demo`,
  `friction_demo`, `convert_terrain`, `tune`, `viewer`, `snapshot`. 17 tests pass.
- **Engine perf (for native 60 fps + Jetson headroom).** Two optimizations, both
  validated (17 tests pass, top-down drive/point-turn render unchanged):
  **(A)** solver tuning — `numSolverIterations` 50→**20** (`EngineConfig.solver_iterations`,
  profile-overridable via `world.solver_iterations`); no measurable change to drive
  distance / roll / pitch / pivot drift / point-turn yaw (10 weakens the marginal
  narrow-gauge turn, so 20 is the floor). **(B)** vectorized the brush-track `step()`
  hot loop (per-contact numpy → batched arrays; dropped the `np.cross`/clip churn) —
  byte-identical forces. Result: physics realtime **flat 1.07→2.28×, terrain 2.63→4.84×**.

- **Native operator window (`live.py`).** PyBullet GUI mode, no GPU readback → true
  60 fps with textured terrain. On-screen controls (debug-param panel): DRIVE / STEER
  throttle sliders (mouse-only driving, 0 = stop), Flippers DOWN/UP + STOP buttons;
  keyboard still works (WASD/QE/arm). Runs on the **host** desktop via `tools/live_host.sh`
  (host is python3.14, the venv is 3.13 → wrapper runs `/usr/bin/python3.13` against the
  venv site-packages; `sudo dnf install python3.13` once).

### 🔜 Next (sim — M2–M5)

- **Scene save/load/sync — ✅ DONE.** `world/scene.py`: a `Scene` manifest (terrain ref +
  inline friction raster + robot base pose/joints + `SceneObject` list) with JSON
  save/load, `capture_scene`/`apply_scene`, and `load_scene_sim`. `apply_scene` upserts
  objects by id, so the same call is both LOAD and a cross-process SYNC tick (the dict is
  what you'd ship over a transport). `MockWorld.spawn_object/remove_object` materialise
  obstacles/SAR targets (the mock twin of perception.Detection). 3 tests.

### ✅ M2/M3/M4 — the bilateral robot interface (sim_server)

The sim now exposes the EXACT robot wire seams, so the autonomy stack + vision
model hook up unchanged (prod only swaps the endpoint). One entry point ties it
together: **`tools/sim_server.py`** (host: `tools/sim_host.sh --profile standard
--terrain`). End-to-end validated: telemetry received, a UDP control command drove
the robot 11.4 m, a point cloud + camera frame were read by external clients. 27 tests.

- **M2 camera RTSP** (`sensors/rtsp.py`): mediamtx (`tools/bin/mediamtx`) + one ffmpeg
  per camera; `rtsp://host:8554/<cam>` H.264. 720p render, downscaled (default 640×360).
- **M3 bilateral rove_sensor_api**: `SimSensorApi` publishes vectornav/kinova/robotiq +
  **4× ODrive** (physics current from the track-force model, I²R thermal temp, vel,
  stuck flag) + **pmic** (SoC/voltage from a drained `Battery`). `RoveControlBridge`
  (`rove_control_bridge`) decodes control over UDP → `RoveControl` (autonomy drives the
  sim like the robot). UDP channel ports added (control 5020, pmic 5014, livox 5022/24,
  ground_truth 5030); a sub can `subscribe=[...]` a subset.
- **M4 sensors**: Livox **PointCloud over UDP** (binary frame, `encode/decode_cloud`;
  now `LVX2` **multi-packet — full cloud, reassembled by frame id**, see the Deployed
  section). **GNSS modes** on the VectorNav (nominal/degraded/denied/
  spoofed — denied drops fix, spoofed creeps the position). **IMU error model** on VN-300
  (`_apply_errors`: random-walk bias + white noise; `errors:false` = ground truth). True
  pose on the `ground_truth` channel for scoring.

- **M2 — camera sensor + RTSP plane.** ✅ Camera `Sensor` done (`sensors/camera.py`):
  per-mount `computeViewMatrix` from the link's optical axis + FOV intrinsics, EGL render
  → RGB + metric depth + segmentation, horizon-levelled by default. 720p sim feeds (real
  is 2048×1536). `cam_arm` rides the arm link. Sensors are built in `runtime` and
  sampled on demand (`sim.sensor(name).sample()`); continuous stepping is opt-in
  (`build(step_sensors=True)` — cameras are too costly to auto-render every tick). ⏳ TODO:
  RTSP emit for `rove_vision_engine`.
- **Lidar (Livox Mid-360) — ✅ done** (`sensors/lidar.py`): non-repetitive golden-angle
  rosette → `rayTestBatch` → point cloud (range/hit-id), live height-coloured cloud in
  `live.py --sensors`. **Foliage/trunk collision live** (`world/mock.py`, default-on):
  trunks promoted to HARD colliders (robot + lidar), foliage to a SOFT cutout collider
  (lidar returns through the leaf gaps, robot passes through) — the gap-through / leaf-hit
  model. Self-occlusion via the synced concave pole mesh. IMU + multi-packet UDP done.
- **M3 — `SimSensorApi` for autonomy.** Promote the telemetry publishers to the full
  physics-derived stream L0 consumes (ODrive current/temp from `appliedJointMotorTorque`,
  pmic/battery integration, stuck detection). Already wire-compatible.
- **M4 — imperfect sensors + GNSS spoofing.** VN300 + Livox IMU error models (bias/
  random-walk/scale/noise), GNSS modes nominal/degraded/denied/adversarial. Livox
  Mid-360 rosette raycast → PointCloud UDP. Ground-truth debug channel (scoring only).
- **M5 — `SimControlBridge` over real transport.** Close the teleop loop end-to-end;
  terrain course. (Terrain + friction backend already in place.)

### ✅ M5 — sim backs the REAL rove_sensor_api binary (2026-06-09)

The genuine `rove_sensor_api` Rust binary now runs **sim-backed**: set
`ROVE_SIM_BACKEND=host:5000` (or drop a `config/sim.toml`) and every hardware
driver is replaced by a **per-sensor mock** (`rove_sensor_api/src/drivers/*/mock.rs`,
each reusing its real driver's schema) fed by the sim over UDP. Autonomy/IK then
connect to a byte-identical `rove_sensor_api` — indistinguishable from the robot.

- **Sim side**: `sim_server --rsa-backend` publishes telemetry on the shared
  **backend ports (6000+)** the mocks subscribe to (control stays on 5020). The
  VectorNav now emits the **full INS record** (working lat/lon ENU→geodetic about
  the Thun datum, velocity NED, body gyro/accel, sats/fix/uncertainty/pressure);
  the ODrive payload was renamed to the real `OdriveNodeState` schema so the mock
  is a verbatim passthrough.
- **Livox Mid-360 IMU**: emitted by the SIM as **native Livox-SDK2 UDP packets**
  (port 56401, 200 Hz, gyro rad/s + accel g, lever-arm applied), whenever the
  lidar runs — a **separate** lidar stream alongside the point clouds (5022/5024).
  It is **not** a rove_sensor_api sensor: that API only covers Pi-board hardware
  (VectorNav/Kinova/Robotiq/ODrives, 7 mocks); lidars are consumed directly.
- **Shared `ports.toml`** (`rove_sensor_api/config/ports.toml`) is the single
  source of truth read by both sides — no port-overlap drift.
- **Verified live**: `rove.sh headless --api` → `/discover` lists 8 sim-backed
  sensors; vectornav serves Thun lat/lon + 1 g; driving via `POST /odrive_3x/command`
  (input_vel) moves the robot; gripper position round-trips. 31 sim tests + 8 Rust
  tests pass.
- **Scene tooling**: `tools/scene_cli.py` (scriptable new/add-object/remove/info/
  validate/merge), `tools/scene_editor.py` (interactive 3D place/move/delete +
  save/export), and `live.py` gained SAVE/EXPORT/SPAWN/DELETE buttons + N/X keys.
- **One launcher**: `tools/rove.sh {headless|gui} [--api] [--profile] [--scene]`.

### ✅ Deployed on think2-gpu-host as a service (2026-06-09)

The full stack now runs **headless as a systemd service** on the dedicated sim
box (`think2-gpu-host`: Quadro **P600 / 2 GB**, 8-vCPU VM, Debian 13), so autonomy
can develop against a live `rove_sensor_api` without anyone babysitting a terminal.

- **Service**: `rove_sim/scripts/install_service.sh` (sudo) installs a boot-persistent
  unit running `tools/rove.sh headless --tiny --api` → physics + 5 cameras (RTSP
  :8554) + 2 Livox (UDP 5022/5024) + the real `rove_sensor_api` (HTTP :8080, sim-backed).
  `uninstall_service.sh` removes it. Runs as the project-owner user (EGL/GPU + venv).
- **Lidar at real fidelity, multi-packet**: the point-cloud UDP transport is now
  **`LVX2` multi-packet** — each Mid-360 scan is fragmented across N datagrams and
  reassembled by frame id (`CloudReassembler`), delivering the **full cloud, no
  decimation** (supersedes the old single-datagram `LVX1` 4000-pt cap). Both Mid-360s
  raycast in **separate worker processes** (`LIDAR_SPLIT=1`). Operating point tuned
  for this host: **4000 rays/scan + 3-thread cap → both sensors hold the real 10 Hz
  while physics stays 0.95–0.98× realtime** (raycast is CPU-bound, not GPU; full table
  in `tools/sim_fleet.sh`). `--tiny` constrains only the cameras (576×432 @24fps, 128px
  tex) to fit the 2 GB GPU; it does NOT touch the lidar.
- **ODrive mock completed — faithful fw 0.6.11 drive-state machine** (`drivers/odrive/
  mock.rs`): the mock now OWNS `axis_state` (the sim hardcodes 8), so **IDLE/CLOSED_LOOP/
  error** behave correctly — `input_vel` actuates only while armed, ESTOP latches
  `ESTOP_REQUESTED` and disarms, arming is refused until `clear_errors`, and
  `procedure_result`/`active_errors` are reflected in telemetry. Drive over the live
  `odrive_test.py` web UI (per-node arm/idle/clear/estop + vel/pos sliders).
- **Portable across machine moves**: two baked source-machine absolute paths that
  broke the stack on copy are now rebased to the local tree at load — terrain manifest
  (`world/mock.py`) and the robotiq gripper STLs (`robot/loader.py`). `ffmpeg` is
  bundled (static, `tools/bin/`) and put on PATH by the launchers — no system install.

### 🧭 Later (autonomy — M6–M9, likely the Rust repo, against the sim's planes)

- **M6** Command Router / mode SM / mission dispatch (WP-D).
- **M7** L0 ReflexEngine (thermal/pitch/stuck/current) on physics-derived telemetry.
- **M8** PositionService: GNSS-denied → SLAM holds; GoTo converges on fused pose.
- **M9** Mission-sequence compiler runs a seed mission via the orchestrator.

---

## Known gaps / deferred

- **Real-mode calibration**: kinova↔URDF joint order/signs/zero-offset + GNSS datum
  default to identity (offline loop round-trips exactly) — calibrate vs hardware before
  live wiring.
- **Perception → object insertion** (`RealWorld`): `NullPerception` stub until vision/
  lidar detection protos exist.
- **forgebot/`rove_ik_engine`** is the arm IK backend for now; slated for removal — the
  sim (PyBullet + SelfCollisionGuard) becomes the authoritative world model. Don't deepen.
- **Terrain** is a thin shell (no underside); hills viewed from a low cam read as floating
  canopies — orbit up. (Trunk/foliage collision is now live — see Lidar above.)
- **GPU physics** (Isaac/MuJoCo-MJX/Warp) only relevant if mock-mode training needs it;
  not required for the Jetson (kinematic). Decision open, not scoped.

---

## Run

```bash
cd capra_roboguard/rove_sim
# mock physics, headless
PYTHONPATH=. ../rove_sim_venv/bin/python -m rove_sim.main --profile standard --world mock
# real/kinematic world model
PYTHONPATH=. ../rove_sim_venv/bin/python -m rove_sim.main --profile standard --world real
# FAST native window (no GPU readback) -- drive + terrain + click-to-paint friction.
# Run on the HOST desktop (DISPLAY=:0), NOT the IDE/flatpak sandbox. The venv is
# python3.13; the host default is 3.14 and can't load its cp313 wheels, so install
# the matching interpreter once: `sudo dnf install python3.13`, then:
tools/live_host.sh --profile standard --terrain
# (inside the python3.13 sandbox the original command also works:)
PYTHONPATH=. ../rove_sim_venv/bin/python tools/live.py --profile standard --terrain
# Qt panel GUI (headless render -> Qt; slower, but full paint overlay + buttons)
QT_QPA_PLATFORM=wayland PYTHONPATH=. ../rove_sim_venv/bin/python tools/gui.py --profile standard --terrain
# tests
PYTHONPATH=. ../rove_sim_venv/bin/python -m pytest tests/ -q
```
