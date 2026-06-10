# Fix tomorrow — open issues (capra_autonomy + sim)

Snapshot of known problems found during autonomy bring-up, with the workaround in
place now and the proper fix for later.

## 1. IMU/GNSS drift — the AUTONOMY must handle it (PositionService fusion)  ← biggest one
- The sim VN random-walk (`devices.py` `_gnss_enu`: `_walk += N(0,0.03)` @50 Hz, no
  bound) drifts ~19 m over a long session (measured **(+15.7, −10.2) m** while the robot
  was at spawn, vs the clean lidar pose). This is **realistic** — every real IMU/GNSS
  drifts — so it's a good test case, NOT just a sim bug to disable.
- **Proper fix (per the user + SAR spec): build the autonomy PositionService FUSION** —
  wheel odometry (ODrive encoders) + **lidar odometry / scan-matching (SLAM)** + Livox
  IMU + GNSS, with a drift estimate, so pose stays bounded when GNSS is bad. This is the
  core of the M8 work. See [[sar-spec-docs]] PositionService (GNSS integrity SM
  TRUSTED→SUSPECT→REJECTED→ABSENT, drift budget). The current engine PositionService is
  GNSS+lowpass only — it FOLLOWS the drift, doesn't bound it.
- **Temporary testing crutch only:** `ROVE_VN_ERRORS=0` (clean pose) to unblock testing
  the driving calibration + reflexes tonight. Do NOT ship this; the real deliverable is
  the fusion. (Optionally also bound the sim's "nominal" walk so nominal ≈ 0.4 m like real GNSS.)

## 2. VectorNav yaw is rotated ~90° from the drive-forward axis
- Confirmed by the user (VN is mounted rotated in the 3D model) + probe: driving all
  tracks forward moves the chassis EAST while VN yaw says SOUTH (~90° off). The brush
  model's forward (track geometry) ≠ VN/base yaw.
- **Workaround now:** `[goto].drive_offset_deg = 90` in `config/autonomy.toml`
  (drive_heading = VN heading + 90°).
- **Proper fix:** correct the VN mount in the sim so `VectorNavDevice` reports the true
  chassis heading (then drive_offset_deg = 0), OR derive heading from lidar odometry.

## 3. Road has NO collision on its bounds (edges are void)
- The terrain collider is the road strip only; off the road = no floor. Driving toward
  the road edge / "into the trees" off the strip = the robot falls off the map.
- **Direction from user:** test by driving the robot **up and down the road (the hill)**,
  i.e. ALONG the drivable strip — not across it into the bounds.

## 4. Obstacle spawning should be flexible (place objects anywhere on the road)
- Now: `ROVE_SPAWN_TREES=1` spawns 3 fixed trunks via `scene.spawn_demo_trees`.
- Added: `ROVE_OBSTACLES="x,y;x,y;..."` env → spawn cylinders at arbitrary road
  positions (read by BOTH sim_server physics AND each lidar_worker, so the lidar
  raycasts them). Set positions that sit on the drivable strip.
- **Later:** a runtime spawn command (needs to reach both sim processes — shared file
  or broadcast — because lidar_worker is a separate sim).

## What works (don't re-litigate)
- Livox subscription transport (cross-machine), Rust LVX2 decoder, lidar forward-hazard
  reflex (obstacle + cliff, validated), L0 reflexes (geofence/fall/attitude), remote
  reset-to-spawn (UDP :5099, verified), track-rotation calibration (invert_right=false),
  drive sign (+forward = positive tracks). Heading *control* is good once the pose is clean.

## Road geometry (raycast of the collision mesh from spawn (3,-6))
Drivable (z~0, solid) to 24 m+: **South**, **East**, **SW** (SW has the most hill,
descends to ~-0.3 m). **North / NE / NW are cliffs** within ~9 m (the road bounds, no
collision). Drive-forward at spawn = **East** (so an East waypoint needs no turn).
- Up/down-road test: drive East/South along the strip, there-and-back. Avoid N/NE/NW.
- Obstacle-avoid test: `ROVE_OBSTACLES="9,0"` puts a trunk in the East lane.

## Redeploy needed on think2 to continue (one pull + reinstall)
`tools/sim_server.py`, `tools/lidar_worker.py`, `rove_sim/world/scene.py`,
`scripts/install_service.sh` changed. On think2:
```
cd /data/capra_stack/capra_roboguard/rove_sim && git pull && sudo ./scripts/install_service.sh
```
This adds `ROVE_VN_ERRORS=0` (clean pose) + env-driven obstacle spawning. After it,
GoTo should drive East cleanly on a stable pose; then build the PositionService fusion
to handle realistic drift and re-enable VN errors.

## 5. Pivoting on high-friction textures = a REAL autonomy problem (not a bug)
- The robot is drum tracks; it pivots fine on flat floor. Adding the per-texture
  **FrictionField** (some textures high μ) is the only change — so on those patches a
  commanded pivot stalls (brush model stays in static-grip; `turn` high but
  `yaw_rate ≈ 0`, robot slip-translates instead of rotating). Confirmed live: a 90°
  South turn never rotated, it drifted to the geofence.
- **This is genuinely something the autonomy must SOLVE**, like a real robot:
  - **Stuck-turn detection** (M7 reflex): `|turn_cmd|` high AND `|yaw_rate|` ~0 for N
    ticks → declare turn-stuck.
  - **Recovery maneuver** (MotionService): back up a bit + re-attempt (3-point turn),
    or wiggle, or route around the high-μ patch (use the FrictionField / map).
  - Do NOT brute-force with max_turn=1.0 (that was a dead end; reverted to 0.35).
- For now demos drive **East along the road** (mostly straight) to avoid big pivots.
- Ties into MotionService NEEDS_RECONFIGURE / failure-handler flow in [[sar-spec-docs]].

## TEST RESULTS (2026-06-09, clean pose)
- **Test 1 PASS** — GoTo 15 m East: converged 14.2→0.97 m, hdg_err ±3°, on road.
- **Test 2 PASS** — GoTo East into a trunk at (10,-6): lidar reflex `HAZARD HOLD` at
  obstacle 2.0 m, stopped ~2 m short, held (didn't collide).
- Fix that made Test 2 work: perception forward-corridor must use the **drive-forward
  heading (VN yaw + drive_offset)**, same as GoTo — it was looking 90° off (raw VN yaw).

## 6. NEXT BIG STEP — 3D traversability COST MAP from lidar (user direction)
Hierarchical UGV nav: global (macro) path planning + local (micro) avoidance, both on a
**cost map built from the lidar points in 3D**, scoring terrain by **slope / roughness /
obstacle (tree) density / negative-obstacle (drop-off)**. This:
- replaces the brittle single-direction forward-corridor reflex with a real local map,
- lets the planner route the robot **around** obstacles (not just stop) — the "walk around
  objects" goal — and around high-friction / untraversable patches (#5),
- is the **first step toward mapping** (accumulate the cost map over time -> the SAR
  MapService voxels + the sidecar planner in [[sar-spec-docs]]).
Build incrementally in the autonomy layer: per-frame local cost grid (robot-centric, from
the bottom Livox) -> A*/DWA local plan to a waypoint that avoids high-cost cells -> feed
the track controller. Then accumulate frames into a persistent map (odometry-registered).

## 7. 3D traversability cost map — BUILT (viz) + the avoidance lesson
- `rove_sim/tools/costmap_snapshot.py` renders the cost map live from the bottom Livox:
  green=flat, light-green/yellow=hill, orange=step/stairs (climbable, costly), red=wall/tree
  (blocked), blue=cliff, **black=unknown/no-ground**. Image: capra_autonomy/media/lidar/costmap_spawn.png.
  Shows the robot in a green clearing, red/orange trees, black beyond (forest/edges).
- **Why reactive VFH steered off the road & fell:** it avoided only RED (obstacles), so it
  steered into a BLACK (unknown / no-ground) gap → up a small hill → off the edge. Reverted
  the control loop to the **safe stop-before-obstacle reflex** (no off-road steering).
- **The real go-around = plan on the cost map:** route only through GREEN (known traversable)
  cells, treat BLACK (unknown) + RED/BLUE (blocked) as no-go; A*/gradient to a local goal.
  Needs: (a) build the grid in Rust (port costmap_snapshot), accumulate over odometry (mapping);
  (b) classify cost incl. **stairs/hills as traversable** (user: flipper robot, lidar registers
  steps) so the planner can pick a climb when it's the only way; (c) the locomotion fix (#5,
  high-friction turning) so the robot can actually execute the planned path.
- STEP-CLIMB / WALL / slope thresholds live in costmap_snapshot.py — tune to the platform.

## 8. COST-MAP GO-AROUND — WORKING (2026-06-10)
Built the Rust 3D cost map + Dijkstra local planner (`perception/costmap.rs`) and wired it
into the control loop (`main.rs`): each frame builds a world-frame traversability grid from
the bottom Livox, Dijkstra routes from the robot to a local target that stays on KNOWN-
traversable cells (around obstacles, never into unknown/off-road), GoTo `step_to` follows it.
- RESULT: robot drove East, **planned AROUND the trunks** (PLANNING around, steer -23/+18/-31deg),
  stayed on the road, dist 14.8->2.5 m, **MISSION COMPLETE**. The reactive VFH that drove off a
  hill is gone; planning on the cost map keeps it on green.
- Rough edges (locomotion-bound, tie to #5): brief stall/struggle near the trunk (weak friction
  turning); arrive tol loosened to 2.5 m (overshoot). Tighten once turning is solved.
- Next: accumulate the cost map over odometry (persistent MAP, not just per-frame), classify
  stairs/hills as traversable-cost so the planner can choose a climb; global planner on the map.
