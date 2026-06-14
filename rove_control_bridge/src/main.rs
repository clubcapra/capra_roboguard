//! rove_control_bridge — Capra Roboguard orchestrator (autonomy + control).
//!
//! ONE front door for Control / Mission / Estop / CamChange. Runs missions
//! (cost-map nav, SAR behaviours, positioning, L0 reflexes) AND passes teleop
//! through, dispatching motion INTENT to rove_ik_engine. Merged from capra_autonomy
//! (the Rust autonomy) + the Python rove_control_bridge (now being ported to Rust).
//!
//! Same wire seams against the sim (think2) or the real robot — only `robot_host`
//! changes (and `--no-reset` on the real robot).

mod behaviors;
mod calibrate;
mod comms;
mod config;
mod control;
mod gripper;
mod ik;
mod mission;
mod perception;
mod position;
mod reflex;
mod router;
mod telemetry_out;
mod transport;
mod validate;

use anyhow::{Context, Result};
use behaviors::goto::Waypoint;
use config::Config;
use control::heading::HeadingController;
use control::tracks::TracksController;
use perception::costmap::CostMap;
use perception::Hazard;
use std::sync::Arc;
use position::{Pose, PositionService};
use reflex::ReflexEngine;
use router::{Action, Router};
use std::path::PathBuf;
use std::time::{Duration, Instant};
use tokio::sync::watch;
use transport::{command::CommandSink, discover, telemetry};
use validate::ground_truth::{self, Truth};

const GROUND_TRUTH_PORT: u16 = 5030;
/// How far ahead on the planned path to aim the local target (pure-pursuit).
const PLAN_LOOKAHEAD: f64 = 3.0;

struct Args {
    config: PathBuf,
    dry_run: bool,
    lidar_probe: bool,
    no_reset: bool,
    calibrate: bool,
}

fn parse_args() -> Args {
    let mut config = PathBuf::from("config/autonomy.toml");
    let mut dry_run = false;
    let mut lidar_probe = false;
    let mut no_reset = false;
    let mut calibrate = false;
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--dry-run" => dry_run = true,
            "--lidar-probe" => lidar_probe = true,
            "--no-reset" => no_reset = true,
            "--calibrate" => calibrate = true,
            "--config" | "-c" => {
                if let Some(p) = it.next() {
                    config = PathBuf::from(p);
                }
            }
            other => eprintln!("warning: ignoring unknown arg {other:?}"),
        }
    }
    Args { config, dry_run, lidar_probe, no_reset, calibrate }
}

/// Ask the sim to respawn the robot at the start pose (sim_server reset port).
async fn reset_robot(host: &str) -> Result<()> {
    const RESET_PORT: u16 = 5099;
    let sock = tokio::net::UdpSocket::bind(("0.0.0.0", 0)).await?;
    sock.send_to(b"RESET", (host, RESET_PORT)).await?;
    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    let args = parse_args();
    let cfg = Config::load(&args.config)?;
    tracing::info!(
        "rove_control_bridge — robot {}:{}{}",
        cfg.robot_host,
        cfg.http_port,
        if args.dry_run { " [DRY-RUN]" } else { "" }
    );

    // --- resolve ports off the live API -----------------------------------
    let disc = discover::discover(&cfg.robot_host, cfg.http_port)
        .context("/discover failed — is rove_sensor_api up?")?;
    let vn = disc
        .get(&cfg.telemetry.vectornav_id)
        .with_context(|| format!("{} not in /discover", cfg.telemetry.vectornav_id))?;
    let vn_data_port = vn.data_port;

    // --- telemetry OUT: aggregate robot state -> RoveTelemetry, republish -----
    let tele_agg = telemetry_out::shared();
    {
        let port = cfg.comms.telemetry_out_port;
        let hz = cfg.comms.telemetry_out_hz;
        let agg = tele_agg.clone();
        tokio::spawn(async move {
            if let Err(e) = telemetry_out::run_publisher(port, agg, hz).await {
                tracing::warn!("telemetry-out publisher ended: {e:#}");
            }
        });
    }
    // fold every ODrive node + the gripper into the aggregate (read-only)
    for (id, ep) in disc.iter() {
        let agg = tele_agg.clone();
        let host = cfg.robot_host.clone();
        let interval_ms = cfg.telemetry.subscribe_ms;
        let dp = ep.data_port;
        if id.starts_with("odrive_") {
            tokio::spawn(async move {
                telemetry::subscribe(&host, dp, interval_ms, move |f| telemetry_out::update_odrive(&agg, &f)).await.ok();
            });
        } else if id.contains("gripper") || id.contains("robotiq") {
            tokio::spawn(async move {
                telemetry::subscribe(&host, dp, interval_ms, move |f| telemetry_out::update_gripper(&agg, &f)).await.ok();
            });
        }
    }

    // --- pose source ------------------------------------------------------
    let (pose_tx, pose_rx) = watch::channel::<Option<Pose>>(None);
    {
        let host = cfg.robot_host.clone();
        let interval_ms = cfg.telemetry.subscribe_ms;
        let mut possvc = PositionService::new(cfg.datum, cfg.position.correction_gain);
        let agg = tele_agg.clone();
        tokio::spawn(async move {
            let r = telemetry::subscribe(&host, vn_data_port, interval_ms, move |frame| {
                telemetry_out::update_vn300(&agg, &frame);
                if let Some(p) = possvc.update(&frame, std::time::Instant::now()) {
                    let _ = pose_tx.send(Some(p));
                }
            })
            .await;
            if let Err(e) = r {
                tracing::error!("telemetry task ended: {e:#}");
            }
        });
    }

    // --- lidar decoder validation (no driving) ----------------------------
    if args.lidar_probe {
        tokio::time::sleep(std::time::Duration::from_millis(300)).await; // let a fix arrive
        perception::probe(&cfg.robot_host, 5024, pose_rx.clone()).await?; // bottom Livox = near sensing
        return Ok(());
    }

    // --- lidar perception: hazard reflex + 3D cost map --------------------
    let (hazard_tx, hazard_rx) = watch::channel::<Hazard>(Hazard::none());
    let (costmap_tx, costmap_rx) = watch::channel::<Option<Arc<CostMap>>>(None);
    {
        let host = cfg.robot_host.clone();
        let p = cfg.perception;
        let off = cfg.goto.drive_offset_deg;
        let pose_rx = pose_rx.clone();
        tokio::spawn(async move {
            if let Err(e) = perception::run(
                host, p.lidar_port, p.obstacle_stop_m, p.cliff_stop_m,
                p.min_ground_per_bin, off, pose_rx, hazard_tx, costmap_tx,
            ).await {
                tracing::warn!("perception task ended: {e:#}");
            }
        });
    }

    // --- ground-truth validator (best-effort) -----------------------------
    let (truth_tx, truth_rx) = watch::channel::<Option<Truth>>(None);
    tokio::spawn(async move {
        if let Err(e) = ground_truth::listen(GROUND_TRUTH_PORT, truth_tx).await {
            tracing::warn!("ground-truth listener ended: {e:#}");
        }
    });

    // --- drive output -----------------------------------------------------
    let sink = CommandSink::new(cfg.robot_host.clone(), args.dry_run).await?;
    let mut tracks = TracksController::new(&cfg.tracks, &sink, &disc)?;
    tracing::info!("track map: {}", tracks.map_summary());

    // --- CALIBRATE mission: run the deployment calibration + write config ---
    if args.calibrate {
        tokio::time::sleep(Duration::from_millis(400)).await; // let a fix arrive
        calibrate::run(&cfg, &disc, &sink, &pose_rx).await?;
        return Ok(());
    }

    // --- reset the robot to spawn (clean, repeatable runs) ----------------
    if !args.dry_run && !args.no_reset {
        tracing::info!("resetting robot to spawn…");
        reset_robot(&cfg.robot_host).await.ok();
        tokio::time::sleep(std::time::Duration::from_millis(1800)).await; // respawn + settle
    }

    // --- wait for first fix, then resolve waypoints -----------------------
    let start = wait_for_fix(&pose_rx).await?;
    tracing::info!(
        "first fix: ENU ({:.2}, {:.2}) yaw_ned {:.1}deg",
        start.x,
        start.y,
        start.yaw_ned_deg
    );
    // SAR safe points: Origin = configured Home or the first fix.
    let mut safe = mission::SafePoints::new();
    let rel = |p: [f64; 2]| {
        if cfg.mission.relative_to_start { [start.x + p[0], start.y + p[1]] } else { p }
    };
    let home = cfg.mission.home.map(rel).unwrap_or([start.x, start.y]);
    safe.set(mission::SafeKind::Origin, home, 1.0, 0.0);

    let (waypoints, terminal, behavior_name) = resolve_mission(&cfg, &start, &safe);
    tracing::info!(
        "mission: {} ({} waypoint(s), terminal {:?}); Home/origin ENU ({:.1}, {:.1})",
        behavior_name, waypoints.len(), terminal, home[0], home[1],
    );
    for (i, w) in waypoints.iter().enumerate() {
        tracing::info!("  wp {i}: ENU ({:.2}, {:.2})", w.x, w.y);
    }
    let mut router = Router::new(waypoints, cfg.goto);
    router.set_hold_at_end(terminal == mission::Terminal::Hold);
    let mut reflexes = ReflexEngine::new(cfg.reflex, (start.x, start.y));
    tracing::info!(
        "reflexes armed: geofence {:.0} m, fall {:.0} m, roll/pitch {:.0}/{:.0} deg",
        cfg.reflex.geofence_radius_m,
        cfg.reflex.fall_floor_m,
        cfg.reflex.max_roll_deg,
        cfg.reflex.max_pitch_deg
    );

    // --- front door (Steam Deck via udp_multiplexer) ----------------------
    let (teleop_tx, teleop_rx) = watch::channel::<Option<comms::TeleopIntent>>(None);
    let (mission_tx, mission_rx) = watch::channel::<Option<comms::proto::Mission>>(None);
    // IK engine forwarder: arm intent (always, if a target is set) and drive
    // teleop (when drive_via_engine). Created if either is wanted.
    let ik_fwd = if cfg.comms.arm_target_entity.is_empty() && !cfg.comms.drive_via_engine {
        None
    } else {
        match ik::IkForwarder::new(
            &cfg.comms.engine_host,
            cfg.comms.engine_port,
            cfg.comms.engine_drive_port,
            cfg.comms.arm_target_entity.clone(),
        )
        .await
        {
            Ok(f) => Some(std::sync::Arc::new(f)),
            Err(e) => {
                tracing::warn!("ik forwarder disabled: {e:#}");
                None
            }
        }
    };
    // Gripper goes DIRECT to the robot API (the IK engine has no gripper).
    let gripper = std::sync::Arc::new(gripper::GripperSender::new(
        &cfg.robot_host, cfg.http_port, args.dry_run,
    ));
    {
        let port = cfg.comms.teleop_port;
        let ik_fwd = ik_fwd.clone();
        let via_engine = cfg.comms.drive_via_engine;
        let gripper = gripper.clone();
        tokio::spawn(async move {
            if let Err(e) = comms::run_teleop_listener(port, teleop_tx, ik_fwd, via_engine, gripper).await {
                tracing::warn!("teleop listener ended: {e:#}");
            }
        });
    }
    {
        let port = cfg.comms.mission_port;
        tokio::spawn(async move {
            if let Err(e) = comms::run_mission_listener(port, mission_tx).await {
                tracing::warn!("mission listener ended: {e:#}");
            }
        });
    }
    let (estop_tx, estop_rx) = watch::channel::<bool>(false);
    {
        let port = cfg.comms.estop_port;
        tokio::spawn(async move {
            if let Err(e) = comms::run_estop_listener(port, estop_tx).await {
                tracing::warn!("estop listener ended: {e:#}");
            }
        });
    }

    // --- control loop -----------------------------------------------------
    run_control_loop(&cfg, &mut router, &mut tracks, &mut reflexes, &mut safe, &pose_rx, &truth_rx, &hazard_rx, &costmap_rx, &teleop_rx, mission_rx, &estop_rx).await?;

    // --- always leave the robot stopped + disarmed ------------------------
    tracks.idle().await.ok();
    tracing::info!("disarmed; bye");
    Ok(())
}

/// Block until the pose source delivers a fix AND the robot is settled (roughly
/// level and still). This avoids arming on a post-reset/landing transient — the
/// robot rocks for a moment after a sim reset or after dropping onto terrain.
async fn wait_for_fix(pose_rx: &watch::Receiver<Option<Pose>>) -> Result<Pose> {
    const LEVEL_DEG: f64 = 5.0;
    const STILL_RAD_S: f64 = 0.1;
    let mut rx = pose_rx.clone();
    let mut settled_ticks = 0u32;
    loop {
        if let Some(p) = *rx.borrow() {
            let level = p.roll_deg.abs() < LEVEL_DEG && p.pitch_deg.abs() < LEVEL_DEG;
            if p.gnss_fix && level && p.yaw_rate.abs() < STILL_RAD_S {
                settled_ticks += 1;
                if settled_ticks >= 25 {
                    // ~0.5 s of quiet
                    return Ok(p);
                }
            } else {
                settled_ticks = 0;
            }
        }
        rx.changed().await.context("pose channel closed")?;
    }
}

/// Compile the configured SAR behaviour into a flyable plan (waypoints + terminal).
fn resolve_mission(
    cfg: &Config,
    start: &Pose,
    safe: &mission::SafePoints,
) -> (Vec<Waypoint>, mission::Terminal, String) {
    let m = &cfg.mission;
    let rel = |p: [f64; 2]| {
        if m.relative_to_start { [start.x + p[0], start.y + p[1]] } else { p }
    };
    let mut wps: Vec<[f64; 2]> = m.waypoints.iter().map(|p| rel(*p)).collect();
    // demo_forward fallback: a single waypoint straight ahead (safest first drive)
    if wps.is_empty() && m.demo_forward_m > 0.0 {
        let h = start.heading_enu_rad();
        wps.push([start.x + m.demo_forward_m * h.cos(), start.y + m.demo_forward_m * h.sin()]);
    }
    let behavior = mission::from_config(
        &m.behavior, &wps, m.orbit_center.map(rel),
        m.orbit_radius, m.orbit_laps, m.retreat_dist, &m.return_target,
    );
    let plan = mission::compile(&behavior, safe, [start.x, start.y], &[]);
    (plan.waypoints, plan.terminal, m.behavior.clone())
}

async fn run_control_loop(
    cfg: &Config,
    router: &mut Router,
    tracks: &mut TracksController<'_>,
    reflexes: &mut ReflexEngine,
    safe: &mut mission::SafePoints,
    pose_rx: &watch::Receiver<Option<Pose>>,
    truth_rx: &watch::Receiver<Option<Truth>>,
    hazard_rx: &watch::Receiver<Hazard>,
    costmap_rx: &watch::Receiver<Option<Arc<CostMap>>>,
    teleop_rx: &watch::Receiver<Option<comms::TeleopIntent>>,
    mut mission_rx: watch::Receiver<Option<comms::proto::Mission>>,
    estop_rx: &watch::Receiver<bool>,
) -> Result<()> {
    let dt = 1.0 / cfg.control.rate_hz;
    let period = Duration::from_secs_f64(dt);
    let stale = Duration::from_millis(cfg.telemetry.pose_stale_ms);
    let log_every = cfg.control.rate_hz.max(1.0) as u64; // ~1 s
    let mut ticks: u64 = 0;
    let mut heading = HeadingController::new(cfg.asserv);
    let mut interval = tokio::time::interval(period);
    // turn-stuck detection + recovery (high-friction skid-steer can stall a turn)
    let mut stuck_ticks: u32 = 0;
    let mut recovery_until: Option<Instant> = None;
    let mut recovery_sign = 1.0_f64;
    // progress watchdog -> hold if a goal can't be reached (e.g. fenced by no-go)
    let mut best_dist = f64::INFINITY;
    let mut last_progress = Instant::now();
    let mut tracked_goal: Option<[f64; 2]> = None;
    // SAR safe-point auto-update + breadcrumb trail + GNSS-quality logging
    let mut breadcrumbs: Vec<[f64; 2]> = Vec::new();
    let mut last_crumb = Instant::now();
    let mut gnss_was_ok = true;

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                tracing::info!("ctrl-c — stopping");
                break;
            }
            _ = interval.tick() => {
                ticks += 1;
                let pose = match *pose_rx.borrow() {
                    Some(p) => p,
                    None => { tracks.idle().await.ok(); continue; }
                };
                // Hold only on a STALE stream. A lost GNSS fix does NOT stop us —
                // the PositionService dead-reckons through it so we can still drive
                // (e.g. home). Only give up if confidence collapses entirely.
                if pose.received_at.elapsed() > stale {
                    tracing::warn!("pose stale — holding");
                    tracks.idle().await.ok();
                    continue;
                }
                if pose.position_confidence < 0.05 {
                    tracing::warn!("position lost (confidence ~0, drift {:.1} m) — holding", pose.drift_m);
                    tracks.idle().await.ok();
                    continue;
                }

                // SAR safe points: record trusted-GNSS + stable-ground posts, and
                // log GNSS-quality transitions (the dead-reckoning signal).
                if pose.gnss_fix && pose.position_confidence > 0.7 {
                    safe.set(mission::SafeKind::LastGnssTrusted, [pose.x, pose.y], pose.position_confidence, pose.drift_m);
                    if pose.roll_deg.abs() < 8.0 && pose.pitch_deg.abs() < 8.0 {
                        safe.set(mission::SafeKind::LastStable, [pose.x, pose.y], pose.position_confidence, pose.drift_m);
                    }
                }
                if pose.gnss_fix != gnss_was_ok {
                    gnss_was_ok = pose.gnss_fix;
                    if pose.gnss_fix {
                        tracing::info!("GNSS REACQUIRED — confidence recovering");
                    } else {
                        tracing::warn!("GNSS LOST — dead-reckoning home on velocity + IMU (SLAM hold)");
                    }
                }
                // breadcrumb trail (for Retreat / BacktrackComm)
                if last_crumb.elapsed() > Duration::from_millis(500) {
                    if breadcrumbs.last().map_or(true, |c| (c[0]-pose.x).hypot(c[1]-pose.y) > 2.0) {
                        breadcrumbs.push([pose.x, pose.y]);
                    }
                    last_crumb = Instant::now();
                }

                // L0 reflex gate — vetoes ALL motion. A trip latches: stop the
                // mission and disarm; recovery is a deliberate reset.
                if let Some(trip) = reflexes.check(&pose) {
                    tracing::error!("ESTOP ({}) — disarming and stopping", trip.reason);
                    tracks.idle().await.ok();
                    break;
                }

                // OPERATOR ESTOP (clearable): true = stop everything and hold;
                // false = the tracks re-arm (clear_errors) on the next drive. Unlike
                // the latched L0 reflex above, this can be reverted from the deck.
                if *estop_rx.borrow() {
                    tracks.idle().await.ok();
                    heading.reset();
                    if ticks % (log_every * 2) == 0 {
                        tracing::warn!("OPERATOR ESTOP — held (send Estop{{active:false}} to clear)");
                    }
                    continue;
                }

                // NEW MISSION from the operator (Mission proto): compile its Step
                // graph to waypoints about the datum and swap the router live.
                if mission_rx.has_changed().unwrap_or(false) {
                    if let Some(m) = mission_rx.borrow_and_update().clone() {
                        let (wps, terminal, n) =
                            mission::wire::compile_mission(&m, &cfg.datum, safe, [pose.x, pose.y]);
                        if wps.is_empty() {
                            tracing::warn!("mission '{}' compiled to 0 waypoints (no geometry steps)", m.name);
                        } else {
                            tracing::info!(
                                "MISSION '{}' START — {} step(s) -> {} waypoint(s), terminal {:?}",
                                m.name, n, wps.len(), terminal);
                            *router = Router::new(wps, cfg.goto);
                            router.set_hold_at_end(terminal == mission::Terminal::Hold);
                            best_dist = f64::INFINITY;
                            last_progress = Instant::now();
                            tracked_goal = None;
                        }
                    }
                }

                // TELEOP PREEMPTS the mission: fresh, active operator intent drives
                // the tracks raw (operator owns heading). The L0 reflex above still
                // vetoes it. (Flippers/arm route through rove_ik_engine — Phase 4.)
                if let Some(t) = *teleop_rx.borrow() {
                    if t.is_active() && t.stamp.elapsed() < Duration::from_millis(500) {
                        heading.reset();
                        // When drives route through the IK engine, the teleop
                        // listener already forwarded tracks+flippers there — don't
                        // also drive the drums here (would double-command them).
                        if !cfg.comms.drive_via_engine {
                            tracks.drive(t.left as f64, t.right as f64, dt).await?;
                        }
                        if ticks % log_every == 0 {
                            tracing::info!(
                                "TELEOP — L{:+.2} R{:+.2} flippers {:?}{}{}",
                                t.left, t.right, t.flippers,
                                if t.has_arm { " +arm" } else { "" },
                                if cfg.comms.drive_via_engine { " [via engine]" } else { "" },
                            );
                        }
                        continue;
                    }
                }

                match router.tick(&pose) {
                    Action::Drive { forward: rf, heading_err: rh, dist } => {
                        // --- active RECOVERY: back up while pivoting to break a stalled
                        // high-friction turn (3-point-turn wiggle); overrides everything ---
                        if let Some(until) = recovery_until {
                            if Instant::now() < until {
                                let rturn = recovery_sign * 0.7;
                                let left = (-0.5 + rturn).clamp(-1.0, 1.0);
                                let right = (-0.5 - rturn).clamp(-1.0, 1.0);
                                tracks.drive(left, right, dt).await?;
                                if ticks % (log_every / 2).max(1) == 0 {
                                    tracing::warn!("RECOVERY — backing + pivoting to break a stalled turn");
                                }
                                continue;
                            }
                            recovery_until = None;
                            heading.reset();
                        }
                        // progress watchdog: if we haven't gotten meaningfully closer to
                        // this goal for a while, it's unreachable (walled off by no-go) ->
                        // hold instead of wandering / arcing forever.
                        let goal_xy = router.current_waypoint().map(|w| [w.x, w.y]);
                        if goal_xy != tracked_goal {
                            tracked_goal = goal_xy;
                            best_dist = f64::INFINITY;
                            last_progress = Instant::now();
                        }
                        if dist < best_dist - 0.5 {
                            best_dist = dist;
                            last_progress = Instant::now();
                        }
                        if last_progress.elapsed() > Duration::from_secs(10) {
                            tracks.idle().await.ok();
                            heading.reset();
                            if ticks % (log_every * 2) == 0 {
                                tracing::warn!(
                                    "goal unreachable — no progress for 10 s (best {:.1} m) — holding", best_dist);
                            }
                            continue;
                        }
                        let hz = *hazard_rx.borrow();
                        let fresh = hz.stamp.elapsed() < Duration::from_millis(800);
                        // A drop-off is ALWAYS a hard stop — never trust a steer over an edge.
                        if fresh && hz.cliff_ahead {
                            tracks.idle().await.ok();
                            heading.reset();
                            if ticks % (log_every / 2).max(1) == 0 {
                                tracing::warn!("HAZARD HOLD — drop-off ahead ({:.1} m)", hz.cliff_dist);
                            }
                            continue;
                        }
                        // Cost-map PLANNER: route to a local target that stays on known
                        // traversable ground (around obstacles, never into unknown/off-road).
                        // Within a lookahead of the goal, just aim straight at it (no jitter).
                        let cm = costmap_rx.borrow().clone();
                        let goal = router.current_waypoint();
                        let (forward, heading_err, planned) = if dist < PLAN_LOOKAHEAD + 1.0 {
                            (rf, rh, false)
                        } else { match (goal, cm) {
                            (Some(w), Some(cm))
                                if cm.stamp.elapsed() < Duration::from_millis(900) =>
                            {
                                match cm.plan([w.x, w.y], PLAN_LOOKAHEAD) {
                                    Some(t) => {
                                        let (f, h) = behaviors::goto::step_to(&pose, t, &cfg.goto);
                                        (f, h, true)
                                    }
                                    None => {
                                        // boxed in by no-go cells -> stop and wait for the map to fill
                                        tracks.idle().await.ok();
                                        heading.reset();
                                        if ticks % (log_every / 2).max(1) == 0 {
                                            tracing::warn!("HAZARD HOLD — no traversable path to goal");
                                        }
                                        continue;
                                    }
                                }
                            }
                            _ => (rf, rh, false), // no cost map yet -> head straight at the goal
                        } };
                        // HARD GUARD: never drive forward INTO a hazard (cliff / steep
                        // descent / unknown) on the actual drive heading -- the full
                        // cost-map check the narrow lidar corridor misses (the fall).
                        let drive_heading =
                            pose.heading_enu_rad() + cfg.goto.drive_offset_deg.to_radians();
                        if forward > 0.01 {
                            let blocked = costmap_rx
                                .borrow()
                                .as_ref()
                                .filter(|cm| cm.stamp.elapsed() < Duration::from_millis(900))
                                .is_some_and(|cm| cm.blocked_ahead([pose.x, pose.y], drive_heading, 2.2));
                            if blocked {
                                tracks.idle().await.ok();
                                heading.reset();
                                if ticks % (log_every / 2).max(1) == 0 {
                                    tracing::warn!("HOLD — hazard ahead on drive heading (cliff/steep/unknown within 2.2 m)");
                                }
                                continue;
                            }
                        }
                        let turn = heading.update(heading_err, pose.yaw_rate, dt);
                        // ARC-CLAMP: keep the inner track ~forward so the robot ARCS
                        // (stays moving -> scrubs) instead of pivoting (stalls).
                        let turn = turn.clamp(-(forward + 0.10), forward + 0.10);
                        // turn-stuck: big heading error but barely yawing (even arcing can't
                        // turn here -- tight spot) -> trigger a reverse-pivot K-turn recovery.
                        if heading_err.abs() > 0.45 && pose.yaw_rate.abs() < 0.06 {
                            stuck_ticks += 1;
                        } else {
                            stuck_ticks = 0;
                        }
                        if stuck_ticks >= 30 {
                            recovery_until = Some(Instant::now() + Duration::from_millis(1200));
                            recovery_sign = if heading_err >= 0.0 { 1.0 } else { -1.0 };
                            stuck_ticks = 0;
                            tracing::warn!(
                                "turn-stuck (hdg_err {:+.0}deg, yaw_rate {:+.2}) -> K-turn recovery",
                                heading_err.to_degrees(), pose.yaw_rate);
                            tracks.idle().await.ok();
                            continue;
                        }
                        let left = (forward + turn).clamp(-1.0, 1.0);
                        let right = (forward - turn).clamp(-1.0, 1.0);
                        tracks.drive(left, right, dt).await?;
                        if ticks % log_every == 0 {
                            log_status(router, &pose, truth_rx, dist, heading_err, turn, left, right);
                            if planned && (heading_err.abs() > 0.15 || hz.obstacle_ahead) {
                                tracing::info!(
                                    "PLANNING around — local target steer {:+.0}deg (obstacle {:.1} m)",
                                    heading_err.to_degrees(), hz.obstacle_dist,
                                );
                            }
                        }
                    }
                    Action::Hold => { heading.reset(); tracks.idle().await.ok(); }
                    Action::MissionComplete => {
                        report_complete(router, &pose, truth_rx);
                        break;
                    }
                }
            }
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn log_status(
    router: &Router,
    pose: &Pose,
    truth_rx: &watch::Receiver<Option<Truth>>,
    dist: f64,
    heading_err: f64,
    turn: f64,
    left: f64,
    right: f64,
) {
    let truth = match *truth_rx.borrow() {
        Some(t) => format!(
            " | truth err {:.2}m",
            (t.x - pose.x).hypot(t.y - pose.y)
        ),
        None => String::new(),
    };
    let gnss = if pose.gnss_fix {
        String::new()
    } else {
        format!(" | GNSS-DENIED conf {:.2} drift {:.1}m", pose.position_confidence, pose.drift_m)
    };
    tracing::info!(
        "wp {}/{} dist {:.2}m hdg_err {:+.0}deg yaw_rate {:+.2} turn {:+.2} L{:+.2} R{:+.2}{}{}",
        router.waypoint_index() + 1,
        router.waypoint_count(),
        dist,
        heading_err.to_degrees(),
        pose.yaw_rate,
        turn,
        left,
        right,
        truth,
        gnss
    );
}

fn report_complete(router: &Router, pose: &Pose, truth_rx: &watch::Receiver<Option<Truth>>) {
    let truth = match *truth_rx.borrow() {
        Some(t) => format!(
            " (ground-truth pose-error {:.2}m)",
            (t.x - pose.x).hypot(t.y - pose.y)
        ),
        None => String::new(),
    };
    tracing::info!(
        "MISSION COMPLETE — {} waypoint(s), final pose ENU ({:.2}, {:.2}){}",
        router.waypoint_count(),
        pose.x,
        pose.y,
        truth
    );
}
