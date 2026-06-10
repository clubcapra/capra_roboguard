//! capra_autonomy — autonomy engine for the Capra Roboguard.
//!
//! Slice 1 (vertical GoTo): subscribe to VectorNav pose via rove_sensor_api,
//! drive track ODrives toward ENU waypoints, validate convergence. Same wire
//! seams against the sim (think2) or the real robot — only `robot_host` changes.

mod behaviors;
mod config;
mod control;
mod mission;
mod perception;
mod position;
mod reflex;
mod router;
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
}

fn parse_args() -> Args {
    let mut config = PathBuf::from("config/autonomy.toml");
    let mut dry_run = false;
    let mut lidar_probe = false;
    let mut no_reset = false;
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--dry-run" => dry_run = true,
            "--lidar-probe" => lidar_probe = true,
            "--no-reset" => no_reset = true,
            "--config" | "-c" => {
                if let Some(p) = it.next() {
                    config = PathBuf::from(p);
                }
            }
            other => eprintln!("warning: ignoring unknown arg {other:?}"),
        }
    }
    Args { config, dry_run, lidar_probe, no_reset }
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
        "capra_autonomy — robot {}:{}{}",
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

    // --- pose source ------------------------------------------------------
    let (pose_tx, pose_rx) = watch::channel::<Option<Pose>>(None);
    {
        let host = cfg.robot_host.clone();
        let interval_ms = cfg.telemetry.subscribe_ms;
        let mut possvc = PositionService::new(cfg.datum, cfg.position.correction_gain);
        tokio::spawn(async move {
            let r = telemetry::subscribe(&host, vn_data_port, interval_ms, move |frame| {
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

    // --- control loop -----------------------------------------------------
    run_control_loop(&cfg, &mut router, &mut tracks, &mut reflexes, &mut safe, &pose_rx, &truth_rx, &hazard_rx, &costmap_rx).await?;

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
    tracing::info!(
        "wp {}/{} dist {:.2}m hdg_err {:+.0}deg yaw_rate {:+.2} turn {:+.2} L{:+.2} R{:+.2}{}",
        router.waypoint_index() + 1,
        router.waypoint_count(),
        dist,
        heading_err.to_degrees(),
        pose.yaw_rate,
        turn,
        left,
        right,
        truth
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
