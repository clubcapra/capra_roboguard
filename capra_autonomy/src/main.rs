//! capra_autonomy — autonomy engine for the Capra Roboguard.
//!
//! Slice 1 (vertical GoTo): subscribe to VectorNav pose via rove_sensor_api,
//! drive track ODrives toward ENU waypoints, validate convergence. Same wire
//! seams against the sim (think2) or the real robot — only `robot_host` changes.

mod behaviors;
mod config;
mod control;
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
use perception::Hazard;
use position::{Pose, PositionService};
use reflex::ReflexEngine;
use router::{Action, Router};
use std::path::PathBuf;
use std::time::Duration;
use tokio::sync::watch;
use transport::{command::CommandSink, discover, telemetry};
use validate::ground_truth::{self, Truth};

const GROUND_TRUTH_PORT: u16 = 5030;

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

    // --- lidar forward-hazard reflex --------------------------------------
    let (hazard_tx, hazard_rx) = watch::channel::<Hazard>(Hazard::none());
    {
        let host = cfg.robot_host.clone();
        let p = cfg.perception;
        let pose_rx = pose_rx.clone();
        tokio::spawn(async move {
            if let Err(e) = perception::run(
                host, p.lidar_port, p.obstacle_stop_m, p.cliff_stop_m,
                p.min_ground_per_bin, pose_rx, hazard_tx,
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
    let waypoints = resolve_waypoints(&cfg, &start);
    if waypoints.is_empty() {
        tracing::warn!("no waypoints — nothing to do (set [mission].demo_forward_m or waypoints)");
    } else {
        for (i, w) in waypoints.iter().enumerate() {
            tracing::info!("waypoint {i}: ENU ({:.2}, {:.2})", w.x, w.y);
        }
    }
    let mut router = Router::new(waypoints, cfg.goto);
    let mut reflexes = ReflexEngine::new(cfg.reflex, (start.x, start.y));
    tracing::info!(
        "reflexes armed: geofence {:.0} m, fall {:.0} m, roll/pitch {:.0}/{:.0} deg",
        cfg.reflex.geofence_radius_m,
        cfg.reflex.fall_floor_m,
        cfg.reflex.max_roll_deg,
        cfg.reflex.max_pitch_deg
    );

    // --- control loop -----------------------------------------------------
    run_control_loop(&cfg, &mut router, &mut tracks, &mut reflexes, &pose_rx, &truth_rx, &hazard_rx).await?;

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

/// Build the ENU waypoint list from config + the start pose.
fn resolve_waypoints(cfg: &Config, start: &Pose) -> Vec<Waypoint> {
    let m = &cfg.mission;
    if !m.waypoints.is_empty() {
        return m
            .waypoints
            .iter()
            .map(|[x, y]| {
                if m.relative_to_start {
                    Waypoint {
                        x: start.x + x,
                        y: start.y + y,
                    }
                } else {
                    Waypoint { x: *x, y: *y }
                }
            })
            .collect();
    }
    if m.demo_forward_m > 0.0 {
        // Straight ahead of the start heading — the safest first live drive.
        let h = start.heading_enu_rad();
        return vec![Waypoint {
            x: start.x + m.demo_forward_m * h.cos(),
            y: start.y + m.demo_forward_m * h.sin(),
        }];
    }
    Vec::new()
}

async fn run_control_loop(
    cfg: &Config,
    router: &mut Router,
    tracks: &mut TracksController<'_>,
    reflexes: &mut ReflexEngine,
    pose_rx: &watch::Receiver<Option<Pose>>,
    truth_rx: &watch::Receiver<Option<Truth>>,
    hazard_rx: &watch::Receiver<Hazard>,
) -> Result<()> {
    let dt = 1.0 / cfg.control.rate_hz;
    let period = Duration::from_secs_f64(dt);
    let stale = Duration::from_millis(cfg.telemetry.pose_stale_ms);
    let log_every = cfg.control.rate_hz.max(1.0) as u64; // ~1 s
    let mut ticks: u64 = 0;
    let mut heading = HeadingController::new(cfg.asserv);
    let mut interval = tokio::time::interval(period);

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
                if pose.received_at.elapsed() > stale || !pose.gnss_fix {
                    tracing::warn!("pose stale/unfixed — holding");
                    tracks.idle().await.ok();
                    continue;
                }

                // L0 reflex gate — vetoes ALL motion. A trip latches: stop the
                // mission and disarm; recovery is a deliberate reset.
                if let Some(trip) = reflexes.check(&pose) {
                    tracing::error!("ESTOP ({}) — disarming and stopping", trip.reason);
                    tracks.idle().await.ok();
                    break;
                }

                match router.tick(&pose) {
                    Action::Drive { forward, heading_err, dist } => {
                        // Lidar forward-hazard gate: stop before obstacles / edges.
                        let hz = *hazard_rx.borrow();
                        let fresh = hz.stamp.elapsed() < Duration::from_millis(800);
                        if fresh && (hz.obstacle_ahead || hz.cliff_ahead) {
                            tracks.idle().await.ok();
                            heading.reset();
                            if ticks % (log_every / 2).max(1) == 0 {
                                tracing::warn!(
                                    "HAZARD HOLD — {} ahead (obstacle {:.1} m, cliff {:.1} m); dist-to-wp {:.1} m",
                                    if hz.obstacle_ahead { "obstacle" } else { "drop-off" },
                                    hz.obstacle_dist, hz.cliff_dist, dist,
                                );
                            }
                            continue;
                        }
                        // Inner IMU heading loop turns the heading error into the
                        // track differential, damping slip-induced yaw (mud).
                        let turn = heading.update(heading_err, pose.yaw_rate, dt);
                        // Sim track convention (verified live by direct probe):
                        //  * +track on BOTH sides drives the chassis BACKWARD, so to
                        //    close range we drive with -forward.
                        //  * turn>0 (toward +heading_err = CCW) is left>right.
                        let drive = -forward;
                        let left = (drive + turn).clamp(-1.0, 1.0);
                        let right = (drive - turn).clamp(-1.0, 1.0);
                        tracks.drive(left, right, dt).await?;
                        if ticks % log_every == 0 {
                            log_status(router, &pose, truth_rx, dist, heading_err, turn, left, right);
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
