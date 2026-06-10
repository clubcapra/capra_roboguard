//! Perception — subscribes to a Livox cloud and derives a forward-hazard summary
//! (the L0 proximity / below-clearance reflex inputs from the SAR spec). It is
//! the safety layer that stops the robot driving into trees or off cliffs,
//! regardless of pose error.
//!
//! Frame note: cloud points are WORLD coordinates and the cloud header carries
//! the sensor's true WORLD position — so we anchor the forward corridor at the
//! sensor's real position (immune to the VN GNSS bias) and take "ahead" from the
//! robot heading.

pub mod cloud;
pub mod costmap;

use crate::position::Pose;
use costmap::CostMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::watch;

// Forward corridor + thresholds (metres). Tuned against the live sim.
const FWD_RANGE: f64 = 6.0; // how far ahead we look
const CORRIDOR_HALF_W: f64 = 0.8; // half-width of the path corridor (~robot width + margin)
const NEAR_MIN: f64 = 0.4; // ignore returns closer than this (self / blind spot)
const OBSTACLE_MIN_H: f64 = 0.30; // a return this far above ground = solid obstacle
const OBSTACLE_MAX_H: f64 = 3.0; // ...up to here (ignore high canopy)
const GROUND_TOL: f64 = 0.25; // |z-ground| under this = a ground return
const LIDAR_HEIGHT: f64 = 0.85; // sensor height above ground (for ground_z estimate)
const BIN_W: f64 = 0.5; // along-distance bin for the ground-continuity (cliff) check
const NBINS: usize = (FWD_RANGE / BIN_W) as usize;

// Polar obstacle map (drive-forward frame) for local avoidance / steering.
pub const FOV_DEG: f64 = 70.0; // steer within +/- this of forward
const SECTOR_DEG: f64 = 5.0;
pub const NSEC: usize = 29; // 2*FOV/SECTOR + 1
const AVOID_LOOKAHEAD: f64 = 6.0; // consider obstacles within this for steering
const ROBOT_CLEAR: f64 = 1.0; // half-width + safety margin (m) widened around obstacles

#[inline]
fn sector_angle_deg(i: usize) -> f64 {
    -FOV_DEG + (i as f64) * SECTOR_DEG
}

/// Pick a clear steering direction (rad, drive-forward relative) nearest the goal
/// direction, given the blocked-sector map. `None` => no clear sector (stop).
pub fn steer(goal_rel_rad: f64, blocked: &[bool; NSEC]) -> Option<f64> {
    let goal_deg = goal_rel_rad.to_degrees().clamp(-FOV_DEG, FOV_DEG);
    let mut best: Option<(f64, f64)> = None; // (deviation, angle_deg)
    for (i, b) in blocked.iter().enumerate() {
        if *b {
            continue;
        }
        let a = sector_angle_deg(i);
        let dev = (a - goal_deg).abs();
        if best.map_or(true, |(bd, _)| dev < bd) {
            best = Some((dev, a));
        }
    }
    best.map(|(_, a)| a.to_radians())
}

#[derive(Debug, Clone, Copy)]
pub struct Hazard {
    pub obstacle_ahead: bool,
    pub cliff_ahead: bool,
    pub obstacle_dist: f64, // INF if none
    pub cliff_dist: f64,    // INF if none
    /// Per-sector blockage (drive-forward frame, -FOV..+FOV) for local avoidance.
    pub blocked: [bool; NSEC],
    pub stamp: Instant,
}

impl Hazard {
    pub fn none() -> Self {
        Self::clear()
    }
    fn clear() -> Self {
        Self {
            obstacle_ahead: false,
            cliff_ahead: false,
            obstacle_dist: f64::INFINITY,
            cliff_dist: f64::INFINITY,
            blocked: [false; NSEC],
            stamp: Instant::now(),
        }
    }
    /// Nearest forward hazard (obstacle or ground edge), metres.
    pub fn fwd_clear(&self) -> f64 {
        self.obstacle_dist.min(self.cliff_dist)
    }
}

/// Analyse one world-frame cloud against the robot heading. `min_ground_per_bin`
/// is the ground-density floor below which a near bin reads as a drop-off.
pub fn analyze(
    points: &[[f32; 3]],
    sensor: [f64; 3],
    heading_rad: f64,
    obstacle_stop: f64,
    cliff_stop: f64,
    min_ground_per_bin: u32,
) -> (Hazard, [u32; NBINS]) {
    let (hx, hy) = (heading_rad.cos(), heading_rad.sin());
    let ground_z = sensor[2] - LIDAR_HEIGHT;
    let mut obstacle_dist = f64::INFINITY;
    let mut ground_bins = [0u32; NBINS];
    let mut blocked = [false; NSEC];

    for p in points {
        let dx = p[0] as f64 - sensor[0];
        let dy = p[1] as f64 - sensor[1];
        let along = dx * hx + dy * hy; // forward
        let lat_s = -dx * hy + dy * hx; // signed lateral (+ = left)
        if along < NEAR_MIN {
            continue;
        }
        let h = p[2] as f64 - ground_z;
        let is_obstacle = h > OBSTACLE_MIN_H && h < OBSTACLE_MAX_H;

        // forward CORRIDOR (narrow): straight-ahead obstacle/cliff for stop logic
        if along <= FWD_RANGE && lat_s.abs() <= CORRIDOR_HALF_W {
            if is_obstacle {
                obstacle_dist = obstacle_dist.min(along);
            } else if h.abs() < GROUND_TOL {
                let bin = ((along - NEAR_MIN) / BIN_W) as usize;
                if bin < NBINS {
                    ground_bins[bin] += 1;
                }
            }
        }

        // polar AVOIDANCE map (wide): block the sectors an obstacle spans, widened
        // by the robot half-width at its range.
        if is_obstacle && along <= AVOID_LOOKAHEAD {
            let r = (along * along + lat_s * lat_s).sqrt();
            let ang = lat_s.atan2(along).to_degrees();
            if ang.abs() <= FOV_DEG + SECTOR_DEG {
                let hw = (ROBOT_CLEAR / r.max(0.3)).atan().to_degrees();
                for (i, b) in blocked.iter_mut().enumerate() {
                    if (sector_angle_deg(i) - ang).abs() <= hw {
                        *b = true;
                    }
                }
            }
        }
    }

    // Cliff = the nearest bin (closer than any obstacle) where the ground
    // disappears. Bins behind an obstacle are occluded, so don't count them.
    let obstacle_bin = if obstacle_dist.is_finite() {
        ((obstacle_dist - NEAR_MIN) / BIN_W) as usize
    } else {
        NBINS
    };
    let mut cliff_dist = f64::INFINITY;
    for (i, c) in ground_bins.iter().enumerate().take(obstacle_bin) {
        if *c < min_ground_per_bin {
            cliff_dist = NEAR_MIN + (i as f64) * BIN_W;
            break;
        }
    }

    let hz = Hazard {
        obstacle_ahead: obstacle_dist < obstacle_stop,
        cliff_ahead: cliff_dist < cliff_stop,
        obstacle_dist,
        cliff_dist,
        blocked,
        stamp: Instant::now(),
    };
    (hz, ground_bins)
}

/// Subscribe to a Livox cloud port (Livox-style registration) and publish a live
/// [`Hazard`] derived against the latest heading.
pub async fn run(
    host: String,
    port: u16,
    obstacle_stop: f64,
    cliff_stop: f64,
    min_ground_per_bin: u32,
    drive_offset_deg: f64,
    pose_rx: watch::Receiver<Option<Pose>>,
    hazard_tx: watch::Sender<Hazard>,
    costmap_tx: watch::Sender<Option<Arc<CostMap>>>,
) -> anyhow::Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", 0)).await?;
    sock.connect((host.as_str(), port)).await?;
    sock.send(b"LVXSUB").await.ok();
    tracing::info!("perception: subscribed to livox {host}:{port}");

    let mut ra = cloud::Reassembler::default();
    let mut buf = vec![0u8; 64 * 1024];
    let mut keepalive = tokio::time::interval(Duration::from_secs(1));
    let mut pmap = costmap::PersistentMap::new(); // accumulates across frames
    let mut last_map_log = Instant::now();
    loop {
        tokio::select! {
            _ = keepalive.tick() => { let _ = sock.send(b"LVXSUB").await; }
            res = sock.recv(&mut buf) => {
                let n = res?;
                let Some(pkt) = cloud::decode(&buf[..n]) else { continue };
                if let Some((points, pose)) = ra.feed(pkt) {
                    // Look along the DRIVE-forward axis (VN yaw + offset), same as
                    // GoTo — the robot drives 90deg off its VN heading.
                    let heading = match *pose_rx.borrow() {
                        Some(p) => p.heading_enu_rad() + drive_offset_deg.to_radians(),
                        None => continue, // no heading yet
                    };
                    let (hz, _bins) = analyze(
                        &points, pose.pos, heading,
                        obstacle_stop, cliff_stop, min_ground_per_bin,
                    );
                    let _ = hazard_tx.send(hz);
                    // accumulate into the PERSISTENT world map, publish a classified
                    // snapshot for the planner (remembers terrain already seen).
                    pmap.update(&points, pose.pos);
                    let _ = costmap_tx.send(Some(Arc::new(pmap.to_costmap(pose.pos))));
                    if last_map_log.elapsed() > Duration::from_secs(3) {
                        last_map_log = Instant::now();
                        tracing::info!("map: {} cells known (persistent)", pmap.known_cells());
                    }
                }
            }
        }
    }
}

/// Debug: subscribe for a few seconds and print decode stats + a hazard read, so
/// the Rust decoder can be validated against the live sim before wiring control.
pub async fn probe(host: &str, port: u16, pose_rx: watch::Receiver<Option<Pose>>) -> anyhow::Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", 0)).await?;
    sock.connect((host, port)).await?;
    sock.send(b"LVXSUB").await.ok();
    let mut ra = cloud::Reassembler::default();
    let mut buf = vec![0u8; 64 * 1024];
    let mut frames = 0;
    let deadline = Instant::now() + Duration::from_secs(6);
    let mut keepalive = tokio::time::interval(Duration::from_secs(1));
    while Instant::now() < deadline {
        tokio::select! {
            _ = keepalive.tick() => { let _ = sock.send(b"LVXSUB").await; }
            res = tokio::time::timeout(Duration::from_secs(2), sock.recv(&mut buf)) => {
                let Ok(Ok(n)) = res else { continue };
                let Some(pkt) = cloud::decode(&buf[..n]) else { continue };
                if let Some((points, pose)) = ra.feed(pkt) {
                    frames += 1;
                    let heading = pose_rx.borrow().map(|p| p.heading_enu_rad()).unwrap_or(0.0);
                    let (hz, bins) = analyze(&points, pose.pos, heading, 1.5, 2.5, 2);
                    tracing::info!(
                        "frame {frames}: {} pts, sensor ({:.1},{:.1},{:.1}) hdg {:.0}deg | \
                         obstacle {:.1}m ({}) cliff {:.1}m ({}) | ground bins {:?}",
                        points.len(), pose.pos[0], pose.pos[1], pose.pos[2], heading.to_degrees(),
                        hz.obstacle_dist, hz.obstacle_ahead, hz.cliff_dist, hz.cliff_ahead, bins,
                    );
                }
            }
        }
    }
    Ok(())
}
