//! Real Mid-360 perception via the livox_bridge `LVXR` UDP stream.
//!
//! The sim path (`perception::run`) consumes world-frame `LVX2`. On the real robot
//! the two Mid-360s are raw on the LAN; the `livox_bridge` sidecar streams each
//! unit's points (sensor frame, m) + IMU as `LVXR` datagrams to localhost. Here we
//! take the BOTTOM lidar (.40, the ground sensor), IMU-LEVEL it (gravity → +Z), and
//! derive a **proximity** [`Hazard`]: stop if an obstacle or a ground edge/hole is
//! close in ANY near direction.
//!
//! Heading-independent on purpose: the body heading isn't trusted yet (side-mounted
//! VN), so "ahead" is unreliable — an omnidirectional near-stop is the safe first
//! reflex. Once heading is calibrated this narrows to the drive corridor + cost-map
//! routing. NOT a cost map yet (that needs a trusted world frame).

use super::Hazard;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::watch;

const HDR: usize = 20;
const MAGIC: &[u8; 4] = b"LVXR";
const BOTTOM_ID: u8 = 40; // the down-pointing ground sensor

// Proximity thresholds (m), tune in the field (start conservative).
const OBST_MIN_H: f64 = 0.30; // a return this far above local ground = obstacle
const OBST_MAX_H: f64 = 2.5;
const GROUND_TOL: f64 = 0.30; // |z - ground| under this = a ground return
const CLIFF_DROP: f64 = 0.6; // ground this much below the local ground = edge/hole
const NSECT: usize = 12; // angular sectors for the cliff (ground-continuity) check

/// One parsed LVXR datagram.
enum Msg {
    Points(Vec<[f32; 3]>),
    /// accel (m/s², specific force ⇒ points up at rest)
    Imu([f64; 3]),
}

fn parse(buf: &[u8]) -> Option<(u8, Msg)> {
    if buf.len() < HDR || &buf[0..4] != MAGIC {
        return None;
    }
    let typ = buf[5];
    let lidar_id = buf[6];
    let count = u16::from_le_bytes([buf[8], buf[9]]) as usize;
    let f32_at = |o: usize| f32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
    match typ {
        1 => {
            let mut pts = Vec::with_capacity(count);
            for i in 0..count {
                let o = HDR + i * 12;
                if o + 12 > buf.len() {
                    break;
                }
                pts.push([f32_at(o), f32_at(o + 4), f32_at(o + 8)]);
            }
            Some((lidar_id, Msg::Points(pts)))
        }
        2 if buf.len() >= HDR + 24 => {
            // gyro_xyz then accel_xyz
            let a = [f32_at(HDR + 12) as f64, f32_at(HDR + 16) as f64, f32_at(HDR + 20) as f64];
            Some((lidar_id, Msg::Imu(a)))
        }
        _ => None,
    }
}

/// Rotation (3x3) taking unit `a` → unit `b`.
fn rot_a_to_b(a: [f64; 3], b: [f64; 3]) -> [[f64; 3]; 3] {
    let na = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt().max(1e-9);
    let a = [a[0] / na, a[1] / na, a[2] / na];
    let c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let v = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    let s = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if s < 1e-8 {
        // parallel or antiparallel
        return if c > 0.0 {
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        } else {
            [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]] // 180° about x
        };
    }
    let k = (1.0 - c) / (s * s);
    // R = I + [v]x + [v]x^2 * k
    [
        [1.0 - k * (v[1] * v[1] + v[2] * v[2]), -v[2] + k * v[0] * v[1], v[1] + k * v[0] * v[2]],
        [v[2] + k * v[0] * v[1], 1.0 - k * (v[0] * v[0] + v[2] * v[2]), -v[0] + k * v[1] * v[2]],
        [-v[1] + k * v[0] * v[2], v[0] + k * v[1] * v[2], 1.0 - k * (v[0] * v[0] + v[1] * v[1])],
    ]
}

/// Level a cloud so the measured gravity (accel, points up) maps to +Z (up).
fn level(points: &[[f32; 3]], accel: [f64; 3]) -> Vec<[f64; 3]> {
    let r = rot_a_to_b(accel, [0.0, 0.0, 1.0]);
    points
        .iter()
        .map(|p| {
            let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
            [
                r[0][0] * x + r[0][1] * y + r[0][2] * z,
                r[1][0] * x + r[1][1] * y + r[1][2] * z,
                r[2][0] * x + r[2][1] * y + r[2][2] * z,
            ]
        })
        .collect()
}

fn det3(m: [[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Least-squares ground plane z = a·x + b·y + c from the per-cell LOWEST z
/// (ignores the cage/obstacles sitting above the ground). Absorbs a slope or a
/// residual leveling error, so flat ground at range isn't mis-read as obstacle.
/// `None` if too few ground cells.
fn fit_ground_plane(pts: &[[f64; 3]], self_r: f64) -> Option<(f64, f64, f64)> {
    use std::collections::HashMap;
    const CELL: f64 = 0.5;
    let mut low: HashMap<(i32, i32), f64> = HashMap::new();
    for p in pts {
        let r = p[0].hypot(p[1]);
        if r <= self_r || r > 6.0 {
            continue;
        }
        let k = ((p[0] / CELL).floor() as i32, (p[1] / CELL).floor() as i32);
        let e = low.entry(k).or_insert(f64::INFINITY);
        if p[2] < *e {
            *e = p[2];
        }
    }
    if low.len() < 8 {
        return None;
    }
    let (mut sxx, mut sxy, mut sx, mut syy, mut sy, mut n) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    let (mut sxz, mut syz, mut sz) = (0.0, 0.0, 0.0);
    for ((ix, iy), z) in &low {
        let (x, y, z) = ((*ix as f64 + 0.5) * CELL, (*iy as f64 + 0.5) * CELL, *z);
        sxx += x * x; sxy += x * y; sx += x; syy += y * y; sy += y; n += 1.0;
        sxz += x * z; syz += y * z; sz += z;
    }
    let m = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]];
    let det = det3(m);
    if det.abs() < 1e-9 {
        return None;
    }
    let col = |c: usize| {
        let mut mm = m;
        for i in 0..3 {
            mm[i][c] = [sxz, syz, sz][i];
        }
        det3(mm) / det
    };
    Some((col(0), col(1), col(2)))
}

/// True if a point's bearing (lidar leveled frame, deg) is within the forward arc
/// `[fwd_offset ± fwd_arc]`. `fwd_arc >= 180` ⇒ omnidirectional.
fn in_arc(x: f64, y: f64, fwd_offset: f64, fwd_arc: f64) -> bool {
    if fwd_arc >= 180.0 {
        return true;
    }
    let bearing = y.atan2(x).to_degrees();
    let mut d = (bearing - fwd_offset) % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d < -180.0 {
        d += 360.0;
    }
    d.abs() <= fwd_arc
}

/// Proximity hazard from a leveled (z-up) ground-sensor cloud, restricted to the
/// forward arc (so clutter behind/beside — e.g. a bush field — doesn't stop us).
/// Returns the nearest obstacle range and the nearest ground edge/hole range.
pub fn analyze(
    leveled: &[[f64; 3]],
    self_r: f64,
    fwd_offset: f64,
    fwd_arc: f64,
    obstacle_stop: f64,
    cliff_stop: f64,
) -> (Hazard, f64) {
    let mut hz = Hazard::none();
    if leveled.len() < 200 {
        return (hz, f64::NAN); // too sparse to trust
    }
    // robust ground plane (fixes the single-ground-z mis-classification)
    let (ga, gb, gc) = match fit_ground_plane(leveled, self_r) {
        Some(p) => p,
        None => return (hz, f64::NAN),
    };

    // nearest OBSTACLE: a return well above the ground plane, in the forward arc
    let mut obstacle = f64::INFINITY;
    let mut obs_bearing = f64::NAN; // lidar-frame bearing of the nearest obstacle (deg)
    // CLIFF: per-sector nearest range where the ground DROPS away (edge/hole)
    let mut sect_drop = [f64::INFINITY; NSECT];
    for p in leveled {
        let r = p[0].hypot(p[1]);
        if r <= self_r || r > 6.0 {
            continue;
        }
        if !in_arc(p[0], p[1], fwd_offset, fwd_arc) {
            continue; // outside the forward arc — ignore (e.g. the bush field behind)
        }
        let h = p[2] - (ga * p[0] + gb * p[1] + gc);
        if h > OBST_MIN_H && h < OBST_MAX_H {
            if r < obstacle {
                obstacle = r;
                obs_bearing = p[1].atan2(p[0]).to_degrees();
            }
        } else if h < -CLIFF_DROP {
            // ground falls away here → edge/hole
            let a = p[1].atan2(p[0]); // [-pi, pi]
            let s = (((a + std::f64::consts::PI) / (2.0 * std::f64::consts::PI)) * NSECT as f64)
                as usize
                % NSECT;
            sect_drop[s] = sect_drop[s].min(r);
        }
    }
    let cliff = sect_drop.iter().cloned().fold(f64::INFINITY, f64::min);

    hz.obstacle_dist = obstacle;
    hz.cliff_dist = cliff;
    hz.obstacle_ahead = obstacle < obstacle_stop;
    hz.cliff_ahead = cliff < cliff_stop;
    hz.stamp = Instant::now();
    (hz, obs_bearing)
}

/// Subscribe to the sidecar's `LVXR` stream and publish a proximity [`Hazard`]
/// from the BOTTOM lidar. (Cost map left empty for now — needs a trusted world
/// frame; the planner stays off and the control loop drives straight + stops on
/// this hazard.)
#[allow(clippy::too_many_arguments)]
pub async fn run(
    pts_port: u16,
    imu_port: u16,
    self_r: f64,
    fwd_offset: f64,
    fwd_arc: f64,
    obstacle_stop: f64,
    cliff_stop: f64,
    require_lidar: bool,
    hazard_tx: watch::Sender<Hazard>,
) -> anyhow::Result<()> {
    let psock = UdpSocket::bind(("0.0.0.0", pts_port)).await?;
    let isock = UdpSocket::bind(("0.0.0.0", imu_port)).await?;
    let arc = if fwd_arc >= 180.0 { "omni".to_string() } else { format!("fwd {fwd_offset:.0}°±{fwd_arc:.0}°") };
    tracing::info!("perception(lvxr): bottom lidar .{BOTTOM_ID} on :{pts_port}/:{imu_port} — proximity reflex (self-mask {self_r:.1} m, {arc})");
    let mut pbuf = vec![0u8; 64 * 1024];
    let mut ibuf = vec![0u8; 1024];
    let mut accel = [0.0f64, 0.0, 9.8]; // gravity estimate (low-pass)
    let mut batch: Vec<[f32; 3]> = Vec::new();
    let mut last_points = Instant::now();
    let mut last_log = Instant::now();
    let mut emit = tokio::time::interval(Duration::from_millis(100));
    loop {
        tokio::select! {
            _ = emit.tick() => {
                if last_points.elapsed() > Duration::from_millis(1000) {
                    // No fresh lidar (sidecar down / lidar lost).
                    if require_lidar {
                        // SAFE default: never drive blind — emit a STOP hazard.
                        let mut hz = Hazard::none();
                        hz.obstacle_ahead = true;
                        hz.obstacle_dist = 0.0;
                        hz.stamp = Instant::now();
                        let _ = hazard_tx.send(hz);
                        if last_log.elapsed() >= Duration::from_secs(2) {
                            last_log = Instant::now();
                            tracing::warn!("perception(lvxr): NO lidar data — HOLDING (require_lidar=true)");
                        }
                    } else if last_log.elapsed() >= Duration::from_secs(5) {
                        last_log = Instant::now();
                        tracing::warn!("perception(lvxr): NO lidar data — driving BLIND (require_lidar=false)");
                    }
                    batch.clear();
                } else if !batch.is_empty() {
                    let leveled = level(&batch, accel);
                    let (hz, obs_bearing) =
                        analyze(&leveled, self_r, fwd_offset, fwd_arc, obstacle_stop, cliff_stop);
                    let _ = hazard_tx.send(hz);
                    batch.clear();
                    if last_log.elapsed() >= Duration::from_secs(2) {
                        last_log = Instant::now();
                        tracing::info!(
                            "perception(lvxr): obstacle {:.1} m @ {:.0}°{} | edge {:.1} m{}  (bearing => fwd_offset calib)",
                            hz.obstacle_dist, obs_bearing,
                            if hz.obstacle_ahead { " STOP" } else { "" },
                            hz.cliff_dist, if hz.cliff_ahead { " STOP" } else { "" },
                        );
                    }
                }
            }
            r = isock.recv(&mut ibuf) => {
                if let Ok(n) = r {
                    if let Some((BOTTOM_ID, Msg::Imu(a))) = parse(&ibuf[..n]) {
                        for k in 0..3 { accel[k] = 0.9 * accel[k] + 0.1 * a[k]; } // low-pass
                    }
                }
            }
            r = psock.recv(&mut pbuf) => {
                if let Ok(n) = r {
                    if let Some((BOTTOM_ID, Msg::Points(mut pts))) = parse(&pbuf[..n]) {
                        batch.append(&mut pts);
                        last_points = Instant::now();
                    }
                }
            }
        }
    }
}
