//! Calibrate mission — a deployment quality-of-life routine that runs the robot
//! in a ~1 m workspace on flat ground and writes an autonomy calibration file.
//!
//! Sequence (per the deployment checklist):
//!   1. GYRO BIAS — sit still, average gyro_z → the rate the heading loop subtracts.
//!   2. FLIPPER ZEROS — drive each flipper down until current rises (it grounds /
//!      hits its hard stop), record the encoder position as the ABSOLUTE ZERO, then
//!      raise it. ODrives lose absolute position across reboots, so we persist it.
//!   3. DRIVE OFFSET + ODOM SCALE — nudge forward then back; the actual travel
//!      direction vs the VectorNav heading gives `drive_offset_deg` (no more manual
//!      probing per robot), and travel distance vs commanded gives the odom scale.
//!
//! Output: `calibration.toml`, loaded over the config on startup. Re-running the
//! calibrate mission overwrites it.

use crate::config::Config;
use crate::position::Pose;
use crate::transport::{command::CommandSink, discover::Discovery};
use anyhow::{Context, Result};
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::watch;

/// What the calibration produced (serialised to calibration.toml).
#[derive(Debug, Default)]
pub struct Calibration {
    pub drive_offset_deg: Option<f64>,
    pub odom_scale: Option<f64>,
    pub gyro_bias_z: Option<f64>,
    pub flipper_zeros: Vec<(u32, f64)>, // (node_id, pos_estimate rev)
}

const FLIPPER_NODES: [u32; 4] = [41, 42, 43, 44]; // FL, FR, BL, BR
const FLIPPER_DOWN_VEL: f64 = -1.5; // rev/s, gentle
const FLIPPER_CURRENT_TRIP: f64 = 2.0; // A above idle => grounded / hard stop
const FLIPPER_TIMEOUT: Duration = Duration::from_millis(4000);

/// Run the full calibration routine and write `calibration.toml`.
pub async fn run(
    cfg: &Config,
    disc: &Discovery,
    sink: &CommandSink,
    pose_rx: &watch::Receiver<Option<Pose>>,
) -> Result<()> {
    tracing::info!("CALIBRATE — place the robot on flat, clear ground (~1 m workspace)");
    let pose0 = wait_pose(pose_rx).await?;
    if pose0.roll_deg.abs() > 8.0 || pose0.pitch_deg.abs() > 8.0 {
        tracing::warn!(
            "not level (roll {:.0} pitch {:.0} deg) — calibration assumes flat ground",
            pose0.roll_deg, pose0.pitch_deg
        );
    }
    let mut cal = Calibration::default();

    // 1) GYRO BIAS (stationary)
    tracing::info!("[1/3] gyro bias — hold still…");
    let mut acc = 0.0;
    let mut n = 0u32;
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_millis(2500) {
        if let Some(p) = *pose_rx.borrow() {
            acc += p.yaw_rate;
            n += 1;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    if n > 0 {
        cal.gyro_bias_z = Some(acc / n as f64);
        tracing::info!("    gyro_bias_z = {:+.5} rad/s ({n} samples)", cal.gyro_bias_z.unwrap());
    }

    // 2) FLIPPER ZEROS
    tracing::info!("[2/3] flipper zeros — driving each flipper down to its hard stop…");
    for node in FLIPPER_NODES {
        match calibrate_flipper(disc, sink, node).await {
            Ok(zero) => {
                tracing::info!("    flipper {node}: zero @ pos {:+.4} rev", zero);
                cal.flipper_zeros.push((node, zero));
            }
            Err(e) => tracing::warn!("    flipper {node}: calibration skipped ({e:#})"),
        }
    }

    // 3) DRIVE OFFSET + ODOM SCALE (forward nudge, then return)
    tracing::info!("[3/3] drive-offset + odom-scale — nudging forward then back…");
    match calibrate_motion(cfg, disc, sink, pose_rx).await {
        Ok((off, scale)) => {
            tracing::info!("    drive_offset_deg = {:+.1}, odom_scale = {:.3}", off, scale);
            cal.drive_offset_deg = Some(off);
            cal.odom_scale = Some(scale);
        }
        Err(e) => tracing::warn!("    motion calibration skipped ({e:#})"),
    }

    write_calibration(&cal)?;
    tracing::info!("CALIBRATE complete — wrote calibration.toml");
    Ok(())
}

/// Drive one flipper down until its current rises (grounded), return the encoder
/// position at that point (the absolute zero), then raise it back.
async fn calibrate_flipper(disc: &Discovery, sink: &CommandSink, node: u32) -> Result<f64> {
    let id = format!("odrive_{node}");
    let dev = disc.get(&id).with_context(|| format!("{id} not in /discover"))?;
    let data_port = dev.data_port;
    let cmd_port = dev.command_port;
    // read socket for this flipper's telemetry
    let tele = UdpSocket::bind(("0.0.0.0", 0)).await?;
    let host = sink.host();
    tele.connect((host.as_str(), data_port)).await?;
    tele.send(b"\x01\x01\x00\x00{\"interval_ms\":20}").await.ok(); // best-effort subscribe

    let idle = flipper_current(&tele).await.unwrap_or(0.4);
    // ARM the flipper axis (clear faults -> closed loop), else input_vel is
    // acknowledged but never actuated (mock.rs is_armed gate). Same as the tracks.
    sink.send(cmd_port, &serde_json::json!({"clear_errors": true})).await.ok();
    sink.send(cmd_port, &serde_json::json!({"axis_state": 8, "input_vel": 0.0})).await.ok();
    tokio::time::sleep(Duration::from_millis(150)).await;
    // drive down (input_vel sign -> step direction: <0 = down), STREAMING each tick
    // so the axis stays armed (watchdog) and the stepped-flipper actuator keeps moving.
    let down = serde_json::json!({"axis_state": 8, "input_vel": FLIPPER_DOWN_VEL});
    let t0 = Instant::now();
    let mut zero = 0.0;
    let mut tripped = false;
    while t0.elapsed() < FLIPPER_TIMEOUT {
        sink.send(cmd_port, &down).await.ok();
        if let Some((cur, pos)) = flipper_state(&tele).await {
            zero = pos;
            if cur > idle + FLIPPER_CURRENT_TRIP {
                tripped = true;
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    // raise back up for ~the time we drove down (also streamed), then idle
    let down_t = t0.elapsed().min(FLIPPER_TIMEOUT);
    let up = serde_json::json!({"axis_state": 8, "input_vel": -FLIPPER_DOWN_VEL});
    let t1 = Instant::now();
    while t1.elapsed() < down_t {
        sink.send(cmd_port, &up).await.ok();
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    sink.send(cmd_port, &serde_json::json!({"axis_state": 8, "input_vel": 0.0})).await.ok();
    sink.send(cmd_port, &serde_json::json!({"axis_state": 1})).await.ok(); // idle
    if !tripped {
        // not a failure for movement — log it; the zero is the end-of-travel pos
        tracing::info!("    flipper {node}: no current trip (using end-of-travel pos)");
    }
    Ok(zero)
}

/// Read one (iq_measured, pos_estimate) from a flipper telemetry socket.
async fn flipper_state(tele: &UdpSocket) -> Option<(f64, f64)> {
    let mut buf = [0u8; 8192];
    let n = tokio::time::timeout(Duration::from_millis(200), tele.recv(&mut buf)).await.ok()?.ok()?;
    // packet = [ver|type|seq:2|json]
    let json = &buf[4..n];
    let v: serde_json::Value = serde_json::from_slice(json).ok()?;
    let cur = v.get("iq_measured").and_then(|x| x.as_f64())?;
    let pos = v.get("pos_estimate").and_then(|x| x.as_f64()).unwrap_or(0.0);
    Some((cur, pos))
}

async fn flipper_current(tele: &UdpSocket) -> Option<f64> {
    flipper_state(tele).await.map(|(c, _)| c)
}

/// Nudge forward, measure actual travel vs VN heading → (drive_offset_deg, odom_scale).
async fn calibrate_motion(
    cfg: &Config,
    disc: &Discovery,
    sink: &CommandSink,
    pose_rx: &watch::Receiver<Option<Pose>>,
) -> Result<(f64, f64)> {
    use crate::control::tracks::TracksController;
    let mut tracks = TracksController::new(&cfg.tracks, sink, disc)?;
    let start = wait_pose(pose_rx).await?;
    let dt = 0.02;
    let drive = 0.30_f64;
    let dur = Duration::from_millis(3000);
    // Creep forward, sampling the INSTANTANEOUS travel-vs-heading offset each step.
    // Averaging these (circularly) is robust to a curving creep: even if the robot
    // arcs, each step's (travel bearing − heading) is the same mount offset, so the
    // mean is clean — unlike the old net-displacement bearing (which the curve bent,
    // and which once wrote a 26°-off drive_offset that drove the robot off a cliff).
    let mut offsets_rad: Vec<f64> = Vec::new();
    let mut prev = start;
    let mut last_sample = Instant::now();
    let t0 = Instant::now();
    while t0.elapsed() < dur {
        tracks.drive(drive, drive, dt).await.ok();
        tokio::time::sleep(Duration::from_millis(20)).await;
        if last_sample.elapsed() >= Duration::from_millis(120) {
            last_sample = Instant::now();
            if let Some(p) = *pose_rx.borrow() {
                let (dx, dy) = (p.x - prev.x, p.y - prev.y);
                if dx.hypot(dy) > 0.03 {
                    offsets_rad.push(dy.atan2(dx) - p.heading_enu_rad());
                    prev = p;
                }
            }
        }
    }
    tracks.idle().await.ok();
    tokio::time::sleep(Duration::from_millis(300)).await;
    let end = wait_pose(pose_rx).await?;
    if offsets_rad.len() < 3 {
        anyhow::bail!("not enough motion samples ({}) — check drive", offsets_rad.len());
    }
    let drive_offset = circular_mean_deg(&offsets_rad);

    let travelled = (end.x - start.x).hypot(end.y - start.y);
    let nominal = drive * cfg.tracks.max_velocity * 0.0899 * 2.0 * std::f64::consts::PI
        * dur.as_secs_f64();
    let odom_scale = travelled / nominal.max(1e-6);

    // return to start (drive backward the same time)
    let t1 = Instant::now();
    while t1.elapsed() < dur {
        tracks.drive(-drive, -drive, dt).await.ok();
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    tracks.idle().await.ok();
    Ok((drive_offset, odom_scale))
}

/// Circular mean of angles (radians in, degrees out, wrapped to (-180,180]).
/// Averages via the unit-vector sum so it handles wraparound (e.g. +170/-170 -> 180).
fn circular_mean_deg(angles_rad: &[f64]) -> f64 {
    let (s, c) = angles_rad
        .iter()
        .fold((0.0, 0.0), |(s, c), a| (s + a.sin(), c + a.cos()));
    wrap_deg(s.atan2(c).to_degrees())
}

fn wrap_deg(d: f64) -> f64 {
    let mut d = d % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d <= -180.0 {
        d += 360.0;
    }
    d
}

async fn wait_pose(pose_rx: &watch::Receiver<Option<Pose>>) -> Result<Pose> {
    for _ in 0..100 {
        if let Some(p) = *pose_rx.borrow() {
            return Ok(p);
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    anyhow::bail!("no pose for calibration")
}

/// Write calibration.toml next to the config (overwrites on re-run).
fn write_calibration(cal: &Calibration) -> Result<()> {
    let mut s = String::from("# autonomy calibration — written by the Calibrate mission.\n");
    s.push_str("# Overwritten each time the calibrate mission runs. Loaded over autonomy.toml.\n\n");
    if let Some(o) = cal.drive_offset_deg {
        s.push_str(&format!("drive_offset_deg = {o:.2}\n"));
    }
    if let Some(sc) = cal.odom_scale {
        s.push_str(&format!("odom_scale = {sc:.4}\n"));
    }
    if let Some(g) = cal.gyro_bias_z {
        s.push_str(&format!("gyro_bias_z = {g:.6}\n"));
    }
    if !cal.flipper_zeros.is_empty() {
        s.push_str("\n[flipper_zeros]   # absolute encoder zero per node (ODrives lose this on reboot)\n");
        for (node, z) in &cal.flipper_zeros {
            s.push_str(&format!("odrive_{node} = {z:.4}\n"));
        }
    }
    std::fs::write("calibration.toml", s).context("writing calibration.toml")?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circular_mean_handles_wrap() {
        let r = |d: f64| d.to_radians();
        // symmetric around 0 -> ~0
        let m = circular_mean_deg(&[r(10.0), r(-10.0), r(5.0), r(-5.0)]);
        assert!(m.abs() < 1e-6, "m={m}");
        // +170 and -170 average to 180 (not 0) — the wraparound the old net-bearing missed
        let m2 = circular_mean_deg(&[r(170.0), r(-170.0)]);
        assert!((m2.abs() - 180.0).abs() < 1e-6, "m2={m2}");
        // a steady ~90 with jitter -> ~90
        let m3 = circular_mean_deg(&[r(88.0), r(92.0), r(90.0), r(91.0)]);
        assert!((m3 - 90.25).abs() < 0.5, "m3={m3}");
    }

    #[test]
    fn wrap_deg_range() {
        assert!((wrap_deg(190.0) - (-170.0)).abs() < 1e-9);
        assert!((wrap_deg(-190.0) - 170.0).abs() < 1e-9);
        assert!((wrap_deg(90.0) - 90.0).abs() < 1e-9);
    }
}
