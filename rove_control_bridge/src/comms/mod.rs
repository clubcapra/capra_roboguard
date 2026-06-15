//! Front door — the udp_multiplexer protos the Steam Deck speaks.
//!
//! The orchestrator receives operator intent here: RoveControl (teleop drive /
//! flippers / arm / gripper), Mission (a behaviour-graph to run), CameraSwitch,
//! and Estop. Teleop is decoded into a [`TeleopIntent`] that the control loop
//! arbitrates against the running mission (teleop preempts; estop clamps).

use prost::Message;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::watch;

/// prost-generated types for the `telemetry` proto package (RoveControl, Mission,
/// CameraSwitch, RoveTelemetry — same definitions the udp_multiplexer compiles).
pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/telemetry.rs"));
}

/// Decoded teleop intent (normalised), with a receipt stamp for staleness.
#[derive(Debug, Clone, Copy)]
pub struct TeleopIntent {
    pub left: f32,           // track left  {-1,1}
    pub right: f32,          // track right {-1,1}
    pub flippers: [i32; 4],  // fl, fr, rl, rr step {-1,0,+1}
    pub has_arm: bool,       // an Ovis (arm 6-DOF twist) was present
    pub gripper: Option<u32>,
    pub stamp: Instant,
}

impl TeleopIntent {
    /// Any non-zero drive/flipper command (used to decide teleop-preempts-mission).
    pub fn is_active(&self) -> bool {
        self.left.abs() > 1e-3
            || self.right.abs() > 1e-3
            || self.flippers.iter().any(|&s| s != 0)
            || self.has_arm
    }
}

fn intent_from(rc: &proto::RoveControl) -> TeleopIntent {
    let (left, right) = rc.tracks.as_ref().map_or((0.0, 0.0), |t| (t.left_vel, t.right_vel));
    let flippers = rc.flippers.as_ref().map_or([0; 4], |f| [f.fl, f.fr, f.rl, f.rr]);
    TeleopIntent {
        left,
        right,
        flippers,
        has_arm: rc.ovis.is_some(),
        gripper: rc.gripper.as_ref().map(|g| g.position),
        stamp: Instant::now(),
    }
}

/// Listen for Estop datagrams (one flag): true = stop everything, false = clear.
/// Publishes the latest estop state for the control loop to honour.
pub async fn run_estop_listener(
    port: u16,
    tx: watch::Sender<bool>,
) -> anyhow::Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", port)).await?;
    tracing::info!("estop front door: listening for Estop on :{port}");
    let mut buf = vec![0u8; 64];
    loop {
        let n = sock.recv(&mut buf).await?;
        match proto::Estop::decode(&buf[..n]) {
            Ok(e) => {
                tracing::warn!("ESTOP {} from operator", if e.active { "ENGAGED" } else { "CLEARED" });
                let _ = tx.send(e.active);
            }
            Err(err) => tracing::warn!("dropping malformed Estop ({n} B): {err}"),
        }
    }
}

/// Listen for Mission datagrams (operator uploads a behaviour graph), publish the
/// decoded proto for the control loop to compile, and reply MissionResult.
pub async fn run_mission_listener(
    port: u16,
    tx: watch::Sender<Option<proto::Mission>>,
) -> anyhow::Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", port)).await?;
    tracing::info!("mission front door: listening for Mission on :{port}");
    let mut buf = vec![0u8; 65536];
    loop {
        let (n, addr) = sock.recv_from(&mut buf).await?;
        match proto::Mission::decode(&buf[..n]) {
            Ok(m) => {
                tracing::info!("MISSION received: '{}' ({} steps)", m.name, m.sequence.len());
                let _ = tx.send(Some(m));
                let res = proto::MissionResult {
                    status: proto::mission_result::Status::Accepted as i32,
                    detail: String::new(),
                    rejected_step: 0,
                };
                let mut out = Vec::new();
                if res.encode(&mut out).is_ok() {
                    sock.send_to(&out, addr).await.ok();
                }
            }
            Err(e) => tracing::warn!("dropping malformed Mission ({n} B): {e}"),
        }
    }
}

/// Listen for RoveControl datagrams (teleop): publish the [`TeleopIntent`] so the
/// control loop preempts the mission, forward tracks + flippers + arm twist (`ovis`)
/// to the IK engine, and send the gripper straight to the robot API.
pub async fn run_teleop_listener(
    port: u16,
    tx: watch::Sender<Option<TeleopIntent>>,
    ik: Option<std::sync::Arc<crate::ik::IkForwarder>>,
    gripper: std::sync::Arc<crate::gripper::GripperSender>,
) -> anyhow::Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", port)).await?;
    tracing::info!(
        "teleop front door: listening for RoveControl on :{port} (tracks/flippers/ovis -> IK engine, gripper -> API); ik forwarder: {}",
        if ik.is_some() { "ENABLED" } else { "DISABLED — arm/drums/flippers WILL BE DROPPED (engine unreachable at startup)" },
    );
    let mut buf = vec![0u8; 8192];
    let mut rx_count: u64 = 0;
    let mut bad_count: u64 = 0;
    let mut last_trace = Instant::now();
    loop {
        let (n, addr) = sock.recv_from(&mut buf).await?;
        match proto::RoveControl::decode(&buf[..n]) {
            Ok(rc) => {
                rx_count += 1;
                if rx_count == 1 {
                    tracing::info!("RX ✓ first RoveControl from {addr} ({n} B) — teleop IS reaching the bridge");
                }
                let intent = intent_from(&rc);
                let _ = tx.send(Some(intent));
                let arm = rc.ovis.is_some();
                if let Some(ik) = &ik {
                    // arm twist -> IK engine (it resolves IK + collision, drives the arm)
                    if let Some(ov) = &rc.ovis {
                        let o = ov.orientation.as_ref();
                        let p = ov.position.as_ref();
                        ik.send_ovis(
                            [o.map_or(0.0, |x| x.yaw), o.map_or(0.0, |x| x.pitch), o.map_or(0.0, |x| x.roll)],
                            [p.map_or(0.0, |x| x.x), p.map_or(0.0, |x| x.y), p.map_or(0.0, |x| x.z)],
                        )
                        .await;
                    }
                    // tracks (drums) + flippers -> IK engine. The control loop never
                    // drives the drums for teleop, so there's no double-command.
                    ik.send_drive(intent.flippers, intent.left, intent.right).await;
                }
                // gripper -> robot API DIRECT (the IK engine has no gripper).
                if let Some(pos) = intent.gripper {
                    gripper.send(pos).await;
                }
                // ~1 Hz trace: what arrived and where it was forwarded. Proves the
                // bridge is receiving AND fanning out (or shows where it stops).
                if last_trace.elapsed() >= Duration::from_secs(1) {
                    last_trace = Instant::now();
                    tracing::info!(
                        "RX #{rx_count} {addr}: tracks L{:+.2} R{:+.2} flip {:?} arm={arm} grip={:?} => engine[{}] (drive+arm), gripper->API={:?}",
                        intent.left, intent.right, intent.flippers, intent.gripper,
                        if ik.is_some() { "ON" } else { "OFF: DROPPED" }, intent.gripper,
                    );
                }
            }
            Err(e) => {
                bad_count += 1;
                if last_trace.elapsed() >= Duration::from_secs(1) {
                    last_trace = Instant::now();
                    tracing::warn!("RX bad: malformed RoveControl from {addr} ({n} B), {bad_count} total: {e}");
                }
            }
        }
    }
}
