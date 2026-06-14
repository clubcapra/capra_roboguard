//! Telemetry OUT — the bridge republishes aggregated robot state as RoveTelemetry
//! to whoever subscribes (operator / udp_multiplexer).
//!
//! It folds the per-sensor rove_sensor_api frames (VectorNav, the 8 ODrive nodes,
//! gripper) into one [`proto::RoveTelemetry`] and, on a subscribe, pushes it at a
//! fixed rate. Subscribe = send any datagram to the port; the push stops if a
//! subscriber goes quiet for a few seconds (re-subscribe to keep it alive).

use crate::comms::proto;
use prost::Message;
use serde_json::Value;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;

/// The shared, continuously-updated aggregate.
pub type Shared = Arc<Mutex<proto::RoveTelemetry>>;

pub fn shared() -> Shared {
    Arc::new(Mutex::new(proto::RoveTelemetry::default()))
}

fn f32_of(v: &Value, k: &str) -> f32 {
    v.get(k).and_then(|x| x.as_f64()).unwrap_or(0.0) as f32
}
fn f64_of(v: &Value, k: &str) -> f64 {
    v.get(k).and_then(|x| x.as_f64()).unwrap_or(0.0)
}

/// Fold a VectorNav JSON frame into RoveTelemetry.vn300.
pub fn update_vn300(shared: &Shared, v: &Value) {
    let vn = proto::Vn300 {
        position: Some(proto::Position {
            lat: f64_of(v, "latitude"),
            lon: f64_of(v, "longitude"),
            alt: f32_of(v, "altitude"),
        }),
        orientation: Some(proto::Orientation {
            yaw: f32_of(v, "yaw"),
            pitch: f32_of(v, "pitch"),
            roll: f32_of(v, "roll"),
        }),
        velocity: Some(proto::Vector3 {
            x: f32_of(v, "vel_east"),
            y: f32_of(v, "vel_north"),
            z: f32_of(v, "vel_down"),
        }),
        accel: Some(proto::Vector3 {
            x: f32_of(v, "accel_x"),
            y: f32_of(v, "accel_y"),
            z: f32_of(v, "accel_z"),
        }),
        gyro: Some(proto::Vector3 {
            x: f32_of(v, "gyro_x"),
            y: f32_of(v, "gyro_y"),
            z: f32_of(v, "gyro_z"),
        }),
    };
    shared.lock().unwrap().vn300 = Some(vn);
}

/// Map a physical ODrive node id (31-34 drums, 41-44 flippers) to a slot 0..7.
fn node_slot(node_id: u32) -> Option<usize> {
    match node_id {
        31..=34 => Some((node_id - 31) as usize),     // 0..3 drums
        41..=44 => Some((node_id - 41 + 4) as usize), // 4..7 flippers
        _ => None,
    }
}

/// Fold an OdriveNodeState JSON frame into the right RoveTelemetry.odrives slot.
pub fn update_odrive(shared: &Shared, v: &Value) {
    let node_id = v.get("node_id").and_then(|x| x.as_u64()).unwrap_or(0) as u32;
    let Some(slot) = node_slot(node_id) else { return };
    let ds = proto::DriveNodeState {
        node_id,
        node_state: v.get("axis_state").and_then(|x| x.as_u64()).unwrap_or(0) as u32,
        node_temp_c: f32_of(v, "temperature"),
        motor_temp_c: f32_of(v, "temperature"),
        motor_amp: f32_of(v, "iq_measured"),
        active_errors: v.get("active_errors").and_then(|x| x.as_i64()).unwrap_or(0) as i32,
        latched_errors: v.get("axis_error").and_then(|x| x.as_i64()).unwrap_or(0) as i32,
        motor_pos: f32_of(v, "pos_estimate"),
        ..Default::default()
    };
    let mut g = shared.lock().unwrap();
    let od = g.odrives.get_or_insert_with(Default::default);
    match slot {
        0 => od.node_1 = Some(ds),
        1 => od.node_2 = Some(ds),
        2 => od.node_3 = Some(ds),
        3 => od.node_4 = Some(ds),
        4 => od.node_5 = Some(ds),
        5 => od.node_6 = Some(ds),
        6 => od.node_7 = Some(ds),
        _ => od.node_8 = Some(ds),
    }
}

/// Fold a Robotiq gripper JSON frame into RoveTelemetry.gripper.
pub fn update_gripper(shared: &Shared, v: &Value) {
    let pos = v.get("position").and_then(|x| x.as_u64()).unwrap_or(0) as u32;
    shared.lock().unwrap().gripper = Some(proto::Gripper { position: pos });
}

/// Subscribe listener + fixed-rate publisher of the aggregate to all subscribers.
pub async fn run_publisher(port: u16, shared: Shared, rate_hz: f64) -> anyhow::Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", port)).await?;
    tracing::info!("telemetry out: publishing RoveTelemetry to subscribers on :{port}");
    let mut subs: HashMap<std::net::SocketAddr, Instant> = HashMap::new();
    let mut buf = [0u8; 256];
    let mut tick = tokio::time::interval(Duration::from_secs_f64(1.0 / rate_hz.max(1.0)));
    let mut seq: u64 = 0;
    loop {
        tokio::select! {
            r = sock.recv_from(&mut buf) => {
                if let Ok((_, addr)) = r {
                    if subs.insert(addr, Instant::now()).is_none() {
                        tracing::info!("telemetry out: new subscriber {addr}");
                    }
                }
            }
            _ = tick.tick() => {
                subs.retain(|_, t| t.elapsed() < Duration::from_secs(3));
                if subs.is_empty() {
                    continue;
                }
                seq = seq.wrapping_add(1);
                let bytes = {
                    let mut g = shared.lock().unwrap();
                    g.timestamp_us = seq; // monotone marker (wall clock filled at the edge)
                    g.encode_to_vec()
                };
                for addr in subs.keys() {
                    sock.send_to(&bytes, addr).await.ok();
                }
            }
        }
    }
}
