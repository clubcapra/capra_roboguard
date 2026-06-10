//! Command path: autonomy → sim.
//!
//! The sim accepts a single **aggregate `RoveControl`** frame on its `control`
//! channel (see `rove_sim/api/control_bridge.py`): tracks + flippers + ovis +
//! gripper. Several mock drivers (the four ODrives, the gripper) each own a
//! *slice* of that frame, so they all write into one `SharedControl` value and a
//! single publisher task pushes the merged frame to the sim. Letting each mock
//! open its own socket to `:5020` would race — last writer would clobber the
//! others. This module is that shared accumulator + publisher.

use std::sync::{Arc, RwLock};
use std::time::Duration;

use serde_json::{json, Value};
use tokio::net::UdpSocket;

use crate::protocol::packet::Packet;

/// The aggregate `RoveControl` frame, shared across the actuation mocks.
pub type SharedControl = Arc<RwLock<Value>>;

/// A fresh, all-zero control frame (robot idle).
pub fn default_control_frame() -> SharedControl {
    Arc::new(RwLock::new(json!({
        "tracks":   {"left": 0.0, "right": 0.0},
        "flippers": {"fl": 0, "fr": 0, "rl": 0, "rr": 0},
        "ovis":     {"vx": 0.0, "vy": 0.0, "vz": 0.0, "wx": 0.0, "wy": 0.0, "wz": 0.0},
        "gripper":  {"position": 0},
        "timestamp_us": 0
    })))
}

/// Set one track side ("left"|"right") to a normalized [-1, 1] velocity.
pub fn set_track(control: &SharedControl, side: &str, value: f64) {
    if let Ok(mut c) = control.write() {
        if let Some(tracks) = c.get_mut("tracks").and_then(Value::as_object_mut) {
            tracks.insert(side.to_string(), json!(value.clamp(-1.0, 1.0)));
        }
    }
}

/// Set the gripper position (0=open .. 255=closed).
pub fn set_gripper(control: &SharedControl, position: i64) {
    if let Ok(mut c) = control.write() {
        if let Some(g) = c.get_mut("gripper").and_then(Value::as_object_mut) {
            g.insert("position".to_string(), json!(position.clamp(0, 255)));
        }
    }
}

/// Set one flipper ("fl"|"fr"|"rl"|"rr") deploy command (-1 down, +1 up, 0 hold).
pub fn set_flipper(control: &SharedControl, key: &str, cmd: i64) {
    if let Ok(mut c) = control.write() {
        if let Some(f) = c.get_mut("flippers").and_then(Value::as_object_mut) {
            f.insert(key.to_string(), json!(cmd.clamp(-1, 1)));
        }
    }
}

/// Publish the shared control frame to the sim's `control` port at a fixed rate.
/// One task owns the socket so the per-mock writers never contend on the wire.
pub fn spawn_control_publisher(host: String, port: u16, control: SharedControl) {
    tokio::spawn(async move {
        let sock = match UdpSocket::bind(("0.0.0.0", 0)).await {
            Ok(s) => s,
            Err(e) => {
                tracing::error!(error = %e, "control publisher: bind failed");
                return;
            }
        };
        let target = format!("{host}:{port}");
        tracing::info!(target = %target, "control publisher: sending RoveControl @ 50 Hz");
        let mut seq: u16 = 0;
        let mut tick = tokio::time::interval(Duration::from_millis(20)); // 50 Hz
        loop {
            tick.tick().await;
            let frame = control.read().unwrap().clone();
            seq = seq.wrapping_add(1);
            let bytes = Packet::data(seq, &frame).encode();
            if let Err(e) = sock.send_to(&bytes, &target).await {
                tracing::warn!(error = %e, "control publisher: send failed");
            }
        }
    });
}
