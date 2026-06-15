//! Bridge → rove_ik_engine: forward all control-proto motion to the engine, which
//! resolves IK + self-collision and drives the actuators on rove_sensor_api.
//!
//! Two channels: the arm twist (`Ovis` proto on :9100) and drive teleop (flipper
//! steps + drum velocities as JSON on :9102). The gripper is NOT here — it goes
//! straight to the robot API (the engine has no gripper chain).

use prost::Message;
use tokio::net::UdpSocket;

/// prost types for the engine's `forgebot.engine` proto package.
pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/forgebot.engine.rs"));
}

/// A fire-and-forget UDP sender to the IK engine. Two channels: the arm twist
/// (`Ovis` proto on the engine's :9100) and drive teleop (flipper steps + drum
/// velocities as JSON on the engine's :9102). The engine translates both into
/// rove_sensor_api commands (with its own output gates).
pub struct IkForwarder {
    sock: UdpSocket,        // arm (Ovis) -> :9100
    drive_sock: UdpSocket,  // flippers + drums (JSON) -> :9102
    target_entity: String,
}

impl IkForwarder {
    pub async fn new(host: &str, port: u16, drive_port: u16, target_entity: String) -> anyhow::Result<Self> {
        let sock = UdpSocket::bind(("0.0.0.0", 0)).await?;
        sock.connect((host, port)).await?;
        let drive_sock = UdpSocket::bind(("0.0.0.0", 0)).await?;
        drive_sock.connect((host, drive_port)).await?;
        tracing::info!(
            "ik forwarder -> rove_ik_engine {host} arm:{port} drive:{drive_port} target='{target_entity}'");
        Ok(Self { sock, drive_sock, target_entity })
    }

    /// Forward drive teleop: flipper steps {-1,0,+1} (fl,fr,rl,rr) and normalised
    /// track velocities [-1,1]. The engine ramps flipper targets and velocity-
    /// commands the drums.
    pub async fn send_drive(&self, flippers: [i32; 4], left: f32, right: f32) {
        let body = serde_json::json!({
            "flippers": flippers,
            "tracks": { "left": left, "right": right },
        });
        if let Ok(buf) = serde_json::to_vec(&body) {
            self.drive_sock.send(&buf).await.ok();
        }
    }

    /// Forward a normalised arm twist (orientation yaw/pitch/roll + position xyz,
    /// each in {-1,1}). The engine scales by its max_lin/ang_vel and integrates.
    pub async fn send_ovis(&self, orientation: [f32; 3], position: [f32; 3]) {
        let ovis = proto::Ovis {
            orientation: Some(proto::Orientation {
                yaw: orientation[0],
                pitch: orientation[1],
                roll: orientation[2],
            }),
            position: Some(proto::Vector3 { x: position[0], y: position[1], z: position[2] }),
            target: self.target_entity.clone(),
            tcp_offset_local: None,
        };
        let mut buf = Vec::with_capacity(64);
        if ovis.encode(&mut buf).is_ok() {
            self.sock.send(&buf).await.ok();
        }
    }
}
