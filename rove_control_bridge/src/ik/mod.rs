//! Bridge → rove_ik_engine: forward all control-proto motion to the engine, which
//! resolves IK + self-collision and drives the actuators on rove_sensor_api.
//!
//! Three channels: the arm twist (`Ovis` proto on :9100) and drive teleop (flipper
//! steps + drum velocities as JSON on :9102) are fire-and-forget UDP; named pose
//! moves go to the engine's HTTP API (`POST /api/v1/poses/goto`). The gripper is
//! NOT here — it goes straight to the robot API (the engine has no gripper chain).

use anyhow::Context;
use prost::Message;
use std::time::Duration;
use tokio::net::UdpSocket;

/// prost types for the engine's `forgebot.engine` proto package.
pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/forgebot.engine.rs"));
}

/// Sanitize a normalised teleop component headed for the engine: NaN/Inf -> 0,
/// then clamp to [-1, 1]. The engine clamps/guards too, but a value-bearing wire
/// (RoveControl) can carry a non-finite float, so we never forward garbage.
#[inline]
fn norm(x: f32) -> f32 {
    if x.is_finite() {
        x.clamp(-1.0, 1.0)
    } else {
        0.0
    }
}

/// A fire-and-forget UDP sender to the IK engine plus a blocking HTTP path for
/// named poses. UDP channels: the arm twist (`Ovis` proto on the engine's :9100)
/// and drive teleop (flipper steps + drum velocities as JSON on the engine's
/// :9102). The engine translates both into rove_sensor_api commands (with its own
/// output gates). Pose moves POST to the engine's HTTP API.
pub struct IkForwarder {
    sock: UdpSocket,        // arm (Ovis) -> :9100
    drive_sock: UdpSocket,  // flippers + drums (JSON) -> :9102
    target_entity: String,
    pose_url: String,       // engine HTTP pose-goto endpoint
}

impl IkForwarder {
    pub async fn new(
        host: &str,
        port: u16,
        drive_port: u16,
        http_port: u16,
        target_entity: String,
    ) -> anyhow::Result<Self> {
        let sock = UdpSocket::bind(("0.0.0.0", 0)).await?;
        sock.connect((host, port)).await?;
        let drive_sock = UdpSocket::bind(("0.0.0.0", 0)).await?;
        drive_sock.connect((host, drive_port)).await?;
        let pose_url = format!("http://{host}:{http_port}/api/v1/poses/goto");
        tracing::info!(
            "ik forwarder -> rove_ik_engine {host} arm:{port} drive:{drive_port} pose:{http_port} target='{target_entity}'");
        Ok(Self { sock, drive_sock, target_entity, pose_url })
    }

    /// Forward drive teleop: flipper steps {-1,0,+1} (fl,fr,rl,rr) and normalised
    /// track velocities [-1,1]. The engine ramps flipper targets and velocity-
    /// commands the drums. Steps are clamped to {-1,0,+1} and velocities sanitized.
    pub async fn send_drive(&self, flippers: [i32; 4], left: f32, right: f32) {
        let steps: [i32; 4] = [
            flippers[0].signum(),
            flippers[1].signum(),
            flippers[2].signum(),
            flippers[3].signum(),
        ];
        let body = serde_json::json!({
            "flippers": steps,
            "tracks": { "left": norm(left), "right": norm(right) },
        });
        if let Ok(buf) = serde_json::to_vec(&body) {
            self.drive_sock.send(&buf).await.ok();
        }
    }

    /// Forward a normalised arm twist (orientation yaw/pitch/roll + position xyz,
    /// each in {-1,1}). The engine scales by its max_lin/ang_vel and integrates.
    /// Components are sanitized (NaN/Inf -> 0) and clamped to [-1, 1].
    pub async fn send_ovis(&self, orientation: [f32; 3], position: [f32; 3]) {
        let ovis = proto::Ovis {
            orientation: Some(proto::Orientation {
                yaw: norm(orientation[0]),
                pitch: norm(orientation[1]),
                roll: norm(orientation[2]),
            }),
            position: Some(proto::Vector3 {
                x: norm(position[0]),
                y: norm(position[1]),
                z: norm(position[2]),
            }),
            target: self.target_entity.clone(),
            tcp_offset_local: None,
        };
        let mut buf = Vec::with_capacity(64);
        if ovis.encode(&mut buf).is_ok() {
            self.sock.send(&buf).await.ok();
        }
    }

    /// Relay a named pose move to the engine: `POST /api/v1/poses/goto` with
    /// `{"name", "speed_deg_s"?}`. The engine plans a collision-checked joint-space
    /// move to the saved pose. Blocking `ureq` runs on a blocking thread so the
    /// async caller isn't stalled. Returns the engine's response body on success,
    /// or an error (unreachable engine / non-2xx / unknown pose).
    pub async fn send_pose(&self, name: String, speed_deg_s: Option<f64>) -> anyhow::Result<String> {
        let url = self.pose_url.clone();
        let body = match speed_deg_s {
            Some(s) => serde_json::json!({ "name": name, "speed_deg_s": s }),
            None => serde_json::json!({ "name": name }),
        };
        let payload = serde_json::to_string(&body).unwrap_or_else(|_| "{}".to_string());
        tokio::task::spawn_blocking(move || -> anyhow::Result<String> {
            match ureq::post(&url)
                .timeout(Duration::from_secs(3))
                .set("Content-Type", "application/json")
                .send_string(&payload)
            {
                Ok(resp) => Ok(resp.into_string().unwrap_or_default()),
                // ureq treats 4xx/5xx as Err(Status); surface the engine's detail.
                Err(ureq::Error::Status(code, resp)) => {
                    let detail = resp.into_string().unwrap_or_default();
                    Err(anyhow::anyhow!("engine HTTP {code}: {detail}"))
                }
                Err(e) => Err(anyhow::anyhow!("pose POST failed: {e}")),
            }
        })
        .await
        .context("pose relay task panicked")?
    }
}
