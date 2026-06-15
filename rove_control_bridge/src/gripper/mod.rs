//! Bridge → rove_sensor_api gripper, DIRECT (not via the IK engine — the engine
//! has no gripper). The Robotiq 2F-140 is a REST sensor: POST a position byte
//! (0 = open .. 255 = closed) to `/<id>/command`. Only flippers, arm (ovis) and
//! tracks route through the engine; the gripper goes straight to the robot API.

use std::sync::atomic::{AtomicI64, Ordering};

/// Posts gripper position to the robot API. De-dupes unchanged positions and
/// honours `--dry-run` (logs instead of sending). HTTP is done on a blocking
/// pool so the async teleop path never stalls.
pub struct GripperSender {
    url: String,
    dry_run: bool,
    last: AtomicI64, // last position sent; -1 = none yet
}

impl GripperSender {
    pub fn new(host: &str, http_port: u16, dry_run: bool) -> Self {
        Self {
            url: format!("http://{host}:{http_port}/robotiq_gripper/command"),
            dry_run,
            last: AtomicI64::new(-1),
        }
    }

    /// Send a gripper position (0..255). No-op if unchanged since the last send.
    pub async fn send(&self, position: u32) {
        let p = position.min(255) as i64;
        let prev = self.last.swap(p, Ordering::Relaxed);
        if prev == p {
            return; // unchanged — don't spam the gripper
        }
        let first = prev == -1; // first command since startup
        if self.dry_run {
            tracing::info!("GRIPPER (dry-run) -> position {p}");
            return;
        }
        let url = self.url.clone();
        tokio::task::spawn_blocking(move || {
            let body = serde_json::json!({ "position": p }).to_string();
            match ureq::post(&url)
                .set("Content-Type", "application/json")
                .send_string(&body)
            {
                // First successful POST logged at INFO (proves the gripper->API
                // path); subsequent ones at DEBUG so a toggling gripper can't
                // flood the log. Failures always WARN.
                Ok(resp) if first => {
                    tracing::info!("GRIPPER first POST ok ({}) -> {url} position {p}", resp.status())
                }
                Ok(resp) => tracing::debug!("GRIPPER POST ok ({}) position {p}", resp.status()),
                Err(e) => tracing::warn!("GRIPPER POST to {url} FAILED: {e}"),
            }
        });
    }
}
