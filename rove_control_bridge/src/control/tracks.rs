//! Differential-track control — normalised (left,right) -> per-ODrive commands.
//!
//! Port of `rove_control_bridge/strategies/tracks_velocity.py`. Each track node
//! receives `{"axis_state":8,"input_vel":<rev/s>}`; the rove_sensor_api odrive
//! mock applies `input_vel` to the sim *only while armed* (axis_state==8), so we
//! carry axis_state on every packet and stream at the loop rate. Idle/estop send
//! zero velocity then `axis_state=1`.

use crate::config::Tracks;
use crate::transport::command::CommandSink;
use crate::transport::discover::Discovery;
use anyhow::{Context, Result};
use serde_json::{json, Value};

const AXIS_IDLE: u32 = 1;
const AXIS_CLOSED_LOOP: u32 = 8;

/// A resolved track node: its command port, which side it drives, and the side
/// inversion sign applied to the setpoint.
struct Node {
    id: u32,
    command_port: u16,
    is_left: bool,
    sign: f64,
}

pub struct TracksController<'a> {
    sink: &'a CommandSink,
    max_velocity: f64,
    slew_per_s: f64,
    nodes: Vec<Node>,
    armed: bool,
    last_left: f64,
    last_right: f64,
}

impl<'a> TracksController<'a> {
    /// Resolve every configured track node to its command port. Errors if a node
    /// id is missing from discovery (e.g. odrive_31 absent).
    pub fn new(cfg: &Tracks, sink: &'a CommandSink, disc: &Discovery) -> Result<Self> {
        let mut nodes = Vec::new();
        for (ids, is_left, invert) in [
            (&cfg.left_nodes, true, cfg.invert_left),
            (&cfg.right_nodes, false, cfg.invert_right),
        ] {
            let sign = if invert { -1.0 } else { 1.0 };
            for &id in ids {
                let key = format!("odrive_{id}");
                let ep = disc
                    .get(&key)
                    .with_context(|| format!("{key} not in /discover"))?;
                nodes.push(Node {
                    id,
                    command_port: ep.command_port,
                    is_left,
                    sign,
                });
            }
        }
        anyhow::ensure!(!nodes.is_empty(), "no track nodes configured");
        Ok(Self {
            sink,
            max_velocity: cfg.max_velocity,
            slew_per_s: cfg.slew_per_s,
            nodes,
            armed: false,
            last_left: 0.0,
            last_right: 0.0,
        })
    }

    /// One-line summary of the resolved node->port map (for startup logging).
    pub fn map_summary(&self) -> String {
        self.nodes
            .iter()
            .map(|n| {
                format!(
                    "{}{}:{}",
                    if n.is_left { "L" } else { "R" },
                    n.id,
                    n.command_port
                )
            })
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Clear errors so the mock/firmware can leave the fault state and arm.
    pub async fn arm(&mut self) -> Result<()> {
        for n in &self.nodes {
            self.sink
                .send(n.command_port, &json!({ "clear_errors": true }))
                .await?;
        }
        self.armed = true;
        tracing::info!("track ODrives armed (clear_errors + closed loop on first drive)");
        Ok(())
    }

    /// Drive with a normalised differential intent. Arms on first call. The
    /// per-side command is **slew-rate limited** (max change per second) so the
    /// tracks never step instantly — an instant 0→full step on opposite tracks
    /// jerks the chassis (it rolled the robot ~47deg in testing). `dt` seconds.
    pub async fn drive(&mut self, left: f64, right: f64, dt: f64) -> Result<()> {
        if !self.armed {
            self.arm().await?;
        }
        let step = self.slew_per_s * dt;
        self.last_left += (left - self.last_left).clamp(-step, step);
        self.last_right += (right - self.last_right).clamp(-step, step);
        for (port, payload) in self.velocity_commands(self.last_left, self.last_right) {
            self.sink.send(port, &payload).await?;
        }
        Ok(())
    }

    /// Stop and disarm: zero velocity, then `axis_state=1` on every node. Not
    /// slew-limited — a stop must be immediate. Resets the slew memory to zero.
    pub async fn idle(&mut self) -> Result<()> {
        for n in &self.nodes {
            self.sink
                .send(n.command_port, &json!({ "input_vel": 0.0 }))
                .await?;
        }
        for n in &self.nodes {
            self.sink
                .send(n.command_port, &json!({ "axis_state": AXIS_IDLE }))
                .await?;
        }
        self.armed = false;
        self.last_left = 0.0;
        self.last_right = 0.0;
        Ok(())
    }

    /// Per-node (port, payload) for a normalised (left,right) intent.
    fn velocity_commands(&self, left: f64, right: f64) -> Vec<(u16, Value)> {
        self.nodes
            .iter()
            .map(|n| {
                let norm = if n.is_left { left } else { right };
                let vel = norm.clamp(-1.0, 1.0) * self.max_velocity * n.sign;
                (
                    n.command_port,
                    json!({ "axis_state": AXIS_CLOSED_LOOP, "input_vel": vel }),
                )
            })
            .collect()
    }
}
