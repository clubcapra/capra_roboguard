//! `SensorDriver` implementation for the Robotiq 2F-140 gripper.

use std::sync::{Arc, RwLock};

use serde_json::Value;
use tokio::sync::mpsc;

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;

use super::state::RobotiqState;
use super::worker::{Cmd, GripperCommand};

pub const ROBOTIQ_ID: &str = "robotiq_gripper";

pub struct RobotiqGripper {
    state: Arc<RwLock<RobotiqState>>,
    cmd_tx: mpsc::UnboundedSender<Cmd>,
}

impl RobotiqGripper {
    pub fn new(state: Arc<RwLock<RobotiqState>>, cmd_tx: mpsc::UnboundedSender<Cmd>) -> Self {
        Self { state, cmd_tx }
    }

    fn send(&self, cmd: Cmd) -> Result<(), DriverError> {
        self.cmd_tx
            .send(cmd)
            .map_err(|e| DriverError::CommandFailed(format!("Robotiq worker channel closed: {e}")))
    }
}

impl SensorDriver for RobotiqGripper {
    fn id(&self) -> &str {
        ROBOTIQ_ID
    }

    fn display_name(&self) -> &str {
        "Robotiq 2F-140 Gripper"
    }

    fn command_mode(&self) -> CommandMode {
        // REST: each command is a one-shot register write. No need to stream.
        CommandMode::Rest
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("activated", "gACT — true once activated", "bool"),
            FieldDescriptor::new("going_to_position", "gGTO — true while moving", "bool"),
            FieldDescriptor::new(
                "status",
                "gSTA — 0=reset, 1=activating, 3=activation complete",
                "u8",
            ),
            FieldDescriptor::new(
                "object_status",
                "gOBJ — 0=moving, 1=object detected opening, 2=object detected closing, 3=at position",
                "u8",
            ),
            FieldDescriptor::new(
                "fault",
                "gFLT — 0=no fault, 0x05–0x0F=fault codes",
                "u8",
            ),
            FieldDescriptor::new(
                "position_request_echo",
                "gPR — last commanded position (0=open, 255=closed)",
                "u8",
            ),
            FieldDescriptor::new(
                "position",
                "gPO — actual jaw position (0=open, 255=closed)",
                "u8",
            ),
            FieldDescriptor::new("current_raw", "gCU raw byte (× 10 mA)", "u8"),
            FieldDescriptor::new("current_a", "Motor current", "f32").with_unit("A"),
            FieldDescriptor::new("link_up", "Modbus channel reachable", "bool"),
            FieldDescriptor::new(
                "timestamp_ns",
                "Unix timestamp (ns) of last status read; 0 until first poll",
                "i64",
            )
            .with_unit("ns"),
        ]
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new(
                "position",
                "rPR — position request, 0=fully open .. 255=fully closed",
                "u8",
            ),
            FieldDescriptor::new("speed", "rSP — speed, 0=min .. 255=max", "u8"),
            FieldDescriptor::new(
                "force",
                "rFR — force, 0=min (no re-grasp) .. 255=max",
                "u8",
            ),
            FieldDescriptor::new(
                "goto",
                "rGTO — true to start motion, false to stop",
                "bool",
            ),
            FieldDescriptor::new(
                "activate",
                "rACT — true=activate, false=deactivate. Recovery from a fault needs a 0→1 edge across two packets.",
                "bool",
            ),
            FieldDescriptor::new(
                "auto_release",
                "rATR/rARD — emergency auto-release: 0=closing, 1=opening. Omit for normal operation.",
                "u8",
            ),
            FieldDescriptor::new(
                "stop",
                "Convenience: clears rGTO so the gripper holds at current position",
                "bool",
            ),
        ]
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        let s = self.state.read().unwrap();
        Ok(serde_json::json!({
            "activated": s.activated,
            "going_to_position": s.going_to_position,
            "status": s.status,
            "object_status": s.object_status,
            "fault": s.fault,
            "position_request_echo": s.position_request_echo,
            "position": s.position,
            "current_raw": s.current_raw,
            "current_a": s.current_a(),
            "link_up": s.link_up,
            "timestamp_ns": s.timestamp_ns,
        }))
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        if payload.get("stop").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::Stop)?;
            return Ok(serde_json::json!({ "sent": ["stop"] }));
        }

        let mut cmd = GripperCommand::default();
        let mut sent: Vec<&str> = Vec::new();

        if let Some(p) = payload.get("position").and_then(Value::as_u64) {
            cmd.position = Some(clamp_byte(p, "position")?);
            sent.push("position");
        }
        if let Some(s) = payload.get("speed").and_then(Value::as_u64) {
            cmd.speed = Some(clamp_byte(s, "speed")?);
            sent.push("speed");
        }
        if let Some(f) = payload.get("force").and_then(Value::as_u64) {
            cmd.force = Some(clamp_byte(f, "force")?);
            sent.push("force");
        }
        if let Some(g) = payload.get("goto").and_then(Value::as_bool) {
            cmd.goto = Some(g);
            sent.push("goto");
        }
        if let Some(a) = payload.get("activate").and_then(Value::as_bool) {
            cmd.activate = Some(a);
            sent.push("activate");
        }
        if let Some(d) = payload.get("auto_release").and_then(Value::as_u64) {
            if d > 1 {
                return Err(DriverError::CommandFailed(format!(
                    "auto_release must be 0 (closing) or 1 (opening), got {d}"
                )));
            }
            cmd.auto_release = Some(d as u8);
            sent.push("auto_release");
        }

        // If only setpoints (position/speed/force) were sent, infer rGTO=true
        // — that matches operator intent: "go to this position" rather than
        // "stage these for the next explicit goto packet". Explicit `goto`
        // wins if present.
        if cmd.goto.is_none()
            && (cmd.position.is_some() || cmd.speed.is_some() || cmd.force.is_some())
        {
            cmd.goto = Some(true);
        }

        if sent.is_empty() {
            return Err(DriverError::CommandFailed(
                "no recognised command fields in payload".into(),
            ));
        }

        self.send(Cmd::Apply(cmd))?;
        Ok(serde_json::json!({ "sent": sent }))
    }

    fn has_estop(&self) -> bool {
        true
    }

    fn estop(&self) -> Result<Value, DriverError> {
        // Stop holds position. The hardware "auto-release" is destructive
        // (slow open against motor current limits) and should only be issued
        // explicitly via the `auto_release` command field.
        self.send(Cmd::Stop)?;
        tracing::warn!("Robotiq ESTOP — gripper held at current position");
        Ok(serde_json::json!({
            "estop": "stopped at current position",
            "note": "for emergency mechanical release send {\"auto_release\": 1}"
        }))
    }
}

fn clamp_byte(v: u64, name: &'static str) -> Result<u8, DriverError> {
    if v > 255 {
        return Err(DriverError::CommandFailed(format!(
            "{name} must be 0..=255, got {v}"
        )));
    }
    Ok(v as u8)
}
