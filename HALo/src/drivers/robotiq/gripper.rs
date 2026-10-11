//! `SensorDriver` implementation for the Robotiq 2F-140 gripper.

use std::sync::{Arc, RwLock};

use serde_json::Value;
use tokio::sync::mpsc;

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;

use super::state::RobotiqState;
use super::worker::{Cmd, GripperCommand};

pub const ROBOTIQ_ID: &str = "robotiq_gripper";

pub fn hardware_readiness(data: &Value) -> Result<(), DriverError> {
    if data.get("activated").and_then(Value::as_bool) != Some(true)
        || data.get("status").and_then(Value::as_u64) != Some(3)
    {
        return Err(DriverError::Unavailable(
            "gripper activation incomplete (requires gACT=1 and gSTA=3)".into(),
        ));
    }
    Ok(())
}

/// Vendor gFLT meanings stay with the driver; shared health only consumes text.
/// Source: Robotiq 2F instruction manual, section 4.4 (Fault Status).
pub fn telemetry_status(data: &Value) -> crate::core::driver::TelemetryStatus {
    let mut status = crate::core::driver::TelemetryStatus::from_fields(
        data,
        "timestamp_ns",
        &["link_up"],
        &[],
        &[],
    );
    if let Some(code) = data
        .get("fault")
        .and_then(Value::as_u64)
        .filter(|c| *c != 0)
    {
        let description = match code {
            0x05 => "Action delayed until activation completes",
            0x07 => "Activation required before motion",
            0x08 => "Operating temperature exceeded; allow the gripper to cool",
            0x09 => "Gripper communication timeout",
            0x0a => "Supply voltage below operating minimum",
            0x0b => "Emergency automatic release in progress",
            0x0c => "Internal gripper fault",
            0x0d => "Activation failed; check for mechanical obstruction",
            0x0e => "Motor overcurrent",
            0x0f => "Emergency automatic release completed; reactivation required",
            _ => "Unrecognized gripper fault",
        };
        status
            .errors
            .push(format!("{description} (gFLT 0x{code:02x})"));
    }
    status
}

/// Robotiq data schema. Free fn so the sim mock (`robotiq::mock`) serves it verbatim.
pub fn data_schema() -> Vec<FieldDescriptor> {
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
        FieldDescriptor::new("fault", "gFLT — 0=no fault, 0x05–0x0F=fault codes", "u8"),
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

/// Robotiq command schema. Free fn so the sim mock can reuse it verbatim.
pub fn command_schema() -> Vec<FieldDescriptor> {
    vec![
        FieldDescriptor::new(
            "position",
            "rPR — position request, 0=fully open .. 255=fully closed",
            "u8",
        ).requiring_ready(),
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
        ).requiring_ready_for(serde_json::json!(true)),
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

pub struct RobotiqGripper {
    state: Arc<RwLock<RobotiqState>>,
    cmd_tx: mpsc::UnboundedSender<Cmd>,
    task: crate::core::task::WorkerTask,
}

impl RobotiqGripper {
    pub fn new(
        state: Arc<RwLock<RobotiqState>>,
        cmd_tx: mpsc::UnboundedSender<Cmd>,
        task: crate::core::task::WorkerTask,
    ) -> Self {
        Self {
            state,
            cmd_tx,
            task,
        }
    }

    fn send(&self, cmd: Cmd) -> Result<(), DriverError> {
        self.cmd_tx
            .send(cmd)
            .map_err(|e| DriverError::CommandFailed(format!("Robotiq worker channel closed: {e}")))
    }
}

impl SensorDriver for RobotiqGripper {
    fn telemetry_status(&self, data: &serde_json::Value) -> crate::core::driver::TelemetryStatus {
        telemetry_status(data)
    }
    fn readiness(&self) -> Result<(), DriverError> {
        hardware_readiness(&self.read_data()?)
    }
    fn diagnostics(&self) -> Vec<String> {
        self.state
            .read()
            .unwrap()
            .error
            .clone()
            .into_iter()
            .collect()
    }
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        Box::pin(async move {
            self.task.stop().await;
            Ok(())
        })
    }
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
        data_schema()
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        command_schema()
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
        if payload
            .get("stop")
            .and_then(Value::as_bool)
            .unwrap_or(false)
        {
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

#[cfg(test)]
mod readiness_tests {
    use super::hardware_readiness;
    use serde_json::json;
    #[test]
    fn controller_activation_is_not_hardware_readiness() {
        assert!(hardware_readiness(&json!({"activated":false,"status":0})).is_err());
        assert!(hardware_readiness(&json!({"activated":true,"status":1})).is_err());
        assert!(hardware_readiness(&json!({"activated":true,"status":3})).is_ok());
    }
}

#[cfg(test)]
mod fault_tests {
    use super::*;
    #[test]
    fn faults_explain_vendor_codes_without_discarding_unknown_codes() {
        for (code, message) in [
            (0x09, "communication timeout"),
            (0x0e, "overcurrent"),
            (0x06, "Unrecognized"),
        ] {
            let status = telemetry_status(
                &serde_json::json!({"timestamp_ns": 1, "link_up": true, "fault": code}),
            );
            assert!(status.errors[0].contains(message));
            assert!(status.errors[0].contains(&format!("0x{code:02x}")));
        }
        assert!(telemetry_status(&serde_json::json!({"fault": 0}))
            .errors
            .is_empty());
    }
}
