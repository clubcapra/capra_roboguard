//! Sim-backed Kinova arm mock driver.
//!
//! Telemetry comes verbatim from the sim's `kinova` channel (joint pos/vel/
//! current/temp). The sim's control sink (`RoveControl`) has no joint-space arm
//! channel, so joint commands are accepted-and-ignored in v1 (Phase 4 may add an
//! `arm:{joint_vel}` channel to close this gap). The arm still streams faithful
//! telemetry, which is what the IK/autonomy layer subscribes to.

use serde_json::{json, Value};

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;
use crate::drivers::sim::feed::SimFeed;

use super::arm::{command_schema, data_schema, KINOVA_ID};

pub struct KinovaMock {
    feed: SimFeed,
    control: crate::drivers::sim::control::SharedControl,
}

impl KinovaMock {
    pub fn new(host: &str, backend_port: u16) -> Self {
        Self::with_control(
            host,
            backend_port,
            crate::drivers::sim::control::default_control_frame(),
        )
    }
    pub fn with_control(
        host: &str,
        backend_port: u16,
        control: crate::drivers::sim::control::SharedControl,
    ) -> Self {
        Self {
            feed: SimFeed::subscribe(host, backend_port),
            control,
        }
    }
}

impl SensorDriver for KinovaMock {
    fn has_estop(&self) -> bool {
        true
    }
    fn estop_confirmation(&self) -> Option<String> {
        Some(super::arm::limp_confirmation())
    }
    fn estop(&self) -> Result<Value, DriverError> {
        self.control.write().unwrap()["arm_limp_mode"] = json!(true);
        Ok(json!({"estop":"sim: trajectories cleared; limp mode enabled", "limp_mode":true}))
    }
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        Box::pin(async move {
            self.feed.stop().await;
            Ok(())
        })
    }
    fn id(&self) -> &str {
        KINOVA_ID
    }

    fn display_name(&self) -> &str {
        "Kinova Gen2 6DOF (sim)"
    }

    fn command_mode(&self) -> CommandMode {
        CommandMode::Stream { interval_ms: 100 }
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        data_schema()
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        command_schema()
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        let mut data = self.feed.latest_or_empty();
        if let Some(object) = data.as_object_mut() {
            object.entry("limp_mode").or_insert(json!(false));
            object.insert("limp_supported".into(), json!(true));
            object.insert(
                "motion_inhibited".into(),
                json!(self.control.read().unwrap()["arm_limp_mode"] == true),
            );
        }
        Ok(data)
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        if let Some(enabled) = payload.get("limp_mode") {
            if payload.as_object().map(|o| o.len()) != Some(1) {
                return Err(DriverError::InvalidCommand(
                    "limp_mode must be sent alone".into(),
                ));
            }
            let enabled = enabled
                .as_bool()
                .ok_or_else(|| DriverError::InvalidCommand("limp_mode must be boolean".into()))?;
            self.control.write().unwrap()["arm_limp_mode"] = json!(enabled);
            return Ok(
                json!({"requested_limp_mode":enabled,"note":"inspect limp_mode telemetry for simulator confirmation"}),
            );
        }
        if self.control.read().unwrap()["arm_limp_mode"] == true {
            return Err(DriverError::Unavailable(
                "disable limp mode before issuing arm commands".into(),
            ));
        }
        // No joint-space sink in the sim control bridge yet — accept & ack honestly.
        let mut sent: Vec<&str> = Vec::new();
        for i in 1..=6 {
            if payload.get(format!("joint_{i}_pos")).is_some() {
                sent.push("angular_position");
                break;
            }
        }
        for i in 1..=6 {
            if payload.get(format!("joint_{i}_vel")).is_some() {
                sent.push("angular_velocity");
                break;
            }
        }
        for k in ["move_home", "erase_trajectories", "set_joint_zero"] {
            if payload.get(k).is_some() {
                sent.push(k);
            }
        }
        Ok(json!({
            "sent": sent,
            "note": "kinova arm command path is stubbed in sim mode (no joint-space sink yet)"
        }))
    }
}
