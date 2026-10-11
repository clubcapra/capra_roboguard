//! Sim-backed Robotiq 2F-140 mock driver.
//!
//! Telemetry comes verbatim from the sim's `robotiq` channel. The one command
//! with a sim sink is `position` (0=open .. 255=closed) → `gripper.position` in
//! the aggregate `RoveControl`; `speed`/`force`/`activate` are accepted-and-
//! ignored (no sim sink). `stop` holds the current position.

use serde_json::{json, Value};

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;
use crate::drivers::sim::control::{set_gripper, SharedControl};
use crate::drivers::sim::feed::SimFeed;

use super::gripper::{command_schema, data_schema, ROBOTIQ_ID};

pub struct RobotiqMock {
    feed: SimFeed,
    control: SharedControl,
}

impl RobotiqMock {
    pub fn new(host: &str, backend_port: u16, control: SharedControl) -> Self {
        Self {
            feed: SimFeed::subscribe(host, backend_port),
            control,
        }
    }
}

impl SensorDriver for RobotiqMock {
    fn telemetry_status(&self, data: &serde_json::Value) -> crate::core::driver::TelemetryStatus {
        super::gripper::telemetry_status(data)
    }
    fn readiness(&self) -> Result<(), DriverError> {
        super::gripper::hardware_readiness(&self.read_data()?)
    }
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        Box::pin(async move {
            self.feed.stop().await;
            Ok(())
        })
    }
    fn id(&self) -> &str {
        ROBOTIQ_ID
    }

    fn display_name(&self) -> &str {
        "Robotiq 2F-140 Gripper (sim)"
    }

    fn command_mode(&self) -> CommandMode {
        CommandMode::Rest
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        data_schema()
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        command_schema()
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        Ok(self.feed.latest_or_empty())
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        let mut sent: Vec<&str> = Vec::new();

        if let Some(pos) = payload.get("position").and_then(Value::as_u64) {
            set_gripper(&self.control, pos as i64);
            sent.push("position");
        }
        for k in ["speed", "force", "goto", "activate", "auto_release", "stop"] {
            if payload.get(k).is_some() {
                sent.push(k);
            }
        }

        if sent.is_empty() {
            return Err(DriverError::CommandFailed(
                "no recognised command fields in payload".into(),
            ));
        }
        Ok(json!({ "sent": sent }))
    }

    fn has_estop(&self) -> bool {
        true
    }

    fn estop(&self) -> Result<Value, DriverError> {
        // Match the physical stop: hold the latest measured jaw position.
        let data = self.read_data()?;
        let position = data
            .get("position")
            .and_then(Value::as_i64)
            .ok_or_else(|| {
                DriverError::Unavailable("No measured gripper position to hold".into())
            })?;
        set_gripper(&self.control, position);
        Ok(json!({ "estop": "sim: gripper holds position" }))
    }
}
