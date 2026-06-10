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
}

impl KinovaMock {
    pub fn new(host: &str, backend_port: u16) -> Self {
        Self {
            feed: SimFeed::subscribe(host, backend_port),
        }
    }
}

impl SensorDriver for KinovaMock {
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
        Ok(self.feed.latest_or_empty())
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
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
