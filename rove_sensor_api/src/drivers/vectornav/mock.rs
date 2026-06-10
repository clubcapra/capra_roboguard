//! Sim-backed VectorNav VN-300 mock driver.
//!
//! Reuses the real driver's `data_schema()`/`command_schema()` so the OpenAPI/
//! Scalar surface and the served wire are identical to hardware — autonomy can't
//! tell this from a real VN-300. Telemetry comes verbatim from the sim's
//! `vectornav` channel (the sim emits the full `VectorNavState` field set).
//!
//! Read-only: the VN-300 has no actuation, so commands (tare/reset/…) are
//! accepted-and-ignored in sim mode.

use serde_json::{json, Value};

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;
use crate::drivers::sim::feed::SimFeed;

use super::sensor::{command_schema, data_schema};

pub const VECTORNAV_SIM_ID: &str = "vectornav_sim";

pub struct VectorNavMock {
    feed: SimFeed,
}

impl VectorNavMock {
    pub fn new(host: &str, backend_port: u16) -> Self {
        Self {
            feed: SimFeed::subscribe(host, backend_port),
        }
    }
}

impl SensorDriver for VectorNavMock {
    fn id(&self) -> &str {
        VECTORNAV_SIM_ID
    }

    fn display_name(&self) -> &str {
        "VectorNav VN-300 (sim)"
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

    fn execute_command(&self, _payload: &Value) -> Result<Value, DriverError> {
        // The INS is read-only; tare/reset/etc. have no physical effect in sim.
        Ok(json!({ "sent": [], "note": "vectornav is read-only in sim mode" }))
    }
}
