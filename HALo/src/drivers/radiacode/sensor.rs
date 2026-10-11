//! Device semantics shared by mock hardware and simulator transports.
use crate::core::{
    driver::{CommandMode, FieldDescriptor, SensorDriver, TelemetryStatus},
    error::DriverError,
};
use serde_json::Value;

pub trait RadiacodeTransport: Send + Sync + 'static {
    fn latest(&self) -> Result<Value, DriverError>;
    fn label(&self) -> &'static str;
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        Box::pin(async { Ok(()) })
    }
}

pub struct RadiacodeDriver {
    id: String,
    display: String,
    transport: Box<dyn RadiacodeTransport>,
}

impl RadiacodeDriver {
    pub fn new(id: String, transport: Box<dyn RadiacodeTransport>) -> Self {
        let display = format!("Radiacode ({})", transport.label());
        Self {
            id,
            display,
            transport,
        }
    }
}

pub fn data_schema() -> Vec<FieldDescriptor> {
    vec![
        FieldDescriptor::new("dose_rate_usv_h", "Dose rate at the detector", "f64")
            .with_unit("µSv/h"),
        FieldDescriptor::new(
            "background_usv_h",
            "Background included in dose rate",
            "f64",
        )
        .with_unit("µSv/h"),
        FieldDescriptor::new("source_count", "Number of simulated point sources", "u32"),
        FieldDescriptor::new("x_m", "Detector world X", "f64").with_unit("m"),
        FieldDescriptor::new("y_m", "Detector world Y", "f64").with_unit("m"),
        FieldDescriptor::new("z_m", "Detector world Z", "f64").with_unit("m"),
        FieldDescriptor::new("connected", "Measurement source is connected", "bool"),
        FieldDescriptor::new(
            "error_code",
            "Driver diagnostic flags; zero means no error",
            "u32",
        ),
        FieldDescriptor::new(
            "transport",
            "mock or simulation; hardware transport is not implemented",
            "String",
        ),
        FieldDescriptor::new("timestamp_ns", "Source sample Unix timestamp", "u64").with_unit("ns"),
    ]
}

impl SensorDriver for RadiacodeDriver {
    fn id(&self) -> &str {
        &self.id
    }
    fn display_name(&self) -> &str {
        &self.display
    }
    fn command_mode(&self) -> CommandMode {
        CommandMode::Rest
    }
    fn data_schema(&self) -> Vec<FieldDescriptor> {
        data_schema()
    }
    fn command_schema(&self) -> Vec<FieldDescriptor> {
        Vec::new()
    }
    fn read_data(&self) -> Result<Value, DriverError> {
        self.transport.latest()
    }
    fn execute_command(&self, _: &Value) -> Result<Value, DriverError> {
        Err(DriverError::InvalidCommand(
            "Radiacode dose telemetry is read-only".into(),
        ))
    }
    fn telemetry_status(&self, data: &Value) -> TelemetryStatus {
        let mut status =
            TelemetryStatus::from_fields(data, "timestamp_ns", &["connected"], &[], &[]);
        if data.get("connected").and_then(Value::as_bool) != Some(true) {
            status.connected = false;
            status
                .errors
                .push("Radiacode measurement source is disconnected".into());
        }
        if !data
            .get("dose_rate_usv_h")
            .and_then(Value::as_f64)
            .is_some_and(|d| d.is_finite() && d >= 0.0)
        {
            status.connected = false;
            status
                .errors
                .push("Radiacode dose-rate measurement is missing or invalid".into());
        }
        let code = data.get("error_code").and_then(Value::as_u64).unwrap_or(0);
        for (bit, text) in [
            (1, "measurement source disconnected"),
            (2, "invalid dose-rate sample"),
            (4, "measurement unavailable"),
        ] {
            if code & bit != 0 {
                status
                    .errors
                    .push(format!("Radiacode: {text} (flag {bit})"));
            }
        }
        if code & !7 != 0 {
            status.errors.push(format!(
                "Radiacode: unknown diagnostic flags 0x{:x}",
                code & !7
            ));
        }
        status
    }
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        self.transport.shutdown()
    }
}
