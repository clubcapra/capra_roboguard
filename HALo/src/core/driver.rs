use serde_json::Value;
use std::time::Duration;

use super::error::DriverError;

/// Whether commands are one-shot or arrive as a continuous stream.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, utoipa::ToSchema,
)]
#[serde(tag = "type")]
pub enum CommandMode {
    /// Request/response: client sends a single command, server replies once.
    Rest,
    /// Streaming: client sends Command packets continuously at `interval_ms`.
    /// Each packet is processed as it arrives — the server does not loop or
    /// re-send anything. Typical use: CAN control loops sending ODrive setpoints.
    Stream {
        /// Expected client send interval in milliseconds.
        interval_ms: u64,
    },
}

impl CommandMode {
    pub fn stream(interval: Duration) -> Self {
        CommandMode::Stream {
            interval_ms: interval.as_millis() as u64,
        }
    }

    pub fn interval(&self) -> Option<Duration> {
        match self {
            CommandMode::Rest => None,
            CommandMode::Stream { interval_ms } => Some(Duration::from_millis(*interval_ms)),
        }
    }
}

/// A driver declares its delivery model; the transport handles every subscriber.
#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    PartialEq,
    Eq,
    serde::Serialize,
    serde::Deserialize,
    utoipa::ToSchema,
)]
pub enum DataDelivery {
    #[default]
    Snapshots,
    Updates,
}
#[derive(Clone)]
pub enum DataUpdate {
    Data(std::sync::Arc<Value>),
    Dropped(u64),
}

/// A driver's interpretation of its telemetry. The supervisor applies timeouts
/// to this sample timestamp without knowing device-specific JSON field names.
pub struct TelemetryStatus {
    pub sample_timestamp_ns: u64,
    pub connected: bool,
    pub errors: Vec<String>,
}

impl TelemetryStatus {
    /// Convenience for drivers with flat telemetry. Every field is supplied
    /// by the driver; optional links report errors without invalidating freshness.
    pub fn from_fields(
        data: &Value,
        timestamp: &str,
        required_links: &[&str],
        optional_links: &[&str],
        error_codes: &[&str],
    ) -> Self {
        let mut errors = Vec::new();
        for name in error_codes {
            if let Some(code) = data.get(*name).and_then(Value::as_u64).filter(|c| *c != 0) {
                errors.push(format!("{name}: {code}"));
            }
        }
        for name in required_links.iter().chain(optional_links) {
            if data.get(*name).and_then(Value::as_bool) == Some(false) {
                errors.push(format!("{name}: false"));
            }
        }
        Self {
            sample_timestamp_ns: data.get(timestamp).and_then(Value::as_u64).unwrap_or(0),
            connected: !required_links
                .iter()
                .any(|name| data.get(*name).and_then(Value::as_bool) == Some(false)),
            errors,
        }
    }
}

/// Describes one field in a data or command schema (for OpenAPI docs).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, utoipa::ToSchema)]
pub struct FieldDescriptor {
    pub name: String,
    pub description: String,
    /// Rust type name: "f64", "u16", "bool", "String", etc.
    pub type_name: String,
    /// Physical unit if applicable: "degrees", "rpm", "m/s".
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    /// Drivers declare command constraints; transport/UI code never infers them
    /// from a hardware brand or a particular command name.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub requires_ready: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ready_value: Option<Value>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub standalone: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub confirmation: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub confirmation_value: Option<Value>,
}

impl FieldDescriptor {
    pub fn new(name: &str, description: &str, type_name: &str) -> Self {
        Self {
            name: name.to_string(),
            description: description.to_string(),
            type_name: type_name.to_string(),
            unit: None,
            requires_ready: false,
            ready_value: None,
            standalone: false,
            confirmation: None,
            confirmation_value: None,
        }
    }

    pub fn with_unit(mut self, unit: &str) -> Self {
        self.unit = Some(unit.to_string());
        self
    }
    pub fn requiring_ready(mut self) -> Self {
        self.requires_ready = true;
        self
    }
    pub fn requiring_ready_for(mut self, value: Value) -> Self {
        self.ready_value = Some(value);
        self
    }
    pub fn standalone(mut self) -> Self {
        self.standalone = true;
        self
    }

    pub fn confirm_for(mut self, value: Value, text: &str) -> Self {
        self.confirmation = Some(text.into());
        self.confirmation_value = Some(value);
        self
    }
}

/// The core trait every sensor driver must implement.
///
/// # Adding a new sensor
///
/// 1. Create `src/drivers/my_sensor.rs` and implement this trait.
/// 2. Add `pub mod my_sensor;` to `src/drivers/mod.rs`.
/// 3. Register its connection factory in the driver composition module,
///    `drivers::loader`, using the supervisor. Console and protocol code need no edits.
///
/// That's it. The framework handles UDP sockets, HTTP routes, and documentation.
pub trait SensorDriver: Send + Sync + 'static {
    /// Drivers using another timestamp or device error/link fields override
    /// this method. Hardware interpretation belongs beside the driver schema.
    fn telemetry_status(&self, data: &Value) -> TelemetryStatus {
        TelemetryStatus::from_fields(data, "timestamp_ns", &[], &[], &[])
    }
    /// Current hardware-worker failures, separate from cached telemetry.
    fn diagnostics(&self) -> Vec<String> {
        Vec::new()
    }
    /// Device-specific initialization beyond a successful controller connection.
    fn readiness(&self) -> Result<(), DriverError> {
        Ok(())
    }
    fn health(&self) -> Result<Value, DriverError> {
        Ok(serde_json::json!({"status":"unmanaged"}))
    }
    /// Stop workers and release transport resources before reconnecting.
    /// Native workers must complete before another SDK session may start.
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        Box::pin(async { Ok(()) })
    }
    /// Update sources use the same UDP DATA envelope and schema as snapshots.
    fn data_delivery(&self) -> DataDelivery {
        DataDelivery::Snapshots
    }
    fn subscribe_updates(&self) -> Option<tokio::sync::broadcast::Receiver<DataUpdate>> {
        None
    }
    /// Unique string ID used in UDP port mapping and HTTP routes.
    fn id(&self) -> &str;

    /// Human-readable name shown in HALo console.
    fn display_name(&self) -> &str;

    /// Describes the data fields this sensor produces.
    fn data_schema(&self) -> Vec<FieldDescriptor>;

    /// Describes the command fields this sensor accepts.
    fn command_schema(&self) -> Vec<FieldDescriptor>;

    /// REST or Stream (with watchdog interval).
    fn command_mode(&self) -> CommandMode;

    /// Read current sensor data. Returns a JSON-serializable value.
    fn read_data(&self) -> Result<Value, DriverError>;

    /// Execute a command. For Stream-mode drivers this is called once per
    /// incoming packet — the client is responsible for the send rate.
    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError>;

    /// Whether this driver supports emergency stop.
    fn has_estop(&self) -> bool {
        false
    }
    /// Optional hardware consequence shown by the generic console before estop.
    fn estop_confirmation(&self) -> Option<String> {
        None
    }

    /// Trigger an emergency stop. Only called if `has_estop()` returns true.
    fn estop(&self) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("estop not supported".into()))
    }

    /// Whether this driver supports configuration read/write.
    fn has_config(&self) -> bool {
        false
    }

    /// Read the device configuration in the format declared by this driver.
    fn read_config(&self) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("config not supported".into()))
    }

    /// Write configuration parameters to the device.
    /// The driver validates its own configuration parameters.
    fn write_config(&self, _config: &Value) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("config not supported".into()))
    }

    /// Whether this driver supports triggering a calibration sequence.
    fn has_calibrate(&self) -> bool {
        false
    }

    /// Trigger a calibration sequence.
    /// Parameters and supported routines belong to the driver.
    fn calibrate(&self, _params: &Value) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed(
            "calibration not supported".into(),
        ))
    }

    /// Whether this driver supports persisting its configuration to the
    /// device's non-volatile memory (e.g. ODrive `save_configuration`).
    fn has_save_config(&self) -> bool {
        false
    }

    /// Persist the current configuration (including any fresh calibration) to
    /// the device's non-volatile memory. Only called if `has_save_config()`
    /// returns true.
    fn save_config(&self) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed(
            "save config not supported".into(),
        ))
    }

    /// Whether this driver supports writing individual endpoints by path.
    /// Reads (`list_endpoints`, `read_endpoint`) are available on every driver
    /// via the default impls; writes need an explicit override.
    fn has_endpoint_write(&self) -> bool {
        false
    }

    /// List the commands this driver accepts as a `name → {type, description, unit}`
    /// map. Default is derived from `command_schema()` so every driver gets a
    /// uniform `GET /{id}/commands` endpoint.
    fn list_commands(&self) -> Result<Value, DriverError> {
        Ok(field_descriptors_to_map(&self.command_schema()))
    }

    /// List the readable endpoints exposed by this driver. Default is derived
    /// from `data_schema()` so every driver gets a uniform `GET /{id}/endpoints`.
    /// Drivers with a richer addressing scheme (ODrive's flat-endpoint map)
    /// override this to expose more.
    fn list_endpoints(&self) -> Result<Value, DriverError> {
        Ok(field_descriptors_to_map(&self.data_schema()))
    }

    /// Read a single endpoint by path. Default extracts the field from the
    /// latest `read_data()` snapshot, so it works for any driver whose data
    /// schema fields can be looked up by name. Drivers that need live device
    /// I/O (e.g. ODrive SDO) override this.
    fn read_endpoint(&self, path: &str) -> Result<Value, DriverError> {
        let snapshot = self.read_data()?;
        let value = snapshot
            .get(path)
            .cloned()
            .ok_or_else(|| DriverError::CommandFailed(format!("endpoint '{path}' not found")))?;
        let dtype = self
            .data_schema()
            .iter()
            .find(|f| f.name == path)
            .map(|f| f.type_name.clone())
            .unwrap_or_else(|| "unknown".to_string());
        Ok(serde_json::json!({
            "path":  path,
            "value": value,
            "type":  dtype,
        }))
    }

    /// Write a single endpoint. Body: `{"value": <number|bool>}`. Only drivers
    /// that override `has_endpoint_write()` to `true` should implement this.
    fn write_endpoint(&self, _path: &str, _val: &Value) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed(
            "endpoint write not supported".into(),
        ))
    }
}

fn field_descriptors_to_map(fields: &[FieldDescriptor]) -> Value {
    let mut map = serde_json::Map::with_capacity(fields.len());
    for f in fields {
        let mut entry = serde_json::Map::new();
        entry.insert("type".into(), Value::from(f.type_name.clone()));
        entry.insert("description".into(), Value::from(f.description.clone()));
        if let Some(unit) = &f.unit {
            entry.insert("unit".into(), Value::from(unit.clone()));
        }
        map.insert(f.name.clone(), Value::Object(entry));
    }
    Value::Object(map)
}
