use serde_json::Value;
use std::time::Duration;

use super::error::DriverError;

/// Whether commands are one-shot or arrive as a continuous stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, utoipa::ToSchema)]
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

/// Describes one field in a data or command schema (for OpenAPI/Scalar docs).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, utoipa::ToSchema)]
pub struct FieldDescriptor {
    pub name: String,
    pub description: String,
    /// Rust type name: "f64", "u16", "bool", "String", etc.
    pub type_name: String,
    /// Physical unit if applicable: "degrees", "rpm", "m/s".
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
}

impl FieldDescriptor {
    pub fn new(name: &str, description: &str, type_name: &str) -> Self {
        Self {
            name: name.to_string(),
            description: description.to_string(),
            type_name: type_name.to_string(),
            unit: None,
        }
    }

    pub fn with_unit(mut self, unit: &str) -> Self {
        self.unit = Some(unit.to_string());
        self
    }
}

/// The core trait every sensor driver must implement.
///
/// # Adding a new sensor
///
/// 1. Create `src/drivers/my_sensor.rs` and implement this trait.
/// 2. Add `pub mod my_sensor;` to `src/drivers/mod.rs`.
/// 3. Register it in `main.rs`: `registry.register(MySensor::new());`
///
/// That's it. The framework handles UDP sockets, HTTP routes, and documentation.
pub trait SensorDriver: Send + Sync + 'static {
    /// Unique string ID used in UDP port mapping and HTTP routes.
    fn id(&self) -> &str;

    /// Human-readable name shown in Scalar UI.
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

    /// Trigger an emergency stop. Only called if `has_estop()` returns true.
    fn estop(&self) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("estop not supported".into()))
    }

    /// Whether this driver supports configuration read/write via SDO.
    fn has_config(&self) -> bool {
        false
    }

    /// Read all config-namespace parameters from the device (dynamically from endpoint map).
    fn read_config(&self) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("config not supported".into()))
    }

    /// Write configuration parameters to the device.
    /// `config` is a JSON object keyed by flat endpoint path, e.g.
    /// `{"axis0.controller.config.vel_limit": 20.0}`.
    fn write_config(&self, config: &Value) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("config not supported".into()))
    }

    /// Whether this driver supports triggering a calibration sequence.
    fn has_calibrate(&self) -> bool {
        false
    }

    /// Trigger a calibration sequence.
    /// `params` may include `{"type": "full"|"motor"|"encoder_index"|"encoder_offset"}`.
    fn calibrate(&self, params: &Value) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("calibration not supported".into()))
    }

    /// Whether this driver supports individual endpoint read/write by path.
    fn has_endpoint_access(&self) -> bool {
        false
    }

    /// List all endpoints in the loaded map (no CAN I/O — just metadata).
    fn list_endpoints(&self) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("endpoint access not supported".into()))
    }

    /// Read a single endpoint by its flat-endpoint path (e.g. `"axis0.config.motor.vel_limit"`).
    fn read_endpoint(&self, _path: &str) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("endpoint access not supported".into()))
    }

    /// Write a single endpoint. Body: `{"value": <number|bool>}`.
    fn write_endpoint(&self, _path: &str, _val: &Value) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed("endpoint access not supported".into()))
    }
}
