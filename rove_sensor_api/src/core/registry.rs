use std::sync::Arc;

use dashmap::DashMap;

use super::driver::{CommandMode, FieldDescriptor, SensorDriver};
use super::error::DriverError;

/// Full description of a registered sensor, used by HTTP/OpenAPI layer.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, utoipa::ToSchema)]
pub struct SensorInfo {
    pub id: String,
    pub display_name: String,
    pub data_schema: Vec<FieldDescriptor>,
    pub command_schema: Vec<FieldDescriptor>,
    pub command_mode: CommandMode,
    pub data_port: u16,
    pub command_port: u16,
    /// Whether this sensor supports emergency stop.
    pub has_estop: bool,
    /// Whether this sensor supports configuration read/write (SDO).
    pub has_config: bool,
    /// Whether this sensor supports triggering a calibration sequence.
    pub has_calibrate: bool,
    /// Whether this sensor supports individual endpoint read/write by path.
    pub has_endpoint_access: bool,
}

/// Central store for all registered sensor drivers.
///
/// Assigns UDP ports automatically on registration.
/// Thread-safe for concurrent reads from UDP tasks.
pub struct SensorRegistry {
    drivers: DashMap<String, Arc<dyn SensorDriver>>,
    port_map: DashMap<String, (u16, u16)>,
    base_port: u16,
}

impl SensorRegistry {
    pub fn new(base_port: u16) -> Self {
        Self {
            drivers: DashMap::new(),
            port_map: DashMap::new(),
            base_port,
        }
    }

    /// Register a sensor driver. Automatically assigns a UDP data port
    /// and command port. Returns the `(data_port, command_port)` pair.
    pub fn register(&self, driver: impl SensorDriver) -> (u16, u16) {
        let id = driver.id().to_string();
        let idx = self.drivers.len() as u16;
        let data_port = self.base_port + (idx * 2);
        let cmd_port = self.base_port + (idx * 2) + 1;

        tracing::info!(
            sensor = id,
            display_name = driver.display_name(),
            data_port,
            cmd_port,
            mode = ?driver.command_mode(),
            "registered sensor"
        );

        self.port_map.insert(id.clone(), (data_port, cmd_port));
        self.drivers.insert(id, Arc::new(driver));
        (data_port, cmd_port)
    }

    /// Get a driver by sensor ID.
    pub fn get(&self, id: &str) -> Result<Arc<dyn SensorDriver>, DriverError> {
        self.drivers
            .get(id)
            .map(|r| r.value().clone())
            .ok_or_else(|| DriverError::NotFound(id.to_string()))
    }

    /// Get the assigned ports for a sensor.
    pub fn ports(&self, id: &str) -> Option<(u16, u16)> {
        self.port_map.get(id).map(|r| *r.value())
    }

    /// List all registered sensors with their full info.
    pub fn list(&self) -> Vec<SensorInfo> {
        self.drivers
            .iter()
            .map(|entry| {
                let driver = entry.value();
                let id = entry.key().clone();
                let (data_port, cmd_port) = *self.port_map.get(&id).unwrap().value();
                SensorInfo {
                    id,
                    display_name: driver.display_name().to_string(),
                    data_schema: driver.data_schema(),
                    command_schema: driver.command_schema(),
                    command_mode: driver.command_mode(),
                    data_port,
                    command_port: cmd_port,
                    has_estop: driver.has_estop(),
                    has_config: driver.has_config(),
                    has_calibrate: driver.has_calibrate(),
                    has_endpoint_access: driver.has_endpoint_access(),
                }
            })
            .collect()
    }

    /// Number of registered sensors.
    pub fn len(&self) -> usize {
        self.drivers.len()
    }

    pub fn is_empty(&self) -> bool {
        self.drivers.is_empty()
    }

    /// Iterate over all driver IDs and their Arc'd drivers.
    pub fn iter_drivers(&self) -> Vec<(String, Arc<dyn SensorDriver>)> {
        self.drivers
            .iter()
            .map(|entry| (entry.key().clone(), entry.value().clone()))
            .collect()
    }
}
