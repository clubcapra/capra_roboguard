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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub estop_confirmation: Option<String>,
    /// Whether this sensor supports configuration read/write (SDO).
    pub has_config: bool,
    /// Whether this sensor supports triggering a calibration sequence.
    pub has_calibrate: bool,
    /// Whether this sensor supports persisting its configuration to
    /// non-volatile memory (e.g. ODrive `save_configuration`).
    pub has_save_config: bool,
    /// Whether this sensor supports *writing* individual endpoints by path.
    /// Reads (`list_endpoints`, `read_endpoint`) are available on every driver
    /// via the default trait impl, so they don't need a flag.
    pub has_endpoint_write: bool,
    #[serde(default)]
    pub data_delivery: super::driver::DataDelivery,
}

/// Driver slots and persistent per-instance UDP reservations.
pub struct SensorRegistry {
    drivers: DashMap<String, Arc<super::slot::DriverSlot>>,
    ports: std::sync::Mutex<super::ports::PortStore>,
}

impl SensorRegistry {
    pub fn new(base_port: u16) -> Self {
        Self::persistent(base_port, None, Default::default()).expect("valid port range")
    }
    pub fn persistent(
        base: u16,
        path: Option<std::path::PathBuf>,
        reserved: std::collections::BTreeSet<u16>,
    ) -> std::io::Result<Self> {
        Ok(Self {
            drivers: DashMap::new(),
            ports: std::sync::Mutex::new(super::ports::PortStore::new(base, path, reserved)?),
        })
    }
    pub fn reserve(&self, id: &str) -> std::io::Result<(u16, u16)> {
        let [data, command] = self.ports.lock().unwrap().ensure(id)?;
        Ok((data, command))
    }
    pub fn set_ports(&self, id: &str, data: u16, command: u16) -> std::io::Result<()> {
        self.ports.lock().unwrap().set(id, [data, command])
    }
    pub fn reserve_external_ports(&self, ports: &[u16]) -> std::io::Result<()> {
        self.ports.lock().unwrap().reserve_external(ports)
    }
    pub fn assignments(&self) -> super::ports::Assignments {
        self.ports.lock().unwrap().assignments.clone()
    }
    pub fn register(&self, driver: impl SensorDriver) -> (u16, u16) {
        let id = driver.id().to_owned();
        let ports = self
            .try_register(id.clone(), Arc::new(driver))
            .expect("sensor registration");
        self.drivers.get(&id).unwrap().mark_sim_ready();
        ports
    }
    pub fn update_health(&self, id: &str, health: super::supervisor::DeviceHealth) {
        if let Some(slot) = self.drivers.get(id) {
            slot.update_health(health);
        }
    }
    pub fn try_register(
        &self,
        id: String,
        driver: Arc<dyn SensorDriver>,
    ) -> Result<(u16, u16), DriverError> {
        let ports = self.reserve(&id)?;
        if let Some(slot) = self.drivers.get(&id) {
            slot.replace(driver)?;
        } else {
            self.drivers.insert(
                id.clone(),
                Arc::new(super::slot::DriverSlot::new(id, driver)),
            );
        }
        Ok(ports)
    }
    pub fn detach(&self, id: &str) -> Option<Arc<dyn SensorDriver>> {
        self.drivers.get(id).and_then(|s| s.take())
    }
    pub fn get(&self, id: &str) -> Result<Arc<dyn SensorDriver>, DriverError> {
        self.drivers
            .get(id)
            .map(|r| r.value().clone() as Arc<dyn SensorDriver>)
            .ok_or_else(|| DriverError::NotFound(id.into()))
    }
    pub fn ports(&self, id: &str) -> Option<(u16, u16)> {
        self.ports
            .lock()
            .unwrap()
            .assignments
            .sensors
            .get(id)
            .map(|p| (p[0], p[1]))
    }
    pub fn list(&self) -> Vec<SensorInfo> {
        self.describe(false)
    }
    pub fn all(&self) -> Vec<SensorInfo> {
        self.describe(true)
    }
    fn describe(&self, include_offline: bool) -> Vec<SensorInfo> {
        let mut result = Vec::new();
        for entry in self.drivers.iter() {
            let driver = entry.value();
            if !include_offline && !driver.online() {
                continue;
            }
            let id = entry.key().clone();
            let (data_port, command_port) = self.ports(&id).unwrap();
            result.push(SensorInfo {
                id,
                display_name: driver.display_name().into(),
                data_schema: driver.data_schema(),
                command_schema: driver.command_schema(),
                command_mode: driver.command_mode(),
                data_port,
                command_port,
                has_estop: driver.has_estop(),
                estop_confirmation: driver.estop_confirmation(),
                has_config: driver.has_config(),
                has_calibrate: driver.has_calibrate(),
                has_save_config: driver.has_save_config(),
                has_endpoint_write: driver.has_endpoint_write(),
                data_delivery: driver.data_delivery(),
            });
        }
        result.sort_by(|a, b| a.id.cmp(&b.id));
        result
    }
    pub fn len(&self) -> usize {
        self.list().len()
    }
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    pub fn iter_drivers(&self) -> Vec<(String, Arc<dyn SensorDriver>)> {
        self.drivers
            .iter()
            .map(|e| (e.key().clone(), e.value().clone() as Arc<dyn SensorDriver>))
            .collect()
    }
}
