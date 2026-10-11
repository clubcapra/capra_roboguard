use super::{
    driver::{CommandMode, DataDelivery, DataUpdate, FieldDescriptor, SensorDriver},
    error::DriverError,
};
use serde_json::Value;
use std::sync::{Arc, Mutex};

/// Stable object captured by HTTP/UDP clients. The hardware session can change
/// without replacing this slot or releasing its transport sockets.
pub struct DriverSlot {
    pub id: String,
    name: String,
    data: Vec<FieldDescriptor>,
    command: Vec<FieldDescriptor>,
    mode: CommandMode,
    flags: [bool; 5],
    estop_confirmation: Option<String>,
    inner: Mutex<Option<Arc<dyn SensorDriver>>>,
    ready: std::sync::atomic::AtomicBool,
    health: Mutex<super::supervisor::DeviceHealth>,
    delivery: DataDelivery,
    updates: tokio::sync::broadcast::Sender<DataUpdate>,
    relay: Mutex<Option<super::task::WorkerTask>>,
}

impl DriverSlot {
    pub fn new(id: String, driver: Arc<dyn SensorDriver>) -> Self {
        let (updates, _) = tokio::sync::broadcast::channel(2048);
        let relay = Self::relay_updates(driver.as_ref(), updates.clone());
        Self {
            delivery: driver.data_delivery(),
            updates,
            relay: Mutex::new(relay),
            id,
            name: driver.display_name().into(),
            data: driver.data_schema(),
            command: driver.command_schema(),
            mode: driver.command_mode(),
            flags: [
                driver.has_estop(),
                driver.has_config(),
                driver.has_calibrate(),
                driver.has_save_config(),
                driver.has_endpoint_write(),
            ],
            estop_confirmation: driver.estop_confirmation(),
            inner: Mutex::new(Some(driver)),
            ready: std::sync::atomic::AtomicBool::new(false),
            health: Mutex::new(Default::default()),
        }
    }
    pub fn replace(&self, driver: Arc<dyn SensorDriver>) -> Result<(), DriverError> {
        // A slot's schema and capabilities are a contract; reconnect may not
        // silently change them underneath existing consumers.
        if serde_json::to_value(driver.data_schema())? != serde_json::to_value(&self.data)?
            || serde_json::to_value(driver.command_schema())?
                != serde_json::to_value(&self.command)?
            || driver.command_mode() != self.mode
            || driver.data_delivery() != self.delivery
            || driver.estop_confirmation() != self.estop_confirmation
            || [
                driver.has_estop(),
                driver.has_config(),
                driver.has_calibrate(),
                driver.has_save_config(),
                driver.has_endpoint_write(),
            ] != self.flags
        {
            return Err(DriverError::InitFailed(
                "driver contract changed; restart required".into(),
            ));
        }
        *self.relay.lock().unwrap() = Self::relay_updates(driver.as_ref(), self.updates.clone());
        *self.inner.lock().unwrap() = Some(driver);
        Ok(())
    }
    fn relay_updates(
        driver: &dyn SensorDriver,
        updates: tokio::sync::broadcast::Sender<DataUpdate>,
    ) -> Option<super::task::WorkerTask> {
        let mut source = driver.subscribe_updates()?;
        Some(super::task::WorkerTask::spawn(async move {
            loop {
                match source.recv().await {
                    Ok(update) => {
                        let _ = updates.send(update);
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                        let _ = updates.send(DataUpdate::Dropped(n));
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                }
            }
        }))
    }
    pub fn take(&self) -> Option<Arc<dyn SensorDriver>> {
        self.relay.lock().unwrap().take();
        self.ready
            .store(false, std::sync::atomic::Ordering::Release);
        let mut health = self.health.lock().unwrap();
        health.stale = true;
        health.healthy = false;
        drop(health);
        self.inner.lock().unwrap().take()
    }
    pub fn online(&self) -> bool {
        self.ready.load(std::sync::atomic::Ordering::Acquire)
    }
    pub fn update_health(&self, health: super::supervisor::DeviceHealth) {
        let ready = !health.stale && (health.healthy || self.online());
        self.ready
            .store(ready, std::sync::atomic::Ordering::Release);
        *self.health.lock().unwrap() = health;
    }
    pub fn mark_sim_ready(&self) {
        self.ready.store(true, std::sync::atomic::Ordering::Release);
    }
    fn call(
        &self,
        f: impl FnOnce(&dyn SensorDriver) -> Result<Value, DriverError>,
    ) -> Result<Value, DriverError> {
        let driver = self
            .inner
            .lock()
            .unwrap()
            .clone()
            .ok_or_else(|| DriverError::Unavailable(self.id.clone()))?;
        f(driver.as_ref())
    }
}

impl SensorDriver for DriverSlot {
    fn health(&self) -> Result<Value, DriverError> {
        Ok(
            serde_json::json!({"controller_connected":self.inner.lock().unwrap().is_some(),"hardware_ready":self.online(),"telemetry":self.health.lock().unwrap().clone()}),
        )
    }
    fn data_delivery(&self) -> DataDelivery {
        self.delivery
    }
    fn subscribe_updates(&self) -> Option<tokio::sync::broadcast::Receiver<DataUpdate>> {
        (self.delivery == DataDelivery::Updates).then(|| self.updates.subscribe())
    }

    fn id(&self) -> &str {
        &self.id
    }
    fn display_name(&self) -> &str {
        &self.name
    }
    fn data_schema(&self) -> Vec<FieldDescriptor> {
        self.data.clone()
    }
    fn command_schema(&self) -> Vec<FieldDescriptor> {
        self.command.clone()
    }
    fn command_mode(&self) -> CommandMode {
        self.mode
    }
    fn has_estop(&self) -> bool {
        self.flags[0]
    }
    fn estop_confirmation(&self) -> Option<String> {
        self.estop_confirmation.clone()
    }
    fn has_config(&self) -> bool {
        self.flags[1]
    }
    fn has_calibrate(&self) -> bool {
        self.flags[2]
    }
    fn has_save_config(&self) -> bool {
        self.flags[3]
    }
    fn has_endpoint_write(&self) -> bool {
        self.flags[4]
    }
    fn read_data(&self) -> Result<Value, DriverError> {
        self.call(|d| d.read_data())
    }
    fn execute_command(&self, v: &Value) -> Result<Value, DriverError> {
        super::validation::command(v, &self.command)?;
        let needs_ready = self.command.iter().any(|field| {
            v.get(&field.name).is_some_and(|value| {
                field.requires_ready
                    || field
                        .ready_value
                        .as_ref()
                        .is_some_and(|required| value == required)
            })
        });
        if needs_ready && !self.online() {
            return Err(DriverError::Unavailable(
                "hardware is not initialized; command rejected".into(),
            ));
        }
        self.call(|d| d.execute_command(v))
    }
    fn estop(&self) -> Result<Value, DriverError> {
        self.call(|d| d.estop())
    }
    fn read_config(&self) -> Result<Value, DriverError> {
        self.call(|d| d.read_config())
    }
    fn write_config(&self, v: &Value) -> Result<Value, DriverError> {
        self.call(|d| d.write_config(v))
    }
    fn calibrate(&self, v: &Value) -> Result<Value, DriverError> {
        if !self.online() {
            return Err(DriverError::Unavailable(
                "hardware is not initialized".into(),
            ));
        }
        self.call(|d| d.calibrate(v))
    }
    fn save_config(&self) -> Result<Value, DriverError> {
        self.call(|d| d.save_config())
    }
    fn list_endpoints(&self) -> Result<Value, DriverError> {
        self.call(|d| d.list_endpoints())
    }
    fn read_endpoint(&self, p: &str) -> Result<Value, DriverError> {
        self.call(|d| d.read_endpoint(p))
    }
    fn write_endpoint(&self, p: &str, v: &Value) -> Result<Value, DriverError> {
        self.call(|d| d.write_endpoint(p, v))
    }
}
