//! Independent recovery loops. Resource-sharing SDKs are retried as a group;
//! stable slots and UDP sockets survive replacement of the hardware session.
use super::{driver::SensorDriver, registry::SensorRegistry};
use crate::{
    logging::LogManager,
    udp::server::{spawn_sensor_udp, UdpListeners},
};
use futures::future::BoxFuture;
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
use tokio::sync::{Mutex as AsyncMutex, Notify};

pub type Devices = Vec<(String, Arc<dyn SensorDriver>)>;
pub type Factory = Arc<dyn Fn() -> BoxFuture<'static, Result<Devices, String>> + Send + Sync>;
pub type ConfigValidator = Arc<dyn Fn(&str) -> Result<Vec<u16>, String> + Send + Sync>;

#[derive(Clone, Serialize, Deserialize, utoipa::ToSchema)]
#[serde(default, deny_unknown_fields)]
pub struct RetryPolicy {
    pub max_attempts: u32,
    pub attempt_timeout_ms: u64,
    pub retry_delay_ms: u64,
    pub stale_after_ms: u64,
}
impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            max_attempts: 3,
            attempt_timeout_ms: 30_000,
            retry_delay_ms: 2_000,
            stale_after_ms: 3_000,
        }
    }
}
impl RetryPolicy {
    pub fn validate(&self) -> Result<(), String> {
        if !(1..=100).contains(&self.max_attempts)
            || !(1_000..=300_000).contains(&self.attempt_timeout_ms)
            || !(100..=300_000).contains(&self.retry_delay_ms)
            || !(500..=60_000).contains(&self.stale_after_ms)
        {
            return Err("invalid retry policy: attempts 1..100, timeout 1000..300000ms, delay 100..300000ms, stale 500..60000ms".into());
        }
        Ok(())
    }
}

#[derive(Clone, Default, Serialize, Deserialize)]
pub struct DeviceHealth {
    pub healthy: bool,
    pub stale: bool,
    pub last_seen_ns: Option<u64>,
    pub errors: Vec<String>,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct DriverStatus {
    pub id: String,
    pub state: String,
    pub attempt: u32,
    pub policy: RetryPolicy,
    pub sensors: BTreeMap<String, DeviceHealth>,
    pub error: Option<String>,
    pub last_error: Option<String>,
    pub next_retry_ms: Option<u64>,
    pub previous_state: Option<String>,
    pub last_ready_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub configuration_endpoint: Option<String>,
}

struct Group {
    status: Mutex<DriverStatus>,
    retry: Notify,
    factory: Factory,
    config: Mutex<Option<(String, ConfigValidator)>>,
}

pub struct Supervisor {
    pub registry: Arc<SensorRegistry>,
    groups: Mutex<BTreeMap<String, Arc<Group>>>,
    listeners: AsyncMutex<BTreeMap<String, UdpListeners>>,
    log: Arc<LogManager>,
    push_ms: u64,
    policy: RetryPolicy,
    tasks: Mutex<Vec<super::task::WorkerTask>>,
    pub simulated: std::sync::atomic::AtomicBool,
}

impl Supervisor {
    pub fn new(
        registry: Arc<SensorRegistry>,
        log: Arc<LogManager>,
        push_ms: u64,
        policy: RetryPolicy,
    ) -> Arc<Self> {
        Arc::new(Self {
            registry,
            groups: Mutex::new(BTreeMap::new()),
            listeners: AsyncMutex::new(BTreeMap::new()),
            log,
            push_ms,
            policy,
            tasks: Mutex::new(Vec::new()),
            simulated: std::sync::atomic::AtomicBool::new(false),
        })
    }
    pub fn keep_task(&self, task: super::task::WorkerTask) {
        self.tasks.lock().unwrap().push(task);
    }
    pub fn add(self: &Arc<Self>, id: &str, factory: Factory) -> Result<(), String> {
        let policy = super::config::load_optional::<RetryPolicy>(&format!("{id}.recovery.toml"))
            .map_err(|e| e.to_string())?
            .unwrap_or_else(|| self.policy.clone());
        policy.validate()?;
        let previous = std::fs::read(
            self.log
                .log_dir()
                .join("driver_state")
                .join(format!("{id}.json")),
        )
        .ok()
        .and_then(|bytes| serde_json::from_slice::<DriverStatus>(&bytes).ok());
        let group = Arc::new(Group {
            status: Mutex::new(DriverStatus {
                id: id.into(),
                state: "connecting".into(),
                attempt: 0,
                policy,
                sensors: BTreeMap::new(),
                error: None,
                last_error: previous.as_ref().and_then(|p| p.last_error.clone()),
                next_retry_ms: None,
                previous_state: previous.as_ref().map(|p| p.state.clone()),
                last_ready_ns: previous.and_then(|p| p.last_ready_ns),
                configuration_endpoint: None,
            }),
            retry: Notify::new(),
            factory,
            config: Mutex::new(None),
        });
        self.groups.lock().unwrap().insert(id.into(), group.clone());
        let manager = self.clone();
        tokio::spawn(async move {
            manager.run(group).await;
        });
        Ok(())
    }
    pub fn set_policy(&self, id: &str, policy: RetryPolicy) -> Result<(), String> {
        policy.validate()?;
        let group = self
            .groups
            .lock()
            .unwrap()
            .get(id)
            .cloned()
            .ok_or("unknown driver")?;
        let path = super::config::config_path(&format!("{id}.recovery.toml"))
            .map_err(|e| e.to_string())?;
        super::config::atomic_write(
            &path,
            toml::to_string_pretty(&policy)
                .map_err(|e| e.to_string())?
                .as_bytes(),
        )
        .map_err(|e| e.to_string())?;
        group.status.lock().unwrap().policy = policy;
        Ok(())
    }
    /// A driver factory opts into connection editing with its own validator.
    pub fn configure(
        &self,
        id: &str,
        filename: &str,
        validate: ConfigValidator,
    ) -> Result<(), String> {
        if filename.contains('/') || filename.contains('\\') || !filename.ends_with(".toml") {
            return Err("invalid configuration filename".into());
        }
        let group = self
            .groups
            .lock()
            .unwrap()
            .get(id)
            .cloned()
            .ok_or("unknown driver")?;
        *group.config.lock().unwrap() = Some((filename.into(), validate));
        group.status.lock().unwrap().configuration_endpoint =
            Some(format!("/admin/drivers/{id}/config"));
        Ok(())
    }
    pub fn configuration(&self, id: &str) -> Result<(std::path::PathBuf, ConfigValidator), String> {
        let group = self
            .groups
            .lock()
            .unwrap()
            .get(id)
            .cloned()
            .ok_or("unknown driver")?;
        let (filename, validate) = group
            .config
            .lock()
            .unwrap()
            .clone()
            .ok_or("connection editing is unavailable for this driver")?;
        Ok((
            super::config::config_path(&filename).map_err(|e| e.to_string())?,
            validate,
        ))
    }
    pub async fn bind_registered(&self) -> Result<(), String> {
        let mut listeners = self.listeners.lock().await;
        for (id, driver) in self.registry.iter_drivers() {
            let (data, command) = self.registry.ports(&id).unwrap();
            listeners.insert(
                id,
                spawn_sensor_udp(driver, data, command, self.log.clone(), self.push_ms)
                    .await
                    .map_err(|e| e.to_string())?,
            );
        }
        Ok(())
    }
    pub fn status(&self) -> Vec<DriverStatus> {
        self.groups
            .lock()
            .unwrap()
            .values()
            .map(|g| g.status.lock().unwrap().clone())
            .collect()
    }
    pub fn retry(&self, id: &str) -> Result<(), String> {
        let group = self
            .groups
            .lock()
            .unwrap()
            .get(id)
            .cloned()
            .ok_or("unknown driver")?;
        let state = group.status.lock().unwrap().state.clone();
        if matches!(
            state.as_str(),
            "connecting" | "stopping" | "timed_out" | "retrying"
        ) {
            return Err("driver recovery is already running".into());
        }
        group.retry.notify_one();
        Ok(())
    }
    fn transition(&self, group: &Group, state: &str, error: Option<String>) {
        let mut status = group.status.lock().unwrap();
        if status.state != state || status.error != error {
            tracing::info!(driver=%status.id, state, ?error, "driver lifecycle");
            let value = serde_json::json!({"state":state,"error":error,"attempt":status.attempt});
            self.log
                .log_input(&status.id, "lifecycle", "", "", state, &value, Ok(&value));
            if state == "online" || state == "degraded" {
                status.last_ready_ns = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .ok()
                    .map(|d| d.as_nanos() as u64);
            }
            status.state = state.into();
            status.error = error.clone();
            if error.is_some() && state != "connecting" && state != "disabled" {
                status.last_error = error.clone();
            }
            let path = self
                .log
                .log_dir()
                .join("driver_state")
                .join(format!("{}.json", status.id));
            if let Err(e) =
                super::config::atomic_write(&path, &serde_json::to_vec_pretty(&*status).unwrap())
            {
                tracing::warn!(driver=%status.id,error=%e,"could not persist driver diagnostics");
            }
        }
        status.state = state.into();
        if error.is_some() && state != "connecting" && state != "disabled" {
            status.last_error = error.clone();
        }
        status.error = error;
    }
    async fn attach(&self, devices: &Devices) -> Result<(), String> {
        let mut listeners = self.listeners.lock().await;
        for (id, device) in devices {
            let (data, command) = self
                .registry
                .try_register(id.clone(), device.clone())
                .map_err(|e| e.to_string())?;
            if !listeners.contains_key(id) {
                let socket = spawn_sensor_udp(
                    self.registry.get(id).unwrap(),
                    data,
                    command,
                    self.log.clone(),
                    self.push_ms,
                )
                .await
                .map_err(|e| format!("{id}: binding UDP {data}/{command}: {e}"))?;
                listeners.insert(id.clone(), socket);
            }
        }
        Ok(())
    }
    async fn detach(&self, group: &Group, devices: Devices) {
        self.transition(group, "stopping", None);
        // Remove all shared-session references before dropping any of them.
        for (id, _) in &devices {
            self.registry.detach(id);
        }
        for (_, device) in &devices {
            if let Err(e) = device.shutdown().await {
                self.transition(group, "stopping", Some(e.to_string()));
            }
        }
        // SDK unload can block while it joins callback threads.
        let _ = tokio::task::spawn_blocking(move || drop(devices)).await;
    }
    async fn run(self: Arc<Self>, group: Arc<Group>) {
        loop {
            let policy = group.status.lock().unwrap().policy.clone();
            for attempt in 1..=policy.max_attempts {
                {
                    let mut s = group.status.lock().unwrap();
                    s.attempt = attempt;
                    s.next_retry_ms = None;
                }
                self.transition(&group, "connecting", None);
                let future = (group.factory)();
                let mut init = tokio::spawn(future);
                let deadline = Duration::from_millis(policy.attempt_timeout_ms);
                let mut timed_out = false;
                let result = match tokio::time::timeout(deadline, &mut init).await {
                    Ok(r) => r,
                    Err(_) => {
                        timed_out = true;
                        self.transition(&group,"timed_out",Some("initialization timed out; waiting for resource cleanup before another attempt".into()));
                        // Native/blocking SDK calls cannot be safely killed. Quarantine
                        // this attempt until it completes; other drivers keep running.
                        init.await
                    }
                };
                let outcome = match result {
                    Ok(Ok(devices)) if !timed_out => {
                        if devices.is_empty() {
                            Err("no hardware found".into())
                        } else if let Err(error) = self.attach(&devices).await {
                            self.detach(&group, devices).await;
                            Err(error)
                        } else {
                            let result = self.monitor(&group, &devices, &policy).await;
                            self.detach(&group, devices).await;
                            result
                        }
                    }
                    Ok(Ok(devices)) => {
                        self.detach(&group, devices).await;
                        Err("initialization timed out".into())
                    }
                    Ok(Err(error)) => Err(error),
                    Err(error) => Err(format!("initialization task: {error}")),
                };
                if outcome.is_ok() {
                    break;
                } // Manual retry starts a fresh bounded cycle.
                let error = outcome.unwrap_err();
                if group.status.lock().unwrap().attempt == 0 {
                    break;
                } // Stable session reset the failure budget.
                if error.starts_with("disabled:") {
                    self.transition(&group, "disabled", Some(error));
                    group.retry.notified().await;
                    break;
                }
                if attempt == policy.max_attempts {
                    self.transition(&group, "failed", Some(error));
                    group.retry.notified().await;
                } else {
                    self.transition(&group, "retrying", Some(error));
                    group.status.lock().unwrap().next_retry_ms = Some(policy.retry_delay_ms);
                    tokio::time::sleep(Duration::from_millis(policy.retry_delay_ms)).await;
                }
            }
        }
    }
    async fn monitor(
        &self,
        group: &Group,
        devices: &Devices,
        policy: &RetryPolicy,
    ) -> Result<(), String> {
        let started = Instant::now();
        let mut seen: BTreeMap<String, (u64, Instant)> = BTreeMap::new();
        let mut ready = false;
        let mut online_since: Option<Instant> = None;
        loop {
            let mut statuses = BTreeMap::new();
            for (id, device) in devices {
                let driver = self.registry.get(id).map_err(|e| e.to_string())?;
                let mut health = match driver.read_data() {
                    Ok(data) => inspect(
                        device.telemetry_status(&data),
                        id,
                        &mut seen,
                        started,
                        policy.stale_after_ms,
                    ),
                    Err(e) => DeviceHealth {
                        stale: true,
                        errors: vec![e.to_string()],
                        ..Default::default()
                    },
                };
                health.errors.extend(device.diagnostics());
                if let Err(e) = device.readiness() {
                    health.errors.push(e.to_string());
                }
                health.healthy = !health.stale && health.errors.is_empty();
                self.registry.update_health(id, health.clone());
                statuses.insert(id.clone(), health);
            }
            let stale = statuses.values().any(|s| s.stale);
            let has_errors = statuses.values().any(|s| !s.errors.is_empty());
            group.status.lock().unwrap().sensors = statuses;
            if !stale && (ready || !has_errors) {
                ready = true;
                let online = online_since.get_or_insert_with(Instant::now);
                // A recovered, stable session gets a new failure budget.
                if online.elapsed() >= Duration::from_secs(10) {
                    group.status.lock().unwrap().attempt = 0;
                }
                self.transition(group, if has_errors { "degraded" } else { "online" }, None);
            } else {
                online_since = None;
                self.transition(
                    group,
                    "connecting",
                    Some(
                        if has_errors {
                            "hardware initialization incomplete or device reports errors"
                        } else {
                            "waiting for fresh telemetry"
                        }
                        .into(),
                    ),
                );
                let limit = if ready {
                    policy.stale_after_ms
                } else {
                    policy.attempt_timeout_ms
                };
                if ready || started.elapsed() >= Duration::from_millis(limit) {
                    return Err("telemetry stopped or did not become ready before timeout".into());
                }
            }
            tokio::select! {
                _ = group.retry.notified() => return Ok(()),
                _ = tokio::time::sleep(Duration::from_millis(500)) => {},
            }
        }
    }
    pub async fn change_ports(&self, id: &str, data: u16, command: u16) -> Result<(), String> {
        // Serialize with attachment and other edits before reading the old map.
        // A reconnect must bind the commissioned pair, and rollback must restore
        // the most recently committed edit rather than a stale snapshot.
        let mut listeners = self.listeners.lock().await;
        let old = self.registry.ports(id).ok_or("unknown port reservation")?;
        let driver = match self.registry.get(id) {
            Ok(driver) => driver,
            Err(_) => {
                return self
                    .registry
                    .set_ports(id, data, command)
                    .map_err(|e| e.to_string())
            }
        };
        if old == (data, command) {
            return Ok(());
        }
        // Validate a clone before interrupting the current transport.
        let mut assignments = self.registry.assignments();
        assignments.sensors.remove(id);
        if data == 0
            || command == 0
            || data == command
            || assignments
                .sensors
                .values()
                .flatten()
                .any(|p| *p == data || *p == command)
        {
            return Err("ports must be distinct, nonzero, and unused by another sensor".into());
        }
        if let Some(old_listener) = listeners.remove(id) {
            old_listener.stop().await;
        }
        let new = spawn_sensor_udp(
            driver.clone(),
            data,
            command,
            self.log.clone(),
            self.push_ms,
        )
        .await;
        match new {
            Ok(new) => {
                if let Err(error) = self.registry.set_ports(id, data, command) {
                    new.stop().await;
                    let restored =
                        spawn_sensor_udp(driver, old.0, old.1, self.log.clone(), self.push_ms)
                            .await
                            .map_err(|e| e.to_string())?;
                    listeners.insert(id.into(), restored);
                    return Err(error.to_string());
                }
                listeners.insert(id.into(), new);
                let value = serde_json::json!({"data_port":data,"command_port":command});
                self.log
                    .log_input(id, "management", "", "", "ports", &value, Ok(&value));
                Ok(())
            }
            Err(error) => {
                let restored =
                    spawn_sensor_udp(driver, old.0, old.1, self.log.clone(), self.push_ms)
                        .await
                        .map_err(|e| format!("{error}; restoring old ports: {e}"))?;
                listeners.insert(id.into(), restored);
                Err(error.to_string())
            }
        }
    }
}

fn inspect(
    telemetry: super::driver::TelemetryStatus,
    id: &str,
    seen: &mut BTreeMap<String, (u64, Instant)>,
    started: Instant,
    stale_ms: u64,
) -> DeviceHealth {
    let timestamp = telemetry.sample_timestamp_ns;
    let entry = seen.entry(id.into()).or_insert((0, started));
    if timestamp != 0 && timestamp != entry.0 {
        *entry = (timestamp, Instant::now());
    }
    let stale = timestamp == 0
        || entry.1.elapsed() >= Duration::from_millis(stale_ms)
        || !telemetry.connected;
    let errors = telemetry.errors;
    DeviceHealth {
        healthy: !stale && errors.is_empty(),
        stale,
        last_seen_ns: (timestamp != 0).then_some(timestamp),
        errors,
    }
}
