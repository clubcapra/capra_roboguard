use std::sync::mpsc::Sender;
use std::sync::Mutex;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use serde_json::Value;

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;

use super::state::KinovaState;
use super::worker::{Cmd, STREAM_HINT_INTERVAL};

pub const KINOVA_ID: &str = "kinova_arm";

const VELOCITY_INGEST_MIN_INTERVAL: Duration = Duration::from_millis(10);

/// Kinova arm data schema. Free fn so the sim mock (`kinova::mock`) serves it verbatim.
pub fn data_schema() -> Vec<FieldDescriptor> {
    let mut v = Vec::with_capacity(32);
    for i in 1..=6 {
        v.push(
            FieldDescriptor::new(&format!("joint_{i}_pos"), "Joint angle", "f32").with_unit("deg"),
        );
    }
    for i in 1..=6 {
        v.push(
            FieldDescriptor::new(&format!("joint_{i}_vel"), "Joint velocity", "f32")
                .with_unit("deg/s"),
        );
    }
    for i in 1..=6 {
        v.push(
            FieldDescriptor::new(&format!("joint_{i}_current"), "Joint current", "f32")
                .with_unit("A"),
        );
    }
    for i in 1..=6 {
        v.push(
            FieldDescriptor::new(&format!("joint_{i}_temp"), "Actuator temperature", "f32")
                .with_unit("°C"),
        );
    }
    v.extend([
        FieldDescriptor::new("limp_mode", "Confirmed SDK torque mode with zero commanded torque and suppressed gravity vector", "bool"),
        FieldDescriptor::new("limp_supported", "Loaded SDK exports every required torque function", "bool"),
        FieldDescriptor::new("motion_inhibited", "Position, velocity and homing commands are blocked during limp mode or an uncertain transition", "bool"),
        FieldDescriptor::new("bus_voltage", "Main 24 V supply", "f32").with_unit("V"),
        FieldDescriptor::new("bus_current", "Total bus current", "f32").with_unit("A"),
        FieldDescriptor::new("accel_x", "Base IMU accel X", "f32").with_unit("G"),
        FieldDescriptor::new("accel_y", "Base IMU accel Y", "f32").with_unit("G"),
        FieldDescriptor::new("accel_z", "Base IMU accel Z", "f32").with_unit("G"),
        FieldDescriptor::new(
            "timestamp_ns",
            "Unix timestamp (ns) of last accepted position read; 0 until first valid poll",
            "i64",
        )
        .with_unit("ns"),
    ]);
    v
}

/// Kinova arm command schema. Free fn so the sim mock can reuse it verbatim.
pub fn command_schema() -> Vec<FieldDescriptor> {
    let mut v = Vec::with_capacity(14);
    for i in 1..=6 {
        v.push(
            FieldDescriptor::new(&format!("joint_{i}_pos"), "Joint position setpoint", "f32")
                .with_unit("deg")
                .requiring_ready(),
        );
    }
    for i in 1..=6 {
        v.push(
            FieldDescriptor::new(&format!("joint_{i}_vel"), "Joint velocity setpoint", "f32")
                .with_unit("deg/s")
                .requiring_ready(),
        );
    }
    v.extend([
        FieldDescriptor::new("limp_mode", "Enable zero commanded torque and suppress gravity compensation; the arm can fall under gravity. Disable explicitly to restore normal control. Send this field alone.", "bool").standalone().requiring_ready_for(serde_json::json!(true)).confirm_for(serde_json::json!(true), "The arm can fall under gravity. Clear its workspace and support its load before releasing it."),
        FieldDescriptor::new(
            "move_home",
            "Drive to the firmware home pose (Ethernet_MoveHome).",
            "bool",
        ).requiring_ready_for(serde_json::json!(true)),
        FieldDescriptor::new(
            "erase_trajectories",
            "Cancel queued position trajectories. Does not touch control state.",
            "bool",
        ),
        FieldDescriptor::new(
            "set_joint_zero",
            "Persist the current physical position of joint N (1..=6) as its new zero (writes to actuator flash). Takes effect after power-cycle.",
            "u8",
        ),
    ]);
    v
}

pub struct KinovaArm {
    state: Arc<RwLock<KinovaState>>,
    cmd_tx: Sender<Cmd>,
    last_velocity_at: Mutex<Option<Instant>>,
    stopping: std::sync::atomic::AtomicBool,
    worker: Mutex<Option<std::thread::JoinHandle<()>>>,
}

impl KinovaArm {
    pub fn new(
        state: Arc<RwLock<KinovaState>>,
        cmd_tx: Sender<Cmd>,
        worker: std::thread::JoinHandle<()>,
    ) -> Self {
        Self {
            state,
            cmd_tx,
            last_velocity_at: Mutex::new(None),
            stopping: std::sync::atomic::AtomicBool::new(false),
            worker: Mutex::new(Some(worker)),
        }
    }

    fn send(&self, cmd: Cmd) -> Result<(), DriverError> {
        if self.stopping.load(std::sync::atomic::Ordering::Acquire) {
            return Err(DriverError::Unavailable(KINOVA_ID.into()));
        }
        if {
            let s = self.state.read().unwrap();
            s.motion_inhibited || s.limp_transition
        } {
            return Err(DriverError::Unavailable(
                "disable limp mode before issuing arm commands".into(),
            ));
        }
        self.cmd_tx
            .send(cmd)
            .map_err(|e| DriverError::CommandFailed(format!("Kinova worker channel closed: {e}")))
    }

    fn set_limp(&self, enabled: bool) -> Result<Value, DriverError> {
        self.request_limp(enabled, false)
    }
    fn request_limp(&self, enabled: bool, emergency: bool) -> Result<Value, DriverError> {
        if self.stopping.load(std::sync::atomic::Ordering::Acquire) {
            return Err(DriverError::Unavailable(KINOVA_ID.into()));
        }
        {
            let mut state = self.state.write().unwrap();
            if !state.limp_supported && !emergency {
                return Err(DriverError::InvalidCommand(
                    "SDK torque functions unavailable".into(),
                ));
            }
            if state.limp_transition && !emergency {
                return Err(DriverError::Unavailable(
                    "limp transition already pending".into(),
                ));
            }
            state.limp_transition = true;
        }
        let (reply_tx, reply_rx) = std::sync::mpsc::channel();
        self.cmd_tx
            .send(if emergency {
                Cmd::Estop(reply_tx)
            } else {
                Cmd::SetLimp(enabled, reply_tx)
            })
            .map_err(|e| {
                let mut state = self.state.write().unwrap();
                state.limp_transition = false;
                state.motion_inhibited = true;
                state.control_error = Some(format!("Arm command worker unavailable: {e}"));
                DriverError::CommandFailed(e.to_string())
            })?;
        match reply_rx.recv_timeout(Duration::from_secs(2)) {
            Ok(Ok(())) => Ok(serde_json::json!({"limp_mode": enabled})),
            Ok(Err(e)) => Err(DriverError::CommandFailed(e)),
            Err(_) => Err(DriverError::Unavailable(
                "limp transition pending; inspect arm health before retrying".into(),
            )),
        }
    }

    fn forward_velocity(&self, joints: [f32; 6]) -> Result<bool, DriverError> {
        let is_stop = joints.iter().all(|&v| v == 0.0);
        let now = Instant::now();
        let mut last = self.last_velocity_at.lock().unwrap();
        match *last {
            Some(t) if !is_stop && now.duration_since(t) < VELOCITY_INGEST_MIN_INTERVAL => {
                Ok(false)
            }
            _ => {
                *last = Some(now);
                drop(last);
                self.send(Cmd::SetAngularVelocity(joints))?;
                Ok(true)
            }
        }
    }
}

impl SensorDriver for KinovaArm {
    fn has_estop(&self) -> bool {
        true
    }
    fn estop_confirmation(&self) -> Option<String> {
        Some(limp_confirmation())
    }
    fn estop(&self) -> Result<Value, DriverError> {
        self.request_limp(true, true)?;
        Ok(serde_json::json!({"estop":"trajectories cleared; limp mode enabled", "limp_mode":true}))
    }
    fn diagnostics(&self) -> Vec<String> {
        let s = self.state.read().unwrap();
        let mut errors = s.errors.clone();
        errors.extend(s.control_error.clone());
        errors.extend(s.command_error.clone());
        errors.extend(s.velocity_error.clone());
        errors.extend(s.keepalive_error.clone());
        errors
    }
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        Box::pin(async move {
            self.stopping
                .store(true, std::sync::atomic::Ordering::Release);
            let _ = self.cmd_tx.send(Cmd::Shutdown);
            let worker = self.worker.lock().unwrap().take();
            if let Some(worker) = worker {
                tokio::task::spawn_blocking(move || worker.join())
                    .await
                    .map_err(|e| DriverError::CommandFailed(e.to_string()))?
                    .map_err(|_| DriverError::CommandFailed("Kinova worker panicked".into()))?;
            }
            Ok(())
        })
    }
    fn id(&self) -> &str {
        KINOVA_ID
    }

    fn display_name(&self) -> &str {
        "Kinova Gen2 6DOF (Custom Spherical)"
    }

    fn command_mode(&self) -> CommandMode {
        CommandMode::Stream {
            interval_ms: STREAM_HINT_INTERVAL.as_millis() as u64,
        }
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        data_schema()
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        command_schema()
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        let s = self.state.read().unwrap();
        let mut o = serde_json::Map::with_capacity(32);
        for i in 0..6 {
            o.insert(
                format!("joint_{}_pos", i + 1),
                serde_json::json!(s.joint_pos[i]),
            );
        }
        for i in 0..6 {
            o.insert(
                format!("joint_{}_vel", i + 1),
                serde_json::json!(s.joint_vel[i]),
            );
        }
        for i in 0..6 {
            o.insert(
                format!("joint_{}_current", i + 1),
                serde_json::json!(s.joint_current[i]),
            );
        }
        for i in 0..6 {
            o.insert(
                format!("joint_{}_temp", i + 1),
                serde_json::json!(s.joint_temp[i]),
            );
        }
        o.insert("limp_mode".into(), serde_json::json!(s.limp_mode));
        o.insert("limp_supported".into(), serde_json::json!(s.limp_supported));
        o.insert(
            "motion_inhibited".into(),
            serde_json::json!(s.motion_inhibited || s.limp_transition),
        );
        o.insert("bus_voltage".into(), serde_json::json!(s.bus_voltage));
        o.insert("bus_current".into(), serde_json::json!(s.bus_current));
        o.insert("accel_x".into(), serde_json::json!(s.accel_x));
        o.insert("accel_y".into(), serde_json::json!(s.accel_y));
        o.insert("accel_z".into(), serde_json::json!(s.accel_z));
        o.insert("timestamp_ns".into(), serde_json::json!(s.timestamp_ns));
        Ok(Value::Object(o))
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        if let Some(enabled) = payload.get("limp_mode") {
            if payload.as_object().map(|o| o.len()) != Some(1) {
                return Err(DriverError::InvalidCommand(
                    "limp_mode must be sent alone".into(),
                ));
            }
            let enabled = enabled
                .as_bool()
                .ok_or_else(|| DriverError::InvalidCommand("limp_mode must be a boolean".into()))?;
            return self.set_limp(enabled);
        }
        let mut sent: Vec<&str> = Vec::new();

        if payload
            .get("erase_trajectories")
            .and_then(Value::as_bool)
            .unwrap_or(false)
        {
            self.send(Cmd::EraseTrajectories)?;
            sent.push("erase_trajectories");
        }
        if payload
            .get("move_home")
            .and_then(Value::as_bool)
            .unwrap_or(false)
        {
            self.send(Cmd::MoveHome)?;
            sent.push("move_home");
        }
        if let Some(j) = payload.get("set_joint_zero").and_then(Value::as_u64) {
            let joint = j as u8;
            let addr = super::ffi::joint_to_actuator_address(joint).ok_or_else(|| {
                DriverError::CommandFailed(format!(
                    "set_joint_zero: joint must be 1..=6, got {joint}"
                ))
            })?;
            self.send(Cmd::SetJointZero(addr))?;
            sent.push("set_joint_zero");
        }

        let pos = collect_joint_array(payload, "pos")?;
        let vel = collect_joint_array(payload, "vel")?;
        match (pos, vel) {
            (Some(_), Some(_)) => {
                return Err(DriverError::CommandFailed(
                    "send either joint_*_pos OR joint_*_vel, not both in the same packet".into(),
                ));
            }
            (Some(joints), None) => {
                self.send(Cmd::SetAngularPosition(joints))?;
                sent.push("angular_position");
            }
            (None, Some(joints)) => {
                if self.forward_velocity(joints)? {
                    sent.push("angular_velocity");
                } else {
                    sent.push("angular_velocity_dropped_rate_limit");
                }
            }
            (None, None) => {}
        }

        if sent.is_empty() {
            return Err(DriverError::CommandFailed(
                "no recognised command fields in payload".into(),
            ));
        }
        Ok(serde_json::json!({ "sent": sent }))
    }

    fn has_calibrate(&self) -> bool {
        true
    }

    fn calibrate(&self, _params: &Value) -> Result<Value, DriverError> {
        self.send(Cmd::MoveHome)?;
        Ok(serde_json::json!({ "started": "MoveHome" }))
    }
}

pub fn limp_confirmation() -> String {
    "The arm can fall under gravity. Clear the area before releasing actuator torque. Normal motion remains blocked until limp mode is explicitly disabled.".into()
}

fn collect_joint_array(payload: &Value, suffix: &str) -> Result<Option<[f32; 6]>, DriverError> {
    let mut found = [None; 6];
    for i in 0..6 {
        let key = format!("joint_{}_{}", i + 1, suffix);
        if let Some(v) = payload.get(&key).and_then(Value::as_f64) {
            found[i] = Some(v as f32);
        }
    }
    let count = found.iter().filter(|v| v.is_some()).count();
    match count {
        0 => Ok(None),
        6 => Ok(Some(found.map(|v| v.unwrap()))),
        n => {
            let missing: Vec<String> = found
                .iter()
                .enumerate()
                .filter_map(|(i, v)| v.is_none().then(|| format!("joint_{}_{}", i + 1, suffix)))
                .collect();
            Err(DriverError::CommandFailed(format!(
                "partial joint_*_{suffix} setpoint ({n}/6 set); missing: {}",
                missing.join(", ")
            )))
        }
    }
}

impl Drop for KinovaArm {
    fn drop(&mut self) {
        self.stopping
            .store(true, std::sync::atomic::Ordering::Release);
        let _ = self.cmd_tx.send(Cmd::Shutdown);
        // Explicit shutdown joins the worker; Drop provides the cancellation
        // fallback for failed attachment or an initialization result discarded.
    }
}
