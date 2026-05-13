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

pub struct KinovaArm {
    state: Arc<RwLock<KinovaState>>,
    cmd_tx: Sender<Cmd>,
    last_velocity_at: Mutex<Option<Instant>>,
}

impl KinovaArm {
    pub fn new(state: Arc<RwLock<KinovaState>>, cmd_tx: Sender<Cmd>) -> Self {
        Self {
            state,
            cmd_tx,
            last_velocity_at: Mutex::new(None),
        }
    }

    fn send(&self, cmd: Cmd) -> Result<(), DriverError> {
        self.cmd_tx
            .send(cmd)
            .map_err(|e| DriverError::CommandFailed(format!("Kinova worker channel closed: {e}")))
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
        let mut v = Vec::with_capacity(32);
        for i in 1..=6 {
            v.push(
                FieldDescriptor::new(&format!("joint_{i}_pos"), "Joint angle", "f32")
                    .with_unit("deg"),
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

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        let mut v = Vec::with_capacity(14);
        for i in 1..=6 {
            v.push(
                FieldDescriptor::new(&format!("joint_{i}_pos"), "Joint position setpoint", "f32")
                    .with_unit("deg"),
            );
        }
        for i in 1..=6 {
            v.push(
                FieldDescriptor::new(&format!("joint_{i}_vel"), "Joint velocity setpoint", "f32")
                    .with_unit("deg/s"),
            );
        }
        v.extend([
            FieldDescriptor::new(
                "move_home",
                "Drive to the firmware home pose (Ethernet_MoveHome).",
                "bool",
            ),
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

    fn read_data(&self) -> Result<Value, DriverError> {
        let s = self.state.read().unwrap();
        let mut o = serde_json::Map::with_capacity(32);
        for i in 0..6 {
            o.insert(format!("joint_{}_pos", i + 1), serde_json::json!(s.joint_pos[i]));
        }
        for i in 0..6 {
            o.insert(format!("joint_{}_vel", i + 1), serde_json::json!(s.joint_vel[i]));
        }
        for i in 0..6 {
            o.insert(format!("joint_{}_current", i + 1), serde_json::json!(s.joint_current[i]));
        }
        for i in 0..6 {
            o.insert(format!("joint_{}_temp", i + 1), serde_json::json!(s.joint_temp[i]));
        }
        o.insert("bus_voltage".into(), serde_json::json!(s.bus_voltage));
        o.insert("bus_current".into(), serde_json::json!(s.bus_current));
        o.insert("accel_x".into(), serde_json::json!(s.accel_x));
        o.insert("accel_y".into(), serde_json::json!(s.accel_y));
        o.insert("accel_z".into(), serde_json::json!(s.accel_z));
        o.insert("timestamp_ns".into(), serde_json::json!(s.timestamp_ns));
        Ok(Value::Object(o))
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        let mut sent: Vec<&str> = Vec::new();

        if payload.get("erase_trajectories").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::EraseTrajectories)?;
            sent.push("erase_trajectories");
        }
        if payload.get("move_home").and_then(Value::as_bool).unwrap_or(false) {
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
                .filter_map(|(i, v)| {
                    v.is_none().then(|| format!("joint_{}_{}", i + 1, suffix))
                })
                .collect();
            Err(DriverError::CommandFailed(format!(
                "partial joint_*_{suffix} setpoint ({n}/6 set); missing: {}",
                missing.join(", ")
            )))
        }
    }
}
