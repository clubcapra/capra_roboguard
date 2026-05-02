//! `SensorDriver` implementation for the Kinova arm.
//!
//! The whole arm is exposed as one device with `joint_1..6_*` data fields and
//! a 6-vector setpoint command. Per-joint addressing like the ODrive CAN
//! driver is intentionally not used — the SDK only accepts whole-arm
//! `TrajectoryPoint` commands.

use std::sync::mpsc::Sender;
use std::sync::Mutex;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use serde_json::Value;

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;

use super::state::KinovaState;
use super::worker::{Cmd, STREAM_HINT_INTERVAL};

/// Stable ID — used as the URL slug and UDP-port lookup key.
pub const KINOVA_ID: &str = "kinova_arm";

/// Minimum spacing between *velocity* setpoints accepted from the UDP/HTTP
/// surface. Matches the arm's 100 Hz DSP cadence — anything faster is
/// guaranteed to fill the arm's 2000-entry trajectory FIFO and translate
/// directly into perceived lag at the operator end. Bursty clients (or a
/// browser slider that fires `oninput` faster than the stream cadence)
/// silently lose the in-between packets here, before they ever hit the
/// worker's mpsc channel.
const VELOCITY_INGEST_MIN_INTERVAL: Duration = Duration::from_millis(10);

pub struct KinovaArm {
    state: Arc<RwLock<KinovaState>>,
    cmd_tx: Sender<Cmd>,
    /// Wall-clock of the last *velocity* setpoint we forwarded to the worker.
    /// Used to drop packets that arrive faster than the arm can consume them.
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

    /// Forward a velocity setpoint, dropping packets that arrive faster than
    /// `VELOCITY_INGEST_MIN_INTERVAL`. An all-zero "stop" setpoint always
    /// goes through (and resets the rate-limit clock) so a halt is never
    /// gated on the rate limit. Returns `true` if forwarded, `false` if
    /// dropped.
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
        // Hint only — the worker holds the most recent velocity setpoint and
        // re-sends it internally, so a slower client cadence still keeps the
        // arm moving (until the hold timeout). Position setpoints are
        // FIFO-driven by the arm and need no streaming at all.
        CommandMode::Stream {
            interval_ms: STREAM_HINT_INTERVAL.as_millis() as u64,
        }
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        let mut v = Vec::with_capacity(40);
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
                FieldDescriptor::new(
                    &format!("joint_{i}_torque"),
                    "Joint torque (requires torque sensors)",
                    "f32",
                )
                .with_unit("Nm"),
            );
        }
        for i in 1..=6 {
            v.push(
                FieldDescriptor::new(
                    &format!("joint_{i}_current"),
                    "Actuator motor current",
                    "f32",
                )
                .with_unit("A"),
            );
        }
        for i in 1..=6 {
            v.push(
                FieldDescriptor::new(
                    &format!("joint_{i}_temp"),
                    "Actuator temperature",
                    "f32",
                )
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
                "control_enabled",
                "Whether the arm is currently accepting control commands",
                "bool",
            ),
            FieldDescriptor::new("retract_state", "RetractType from QuickStatus", "u8"),
            FieldDescriptor::new("robot_type", "0 = JACO, 1 = MICO, etc", "u8"),
            FieldDescriptor::new(
                "torque_sensors_available",
                "Whether torque sensors are populated",
                "bool",
            ),
            FieldDescriptor::new(
                "estopped",
                "True after estop() was called and StartControl has not been re-issued",
                "bool",
            ),
            FieldDescriptor::new(
                "timestamp_ns",
                "Unix timestamp (ns) of last successful telemetry refresh; 0 until first poll",
                "i64",
            )
            .with_unit("ns"),
        ]);
        v
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        let mut v = Vec::with_capacity(20);
        v.push(FieldDescriptor::new(
            "control_mode",
            "Set to \"angular\" to (re-)switch to angular control",
            "String",
        ));
        for i in 1..=6 {
            v.push(
                FieldDescriptor::new(&format!("joint_{i}_pos"), "Joint position setpoint", "f32")
                    .with_unit("deg"),
            );
        }
        for i in 1..=6 {
            v.push(
                FieldDescriptor::new(
                    &format!("joint_{i}_vel"),
                    "Joint velocity setpoint (use *instead* of joint_*_pos)",
                    "f32",
                )
                .with_unit("deg/s"),
            );
        }
        v.extend([
            FieldDescriptor::new("move_home", "Move arm to its built-in HOME pose", "bool"),
            FieldDescriptor::new("clear_errors", "Clear the SDK error log", "bool"),
            FieldDescriptor::new(
                "erase_trajectories",
                "Cancel queued trajectories without disabling control",
                "bool",
            ),
            FieldDescriptor::new(
                "start_control",
                "Re-enable control after an estop (calls StartControlAPI)",
                "bool",
            ),
            FieldDescriptor::new(
                "hard_stop",
                "Emergency stop: StopControlAPI + EraseAllTrajectories. Short-circuits all other fields in the same packet. Recover with start_control:true.",
                "bool",
            ),
            FieldDescriptor::new(
                "set_joint_zero",
                "Persist current position of joint N (1..=6) as its new zero reference. Writes to actuator flash — call deliberately.",
                "u8",
            ),
            FieldDescriptor::new(
                "gripper_probe",
                "DIAGNOSTIC: switch into RS-485 passthrough and send a Robotiq Modbus probe to slave 9. Hijacks the bus — restart the API process afterward to recover joint control.",
                "bool",
            ),
            FieldDescriptor::new(
                "gripper_diagnostic",
                "DIAGNOSTIC: non-destructive. Calls GetGripperStatus + InitFingers + GetGripperStatus. Logs whether the SDK sees a Robotiq at the joint-7 slot.",
                "bool",
            ),
            FieldDescriptor::new(
                "gripper_activate",
                "Hijacks the bus into RS-485 passthrough and writes a single Robotiq 2F-140 activation frame (FC06 slave 9 reg 0x03E8 = 1). Open-loop. Watch the gripper LED. Restart the API process afterward.",
                "bool",
            ),
        ]);
        v
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        let s = self.state.read().unwrap();
        let mut o = serde_json::Map::with_capacity(40);
        for i in 0..6 {
            o.insert(format!("joint_{}_pos", i + 1), serde_json::json!(s.joint_pos[i]));
        }
        for i in 0..6 {
            o.insert(format!("joint_{}_vel", i + 1), serde_json::json!(s.joint_vel[i]));
        }
        for i in 0..6 {
            o.insert(
                format!("joint_{}_torque", i + 1),
                serde_json::json!(s.joint_torque[i]),
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
        o.insert("bus_voltage".into(), serde_json::json!(s.bus_voltage));
        o.insert("bus_current".into(), serde_json::json!(s.bus_current));
        o.insert("accel_x".into(), serde_json::json!(s.accel_x));
        o.insert("accel_y".into(), serde_json::json!(s.accel_y));
        o.insert("accel_z".into(), serde_json::json!(s.accel_z));
        o.insert("control_enabled".into(), serde_json::json!(s.control_enabled));
        o.insert("retract_state".into(), serde_json::json!(s.retract_state));
        o.insert("robot_type".into(), serde_json::json!(s.robot_type));
        o.insert(
            "torque_sensors_available".into(),
            serde_json::json!(s.torque_sensors_available),
        );
        o.insert("estopped".into(), serde_json::json!(s.estopped));
        o.insert("timestamp_ns".into(), serde_json::json!(s.timestamp_ns));
        Ok(Value::Object(o))
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        let mut sent: Vec<&str> = Vec::new();

        // --- Hard stop has highest priority and short-circuits everything else.
        // Same effect as POST /kinova_arm/estop: StopControlAPI + EraseAllTrajectories.
        // Caller must send {"start_control": true} before any further setpoints land.
        if payload.get("hard_stop").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::Estop)?;
            return Ok(serde_json::json!({
                "sent": ["hard_stop"],
                "recovery_hint": "send {\"start_control\": true} once it is safe to resume",
            }));
        }

        // --- One-shot actions, executed in safest order ---
        if payload.get("clear_errors").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::ClearErrors)?;
            sent.push("clear_errors");
        }
        if payload.get("start_control").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::StartControl)?;
            sent.push("start_control");
        }
        if let Some(mode) = payload.get("control_mode").and_then(Value::as_str) {
            // Treat empty / whitespace as unset (Scalar's form auto-sends "").
            match mode.trim() {
                "" => {}
                "angular" => {
                    self.send(Cmd::SetAngularControl)?;
                    sent.push("set_angular_control");
                }
                other => {
                    return Err(DriverError::CommandFailed(format!(
                        "unsupported control_mode '{other}'; only 'angular' is supported on this custom arm"
                    )));
                }
            }
        }
        if payload.get("erase_trajectories").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::EraseTrajectories)?;
            sent.push("erase_trajectories");
        }
        if payload.get("move_home").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::MoveHome)?;
            sent.push("move_home");
        }
        if payload.get("gripper_probe").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::GripperProbe)?;
            return Ok(serde_json::json!({
                "sent": ["gripper_probe"],
                "warning": "RS485 mode hijacks normal control. Check API logs for the response bytes, then restart the rove_sensor_api process."
            }));
        }
        if payload.get("gripper_diagnostic").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::GripperDiagnostic)?;
            return Ok(serde_json::json!({
                "sent": ["gripper_diagnostic"],
                "note": "Non-destructive — runs GetGripperStatus + InitFingers. Check API logs for finger state before/after init."
            }));
        }
        if payload.get("gripper_activate").and_then(Value::as_bool).unwrap_or(false) {
            self.send(Cmd::GripperActivate)?;
            return Ok(serde_json::json!({
                "sent": ["gripper_activate"],
                "warning": "RS485 mode hijacks normal control. Watch the gripper LED — solid red → blinking red/blue means the write reached it. Restart the rove_sensor_api process afterward."
            }));
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

        // --- Setpoints. Position OR velocity, never both in one packet. ---
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

    fn has_estop(&self) -> bool {
        true
    }

    fn estop(&self) -> Result<Value, DriverError> {
        self.send(Cmd::Estop)?;
        tracing::warn!("Kinova ESTOP queued — trajectories will be erased and control stopped");
        Ok(serde_json::json!({
            "estop": "queued",
            "recovery_hint": "send {\"start_control\": true} once the situation is safe"
        }))
    }

    fn has_calibrate(&self) -> bool {
        true
    }

    /// "Calibration" on this arm = move to the built-in HOME pose. The legacy
    /// SDK has no per-joint calibration analogue (Kinova actuators self-zero
    /// at boot); HOME is the closest thing.
    fn calibrate(&self, _params: &Value) -> Result<Value, DriverError> {
        self.send(Cmd::MoveHome)?;
        Ok(serde_json::json!({
            "started": "MoveHome",
            "note": "legacy Kinova SDK has no per-joint calibration; arm self-zeros at boot"
        }))
    }
}

/// Read `joint_1_<suffix>..joint_6_<suffix>` from the payload as a 6-vector.
///
/// Returns:
/// - `Ok(None)` if no `joint_*_<suffix>` keys are present at all,
/// - `Ok(Some([f32; 6]))` if **all six** are present,
/// - `Err(...)` if some but not all six are present (partial setpoints
///   are rejected because the SDK only accepts whole-arm commands —
///   silently zero-filling the missing joints would yank the arm).
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
                "partial joint_*_{suffix} setpoint ({n}/6 set); the arm only accepts whole-arm commands. Missing: {}",
                missing.join(", ")
            )))
        }
    }
}
