use std::sync::{Arc, RwLock};
use std::time::Duration;

use serde_json::Value;
use tokio::sync::Notify;
use tokio_util::sync::CancellationToken;

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;

use super::bus::CanBus;
use super::endpoints::SharedEndpointMap;
use super::protocol::{
    encode_set_axis_state, encode_set_input_pos, encode_sdo_write_bool, encode_sdo_write_f32,
    encode_sdo_write_i32,
    CMD_CLEAR_ERRORS, CMD_ESTOP, CMD_REBOOT, CMD_SET_AXIS_STATE, CMD_SET_CONTROLLER_MODE,
    CMD_SET_INPUT_POS, CMD_SET_INPUT_TORQUE, CMD_SET_INPUT_VEL, CMD_SET_LIMITS,
    CMD_SET_POSITION_GAIN, CMD_SET_TRAJ_ACCEL_LIMITS, CMD_SET_TRAJ_INERTIA,
    CMD_SET_TRAJ_VEL_LIMIT, CMD_SET_VEL_GAINS,
    encode_set_controller_mode, encode_set_input_vel, encode_set_input_torque,
    encode_set_limits, encode_set_traj_vel_limit, encode_set_traj_accel_limits,
    encode_set_traj_inertia, encode_set_position_gain, encode_set_vel_gains,
};
use super::state::OdriveNodeState;

/// Watchdog keepalive configuration.
///
/// The driver sends `Set Input Pos(pos, vel_ff, torque_ff)` plus optionally
/// `Set Axis State(axis_state)` at `interval_ms` to keep the ODrive CAN
/// watchdog from triggering.
#[derive(Debug, Clone)]
pub struct WatchdogConfig {
    /// Send interval in milliseconds. Default: 100.
    pub interval_ms: u64,
    /// Axis state sent on each watchdog tick (1 = Idle, 8 = ClosedLoopControl).
    /// Default is Idle — drives are safe when no control loop is running.
    /// Set to None to skip sending Set Axis State on each tick.
    pub axis_state: Option<u32>,
    /// Position setpoint for the zero message (revolutions). Default: 0.0.
    pub input_pos: f32,
    /// Velocity feedforward for the zero message (rev/s). Default: 0.0.
    pub input_vel_ff: f32,
    /// Torque feedforward for the zero message (Nm). Default: 0.0.
    pub input_torque_ff: f32,
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            interval_ms: 250,
            // Do NOT send SET_AXIS_STATE in the watchdog. In ODrive 0.6.x, transitioning to Idle
            // clears active errors including ESTOP_REQUESTED, making estop useless.
            // The ODrive's own CAN watchdog (axis0.config.watchdog_timeout) handles safety shutdown
            // if CAN messages stop entirely. We just send a zero-vel keepalive.
            axis_state: None,
            input_pos: 0.0,
            input_vel_ff: 0.0,
            input_torque_ff: 0.0,
        }
    }
}

/// SensorDriver implementation for one ODrive CAN node.
pub struct OdriveNode {
    node_id: u8,
    id_str: String,
    display_name: String,
    bus: Arc<CanBus>,
    state: Arc<RwLock<OdriveNodeState>>,
    watchdog: WatchdogConfig,
    /// Notified on every received command — resets the watchdog timer.
    command_notify: Arc<Notify>,
    _watchdog_cancel: CancellationToken,
    /// Shared SDO endpoint map (empty until loaded via env var or HTTP upload).
    endpoint_map: SharedEndpointMap,
}

impl OdriveNode {
    /// Create a new node and start its watchdog task.
    pub fn new(
        node_id: u8,
        bus: Arc<CanBus>,
        state: Arc<RwLock<OdriveNodeState>>,
        watchdog: WatchdogConfig,
        endpoint_map: SharedEndpointMap,
    ) -> Self {
        let id_str = format!("odrive_{}", node_id);
        let display_name = format!("ODrive Node {}", node_id);

        let cancel = CancellationToken::new();
        let command_notify = Arc::new(Notify::new());

        tokio::spawn(watchdog_task(
            node_id,
            bus.clone(),
            watchdog.clone(),
            command_notify.clone(),
            cancel.clone(),
        ));

        tokio::spawn(can_init_task(
            node_id,
            bus.clone(),
            endpoint_map.clone(),
            cancel.clone(),
        ));

        Self {
            node_id,
            id_str,
            display_name,
            bus,
            state,
            watchdog,
            command_notify,
            _watchdog_cancel: cancel,
            endpoint_map,
        }
    }

    /// Blocking helper: send a CAN frame from a sync context inside an async runtime.
    fn send_blocking(&self, cmd_id: u32, data: Vec<u8>) -> Result<(), DriverError> {
        let bus = self.bus.clone();
        let node_id = self.node_id;
        tokio::task::block_in_place(|| {
            tokio::runtime::Handle::current()
                .block_on(async move { bus.send(node_id, cmd_id, &data).await })
        })
        .map_err(|e| DriverError::CommandFailed(e.to_string()))
    }

    /// Blocking SDO read: sends RxSdo and waits up to `timeout` for TxSdo response.
    fn sdo_read_blocking(&self, endpoint_id: u16, timeout: Duration) -> Result<[u8; 4], DriverError> {
        let bus = self.bus.clone();
        let node_id = self.node_id;
        tokio::task::block_in_place(|| {
            tokio::runtime::Handle::current()
                .block_on(async move { bus.sdo_read(node_id, endpoint_id, timeout).await })
        })
        .map_err(|e| DriverError::CommandFailed(e.to_string()))
    }

    /// Blocking SDO write: sends RxSdo write request (no response waiting).
    fn sdo_write_blocking(&self, endpoint_id: u16, payload: [u8; 8]) -> Result<(), DriverError> {
        let bus = self.bus.clone();
        let node_id = self.node_id;
        tokio::task::block_in_place(|| {
            tokio::runtime::Handle::current()
                .block_on(async move { bus.sdo_write(node_id, endpoint_id, payload).await })
        })
        .map_err(|e| DriverError::CommandFailed(e.to_string()))
    }
}

/// Decode 4 raw SDO bytes to a JSON value using the endpoint dtype string.
///
/// f32 values of ±infinity or NaN (used by ODrive to mean "no limit") are
/// returned as the strings `"inf"`, `"-inf"`, and `"nan"` rather than JSON null.
fn decode_sdo_value(dtype: &str, bytes: [u8; 4]) -> Value {
    match dtype {
        "float" => {
            let v = f32::from_le_bytes(bytes);
            if v.is_nan() {
                Value::String("nan".into())
            } else if v.is_infinite() {
                Value::String(if v > 0.0 { "inf".into() } else { "-inf".into() })
            } else {
                serde_json::json!(v)
            }
        }
        "uint32" => serde_json::json!(u32::from_le_bytes(bytes)),
        "uint16" => serde_json::json!(u16::from_le_bytes([bytes[0], bytes[1]])),
        "uint8"  => serde_json::json!(bytes[0] as u32),
        "int32"  => serde_json::json!(i32::from_le_bytes(bytes)),
        "bool"   => serde_json::json!(bytes[0] != 0),
        _ => serde_json::json!({
            "raw_hex": format!("{:02x}{:02x}{:02x}{:02x}", bytes[0], bytes[1], bytes[2], bytes[3]),
            "type": dtype,
        }),
    }
}

/// Encode a JSON value into an 8-byte SDO write payload using the endpoint dtype.
/// Returns `None` if the value cannot be coerced to the required type.
fn encode_sdo_value(ep_id: u16, dtype: &str, val: &Value) -> Option<[u8; 8]> {
    match dtype {
        "float"           => Some(encode_sdo_write_f32(ep_id, val.as_f64()? as f32)),
        "uint32" | "uint16" | "uint8" => Some(encode_sdo_write_i32(ep_id, val.as_u64()? as i32)),
        "int32"           => Some(encode_sdo_write_i32(ep_id, val.as_i64()? as i32)),
        "bool"            => Some(encode_sdo_write_bool(ep_id, val.as_bool()?)),
        _                 => None,
    }
}

/// Watchdog keepalive task — runs for the lifetime of the OdriveNode.
///
/// Fires only when no command has arrived within `interval_ms`. Each call to
/// `execute_command` resets the timer via `notify`, so the watchdog never
/// overlaps with an active command stream.
async fn watchdog_task(
    node_id: u8,
    bus: Arc<CanBus>,
    cfg: WatchdogConfig,
    notify: Arc<Notify>,
    cancel: CancellationToken,
) {
    let interval = Duration::from_millis(cfg.interval_ms);
    loop {
        tokio::select! {
            // Timer elapsed without a command — send the idle watchdog frame.
            _ = tokio::time::sleep(interval) => {
                if let Some(state) = cfg.axis_state {
                    let data = encode_set_axis_state(state).to_vec();
                    if let Err(e) = bus.send(node_id, CMD_SET_AXIS_STATE, &data).await {
                        tracing::warn!(node_id, error = %e, "watchdog: set axis state failed");
                    }
                }
                let data = encode_set_input_pos(cfg.input_pos, cfg.input_vel_ff, cfg.input_torque_ff).to_vec();
                if let Err(e) = bus.send(node_id, CMD_SET_INPUT_POS, &data).await {
                    tracing::warn!(node_id, error = %e, "watchdog: set input pos failed");
                }
            }
            // Command arrived — reset the timer by looping back to a fresh sleep.
            _ = notify.notified() => {}
            _ = cancel.cancelled() => {
                tracing::info!(node_id, "ODrive watchdog stopped");
                break;
            }
        }
    }
}

/// CAN cyclic-message initialisation task.
///
/// In ODrive fw ≥ 0.6 all cyclic messages except Heartbeat (100 ms) and
/// Get_Encoder_Estimates (10 ms) are **disabled by default** (msg_rate_ms = 0).
/// This task waits for the endpoint map to be loaded, then configures the ODrive
/// to broadcast all telemetry messages we need.
///
/// Desired rates (all in ms — lower = faster):
///   iq:           50 ms
///   error:        50 ms
///   temperature: 500 ms
///   bus_voltage:  50 ms
///   torques:      50 ms
///
/// Retries every 3 s if the write fails or the endpoint map is not loaded yet.
async fn can_init_task(
    node_id: u8,
    bus: Arc<CanBus>,
    endpoint_map: SharedEndpointMap,
    cancel: CancellationToken,
) {
    // Rate configuration: endpoint path → desired interval in ms.
    let rates: &[(&str, u32)] = &[
        ("axis0.config.can.iq_msg_rate_ms",            50),
        ("axis0.config.can.error_msg_rate_ms",          50),
        ("axis0.config.can.temperature_msg_rate_ms",   500),
        ("axis0.config.can.bus_voltage_msg_rate_ms",    50),
        ("axis0.config.can.torques_msg_rate_ms",        50),
    ];

    loop {
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_secs(3)) => {}
            _ = cancel.cancelled() => return,
        }

        // Collect endpoint IDs for each rate config path.
        let writes: Vec<(u16, u32)> = {
            let map = endpoint_map.read().unwrap();
            if map.is_empty() { continue; }
            rates.iter()
                .filter_map(|(path, rate)| map.get(*path).map(|e| (e.id, *rate)))
                .collect()
        };

        if writes.is_empty() {
            // Endpoint map loaded but none of the rate paths found — wrong firmware.
            tracing::warn!(node_id, "CAN rate endpoints not found — skipping init (fw < 0.6?)");
            return;
        }

        let mut all_ok = true;
        for (ep_id, rate_ms) in &writes {
            let payload = super::protocol::encode_sdo_write_i32(*ep_id, *rate_ms as i32);
            if let Err(e) = bus.sdo_write(node_id, *ep_id, payload).await {
                tracing::warn!(node_id, error = %e, "CAN init SDO write failed — will retry");
                all_ok = false;
                break;
            }
            // Small gap between writes to avoid flooding the bus.
            tokio::time::sleep(Duration::from_millis(20)).await;
        }

        if all_ok {
            tracing::info!(node_id, "ODrive CAN cyclic messages configured");
            return; // Done — no need to loop again.
        }
    }
}

impl SensorDriver for OdriveNode {
    fn id(&self) -> &str {
        &self.id_str
    }

    fn display_name(&self) -> &str {
        &self.display_name
    }

    fn command_mode(&self) -> CommandMode {
        CommandMode::Stream {
            interval_ms: self.watchdog.interval_ms,
        }
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("node_id",           "ODrive CAN node ID",                         "u8"),
            FieldDescriptor::new("axis_error",         "Axis error flags (0 = no error)",            "u32"),
            FieldDescriptor::new("axis_state",         "Current axis state (8 = ClosedLoopControl)", "u8"),
            FieldDescriptor::new("procedure_result",   "Procedure_Result (fw≥0.6 Heartbeat byte 5)", "u8"),
            FieldDescriptor::new("trajectory_done",    "Trajectory done flag",                       "bool"),
            FieldDescriptor::new("pos_estimate",       "Position estimate",    "f32").with_unit("rev"),
            FieldDescriptor::new("vel_estimate",       "Velocity estimate",    "f32").with_unit("rev/s"),
            FieldDescriptor::new("shadow_count",       "Encoder shadow count", "i32").with_unit("counts"),
            FieldDescriptor::new("count_cpr",          "Encoder CPR",          "i32").with_unit("counts"),
            FieldDescriptor::new("iq_setpoint",        "Current setpoint",     "f32").with_unit("A"),
            FieldDescriptor::new("iq_measured",        "Measured phase current","f32").with_unit("A"),
            FieldDescriptor::new("bus_voltage",        "DC bus voltage",        "f32").with_unit("V"),
            FieldDescriptor::new("bus_current",        "DC bus current",        "f32").with_unit("A"),
            FieldDescriptor::new("active_errors",      "Active error flags (Get_Error)",   "u32"),
            FieldDescriptor::new("disarm_reason",      "Disarm reason (Get_Error)",        "u32"),
            FieldDescriptor::new("torque_target",      "Torque target (Get_Torques)",      "f32").with_unit("Nm"),
            FieldDescriptor::new("torque_estimate",    "Torque estimate (Get_Torques)",    "f32").with_unit("Nm"),
            FieldDescriptor::new("electrical_power",   "Electrical power (Get_Powers)",    "f32").with_unit("W"),
            FieldDescriptor::new("mechanical_power",   "Mechanical power (Get_Powers)",    "f32").with_unit("W"),
            FieldDescriptor::new("fet_temp",           "FET temperature (Get_Temperature, null until received)", "f32").with_unit("°C"),
            FieldDescriptor::new("motor_temp",         "Motor temperature (Get_Temperature, null until received)", "f32").with_unit("°C"),
            FieldDescriptor::new("timestamp_ns",       "Unix timestamp (ns) of the last CAN frame received from this node; 0 until first frame", "i64").with_unit("ns"),
        ]
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            // --- Combined setpoint (primary command) ---
            FieldDescriptor::new(
                "axis_state",
                "Target axis state (optional; 1=Idle, 8=ClosedLoopControl)",
                "u32",
            ),
            FieldDescriptor::new("input_pos", "Position setpoint", "f32").with_unit("rev"),
            FieldDescriptor::new("input_vel_ff", "Velocity feedforward", "f32").with_unit("rev/s"),
            FieldDescriptor::new("input_torque_ff", "Torque feedforward", "f32").with_unit("Nm"),
            // --- Controller mode ---
            FieldDescriptor::new(
                "control_mode",
                "Control mode (0=Voltage,1=Torque,2=Velocity,3=Position)",
                "u32",
            ),
            FieldDescriptor::new(
                "input_mode",
                "Input mode (0=Inactive,1=Passthrough,2=VelRamp,3=PosFilter,5=TrapTraj...)",
                "u32",
            ),
            // --- Alternative setpoints ---
            FieldDescriptor::new("input_vel", "Velocity setpoint (use instead of input_pos)", "f32").with_unit("rev/s"),
            FieldDescriptor::new("input_torque", "Torque setpoint (use instead of input_pos)", "f32").with_unit("Nm"),
            // --- Limits ---
            FieldDescriptor::new("velocity_limit", "Velocity limit", "f32").with_unit("rev/s"),
            FieldDescriptor::new("current_limit", "Current limit", "f32").with_unit("A"),
            // --- Trajectory planner ---
            FieldDescriptor::new("traj_vel_limit", "Trajectory velocity limit", "f32").with_unit("rev/s"),
            FieldDescriptor::new("traj_accel_limit", "Trajectory acceleration limit", "f32").with_unit("rev/s²"),
            FieldDescriptor::new("traj_decel_limit", "Trajectory deceleration limit", "f32").with_unit("rev/s²"),
            FieldDescriptor::new("traj_inertia", "Trajectory inertia", "f32").with_unit("Nm/(rev/s²)"),
            // --- Gains ---
            FieldDescriptor::new("pos_gain", "Position P gain", "f32"),
            FieldDescriptor::new("vel_gain", "Velocity P gain", "f32"),
            FieldDescriptor::new("vel_integrator_gain", "Velocity integrator gain", "f32"),
            // --- Actions ---
            FieldDescriptor::new("clear_errors", "Set true to clear all errors", "bool"),
            FieldDescriptor::new("reboot", "Set true to reboot the ODrive", "bool"),
        ]
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        let s = self.state.read().unwrap();
        Ok(serde_json::json!({
            "node_id":           self.node_id,
            "axis_error":        s.axis_error,
            "axis_state":        s.axis_state,
            "procedure_result":  s.procedure_result,
            "trajectory_done":   s.trajectory_done,
            "pos_estimate":      s.pos_estimate,
            "vel_estimate":      s.vel_estimate,
            "shadow_count":      s.shadow_count,
            "count_cpr":         s.count_cpr,
            "iq_setpoint":       s.iq_setpoint,
            "iq_measured":       s.iq_measured,
            "bus_voltage":       s.bus_voltage,
            "bus_current":       s.bus_current,
            "active_errors":     s.active_errors,
            "disarm_reason":     s.disarm_reason,
            "torque_target":     s.torque_target,
            "torque_estimate":   s.torque_estimate,
            "electrical_power":  s.electrical_power,
            "mechanical_power":  s.mechanical_power,
            "fet_temp":          s.fet_temp,
            "motor_temp":        s.motor_temp,
            "timestamp_ns":      s.timestamp_ns,
        }))
    }

    /// Execute a command. Dispatches sub-commands from the JSON payload.
    ///
    /// Primary path for streaming: `{ "axis_state": u32?, "input_pos": f32,
    /// "input_vel_ff": f32, "input_torque_ff": f32 }`.
    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        // Reset the watchdog timer — prevents idle frames from interleaving with this command.
        self.command_notify.notify_one();

        let mut sent = Vec::<&str>::new();

        // --- Actions ---
        if payload.get("clear_errors").and_then(Value::as_bool).unwrap_or(false) {
            // ODrive expects 1 byte (identify flag = 0). An empty frame is not the same DLC.
            self.send_blocking(CMD_CLEAR_ERRORS, vec![0x00])?;
            sent.push("clear_errors");
        }
        if payload.get("reboot").and_then(Value::as_bool).unwrap_or(false) {
            self.send_blocking(CMD_REBOOT, vec![])?;
            sent.push("reboot");
            return Ok(serde_json::json!({ "sent": sent }));
        }

        // --- Axis state ---
        if let Some(state) = payload.get("axis_state").and_then(Value::as_u64) {
            let data = encode_set_axis_state(state as u32).to_vec();
            self.send_blocking(CMD_SET_AXIS_STATE, data)?;
            sent.push("axis_state");
        }

        // --- Controller mode ---
        let has_control_mode = payload.get("control_mode").is_some();
        let has_input_mode = payload.get("input_mode").is_some();
        if has_control_mode || has_input_mode {
            let ctrl = payload.get("control_mode").and_then(Value::as_u64).unwrap_or(3) as u32;
            let inp = payload.get("input_mode").and_then(Value::as_u64).unwrap_or(1) as u32;
            let data = encode_set_controller_mode(ctrl, inp).to_vec();
            self.send_blocking(CMD_SET_CONTROLLER_MODE, data)?;
            sent.push("controller_mode");
        }

        // --- Setpoint: choose input_pos / input_vel / input_torque (exclusive) ---
        if payload.get("input_torque").is_some() {
            let torque = payload["input_torque"].as_f64().unwrap_or(0.0) as f32;
            let data = encode_set_input_torque(torque).to_vec();
            self.send_blocking(CMD_SET_INPUT_TORQUE, data)?;
            sent.push("input_torque");
        } else if payload.get("input_vel").is_some() {
            let vel = payload["input_vel"].as_f64().unwrap_or(0.0) as f32;
            let torque_ff = payload.get("input_torque_ff").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let data = encode_set_input_vel(vel, torque_ff).to_vec();
            self.send_blocking(CMD_SET_INPUT_VEL, data)?;
            sent.push("input_vel");
        } else if payload.get("input_pos").is_some()
            || payload.get("input_vel_ff").is_some()
            || payload.get("input_torque_ff").is_some()
        {
            let pos = payload.get("input_pos").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let vel_ff = payload.get("input_vel_ff").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let torque_ff = payload.get("input_torque_ff").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let data = encode_set_input_pos(pos, vel_ff, torque_ff).to_vec();
            self.send_blocking(CMD_SET_INPUT_POS, data)?;
            sent.push("input_pos");
        }

        // --- Limits ---
        if payload.get("velocity_limit").is_some() || payload.get("current_limit").is_some() {
            let vel = payload.get("velocity_limit").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let cur = payload.get("current_limit").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let data = encode_set_limits(vel, cur).to_vec();
            self.send_blocking(CMD_SET_LIMITS, data)?;
            sent.push("limits");
        }

        // --- Trajectory planner ---
        if let Some(v) = payload.get("traj_vel_limit").and_then(Value::as_f64) {
            let data = encode_set_traj_vel_limit(v as f32).to_vec();
            self.send_blocking(CMD_SET_TRAJ_VEL_LIMIT, data)?;
            sent.push("traj_vel_limit");
        }
        if payload.get("traj_accel_limit").is_some() || payload.get("traj_decel_limit").is_some() {
            let accel = payload.get("traj_accel_limit").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let decel = payload.get("traj_decel_limit").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let data = encode_set_traj_accel_limits(accel, decel).to_vec();
            self.send_blocking(CMD_SET_TRAJ_ACCEL_LIMITS, data)?;
            sent.push("traj_accel_limits");
        }
        if let Some(v) = payload.get("traj_inertia").and_then(Value::as_f64) {
            let data = encode_set_traj_inertia(v as f32).to_vec();
            self.send_blocking(CMD_SET_TRAJ_INERTIA, data)?;
            sent.push("traj_inertia");
        }

        // --- Gains ---
        if let Some(v) = payload.get("pos_gain").and_then(Value::as_f64) {
            let data = encode_set_position_gain(v as f32).to_vec();
            self.send_blocking(CMD_SET_POSITION_GAIN, data)?;
            sent.push("pos_gain");
        }
        if payload.get("vel_gain").is_some() || payload.get("vel_integrator_gain").is_some() {
            let vg = payload.get("vel_gain").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let vig = payload.get("vel_integrator_gain").and_then(Value::as_f64).unwrap_or(0.0) as f32;
            let data = encode_set_vel_gains(vg, vig).to_vec();
            self.send_blocking(CMD_SET_VEL_GAINS, data)?;
            sent.push("vel_gains");
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
        self.command_notify.notify_one();
        self.send_blocking(CMD_ESTOP, vec![])?;
        tracing::warn!(node_id = self.node_id, "ODrive ESTOP sent");
        Ok(serde_json::json!({ "estop": "sent", "node_id": self.node_id }))
    }

    fn has_config(&self) -> bool {
        true
    }

    /// Read all config-namespace parameters via SDO, driven by the loaded endpoint map.
    ///
    /// Covers every endpoint whose path starts with `axis0.config.`, `axis0.controller.config.`,
    /// `inc_encoder0.config.`, or `config.`, has a readable access mode, and fits in 4 bytes.
    /// Requires `flat_endpoints.json` — set `ODRIVE_HW_VERSION` + `ODRIVE_FW_VERSION` or
    /// upload via `POST /odrive/endpoints`.
    fn read_config(&self) -> Result<Value, DriverError> {
        // Collect all matching endpoints under the lock, then drop before CAN I/O.
        let to_read: Vec<(String, u16, String)> = {
            let map = self.endpoint_map.read().unwrap();
            if map.is_empty() {
                return Err(DriverError::CommandFailed(
                    "endpoint map not loaded. \
                     Set ODRIVE_HW_VERSION (e.g. 4.4.58) and ODRIVE_FW_VERSION (e.g. latest) \
                     to auto-fetch, or upload flat_endpoints.json via POST /odrive/endpoints.".into(),
                ));
            }
            map.iter()
                .filter(|(path, info)| {
                    (path.starts_with("axis0.config.")
                        || path.starts_with("axis0.controller.config.")
                        || path.starts_with("inc_encoder0.config.")
                        || path.starts_with("config."))
                        && info.access.contains('r')
                        && !matches!(
                            info.dtype.as_str(),
                            "function" | "uint64" | "int64" | "endpoint_ref"
                        )
                })
                .map(|(path, info)| (path.clone(), info.id, info.dtype.clone()))
                .collect()
        };

        const SDO_TIMEOUT: Duration = Duration::from_millis(200);
        let mut result = serde_json::Map::new();
        result.insert("node_id".into(), serde_json::json!(self.node_id));

        for (path, ep_id, dtype) in &to_read {
            match self.sdo_read_blocking(*ep_id, SDO_TIMEOUT) {
                Err(e) => {
                    result.insert(path.clone(), serde_json::json!({"error": e.to_string()}));
                }
                Ok(bytes) => {
                    result.insert(path.clone(), decode_sdo_value(&dtype, bytes));
                }
            }
        }

        Ok(Value::Object(result))
    }

    /// Write config parameters via SDO.
    ///
    /// Body is a JSON object keyed by the full flat-endpoint path:
    /// `{"axis0.controller.config.vel_limit": 20.0, "axis0.config.motor.pole_pairs": 7}`
    ///
    /// Any key present in the loaded endpoint map with write access is accepted.
    fn write_config(&self, config: &Value) -> Result<Value, DriverError> {
        let obj = config.as_object().ok_or_else(|| {
            DriverError::CommandFailed("body must be a JSON object".into())
        })?;

        // Resolve endpoint IDs and types under the lock, then drop before CAN I/O.
        let writes: Vec<(String, u16, String, Value)> = {
            let map = self.endpoint_map.read().unwrap();
            if map.is_empty() {
                return Err(DriverError::CommandFailed(
                    "endpoint map not loaded — upload flat_endpoints.json via POST /odrive/endpoints".into(),
                ));
            }
            obj.iter().filter_map(|(key, val)| {
                let info = map.get(key)?;
                if !info.access.contains('w') { return None; }
                if matches!(info.dtype.as_str(), "function" | "uint64" | "int64" | "endpoint_ref") {
                    return None;
                }
                Some((key.clone(), info.id, info.dtype.clone(), val.clone()))
            }).collect()
        };

        let mut written = Vec::<String>::new();
        let mut errors = serde_json::Map::new();

        for (key, ep_id, dtype, val) in &writes {
            match encode_sdo_value(*ep_id, dtype, val) {
                None => {
                    errors.insert(key.clone(), serde_json::json!(
                        format!("cannot encode '{val}' for type '{dtype}'")
                    ));
                }
                Some(payload) => match self.sdo_write_blocking(*ep_id, payload) {
                    Ok(_) => written.push(key.clone()),
                    Err(e) => { errors.insert(key.clone(), serde_json::json!(e.to_string())); }
                },
            }
        }

        Ok(serde_json::json!({ "written": written, "errors": errors }))
    }

    fn has_endpoint_write(&self) -> bool {
        true
    }

    /// List all endpoints in the loaded map (no CAN I/O).
    fn list_endpoints(&self) -> Result<Value, DriverError> {
        let map = self.endpoint_map.read().unwrap();
        if map.is_empty() {
            return Err(DriverError::CommandFailed(
                "endpoint map not loaded — set ODRIVE_HW_VERSION + ODRIVE_FW_VERSION or upload via POST /odrive/endpoints".into(),
            ));
        }
        let endpoints: serde_json::Map<String, Value> = map.iter()
            .filter(|(_, info)| info.dtype != "function")
            .map(|(k, v)| (k.clone(), serde_json::json!({
                "id":     v.id,
                "type":   v.dtype,
                "access": v.access,
            })))
            .collect();
        Ok(Value::Object(endpoints))
    }

    /// Read a single endpoint by its flat-endpoint path.
    fn read_endpoint(&self, path: &str) -> Result<Value, DriverError> {
        let (ep_id, dtype) = {
            let map = self.endpoint_map.read().unwrap();
            let info = map.get(path).ok_or_else(|| {
                DriverError::CommandFailed(format!("endpoint '{path}' not found in map"))
            })?;
            if !info.access.contains('r') {
                return Err(DriverError::CommandFailed(
                    format!("endpoint '{path}' is not readable (access='{}')", info.access)
                ));
            }
            if matches!(info.dtype.as_str(), "function" | "uint64" | "int64" | "endpoint_ref") {
                return Err(DriverError::CommandFailed(
                    format!("endpoint '{path}' type '{}' not supported over 4-byte SDO", info.dtype)
                ));
            }
            (info.id, info.dtype.clone())
        };

        let bytes = self.sdo_read_blocking(ep_id, Duration::from_millis(200))?;
        Ok(serde_json::json!({
            "path":  path,
            "value": decode_sdo_value(&dtype, bytes),
            "type":  dtype,
        }))
    }

    /// Write a single endpoint. Body: `{"value": <number|bool>}`.
    fn write_endpoint(&self, path: &str, body: &Value) -> Result<Value, DriverError> {
        let val = body.get("value").ok_or_else(|| {
            DriverError::CommandFailed("body must contain a 'value' field".into())
        })?;

        let (ep_id, dtype) = {
            let map = self.endpoint_map.read().unwrap();
            let info = map.get(path).ok_or_else(|| {
                DriverError::CommandFailed(format!("endpoint '{path}' not found in map"))
            })?;
            if !info.access.contains('w') {
                return Err(DriverError::CommandFailed(
                    format!("endpoint '{path}' is not writable (access='{}')", info.access)
                ));
            }
            (info.id, info.dtype.clone())
        };

        let payload = encode_sdo_value(ep_id, &dtype, val).ok_or_else(|| {
            DriverError::CommandFailed(
                format!("cannot encode '{val}' for endpoint type '{dtype}'")
            )
        })?;

        self.sdo_write_blocking(ep_id, payload)?;
        Ok(serde_json::json!({ "path": path, "written": true }))
    }

    fn has_calibrate(&self) -> bool {
        true
    }

    /// Trigger a calibration sequence by setting the axis state.
    ///
    /// Body: `{"type": "full" | "motor" | "encoder_index" | "encoder_offset"}`
    /// Defaults to `"full"` if `type` is omitted.
    fn calibrate(&self, params: &Value) -> Result<Value, DriverError> {
        let cal_type = params
            .get("type")
            .and_then(Value::as_str)
            .unwrap_or("full");

        let (axis_state, label) = match cal_type {
            "full"           => (3u32, "FullCalibrationSequence"),
            "motor"          => (4u32, "MotorCalibration"),
            "encoder_index"  => (6u32, "EncoderIndexSearch"),
            "encoder_offset" => (7u32, "EncoderOffsetCalibration"),
            other => {
                return Err(DriverError::CommandFailed(format!(
                    "unknown calibration type '{other}'; use: full | motor | encoder_index | encoder_offset"
                )));
            }
        };

        // Clear errors first — ODrive won't transition to calibration if any errors are active.
        // The clear_errors frame requires exactly 1 byte (identify flag = 0).
        self.send_blocking(CMD_CLEAR_ERRORS, vec![0x00])?;

        let data = encode_set_axis_state(axis_state).to_vec();
        self.send_blocking(CMD_SET_AXIS_STATE, data)?;

        tracing::info!(
            node_id = self.node_id,
            cal_type,
            axis_state,
            "calibration sequence started"
        );

        Ok(serde_json::json!({
            "node_id":    self.node_id,
            "calibration": label,
            "axis_state": axis_state,
            "status":     "started",
        }))
    }
}
