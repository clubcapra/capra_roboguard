/// ODrive CANSimple protocol constants and frame encoders/decoders.
///
/// CAN ID layout: `(node_id << 5) | cmd_id`
///   - node_id: 6 bits (0–63)
///   - cmd_id:  5 bits (0–31)
///
/// All multi-byte fields are little-endian IEEE 754 (floats) or uint/int.

// ── Command IDs (ODrive fw ≥ 0.6 / ODrive Pro CANSimple) ─────────────────────
//
// Mapping changed significantly vs fw 0.5.x:
//   0x003 was Get_Motor_Error     → now Get_Error (active_errors + disarm_reason)
//   0x004 was Get_Encoder_Error   → now RxSdo (host→device, never broadcast)
//   0x015 was Get_Sensorless_Est  → now Get_Temperature (FET + motor, cyclic)
//   0x01C was Get_ADC_Voltage     → now Get_Torques (torque_target + torque_estimate, cyclic)
//   0x01D was Get_Controller_Error→ now Get_Powers (electrical + mechanical, cyclic)
//
// Cyclic message defaults — DISABLED unless configured:
//   heartbeat_msg_rate_ms  = 100  (only enabled by default)
//   encoder_msg_rate_ms    = 10   (only enabled by default)
//   iq_msg_rate_ms         = 0    → must set via SDO on startup
//   error_msg_rate_ms      = 0    → must set via SDO on startup
//   temperature_msg_rate_ms= 0    → must set via SDO on startup
//   bus_voltage_msg_rate_ms= 0    → must set via SDO on startup
//   torques_msg_rate_ms    = 0    → must set via SDO on startup

pub const CMD_HEARTBEAT: u32            = 0x001;
pub const CMD_ESTOP: u32                = 0x002;
/// Get_Error (fw≥0.6): [active_errors:u32, disarm_reason:u32]
pub const CMD_GET_ERROR: u32            = 0x003;
pub const CMD_SET_AXIS_STATE: u32       = 0x007;
pub const CMD_ENCODER_ESTIMATES: u32    = 0x009;
pub const CMD_ENCODER_COUNT: u32        = 0x00A;
pub const CMD_SET_CONTROLLER_MODE: u32  = 0x00B;
pub const CMD_SET_INPUT_POS: u32        = 0x00C;
pub const CMD_SET_INPUT_VEL: u32        = 0x00D;
pub const CMD_SET_INPUT_TORQUE: u32     = 0x00E;
pub const CMD_SET_LIMITS: u32           = 0x00F;
pub const CMD_START_ANTICOGGING: u32    = 0x010;
pub const CMD_SET_TRAJ_VEL_LIMIT: u32   = 0x011;
pub const CMD_SET_TRAJ_ACCEL_LIMITS: u32= 0x012;
pub const CMD_SET_TRAJ_INERTIA: u32     = 0x013;
pub const CMD_GET_IQ: u32               = 0x014;
/// Get_Temperature (fw≥0.6): [fet_temp:f32, motor_temp:f32] — cyclic, disabled by default
pub const CMD_GET_TEMPERATURE: u32      = 0x015;
pub const CMD_REBOOT: u32               = 0x016;
pub const CMD_GET_BUS_VOLTAGE_CURRENT: u32 = 0x017;
pub const CMD_CLEAR_ERRORS: u32         = 0x018;
pub const CMD_SET_POSITION_GAIN: u32    = 0x01A;
pub const CMD_SET_VEL_GAINS: u32        = 0x01B;
/// Get_Torques (fw≥0.6): [torque_target:f32, torque_estimate:f32] — cyclic, disabled by default
pub const CMD_GET_TORQUES: u32          = 0x01C;
/// Get_Powers (fw≥0.6): [electrical_power:f32, mechanical_power:f32] — cyclic, disabled by default
pub const CMD_GET_POWERS: u32           = 0x01D;

// ── SDO — endpoint read/write (flat_endpoints.json, ODrive Pro / fw ≥ 0.6) ──
pub const CMD_RXSDO: u32 = 0x004; // host → ODrive: read/write endpoint request
pub const CMD_TXSDO: u32 = 0x005; // ODrive → host: read/write endpoint response

pub const SDO_OPCODE_READ: u8 = 0x00;
pub const SDO_OPCODE_WRITE: u8 = 0x01;

// ── ID helpers ───────────────────────────────────────────────────────────────

/// Build a standard 11-bit CAN ID from ODrive node_id and command_id.
#[inline]
pub fn can_id(node_id: u8, cmd_id: u32) -> u32 {
    ((node_id as u32) << 5) | cmd_id
}

/// Split a raw CAN frame ID back into (node_id, cmd_id).
#[inline]
pub fn split_can_id(raw: u32) -> (u8, u32) {
    ((raw >> 5) as u8, raw & 0x1F)
}

// ── Axis / control enums ─────────────────────────────────────────────────────

#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum AxisState {
    Undefined = 0,
    Idle = 1,
    StartupSequence = 2,
    FullCalibrationSequence = 3,
    MotorCalibration = 4,
    EncoderIndexSearch = 6,
    EncoderOffsetCalibration = 7,
    ClosedLoopControl = 8,
    LockinSpin = 9,
    EncoderDirFind = 10,
    Homing = 11,
    EncoderHallPolarityCalibration = 12,
    EncoderHallPhaseCalibration = 13,
}

impl AxisState {
    pub fn from_u32(v: u32) -> Option<Self> {
        match v {
            0 => Some(Self::Undefined),
            1 => Some(Self::Idle),
            2 => Some(Self::StartupSequence),
            3 => Some(Self::FullCalibrationSequence),
            4 => Some(Self::MotorCalibration),
            6 => Some(Self::EncoderIndexSearch),
            7 => Some(Self::EncoderOffsetCalibration),
            8 => Some(Self::ClosedLoopControl),
            9 => Some(Self::LockinSpin),
            10 => Some(Self::EncoderDirFind),
            11 => Some(Self::Homing),
            12 => Some(Self::EncoderHallPolarityCalibration),
            13 => Some(Self::EncoderHallPhaseCalibration),
            _ => None,
        }
    }
}

#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ControlMode {
    VoltageControl = 0,
    TorqueControl = 1,
    VelocityControl = 2,
    PositionControl = 3,
}

#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum InputMode {
    Inactive = 0,
    Passthrough = 1,
    VelRamp = 2,
    PosFilter = 3,
    MixChannels = 4,
    TrapTraj = 5,
    TorqueRamp = 6,
    Mirror = 7,
    Tuning = 8,
}

// ── Frame encoders ───────────────────────────────────────────────────────────

/// E-Stop: no payload, sends CMD_ESTOP to the node.
pub fn encode_estop() -> [u8; 0] {
    []
}

/// Set Axis State (0x007): [axis_state: u32 LE]
pub fn encode_set_axis_state(state: u32) -> [u8; 4] {
    state.to_le_bytes()
}

/// Set Controller Mode (0x00B): [control_mode: u32 LE, input_mode: u32 LE]
pub fn encode_set_controller_mode(control_mode: u32, input_mode: u32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0..4].copy_from_slice(&control_mode.to_le_bytes());
    buf[4..8].copy_from_slice(&input_mode.to_le_bytes());
    buf
}

/// Set Input Pos (0x00C): [input_pos: f32, vel_ff: i16 (×0.001 rev/s), torque_ff: i16 (×0.001 Nm)]
pub fn encode_set_input_pos(pos: f32, vel_ff: f32, torque_ff: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0..4].copy_from_slice(&pos.to_le_bytes());
    // vel_ff and torque_ff are scaled int16 (unit = 0.001)
    let vel_ff_i = (vel_ff * 1000.0).round() as i16;
    let torque_ff_i = (torque_ff * 1000.0).round() as i16;
    buf[4..6].copy_from_slice(&vel_ff_i.to_le_bytes());
    buf[6..8].copy_from_slice(&torque_ff_i.to_le_bytes());
    buf
}

/// Set Input Vel (0x00D): [input_vel: f32, torque_ff: f32]
pub fn encode_set_input_vel(vel: f32, torque_ff: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0..4].copy_from_slice(&vel.to_le_bytes());
    buf[4..8].copy_from_slice(&torque_ff.to_le_bytes());
    buf
}

/// Set Input Torque (0x00E): [input_torque: f32]
pub fn encode_set_input_torque(torque: f32) -> [u8; 4] {
    torque.to_le_bytes()
}

/// Set Limits (0x00F): [velocity_limit: f32, current_limit: f32]
pub fn encode_set_limits(velocity_limit: f32, current_limit: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0..4].copy_from_slice(&velocity_limit.to_le_bytes());
    buf[4..8].copy_from_slice(&current_limit.to_le_bytes());
    buf
}

/// Set Traj Vel Limit (0x011): [traj_vel_limit: f32]
pub fn encode_set_traj_vel_limit(limit: f32) -> [u8; 4] {
    limit.to_le_bytes()
}

/// Set Traj Accel Limits (0x012): [traj_accel_limit: f32, traj_decel_limit: f32]
pub fn encode_set_traj_accel_limits(accel: f32, decel: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0..4].copy_from_slice(&accel.to_le_bytes());
    buf[4..8].copy_from_slice(&decel.to_le_bytes());
    buf
}

/// Set Traj Inertia (0x013): [inertia: f32]
pub fn encode_set_traj_inertia(inertia: f32) -> [u8; 4] {
    inertia.to_le_bytes()
}

/// Set Position Gain (0x01A): [pos_gain: f32]
pub fn encode_set_position_gain(pos_gain: f32) -> [u8; 4] {
    pos_gain.to_le_bytes()
}

/// Set Vel Gains (0x01B): [vel_gain: f32, vel_integrator_gain: f32]
pub fn encode_set_vel_gains(vel_gain: f32, vel_integrator_gain: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0..4].copy_from_slice(&vel_gain.to_le_bytes());
    buf[4..8].copy_from_slice(&vel_integrator_gain.to_le_bytes());
    buf
}

// ── Frame decoders ───────────────────────────────────────────────────────────

/// Heartbeat (0x001) — fw≥0.6 format:
/// [axis_error:u32, axis_state:u8, procedure_result:u8, trajectory_done_flag:u8, _pad:u8]
pub struct HeartbeatFrame {
    pub axis_error: u32,
    pub axis_state: u8,
    /// Procedure_Result (replaces motor_flags from fw 0.5.x).
    pub procedure_result: u8,
    /// Trajectory_Done_Flag packed into byte 6 (LSB).
    pub trajectory_done: bool,
}

pub fn decode_heartbeat(data: &[u8]) -> Option<HeartbeatFrame> {
    if data.len() < 6 {
        return None;
    }
    Some(HeartbeatFrame {
        axis_error:       u32::from_le_bytes(data[0..4].try_into().ok()?),
        axis_state:       data[4],
        procedure_result: data[5],
        trajectory_done:  data.get(6).copied().unwrap_or(0) & 0x01 != 0,
    })
}

/// Encoder Estimates (0x009): [pos_estimate: f32, vel_estimate: f32]
pub struct EncoderEstimatesFrame {
    pub pos_estimate: f32,
    pub vel_estimate: f32,
}

pub fn decode_encoder_estimates(data: &[u8]) -> Option<EncoderEstimatesFrame> {
    if data.len() < 8 {
        return None;
    }
    Some(EncoderEstimatesFrame {
        pos_estimate: f32::from_le_bytes(data[0..4].try_into().ok()?),
        vel_estimate: f32::from_le_bytes(data[4..8].try_into().ok()?),
    })
}

/// Encoder Count (0x00A): [shadow_count: i32, count_cpr: i32]
pub struct EncoderCountFrame {
    pub shadow_count: i32,
    pub count_cpr: i32,
}

pub fn decode_encoder_count(data: &[u8]) -> Option<EncoderCountFrame> {
    if data.len() < 8 {
        return None;
    }
    Some(EncoderCountFrame {
        shadow_count: i32::from_le_bytes(data[0..4].try_into().ok()?),
        count_cpr: i32::from_le_bytes(data[4..8].try_into().ok()?),
    })
}

/// Get Iq (0x014): [iq_setpoint: f32, iq_measured: f32]
pub struct IqFrame {
    pub iq_setpoint: f32,
    pub iq_measured: f32,
}

pub fn decode_iq(data: &[u8]) -> Option<IqFrame> {
    if data.len() < 8 {
        return None;
    }
    Some(IqFrame {
        iq_setpoint: f32::from_le_bytes(data[0..4].try_into().ok()?),
        iq_measured: f32::from_le_bytes(data[4..8].try_into().ok()?),
    })
}

/// Bus Voltage/Current (0x017): [bus_voltage: f32, bus_current: f32]
pub struct BusVIFrame {
    pub bus_voltage: f32,
    pub bus_current: f32,
}

pub fn decode_bus_vi(data: &[u8]) -> Option<BusVIFrame> {
    if data.len() < 8 {
        return None;
    }
    Some(BusVIFrame {
        bus_voltage: f32::from_le_bytes(data[0..4].try_into().ok()?),
        bus_current: f32::from_le_bytes(data[4..8].try_into().ok()?),
    })
}

/// Get_Error (0x003, fw≥0.6): [active_errors:u32, disarm_reason:u32]
/// cyclic, disabled by default (error_msg_rate_ms = 0).
pub struct GetErrorFrame {
    pub active_errors: u32,
    pub disarm_reason: u32,
}

pub fn decode_get_error(data: &[u8]) -> Option<GetErrorFrame> {
    if data.len() < 8 {
        return None;
    }
    Some(GetErrorFrame {
        active_errors: u32::from_le_bytes(data[0..4].try_into().ok()?),
        disarm_reason: u32::from_le_bytes(data[4..8].try_into().ok()?),
    })
}

/// Get_Temperature (0x015, fw≥0.6): [fet_temp:f32, motor_temp:f32]
/// cyclic, disabled by default (temperature_msg_rate_ms = 0).
pub struct TemperatureFrame {
    pub fet_temp:   f32,
    pub motor_temp: f32,
}

pub fn decode_temperature(data: &[u8]) -> Option<TemperatureFrame> {
    if data.len() < 8 {
        return None;
    }
    Some(TemperatureFrame {
        fet_temp:   f32::from_le_bytes(data[0..4].try_into().ok()?),
        motor_temp: f32::from_le_bytes(data[4..8].try_into().ok()?),
    })
}

/// Get_Torques (0x01C, fw≥0.6): [torque_target:f32, torque_estimate:f32]
/// cyclic, disabled by default (torques_msg_rate_ms = 0).
pub struct TorquesFrame {
    pub torque_target:   f32,
    pub torque_estimate: f32,
}

pub fn decode_torques(data: &[u8]) -> Option<TorquesFrame> {
    if data.len() < 8 {
        return None;
    }
    Some(TorquesFrame {
        torque_target:   f32::from_le_bytes(data[0..4].try_into().ok()?),
        torque_estimate: f32::from_le_bytes(data[4..8].try_into().ok()?),
    })
}

/// Get_Powers (0x01D, fw≥0.6): [electrical_power:f32, mechanical_power:f32]
/// cyclic, disabled by default (torques_msg_rate_ms = 0).
pub struct PowersFrame {
    pub electrical_power: f32,
    pub mechanical_power: f32,
}

pub fn decode_powers(data: &[u8]) -> Option<PowersFrame> {
    if data.len() < 8 {
        return None;
    }
    Some(PowersFrame {
        electrical_power: f32::from_le_bytes(data[0..4].try_into().ok()?),
        mechanical_power: f32::from_le_bytes(data[4..8].try_into().ok()?),
    })
}

// ── SDO frame encoders/decoders ──────────────────────────────────────────────

/// RxSdo read request (send with CMD_RXSDO): [opcode=0x00, ep_lo, ep_hi, 0x00]
pub fn encode_sdo_read(endpoint_id: u16) -> [u8; 4] {
    [SDO_OPCODE_READ, endpoint_id as u8, (endpoint_id >> 8) as u8, 0x00]
}

/// RxSdo write request for f32 (send with CMD_RXSDO): [0x01, ep_lo, ep_hi, 0x00, b0..b3]
pub fn encode_sdo_write_f32(endpoint_id: u16, val: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = SDO_OPCODE_WRITE;
    buf[1] = endpoint_id as u8;
    buf[2] = (endpoint_id >> 8) as u8;
    buf[3] = 0x00;
    buf[4..8].copy_from_slice(&val.to_le_bytes());
    buf
}

/// RxSdo write request for i32.
pub fn encode_sdo_write_i32(endpoint_id: u16, val: i32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = SDO_OPCODE_WRITE;
    buf[1] = endpoint_id as u8;
    buf[2] = (endpoint_id >> 8) as u8;
    buf[3] = 0x00;
    buf[4..8].copy_from_slice(&val.to_le_bytes());
    buf
}

/// RxSdo write request for bool (encoded as u32).
pub fn encode_sdo_write_bool(endpoint_id: u16, val: bool) -> [u8; 8] {
    encode_sdo_write_i32(endpoint_id, if val { 1 } else { 0 })
}

/// Decode a TxSdo response frame: returns `(endpoint_id, raw_4_bytes)`.
///
/// ODrive sends variable-length frames depending on the endpoint type:
/// - bool / uint8  → 5 bytes (header 4 + 1 byte value)
/// - uint16        → 6 bytes (header 4 + 2 byte value)
/// - float / uint32 / int32 → 8 bytes (header 4 + 4 byte value)
///
/// We zero-pad smaller payloads so callers always get a uniform 4-byte buffer.
pub fn decode_sdo_response(data: &[u8]) -> Option<(u16, [u8; 4])> {
    if data.len() < 5 {
        return None; // need at least 4-byte header + 1 byte value
    }
    let endpoint_id = u16::from_le_bytes([data[1], data[2]]);
    let mut value_bytes = [0u8; 4];
    let value_len = (data.len() - 4).min(4);
    value_bytes[..value_len].copy_from_slice(&data[4..4 + value_len]);
    Some((endpoint_id, value_bytes))
}
