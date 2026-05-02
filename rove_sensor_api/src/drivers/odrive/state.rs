/// Cached telemetry for one ODrive node, updated by the CAN receive loop.
///
/// All fields mirror ODrive CANSimple frame payloads (fw ≥ 0.6 / ODrive Pro).
/// The receive loop writes these; `read_data()` reads them.
#[derive(Debug, Clone, Default)]
pub struct OdriveNodeState {
    // --- Heartbeat (0x001, always cyclic @ 100ms) ---
    pub axis_error: u32,
    /// Raw axis state — 8 = ClosedLoopControl.
    pub axis_state: u8,
    /// Procedure_Result field (fw≥0.6; replaces motor_flags from 0.5.x).
    pub procedure_result: u8,
    pub trajectory_done: bool,

    // --- Encoder estimates (0x009, cyclic @ 10ms) ---
    pub pos_estimate: f32,
    pub vel_estimate: f32,

    // --- Encoder count (0x00A, cyclic, optional) ---
    pub shadow_count: i32,
    pub count_cpr: i32,

    // --- IQ (0x014, cyclic — enabled by CAN init task) ---
    pub iq_setpoint: f32,
    pub iq_measured: f32,

    // --- Bus voltage / current (0x017, cyclic — enabled by CAN init task) ---
    pub bus_voltage: f32,
    pub bus_current: f32,

    // --- Get_Error (0x003, cyclic — enabled by CAN init task) ---
    /// Active error flags (axis-level, matches AXIS_ERRORS bitmask).
    pub active_errors: u32,
    pub disarm_reason: u32,

    // --- Get_Temperature (0x015, cyclic — enabled by CAN init task) ---
    /// FET thermistor temperature in °C. `None` until first broadcast received.
    pub fet_temp: Option<f32>,
    /// Motor thermistor temperature in °C. `None` until first broadcast received.
    pub motor_temp: Option<f32>,

    // --- Get_Torques (0x01C, cyclic — enabled by CAN init task) ---
    pub torque_target: f32,
    pub torque_estimate: f32,

    // --- Get_Powers (0x01D, cyclic — enabled by CAN init task) ---
    pub electrical_power: f32,
    pub mechanical_power: f32,

    // --- Receive timestamp ---
    /// Unix timestamp in nanoseconds of the last CAN frame received from this node.
    /// 0 until the first frame arrives. Used to detect stale or out-of-order data.
    pub timestamp_ns: i64,
}
