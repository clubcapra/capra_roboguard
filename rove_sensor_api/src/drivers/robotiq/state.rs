//! Cached telemetry snapshot for the Robotiq 2F-140 gripper.
//!
//! Populated by the worker each time it reads holding registers
//! 0x07D0..0x07D2 (status). Read by `RobotiqGripper::read_data` via
//! `Arc<RwLock<RobotiqState>>`.

#[derive(Debug, Clone, Default)]
pub struct RobotiqState {
    /// gACT — 0 = reset, 1 = activated.
    pub activated: bool,
    /// gGTO — 0 = stopped, 1 = going to position.
    pub going_to_position: bool,
    /// gSTA — 0 = reset, 1 = activating, 3 = activation complete.
    pub status: u8,
    /// gOBJ — 0 = moving, 1 = object detected while opening, 2 = object
    /// detected while closing, 3 = at requested position / no object.
    pub object_status: u8,
    /// gFLT — 0x00 = no fault, 0x05–0x0F = various fault codes.
    pub fault: u8,
    /// gPR — echo of the last commanded position (0..255).
    pub position_request_echo: u8,
    /// gPO — actual jaw position (0 = fully open, 255 = fully closed).
    pub position: u8,
    /// gCU — motor current, ≈ value × 10 mA.
    pub current_raw: u8,

    /// Wall-clock timestamp (ns) of the last successful status read; 0 until
    /// the first poll lands.
    pub timestamp_ns: i64,
    /// Set after `connect()` completes successfully — tells consumers the
    /// channel is live even before the first poll completes.
    pub link_up: bool,
}

impl RobotiqState {
    /// Convenience: motor current in amps (gCU * 10 mA).
    pub fn current_a(&self) -> f32 {
        self.current_raw as f32 * 0.01
    }
}
