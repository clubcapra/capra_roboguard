//! Cached telemetry snapshot for the Kinova arm.
//!
//! Updated by the worker thread after each round of `Get*` SDK calls; read
//! from any thread by `KinovaArm::read_data` via `Arc<RwLock<KinovaState>>`.

#[derive(Debug, Clone, Default)]
pub struct KinovaState {
    /// Joint angle (degrees), index 0..5 == joint 1..6.
    pub joint_pos: [f32; 6],
    /// Joint velocity (deg/s).
    pub joint_vel: [f32; 6],
    /// Joint torque / force (Nm — units depend on torque sensor calibration).
    pub joint_torque: [f32; 6],
    /// Joint motor current (A).
    pub joint_current: [f32; 6],
    /// Per-actuator temperature (°C).
    pub joint_temp: [f32; 6],

    /// Main 24 V supply voltage.
    pub bus_voltage: f32,
    /// Total bus current draw (A).
    pub bus_current: f32,

    /// Base IMU acceleration (G), only populated if the arm has a base IMU.
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,

    /// `QuickStatus.ControlEnableStatus` — true if the arm is accepting
    /// control commands (note SDK semantics: 0 = ON in the raw struct, we
    /// invert here for clarity).
    pub control_enabled: bool,
    /// `QuickStatus.RetractType`.
    pub retract_state: u8,
    /// `QuickStatus.RobotType` (0 = JACO, 1 = MICO, etc).
    pub robot_type: u8,
    /// `QuickStatus.TorqueSensorsStatus`.
    pub torque_sensors_available: bool,

    /// Last estop state we set ourselves (driver-side flag — true after the
    /// caller invoked `estop()`, cleared when control is restarted).
    pub estopped: bool,

    /// Wall-clock timestamp (ns) of the last successful telemetry refresh.
    /// `0` until the first refresh completes.
    pub timestamp_ns: i64,
}
