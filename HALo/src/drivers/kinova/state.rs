#[derive(Debug, Clone, Default)]
pub struct KinovaState {
    pub joint_pos: [f32; 6],
    pub joint_vel: [f32; 6],     // deg/s — from GetAngularVelocity
    pub joint_current: [f32; 6], // A   — from GetAngularCurrent
    pub joint_temp: [f32; 6],    // °C  — from GetGeneralInformations.ActuatorsTemperatures
    pub bus_voltage: f32,        // V   — from GetGeneralInformations.SupplyVoltage
    pub bus_current: f32,        // A   — from GetGeneralInformations.TotalCurrent
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub timestamp_ns: i64,
    pub errors: Vec<String>,
    pub limp_supported: bool,
    pub limp_mode: bool,
    pub motion_inhibited: bool,
    pub limp_transition: bool,
    pub control_error: Option<String>,
    pub command_error: Option<String>,
    pub velocity_error: Option<String>,
    pub keepalive_error: Option<String>,
}
