#![allow(non_snake_case)]

use std::os::raw::{c_int, c_uchar, c_ulong, c_ushort};

pub const MAX_KINOVA_DEVICE: usize = 20;
pub const SERIAL_LENGTH: usize = 20;

pub const POSITION_TYPE_ANGULAR_POSITION: c_int = 2;
pub const POSITION_TYPE_ANGULAR_VELOCITY: c_int = 8;
pub const HAND_MODE_POSITION_MODE: c_int = 1; // InitStruct() default — matches C++ wrapper

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct AngularInfo {
    pub Actuator1: f32,
    pub Actuator2: f32,
    pub Actuator3: f32,
    pub Actuator4: f32,
    pub Actuator5: f32,
    pub Actuator6: f32,
    pub Actuator7: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct CartesianInfo {
    pub X: f32,
    pub Y: f32,
    pub Z: f32,
    pub ThetaX: f32,
    pub ThetaY: f32,
    pub ThetaZ: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct FingersPosition {
    pub Finger1: f32,
    pub Finger2: f32,
    pub Finger3: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct AngularPosition {
    pub Actuators: AngularInfo,
    pub Fingers: FingersPosition,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Limitation {
    pub speedParameter1: f32,
    pub speedParameter2: f32,
    pub speedParameter3: f32,
    pub forceParameter1: f32,
    pub forceParameter2: f32,
    pub forceParameter3: f32,
    pub accelerationParameter1: f32,
    pub accelerationParameter2: f32,
    pub accelerationParameter3: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct UserPosition {
    pub Type: c_int,
    pub Delay: f32,
    pub CartesianPosition: CartesianInfo,
    pub Actuators: AngularInfo,
    pub HandMode: c_int,
    pub Fingers: FingersPosition,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct TrajectoryPoint {
    pub Position: UserPosition,
    pub LimitationsActive: c_int,
    pub SynchroType: c_int,
    pub Limitations: Limitation,
}

// --- GeneralInformations and its dependencies ---
// Mirrors KinovaTypes.h exactly.  All sizes verified against the header.

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct SystemStatus {
    pub JoystickActive: u32,
    pub RetractStatus: u32,
    pub DrinkingMode: u32,
    pub ArmLaterality: u32,
    pub TranslationActive: u32,
    pub RotationActive: u32,
    pub FingersActive: u32,
    pub WarningOverchargeForce: u32,
    pub WarningOverchargeFingers: u32,
    pub WarningLowVoltage: u32,
    pub MajorErrorOccured: u32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct JoystickCommand {
    pub ButtonValue: [i16; 16],
    pub InclineLeftRight: f32,
    pub InclineForwardBackward: f32,
    pub Rotate: f32,
    pub MoveLeftRight: f32,
    pub MoveForwardBackward: f32,
    pub PushPull: f32,
}

// MAXACTUATORNUMBER = 7 in KinovaTypes.h
#[repr(C)]
pub struct GeneralInformations {
    pub TimeAbsolute: f64,
    pub TimeFromStartup: f64,
    pub IndexStartup: u32,
    pub ExpansionLong1: i32,
    pub TimeStampSavings: f32,
    pub ExpansionFloat: f32,
    pub SupplyVoltage: f32,
    pub TotalCurrent: f32,
    pub Power: f32,
    pub AveragePower: f32,
    pub AccelerationX: f32,
    pub AccelerationY: f32,
    pub AccelerationZ: f32,
    pub SensorExpansion1: f32,
    pub SensorExpansion2: f32,
    pub SensorExpansion3: f32,
    pub CodeVersion: u32,
    pub CodeRevision: u32,
    pub Status: u16,
    pub Controller: u16,
    pub ControlMode: u16,
    pub HandMode: u16,
    pub ConnectedActuatorCount: u16,
    pub PositionType: u16,
    pub ErrorsSpiExpansion1: u16,
    pub ErrorsSpiExpansion2: u16,
    pub ErrorsMainSPICount: u16,
    pub ErrorsExternalSPICount: u16,
    pub ErrorsMainCANCount: u16,
    pub ErrorsExternalCANCount: u16,
    pub ActualSystemStatus: SystemStatus,   // 11 × u32 = 44 bytes
    pub Position: UserPosition,              // 76 bytes
    pub Command: UserPosition,
    pub Current: UserPosition,
    pub Force: UserPosition,
    pub ActualLimitations: Limitation,       // 9 × f32 = 36 bytes
    pub ControlIncrement: [f32; 7],
    pub FingerControlIncrement: [f32; 3],
    pub ActualJoystickCommand: JoystickCommand, // 16×i16 + 6×f32 = 56 bytes
    pub PeripheralsConnected: [u32; 4],
    pub PeripheralsDeviceID: [u32; 4],
    pub ActuatorsTemperatures: [f32; 7],    // per-actuator °C
    pub FingersTemperatures: [f32; 3],
    pub FutureTemperatures: [f32; 3],
    pub ActuatorsCommErrors: [i32; 7],
    pub FingersCommErrors: [i32; 3],
    pub ExpansionLong2: i32,
    pub ControlTimeAbsolute: f64,
    pub ControlTimeFromStartup: f64,
    pub ExpansionsBytes: [u8; 192],
}

impl Default for GeneralInformations {
    fn default() -> Self {
        // SAFETY: GeneralInformations is a plain-old-data C struct; zeroing is valid.
        unsafe { std::mem::zeroed() }
    }
}

// --- Remaining existing structs ---

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct SensorsInfo {
    pub Voltage: f32,
    pub Current: f32,
    pub AccelerationX: f32,
    pub AccelerationY: f32,
    pub AccelerationZ: f32,
    pub ActuatorTemp1: f32,
    pub ActuatorTemp2: f32,
    pub ActuatorTemp3: f32,
    pub ActuatorTemp4: f32,
    pub ActuatorTemp5: f32,
    pub ActuatorTemp6: f32,
    pub ActuatorTemp7: f32,
    pub FingerTemp1: f32,
    pub FingerTemp2: f32,
    pub FingerTemp3: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct KinovaDevice {
    pub SerialNumber: [u8; SERIAL_LENGTH],
    pub Model: [u8; SERIAL_LENGTH],
    pub VersionMajor: c_int,
    pub VersionMinor: c_int,
    pub VersionRelease: c_int,
    pub DeviceType: c_int,
    pub DeviceID: c_int,
}

impl Default for KinovaDevice {
    fn default() -> Self {
        Self {
            SerialNumber: [0; SERIAL_LENGTH],
            Model: [0; SERIAL_LENGTH],
            VersionMajor: 0,
            VersionMinor: 0,
            VersionRelease: 0,
            DeviceType: 0,
            DeviceID: 0,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct EthernetCommConfig {
    pub localIpAddress: c_ulong,
    pub subnetMask: c_ulong,
    pub robotIpAddress: c_ulong,
    pub localCmdport: c_ushort,
    pub localBcastPort: c_ushort,
    pub robotPort: c_ushort,
    pub rxTimeOutInMs: c_ulong,
}

// --- Function pointer types ---

pub type FnInitEthernetAPI = unsafe extern "C" fn(*mut EthernetCommConfig) -> c_int;
pub type FnInitApi = unsafe extern "C" fn() -> c_int; // USB: InitAPI(void)
pub type FnCloseAPI = unsafe extern "C" fn() -> c_int;
pub type FnRefresDevicesList = unsafe extern "C" fn() -> c_int;
pub type FnGetDevices =
    unsafe extern "C" fn(devices: *mut KinovaDevice, result: *mut c_int) -> c_int;
pub type FnSetActiveDevice = unsafe extern "C" fn(device: KinovaDevice) -> c_int;
pub type FnStartControlAPI = unsafe extern "C" fn() -> c_int;
pub type FnStopControlAPI = unsafe extern "C" fn() -> c_int;
pub type FnSetAngularControl = unsafe extern "C" fn() -> c_int;
pub type FnSendBasicTrajectory = unsafe extern "C" fn(point: TrajectoryPoint) -> c_int;
pub type FnSendAdvanceTrajectory = unsafe extern "C" fn(point: TrajectoryPoint) -> c_int;
pub type FnEraseAllTrajectories = unsafe extern "C" fn() -> c_int;
pub type FnMoveHome = unsafe extern "C" fn() -> c_int;
pub type FnGetAngularPosition = unsafe extern "C" fn(response: *mut AngularPosition) -> c_int;
pub type FnGetAngularCommand = unsafe extern "C" fn(response: *mut AngularPosition) -> c_int;
pub type FnGetAngularVelocity = unsafe extern "C" fn(response: *mut AngularPosition) -> c_int;
pub type FnGetAngularForce = unsafe extern "C" fn(response: *mut AngularPosition) -> c_int;
pub type FnGetAngularCurrent = unsafe extern "C" fn(response: *mut AngularPosition) -> c_int;
pub type FnGetGeneralInformations =
    unsafe extern "C" fn(response: *mut GeneralInformations) -> c_int;
pub type FnSetJointZero = unsafe extern "C" fn(actuator_address: c_int) -> c_int;

// --- Helpers ---

pub const ACTUATOR_ADDRESSES: [i32; 7] = [16, 17, 18, 19, 20, 21, 25];

pub fn joint_to_actuator_address(joint: u8) -> Option<i32> {
    if !(1..=6).contains(&joint) {
        return None;
    }
    Some(ACTUATOR_ADDRESSES[(joint - 1) as usize])
}

pub fn angular_position_point(joints_deg: [f32; 6]) -> TrajectoryPoint {
    let mut p = TrajectoryPoint::default();
    p.Position.Type = POSITION_TYPE_ANGULAR_POSITION;
    p.Position.HandMode = HAND_MODE_POSITION_MODE;
    p.Position.Actuators = AngularInfo {
        Actuator1: joints_deg[0],
        Actuator2: joints_deg[1],
        Actuator3: joints_deg[2],
        Actuator4: joints_deg[3],
        Actuator5: joints_deg[4],
        Actuator6: joints_deg[5],
        Actuator7: 0.0,
    };
    p
}

pub fn angular_velocity_point(joints_dps: [f32; 6]) -> TrajectoryPoint {
    let mut p = TrajectoryPoint::default();
    p.Position.Type = POSITION_TYPE_ANGULAR_VELOCITY;
    p.Position.HandMode = HAND_MODE_POSITION_MODE; // matches C++ InitStruct() default
    p.Position.Actuators = AngularInfo {
        Actuator1: joints_dps[0],
        Actuator2: joints_dps[1],
        Actuator3: joints_dps[2],
        Actuator4: joints_dps[3],
        Actuator5: joints_dps[4],
        Actuator6: joints_dps[5],
        Actuator7: 0.0,
    };
    p
}
