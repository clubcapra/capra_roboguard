//! Raw C ABI bindings to the legacy Kinova SDK.
//!
//! Layouts are taken verbatim from `KinovaTypes.h` and
//! `Kinova.API.EthCommLayerUbuntu.h` in the Kinovarobotics/kinova_sdk_recompiled
//! repo. Field order, sizes, and integer widths must match the C headers
//! exactly — the SDK is closed-source, we cannot rebuild against changed types.
//!
//! Only the subset needed by this driver is bound. Cartesian, force, torque,
//! gravity, and protection-zone APIs are intentionally omitted (Capra's custom
//! arm has non-stock kinematics, so cartesian features return garbage).

#![allow(non_snake_case)]

use std::os::raw::{c_int, c_uchar, c_ulong, c_ushort};

/// `MAX_KINOVA_DEVICE` from `Kinova.API.EthCommLayerUbuntu.h`.
pub const MAX_KINOVA_DEVICE: usize = 20;

/// `SERIAL_LENGTH` / `STRING_LENGTH` — both 20 in the SDK.
pub const SERIAL_LENGTH: usize = 20;

/// `POSITION_TYPE::ANGULAR_POSITION`.
pub const POSITION_TYPE_ANGULAR_POSITION: c_int = 2;
/// `POSITION_TYPE::ANGULAR_VELOCITY`.
pub const POSITION_TYPE_ANGULAR_VELOCITY: c_int = 8;

/// `HAND_MODE::HAND_NOMOVEMENT` — fingers will not move during a trajectory.
pub const HAND_MODE_HAND_NOMOVEMENT: c_int = 0;

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
#[derive(Debug, Clone, Copy, Default)]
pub struct QuickStatus {
    pub Finger1Status: c_uchar,
    pub Finger2Status: c_uchar,
    pub Finger3Status: c_uchar,
    pub RetractType: c_uchar,
    pub RetractComplexity: c_uchar,
    pub ControlEnableStatus: c_uchar,
    pub ControlActiveModule: c_uchar,
    pub ControlFrameType: c_uchar,
    pub CartesianFaultState: c_uchar,
    pub ForceControlStatus: c_uchar,
    pub CurrentLimitationStatus: c_uchar,
    pub RobotType: c_uchar,
    pub RobotEdition: c_uchar,
    pub TorqueSensorsStatus: c_uchar,
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

// --- Function pointer signatures (resolved via libloading::Symbol) ---

pub type FnInitEthernetAPI = unsafe extern "C" fn(*mut EthernetCommConfig) -> c_int;
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
pub type FnClearErrorLog = unsafe extern "C" fn() -> c_int;
pub type FnGetAngularPosition = unsafe extern "C" fn(response: *mut AngularPosition) -> c_int;
pub type FnGetAngularVelocity = unsafe extern "C" fn(response: *mut AngularPosition) -> c_int;
pub type FnGetAngularForce = unsafe extern "C" fn(response: *mut AngularPosition) -> c_int;
pub type FnGetAngularCurrent = unsafe extern "C" fn(response: *mut AngularPosition) -> c_int;
pub type FnGetSensorsInfo = unsafe extern "C" fn(response: *mut SensorsInfo) -> c_int;
pub type FnGetQuickStatus = unsafe extern "C" fn(response: *mut QuickStatus) -> c_int;
pub type FnSetJointZero = unsafe extern "C" fn(actuator_address: c_int) -> c_int;

/// `KinovaTypes.h::Finger` — 116 bytes. Represents one finger of the gripper.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
#[allow(non_snake_case)]
pub struct Finger {
    pub ID: [u8; 20],
    pub ActualCommand: f32,
    pub ActualSpeed: f32,
    pub ActualForce: f32,
    pub ActualAcceleration: f32,
    pub ActualCurrent: f32,
    pub ActualPosition: f32,
    pub ActualAverageCurrent: f32,
    pub ActualTemperature: f32,
    pub CommunicationErrors: i32,
    pub OscillatorTuningValue: i32,
    pub CycleCount: f32,
    pub RunTime: f32,
    pub PeakMaxTemp: f32,
    pub PeakMinTemp: f32,
    pub PeakCurrent: f32,
    pub MaxSpeed: f32,
    pub MaxForce: f32,
    pub MaxAcceleration: f32,
    pub MaxCurrent: f32,
    pub MaxAngle: f32,
    pub MinAngle: f32,
    pub DeviceID: u32,
    pub CodeVersion: u32,
    pub IsFingerInit: u16,
    pub Index: u16,
    pub FingerAddress: u16,
    pub IsFingerConnected: u16,
}

impl Default for Finger {
    fn default() -> Self {
        // Safety: `Finger` is `#[repr(C)]` with no padding-sensitive layout
        // tricks; all-zero is a valid representation for every field.
        unsafe { std::mem::zeroed() }
    }
}

/// `KinovaTypes.h::Gripper` — `char Model[20]` + 3 × Finger.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
#[allow(non_snake_case)]
pub struct Gripper {
    pub Model: [u8; 20],
    pub Fingers: [Finger; 3],
}

impl Default for Gripper {
    fn default() -> Self {
        Self {
            Model: [0u8; 20],
            Fingers: [Finger::default(); 3],
        }
    }
}

pub type FnGetGripperStatus = unsafe extern "C" fn(response: *mut Gripper) -> c_int;
pub type FnInitFingers = unsafe extern "C" fn() -> c_int;

/// Kinova RS-485 message frame, 20 bytes on the wire.
///
/// Layout matches `Kinova.API.EthCommLayerUbuntu.h` exactly. The 16-byte
/// data union is exposed as raw bytes here — interpretation as float / u32
/// is the caller's job.
///
/// **NOT** a Modbus frame. The Kinova SDK uses this struct over its own
/// internal-actuator bus; the on-wire encoding is undocumented but assumed
/// to be the 20-byte struct verbatim plus whatever bus-layer framing the
/// comm layer adds (preamble / CRC). Whether arbitrary Modbus bytes
/// shoehorned into this struct reach the gripper meaningfully is exactly
/// what `Cmd::GripperProbe` tests.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
#[allow(non_snake_case)]
pub struct RS485Message {
    pub Command: i16,
    pub SourceAddress: u8,
    pub DestinationAddress: u8,
    pub DataByte: [u8; 16],
}

impl RS485Message {
    pub fn zeroed() -> Self {
        Self {
            Command: 0,
            SourceAddress: 0,
            DestinationAddress: 0,
            DataByte: [0u8; 16],
        }
    }

    /// Reinterpret as a 20-byte buffer for hex logging.
    pub fn as_bytes(&self) -> [u8; 20] {
        // Safety: `RS485Message` is `#[repr(C)]` with no padding (every
        // field is byte-aligned), so transmuting to `[u8; 20]` is sound.
        unsafe { std::mem::transmute(*self) }
    }
}

pub type FnOpenRS485Activate = unsafe extern "C" fn() -> c_int;
pub type FnOpenRS485Write = unsafe extern "C" fn(
    packages_out: *const RS485Message,
    qty_to_send: c_int,
    qty_sent: *mut c_int,
) -> c_int;
pub type FnOpenRS485Read = unsafe extern "C" fn(
    packages_in: *mut RS485Message,
    qty_wanted: c_int,
    qty_received: *mut c_int,
) -> c_int;

/// Modbus RTU CRC-16 (poly 0xA001, init 0xFFFF). Returns the 16-bit CRC
/// in standard low-then-high byte order.
pub fn modbus_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

/// Build a Robotiq 2F-140 Modbus RTU "Read Status" frame and stuff its
/// 8 bytes into the first 8 bytes of an `RS485Message`. Slave 9, FC03,
/// registers 0x07D0..0x07D2 (3 regs).
///
/// Wire bytes: `09 03 07 D0 00 03 [crc_lo] [crc_hi]`
pub fn robotiq_probe_frame() -> RS485Message {
    let payload: [u8; 6] = [0x09, 0x03, 0x07, 0xD0, 0x00, 0x03];
    let crc = modbus_crc16(&payload);
    let mut m = RS485Message::zeroed();
    // Stuff Modbus byte 0..1 into Command (host LE), 2 into Source, 3 into
    // Dest, 4..5 + CRC into DataByte[0..3]. Remaining 12 bytes stay zero.
    m.Command = i16::from_le_bytes([payload[0], payload[1]]);
    m.SourceAddress = payload[2];
    m.DestinationAddress = payload[3];
    m.DataByte[0] = payload[4];
    m.DataByte[1] = payload[5];
    m.DataByte[2] = (crc & 0xFF) as u8;
    m.DataByte[3] = (crc >> 8) as u8;
    m
}

/// Build a Robotiq 2F-140 Modbus RTU activation frame: FC06 (write single
/// register), slave 9, register 0x03E8, value 0x0001 (sets rACT=1).
///
/// Wire bytes: `09 06 03 E8 00 01 [crc_lo] [crc_hi]` — 8 bytes, stuffed into
/// the first 8 bytes of an `RS485Message` exactly like `robotiq_probe_frame`.
/// A red-LED Robotiq should transition to blinking red/blue within ~1 s of
/// this frame landing on the bus.
pub fn robotiq_activate_frame() -> RS485Message {
    let payload: [u8; 6] = [0x09, 0x06, 0x03, 0xE8, 0x00, 0x01];
    let crc = modbus_crc16(&payload);
    let mut m = RS485Message::zeroed();
    m.Command = i16::from_le_bytes([payload[0], payload[1]]);
    m.SourceAddress = payload[2];
    m.DestinationAddress = payload[3];
    m.DataByte[0] = payload[4];
    m.DataByte[1] = payload[5];
    m.DataByte[2] = (crc & 0xFF) as u8;
    m.DataByte[3] = (crc >> 8) as u8;
    m
}

/// Per-joint actuator address on the legacy Kinova RS-485 bus, used by
/// `Ethernet_SetJointZero`/`SetTorqueZero`/`SetActuatorPID`. The 7th slot
/// (address 25) belongs to the wrist on 7DOF arms — unused on this 6DOF
/// build.
pub const ACTUATOR_ADDRESSES: [i32; 7] = [16, 17, 18, 19, 20, 21, 25];

/// Resolve a 1-indexed joint number (1..=6) to the corresponding actuator
/// bus address. Returns `None` for out-of-range joints so the caller can
/// surface a clean error rather than poking a random address.
pub fn joint_to_actuator_address(joint: u8) -> Option<i32> {
    if !(1..=6).contains(&joint) {
        return None;
    }
    Some(ACTUATOR_ADDRESSES[(joint - 1) as usize])
}

/// Build a `TrajectoryPoint` for a 6DOF angular position setpoint (degrees).
/// Slot 7 is zeroed — unused on this arm.
pub fn angular_position_point(joints_deg: [f32; 6]) -> TrajectoryPoint {
    let mut p = TrajectoryPoint::default();
    p.Position.Type = POSITION_TYPE_ANGULAR_POSITION;
    p.Position.HandMode = HAND_MODE_HAND_NOMOVEMENT;
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

/// Build a `TrajectoryPoint` for a 6DOF angular velocity setpoint (deg/s).
///
/// Matches clubcapra/ovis `KinovaComm::setJointVelocities`: zero everything,
/// set `Position.Type = ANGULAR_VELOCITY`, drop the velocities into
/// `Position.Actuators`, leave `Limitations` and `LimitationsActive` at zero,
/// then send via `Ethernet_SendAdvanceTrajectory` (not the basic variant —
/// the basic call is for completed trajectory points and ignores velocity).
pub fn angular_velocity_point(joints_dps: [f32; 6]) -> TrajectoryPoint {
    let mut p = TrajectoryPoint::default();
    p.Position.Type = POSITION_TYPE_ANGULAR_VELOCITY;
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
