//! Safe Rust wrapper around the Kinova legacy SDK shared libraries.
//!
//! Owns two `libloading::os::unix::Library` handles (kept alive for the
//! lifetime of the wrapper) plus resolved function pointers for the calls
//! the driver actually uses.
//!
//! **Threading:** the SDK is *not* documented as thread-safe. Construct one
//! `KinovaSdk` and call all methods from a single thread (the worker).

use std::os::raw::c_int;
use std::path::{Path, PathBuf};

use libloading::os::unix::{Library, RTLD_GLOBAL, RTLD_NOW};

use super::ffi::*;

/// Rust mirror of the SDK's "no error" return.
const NO_ERROR_KINOVA: i32 = 1;

#[derive(Debug, thiserror::Error)]
pub enum SdkError {
    #[error("loading {0}: {1}")]
    LoadLibrary(PathBuf, #[source] libloading::Error),
    #[error("resolving symbol {0}: {1}")]
    ResolveSymbol(&'static str, #[source] libloading::Error),
    #[error("Kinova {call} returned {code}")]
    Call { call: &'static str, code: i32 },
    #[error("no Kinova devices found on the network")]
    NoDevices,
}

/// All function pointers we need, resolved up-front.
struct Vt {
    init_ethernet_api: FnInitEthernetAPI,
    close_api: FnCloseAPI,
    refres_devices_list: FnRefresDevicesList,
    get_devices: FnGetDevices,
    set_active_device: FnSetActiveDevice,
    start_control_api: FnStartControlAPI,
    stop_control_api: FnStopControlAPI,
    set_angular_control: FnSetAngularControl,
    send_basic_trajectory: FnSendBasicTrajectory,
    send_advance_trajectory: FnSendAdvanceTrajectory,
    erase_all_trajectories: FnEraseAllTrajectories,
    move_home: FnMoveHome,
    clear_error_log: FnClearErrorLog,
    get_angular_position: FnGetAngularPosition,
    get_angular_velocity: FnGetAngularVelocity,
    get_angular_force: FnGetAngularForce,
    get_angular_current: FnGetAngularCurrent,
    get_sensors_info: FnGetSensorsInfo,
    get_quick_status: FnGetQuickStatus,
    set_joint_zero: FnSetJointZero,
    get_gripper_status: FnGetGripperStatus,
    init_fingers: FnInitFingers,
    // RS485 passthrough — resolved from the *comm* layer .so, not the
    // command layer. Activating leaves the arm in a state where the
    // normal `Ethernet_*` API stops responding (per Kinova docs); the
    // process must be restarted to recover.
    open_rs485_activate: FnOpenRS485Activate,
    open_rs485_write: FnOpenRS485Write,
    open_rs485_read: FnOpenRS485Read,
}

/// Held but not used directly — keep the `.so` files mapped for the lifetime
/// of the `KinovaSdk`. `_comm_lib` must be loaded first so its SONAME
/// (`Kinova.API.EthCommLayerUbuntu.so`) is registered before the command
/// layer's internal `dlopen` runs during its constructor.
pub struct KinovaSdk {
    _comm_lib: Library,
    _command_lib: Library,
    vt: Vt,
}

impl KinovaSdk {
    /// Load both .so files from `lib_dir` and resolve every function pointer
    /// we need. Does **not** initialise the Ethernet session — call
    /// [`init_ethernet`] after.
    pub fn load(lib_dir: &Path) -> Result<Self, SdkError> {
        let comm_path = lib_dir.join("EthCommLayerUbuntu.so");
        let command_path = lib_dir.join("EthCommandLayerUbuntu.so");

        let comm_lib = unsafe { load_global(&comm_path)? };
        let command_lib = unsafe { load_global(&command_path)? };

        let vt = unsafe {
            Vt {
                init_ethernet_api: *resolve(&command_lib, b"Ethernet_InitEthernetAPI")?,
                close_api: *resolve(&command_lib, b"Ethernet_CloseAPI")?,
                refres_devices_list: *resolve(&command_lib, b"Ethernet_RefresDevicesList")?,
                get_devices: *resolve(&command_lib, b"Ethernet_GetDevices")?,
                set_active_device: *resolve(&command_lib, b"Ethernet_SetActiveDevice")?,
                start_control_api: *resolve(&command_lib, b"Ethernet_StartControlAPI")?,
                stop_control_api: *resolve(&command_lib, b"Ethernet_StopControlAPI")?,
                set_angular_control: *resolve(&command_lib, b"Ethernet_SetAngularControl")?,
                send_basic_trajectory: *resolve(&command_lib, b"Ethernet_SendBasicTrajectory")?,
                send_advance_trajectory: *resolve(&command_lib, b"Ethernet_SendAdvanceTrajectory")?,
                erase_all_trajectories: *resolve(&command_lib, b"Ethernet_EraseAllTrajectories")?,
                move_home: *resolve(&command_lib, b"Ethernet_MoveHome")?,
                clear_error_log: *resolve(&command_lib, b"Ethernet_ClearErrorLog")?,
                get_angular_position: *resolve(&command_lib, b"Ethernet_GetAngularPosition")?,
                get_angular_velocity: *resolve(&command_lib, b"Ethernet_GetAngularVelocity")?,
                get_angular_force: *resolve(&command_lib, b"Ethernet_GetAngularForce")?,
                get_angular_current: *resolve(&command_lib, b"Ethernet_GetAngularCurrent")?,
                get_sensors_info: *resolve(&command_lib, b"Ethernet_GetSensorsInfo")?,
                get_quick_status: *resolve(&command_lib, b"Ethernet_GetQuickStatus")?,
                set_joint_zero: *resolve(&command_lib, b"Ethernet_SetJointZero")?,
                get_gripper_status: *resolve(&command_lib, b"Ethernet_GetGripperStatus")?,
                init_fingers: *resolve(&command_lib, b"Ethernet_InitFingers")?,
                open_rs485_activate: *resolve(
                    &comm_lib,
                    b"Ethernet_Communication_OpenRS485_Activate",
                )?,
                open_rs485_write: *resolve(
                    &comm_lib,
                    b"Ethernet_Communication_OpenRS485_Write",
                )?,
                open_rs485_read: *resolve(
                    &comm_lib,
                    b"Ethernet_Communication_OpenRS485_Read",
                )?,
            }
        };

        Ok(Self {
            _comm_lib: comm_lib,
            _command_lib: command_lib,
            vt,
        })
    }

    /// Initialise the API in Ethernet mode.
    ///
    /// **Only** `Ethernet_InitEthernetAPI` is called — `Ethernet_InitAPI`
    /// is the USB-mode init and calling it first makes the SDK try to talk
    /// to a USB arm with a defaulted config, returning error 1010. This
    /// matches the clubcapra reference driver's init path.
    pub fn init_ethernet(&self, mut config: EthernetCommConfig) -> Result<(), SdkError> {
        check("InitEthernetAPI", unsafe {
            (self.vt.init_ethernet_api)(&mut config)
        })?;
        Ok(())
    }

    /// Trigger a broadcast scan of the configured subnet. Must be called
    /// before `get_devices` after a fresh `InitEthernetAPI` — otherwise
    /// `get_devices` returns an empty cached list (the SDK does not scan
    /// during init). Note the misspelling "Refres" matches the SDK export.
    pub fn refresh_devices_list(&self) -> Result<(), SdkError> {
        check("RefresDevicesList", unsafe {
            (self.vt.refres_devices_list)()
        })
    }

    /// Discover devices on the configured Ethernet subnet. Returns the full
    /// list (max 20).
    pub fn get_devices(&self) -> Result<Vec<KinovaDevice>, SdkError> {
        let mut buf = [KinovaDevice::default(); MAX_KINOVA_DEVICE];
        let mut result: i32 = 0;
        let count = unsafe { (self.vt.get_devices)(buf.as_mut_ptr(), &mut result) };
        // get_devices returns the device count; result is the SDK error code.
        check("GetDevices", result)?;
        if count <= 0 {
            return Err(SdkError::NoDevices);
        }
        Ok(buf[..count as usize].to_vec())
    }

    pub fn set_active_device(&self, device: KinovaDevice) -> Result<(), SdkError> {
        check("SetActiveDevice", unsafe {
            (self.vt.set_active_device)(device)
        })
    }

    pub fn start_control(&self) -> Result<(), SdkError> {
        check("StartControlAPI", unsafe { (self.vt.start_control_api)() })
    }

    pub fn stop_control(&self) -> Result<(), SdkError> {
        check("StopControlAPI", unsafe { (self.vt.stop_control_api)() })
    }

    pub fn set_angular_control(&self) -> Result<(), SdkError> {
        check("SetAngularControl", unsafe {
            (self.vt.set_angular_control)()
        })
    }

    pub fn send_basic_trajectory(&self, point: TrajectoryPoint) -> Result<(), SdkError> {
        check("SendBasicTrajectory", unsafe {
            (self.vt.send_basic_trajectory)(point)
        })
    }

    /// Like `send_basic_trajectory` but honours `LimitationsActive` and the
    /// `Limitations` struct. Required for velocity setpoints — the basic
    /// variant ignores `speedParameter1/2` and the firmware falls back to
    /// conservative default speed caps that make the arm crawl.
    pub fn send_advance_trajectory(&self, point: TrajectoryPoint) -> Result<(), SdkError> {
        check("SendAdvanceTrajectory", unsafe {
            (self.vt.send_advance_trajectory)(point)
        })
    }

    pub fn erase_all_trajectories(&self) -> Result<(), SdkError> {
        check("EraseAllTrajectories", unsafe {
            (self.vt.erase_all_trajectories)()
        })
    }

    pub fn move_home(&self) -> Result<(), SdkError> {
        check("MoveHome", unsafe { (self.vt.move_home)() })
    }

    pub fn clear_error_log(&self) -> Result<(), SdkError> {
        check("ClearErrorLog", unsafe { (self.vt.clear_error_log)() })
    }

    pub fn get_angular_position(&self) -> Result<AngularPosition, SdkError> {
        let mut out = AngularPosition::default();
        check("GetAngularPosition", unsafe {
            (self.vt.get_angular_position)(&mut out)
        })?;
        Ok(out)
    }

    pub fn get_angular_velocity(&self) -> Result<AngularPosition, SdkError> {
        let mut out = AngularPosition::default();
        check("GetAngularVelocity", unsafe {
            (self.vt.get_angular_velocity)(&mut out)
        })?;
        Ok(out)
    }

    pub fn get_angular_force(&self) -> Result<AngularPosition, SdkError> {
        let mut out = AngularPosition::default();
        check("GetAngularForce", unsafe {
            (self.vt.get_angular_force)(&mut out)
        })?;
        Ok(out)
    }

    pub fn get_angular_current(&self) -> Result<AngularPosition, SdkError> {
        let mut out = AngularPosition::default();
        check("GetAngularCurrent", unsafe {
            (self.vt.get_angular_current)(&mut out)
        })?;
        Ok(out)
    }

    pub fn get_sensors_info(&self) -> Result<SensorsInfo, SdkError> {
        let mut out = SensorsInfo::default();
        check("GetSensorsInfo", unsafe {
            (self.vt.get_sensors_info)(&mut out)
        })?;
        Ok(out)
    }

    pub fn get_quick_status(&self) -> Result<QuickStatus, SdkError> {
        let mut out = QuickStatus::default();
        check("GetQuickStatus", unsafe {
            (self.vt.get_quick_status)(&mut out)
        })?;
        Ok(out)
    }

    /// Set the current position of one actuator as its new zero reference.
    /// **Persists in the actuator's flash — call only when intentionally
    /// re-zeroing.** Use `joint_to_actuator_address` to convert the
    /// 1-indexed joint number to the bus address.
    pub fn set_joint_zero(&self, actuator_address: i32) -> Result<(), SdkError> {
        check("SetJointZero", unsafe {
            (self.vt.set_joint_zero)(actuator_address)
        })
    }

    /// Read the SDK's view of the gripper (model name + 3 finger snapshots).
    /// On a Kinova-native gripper this returns sane data; on a Capra-modded
    /// arm with a Robotiq, the contents tell us whether the firmware sees
    /// any device on the joint-7 slot.
    pub fn get_gripper_status(&self) -> Result<Gripper, SdkError> {
        let mut g = Gripper::default();
        check("GetGripperStatus", unsafe {
            (self.vt.get_gripper_status)(&mut g)
        })?;
        Ok(g)
    }

    /// Run the SDK's finger initialisation sequence. On Kinova-native fingers
    /// this drives them through their range to find limits; on a custom arm
    /// it might trigger whatever bridge the firmware uses to wake up an
    /// attached Robotiq.
    pub fn init_fingers(&self) -> Result<(), SdkError> {
        check("InitFingers", unsafe { (self.vt.init_fingers)() })
    }

    /// Switch the arm into raw RS-485 mode. **One-way**: per Kinova docs the
    /// normal `Ethernet_*` API stops responding after this call and the
    /// process must be restarted to recover. Used only by the gripper probe.
    pub fn rs485_activate(&self) -> Result<(), SdkError> {
        check("OpenRS485_Activate", unsafe {
            (self.vt.open_rs485_activate)()
        })
    }

    /// Send one or more RS485 messages on the bus. Returns the number actually
    /// transmitted (the SDK may transmit fewer than requested under congestion).
    pub fn rs485_write(&self, msgs: &[RS485Message]) -> Result<usize, SdkError> {
        let mut sent: c_int = 0;
        check("OpenRS485_Write", unsafe {
            (self.vt.open_rs485_write)(msgs.as_ptr(), msgs.len() as c_int, &mut sent)
        })?;
        Ok(sent as usize)
    }

    /// Read up to `qty_wanted` messages from the bus (max 50, per the SDK
    /// header). The SDK applies the configured `rxTimeOutInMs` internally.
    pub fn rs485_read(&self, qty_wanted: usize) -> Result<Vec<RS485Message>, SdkError> {
        let qty_wanted = qty_wanted.min(50);
        let mut buf = [RS485Message::zeroed(); 50];
        let mut got: c_int = 0;
        check("OpenRS485_Read", unsafe {
            (self.vt.open_rs485_read)(buf.as_mut_ptr(), qty_wanted as c_int, &mut got)
        })?;
        Ok(buf[..got as usize].to_vec())
    }
}

impl Drop for KinovaSdk {
    fn drop(&mut self) {
        // Best-effort shutdown — ignore errors, the libraries are about to unload.
        let _ = check("StopControlAPI", unsafe { (self.vt.stop_control_api)() });
        let _ = check("CloseAPI", unsafe { (self.vt.close_api)() });
    }
}

unsafe fn load_global(path: &Path) -> Result<Library, SdkError> {
    Library::open(Some(path), RTLD_NOW | RTLD_GLOBAL)
        .map_err(|e| SdkError::LoadLibrary(path.to_path_buf(), e))
}

unsafe fn resolve<T>(
    lib: &Library,
    name: &'static [u8],
) -> Result<libloading::os::unix::Symbol<T>, SdkError> {
    lib.get(name).map_err(|e| {
        SdkError::ResolveSymbol(
            std::str::from_utf8(name).unwrap_or("<non-utf8>"),
            e,
        )
    })
}

fn check(call: &'static str, code: i32) -> Result<(), SdkError> {
    if code == NO_ERROR_KINOVA {
        Ok(())
    } else {
        Err(SdkError::Call { call, code })
    }
}
