use std::path::{Path, PathBuf};

use libloading::os::unix::{Library, RTLD_GLOBAL, RTLD_NOW};

use super::ffi::*;

const NO_ERROR_KINOVA: i32 = 1;

#[derive(Debug, thiserror::Error)]
pub enum SdkError {
    #[error("loading {0}: {1}")]
    LoadLibrary(PathBuf, #[source] libloading::Error),
    #[error("resolving symbol {0}: {1}")]
    ResolveSymbol(&'static str, #[source] libloading::Error),
    #[error("Kinova {call} returned {code}")]
    Call { call: &'static str, code: i32 },
    #[error("no Kinova devices found")]
    NoDevices,
}

// Function pointers shared between USB and Ethernet (same signatures, different
// symbol names — Ethernet has "Ethernet_" prefix, USB does not).
struct Vt {
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
    get_angular_position: FnGetAngularPosition,
    get_angular_velocity: FnGetAngularVelocity,
    get_angular_current: FnGetAngularCurrent,
    get_angular_force: FnGetAngularForce,
    get_general_informations: FnGetGeneralInformations,
    set_joint_zero: FnSetJointZero,
}

pub struct KinovaSdk {
    _libusb: Option<Library>, // pre-loaded for USB to put libusb symbols in global ns
    _comm_lib: Library,
    _command_lib: Library,
    vt: Vt,
    // Ethernet init: Ethernet_InitEthernetAPI(config)
    eth_init_fn: Option<FnInitEthernetAPI>,
    // USB init: InitAPI() — no args, different signature
    usb_init_fn: Option<FnInitApi>,
}

impl KinovaSdk {
    /// Load the Ethernet SDK (EthCommandLayerUbuntu.so).
    pub fn load_ethernet(lib_dir: &Path) -> Result<Self, SdkError> {
        let comm_lib = unsafe { load_global(&lib_dir.join("EthCommLayerUbuntu.so"))? };
        let cmd_lib = unsafe { load_global(&lib_dir.join("EthCommandLayerUbuntu.so"))? };

        let eth_init_fn = unsafe { *resolve::<FnInitEthernetAPI>(&cmd_lib, b"Ethernet_InitEthernetAPI")? };

        let vt = unsafe {
            Vt {
                close_api: *resolve(&cmd_lib, b"Ethernet_CloseAPI")?,
                refres_devices_list: *resolve(&cmd_lib, b"Ethernet_RefresDevicesList")?,
                get_devices: *resolve(&cmd_lib, b"Ethernet_GetDevices")?,
                set_active_device: *resolve(&cmd_lib, b"Ethernet_SetActiveDevice")?,
                start_control_api: *resolve(&cmd_lib, b"Ethernet_StartControlAPI")?,
                stop_control_api: *resolve(&cmd_lib, b"Ethernet_StopControlAPI")?,
                set_angular_control: *resolve(&cmd_lib, b"Ethernet_SetAngularControl")?,
                send_basic_trajectory: *resolve(&cmd_lib, b"Ethernet_SendBasicTrajectory")?,
                send_advance_trajectory: *resolve(&cmd_lib, b"Ethernet_SendAdvanceTrajectory")?,
                erase_all_trajectories: *resolve(&cmd_lib, b"Ethernet_EraseAllTrajectories")?,
                move_home: *resolve(&cmd_lib, b"Ethernet_MoveHome")?,
                get_angular_position: *resolve(&cmd_lib, b"Ethernet_GetAngularPosition")?,
                get_angular_velocity: *resolve(&cmd_lib, b"Ethernet_GetAngularVelocity")?,
                get_angular_current: *resolve(&cmd_lib, b"Ethernet_GetAngularCurrent")?,
                get_angular_force: *resolve(&cmd_lib, b"Ethernet_GetAngularForce")?,
                get_general_informations: *resolve(&cmd_lib, b"Ethernet_GetGeneralInformations")?,
                set_joint_zero: *resolve(&cmd_lib, b"Ethernet_SetJointZero")?,
            }
        };

        Ok(Self {
            _libusb: None,
            _comm_lib: comm_lib,
            _command_lib: cmd_lib,
            vt,
            eth_init_fn: Some(eth_init_fn),
            usb_init_fn: None,
        })
    }

    /// Load the USB SDK (USBCommandLayerUbuntu.so).
    /// Functions have the same signatures as Ethernet but no "Ethernet_" prefix.
    /// Init uses InitAPI() (no arguments).  Note: USBCommandLayerUbuntu.so does
    /// NOT export InitEthernetAPI despite the header listing it.
    pub fn load_usb(lib_dir: &Path) -> Result<Self, SdkError> {
        // USBCommLayerUbuntu.so uses libusb internally but doesn't list it as
        // a direct DT_NEEDED — it relies on libusb being in the global symbol
        // table.  Pre-load it so RTLD_NOW resolves libusb_close and friends.
        let libusb = unsafe {
            Library::open(Some("libusb-1.0.so.0"), RTLD_NOW | RTLD_GLOBAL)
                .map_err(|e| SdkError::LoadLibrary(PathBuf::from("libusb-1.0.so.0"), e))?
        };

        let comm_lib = unsafe { load_global(&lib_dir.join("USBCommLayerUbuntu.so"))? };
        let cmd_lib = unsafe { load_global(&lib_dir.join("USBCommandLayerUbuntu.so"))? };

        let usb_init_fn: FnInitApi = unsafe { *resolve::<FnInitApi>(&cmd_lib, b"InitAPI")? };

        let vt = unsafe {
            Vt {
                close_api: *resolve(&cmd_lib, b"CloseAPI")?,
                refres_devices_list: *resolve(&cmd_lib, b"RefresDevicesList")?,
                get_devices: *resolve(&cmd_lib, b"GetDevices")?,
                set_active_device: *resolve(&cmd_lib, b"SetActiveDevice")?,
                start_control_api: *resolve(&cmd_lib, b"StartControlAPI")?,
                stop_control_api: *resolve(&cmd_lib, b"StopControlAPI")?,
                set_angular_control: *resolve(&cmd_lib, b"SetAngularControl")?,
                send_basic_trajectory: *resolve(&cmd_lib, b"SendBasicTrajectory")?,
                send_advance_trajectory: *resolve(&cmd_lib, b"SendAdvanceTrajectory")?,
                erase_all_trajectories: *resolve(&cmd_lib, b"EraseAllTrajectories")?,
                move_home: *resolve(&cmd_lib, b"MoveHome")?,
                get_angular_position: *resolve(&cmd_lib, b"GetAngularPosition")?,
                get_angular_velocity: *resolve(&cmd_lib, b"GetAngularVelocity")?,
                get_angular_current: *resolve(&cmd_lib, b"GetAngularCurrent")?,
                get_angular_force: *resolve(&cmd_lib, b"GetAngularForce")?,
                get_general_informations: *resolve(&cmd_lib, b"GetGeneralInformations")?,
                set_joint_zero: *resolve(&cmd_lib, b"SetJointZero")?,
            }
        };

        Ok(Self {
            _libusb: Some(libusb),
            _comm_lib: comm_lib,
            _command_lib: cmd_lib,
            vt,
            eth_init_fn: None,
            usb_init_fn: Some(usb_init_fn),
        })
    }

    pub fn is_usb(&self) -> bool {
        self.usb_init_fn.is_some()
    }

    pub fn init_ethernet(&self, mut config: EthernetCommConfig) -> Result<(), SdkError> {
        let f = self.eth_init_fn.expect("init_ethernet called on USB SDK");
        check("InitEthernetAPI", unsafe { f(&mut config) })
    }

    pub fn init_usb(&self) -> Result<(), SdkError> {
        let f = self.usb_init_fn.expect("init_usb called on Ethernet SDK");
        check("InitAPI", unsafe { f() })
    }

    pub fn refresh_devices_list(&self) -> Result<(), SdkError> {
        check("RefresDevicesList", unsafe { (self.vt.refres_devices_list)() })
    }

    pub fn get_devices(&self) -> Result<Vec<KinovaDevice>, SdkError> {
        let mut buf = [KinovaDevice::default(); MAX_KINOVA_DEVICE];
        let mut result: i32 = 0;
        let count = unsafe { (self.vt.get_devices)(buf.as_mut_ptr(), &mut result) };
        check("GetDevices", result)?;
        if count <= 0 {
            return Err(SdkError::NoDevices);
        }
        Ok(buf[..count as usize].to_vec())
    }

    pub fn set_active_device(&self, device: KinovaDevice) -> Result<(), SdkError> {
        check("SetActiveDevice", unsafe { (self.vt.set_active_device)(device) })
    }

    pub fn start_control(&self) -> Result<(), SdkError> {
        check("StartControlAPI", unsafe { (self.vt.start_control_api)() })
    }

    pub fn set_angular_control(&self) -> Result<(), SdkError> {
        check("SetAngularControl", unsafe { (self.vt.set_angular_control)() })
    }

    pub fn send_basic_trajectory(&self, point: TrajectoryPoint) -> Result<(), SdkError> {
        check("SendBasicTrajectory", unsafe { (self.vt.send_basic_trajectory)(point) })
    }

    pub fn send_advance_trajectory(&self, point: TrajectoryPoint) -> Result<(), SdkError> {
        check("SendAdvanceTrajectory", unsafe { (self.vt.send_advance_trajectory)(point) })
    }

    pub fn erase_all_trajectories(&self) -> Result<(), SdkError> {
        check("EraseAllTrajectories", unsafe { (self.vt.erase_all_trajectories)() })
    }

    pub fn move_home(&self) -> Result<(), SdkError> {
        check("MoveHome", unsafe { (self.vt.move_home)() })
    }

    pub fn get_angular_position(&self) -> Result<AngularPosition, SdkError> {
        let mut out = AngularPosition::default();
        check("GetAngularPosition", unsafe { (self.vt.get_angular_position)(&mut out) })?;
        Ok(out)
    }

    pub fn get_angular_velocity(&self) -> Result<AngularPosition, SdkError> {
        let mut out = AngularPosition::default();
        check("GetAngularVelocity", unsafe { (self.vt.get_angular_velocity)(&mut out) })?;
        Ok(out)
    }

    pub fn get_angular_current(&self) -> Result<AngularPosition, SdkError> {
        let mut out = AngularPosition::default();
        check("GetAngularCurrent", unsafe { (self.vt.get_angular_current)(&mut out) })?;
        Ok(out)
    }

    pub fn get_angular_force(&self) -> Result<AngularPosition, SdkError> {
        let mut out = AngularPosition::default();
        check("GetAngularForce", unsafe { (self.vt.get_angular_force)(&mut out) })?;
        Ok(out)
    }

    pub fn get_general_informations(&self) -> Result<GeneralInformations, SdkError> {
        let mut out = GeneralInformations::default();
        check("GetGeneralInformations", unsafe {
            (self.vt.get_general_informations)(&mut out)
        })?;
        Ok(out)
    }

    pub fn set_joint_zero(&self, actuator_address: i32) -> Result<(), SdkError> {
        check("SetJointZero", unsafe { (self.vt.set_joint_zero)(actuator_address) })
    }
}

impl Drop for KinovaSdk {
    fn drop(&mut self) {
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
        SdkError::ResolveSymbol(std::str::from_utf8(name).unwrap_or("<non-utf8>"), e)
    })
}

fn check(call: &'static str, code: i32) -> Result<(), SdkError> {
    if code == NO_ERROR_KINOVA {
        Ok(())
    } else {
        Err(SdkError::Call { call, code })
    }
}
