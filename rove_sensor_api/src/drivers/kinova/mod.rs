pub mod arm;
pub mod config;
pub mod ffi;
pub mod sdk;
pub mod state;
pub mod worker;

use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use arm::KinovaArm;
use config::KinovaConfig;
use ffi::{EthernetCommConfig, KinovaDevice};
use sdk::{KinovaSdk, SdkError};
use state::KinovaState;

#[derive(Debug, thiserror::Error)]
pub enum ConnectError {
    #[error(transparent)]
    Sdk(#[from] SdkError),
}

const VENDOR_SUBPATH: &str = "vendor/kinova/aarch64";

const DISCOVERY_MAX_ATTEMPTS: u32 = 5;
const DISCOVERY_BROADCAST_WAIT: Duration = Duration::from_millis(500);
const DISCOVERY_RETRY_DELAY: Duration = Duration::from_secs(1);

fn resolve_lib_dir(cfg: &KinovaConfig) -> PathBuf {
    if let Some(dir) = &cfg.lib_dir {
        return dir.clone();
    }
    if let Ok(manifest) = std::env::var("CARGO_MANIFEST_DIR") {
        let p = PathBuf::from(manifest).join(VENDOR_SUBPATH);
        if p.exists() {
            return p;
        }
    }
    if let Some(p) = walk_up_for_vendor(std::env::current_exe().ok().as_deref()) {
        return p;
    }
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/vendor/kinova/aarch64"))
}

fn walk_up_for_vendor(start: Option<&Path>) -> Option<PathBuf> {
    let mut cursor = start?.parent()?;
    loop {
        let candidate = cursor.join(VENDOR_SUBPATH);
        if candidate.exists() {
            return Some(candidate);
        }
        cursor = cursor.parent()?;
    }
}

fn ethernet_config(cfg: &KinovaConfig) -> EthernetCommConfig {
    EthernetCommConfig {
        localIpAddress: u32::from_le_bytes(cfg.local_ip.octets()) as _,
        subnetMask: u32::from_le_bytes(cfg.local_subnet.octets()) as _,
        robotIpAddress: u32::from_le_bytes(cfg.robot_ip.octets()) as _,
        localCmdport: cfg.local_cmd_port,
        localBcastPort: cfg.local_bcast_port,
        robotPort: cfg.robot_port,
        rxTimeOutInMs: cfg.rx_timeout_ms as _,
    }
}

pub fn connect(cfg: &KinovaConfig) -> Result<KinovaArm, ConnectError> {
    let lib_dir = resolve_lib_dir(cfg);
    let eth = ethernet_config(cfg);
    let use_usb = cfg.use_usb;
    let state = Arc::new(RwLock::new(KinovaState::default()));
    let (cmd_tx, cmd_rx) = mpsc::channel();
    let (init_tx, init_rx) = mpsc::sync_channel::<Result<(), ConnectError>>(0);

    let offsets = cfg.joint_offsets;
    let command_rate_hz = cfg.command_rate_hz.max(1);
    let state_for_worker = state.clone();

    std::thread::Builder::new()
        .name("kinova-worker".into())
        .spawn(move || {
            let sdk = match init_sdk(&lib_dir, eth, use_usb) {
                Ok(s) => s,
                Err(e) => {
                    let _ = init_tx.send(Err(e));
                    return;
                }
            };
            if init_tx.send(Ok(())).is_err() {
                return;
            }
            worker::run(sdk, cmd_rx, state_for_worker, offsets, command_rate_hz);
        })
        .expect("spawn kinova-worker thread");

    match init_rx.recv() {
        Ok(Ok(())) => Ok(KinovaArm::new(state, cmd_tx)),
        Ok(Err(e)) => Err(e),
        Err(_) => Err(ConnectError::Sdk(SdkError::NoDevices)),
    }
}

fn init_sdk(lib_dir: &Path, eth: EthernetCommConfig, use_usb: bool) -> Result<KinovaSdk, ConnectError> {
    if use_usb {
        tracing::info!(?lib_dir, "loading Kinova USB SDK libraries");
        let sdk = KinovaSdk::load_usb(lib_dir)?;
        tracing::info!("initialising Kinova USB API");
        sdk.init_usb()?;

        // USB: devices are found directly via USB enumeration, no broadcast needed.
        let device = discover_arm_usb(&sdk)?;
        finish_init(sdk, device)
    } else {
        tracing::info!(?lib_dir, "loading Kinova Ethernet SDK libraries");
        let sdk = KinovaSdk::load_ethernet(lib_dir)?;
        tracing::info!(
            local_cmd_port = eth.localCmdport,
            robot_port = eth.robotPort,
            rx_timeout_ms = eth.rxTimeOutInMs,
            "initialising Kinova Ethernet API"
        );
        sdk.init_ethernet(eth)?;
        let device = discover_arm_ethernet(&sdk)?;
        finish_init(sdk, device)
    }
}

/// Shared post-discovery setup.
fn finish_init(sdk: KinovaSdk, device: KinovaDevice) -> Result<KinovaSdk, ConnectError> {
    let serial = cstr_to_string(&device.SerialNumber);
    let model = cstr_to_string(&device.Model);
    tracing::info!(
        serial = %serial,
        model = %model,
        device_type = device.DeviceType,
        transport = if sdk.is_usb() { "USB" } else { "Ethernet" },
        firmware = format!(
            "{}.{}.{}",
            device.VersionMajor, device.VersionMinor, device.VersionRelease
        ),
        "Kinova device found"
    );

    retry_sdk("SetActiveDevice", 3, Duration::from_millis(300), || {
        sdk.set_active_device(device)
    })?;

    // StartControlAPI puts the arm into API control mode.  Without it,
    // SendBasicTrajectory with ANGULAR_VELOCITY type is rejected with 1022
    // (the arm is not in a mode that accepts velocity commands from the API).
    // Position commands (MoveHome, ANGULAR_POSITION) work regardless because
    // they use the firmware's internal trajectory system.
    retry_sdk("StartControlAPI", 3, Duration::from_millis(300), || {
        sdk.start_control()
    })?;

    // SetAngularControl selects the angular (joint-space) control frame.
    // Required before streaming ANGULAR_VELOCITY commands.
    retry_sdk("SetAngularControl", 3, Duration::from_millis(300), || {
        sdk.set_angular_control()
    })?;

    Ok(sdk)
}

/// USB discovery: the C++ reference wrapper calls InitAPI() then GetDevices()
/// directly — no RefresDevicesList.  InitAPI() returns before libusb finishes
/// enumerating the device, so GetDevices() immediately after may return a
/// partially-initialised entry (firmware "0.0.0", garbage model string) that
/// causes SetActiveDevice to segfault.  We wait for enumeration to settle and
/// reject entries that are still initialising.
fn discover_arm_usb(sdk: &KinovaSdk) -> Result<KinovaDevice, ConnectError> {
    for attempt in 1..=DISCOVERY_MAX_ATTEMPTS {
        tracing::info!(attempt, max = DISCOVERY_MAX_ATTEMPTS, "Kinova USB: waiting for device");

        // Give libusb time to finish enumerating the arm.  No RefresDevicesList —
        // it confuses the USB stack and is not called by the reference C++ wrapper.
        std::thread::sleep(Duration::from_secs(1));

        match sdk.get_devices() {
            Ok(d) if !d.is_empty() => {
                let dev = d[0];
                // Reject partially-initialised entries.  A real device has a
                // non-zero firmware version; firmware 0.0.0 means the USB stack
                // hasn't finished enumerating yet.
                let ready = dev.DeviceType >= 0
                    && (dev.VersionMajor > 0 || dev.VersionMinor > 0 || dev.VersionRelease > 0);
                if ready {
                    tracing::info!(attempt, device_type = dev.DeviceType, "Kinova USB: device ready");
                    return Ok(dev);
                } else {
                    tracing::warn!(
                        attempt,
                        max = DISCOVERY_MAX_ATTEMPTS,
                        device_type = dev.DeviceType,
                        firmware = format!("{}.{}.{}", dev.VersionMajor, dev.VersionMinor, dev.VersionRelease),
                        "Kinova USB: device not yet initialised — retrying"
                    );
                }
            }
            Ok(_) | Err(SdkError::NoDevices) => {
                tracing::warn!(attempt, max = DISCOVERY_MAX_ATTEMPTS, "Kinova USB: no device yet");
            }
            Err(e) => {
                tracing::warn!(attempt, error = %e, "Kinova USB: GetDevices error");
            }
        }
    }
    Err(ConnectError::Sdk(SdkError::NoDevices))
}

/// Ethernet discovery: broadcast and wait for the arm to respond.
fn discover_arm_ethernet(sdk: &KinovaSdk) -> Result<KinovaDevice, ConnectError> {
    for attempt in 1..=DISCOVERY_MAX_ATTEMPTS {
        tracing::info!(attempt, max = DISCOVERY_MAX_ATTEMPTS, "Kinova: broadcasting discovery");

        if let Err(e) = sdk.refresh_devices_list() {
            tracing::warn!(attempt, error = %e, "Kinova: RefresDevicesList failed");
        }
        std::thread::sleep(DISCOVERY_BROADCAST_WAIT);

        match sdk.get_devices() {
            Ok(d) if !d.is_empty() => {
                tracing::info!(attempt, count = d.len(), "Kinova: arm discovered");
                return Ok(d[0]);
            }
            Ok(_) => tracing::warn!(attempt, max = DISCOVERY_MAX_ATTEMPTS, "Kinova: no devices"),
            Err(SdkError::NoDevices) => tracing::warn!(attempt, max = DISCOVERY_MAX_ATTEMPTS, "Kinova: no arm found"),
            Err(e) => tracing::warn!(attempt, error = %e, "Kinova: GetDevices error"),
        }

        if attempt < DISCOVERY_MAX_ATTEMPTS {
            tracing::info!(attempt, retry_ms = DISCOVERY_RETRY_DELAY.as_millis(), "Kinova: retrying");
            std::thread::sleep(DISCOVERY_RETRY_DELAY);
        }
    }
    tracing::error!(attempts = DISCOVERY_MAX_ATTEMPTS, "Kinova: arm not found — check power and robot_ip");
    Err(ConnectError::Sdk(SdkError::NoDevices))
}

fn retry_sdk<F>(name: &'static str, attempts: u32, delay: Duration, mut f: F) -> Result<(), ConnectError>
where
    F: FnMut() -> Result<(), SdkError>,
{
    for attempt in 1..=attempts {
        match f() {
            Ok(()) => return Ok(()),
            Err(e) if attempt < attempts => {
                tracing::warn!(attempt, max = attempts, error = %e, delay_ms = delay.as_millis(), "Kinova: {name} failed — retrying");
                std::thread::sleep(delay);
            }
            Err(e) => return Err(ConnectError::Sdk(e)),
        }
    }
    unreachable!()
}

fn cstr_to_string(buf: &[u8]) -> String {
    let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    String::from_utf8_lossy(&buf[..len]).into_owned()
}
