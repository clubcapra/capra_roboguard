//! Kinova Gen2 6DOF arm driver (legacy SDK over Ethernet).
//!
//! # Overview
//!
//! This driver wraps the closed-source Kinova legacy SDK shared libraries
//! (vendored at `vendor/kinova/aarch64/`). The arm is exposed as a single
//! `SensorDriver` named `kinova_arm` — there is no per-joint addressing
//! because the SDK only accepts whole-arm `TrajectoryPoint` commands.
//!
//! The Capra arm uses a custom 6DOF spherical configuration; the SDK's
//! cartesian features rely on stock kinematics and would return garbage, so
//! they are deliberately not bound. Direct angular (joint) control only.
//!
//! # Configuration (env vars)
//!
//! - `KINOVA_LIB_DIR`         — directory containing the .so files. Default
//!   resolves to `<crate-root>/vendor/kinova/aarch64` at compile time, so the
//!   binary works regardless of CWD; override for deployed builds where the
//!   binary has moved away from the source tree.
//! - `KINOVA_LOCAL_IP`        — *required*, this Pi on the arm subnet (Roboguard: `192.168.2.37`)
//! - `KINOVA_LOCAL_SUBNET`    — default `255.255.255.0`
//! - `KINOVA_ROBOT_IP`        — default `192.168.2.50`. The Capra arm is sometimes
//!   reconfigured to `192.168.2.5` instead — override via env if discovery fails.
//! - `KINOVA_LOCAL_CMD_PORT`  — default `25015`
//! - `KINOVA_LOCAL_BCAST_PORT`— default `25025`
//! - `KINOVA_ROBOT_PORT`      — default `55000`
//! - `KINOVA_RX_TIMEOUT_MS`   — default `1000`
//!
//! On Capra Roboguard the .so files have soname-relative dependencies on
//! each other; we load `EthCommLayerUbuntu.so` first under `RTLD_GLOBAL` so
//! its SONAME is registered before the command layer's internal `dlopen`
//! runs.

pub mod arm;
pub mod ffi;
pub mod sdk;
pub mod state;
pub mod worker;

use std::net::Ipv4Addr;
use std::path::PathBuf;
use std::str::FromStr;
use std::sync::mpsc;
use std::sync::{Arc, RwLock};

use arm::KinovaArm;
use ffi::EthernetCommConfig;
use sdk::{KinovaSdk, SdkError};
use state::KinovaState;

#[derive(Debug, thiserror::Error)]
pub enum ConnectError {
    #[error("missing required env var {0}")]
    MissingEnv(&'static str),
    #[error("invalid IP/port for {0}: {1}")]
    InvalidConfig(&'static str, String),
    #[error(transparent)]
    Sdk(#[from] SdkError),
}

/// Read `EthernetCommConfig` from environment variables.
fn config_from_env() -> Result<EthernetCommConfig, ConnectError> {
    let local_ip_s = std::env::var("KINOVA_LOCAL_IP")
        .map_err(|_| ConnectError::MissingEnv("KINOVA_LOCAL_IP"))?;
    let subnet_s =
        std::env::var("KINOVA_LOCAL_SUBNET").unwrap_or_else(|_| "255.255.255.0".to_string());
    let robot_ip_s =
        std::env::var("KINOVA_ROBOT_IP").unwrap_or_else(|_| "192.168.2.50".to_string());

    let local_cmd_port = parse_port("KINOVA_LOCAL_CMD_PORT", 25015)?;
    let local_bcast_port = parse_port("KINOVA_LOCAL_BCAST_PORT", 25025)?;
    let robot_port = parse_port("KINOVA_ROBOT_PORT", 55000)?;
    // Default is intentionally aggressive: with a 75 Hz velocity stream a
    // single 1000 ms stall cascades into 75+ queued packets and several
    // seconds of perceived lag. 100 ms is well above the typical Ethernet
    // RTT (<5 ms) so legitimate replies still arrive on time, while a lost
    // ack only costs one tick of latency before the SDK gives up.
    let rx_timeout: u32 = std::env::var("KINOVA_RX_TIMEOUT_MS")
        .ok()
        .map(|s| s.parse())
        .transpose()
        .map_err(|e: std::num::ParseIntError| {
            ConnectError::InvalidConfig("KINOVA_RX_TIMEOUT_MS", e.to_string())
        })?
        .unwrap_or(100);

    Ok(EthernetCommConfig {
        localIpAddress: ip_to_inet_addr("KINOVA_LOCAL_IP", &local_ip_s)? as _,
        subnetMask: ip_to_inet_addr("KINOVA_LOCAL_SUBNET", &subnet_s)? as _,
        robotIpAddress: ip_to_inet_addr("KINOVA_ROBOT_IP", &robot_ip_s)? as _,
        localCmdport: local_cmd_port,
        localBcastPort: local_bcast_port,
        robotPort: robot_port,
        rxTimeOutInMs: rx_timeout as _,
    })
}

fn parse_port(name: &'static str, default: u16) -> Result<u16, ConnectError> {
    match std::env::var(name) {
        Err(_) => Ok(default),
        Ok(s) => s
            .parse()
            .map_err(|e: std::num::ParseIntError| ConnectError::InvalidConfig(name, e.to_string())),
    }
}

/// Convert a dotted-quad IP into the same 32-bit value `inet_addr()` would
/// produce: bytes are stored in network order so reading the low byte first
/// gives octet 0. Encoded as `u32::from_le_bytes(octets)`.
fn ip_to_inet_addr(name: &'static str, s: &str) -> Result<u32, ConnectError> {
    let addr = Ipv4Addr::from_str(s.trim())
        .map_err(|e| ConnectError::InvalidConfig(name, e.to_string()))?;
    Ok(u32::from_le_bytes(addr.octets()))
}

/// Connect to the Kinova arm and return a ready-to-register `KinovaArm`.
///
/// Steps performed:
/// 1. Load the two .so files (comm layer first, RTLD_GLOBAL).
/// 2. `Ethernet_InitAPI` + `Ethernet_InitEthernetAPI`.
/// 3. `Ethernet_GetDevices`, pick the first one returned.
/// 4. `Ethernet_SetActiveDevice`.
/// 5. `Ethernet_StartControlAPI` + `Ethernet_SetAngularControl`.
/// 6. Spawn the worker thread; return the driver.
///
/// On any failure the SDK is dropped (auto-close via `KinovaSdk::drop`).
pub fn connect() -> Result<KinovaArm, ConnectError> {
    // Default to a path relative to the crate root (baked at compile time)
    // so `cargo run` works from any CWD. Override via env for deployed builds.
    let lib_dir: PathBuf = std::env::var("KINOVA_LIB_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/vendor/kinova/aarch64"))
        });

    tracing::info!(?lib_dir, "loading Kinova SDK libraries");
    let sdk = KinovaSdk::load(&lib_dir)?;

    let cfg = config_from_env()?;
    tracing::info!(
        local_cmd_port = cfg.localCmdport,
        robot_port = cfg.robotPort,
        rx_timeout_ms = cfg.rxTimeOutInMs,
        "initialising Kinova Ethernet API"
    );
    sdk.init_ethernet(cfg)?;

    // GetDevices returns an empty cached list right after init; the SDK only
    // populates the list when RefresDevicesList runs a fresh broadcast scan.
    sdk.refresh_devices_list()?;

    let devices = sdk.get_devices()?;
    let device = devices[0];
    let serial = cstr_to_string(&device.SerialNumber);
    let model = cstr_to_string(&device.Model);
    tracing::info!(
        serial = %serial,
        model = %model,
        device_type = device.DeviceType,
        firmware = format!("{}.{}.{}", device.VersionMajor, device.VersionMinor, device.VersionRelease),
        "Kinova device found"
    );

    sdk.set_active_device(device)?;
    sdk.start_control()?;
    sdk.set_angular_control()?;
    // The arm's 2000-entry trajectory FIFO is *not* cleared on
    // `StartControlAPI` — stale velocity setpoints from a previous run will
    // sit at the front of the queue and get processed at 100 Hz before the
    // arm reaches our actual setpoints, manifesting as several seconds of
    // perceived lag at the operator end. Wipe it on every connect.
    if let Err(e) = sdk.erase_all_trajectories() {
        tracing::warn!(error = %e, "Kinova: initial EraseAllTrajectories failed (continuing)");
    }

    let state = Arc::new(RwLock::new(KinovaState::default()));
    let (cmd_tx, cmd_rx) = mpsc::channel();

    {
        let state = state.clone();
        std::thread::Builder::new()
            .name("kinova-worker".into())
            .spawn(move || worker::run(sdk, cmd_rx, state))
            .expect("spawn kinova-worker thread");
    }

    Ok(KinovaArm::new(state, cmd_tx))
}

/// Decode a fixed-length C string buffer (NUL-padded, possibly non-UTF-8) for
/// logging.
fn cstr_to_string(buf: &[u8]) -> String {
    let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    String::from_utf8_lossy(&buf[..len]).into_owned()
}
