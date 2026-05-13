use std::sync::Arc;
use std::time::Duration;

mod core;
mod drivers;
mod http;
mod logging;
mod protocol;
mod udp;

use std::path::PathBuf;

use core::registry::SensorRegistry;
use core::server_config;
use drivers::kinova::{self, config::KinovaConfig};
use drivers::odrive::{discover_nodes, endpoints::SharedEndpointMap, node::WatchdogConfig};
use drivers::robotiq::{self, config::RobotiqConfig};
use drivers::vectornav::{self, config::VectorNavConfig};
use http::routes::build_router;
use logging::LogManager;
use udp::server::spawn_sensor_udp;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "capra_rove_interface=info".into()),
        )
        .init();

    // --- Sensor Registration ---
    let registry = Arc::new(SensorRegistry::new(5000));

    // --- VectorNav VN-300 ---
    // Configured via config/vectornav.toml. Missing file → driver skipped.
    match VectorNavConfig::load() {
        Ok(Some(cfg)) => match vectornav::connect(&cfg.port, cfg.baudrate).await {
            Ok(sensor) => {
                registry.register(sensor);
            }
            Err(e) => {
                tracing::warn!(error = %e, port = cfg.port, "VectorNav connect failed — continuing without it");
            }
        },
        Ok(None) => {
            tracing::info!("config/vectornav.toml not found — skipping VectorNav.");
        }
        Err(e) => {
            tracing::warn!(error = %e, "VectorNav config load failed — skipping.");
        }
    }

    // --- Kinova Gen2 6DOF arm (legacy SDK over Ethernet) ---
    // Configured via config/kinova.toml. Missing file → driver skipped.
    // The .so files are vendored at vendor/kinova/aarch64/.
    match KinovaConfig::load() {
        Ok(Some(cfg)) => match kinova::connect(&cfg) {
            Ok(arm) => {
                registry.register(arm);
            }
            Err(e) => {
                tracing::warn!(error = %e, "Kinova connect failed — continuing without the arm");
            }
        },
        Ok(None) => {
            tracing::info!("config/kinova.toml not found — skipping Kinova arm.");
        }
        Err(e) => {
            tracing::warn!(error = %e, "Kinova config load failed — skipping.");
        }
    }

    // --- Robotiq 2F-140 gripper (Modbus RTU over USB→RS-485) ---
    // Configured via config/robotiq.toml. Missing file → driver skipped.
    match RobotiqConfig::load() {
        Ok(Some(cfg)) => match robotiq::connect(&cfg).await {
            Ok(gripper) => {
                registry.register(gripper);
            }
            Err(e) => {
                tracing::warn!(error = %e, "Robotiq connect failed — continuing without the gripper");
            }
        },
        Ok(None) => {
            tracing::info!("config/robotiq.toml not found — skipping Robotiq gripper.");
        }
        Err(e) => {
            tracing::warn!(error = %e, "Robotiq config load failed — skipping.");
        }
    }

    // --- ODrive Discovery ---
    // Scans the CAN bus for heartbeat frames for 2 seconds.
    // Each discovered node gets its own UDP ports and Scalar endpoints.
    let odrive_iface = std::env::var("CAN_IFACE").unwrap_or_else(|_| "can0".to_string());
    let watchdog = WatchdogConfig::default(); // 100ms, setpoint-only keepalive

    let shared_endpoints: SharedEndpointMap = match discover_nodes(&odrive_iface, Duration::from_secs(2), watchdog).await {
        Ok((nodes, ep_map)) => {
            for node in nodes {
                registry.register(node);
            }
            ep_map
        }
        Err(e) => {
            tracing::warn!(error = %e, iface = odrive_iface, "ODrive discovery failed — continuing without ODrives");
            drivers::odrive::endpoints::new_shared()
        }
    };

    // --- Logging ---
    // CSV files live under LOG_DIR (default ./logs). Per-sensor files split
    // hourly: logs/<YYYY-MM-DD>/<HH>/<sensor>.csv. Commands go to the same
    // hour bucket as logs/<YYYY-MM-DD>/<HH>/inputs.csv.
    let log_dir: PathBuf = std::env::var("LOG_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("./logs"));
    let log_mgr = Arc::new(LogManager::new(log_dir.clone())?);
    let poll_hz: u64 = std::env::var("LOG_POLL_HZ")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(10);
    let poll_period = Duration::from_millis(1000 / poll_hz.max(1));
    log_mgr
        .clone()
        .spawn_polling(registry.clone(), poll_period);
    tracing::info!(dir = %log_dir.display(), poll_hz, "logging started");

    // --- Start UDP listeners ---
    let server_cfg = server_config::load()?;
    tracing::info!(
        default_push_interval_ms = server_cfg.default_push_interval_ms,
        "server config loaded"
    );
    for (id, driver) in registry.iter_drivers() {
        let (data_port, cmd_port) = registry.ports(&id).unwrap();
        spawn_sensor_udp(
            driver,
            data_port,
            cmd_port,
            log_mgr.clone(),
            server_cfg.default_push_interval_ms,
        )
        .await?;
    }

    // --- Start HTTP server with Scalar UI ---
    let app = build_router(registry.clone(), shared_endpoints, log_mgr.clone());
    let listener = tokio::net::TcpListener::bind("0.0.0.0:8080").await?;

    tracing::info!("Scalar UI:   http://localhost:8080/docs");
    tracing::info!("OpenAPI:     http://localhost:8080/openapi.json");
    tracing::info!("Discover:    http://localhost:8080/discover");
    tracing::info!("{} sensors registered", registry.len());

    axum::serve(
        listener,
        app.into_make_service_with_connect_info::<std::net::SocketAddr>(),
    )
    .await?;

    Ok(())
}
