use std::sync::Arc;
use std::time::Duration;

mod core;
mod drivers;
mod http;
mod protocol;
mod udp;

use core::registry::SensorRegistry;
use drivers::kinova;
use drivers::odrive::{discover_nodes, endpoints::SharedEndpointMap, node::WatchdogConfig};
use drivers::vectornav;
use http::routes::build_router;
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
    // Defaults match the udev rule in README.md (/dev/ttyUSB_VN300 at 115200 baud).
    // Override with VN_PORT and VN_BAUD env vars if needed.
    let vn_port = std::env::var("VN_PORT").unwrap_or_else(|_| "/dev/ttyUSB_VN300".to_string());
    let vn_baud: u32 = std::env::var("VN_BAUD")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(115200);
    match vectornav::connect(&vn_port, vn_baud).await {
        Ok(sensor) => {
            registry.register(sensor);
        }
        Err(e) => {
            tracing::warn!(error = %e, port = vn_port, "VectorNav connect failed — continuing without it");
        }
    }

    // --- Kinova Gen2 6DOF arm (legacy SDK over Ethernet) ---
    // Set KINOVA_LOCAL_IP (and optionally other KINOVA_* vars) to enable.
    // Without KINOVA_LOCAL_IP we skip silently — same graceful pattern as
    // VectorNav. The .so files are vendored at vendor/kinova/aarch64/.
    if std::env::var("KINOVA_LOCAL_IP").is_ok() {
        match kinova::connect() {
            Ok(arm) => {
                registry.register(arm);
            }
            Err(e) => {
                tracing::warn!(error = %e, "Kinova connect failed — continuing without the arm");
            }
        }
    } else {
        tracing::info!(
            "KINOVA_LOCAL_IP not set — skipping Kinova arm. Set KINOVA_LOCAL_IP=192.168.2.37 (Roboguard Pi) to enable."
        );
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

    // --- Start UDP listeners ---
    for (id, driver) in registry.iter_drivers() {
        let (data_port, cmd_port) = registry.ports(&id).unwrap();
        spawn_sensor_udp(driver, data_port, cmd_port).await?;
    }

    // --- Start HTTP server with Scalar UI ---
    let app = build_router(registry.clone(), shared_endpoints);
    let listener = tokio::net::TcpListener::bind("0.0.0.0:8080").await?;

    tracing::info!("Scalar UI:   http://localhost:8080/docs");
    tracing::info!("OpenAPI:     http://localhost:8080/openapi.json");
    tracing::info!("Discover:    http://localhost:8080/discover");
    tracing::info!("{} sensors registered", registry.len());

    axum::serve(listener, app).await?;

    Ok(())
}
