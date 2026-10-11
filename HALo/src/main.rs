use halo::{
    core::{
        config,
        registry::SensorRegistry,
        server_config,
        supervisor::{RetryPolicy, Supervisor},
    },
    drivers, http,
    logging::LogManager,
};
use std::{collections::BTreeSet, path::PathBuf, sync::Arc, time::Duration};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    if std::env::args().nth(1).as_deref() == Some("--set-password") {
        use std::io::{self, BufRead};
        eprintln!("Read password from stdin (12..1024 bytes):");
        let mut password = String::new();
        io::stdin().lock().read_line(&mut password)?;
        let password = password.trim_end_matches(['\r', '\n']);
        http::auth::set_password(password).map_err(std::io::Error::other)?;
        eprintln!("Password hash saved. Restart HALo to load it.");
        return Ok(());
    }
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "halo=info".into()),
        )
        .init();
    let server = server_config::load()?;
    if !(10..=60_000).contains(&server.default_push_interval_ms) {
        return Err("default push interval must be 10..60000ms".into());
    }
    let policy = config::load_optional::<RetryPolicy>("recovery.toml")?.unwrap_or_default();
    policy.validate().map_err(std::io::Error::other)?;
    let sim = drivers::sim::config::SimConfig::resolve()?;
    let catalog = drivers::loader::DriverCatalog::default();
    let ports = match &sim {
        Some(sim) => sim.ports.clone(),
        None => config::load_optional::<drivers::sim::config::PortsFile>("ports.toml")?
            .unwrap_or_default(),
    };
    if server.http_port == 0 || ports.served_base == 0 || ports.backend_base == 0 {
        return Err("invalid server or simulator port range".into());
    }
    let mut reserved = BTreeSet::new();
    if let Some(sim) = &sim {
        reserved.extend(sim.reserved_udp_ports()?);
    }
    if sim.is_none() {
        reserved.extend(catalog.reserved_udp_ports());
    }
    // Keep simulator and hardware reservations separate; identities may overlap
    // while their robot-instance port maps differ.
    let path = config::config_path(if sim.is_some() {
        "ports.sim.json"
    } else {
        "ports.hardware.json"
    })?;
    let registry = Arc::new(SensorRegistry::persistent(
        ports.served_base,
        Some(path),
        reserved,
    )?);
    // Explicit TOML pins seed a new instance. Once commissioned, the saved
    // assignments are authoritative and UI edits survive every restart.
    for (id, pin) in &ports.overrides {
        if registry.ports(id).is_none() {
            if let Some(data) = pin.served_data {
                registry.set_ports(
                    id,
                    data,
                    pin.served_cmd
                        .unwrap_or(data.checked_add(1).ok_or("port overflow")?),
                )?;
            } else if let Some(command) = pin.served_cmd {
                let (data, _) = registry.reserve(id)?;
                registry.set_ports(id, data, command)?;
            }
        }
    }
    let logs = Arc::new(LogManager::new(
        std::env::var("LOG_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("logs")),
    )?);
    let poll_hz = std::env::var("LOG_POLL_HZ")
        .ok()
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(10);
    logs.clone().spawn_polling(
        registry.clone(),
        Duration::from_millis((1000 / poll_hz.max(1)).max(1)),
    );
    let manager = Supervisor::new(
        registry.clone(),
        logs.clone(),
        server.default_push_interval_ms,
        policy,
    );
    let auth = http::auth::Auth::load().map_err(std::io::Error::other)?;
    let listener = tokio::net::TcpListener::bind(("0.0.0.0", server.http_port)).await?;
    if let Some(sim) = sim {
        tracing::warn!("Simulator adapter enabled; physical hardware probing disabled");
        drivers::sim::start(&manager, &sim).map_err(std::io::Error::other)?;
    } else {
        catalog.start(&manager).map_err(std::io::Error::other)?;
    }
    let app = http::routes::build_router(registry, catalog.http_extensions(), logs)
        .merge(http::auth::router(auth.clone()))
        .merge(http::management::router(manager, auth));
    tracing::info!(
        port = server.http_port,
        "HALo console ready; discovery and OpenAPI update as hardware becomes ready"
    );
    axum::serve(
        listener,
        app.into_make_service_with_connect_info::<std::net::SocketAddr>(),
    )
    .await?;
    Ok(())
}
