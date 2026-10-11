//! Hardware factories read instance configuration on each recovery attempt.
use crate::{
    core::{
        config::{self, DriverConfig},
        driver::SensorDriver,
        supervisor::{Factory, Supervisor},
    },
    drivers,
};
use drivers::{
    kinova::config::KinovaConfig, livox::config::LivoxConfig, odrive::config::OdriveConfig,
    robotiq::config::RobotiqConfig, vectornav::config::VectorNavConfig,
};
use std::sync::Arc;

/// Resources and startup hooks owned by the compiled driver catalogue. Adding
/// hardware changes drivers and this catalogue, never the shared HTTP/core layer.
pub struct DriverCatalog {
    endpoints: drivers::odrive::endpoints::SharedEndpointMap,
}

impl Default for DriverCatalog {
    fn default() -> Self {
        Self {
            endpoints: drivers::odrive::endpoints::new_shared(),
        }
    }
}

impl DriverCatalog {
    pub fn reserved_udp_ports(&self) -> std::collections::BTreeSet<u16> {
        configured_ports::<KinovaConfig>("kinova.toml")
            .into_iter()
            .chain(configured_ports::<LivoxConfig>("livox.toml"))
            .collect()
    }

    pub fn http_extensions(&self) -> Vec<crate::http::extension::DriverHttpExtension> {
        vec![drivers::odrive::http::extension(self.endpoints.clone())]
    }

    pub fn start(&self, manager: &Arc<Supervisor>) -> Result<(), String> {
        start(manager, self.endpoints.clone())
    }
}

fn configured_ports<T: DriverConfig>(filename: &str) -> Vec<u16> {
    config::load_driver_config::<T>(filename)
        .ok()
        .flatten()
        .and_then(|value| value.reserved_udp_ports().ok())
        .unwrap_or_default()
}

fn require<T: DriverConfig>(filename: &str) -> Result<T, String> {
    config::load_driver_config(filename)
        .map_err(|e| e.to_string())?
        .ok_or_else(|| format!("disabled: no {filename}"))
}

/// One registration also wires validated connection editing. Its rules and
/// internal UDP reservations come from the driver's configuration type.
pub fn configure<T: DriverConfig + 'static>(
    manager: &Supervisor,
    id: &str,
    filename: &'static str,
) -> Result<(), String> {
    manager.configure(
        id,
        filename,
        Arc::new(move |text| {
            let path = config::config_path(filename).map_err(|e| e.to_string())?;
            config::parse_driver_config::<T>(text, &path)
                .map_err(|e| e.to_string())?
                .reserved_udp_ports()
        }),
    )
}

pub fn start(
    manager: &Arc<Supervisor>,
    endpoints: drivers::odrive::endpoints::SharedEndpointMap,
) -> Result<(), String> {
    // Reserve configured identities before asynchronous connection attempts.
    // VectorNav's original ID survives edits to its physical serial path.
    let old_id = manager
        .registry
        .assignments()
        .sensors
        .keys()
        .find(|id| id.starts_with("vectornav_"))
        .cloned();
    let vn_id = old_id.or_else(|| {
        VectorNavConfig::load().ok().flatten().map(|c| {
            format!(
                "vectornav_{}",
                c.port.rsplit('/').next().unwrap_or("serial")
            )
        })
    });
    if let Some(id) = &vn_id {
        manager.registry.reserve(id).map_err(|e| e.to_string())?;
    }
    for (kind, id) in [("kinova", "kinova_arm"), ("robotiq", "robotiq_gripper")] {
        if config::config_path(&format!("{kind}.toml"))
            .map_err(|e| e.to_string())?
            .exists()
        {
            manager.registry.reserve(id).map_err(|e| e.to_string())?;
        }
    }
    if let Ok(Some(c)) = drivers::livox::config::LivoxConfig::load(
        &config::config_path("livox.toml").map_err(|e| e.to_string())?,
    ) {
        for id in c.lidar_ids() {
            manager
                .registry
                .reserve(&format!("livox_{id}"))
                .map_err(|e| e.to_string())?;
        }
    }

    if let Ok(Some(c)) =
        config::load_driver_config::<drivers::radiacode::config::RadiacodeConfig>("radiacode.toml")
    {
        manager.registry.reserve(&c.id).map_err(|e| e.to_string())?;
    }
    manager.add(
        "radiacode",
        Arc::new(|| {
            Box::pin(async {
                let c: drivers::radiacode::config::RadiacodeConfig = require("radiacode.toml")?;
                let driver = drivers::radiacode::connect(&c);
                Ok(vec![(c.id, Arc::new(driver) as Arc<dyn SensorDriver>)])
            })
        }),
    )?;
    configure::<drivers::radiacode::config::RadiacodeConfig>(
        manager,
        "radiacode",
        "radiacode.toml",
    )?;
    let registry = manager.registry.clone();
    let vectornav: Factory = Arc::new(move || {
        let id = vn_id.clone().or_else(|| {
            registry
                .assignments()
                .sensors
                .keys()
                .find(|id| id.starts_with("vectornav_"))
                .cloned()
        });
        let registry = registry.clone();
        Box::pin(async move {
            let c: VectorNavConfig = require("vectornav.toml")?;
            let id = id.unwrap_or_else(|| {
                format!(
                    "vectornav_{}",
                    c.port.rsplit('/').next().unwrap_or("serial")
                )
            });
            registry.reserve(&id).map_err(|e| e.to_string())?;
            let d = drivers::vectornav::connect(&c.port, c.baudrate)
                .await
                .map_err(|e| e.to_string())?;
            Ok(vec![(id, Arc::new(d) as Arc<dyn SensorDriver>)])
        })
    });
    manager.add("vectornav", vectornav)?;
    manager.add(
        "kinova",
        Arc::new(|| {
            Box::pin(async {
                let c: KinovaConfig = require("kinova.toml")?;
                let d = tokio::task::spawn_blocking(move || drivers::kinova::connect(&c))
                    .await
                    .map_err(|e| e.to_string())?
                    .map_err(|e| e.to_string())?;
                Ok(vec![(
                    "kinova_arm".into(),
                    Arc::new(d) as Arc<dyn SensorDriver>,
                )])
            })
        }),
    )?;
    manager.add(
        "robotiq",
        Arc::new(|| {
            Box::pin(async {
                let c: RobotiqConfig = require("robotiq.toml")?;
                let d = drivers::robotiq::connect(&c)
                    .await
                    .map_err(|e| e.to_string())?;
                Ok(vec![(
                    "robotiq_gripper".into(),
                    Arc::new(d) as Arc<dyn SensorDriver>,
                )])
            })
        }),
    )?;
    manager.add(
        "odrive",
        Arc::new(move || {
            let endpoints = endpoints.clone();
            Box::pin(async move {
                let c = config::load_driver_config::<OdriveConfig>("odrive.toml")
                    .map_err(|e| e.to_string())?
                    .unwrap_or_default();
                c.validate()?;
                let (nodes, _) = drivers::odrive::discover_nodes_using_map(
                    &c.interface,
                    std::time::Duration::from_millis(c.discovery_ms),
                    Default::default(),
                    endpoints,
                )
                .await
                .map_err(|e| e.to_string())?;
                Ok(nodes
                    .into_iter()
                    .map(|n| (n.id().into(), Arc::new(n) as Arc<dyn SensorDriver>))
                    .collect())
            })
        }),
    )?;
    manager.add(
        "livox",
        Arc::new(|| {
            Box::pin(async {
                let c: LivoxConfig = require("livox.toml")?;
                let ds = tokio::task::spawn_blocking(move || drivers::livox::connect(c))
                    .await
                    .map_err(|e| e.to_string())?
                    .map_err(|e| e.to_string())?;
                Ok(ds
                    .into_iter()
                    .map(|d| (d.id().into(), Arc::new(d) as Arc<dyn SensorDriver>))
                    .collect())
            })
        }),
    )?;
    configure::<VectorNavConfig>(manager, "vectornav", "vectornav.toml")?;
    configure::<RobotiqConfig>(manager, "robotiq", "robotiq.toml")?;
    configure::<KinovaConfig>(manager, "kinova", "kinova.toml")?;
    configure::<OdriveConfig>(manager, "odrive", "odrive.toml")?;
    configure::<LivoxConfig>(manager, "livox", "livox.toml")?;
    Ok(())
}
