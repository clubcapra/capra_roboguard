//! Sim-backend mode: replace every hardware driver with a per-sensor mock fed by
//! the simulator (`rove_sim`).
//!
//! When `config/sim.toml` is present (or `ROVE_SIM_BACKEND` is set), `main.rs`
//! calls [`start`] instead of probing real hardware. Each sensor keeps its
//! own distinct mock driver type (co-located with its real driver, reusing the
//! real schema), so autonomy sees a byte-identical `HALo` surface —
//! it just happens to be backed by physics instead of serial/CAN/modbus.

/// Stable simulator backend slots. Append adapters; never reorder entries.
pub const BACKEND_CHANNELS: &[&str] = &[
    "vectornav_sim",
    "kinova_arm",
    "robotiq_gripper",
    "odrive_31",
    "odrive_32",
    "odrive_33",
    "odrive_34",
    "odrive_41",
    "odrive_42",
    "odrive_43",
    "odrive_44",
    "pmic",
    "livox_40",
    "livox_41",
    "radiacode_arm",
];

pub mod config;
pub mod control;
pub mod feed;

use config::SimConfig;
use control::{default_control_frame, spawn_control_publisher};

use crate::drivers::kinova::arm::KINOVA_ID;
use crate::drivers::kinova::mock::KinovaMock;
use crate::drivers::odrive::mock::{Axis, OdriveMock};
use crate::drivers::robotiq::gripper::ROBOTIQ_ID;
use crate::drivers::robotiq::mock::RobotiqMock;
use crate::drivers::vectornav::mock::{VectorNavMock, VECTORNAV_SIM_ID};

/// Run the proven simulator mocks through the same readiness and recovery
/// supervisor as physical drivers. Robot-specific composition lives here.
pub fn start(
    manager: &std::sync::Arc<crate::core::supervisor::Supervisor>,
    sim: &SimConfig,
) -> Result<(), String> {
    use crate::core::driver::SensorDriver;
    use std::sync::Arc;
    manager
        .simulated
        .store(true, std::sync::atomic::Ordering::Release);
    let control = default_control_frame();
    manager.keep_task(spawn_control_publisher(
        sim.host.clone(),
        sim.control_port(),
        control.clone(),
    ));
    let identities = [
        VECTORNAV_SIM_ID,
        KINOVA_ID,
        ROBOTIQ_ID,
        "odrive_31",
        "odrive_32",
        "odrive_33",
        "odrive_34",
        "odrive_41",
        "odrive_42",
        "odrive_43",
        "odrive_44",
    ];
    for id in identities {
        manager.registry.reserve(id).map_err(|e| e.to_string())?;
    }
    for (index, id) in identities.into_iter().enumerate() {
        let hardware = Arc::new(crate::drivers::odrive::mock::MockHardwareState::default());
        let sim = sim.clone();
        let control = control.clone();
        let group = match index {
            0 => "vectornav",
            1 => "kinova",
            2 => "robotiq",
            _ => id,
        };
        manager.add(
            group,
            Arc::new(move || {
                let hardware = hardware.clone();
                let sim = sim.clone();
                let control = control.clone();
                Box::pin(async move {
                    let host = sim.host.as_str();
                    let port = sim.backend(id, index as u16);
                    let driver: Arc<dyn SensorDriver> = match index {
                        0 => Arc::new(VectorNavMock::new(host, port)),
                        1 => Arc::new(KinovaMock::with_control(host, port, control)),
                        2 => Arc::new(RobotiqMock::new(host, port, control)),
                        _ => {
                            let (node, axis) = match index {
                                3 => (31, Axis::Track("left")),
                                4 => (32, Axis::Track("right")),
                                5 => (33, Axis::Track("right")),
                                6 => (34, Axis::Track("left")),
                                7 => (41, Axis::Flipper("fl")),
                                8 => (42, Axis::Flipper("fr")),
                                9 => (43, Axis::Flipper("rl")),
                                _ => (44, Axis::Flipper("rr")),
                            };
                            Arc::new(OdriveMock::with_hardware(
                                node,
                                axis,
                                host,
                                port,
                                control,
                                sim.odrive_max_vel_rev_s,
                                hardware,
                            ))
                        }
                    };
                    Ok(vec![(id.into(), driver)])
                })
            }),
        )?;
    }
    for id in ["livox_40", "livox_41"] {
        manager.registry.reserve(id).map_err(|e| e.to_string())?;
    }
    let radiacode_sim = sim.clone();
    let sim = sim.clone();
    manager.add(
        "livox",
        Arc::new(move || {
            let inputs = vec![
                (40, sim.backend("livox_40", 12)),
                (41, sim.backend("livox_41", 13)),
            ];
            Box::pin(async move {
                let drivers = crate::drivers::livox::connect_sim(&inputs, 1000)
                    .await
                    .map_err(|e| e.to_string())?;
                Ok(drivers
                    .into_iter()
                    .map(|d| (d.id().to_owned(), Arc::new(d) as Arc<dyn SensorDriver>))
                    .collect())
            })
        }),
    )?;
    if crate::core::config::config_path("radiacode.toml")
        .map_err(|e| e.to_string())?
        .exists()
    {
        if let Ok(Some(c)) = crate::core::config::load_driver_config::<
            crate::drivers::radiacode::config::RadiacodeConfig,
        >("radiacode.toml")
        {
            manager.registry.reserve(&c.id).map_err(|e| e.to_string())?;
        }
        let sim = radiacode_sim.clone();
        manager.add(
            "radiacode",
            Arc::new(move || {
                let sim = sim.clone();
                Box::pin(async move {
                    let c = crate::core::config::load_driver_config::<
                        crate::drivers::radiacode::config::RadiacodeConfig,
                    >("radiacode.toml")
                    .map_err(|e| e.to_string())?
                    .ok_or("disabled: no radiacode.toml")?;
                    let driver = crate::drivers::radiacode::mock::connect_sim(
                        &c,
                        &sim.host,
                        sim.backend(&c.id, 14),
                    );
                    Ok(vec![(c.id, Arc::new(driver) as Arc<dyn SensorDriver>)])
                })
            }),
        )?;
        crate::drivers::loader::configure::<crate::drivers::radiacode::config::RadiacodeConfig>(
            manager,
            "radiacode",
            "radiacode.toml",
        )?;
    }
    Ok(())
}
