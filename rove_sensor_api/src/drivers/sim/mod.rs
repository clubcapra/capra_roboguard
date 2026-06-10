//! Sim-backend mode: replace every hardware driver with a per-sensor mock fed by
//! the simulator (`rove_sim`).
//!
//! When `config/sim.toml` is present (or `ROVE_SIM_BACKEND` is set), `main.rs`
//! calls [`register_all`] instead of probing real hardware. Each sensor keeps its
//! own distinct mock driver type (co-located with its real driver, reusing the
//! real schema), so autonomy sees a byte-identical `rove_sensor_api` surface —
//! it just happens to be backed by physics instead of serial/CAN/modbus.

pub mod config;
pub mod control;
pub mod feed;

use crate::core::registry::SensorRegistry;
use config::SimConfig;
use control::{default_control_frame, spawn_control_publisher};

use crate::drivers::kinova::arm::KINOVA_ID;
use crate::drivers::kinova::mock::KinovaMock;
use crate::drivers::odrive::mock::{Axis, OdriveMock};
use crate::drivers::robotiq::gripper::ROBOTIQ_ID;
use crate::drivers::robotiq::mock::RobotiqMock;
use crate::drivers::vectornav::mock::{VectorNavMock, VECTORNAV_SIM_ID};

/// Register the full sim-backed mock set, in a deterministic order so the served
/// (autonomy-facing) ports are stable. Also spawns the single control publisher
/// that forwards aggregated commands back to the sim.
///
/// NOTE: only hardware wired to the Pi control board lives here (VectorNav,
/// Kinova, Robotiq, ODrives). The lidars are a SEPARATE subsystem — their point
/// clouds and built-in IMU are published by the sim on their own UDP streams and
/// consumed directly (SLAM/mapping), never through this API.
pub fn register_all(reg: &SensorRegistry, sim: &SimConfig) {
    let host = sim.host.as_str();
    let max_vel = sim.odrive_max_vel_rev_s;

    let control = default_control_frame();
    spawn_control_publisher(sim.host.clone(), sim.control_port(), control.clone());

    // idx MUST match the shared CHANNEL_ORDER (sim publishes telemetry on
    // backend_base + 2*idx; the served 5000+ ports are assigned by the registry in
    // insertion order, independently).
    reg.register(VectorNavMock::new(host, sim.backend(VECTORNAV_SIM_ID, 0)));
    reg.register(KinovaMock::new(host, sim.backend(KINOVA_ID, 1)));
    reg.register(RobotiqMock::new(host, sim.backend(ROBOTIQ_ID, 2), control.clone()));
    // 4 drums (track sides) — per the rove_control_bridge config 31,34=LEFT, 32,33=RIGHT.
    reg.register(OdriveMock::new(31, Axis::Track("left"),  host, sim.backend("odrive_31", 3), control.clone(), max_vel));
    reg.register(OdriveMock::new(32, Axis::Track("right"), host, sim.backend("odrive_32", 4), control.clone(), max_vel));
    reg.register(OdriveMock::new(33, Axis::Track("right"), host, sim.backend("odrive_33", 5), control.clone(), max_vel));
    reg.register(OdriveMock::new(34, Axis::Track("left"),  host, sim.backend("odrive_34", 6), control.clone(), max_vel));
    // 4 flippers (FL/FR/RL/RR -> fl/fr/rl/rr in the control frame).
    reg.register(OdriveMock::new(41, Axis::Flipper("fl"), host, sim.backend("odrive_41", 7), control.clone(), max_vel));
    reg.register(OdriveMock::new(42, Axis::Flipper("fr"), host, sim.backend("odrive_42", 8), control.clone(), max_vel));
    reg.register(OdriveMock::new(43, Axis::Flipper("rl"), host, sim.backend("odrive_43", 9), control.clone(), max_vel));
    reg.register(OdriveMock::new(44, Axis::Flipper("rr"), host, sim.backend("odrive_44", 10), control.clone(), max_vel));

    tracing::info!(host = %sim.host, "sim backend: 11 mock drivers registered (Pi-board hardware only; 8 ODrives: 4 drums + 4 flippers; lidars are separate)");
}
