//! Wire transport to rove_sensor_api: packet codec, port discovery, telemetry
//! subscription, and command sending. Identical bytes against sim or real robot.

pub mod command;
pub mod discover;
pub mod packet;
pub mod telemetry;
