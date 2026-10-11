//! Wire transport to HALo: packet codec, port discovery, telemetry
//! subscription, and command sending. Identical bytes against sim or real robot.

pub mod command;
pub mod discover;
pub mod packet;
pub mod telemetry;
