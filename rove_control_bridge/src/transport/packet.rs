//! rove_sensor_api UDP wire codec.
//!
//! Mirrors `rove_sensor_api/src/protocol/packet.rs` and the sim's
//! `rove_sim/rove_sim/transport/packet.py`:
//!
//! ```text
//! [ version:1B=0x01 | msg_type:1B | seq:u16 LE | JSON payload (UTF-8) ]
//! ```

use anyhow::{bail, Result};
use serde_json::Value;

pub const VERSION: u8 = 0x01;
pub const HEADER_LEN: usize = 4;

// Message types we use.
pub const MSG_SUBSCRIBE: u8 = 0x01;
pub const MSG_DATA: u8 = 0x03;
pub const MSG_COMMAND: u8 = 0x10;

/// Encode a frame: header + JSON payload.
pub fn encode(msg_type: u8, seq: u16, payload: &Value) -> Vec<u8> {
    let json = serde_json::to_vec(payload).expect("payload serialises");
    let mut buf = Vec::with_capacity(HEADER_LEN + json.len());
    buf.push(VERSION);
    buf.push(msg_type);
    buf.extend_from_slice(&seq.to_le_bytes());
    buf.extend_from_slice(&json);
    buf
}

/// Decode a frame -> (msg_type, seq, payload). Errors on a short/garbled frame.
pub fn decode(buf: &[u8]) -> Result<(u8, u16, Value)> {
    if buf.len() < HEADER_LEN {
        bail!("frame too short: {} bytes", buf.len());
    }
    if buf[0] != VERSION {
        bail!("bad version byte: 0x{:02x}", buf[0]);
    }
    let msg_type = buf[1];
    let seq = u16::from_le_bytes([buf[2], buf[3]]);
    let payload = if buf.len() > HEADER_LEN {
        serde_json::from_slice(&buf[HEADER_LEN..])?
    } else {
        Value::Null
    };
    Ok((msg_type, seq, payload))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn roundtrip() {
        let p = json!({"axis_state": 8, "input_vel": 25.0});
        let bytes = encode(MSG_COMMAND, 7, &p);
        assert_eq!(bytes[0], VERSION);
        assert_eq!(bytes[1], MSG_COMMAND);
        assert_eq!(u16::from_le_bytes([bytes[2], bytes[3]]), 7);
        let (mt, seq, val) = decode(&bytes).unwrap();
        assert_eq!(mt, MSG_COMMAND);
        assert_eq!(seq, 7);
        assert_eq!(val, p);
    }

    #[test]
    fn rejects_short_and_bad_version() {
        assert!(decode(&[0x01, 0x03]).is_err());
        assert!(decode(&[0x99, 0x03, 0, 0]).is_err());
    }
}
