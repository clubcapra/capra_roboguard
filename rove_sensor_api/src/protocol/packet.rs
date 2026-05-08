use crate::core::error::DriverError;

/// UDP message types.
///
/// Wire format (little-endian):
/// ```text
/// ┌──────────┬──────────┬───────────┬──────────────────────┐
/// │ version  │ msg_type │ seq_num   │ payload (JSON bytes)  │
/// │ 1 byte   │ 1 byte   │ 2 bytes   │ variable length       │
/// └──────────┴──────────┴───────────┴──────────────────────┘
/// ```
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    // --- Data subscription (data port) ---
    /// Client -> Server: subscribe to sensor data pushes.
    /// Payload (optional): `{"interval_ms": 100}` to override push rate.
    Subscribe = 0x01,
    /// Client -> Server: stop receiving data pushes.
    Unsubscribe = 0x02,
    /// Server -> Client: pushed sensor data JSON (sent continuously to subscribers).
    Data = 0x03,
    /// Server -> Client: acknowledgement for subscribe/unsubscribe.
    SubscribeAck = 0x04,

    // --- Commands (command port) ---
    /// Client -> Server: command with JSON payload.
    /// For stream-mode drivers, the client sends this continuously at the driver's interval.
    Command = 0x10,
    /// Server -> Client: command result JSON.
    CommandAck = 0x11,

    // --- General ---
    /// Server -> Client: error message.
    Error = 0xFF,
}

impl MessageType {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(Self::Subscribe),
            0x02 => Some(Self::Unsubscribe),
            0x03 => Some(Self::Data),
            0x04 => Some(Self::SubscribeAck),
            0x10 => Some(Self::Command),
            0x11 => Some(Self::CommandAck),
            0xFF => Some(Self::Error),
            _ => None,
        }
    }
}

pub const PROTOCOL_VERSION: u8 = 1;
pub const HEADER_SIZE: usize = 4;

/// A UDP packet with a 4-byte header and JSON payload.
#[derive(Debug, Clone)]
pub struct Packet {
    pub version: u8,
    pub msg_type: MessageType,
    pub seq_num: u16,
    pub payload: Vec<u8>,
}

impl Packet {
    pub fn new(msg_type: MessageType, seq_num: u16, payload: Vec<u8>) -> Self {
        Self {
            version: PROTOCOL_VERSION,
            msg_type,
            seq_num,
            payload,
        }
    }

    pub fn subscribe(seq: u16) -> Self {
        Self::new(MessageType::Subscribe, seq, Vec::new())
    }

    pub fn unsubscribe(seq: u16) -> Self {
        Self::new(MessageType::Unsubscribe, seq, Vec::new())
    }

    pub fn data(seq: u16, data: &serde_json::Value) -> Self {
        Self::new(
            MessageType::Data,
            seq,
            serde_json::to_vec(data).unwrap_or_default(),
        )
    }

    pub fn subscribe_ack(seq: u16, msg: &str) -> Self {
        Self::new(
            MessageType::SubscribeAck,
            seq,
            serde_json::to_vec(&serde_json::json!({"status": msg})).unwrap_or_default(),
        )
    }

    pub fn command(seq: u16, payload: &serde_json::Value) -> Self {
        Self::new(
            MessageType::Command,
            seq,
            serde_json::to_vec(payload).unwrap_or_default(),
        )
    }

    pub fn command_ack(seq: u16, result: &serde_json::Value) -> Self {
        Self::new(
            MessageType::CommandAck,
            seq,
            serde_json::to_vec(result).unwrap_or_default(),
        )
    }

    pub fn error(seq: u16, message: &str) -> Self {
        Self::new(
            MessageType::Error,
            seq,
            serde_json::to_vec(&serde_json::json!({"error": message})).unwrap_or_default(),
        )
    }

    /// Encode to wire format.
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(HEADER_SIZE + self.payload.len());
        buf.push(self.version);
        buf.push(self.msg_type as u8);
        buf.extend_from_slice(&self.seq_num.to_le_bytes());
        buf.extend_from_slice(&self.payload);
        buf
    }

    /// Decode from wire format.
    pub fn decode(data: &[u8]) -> Result<Self, DriverError> {
        if data.len() < HEADER_SIZE {
            return Err(DriverError::CommandFailed(format!(
                "packet too short: {} bytes, need at least {}",
                data.len(),
                HEADER_SIZE
            )));
        }

        let version = data[0];
        if version != PROTOCOL_VERSION {
            return Err(DriverError::CommandFailed(format!(
                "unsupported protocol version: {version}"
            )));
        }

        let msg_type = MessageType::from_u8(data[1]).ok_or_else(|| {
            DriverError::CommandFailed(format!("unknown message type: 0x{:02X}", data[1]))
        })?;

        let seq_num = u16::from_le_bytes([data[2], data[3]]);
        let payload = data[HEADER_SIZE..].to_vec();

        Ok(Self {
            version,
            msg_type,
            seq_num,
            payload,
        })
    }

    /// Parse payload as JSON value.
    pub fn json_payload(&self) -> Result<serde_json::Value, DriverError> {
        if self.payload.is_empty() {
            return Ok(serde_json::Value::Null);
        }
        Ok(serde_json::from_slice(&self.payload)?)
    }
}
