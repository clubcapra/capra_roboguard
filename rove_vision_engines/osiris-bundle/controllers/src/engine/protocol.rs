//! Binary framing protocol for Rust ↔ Python engine communication.
//!
//! Wire format:
//!   [4 bytes: payload length (big-endian u32)] [1 byte: type tag] [payload]
//!
//! Type tags:
//!   0x01 = control (UTF-8 JSON)
//!   0x02 = frame  (36-byte feed_id + raw BGR pixel data)

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UnixStream;

use crate::error::OsirisError;

pub const MSG_CONTROL: u8 = 0x01;
pub const MSG_FRAME: u8 = 0x02;
pub const FEED_ID_LEN: usize = 36;

/// A message received from an engine process.
#[derive(Debug)]
pub enum ProtocolMessage {
    Control(serde_json::Value),
}

/// Send a JSON control message to the engine.
pub async fn send_control(
    stream: &mut UnixStream,
    msg: &serde_json::Value,
) -> Result<(), OsirisError> {
    let payload = serde_json::to_vec(msg)
        .map_err(|e| OsirisError::ProtocolError(format!("JSON serialize: {e}")))?;

    let mut header = [0u8; 5];
    header[..4].copy_from_slice(&(payload.len() as u32).to_be_bytes());
    header[4] = MSG_CONTROL;

    stream
        .write_all(&header)
        .await
        .map_err(|e| OsirisError::ProtocolError(format!("write header: {e}")))?;
    stream
        .write_all(&payload)
        .await
        .map_err(|e| OsirisError::ProtocolError(format!("write payload: {e}")))?;
    stream
        .flush()
        .await
        .map_err(|e| OsirisError::ProtocolError(format!("flush: {e}")))?;

    Ok(())
}

/// Send a raw video frame to the engine.
///
/// The payload is: 36-byte feed_id (UTF-8, zero-padded) + raw BGR frame bytes.
pub async fn send_frame(
    stream: &mut UnixStream,
    feed_id: &str,
    frame: &[u8],
) -> Result<(), OsirisError> {
    let payload_len = FEED_ID_LEN + frame.len();

    let mut header = [0u8; 5];
    header[..4].copy_from_slice(&(payload_len as u32).to_be_bytes());
    header[4] = MSG_FRAME;

    // Prepare zero-padded feed_id
    let mut feed_id_bytes = [0u8; FEED_ID_LEN];
    let id_bytes = feed_id.as_bytes();
    let copy_len = id_bytes.len().min(FEED_ID_LEN);
    feed_id_bytes[..copy_len].copy_from_slice(&id_bytes[..copy_len]);

    stream
        .write_all(&header)
        .await
        .map_err(|e| OsirisError::ProtocolError(format!("write header: {e}")))?;
    stream
        .write_all(&feed_id_bytes)
        .await
        .map_err(|e| OsirisError::ProtocolError(format!("write feed_id: {e}")))?;
    stream
        .write_all(frame)
        .await
        .map_err(|e| OsirisError::ProtocolError(format!("write frame: {e}")))?;
    stream
        .flush()
        .await
        .map_err(|e| OsirisError::ProtocolError(format!("flush: {e}")))?;

    Ok(())
}

/// Receive one message from the engine.
pub async fn recv_message(stream: &mut UnixStream) -> Result<ProtocolMessage, OsirisError> {
    let mut header = [0u8; 5];
    stream
        .read_exact(&mut header)
        .await
        .map_err(|e| OsirisError::ProtocolError(format!("read header: {e}")))?;

    let length = u32::from_be_bytes([header[0], header[1], header[2], header[3]]) as usize;
    let msg_type = header[4];

    let mut payload = vec![0u8; length];
    if length > 0 {
        stream
            .read_exact(&mut payload)
            .await
            .map_err(|e| OsirisError::ProtocolError(format!("read payload: {e}")))?;
    }

    match msg_type {
        MSG_CONTROL => {
            let value: serde_json::Value = serde_json::from_slice(&payload)
                .map_err(|e| OsirisError::ProtocolError(format!("JSON parse: {e}")))?;
            Ok(ProtocolMessage::Control(value))
        }
        other => Err(OsirisError::ProtocolError(format!(
            "unexpected message type from engine: 0x{other:02x}"
        ))),
    }
}
