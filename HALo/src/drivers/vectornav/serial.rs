//! Serial transport for the VectorNav driver.
//!
//! Owns a `tokio_serial::SerialStream` split into a buffered reader (for the
//! background async-message loop) and a `Mutex<WriteHalf>` shared by the
//! `SensorDriver` for command sends.
//!
//! The read loop accepts both ASCII messages (`$VN…\r\n`) and binary frames
//! (`0xFA …`) on the same wire — it dispatches by inspecting the first byte
//! of each fresh frame. ASCII responses to commands and binary data frames
//! can therefore interleave without confusing each other.

use std::io;
use std::sync::{Arc, RwLock};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use tokio::io::{AsyncReadExt, AsyncWriteExt, BufReader, ReadHalf, WriteHalf};
use tokio::sync::Mutex;
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use tokio_util::sync::CancellationToken;

use super::binary::{self, FrameStep};
use super::protocol::{parse_line, ParsedMessage};
use super::state::VectorNavState;

/// Maximum bytes for one ASCII line. VN messages are well under 256 bytes;
/// 1024 protects against runaway garbage on the line.
const MAX_LINE_BYTES: usize = 1024;

/// Hard cap on the read buffer. Cuts off pathological growth (e.g. an ASCII
/// `$` that never terminates because the wire turned to noise).
const MAX_BUFFER_BYTES: usize = 16 * 1024;

/// Shared write handle: `Mutex` so multiple commands can be serialised through
/// the same serial port without interleaving bytes.
pub type SerialWriter = Arc<Mutex<WriteHalf<SerialStream>>>;

/// Open the serial port and return the split halves wrapped for shared use.
pub fn open(port_name: &str, baudrate: u32) -> io::Result<(ReadHalf<SerialStream>, SerialWriter)> {
    let stream = tokio_serial::new(port_name, baudrate)
        .timeout(Duration::from_millis(50))
        .open_native_async()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    let (rd, wr) = tokio::io::split(stream);
    Ok((rd, Arc::new(Mutex::new(wr))))
}

/// Send a fully-framed VN command (already including `\r\n`) over the writer.
pub async fn send_command(writer: &SerialWriter, cmd: &str) -> io::Result<()> {
    let mut guard = writer.lock().await;
    guard.write_all(cmd.as_bytes()).await?;
    guard.flush().await
}

/// Background read loop: accumulate bytes and drain whole frames.
///
/// Cancels cleanly when `cancel` fires. Recoverable errors (timeout, partial
/// read) loop forever; unrecoverable ones (port closed, EOF) end the task.
pub async fn run_read_loop(
    read_half: ReadHalf<SerialStream>,
    state: Arc<RwLock<VectorNavState>>,
    cancel: CancellationToken,
) {
    let mut reader = BufReader::with_capacity(2048, read_half);
    let mut buf: Vec<u8> = Vec::with_capacity(4096);
    let mut tmp = [0u8; 1024];

    loop {
        let n = tokio::select! {
            r = reader.read(&mut tmp) => match r {
                Ok(0) => {
                    tracing::warn!("VectorNav serial port reached EOF — read loop exiting");
                    return;
                }
                Ok(n) => n,
                Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
                Err(e) => {
                    tracing::warn!(error = %e, "VectorNav serial read error — backing off 100ms");
                    tokio::time::sleep(Duration::from_millis(100)).await;
                    continue;
                }
            },
            _ = cancel.cancelled() => {
                tracing::info!("VectorNav read loop stopped");
                return;
            }
        };

        buf.extend_from_slice(&tmp[..n]);

        if buf.len() > MAX_BUFFER_BYTES {
            // Drop the oldest data to bound memory; we'll resync on the next
            // sync byte / `$`.
            let drop_n = buf.len() - MAX_BUFFER_BYTES;
            buf.drain(..drop_n);
            let mut s = state.write().unwrap();
            s.messages_dropped = s.messages_dropped.saturating_add(1);
        }

        drain_frames(&mut buf, &state);
    }
}

/// Pull as many complete frames as possible off the front of `buf`. Returns
/// when the next frame is incomplete or the buffer is empty.
fn drain_frames(buf: &mut Vec<u8>, state: &Arc<RwLock<VectorNavState>>) {
    loop {
        if buf.is_empty() {
            return;
        }
        match buf[0] {
            b'$' => {
                if !consume_ascii(buf, state) {
                    return;
                }
            }
            binary::SYNC_BYTE => {
                if !consume_binary(buf, state) {
                    return;
                }
            }
            _ => {
                // Stray byte (CR/LF leftover, line-noise) — skip and resync.
                buf.drain(..1);
            }
        }
    }
}

/// Try to consume one ASCII line from the front of `buf`. Returns `true` if a
/// line was consumed (the loop should continue), `false` if more bytes are
/// needed.
fn consume_ascii(buf: &mut Vec<u8>, state: &Arc<RwLock<VectorNavState>>) -> bool {
    let Some(end) = buf.iter().position(|&b| b == b'\n') else {
        return false; // wait for more bytes
    };

    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64;

    let line_bytes: Vec<u8> = buf.drain(..=end).collect();
    if line_bytes.len() > MAX_LINE_BYTES {
        let mut s = state.write().unwrap();
        s.messages_dropped = s.messages_dropped.saturating_add(1);
        return true;
    }
    let line = match std::str::from_utf8(&line_bytes) {
        Ok(v) => v.trim_end_matches(['\r', '\n']),
        Err(_) => {
            let mut s = state.write().unwrap();
            s.messages_dropped = s.messages_dropped.saturating_add(1);
            return true;
        }
    };
    if line.is_empty() {
        return true;
    }

    let mut s = state.write().unwrap();
    match parse_line(line, &mut s) {
        ParsedMessage::Async(_) => {
            s.timestamp_ns = now_ns;
            s.messages_parsed = s.messages_parsed.saturating_add(1);
        }
        ParsedMessage::Response { header, .. } => {
            tracing::debug!(header, "VN response received");
        }
        ParsedMessage::Error(code) => {
            tracing::warn!(code, "VectorNav reported an error");
            s.messages_dropped = s.messages_dropped.saturating_add(1);
        }
        ParsedMessage::Invalid(reason) => {
            tracing::trace!(reason, line, "VN line rejected");
            s.messages_dropped = s.messages_dropped.saturating_add(1);
        }
        ParsedMessage::Ignored => {}
    }
    true
}

/// Try to consume one binary frame from the front of `buf`. Returns `true`
/// if progress was made (consumed or resynced), `false` if more bytes are
/// needed.
fn consume_binary(buf: &mut Vec<u8>, state: &Arc<RwLock<VectorNavState>>) -> bool {
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64;

    let mut s = state.write().unwrap();
    match binary::try_consume(buf, &mut s) {
        FrameStep::Consumed(n) => {
            s.timestamp_ns = now_ns;
            s.messages_parsed = s.messages_parsed.saturating_add(1);
            drop(s);
            buf.drain(..n);
            true
        }
        FrameStep::Resync(n) => {
            s.messages_dropped = s.messages_dropped.saturating_add(1);
            drop(s);
            buf.drain(..n);
            true
        }
        FrameStep::NeedMore => false,
    }
}
