//! `SimFeed` — the one piece of shared plumbing every sim mock driver owns.
//!
//! When `rove_sensor_api` runs in **sim-backend mode**, each device's telemetry
//! comes from a running simulator (`rove_sim`) instead of real hardware. The sim
//! publishes one JSON DATA frame per channel to a dedicated backend UDP port
//! (fire-and-forget, no subscribe handshake — see `rove_sim/transport/udp.py`).
//!
//! `SimFeed` binds that backend port, decodes incoming frames, and caches the
//! latest payload behind an `Arc<RwLock<Value>>`. A mock driver's `read_data()`
//! is then a one-line clone of `feed.latest()`. The decode step is pluggable:
//! the default parses the `[version|msg_type|seq|JSON]` packet codec; Livox uses
//! `subscribe_raw` to parse native Livox UDP IMU packets instead.
//!
//! This is deliberately the *only* code the mocks share — each sensor still has
//! its own distinct mock driver type, co-located with its real driver, so a mock
//! stays reusable/swappable if a sensor is replaced.

use std::sync::{Arc, RwLock};

use serde_json::Value;
use tokio::net::UdpSocket;

use crate::protocol::packet::{MessageType, Packet};

/// Decodes one received datagram into a JSON value, or `None` to drop it.
pub type DecodeFn = Arc<dyn Fn(&[u8]) -> Option<Value> + Send + Sync>;

/// A cached, continuously-updated view of one sim channel.
#[derive(Clone)]
pub struct SimFeed {
    state: Arc<RwLock<Value>>,
}

impl SimFeed {
    /// Subscribe to a sim channel that speaks the standard `rove_sensor_api`
    /// packet codec (`MessageType::Data` frames carrying JSON).
    pub fn subscribe(host: &str, port: u16) -> Self {
        Self::spawn(host.to_string(), port, default_decode())
    }

    /// Subscribe to a channel with a custom wire format (e.g. native Livox).
    pub fn subscribe_raw(host: &str, port: u16, decode: DecodeFn) -> Self {
        Self::spawn(host.to_string(), port, decode)
    }

    /// The latest decoded payload (`Value::Null` until the first frame arrives).
    pub fn latest(&self) -> Value {
        self.state.read().unwrap().clone()
    }

    /// Like [`latest`](Self::latest) but returns an empty JSON object instead of
    /// `null` before the first frame, so HTTP/JSON consumers always see an object.
    pub fn latest_or_empty(&self) -> Value {
        let v = self.latest();
        if v.is_null() {
            Value::Object(Default::default())
        } else {
            v
        }
    }

    fn spawn(host: String, port: u16, decode: DecodeFn) -> Self {
        let state = Arc::new(RwLock::new(Value::Null));
        let st = state.clone();
        tokio::spawn(async move {
            // Bind the backend port the sim publishes this channel to. The sim
            // only sends (never binds) it, so there is no contention on loopback.
            let sock = match UdpSocket::bind(("0.0.0.0", port)).await {
                Ok(s) => s,
                Err(e) => {
                    tracing::error!(port, error = %e, "sim feed: bind failed");
                    return;
                }
            };
            tracing::info!(host = %host, port, "sim feed: listening");
            let mut buf = vec![0u8; 65535];
            loop {
                match sock.recv_from(&mut buf).await {
                    Ok((n, _)) => {
                        if let Some(v) = decode(&buf[..n]) {
                            *st.write().unwrap() = v;
                        }
                    }
                    Err(e) => {
                        tracing::warn!(port, error = %e, "sim feed: recv error");
                    }
                }
            }
        });
        Self { state }
    }
}

/// Default decoder: the `rove_sensor_api` packet codec, keeping only DATA frames.
fn default_decode() -> DecodeFn {
    Arc::new(|bytes: &[u8]| {
        let pkt = Packet::decode(bytes).ok()?;
        if pkt.msg_type != MessageType::Data {
            return None;
        }
        pkt.json_payload().ok()
    })
}
