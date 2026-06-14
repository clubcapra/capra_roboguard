//! Telemetry subscriber for a single rove_sensor_api data port.
//!
//! Protocol: send a `Subscribe` (0x01) frame to the data port, then receive
//! `Data` (0x03) frames pushed back to our source address. We re-send Subscribe
//! periodically as a cheap keepalive in case the server forgets us.

use super::packet::{decode, encode, MSG_DATA, MSG_SUBSCRIBE};
use anyhow::{Context, Result};
use serde_json::json;
use std::time::Duration;
use tokio::net::UdpSocket;

/// Run until cancelled, invoking `on_frame` for each decoded Data payload.
pub async fn subscribe<F>(
    host: &str,
    data_port: u16,
    interval_ms: u32,
    mut on_frame: F,
) -> Result<()>
where
    F: FnMut(serde_json::Value),
{
    let sock = UdpSocket::bind(("0.0.0.0", 0))
        .await
        .context("binding telemetry socket")?;
    sock.connect((host, data_port))
        .await
        .with_context(|| format!("connecting telemetry to {host}:{data_port}"))?;

    let sub = encode(MSG_SUBSCRIBE, 0, &json!({ "interval_ms": interval_ms }));
    sock.send(&sub).await.context("sending Subscribe")?;
    tracing::info!("subscribed to telemetry {host}:{data_port} @ {interval_ms} ms");

    let mut buf = vec![0u8; 64 * 1024];
    let mut keepalive = tokio::time::interval(Duration::from_secs(2));
    keepalive.tick().await; // consume immediate first tick

    loop {
        tokio::select! {
            _ = keepalive.tick() => {
                let _ = sock.send(&sub).await; // best-effort re-subscribe
            }
            res = sock.recv(&mut buf) => {
                let n = res.context("telemetry recv")?;
                match decode(&buf[..n]) {
                    Ok((MSG_DATA, _seq, payload)) => on_frame(payload),
                    Ok(_) => {}              // SubscribeAck etc. — ignore
                    Err(e) => tracing::debug!("dropping bad telemetry frame: {e}"),
                }
            }
        }
    }
}
