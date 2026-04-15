//! Telemetry subscriber for a single rove_sensor_api data port.
//!
//! Protocol: send a `Subscribe` (0x01) frame to the data port, then receive
//! `Data` (0x03) frames pushed back to our source address. We re-send Subscribe
//! periodically as a cheap keepalive in case the server forgets us.
//!
//! Resilient by design: a recv error (ICMP "port unreachable" when sensor_api is
//! reloaded / a driver bounces — errno 111) does NOT end the task. It reconnects
//! and re-subscribes, so a transient sensor_api blip can't drop the pose channel
//! and take the whole bridge down with it. Runs until the task is aborted.

use super::packet::{decode, encode, MSG_DATA, MSG_SUBSCRIBE};
use anyhow::Result;
use serde_json::json;
use std::time::Duration;
use tokio::net::UdpSocket;

/// Connect a fresh socket and send the initial Subscribe. Returns the socket.
async fn connect_and_subscribe(host: &str, data_port: u16, sub: &[u8]) -> Result<UdpSocket> {
    let sock = UdpSocket::bind(("0.0.0.0", 0)).await?;
    sock.connect((host, data_port)).await?;
    sock.send(sub).await?;
    Ok(sock)
}

/// Subscribe to `host:data_port` and invoke `on_frame` for each decoded Data
/// payload, reconnecting across transient errors. Never returns on a recoverable
/// fault — only when the spawning task is aborted.
pub async fn subscribe<F>(
    host: &str,
    data_port: u16,
    interval_ms: u32,
    mut on_frame: F,
) -> Result<()>
where
    F: FnMut(serde_json::Value),
{
    let sub = encode(MSG_SUBSCRIBE, 0, &json!({ "interval_ms": interval_ms }));
    let mut buf = vec![0u8; 64 * 1024];
    let mut connect_fails: u32 = 0;
    let mut announced = false;

    loop {
        // (Re)connect, backing off so a fully-down sensor_api doesn't busy-loop.
        let sock = match connect_and_subscribe(host, data_port, &sub).await {
            Ok(s) => {
                connect_fails = 0;
                if !announced {
                    tracing::info!("subscribed to telemetry {host}:{data_port} @ {interval_ms} ms");
                    announced = true;
                } else {
                    tracing::info!("re-subscribed to telemetry {host}:{data_port}");
                }
                s
            }
            Err(e) => {
                connect_fails += 1;
                // Warn on the first failure of an outage, then quiet down.
                if connect_fails == 1 {
                    tracing::warn!("telemetry {host}:{data_port} connect failed: {e:#} — retrying");
                } else {
                    tracing::debug!("telemetry {host}:{data_port} connect retry {connect_fails}: {e:#}");
                }
                tokio::time::sleep(Duration::from_secs(1)).await;
                continue;
            }
        };

        let mut keepalive = tokio::time::interval(Duration::from_secs(2));
        keepalive.tick().await; // consume immediate first tick

        // Receive loop for this connection; break out to reconnect on any error.
        loop {
            tokio::select! {
                _ = keepalive.tick() => {
                    let _ = sock.send(&sub).await; // best-effort re-subscribe keepalive
                }
                res = sock.recv(&mut buf) => {
                    match res {
                        Ok(n) => match decode(&buf[..n]) {
                            Ok((MSG_DATA, _seq, payload)) => on_frame(payload),
                            Ok(_) => {}              // SubscribeAck etc. — ignore
                            Err(e) => tracing::debug!("dropping bad telemetry frame: {e}"),
                        },
                        Err(e) => {
                            // ICMP port-unreachable / sensor_api reload bounce — do
                            // NOT end the task (that would drop the pose channel and
                            // crash the bridge). Reconnect + re-subscribe instead.
                            tracing::warn!("telemetry {host}:{data_port} recv error: {e} — reconnecting");
                            break;
                        }
                    }
                }
            }
        }
        tokio::time::sleep(Duration::from_millis(500)).await; // brief backoff before reconnect
    }
}
