use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use tokio_util::sync::CancellationToken;

use crate::core::driver::SensorDriver;
use crate::logging::LogManager;
use crate::protocol::packet::{MessageType, Packet};

const MAX_PACKET_SIZE: usize = 4096;

struct Subscriber {
    cancel: CancellationToken,
}

/// Spawn UDP listeners for a single sensor: one for data subscriptions, one for commands.
///
/// `default_push_interval_ms` is the fallback push rate used when a Subscribe
/// packet doesn't carry its own `interval_ms` — comes from `config/server.toml`.
pub async fn spawn_sensor_udp(
    driver: Arc<dyn SensorDriver>,
    data_port: u16,
    cmd_port: u16,
    log_mgr: Arc<LogManager>,
    default_push_interval_ms: u64,
) -> Result<(), std::io::Error> {
    let sensor_id = driver.id();
    let sensor_id_owned = sensor_id.to_string();

    // --- Data port: subscription-based push ---
    let data_driver = driver.clone();
    let data_sock = Arc::new(UdpSocket::bind(("0.0.0.0", data_port)).await?);
    tracing::info!(sensor = sensor_id, port = data_port, "UDP data socket bound");

    let subscribers: Arc<Mutex<HashMap<SocketAddr, Subscriber>>> =
        Arc::new(Mutex::new(HashMap::new()));

    tokio::spawn({
        let data_sock = data_sock.clone();
        let subscribers = subscribers.clone();
        let data_driver = data_driver.clone();

        async move {
            let mut buf = [0u8; MAX_PACKET_SIZE];
            loop {
                let (len, addr) = match data_sock.recv_from(&mut buf).await {
                    Ok(r) => r,
                    Err(e) => {
                        tracing::error!(error = %e, "UDP data recv error");
                        continue;
                    }
                };

                let packet = match Packet::decode(&buf[..len]) {
                    Ok(p) => p,
                    Err(e) => {
                        tracing::warn!(error = %e, "invalid packet on data port");
                        continue;
                    }
                };

                match packet.msg_type {
                    MessageType::Subscribe => {
                        // Parse optional interval from payload
                        let interval_ms = packet
                            .json_payload()
                            .ok()
                            .and_then(|v| v.get("interval_ms")?.as_u64())
                            .unwrap_or(default_push_interval_ms);
                        let interval = Duration::from_millis(interval_ms);

                        let mut subs = subscribers.lock().await;

                        // Cancel existing subscription if re-subscribing
                        if let Some(old) = subs.remove(&addr) {
                            old.cancel.cancel();
                        }

                        let cancel = CancellationToken::new();
                        subs.insert(addr, Subscriber { cancel: cancel.clone() });

                        // Spawn a push task for this subscriber
                        let push_sock = data_sock.clone();
                        let push_driver = data_driver.clone();
                        tokio::spawn(async move {
                            let mut ticker = tokio::time::interval(interval);
                            let mut seq: u16 = 0;
                            loop {
                                tokio::select! {
                                    _ = ticker.tick() => {
                                        match push_driver.read_data() {
                                            Ok(data) => {
                                                let pkt = Packet::data(seq, &data);
                                                let _ = push_sock.send_to(&pkt.encode(), addr).await;
                                                seq = seq.wrapping_add(1);
                                            }
                                            Err(e) => {
                                                tracing::warn!(error = %e, "data push read failed");
                                            }
                                        }
                                    }
                                    _ = cancel.cancelled() => break,
                                }
                            }
                        });

                        tracing::info!(%addr, interval_ms, "subscriber added");
                        let ack = Packet::subscribe_ack(
                            packet.seq_num,
                            &format!("subscribed at {}ms", interval_ms),
                        );
                        let _ = data_sock.send_to(&ack.encode(), addr).await;
                    }

                    MessageType::Unsubscribe => {
                        let mut subs = subscribers.lock().await;
                        if let Some(sub) = subs.remove(&addr) {
                            sub.cancel.cancel();
                            tracing::info!(%addr, "subscriber removed");
                            let ack =
                                Packet::subscribe_ack(packet.seq_num, "unsubscribed");
                            let _ = data_sock.send_to(&ack.encode(), addr).await;
                        } else {
                            let ack = Packet::subscribe_ack(
                                packet.seq_num,
                                "not subscribed",
                            );
                            let _ = data_sock.send_to(&ack.encode(), addr).await;
                        }
                    }

                    other => {
                        let err = Packet::error(
                            packet.seq_num,
                            &format!("unexpected message type on data port: {other:?}, expected Subscribe (0x01) or Unsubscribe (0x02)"),
                        );
                        let _ = data_sock.send_to(&err.encode(), addr).await;
                    }
                }
            }
        }
    });

    // --- Command port: handles Command, StreamStart, StreamStop ---
    let cmd_driver = driver.clone();
    let cmd_sock = Arc::new(UdpSocket::bind(("0.0.0.0", cmd_port)).await?);
    tracing::info!(sensor = sensor_id, port = cmd_port, "UDP command socket bound");

    tokio::spawn(async move {
        let mut buf = [0u8; MAX_PACKET_SIZE];

        loop {
            let (len, addr) = match cmd_sock.recv_from(&mut buf).await {
                Ok(r) => r,
                Err(e) => {
                    tracing::error!(error = %e, "UDP cmd recv error");
                    continue;
                }
            };

            let packet = match Packet::decode(&buf[..len]) {
                Ok(p) => p,
                Err(e) => {
                    tracing::warn!(error = %e, "invalid packet on cmd port");
                    continue;
                }
            };

            let response = match packet.msg_type {
                MessageType::Command => {
                    let payload = match packet.json_payload() {
                        Ok(p) => p,
                        Err(e) => {
                            let r = Packet::error(packet.seq_num, &e.to_string());
                            let _ = cmd_sock.send_to(&r.encode(), addr).await;
                            continue;
                        }
                    };
                    let outcome = cmd_driver.execute_command(&payload);
                    let client = addr.to_string();
                    match &outcome {
                        Ok(result) => log_mgr.log_input(
                            &sensor_id_owned,
                            "udp",
                            &client,
                            "",
                            "command",
                            &payload,
                            Ok(result),
                        ),
                        Err(e) => log_mgr.log_input(
                            &sensor_id_owned,
                            "udp",
                            &client,
                            "",
                            "command",
                            &payload,
                            Err(&e.to_string()),
                        ),
                    }
                    match outcome {
                        Ok(result) => Packet::command_ack(packet.seq_num, &result),
                        Err(e) => Packet::error(packet.seq_num, &e.to_string()),
                    }
                }

                other => Packet::error(
                    packet.seq_num,
                    &format!("unexpected message type on cmd port: {other:?}"),
                ),
            };

            if let Err(e) = cmd_sock.send_to(&response.encode(), addr).await {
                tracing::warn!(error = %e, "UDP cmd send error");
            }
        }
    });

    Ok(())
}
