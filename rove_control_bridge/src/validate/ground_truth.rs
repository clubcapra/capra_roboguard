//! Ground-truth listener — sim-only scoring cross-check.
//!
//! The sim pushes true pose as `{"pos":[x,y,z],"orn":[qx,qy,qz,w],"t":...}` to a
//! fixed `host:5030` (DATA frames, no handshake). It is published to the sim's
//! `--host`, so a remote engine only sees frames if the sim points at us
//! (default bind is 127.0.0.1). Best-effort: if nothing arrives we just warn.
//!
//! Never feeds control — this is purely to confirm the VectorNav-derived ENU
//! pose tracks reality.

use crate::transport::packet::{decode, MSG_DATA};
use anyhow::Result;
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::sync::watch;

/// True pose from the sim, in sim-world ENU (same origin as the datum).
#[derive(Debug, Clone, Copy)]
pub struct Truth {
    pub x: f64,
    pub y: f64,
}

/// Listen on `0.0.0.0:port`, publishing the latest truth to `tx`. Warns once if
/// no frame arrives in the first few seconds.
pub async fn listen(port: u16, tx: watch::Sender<Option<Truth>>) -> Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", port)).await?;
    tracing::info!("ground-truth validator listening on :{port} (best-effort)");

    let mut buf = vec![0u8; 64 * 1024];
    let mut warned = false;
    loop {
        let recv = tokio::time::timeout(Duration::from_secs(4), sock.recv(&mut buf)).await;
        match recv {
            Ok(Ok(n)) => {
                if let Ok((MSG_DATA, _seq, payload)) = decode(&buf[..n]) {
                    if let Some(pos) = payload.get("pos").and_then(|p| p.as_array()) {
                        if pos.len() >= 2 {
                            let x = pos[0].as_f64().unwrap_or(0.0);
                            let y = pos[1].as_f64().unwrap_or(0.0);
                            let _ = tx.send(Some(Truth { x, y }));
                        }
                    }
                }
            }
            Ok(Err(e)) => return Err(e.into()),
            Err(_) => {
                if !warned {
                    tracing::warn!(
                        "no ground-truth frames on :{port} — expected unless the sim's \
                         --host points at this machine; using VectorNav pose for scoring"
                    );
                    warned = true;
                }
            }
        }
    }
}
