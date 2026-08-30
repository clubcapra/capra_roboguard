//! External GNSS source — the real robot's position fix as a UDP JSON broadcast.
//!
//! The VectorNav on this build has a working IMU but no GNSS, so position comes
//! from a separate service: the on-robot `mpu5-gps-restream` reads the MPU5
//! radio's gpsd and broadcasts a JSON fix to `255.255.255.255:<port>` (default
//! 7010) whenever it has a 3D fix. We bind that port and republish the latest
//! [`GnssFix`]; the PositionService fuses it with the VN attitude into a `Pose`.
//! (In the sim the VN frame carries the fix in-band, so this listener is not used
//! — see `[gnss].source`.)
//!
//! Wire format — one JSON object per datagram; the fields we consume:
//! ```json
//! {"lat":46.7512,"lon":7.6131,"alt_msl":560.0,"speed_ms":0.4,"track_deg":91.2,
//!  "accuracy_m":1.3, "utm_zone":"32U","easting":..,"northing":..,"elrob":".."}
//! ```
//! The service only emits on a 3D fix, so a parsed datagram *is* a fix.

use std::time::Instant;
use tokio::net::UdpSocket;
use tokio::sync::watch;

/// One decoded GNSS fix from the broadcast (raw geodetic; the ENU conversion
/// happens in the PositionService, the single owner of the datum).
#[derive(Debug, Clone, Copy)]
pub struct GnssFix {
    pub lat: f64,
    pub lon: f64,
    /// Altitude above mean sea level (m).
    pub alt_msl: f64,
    /// Ground speed (m/s) — used to dead-reckon between fixes.
    pub speed_ms: f64,
    /// Course over ground (deg, clockwise from true North) — velocity heading.
    pub track_deg: f64,
    /// Horizontal 1-sigma position estimate (m), from gpsd `eph`.
    pub accuracy_m: f64,
    pub received_at: Instant,
}

/// Parse one broadcast datagram. `None` if it isn't the expected JSON or carries
/// no position (lat/lon are the only required fields — everything else defaults).
pub fn parse(buf: &[u8]) -> Option<GnssFix> {
    let v: serde_json::Value = serde_json::from_slice(buf).ok()?;
    Some(GnssFix {
        lat: v.get("lat")?.as_f64()?,
        lon: v.get("lon")?.as_f64()?,
        alt_msl: v.get("alt_msl").and_then(|x| x.as_f64()).unwrap_or(0.0),
        speed_ms: v.get("speed_ms").and_then(|x| x.as_f64()).unwrap_or(0.0),
        track_deg: v.get("track_deg").and_then(|x| x.as_f64()).unwrap_or(0.0),
        accuracy_m: v.get("accuracy_m").and_then(|x| x.as_f64()).unwrap_or(99.0),
        received_at: Instant::now(),
    })
}

/// Bind the broadcast port and publish the latest fix on `tx`. Receiving a
/// broadcast needs only a plain bind to `0.0.0.0:port` (SO_BROADCAST gates
/// *sending* to a broadcast address, not receiving). Resilient by design: a recv
/// error is logged and the loop continues — a transient blip can't drop the
/// channel and take the bridge down.
pub async fn listen(port: u16, tx: watch::Sender<Option<GnssFix>>) -> anyhow::Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", port)).await?;
    tracing::info!("gnss: listening for broadcast fixes on :{port}");
    let mut buf = vec![0u8; 4096];
    let mut announced = false;
    loop {
        match sock.recv_from(&mut buf).await {
            Ok((n, addr)) => match parse(&buf[..n]) {
                Some(fix) => {
                    if !announced {
                        tracing::info!(
                            "gnss: first fix from {addr} — lat {:.6} lon {:.6} alt {:.1} acc {:.1} m",
                            fix.lat, fix.lon, fix.alt_msl, fix.accuracy_m,
                        );
                        announced = true;
                    }
                    let _ = tx.send(Some(fix));
                }
                None => tracing::debug!("gnss: dropping non-fix datagram ({n} B) from {addr}"),
            },
            Err(e) => {
                tracing::warn!("gnss: recv error: {e} — continuing");
                tokio::time::sleep(std::time::Duration::from_millis(200)).await;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_restream_payload() {
        // A real mpu5-gps-restream datagram (the bytes it sends to :7010).
        let j = br#"{"timestamp":1.0,"utm_zone":"32U","easting":1.0,"northing":2.0,"lat":46.7512,"lon":7.6131,"alt_msl":560.5,"speed_ms":0.4,"track_deg":90.0,"accuracy_m":1.2,"elrob":"x"}"#;
        let f = parse(j).expect("should parse");
        assert!((f.lat - 46.7512).abs() < 1e-9);
        assert!((f.lon - 7.6131).abs() < 1e-9);
        assert!((f.alt_msl - 560.5).abs() < 1e-9);
        assert!((f.track_deg - 90.0).abs() < 1e-9);
        assert!((f.accuracy_m - 1.2).abs() < 1e-9);
    }

    #[test]
    fn rejects_garbage() {
        assert!(parse(b"not json at all").is_none());
    }

    #[test]
    fn rejects_missing_position() {
        assert!(parse(br#"{"speed_ms":1.0,"track_deg":2.0}"#).is_none());
    }
}
