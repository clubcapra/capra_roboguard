//! `GET /discover` — resolve each sensor's data + command UDP ports.
//!
//! rove_sensor_api advertises ports dynamically (registration order), so we read
//! them rather than hard-coding. Returns id -> (data_port, command_port).

use anyhow::{Context, Result};
use std::collections::HashMap;

#[derive(Debug, Clone, Copy)]
pub struct Endpoints {
    pub data_port: u16,
    pub command_port: u16,
}

pub type Discovery = HashMap<String, Endpoints>;

/// Blocking one-shot at startup. `host`/`port` point at rove_sensor_api's HTTP API.
pub fn discover(host: &str, port: u16) -> Result<Discovery> {
    let url = format!("http://{host}:{port}/discover");
    let body = ureq::get(&url)
        .timeout(std::time::Duration::from_secs(5))
        .call()
        .with_context(|| format!("GET {url}"))?
        .into_string()
        .context("reading /discover body")?;

    let parsed: serde_json::Value =
        serde_json::from_str(&body).context("parsing /discover JSON")?;
    let sensors = parsed
        .get("sensors")
        .and_then(|s| s.as_array())
        .context("/discover has no `sensors` array")?;

    let mut map = Discovery::new();
    for s in sensors {
        let id = match s.get("id").and_then(|v| v.as_str()) {
            Some(id) => id.to_string(),
            None => continue,
        };
        let data_port = s.get("data_port").and_then(|v| v.as_u64());
        let command_port = s.get("command_port").and_then(|v| v.as_u64());
        if let (Some(d), Some(c)) = (data_port, command_port) {
            map.insert(
                id,
                Endpoints {
                    data_port: d as u16,
                    command_port: c as u16,
                },
            );
        }
    }
    Ok(map)
}
