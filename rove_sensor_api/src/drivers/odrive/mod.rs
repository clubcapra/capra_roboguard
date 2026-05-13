pub mod bus;
pub mod endpoints;
pub mod node;
pub mod protocol;
pub mod state;

use std::collections::HashMap;
use std::io;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use socketcan::{tokio::CanSocket, EmbeddedFrame};

use bus::{CanBus, NodeStates};
use endpoints::{auto_fetch_endpoints, load_from_file, new_shared, SharedEndpointMap};
use node::{OdriveNode, WatchdogConfig};
use protocol::{split_can_id, CMD_HEARTBEAT};
use state::OdriveNodeState;

/// Scan the CAN bus for ODrive heartbeat frames and register one `OdriveNode`
/// per discovered node ID.
///
/// - `iface`:    SocketCAN interface name, e.g. `"can0"`.
/// - `timeout`:  How long to listen for heartbeats (default: 2 s).
/// - `watchdog`: Watchdog configuration applied to every discovered node.
///
/// Reads `ODRIVE_ENDPOINTS` env var for the path to `flat_endpoints.json`.
/// If unset or unreadable, config endpoints are disabled but calibration still works.
///
/// Returns `(nodes, shared_endpoint_map)`. The endpoint map starts empty (or
/// pre-loaded from `ODRIVE_ENDPOINTS`). Pass it to `build_router` so the
/// `POST /odrive/endpoints` route can update it live without a restart.
pub async fn discover_nodes(
    iface: &str,
    timeout: Duration,
    watchdog: WatchdogConfig,
) -> io::Result<(Vec<OdriveNode>, SharedEndpointMap)> {
    tracing::info!(iface, ?timeout, "scanning CAN bus for ODrive nodes");

    // Create the shared endpoint map (shared across all nodes, same firmware).
    let endpoint_map = new_shared();

    // Load priority:
    // 1. ODRIVE_ENDPOINTS — explicit local file path
    // 2. ODRIVE_HW_VERSION + ODRIVE_FW_VERSION — auto-fetch from ODrive CDN (cached offline)
    // 3. Neither — wait for POST /odrive/endpoints upload
    if let Ok(path) = std::env::var("ODRIVE_ENDPOINTS") {
        if let Err(e) = load_from_file(&endpoint_map, &path) {
            tracing::warn!(error = %e, "ODrive endpoint map load failed — upload via POST /odrive/endpoints");
        }
    } else {
        let hw = std::env::var("ODRIVE_HW_VERSION").ok();
        let fw = std::env::var("ODRIVE_FW_VERSION").ok();
        match (hw, fw) {
            (Some(hw_ver), Some(fw_ver)) => {
                match auto_fetch_endpoints(&endpoint_map, &hw_ver, &fw_ver).await {
                    Ok(count) => tracing::info!(
                        hw_version = %hw_ver, fw_version = %fw_ver, count,
                        "ODrive endpoint map ready"
                    ),
                    Err(e) => tracing::warn!(
                        error = %e, hw_version = %hw_ver, fw_version = %fw_ver,
                        "ODrive endpoint auto-fetch failed — upload via POST /odrive/endpoints"
                    ),
                }
            }
            (hw, fw) => {
                if hw.is_none() || fw.is_none() {
                    tracing::info!(
                        "Set ODRIVE_HW_VERSION (e.g. 4.4.58) and ODRIVE_FW_VERSION (e.g. 0.6.11 or latest) \
                         for automatic endpoint loading, or upload flat_endpoints.json via POST /odrive/endpoints in Scalar"
                    );
                }
            }
        }
    };

    // Open a temporary socket just for discovery
    let sock = CanSocket::open(iface)?;

    let mut found: HashMap<u8, ()> = HashMap::new();
    let deadline = tokio::time::Instant::now() + timeout;

    loop {
        match tokio::time::timeout_at(deadline, sock.read_frame()).await {
            Ok(Ok(frame)) => {
                let raw_id = match frame.id() {
                    socketcan::Id::Standard(id) => id.as_raw() as u32,
                    socketcan::Id::Extended(id) => id.as_raw(),
                };
                let (node_id, cmd_id) = split_can_id(raw_id);
                if cmd_id == CMD_HEARTBEAT {
                    if found.insert(node_id, ()).is_none() {
                        tracing::info!(node_id, iface, "discovered ODrive node");
                    }
                }
            }
            _ => break, // timeout or socket error → done scanning
        }
    }

    if found.is_empty() {
        tracing::warn!(iface, "no ODrive nodes discovered on CAN bus");
        return Ok((Vec::new(), endpoint_map));
    }

    // Build the shared state map and open the live bus
    let states: NodeStates = Arc::new(RwLock::new(HashMap::new()));
    for &node_id in found.keys() {
        states
            .write()
            .unwrap()
            .insert(node_id, Arc::new(RwLock::new(OdriveNodeState::default())));
    }

    let bus = CanBus::open(iface, states.clone())?;

    // Create one OdriveNode per discovered node
    let nodes = found
        .keys()
        .map(|&node_id| {
            let state = states.read().unwrap()[&node_id].clone();
            OdriveNode::new(node_id, bus.clone(), state, watchdog.clone(), endpoint_map.clone())
        })
        .collect();

    tracing::info!(count = found.len(), iface, "ODrive discovery complete");
    Ok((nodes, endpoint_map))
}
