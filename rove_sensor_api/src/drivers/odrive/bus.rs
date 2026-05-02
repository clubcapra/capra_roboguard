use std::collections::HashMap;
use std::io;
use std::sync::{Arc, RwLock};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dashmap::DashMap;
use socketcan::{tokio::CanSocket, CanFrame, EmbeddedFrame, StandardId};
use tokio::sync::{oneshot, Mutex};

use super::protocol::{
    can_id, decode_bus_vi, decode_encoder_count, decode_encoder_estimates, decode_get_error,
    decode_heartbeat, decode_iq, decode_powers, decode_sdo_response, decode_temperature,
    decode_torques, encode_sdo_read, split_can_id, CMD_ENCODER_COUNT, CMD_ENCODER_ESTIMATES,
    CMD_GET_BUS_VOLTAGE_CURRENT, CMD_GET_ERROR, CMD_GET_IQ, CMD_GET_POWERS, CMD_GET_TEMPERATURE,
    CMD_GET_TORQUES, CMD_HEARTBEAT, CMD_RXSDO, CMD_TXSDO,
};
use super::state::OdriveNodeState;

/// Shared CAN bus handle. One instance per CAN interface.
///
/// Holds a write socket behind a Mutex for sends from multiple async tasks.
/// The receive loop runs in a background task with its own socket.
pub struct CanBus {
    iface: String,
    tx: Mutex<CanSocket>,
    /// Pending SDO read requests: keyed by (node_id, endpoint_id).
    pending_sdo: Arc<DashMap<(u8, u16), oneshot::Sender<[u8; 4]>>>,
}

/// Per-node state map shared between the bus receive loop and each OdriveNode.
pub type NodeStates = Arc<RwLock<HashMap<u8, Arc<RwLock<OdriveNodeState>>>>>;

impl CanBus {
    /// Open the CAN interface, start the receive loop, and return a shared bus handle.
    pub fn open(iface: &str, states: NodeStates) -> io::Result<Arc<Self>> {
        let tx = CanSocket::open(iface)?;
        let rx = CanSocket::open(iface)?;
        let pending_sdo: Arc<DashMap<(u8, u16), oneshot::Sender<[u8; 4]>>> =
            Arc::new(DashMap::new());

        let bus = Arc::new(Self {
            iface: iface.to_string(),
            tx: Mutex::new(tx),
            pending_sdo: pending_sdo.clone(),
        });

        tokio::spawn(receive_loop(rx, states, pending_sdo));

        tracing::info!(iface, "CAN bus opened");
        Ok(bus)
    }

    pub fn iface(&self) -> &str {
        &self.iface
    }

    /// Send a CAN frame to `node_id` with `cmd_id` and `data`.
    pub async fn send(&self, node_id: u8, cmd_id: u32, data: &[u8]) -> io::Result<()> {
        let raw_id = can_id(node_id, cmd_id);
        let id = StandardId::new(raw_id as u16)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "CAN ID overflow"))?;
        let frame = CanFrame::new(id, data)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "frame too long"))?;
        self.tx.lock().await.write_frame(frame).await
    }

    /// Read an endpoint via SDO. Sends an RxSdo request and waits for the TxSdo response.
    pub async fn sdo_read(
        &self,
        node_id: u8,
        endpoint_id: u16,
        timeout: Duration,
    ) -> io::Result<[u8; 4]> {
        let (tx, rx) = oneshot::channel();
        self.pending_sdo.insert((node_id, endpoint_id), tx);

        let data = encode_sdo_read(endpoint_id);
        if let Err(e) = self.send(node_id, CMD_RXSDO, &data).await {
            self.pending_sdo.remove(&(node_id, endpoint_id));
            return Err(e);
        }

        match tokio::time::timeout(timeout, rx).await {
            Ok(Ok(bytes)) => Ok(bytes),
            Ok(Err(_)) => Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "SDO channel dropped",
            )),
            Err(_) => {
                self.pending_sdo.remove(&(node_id, endpoint_id));
                Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!("SDO read timeout for endpoint {endpoint_id}"),
                ))
            }
        }
    }

    /// Write an endpoint value via SDO (raw 8-byte RxSdo write frame).
    /// The drive responds with a TxSdo confirmation; we discard it.
    pub async fn sdo_write(&self, node_id: u8, _endpoint_id: u16, payload: [u8; 8]) -> io::Result<()> {
        self.send(node_id, CMD_RXSDO, &payload).await
    }
}

/// Background task: read frames and update node state cache.
async fn receive_loop(
    rx: CanSocket,
    states: NodeStates,
    pending_sdo: Arc<DashMap<(u8, u16), oneshot::Sender<[u8; 4]>>>,
) {
    loop {
        let frame = match rx.read_frame().await {
            Ok(f) => f,
            Err(e) => {
                tracing::warn!(error = %e, "CAN receive error");
                continue;
            }
        };

        let raw_id = match frame.id() {
            socketcan::Id::Standard(id) => id.as_raw() as u32,
            socketcan::Id::Extended(id) => id.as_raw(),
        };

        let (node_id, cmd_id) = split_can_id(raw_id);
        let data = frame.data();

        // Handle SDO responses before node-state lookup (may come from any node).
        if cmd_id == CMD_TXSDO {
            if let Some((ep_id, value_bytes)) = decode_sdo_response(data) {
                if let Some((_, tx)) = pending_sdo.remove(&(node_id, ep_id)) {
                    let _ = tx.send(value_bytes);
                }
            }
            continue;
        }

        // Look up the node state — skip unknown nodes
        let state_arc = {
            match states.read().unwrap().get(&node_id).cloned() {
                Some(s) => s,
                None => continue,
            }
        };

        let now_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as i64;

        let mut state = state_arc.write().unwrap();
        let mut decoded = true;

        match cmd_id {
            CMD_HEARTBEAT => {
                if let Some(f) = decode_heartbeat(data) {
                    state.axis_error       = f.axis_error;
                    state.axis_state       = f.axis_state;
                    state.procedure_result = f.procedure_result;
                    state.trajectory_done  = f.trajectory_done;
                } else { decoded = false; }
            }
            CMD_ENCODER_ESTIMATES => {
                if let Some(f) = decode_encoder_estimates(data) {
                    state.pos_estimate = f.pos_estimate;
                    state.vel_estimate = f.vel_estimate;
                } else { decoded = false; }
            }
            CMD_ENCODER_COUNT => {
                if let Some(f) = decode_encoder_count(data) {
                    state.shadow_count = f.shadow_count;
                    state.count_cpr    = f.count_cpr;
                } else { decoded = false; }
            }
            CMD_GET_IQ => {
                if let Some(f) = decode_iq(data) {
                    state.iq_setpoint = f.iq_setpoint;
                    state.iq_measured  = f.iq_measured;
                } else { decoded = false; }
            }
            CMD_GET_BUS_VOLTAGE_CURRENT => {
                if let Some(f) = decode_bus_vi(data) {
                    state.bus_voltage = f.bus_voltage;
                    state.bus_current = f.bus_current;
                } else { decoded = false; }
            }
            CMD_GET_ERROR => {
                if let Some(f) = decode_get_error(data) {
                    state.active_errors  = f.active_errors;
                    state.disarm_reason  = f.disarm_reason;
                } else { decoded = false; }
            }
            CMD_GET_TEMPERATURE => {
                if let Some(f) = decode_temperature(data) {
                    if f.fet_temp.is_finite()   { state.fet_temp   = Some(f.fet_temp);   }
                    if f.motor_temp.is_finite() { state.motor_temp = Some(f.motor_temp); }
                } else { decoded = false; }
            }
            CMD_GET_TORQUES => {
                if let Some(f) = decode_torques(data) {
                    state.torque_target   = f.torque_target;
                    state.torque_estimate = f.torque_estimate;
                } else { decoded = false; }
            }
            CMD_GET_POWERS => {
                if let Some(f) = decode_powers(data) {
                    state.electrical_power = f.electrical_power;
                    state.mechanical_power = f.mechanical_power;
                } else { decoded = false; }
            }
            _ => { decoded = false; }
        }

        if decoded {
            state.timestamp_ns = now_ns;
        }
    }
}
