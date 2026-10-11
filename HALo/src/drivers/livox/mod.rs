//! In-process Livox SDK2 driver. A small native library owns the SDK callbacks;
//! Rust owns state, discovery/HTTP/UDP telemetry, and the compatible LVXR stream.

pub mod config;
mod packet;

use std::collections::HashMap;
use std::ffi::{c_char, c_void, CString};
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use libloading::Library;
use serde_json::{json, Value};

use crate::core::driver::{CommandMode, DataDelivery, DataUpdate, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;
use config::LivoxConfig;
use packet::Payload;

#[derive(Default)]
struct State {
    points: Vec<[f32; 3]>,
    gyro: Option<[f32; 3]>,
    accel: Option<[f32; 3]>,
    points_timestamp_ns: u64,
    imu_timestamp_ns: u64,
    last_points: Option<Instant>,
    last_imu: Option<Instant>,
    point_packets: u64,
    imu_packets: u64,
}

struct Shared {
    states: HashMap<u8, Mutex<State>>,
    updates: HashMap<u8, tokio::sync::broadcast::Sender<DataUpdate>>,
    stale: Duration,
    socket: UdpSocket,
    points_target: Option<SocketAddr>,
    imu_target: Option<SocketAddr>,
}

impl Shared {
    fn ingest(&self, bytes: &[u8]) {
        let Some(packet) = packet::decode(bytes) else {
            return;
        };
        let Some(state) = self.states.get(&packet.lidar_id) else {
            return;
        };
        let Ok(mut state) = state.lock() else { return };
        let points = match packet.payload {
            Payload::Points(points) => {
                state.points = points;
                state.points_timestamp_ns = packet.timestamp_ns;
                state.last_points = Some(Instant::now());
                state.point_packets += 1;
                true
            }
            Payload::Imu { gyro, accel } => {
                state.gyro = Some(gyro);
                state.accel = Some(accel);
                state.imu_timestamp_ns = packet.timestamp_ns;
                state.last_imu = Some(Instant::now());
                state.imu_packets += 1;
                false
            }
        };
        let source = &self.updates[&packet.lidar_id];
        if source.receiver_count() > 0 {
            if points {
                let parts = state.points.len().div_ceil(128).max(1);
                for part in 0..parts {
                    let first = part * 128;
                    let last = (first + 128).min(state.points.len());
                    let data = snapshot(
                        &state,
                        packet.lidar_id,
                        self.stale,
                        &state.points[first..last],
                        "points",
                        part,
                        parts,
                    );
                    let _ = source.send(DataUpdate::Data(Arc::new(data)));
                }
            } else {
                let data = snapshot(&state, packet.lidar_id, self.stale, &[], "imu", 0, 1);
                let _ = source.send(DataUpdate::Data(Arc::new(data)));
            }
        }
        drop(state);
        let target = if points {
            self.points_target
        } else {
            self.imu_target
        };
        if let Some(target) = target {
            let _ = self.socket.send_to(bytes, target);
        }
    }
}

type Callback = unsafe extern "C" fn(*const u8, usize, *mut c_void);
type Start = unsafe extern "C" fn(*const c_char, Callback, *mut c_void) -> i32;
type Stop = unsafe extern "C" fn();

/// The SDK threads borrow Shared. Stop joins them before Shared/library drop.
struct Session {
    shared: Arc<Shared>,
    native: Option<(Stop, Library)>,
    _configuration: Option<config::SdkConfigFile>,
    task: Option<crate::core::task::WorkerTask>,
}

impl Drop for Session {
    fn drop(&mut self) {
        if let Some((stop, _)) = &self.native {
            unsafe { stop() };
        }
    }
}

unsafe extern "C" fn on_packet(data: *const u8, len: usize, context: *mut c_void) {
    if data.is_null() || context.is_null() || !(20..=65507).contains(&len) {
        return;
    }
    // Never unwind across the C ABI. The library guarantees both pointers remain
    // valid for the call; Session retains Shared until SDK shutdown has joined.
    let _ = std::panic::catch_unwind(|| {
        let shared = unsafe { &*(context as *const Shared) };
        let bytes = unsafe { std::slice::from_raw_parts(data, len) };
        shared.ingest(bytes);
    });
}

pub struct LivoxDriver {
    id: String,
    lidar_id: u8,
    stale: Duration,
    session: Arc<Session>,
}

fn snapshot(
    state: &State,
    id: u8,
    stale: Duration,
    points: &[[f32; 3]],
    kind: &str,
    part: usize,
    parts: usize,
) -> Value {
    json!({"connected":state.last_points.is_some_and(|t| t.elapsed() < stale),
        "imu_connected":state.last_imu.is_some_and(|t| t.elapsed() < stale),
        "lidar_id":id, "points":points, "gyro":state.gyro,"accel":state.accel,
        "points_timestamp_ns":state.points_timestamp_ns,"imu_timestamp_ns":state.imu_timestamp_ns,
        "point_packets":state.point_packets,"imu_packets":state.imu_packets,
        "packet_type":kind,"point_packet_index":part,"point_packet_parts":parts})
}
fn shared(
    ids: &[u8],
    stale_ms: u64,
    points_target: Option<SocketAddr>,
    imu_target: Option<SocketAddr>,
) -> Result<Arc<Shared>, DriverError> {
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.set_nonblocking(true)?;
    Ok(Arc::new(Shared {
        states: ids
            .iter()
            .map(|id| (*id, Mutex::new(State::default())))
            .collect(),
        updates: ids
            .iter()
            .map(|id| (*id, tokio::sync::broadcast::channel(2048).0))
            .collect(),
        stale: Duration::from_millis(stale_ms),
        socket,
        points_target,
        imu_target,
    }))
}

/// The simulator emits the same LVXR datagrams as the native SDK adapter.
/// A single owned feed provides both lidars and the same fanout implementation.
pub async fn connect_sim(
    inputs: &[(u8, u16)],
    stale_ms: u64,
) -> Result<Vec<LivoxDriver>, DriverError> {
    let ids: Vec<_> = inputs.iter().map(|(id, _)| *id).collect();
    let shared = shared(&ids, stale_ms, None, None)?;
    let mut feeds = Vec::new();
    for (id, port) in inputs {
        let socket = tokio::net::UdpSocket::bind(("0.0.0.0", *port)).await?;
        let state = shared.clone();
        let id = *id;
        feeds.push(Box::pin(async move {
            let mut buffer = vec![0u8; 65535];
            while let Ok((n, _)) = socket.recv_from(&mut buffer).await {
                if packet::decode(&buffer[..n]).is_some_and(|p| p.lidar_id == id) {
                    state.ingest(&buffer[..n]);
                }
            }
        }));
    }
    let task = crate::core::task::WorkerTask::spawn(async move {
        futures::future::join_all(feeds).await;
    });
    let session = Arc::new(Session {
        shared,
        native: None,
        _configuration: None,
        task: Some(task),
    });
    Ok(ids
        .into_iter()
        .map(|lidar_id| LivoxDriver {
            id: format!("livox_{lidar_id}"),
            lidar_id,
            stale: Duration::from_millis(stale_ms),
            session: session.clone(),
        })
        .collect())
}

/// One SDK session for all configured lidars; no subprocess or external service.
pub fn connect(cfg: LivoxConfig) -> Result<Vec<LivoxDriver>, DriverError> {
    use crate::core::config::DriverConfig;
    cfg.validate().map_err(DriverError::InitFailed)?;
    let ids = cfg.lidar_ids();
    let configuration = cfg.sdk_file()?;
    let shared = shared(&ids, cfg.stale_ms, cfg.points_target, cfg.imu_target)?;
    let path = CString::new(configuration.path.to_string_lossy().as_bytes())
        .map_err(|e| DriverError::InitFailed(e.to_string()))?;
    // The library ABI is deliberately small and independent of SDK struct layouts.
    let session = unsafe {
        let library = Library::new(&cfg.library).map_err(|e| {
            DriverError::InitFailed(format!("Livox library {}: {e}", cfg.library.display()))
        })?;
        let start: Start = *library
            .get(b"halo_livox_start\0")
            .map_err(|e| DriverError::InitFailed(e.to_string()))?;
        let stop: Stop = *library
            .get(b"halo_livox_stop\0")
            .map_err(|e| DriverError::InitFailed(e.to_string()))?;
        let result = start(
            path.as_ptr(),
            on_packet,
            Arc::as_ptr(&shared) as *mut c_void,
        );
        if result != 0 {
            // start cleans up its own failed initialization; it must not stop
            // another session if the native SDK is already in use.
            return Err(DriverError::InitFailed(format!(
                "Livox SDK start failed ({result})"
            )));
        }
        Arc::new(Session {
            shared,
            native: Some((stop, library)),
            _configuration: Some(configuration),
            task: None,
        })
    };
    Ok(ids
        .into_iter()
        .map(|lidar_id| LivoxDriver {
            id: format!("livox_{lidar_id}"),
            lidar_id,
            stale: Duration::from_millis(cfg.stale_ms),
            session: session.clone(),
        })
        .collect())
}

impl SensorDriver for LivoxDriver {
    fn telemetry_status(&self, data: &serde_json::Value) -> crate::core::driver::TelemetryStatus {
        crate::core::driver::TelemetryStatus::from_fields(
            data,
            "points_timestamp_ns",
            &["connected"],
            &["imu_connected"],
            &[],
        )
    }
    fn data_delivery(&self) -> DataDelivery {
        DataDelivery::Updates
    }
    fn subscribe_updates(&self) -> Option<tokio::sync::broadcast::Receiver<DataUpdate>> {
        Some(self.session.shared.updates[&self.lidar_id].subscribe())
    }
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        Box::pin(async {
            if let Some(task) = &self.session.task {
                task.stop().await;
            }
            Ok(())
        })
    }
    fn id(&self) -> &str {
        &self.id
    }
    fn display_name(&self) -> &str {
        "Livox Mid-360"
    }
    fn data_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("connected", "Fresh point-cloud packets received", "bool"),
            FieldDescriptor::new("imu_connected", "Fresh IMU packets received", "bool"),
            FieldDescriptor::new("lidar_id", "Last octet of the lidar IP", "u8"),
            FieldDescriptor::new(
                "points",
                "Point packet segment in sensor frame; UDP update subscriptions deliver every segment",
                "Vec<[f32;3]>",
            )
            .with_unit("m"),
            FieldDescriptor::new(
                "gyro",
                "Latest angular velocity, sensor frame",
                "Option<[f32;3]>",
            )
            .with_unit("rad/s"),
            FieldDescriptor::new(
                "accel",
                "Latest acceleration, sensor frame",
                "Option<[f32;3]>",
            )
            .with_unit("g"),
            FieldDescriptor::new(
                "points_timestamp_ns",
                "Point packet host arrival time",
                "u64",
            )
            .with_unit("ns"),
            FieldDescriptor::new("imu_timestamp_ns", "IMU packet host arrival time", "u64")
                .with_unit("ns"),
            FieldDescriptor::new("point_packets", "Received point packet count", "u64"),
            FieldDescriptor::new("imu_packets", "Received IMU packet count", "u64"),
            FieldDescriptor::new("packet_type", "snapshot, points or imu; IMU updates have no points", "String"),
            FieldDescriptor::new("point_packet_index", "Zero-based point segment index", "u16"),
            FieldDescriptor::new("point_packet_parts", "Segments in this point packet", "u16"),
        ]
    }
    fn command_schema(&self) -> Vec<FieldDescriptor> {
        vec![]
    }
    fn command_mode(&self) -> CommandMode {
        CommandMode::Rest
    }
    fn read_data(&self) -> Result<Value, DriverError> {
        let state = self.session.shared.states[&self.lidar_id]
            .lock()
            .map_err(|_| DriverError::CommandFailed("Livox state lock poisoned".into()))?;
        Ok(snapshot(
            &state,
            self.lidar_id,
            self.stale,
            &state.points[..state.points.len().min(128)],
            "snapshot",
            0,
            state.points.len().div_ceil(128).max(1),
        ))
    }

    fn execute_command(&self, _payload: &Value) -> Result<Value, DriverError> {
        Err(DriverError::CommandFailed(
            "Livox telemetry is read-only".into(),
        ))
    }
}
