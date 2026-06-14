//! Fan-out decoded frames to assigned engines and collect detections.
//!
//! For each (engine, feed) pair, a bridge task:
//! 1. Subscribes to the feed's broadcast channel
//! 2. Sends frames to the engine via UDS
//! 3. Reads detection responses
//! 4. Publishes detections to a per-engine-per-feed broadcast channel

use std::collections::HashSet;
use std::sync::{Arc, RwLock};

use tokio::sync::broadcast;

use crate::engine::protocol;
use crate::engine::registry::EngineRegistry;
use crate::feed::decoder::Frame;
use crate::feed::tracker::{ByteTracker, TrackerConfig};
use crate::models::detection::WsDetectionMessage;

/// Shared, live-updatable "classes to look for" filter for a feed.
/// `None` reports every class; `Some(set)` keeps only those classes.
pub type ClassFilter = Arc<RwLock<Option<HashSet<String>>>>;

/// Start a bridge task that forwards frames to an engine and publishes detections.
///
/// `tracker_cfg` enables ByteTrack track_id assignment for this (engine, feed).
/// `class_filter` (shared across the feed) limits which classes are reported and
/// can be changed live.
/// Returns the detection broadcast sender for WebSocket subscribers.
pub fn start_bridge(
    engine_name: String,
    feed_id: String,
    frame_rx: broadcast::Receiver<Frame>,
    registry: Arc<EngineRegistry>,
    width: u32,
    height: u32,
    tracker_cfg: Option<TrackerConfig>,
    class_filter: ClassFilter,
) -> broadcast::Sender<WsDetectionMessage> {
    let (detection_tx, _) = broadcast::channel::<WsDetectionMessage>(64);
    let detection_tx_clone = detection_tx.clone();

    tokio::spawn(bridge_loop(
        engine_name,
        feed_id,
        frame_rx,
        registry,
        detection_tx_clone,
        width,
        height,
        tracker_cfg,
        class_filter,
    ));

    detection_tx
}

async fn bridge_loop(
    engine_name: String,
    feed_id: String,
    mut frame_rx: broadcast::Receiver<Frame>,
    registry: Arc<EngineRegistry>,
    detection_tx: broadcast::Sender<WsDetectionMessage>,
    width: u32,
    height: u32,
    tracker_cfg: Option<TrackerConfig>,
    class_filter: ClassFilter,
) {
    // Per-(engine,feed) ByteTrack instance; frames arrive in temporal order here.
    let mut tracker = tracker_cfg.map(ByteTracker::new);
    // Configure feed on the engine
    let process = match registry.get_process(&engine_name) {
        Some(p) => p,
        None => {
            tracing::error!("Bridge: engine '{engine_name}' not found");
            return;
        }
    };

    {
        let mut proc = process.lock().await;
        let configure_msg = serde_json::json!({
            "cmd": "configure_feed",
            "feed_id": feed_id,
            "width": width,
            "height": height,
            "channels": 3,
        });

        if let Err(e) = protocol::send_control(&mut proc.stream, &configure_msg).await {
            tracing::error!("Bridge: failed to configure feed on '{engine_name}': {e}");
            return;
        }

        // Read configure response
        match protocol::recv_message(&mut proc.stream).await {
            Ok(protocol::ProtocolMessage::Control(val)) => {
                if val.get("error").is_some() {
                    tracing::error!("Bridge: engine '{engine_name}' rejected feed config: {val}");
                    return;
                }
            }
            Err(e) => {
                tracing::error!("Bridge: failed to read config response from '{engine_name}': {e}");
                return;
            }
        }
    }

    tracing::info!("Bridge: {engine_name} ↔ feed {feed_id} active");

    loop {
        let frame = match frame_rx.recv().await {
            Ok(f) => f,
            Err(broadcast::error::RecvError::Lagged(n)) => {
                tracing::debug!(
                    "Bridge {engine_name}/{feed_id}: skipped {n} frames (backpressure)"
                );
                continue;
            }
            Err(broadcast::error::RecvError::Closed) => {
                tracing::info!("Bridge {engine_name}/{feed_id}: feed closed");
                break;
            }
        };

        let process = match registry.get_process(&engine_name) {
            Some(p) => p,
            None => break,
        };

        let mut proc = process.lock().await;

        // Send frame
        if let Err(e) = protocol::send_frame(&mut proc.stream, &feed_id, &frame.data).await {
            tracing::error!("Bridge {engine_name}/{feed_id}: send frame error: {e}");
            break;
        }

        // Read detection response
        match protocol::recv_message(&mut proc.stream).await {
            Ok(protocol::ProtocolMessage::Control(val)) => {
                if let Some(err) = val.get("error") {
                    tracing::warn!(
                        "Bridge {engine_name}/{feed_id}: engine error: {err}"
                    );
                    continue;
                }

                let mut detections: Vec<_> = val
                    .get("detections")
                    .and_then(|d| d.as_array())
                    .map(|arr| {
                        arr.iter()
                            .filter_map(crate::models::detection::Detection::from_json_value)
                            .collect()
                    })
                    .unwrap_or_default();

                // Keep only the classes the feed is looking for (live-updatable).
                {
                    let guard = class_filter.read().unwrap();
                    if let Some(set) = guard.as_ref() {
                        detections.retain(|d| set.contains(&d.class));
                    }
                }

                // Assign stable track_ids on top of the engine's raw detections.
                if let Some(t) = tracker.as_mut() {
                    t.update(&mut detections);
                }

                let inference_ms = val
                    .get("inference_ms")
                    .and_then(|v| v.as_f64())
                    .unwrap_or(0.0);

                let ws_msg = WsDetectionMessage {
                    feed_id: feed_id.clone(),
                    engine: engine_name.clone(),
                    timestamp_ms: frame.timestamp_ms,
                    inference_ms,
                    detections,
                };

                // Publish to WebSocket subscribers (ignore if no receivers)
                let _ = detection_tx.send(ws_msg);
            }
            Err(e) => {
                tracing::error!("Bridge {engine_name}/{feed_id}: recv error: {e}");
                break;
            }
        }
    }

    // Cleanup: remove feed from engine
    if let Some(process) = registry.get_process(&engine_name) {
        let mut proc = process.lock().await;
        let remove_msg = serde_json::json!({
            "cmd": "remove_feed",
            "feed_id": feed_id,
        });
        let _ = protocol::send_control(&mut proc.stream, &remove_msg).await;
        let _ = protocol::recv_message(&mut proc.stream).await;
    }

    tracing::info!("Bridge {engine_name}/{feed_id}: stopped");
}
