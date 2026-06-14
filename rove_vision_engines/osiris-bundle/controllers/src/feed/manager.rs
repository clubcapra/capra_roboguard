//! Feed lifecycle management: create, stop, and list active video feeds.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, RwLock};

use tokio::sync::{broadcast, Mutex};

use crate::engine::registry::EngineRegistry;
use crate::error::OsirisError;
use crate::feed::decoder;
use crate::feed::distributor;
use crate::feed::distributor::ClassFilter;
use crate::feed::tracker::TrackerConfig;
use crate::models::detection::WsDetectionMessage;
use crate::models::feed::{FeedConfig, FeedCreate, FeedInfo, FeedStatus};

/// Build a class-filter set from a request's class list.
/// `None` or an empty list means "report every class".
fn make_filter(classes: &Option<Vec<String>>) -> Option<HashSet<String>> {
    match classes {
        Some(list) if !list.is_empty() => Some(list.iter().cloned().collect()),
        _ => None,
    }
}

/// Internal state for an active feed.
struct ActiveFeed {
    config: FeedConfig,
    /// Broadcast sender for decoded frames (kept alive to prevent channel closing).
    _frame_tx: broadcast::Sender<decoder::Frame>,
    /// Per-engine detection broadcast senders.
    detection_txs: HashMap<String, broadcast::Sender<WsDetectionMessage>>,
    /// Shared, live-updatable "classes to look for" filter (all bridges read it).
    class_filter: ClassFilter,
    /// Decoder task handle.
    _decoder_handle: tokio::task::JoinHandle<()>,
}

/// Manages all active video feeds.
pub struct FeedManager {
    feeds: Mutex<HashMap<String, ActiveFeed>>,
    engines: Arc<EngineRegistry>,
}

impl FeedManager {
    pub fn new(engines: Arc<EngineRegistry>) -> Self {
        Self {
            feeds: Mutex::new(HashMap::new()),
            engines,
        }
    }

    /// Resolve the ByteTrack config for an engine on a new feed.
    ///
    /// Manifest `[tracking]` is the default. A feed's `track` field overrides it:
    /// `Some(false)` force-disables; `Some(true)` force-enables (using manifest
    /// params if present, otherwise defaults).
    fn resolve_tracker(&self, engine_name: &str, override_on: Option<bool>) -> Option<TrackerConfig> {
        if override_on == Some(false) {
            return None;
        }
        let section = self.engines.manifest(engine_name).and_then(|m| m.tracking);
        match section {
            Some(s) if s.enabled || override_on == Some(true) => Some(TrackerConfig::from_section(&s)),
            None if override_on == Some(true) => Some(TrackerConfig::default()),
            _ => None,
        }
    }

    /// Create a new feed: start decoding and bridge to engines.
    pub async fn create_feed(&self, req: FeedCreate) -> Result<FeedInfo, OsirisError> {
        // Validate engines exist and are ready
        for engine_name in &req.engines {
            if !self.engines.is_ready(engine_name) {
                return Err(OsirisError::EngineNotFound(engine_name.clone()));
            }
        }

        let feed_id = uuid::Uuid::new_v4().to_string();

        // Resolve the source the decoder actually consumes. `webrtc://<stream>` is
        // rewritten to the RTSP republish of the bundled MediaMTX gateway so a
        // WebRTC publisher "just works"; everything else passes through unchanged.
        let resolved_source = normalize_source(&req.source);

        // Probe source for resolution
        let (width, height) = decoder::probe_source(&resolved_source)
            .await
            .map_err(|e| OsirisError::FeedSourceError(format!("probe: {e}")))?;

        // Start decoder
        let (frame_tx, decoder_handle) =
            decoder::start_decoding(feed_id.clone(), resolved_source, width, height);

        // Shared class filter, read live by every bridge on this feed.
        let class_filter: ClassFilter = Arc::new(RwLock::new(make_filter(&req.classes)));

        // Start bridge tasks per engine
        let mut detection_txs = HashMap::new();
        for engine_name in &req.engines {
            let frame_rx = frame_tx.subscribe();
            let tracker_cfg = self.resolve_tracker(engine_name, req.track);
            let detection_tx = distributor::start_bridge(
                engine_name.clone(),
                feed_id.clone(),
                frame_rx,
                self.engines.clone(),
                width,
                height,
                tracker_cfg,
                class_filter.clone(),
            );
            detection_txs.insert(engine_name.clone(), detection_tx);
        }

        let config = FeedConfig {
            id: feed_id.clone(),
            source: req.source,
            engines: req.engines,
            status: FeedStatus::Running,
            width,
            height,
            classes: req.classes,
        };

        let info = FeedInfo::from(&config);

        let active_feed = ActiveFeed {
            config,
            _frame_tx: frame_tx,
            detection_txs,
            class_filter,
            _decoder_handle: decoder_handle,
        };

        self.feeds.lock().await.insert(feed_id, active_feed);

        Ok(info)
    }

    /// Stop and remove a feed.
    pub async fn delete_feed(&self, feed_id: &str) -> Result<(), OsirisError> {
        let mut feeds = self.feeds.lock().await;
        let feed = feeds
            .remove(feed_id)
            .ok_or_else(|| OsirisError::FeedNotFound(feed_id.to_string()))?;

        // Dropping the frame_tx will close the broadcast channel,
        // which will cause all bridge tasks to stop.
        // The decoder handle will be aborted when dropped.
        feed._decoder_handle.abort();

        tracing::info!("Feed '{feed_id}' stopped");
        Ok(())
    }

    /// Update a running feed's "classes to look for" filter live (no restart).
    /// `None`/empty reverts to reporting every class. Returns the updated info.
    pub async fn set_classes(
        &self,
        feed_id: &str,
        classes: Option<Vec<String>>,
    ) -> Result<FeedInfo, OsirisError> {
        let mut feeds = self.feeds.lock().await;
        let feed = feeds
            .get_mut(feed_id)
            .ok_or_else(|| OsirisError::FeedNotFound(feed_id.to_string()))?;

        // Swap the shared filter — bridges pick it up on their next frame.
        *feed.class_filter.write().unwrap() = make_filter(&classes);
        feed.config.classes = classes;

        tracing::info!("Feed '{feed_id}' class filter updated: {:?}", feed.config.classes);
        Ok(FeedInfo::from(&feed.config))
    }

    /// List all active feeds.
    pub async fn list_feeds(&self) -> Vec<FeedInfo> {
        self.feeds
            .lock()
            .await
            .values()
            .map(|f| FeedInfo::from(&f.config))
            .collect()
    }

    /// Subscribe to detections for a specific engine+feed pair.
    pub async fn subscribe_detections(
        &self,
        engine_name: &str,
        feed_id: &str,
    ) -> Result<broadcast::Receiver<WsDetectionMessage>, OsirisError> {
        let feeds = self.feeds.lock().await;
        let feed = feeds
            .get(feed_id)
            .ok_or_else(|| OsirisError::FeedNotFound(feed_id.to_string()))?;

        let tx = feed
            .detection_txs
            .get(engine_name)
            .ok_or_else(|| {
                OsirisError::EngineNotFound(format!(
                    "Engine '{engine_name}' not assigned to feed '{feed_id}'"
                ))
            })?;

        Ok(tx.subscribe())
    }
}

/// Rewrite a feed source into the URL the decoder consumes.
///
/// `webrtc://<stream>` -> `<gateway>/<stream>` where `<gateway>` is
/// `$OSIRIS_WEBRTC_GATEWAY` (default `rtsp://127.0.0.1:8554`). This points the
/// ffmpeg decoder at the RTSP republish of the bundled MediaMTX gateway, which a
/// WebRTC publisher feeds via WHIP. All other sources pass through verbatim.
fn normalize_source(source: &str) -> String {
    if let Some(stream) = source.strip_prefix("webrtc://") {
        let gateway = std::env::var("OSIRIS_WEBRTC_GATEWAY")
            .unwrap_or_else(|_| "rtsp://127.0.0.1:8554".to_string());
        let gateway = gateway.trim_end_matches('/');
        let stream = stream.trim_start_matches('/');
        format!("{gateway}/{stream}")
    } else {
        source.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::normalize_source;

    #[test]
    fn webrtc_rewrites_to_default_gateway() {
        // Note: relies on OSIRIS_WEBRTC_GATEWAY being unset in the test env.
        assert_eq!(
            normalize_source("webrtc://mystream"),
            "rtsp://127.0.0.1:8554/mystream"
        );
    }

    #[test]
    fn non_webrtc_passes_through() {
        assert_eq!(normalize_source("rtsp://cam/live"), "rtsp://cam/live");
        assert_eq!(normalize_source("/dev/video0"), "/dev/video0");
        assert_eq!(normalize_source("/tmp/clip.mp4"), "/tmp/clip.mp4");
    }
}
