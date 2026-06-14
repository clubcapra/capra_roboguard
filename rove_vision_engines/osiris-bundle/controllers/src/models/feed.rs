use serde::{Deserialize, Serialize};
use utoipa::ToSchema;

/// Request body to create a new feed.
#[derive(Debug, Clone, Deserialize, ToSchema)]
pub struct FeedCreate {
    /// Video source: RTSP URL, HTTP stream, file path, or device (e.g. "/dev/video0")
    pub source: String,
    /// List of engine names to run on this feed
    pub engines: Vec<String>,
    /// Override ByteTrack tracking: `true` force-on, `false` force-off.
    /// When omitted, each engine's manifest `[tracking]` setting applies.
    #[serde(default)]
    pub track: Option<bool>,
    /// Only report detections of these classes (e.g. ["person", "knife"]).
    /// Omit or leave empty to report every class. Can be changed live via
    /// `PUT /api/feeds/{id}/classes`.
    #[serde(default)]
    pub classes: Option<Vec<String>>,
}

/// Body for updating a running feed's class filter.
#[derive(Debug, Clone, Deserialize, ToSchema)]
pub struct FeedClasses {
    /// Classes to look for. `null` or empty reports every class.
    #[serde(default)]
    pub classes: Option<Vec<String>>,
}

/// Feed configuration stored internally.
#[derive(Debug, Clone)]
pub struct FeedConfig {
    pub id: String,
    pub source: String,
    pub engines: Vec<String>,
    pub status: FeedStatus,
    pub width: u32,
    pub height: u32,
    pub classes: Option<Vec<String>>,
}

/// Current status of a video feed.
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum FeedStatus {
    Starting,
    Running,
    Stopped,
    Error,
}

/// Feed info returned by the API.
#[derive(Debug, Clone, Serialize, ToSchema)]
pub struct FeedInfo {
    pub id: String,
    pub source: String,
    pub engines: Vec<String>,
    pub status: FeedStatus,
    pub width: u32,
    pub height: u32,
    /// Active class filter; `null` means all classes are reported.
    pub classes: Option<Vec<String>>,
}

impl From<&FeedConfig> for FeedInfo {
    fn from(cfg: &FeedConfig) -> Self {
        FeedInfo {
            id: cfg.id.clone(),
            source: cfg.source.clone(),
            engines: cfg.engines.clone(),
            status: cfg.status.clone(),
            width: cfg.width,
            height: cfg.height,
            classes: cfg.classes.clone(),
        }
    }
}
