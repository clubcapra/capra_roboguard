use serde::{Deserialize, Serialize};
use utoipa::ToSchema;

/// A single object detection result.
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct Detection {
    /// Detected class name
    pub class: String,
    /// Confidence score [0.0, 1.0]
    pub confidence: f64,
    /// Bounding box in pixel coordinates
    pub bbox: BBox,
    /// Optional tracking ID
    pub track_id: Option<u64>,
    /// Optional keypoints (for pose estimation engines)
    /// Each keypoint is [x, y, visibility]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub keypoints: Option<Vec<Vec<f64>>>,
}

/// Bounding box: top-left corner + dimensions.
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct BBox {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

/// WebSocket message sent to clients.
#[derive(Debug, Clone, Serialize)]
pub struct WsDetectionMessage {
    pub feed_id: String,
    pub engine: String,
    pub timestamp_ms: u64,
    pub inference_ms: f64,
    pub detections: Vec<Detection>,
}

impl Detection {
    pub fn from_json_value(val: &serde_json::Value) -> Option<Self> {
        let class = val.get("class")?.as_str()?.to_string();
        let confidence = val.get("confidence")?.as_f64()?;
        let bbox_arr = val.get("bbox")?.as_array()?;
        if bbox_arr.len() < 4 {
            return None;
        }
        let bbox = BBox {
            x: bbox_arr[0].as_f64()?,
            y: bbox_arr[1].as_f64()?,
            width: bbox_arr[2].as_f64()?,
            height: bbox_arr[3].as_f64()?,
        };
        let track_id = val
            .get("track_id")
            .and_then(|v| v.as_u64());

        // Parse optional keypoints (list of [x, y, visibility])
        let keypoints = val.get("keypoints").and_then(|kp| kp.as_array()).map(|arr| {
            arr.iter()
                .filter_map(|pt| {
                    pt.as_array().map(|coords| {
                        coords.iter().filter_map(|c| c.as_f64()).collect::<Vec<f64>>()
                    })
                })
                .collect::<Vec<Vec<f64>>>()
        });

        Some(Detection {
            class,
            confidence,
            bbox,
            track_id,
            keypoints,
        })
    }
}
