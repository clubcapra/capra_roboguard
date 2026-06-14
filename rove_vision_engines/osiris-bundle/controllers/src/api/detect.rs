//! Single-image detection endpoint.
//!
//! POST /api/detect/{engine_name}
//!   - Accepts: multipart/form-data with an "image" field, or raw image bytes
//!   - Returns: JSON array of detections
//!
//! This is the universal REST endpoint for running any vision engine
//! on a single image without setting up a video feed.

use std::sync::Arc;

use axum::extract::{Multipart, Path, Query, State};
use axum::routing::post;
use axum::{Json, Router};
use serde::{Deserialize, Serialize};
use utoipa::ToSchema;

use crate::config::AppState;
use crate::engine::protocol;
use crate::error::OsirisError;
use crate::models::detection::Detection;

pub fn router() -> Router<Arc<AppState>> {
    Router::new().route("/detect/{engine_name}", post(detect_image))
}

#[derive(Debug, Deserialize)]
pub struct DetectQuery {
    /// Override confidence threshold (optional)
    pub confidence: Option<f64>,
}

#[derive(Debug, Serialize, ToSchema)]
pub struct DetectResponse {
    pub engine: String,
    pub detections: Vec<Detection>,
    pub inference_ms: f64,
    pub image_width: u32,
    pub image_height: u32,
}

/// Run detection on a single image.
///
/// Accepts multipart/form-data with an "image" field containing the image file.
/// The image is decoded to raw BGR, sent to the engine, and detections are returned.
#[utoipa::path(
    post,
    path = "/api/detect/{engine_name}",
    params(
        ("engine_name" = String, Path, description = "Engine to run detection with"),
    ),
    responses(
        (status = 200, description = "Detection results", body = DetectResponse),
        (status = 404, description = "Engine not found"),
        (status = 400, description = "Invalid image"),
    ),
    tag = "detect"
)]
pub async fn detect_image(
    State(state): State<Arc<AppState>>,
    Path(engine_name): Path<String>,
    Query(query): Query<DetectQuery>,
    mut multipart: Multipart,
) -> Result<Json<DetectResponse>, OsirisError> {
    // Get image bytes from multipart
    let mut image_bytes: Option<Vec<u8>> = None;

    while let Some(field) = multipart
        .next_field()
        .await
        .map_err(|e| OsirisError::FeedSourceError(format!("multipart error: {e}")))?
    {
        let name = field.name().unwrap_or("").to_string();
        if name == "image" || name == "file" {
            let data = field
                .bytes()
                .await
                .map_err(|e| OsirisError::FeedSourceError(format!("read field: {e}")))?;
            image_bytes = Some(data.to_vec());
            break;
        }
    }

    let image_bytes = image_bytes
        .ok_or_else(|| OsirisError::FeedSourceError("No 'image' field in multipart".into()))?;

    // Decode image using ffmpeg (image file -> raw BGR)
    let (bgr_data, width, height) = decode_image_to_bgr(&image_bytes).await?;

    // Get engine process
    let process = state
        .engines
        .get_process(&engine_name)
        .ok_or_else(|| OsirisError::EngineNotFound(engine_name.clone()))?;

    // Use a temporary feed ID for this one-shot detection
    let feed_id = uuid::Uuid::new_v4().to_string();

    let mut proc = process.lock().await;

    // Configure feed dimensions
    let configure_msg = serde_json::json!({
        "cmd": "configure_feed",
        "feed_id": feed_id,
        "width": width,
        "height": height,
        "channels": 3,
    });
    protocol::send_control(&mut proc.stream, &configure_msg).await?;
    let _ = protocol::recv_message(&mut proc.stream).await?;

    // Send frame
    protocol::send_frame(&mut proc.stream, &feed_id, &bgr_data).await?;

    // Receive detections
    let response = protocol::recv_message(&mut proc.stream).await?;

    // Clean up feed
    let remove_msg = serde_json::json!({
        "cmd": "remove_feed",
        "feed_id": feed_id,
    });
    protocol::send_control(&mut proc.stream, &remove_msg).await?;
    let _ = protocol::recv_message(&mut proc.stream).await?;

    // Parse response
    let (detections, inference_ms) = match response {
        protocol::ProtocolMessage::Control(val) => {
            if let Some(err) = val.get("error") {
                return Err(OsirisError::Internal(format!("Engine error: {err}")));
            }

            let mut dets: Vec<Detection> = val
                .get("detections")
                .and_then(|d| d.as_array())
                .map(|arr| {
                    arr.iter()
                        .filter_map(Detection::from_json_value)
                        .collect()
                })
                .unwrap_or_default();

            // Apply confidence override if specified
            if let Some(min_conf) = query.confidence {
                dets.retain(|d| d.confidence >= min_conf);
            }

            let ms = val.get("inference_ms").and_then(|v| v.as_f64()).unwrap_or(0.0);
            (dets, ms)
        }
    };

    Ok(Json(DetectResponse {
        engine: engine_name,
        detections,
        inference_ms,
        image_width: width,
        image_height: height,
    }))
}

/// Decode an image file (JPEG, PNG, etc.) to raw BGR24 pixels using ffmpeg.
async fn decode_image_to_bgr(data: &[u8]) -> Result<(Vec<u8>, u32, u32), OsirisError> {
    use tokio::io::AsyncWriteExt;
    use tokio::process::Command;

    // First, probe dimensions
    let mut probe = Command::new("ffprobe")
        .args([
            "-v", "error",
            "-select_streams", "v:0",
            "-show_entries", "stream=width,height",
            "-of", "csv=p=0:s=x",
            "-i", "pipe:0",
        ])
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| OsirisError::FeedSourceError(format!("ffprobe spawn: {e}")))?;

    let mut probe_stdin = probe.stdin.take().unwrap();
    let data_clone = data.to_vec();
    tokio::spawn(async move {
        let _ = probe_stdin.write_all(&data_clone).await;
        drop(probe_stdin);
    });

    let probe_output = probe
        .wait_with_output()
        .await
        .map_err(|e| OsirisError::FeedSourceError(format!("ffprobe: {e}")))?;

    let dims = String::from_utf8_lossy(&probe_output.stdout);
    let parts: Vec<&str> = dims.trim().split('x').collect();
    if parts.len() < 2 {
        return Err(OsirisError::FeedSourceError(format!(
            "Could not determine image dimensions: {dims}"
        )));
    }
    let width: u32 = parts[0]
        .parse()
        .map_err(|_| OsirisError::FeedSourceError("Invalid width".into()))?;
    let height: u32 = parts[1]
        .parse()
        .map_err(|_| OsirisError::FeedSourceError("Invalid height".into()))?;

    // Decode to raw BGR24
    let mut ffmpeg = Command::new("ffmpeg")
        .args([
            "-hide_banner",
            "-loglevel", "error",
            "-i", "pipe:0",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-frames:v", "1",
            "-",
        ])
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| OsirisError::FeedSourceError(format!("ffmpeg spawn: {e}")))?;

    let mut ffmpeg_stdin = ffmpeg.stdin.take().unwrap();
    let data_clone = data.to_vec();
    tokio::spawn(async move {
        let _ = ffmpeg_stdin.write_all(&data_clone).await;
        drop(ffmpeg_stdin);
    });

    let output = ffmpeg
        .wait_with_output()
        .await
        .map_err(|e| OsirisError::FeedSourceError(format!("ffmpeg: {e}")))?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(OsirisError::FeedSourceError(format!(
            "ffmpeg decode failed: {stderr}"
        )));
    }

    let expected = (width * height * 3) as usize;
    if output.stdout.len() != expected {
        return Err(OsirisError::FeedSourceError(format!(
            "BGR output size mismatch: got {} expected {expected}",
            output.stdout.len()
        )));
    }

    Ok((output.stdout, width, height))
}
