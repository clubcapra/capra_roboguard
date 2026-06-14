//! Video frame extraction using ffmpeg as a subprocess.
//!
//! Decodes video from various sources (RTSP, HTTP, files, USB cameras)
//! into raw BGR frames by piping `ffmpeg -f rawvideo` to stdout.
//! This avoids the libclang build dependency that ffmpeg-next requires.

use std::process::Stdio;
use std::sync::Arc;

use tokio::io::AsyncReadExt;
use tokio::process::Command;
use tokio::sync::broadcast;

/// A decoded video frame (raw BGR24 pixels).
#[derive(Debug, Clone)]
pub struct Frame {
    pub data: Arc<Vec<u8>>,
    pub timestamp_ms: u64,
}

/// Start decoding a video source in a background task.
///
/// Returns the broadcast sender (for subscribing) and a handle to stop decoding.
pub fn start_decoding(
    feed_id: String,
    source: String,
    width: u32,
    height: u32,
) -> (broadcast::Sender<Frame>, tokio::task::JoinHandle<()>) {
    let (tx, _) = broadcast::channel::<Frame>(4);
    let tx_clone = tx.clone();

    let handle = tokio::spawn(async move {
        if let Err(e) = decode_loop(&feed_id, &source, width, height, &tx_clone).await {
            tracing::error!("Decoder for feed '{feed_id}' stopped: {e}");
        }
    });

    (tx, handle)
}

/// Extra ffmpeg/ffprobe input-demuxer args for a given source.
///
/// Linux USB cameras (`/dev/video*`) need an explicit `-f v4l2`; bare `-i /dev/videoN`
/// is unreliable. Everything else (RTSP/HTTP/file) lets ffmpeg auto-detect.
fn input_format_args(source: &str) -> Vec<String> {
    if source.starts_with("/dev/video") {
        vec!["-f".to_string(), "v4l2".to_string()]
    } else {
        Vec::new()
    }
}

async fn decode_loop(
    feed_id: &str,
    source: &str,
    width: u32,
    height: u32,
    tx: &broadcast::Sender<Frame>,
) -> anyhow::Result<()> {
    // Build ffmpeg command to output raw BGR24 frames to stdout
    let mut cmd = Command::new("ffmpeg");
    cmd.args([
        "-hide_banner",
        "-loglevel", "warning",
        // Input options — for RTSP/streams, reduce latency
        "-fflags", "+nobuffer",
        "-flags", "low_delay",
    ]);
    // Demuxer hint for the input (e.g. -f v4l2 for USB cameras on Linux).
    cmd.args(input_format_args(source));
    cmd.args([
        "-i", source,
        // Output: raw BGR24 video frames to stdout
        "-f", "rawvideo",
        "-pix_fmt", "bgr24",
        "-s", &format!("{width}x{height}"),
        "-an",  // no audio
        "-",    // output to stdout
    ]);

    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());
    cmd.kill_on_drop(true);

    let mut child = cmd.spawn()?;

    let mut stdout = child
        .stdout
        .take()
        .ok_or_else(|| anyhow::anyhow!("Failed to capture ffmpeg stdout"))?;

    let frame_size = (width * height * 3) as usize; // BGR24
    let mut buffer = vec![0u8; frame_size];
    let mut frame_count: u64 = 0;

    tracing::info!("Feed '{feed_id}': decoding {source} at {width}x{height}");

    loop {
        // Read exactly one frame
        match stdout.read_exact(&mut buffer).await {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                tracing::info!("Feed '{feed_id}': end of stream");
                break;
            }
            Err(e) => {
                return Err(e.into());
            }
        }

        let frame = Frame {
            data: Arc::new(buffer.clone()),
            timestamp_ms: frame_count * 33, // approximate at ~30fps
        };

        // If no receivers, just drop the frame
        let _ = tx.send(frame);

        frame_count += 1;
        if frame_count % 300 == 0 {
            tracing::debug!("Feed '{feed_id}': decoded {frame_count} frames");
        }
    }

    let _ = child.kill().await;

    tracing::info!("Feed '{feed_id}': decoding complete ({frame_count} frames)");
    Ok(())
}

/// Probe a video source for its resolution using ffprobe.
pub async fn probe_source(source: &str) -> anyhow::Result<(u32, u32)> {
    let mut cmd = Command::new("ffprobe");
    cmd.args([
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "csv=p=0:s=x",
    ]);
    cmd.args(input_format_args(source));
    cmd.arg(source);
    let output = cmd.output().await?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(anyhow::anyhow!("ffprobe failed: {stderr}"));
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    let parts: Vec<&str> = stdout.trim().split('x').collect();
    if parts.len() < 2 {
        return Err(anyhow::anyhow!(
            "Unexpected ffprobe output: {stdout}"
        ));
    }

    let width: u32 = parts[0].parse()?;
    let height: u32 = parts[1].parse()?;

    Ok((width, height))
}
