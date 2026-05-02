/// ODrive flat_endpoints.json — live-updateable shared endpoint map with auto-fetch.
///
/// ## Priority order (first match wins):
///
/// 1. `ODRIVE_ENDPOINTS=/path/to/flat_endpoints.json`   — explicit local file
/// 2. `ODRIVE_HW_VERSION` + `ODRIVE_FW_VERSION`         — auto-fetch from ODrive CDN
///    (cached at `~/.cache/capra-rove/`; works offline once cached)
/// 3. Neither — wait for `POST /odrive/endpoints` upload in Scalar
///
/// ## Env vars:
///
/// | Variable            | Example   | Description                                    |
/// |---------------------|-----------|------------------------------------------------|
/// | `ODRIVE_HW_VERSION` | `4.4.58`  | `product_line.hw_version.variant` from board   |
/// | `ODRIVE_FW_VERSION` | `0.6.11`  | Firmware version, or `latest` for newest stable|
/// | `ODRIVE_ENDPOINTS`  | `/tmp/f.json` | Override: skip auto-fetch, use this file   |
/// | `ODRIVE_CACHE_DIR`  | `/tmp`    | Cache dir (default: `~/.cache/capra-rove`)     |
///
/// ## Getting your hardware version string:
/// - ODrive Pro v4.4-58V  → `4.4.58`
/// - ODrive S1 X4         → `5.2.0`
/// Run `GET /odrive_N/config` — the error will prompt you if unset.
use std::collections::HashMap;
use std::io;
use std::path::PathBuf;
use std::sync::{Arc, RwLock};

use serde::Deserialize;

// ── Endpoint info ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Deserialize, serde::Serialize)]
pub struct EndpointInfo {
    pub id: u16,
    #[serde(rename = "type")]
    pub dtype: String,
    /// Access mode from flat_endpoints.json: "r", "rw", or "" for functions.
    #[serde(default)]
    pub access: String,
}

#[derive(Debug, Deserialize)]
struct FlatEndpointsFile {
    endpoints: HashMap<String, EndpointInfo>,
}

// ── Shared map type ───────────────────────────────────────────────────────────

pub type SharedEndpointMap = Arc<RwLock<HashMap<String, EndpointInfo>>>;

pub fn new_shared() -> SharedEndpointMap {
    Arc::new(RwLock::new(HashMap::new()))
}

// ── Parse / load ─────────────────────────────────────────────────────────────

fn parse_endpoints_str(
    content: &str,
) -> Result<HashMap<String, EndpointInfo>, Box<dyn std::error::Error + Send + Sync>> {
    let file: FlatEndpointsFile = serde_json::from_str(content)
        .map_err(|e| format!("cannot parse endpoints JSON: {e}"))?;
    Ok(file.endpoints)
}

pub fn load_from_file(
    shared: &SharedEndpointMap,
    path: &str,
) -> Result<usize, Box<dyn std::error::Error + Send + Sync>> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read '{path}': {e}"))?;
    let map = parse_endpoints_str(&content)?;
    let count = map.len();
    *shared.write().unwrap() = map;
    tracing::info!(path, count, "loaded ODrive flat_endpoints.json");
    Ok(count)
}

pub fn load_from_str(
    shared: &SharedEndpointMap,
    content: &str,
) -> Result<usize, Box<dyn std::error::Error + Send + Sync>> {
    let map = parse_endpoints_str(content)?;
    let count = map.len();
    *shared.write().unwrap() = map;
    tracing::info!(count, "ODrive endpoint map updated");
    Ok(count)
}

// ── ODrive CDN auto-fetch via curl ────────────────────────────────────────────

const FIRMWARE_INDEX_URL: &str = "https://api.odriverobotics.com/releases/firmware/index";
const CDN_BASE: &str = "https://odrive-cdn.nyc3.digitaloceanspaces.com/releases/firmware";

#[derive(Debug, Deserialize)]
struct FirmwareIndex {
    commits: Vec<FirmwareCommit>,
    channels: Vec<FirmwareChannel>,
}

#[derive(Debug, Deserialize)]
struct FirmwareCommit {
    commit_hash: String,
    board: Vec<u32>,
    content: String,
    #[serde(default)]
    index: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct FirmwareChannel {
    channel: String,
    commits: Vec<String>,
}

/// Parse "A.B.C" hw version string into numeric components.
fn parse_hw_version(s: &str) -> Option<[u32; 3]> {
    let p: Vec<&str> = s.split('.').collect();
    if p.len() != 3 { return None; }
    Some([p[0].parse().ok()?, p[1].parse().ok()?, p[2].parse().ok()?])
}

/// Resolve "latest" to the last public stable version from the master channel.
fn resolve_fw_version<'a>(index: &'a FirmwareIndex, requested: &'a str) -> &'a str {
    if requested != "latest" { return requested; }
    index.channels.iter()
        .find(|c| c.channel == "master")
        .and_then(|c| c.commits.last())
        .map(|s| s.as_str())
        .unwrap_or(requested)
}

/// Determine cache path for a hw/fw combination.
fn cache_path(hw: &str, fw: &str) -> PathBuf {
    let dir = std::env::var("ODRIVE_CACHE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            dirs::cache_dir()
                .unwrap_or_else(|| PathBuf::from("/tmp"))
                .join("capra-rove")
        });
    let hw_safe = hw.replace('.', "_");
    let fw_safe = fw.replace(['.', '-'], "_");
    dir.join(format!("flat_endpoints_hw{hw_safe}_fw{fw_safe}.json"))
}

/// Fetch a URL via `curl` and return the response body as a String.
async fn curl_get(url: &str) -> io::Result<String> {
    let output = tokio::process::Command::new("curl")
        .args(["--silent", "--location", "--max-time", "15", "--fail", url])
        .output()
        .await?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(io::Error::new(
            io::ErrorKind::Other,
            format!("curl failed ({}): {}", output.status, stderr.trim()),
        ));
    }
    String::from_utf8(output.stdout)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
}

/// Auto-fetch flat_endpoints.json from the ODrive CDN, with local caching.
///
/// - `hw_version`: e.g. `"4.4.58"` (ODrive Pro v4.4-58V), `"5.2.0"` (S1 X4)
/// - `fw_version`: e.g. `"0.6.11"`, `"0.6.11-1"`, or `"latest"`
pub async fn auto_fetch_endpoints(
    shared: &SharedEndpointMap,
    hw_version: &str,
    fw_version: &str,
) -> Result<usize, Box<dyn std::error::Error + Send + Sync>> {
    let hw = parse_hw_version(hw_version).ok_or_else(|| {
        format!("invalid ODRIVE_HW_VERSION '{hw_version}' — expected A.B.C e.g. 4.4.58")
    })?;

    let cache = cache_path(hw_version, fw_version);

    // --- Offline cache hit ---
    if cache.exists() {
        tracing::info!(
            path = %cache.display(), hw_version, fw_version,
            "loading ODrive endpoints from cache (offline-safe)"
        );
        let content = std::fs::read_to_string(&cache)
            .map_err(|e| format!("cache read failed: {e}"))?;
        return load_from_str(shared, &content);
    }

    // --- Fetch release index ---
    tracing::info!(hw_version, fw_version, "fetching ODrive firmware release index");
    let index_json = curl_get(FIRMWARE_INDEX_URL).await.map_err(|e| {
        format!(
            "cannot reach ODrive CDN ({e}). \
             Set ODRIVE_HW_VERSION + ODRIVE_FW_VERSION to auto-fetch when online, \
             or upload flat_endpoints.json via POST /odrive/endpoints"
        )
    })?;

    let index: FirmwareIndex = serde_json::from_str(&index_json)
        .map_err(|e| format!("cannot parse firmware index: {e}"))?;

    let resolved_fw = resolve_fw_version(&index, fw_version);
    tracing::info!(resolved_fw, "resolved firmware version");

    // --- Find matching commit ---
    let commit = index.commits.iter().find(|c| {
        c.commit_hash == resolved_fw
            && c.board.len() >= 3
            && c.board[0] == hw[0]
            && c.board[1] == hw[1]
            && c.board[2] == hw[2]
            && c.index.iter().any(|f| f == "flat_endpoints.json")
    }).ok_or_else(|| format!(
        "no flat_endpoints.json found in index for hw={hw_version} fw={resolved_fw}. \
         Verify ODRIVE_HW_VERSION (e.g. 4.4.58 for ODrive Pro, 5.2.0 for S1 X4) \
         and ODRIVE_FW_VERSION (e.g. 0.6.11 or latest)."
    ))?;

    let url = format!("{CDN_BASE}/{}/flat_endpoints.json", commit.content);
    tracing::info!(%url, "downloading ODrive flat_endpoints.json");

    let content = curl_get(&url).await
        .map_err(|e| format!("download failed: {e}"))?;

    // --- Save to cache for offline use ---
    if let Some(parent) = cache.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    match std::fs::write(&cache, &content) {
        Ok(_) => tracing::info!(path = %cache.display(), "cached ODrive endpoints for offline use"),
        Err(e) => tracing::warn!(path = %cache.display(), error = %e, "failed to cache endpoints"),
    }

    load_from_str(shared, &content)
}
