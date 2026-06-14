use serde::{Deserialize, Serialize};
use utoipa::ToSchema;

/// Parsed manifest.toml for a vision engine.
#[derive(Debug, Clone, Deserialize)]
pub struct Manifest {
    pub engine: EngineSection,
    pub model: ModelSection,
    pub inference: InferenceSection,
    pub classes: ClassesSection,
    /// Optional ByteTrack tracking layer applied by the orchestrator.
    #[serde(default)]
    pub tracking: Option<TrackingSection>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct EngineSection {
    pub name: String,
    pub version: String,
    pub description: String,
    /// Execution runtime: "python" (default) or "rust". When absent, the loader
    /// infers it from the engine directory (presence of Cargo.toml -> rust).
    #[serde(default)]
    pub runtime: Option<String>,
    /// Python entry script (relative to the engine dir). Defaults to "engine.py".
    /// Only used by the python runtime.
    #[serde(default)]
    pub python_entry: Option<String>,
    /// Built binary name for rust engines (the `[[bin]]` name in Cargo.toml).
    /// Defaults to "osiris-rust-engine". Only used by the rust runtime.
    #[serde(default)]
    pub bin: Option<String>,
}

/// Which runtime an engine process uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineRuntime {
    Python,
    Rust,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ModelSection {
    #[serde(rename = "type")]
    pub model_type: String,
    pub weights: String,
    pub input_size: [u32; 2],
    pub device: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct InferenceSection {
    pub confidence_threshold: f64,
    pub nms_threshold: f64,
    pub max_detections: u32,
    /// Logit temperature for confidence calibration. Logits are divided by this
    /// before sigmoid; values > 1 raise (spread up) under-confident scores.
    #[serde(default = "default_temperature")]
    pub temperature: f64,
}

fn default_temperature() -> f64 {
    1.0
}

#[derive(Debug, Clone, Deserialize)]
pub struct ClassesSection {
    #[serde(default)]
    pub names: Vec<String>,
    pub file: Option<String>,
}

/// ByteTrack tracking config. Lives with the engine so it travels into exports.
#[derive(Debug, Clone, Deserialize)]
pub struct TrackingSection {
    /// Master switch. When false, no track_ids are assigned for this engine.
    #[serde(default)]
    pub enabled: bool,
    #[serde(default = "t_high")]
    pub high_thresh: f64,
    #[serde(default = "t_low")]
    pub low_thresh: f64,
    #[serde(default = "t_iou_high")]
    pub iou_high: f64,
    #[serde(default = "t_iou_low")]
    pub iou_low: f64,
    #[serde(default = "t_new")]
    pub new_track_thresh: f64,
    #[serde(default = "t_buffer")]
    pub track_buffer: u32,
    #[serde(default)]
    pub min_box_area: f64,
    /// Restrict tracking to these classes (e.g. ["person"]). Omit to track all.
    #[serde(default)]
    pub classes: Option<Vec<String>>,
}

fn t_high() -> f64 { 0.5 }
fn t_low() -> f64 { 0.1 }
fn t_iou_high() -> f64 { 0.2 }
fn t_iou_low() -> f64 { 0.5 }
fn t_new() -> f64 { 0.6 }
fn t_buffer() -> u32 { 30 }

/// Engine info returned by the API.
#[derive(Debug, Clone, Serialize, ToSchema)]
pub struct EngineInfo {
    pub name: String,
    pub version: String,
    pub description: String,
    pub model_type: String,
    pub classes: Vec<String>,
    pub input_size: [u32; 2],
    pub status: String,
    pub ws_endpoint_template: String,
    /// Whether ByteTrack tracking is enabled for this engine.
    pub tracking: bool,
}

impl Manifest {
    /// Whether the orchestrator should run ByteTrack for this engine.
    pub fn tracking_enabled(&self) -> bool {
        self.tracking.as_ref().is_some_and(|t| t.enabled)
    }

    /// Determine the execution runtime for this engine.
    ///
    /// An explicit `[engine] runtime = "..."` wins. Otherwise the runtime is
    /// inferred from the engine directory: a `Cargo.toml` means a rust engine,
    /// anything else falls back to python.
    pub fn runtime(&self, engine_dir: &std::path::Path) -> EngineRuntime {
        match self.engine.runtime.as_deref() {
            Some("rust") => EngineRuntime::Rust,
            Some("python") => EngineRuntime::Python,
            _ => {
                if engine_dir.join("Cargo.toml").exists() {
                    EngineRuntime::Rust
                } else {
                    EngineRuntime::Python
                }
            }
        }
    }

    /// Python entry script name (relative to the engine dir). Defaults to "engine.py".
    pub fn python_entry(&self) -> &str {
        self.engine
            .python_entry
            .as_deref()
            .unwrap_or("engine.py")
    }

    /// Built binary name for a rust engine. Defaults to "osiris-rust-engine".
    pub fn rust_bin(&self) -> &str {
        self.engine.bin.as_deref().unwrap_or("osiris-rust-engine")
    }

    /// Resolve class names, loading from file if needed.
    pub fn resolve_classes(&self, engine_dir: &std::path::Path) -> Vec<String> {
        if !self.classes.names.is_empty() {
            return self.classes.names.clone();
        }
        if let Some(ref file) = self.classes.file {
            let path = engine_dir.join(file);
            if let Ok(content) = std::fs::read_to_string(&path) {
                return content
                    .lines()
                    .map(|l| l.trim().to_string())
                    .filter(|l| !l.is_empty())
                    .collect();
            }
        }
        Vec::new()
    }
}
