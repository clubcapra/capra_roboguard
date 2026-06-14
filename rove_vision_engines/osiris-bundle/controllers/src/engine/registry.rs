//! Concurrent registry of active vision engines.

use std::sync::Arc;

use dashmap::DashMap;
use tokio::sync::Mutex;

use crate::engine::process::EngineProcess;
use crate::models::manifest::{EngineInfo, Manifest};

/// Status of an engine in the registry.
#[derive(Debug, Clone)]
pub enum EngineStatus {
    Loading,
    Ready,
    Error(String),
}

impl std::fmt::Display for EngineStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EngineStatus::Loading => write!(f, "loading"),
            EngineStatus::Ready => write!(f, "ready"),
            EngineStatus::Error(e) => write!(f, "error: {e}"),
        }
    }
}

/// Handle to a registered engine.
pub struct EngineHandle {
    pub name: String,
    pub manifest: Manifest,
    pub status: EngineStatus,
    pub process: Option<Arc<Mutex<EngineProcess>>>,
    pub metadata: serde_json::Value,
}

/// Thread-safe registry of all loaded engines.
pub struct EngineRegistry {
    engines: DashMap<String, EngineHandle>,
}

impl EngineRegistry {
    pub fn new() -> Self {
        Self {
            engines: DashMap::new(),
        }
    }

    /// Register an engine as loading (before spawn).
    pub fn register_loading(&self, name: &str, manifest: Manifest) {
        self.engines.insert(
            name.to_string(),
            EngineHandle {
                name: name.to_string(),
                manifest,
                status: EngineStatus::Loading,
                process: None,
                metadata: serde_json::Value::Null,
            },
        );
    }

    /// Mark an engine as ready with its running process.
    pub fn set_ready(&self, name: &str, process: EngineProcess) {
        if let Some(mut entry) = self.engines.get_mut(name) {
            entry.metadata = process.metadata.clone();
            entry.status = EngineStatus::Ready;
            entry.process = Some(Arc::new(Mutex::new(process)));
        }
    }

    /// Mark an engine as errored.
    pub fn set_error(&self, name: &str, error: String) {
        if let Some(mut entry) = self.engines.get_mut(name) {
            entry.status = EngineStatus::Error(error);
            entry.process = None;
        }
    }

    /// Remove an engine from the registry, shutting it down if running.
    pub async fn unregister(&self, name: &str) {
        if let Some((_, handle)) = self.engines.remove(name) {
            if let Some(process) = handle.process {
                let mut proc = process.lock().await;
                proc.shutdown().await;
            }
        }
    }

    /// Get a cloneable reference to an engine's process.
    pub fn get_process(&self, name: &str) -> Option<Arc<Mutex<EngineProcess>>> {
        self.engines
            .get(name)
            .and_then(|h| h.process.clone())
    }

    /// Check if an engine exists and is ready.
    pub fn is_ready(&self, name: &str) -> bool {
        self.engines
            .get(name)
            .is_some_and(|h| matches!(h.status, EngineStatus::Ready))
    }

    /// List all engines with their info.
    pub fn list(&self) -> Vec<EngineInfo> {
        self.engines
            .iter()
            .map(|entry| {
                let h = entry.value();
                EngineInfo {
                    name: h.name.clone(),
                    version: h.manifest.engine.version.clone(),
                    description: h.manifest.engine.description.clone(),
                    model_type: h.manifest.model.model_type.clone(),
                    classes: h.manifest.classes.names.clone(),
                    input_size: h.manifest.model.input_size,
                    status: h.status.to_string(),
                    ws_endpoint_template: format!("/ws/{}/{{feed_id}}", h.name),
                    tracking: h.manifest.tracking_enabled(),
                }
            })
            .collect()
    }

    /// Clone an engine's parsed manifest (used to resolve tracking config).
    pub fn manifest(&self, name: &str) -> Option<Manifest> {
        self.engines.get(name).map(|h| h.manifest.clone())
    }
}
