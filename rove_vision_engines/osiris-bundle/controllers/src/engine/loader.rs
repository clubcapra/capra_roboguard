//! Load vision engines from the vision_engines directory.

use std::path::Path;
use std::sync::Arc;

use crate::engine::process::EngineProcess;
use crate::engine::registry::EngineRegistry;
use crate::models::manifest::Manifest;

/// Load a single engine from its directory.
pub async fn load_engine(
    engine_dir: &Path,
    registry: &EngineRegistry,
) -> anyhow::Result<()> {
    let manifest_path = engine_dir.join("manifest.toml");
    if !manifest_path.exists() {
        tracing::warn!(
            "Skipping {}: no manifest.toml",
            engine_dir.display()
        );
        return Ok(());
    }

    let manifest_str = std::fs::read_to_string(&manifest_path)?;
    let manifest: Manifest = toml::from_str(&manifest_str)
        .map_err(|e| anyhow::anyhow!("Failed to parse {}: {e}", manifest_path.display()))?;

    let name = manifest.engine.name.clone();
    tracing::info!("Loading engine '{name}' from {}", engine_dir.display());

    registry.register_loading(&name, manifest.clone());

    match EngineProcess::spawn(engine_dir, manifest).await {
        Ok(process) => {
            registry.set_ready(&name, process);
            tracing::info!("Engine '{name}' is ready");
        }
        Err(e) => {
            let err_msg = e.to_string();
            tracing::error!("Engine '{name}' failed to start: {err_msg}");
            registry.set_error(&name, err_msg);
        }
    }

    Ok(())
}

/// Scan the vision_engines directory and load all engines.
pub async fn load_all_engines(
    engines_dir: &Path,
    registry: &Arc<EngineRegistry>,
) -> anyhow::Result<()> {
    if !engines_dir.exists() {
        tracing::warn!(
            "Vision engines directory does not exist: {}",
            engines_dir.display()
        );
        return Ok(());
    }

    let mut entries = std::fs::read_dir(engines_dir)?;
    while let Some(entry) = entries.next() {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            if let Err(e) = load_engine(&path, registry).await {
                tracing::error!(
                    "Failed to load engine from {}: {e}",
                    path.display()
                );
            }
        }
    }

    Ok(())
}
