//! File system watcher for hot-reloading vision engines.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use notify::{Config, Event, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use tokio::sync::mpsc;

use crate::engine::loader::load_engine;
use crate::engine::registry::EngineRegistry;

pub struct EngineWatcher {
    _watcher: RecommendedWatcher,
}

impl EngineWatcher {
    /// Start watching the vision_engines directory for changes.
    pub fn start(
        engines_dir: PathBuf,
        registry: Arc<EngineRegistry>,
    ) -> anyhow::Result<Self> {
        let (tx, rx) = mpsc::channel::<Event>(100);

        // Use default config — on Linux this uses inotify (event-driven, no polling).
        // Only fires when files actually change.
        let mut watcher = RecommendedWatcher::new(
            move |result: Result<Event, notify::Error>| {
                if let Ok(event) = result {
                    let _ = tx.blocking_send(event);
                }
            },
            Config::default(),
        )?;

        // Watch non-recursively to avoid .venv/pip noise triggering reloads.
        // We only care about manifest.toml in direct child directories.
        watcher.watch(&engines_dir, RecursiveMode::NonRecursive)?;
        // Also watch each engine subdirectory (one level deep) for manifest.toml changes
        if let Ok(entries) = std::fs::read_dir(&engines_dir) {
            for entry in entries.flatten() {
                if entry.path().is_dir() {
                    let _ = watcher.watch(&entry.path(), RecursiveMode::NonRecursive);
                }
            }
        }

        tracing::info!(
            "Watching for engine changes in {}",
            engines_dir.display()
        );

        // Spawn the event processing task
        tokio::spawn(process_events(rx, engines_dir, registry));

        Ok(Self { _watcher: watcher })
    }
}

async fn process_events(
    mut rx: mpsc::Receiver<Event>,
    engines_dir: PathBuf,
    registry: Arc<EngineRegistry>,
) {
    // Debounce: collect events for 2 seconds before processing
    let mut pending_engines: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut debounce_timer: Option<tokio::time::Instant> = None;

    loop {
        tokio::select! {
            event = rx.recv() => {
                let Some(event) = event else { break };

                // Only react to create/modify events on manifest.toml
                let dominated = matches!(
                    event.kind,
                    EventKind::Create(_) | EventKind::Modify(_)
                );
                if !dominated { continue; }

                for path in &event.paths {
                    let is_manifest = path.file_name()
                        .is_some_and(|f| f == "manifest.toml");

                    if !is_manifest {
                        continue;
                    }

                    if let Some(engine_name) = extract_engine_name(path, &engines_dir) {
                        pending_engines.insert(engine_name);
                        debounce_timer = Some(tokio::time::Instant::now() + Duration::from_secs(2));
                    }
                }
            }
            _ = async {
                if let Some(deadline) = debounce_timer {
                    tokio::time::sleep_until(deadline).await;
                } else {
                    // Sleep forever if no timer is set
                    std::future::pending::<()>().await;
                }
            } => {
                debounce_timer = None;

                for engine_name in pending_engines.drain() {
                    let engine_dir = engines_dir.join(&engine_name);
                    let manifest_path = engine_dir.join("manifest.toml");

                    if manifest_path.exists() {
                        tracing::info!("Hot-reloading engine '{engine_name}'");
                        // Unregister old version first
                        registry.unregister(&engine_name).await;
                        // Load the new version
                        if let Err(e) = load_engine(&engine_dir, &registry).await {
                            tracing::error!("Failed to reload engine '{engine_name}': {e}");
                        }
                    } else if !engine_dir.exists() {
                        tracing::info!("Engine '{engine_name}' removed, unregistering");
                        registry.unregister(&engine_name).await;
                    }
                }
            }
        }
    }
}

/// Extract the engine name from a path inside the engines directory.
fn extract_engine_name(path: &std::path::Path, engines_dir: &std::path::Path) -> Option<String> {
    let relative = path.strip_prefix(engines_dir).ok()?;
    let first_component = relative.components().next()?;
    match first_component {
        std::path::Component::Normal(name) => Some(name.to_string_lossy().to_string()),
        _ => None,
    }
}
