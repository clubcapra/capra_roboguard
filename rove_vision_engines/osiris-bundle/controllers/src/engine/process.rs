//! Engine process lifecycle: spawn Python .venv, connect via UDS, initialize.

use std::path::{Path, PathBuf};
use std::time::Duration;

use serde_json::json;
use tokio::net::UnixStream;
use tokio::process::{Child, Command};

use crate::engine::protocol::{self, ProtocolMessage};
use crate::error::OsirisError;
use crate::models::manifest::Manifest;

/// A running engine process with its UDS connection.
pub struct EngineProcess {
    pub child: Child,
    pub stream: UnixStream,
    pub socket_path: PathBuf,
    pub metadata: serde_json::Value,
}

impl EngineProcess {
    /// Spawn an engine process and perform the initialization handshake.
    ///
    /// The launch step is runtime-specific (python venv vs. compiled rust binary),
    /// but everything after the process is up — waiting for the `.ready` marker,
    /// connecting over UDS, and the `initialize` handshake — is identical.
    pub async fn spawn(engine_dir: &Path, manifest: Manifest) -> Result<Self, OsirisError> {
        let name = manifest.engine.name.clone();

        // Generate unique socket path
        let socket_name = format!("osiris_{}_{}.sock", name, uuid::Uuid::new_v4());
        let socket_path = std::env::temp_dir().join(&socket_name);

        // Clean up stale socket
        if socket_path.exists() {
            let _ = std::fs::remove_file(&socket_path);
        }

        // Launch the engine process for its declared/inferred runtime.
        let runtime = manifest.runtime(engine_dir);
        tracing::info!(
            "Spawning {runtime:?} engine '{name}' with socket {}",
            socket_path.display()
        );

        let child = match runtime {
            crate::models::manifest::EngineRuntime::Python => {
                launch_python(engine_dir, &manifest, &socket_path).await?
            }
            crate::models::manifest::EngineRuntime::Rust => {
                launch_rust(engine_dir, &manifest, &socket_path).await?
            }
        };

        // Wait for the ready signal (the .ready file)
        let ready_path = PathBuf::from(format!("{}.ready", socket_path.display()));
        wait_for_ready(&ready_path).await?;

        // Connect to the engine's UDS
        let mut stream = UnixStream::connect(&socket_path).await.map_err(|e| {
            OsirisError::EngineStartFailed(format!("Failed to connect to {name} UDS: {e}"))
        })?;

        // Send initialize command
        let classes = manifest.resolve_classes(engine_dir);
        let init_config = json!({
            "weights": manifest.model.weights,
            "device": manifest.model.device,
            "input_size": manifest.model.input_size,
            "confidence_threshold": manifest.inference.confidence_threshold,
            "nms_threshold": manifest.inference.nms_threshold,
            "max_detections": manifest.inference.max_detections,
            "temperature": manifest.inference.temperature,
            "classes": classes,
        });

        let init_msg = json!({
            "cmd": "initialize",
            "config": init_config,
        });

        protocol::send_control(&mut stream, &init_msg).await?;

        // Wait for ready response. First-run engines may download model weights
        // here (e.g. HuggingFace), which can take minutes — hence a generous,
        // env-tunable timeout.
        let response = tokio::time::timeout(
            Duration::from_secs(init_timeout_secs()),
            protocol::recv_message(&mut stream),
        )
        .await
        .map_err(|_| OsirisError::EngineStartFailed(format!("{name}: initialize timeout")))?
        .map_err(|e| OsirisError::EngineStartFailed(format!("{name}: {e}")))?;

        let metadata = match response {
            ProtocolMessage::Control(val) => {
                if let Some(err) = val.get("error") {
                    return Err(OsirisError::EngineStartFailed(format!(
                        "{name}: {err}"
                    )));
                }
                val.get("metadata").cloned().unwrap_or(json!({}))
            }
        };

        tracing::info!("Engine '{name}' initialized successfully");

        Ok(EngineProcess {
            child,
            stream,
            socket_path,
            metadata,
        })
    }

    /// Send a shutdown command and wait for the process to exit.
    pub async fn shutdown(&mut self) {
        let shutdown_msg = json!({"cmd": "shutdown"});
        let _ = protocol::send_control(&mut self.stream, &shutdown_msg).await;

        // Give it a few seconds to exit gracefully
        let _ = tokio::time::timeout(Duration::from_secs(5), self.child.wait()).await;

        // Force kill if still running
        let _ = self.child.kill().await;

        // Clean up socket
        let _ = std::fs::remove_file(&self.socket_path);
        let ready_path = format!("{}.ready", self.socket_path.display());
        let _ = std::fs::remove_file(&ready_path);
    }
}

/// Launch a python engine: ensure the venv + requirements, then spawn `python entry.py <sock>`.
async fn launch_python(
    engine_dir: &Path,
    manifest: &Manifest,
    socket_path: &Path,
) -> Result<Child, OsirisError> {
    let name = &manifest.engine.name;
    let venv_dir = engine_dir.join(".venv");

    // Ensure .venv exists
    if !venv_dir.exists() {
        tracing::info!("Creating venv for engine '{name}'");
        ensure_venv(&venv_dir).await?;
        install_requirements(engine_dir, &venv_dir).await?;
    } else {
        // Venv exists — check if requirements are installed
        // (look for a marker file we write after a successful install)
        let marker = venv_dir.join(".osiris_installed");
        let requirements = engine_dir.join("requirements.txt");

        let needs_install = if !marker.exists() && requirements.exists() {
            true
        } else if marker.exists() && requirements.exists() {
            // Re-install if requirements.txt is newer than marker
            let req_mtime = std::fs::metadata(&requirements)
                .and_then(|m| m.modified()).ok();
            let marker_mtime = std::fs::metadata(&marker)
                .and_then(|m| m.modified()).ok();
            match (req_mtime, marker_mtime) {
                (Some(r), Some(m)) => r > m,
                _ => false,
            }
        } else {
            false
        };

        if needs_install {
            tracing::info!("Installing/updating requirements for engine '{name}'");
            install_requirements(engine_dir, &venv_dir).await?;
        }
    }

    let python = venv_dir.join("bin").join("python");
    let entry_point = engine_dir.join(manifest.python_entry());

    if !entry_point.exists() {
        return Err(OsirisError::EngineStartFailed(format!(
            "Entry point not found: {}",
            entry_point.display()
        )));
    }

    Command::new(&python)
        .arg(&entry_point)
        .arg(socket_path)
        .current_dir(engine_dir)
        .env("PYTHONUNBUFFERED", "1")
        .kill_on_drop(true)
        .spawn()
        .map_err(|e| OsirisError::EngineStartFailed(format!("Failed to spawn {name}: {e}")))
}

/// Launch a rust engine: ensure the release binary is built, then spawn `<bin> <sock>`.
async fn launch_rust(
    engine_dir: &Path,
    manifest: &Manifest,
    socket_path: &Path,
) -> Result<Child, OsirisError> {
    let name = &manifest.engine.name;
    let bin_name = manifest.rust_bin();
    let binary = engine_dir.join("target").join("release").join(bin_name);

    if rust_needs_build(engine_dir, &binary) {
        tracing::info!("Building rust engine '{name}' (cargo build --release)");
        ensure_built(engine_dir).await?;
    }

    if !binary.exists() {
        return Err(OsirisError::EngineStartFailed(format!(
            "Rust engine binary not found after build: {}",
            binary.display()
        )));
    }

    Command::new(&binary)
        .arg(socket_path)
        .current_dir(engine_dir)
        .kill_on_drop(true)
        .spawn()
        .map_err(|e| OsirisError::EngineStartFailed(format!("Failed to spawn {name}: {e}")))
}

/// True if the release binary is missing or any source file / Cargo.toml is newer than it.
fn rust_needs_build(engine_dir: &Path, binary: &Path) -> bool {
    let bin_mtime = match std::fs::metadata(binary).and_then(|m| m.modified()) {
        Ok(t) => t,
        Err(_) => return true, // no binary yet
    };

    let mut newest_src: Option<std::time::SystemTime> = None;
    let mut consider = |p: &Path| {
        if let Ok(t) = std::fs::metadata(p).and_then(|m| m.modified()) {
            if newest_src.map_or(true, |n| t > n) {
                newest_src = Some(t);
            }
        }
    };

    consider(&engine_dir.join("Cargo.toml"));
    // Walk src/ recursively for the newest source mtime.
    let mut stack = vec![engine_dir.join("src")];
    while let Some(dir) = stack.pop() {
        if let Ok(entries) = std::fs::read_dir(&dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                } else {
                    consider(&path);
                }
            }
        }
    }

    match newest_src {
        Some(src) => src > bin_mtime,
        None => false,
    }
}

async fn ensure_built(engine_dir: &Path) -> Result<(), OsirisError> {
    let status = Command::new("cargo")
        .args(["build", "--release"])
        .current_dir(engine_dir)
        .status()
        .await
        .map_err(|e| OsirisError::EngineStartFailed(format!("cargo build: {e}")))?;

    if !status.success() {
        return Err(OsirisError::EngineStartFailed(
            "cargo build --release failed".into(),
        ));
    }
    Ok(())
}

async fn ensure_venv(venv_dir: &Path) -> Result<(), OsirisError> {
    let status = Command::new("python3")
        .args(["-m", "venv", &venv_dir.to_string_lossy()])
        .status()
        .await
        .map_err(|e| OsirisError::EngineStartFailed(format!("venv creation: {e}")))?;

    if !status.success() {
        return Err(OsirisError::EngineStartFailed(
            "python3 -m venv failed".into(),
        ));
    }
    Ok(())
}

async fn install_requirements(engine_dir: &Path, venv_dir: &Path) -> Result<(), OsirisError> {
    let requirements = engine_dir.join("requirements.txt");
    if !requirements.exists() {
        return Ok(());
    }

    let python = venv_dir.join("bin").join("python");
    let pip = venv_dir.join("bin").join("pip");

    // Use a TMPDIR on the engine's parent disk (engines/.pip_tmp) to avoid
    // running out of space on small /tmp tmpfs setups.
    let tmpdir = engine_dir.join(".pip_tmp");
    let _ = std::fs::create_dir_all(&tmpdir);

    // Prefer the GPU-aware installer (detects the NVIDIA driver's CUDA version
    // and installs matching torch/onnxruntime wheels, or CPU wheels). It lives
    // with the shared engine template — same path engines use for OsirisEngine,
    // so it's present in dev and inside exported bundles.
    let installer = engine_dir
        .join("..")
        .join("..")
        .join("templates")
        .join("engine_template")
        .join("osiris_install.py");

    let status = if installer.exists() {
        tracing::info!(
            "Installing requirements (GPU-aware) for engine in {}",
            engine_dir.display()
        );
        Command::new(&python)
            .arg(&installer)
            .arg(&requirements)
            .env("TMPDIR", &tmpdir)
            .env("PIP_NO_CACHE_DIR", "1")
            .current_dir(engine_dir)
            .status()
            .await
            .map_err(|e| OsirisError::EngineStartFailed(format!("gpu-aware install: {e}")))?
    } else {
        tracing::info!("Installing requirements for engine in {}", engine_dir.display());
        Command::new(&pip)
            .args(["install", "-r", &requirements.to_string_lossy()])
            .env("TMPDIR", &tmpdir)
            .env("PIP_NO_CACHE_DIR", "1")
            .current_dir(engine_dir)
            .status()
            .await
            .map_err(|e| OsirisError::EngineStartFailed(format!("pip install: {e}")))?
    };

    // Cleanup tmp dir
    let _ = std::fs::remove_dir_all(&tmpdir);

    if !status.success() {
        return Err(OsirisError::EngineStartFailed(
            "pip install -r requirements.txt failed".into(),
        ));
    }

    // Write marker file so we know requirements are installed
    let marker = venv_dir.join(".osiris_installed");
    let _ = std::fs::write(&marker, "");

    Ok(())
}

async fn wait_for_ready(ready_path: &Path) -> Result<(), OsirisError> {
    let start = std::time::Instant::now();
    let timeout = Duration::from_secs(ready_timeout_secs());

    loop {
        if ready_path.exists() {
            return Ok(());
        }
        if start.elapsed() > timeout {
            return Err(OsirisError::EngineStartFailed(
                "Timed out waiting for engine ready signal".into(),
            ));
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

/// Seconds to wait for an engine's `.ready` signal (process start + heavy
/// imports like torch). Override with `OSIRIS_ENGINE_READY_TIMEOUT`.
fn ready_timeout_secs() -> u64 {
    std::env::var("OSIRIS_ENGINE_READY_TIMEOUT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(120)
}

/// Seconds to wait for an engine's `initialize` response. First-run engines may
/// download model weights here (minutes on a slow link). Override with
/// `OSIRIS_ENGINE_INIT_TIMEOUT`.
fn init_timeout_secs() -> u64 {
    std::env::var("OSIRIS_ENGINE_INIT_TIMEOUT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(900)
}
