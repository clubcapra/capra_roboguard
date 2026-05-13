//! Shared helpers for locating and loading per-driver TOML configuration.
//!
//! Each driver that needs static configuration owns a `<driver>.toml` file
//! living in the project's `config/` directory (next to `Cargo.toml`). This
//! module finds that directory at runtime so a renamed checkout, a stale
//! incremental build, or a deployed binary in `target/release/` all keep
//! working without recompilation.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use serde::de::DeserializeOwned;

const CONFIG_SUBPATH: &str = "config";

#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("config file not found: {0}")]
    NotFound(PathBuf),
    #[error("could not locate config/ directory (looked from CARGO_MANIFEST_DIR and current_exe)")]
    DirNotFound,
    #[error("reading {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("parsing {path}: {source}")]
    Parse {
        path: PathBuf,
        #[source]
        source: toml::de::Error,
    },
}

/// Locate the project's `config/` directory. Cached after the first call.
///
/// Resolution order:
/// 1. `$CARGO_MANIFEST_DIR/config` — set by `cargo run`, authoritative when
///    the binary was launched via cargo.
/// 2. Walk up from `current_exe()` looking for a `config/` sibling — handles
///    `target/{debug,release}/<bin>` and other deployed layouts.
pub fn config_dir() -> Option<&'static Path> {
    static CACHED: OnceLock<Option<PathBuf>> = OnceLock::new();
    CACHED
        .get_or_init(|| {
            if let Ok(manifest) = std::env::var("CARGO_MANIFEST_DIR") {
                let p = PathBuf::from(manifest).join(CONFIG_SUBPATH);
                if p.is_dir() {
                    return Some(p);
                }
            }
            let exe = std::env::current_exe().ok()?;
            let mut cursor = exe.parent()?;
            loop {
                let candidate = cursor.join(CONFIG_SUBPATH);
                if candidate.is_dir() {
                    return Some(candidate);
                }
                cursor = cursor.parent()?;
            }
        })
        .as_deref()
}

/// Resolve the path to a single config file (e.g. `"kinova.toml"`) without
/// checking whether it exists.
pub fn config_path(name: &str) -> Result<PathBuf, ConfigError> {
    Ok(config_dir().ok_or(ConfigError::DirNotFound)?.join(name))
}

/// Load and parse a config file. Returns `Ok(None)` when the file is absent
/// (so callers can treat "no config" as "skip this driver" without having
/// to disambiguate IO errors from missing-file).
pub fn load_optional<T: DeserializeOwned>(name: &str) -> Result<Option<T>, ConfigError> {
    let path = config_path(name)?;
    if !path.exists() {
        return Ok(None);
    }
    let text = fs::read_to_string(&path).map_err(|source| ConfigError::Io {
        path: path.clone(),
        source,
    })?;
    let value = toml::from_str(&text).map_err(|source| ConfigError::Parse {
        path: path.clone(),
        source,
    })?;
    Ok(Some(value))
}
