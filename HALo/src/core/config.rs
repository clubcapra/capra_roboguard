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

/// Shared configuration contract. Hardware rules and SDK translation remain in
/// the driver; the framework handles TOML parsing and instance-relative paths.
pub trait DriverConfig: DeserializeOwned {
    fn validate(&self) -> Result<(), String>;
    fn resolve_paths(&mut self, _base: &Path) {}
    fn reserved_udp_ports(&self) -> Result<Vec<u16>, String> {
        Ok(Vec::new())
    }
}

pub fn parse_driver_config<T: DriverConfig>(text: &str, path: &Path) -> Result<T, ConfigError> {
    let mut value: T = toml::from_str(text).map_err(|source| ConfigError::Parse {
        path: path.into(),
        source,
    })?;
    value.resolve_paths(path.parent().unwrap_or_else(|| Path::new(".")));
    value
        .validate()
        .map_err(|error| ConfigError::Invalid(format!("{}: {error}", path.display())))?;
    Ok(value)
}

pub fn load_driver_config<T: DriverConfig>(name: &str) -> Result<Option<T>, ConfigError> {
    let path = config_path(name)?;
    let text = match fs::read_to_string(&path) {
        Ok(text) => text,
        Err(source) if source.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(source) => return Err(ConfigError::Io { path, source }),
    };
    parse_driver_config(&text, &path).map(Some)
}

const CONFIG_SUBPATH: &str = "config";

#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("invalid configuration: {0}")]
    Invalid(String),
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
            if let Some(dir) = std::env::var_os("HALO_CONFIG_DIR") {
                return Some(PathBuf::from(dir));
            }
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

/// Persist a complete configuration before publishing the new runtime state.
/// The temporary file is on the same filesystem; rename is atomic.
pub fn atomic_write(path: &Path, bytes: &[u8]) -> io::Result<()> {
    use std::io::Write;
    use std::os::unix::fs::OpenOptionsExt;
    let parent = path
        .parent()
        .ok_or_else(|| io::Error::other("no parent directory"))?;
    fs::create_dir_all(parent)?;
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let temp = path.with_extension(format!(
        "{}.{}.tmp",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let mut f = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(&temp)?;
    use std::os::unix::fs::PermissionsExt;
    f.set_permissions(fs::Permissions::from_mode(0o600))?;
    let result = (|| {
        f.write_all(bytes)?;
        f.sync_all()?;
        fs::rename(&temp, path)?;
        fs::File::open(parent)?.sync_all()
    })();
    if result.is_err() {
        let _ = fs::remove_file(temp);
    }
    result
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
