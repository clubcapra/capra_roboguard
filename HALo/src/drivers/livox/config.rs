use crate::core::config::{self, DriverConfig};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::net::{Ipv4Addr, SocketAddr};
use std::path::{Path, PathBuf};

/// One TOML config. Vendor JSON is an internal, session-owned SDK artifact.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LivoxConfig {
    #[serde(default = "default_library")]
    pub library: PathBuf,
    pub host_ip: Ipv4Addr,
    pub lidar_ips: Vec<Ipv4Addr>,
    pub multicast_ip: Option<Ipv4Addr>,
    #[serde(default = "default_stale_ms")]
    pub stale_ms: u64,
    #[serde(default = "default_host_ports")]
    pub host_ports: NetworkPorts,
    #[serde(default = "default_lidar_ports")]
    pub lidar_ports: NetworkPorts,
    /// Compatibility opt-ins for older binary consumers; standard subscribers
    /// discover their bindings and require no output destinations.
    pub points_target: Option<SocketAddr>,
    pub imu_target: Option<SocketAddr>,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct NetworkPorts {
    pub cmd_data_port: u16,
    pub push_msg_port: u16,
    pub point_data_port: u16,
    pub imu_data_port: u16,
    pub log_data_port: u16,
}
impl NetworkPorts {
    fn values(&self) -> Vec<u16> {
        vec![
            self.cmd_data_port,
            self.push_msg_port,
            self.point_data_port,
            self.imu_data_port,
            self.log_data_port,
        ]
    }
    fn valid(&self) -> bool {
        let values = self.values();
        !values.contains(&0) && values.iter().collect::<HashSet<_>>().len() == values.len()
    }
}
fn default_library() -> PathBuf {
    PathBuf::from("../src/drivers/livox/native/build/libhalo_livox.so")
}
fn default_stale_ms() -> u64 {
    1000
}
fn default_host_ports() -> NetworkPorts {
    NetworkPorts {
        cmd_data_port: 56101,
        push_msg_port: 56201,
        point_data_port: 56301,
        imu_data_port: 56401,
        log_data_port: 56501,
    }
}
fn default_lidar_ports() -> NetworkPorts {
    NetworkPorts {
        cmd_data_port: 56100,
        push_msg_port: 56200,
        point_data_port: 56300,
        imu_data_port: 56400,
        log_data_port: 56500,
    }
}
impl DriverConfig for LivoxConfig {
    fn validate(&self) -> Result<(), String> {
        let unicast =
            |ip: &Ipv4Addr| !ip.is_unspecified() && !ip.is_multicast() && !ip.is_broadcast();
        let ids = self.lidar_ids();
        if !unicast(&self.host_ip)
            || self.lidar_ips.is_empty()
            || !self.lidar_ips.iter().all(unicast)
            || ids.iter().collect::<HashSet<_>>().len() != ids.len()
            || self.multicast_ip.is_some_and(|ip| !ip.is_multicast())
            || self.stale_ms == 0
            || !self.host_ports.valid()
            || !self.lidar_ports.valid()
            || self.library.as_os_str().is_empty()
        {
            return Err("Livox requires a unicast host, lidar IPs with unique final octets, valid SDK ports, optional multicast address and positive freshness timeout".into());
        }
        Ok(())
    }
    fn resolve_paths(&mut self, base: &Path) {
        if self.library.is_relative() {
            self.library = base.join(&self.library);
        }
    }
    fn reserved_udp_ports(&self) -> Result<Vec<u16>, String> {
        Ok(self.host_ports.values())
    }
}
impl LivoxConfig {
    pub fn load(path: &Path) -> Result<Option<Self>, config::ConfigError> {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(source) if source.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(source) => {
                return Err(config::ConfigError::Io {
                    path: path.into(),
                    source,
                })
            }
        };
        config::parse_driver_config(&text, path).map(Some)
    }
    pub fn lidar_ids(&self) -> Vec<u8> {
        self.lidar_ips.iter().map(|ip| ip.octets()[3]).collect()
    }
    fn sdk_document(&self) -> serde_json::Value {
        let mut host = serde_json::to_value(&self.host_ports).expect("serializable SDK ports");
        host["host_ip"] = serde_json::json!(self.host_ip);
        host["lidar_ip"] = serde_json::json!(self.lidar_ips);
        if let Some(ip) = self.multicast_ip {
            host["multicast_ip"] = serde_json::json!(ip);
        }
        serde_json::json!({ "MID360": { "lidar_net_info": self.lidar_ports, "host_net_info": [host] } })
    }
    pub(crate) fn sdk_file(&self) -> std::io::Result<SdkConfigFile> {
        use rand_core::RngCore;
        use std::io::Write;
        use std::os::unix::fs::OpenOptionsExt;
        let mut nonce = [0u8; 16];
        rand_core::OsRng.fill_bytes(&mut nonce);
        let name: String = nonce.iter().map(|byte| format!("{byte:02x}")).collect();
        let path = std::env::temp_dir().join(format!("halo-livox-{name}.json"));
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .open(&path)?;
        let owned = SdkConfigFile { path };
        file.write_all(&serde_json::to_vec(&self.sdk_document())?)?;
        file.sync_all()?;
        Ok(owned)
    }
}
/// Retained until SDK teardown and removed on both failure and normal shutdown.
pub(crate) struct SdkConfigFile {
    pub path: PathBuf,
}
impl Drop for SdkConfigFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn single_toml_generates_private_sdk_input_and_reserves_host_ports() {
        let c: LivoxConfig = config::parse_driver_config(
            "host_ip = '192.0.2.1'\nlidar_ips = ['192.0.2.10', '192.0.2.11']\n",
            Path::new("/tmp/instance/livox.toml"),
        )
        .unwrap();
        assert_eq!(c.lidar_ids(), [10, 11]);
        assert_eq!(
            c.reserved_udp_ports().unwrap(),
            [56101, 56201, 56301, 56401, 56501]
        );
        assert!(c.library.starts_with("/tmp/instance"));
        let sdk = c.sdk_file().unwrap();
        let value: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&sdk.path).unwrap()).unwrap();
        assert_eq!(value["MID360"]["host_net_info"][0]["host_ip"], "192.0.2.1");
        assert_eq!(
            value["MID360"]["host_net_info"][0]["lidar_ip"][1],
            "192.0.2.11"
        );
        use std::os::unix::fs::PermissionsExt;
        assert_eq!(
            std::fs::metadata(&sdk.path).unwrap().permissions().mode() & 0o777,
            0o600
        );
        let path = sdk.path.clone();
        drop(sdk);
        assert!(!path.exists());
        assert!(config::parse_driver_config::<LivoxConfig>(
            "host_ip = '192.0.2.1'\nlidar_ips = ['192.0.2.10', '198.51.100.10']\n",
            Path::new("livox.toml"),
        )
        .is_err());
    }
}
