use crate::core::config::DriverConfig;

#[derive(Debug, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct OdriveConfig {
    pub interface: String,
    pub discovery_ms: u64,
}

impl Default for OdriveConfig {
    fn default() -> Self {
        Self {
            interface: std::env::var("CAN_IFACE").unwrap_or_else(|_| "can0".into()),
            discovery_ms: 2_000,
        }
    }
}

impl DriverConfig for OdriveConfig {
    fn validate(&self) -> Result<(), String> {
        if self.interface.is_empty() || !(100..=30_000).contains(&self.discovery_ms) {
            return Err("invalid CAN interface or discovery interval".into());
        }
        Ok(())
    }
}
