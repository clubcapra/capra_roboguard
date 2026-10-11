use crate::core::config::DriverConfig;
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RadiacodeConfig {
    pub id: String,
    #[serde(default)]
    pub transport: Transport,
    #[serde(default = "default_dose")]
    pub mock_dose_rate_usv_h: f64,
}

#[derive(Clone, Debug, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Transport {
    #[default]
    Mock,
}

fn default_dose() -> f64 {
    0.1
}

impl DriverConfig for RadiacodeConfig {
    fn validate(&self) -> Result<(), String> {
        if self.id.is_empty()
            || !self
                .id
                .bytes()
                .all(|c| c.is_ascii_alphanumeric() || c == b'_')
        {
            return Err("Radiacode id requires letters, numbers or underscores".into());
        }
        if !self.mock_dose_rate_usv_h.is_finite() || self.mock_dose_rate_usv_h < 0.0 {
            return Err("mock dose rate must be finite and nonnegative".into());
        }
        Ok(())
    }
}
