//! Replace this mock transport with verified USB/BLE hardware I/O later.
use super::{
    config::RadiacodeConfig,
    sensor::{RadiacodeDriver, RadiacodeTransport},
};
use crate::core::error::DriverError;
use crate::drivers::sim::feed::SimFeed;
use serde_json::{json, Value};

pub struct MockHardware {
    dose: f64,
}
impl RadiacodeTransport for MockHardware {
    fn label(&self) -> &'static str {
        "mock hardware"
    }
    fn latest(&self) -> Result<Value, DriverError> {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        Ok(
            json!({"dose_rate_usv_h":self.dose,"background_usv_h":self.dose,"source_count":0,
                  "x_m":0.0,"y_m":0.0,"z_m":0.0,"connected":true,"error_code":0,
                  "transport":"mock","timestamp_ns":stamp}),
        )
    }
}

pub fn connect_mock(config: &RadiacodeConfig) -> RadiacodeDriver {
    RadiacodeDriver::new(
        config.id.clone(),
        Box::new(MockHardware {
            dose: config.mock_dose_rate_usv_h,
        }),
    )
}

struct Simulator {
    feed: SimFeed,
}
impl RadiacodeTransport for Simulator {
    fn label(&self) -> &'static str {
        "simulation"
    }
    fn latest(&self) -> Result<Value, DriverError> {
        Ok(self.feed.latest_or_empty())
    }
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        Box::pin(async move {
            self.feed.stop().await;
            Ok(())
        })
    }
}

pub fn connect_sim(config: &RadiacodeConfig, host: &str, backend_port: u16) -> RadiacodeDriver {
    RadiacodeDriver::new(
        config.id.clone(),
        Box::new(Simulator {
            feed: SimFeed::subscribe(host, backend_port),
        }),
    )
}
