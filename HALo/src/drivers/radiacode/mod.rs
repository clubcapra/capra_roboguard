//! Radiacode dose telemetry, with swappable mock-hardware/simulator transports.
//! Actual USB/BLE protocol, energy spectrum and count calibration are pending.
pub mod config;
pub mod mock;
pub mod sensor;

pub fn connect(config: &config::RadiacodeConfig) -> sensor::RadiacodeDriver {
    match config.transport {
        config::Transport::Mock => mock::connect_mock(config),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{
        config::{parse_driver_config, DriverConfig},
        driver::SensorDriver,
    };
    use serde_json::json;
    #[test]
    fn config_validation_and_shared_mock_schema() {
        let cfg = valid_config();
        cfg.validate().unwrap();
        let driver = connect(&cfg);
        let data = driver.read_data().unwrap();
        assert!(driver.telemetry_status(&data).connected);
        assert_eq!(driver.data_schema().len(), data.as_object().unwrap().len());
        assert!(driver.command_schema().is_empty());
        assert!(driver.execute_command(&json!({"estop":true})).is_err());
        assert!(parse_driver_config::<config::RadiacodeConfig>(
            "id='a'\ntransport='usb'",
            std::path::Path::new("radiacode.toml")
        )
        .is_err());
    }
    fn valid_config() -> config::RadiacodeConfig {
        config::RadiacodeConfig { id:"radiacode_arm".into(), transport:config::Transport::Mock, mock_dose_rate_usv_h:0.1 }
    }
    #[test]
    fn telemetry_validates_measurement_and_translates_errors() {
        let driver = connect(&valid_config());
        assert!(
            !driver
                .telemetry_status(&json!({"timestamp_ns":10,"connected":true}))
                .connected
        );
        let status = driver.telemetry_status(
            &json!({"timestamp_ns":10,"connected":true,"dose_rate_usv_h":1.0,"error_code":10}),
        );
        assert!(status
            .errors
            .iter()
            .any(|s| s.contains("invalid dose-rate")));
        assert!(status.errors.iter().any(|s| s.contains("0x8")));
        assert_eq!(status.sample_timestamp_ns, 10);
    }
    #[tokio::test]
    async fn simulator_feed_preserves_source_data_and_releases_its_port() {
        use crate::protocol::packet::Packet;
        let probe = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        let port = probe.local_addr().unwrap().port();
        drop(probe);
        let driver = mock::connect_sim(&valid_config(), "127.0.0.1", port);
        let sender = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let payload =
            json!({"dose_rate_usv_h":2.5,"connected":true,"error_code":0,"timestamp_ns":123});
        let packet = Packet::data(1, &payload).encode();
        for _ in 0..20 {
            sender.send_to(&packet, ("127.0.0.1", port)).await.unwrap();
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            if driver.read_data().unwrap() == payload {
                break;
            }
        }
        assert_eq!(driver.read_data().unwrap(), payload);
        assert!(driver.telemetry_status(&payload).connected);
        driver.shutdown().await.unwrap();
        let rebound = tokio::net::UdpSocket::bind(("127.0.0.1", port))
            .await
            .unwrap();
        drop(rebound);
    }
    #[test]
    fn simulator_catalogue_reserves_new_backend_without_port_overflow() {
        use crate::drivers::sim::config::{PortsFile, SimConfig};
        let mut sim = SimConfig { host:"127.0.0.1".into(), odrive_max_vel_rev_s:20.0, ports:PortsFile::default() };
        assert!(sim.reserved_udp_ports().unwrap().contains(&6028));
        sim.ports.backend_base = 65508;
        assert!(sim.reserved_udp_ports().is_err());
    }

}
