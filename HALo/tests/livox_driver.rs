use std::net::UdpSocket;
use std::path::Path;
use std::process::Command;
use std::time::Duration;

use halo::core::registry::SensorRegistry;
use halo::drivers::livox::{self, config::LivoxConfig};

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn native_driver_discovery_streams_freshness_and_shared_session_lifetime() {
    let temp = std::env::temp_dir().join(format!("rove-livox-test-{}", std::process::id()));
    std::fs::create_dir_all(&temp).unwrap();
    let library = temp.join("liblivox_test.so");
    let rustc = std::env::var("RUSTC").unwrap_or_else(|_| "rustc".into());
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/livox_native.rs");
    assert!(Command::new(rustc)
        .args(["--crate-type=cdylib", "--edition=2021"])
        .arg(fixture)
        .arg("-o")
        .arg(&library)
        .status()
        .unwrap()
        .success());
    let points = UdpSocket::bind("127.0.0.1:0").unwrap();
    let imu = UdpSocket::bind("127.0.0.1:0").unwrap();
    points
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    imu.set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    let config = || {
        let text = format!("host_ip = '192.0.2.1'\nlidar_ips = ['192.0.2.40', '192.0.2.41']\nlibrary = '{}'\npoints_target = '{}'\nimu_target = '{}'\n", library.display(), points.local_addr().unwrap(), imu.local_addr().unwrap());
        halo::core::config::parse_driver_config::<LivoxConfig>(&text, &temp.join("livox.toml"))
            .unwrap()
    };
    let observer = unsafe { libloading::Library::new(&library).unwrap() };
    let stops: libloading::Symbol<extern "C" fn() -> usize> =
        unsafe { observer.get(b"halo_livox_stop_count").unwrap() };
    let config_path: libloading::Symbol<unsafe extern "C" fn(*mut u8, usize) -> usize> =
        unsafe { observer.get(b"halo_livox_config_path").unwrap() };
    let registry = SensorRegistry::new(5000);
    for driver in livox::connect(config()).unwrap() {
        registry.register(driver);
    }
    assert_eq!(registry.len(), 2);
    assert_eq!(registry.ports("livox_40"), Some((5000, 5001)));
    assert_eq!(registry.ports("livox_41"), Some((5002, 5003)));
    let driver = registry.get("livox_40").unwrap();
    let data = driver.read_data().unwrap();
    assert_eq!(data["connected"], true);
    assert_eq!(data["imu_connected"], true);
    assert_eq!(data["points"], serde_json::json!([[1.0, 2.0, 3.0]]));
    assert_eq!(data["point_packets"], 1);
    assert_eq!(data["imu_packets"], 1);
    let top = registry.get("livox_41").unwrap().read_data().unwrap();
    assert_eq!(top["points"], serde_json::json!([[4.0, 5.0, 6.0]]));
    assert_eq!(top["imu_connected"], false);
    assert!(driver.execute_command(&serde_json::json!({})).is_err());
    let mut buffer = [0_u8; 256];
    for id in [40, 41] {
        let n = points.recv(&mut buffer).unwrap();
        assert_eq!(&buffer[..4], b"LVXR");
        assert_eq!((n, buffer[5], buffer[6]), (32, 1, id));
        assert_eq!(
            f32::from_le_bytes(buffer[20..24].try_into().unwrap()),
            if id == 40 { 1.0 } else { 4.0 }
        );
    }
    assert!(points.recv(&mut buffer).is_err());
    let n = imu.recv(&mut buffer).unwrap();
    assert_eq!((n, buffer[5], buffer[6]), (44, 2, 40));
    // Attempting another SDK session must not shut down the existing one.
    assert!(livox::connect(config()).is_err());
    let mut first = driver.subscribe_updates().unwrap();
    let mut second = driver.subscribe_updates().unwrap();
    let emitter = unsafe { libloading::Library::new(&library).unwrap() };
    let emit: libloading::Symbol<unsafe extern "C" fn()> =
        unsafe { emitter.get(b"halo_livox_emit").unwrap() };
    unsafe { emit() };
    for part in 0..4 {
        let a = tokio::time::timeout(Duration::from_secs(1), first.recv())
            .await
            .unwrap()
            .unwrap();
        let b = tokio::time::timeout(Duration::from_secs(1), second.recv())
            .await
            .unwrap()
            .unwrap();
        let (halo::core::driver::DataUpdate::Data(a), halo::core::driver::DataUpdate::Data(b)) =
            (a, b)
        else {
            panic!("unexpected dropped data");
        };
        assert_eq!(a, b);
        assert_eq!(a["packet_type"], "points");
        assert_eq!(a["point_packet_index"], part);
        assert_eq!(a["point_packet_parts"], 4);
        assert_eq!(a["points"].as_array().unwrap().len(), 128);
        assert_eq!(a["points"][0][0].as_f64(), Some((part * 128) as f64));
    }
    let halo::core::driver::DataUpdate::Data(imu_data) = first.recv().await.unwrap() else {
        panic!("dropped IMU");
    };
    assert_eq!(imu_data["packet_type"], "imu");
    assert_eq!(imu_data["points"], serde_json::json!([]));
    let mut large = vec![0u8; 65535];
    assert_eq!(points.recv(&mut large).unwrap(), 20 + 512 * 12);
    assert_eq!(imu.recv(&mut large).unwrap(), 44);
    drop(emitter);
    tokio::time::sleep(Duration::from_millis(1000)).await;
    assert_eq!(driver.read_data().unwrap()["connected"], false);
    assert_eq!(driver.read_data().unwrap()["imu_connected"], false);
    let mut path_buffer = [0u8; 4096];
    let path_len = unsafe { config_path(path_buffer.as_mut_ptr(), path_buffer.len()) };
    let generated_config =
        std::path::PathBuf::from(std::str::from_utf8(&path_buffer[..path_len]).unwrap());
    assert!(generated_config.is_file());
    drop(registry);
    assert_eq!(stops(), 0); // An outstanding sensor reference keeps SDK alive.
    assert!(generated_config.is_file());
    drop(driver);
    assert_eq!(stops(), 1);
    assert!(!generated_config.exists());
    let n = points.recv(&mut buffer).unwrap();
    assert_eq!((n, buffer[6]), (32, 40)); // Callback during stop still had valid Rust state.
                                          // Failed startup leaves no session behind, allowing a clean subsequent start.
    let mut failing = config();
    failing.host_ip = "192.0.2.254".parse().unwrap();
    assert!(livox::connect(failing).is_err());
    assert_eq!(stops(), 1);
    drop(livox::connect(config()).unwrap());
    assert_eq!(stops(), 2);
    drop(observer);
    std::fs::remove_dir_all(temp).unwrap();
}
