use axum::{
    body::{to_bytes, Body},
    http::Request,
};
use halo::{
    core::{
        driver::{CommandMode, FieldDescriptor, SensorDriver},
        error::DriverError,
        registry::SensorRegistry,
        slot::DriverSlot,
        supervisor::{Factory, RetryPolicy, Supervisor},
    },
    logging::LogManager,
    protocol::packet::{MessageType, Packet},
};
use serde_json::{json, Value};
use std::{
    sync::{
        atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering},
        Arc,
    },
    time::Duration,
};
use tower::ServiceExt;

struct Temp(std::path::PathBuf);
impl Temp {
    fn new() -> Self {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let path = std::env::temp_dir().join(format!(
            "halo-test-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
}
impl Drop for Temp {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn free_ports() -> (u16, u16) {
    let data = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    let command = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    (
        data.local_addr().unwrap().port(),
        command.local_addr().unwrap().port(),
    )
}
struct Device {
    id: String,
    generation: usize,
    ready: Arc<AtomicBool>,
    timestamp: AtomicU64,
    stopped: Arc<AtomicUsize>,
}
impl SensorDriver for Device {
    fn telemetry_status(&self, data: &Value) -> halo::core::driver::TelemetryStatus {
        halo::core::driver::TelemetryStatus::from_fields(data, "sample_clock", &[], &[], &[])
    }
    fn id(&self) -> &str {
        &self.id
    }
    fn display_name(&self) -> &str {
        "Lifecycle fixture"
    }
    fn data_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("generation", "session generation", "u64"),
            FieldDescriptor::new("sample_clock", "fresh sample", "u64"),
        ]
    }
    fn command_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("ping", "ping", "bool"),
            FieldDescriptor::new("advance", "fixture movement", "f64").requiring_ready(),
            FieldDescriptor::new("release", "fixture mode", "bool")
                .requiring_ready_for(json!(true))
                .standalone(),
        ]
    }
    fn command_mode(&self) -> CommandMode {
        CommandMode::Rest
    }
    fn read_data(&self) -> Result<Value, DriverError> {
        Ok(
            json!({"generation":self.generation,"sample_clock":if self.ready.load(Ordering::SeqCst) {self.timestamp.fetch_add(1,Ordering::SeqCst)+1} else {0}}),
        )
    }
    fn execute_command(&self, _: &Value) -> Result<Value, DriverError> {
        Ok(json!({"generation":self.generation}))
    }
    fn has_estop(&self) -> bool {
        true
    }
    fn estop(&self) -> Result<Value, DriverError> {
        Ok(json!({"stopped": true}))
    }
    fn estop_confirmation(&self) -> Option<String> {
        Some("Stop fixture motion?".into())
    }
    fn shutdown(&self) -> futures::future::BoxFuture<'_, Result<(), DriverError>> {
        Box::pin(async move {
            self.stopped.fetch_add(1, Ordering::SeqCst);
            Ok(())
        })
    }
}

#[test]
fn command_constraints_follow_driver_metadata_for_arbitrary_field_names() {
    let slot = DriverSlot::new(
        "unbranded_fixture".into(),
        Arc::new(Device {
            id: "unbranded_fixture".into(),
            generation: 1,
            ready: Arc::new(AtomicBool::new(false)),
            timestamp: AtomicU64::new(0),
            stopped: Arc::new(AtomicUsize::new(0)),
        }),
    );
    assert!(slot.execute_command(&json!({"ping": true})).is_ok());
    assert!(slot.execute_command(&json!({"release": false})).is_ok());
    assert!(matches!(
        slot.execute_command(&json!({"advance": 1.0})),
        Err(DriverError::Unavailable(_))
    ));
    assert!(matches!(
        slot.execute_command(&json!({"release": true})),
        Err(DriverError::Unavailable(_))
    ));
    slot.mark_sim_ready();
    assert!(slot.execute_command(&json!({"advance": 1.0})).is_ok());
    assert!(slot.execute_command(&json!({"release": true})).is_ok());
    assert!(matches!(
        slot.execute_command(&json!({"release": false, "ping": true})),
        Err(DriverError::InvalidCommand(_))
    ));
    assert!(matches!(
        slot.execute_command(&json!({"release": "true"})),
        Err(DriverError::InvalidCommand(_))
    ));
}
fn factory(
    id: &str,
    ready: Arc<AtomicBool>,
    generations: Arc<AtomicUsize>,
    stopped: Arc<AtomicUsize>,
) -> Factory {
    let id = id.to_owned();
    Arc::new(move || {
        let id = id.clone();
        let ready = ready.clone();
        let stopped = stopped.clone();
        let generation = generations.fetch_add(1, Ordering::SeqCst) + 1;
        Box::pin(async move {
            Ok(vec![(
                id.clone(),
                Arc::new(Device {
                    id,
                    generation,
                    ready,
                    stopped,
                    timestamp: AtomicU64::new(0),
                }) as Arc<dyn SensorDriver>,
            )])
        })
    })
}
fn manager(temp: &Temp, reg: Arc<SensorRegistry>, attempts: u32) -> Arc<Supervisor> {
    Supervisor::new(
        reg,
        Arc::new(LogManager::new(temp.0.join("logs")).unwrap()),
        20,
        RetryPolicy {
            max_attempts: attempts,
            attempt_timeout_ms: 1000,
            retry_delay_ms: 100,
            stale_after_ms: 500,
        },
    )
}
async fn wait_state(manager: &Supervisor, id: &str, wanted: &str) {
    tokio::time::timeout(Duration::from_secs(8), async {
        loop {
            if manager
                .status()
                .iter()
                .any(|s| s.id == id && s.state == wanted)
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .unwrap_or_else(|_| {
        panic!(
            "{id} never reached {wanted}: {}",
            serde_json::to_string(&manager.status()).unwrap()
        )
    });
}

#[test]
fn reservations_survive_missing_devices_reversed_registration_and_invalid_edits() {
    let temp = Temp::new();
    let path = temp.0.join("ports.json");
    let first = SensorRegistry::persistent(5000, Some(path.clone()), [5100].into()).unwrap();
    let arm = first.reserve("kinova_arm").unwrap();
    let gripper = first.reserve("robotiq_gripper").unwrap();
    let drive = first.reserve("odrive_31").unwrap();
    drop(first);
    let next = SensorRegistry::persistent(5000, Some(path), [5100].into()).unwrap();
    assert_eq!(next.reserve("odrive_31").unwrap(), drive);
    assert_eq!(next.reserve("kinova_arm").unwrap(), arm);
    assert_eq!(next.ports("robotiq_gripper"), Some(gripper));
    assert!(next.set_ports("odrive_31", arm.0, 6001).is_err());
    assert!(next.set_ports("odrive_31", 5100, 6001).is_err());
    assert!(next.set_ports("odrive_31", 0, 6001).is_err());
    assert!(next.set_ports("odrive_31", 6000, 6000).is_err());
    assert_eq!(next.ports("odrive_31"), Some(drive));
    assert!(next.reserve_external_ports(&[arm.0]).is_err());
    next.reserve_external_ports(&[5006, 5007]).unwrap();
    assert_eq!(next.reserve("livox_40").unwrap(), (5008, 5009));
    assert!(next.set_ports("livox_40", 5006, 5007).is_err());
}

#[test]
fn commissioned_ports_survive_base_changes_and_hardware_additions_and_removal() {
    let temp = Temp::new();
    let path = temp.0.join("ports.json");
    let first = SensorRegistry::persistent(5000, Some(path.clone()), Default::default()).unwrap();
    first.reserve("sensor_a").unwrap();
    let absent = first.reserve("sensor_b").unwrap();
    first.set_ports("sensor_a", 6200, 6201).unwrap();
    let commissioned = first.assignments().sensors;
    drop(first);

    // A new allocation base and an absent sensor must not change any saved pair.
    let next = SensorRegistry::persistent(7000, Some(path.clone()), Default::default()).unwrap();
    assert_eq!(next.reserve("sensor_a").unwrap(), (6200, 6201));
    assert_eq!(next.ports("sensor_b"), Some(absent));
    let added = next.reserve("sensor_c").unwrap();
    assert_eq!(added, (7000, 7001));
    assert!(next.set_ports("sensor_c", absent.0, absent.1).is_err());
    for (id, pair) in &commissioned {
        assert_eq!(next.ports(id), Some((pair[0], pair[1])));
    }
    drop(next);

    let restored =
        SensorRegistry::persistent(8000, Some(path.clone()), Default::default()).unwrap();
    // Rediscovery order has no effect; the previously absent device returns.
    assert_eq!(restored.reserve("sensor_c").unwrap(), added);
    assert_eq!(restored.reserve("sensor_b").unwrap(), absent);
    assert_eq!(restored.reserve("sensor_a").unwrap(), (6200, 6201));
    drop(restored);

    // An unreadable saved map must fail visibly, never manufacture new bindings.
    std::fs::write(&path, b"corrupt saved assignments").unwrap();
    assert!(SensorRegistry::persistent(5000, Some(path), Default::default()).is_err());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn saved_port_conflict_reports_failure_and_rebinds_the_same_pair_after_retry() {
    let temp = Temp::new();
    let reg = Arc::new(
        SensorRegistry::persistent(5000, Some(temp.0.join("ports.json")), Default::default())
            .unwrap(),
    );
    let busy = std::net::UdpSocket::bind("0.0.0.0:0").unwrap();
    let data_port = busy.local_addr().unwrap().port();
    let command_port = free_ports().1;
    reg.set_ports("sensor", data_port, command_port).unwrap();
    let manager = manager(&temp, reg.clone(), 1);
    manager
        .add(
            "sensor",
            factory(
                "sensor",
                Arc::new(AtomicBool::new(true)),
                Arc::new(AtomicUsize::new(0)),
                Arc::new(AtomicUsize::new(0)),
            ),
        )
        .unwrap();
    wait_state(&manager, "sensor", "failed").await;
    assert_eq!(reg.ports("sensor"), Some((data_port, command_port)));
    assert!(manager.status()[0]
        .error
        .as_ref()
        .unwrap()
        .contains("binding UDP"));
    assert!(reg.list().is_empty());
    drop(busy);
    manager.retry("sensor").unwrap();
    wait_state(&manager, "sensor", "online").await;
    assert_eq!(reg.list()[0].data_port, data_port);
    assert_eq!(reg.list()[0].command_port, command_port);
    let client = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
    client
        .send_to(
            &Packet::new(
                MessageType::Command,
                1,
                serde_json::to_vec(&json!({"ping":true})).unwrap(),
            )
            .encode(),
            ("127.0.0.1", command_port),
        )
        .await
        .unwrap();
    let mut buffer = [0u8; 4096];
    let n = tokio::time::timeout(Duration::from_secs(2), client.recv(&mut buffer))
        .await
        .unwrap()
        .unwrap();
    assert_eq!(
        Packet::decode(&buffer[..n]).unwrap().msg_type,
        MessageType::CommandAck
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn controller_without_hardware_is_not_discoverable_and_failure_budget_is_bounded() {
    let temp = Temp::new();
    let reg = Arc::new(SensorRegistry::new(5000));
    let ports = free_ports();
    reg.set_ports("kinova_arm", ports.0, ports.1).unwrap();
    let manager = manager(&temp, reg.clone(), 2);
    let ready = Arc::new(AtomicBool::new(false));
    let generations = Arc::new(AtomicUsize::new(0));
    let stopped = Arc::new(AtomicUsize::new(0));
    manager
        .add(
            "controller_fixture",
            factory(
                "kinova_arm",
                ready.clone(),
                generations.clone(),
                stopped.clone(),
            ),
        )
        .unwrap();
    tokio::time::sleep(Duration::from_millis(80)).await;
    assert!(reg.list().is_empty());
    assert_eq!(
        reg.get("kinova_arm").unwrap().health().unwrap()["controller_connected"],
        true
    );
    assert_eq!(
        reg.get("kinova_arm").unwrap().health().unwrap()["hardware_ready"],
        false
    );
    wait_state(&manager, "controller_fixture", "failed").await;
    assert_eq!(generations.load(Ordering::SeqCst), 2);
    assert_eq!(stopped.load(Ordering::SeqCst), 2);
    assert!(reg.list().is_empty());
    ready.store(true, Ordering::SeqCst);
    manager.retry("controller_fixture").unwrap();
    wait_state(&manager, "controller_fixture", "online").await;
    assert_eq!(reg.list()[0].id, "kinova_arm");
    assert_eq!(reg.ports("kinova_arm"), Some(ports));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn retry_preserves_udp_subscription_and_other_drivers_and_port_changes_rollback() {
    let temp = Temp::new();
    let path = temp.0.join("ports.json");
    let reg =
        Arc::new(SensorRegistry::persistent(5000, Some(path.clone()), Default::default()).unwrap());
    let ports = free_ports();
    let other = free_ports();
    reg.set_ports("device", ports.0, ports.1).unwrap();
    reg.set_ports("other", other.0, other.1).unwrap();
    let manager = manager(&temp, reg.clone(), 3);
    // A new driver/configuration pair registers through the same public hook.
    #[derive(serde::Deserialize)]
    #[serde(deny_unknown_fields)]
    struct FixtureConfig {
        bus: String,
    }
    impl halo::core::config::DriverConfig for FixtureConfig {
        fn validate(&self) -> Result<(), String> {
            if self.bus.is_empty() {
                Err("bus cannot be empty".into())
            } else {
                Ok(())
            }
        }
    }
    let generations = Arc::new(AtomicUsize::new(0));
    let stopped = Arc::new(AtomicUsize::new(0));
    let ready = Arc::new(AtomicBool::new(true));
    manager
        .add(
            "device_fixture",
            factory(
                "device",
                ready.clone(),
                generations.clone(),
                stopped.clone(),
            ),
        )
        .unwrap();
    halo::drivers::loader::configure::<FixtureConfig>(&manager, "device_fixture", "fixture.toml")
        .unwrap();
    let (_, validate) = manager.configuration("device_fixture").unwrap();
    assert!(validate("bus = 'testbus'").is_ok());
    assert!(validate("bus = ''").is_err());
    assert!(validate("bus = 'testbus'\nunknown = 1").is_err());
    assert_eq!(
        manager.status()[0].configuration_endpoint.as_deref(),
        Some("/admin/drivers/device_fixture/config")
    );
    let other_generations = Arc::new(AtomicUsize::new(0));
    manager
        .add(
            "other_fixture",
            factory(
                "other",
                Arc::new(AtomicBool::new(true)),
                other_generations.clone(),
                Arc::new(AtomicUsize::new(0)),
            ),
        )
        .unwrap();
    wait_state(&manager, "device_fixture", "online").await;
    wait_state(&manager, "other_fixture", "online").await;
    let socket = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
    socket.connect(("127.0.0.1", ports.0)).await.unwrap();
    // Zero used to panic the UDP push task. A rejected request must leave it usable.
    socket
        .send(
            &Packet::new(
                MessageType::Subscribe,
                1,
                serde_json::to_vec(&json!({"interval_ms":0})).unwrap(),
            )
            .encode(),
        )
        .await
        .unwrap();
    let mut buffer = [0u8; 4096];
    let n = socket.recv(&mut buffer).await.unwrap();
    assert_eq!(
        Packet::decode(&buffer[..n]).unwrap().msg_type,
        MessageType::Error
    );
    socket
        .send(
            &Packet::new(
                MessageType::Subscribe,
                2,
                serde_json::to_vec(&json!({"interval_ms":20})).unwrap(),
            )
            .encode(),
        )
        .await
        .unwrap();
    manager.retry("device_fixture").unwrap();
    wait_state(&manager, "device_fixture", "online").await;
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let n = socket.recv(&mut buffer).await.unwrap();
            let p = Packet::decode(&buffer[..n]).unwrap();
            if p.msg_type == MessageType::Data && p.json_payload().unwrap()["generation"] == 2 {
                break;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(stopped.load(Ordering::SeqCst), 1);
    assert_eq!(other_generations.load(Ordering::SeqCst), 1);
    assert_eq!(reg.ports("device"), Some(ports));
    let busy = std::net::UdpSocket::bind("0.0.0.0:0").unwrap();
    let conflict = busy.local_addr().unwrap().port();
    let new = free_ports();
    assert!(manager
        .change_ports("device", new.0, conflict)
        .await
        .is_err());
    assert_eq!(reg.ports("device"), Some(ports));
    let socket = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
    socket
        .send_to(
            &Packet::new(
                MessageType::Command,
                3,
                serde_json::to_vec(&json!({"ping":true})).unwrap(),
            )
            .encode(),
            ("127.0.0.1", ports.1),
        )
        .await
        .unwrap();
    let n = tokio::time::timeout(Duration::from_secs(2), socket.recv(&mut buffer))
        .await
        .unwrap()
        .unwrap();
    assert_eq!(
        Packet::decode(&buffer[..n]).unwrap().msg_type,
        MessageType::CommandAck
    );
    manager
        .change_ports("device", ports.0, new.1)
        .await
        .unwrap();
    assert_eq!(reg.ports("device"), Some((ports.0, new.1)));
    assert!(matches!(
        reg.get("device")
            .unwrap()
            .execute_command(&json!({"ping":"wrong type"})),
        Err(DriverError::InvalidCommand(_))
    ));
    // A physical interruption triggers automatic recovery using the UI-edited
    // ports. Discovery may disappear, but the saved reservation remains.
    ready.store(false, Ordering::SeqCst);
    tokio::time::timeout(Duration::from_secs(3), async {
        while reg.list().iter().any(|sensor| sensor.id == "device") {
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .unwrap();
    assert_eq!(reg.ports("device"), Some((ports.0, new.1)));
    ready.store(true, Ordering::SeqCst);
    wait_state(&manager, "device_fixture", "online").await;
    assert!(generations.load(Ordering::SeqCst) >= 3);
    assert_eq!(other_generations.load(Ordering::SeqCst), 1);
    let saved = SensorRegistry::persistent(9000, Some(path), Default::default()).unwrap();
    assert_eq!(saved.ports("device"), Some((ports.0, new.1)));
    assert_eq!(saved.ports("other"), Some(other));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn discovery_and_openapi_follow_new_devices_and_live_ports_without_rebuilding_router() {
    let temp = Temp::new();
    let reg = Arc::new(SensorRegistry::new(5000));
    let logs = Arc::new(LogManager::new(temp.0.join("logs")).unwrap());
    let app = halo::http::routes::build_router(reg.clone(), Vec::new(), logs);
    let device = Device {
        id: "dynamic_device".into(),
        generation: 1,
        ready: Arc::new(AtomicBool::new(true)),
        timestamp: AtomicU64::new(0),
        stopped: Arc::new(AtomicUsize::new(0)),
    };
    reg.register(device);
    reg.set_ports("dynamic_device", 6010, 6011).unwrap();
    async fn json_get(app: axum::Router, path: &str) -> Value {
        let r = app
            .oneshot(Request::builder().uri(path).body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(r.status(), 200);
        serde_json::from_slice(&to_bytes(r.into_body(), usize::MAX).await.unwrap()).unwrap()
    }
    let discover = json_get(app.clone(), "/discover").await;
    let sensor = &discover["sensors"][0];
    for key in [
        "id",
        "display_name",
        "command_mode",
        "data_port",
        "command_port",
        "endpoints",
    ] {
        assert!(sensor.get(key).is_some(), "missing compatible field {key}");
    }
    assert_eq!(sensor["data_port"], 6010);
    assert_eq!(sensor["endpoints"]["health"], "/dynamic_device/health");
    let info = json_get(app.clone(), "/dynamic_device/info").await;
    assert_eq!(info["command_port"], 6011);
    assert_eq!(info["has_estop"], true);
    assert_eq!(info["estop_confirmation"], "Stop fixture motion?");
    let response = app
        .clone()
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/dynamic_device/estop")
                .extension(axum::extract::ConnectInfo(
                    "127.0.0.1:12345".parse::<std::net::SocketAddr>().unwrap(),
                ))
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), 200);
    assert_eq!(sensor["endpoints"]["estop"], "/dynamic_device/estop");
    assert_eq!(
        info["udp_protocol"]["header_format"],
        "| version (1B) | msg_type (1B) | seq_num (2B LE) | payload (JSON) |"
    );
    let spec = json_get(app, "/openapi.json").await;
    assert!(spec["paths"]["/dynamic_device/data"]["get"].is_object());
    assert!(spec["components"]["schemas"]["DiscoverResponse"].is_object());
}

#[tokio::test]
async fn driver_owned_http_extension_preserves_legacy_upload_and_documentation() {
    let temp = Temp::new();
    let endpoints = halo::drivers::odrive::endpoints::new_shared();
    let app = halo::http::routes::build_router(
        Arc::new(SensorRegistry::new(5000)),
        vec![halo::drivers::odrive::http::extension(endpoints.clone())],
        Arc::new(LogManager::new(temp.0.join("logs")).unwrap()),
    );
    let response = app
        .clone()
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/odrive/endpoints")
                .header("content-type", "application/json")
                .body(Body::from(
                    r#"{"endpoints":{"axis.config.test":{"id":7,"type":"float","access":"rw"}}}"#,
                ))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), 200);
    let value: Value =
        serde_json::from_slice(&to_bytes(response.into_body(), usize::MAX).await.unwrap()).unwrap();
    assert_eq!(value, json!({"loaded":1,"status":"ok"}));
    assert_eq!(endpoints.read().unwrap()["axis.config.test"].id, 7);
    let response = app
        .oneshot(
            Request::builder()
                .uri("/openapi.json")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    let value: Value =
        serde_json::from_slice(&to_bytes(response.into_body(), usize::MAX).await.unwrap()).unwrap();
    assert!(value["paths"]["/odrive/endpoints"]["post"].is_object());
}
