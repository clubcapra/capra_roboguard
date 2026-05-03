use std::net::SocketAddr;
use std::sync::Arc;

use axum::extract::{ConnectInfo, Path, State};
use axum::http::{HeaderMap, StatusCode};
use axum::response::Html;
use axum::routing::{get, post};
use axum::{Json, Router};
use serde_json::Value;

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::registry::{SensorInfo, SensorRegistry};
use crate::drivers::odrive::endpoints::{load_from_str, SharedEndpointMap};
use crate::logging::LogManager;
use crate::protocol::packet;

// ── Response types ──────────────────────────────────────────────────────────

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct DiscoverResponse {
    pub sensors: Vec<SensorSummary>,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct SensorSummary {
    pub id: String,
    pub display_name: String,
    pub command_mode: CommandMode,
    pub data_port: u16,
    pub command_port: u16,
    /// HTTP paths for this sensor.
    pub endpoints: SensorEndpoints,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct SensorEndpoints {
    pub info: String,
    pub data: String,
    pub command: String,
    pub commands: String,
    pub endpoints: String,
    pub endpoint: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub estop: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calibrate: Option<String>,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct SensorInfoResponse {
    pub id: String,
    pub display_name: String,
    pub command_mode: CommandMode,
    pub data_port: u16,
    pub command_port: u16,
    pub data_schema: Vec<FieldDescriptor>,
    pub command_schema: Vec<FieldDescriptor>,
    pub udp_protocol: UdpProtocolInfo,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct UdpProtocolInfo {
    pub header_format: String,
    pub data_subscription: DataSubscriptionInfo,
    pub command_protocol: CommandProtocolInfo,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct DataSubscriptionInfo {
    pub description: String,
    pub flow: String,
    pub subscribe_packet: PacketExample,
    pub unsubscribe_packet: PacketExample,
    pub data_push_packet: PacketExample,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct CommandProtocolInfo {
    pub description: String,
    pub flow: String,
    pub packets: Vec<PacketExample>,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct PacketExample {
    pub name: String,
    pub description: String,
    pub header_hex: String,
    pub payload_example: Option<Value>,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct CommandResult {
    pub status: String,
    pub result: Value,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct ErrorResponse {
    pub error: String,
}

// ── Shared handler state for per-sensor routes ──────────────────────────────

#[derive(Clone)]
struct SensorState {
    driver: Arc<dyn SensorDriver>,
    info: Arc<SensorInfo>,
    log_mgr: Arc<LogManager>,
}

// ── Protocol info builder ───────────────────────────────────────────────────

fn build_protocol_info(info: &SensorInfo) -> UdpProtocolInfo {
    let example_payload = build_example_payload(&info.command_schema);

    let mut cmd_packets = vec![PacketExample {
        name: "Command".to_string(),
        description: "Send a one-shot command".to_string(),
        header_hex: format!(
            "{:02X} {:02X} 01 00 + JSON",
            packet::PROTOCOL_VERSION,
            packet::MessageType::Command as u8
        ),
        payload_example: Some(example_payload.clone()),
    }];

    let cmd_flow = match &info.command_mode {
        CommandMode::Rest => "Client --Command(0x10)--> Robot --CommandAck(0x11)--> Client".to_string(),
        CommandMode::Stream { interval_ms } => {
            cmd_packets[0].description = format!(
                "Send command packet (repeat every ~{}ms; each packet processed on arrival)",
                interval_ms
            );
            format!(
                "Client --Command(0x10) every ~{}ms--> Robot --CommandAck(0x11)--> Client",
                interval_ms
            )
        }
    };

    UdpProtocolInfo {
        header_format: "| version (1B) | msg_type (1B) | seq_num (2B LE) | payload (JSON) |"
            .to_string(),
        data_subscription: DataSubscriptionInfo {
            description: format!(
                "Send Subscribe to UDP port {} and the robot will push sensor data to your address.",
                info.data_port
            ),
            flow: "Client --Subscribe(0x01)--> Robot --SubscribeAck(0x04)--> Client, then Robot --Data(0x03)--> Client (continuously)".to_string(),
            subscribe_packet: PacketExample {
                name: "Subscribe".to_string(),
                description: "Subscribe to data pushes. Optional payload: {\"interval_ms\": 100}".to_string(),
                header_hex: format!(
                    "{:02X} {:02X} 01 00",
                    packet::PROTOCOL_VERSION,
                    packet::MessageType::Subscribe as u8
                ),
                payload_example: Some(serde_json::json!({"interval_ms": 100})),
            },
            unsubscribe_packet: PacketExample {
                name: "Unsubscribe".to_string(),
                description: "Stop receiving data pushes".to_string(),
                header_hex: format!(
                    "{:02X} {:02X} 01 00",
                    packet::PROTOCOL_VERSION,
                    packet::MessageType::Unsubscribe as u8
                ),
                payload_example: None,
            },
            data_push_packet: PacketExample {
                name: "Data".to_string(),
                description: "Pushed by robot to subscriber".to_string(),
                header_hex: format!(
                    "{:02X} {:02X} XX XX + JSON",
                    packet::PROTOCOL_VERSION,
                    packet::MessageType::Data as u8
                ),
                payload_example: Some(build_example_payload(&info.data_schema)),
            },
        },
        command_protocol: CommandProtocolInfo {
            description: format!(
                "Send commands to UDP port {}. Mode: {:?}.",
                info.command_port, info.command_mode
            ),
            flow: cmd_flow,
            packets: cmd_packets,
        },
    }
}

fn build_example_payload(schema: &[FieldDescriptor]) -> Value {
    let mut map = serde_json::Map::new();
    for field in schema {
        let example = match field.type_name.as_str() {
            "f64" | "f32" => Value::from(0.0_f64),
            "u8" | "u16" | "u32" | "u64" | "i8" | "i16" | "i32" | "i64" => Value::from(0),
            "bool" => Value::from(false),
            "String" | "str" => Value::from(""),
            _ => Value::Null,
        };
        map.insert(field.name.clone(), example);
    }
    Value::Object(map)
}

// ── Handlers ────────────────────────────────────────────────────────────────

async fn discover(State(reg): State<Arc<SensorRegistry>>) -> Json<DiscoverResponse> {
    let sensors = reg
        .list()
        .into_iter()
        .map(|s| {
            let endpoints = SensorEndpoints {
                info: format!("/{}/info", s.id),
                data: format!("/{}/data", s.id),
                command: format!("/{}/command", s.id),
                commands: format!("/{}/commands", s.id),
                endpoints: format!("/{}/endpoints", s.id),
                endpoint: format!("/{}/endpoint/{{path}}", s.id),
                estop: s.has_estop.then(|| format!("/{}/estop", s.id)),
                config: s.has_config.then(|| format!("/{}/config", s.id)),
                calibrate: s.has_calibrate.then(|| format!("/{}/calibrate", s.id)),
            };
            SensorSummary {
                id: s.id,
                display_name: s.display_name,
                command_mode: s.command_mode,
                data_port: s.data_port,
                command_port: s.command_port,
                endpoints,
            }
        })
        .collect();
    Json(DiscoverResponse { sensors })
}

async fn sensor_info(State(state): State<SensorState>) -> Json<SensorInfoResponse> {
    let info = &state.info;
    let protocol = build_protocol_info(info);
    Json(SensorInfoResponse {
        id: info.id.clone(),
        display_name: info.display_name.clone(),
        command_mode: info.command_mode.clone(),
        data_port: info.data_port,
        command_port: info.command_port,
        data_schema: info.data_schema.clone(),
        command_schema: info.command_schema.clone(),
        udp_protocol: protocol,
    })
}

async fn sensor_data(
    State(state): State<SensorState>,
) -> Result<Json<Value>, (StatusCode, Json<ErrorResponse>)> {
    let data = state.driver.read_data().map_err(|e| {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(ErrorResponse {
                error: e.to_string(),
            }),
        )
    })?;
    Ok(Json(data))
}

async fn sensor_command(
    State(state): State<SensorState>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(payload): Json<Value>,
) -> Result<Json<CommandResult>, (StatusCode, Json<ErrorResponse>)> {
    let outcome = state.driver.execute_command(&payload);
    log_outcome(&state, addr, &headers, "command", &payload, &outcome);
    let result = outcome.map_err(internal_err)?;
    Ok(Json(CommandResult {
        status: "ok".to_string(),
        result,
    }))
}

async fn sensor_estop(
    State(state): State<SensorState>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
) -> Result<Json<CommandResult>, (StatusCode, Json<ErrorResponse>)> {
    let outcome = state.driver.estop();
    log_outcome(&state, addr, &headers, "estop", &Value::Null, &outcome);
    let result = outcome.map_err(internal_err)?;
    Ok(Json(CommandResult {
        status: "ok".to_string(),
        result,
    }))
}

fn internal_err(e: crate::core::error::DriverError) -> (StatusCode, Json<ErrorResponse>) {
    (
        StatusCode::INTERNAL_SERVER_ERROR,
        Json(ErrorResponse { error: e.to_string() }),
    )
}

fn user_agent(headers: &HeaderMap) -> &str {
    headers
        .get(axum::http::header::USER_AGENT)
        .and_then(|v| v.to_str().ok())
        .unwrap_or("")
}

fn log_outcome(
    state: &SensorState,
    addr: SocketAddr,
    headers: &HeaderMap,
    kind: &str,
    payload: &Value,
    outcome: &Result<Value, crate::core::error::DriverError>,
) {
    let client = addr.to_string();
    let ua = user_agent(headers);
    match outcome {
        Ok(v) => state
            .log_mgr
            .log_input(&state.info.id, "http", &client, ua, kind, payload, Ok(v)),
        Err(e) => state.log_mgr.log_input(
            &state.info.id,
            "http",
            &client,
            ua,
            kind,
            payload,
            Err(&e.to_string()),
        ),
    }
}

async fn upload_endpoints(
    State(ep_map): State<SharedEndpointMap>,
    body: axum::body::Bytes,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<ErrorResponse>)> {
    let content = std::str::from_utf8(&body).map_err(|e| {
        (
            StatusCode::BAD_REQUEST,
            Json(ErrorResponse { error: format!("invalid UTF-8: {e}") }),
        )
    })?;
    let count = load_from_str(&ep_map, content).map_err(|e| {
        (
            StatusCode::BAD_REQUEST,
            Json(ErrorResponse { error: e.to_string() }),
        )
    })?;
    Ok(Json(serde_json::json!({ "loaded": count, "status": "ok" })))
}

async fn sensor_read_config(
    State(state): State<SensorState>,
) -> Result<Json<Value>, (StatusCode, Json<ErrorResponse>)> {
    let result = state.driver.read_config().map_err(|e| {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(ErrorResponse {
                error: e.to_string(),
            }),
        )
    })?;
    Ok(Json(result))
}

async fn sensor_write_config(
    State(state): State<SensorState>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(payload): Json<Value>,
) -> Result<Json<CommandResult>, (StatusCode, Json<ErrorResponse>)> {
    let outcome = state.driver.write_config(&payload);
    log_outcome(&state, addr, &headers, "write_config", &payload, &outcome);
    let result = outcome.map_err(internal_err)?;
    Ok(Json(CommandResult {
        status: "ok".to_string(),
        result,
    }))
}

async fn sensor_calibrate(
    State(state): State<SensorState>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(payload): Json<Value>,
) -> Result<Json<CommandResult>, (StatusCode, Json<ErrorResponse>)> {
    let outcome = state.driver.calibrate(&payload);
    log_outcome(&state, addr, &headers, "calibrate", &payload, &outcome);
    let result = outcome.map_err(internal_err)?;
    Ok(Json(CommandResult {
        status: "ok".to_string(),
        result,
    }))
}

async fn sensor_list_commands(
    State(state): State<SensorState>,
) -> Result<Json<Value>, (StatusCode, Json<ErrorResponse>)> {
    let result = state.driver.list_commands().map_err(|e| {
        (StatusCode::INTERNAL_SERVER_ERROR, Json(ErrorResponse { error: e.to_string() }))
    })?;
    Ok(Json(result))
}

async fn sensor_list_endpoints(
    State(state): State<SensorState>,
) -> Result<Json<Value>, (StatusCode, Json<ErrorResponse>)> {
    let result = state.driver.list_endpoints().map_err(|e| {
        (StatusCode::INTERNAL_SERVER_ERROR, Json(ErrorResponse { error: e.to_string() }))
    })?;
    Ok(Json(result))
}

async fn sensor_read_endpoint(
    State(state): State<SensorState>,
    Path(path): Path<String>,
) -> Result<Json<Value>, (StatusCode, Json<ErrorResponse>)> {
    let result = state.driver.read_endpoint(&path).map_err(|e| {
        (StatusCode::INTERNAL_SERVER_ERROR, Json(ErrorResponse { error: e.to_string() }))
    })?;
    Ok(Json(result))
}

async fn sensor_write_endpoint(
    State(state): State<SensorState>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Path(path): Path<String>,
    Json(payload): Json<Value>,
) -> Result<Json<Value>, (StatusCode, Json<ErrorResponse>)> {
    let outcome = state.driver.write_endpoint(&path, &payload);
    let logged_payload = serde_json::json!({"path": path, "body": payload});
    log_outcome(&state, addr, &headers, "write_endpoint", &logged_payload, &outcome);
    let result = outcome.map_err(internal_err)?;
    Ok(Json(result))
}

// ── Log file routes ─────────────────────────────────────────────────────────

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct LogIndex {
    pub root: String,
    pub days: Vec<LogDay>,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct LogDay {
    pub date: String,
    pub hours: Vec<LogHour>,
}

#[derive(serde::Serialize, utoipa::ToSchema)]
pub struct LogHour {
    pub hour: String,
    pub sensors: Vec<String>,
    pub inputs: Option<String>,
}

async fn logs_index(
    State(log_mgr): State<Arc<LogManager>>,
) -> Result<Json<LogIndex>, (StatusCode, Json<ErrorResponse>)> {
    let root = log_mgr.log_dir().to_path_buf();
    let mut days: Vec<LogDay> = Vec::new();

    let day_entries = std::fs::read_dir(&root).map_err(|e| {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(ErrorResponse { error: format!("read log dir: {e}") }),
        )
    })?;

    let mut day_names: Vec<String> = day_entries
        .filter_map(Result::ok)
        .filter(|e| e.file_type().map(|t| t.is_dir()).unwrap_or(false))
        .filter_map(|e| e.file_name().into_string().ok())
        .collect();
    day_names.sort();

    for date in day_names {
        let day_dir = root.join(&date);
        let mut hour_names: Vec<String> = std::fs::read_dir(&day_dir)
            .into_iter()
            .flatten()
            .filter_map(Result::ok)
            .filter(|e| e.file_type().map(|t| t.is_dir()).unwrap_or(false))
            .filter_map(|e| e.file_name().into_string().ok())
            .collect();
        hour_names.sort();

        let mut hours: Vec<LogHour> = Vec::new();
        for hour in hour_names {
            let hour_dir = day_dir.join(&hour);
            let mut sensors: Vec<String> = Vec::new();
            let mut inputs: Option<String> = None;
            for entry in std::fs::read_dir(&hour_dir).into_iter().flatten().flatten() {
                let name = match entry.file_name().into_string() {
                    Ok(s) => s,
                    Err(_) => continue,
                };
                let rel = format!("{date}/{hour}/{name}");
                if name == "inputs.csv" {
                    inputs = Some(rel);
                } else if name.ends_with(".csv") {
                    let id = name.trim_end_matches(".csv").to_string();
                    sensors.push(id);
                }
            }
            sensors.sort();
            hours.push(LogHour { hour, sensors, inputs });
        }
        days.push(LogDay { date, hours });
    }

    Ok(Json(LogIndex {
        root: root.display().to_string(),
        days,
    }))
}

async fn logs_file(
    State(log_mgr): State<Arc<LogManager>>,
    Path(path): Path<String>,
) -> Result<axum::response::Response, (StatusCode, Json<ErrorResponse>)> {
    use axum::body::Body;
    use axum::http::header;
    use axum::response::IntoResponse;

    // Resolve and validate: the requested path must canonicalize to something
    // inside the configured log directory. Refuses absolute paths, traversal,
    // and symlinks pointing outside the tree.
    if path.contains("..") || path.starts_with('/') {
        return Err((
            StatusCode::BAD_REQUEST,
            Json(ErrorResponse { error: "invalid path".into() }),
        ));
    }
    let candidate = log_mgr.log_dir().join(&path);
    let root = log_mgr.log_dir().canonicalize().map_err(|e| {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(ErrorResponse { error: format!("log root: {e}") }),
        )
    })?;
    let resolved = candidate.canonicalize().map_err(|_| {
        (
            StatusCode::NOT_FOUND,
            Json(ErrorResponse { error: format!("not found: {path}") }),
        )
    })?;
    if !resolved.starts_with(&root) {
        return Err((
            StatusCode::BAD_REQUEST,
            Json(ErrorResponse { error: "path escapes log directory".into() }),
        ));
    }

    let bytes = std::fs::read(&resolved).map_err(|e| {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(ErrorResponse { error: e.to_string() }),
        )
    })?;
    let filename = resolved
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("log.csv")
        .to_string();
    let disposition = format!("attachment; filename=\"{filename}\"");
    let mut response = (Body::from(bytes)).into_response();
    response
        .headers_mut()
        .insert(header::CONTENT_TYPE, "text/csv".parse().unwrap());
    if let Ok(v) = disposition.parse() {
        response
            .headers_mut()
            .insert(header::CONTENT_DISPOSITION, v);
    }
    Ok(response)
}

// ── Scalar UI ───────────────────────────────────────────────────────────────

async fn serve_scalar() -> Html<String> {
    Html(
        r#"<!DOCTYPE html>
<html>
<head>
    <title>Capra Rove - Sensor Interface</title>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
</head>
<body>
    <script id="api-reference" data-url="/openapi.json"></script>
    <script src="https://cdn.jsdelivr.net/npm/@scalar/api-reference"></script>
</body>
</html>"#
            .to_string(),
    )
}

// ── Router builder ──────────────────────────────────────────────────────────

pub fn build_router(
    registry: Arc<SensorRegistry>,
    endpoint_map: SharedEndpointMap,
    log_mgr: Arc<LogManager>,
) -> Router {
    let openapi = build_openapi(&registry);
    let openapi = Arc::new(openapi);

    let mut app = Router::new()
        .route("/discover", get(discover))
        .with_state(registry.clone())
        .route("/odrive/endpoints", post(upload_endpoints))
        .with_state(endpoint_map);

    // /logs/* routes — share the log manager.
    let logs_router: Router = Router::new()
        .route("/logs", get(logs_index))
        .route("/logs/file/{*path}", get(logs_file))
        .with_state(log_mgr.clone());
    app = app.merge(logs_router);

    // Generate per-sensor routes: /{sensor_id}/info, /{sensor_id}/data, /{sensor_id}/command
    for sensor in registry.list() {
        let driver = registry.get(&sensor.id).unwrap();
        let state = SensorState {
            driver,
            info: Arc::new(sensor.clone()),
            log_mgr: log_mgr.clone(),
        };

        let mut sensor_router = Router::new()
            .route("/info", get(sensor_info))
            .route("/data", get(sensor_data))
            .route("/command", post(sensor_command))
            .route("/commands", get(sensor_list_commands))
            .route("/endpoints", get(sensor_list_endpoints));

        // Endpoint reads are uniform across drivers (default impl extracts
        // from `read_data` for non-ODrive drivers). Writes need an opt-in.
        sensor_router = if sensor.has_endpoint_write {
            sensor_router.route(
                "/endpoint/{*path}",
                get(sensor_read_endpoint).post(sensor_write_endpoint),
            )
        } else {
            sensor_router.route("/endpoint/{*path}", get(sensor_read_endpoint))
        };

        if sensor.has_estop {
            sensor_router = sensor_router.route("/estop", post(sensor_estop));
        }
        if sensor.has_config {
            sensor_router = sensor_router
                .route("/config", get(sensor_read_config))
                .route("/config", post(sensor_write_config));
        }
        if sensor.has_calibrate {
            sensor_router = sensor_router.route("/calibrate", post(sensor_calibrate));
        }

        let sensor_router = sensor_router.with_state(state);

        app = app.nest(&format!("/{}", sensor.id), sensor_router);
    }

    // Scalar UI + OpenAPI spec
    app = app
        .route("/docs", get(serve_scalar))
        .route(
            "/openapi.json",
            get({
                let spec = openapi.clone();
                move || {
                    let spec = spec.clone();
                    async move { Json(spec.as_ref().clone()) }
                }
            }),
        );

    app
}

// ── OpenAPI spec builder ────────────────────────────────────────────────────

fn build_openapi(registry: &SensorRegistry) -> utoipa::openapi::OpenApi {
    use utoipa::openapi::path::{OperationBuilder, PathItemBuilder};
    use utoipa::openapi::request_body::RequestBodyBuilder;
    use utoipa::openapi::response::ResponseBuilder;
    use utoipa::openapi::{ContentBuilder, HttpMethod, PathsBuilder, RefOr};
    use utoipa::OpenApi;

    #[derive(OpenApi)]
    #[openapi(
        components(schemas(
            DiscoverResponse,
            SensorSummary,
            SensorEndpoints,
            SensorInfoResponse,
            UdpProtocolInfo,
            DataSubscriptionInfo,
            CommandProtocolInfo,
            PacketExample,
            CommandResult,
            ErrorResponse,
            FieldDescriptor,
            CommandMode,
            LogIndex,
            LogDay,
            LogHour,
        )),
        info(
            title = "Capra Rove Sensor Interface",
            description = "Robot sensor API with UDP transport and HTTP documentation.\n\n## Discovery\n\n`GET /discover` lists all sensors with their endpoints and UDP ports.\n\n## Per-Sensor Endpoints\n\nEach sensor has its own routes:\n- `GET /{sensor_id}/info` - schema, commands, UDP packet format\n- `GET /{sensor_id}/data` - current data snapshot\n- `POST /{sensor_id}/command` - send a command\n- `POST /{sensor_id}/estop` - emergency stop (supported drivers only)\n\n## UDP Protocol\n\nPackets: `| version (1B) | msg_type (1B) | seq_num (2B LE) | JSON payload |`\n\n### Data Subscription\nSend **Subscribe (0x01)** to the sensor's data port. The robot pushes **Data (0x03)** packets to your address continuously. Send **Unsubscribe (0x02)** to stop.\n\n### Commands\n- **REST sensors**: Send **Command (0x10)**, get **CommandAck (0x11)**.\n- **Stream sensors** (CAN watchdog): Send **StreamStart (0x12)** once, robot re-sends to hardware at interval. **StreamStop (0x13)** to cancel.",
            version = "0.1.0"
        ),
        tags(
            (name = "discovery", description = "Discover available sensors"),
        )
    )]
    struct ApiDoc;

    let mut doc = ApiDoc::openapi();

    // Add /odrive/endpoints upload path
    let upload_ep_op = OperationBuilder::new()
        .tag("odrive")
        .summary(Some("Upload flat_endpoints.json"))
        .description(Some(
            "Upload the ODrive `flat_endpoints.json` file to enable config read/write on all nodes.\n\n\
             **How to get the file** (on your dev machine where odrivetool is installed):\n\
             ```\npython3 -c \"import odrive, os; print(os.path.dirname(odrive.__file__))\"\n```\
             Then find `flat_endpoints.json` in that directory.\n\n\
             **Upload via curl:**\n\
             ```\ncurl -X POST http://raspberrypi.local:8080/odrive/endpoints \\\n  \
             -H 'Content-Type: application/json' \\\n  \
             --data-binary @flat_endpoints.json\n```\n\n\
             Or paste the file contents directly in the request body below.",
        ))
        .request_body(Some(
            RequestBodyBuilder::new()
                .content(
                    "application/json",
                    ContentBuilder::new()
                        .example(Some(serde_json::json!({
                            "fw_version": "0.6.11",
                            "hw_version": "1.0.0",
                            "endpoints": {
                                "axis0.motor.config.phase_resistance": {"id": 123, "type": "float"},
                                "axis0.controller.config.vel_limit": {"id": 456, "type": "float"}
                            }
                        })))
                        .build(),
                )
                .required(Some(utoipa::openapi::Required::True))
                .build(),
        ))
        .response(
            "200",
            ResponseBuilder::new()
                .description("Endpoints loaded")
                .content(
                    "application/json",
                    ContentBuilder::new()
                        .example(Some(serde_json::json!({"loaded": 1234, "status": "ok"})))
                        .build(),
                )
                .build(),
        )
        .build();

    let mut paths = PathsBuilder::new()
        .path(
            "/odrive/endpoints",
            PathItemBuilder::new()
                .operation(HttpMethod::Post, upload_ep_op)
                .build(),
        );

    doc.tags.get_or_insert_with(Vec::new).push(
        utoipa::openapi::tag::TagBuilder::new()
            .name("odrive")
            .description(Some("ODrive global operations (endpoint map upload)"))
            .build(),
    );

    // Add /discover path manually
    let discover_op = OperationBuilder::new()
        .tag("discovery")
        .summary(Some("List all available sensors"))
        .description(Some(
            "Returns every registered sensor with its ID, name, UDP ports, command mode, and HTTP endpoint paths.",
        ))
        .response(
            "200",
            ResponseBuilder::new()
                .description("List of sensors")
                .content(
                    "application/json",
                    ContentBuilder::new()
                        .schema(Some(RefOr::Ref(utoipa::openapi::Ref::from_schema_name(
                            "DiscoverResponse",
                        ))))
                        .build(),
                )
                .build(),
        )
        .build();

    paths = paths.path(
        "/discover",
        PathItemBuilder::new()
            .operation(HttpMethod::Get, discover_op)
            .build(),
    );

    // /logs paths — index + per-file download
    doc.tags.get_or_insert_with(Vec::new).push(
        utoipa::openapi::tag::TagBuilder::new()
            .name("logs")
            .description(Some(
                "Time-series CSV logs. Sensor snapshots and commands are written to \
                 LOG_DIR/<YYYY-MM-DD>/<HH>/<sensor>.csv and inputs.csv, split hourly.",
            ))
            .build(),
    );

    let logs_index_op = OperationBuilder::new()
        .tag("logs")
        .summary(Some("List available log files"))
        .description(Some(
            "Returns the directory tree under `LOG_DIR` as `{date → {hour → {sensors[], inputs}}}`. \
             Use `GET /logs/file/{path}` with the relative path to download a specific CSV.",
        ))
        .response(
            "200",
            ResponseBuilder::new()
                .description("Available log files")
                .content(
                    "application/json",
                    ContentBuilder::new()
                        .schema(Some(RefOr::Ref(utoipa::openapi::Ref::from_schema_name(
                            "LogIndex",
                        ))))
                        .build(),
                )
                .build(),
        )
        .build();

    let logs_file_op = OperationBuilder::new()
        .tag("logs")
        .summary(Some("Download a log file"))
        .description(Some(
            "Download a CSV file from the log tree. `path` is a forward-slash-separated \
             path relative to `LOG_DIR`, e.g. `2026-05-02/14/kinova_arm.csv` or \
             `2026-05-02/14/inputs.csv`. Path traversal (`..`) is rejected.",
        ))
        .parameter(
            utoipa::openapi::path::ParameterBuilder::new()
                .name("path")
                .parameter_in(utoipa::openapi::path::ParameterIn::Path)
                .required(utoipa::openapi::Required::True)
                .description(Some(
                    "Relative path under LOG_DIR, e.g. 2026-05-02/14/kinova_arm.csv",
                ))
                .build(),
        )
        .response(
            "200",
            ResponseBuilder::new()
                .description("CSV file contents")
                .content("text/csv", ContentBuilder::new().build())
                .build(),
        )
        .build();

    paths = paths
        .path(
            "/logs",
            PathItemBuilder::new()
                .operation(HttpMethod::Get, logs_index_op)
                .build(),
        )
        .path(
            "/logs/file/{path}",
            PathItemBuilder::new()
                .operation(HttpMethod::Get, logs_file_op)
                .build(),
        );

    // Generate per-sensor paths
    for sensor in registry.list() {
        let tag = &sensor.id;
        let mode_desc = match &sensor.command_mode {
            CommandMode::Rest => "REST (one-shot)".to_string(),
            CommandMode::Stream { interval_ms } => {
                format!("Stream ({}ms watchdog)", interval_ms)
            }
        };

        // /{id}/info
        let info_op = OperationBuilder::new()
            .tag(tag)
            .summary(Some(format!("{} - Info", sensor.display_name)))
            .description(Some(format!(
                "Full schema and UDP protocol details for **{}**.\n\nMode: {}\nData UDP port: {}\nCommand UDP port: {}",
                sensor.display_name, mode_desc, sensor.data_port, sensor.command_port
            )))
            .response(
                "200",
                ResponseBuilder::new()
                    .description("Sensor info with schemas and UDP protocol")
                    .content(
                        "application/json",
                        ContentBuilder::new()
                            .schema(Some(RefOr::Ref(utoipa::openapi::Ref::from_schema_name(
                                "SensorInfoResponse",
                            ))))
                            .build(),
                    )
                    .build(),
            )
            .build();

        // /{id}/data
        let data_op = OperationBuilder::new()
            .tag(tag)
            .summary(Some(format!("{} - Read Data", sensor.display_name)))
            .description(Some(format!(
                "Read current data from **{}**.\n\nThis mirrors what you'd get via UDP subscription on port {}.",
                sensor.display_name, sensor.data_port
            )))
            .response(
                "200",
                ResponseBuilder::new()
                    .description("Current sensor data")
                    .content(
                        "application/json",
                        ContentBuilder::new()
                            .schema(Some(build_data_schema(&sensor.data_schema)))
                            .build(),
                    )
                    .build(),
            )
            .build();

        // /{id}/command
        let cmd_op = OperationBuilder::new()
            .tag(tag)
            .summary(Some(format!("{} - Send Command", sensor.display_name)))
            .description(Some(format!(
                "Send a command to **{}**.\n\nMode: {}\nUDP command port: {}\n\nThe JSON body is the same payload used in UDP Command (0x10) packets.",
                sensor.display_name, mode_desc, sensor.command_port
            )))
            .request_body(Some(
                RequestBodyBuilder::new()
                    .content(
                        "application/json",
                        ContentBuilder::new()
                            .schema(Some(build_command_schema(&sensor.command_schema)))
                            .example(Some(build_example_payload(&sensor.command_schema)))
                            .build(),
                    )
                    .required(Some(utoipa::openapi::Required::True))
                    .build(),
            ))
            .response(
                "200",
                ResponseBuilder::new()
                    .description("Command result")
                    .content(
                        "application/json",
                        ContentBuilder::new()
                            .schema(Some(RefOr::Ref(utoipa::openapi::Ref::from_schema_name(
                                "CommandResult",
                            ))))
                            .build(),
                    )
                    .build(),
            )
            .build();

        let base = format!("/{}", sensor.id);
        paths = paths
            .path(
                format!("{}/info", base),
                PathItemBuilder::new()
                    .operation(HttpMethod::Get, info_op)
                    .build(),
            )
            .path(
                format!("{}/data", base),
                PathItemBuilder::new()
                    .operation(HttpMethod::Get, data_op)
                    .build(),
            )
            .path(
                format!("{}/command", base),
                PathItemBuilder::new()
                    .operation(HttpMethod::Post, cmd_op)
                    .build(),
            );

        if sensor.has_estop {
            let estop_op = OperationBuilder::new()
                .tag(tag)
                .summary(Some(format!("{} - Emergency Stop", sensor.display_name)))
                .description(Some(format!(
                    "Send an **immediate emergency stop** to **{}**.\n\nDisarms the motor with `ESTOP_REQUESTED`. No payload required.",
                    sensor.display_name
                )))
                .response(
                    "200",
                    ResponseBuilder::new()
                        .description("E-stop acknowledged")
                        .content(
                            "application/json",
                            ContentBuilder::new()
                                .schema(Some(RefOr::Ref(utoipa::openapi::Ref::from_schema_name(
                                    "CommandResult",
                                ))))
                                .build(),
                        )
                        .build(),
                )
                .build();

            paths = paths.path(
                format!("{}/estop", base),
                PathItemBuilder::new()
                    .operation(HttpMethod::Post, estop_op)
                    .build(),
            );
        }

        if sensor.has_config {
            let config_read_op = OperationBuilder::new()
                .tag(tag)
                .summary(Some(format!("{} - Read Config", sensor.display_name)))
                .description(Some(format!(
                    "Read calibration/configuration parameters from **{}** via CAN SDO.\n\nReturns float and integer fields read from the drive firmware using `flat_endpoints.json`. Requires `ODRIVE_ENDPOINTS` env var at startup.",
                    sensor.display_name
                )))
                .response(
                    "200",
                    ResponseBuilder::new()
                        .description("Config parameters")
                        .content("application/json", ContentBuilder::new().build())
                        .build(),
                )
                .build();

            let config_write_op = OperationBuilder::new()
                .tag(tag)
                .summary(Some(format!("{} - Write Config", sensor.display_name)))
                .description(Some(format!(
                    "Write configuration parameters to **{}** via CAN SDO.\n\n**Supported keys** (all optional):\n- `phase_resistance` (float, Ω)\n- `phase_inductance` (float, H)\n- `current_lim` (float, A)\n- `vel_limit` (float, rev/s)\n- `pos_gain` (float)\n- `vel_gain` (float)\n- `vel_integrator_gain` (float)\n- `pole_pairs` (int)\n- `cpr` (int, counts per revolution)",
                    sensor.display_name
                )))
                .request_body(Some(
                    RequestBodyBuilder::new()
                        .content(
                            "application/json",
                            ContentBuilder::new()
                                .example(Some(serde_json::json!({
                                    "vel_limit": 20.0,
                                    "current_lim": 40.0,
                                    "vel_gain": 0.16,
                                    "vel_integrator_gain": 0.32,
                                })))
                                .build(),
                        )
                        .required(Some(utoipa::openapi::Required::True))
                        .build(),
                ))
                .response(
                    "200",
                    ResponseBuilder::new()
                        .description("Written keys and any errors")
                        .content(
                            "application/json",
                            ContentBuilder::new()
                                .schema(Some(RefOr::Ref(utoipa::openapi::Ref::from_schema_name(
                                    "CommandResult",
                                ))))
                                .build(),
                        )
                        .build(),
                )
                .build();

            paths = paths
                .path(
                    format!("{}/config", base),
                    PathItemBuilder::new()
                        .operation(HttpMethod::Get, config_read_op)
                        .operation(HttpMethod::Post, config_write_op)
                        .build(),
                );
        }

        // /commands — uniform across all drivers
        let commands_op = OperationBuilder::new()
            .tag(tag)
            .summary(Some(format!("{} - List Commands", sensor.display_name)))
            .description(Some(format!(
                "List the commands accepted by **{}** as a `name → {{type, description, unit}}` map. \
                 Same fields as `command_schema` in `/info`, presented as a lookup table.",
                sensor.display_name
            )))
            .response("200", ResponseBuilder::new()
                .description("Map of command name → metadata")
                .content("application/json", ContentBuilder::new().build())
                .build())
            .build();

        // /endpoints — uniform across all drivers
        let list_op = OperationBuilder::new()
            .tag(tag)
            .summary(Some(format!("{} - List Endpoints", sensor.display_name)))
            .description(Some(format!(
                "List the readable endpoints exposed by **{}**. \
                 For most drivers this is the data-schema field set; for ODrive it is the loaded \
                 `flat_endpoints.json` map. Use `GET /{}/endpoint/{{path}}` to read a single value.",
                sensor.display_name, sensor.id
            )))
            .response("200", ResponseBuilder::new()
                .description("Map of path → endpoint metadata")
                .content("application/json", ContentBuilder::new().build())
                .build())
            .build();

        let read_ep_op = OperationBuilder::new()
            .tag(tag)
            .summary(Some(format!("{} - Read Endpoint", sensor.display_name)))
            .description(Some(format!(
                "Read a single endpoint from **{}** by path. Returns `{{path, value, type}}`.\n\n\
                 Paths come from `GET /{}/endpoints`.",
                sensor.display_name, sensor.id
            )))
            .response("200", ResponseBuilder::new()
                .description("Endpoint value")
                .content("application/json", ContentBuilder::new().build())
                .build())
            .build();

        let mut endpoint_path_item = PathItemBuilder::new()
            .operation(HttpMethod::Get, read_ep_op);

        if sensor.has_endpoint_write {
            let write_ep_op = OperationBuilder::new()
                .tag(tag)
                .summary(Some(format!("{} - Write Endpoint", sensor.display_name)))
                .description(Some(format!(
                    "Write a single endpoint on **{}** by path. \
                     Body must be `{{\"value\": <number|bool>}}` matching the endpoint type.",
                    sensor.display_name
                )))
                .request_body(Some(RequestBodyBuilder::new()
                    .content("application/json", ContentBuilder::new()
                        .example(Some(serde_json::json!({"value": 20.0})))
                        .build())
                    .required(Some(utoipa::openapi::Required::True))
                    .build()))
                .response("200", ResponseBuilder::new()
                    .description("Write confirmed")
                    .content("application/json", ContentBuilder::new().build())
                    .build())
                .build();
            endpoint_path_item = endpoint_path_item.operation(HttpMethod::Post, write_ep_op);
        }

        paths = paths
            .path(format!("{}/commands", base), PathItemBuilder::new()
                .operation(HttpMethod::Get, commands_op)
                .build())
            .path(format!("{}/endpoints", base), PathItemBuilder::new()
                .operation(HttpMethod::Get, list_op)
                .build())
            .path(format!("{}/endpoint/{{path}}", base), endpoint_path_item.build());

        if sensor.has_calibrate {
            let cal_op = OperationBuilder::new()
                .tag(tag)
                .summary(Some(format!("{} - Calibrate", sensor.display_name)))
                .description(Some(format!(
                    "Start a calibration sequence on **{}**.\n\n**Body**: `{{\"type\": \"full\" | \"motor\" | \"encoder_index\" | \"encoder_offset\"}}`\n\n| type | Axis State | Description |\n|---|---|---|\n| `full` | 3 | Full calibration (motor + encoder) |\n| `motor` | 4 | Motor calibration only |\n| `encoder_index` | 6 | Encoder index search |\n| `encoder_offset` | 7 | Encoder offset calibration |\n\nThe drive must be in **Idle** state before calibrating. The sequence runs asynchronously — poll `/data` to watch `axis_state` return to Idle (1).",
                    sensor.display_name
                )))
                .request_body(Some(
                    RequestBodyBuilder::new()
                        .content(
                            "application/json",
                            ContentBuilder::new()
                                .example(Some(serde_json::json!({"type": "full"})))
                                .build(),
                        )
                        .required(Some(utoipa::openapi::Required::True))
                        .build(),
                ))
                .response(
                    "200",
                    ResponseBuilder::new()
                        .description("Calibration started")
                        .content(
                            "application/json",
                            ContentBuilder::new()
                                .schema(Some(RefOr::Ref(utoipa::openapi::Ref::from_schema_name(
                                    "CommandResult",
                                ))))
                                .build(),
                        )
                        .build(),
                )
                .build();

            paths = paths.path(
                format!("{}/calibrate", base),
                PathItemBuilder::new()
                    .operation(HttpMethod::Post, cal_op)
                    .build(),
            );
        }

        // Add sensor as a tag
        doc.tags.get_or_insert_with(Vec::new).push(
            utoipa::openapi::tag::TagBuilder::new()
                .name(tag)
                .description(Some(format!(
                    "{} | {} | Data UDP:{} | Cmd UDP:{}",
                    sensor.display_name, mode_desc, sensor.data_port, sensor.command_port
                )))
                .build(),
        );
    }

    doc.paths = paths.build();
    doc
}

fn build_data_schema(
    fields: &[FieldDescriptor],
) -> utoipa::openapi::RefOr<utoipa::openapi::Schema> {
    use utoipa::openapi::schema::ObjectBuilder;
    use utoipa::openapi::{RefOr, Schema};

    let mut obj = ObjectBuilder::new();
    for field in fields {
        let field_schema = type_to_schema(&field.type_name);
        obj = obj.property(&field.name, field_schema);
    }
    RefOr::T(Schema::Object(obj.build()))
}

fn build_command_schema(
    fields: &[FieldDescriptor],
) -> utoipa::openapi::RefOr<utoipa::openapi::Schema> {
    use utoipa::openapi::schema::ObjectBuilder;
    use utoipa::openapi::{RefOr, Schema};

    let mut obj = ObjectBuilder::new();
    for field in fields {
        let field_schema = type_to_schema(&field.type_name);
        obj = obj.property(&field.name, field_schema);
    }
    RefOr::T(Schema::Object(obj.build()))
}

fn type_to_schema(type_name: &str) -> utoipa::openapi::RefOr<utoipa::openapi::Schema> {
    use utoipa::openapi::schema::{ObjectBuilder, Type};
    use utoipa::openapi::{RefOr, Schema};

    let obj = match type_name {
        "f64" | "f32" => ObjectBuilder::new().schema_type(Type::Number),
        "u8" | "u16" | "u32" | "u64" | "i8" | "i16" | "i32" | "i64" => {
            ObjectBuilder::new().schema_type(Type::Integer)
        }
        "bool" => ObjectBuilder::new().schema_type(Type::Boolean),
        "String" | "str" => ObjectBuilder::new().schema_type(Type::String),
        _ => ObjectBuilder::new(),
    };
    RefOr::T(Schema::Object(obj.build()))
}
