use super::auth::Auth;
use crate::core::{config, supervisor::Supervisor};
use axum::{
    extract::{Path, State},
    http::StatusCode,
    middleware,
    routing::{get, post},
    Json, Router,
};
use serde_json::{json, Value};
use std::sync::Arc;
type Error = (StatusCode, Json<Value>);
fn bad(error: impl ToString) -> Error {
    (
        StatusCode::BAD_REQUEST,
        Json(json!({"error":error.to_string()})),
    )
}

pub fn router(manager: Arc<Supervisor>, auth: Arc<Auth>) -> Router {
    let protected = Router::new()
        .route("/admin/state", get(state))
        .route("/admin/drivers/{id}/retry", post(retry))
        .route(
            "/admin/drivers/{id}/config",
            get(read_config).post(save_config),
        )
        .route("/admin/drivers/{id}/policy", post(save_policy))
        .route("/admin/sensors/{id}/ports", post(ports))
        .route("/admin/sensors/{id}/command", post(command))
        .route("/admin/sensors/{id}/estop", post(estop))
        .route("/admin/reload", post(super::routes::reload))
        .with_state(manager.clone())
        .route_layer(middleware::from_fn_with_state(auth, super::auth::protect));
    protected.merge(
        Router::new()
            .route("/health", get(health))
            .with_state(manager),
    )
}
async fn state(State(manager): State<Arc<Supervisor>>) -> Json<Value> {
    Json(
        json!({"backend":if manager.simulated.load(std::sync::atomic::Ordering::Acquire) {"simulator"} else {"hardware"},"drivers":manager.status(),"sensors":manager.registry.all(),"ports":manager.registry.assignments().sensors}),
    )
}
async fn retry(
    State(manager): State<Arc<Supervisor>>,
    Path(id): Path<String>,
) -> Result<Json<Value>, Error> {
    manager
        .retry(&id)
        .map_err(|e| (StatusCode::CONFLICT, Json(json!({"error":e}))))?;
    Ok(Json(json!({"status":"retry scheduled","driver":id})))
}
async fn command(
    State(manager): State<Arc<Supervisor>>,
    Path(id): Path<String>,
    Json(payload): Json<Value>,
) -> Result<Json<Value>, Error> {
    let driver = manager.registry.get(&id).map_err(bad)?;
    let result = tokio::task::spawn_blocking(move || driver.execute_command(&payload))
        .await
        .map_err(bad)?
        .map_err(bad)?;
    Ok(Json(result))
}
async fn read_config(
    State(manager): State<Arc<Supervisor>>,
    Path(id): Path<String>,
) -> Result<Json<Value>, Error> {
    let (path, _) = manager.configuration(&id).map_err(bad)?;
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => String::new(),
        Err(e) => return Err(bad(e)),
    };
    Ok(Json(json!({"text":text})))
}

async fn estop(
    State(manager): State<Arc<Supervisor>>,
    Path(id): Path<String>,
) -> Result<Json<Value>, Error> {
    let driver = manager.registry.get(&id).map_err(bad)?;
    if !driver.has_estop() {
        return Err(bad("driver does not support emergency stop"));
    }
    let result = tokio::task::spawn_blocking(move || driver.estop())
        .await
        .map_err(bad)?
        .map_err(bad)?;
    Ok(Json(result))
}
#[derive(serde::Deserialize, utoipa::ToSchema)]
pub(crate) struct ConfigEdit {
    text: String,
}
async fn save_config(
    State(manager): State<Arc<Supervisor>>,
    Path(id): Path<String>,
    Json(edit): Json<ConfigEdit>,
) -> Result<Json<Value>, Error> {
    let (path, validate) = manager.configuration(&id).map_err(bad)?;
    let connection_ports = validate(&edit.text).map_err(bad)?;
    if manager.simulated.load(std::sync::atomic::Ordering::Acquire) {
        return Err(bad(
            "physical connection editing is unavailable in simulator mode",
        ));
    }
    let driver = manager
        .status()
        .into_iter()
        .find(|s| s.id == id)
        .ok_or_else(|| bad("hardware configuration unavailable in simulator mode"))?;
    if matches!(
        driver.state.as_str(),
        "connecting" | "stopping" | "retrying" | "timed_out"
    ) {
        return Err((
            StatusCode::CONFLICT,
            Json(json!({"error":"wait for current recovery to finish"})),
        ));
    }
    manager
        .registry
        .reserve_external_ports(&connection_ports)
        .map_err(bad)?;
    config::atomic_write(&path, edit.text.as_bytes()).map_err(bad)?;
    manager.retry(&id).map_err(bad)?;
    Ok(Json(json!({"status":"saved; retry scheduled"})))
}
#[derive(serde::Deserialize, utoipa::ToSchema)]
pub(crate) struct Ports {
    data_port: u16,
    command_port: u16,
}
async fn ports(
    State(manager): State<Arc<Supervisor>>,
    Path(id): Path<String>,
    Json(ports): Json<Ports>,
) -> Result<Json<Value>, Error> {
    manager
        .change_ports(&id, ports.data_port, ports.command_port)
        .await
        .map_err(bad)?;
    Ok(Json(json!({"status":"saved"})))
}
async fn save_policy(
    State(manager): State<Arc<Supervisor>>,
    Path(id): Path<String>,
    Json(policy): Json<crate::core::supervisor::RetryPolicy>,
) -> Result<Json<Value>, Error> {
    manager.set_policy(&id, policy).map_err(bad)?;
    Ok(Json(
        json!({"status":"saved; applies to next recovery cycle"}),
    ))
}

async fn health(State(manager): State<Arc<Supervisor>>) -> Json<Value> {
    let drivers = manager.status();
    let state = if drivers
        .iter()
        .any(|d| d.state == "failed" || d.state == "degraded")
    {
        "degraded"
    } else if drivers
        .iter()
        .any(|d| ["connecting", "retrying", "stopping", "timed_out"].contains(&d.state.as_str()))
    {
        "initializing"
    } else {
        "ready"
    };
    Json(json!({"status":state,"drivers":drivers}))
}
