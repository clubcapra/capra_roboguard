use std::sync::Arc;

use axum::extract::State;
use axum::routing::get;
use axum::{Json, Router};

use crate::config::AppState;
use crate::models::manifest::EngineInfo;

pub fn router() -> Router<Arc<AppState>> {
    Router::new().route("/engines", get(list_engines))
}

/// List all registered vision engines.
#[utoipa::path(
    get,
    path = "/api/engines",
    responses(
        (status = 200, description = "List of vision engines", body = Vec<EngineInfo>)
    ),
    tag = "engines"
)]
pub async fn list_engines(State(state): State<Arc<AppState>>) -> Json<Vec<EngineInfo>> {
    Json(state.engines.list())
}
