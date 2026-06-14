pub mod detect;
pub mod engines;
pub mod feeds;
pub mod ws;

use std::sync::Arc;

use axum::Router;

use crate::config::AppState;

/// REST API routes (nested under /api).
pub fn rest_router() -> Router<Arc<AppState>> {
    Router::new()
        .merge(engines::router())
        .merge(feeds::router())
        .merge(detect::router())
}
