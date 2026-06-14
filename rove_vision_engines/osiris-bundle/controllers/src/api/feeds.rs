use std::sync::Arc;

use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::routing::{delete, post, put};
use axum::{Json, Router};

use crate::config::AppState;
use crate::error::OsirisError;
use crate::models::feed::{FeedClasses, FeedCreate, FeedInfo};

pub fn router() -> Router<Arc<AppState>> {
    Router::new()
        .route("/feeds", post(create_feed).get(list_feeds))
        .route("/feeds/{feed_id}", delete(delete_feed))
        .route("/feeds/{feed_id}/classes", put(update_feed_classes))
}

/// Create a new video feed with assigned engines.
#[utoipa::path(
    post,
    path = "/api/feeds",
    request_body = FeedCreate,
    responses(
        (status = 201, description = "Feed created", body = FeedInfo),
        (status = 400, description = "Invalid source or engine"),
        (status = 404, description = "Engine not found"),
    ),
    tag = "feeds"
)]
pub async fn create_feed(
    State(state): State<Arc<AppState>>,
    Json(req): Json<FeedCreate>,
) -> Result<(StatusCode, Json<FeedInfo>), OsirisError> {
    let info = state.feeds.create_feed(req).await?;
    Ok((StatusCode::CREATED, Json(info)))
}

/// List all active feeds.
#[utoipa::path(
    get,
    path = "/api/feeds",
    responses(
        (status = 200, description = "List of active feeds", body = Vec<FeedInfo>)
    ),
    tag = "feeds"
)]
pub async fn list_feeds(State(state): State<Arc<AppState>>) -> Json<Vec<FeedInfo>> {
    Json(state.feeds.list_feeds().await)
}

/// Stop and remove a feed.
#[utoipa::path(
    delete,
    path = "/api/feeds/{feed_id}",
    params(
        ("feed_id" = String, Path, description = "Feed UUID")
    ),
    responses(
        (status = 204, description = "Feed deleted"),
        (status = 404, description = "Feed not found"),
    ),
    tag = "feeds"
)]
pub async fn delete_feed(
    State(state): State<Arc<AppState>>,
    Path(feed_id): Path<String>,
) -> Result<StatusCode, OsirisError> {
    state.feeds.delete_feed(&feed_id).await?;
    Ok(StatusCode::NO_CONTENT)
}

/// Update which classes a running feed looks for (live, no restart).
#[utoipa::path(
    put,
    path = "/api/feeds/{feed_id}/classes",
    params(
        ("feed_id" = String, Path, description = "Feed UUID")
    ),
    request_body = FeedClasses,
    responses(
        (status = 200, description = "Class filter updated", body = FeedInfo),
        (status = 404, description = "Feed not found"),
    ),
    tag = "feeds"
)]
pub async fn update_feed_classes(
    State(state): State<Arc<AppState>>,
    Path(feed_id): Path<String>,
    Json(req): Json<FeedClasses>,
) -> Result<Json<FeedInfo>, OsirisError> {
    let info = state.feeds.set_classes(&feed_id, req.classes).await?;
    Ok(Json(info))
}
