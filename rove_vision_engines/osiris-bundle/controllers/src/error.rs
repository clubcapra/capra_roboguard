use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use serde_json::json;

#[derive(Debug, thiserror::Error)]
pub enum OsirisError {
    #[error("Engine not found: {0}")]
    EngineNotFound(String),

    #[error("Feed not found: {0}")]
    FeedNotFound(String),

    #[error("Engine failed to start: {0}")]
    EngineStartFailed(String),

    #[error("Feed source error: {0}")]
    FeedSourceError(String),

    #[error("Protocol error: {0}")]
    ProtocolError(String),

    #[error("{0}")]
    Internal(String),
}

impl IntoResponse for OsirisError {
    fn into_response(self) -> Response {
        let (status, message) = match &self {
            OsirisError::EngineNotFound(_) => (StatusCode::NOT_FOUND, self.to_string()),
            OsirisError::FeedNotFound(_) => (StatusCode::NOT_FOUND, self.to_string()),
            OsirisError::EngineStartFailed(_) => {
                (StatusCode::INTERNAL_SERVER_ERROR, self.to_string())
            }
            OsirisError::FeedSourceError(_) => (StatusCode::BAD_REQUEST, self.to_string()),
            OsirisError::ProtocolError(_) => {
                (StatusCode::INTERNAL_SERVER_ERROR, self.to_string())
            }
            OsirisError::Internal(_) => (StatusCode::INTERNAL_SERVER_ERROR, self.to_string()),
        };

        let body = json!({ "error": message });
        (status, axum::Json(body)).into_response()
    }
}
