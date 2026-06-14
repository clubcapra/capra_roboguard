//! WebSocket endpoint for streaming detections.
//!
//! Clients connect to /ws/{engine_name}/{feed_id} to receive real-time
//! detection JSON messages for a specific engine and feed pair.

use std::sync::Arc;

use axum::extract::ws::{Message, WebSocket};
use axum::extract::{Path, State, WebSocketUpgrade};
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;

use crate::config::AppState;

pub fn router() -> Router<Arc<AppState>> {
    Router::new().route("/ws/{engine_name}/{feed_id}", get(ws_handler))
}

async fn ws_handler(
    ws: WebSocketUpgrade,
    Path((engine_name, feed_id)): Path<(String, String)>,
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    tracing::info!("WS connection request: {engine_name}/{feed_id}");

    ws.on_upgrade(move |socket| handle_socket(socket, engine_name, feed_id, state))
}

async fn handle_socket(
    mut socket: WebSocket,
    engine_name: String,
    feed_id: String,
    state: Arc<AppState>,
) {
    // Subscribe to detections for this engine+feed pair
    let mut rx = match state.feeds.subscribe_detections(&engine_name, &feed_id).await {
        Ok(rx) => rx,
        Err(e) => {
            let err_msg = serde_json::json!({"error": e.to_string()});
            let _ = socket
                .send(Message::Text(err_msg.to_string().into()))
                .await;
            return;
        }
    };

    tracing::info!("WS client connected: {engine_name}/{feed_id}");

    loop {
        tokio::select! {
            // Forward detections to client
            result = rx.recv() => {
                match result {
                    Ok(msg) => {
                        let json = match serde_json::to_string(&msg) {
                            Ok(j) => j,
                            Err(e) => {
                                tracing::error!("WS serialize error: {e}");
                                continue;
                            }
                        };
                        if socket.send(Message::Text(json.into())).await.is_err() {
                            break; // Client disconnected
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                        tracing::debug!("WS {engine_name}/{feed_id}: lagged {n} messages");
                        continue;
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                        let _ = socket.send(Message::Text(
                            r#"{"event":"feed_closed"}"#.to_string().into()
                        )).await;
                        break;
                    }
                }
            }
            // Handle incoming messages from client (pings, close)
            msg = socket.recv() => {
                match msg {
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Ok(Message::Ping(data))) => {
                        let _ = socket.send(Message::Pong(data)).await;
                    }
                    _ => {}
                }
            }
        }
    }

    tracing::info!("WS client disconnected: {engine_name}/{feed_id}");
}
