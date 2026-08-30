//! REST API — post GoTo missions (and read status) over HTTP.
//!
//! Missions arrive over HTTP (matching the rest of the stack — rove_sensor_api and
//! rove_ik_engine are REST), not the legacy UDP `Mission` proto. A posted GoTo is
//! turned into the SAME `Mission` the control loop already compiles, so the
//! autonomy path (router → GoTo → cost-map planner → reflexes → tracks) is
//! unchanged. The UDP mission listener stays as a fallback.
//!
//!   POST /api/v1/goto   {"lat":46.96,"lon":7.79}      # absolute geodetic
//!   POST /api/v1/goto   {"east":5.0,"north":0.0}      # ENU offset about the datum
//!   GET  /api/v1/status                               # latest fused pose
//!   GET  /healthz

use std::sync::Arc;

use axum::{
    extract::State,
    http::StatusCode,
    routing::{get, post},
    Json, Router,
};
use serde::Deserialize;
use serde_json::json;
use tokio::sync::watch;

use crate::comms::proto;
use crate::config::Datum;
use crate::position::{geodetic, Pose};

#[derive(Clone)]
pub struct ApiState {
    /// Shared with the UDP mission listener; the control loop reads the rx.
    pub mission_tx: Arc<watch::Sender<Option<proto::Mission>>>,
    pub datum: Datum,
    pub pose_rx: watch::Receiver<Option<Pose>>,
}

#[derive(Deserialize)]
struct GotoReq {
    lat: Option<f64>,
    lon: Option<f64>,
    east: Option<f64>,  // ENU offset about the site datum (alternative to lat/lon)
    north: Option<f64>,
    name: Option<String>,
}

pub fn router(state: ApiState) -> Router {
    Router::new()
        .route("/api/v1/goto", post(goto))
        .route("/api/v1/status", get(status))
        .route("/healthz", get(|| async { "ok" }))
        .with_state(state)
}

async fn goto(
    State(st): State<ApiState>,
    Json(req): Json<GotoReq>,
) -> (StatusCode, Json<serde_json::Value>) {
    let (lat, lon) = match (req.lat, req.lon, req.east, req.north) {
        (Some(lat), Some(lon), _, _) => (lat, lon),
        (_, _, Some(e), Some(n)) => geodetic::enu_to_geodetic(e, n, st.datum.lat, st.datum.lon),
        _ => {
            return (
                StatusCode::BAD_REQUEST,
                Json(json!({"error": "need {lat,lon} or {east,north}"})),
            )
        }
    };
    let m = proto::Mission {
        schema_version: 0,
        firmware_hash: String::new(),
        name: req.name.unwrap_or_else(|| "rest-goto".into()),
        variables: vec![],
        sequence: vec![proto::Step {
            command: "goto".into(),
            params: vec![proto::Param {
                name: "coordinate".into(),
                value: Some(proto::Value {
                    kind: Some(proto::value::Kind::Coordinate(proto::Coord {
                        lat,
                        lon,
                        alt: 0.0,
                    })),
                }),
            }],
            binds: String::new(),
            transition: None,
        }],
    };
    let (ex, ny) = geodetic::geodetic_to_enu(lat, lon, st.datum.lat, st.datum.lon);
    // Same channel the UDP listener feeds; the control loop compiles + swaps the
    // router on its next tick (once a GNSS fix has armed autonomy).
    let _ = st.mission_tx.send(Some(m));
    tracing::info!("REST GoTo accepted: lat {lat:.7} lon {lon:.7} (ENU {ex:.1}, {ny:.1})");
    (
        StatusCode::ACCEPTED,
        Json(json!({ "accepted": true, "lat": lat, "lon": lon, "enu": [ex, ny] })),
    )
}

async fn status(State(st): State<ApiState>) -> Json<serde_json::Value> {
    match *st.pose_rx.borrow() {
        Some(p) => Json(json!({
            "fix": p.gnss_fix,
            "x": p.x, "y": p.y, "z": p.z,
            "yaw_ned_deg": p.yaw_ned_deg, "roll_deg": p.roll_deg, "pitch_deg": p.pitch_deg,
            "tilt_deg": p.tilt_deg,
            "position_confidence": p.position_confidence, "drift_m": p.drift_m,
        })),
        None => Json(json!({ "fix": false, "pose": null })),
    }
}

/// Bind + serve the REST API until aborted.
pub async fn serve(port: u16, state: ApiState) -> anyhow::Result<()> {
    let listener = tokio::net::TcpListener::bind(("0.0.0.0", port)).await?;
    tracing::info!("REST API up on :{port} (POST /api/v1/goto, GET /api/v1/status)");
    axum::serve(listener, router(state).into_make_service()).await?;
    Ok(())
}
