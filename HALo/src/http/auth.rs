use argon2::{
    password_hash::{PasswordHash, PasswordHasher, PasswordVerifier, SaltString},
    Argon2,
};
use axum::{
    extract::{Request, State},
    http::{header, HeaderMap, StatusCode},
    middleware::{self, Next},
    response::{IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use rand_core::{OsRng, RngCore};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

pub struct Auth {
    hash: Mutex<Option<String>>,
    sessions: Mutex<HashMap<String, Instant>>,
    next_login: Mutex<Instant>,
    hashing: Arc<tokio::sync::Semaphore>,
    secure_cookie: bool,
}
impl Auth {
    pub fn load() -> Result<Arc<Self>, String> {
        let path = crate::core::config::config_path("password.hash").map_err(|e| e.to_string())?;
        let hash = match std::fs::read_to_string(path) {
            Ok(hash) => {
                let hash = hash.trim().to_owned();
                let parsed = PasswordHash::new(&hash).map_err(|e| e.to_string())?;
                if parsed.algorithm.as_str() != "argon2id" {
                    return Err("password hash must use Argon2id".into());
                }
                Some(hash)
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
            Err(e) => return Err(e.to_string()),
        };
        Ok(Arc::new(Self {
            hash: Mutex::new(hash),
            sessions: Mutex::new(HashMap::new()),
            next_login: Mutex::new(Instant::now()),
            hashing: Arc::new(tokio::sync::Semaphore::new(1)),
            secure_cookie: std::env::var("HALO_SECURE_COOKIE").is_ok_and(|v| v == "1"),
        }))
    }
    pub fn authenticated(&self, headers: &HeaderMap) -> bool {
        let Some(token) = headers
            .get(header::COOKIE)
            .and_then(|s| s.to_str().ok())
            .and_then(|s| {
                s.split(';')
                    .find_map(|v| v.trim().strip_prefix("halo_session="))
            })
        else {
            return false;
        };
        self.sessions
            .lock()
            .unwrap()
            .get(token)
            .is_some_and(|at| at.elapsed() < Duration::from_secs(28_800))
    }
    fn cookie(&self, token: &str, max_age: u32) -> String {
        format!(
            "halo_session={token}; Path=/; HttpOnly; SameSite=Strict; Max-Age={max_age}{}",
            if self.secure_cookie { "; Secure" } else { "" }
        )
    }
}
pub fn set_password(password: &str) -> Result<(), String> {
    persist_password(&hash_password(password)?)
}
fn hash_password(password: &str) -> Result<String, String> {
    if !(12..=1024).contains(&password.len()) {
        return Err("password must contain 12..1024 bytes".into());
    }
    let salt = SaltString::generate(&mut OsRng);
    let hash = Argon2::default()
        .hash_password(password.as_bytes(), &salt)
        .map_err(|e| e.to_string())?
        .to_string();
    Ok(hash)
}
fn persist_password(hash: &str) -> Result<(), String> {
    let path = crate::core::config::config_path("password.hash").map_err(|e| e.to_string())?;
    crate::core::config::atomic_write(&path, hash.as_bytes()).map_err(|e| e.to_string())
}

pub async fn protect(State(auth): State<Arc<Auth>>, req: Request, next: Next) -> Response {
    if !auth.authenticated(req.headers()) {
        return (
            StatusCode::UNAUTHORIZED,
            Json(serde_json::json!({"error":"login required"})),
        )
            .into_response();
    }
    if req.method() != axum::http::Method::GET
        && req
            .headers()
            .get("x-halo-request")
            .and_then(|v| v.to_str().ok())
            != Some("1")
    {
        return (
            StatusCode::FORBIDDEN,
            Json(serde_json::json!({"error":"missing management request header"})),
        )
            .into_response();
    }
    let mut response = next.run(req).await;
    response.headers_mut().insert(
        header::CACHE_CONTROL,
        axum::http::HeaderValue::from_static("no-store"),
    );
    response
}

pub fn router(auth: Arc<Auth>) -> Router {
    let protected = Router::new()
        .route("/auth/password", post(change_password))
        .with_state(auth.clone())
        .route_layer(middleware::from_fn_with_state(auth.clone(), protect));
    Router::new()
        .route("/auth/status", get(status))
        .route("/auth/login", post(login))
        .route("/auth/logout", post(logout))
        .with_state(auth)
        .merge(protected)
}
async fn status(State(auth): State<Arc<Auth>>, headers: HeaderMap) -> Json<serde_json::Value> {
    let configured = auth.hash.lock().unwrap().is_some();
    Json(serde_json::json!({"configured":configured,"authenticated":auth.authenticated(&headers)}))
}
#[derive(serde::Deserialize, utoipa::ToSchema)]
pub(crate) struct Credentials {
    password: String,
}
async fn login(
    State(auth): State<Arc<Auth>>,
    headers: HeaderMap,
    Json(credentials): Json<Credentials>,
) -> Response {
    if headers.get("x-halo-request").and_then(|v| v.to_str().ok()) != Some("1") {
        return StatusCode::FORBIDDEN.into_response();
    }
    let Some(hash) = auth.hash.lock().unwrap().clone() else {
        return (StatusCode::SERVICE_UNAVAILABLE,Json(serde_json::json!({"error":"Set a password locally with halo --set-password, then restart HALo."}))).into_response();
    };
    if credentials.password.len() > 1024 {
        return StatusCode::BAD_REQUEST.into_response();
    }
    {
        let mut next = auth.next_login.lock().unwrap();
        if Instant::now() < *next {
            return (
                StatusCode::TOO_MANY_REQUESTS,
                Json(serde_json::json!({"error":"Wait before trying again."})),
            )
                .into_response();
        }
        *next = Instant::now() + Duration::from_secs(2);
    }
    let Ok(permit) = auth.hashing.clone().try_acquire_owned() else {
        return StatusCode::TOO_MANY_REQUESTS.into_response();
    };
    let verified_hash = hash.clone();
    let valid = tokio::task::spawn_blocking(move || {
        let _permit = permit;
        PasswordHash::new(&hash).is_ok_and(|hash| {
            Argon2::default()
                .verify_password(credentials.password.as_bytes(), &hash)
                .is_ok()
        })
    })
    .await
    .unwrap_or(false);
    if !valid {
        return (
            StatusCode::UNAUTHORIZED,
            Json(serde_json::json!({"error":"Incorrect password."})),
        )
            .into_response();
    }
    // Reject a login verified against a password rotated during hashing.
    let current_hash = auth.hash.lock().unwrap();
    if current_hash.as_ref() != Some(&verified_hash) {
        return StatusCode::UNAUTHORIZED.into_response();
    }
    let mut random = [0u8; 32];
    OsRng.fill_bytes(&mut random);
    let token: String = random.iter().map(|b| format!("{b:02x}")).collect();
    {
        let mut sessions = auth.sessions.lock().unwrap();
        sessions.retain(|_, at| at.elapsed() < Duration::from_secs(28_800));
        if sessions.len() >= 32 {
            sessions.clear();
        }
        sessions.insert(token.clone(), Instant::now());
    }
    drop(current_hash);
    (
        [
            (header::SET_COOKIE, auth.cookie(&token, 28_800)),
            (header::CACHE_CONTROL, "no-store".into()),
        ],
        Json(serde_json::json!({"status":"ok"})),
    )
        .into_response()
}
async fn logout(State(auth): State<Arc<Auth>>, headers: HeaderMap) -> Response {
    if headers.get("x-halo-request").and_then(|v| v.to_str().ok()) != Some("1") {
        return StatusCode::FORBIDDEN.into_response();
    }
    if let Some(token) = headers
        .get(header::COOKIE)
        .and_then(|v| v.to_str().ok())
        .and_then(|s| {
            s.split(';')
                .find_map(|v| v.trim().strip_prefix("halo_session="))
        })
    {
        auth.sessions.lock().unwrap().remove(token);
    }
    (
        [(header::SET_COOKIE, auth.cookie("", 0))],
        Json(serde_json::json!({"status":"ok"})),
    )
        .into_response()
}

#[derive(serde::Deserialize, utoipa::ToSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct PasswordChange {
    current_password: String,
    new_password: String,
}
async fn change_password(
    State(auth): State<Arc<Auth>>,
    Json(change): Json<PasswordChange>,
) -> Response {
    let error = |status, text| (status, Json(serde_json::json!({"error":text}))).into_response();
    if change.current_password.len() > 1024 || !(12..=1024).contains(&change.new_password.len()) {
        return error(
            StatusCode::BAD_REQUEST,
            "New password must contain 12..1024 bytes.",
        );
    }
    {
        let mut next = auth.next_login.lock().unwrap();
        if Instant::now() < *next {
            return error(StatusCode::TOO_MANY_REQUESTS, "Wait before trying again.");
        }
        *next = Instant::now() + Duration::from_secs(2);
    }
    let Some(expected) = auth.hash.lock().unwrap().clone() else {
        return StatusCode::SERVICE_UNAVAILABLE.into_response();
    };
    let Ok(permit) = auth.hashing.clone().try_acquire_owned() else {
        return StatusCode::TOO_MANY_REQUESTS.into_response();
    };
    let old = expected.clone();
    let result = tokio::task::spawn_blocking(move || {
        let _permit = permit;
        if !PasswordHash::new(&old).is_ok_and(|hash| {
            Argon2::default()
                .verify_password(change.current_password.as_bytes(), &hash)
                .is_ok()
        }) {
            return Ok(None);
        }
        hash_password(&change.new_password).map(Some)
    })
    .await;
    let hash = match result {
        Ok(Ok(Some(hash))) => hash,
        Ok(Ok(None)) => return error(StatusCode::BAD_REQUEST, "Current password is incorrect."),
        _ => {
            return error(
                StatusCode::INTERNAL_SERVER_ERROR,
                "Could not hash the new password.",
            )
        }
    };
    let mut current = auth.hash.lock().unwrap();
    if current.as_ref() != Some(&expected) {
        return StatusCode::CONFLICT.into_response();
    }
    if let Err(message) = persist_password(&hash) {
        return (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({"error":message})),
        )
            .into_response();
    }
    *current = Some(hash);
    auth.sessions.lock().unwrap().clear();
    (
        [(header::SET_COOKIE, auth.cookie("", 0))],
        Json(serde_json::json!({"status":"password changed; sign in again"})),
    )
        .into_response()
}
