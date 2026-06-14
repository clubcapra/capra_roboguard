mod api;
mod config;
mod engine;
mod error;
mod feed;
mod models;

use std::sync::Arc;

use axum::Router;
use tower_http::cors::CorsLayer;
use tower_http::trace::TraceLayer;
use tracing_subscriber::EnvFilter;
use axum::response::Html;
use utoipa::OpenApi;
use utoipa_scalar::Scalar;

use crate::config::AppState;
use crate::engine::registry::EngineRegistry;
use crate::engine::watcher::EngineWatcher;
use crate::feed::manager::FeedManager;

#[derive(OpenApi)]
#[openapi(
    info(
        title = "Osiris Vision API",
        version = "0.1.0",
        description = "Vision AI orchestrator with real-time WebSocket streaming"
    ),
    paths(
        api::engines::list_engines,
        api::feeds::create_feed,
        api::feeds::list_feeds,
        api::feeds::delete_feed,
        api::feeds::update_feed_classes,
        api::detect::detect_image,
    ),
    components(schemas(
        models::manifest::EngineInfo,
        models::detection::Detection,
        models::detection::BBox,
        models::feed::FeedCreate,
        models::feed::FeedClasses,
        models::feed::FeedStatus,
        models::feed::FeedInfo,
        api::detect::DetectResponse,
    ))
)]
struct ApiDoc;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()))
        .init();

    let vision_engines_dir = std::env::var("OSIRIS_ENGINES_DIR")
        .unwrap_or_else(|_| "../vision_engines".to_string());
    let vision_engines_dir = std::path::Path::new(&vision_engines_dir).canonicalize()
        .unwrap_or_else(|_| std::path::PathBuf::from("../vision_engines"));

    let engines = Arc::new(EngineRegistry::new());
    let feeds = Arc::new(FeedManager::new(engines.clone()));

    let state = Arc::new(AppState {
        engines: engines.clone(),
        feeds,
    });

    // Load existing engines
    engine::loader::load_all_engines(&vision_engines_dir, &engines).await?;

    // Start file watcher for hot-reload
    let _watcher = EngineWatcher::start(vision_engines_dir, engines.clone())?;

    let scalar_html = Scalar::new(ApiDoc::openapi()).to_html();

    let app = Router::new()
        .route("/docs", axum::routing::get(move || {
            let html = scalar_html.clone();
            async move { Html(html) }
        }))
        .nest("/api", api::rest_router())
        .merge(api::ws::router())
        .with_state(state)
        .layer(axum::extract::DefaultBodyLimit::max(50 * 1024 * 1024)) // 50MB
        .layer(CorsLayer::permissive())
        .layer(TraceLayer::new_for_http());

    let bind = std::env::var("OSIRIS_BIND").unwrap_or_else(|_| "0.0.0.0:9090".to_string());
    let listener = tokio::net::TcpListener::bind(&bind).await?;
    tracing::info!("Osiris controller listening on {bind}");
    tracing::info!("Scalar API docs at http://{bind}/docs");

    axum::serve(listener, app).await?;
    Ok(())
}
