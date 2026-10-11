//! Optional driver-owned routes for legacy or shared-resource operations.
//! Standard per-device capabilities need no HTTP extension.
pub struct DriverHttpExtension {
    pub router: axum::Router,
    pub documentation: utoipa::openapi::OpenApi,
}
