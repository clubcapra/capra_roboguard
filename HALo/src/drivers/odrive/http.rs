//! ODrive's shared endpoint upload remains driver-owned and wire compatible.
use super::endpoints::{load_from_str, SharedEndpointMap};
use crate::http::{extension::DriverHttpExtension, routes::ErrorResponse};
use axum::{extract::State, http::StatusCode, routing::post, Json, Router};

async fn upload_endpoints(
    State(ep_map): State<SharedEndpointMap>,
    body: axum::body::Bytes,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<ErrorResponse>)> {
    let content = std::str::from_utf8(&body).map_err(|e| {
        (
            StatusCode::BAD_REQUEST,
            Json(ErrorResponse {
                error: format!("invalid UTF-8: {e}"),
            }),
        )
    })?;
    let count = load_from_str(&ep_map, content).map_err(|e| {
        (
            StatusCode::BAD_REQUEST,
            Json(ErrorResponse {
                error: e.to_string(),
            }),
        )
    })?;
    Ok(Json(serde_json::json!({ "loaded": count, "status": "ok" })))
}

pub fn extension(endpoint_map: SharedEndpointMap) -> DriverHttpExtension {
    use utoipa::openapi::path::{OperationBuilder, PathItemBuilder};
    use utoipa::openapi::request_body::RequestBodyBuilder;
    use utoipa::openapi::response::ResponseBuilder;
    use utoipa::openapi::{ContentBuilder, HttpMethod, OpenApiBuilder, PathsBuilder};
    let upload_ep_op = OperationBuilder::new()
        .tag("odrive")
        .summary(Some("Upload flat_endpoints.json"))
        .description(Some(
            "Upload the ODrive `flat_endpoints.json` file to enable config read/write on all nodes.\n\n\
             **How to get the file** (on your dev machine where odrivetool is installed):\n\
             ```\npython3 -c \"import odrive, os; print(os.path.dirname(odrive.__file__))\"\n```\
             Then find `flat_endpoints.json` in that directory.\n\n\
             **Upload via curl:**\n\
             ```\ncurl -X POST http://raspberrypi.local:8080/odrive/endpoints \\\n  \
             -H 'Content-Type: application/json' \\\n  \
             --data-binary @flat_endpoints.json\n```\n\n\
             Or paste the file contents directly in the request body below.",
        ))
        .request_body(Some(
            RequestBodyBuilder::new()
                .content(
                    "application/json",
                    ContentBuilder::new()
                        .example(Some(serde_json::json!({
                            "fw_version": "0.6.11",
                            "hw_version": "1.0.0",
                            "endpoints": {
                                "axis0.motor.config.phase_resistance": {"id": 123, "type": "float"},
                                "axis0.controller.config.vel_limit": {"id": 456, "type": "float"}
                            }
                        })))
                        .build(),
                )
                .required(Some(utoipa::openapi::Required::True))
                .build(),
        ))
        .response(
            "200",
            ResponseBuilder::new()
                .description("Endpoints loaded")
                .content(
                    "application/json",
                    ContentBuilder::new()
                        .example(Some(serde_json::json!({"loaded": 1234, "status": "ok"})))
                        .build(),
                )
                .build(),
        )
        .build();

    let paths = PathsBuilder::new().path(
        "/odrive/endpoints",
        PathItemBuilder::new()
            .operation(HttpMethod::Post, upload_ep_op)
            .build(),
    );

    let tag = utoipa::openapi::tag::TagBuilder::new()
        .name("odrive")
        .description(Some("ODrive global operations (endpoint map upload)"))
        .build();

    DriverHttpExtension {
        router: Router::new()
            .route("/odrive/endpoints", post(upload_endpoints))
            .with_state(endpoint_map),
        documentation: OpenApiBuilder::new()
            .paths(paths.build())
            .tags(Some(vec![tag]))
            .build(),
    }
}
