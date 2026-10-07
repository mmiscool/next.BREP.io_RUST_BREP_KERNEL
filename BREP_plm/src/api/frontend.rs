//! Embedded frontend with optional per-request overrides from `--web-dir`.
use super::Shared;
use axum::{
    extract::{Path, State},
    http::{header, StatusCode},
    response::{IntoResponse, Response},
};

pub async fn index(State(db): State<Shared>) -> Response {
    serve(&db, "index.html").await
}

pub async fn file(State(db): State<Shared>, Path(path): Path<String>) -> Response {
    // An unknown API or CAD route must never resolve to a frontend file.
    if matches!(path.split('/').next(), Some("api" | "cad")) {
        return StatusCode::NOT_FOUND.into_response();
    }
    serve(&db, &path).await
}

async fn serve(db: &Shared, path: &str) -> Response {
    let root = db.security().config.web_dir.as_deref();
    let mime = crate::cad::media_type(std::path::Path::new(path));
    let mut response = if let Some(file) = root.and_then(|root| crate::cad::resolve(root, path)) {
        match tokio::fs::read(file).await {
            Ok(bytes) => ([(header::CONTENT_TYPE, mime)], bytes).into_response(),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => embedded(path, mime),
            Err(_) => StatusCode::INTERNAL_SERVER_ERROR.into_response(),
        }
    } else {
        embedded(path, mime)
    };
    if root.is_some() {
        response.headers_mut().insert(header::CACHE_CONTROL, "no-store".parse().unwrap());
    }
    response
}

fn embedded(path: &str, mime: &'static str) -> Response {
    let bytes: &'static [u8] = match path {
        "index.html" => include_bytes!("../../web/index.html"),
        "workflows.js" => include_bytes!("../../web/workflows.js"),
        "setup.js" => include_bytes!("../../web/setup.js"),
        "app.js" => include_bytes!("../../web/app.js"),
        "split-panes.js" => include_bytes!("../../web/split-panes.js"),
        "style.css" => include_bytes!("../../web/style.css"),
        "native-import.js" => include_bytes!("../../web/native-import.js"),
        "workbench.js" => include_bytes!("../../web/workbench.js"),
        "open-pages.js" => include_bytes!("../../web/open-pages.js"),
        "file-documents.js" => include_bytes!("../../web/file-documents.js"),
        "markdown-editor.js" => include_bytes!("../../web/markdown-editor.js"),
        "vendor/toastui-editor.js" => include_bytes!("../../web/vendor/toastui-editor.js"),
        "vendor/toastui-editor.css" => include_bytes!("../../web/vendor/toastui-editor.css"),
        "vendor/purify.min.js" => include_bytes!("../../web/vendor/purify.min.js"),
        _ => return StatusCode::NOT_FOUND.into_response(),
    };
    ([(header::CONTENT_TYPE, mime)], bytes).into_response()
}
