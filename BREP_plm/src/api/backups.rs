//! Backups and exports over HTTP — administrators only. The work is in
//! [`crate::backup`].

use axum::body::Body;
use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap, HeaderValue};
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{blocking, require_admin, Shared};
use crate::backup;
use crate::Error;

/// What is configured, the last backup, and the saved ones.
pub async fn status(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Value>, Error> {
    require_admin(&db, &headers)?;
    let config = &db.security().config;
    let saved = config.backup_dir.as_deref().map(backup::saved).unwrap_or_default();
    Ok(Json(json!({
        "dir": config.backup_dir.as_ref().map(|d| d.display().to_string()),
        "every_minutes": config.backup_every_minutes,
        "keep": if config.backup_keep == 0 { backup::DEFAULT_KEEP } else { config.backup_keep },
        "running": db.backups().running(),
        "last": db.backups().last(),
        "saved": saved,
    })))
}

/// "Back up now" into the backup directory.
pub async fn run(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Value>, Error> {
    require_admin(&db, &headers)?;
    let last = blocking(move || db.backup_now("button")).await?;
    Ok(Json(json!({ "ok": true, "last": last })))
}

/// Stream a file from disk; with `remove`, unlink it once it is open (the
/// open handle keeps it readable until the stream ends).
async fn stream_file(path: &std::path::Path, name: &str, media: &'static str, remove: bool) -> Result<Response, Error> {
    let file = tokio::fs::File::open(path).await.map_err(Error::internal)?;
    let size = file.metadata().await.map_err(Error::internal)?.len();
    if remove {
        let _ = tokio::fs::remove_file(path).await;
    }
    let stream = futures_util::stream::unfold(file, |mut file| async move {
        use tokio::io::AsyncReadExt;
        let mut buffer = vec![0u8; 256 * 1024];
        match file.read(&mut buffer).await {
            Ok(0) => None,
            Ok(n) => {
                buffer.truncate(n);
                Some((Ok::<_, std::io::Error>(buffer), file))
            }
            Err(e) => Some((Err(e), file)),
        }
    });
    let mut response = Body::from_stream(stream).into_response();
    let h = response.headers_mut();
    h.insert(header::CONTENT_TYPE, HeaderValue::from_static(media));
    h.insert(header::CONTENT_LENGTH, HeaderValue::from(size));
    if let Ok(value) = HeaderValue::from_str(&format!("attachment; filename=\"{name}\"")) {
        h.insert(header::CONTENT_DISPOSITION, value);
    }
    Ok(response)
}

/// Download a saved backup by name.
pub async fn saved_file(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(name): Path<String>,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    if !backup::is_backup_name(&name) {
        return Err(Error::bad_request("not a backup name"));
    }
    let dir = db.security().config.backup_dir.clone().ok_or_else(|| Error::not_found("backup directory"))?;
    let path = dir.join(&name);
    if !path.is_file() {
        return Err(Error::not_found("backup"));
    }
    stream_file(&path, &name, "application/gzip", false).await
}

/// Take a fresh backup and download it; nothing is kept on the server.
pub async fn download(State(db): State<Shared>, headers: HeaderMap) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    let (path, _) = blocking(move || db.backup_to_temp()).await?;
    let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| "backup.tar.gz".into());
    // The temporary name carries an id after the time; offer the plain one.
    let offered = match name.rsplit_once('-') {
        Some((head, _)) => format!("{head}{}", backup::SUFFIX),
        None => name,
    };
    stream_file(&path, &offered, "application/gzip", true).await
}

#[derive(Debug, Deserialize, Default)]
pub struct ExportQuery {
    /// `json` (default), `parts-csv` or `structure-csv`.
    #[serde(default)]
    pub format: String,
}

/// The metadata export, as a download.
pub async fn export(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(query): Query<ExportQuery>,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    let stamp = backup::stamp(crate::db::now());
    let (body, media, name) = match query.format.as_str() {
        "" | "json" => (
            db.read(|state| serde_json::to_string_pretty(&backup::export_json(state))).map_err(Error::internal)?,
            "application/json",
            format!("brep-plm-export-{stamp}.json"),
        ),
        "parts-csv" => (db.read(backup::parts_csv), "text/csv; charset=utf-8", format!("brep-plm-parts-{stamp}.csv")),
        "structure-csv" => (
            db.read(backup::structure_csv),
            "text/csv; charset=utf-8",
            format!("brep-plm-structure-{stamp}.csv"),
        ),
        other => return Err(Error::bad_request(format!("format is json, parts-csv or structure-csv, not '{other}'"))),
    };
    let mut response = body.into_response();
    let h = response.headers_mut();
    h.insert(header::CONTENT_TYPE, HeaderValue::from_static(media));
    if let Ok(value) = HeaderValue::from_str(&format!("attachment; filename=\"{name}\"")) {
        h.insert(header::CONTENT_DISPOSITION, value);
    }
    Ok(response)
}
