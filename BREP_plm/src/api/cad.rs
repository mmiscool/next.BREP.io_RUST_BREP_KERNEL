//! The hosted CAD app ([`crate::cad`]) and `GET /cad/config`, the server
//! describing itself to an installer (plm-cad-integration-todo §2 P2, P4).

use axum::extract::{Path, State};
use axum::http::{header, HeaderMap, HeaderValue, StatusCode};
use axum::body::Body;
use axum::response::{IntoResponse, Redirect, Response};
use axum::Json;

use super::accounts::PartTypeView;
use super::{identify, Shared};
use crate::cad;

/// `/cad`, `/cad/`, `/cad/app`, `/cad/app/`: to the page.
pub async fn entry() -> Redirect {
    Redirect::to(cad::ENTRY)
}

/// `GET /cad/app/*path`: one file of the bundle.
pub async fn file(State(db): State<Shared>, Path(path): Path<String>, headers: HeaderMap) -> Response {
    let config = &db.security().config;
    let connect = &config.cad_connect_src;
    let bare = cad::policy(&(Vec::new(), Vec::new()), connect);
    let refuse = |status: StatusCode, message: &str| {
        let mut response = (status, Json(serde_json::json!({ "error": message }))).into_response();
        set(&mut response, header::CONTENT_SECURITY_POLICY, &bare);
        set(&mut response, header::CACHE_CONTROL, "no-store");
        response
    };
    let Some(root) = config.cad_app_dir.as_deref().filter(|dir| dir.is_dir()) else {
        return refuse(
            StatusCode::NOT_FOUND,
            "no CAD app is installed on this server — the administrator puts a build's web/ and pkg/ into the --cad-app directory",
        );
    };
    // A directory URL means its page.
    let path = if path.is_empty() || path.ends_with('/') { format!("{path}index.html") } else { path };
    let Some(found) = cad::resolve(root, &path) else {
        return refuse(StatusCode::NOT_FOUND, "no such file in the CAD app");
    };
    let meta = match std::fs::metadata(&found) {
        Ok(meta) => meta,
        Err(_) => return refuse(StatusCode::NOT_FOUND, "no such file in the CAD app"),
    };
    let etag = cad::etag(&meta);
    let media = cad::media_type(&found);
    let page = media.starts_with("text/html");
    let fresh = headers
        .get(header::IF_NONE_MATCH)
        .and_then(|v| v.to_str().ok())
        .is_some_and(|sent| sent.split(',').any(|tag| tag.trim() == etag || tag.trim() == "*"));

    // The page is small and its policy depends on its text, so it is read
    // whole; everything else streams.
    let (body, policy) = if page {
        let text = match std::fs::read_to_string(&found) {
            Ok(text) => text,
            Err(error) => return refuse(StatusCode::INTERNAL_SERVER_ERROR, &format!("cannot read the page: {error}")),
        };
        let policy = cad::policy(&cad::inline_hashes(&text), connect);
        (Body::from(text), policy)
    } else if fresh {
        (Body::empty(), bare)
    } else {
        let file = match tokio::fs::File::open(&found).await {
            Ok(file) => file,
            Err(error) => return refuse(StatusCode::INTERNAL_SERVER_ERROR, &format!("cannot read the file: {error}")),
        };
        (Body::from_stream(stream(file)), bare)
    };
    let mut response = if fresh {
        StatusCode::NOT_MODIFIED.into_response()
    } else {
        let mut response = body.into_response();
        set(&mut response, header::CONTENT_TYPE, media);
        response.headers_mut().insert(header::CONTENT_LENGTH, HeaderValue::from(meta.len()));
        response
    };
    set(&mut response, header::ETAG, &etag);
    set(&mut response, header::CACHE_CONTROL, "no-cache");
    set(&mut response, header::CONTENT_SECURITY_POLICY, &policy);
    response
}

fn set(response: &mut Response, name: header::HeaderName, value: &str) {
    if let Ok(value) = HeaderValue::from_str(value) {
        response.headers_mut().insert(name, value);
    }
}

fn stream(file: tokio::fs::File) -> impl futures_util::Stream<Item = Result<Vec<u8>, std::io::Error>> {
    futures_util::stream::unfold(file, |mut file| async move {
        use tokio::io::AsyncReadExt;
        let mut buffer = vec![0u8; 256 * 1024];
        match file.read(&mut buffer).await {
            Ok(0) => None,
            Ok(n) => {
                buffer.truncate(n);
                Some((Ok(buffer), file))
            }
            Err(e) => Some((Err(e), file)),
        }
    })
}

/// `GET /cad/config`: what an installer or a CAD app needs to know about this
/// server, given only its hostname.
///
/// **No sign-in is needed**, because the caller has nothing else yet: an
/// installer script reads this before any token exists. It carries nothing
/// secret — the URL the caller already used, the version fields `/api/me`
/// also carries ([`crate::version`]), and where the hosted app is. The part
/// types are the catalog's shape, not public, so they are listed only when the
/// request carries a valid credential, and are `null` otherwise.
///
/// `native` and `installers` name, per platform, the native app and the
/// installer script the `--cad-app` directory holds, each `{ path, size,
/// sha256 }` or `null`: the installer downloads the binary from `path` and
/// refuses it unless the hash matches.
///
/// `url` is rebuilt from the request's `Host` and whether it arrived over TLS
/// ([`crate::security::ServerConfig::is_secure_request`], which honours
/// `--trust-proxy`), never from `--bind`: behind a proxy the bind address is
/// not what a client can reach.
pub async fn config(State(db): State<Shared>, headers: HeaderMap) -> Response {
    let security = &db.security().config;
    let scheme = if security.is_secure_request(&headers) { "https" } else { "http" };
    let url = headers
        .get(header::HOST)
        .and_then(|v| v.to_str().ok())
        .filter(|host| !host.is_empty() && host.chars().all(|c| c.is_ascii_graphic()))
        .map(|host| format!("{scheme}://{host}"));
    let app = security
        .cad_app_dir
        .as_deref()
        .is_some_and(|dir| cad::resolve(dir, "web/index.html").is_some())
        .then_some(cad::ENTRY);
    // `null` for a platform whose file is absent; hashing a new file is
    // blocking work, so it runs on the blocking pool.
    let root = security.cad_app_dir.clone();
    let (native, installers) = tokio::task::spawn_blocking(move || {
        let list = |entries: &[(&str, &str)]| {
            entries
                .iter()
                .map(|(name, path)| {
                    let found = root.as_deref().and_then(|dir| cad::listed(dir, path));
                    (name.to_string(), serde_json::to_value(found).unwrap_or_default())
                })
                .collect::<serde_json::Map<_, _>>()
        };
        (list(cad::NATIVE), list(cad::INSTALLERS))
    })
    .await
    .unwrap_or_default();
    let part_types: Option<Vec<PartTypeView>> = identify(&db, &headers)
        .ok()
        .map(|_| db.read(|state| state.part_types.iter().map(PartTypeView::from).collect()));
    let mut response = Json(serde_json::json!({
        "url": url,
        "server_version": crate::version::SERVER_VERSION,
        "min_client_version": security.min_client_version(),
        "features": crate::version::FEATURES,
        "app": app,
        "part_types": part_types,
        "native": native,
        "installers": installers,
        // The hosted app's extra `connect-src` origins, so it can say a
        // feature is unavailable here instead of letting the browser refuse
        // its request (BREP_app's `offsite` module).
        "connect_src": security.cad_connect_src,
    }))
    .into_response();
    set(&mut response, header::CACHE_CONTROL, "no-store");
    response
}
