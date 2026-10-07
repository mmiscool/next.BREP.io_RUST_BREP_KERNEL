//! The per-user preferences store (plm-cad-integration-todo D1, P5): where the
//! CAD app keeps its reserved keys when the PLM is its store, so a user's
//! settings, dock layout, pins and libraries follow them to any machine, and
//! the mirror of their unsaved work (`@recovery`) can be resumed on another.
//!
//! | Route | Answers |
//! | --- | --- |
//! | `GET /api/me/preferences[?since=v]` | `{ version, keys: [{ key, version, size, modified, deleted }] }` |
//! | `GET /api/me/preferences/:key` | the value's bytes, `X-Preference-Version: v`; `204` (no body) when unset |
//! | `PUT /api/me/preferences/:key` | `{ ok, key, version, size }` |
//! | `DELETE /api/me/preferences/:key` | `{ ok, key, version }` |
//!
//! # Whose
//!
//! The caller's, always: there is no user in the path, so no request can name
//! another account's set — an administrator's included. A browser session or
//! an API token of the same account reads and writes the same set; a `read`
//! token may only read, and the scope rules apply as to any route.
//!
//! # Which keys
//!
//! Exactly [`KEYS`], the CAD app's reserved keys. Any other key is a `400`:
//! documents go to the store routes, and directory rows are not used in PLM
//! mode (D1). A value is opaque JSON — it must parse, and is kept byte for
//! byte.
//!
//! # How a second client notices
//!
//! Each user's set has one VERSION, bumped by every write and every delete of
//! a live key, and each row carries the version it was last written at. A
//! client polls `GET /api/me/preferences?since=<the version it holds>` and
//! gets back only the keys that moved, a deleted one with `deleted: true`;
//! `since` absent or 0 lists the live keys. That is the store feed's shape
//! (`/api/store/changes?since=`) without its `stale`: a deleted key is kept as
//! a row, so the answer is always complete. The last write wins; there is no
//! precondition.
//!
//! This version is NOT the store's `seq`: a preference is not a document, and
//! a dock layout saved must not move every other client's document feed.
//!
//! # Where
//!
//! The `preferences` table of `plm.sqlite`, read and written directly — not
//! the in-memory state every other record lives in, because every change
//! re-serializes that state's small fields and a `@recovery` value is whole
//! unsaved models. Being in `plm.sqlite`, it is in every backup, and a restore
//! brings it back.
//!
//! # The audit log
//!
//! A write or delete of any key but `@recovery` is logged as kind `preference`,
//! entity `<user id>/<key>`, action `update` or `delete`, with the value's size
//! and never the value, in the SAME transaction as the write. `@recovery` is
//! not logged: the app rewrites it seconds after every edit while a document
//! is unsaved and removes it the moment one is saved, so its events would
//! bury everything else a user did, and they would record nothing a reviewer
//! can act on — the document saves they lead to are logged.
//!
//! # Size
//!
//! A settings key holds at most [`PREFERENCE_LIMIT`]; `@recovery` holds unsaved
//! documents, so it takes the document limit (`--max-document-mb`). Over
//! either is a `413` with a sentence.

use axum::body::Body;
use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;
use serde_json::json;

use super::store::{megabytes, read_bounded};
use super::{blocking, require_user, Shared};
use crate::model::{AuditEvent, EntityRef, User};
use crate::Error;

/// The largest value a settings key may hold, in bytes.
pub const PREFERENCE_LIMIT: u64 = 256 * 1024;

/// The mirror of the app's autosave blob.
pub const RECOVERY: &str = "@recovery";

/// The keys a user may keep: the CAD app's reserved keys (`BREP_app`'s
/// `store.rs`), and nothing else.
pub const KEYS: &[&str] = &[
    "@settings",
    "@dock_layout",
    "@pinned",
    "@feature_palette_display",
    "@kicad_library",
    "@recent_documents",
    RECOVERY,
];

/// The header a value's version rides on.
pub const VERSION_HEADER: &str = "x-preference-version";

fn reserved(key: &str) -> Result<&'static str, Error> {
    KEYS.iter().copied().find(|k| *k == key).ok_or_else(|| {
        Error::bad_request(format!("'{key}' is not a preference — the keys are {}", KEYS.join(", ")))
    })
}

#[derive(Debug, Deserialize)]
pub struct Since {
    #[serde(default)]
    pub since: u64,
}

/// The caller's keys, as metadata.
pub async fn list(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(query): Query<Since>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let (version, keys) = db.sql().preferences(&user.id, query.since).map_err(Error::internal)?;
    Ok(Json(json!({ "version": version, "keys": keys })).into_response())
}

/// One of the caller's values, byte for byte.
pub async fn read(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(key): Path<String>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let key = reserved(&key)?;
    let found = blocking(move || db.sql().preference(&user.id, key).map_err(Error::internal)).await?;
    // Unset is an answer, not an error — as a revision with no document answers `204` on
    // the store routes. A `404` here made a browser log an error for every key a new
    // user had never set, on every first visit.
    let Some((value, version)) = found else {
        return Ok(StatusCode::NO_CONTENT.into_response());
    };
    Ok((
        StatusCode::OK,
        [(header::CONTENT_TYPE, "application/json; charset=utf-8".to_string()), (header::HeaderName::from_static(VERSION_HEADER), version.to_string())],
        value,
    )
        .into_response())
}

/// Set one of the caller's values.
pub async fn write(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(key): Path<String>,
    body: Body,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let key = reserved(&key)?;
    let limit = if key == RECOVERY { db.security().config.document_limit() } else { PREFERENCE_LIMIT };
    let what = if key == RECOVERY { "document" } else { "preference" };
    let value = read_bounded(&headers, body, limit, || {
        format!("the {key} value is over the {} {what} limit", megabytes(limit))
    })
    .await?;
    if serde_json::from_str::<serde_json::Value>(&value).is_err() {
        return Err(Error::bad_request(format!("a preference must be JSON, and the {key} value is not")));
    }
    let size = value.len();
    let version = blocking(move || {
        let event = audited(&user, key, "update", format!("{size} bytes"));
        db.sql().set_preference(&user.id, key, Some(&value), crate::db::now(), event).map_err(Error::internal)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "key": key, "version": version, "size": size })).into_response())
}

/// Remove one of the caller's values. Removing one that is not set is not an
/// error and changes nothing.
pub async fn remove(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(key): Path<String>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let key = reserved(&key)?;
    let version = blocking(move || {
        let event = audited(&user, key, "delete", String::new());
        db.sql().set_preference(&user.id, key, None, crate::db::now(), event).map_err(Error::internal)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "key": key, "version": version })).into_response())
}

/// The event a change to `key` is logged as, or none for `@recovery`.
fn audited(user: &User, key: &str, action: &str, detail: String) -> Option<AuditEvent> {
    (key != RECOVERY).then(|| AuditEvent {
        id: 0,
        at: crate::db::now(),
        actor: crate::audit::current_actor(),
        action: action.into(),
        entity: EntityRef {
            kind: "preference".into(),
            id: format!("{}/{key}", user.id),
            label: format!("{} {key}", user.username),
            part_id: String::new(),
        },
        changes: Default::default(),
        detail,
        seq: 0,
    })
}
