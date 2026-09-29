//! The administrator's policy settings. Every signed-in user reads them — the
//! page needs to know whether "New revision" is open while a draft is — and
//! only an administrator changes them.

use axum::extract::State;
use axum::http::HeaderMap;
use axum::Json;

use super::{require_admin, require_user, Shared};
use crate::model::Settings;
use crate::Error;

pub async fn get(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<serde_json::Value>, Error> {
    require_user(&db, &headers)?;
    Ok(Json(with_server_facts(&db, db.settings())))
}

/// The settings, plus what the server was started with that bears on them:
/// whether `--lock-script-editor` overrides the editor setting.
fn with_server_facts(db: &crate::db::Db, settings: Settings) -> serde_json::Value {
    let mut value = serde_json::to_value(settings).unwrap_or_default();
    if let Some(map) = value.as_object_mut() {
        map.insert(
            "script_editor_locked".into(),
            db.security().config.lock_script_editor.into(),
        );
    }
    value
}

/// Merge the given settings; unknown keys and non-boolean values are refused
/// ([`crate::db::Db::update_settings`]).
pub async fn update(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<serde_json::Value>,
) -> Result<Json<serde_json::Value>, Error> {
    require_admin(&db, &headers)?;
    let settings = db.update_settings(&body)?;
    Ok(Json(with_server_facts(&db, settings)))
}
