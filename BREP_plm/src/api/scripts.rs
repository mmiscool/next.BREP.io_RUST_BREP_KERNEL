//! The administrator's script editor: list, read, write and delete the files
//! in the scripts directory, and test-run a function.
//!
//! Every route needs the admin group. The directory stays the source of truth
//! — these routes read and write those same files, so an edit here shows up
//! in `git status` and a `git pull` shows up here.
//!
//! # A test run is a real run
//!
//! `POST /api/scripts/run` calls the function with the server's full host
//! bindings, as the signed-in admin. Nothing is sandboxed: a test run of an
//! `afterRelease` that posts to the ERP really posts to the ERP, and a
//! `plm.updatePart` really updates the part. The page says so beside the
//! button.

use axum::extract::{Path, State};
use axum::http::HeaderMap;
use axum::response::Response;
use axum::Json;
use serde::{Deserialize, Serialize};
use serde_json::Value;

use super::{blocking, ok_seq, require_admin, Shared};
use crate::scripting::{self, HookDoc, HookResult, ScriptFile, HOOKS};
use crate::Error;

#[derive(Debug, Serialize)]
pub struct Listing {
    /// Where the files are, so an admin knows what to put under git.
    pub dir: String,
    pub files: Vec<ScriptFile>,
    pub hooks: &'static [HookDoc],
}

pub async fn list(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Listing>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(Listing {
        dir: db.scripts().dir().display().to_string(),
        files: db.scripts().list(),
        hooks: HOOKS,
    }))
}

#[derive(Debug, Serialize)]
pub struct FileText {
    pub path: String,
    pub text: String,
}

pub async fn read(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(path): Path<String>,
) -> Result<Json<FileText>, Error> {
    require_admin(&db, &headers)?;
    let text = db
        .scripts()
        .source(&path)?
        .ok_or_else(|| Error::not_found("script"))?;
    Ok(Json(FileText { path, text: text.to_string() }))
}

/// Write a script. The body is the file's text, verbatim.
pub async fn write(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(path): Path<String>,
    body: String,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    editor_allowed(&db)?;
    db.scripts().write(&path, &body)?;
    db.record("script-write", script_entity(&path), format!("{} bytes", body.len()));
    Ok(ok_seq(&db))
}

pub async fn remove(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(path): Path<String>,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    editor_allowed(&db)?;
    db.scripts().delete(&path)?;
    db.record("script-delete", script_entity(&path), "");
    Ok(ok_seq(&db))
}

#[derive(Debug, Deserialize)]
pub struct RunRequest {
    /// The file to run, and the name its messages use.
    pub path: String,
    /// Run this text instead of the saved file — the editor's unsaved buffer.
    #[serde(default)]
    pub source: Option<String>,
    pub function: String,
    #[serde(default)]
    pub input: Value,
}

#[derive(Debug, Serialize)]
pub struct RunResult {
    /// The function returned.
    pub ok: bool,
    pub value: Value,
    /// Why it refused, when it did — the message a user would see.
    pub error: Option<String>,
    pub logs: Vec<String>,
    pub millis: u128,
}

pub async fn run(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(request): Json<RunRequest>,
) -> Result<Json<RunResult>, Error> {
    let user = require_admin(&db, &headers)?;
    editor_allowed(&db)?;
    let function = request.function.trim().to_string();
    if function.is_empty() || !function.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '$') {
        return Err(Error::bad_request("name the function to run"));
    }
    // A test run is a real run with the full host API, so it is on the record
    // before it starts — even one that never returns.
    db.record(
        "script-run",
        script_entity(&request.path),
        format!(
            "{}() {}",
            function,
            if request.source.is_some() { "from the editor's text" } else { "from the saved file" }
        ),
    );
    let result = blocking(move || {
        let started = std::time::Instant::now();
        let outcome = match request.source {
            Some(source) => {
                db.scripts().resolve(&request.path)?;
                scripting::run_source(&db, &user, &source, &request.path, &function, &request.input)
            }
            None => scripting::run(&db, &user, &request.path, &function, &request.input)?,
        };
        let millis = started.elapsed().as_millis();
        Ok(match outcome {
            HookResult::Absent => {
                return Err(Error::not_found(format!("script '{}'", request.path)));
            }
            HookResult::Returned(success) => RunResult {
                ok: true,
                value: success.value,
                error: None,
                logs: success.logs,
                millis,
            },
            HookResult::Refused(failure) => RunResult {
                ok: false,
                value: Value::Null,
                error: Some(failure.message),
                logs: failure.logs,
                millis,
            },
        })
    })
    .await?;
    Ok(Json(result))
}

/// Refuse an edit or a test run while the in-browser editor is off
/// ([`crate::model::Settings::script_editor_enabled`], or the server's
/// `--lock-script-editor`). Listing and reading stay open: the scripts are
/// still live, and an admin can see what runs.
fn editor_allowed(db: &crate::db::Db) -> Result<(), Error> {
    if db.script_editor_allowed() {
        return Ok(());
    }
    Err(Error::forbidden(if db.security().config.lock_script_editor {
        "the in-browser script editor is locked off on this server — edit the scripts directory on the host"
    } else {
        "the in-browser script editor is turned off in Settings — edit the scripts directory on the host, or turn it back on"
    }))
}

fn script_entity(path: &str) -> crate::model::EntityRef {
    crate::model::EntityRef { kind: "script".into(), id: path.to_string(), label: path.to_string(), part_id: String::new() }
}
