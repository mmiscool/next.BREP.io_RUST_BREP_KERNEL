//! Administrator scripts: the directory they live in, the hooks the server
//! calls, and the test run the web editor offers.
//!
//! The operator's design:
//!
//! * Scripts are plain `.js` FILES in a directory on the server
//!   (`--scripts`, default `<data>/scripts`). The directory is the source of
//!   truth, so an administrator can keep it in git. The web editor writes the
//!   same files. History is git's job.
//! * A changed file takes effect on the NEXT call: [`Scripts::source`] re-reads
//!   a file whenever its modification time or length differs from what it
//!   cached. There is no watcher thread and no restart.
//! * Scripts are FULLY TRUSTED — network, the PLM API, the file system and
//!   child processes ([`host`]).
//! * A script that throws, fails to parse or times out in a host call REFUSES
//!   the operation it hooks. The one exception is [`AFTER_RELEASE`], which runs
//!   after the release is committed and so can only report.
//!
//! # The hooks
//!
//! One file per hook, each defining one global function that takes one JSON
//! argument:
//!
//! | File | Function | When |
//! | --- | --- | --- |
//! | the type's own file | `partNumber(input)` | creating a part of a Script-mode type |
//! | [`REVISION_LABEL`] | `revisionLabel(input)` | creating any revision, when the file exists |
//! | [`BEFORE_RELEASE`] | `beforeRelease(input)` | before a release commits, when the file exists |
//! | [`AFTER_RELEASE`] | `afterRelease(input)` | after a release commits, when the file exists |
//!
//! A hook NEVER runs while the store lock is held: the store decides on a
//! snapshot, runs the hook, and then re-checks everything the hook's answer
//! depends on inside its one locked write (see [`crate::db`]).

// The interpreter core lives in its own crate so the CAD app can build it,
// wasm included (BREP_script_core); it keeps its old name here.
pub use brep_script_core as engine;
pub mod host;

use std::collections::HashMap;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::SystemTime;

use serde::Serialize;
use serde_json::Value;

use crate::db::Db;
use crate::model::User;
use crate::Error;

/// Suggests or validates the label of every new revision.
pub const REVISION_LABEL: &str = "revision-label.js";
/// May refuse a release.
pub const BEFORE_RELEASE: &str = "before-release.js";
/// Told about a release after it commits.
pub const AFTER_RELEASE: &str = "after-release.js";
/// Proposes a review round's reviewers when a revision is submitted.
pub const REVIEWERS: &str = "reviewers.js";
/// Told about every review event after it commits — the notification hook.
pub const REVIEW_EVENT: &str = "review-event.js";
/// The file a new Script-mode part type is offered by default.
pub const DEFAULT_PART_NUMBER: &str = "part-number.js";

/// The hooks, for the page: which file, which function, what it is for.
#[derive(Debug, Clone, Serialize)]
pub struct HookDoc {
    pub file: &'static str,
    pub function: &'static str,
    pub when: &'static str,
}

pub const HOOKS: &[HookDoc] = &[
    HookDoc {
        file: "(named by the part type)",
        function: "partNumber",
        when: "creating a part of a Script-mode part type; return the number or throw to refuse",
    },
    HookDoc {
        file: "(named by the change-order numbering)",
        function: "ecoNumber",
        when: "creating a change order when change orders number by script; return the number or throw to refuse",
    },
    HookDoc {
        file: REVISION_LABEL,
        function: "revisionLabel",
        when: "creating a revision; return the label or throw to refuse",
    },
    HookDoc {
        file: BEFORE_RELEASE,
        function: "beforeRelease",
        when: "before a release commits; throw to refuse the release",
    },
    HookDoc {
        file: AFTER_RELEASE,
        function: "afterRelease",
        when: "after a release commits; a throw is reported, the release stands",
    },
    HookDoc {
        file: REVIEWERS,
        function: "reviewers",
        when: "submitting a revision or a change order (input.kind is eco) for review; return { reviewers, required_approvals, due } to change the round, or throw to refuse",
    },
    HookDoc {
        file: REVIEW_EVENT,
        function: "reviewEvent",
        when: "after a review is opened, decided, withdrawn, changed or commented on, and a change order released or cancelled; for notifications — a throw is reported, the event stands",
    },
];

/// One file in the directory, as the page lists it.
#[derive(Debug, Clone, Serialize)]
pub struct ScriptFile {
    pub path: String,
    pub size: u64,
    pub modified: u64,
}

struct Cached {
    modified: Option<SystemTime>,
    len: u64,
    text: Arc<str>,
}

/// The scripts directory and a modification-time cache of its files.
pub struct Scripts {
    dir: PathBuf,
    cache: Mutex<HashMap<PathBuf, Cached>>,
}

impl Scripts {
    pub fn new(dir: impl Into<PathBuf>) -> io::Result<Self> {
        let dir = dir.into();
        fs::create_dir_all(&dir)?;
        Ok(Scripts { dir, cache: Mutex::new(HashMap::new()) })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// The file `relative` names, refusing anything that could leave the
    /// directory or is not a `.js` file. Scripts are fully trusted once they
    /// run; the EDITOR route still should not write outside its directory.
    pub fn resolve(&self, relative: &str) -> Result<PathBuf, Error> {
        let relative = relative.trim().trim_start_matches('/');
        let mut path = self.dir.clone();
        let refuse = || Error::bad_request(format!("'{relative}' is not a usable script path — letters, digits, '-', '_', '.', '/' and a .js name"));
        if !relative.ends_with(".js") {
            return Err(refuse());
        }
        for segment in relative.split('/') {
            if segment.is_empty()
                || segment.starts_with('.')
                || !segment
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.'))
            {
                return Err(refuse());
            }
            path.push(segment);
        }
        Ok(path)
    }

    /// The current text of `relative`, or `None` when there is no such file.
    ///
    /// This is the reload rule: the cached text is used only while the file's
    /// modification time AND length are what they were when it was read. A
    /// `git pull`, an editor save or the web editor all change one of them.
    pub fn source(&self, relative: &str) -> Result<Option<Arc<str>>, Error> {
        let path = self.resolve(relative)?;
        let meta = match fs::metadata(&path) {
            Ok(meta) if meta.is_file() => meta,
            Ok(_) => return Ok(None),
            Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(None),
            Err(error) => return Err(Error::internal(error)),
        };
        let modified = meta.modified().ok();
        let len = meta.len();
        let mut cache = self.cache.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(hit) = cache.get(&path) {
            if hit.modified == modified && hit.len == len {
                return Ok(Some(hit.text.clone()));
            }
        }
        let text: Arc<str> = match fs::read_to_string(&path) {
            Ok(text) => text.into(),
            Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(None),
            Err(error) => return Err(Error::internal(error)),
        };
        cache.insert(path, Cached { modified, len, text: text.clone() });
        Ok(Some(text))
    }

    /// Every `.js` file under the directory, sorted by path.
    pub fn list(&self) -> Vec<ScriptFile> {
        let mut out = Vec::new();
        walk(&self.dir, &self.dir, &mut out);
        out.sort_by(|a, b| a.path.cmp(&b.path));
        out
    }

    /// Write a script through a temporary file and a rename, as the store does.
    pub fn write(&self, relative: &str, text: &str) -> Result<(), Error> {
        let path = self.resolve(relative)?;
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(Error::internal)?;
        }
        let temp = path.with_extension("js.tmp");
        fs::write(&temp, text).map_err(Error::internal)?;
        fs::rename(&temp, &path).map_err(Error::internal)?;
        self.forget(&path);
        Ok(())
    }

    pub fn delete(&self, relative: &str) -> Result<(), Error> {
        let path = self.resolve(relative)?;
        match fs::remove_file(&path) {
            Ok(()) => {}
            Err(error) if error.kind() == io::ErrorKind::NotFound => {}
            Err(error) => return Err(Error::internal(error)),
        }
        self.forget(&path);
        Ok(())
    }

    fn forget(&self, path: &Path) {
        self.cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .remove(path);
    }
}

fn walk(root: &Path, dir: &Path, out: &mut Vec<ScriptFile>) {
    let Ok(entries) = fs::read_dir(dir) else { return };
    for entry in entries.flatten() {
        let path = entry.path();
        let name = entry.file_name();
        if name.to_string_lossy().starts_with('.') {
            continue; // .git and editor droppings
        }
        let Ok(meta) = entry.metadata() else { continue };
        if meta.is_dir() {
            walk(root, &path, out);
        } else if path.extension().is_some_and(|e| e == "js") {
            let relative = path
                .strip_prefix(root)
                .map(|p| p.to_string_lossy().replace('\\', "/"))
                .unwrap_or_default();
            let modified = meta
                .modified()
                .ok()
                .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
                .map(|d| d.as_secs())
                .unwrap_or(0);
            out.push(ScriptFile { path: relative, size: meta.len(), modified });
        }
    }
}

/// What a hook call produced, for the caller that runs it.
pub enum HookResult {
    /// The file does not exist. For an optional hook this means "do what the
    /// server does without it".
    Absent,
    Returned(engine::Success),
    Refused(engine::Failure),
}

/// Run `function` in the script `relative`, with the server's host bindings
/// acting as `user`.
///
/// Call this OUTSIDE [`Db::mutate`] — it may block on the network or a child
/// process, and the host's `plm` object takes the store lock itself.
pub fn run(db: &Db, user: &User, relative: &str, function: &str, input: &Value) -> Result<HookResult, Error> {
    let Some(source) = db.scripts().source(relative)? else {
        return Ok(HookResult::Absent);
    };
    Ok(run_source(db, user, &source, relative, function, input))
}

/// Run unsaved `source` — the editor's test run of text not yet written.
pub fn run_source(db: &Db, user: &User, source: &str, name: &str, function: &str, input: &Value) -> HookResult {
    let host = host::Host::new(db.clone(), user.clone());
    match engine::call(source, name, function, input, &|context, _| host.install(context)) {
        Ok(success) => HookResult::Returned(success),
        Err(failure) => {
            for line in &failure.logs {
                eprintln!("brep-plm: script {name}: {line}");
            }
            eprintln!("brep-plm: script {name}: {function}() refused: {}", failure.message);
            HookResult::Refused(failure)
        }
    }
}

/// The user as a script sees it: identity and groups, never the verifier.
pub fn user_json(user: &User) -> Value {
    serde_json::json!({
        "id": user.id,
        "username": user.username,
        "display_name": user.display_name,
        "email": user.email,
        "groups": user.groups,
    })
}

/// Read a hook's answer as a string: either the string itself or the named
/// field of a returned object (`"B"` or `{ label: "B" }`).
pub fn string_answer(value: &Value, field: &str) -> Option<String> {
    match value {
        Value::String(text) => Some(text.clone()),
        Value::Object(map) => map.get(field).and_then(|v| v.as_str()).map(str::to_string),
        _ => None,
    }
}
