//! The store: the metadata in SQLite (`plm.sqlite`, see [`crate::sql`]), one
//! file per CAD document, and the monotonic change sequence the CAD app's
//! mirror will key its cache on.
//!
//! # Memory, then disk, in one critical section
//!
//! The whole metadata is held in memory under one lock and every read is
//! answered from there. Every change goes through [`Db::mutate`]: under the
//! write lock, the change's closure checks its rules against the in-memory
//! state and edits it, and then — still under the lock — exactly what it
//! touched is written to SQLite in ONE transaction, together with the
//! change's audit events. If the closure refuses, or the transaction fails,
//! the in-memory edits are undone and nothing is on disk. So every rule that
//! "runs inside the lock" runs in the same critical section as the write that
//! depends on it, and the schema's own constraints (one number per part, one
//! label per revision within a part) refuse anything a rule missed.
//!
//! A change costs what it touched, not what the store holds: the part list
//! journals the parts a change touches ([`crate::table::Parts`]), and the
//! other state fields — users, sessions, the catalog, settings — are small.
//!
//! # Documents
//!
//! Write to `<path>.tmp`, then rename. A rename over an existing file is
//! atomic on every platform this runs on, so a process killed mid-write leaves
//! either the old file or the new one and never a truncated one.
//!
//! # The sequence
//!
//! `seq` increments on every mutation, whoever made it, and every mutating
//! response carries it. That is the `mutation_generation()` the CAD app's
//! `ModelStore` requires — a REQUIRED trait method precisely so a backend
//! cannot compile while silently never invalidating anything. `changes(since)`
//! then answers "which keys moved", which is what drives a client's per-key
//! refresh instead of a re-hydrate.
//!
//! The change log is bounded ([`CHANGE_LOG_LIMIT`]). A client that asks from
//! further back than the log reaches is told `stale`, and its correct response
//! is to re-read the index rather than to trust a short answer.
//!
//! # From the file store
//!
//! A data directory written before SQLite holds `plm.json` and `audit.jsonl`.
//! [`Db::open`] imports both in one transaction into a new `plm.sqlite`, then
//! renames them `*.migrated-<unix time>` ([`migrate_files`]).

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::{Arc, RwLock};

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::audit::{self, AuditFilter, AuditSink};
use crate::auth;
use crate::bom;
use crate::catalog;
use crate::lifecycle;
use crate::model::{
    groups, ApiToken, AttributeDef, AuditEvent, EntityRef, Category, Company, DocumentClass, Lifecycle, Lock, NumberMode, Part, PartType, Revision, Session, Settings,
    Timestamp, Use, User,
};
use crate::scripting::{self, HookResult, Scripts};
use crate::security::{Security, ServerConfig};
use crate::sourcing;
use crate::sql::{self, Durability, Sql};
use crate::table::Parts;
use crate::Error;

/// How many recent key changes the log keeps. Past this a client is told to
/// re-index rather than given a partial answer.
pub const CHANGE_LOG_LIMIT: usize = 4096;

/// One key that changed, and the sequence it changed at.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Change {
    pub seq: u64,
    pub key: String,
}

/// Everything the server holds in memory, and exactly what the metadata file
/// contains.
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct State {
    #[serde(default)]
    pub seq: u64,
    #[serde(default)]
    pub users: Vec<User>,
    /// Browser sessions, journaled ([`crate::journal`]): one is added per
    /// sign-in, so this grows with use, and a list outside a journal costs
    /// every write O(its length).
    #[serde(default)]
    pub sessions: crate::journal::Journaled<Session>,
    /// Bearer tokens for callers that are not a browser ([`ApiToken`]).
    #[serde(default)]
    pub api_tokens: Vec<ApiToken>,
    #[serde(default)]
    pub part_types: Vec<PartType>,
    #[serde(default)]
    pub parts: Parts,
    /// The catalog tree, flat: each category names its parent.
    #[serde(default)]
    pub categories: Vec<Category>,
    /// Who makes parts ([`Company`]); a manufacturer part names one.
    #[serde(default)]
    pub manufacturers: Vec<Company>,
    /// Who sells parts ([`Company`]); a supplier offer names one.
    #[serde(default)]
    pub suppliers: Vec<Company>,
    /// The administrator's policy choices ([`Settings`]).
    #[serde(default)]
    pub settings: Settings,
    #[serde(default)]
    pub changes: Vec<Change>,
    /// The lowest sequence the change log can still answer for completely.
    /// It moves only when [`CHANGE_LOG_LIMIT`] forces entries out, so a
    /// client is told `stale` when the log has actually DROPPED something it
    /// needed — not merely because nothing happened at the sequence it asked
    /// from, which is the ordinary case for a client that has been idle.
    #[serde(default)]
    pub changes_from: u64,
    /// Change orders, oldest first ([`crate::model::ChangeOrder`]).
    #[serde(default)]
    pub change_orders: Vec<crate::model::ChangeOrder>,
    /// How change orders are numbered: a part type's numbering, with its own
    /// counter ([`crate::model::default_eco_numbering`]).
    #[serde(default)]
    pub eco_numbering: crate::model::EcoNumbering,
    /// Every user's workspace entries ([`crate::model::WorkspaceEntry`]),
    /// in creation order.
    #[serde(default)]
    pub workspace: crate::workspace::Workspace,
}

impl State {
    /// Record that every revision of `part_id` moved: a lock, a lifecycle or
    /// a supersede changes the store index's rows for them (`locked_by`,
    /// `lifecycle`, `editable`) without writing a document, and the change
    /// feed has to name them so a client can re-read exactly those rows
    /// (`GET /api/store/index?keys=`) instead of the whole index.
    pub(crate) fn touch_part(&mut self, part_id: &str) {
        let keys: Vec<String> = self
            .part(part_id)
            .map(|part| part.revisions.iter().map(|r| r.document_key(&part.id)).collect())
            .unwrap_or_default();
        for key in keys {
            self.touch(key);
        }
    }

    /// Record that `key`'s document moved at the current sequence. Called
    /// inside a mutation, AFTER [`Db::mutate`] has bumped `seq`.
    pub(crate) fn touch(&mut self, key: String) {
        let seq = self.seq;
        self.changes.retain(|change| change.key != key);
        self.changes.push(Change { seq, key });
        if self.changes.len() > CHANGE_LOG_LIMIT {
            let excess = self.changes.len() - CHANGE_LOG_LIMIT;
            let dropped: Vec<Change> = self.changes.drain(..excess).collect();
            if let Some(last) = dropped.last() {
                self.changes_from = self.changes_from.max(last.seq);
            }
        }
    }

    pub fn user(&self, id: &str) -> Option<&User> {
        self.users.iter().find(|u| u.id == id)
    }

    pub fn user_by_name(&self, username: &str) -> Option<&User> {
        let username = username.trim().to_ascii_lowercase();
        self.users
            .iter()
            .find(|u| u.username.to_ascii_lowercase() == username)
    }

    pub fn part(&self, id: &str) -> Option<&Part> {
        self.parts.get(id)
    }

    pub fn part_type(&self, id: &str) -> Option<&PartType> {
        self.part_types.iter().find(|t| t.id == id)
    }

    /// A part by its id, or else by its number.
    pub fn part_by_id_or_number(&self, key: &str) -> Option<&Part> {
        let key = key.trim();
        self.part(key).or_else(|| self.part_with_number(key))
    }

    /// The part that already carries `number`, compared without regard to
    /// ASCII case. A part number is unique across ALL parts, whatever type
    /// made it: it is what an ERP, a BOM and a person reading a drawing
    /// use to mean exactly one part.
    pub fn part_with_number(&self, number: &str) -> Option<&Part> {
        let number = number.trim();
        self.parts.iter().find(|p| p.number.eq_ignore_ascii_case(number))
    }

    /// The oldest sequence the change log can still answer for completely.
    /// A client asking from before this is told `stale` and must re-index.
    pub fn changes_floor(&self) -> u64 {
        self.changes_from
    }
}

/// The server's whole persistent state: the metadata under one lock (and in
/// SQLite), the document tree beside it, and the administrator's scripts.
///
/// A `Db` is a HANDLE: cloning it is cheap and every clone is the same store.
/// That is what lets a script's `plm.*` bindings hold the store for the length
/// of one hook call.
#[derive(Clone)]
pub struct Db {
    root: PathBuf,
    state: Arc<RwLock<State>>,
    scripts: Arc<Scripts>,
    security: Arc<Security>,
    /// The metadata on disk, and the audit log and search index with it.
    sql: Arc<Sql>,
    /// This process's hold on the data directory ([`crate::dirlock`]), for as
    /// long as any handle to the store is alive.
    _lock: Arc<crate::dirlock::DirLock>,
    /// The last backup this process took, and whether one is running.
    backups: Arc<crate::backup::Status>,
}

/// Whole seconds since the epoch.
pub fn now() -> Timestamp {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

impl Db {
    /// Open (or create) the store at `root`.
    ///
    /// Returns the generated administrator password when — and only when —
    /// this call seeded the first account. It is printed once at boot and
    /// never recoverable, which is the point: no build of this server ships a
    /// known password.
    pub fn open(root: impl Into<PathBuf>) -> io::Result<(Self, Option<String>)> {
        Self::open_with_scripts(root, None)
    }

    /// [`Db::open`], with the scripts directory somewhere other than
    /// `<root>/scripts` — typically a git checkout the administrator manages.
    /// This is the path `serve` opens with, always at [`Durability::Full`].
    pub fn open_with_scripts(
        root: impl Into<PathBuf>,
        scripts: Option<PathBuf>,
    ) -> io::Result<(Self, Option<String>)> {
        Self::open_with(root, scripts, Durability::Full)
    }

    /// [`Db::open_with_scripts`] at a chosen [`Durability`]. The test suites
    /// open with [`Durability::Normal`]; the server never does.
    pub fn open_with(
        root: impl Into<PathBuf>,
        scripts: Option<PathBuf>,
        durability: Durability,
    ) -> io::Result<(Self, Option<String>)> {
        let root = root.into();
        // First, before anything in the directory is read or moved: two
        // servers on one directory would each hold their own copy of the
        // metadata and overwrite each other's changes.
        let lock = crate::dirlock::DirLock::acquire(&root)?;
        fs::create_dir_all(root.join("docs"))?;
        let scripts = Scripts::new(scripts.unwrap_or_else(|| root.join("scripts")))?;
        if let Some(report) = migrate_files(&root, false)? {
            eprintln!("brep-plm: {report}");
        }
        let sql = Sql::open_with(root.join(SQLITE_FILE), durability).map_err(io::Error::other)?;
        let loaded = sql.load().map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        let fresh = loaded.is_none();
        let mut state = loaded.unwrap_or_default();
        let before = sql::small_value(&mut state);

        let mut seeded = None;
        if state.part_types.is_empty() {
            // The DEFAULT, not the rule: prefix and width are data from here
            // on, and an operator adds more types through the API.
            state.part_types.push(PartType {
                id: "component".into(),
                name: "Component".into(),
                prefix: "CPART".into(),
                digits: 9,
                next: 1,
                created_at: now(),
                mode: Default::default(),
            });
        }
        if state.users.is_empty() {
            let password = auth::random_token(12);
            state.users.push(User {
                id: auth::new_id(),
                username: "admin".into(),
                display_name: "Administrator".into(),
                email: String::new(),
                password: auth::hash_password(&password),
                groups: vec![groups::ADMIN.into()],
                active: true,
                created_at: now(),
            });
            seeded = Some(password);
        }
        let after = sql::small_value(&mut state);
        if fresh || before != after {
            let change = sql::Change {
                before: &if fresh { Value::Object(Default::default()) } else { before },
                after: &after,
                parts: Vec::new(),
                categories: &state.categories,
                reindex: Vec::new(),
                all_parts: &state.parts,
                workspace: Vec::new(),
                sessions: Vec::new(),
            };
            sql.write(&change, Vec::new()).map_err(io::Error::other)?;
        }

        let db = Db {
            root,
            state: Arc::new(RwLock::new(state)),
            scripts: Arc::new(scripts),
            security: Arc::new(Security::default()),
            sql: Arc::new(sql),
            _lock: Arc::new(lock),
            backups: Arc::new(crate::backup::Status::default()),
        };
        // What a crash between an upload's move and its commit can leave:
        // nothing else is running yet, and the directory is ours alone.
        match db.sweep_blobs() {
            Ok(0) => {}
            Ok(n) => eprintln!("brep-plm: removed {n} unreferenced attachment file(s)"),
            Err(error) => eprintln!("brep-plm: could not sweep attachment files: {error}"),
        }
        Ok((db, seeded))
    }

    /// The administrator's scripts directory.
    /// The data directory.
    pub fn root(&self) -> &Path {
        &self.root
    }

    pub fn scripts(&self) -> &Scripts {
        &self.scripts
    }

    /// This store with the network-facing configuration `config`. Called once,
    /// at start-up, before the store is shared; the throttle starts empty.
    pub fn with_config(mut self, config: ServerConfig) -> Self {
        self.security = Arc::new(Security { config, ..Security::default() });
        self
    }

    /// The last backup this process took.
    pub fn backups(&self) -> &crate::backup::Status {
        &self.backups
    }

    /// The hardening layer: configuration and the sign-in throttle.
    pub fn security(&self) -> &Security {
        &self.security
    }

    /// Whether administrators may edit and test-run scripts in the browser:
    /// the setting, unless `--lock-script-editor` forces it off.
    pub fn script_editor_allowed(&self) -> bool {
        !self.security.config.lock_script_editor && self.read(|state| state.settings.script_editor_enabled)
    }

    /// Read the metadata under the shared lock.
    pub fn read<R>(&self, f: impl FnOnce(&State) -> R) -> R {
        let state = self.state.read().unwrap_or_else(|e| e.into_inner());
        f(&state)
    }

    /// Mutate the metadata under the exclusive lock, bump the sequence, and
    /// PERSIST before returning.
    ///
    /// Persisting inside the guard is what makes number allocation safe: two
    /// concurrent part creations serialize on this lock, and neither response
    /// is sent until the number it handed out is committed. A closure that
    /// returns `Err` leaves the state untouched: the parts it touched are put
    /// back from the journal ([`Parts`]) and every other field from the copy
    /// taken before it ran. The same happens when the write to disk fails —
    /// a UNIQUE constraint the closure's own check missed, a full disk.
    ///
    /// The change's audit events ([`crate::audit`]) are computed from exactly
    /// what it touched and committed in the SAME transaction as the change, so
    /// a change is never on disk without its events, nor an event without its
    /// change. Every write path goes through here, so every change is logged
    /// without any of them asking.
    pub fn mutate<R>(&self, f: impl FnOnce(&mut State) -> Result<R, Error>) -> Result<R, Error> {
        let mut state = self.state.write().unwrap_or_else(|e| e.into_inner());
        let before = sql::small_value(&mut state);
        state.parts.begin();
        state.workspace.begin();
        state.sessions.begin();
        state.seq += 1;
        match f(&mut state) {
            Ok(value) => {
                let touched = state.parts.commit();
                let entries = state.workspace.commit();
                let sessions = state.sessions.commit();
                match self.persist(&mut state, &before, &touched, &entries, &sessions) {
                    Ok(()) => Ok(value),
                    Err(error) => {
                        state.parts.undo(touched);
                        state.workspace.undo(entries);
                        state.sessions.undo(sessions);
                        restore_small(&mut state, &before)?;
                        Err(error)
                    }
                }
            }
            Err(error) => {
                state.parts.rollback();
                state.workspace.rollback();
                state.sessions.rollback();
                restore_small(&mut state, &before)?;
                Err(error)
            }
        }
    }

    /// Write one committed change and its audit events in one transaction.
    fn persist(
        &self,
        state: &mut State,
        before: &Value,
        touched: &[crate::table::Touched],
        entries: &[crate::journal::Touched<crate::model::WorkspaceEntry>],
        sessions: &[crate::journal::Touched<Session>],
    ) -> Result<(), Error> {
        let after = sql::small_value(state);
        // A part a change borrowed mutably but did not actually change is
        // neither written nor audited.
        let mut old_parts = Vec::new();
        let mut new_parts = Vec::new();
        let mut changed = Vec::new();
        for t in touched {
            let now_part = &state.parts[t.position];
            let now_value = serde_json::to_value(now_part).map_err(Error::internal)?;
            let was_value = match &t.before {
                Some(part) => Some(serde_json::to_value(part).map_err(Error::internal)?),
                None => None,
            };
            if was_value.as_ref() == Some(&now_value) {
                continue;
            }
            old_parts.extend(was_value);
            new_parts.push(now_value);
            changed.push((t.position, now_part));
        }
        // The journaled lists the same way: only what was touched, and only
        // what actually changed, is written and audited.
        let (old_entries, new_entries, workspace) = journal_rows(&state.workspace, entries)?;
        let (old_sessions, new_sessions, session_rows) = journal_rows(&state.sessions, sessions)?;
        let mut old = before.clone();
        let mut new = after.clone();
        if let (Some(o), Some(n)) = (old.as_object_mut(), new.as_object_mut()) {
            o.insert("parts".into(), Value::Array(old_parts));
            n.insert("parts".into(), Value::Array(new_parts));
            o.insert("workspace".into(), Value::Array(old_entries));
            n.insert("workspace".into(), Value::Array(new_entries));
            o.insert("sessions".into(), Value::Array(old_sessions));
            n.insert("sessions".into(), Value::Array(new_sessions));
        }
        let events = audit::refine(audit::diff(&old, &new, &audit::current_actor(), now(), state.seq));
        // A part's search row holds its category's PATH ("Fasteners /
        // Screws"): re-index the parts whose category's path the change moved
        // — a rename, a move, a category created for text a part already had.
        let mut reindex = Vec::new();
        if before.get("categories") != after.get("categories") {
            let old_categories: Vec<Category> =
                serde_json::from_value(before.get("categories").cloned().unwrap_or_default()).unwrap_or_default();
            let moved: BTreeSet<String> = old_categories
                .iter()
                .chain(state.categories.iter())
                .map(|c| c.id.to_ascii_lowercase())
                .filter(|id| catalog::path_name(&old_categories, id) != catalog::path_name(&state.categories, id))
                .collect();
            reindex = state
                .parts
                .iter()
                .enumerate()
                .filter(|(_, p)| moved.contains(&p.category.to_ascii_lowercase()))
                .map(|(i, _)| i)
                .collect();
        }
        let change = sql::Change {
            before,
            after: &after,
            parts: changed,
            categories: &state.categories,
            reindex,
            all_parts: &state.parts,
            workspace,
            sessions: session_rows,
        };
        self.sql.write(&change, events).map(|_| ()).map_err(|error| {
            if sql::is_constraint(&error) {
                Error::conflict(format!("the store refused the change: {error}"))
            } else {
                Error::internal(error)
            }
        })
    }

    /// Record an event that is not a change to the store: a failed sign-in, a
    /// script file edited or run in the browser.
    pub fn record(&self, action: &str, entity: EntityRef, detail: impl Into<String>) {
        let event = AuditEvent {
            id: 0,
            at: now(),
            actor: audit::current_actor(),
            action: action.to_string(),
            entity,
            changes: BTreeMap::new(),
            detail: detail.into(),
            seq: 0,
        };
        if let Err(error) = self.sql.append(vec![event]) {
            eprintln!("brep-plm: audit: could not append to the log: {error}");
        }
    }

    /// Audit events matching `filter`, newest first.
    pub fn audit_query(&self, filter: &AuditFilter) -> Result<Vec<AuditEvent>, Error> {
        self.sql.query(filter).map_err(Error::internal)
    }

    /// The SQLite store, for the search index and diagnostics.
    pub fn sql(&self) -> &Sql {
        &self.sql
    }

    // -- documents ---------------------------------------------------------

    pub(crate) fn document_path(&self, key: &str) -> Result<PathBuf, Error> {
        let mut path = self.root.join("docs");
        for segment in key.split('/') {
            if segment.is_empty()
                || segment == "."
                || segment == ".."
                || !segment
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
            {
                return Err(Error::bad_request(format!(
                    "'{key}' is not a usable document key"
                )));
            }
            path.push(segment);
        }
        Ok(path.with_extension("json"))
    }

    /// One document's text, or `None` when it has never been written.
    pub fn read_document(&self, key: &str) -> Result<Option<String>, Error> {
        let path = self.document_path(key)?;
        match fs::read_to_string(path) {
            Ok(text) => Ok(Some(text)),
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(None),
            Err(error) => Err(Error::internal(error)),
        }
    }

    /// Write a document and record its hash on the revision.
    ///
    /// The editable check is HERE, not only in the API layer: this is the one
    /// function that can change a revision's bytes, so it is the one that has
    /// to refuse a released revision. A guard that lives only in the handler
    /// is a guard that the next caller forgets.
    ///
    /// Everything happens inside one [`Db::mutate`], which is what makes the
    /// check race-free — a release landing between a separate check and the
    /// write would otherwise rewrite released bytes. Within the guard the file
    /// still lands BEFORE the metadata that describes it, so a crash between
    /// the two leaves a document whose hash is not yet recorded, rather than a
    /// recorded hash for bytes that do not exist.
    pub fn write_document(&self, part_id: &str, revision_id: &str, body: &str) -> Result<u64, Error> {
        let key = format!("part/{part_id}/rev/{revision_id}");
        let path = self.document_path(&key)?;
        let hash = auth::content_hash(body);
        let size = body.len() as u64;
        self.mutate(move |state| {
            let part = state.parts.get_mut(&part_id)
                .ok_or_else(|| Error::not_found("part"))?;
            let number = part.number.clone();
            let revision = part
                .revision_mut(revision_id)
                .ok_or_else(|| Error::not_found("revision"))?;
            if !revision.lifecycle.is_editable() {
                return Err(Error::conflict(format!(
                    "{number} revision {} is {} and cannot be written — start a new revision",
                    revision.label,
                    revision.lifecycle.as_str()
                )));
            }
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent).map_err(Error::internal)?;
            }
            write_atomic(&path, body).map_err(Error::internal)?;
            revision.content_hash = hash;
            revision.size = size;
            revision.modified_at = now();
            // A document someone SAVED supersedes a bake still waiting: the
            // CAD app evaluated what it saved. A worker that had the job is
            // refused when it reports back (see `Db::finish_bake`).
            if revision.needs_bake() {
                if let Some(bake) = revision.bake.as_mut() {
                    bake.status = crate::model::BakeStatus::Done;
                    bake.error = "superseded by a saved document".into();
                    bake.finished_by = None;
                    bake.finished_at = Some(now());
                }
            }
            // Approvals given on the document before this save go stale (D8).
            crate::review::document_changed(state, part_id, revision_id);
            state.touch(key);
            Ok(state.seq)
        })
    }

    /// Remove a document's bytes. The revision record stays — a revision with
    /// no document is a legitimate state (a part imported with no 3D model).
    pub fn remove_document(&self, key: &str) -> Result<(), Error> {
        let path = self.document_path(key)?;
        match fs::remove_file(path) {
            Ok(()) => Ok(()),
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
            Err(error) => Err(Error::internal(error)),
        }
    }
}

/// The metadata file in a data directory.
pub const SQLITE_FILE: &str = "plm.sqlite";

/// Put every field but the parts back as `before` had it.
/// Whether `session` has ended by the settings' limits at `now`: idle for
/// longer than `session_idle_minutes`, or signed in longer ago than
/// `session_max_hours` (0: no such limit).
pub fn session_expired(settings: &Settings, session: &Session, now: Timestamp) -> bool {
    let idle = now.saturating_sub(session.last_seen) > settings.session_idle_minutes.saturating_mul(60);
    let old = settings.session_max_hours > 0
        && now.saturating_sub(session.created_at) > settings.session_max_hours.saturating_mul(3600);
    idle || old
}

impl Db {
    /// Remove every expired session ([`session_expired`]), as the system;
    /// how many went.
    ///
    /// A session used to be removed only by signing out, or when its cookie
    /// came back after it expired. A browser that simply stops sending the
    /// cookie — it expired there, the user cleared it, a private window
    /// closed — left its session in the store for good, and every sign-in
    /// adds one: the list grew with use and nothing bounded it. This runs at
    /// every sign-in — the only way the list grows — so it holds the sessions
    /// that could still be used, plus whatever expired since the last
    /// sign-in. Not at start-up: opening a store changes nothing in it (an
    /// import or a restore opens exactly what it was given). Nothing is
    /// written when nothing has expired.
    pub fn prune_sessions(&self) -> Result<usize, Error> {
        let at = now();
        let expired = self.read(|state| state.sessions.iter().filter(|s| session_expired(&state.settings, s, at)).count());
        if expired == 0 {
            return Ok(0);
        }
        audit::as_actor(crate::model::Actor::system(), || {
            self.mutate(move |state| {
                let settings = state.settings.clone();
                let before = state.sessions.len();
                state.sessions.retain(|s| !session_expired(&settings, s, at));
                Ok(before - state.sessions.len())
            })
        })
    }
}

/// A journaled list's change as the audit and the store want it: the touched
/// records before and after (only those that changed), and each changed key
/// with what it is now (`None`: removed).
#[allow(clippy::type_complexity)]
fn journal_rows<'a, T>(
    list: &'a crate::journal::Journaled<T>,
    touched: &'a [crate::journal::Touched<T>],
) -> Result<(Vec<Value>, Vec<Value>, Vec<(String, Option<&'a T>)>), Error>
where
    T: crate::journal::Keyed + Clone + PartialEq + Serialize,
{
    let mut old = Vec::new();
    let mut new = Vec::new();
    let mut rows = Vec::new();
    for (t, now) in list.changed(touched) {
        old.extend(t.before.as_ref().map(serde_json::to_value).transpose().map_err(Error::internal)?);
        new.extend(now.map(serde_json::to_value).transpose().map_err(Error::internal)?);
        rows.push((t.key.clone(), now));
    }
    Ok((old, new, rows))
}

fn restore_small(state: &mut State, before: &Value) -> Result<(), Error> {
    let parts = std::mem::take(&mut state.parts);
    let workspace = std::mem::take(&mut state.workspace);
    let sessions = std::mem::take(&mut state.sessions);
    *state = serde_json::from_value(before.clone()).map_err(Error::internal)?;
    state.parts = parts;
    state.workspace = workspace;
    state.sessions = sessions;
    Ok(())
}

/// Import a file-store data directory (`plm.json`, and `audit.jsonl` if it is
/// there) into a new `plm.sqlite`, and say what was imported.
///
/// Nothing happens when `plm.sqlite` already exists or there is no
/// `plm.json`. The import is built in `plm.sqlite.migrating` in ONE
/// transaction, and only a complete file is renamed into place; a crash
/// before that leaves the old files untouched and the next start does it
/// again. Afterwards the old files are renamed `<name>.migrated-<unix time>`,
/// never deleted. A crash between those renames leaves a `plm.json` beside a
/// finished `plm.sqlite`, which the next start moves aside.
///
/// With `dry_run` the import is built in a scratch file beside the data (not
/// in it), counted, and deleted: the data directory is not changed.
pub fn migrate_files(root: &Path, dry_run: bool) -> io::Result<Option<String>> {
    let json = root.join("plm.json");
    let audit_file = root.join("audit.jsonl");
    let sqlite = root.join(SQLITE_FILE);
    let stamp = now();
    if sqlite.exists() {
        if json.exists() && !dry_run {
            let migrated = Sql::open(&sqlite).and_then(|sql| sql.meta("migrated_from")).map_err(io::Error::other)?;
            if migrated.is_some() {
                fs::rename(&json, root.join(format!("plm.json.migrated-{stamp}")))?;
                if audit_file.exists() {
                    fs::rename(&audit_file, root.join(format!("audit.jsonl.migrated-{stamp}")))?;
                }
                return Ok(Some("moved aside a plm.json left by an interrupted migration".into()));
            }
        }
        return Ok(None);
    }
    if !json.exists() {
        return Ok(None);
    }
    let text = fs::read_to_string(&json)?;
    let mut state: State = serde_json::from_str(&text)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("{json:?}: {e}")))?;
    let mut events = Vec::new();
    let mut skipped = 0usize;
    if audit_file.exists() {
        use std::io::BufRead;
        for line in io::BufReader::new(fs::File::open(&audit_file)?).lines() {
            let line = line?;
            if line.trim().is_empty() {
                continue;
            }
            match serde_json::from_str::<AuditEvent>(&line) {
                Ok(event) => events.push(event),
                // A torn last line from a crash: the file store skipped it
                // on every read, and so does the import.
                Err(_) => skipped += 1,
            }
        }
    }
    let building = if dry_run {
        std::env::temp_dir().join(format!("brep-plm-dry-run-{}-{stamp}.sqlite", std::process::id()))
    } else {
        root.join("plm.sqlite.migrating")
    };
    let remove_building = |path: &Path| {
        for suffix in ["", "-wal", "-shm"] {
            let _ = fs::remove_file(format!("{}{suffix}", path.display()));
        }
    };
    remove_building(&building);
    let event_count = events.len();
    let counts = {
        let sql = Sql::open(&building).map_err(io::Error::other)?;
        sql.import(&mut state, events).map_err(io::Error::other)?;
        sql.set_meta(
            "migrated_from",
            &json!({ "at": stamp, "plm_json_bytes": text.len(), "audit_events": event_count, "audit_lines_skipped": skipped })
                .to_string(),
        )
        .map_err(io::Error::other)?;
        sql.checkpoint().map_err(io::Error::other)?;
        sql.counts().map_err(io::Error::other)?
    };
    let summary: Vec<String> = counts.iter().map(|(table, n)| format!("{table} {n}")).collect();
    let mut report = format!(
        "{} plm.json{} into {}: {}",
        if dry_run { "dry run: would import" } else { "imported" },
        if audit_file.exists() { " and audit.jsonl" } else { "" },
        SQLITE_FILE,
        summary.join(", ")
    );
    if skipped > 0 {
        report.push_str(&format!(" ({skipped} unreadable audit line(s) skipped)"));
    }
    if dry_run {
        remove_building(&building);
        return Ok(Some(report));
    }
    for suffix in ["-wal", "-shm"] {
        let _ = fs::remove_file(format!("{}{suffix}", building.display()));
    }
    fs::rename(&building, &sqlite)?;
    fs::rename(&json, root.join(format!("plm.json.migrated-{stamp}")))?;
    if audit_file.exists() {
        fs::rename(&audit_file, root.join(format!("audit.jsonl.migrated-{stamp}")))?;
    }
    report.push_str(&format!("; the old files are now *.migrated-{stamp}"));
    Ok(Some(report))
}

/// Write `body` to `path` through a temporary file and a rename, so a reader
/// never sees a partial file.
pub(crate) fn write_atomic(path: &Path, body: &str) -> io::Result<()> {
    let temp = path.with_extension("tmp");
    fs::write(&temp, body)?;
    fs::rename(&temp, path)
}

// ===========================================================================
// Operations — the rules that are about MORE than one record
// ===========================================================================

impl Db {
    /// Create a part of the default shape: the type decides the number, the
    /// document is normal, the first revision takes the suggested label.
    pub fn create_part(
        &self,
        author: &User,
        part_type_id: &str,
        name: &str,
        description: &str,
        category: &str,
    ) -> Result<Part, Error> {
        self.create_part_with(
            author,
            &PartSpec {
                part_type: part_type_id.to_string(),
                name: name.to_string(),
                description: description.to_string(),
                category: category.to_string(),
                ..PartSpec::default()
            },
        )
    }

    /// Create a part with its first draft revision.
    ///
    /// The number comes from the type's [`NumberMode`]. A Script-mode type and
    /// the revision-label hook are asked OUTSIDE the store lock, on a snapshot;
    /// then one [`Db::mutate`] re-checks everything their answers depend on —
    /// that the type's mode did not change meanwhile, and that the number and
    /// label are still free. Two concurrent creates that a script gave the
    /// same number therefore end as one part and one refusal, never two parts.
    pub fn create_part_with(&self, author: &User, spec: &PartSpec) -> Result<Part, Error> {
        check_origin(spec.origin)?;
        let external_ref = spec.external_ref.trim().to_string();
        let origin = spec.origin;
        let name = spec.name.trim().to_string();
        if name.is_empty() {
            return Err(Error::bad_request("a part needs a name"));
        }
        let kind = self
            .read(|state| state.part_type(&spec.part_type).cloned())
            .ok_or_else(|| Error::bad_request(format!("no part type '{}'", spec.part_type)))?;
        let typed = spec.number.trim().to_string();
        let tags = catalog::tidy_tags(spec.tags.iter().cloned());
        let part_view = json!({
            "name": name,
            "description": spec.description.trim(),
            "category": spec.category.trim(),
            "tags": tags,
            "attributes": spec.attributes,
            "document_class": spec.document_class,
        });

        let given: Option<String> = match &kind.mode {
            NumberMode::Counter => {
                if !typed.is_empty() {
                    return Err(Error::bad_request(format!(
                        "part type '{}' numbers by its counter — leave the number empty",
                        kind.id
                    )));
                }
                None
            }
            NumberMode::Free => {
                if typed.is_empty() {
                    return Err(Error::bad_request(format!(
                        "part type '{}' takes the number you type — enter one",
                        kind.id
                    )));
                }
                Some(typed)
            }
            NumberMode::Pattern { regex } => {
                let pattern = full_match(regex)?;
                if typed.is_empty() {
                    return Err(Error::bad_request(format!(
                        "part type '{}' takes a typed number matching {regex} — enter one",
                        kind.id
                    )));
                }
                if !pattern.is_match(&typed) {
                    return Err(Error::bad_request(format!(
                        "'{typed}' does not match part type '{}' ({regex})",
                        kind.id
                    )));
                }
                Some(typed)
            }
            NumberMode::Script { script } => {
                let input = json!({
                    "requested": typed,
                    "partType": kind,
                    "part": part_view,
                    "user": scripting::user_json(author),
                });
                let answer = match scripting::run(self, author, script, "partNumber", &input)? {
                    HookResult::Absent => {
                        return Err(Error::conflict(format!(
                            "part type '{}' numbers by the script '{script}', which is not in the scripts directory",
                            kind.id
                        )))
                    }
                    HookResult::Refused(failure) => {
                        return Err(Error::conflict(format!("{script}: {}", failure.message)))
                    }
                    HookResult::Returned(success) => success.value,
                };
                let number = scripting::string_answer(&answer, "number").ok_or_else(|| {
                    Error::conflict(format!(
                        "{script}: partNumber() must return the number, or {{ number }} — it returned {answer}"
                    ))
                })?;
                if spec.exact && !number.eq_ignore_ascii_case(&typed) {
                    return Err(Error::conflict(format!(
                        "{script} would number it {number}, not {typed} — a family row names its own number"
                    )));
                }
                Some(number)
            }
        };
        if let Some(number) = &given {
            check_number(number)?;
        }

        let requested_label = spec.label.trim().to_string();
        let label = self.decide_label(author, None, &part_view, &requested_label, "A")?;
        if spec.exact {
            exact_label(&requested_label, &label)?;
        }

        let author_id = author.id.clone();
        let document_class = spec.document_class;
        let description = spec.description.trim().to_string();
        let requested_category = spec.category.trim().to_string();
        let requested_values = spec.attributes.clone();
        let workspace = spec.workspace.clone();
        self.mutate(move |state| {
            // The category and the values are checked HERE, against the tree
            // as it is at the write — an admin may be editing it meanwhile.
            let category = known_category(&state.categories, &requested_category)?;
            let mut attributes = BTreeMap::new();
            catalog::apply_values(&state.categories, &category, &mut attributes, &requested_values)?;
            let current = state
                .part_types
                .iter_mut()
                .find(|t| t.id == kind.id)
                .ok_or_else(|| Error::bad_request(format!("no part type '{}'", kind.id)))?;
            if current.mode != kind.mode {
                return Err(Error::conflict(format!(
                    "part type '{}' changed its numbering while this part was being created — try again",
                    kind.id
                )));
            }
            let (number, sequence) = match given {
                Some(number) => (number, 0),
                None => {
                    // The counter skips numbers another type already gave out:
                    // a free-text or scripted number may land in its range.
                    let mut sequence = current.next;
                    let mut number = current.format_number(sequence);
                    while sequence <= current.capacity()
                        && state_has_number(&state.parts, &number)
                    {
                        sequence += 1;
                        number = current.format_number(sequence);
                    }
                    if sequence > current.capacity() {
                        return Err(Error::conflict(format!(
                            "part type '{}' is exhausted: {} digits cannot spell {}",
                            current.id, current.digits, sequence
                        )));
                    }
                    current.next = sequence + 1;
                    (number, sequence)
                }
            };
            if let Some(owner) = state.part_with_number(&number) {
                return Err(Error::conflict(format!(
                    "part number {number} already exists ({})",
                    owner.name
                )));
            }

            external_ref_clash(state, &kind.id, &external_ref, "")?;
            let stamp = now();
            let mut revision = Revision::draft(auth::new_id(), label, author_id.clone(), stamp);
            revision.origin = origin;
            let part = Part {
                id: auth::new_id(),
                number,
                part_type: kind.id.clone(),
                sequence,
                document_class,
                name,
                description,
                category,
                tags,
                external_ref,
                attributes,
                sourcing: Vec::new(),
                member_part_type: String::new(),
                attachments: Vec::new(),
                created_by: author_id,
                created_at: stamp,
                revisions: vec![revision],
            };
            state.parts.push(part.clone());
            state.touch(part.revisions[0].document_key(&part.id));
            if let Some(folder) = &workspace {
                crate::workspace::link_new_part(state, &part.created_by, folder, &part)?;
            }
            Ok(part)
        })
    }

    /// The label a new revision gets: the revision-label hook's answer when
    /// that script exists, else what the user typed, else `suggested`.
    fn decide_label(
        &self,
        user: &User,
        part: Option<&Part>,
        part_view: &Value,
        requested: &str,
        suggested: &str,
    ) -> Result<String, Error> {
        let existing: Vec<&str> = part
            .map(|p| p.revisions.iter().map(|r| r.label.as_str()).collect())
            .unwrap_or_default();
        let input = json!({
            "requested": requested,
            "suggested": suggested,
            "existing": existing,
            "part": part.map(|p| serde_json::to_value(p).unwrap_or(Value::Null)).unwrap_or_else(|| part_view.clone()),
            "user": scripting::user_json(user),
        });
        let label = match scripting::run(self, user, scripting::REVISION_LABEL, "revisionLabel", &input)? {
            HookResult::Absent => {
                if requested.is_empty() { suggested.to_string() } else { requested.to_string() }
            }
            HookResult::Refused(failure) => {
                return Err(Error::conflict(format!("{}: {}", scripting::REVISION_LABEL, failure.message)))
            }
            HookResult::Returned(success) => match &success.value {
                // Returning nothing means "no opinion".
                Value::Null => {
                    if requested.is_empty() { suggested.to_string() } else { requested.to_string() }
                }
                other => scripting::string_answer(other, "label").ok_or_else(|| {
                    Error::conflict(format!(
                        "{}: revisionLabel() must return the label, or {{ label }} — it returned {other}",
                        scripting::REVISION_LABEL
                    ))
                })?,
            },
        };
        check_label(&label)?;
        Ok(label)
    }

    /// Start a new revision of a part, labelled with the suggestion.
    pub fn create_revision(&self, author: &User, part_id: &str) -> Result<Revision, Error> {
        self.create_revision_labelled(author, part_id, "")
    }

    /// Start a new revision of a part.
    ///
    /// While another revision is in work this is refused only when the
    /// administrator has turned [`Settings::allow_multiple_open_drafts`] off;
    /// by default several revisions may be in work at once. The new draft
    /// inherits the document of the revision currently RELEASED — or, when
    /// none is, the newest revision — so revising a part starts from what the
    /// part currently is, never from somebody else's unfinished draft. That
    /// includes an imported part, which is revised rather than forked.
    ///
    /// `label` is free text; empty takes the suggestion. It must not repeat a
    /// label the part already has.
    pub fn create_revision_labelled(
        &self,
        author: &User,
        part_id: &str,
        label: &str,
    ) -> Result<Revision, Error> {
        self.create_revision_as(author, part_id, label, false, crate::model::Origin::Authored)
    }

    /// [`Db::create_revision_labelled`] with the new revision's origin: a
    /// library re-import makes an `imported` one, so the next re-import
    /// still knows the latest revision is its own.
    pub fn create_revision_from(
        &self,
        author: &User,
        part_id: &str,
        label: &str,
        origin: crate::model::Origin,
    ) -> Result<Revision, Error> {
        check_origin(origin)?;
        self.create_revision_as(author, part_id, label, false, origin)
    }

    /// [`Db::create_revision_labelled`]. With `exact`, the revision-label hook
    /// may refuse `label` but not rename it — what a family row needs, since
    /// the row names the revision it writes.
    pub(crate) fn create_revision_as(
        &self,
        author: &User,
        part_id: &str,
        label: &str,
        exact: bool,
        origin: crate::model::Origin,
    ) -> Result<Revision, Error> {
        let part = self
            .read(|state| state.part(part_id).cloned())
            .ok_or_else(|| Error::not_found("part"))?;
        let single = !self.read(|state| state.settings.allow_multiple_open_drafts);
        if let Some(open) = part.open_draft().filter(|_| single) {
            return Err(one_draft_refusal(open, &part));
        }
        let suggested = suggest_label(&part);
        let requested = label.trim();
        let label = self.decide_label(author, Some(&part), &Value::Null, requested, &suggested)?;
        if exact {
            exact_label(requested, &label)?;
        }

        let carried_revision = part.current_release().or(part.latest()).map(|r| r.id.clone());
        let carried = part.current_release().or(part.latest()).map(|r| r.document_key(part_id));
        let carried_body = match carried {
            Some(key) => self.read_document(&key)?,
            None => None,
        };
        let carried_hash = carried_body.as_deref().map(auth::content_hash);

        let author_id = author.id.clone();
        let revision = self.mutate(move |state| {
            // Again inside the lock: the setting, or the part, may have
            // changed since the look above.
            if !state.settings.allow_multiple_open_drafts {
                let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
                if let Some(open) = part.open_draft() {
                    return Err(one_draft_refusal(open, part));
                }
            }
            let part = state.parts.get_mut(&part_id)
                .ok_or_else(|| Error::not_found("part"))?;
            if let Some(clash) = part.revision_by_label(&label) {
                return Err(Error::conflict(format!(
                    "{} already has a revision labelled {}",
                    part.number, clash.label
                )));
            }
            let stamp = now();
            let mut revision = Revision::draft(auth::new_id(), label, author_id, stamp);
            revision.origin = origin;
            // The uses list travels with the document it describes.
            revision.uses = carried_revision
                .as_deref()
                .and_then(|id| part.revision(id))
                .map(|r| r.uses.clone())
                .unwrap_or_default();
            // So do its files: new references to the same blobs, so removing
            // one from the new revision never touches the old one.
            revision.attachments = carried_revision
                .as_deref()
                .and_then(|id| part.revision(id))
                .map(|r| {
                    r.attachments
                        .iter()
                        .map(|a| crate::model::Attachment { id: auth::new_id(), ..a.clone() })
                        .collect()
                })
                .unwrap_or_default();
            // And its picture, while it still pictures the carried document
            // (the same bytes are written below, so the hash still matches).
            revision.thumbnail = carried_revision
                .as_deref()
                .and_then(|id| part.revision(id))
                .and_then(|r| r.thumbnail.clone())
                .filter(|t| carried_hash.as_deref() == Some(t.content_hash.as_str()));
            part.revisions.push(revision.clone());
            state.touch(revision.document_key(part_id));
            Ok(revision)
        })?;

        if let Some(body) = carried_body {
            self.write_document(part_id, &revision.id, &body)?;
        }
        Ok(revision)
    }

    /// Change a part's descriptive fields — `name`, `description`,
    /// `category`, `tags`, `external_ref` and `attributes`. The number, the
    /// type and the document class are identity and never change here.
    ///
    /// `attributes` MERGES: each key given is set (type-checked against the
    /// part's category, the new one if `category` changes in the same call),
    /// and a `null` clears it. Keys not mentioned are untouched — so moving a
    /// part to another category keeps the values the new schema lacks.
    ///
    /// This is the write behind `PATCH /api/parts/:id` and a script's
    /// `plm.updatePart`.
    pub fn update_part(&self, key: &str, fields: &Value) -> Result<Part, Error> {
        let Some(fields) = fields.as_object() else {
            return Err(Error::bad_request("the fields to change must be an object"));
        };
        for name in fields.keys() {
            if !matches!(
                name.as_str(),
                "name" | "description" | "category" | "tags" | "external_ref" | "attributes" | "member_part_type"
            ) {
                return Err(Error::bad_request(format!(
                    "'{name}' cannot be changed — only name, description, category, tags, attributes, external_ref and member_part_type"
                )));
            }
        }
        let fields = fields.clone();
        let key = key.to_string();
        self.mutate(move |state| {
            let id = state
                .part_by_id_or_number(&key)
                .map(|p| p.id.clone())
                .ok_or_else(|| Error::not_found("part"))?;
            if let Some(value) = fields.get("external_ref") {
                let reference = value.as_str().map(str::trim).unwrap_or_default();
                let kind = state.part(&id).map(|p| p.part_type.clone()).unwrap_or_default();
                external_ref_clash(state, &kind, reference, &id)?;
            }
            // Borrow the parts and the tree separately: the part is written,
            // the tree is only read.
            let State { parts, categories, settings, part_types, .. } = &mut *state;
            let part = parts.get_mut(&id).expect("found above");
            if let Some(value) = fields.get("member_part_type") {
                let requested = value.as_str().map(str::trim).unwrap_or_default();
                if part.document_class == DocumentClass::Normal && !requested.is_empty() {
                    return Err(Error::bad_request(
                        "only a family or a template has a member part type",
                    ));
                }
                if !requested.is_empty() && !part_types.iter().any(|t| t.id == requested) {
                    return Err(Error::bad_request(format!("no part type '{requested}'")));
                }
                part.member_part_type = requested.to_string();
            }
            let before = (part.category.clone(), part.attributes.clone());
            let updated = apply_part_fields(categories, part, &fields)?;

            // Judged on what CHANGED, not on which fields were sent: the edit
            // dialog re-sends every field, and renaming a released part must
            // not trip a lock on values it left alone.
            if settings.lock_released_attributes
                && part.catalog_frozen()
                && before != (updated.category.clone(), updated.attributes.clone())
            {
                return Err(Error::conflict(format!(
                    "{} is released and its category and attributes are locked — start a new revision to change them",
                    part.number
                )));
            }
            Ok(updated)
        })
    }
}

/// A family row and a template spin-out name their number and revision
/// themselves: the revision-label hook may refuse the label, but a hook that
/// RENAMES it would leave the table naming a revision that does not exist.
fn exact_label(requested: &str, decided: &str) -> Result<(), Error> {
    if !requested.is_empty() && !decided.eq_ignore_ascii_case(requested) {
        return Err(Error::conflict(format!(
            "{} would label it {decided}, not {requested} — a family row names its own revision",
            scripting::REVISION_LABEL
        )));
    }
    Ok(())
}

/// A document still waiting for its bake cannot release: once released its
/// bytes are frozen, and the bake could never be written.
fn bake_gate(revision: &Revision) -> Result<(), Error> {
    match &revision.bake {
        Some(bake) if revision.needs_bake() => Err(Error::conflict(format!(
            "revision {} is waiting for a bake ({}{}) — it releases once a CAD worker has baked it, or once someone saves it from the CAD app",
            revision.label,
            bake.status.as_str(),
            if bake.error.is_empty() { String::new() } else { format!(": {}", bake.error) }
        ))),
        _ => Ok(()),
    }
}

/// The refusal [`Settings::allow_multiple_open_drafts`] = off gives.
fn one_draft_refusal(open: &Revision, part: &Part) -> Error {
    Error::conflict(format!(
        "revision {} of {} is still open — release or delete it first (this server allows one revision in work at a time)",
        open.label, part.number
    ))
}

impl Db {
    /// The administrator's settings as they stand.
    pub fn settings(&self) -> Settings {
        self.read(|state| state.settings.clone())
    }

    /// Change settings. `fields` MERGES: each key given is set, the rest are
    /// untouched. An unknown key or a value of the wrong type is refused
    /// rather than ignored, so a typo in an admin's request cannot silently
    /// leave a policy where it was.
    pub fn update_settings(&self, fields: &Value) -> Result<Settings, Error> {
        let Some(fields) = fields.as_object() else {
            return Err(Error::bad_request("the settings to change must be an object"));
        };
        let mut merged = serde_json::to_value(self.settings()).map_err(Error::internal)?;
        for (key, value) in fields {
            let slot = merged
                .get_mut(key)
                .ok_or_else(|| Error::bad_request(format!("there is no setting '{key}'")))?;
            if slot.is_boolean() && !value.is_boolean() {
                return Err(Error::bad_request(format!("'{key}' must be true or false")));
            }
            if slot.is_u64() {
                let minimum = if key == "session_idle_minutes" { 5 } else { 0 };
                match value.as_u64() {
                    Some(n) if n >= minimum => {}
                    _ => {
                        return Err(Error::bad_request(format!(
                            "'{key}' must be a whole number of at least {minimum}"
                        )))
                    }
                }
            }
            *slot = value.clone();
        }
        self.read(|state| crate::review::normalize_settings_json(state, &mut merged))?;
        let settings: Settings = serde_json::from_value(merged)
            .map_err(|e| Error::bad_request(format!("those settings do not read: {e}")))?;
        self.mutate(move |state| {
            // The review rules name users, groups, part types and categories:
            // checked against the store as it is at the write.
            let mut settings = settings;
            crate::review::check_settings(state, &mut settings)?;
            state.settings = settings.clone();
            Ok(settings)
        })
    }
}

/// The body of [`Db::update_part`], inside its lock. `categories` is the tree
/// as it is at the write.
fn apply_part_fields(
    categories: &[Category],
    part: &mut Part,
    fields: &serde_json::Map<String, Value>,
) -> Result<Part, Error> {
    let text = |value: &Value| value.as_str().map(|s| s.trim().to_string());
    if let Some(value) = fields.get("name") {
        let name = text(value)
            .filter(|n| !n.is_empty())
            .ok_or_else(|| Error::bad_request("a part needs a name"))?;
        part.name = name;
    }
    if let Some(value) = fields.get("description") {
        part.description = text(value).unwrap_or_default();
    }
    if let Some(value) = fields.get("category") {
        let requested = text(value).unwrap_or_default();
        // Re-sending the text a part already carries is not a change, so a
        // category written before the catalog existed survives an edit of the
        // part's name.
        if requested != part.category {
            part.category = known_category(categories, &requested)?;
        }
    }
    if let Some(value) = fields.get("external_ref") {
        part.external_ref = text(value).unwrap_or_default();
    }
    if let Some(value) = fields.get("tags") {
        part.tags = catalog::tidy_tags(
            value
                .as_array()
                .ok_or_else(|| Error::bad_request("tags must be a list of strings"))?
                .iter()
                .filter_map(|t| t.as_str().map(str::to_string)),
        );
    }
    // After the category, so a move and the new category's values can land in
    // one call.
    if let Some(value) = fields.get("attributes") {
        let changes = value
            .as_object()
            .ok_or_else(|| Error::bad_request("attributes must be an object of key: value"))?;
        catalog::apply_values(categories, &part.category, &mut part.attributes, changes)?;
    }
    Ok(part.clone())
}

/// The category id to store for `requested`: empty for none, else the id of
/// an existing category (the text may differ in case). A category that does
/// not exist is refused rather than stored as free text, so a typo cannot
/// quietly file a part nowhere.
fn known_category(categories: &[Category], requested: &str) -> Result<String, Error> {
    let requested = requested.trim();
    if requested.is_empty() {
        return Ok(String::new());
    }
    catalog::find(categories, requested)
        .map(|c| c.id.clone())
        .ok_or_else(|| Error::bad_request(format!("there is no category '{requested}'")))
}

/// What a parts listing is narrowed to. Every field is optional; empty means
/// "no constraint".
#[derive(Debug, Clone, Default)]
pub struct PartFilter {
    /// Text matched against the number, name, description, category name,
    /// tags, attribute values, and every MPN and SPN.
    pub q: String,
    /// A category id: the parts in it AND in every category beneath it.
    /// [`UNCATEGORIZED`] selects the parts in no known category.
    pub category: String,
    /// A tag, matched whole and without regard to case.
    pub tag: String,
    /// A manufacturer, by id or name: the parts with a manufacturer part
    /// from it.
    pub manufacturer: String,
    /// A supplier, by id or name: the parts with an offer from it.
    pub supplier: String,
    /// Catalog attribute conditions, all of which a part must meet
    /// ([`AttributeFilter`]).
    pub attributes: Vec<AttributeFilter>,
    /// An external reference, matched exactly (a library import's lookup).
    pub external_ref: String,
    /// A part type id, matched exactly.
    pub part_type: String,
}

/// One attribute condition of a parts listing (`attr.<key>=…`,
/// `attr.<key>.min=…`, `attr.<key>.max=…`), as the caller typed it. It is
/// read against the attribute's type in the catalog ([`attribute_matchers`]).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AttributeFilter {
    pub key: String,
    /// The part's value must equal one of these (any of); empty: any value.
    pub values: Vec<String>,
    /// Inclusive bounds, for a number attribute.
    pub min: Option<String>,
    pub max: Option<String>,
}

impl AttributeFilter {
    /// The conditions in a query string's `attr.*` pairs, merged per key in
    /// the order they first appear. Anything after `attr.<key>` other than
    /// `.min` or `.max` is refused. Other pairs are ignored.
    pub fn from_query(pairs: &[(String, String)]) -> Result<Vec<AttributeFilter>, Error> {
        let mut out: Vec<AttributeFilter> = Vec::new();
        for (name, value) in pairs {
            let Some(rest) = name.strip_prefix("attr.") else { continue };
            let (key, bound) = match rest.rsplit_once('.') {
                Some((key, "min")) => (key, Some(true)),
                Some((key, "max")) => (key, Some(false)),
                Some((_, other)) if !rest.is_empty() => {
                    // An attribute key has no dots (catalog::usable_key), so
                    // anything after one is a suffix, and only two exist.
                    return Err(Error::bad_request(format!(
                        "'{name}': an attribute filter is attr.<key>, attr.<key>.min or attr.<key>.max, not .{other}"
                    )));
                }
                _ => (rest, None),
            };
            let key = key.trim();
            if key.is_empty() {
                return Err(Error::bad_request(format!("'{name}' names no attribute")));
            }
            let at = match out.iter().position(|f| f.key == key) {
                Some(i) => i,
                None => {
                    out.push(AttributeFilter { key: key.to_string(), ..AttributeFilter::default() });
                    out.len() - 1
                }
            };
            let filter = &mut out[at];
            match bound {
                None => filter.values.push(value.clone()),
                Some(true) => filter.min = Some(value.clone()),
                Some(false) => filter.max = Some(value.clone()),
            }
        }
        Ok(out)
    }
}

/// An [`AttributeFilter`] read against the catalog: its type, its values in
/// the canonical form they are stored in, its bounds as numbers.
#[derive(Debug, Clone)]
pub struct AttributeMatcher {
    key: String,
    any_of: Vec<Value>,
    min: Option<f64>,
    max: Option<f64>,
}

impl AttributeMatcher {
    fn matches(&self, part: &Part) -> bool {
        let Some(have) = part.attributes.get(&self.key).filter(|v| !v.is_null()) else { return false };
        let number = have.as_f64();
        let equal = |want: &Value| match (want, have) {
            (Value::Number(w), Value::Number(h)) => w.as_f64() == h.as_f64(),
            (Value::String(w), Value::String(h)) => w.eq_ignore_ascii_case(h.trim()),
            (w, h) => w == h,
        };
        (self.any_of.is_empty() || self.any_of.iter().any(equal))
            && self.min.is_none_or(|min| number.is_some_and(|n| n >= min))
            && self.max.is_none_or(|max| number.is_some_and(|n| n <= max))
    }
}

/// Read `filter`'s attribute conditions against the catalog, or say why they
/// cannot be read (a `400`).
///
/// With a `category` the key must be in that category's effective schema.
/// Without one it must be defined by some category, and every category that
/// defines it must agree on its type — else the caller is told to narrow by
/// category. A value is read as the catalog reads form text
/// ([`catalog::check_value`]), so `12.5`, `yes` and an enum value in any case
/// all work; `min` and `max` are for numbers only.
pub fn attribute_matchers(state: &State, filter: &PartFilter) -> Result<Vec<AttributeMatcher>, Error> {
    use crate::model::{AttributeDef, AttributeKind};
    let category = catalog::find(&state.categories, filter.category.trim()).map(|c| c.id.clone());
    let mut out = Vec::new();
    for condition in &filter.attributes {
        let key = condition.key.as_str();
        let defs: Vec<(String, AttributeDef)> = match &category {
            Some(id) => catalog::schema(&state.categories, id)?
                .into_iter()
                .filter(|e| e.def.key == key)
                .map(|e| (id.clone(), e.def))
                .collect(),
            None => state
                .categories
                .iter()
                .flat_map(|c| c.attributes.iter().filter(|d| d.key == key).map(move |d| (c.id.clone(), d.clone())))
                .collect(),
        };
        let Some((first_category, def)) = defs.first().cloned() else {
            return Err(Error::bad_request(match &category {
                Some(id) => format!("the category '{id}' has no attribute '{key}'"),
                None => format!("no category has an attribute '{key}'"),
            }));
        };
        if let Some((other, clash)) = defs.iter().find(|(_, d)| d.kind.as_str() != def.kind.as_str()) {
            return Err(Error::bad_request(format!(
                "'{key}' is a {} in '{first_category}' but a {} in '{other}' — give a category to filter by it",
                def.kind.as_str(),
                clash.kind.as_str()
            )));
        }
        let mut any_of = Vec::new();
        for raw in &condition.values {
            // An enum's values may differ between the categories that define
            // the key: a value any of them lists is accepted.
            let read = defs.iter().map(|(_, d)| catalog::check_value(d, &Value::String(raw.clone())));
            let mut first_error = None;
            let mut value = None;
            for result in read {
                match result {
                    Ok(v) => {
                        value = v;
                        first_error = None;
                        break;
                    }
                    Err(e) => {
                        first_error.get_or_insert(e);
                    }
                }
            }
            if let Some(error) = first_error {
                return Err(Error::bad_request(format!("attr.{key}: {error}")));
            }
            let Some(value) = value else {
                return Err(Error::bad_request(format!("attr.{key} is empty — leave it out to filter on nothing")));
            };
            any_of.push(value);
        }
        let bound = |raw: &Option<String>, which: &str| -> Result<Option<f64>, Error> {
            let Some(raw) = raw else { return Ok(None) };
            if !matches!(def.kind, AttributeKind::Number { .. }) {
                return Err(Error::bad_request(format!(
                    "attr.{key}.{which}: '{key}' is a {}, and only a number has a range",
                    def.kind.as_str()
                )));
            }
            raw.trim()
                .parse::<f64>()
                .ok()
                .filter(|n| n.is_finite())
                .map(Some)
                .ok_or_else(|| Error::bad_request(format!("attr.{key}.{which} must be a number, not '{raw}'")))
        };
        out.push(AttributeMatcher {
            key: key.to_string(),
            any_of,
            min: bound(&condition.min, "min")?,
            max: bound(&condition.max, "max")?,
        });
    }
    Ok(out)
}

/// One page of a parts listing ([`Db::find_parts_page`]).
#[derive(Debug, Default)]
pub struct PartPage {
    pub parts: Vec<Part>,
    /// The cursor for the next page, or `None` when this is the last.
    pub next: Option<String>,
    cursors: Vec<i64>,
}

impl PartPage {
    /// Trim a page collected one past `limit`, and set its cursor.
    fn finish(&mut self, limit: Option<usize>) {
        if let Some(limit) = limit {
            if self.parts.len() > limit {
                self.parts.truncate(limit);
                self.next = self.cursors.get(limit - 1).map(|c| c.to_string());
            }
        }
        self.cursors.clear();
    }
}

/// A [`PartFilter`] resolved against the store: the exact rules a part must
/// pass, whichever way the candidates were found.
struct Matcher {
    needle: String,
    tag: String,
    /// The category ids (lower-case) a part must be in; `Some(empty)` with
    /// `uncategorized` for "in no known category".
    within: Option<BTreeSet<String>>,
    uncategorized: bool,
    /// `Some(None)`: the filter names a company that does not exist, which
    /// matches nothing rather than silently widening to everything.
    manufacturer: Option<Option<String>>,
    supplier: Option<Option<String>>,
    external_ref: String,
    part_type: String,
    /// The attribute conditions, read; `None` when they could not be read,
    /// which matches nothing (the route refuses such a filter with a 400
    /// before it gets here).
    attributes: Option<Vec<AttributeMatcher>>,
}

impl Matcher {
    fn new(state: &State, filter: &PartFilter) -> Self {
        let within = match filter.category.trim() {
            "" => None,
            UNCATEGORIZED => Some(BTreeSet::new()),
            id => Some(catalog::descendants(&state.categories, id)),
        };
        let company = |which: sourcing::Companies, key: &str| -> Option<Option<String>> {
            let key = key.trim();
            if key.is_empty() {
                return None;
            }
            Some(sourcing::find_company(which.list(state), key).map(|c| c.id.clone()))
        };
        Matcher {
            needle: filter.q.trim().to_ascii_lowercase(),
            tag: filter.tag.trim().to_string(),
            within,
            uncategorized: filter.category.trim() == UNCATEGORIZED,
            manufacturer: company(sourcing::Companies::Manufacturers, &filter.manufacturer),
            supplier: company(sourcing::Companies::Suppliers, &filter.supplier),
            attributes: attribute_matchers(state, filter).ok(),
            external_ref: filter.external_ref.trim().to_string(),
            part_type: filter.part_type.trim().to_string(),
        }
    }

    /// A filter no part can pass: it names a company that does not exist,
    /// or attribute conditions that do not read.
    fn impossible(&self) -> bool {
        matches!(self.manufacturer, Some(None)) || matches!(self.supplier, Some(None)) || self.attributes.is_none()
    }

    fn matches(&self, categories: &[Category], part: &Part) -> bool {
        (match &self.within {
            None => true,
            Some(_) if self.uncategorized => catalog::find(categories, &part.category).is_none(),
            Some(ids) => ids.contains(&part.category.to_ascii_lowercase()),
        }) && (self.tag.is_empty() || part.tags.iter().any(|t| t.eq_ignore_ascii_case(&self.tag)))
            && (match &self.manufacturer {
                None => true,
                Some(None) => false,
                Some(Some(id)) => part.sourcing.iter().any(|mp| &mp.manufacturer == id),
            })
            && (match &self.supplier {
                None => true,
                Some(None) => false,
                Some(Some(id)) => part.sourcing.iter().flat_map(|mp| &mp.offers).any(|o| &o.supplier == id),
            })
            && (self.external_ref.is_empty() || part.external_ref == self.external_ref)
            && (self.part_type.is_empty() || part.part_type == self.part_type)
            && self.attributes.as_ref().is_some_and(|all| all.iter().all(|a| a.matches(part)))
            && (self.needle.is_empty() || part_matches(categories, part, &self.needle))
    }
}

/// The `category` filter value that selects parts in no known category —
/// empty, or text from before the catalog that names no category.
pub const UNCATEGORIZED: &str = "_none";

impl Db {
    /// Search parts by number, name, category or tag — what a script's
    /// `plm.findParts` asks.
    pub fn search_parts(&self, query: &str) -> Vec<Part> {
        self.find_parts(&PartFilter { q: query.to_string(), ..PartFilter::default() })
    }

    /// The parts that pass `filter`, in creation order.
    pub fn find_parts(&self, filter: &PartFilter) -> Vec<Part> {
        self.find_parts_page(filter, None, None).parts
    }

    /// One page of the parts that pass `filter`, in creation order: at most
    /// `limit`, continuing after the cursor `after` (a [`PartPage::next`]).
    ///
    /// The SQLite side narrows ([`crate::sql::Sql::candidates`]): the trigram
    /// index finds the parts whose searchable text contains the search text,
    /// and indexed columns apply the category, tag and company filters. Every
    /// candidate is then checked against the in-memory part with the exact
    /// rules below, so the answer is the one the rules give — the index only
    /// decides how few parts are looked at. Search text shorter than three
    /// characters (the trigram minimum) narrows nothing and is checked on
    /// every part the other filters leave. Should the index fail, the answer
    /// comes from a scan of memory rather than an error.
    pub fn find_parts_page(&self, filter: &PartFilter, after: Option<&str>, limit: Option<usize>) -> PartPage {
        let matcher = self.read(|state| Matcher::new(state, filter));
        let after: Option<i64> = after.and_then(|a| a.trim().parse().ok());
        let limit = limit.map(|l| l.max(1));
        if matcher.impossible() {
            return PartPage::default();
        }
        // An exact external reference picks one part, or a handful: a walk of
        // memory finds it without paging through the index 256 rows at a time.
        if !matcher.external_ref.is_empty() {
            return self.scanned_page(&matcher, after, limit);
        }
        match self.indexed_page(&matcher, after, limit) {
            Ok(page) => page,
            Err(error) => {
                eprintln!("brep-plm: search index: {error}; answering from a scan");
                self.scanned_page(&matcher, after, limit)
            }
        }
    }

    fn indexed_page(&self, matcher: &Matcher, mut after: Option<i64>, limit: Option<usize>) -> rusqlite::Result<PartPage> {
        let categories: Option<Vec<String>> = match (&matcher.within, matcher.uncategorized) {
            (Some(ids), false) => Some(ids.iter().cloned().collect()),
            _ => None,
        };
        let needle = (matcher.needle.chars().count() >= 3).then_some(matcher.needle.as_str());
        let mut page = PartPage::default();
        // One more than the page, to know whether there IS a next page.
        let wanted = limit.map(|l| l + 1);
        let batch = wanted.map(|w| w.max(256));
        loop {
            let rows = self.sql.candidates(&sql::Candidates {
                needle,
                categories: categories.as_deref(),
                uncategorized: matcher.uncategorized,
                tag: (!matcher.tag.is_empty()).then_some(matcher.tag.as_str()),
                manufacturer: matcher.manufacturer.as_ref().and_then(|m| m.as_deref()),
                supplier: matcher.supplier.as_ref().and_then(|s| s.as_deref()),
                after,
                limit: batch,
            })?;
            let exhausted = batch.is_none_or(|b| rows.len() < b);
            let full = self.read(|state| {
                for (ord, id) in &rows {
                    after = Some(*ord);
                    let Some(part) = state.parts.get(id) else { continue };
                    if matcher.matches(&state.categories, part) {
                        page.parts.push(part.clone());
                        page.cursors.push(*ord);
                        if wanted.is_some_and(|w| page.parts.len() >= w) {
                            return true;
                        }
                    }
                }
                false
            });
            if full || exhausted {
                break;
            }
        }
        page.finish(limit);
        Ok(page)
    }

    fn scanned_page(&self, matcher: &Matcher, after: Option<i64>, limit: Option<usize>) -> PartPage {
        let mut page = PartPage::default();
        self.read(|state| {
            for (ord, part) in state.parts.iter().enumerate() {
                let ord = ord as i64;
                if after.is_some_and(|a| ord <= a) || !matcher.matches(&state.categories, part) {
                    continue;
                }
                page.parts.push(part.clone());
                page.cursors.push(ord);
                if limit.is_some_and(|l| page.parts.len() > l) {
                    break;
                }
            }
        });
        page.finish(limit);
        page
    }

    /// [`Db::find_parts`] by a scan of memory alone, with no index — what the
    /// index's answers are checked against.
    pub fn find_parts_scanning(&self, filter: &PartFilter) -> Vec<Part> {
        let matcher = self.read(|state| Matcher::new(state, filter));
        if matcher.impossible() {
            return Vec::new();
        }
        self.scanned_page(&matcher, None, None).parts
    }

    /// Take the lock on a draft revision.
    pub fn checkout(
        &self,
        user: &User,
        part_id: &str,
        revision_id: &str,
        client_id: &str,
    ) -> Result<(), Error> {
        let user_id = user.id.clone();
        let client_id = client_id.to_string();
        self.mutate(move |state| {
            let (_, revision) = find_revision_mut(state, part_id, revision_id)?;
            if !revision.lifecycle.is_editable() {
                return Err(Error::conflict(format!(
                    "revision {} is {} — only a draft can be checked out",
                    revision.label,
                    revision.lifecycle.as_str()
                )));
            }
            if let Some(lock) = &revision.lock {
                if lock.user_id != user_id {
                    return Err(Error::conflict(
                        "already checked out by another user".to_string(),
                    ));
                }
                // The same user re-taking their own lock from another client
                // is a no-op that re-stamps which client holds it.
            }
            revision.lock = Some(Lock {
                user_id,
                client_id,
                acquired_at: now(),
            });
            state.touch_part(part_id);
            Ok(())
        })
    }

    /// Release the lock. `force` is the BREAK path and requires the check-in
    /// group; without it only the holder may check in.
    pub fn checkin(
        &self,
        user: &User,
        part_id: &str,
        revision_id: &str,
        force: bool,
    ) -> Result<(), Error> {
        let user_id = user.id.clone();
        let may_break = user.can_checkin();
        self.mutate(move |state| {
            let (_, revision) = find_revision_mut(state, part_id, revision_id)?;
            let Some(lock) = revision.lock.clone() else {
                return Err(Error::conflict("nothing is checked out".to_string()));
            };
            if lock.user_id != user_id {
                if !force {
                    return Err(Error::forbidden(
                        "another user holds this lock — breaking it needs the check-in group",
                    ));
                }
                if !may_break {
                    return Err(Error::forbidden(
                        "breaking another user's lock needs the check-in group",
                    ));
                }
            }
            revision.lock = None;
            state.touch_part(part_id);
            Ok(())
        })
    }

    /// Move a revision's lifecycle state, applying the rules in
    /// [`crate::lifecycle`] and the automatic supersede.
    ///
    /// A release runs two hooks when their files exist. `beforeRelease` runs
    /// first, outside the store lock, and a throw refuses the release; the
    /// locked write then re-checks that the revision's bytes are the ones the
    /// hook saw. `afterRelease` runs once the release is committed: it cannot
    /// undo it, so its failure comes back as a WARNING, the release standing.
    pub fn transition(
        &self,
        user: &User,
        part_id: &str,
        revision_id: &str,
        to: Lifecycle,
    ) -> Result<Vec<String>, Error> {
        if to == Lifecycle::Superseded {
            return Err(Error::bad_request(
                "superseded is applied automatically when a newer revision releases",
            ));
        }
        if to == Lifecycle::Released && !user.can_checkin() {
            return Err(Error::forbidden("releasing needs the check-in group"));
        }
        // Submitting for review and pulling back from it are an author's work.
        if matches!(to, Lifecycle::InReview | Lifecycle::Draft) && !user.can_author() {
            return Err(Error::forbidden("submitting for review, or withdrawing, needs the author group"));
        }
        // Submitting opens a review round ([`crate::review`]).
        if to == Lifecycle::InReview {
            return self.submit_for_review(user, part_id, revision_id, &crate::review::Submission::default());
        }

        // A revision an open change order names is released (or obsoleted)
        // through it — refused alone when the administrator says so.
        if matches!(to, Lifecycle::Released | Lifecycle::Obsolete) {
            self.read(|state| eco_hold_gate(state, revision_id, to))?;
        }

        // The before-release gate, on a snapshot. Refusals the locked write
        // would make anyway are made first, so a script is never asked about
        // a release that cannot happen.
        let mut seen_hash = None;
        if to == Lifecycle::Released {
            let (part, revision) = self.snapshot(part_id, revision_id)?;
            lifecycle::check_transition(revision.lifecycle, to).map_err(Error::conflict)?;
            self.read(|state| catalog_gate(&state.categories, &part))?;
            self.read(|state| children_gate(state, &part, &revision))?;
            self.read(|state| review_gate(state, &part, &revision))?;
            if revision.lock.is_some() {
                return Err(Error::conflict(format!(
                    "revision {} is checked out — check it in before releasing",
                    revision.label
                )));
            }
            bake_gate(&revision)?;
            let input = self.read(|state| release_input(state, &part, &revision, user));
            match scripting::run(self, user, scripting::BEFORE_RELEASE, "beforeRelease", &input)? {
                HookResult::Absent | HookResult::Returned(_) => {}
                HookResult::Refused(failure) => {
                    return Err(Error::conflict(format!(
                        "{} refused the release: {}",
                        scripting::BEFORE_RELEASE,
                        failure.message
                    )))
                }
            }
            seen_hash = Some(revision.content_hash.clone());
        }

        let user_id = user.id.clone();
        self.mutate(move |state| {
            if to == Lifecycle::Released {
                // Again INSIDE the lock: the part's values or its category's
                // schema may have changed while the hook ran.
                let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
                catalog_gate(&state.categories, part)?;
                let revision = part.revision(revision_id).ok_or_else(|| Error::not_found("revision"))?;
                children_gate(state, part, revision)?;
                review_gate(state, part, revision)?;
            }
            if matches!(to, Lifecycle::Released | Lifecycle::Obsolete) {
                eco_hold_gate(state, revision_id, to)?;
            }
            let (_, revision) = find_revision_mut(state, part_id, revision_id)?;
            lifecycle::check_transition(revision.lifecycle, to).map_err(Error::conflict)?;
            // Pulling a revision out of review ends its round.
            if revision.lifecycle == Lifecycle::InReview && to == Lifecycle::Draft {
                if let Some(review) = revision.live_review_mut() {
                    crate::review::withdraw(review, &user_id, now());
                }
            }
            if let Some(hash) = &seen_hash {
                if *hash != revision.content_hash {
                    return Err(Error::conflict(format!(
                        "revision {} changed while {} was checking it — release it again",
                        revision.label,
                        scripting::BEFORE_RELEASE
                    )));
                }
            }
            if to == Lifecycle::Released {
                bake_gate(revision)?;
                if let Some(lock) = &revision.lock {
                    return Err(Error::conflict(format!(
                        "revision {} is checked out (held since {}) — check it in before releasing",
                        revision.label, lock.acquired_at
                    )));
                }
                crate::review::close_on_release(revision, &user_id, now());
                revision.released_by = Some(user_id);
                revision.released_at = Some(now());
                revision.lock = None;
            }
            revision.lifecycle = to;
            let released_id = revision.id.clone();

            // The automatic supersede: every OTHER released revision of this
            // part steps down. Applied here so no caller can release without
            // it.
            if to == Lifecycle::Released {
                let part = state.parts.get_mut(&part_id)
                    .ok_or_else(|| Error::not_found("part"))?;
                for other in part.revisions.iter_mut() {
                    if other.id != released_id && other.lifecycle == Lifecycle::Released {
                        other.lifecycle = Lifecycle::Superseded;
                    }
                }
            }
            state.touch_part(&part_id);
            Ok(())
        })?;

        let mut warnings = Vec::new();
        if matches!(to, Lifecycle::Released | Lifecycle::Obsolete) {
            if let Some(number) = self.read(|state| crate::eco::standalone_holder(state, revision_id).map(|e| e.number.clone())) {
                warnings.push(format!(
                    "{} outside {number}, which names this revision — that change order will not release until the item is removed",
                    if to == Lifecycle::Released { "released" } else { "obsoleted" }
                ));
            }
        }
        if to == Lifecycle::Draft {
            warnings.extend(self.review_withdrawn(user, part_id, revision_id));
        }
        if to == Lifecycle::Released {
            let (part, revision) = self.snapshot(part_id, revision_id)?;
            // With the gate off the release stands, and says what it did not
            // wait for.
            let unreleased = self.read(|state| bom::unreleased_children(state, &revision));
            if !unreleased.is_empty() {
                warnings.push(format!("released, but it uses unreleased parts: {}", unreleased.join("; ")));
            }
            let input = self.read(|state| release_input(state, &part, &revision, user));
            match scripting::run(self, user, scripting::AFTER_RELEASE, "afterRelease", &input) {
                Ok(HookResult::Absent) | Ok(HookResult::Returned(_)) => {}
                Ok(HookResult::Refused(failure)) => warnings.push(format!(
                    "released, but {} failed: {}",
                    scripting::AFTER_RELEASE,
                    failure.message
                )),
                Err(error) => warnings.push(format!(
                    "released, but {} could not run: {}",
                    scripting::AFTER_RELEASE,
                    error.message
                )),
            }
        }
        Ok(warnings)
    }

    /// A copy of one part and one of its revisions, for a hook to read.
    pub(crate) fn snapshot(&self, part_id: &str, revision_id: &str) -> Result<(Part, Revision), Error> {
        self.read(|state| {
            let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
            let revision = part
                .revision(revision_id)
                .ok_or_else(|| Error::not_found("revision"))?;
            Ok((part.clone(), revision.clone()))
        })
    }

    /// Delete a draft revision outright — the only removal this server allows.
    /// A released revision is never deleted; it is obsoleted.
    pub fn delete_draft(&self, user: &User, part_id: &str, revision_id: &str) -> Result<(), Error> {
        let user_id = user.id.clone();
        let may_break = user.can_checkin();
        let key = format!("part/{part_id}/rev/{revision_id}");
        let root = self.root().to_path_buf();
        self.mutate(|state| {
            let (_, revision) = find_revision_mut(state, part_id, revision_id)?;
            if !revision.lifecycle.is_editable() {
                return Err(Error::conflict(
                    "only a draft can be deleted — release history is permanent".to_string(),
                ));
            }
            if let Some(lock) = &revision.lock {
                if lock.user_id != user_id && !may_break {
                    return Err(Error::forbidden(
                        "another user has this draft checked out",
                    ));
                }
            }
            let part = state.parts.get_mut(&part_id)
                .ok_or_else(|| Error::not_found("part"))?;
            if part.revisions.len() == 1 {
                return Err(Error::conflict(
                    "a part keeps at least one revision".to_string(),
                ));
            }
            // A revision another list pins cannot go: that list would name
            // nothing.
            if let Some((parent, rev)) = pinned_by(state, part_id, revision_id) {
                return Err(Error::conflict(format!(
                    "this revision is used by {parent} rev {rev} — remove it from that list first"
                )));
            }
            let part = state.parts.get_mut(&part_id)
                .ok_or_else(|| Error::not_found("part"))?;
            let thumbnail = part.revision(revision_id).and_then(|r| r.thumbnail.as_ref()).map(|t| t.sha256.clone());
            part.revisions.retain(|r| r.id != revision_id);
            if let Some(sha) = thumbnail {
                crate::attach::remove_if_unreferenced(state, &root, &sha)?;
            }
            state.touch(key.clone());
            Ok(())
        })?;
        self.remove_document(&key)
    }

    /// The keys that changed since `since`, and whether the log still reaches
    /// that far back.
    pub fn changes_since(&self, since: u64) -> (u64, bool, Vec<String>) {
        self.read(|state| {
            let stale = since < state.changes_floor();
            let keys: Vec<String> = state
                .changes
                .iter()
                .filter(|c| c.seq > since)
                .map(|c| c.key.clone())
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect();
            (state.seq, stale, keys)
        })
    }
}

// ===========================================================================
// The catalog tree
// ===========================================================================

/// What a category is made from, or changed to. On a change, `None` leaves a
/// field as it is.
#[derive(Debug, Clone, Default)]
pub struct CategoryChange {
    pub name: Option<String>,
    /// `Some("")` moves the category to the top level.
    pub parent: Option<String>,
    /// Replaces the category's OWN attribute list.
    pub attributes: Option<Vec<AttributeDef>>,
}

impl Db {
    /// Add a category. The whole tree is re-checked inside the write
    /// ([`catalog::check_tree`]), so a parent that vanished meanwhile or a
    /// key an ancestor gained meanwhile is caught there.
    pub fn create_category(&self, id: &str, change: CategoryChange) -> Result<Category, Error> {
        let id = catalog::normalize_id(id)?;
        let name = change.name.unwrap_or_default().trim().to_string();
        if name.is_empty() {
            return Err(Error::bad_request("a category needs a name"));
        }
        let attributes = catalog::check_definitions(change.attributes.unwrap_or_default())?;
        let parent = change.parent.unwrap_or_default().trim().to_string();
        self.mutate(move |state| {
            if catalog::find(&state.categories, &id).is_some() {
                return Err(Error::conflict(format!("category '{id}' exists")));
            }
            let parent = if parent.is_empty() {
                String::new()
            } else {
                catalog::find(&state.categories, &parent)
                    .map(|c| c.id.clone())
                    .ok_or_else(|| Error::bad_request(format!("there is no category '{parent}'")))?
            };
            let category = Category { id, name, parent, attributes, created_at: now() };
            state.categories.push(category.clone());
            catalog::check_tree(&state.categories)?;
            Ok(category)
        })
    }

    /// Rename a category, move it, or replace its own attributes.
    ///
    /// Changing an attribute's type, or dropping one, leaves the values parts
    /// already hold where they are: a value that no longer fits is reported
    /// at the next release of that part, and a value whose key is gone is
    /// inert. Nothing is rewritten behind anyone's back.
    pub fn update_category(&self, id: &str, change: CategoryChange) -> Result<Category, Error> {
        let attributes = change.attributes.map(catalog::check_definitions).transpose()?;
        let id = id.trim().to_string();
        self.mutate(move |state| {
            let index = state
                .categories
                .iter()
                .position(|c| c.id.eq_ignore_ascii_case(&id))
                .ok_or_else(|| Error::not_found(format!("category '{id}'")))?;
            if let Some(name) = change.name {
                let name = name.trim().to_string();
                if name.is_empty() {
                    return Err(Error::bad_request("a category needs a name"));
                }
                state.categories[index].name = name;
            }
            if let Some(parent) = change.parent {
                let parent = parent.trim();
                state.categories[index].parent = if parent.is_empty() {
                    String::new()
                } else {
                    catalog::find(&state.categories, parent)
                        .map(|c| c.id.clone())
                        .ok_or_else(|| Error::bad_request(format!("there is no category '{parent}'")))?
                };
            }
            if let Some(attributes) = attributes {
                state.categories[index].attributes = attributes;
            }
            // A cycle, a too-deep chain, or a key now defined twice along a
            // chain — in EITHER direction — is refused here, and the mutation
            // rolls back.
            catalog::check_tree(&state.categories)?;
            Ok(state.categories[index].clone())
        })
    }

    /// Remove a category that nothing uses. A category with children, or with
    /// any part filed in it, is refused: deleting it would silently re-file
    /// those parts as uncategorized.
    pub fn delete_category(&self, id: &str) -> Result<(), Error> {
        let id = id.trim().to_string();
        self.mutate(move |state| {
            let category = catalog::find(&state.categories, &id)
                .ok_or_else(|| Error::not_found(format!("category '{id}'")))?
                .id
                .clone();
            if let Some(child) = state.categories.iter().find(|c| c.parent.eq_ignore_ascii_case(&category)) {
                return Err(Error::conflict(format!(
                    "category '{category}' has a sub-category '{}' — move or delete it first",
                    child.id
                )));
            }
            let used = state
                .parts
                .iter()
                .filter(|p| p.category.eq_ignore_ascii_case(&category))
                .count();
            if used > 0 {
                return Err(Error::conflict(format!(
                    "category '{category}' holds {used} part{} — move them to another category first",
                    if used == 1 { "" } else { "s" }
                )));
            }
            state.categories.retain(|c| c.id != category);
            Ok(())
        })
    }
}

/// Whether `needle` (lower-case) appears in anything a person would search a
/// part by.
fn part_matches(categories: &[Category], part: &Part, needle: &str) -> bool {
    let hit = |text: &str| text.to_ascii_lowercase().contains(needle);
    hit(&part.number)
        || hit(&part.name)
        || hit(&part.description)
        || hit(&part.category)
        || hit(&catalog::path_name(categories, &part.category))
        || part.tags.iter().any(|t| hit(t))
        || part.attributes.values().any(|value| match value {
            Value::String(text) => hit(text),
            Value::Number(n) => hit(&n.to_string()),
            _ => false,
        })
        || part.sourcing.iter().any(|mp| hit(&mp.mpn) || mp.offers.iter().any(|o| hit(&o.spn)))
}

/// Refuse a release whose part is missing a required attribute, or holds a
/// value its category's schema no longer accepts. Every problem is named at
/// once, so a user fixes them in one pass.
fn catalog_gate(categories: &[Category], part: &Part) -> Result<(), Error> {
    let problems = catalog::release_problems(categories, part);
    if problems.is_empty() {
        return Ok(());
    }
    Err(Error::conflict(format!(
        "{} cannot release until its catalog values are complete: {}",
        part.number,
        problems.join("; ")
    )))
}

/// Refuse a release while something the revision uses is not Released —
/// only when the administrator has turned
/// [`Settings::require_released_children`] on. Every child is named at once.
/// With [`Settings::eco_holds_revisions`] on, a revision an open change
/// order names is released and obsoleted only through it.
fn eco_hold_gate(state: &State, revision_id: &str, to: Lifecycle) -> Result<(), Error> {
    if !state.settings.eco_holds_revisions {
        return Ok(());
    }
    match crate::eco::standalone_holder(state, revision_id) {
        Some(eco) => Err(Error::conflict(format!(
            "this revision is in {} ({}) — {} it through the change order, or take it out of there first",
            eco.number,
            eco.state.as_str(),
            if to == Lifecycle::Released { "release" } else { "obsolete" }
        ))),
        None => Ok(()),
    }
}

/// A revision whose review rule is not met cannot release ([`crate::review`]).
fn review_gate(state: &State, part: &Part, revision: &Revision) -> Result<(), Error> {
    let (rule, _) = crate::review::rule_for(&state.settings, &state.categories, part);
    crate::review::release_gate(revision, &rule, &part.number)
}

fn children_gate(state: &State, part: &Part, revision: &Revision) -> Result<(), Error> {
    if !state.settings.require_released_children {
        return Ok(());
    }
    let problems = bom::unreleased_children(state, revision);
    if problems.is_empty() {
        return Ok(());
    }
    Err(Error::conflict(format!(
        "{} rev {} cannot release until everything it uses is released: {}",
        part.number,
        revision.label,
        problems.join("; ")
    )))
}

/// The first revision whose uses list pins `revision_id` of `part_id`, as
/// "number" and "label".
fn pinned_by(state: &State, part_id: &str, revision_id: &str) -> Option<(String, String)> {
    state.parts.iter().find_map(|parent| {
        parent.revisions.iter().find_map(|rev| {
            rev.uses
                .iter()
                .any(|u| u.part == part_id && u.revision == revision_id)
                .then(|| (parent.number.clone(), rev.label.clone()))
        })
    })
}

/// The two conditions a revision's document and uses list are written under,
/// checked together so the refusal says which one failed: the revision is in
/// work, and `user` holds its lock.
pub fn check_writable(state: &State, user: &User, part_id: &str, revision_id: &str) -> Result<(), Error> {
    let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
    let revision = part
        .revision(revision_id)
        .ok_or_else(|| Error::not_found("revision"))?;
    if !revision.lifecycle.is_editable() {
        return Err(Error::conflict(format!(
            "{} revision {} is {} and cannot be written — start a new revision",
            part.number,
            revision.label,
            revision.lifecycle.as_str()
        )));
    }
    match &revision.lock {
        None => Err(Error::conflict(format!(
            "{} revision {} is not checked out — check it out before saving",
            part.number, revision.label
        ))),
        Some(lock) if lock.user_id != user.id => {
            let holder = state
                .user(&lock.user_id)
                .map(|u| u.username.clone())
                .unwrap_or_else(|| "another user".to_string());
            Err(Error::conflict(format!(
                "{} revision {} is checked out by {holder}",
                part.number, revision.label
            )))
        }
        Some(_) => Ok(()),
    }
}

impl Db {
    /// Replace a revision's uses list — the call the CAD app makes on save,
    /// and the web page's stand-in until it does. Written under the same two
    /// conditions as the document ([`check_writable`]), so a released
    /// revision's list never changes; checked by [`bom::check_uses`] inside
    /// the same lock, so no two concurrent writes can close a cycle between
    /// them.
    pub fn set_uses(&self, user: &User, part_id: &str, revision_id: &str, body: &Value) -> Result<Vec<Use>, Error> {
        let user = user.clone();
        let body = body.clone();
        self.mutate(move |state| {
            check_writable(state, &user, part_id, revision_id)?;
            let uses = bom::check_uses(state, part_id, &body)?;
            let (_, revision) = find_revision_mut(state, part_id, revision_id)?;
            revision.uses = uses.clone();
            revision.modified_at = now();
            Ok(uses)
        })
    }
}

/// What the release hooks receive.
pub(crate) fn release_input(state: &State, part: &Part, revision: &Revision, user: &User) -> Value {
    json!({
        "part": part,
        "sourcing": sourcing::resolved(state, part),
        "revision": revision,
        "uses": bom::resolved_uses(state, revision),
        "unreleased_children": bom::unreleased_children(state, revision),
        "document_key": revision.document_key(&part.id),
        "user": scripting::user_json(user),
    })
}

/// Whether any part already carries `number` (ASCII case ignored).
fn state_has_number(parts: &[Part], number: &str) -> bool {
    parts.iter().any(|p| p.number.eq_ignore_ascii_case(number))
}

/// An admin's pattern, anchored so it must match the WHOLE number: a pattern
/// of `\d{5}` should not accept `ABC12345XYZ`.
pub fn full_match(pattern: &str) -> Result<regex::Regex, Error> {
    regex::Regex::new(&format!("^(?:{pattern})$"))
        .map_err(|e| Error::bad_request(format!("'{pattern}' is not a usable pattern: {e}")))
}

/// The longest part number or revision label the store accepts.
pub const MAX_IDENTIFIER: usize = 64;

/// A number is text a person can read aloud and type back: not empty, not
/// padded, no control characters, and short enough to print on a label.
pub fn check_number(number: &str) -> Result<(), Error> {
    check_identifier(number, "a part number")
}

/// A label follows the same rule as a number.
pub fn check_label(label: &str) -> Result<(), Error> {
    check_identifier(label, "a revision label")
}

fn check_identifier(text: &str, what: &str) -> Result<(), Error> {
    if text.trim().is_empty() {
        return Err(Error::bad_request(format!("{what} cannot be empty")));
    }
    if text.trim() != text {
        return Err(Error::bad_request(format!("{what} cannot start or end with a space")));
    }
    if text.chars().any(char::is_control) {
        return Err(Error::bad_request(format!("{what} cannot hold control characters")));
    }
    if text.chars().count() > MAX_IDENTIFIER {
        return Err(Error::bad_request(format!("{what} is at most {MAX_IDENTIFIER} characters")));
    }
    Ok(())
}

/// The label offered for a part's next revision: the one after the newest
/// revision's, skipping any the part already has. Free-text labels can be
/// anything, so the walk may restart at `A`; skipping taken labels is what
/// keeps the suggestion usable.
pub fn suggest_label(part: &Part) -> String {
    let mut candidate = lifecycle::next_revision_label(part.latest().map(|r| r.label.as_str()));
    while part.revision_by_label(&candidate).is_some() {
        candidate = lifecycle::next_revision_label(Some(&candidate));
    }
    candidate
}

/// What a new part is made from. Empty `number` lets the type decide (a
/// counter) or leaves it to the script; empty `label` takes the suggestion.
#[derive(Debug, Clone, Default)]
pub struct PartSpec {
    pub part_type: String,
    pub number: String,
    pub name: String,
    pub description: String,
    pub category: String,
    pub document_class: DocumentClass,
    pub label: String,
    pub tags: Vec<String>,
    /// Attribute values, checked against the category's schema. A draft may
    /// leave required ones out; the release gate asks for them.
    pub attributes: serde_json::Map<String, Value>,
    /// Use `number` and `label` as typed: a script may refuse them but not
    /// change them. A family row names its member's number and revision.
    pub exact: bool,
    /// Where the part comes from in another system (a KiCad `library_id`).
    /// Unique among parts of the same type ([`external_ref_clash`]).
    pub external_ref: String,
    /// The first revision's origin: authored, or imported by a library
    /// import. Generated is Generate's alone.
    pub origin: crate::model::Origin,
    /// Link the new part into its creator's workspace, in this folder (empty:
    /// the top), in the same transaction as the part. `None` links nothing.
    /// Only [`crate::workspace::link_on_create`] decides it.
    pub workspace: Option<String>,
}

/// The refusal for giving a part of type `part_type` the external reference
/// `external_ref` when another part of that type already has it — checked
/// inside the store's write lock, so two imports of one library entry at
/// once cannot both make a part. `except` is the part being edited. An empty
/// reference never clashes.
pub fn external_ref_clash(state: &State, part_type: &str, external_ref: &str, except: &str) -> Result<(), Error> {
    if external_ref.is_empty() {
        return Ok(());
    }
    match state.parts.iter().find(|p| p.id != except && p.part_type == part_type && p.external_ref == external_ref) {
        Some(owner) => Err(Error::conflict(format!(
            "external_ref '{external_ref}' is already {} ({}) — look it up with ?external_ref= and use that part",
            owner.number, part_type
        ))),
        None => Ok(()),
    }
}

/// Refuse `origin: generated` from a caller: only Generate makes those.
pub fn check_origin(origin: crate::model::Origin) -> Result<(), Error> {
    if origin == crate::model::Origin::Generated {
        return Err(Error::bad_request("origin 'generated' is set by Generate alone — use authored or imported"));
    }
    Ok(())
}

/// Find a revision for mutation, reporting which half was missing.
pub(crate) fn find_revision_mut<'a>(
    state: &'a mut State,
    part_id: &str,
    revision_id: &str,
) -> Result<(String, &'a mut Revision), Error> {
    let part = state.parts.get_mut(&part_id)
        .ok_or_else(|| Error::not_found("part"))?;
    let number = part.number.clone();
    let revision = part
        .revision_mut(revision_id)
        .ok_or_else(|| Error::not_found("revision"))?;
    Ok((number, revision))
}
