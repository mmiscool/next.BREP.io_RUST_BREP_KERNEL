//! The audit log: who changed what, and when, for every change to the store.
//!
//! # How every change is caught
//!
//! Nothing calls "log this" at each of the store's many write paths — one
//! forgotten call would be a silent hole. Instead [`crate::db::Db::mutate`],
//! which every change already goes through, compares the state before and
//! after and turns the difference into events ([`diff`]). A write path added
//! later is logged without anyone remembering to. The few events that are not
//! a change to the store — a failed sign-in, a script file edited in the
//! browser, a test run — are recorded explicitly ([`crate::db::Db::record`]).
//!
//! An event is written in the SAME SQLite transaction as its change
//! ([`crate::sql::Sql::write`]): a change is never on disk without its events,
//! and a change that is refused or rolled back leaves none.
//!
//! # Who
//!
//! The request guard knows who a request is; the store does not, and most of
//! its methods are not told. So the guard runs each request inside a
//! task-local [`Actor`] scope ([`scope`]), the blocking pool carries it across
//! ([`crate::api::blocking`]), and `mutate` reads it ([`current_actor`]).
//! Anything outside a request — start-up, a test driving the store directly —
//! is the `system` actor unless it says otherwise ([`as_actor`]).
//!
//! # Where
//!
//! Behind [`AuditSink`], so the storage can change without the rest noticing.
//! The store's sink is the `audit` table of `plm.sqlite` ([`crate::sql::Sql`]),
//! with an index for each filter the API offers. Nothing is ever deleted from
//! it. [`JsonlSink`] is the file store's sink (`audit.jsonl`), kept so an old
//! log can still be read; [`crate::db::migrate_files`] imports one.
//!
//! # Actions
//!
//! `create`, `update`, `delete` for anything; and more specific ones where a
//! field says what happened:
//! * revisions: `release`, `submit` (to in review), `supersede`, `obsolete`,
//!   `return-to-draft`, `checkout`, `checkin`, `break-lock`, `document`,
//!   `uses`, `bake`;
//! * parts: `sourcing`;
//! * users: `password`, `groups`, `enable`, `disable`;
//! * tokens: `revoke` (a deleted token);
//! * sign-ins: `sign-in`, `session-ended`, `sign-in-failed`;
//! * scripts: `script-write`, `script-delete`, `script-run`.
//! * preferences (kind `preference`, entity `<user id>/<key>`): `update`,
//!   `delete` — written by [`crate::sql::Sql::set_preference`] in the write's
//!   own transaction, since preferences do not go through `mutate`; the
//!   `@recovery` mirror is not logged ([`crate::api::preferences`]).

use std::collections::{BTreeMap, BTreeSet};
use std::fs::{File, OpenOptions};
use std::io::{self, BufRead, BufReader, Write};
use std::path::PathBuf;
use std::sync::Mutex;

use serde::Deserialize;
use serde_json::{Map, Value};

use crate::model::{Actor, AuditEvent, EntityRef, FieldChange, Timestamp};

// ===========================================================================
// Who is acting
// ===========================================================================

tokio::task_local! {
    static ACTOR: Actor;
}

/// Who the current request is, or the `system` actor outside one.
pub fn current_actor() -> Actor {
    ACTOR.try_with(Clone::clone).unwrap_or_else(|_| Actor::system())
}

/// Run `work` as `actor`, synchronously — the blocking pool, a test.
pub fn as_actor<R>(actor: Actor, work: impl FnOnce() -> R) -> R {
    ACTOR.sync_scope(actor, work)
}

/// Run a future as `actor` — the request guard.
pub async fn scope<F: std::future::Future>(actor: Actor, future: F) -> F::Output {
    ACTOR.scope(actor, future).await
}

// ===========================================================================
// Where events go
// ===========================================================================

/// What an event store must do. Appends assign the ids; nothing edits or
/// deletes an event.
pub trait AuditSink: Send + Sync {
    /// Append `events` in order, numbering each. Returns them numbered.
    fn append(&self, events: Vec<AuditEvent>) -> io::Result<Vec<AuditEvent>>;
    /// The events matching `filter`, newest first.
    fn query(&self, filter: &AuditFilter) -> io::Result<Vec<AuditEvent>>;
}

/// Narrowing a query. Every field is optional; empty means no constraint.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct AuditFilter {
    /// The entity's kind (`part`, `revision`, `user`, …).
    #[serde(default)]
    pub kind: String,
    /// The entity's id.
    #[serde(default)]
    pub entity: String,
    /// A part's id: the part's own events AND its revisions'.
    #[serde(default)]
    pub part: String,
    /// A user id or username (the username ignoring ASCII case).
    #[serde(default)]
    pub user: String,
    #[serde(default)]
    pub action: String,
    /// Unix seconds, inclusive.
    #[serde(default)]
    pub since: Option<Timestamp>,
    #[serde(default)]
    pub until: Option<Timestamp>,
    /// Only events older than this id — the next page.
    #[serde(default)]
    pub before: Option<u64>,
    /// At most this many; the API caps it.
    #[serde(default)]
    pub limit: Option<usize>,
    /// Only these kinds (the API narrows a non-administrator to parts and
    /// revisions). Empty: any.
    #[serde(skip)]
    pub kinds: Vec<String>,
}

impl AuditFilter {
    pub fn matches(&self, event: &AuditEvent) -> bool {
        (self.kind.is_empty() || event.entity.kind == self.kind)
            && (self.kinds.is_empty() || self.kinds.iter().any(|k| *k == event.entity.kind))
            && (self.entity.is_empty() || event.entity.id == self.entity)
            && (self.part.is_empty()
                || event.entity.part_id == self.part
                || (event.entity.kind == "part" && event.entity.id == self.part))
            && (self.user.is_empty()
                || event.actor.user_id == self.user
                || event.actor.username.eq_ignore_ascii_case(&self.user))
            && (self.action.is_empty() || event.action == self.action)
            && self.since.is_none_or(|t| event.at >= t)
            && self.until.is_none_or(|t| event.at <= t)
            && self.before.is_none_or(|id| event.id < id)
    }
}

/// `audit.jsonl`: one event per line, appended, never rewritten.
///
/// A query reads the whole file. A line that does not parse (a torn write from
/// a crash) is skipped, not fatal. No longer the store's sink: see
/// [`crate::sql::Sql`].
pub struct JsonlSink {
    path: PathBuf,
    /// The next id, and the lock that keeps two appends from interleaving.
    next: Mutex<u64>,
}

impl JsonlSink {
    pub fn open(path: impl Into<PathBuf>) -> io::Result<Self> {
        let path = path.into();
        let mut last = 0;
        if path.exists() {
            for line in BufReader::new(File::open(&path)?).lines() {
                if let Ok(event) = serde_json::from_str::<AuditEvent>(&line?) {
                    last = last.max(event.id);
                }
            }
        }
        Ok(JsonlSink { path, next: Mutex::new(last + 1) })
    }
}

impl AuditSink for JsonlSink {
    fn append(&self, mut events: Vec<AuditEvent>) -> io::Result<Vec<AuditEvent>> {
        if events.is_empty() {
            return Ok(events);
        }
        let mut next = self.next.lock().unwrap_or_else(|e| e.into_inner());
        let mut text = String::new();
        let mut id = *next;
        for event in &mut events {
            event.id = id;
            id += 1;
            text.push_str(&serde_json::to_string(event).map_err(io::Error::other)?);
            text.push('\n');
        }
        // One write per batch, in append mode, so a batch lands whole or a
        // reader sees a torn last line it skips.
        let mut file = OpenOptions::new().create(true).append(true).open(&self.path)?;
        file.write_all(text.as_bytes())?;
        file.flush()?;
        *next = id;
        Ok(events)
    }

    fn query(&self, filter: &AuditFilter) -> io::Result<Vec<AuditEvent>> {
        // Hold the append lock so a query never sees half a batch.
        let _next = self.next.lock().unwrap_or_else(|e| e.into_inner());
        if !self.path.exists() {
            return Ok(Vec::new());
        }
        let mut found = Vec::new();
        for line in BufReader::new(File::open(&self.path)?).lines() {
            let Ok(event) = serde_json::from_str::<AuditEvent>(&line?) else { continue };
            if filter.matches(&event) {
                found.push(event);
            }
        }
        found.reverse();
        if let Some(limit) = filter.limit {
            found.truncate(limit);
        }
        Ok(found)
    }
}

// ===========================================================================
// Turning a change into events
// ===========================================================================

/// Fields whose change alone is not worth an event: bookkeeping that moves
/// with every real change, or on every request.
const QUIET_FIELDS: [&str; 4] = ["modified_at", "last_seen", "last_used_at", "next"];

/// Fields whose values are secrets: the event says they changed, never what to.
const SECRET_FIELDS: [&str; 3] = ["password", "hash", "csrf"];

/// Store keys that are the store's own bookkeeping, not anyone's record.
const BOOKKEEPING: [&str; 3] = ["seq", "changes", "changes_from"];

/// Collections of records keyed by `id`: the store key, the entity kind, and
/// the field a person calls one by.
const COLLECTIONS: [(&str, &str, &str); 8] = [
    ("workspace", "workspace", "name"),
    ("change_orders", "eco", "number"),
    ("users", "user", "username"),
    ("api_tokens", "token", "name"),
    ("part_types", "part-type", "name"),
    ("categories", "category", "name"),
    ("manufacturers", "manufacturer", "name"),
    ("suppliers", "supplier", "name"),
];

/// Every event that turns the store from `before` into `after`, by `actor`
/// at `at`, stamped with the new `seq`. Ids are left 0 for the sink.
pub fn diff(before: &Value, after: &Value, actor: &Actor, at: Timestamp, seq: u64) -> Vec<AuditEvent> {
    let mut out = Vec::new();
    let mut push = |action: &str, entity: EntityRef, changes: BTreeMap<String, FieldChange>| {
        out.push(AuditEvent {
            id: 0,
            at,
            actor: actor.clone(),
            action: action.to_string(),
            entity,
            changes,
            detail: String::new(),
            seq,
        });
    };
    let empty = Map::new();
    let before_map = before.as_object().unwrap_or(&empty);
    let after_map = after.as_object().unwrap_or(&empty);
    let keys: BTreeSet<&String> = before_map.keys().chain(after_map.keys()).collect();

    for key in keys {
        let old = before_map.get(key).unwrap_or(&Value::Null);
        let new = after_map.get(key).unwrap_or(&Value::Null);
        if old == new || BOOKKEEPING.contains(&key.as_str()) {
            continue;
        }
        match key.as_str() {
            "parts" => diff_parts(old, new, &mut push),
            "sessions" => diff_sessions(old, new, before_map, after_map, &mut push),
            "settings" => {
                let changes = field_changes(old, new, &[]);
                if !changes.is_empty() {
                    push("update", entity("settings", "settings", "Settings", ""), changes);
                }
            }
            name => {
                if let Some((_, kind, label)) = COLLECTIONS.iter().find(|(store, _, _)| *store == name) {
                    diff_collection(old, new, kind, label, &mut push);
                } else if keyed(old) && keyed(new) {
                    // A collection a later slice adds: logged by its store
                    // name, rather than silently not at all.
                    diff_collection(old, new, name, "name", &mut push);
                } else {
                    let mut changes = BTreeMap::new();
                    changes.insert(name.to_string(), FieldChange { before: old.clone(), after: new.clone() });
                    push("update", entity(name, name, name, ""), changes);
                }
            }
        }
    }
    out
}

fn entity(kind: &str, id: &str, label: &str, part_id: &str) -> EntityRef {
    EntityRef { kind: kind.into(), id: id.into(), label: label.into(), part_id: part_id.into() }
}

/// Whether `value` is a list of records with ids (or empty).
fn keyed(value: &Value) -> bool {
    match value {
        Value::Array(items) => items.iter().all(|item| item.get("id").and_then(Value::as_str).is_some()),
        Value::Null => true,
        _ => false,
    }
}

fn by_id(value: &Value) -> BTreeMap<String, &Value> {
    value
        .as_array()
        .map(|items| {
            items
                .iter()
                .filter_map(|item| Some((item.get("id")?.as_str()?.to_string(), item)))
                .collect()
        })
        .unwrap_or_default()
}

fn text<'a>(value: &'a Value, field: &str) -> &'a str {
    value.get(field).and_then(Value::as_str).unwrap_or("")
}

/// The fields that differ between two records, minus `skip` and the quiet
/// ones, with secrets redacted. A missing record is `null`: every field of
/// the other side appears.
fn field_changes(before: &Value, after: &Value, skip: &[&str]) -> BTreeMap<String, FieldChange> {
    let empty = Map::new();
    let old = before.as_object().unwrap_or(&empty);
    let new = after.as_object().unwrap_or(&empty);
    let mut changes = BTreeMap::new();
    for key in old.keys().chain(new.keys()).collect::<BTreeSet<_>>() {
        if skip.contains(&key.as_str()) || QUIET_FIELDS.contains(&key.as_str()) {
            continue;
        }
        let a = old.get(key).cloned().unwrap_or(Value::Null);
        let b = new.get(key).cloned().unwrap_or(Value::Null);
        if a == b {
            continue;
        }
        let change = if SECRET_FIELDS.contains(&key.as_str()) {
            let mark = |v: &Value| if v.is_null() { Value::Null } else { Value::from("(secret)") };
            FieldChange { before: mark(&a), after: mark(&b) }
        } else {
            FieldChange { before: a, after: b }
        };
        changes.insert(key.clone(), change);
    }
    changes
}

fn diff_collection(
    old: &Value,
    new: &Value,
    kind: &str,
    label_field: &str,
    push: &mut impl FnMut(&str, EntityRef, BTreeMap<String, FieldChange>),
) {
    let before = by_id(old);
    let after = by_id(new);
    for id in before.keys().chain(after.keys()).collect::<BTreeSet<_>>() {
        let (a, b) = (before.get(id).copied(), after.get(id).copied());
        let record = b.or(a).unwrap_or(&Value::Null);
        let who = entity(kind, id, text(record, label_field), "");
        let changes = field_changes(a.unwrap_or(&Value::Null), b.unwrap_or(&Value::Null), &[]);
        match (a, b) {
            (None, Some(_)) => push("create", who, changes),
            (Some(_), None) => push(if kind == "token" { "revoke" } else { "delete" }, who, changes),
            _ if changes.is_empty() => {}
            _ => {
                let action = match kind {
                    "user" => user_action(&changes),
                    "eco" => eco_action(&changes),
                    "workspace" => workspace_action(&changes),
                    _ => "update",
                };
                push(action, who, changes);
            }
        }
    }
}

/// A workspace entry's change, named by what moved: a new file version
/// (a replace or a restore), a move to another folder, a rename, or a link
/// pinned or set to follow.
fn workspace_action(changes: &BTreeMap<String, FieldChange>) -> &'static str {
    let moved: Vec<&str> = changes.keys().map(String::as_str).filter(|k| !QUIET_FIELDS.contains(k)).collect();
    match moved.as_slice() {
        ["versions"] => "version",
        ["parent"] | ["name", "parent"] => "move",
        ["name"] => "rename",
        ["link"] => "pin",
        _ => "update",
    }
}

fn user_action(changes: &BTreeMap<String, FieldChange>) -> &'static str {
    let only = |field: &str| changes.len() == 1 && changes.contains_key(field);
    if only("password") {
        "password"
    } else if only("groups") {
        "groups"
    } else if only("active") {
        if changes["active"].after == Value::Bool(true) { "enable" } else { "disable" }
    } else {
        "update"
    }
}

fn diff_parts(old: &Value, new: &Value, push: &mut impl FnMut(&str, EntityRef, BTreeMap<String, FieldChange>)) {
    let before = by_id(old);
    let after = by_id(new);
    for id in before.keys().chain(after.keys()).collect::<BTreeSet<_>>() {
        let (a, b) = (before.get(id).copied(), after.get(id).copied());
        let part = b.or(a).unwrap_or(&Value::Null);
        let number = text(part, "number").to_string();
        let changes = field_changes(a.unwrap_or(&Value::Null), b.unwrap_or(&Value::Null), &["revisions"]);
        let who = entity("part", id, &number, "");
        match (a, b) {
            (None, Some(_)) => push("create", who, changes),
            (Some(_), None) => push("delete", who, changes),
            _ if changes.is_empty() => {}
            _ => {
                let action = if changes.len() == 1 && changes.contains_key("sourcing") { "sourcing" } else { "update" };
                push(action, who, changes);
            }
        }
        let revs_before = by_id(a.and_then(|p| p.get("revisions")).unwrap_or(&Value::Null));
        let revs_after = by_id(b.and_then(|p| p.get("revisions")).unwrap_or(&Value::Null));
        for rev in revs_before.keys().chain(revs_after.keys()).collect::<BTreeSet<_>>() {
            let (x, y) = (revs_before.get(rev).copied(), revs_after.get(rev).copied());
            let record = y.or(x).unwrap_or(&Value::Null);
            let label = format!("{number} rev {}", text(record, "label"));
            let who = entity("revision", rev, &label, id);
            let changes = field_changes(x.unwrap_or(&Value::Null), y.unwrap_or(&Value::Null), &[]);
            match (x, y) {
                (None, Some(_)) => push("create", who, changes),
                (Some(_), None) => push("delete", who, changes),
                _ if changes.is_empty() => {}
                _ => {
                    let action = revision_action(&changes);
                    push(action, who, changes);
                }
            }
        }
    }
}

/// The one word that best says what happened to a revision. Several fields
/// can change at once (a release also stamps who and when); the most
/// significant one names the event, and every field is still in `changes`.
fn revision_action(changes: &BTreeMap<String, FieldChange>) -> &'static str {
    // A reviewer's decision names the event even when it also moved the
    // lifecycle (a rejection sends the revision back to Draft).
    if let Some(change) = changes.get("reviews") {
        if let Some(verdict) = new_decision(&change.before, &change.after) {
            return verdict;
        }
        if let Some(status) = last_review_status(&change.after) {
            if status == "withdrawn" && last_review_status(&change.before) != Some("withdrawn") {
                return "withdraw";
            }
        }
    }
    if let Some(change) = changes.get("lifecycle") {
        return match change.after.as_str().unwrap_or("") {
            "released" => "release",
            "inreview" => "submit",
            "superseded" => "supersede",
            "obsolete" => "obsolete",
            "draft" => "return-to-draft",
            _ => "update",
        };
    }
    if let Some(change) = changes.get("lock") {
        return match (change.before.is_null(), change.after.is_null()) {
            (true, false) => "checkout",
            (false, true) => {
                // The holder checking in, or someone else breaking the lock:
                // the diff cannot tell who acted, so the caller's name is in
                // the actor and the holder's in `before`.
                "checkin"
            }
            _ => "checkout",
        };
    }
    if changes.contains_key("content_hash") {
        return "document";
    }
    if changes.contains_key("uses") {
        return "uses";
    }
    if changes.contains_key("bake") {
        return "bake";
    }
    if changes.contains_key("reviews") {
        return "review-update";
    }
    if changes.contains_key("comments") {
        return "comment";
    }
    "update"
}

/// The one word that best says what happened to a change order: a decision,
/// then its state, then its items, its round, its discussion.
fn eco_action(changes: &BTreeMap<String, FieldChange>) -> &'static str {
    if let Some(change) = changes.get("reviews") {
        if let Some(verdict) = new_decision(&change.before, &change.after) {
            return verdict;
        }
    }
    if let Some(change) = changes.get("state") {
        let was = change.before.as_str().unwrap_or("");
        return match change.after.as_str().unwrap_or("") {
            "inreview" | "approved" if was == "draft" => "submit",
            "approved" => "approve",
            "released" => "release",
            "cancelled" => "cancel",
            "draft" => "withdraw",
            _ => "update",
        };
    }
    if changes.contains_key("items") {
        return "items";
    }
    if changes.contains_key("reviews") {
        return "review-update";
    }
    if changes.contains_key("comments") {
        return "comment";
    }
    "update"
}

/// The verdict of a decision `after` has that `before` did not, in the last
/// review round of a `reviews` list.
pub(crate) fn new_decision(before: &Value, after: &Value) -> Option<&'static str> {
    let decisions = |v: &Value| -> usize {
        v.as_array()
            .and_then(|rounds| rounds.last())
            .and_then(|r| r.get("decisions"))
            .and_then(Value::as_array)
            .map(Vec::len)
            .unwrap_or(0)
    };
    let same_round = |v: &Value| v.as_array().and_then(|r| r.last()).and_then(|r| r.get("id")).cloned();
    let grew = decisions(after) > decisions(before) || (same_round(before) != same_round(after) && decisions(after) > 0);
    if !grew {
        return None;
    }
    let verdict = after
        .as_array()?
        .last()?
        .get("decisions")?
        .as_array()?
        .last()?
        .get("verdict")?
        .as_str()?;
    Some(if verdict == "reject" { "reject" } else { "approve" })
}

/// The status of the last round in a `reviews` list.
pub(crate) fn last_review_status(value: &Value) -> Option<&str> {
    value.as_array()?.last()?.get("status")?.as_str()
}

fn diff_sessions(
    old: &Value,
    new: &Value,
    before: &Map<String, Value>,
    after: &Map<String, Value>,
    push: &mut impl FnMut(&str, EntityRef, BTreeMap<String, FieldChange>),
) {
    // Keyed by token, which never leaves this function: the event names the
    // user, not the session.
    let tokens = |value: &Value| -> BTreeMap<String, String> {
        value
            .as_array()
            .map(|items| {
                items
                    .iter()
                    .filter_map(|s| Some((s.get("token")?.as_str()?.to_string(), s.get("user_id")?.as_str()?.to_string())))
                    .collect()
            })
            .unwrap_or_default()
    };
    let username = |user_id: &str| -> String {
        for map in [after, before] {
            if let Some(users) = map.get("users").and_then(Value::as_array) {
                if let Some(user) = users.iter().find(|u| text(u, "id") == user_id) {
                    return text(user, "username").to_string();
                }
            }
        }
        String::new()
    };
    let (a, b) = (tokens(old), tokens(new));
    for (token, user_id) in &b {
        if !a.contains_key(token) {
            push("sign-in", entity("user", user_id, &username(user_id), ""), BTreeMap::new());
        }
    }
    // Several sessions of one user ending at once (sign out everywhere) are
    // one event, with the count.
    let mut ended: BTreeMap<&str, u64> = BTreeMap::new();
    for (token, user_id) in &a {
        if !b.contains_key(token) {
            *ended.entry(user_id).or_default() += 1;
        }
    }
    for (user_id, count) in ended {
        let mut changes = BTreeMap::new();
        changes.insert("sessions".to_string(), FieldChange { before: Value::from(count), after: Value::from(0) });
        push("session-ended", entity("user", user_id, &username(user_id), ""), changes);
    }
}

/// The lock change of a `checkin` event names whose lock it was; when that
/// is not the actor, the event is really a broken lock.
pub fn refine(mut events: Vec<AuditEvent>) -> Vec<AuditEvent> {
    for event in &mut events {
        if event.action == "checkin" {
            let holder = event
                .changes
                .get("lock")
                .and_then(|c| c.before.get("user_id"))
                .and_then(Value::as_str)
                .unwrap_or("");
            if !holder.is_empty() && holder != event.actor.user_id {
                event.action = "break-lock".into();
            }
        }
    }
    events
}

/// CSV of events, for the admin's export.
pub fn to_csv(events: &[AuditEvent]) -> String {
    let quote = |text: &str| {
        if text.contains([',', '"', '\n', '\r']) {
            format!("\"{}\"", text.replace('"', "\"\""))
        } else {
            text.to_string()
        }
    };
    let mut out = String::from("id,at,user,via,ip,action,kind,entity,label,part,changes,detail,seq\n");
    for e in events {
        let changes = if e.changes.is_empty() {
            String::new()
        } else {
            serde_json::to_string(&e.changes).unwrap_or_default()
        };
        let row = [
            e.id.to_string(),
            e.at.to_string(),
            e.actor.username.clone(),
            e.actor.via.clone(),
            e.actor.ip.clone(),
            e.action.clone(),
            e.entity.kind.clone(),
            e.entity.id.clone(),
            e.entity.label.clone(),
            e.entity.part_id.clone(),
            changes,
            e.detail.clone(),
            e.seq.to_string(),
        ];
        out.push_str(&row.iter().map(|f| quote(f)).collect::<Vec<_>>().join(","));
        out.push('\n');
    }
    out
}

