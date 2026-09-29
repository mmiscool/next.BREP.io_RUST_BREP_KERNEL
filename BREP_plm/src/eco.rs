//! Change orders — ECOs (round 9 item 5).
//!
//! A change order names revisions and what to do with each: release a Draft or
//! In-review revision, or obsolete a Released or Superseded one. It is
//! reviewed as a whole with the same machinery as a revision
//! ([`crate::review`]), and then released as a whole: every item's gates are
//! checked, every release hook asked, and every item applied in ONE store
//! transaction — all of them, or none.
//!
//! # Lifecycle
//!
//! `Draft → InReview → Approved → Released`, and `Cancelled` from any open
//! state. Submitting opens a review round with [`Settings::eco_review_rule`]
//! (0 approvals by default, so a submitted change order is approved at once).
//! When the round's rule is met the change order becomes `Approved`; a
//! rejection sends it back to `Draft`, and so does withdrawing it. Its items
//! change only in `Draft`, so a round always judges the items it was opened
//! on.
//!
//! # What a release checks, per item
//!
//! The same gates as releasing the revision alone — the lifecycle move, the
//! lock, a waiting bake, complete catalog values, and (when the setting is
//! on) released children, where a child released by the SAME change order
//! counts as released — then `beforeRelease` for each release item, outside
//! the lock. Inside the locked write every gate runs again, each document must
//! be the one its hook saw, and then everything is applied. The revisions'
//! own review rounds are not a gate here: the change order's approval covers
//! its items.
//!
//! A revision is in at most one open change order. Releasing or obsoleting
//! it on its own is allowed with a warning, unless
//! [`Settings::eco_holds_revisions`] is on.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::bom;
use crate::db::{self, now, Db, State};
use crate::lifecycle;
use crate::model::{
    ChangeOrder, EcoAction, EcoItem, EcoPriority, EcoState, Lifecycle, NumberMode, Part, PartType, Revision,
    ReviewStatus, User, Verdict,
};
use crate::review::{self, CommentView, ReviewView, RoundChange};
use crate::scripting::{self, HookResult};
use crate::Error;

/// The longest title and text fields a change order keeps.
pub const MAX_TITLE: usize = 200;
pub const MAX_TEXT: usize = 20_000;

// ===========================================================================
// Rules on the state
// ===========================================================================

/// The open change order (other than `except`) that names `revision_id`.
pub fn holder<'a>(state: &'a State, revision_id: &str, except: &str) -> Option<&'a ChangeOrder> {
    state
        .change_orders
        .iter()
        .find(|e| e.id != except && e.state.is_open() && e.item(revision_id).is_some())
}

/// The change order with this id, or else this number (ignoring case).
pub fn find<'a>(state: &'a State, key: &str) -> Option<&'a ChangeOrder> {
    let key = key.trim();
    state
        .change_orders
        .iter()
        .find(|e| e.id == key)
        .or_else(|| state.change_orders.iter().find(|e| e.number.eq_ignore_ascii_case(key)))
}

fn find_mut<'a>(state: &'a mut State, key: &str) -> Result<&'a mut ChangeOrder, Error> {
    let id = find(state, key).map(|e| e.id.clone()).ok_or_else(|| Error::not_found("change order"))?;
    state.change_orders.iter_mut().find(|e| e.id == id).ok_or_else(|| Error::not_found("change order"))
}

/// What stops `item` of `eco` being applied now, if anything. `releasing` is
/// every revision the change order releases — a child among them counts as
/// released.
pub fn item_problem(state: &State, eco: &ChangeOrder, item: &EcoItem, releasing: &BTreeSet<String>) -> Option<String> {
    let Some(part) = state.part(&item.part_id) else {
        return Some("its part no longer exists".into());
    };
    let Some(revision) = part.revision(&item.revision_id) else {
        return Some(format!("{}: its revision no longer exists", part.number));
    };
    let name = format!("{} rev {}", part.number, revision.label);
    let problem = match item.action {
        EcoAction::Release => release_problem(state, part, revision, releasing),
        EcoAction::Obsolete => obsolete_problem(revision, releasing, part),
    };
    let problem = problem.or_else(|| {
        holder(state, &revision.id, &eco.id).map(|other| format!("it is also in {}", other.number))
    });
    problem.map(|p| format!("{name}: {p}"))
}

fn release_problem(state: &State, part: &Part, revision: &Revision, releasing: &BTreeSet<String>) -> Option<String> {
    if let Err(refusal) = lifecycle::check_transition(revision.lifecycle, Lifecycle::Released) {
        return Some(refusal);
    }
    if let Some(lock) = &revision.lock {
        let who = state.user(&lock.user_id).map(|u| u.username.clone()).unwrap_or_else(|| lock.user_id.clone());
        return Some(format!("it is checked out by {who} — check it in first"));
    }
    if revision.needs_bake() {
        return Some("it is waiting for a bake".into());
    }
    let missing = crate::catalog::release_problems(&state.categories, part);
    if !missing.is_empty() {
        return Some(format!("its catalog values are incomplete: {}", missing.join("; ")));
    }
    if state.settings.require_released_children {
        let unreleased = unreleased_children_after(state, revision, releasing);
        if !unreleased.is_empty() {
            return Some(format!("it uses unreleased parts: {}", unreleased.join("; ")));
        }
    }
    None
}

fn obsolete_problem(revision: &Revision, releasing: &BTreeSet<String>, part: &Part) -> Option<String> {
    // A revision this order also releases is not obsoleted by it; one that
    // the order's release will SUPERSEDE is still obsoleted afterwards.
    if releasing.contains(&revision.id) {
        return Some("the same change order releases it".into());
    }
    let _ = part;
    lifecycle::check_transition(revision.lifecycle, Lifecycle::Obsolete).err()
}

/// [`bom::unreleased_children`], counting as released every child that
/// `releasing` will release: a pinned child among them, or — for a floating
/// line — any revision of that child part among them.
pub fn unreleased_children_after(state: &State, revision: &Revision, releasing: &BTreeSet<String>) -> Vec<String> {
    let mut out = Vec::new();
    for line in &revision.uses {
        let Some(child) = state.part(&line.part) else {
            out.push(format!("a used part ({}) no longer exists", line.part));
            continue;
        };
        let released_here = if line.revision.is_empty() {
            child.revisions.iter().any(|r| releasing.contains(&r.id))
        } else {
            releasing.contains(&line.revision)
        };
        if released_here {
            continue;
        }
        match bom::resolve(child, line) {
            None => out.push(format!("{}: the revision it names no longer exists", child.number)),
            Some(_) if line.revision.is_empty() && child.current_release().is_none() => {
                out.push(format!("{} has no released revision", child.number))
            }
            Some(rev) if rev.lifecycle != Lifecycle::Released => {
                out.push(format!("{} rev {} is {}", child.number, rev.label, rev.lifecycle.as_str()))
            }
            Some(_) => {}
        }
    }
    out
}

/// Every revision the change order releases.
pub fn releasing(eco: &ChangeOrder) -> BTreeSet<String> {
    eco.items
        .iter()
        .filter(|i| i.action == EcoAction::Release)
        .map(|i| i.revision_id.clone())
        .collect()
}

/// Every problem that would stop `eco` releasing now.
pub fn problems(state: &State, eco: &ChangeOrder) -> Vec<String> {
    let releasing = releasing(eco);
    eco.items.iter().filter_map(|item| item_problem(state, eco, item, &releasing)).collect()
}

/// The revision a change order's release or obsolete of it would conflict
/// with, for `transition` of that revision alone: the open change order
/// naming it, if any.
pub fn standalone_holder<'a>(state: &'a State, revision_id: &str) -> Option<&'a ChangeOrder> {
    holder(state, revision_id, "")
}

fn check_text(what: &str, text: &str, max: usize) -> Result<String, Error> {
    let text = text.trim();
    if text.chars().count() > max {
        return Err(Error::bad_request(format!("the {what} is at most {max} characters")));
    }
    Ok(text.to_string())
}

pub fn priority(text: &str) -> Result<EcoPriority, Error> {
    match text.trim().to_ascii_lowercase().as_str() {
        "" | "normal" => Ok(EcoPriority::Normal),
        "low" => Ok(EcoPriority::Low),
        "high" => Ok(EcoPriority::High),
        "critical" => Ok(EcoPriority::Critical),
        other => Err(Error::bad_request(format!("'{other}' is not a priority — low, normal, high or critical"))),
    }
}

pub fn action(text: &str) -> Result<EcoAction, Error> {
    match text.trim().to_ascii_lowercase().as_str() {
        "" | "release" => Ok(EcoAction::Release),
        "obsolete" => Ok(EcoAction::Obsolete),
        other => Err(Error::bad_request(format!("'{other}' is not an action — release or obsolete"))),
    }
}

// ===========================================================================
// What the page shows
// ===========================================================================

/// A change order in a list.
#[derive(Debug, Serialize)]
pub struct EcoRow {
    pub id: String,
    pub number: String,
    pub title: String,
    pub state: &'static str,
    pub priority: &'static str,
    /// How many revisions it names. Not `items`: the detail view flattens
    /// this row beside its own `items` list, and a key twice in one object
    /// is refused by any strict reader (serde) and silently resolved by a
    /// lenient one.
    pub item_count: usize,
    pub created_by: String,
    pub created_at: u64,
    pub released_at: Option<u64>,
    /// The current round's status, if it has one.
    pub review: Option<&'static str>,
}

/// One item as the page shows it: what it is, what it changes, and what
/// would stop it.
#[derive(Debug, Serialize)]
pub struct ItemView {
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub revision_id: String,
    pub label: String,
    pub lifecycle: &'static str,
    pub action: EcoAction,
    pub note: String,
    /// For a release: the revision it replaces as current, if there is one.
    pub replaces: Option<String>,
    pub replaces_id: Option<String>,
    /// For a release: whether the document differs from what it replaces.
    pub document_changed: Option<bool>,
    pub content_hash: String,
    pub previous_hash: String,
    /// For a release of an assembly: its uses list against the one it replaces.
    pub bom_diff: Option<bom::BomDiff>,
    /// For a release: catalog changes to the part since it last released.
    pub attribute_changes: Vec<AttributeChange>,
    /// Why it could not be applied now; empty when nothing stops it.
    pub problem: String,
}

/// One change to a part's catalog values, from the audit log.
#[derive(Debug, Serialize)]
pub struct AttributeChange {
    pub at: u64,
    pub by: String,
    pub field: String,
    pub before: Value,
    pub after: Value,
}

/// A change order as its page shows it.
#[derive(Debug, Serialize)]
pub struct EcoView {
    #[serde(flatten)]
    pub eco: EcoRow,
    pub description: String,
    pub reason: String,
    pub released_by: Option<String>,
    pub cancelled_by: Option<String>,
    pub cancelled_at: Option<u64>,
    pub items: Vec<ItemView>,
    pub current: Option<ReviewView>,
    pub history: Vec<ReviewView>,
    pub comments: Vec<CommentView>,
    pub rule: crate::model::ReviewRule,
    /// Every problem that would stop the release now.
    pub problems: Vec<String>,
    pub can_edit: bool,
    pub can_release: bool,
    pub can_comment: bool,
}

fn name_of(state: &State, id: &str) -> String {
    state.user(id).map(|u| u.username.clone()).unwrap_or_else(|| id.to_string())
}

pub fn row(state: &State, eco: &ChangeOrder) -> EcoRow {
    EcoRow {
        id: eco.id.clone(),
        number: eco.number.clone(),
        title: eco.title.clone(),
        state: eco.state.as_str(),
        priority: eco.priority.as_str(),
        item_count: eco.items.len(),
        created_by: name_of(state, &eco.created_by),
        created_at: eco.created_at,
        released_at: eco.released_at,
        review: eco.reviews.last().map(|r| r.status.as_str()),
    }
}

// ===========================================================================
// The store's operations
// ===========================================================================

/// What a new change order is made from.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct NewEco {
    #[serde(default)]
    pub number: String,
    pub title: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub reason: String,
    #[serde(default)]
    pub priority: String,
}

/// A change to a change order's words. `None` leaves a field as it is.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct EcoChange {
    #[serde(default)]
    pub title: Option<String>,
    #[serde(default)]
    pub description: Option<String>,
    #[serde(default)]
    pub reason: Option<String>,
    #[serde(default)]
    pub priority: Option<String>,
}

/// An item to add: a part by id or number, a revision by id or label.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct NewItem {
    pub part: String,
    pub revision: String,
    #[serde(default)]
    pub action: String,
    #[serde(default)]
    pub note: String,
}

/// A change to the ECO numbering. `None` leaves a field as it is.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct NumberingChange {
    #[serde(default)]
    pub prefix: Option<String>,
    #[serde(default)]
    pub digits: Option<u32>,
    #[serde(default)]
    pub next: Option<u64>,
    #[serde(default)]
    pub mode: Option<NumberMode>,
}

impl Db {
    // -- numbering ---------------------------------------------------------

    /// The number a new change order gets, decided outside the lock: a typed
    /// one checked against the mode, or the script's answer. `None` means
    /// "the counter decides, inside the lock".
    fn eco_number(&self, user: &User, numbering: &PartType, typed: &str, input: &NewEco) -> Result<Option<String>, Error> {
        let typed = typed.trim().to_string();
        let given = match &numbering.mode {
            NumberMode::Counter => {
                if !typed.is_empty() {
                    return Err(Error::bad_request("change orders number by a counter — leave the number empty"));
                }
                None
            }
            NumberMode::Free => {
                if typed.is_empty() {
                    return Err(Error::bad_request("change orders take the number you type — enter one"));
                }
                Some(typed)
            }
            NumberMode::Pattern { regex } => {
                if !db::full_match(regex)?.is_match(&typed) {
                    return Err(Error::bad_request(format!(
                        "'{typed}' does not match the change-order number pattern ({regex})"
                    )));
                }
                Some(typed)
            }
            NumberMode::Script { script } => {
                let request = json!({
                    "requested": typed,
                    "numbering": numbering,
                    "eco": { "title": input.title, "reason": input.reason, "priority": input.priority },
                    "user": scripting::user_json(user),
                });
                let answer = match scripting::run(self, user, script, "ecoNumber", &request)? {
                    HookResult::Absent => {
                        return Err(Error::conflict(format!(
                            "change orders number by the script '{script}', which is not in the scripts directory"
                        )))
                    }
                    HookResult::Refused(failure) => return Err(Error::conflict(format!("{script}: {}", failure.message))),
                    HookResult::Returned(success) => success.value,
                };
                Some(scripting::string_answer(&answer, "number").ok_or_else(|| {
                    Error::conflict(format!("{script}: ecoNumber() must return the number, or {{ number }} — it returned {answer}"))
                })?)
            }
        };
        if let Some(number) = &given {
            db::check_number(number)?;
        }
        Ok(given)
    }

    /// Change how change orders are numbered. Administrators only (the API
    /// checks).
    pub fn update_eco_numbering(&self, change: &NumberingChange) -> Result<PartType, Error> {
        if let Some(digits) = change.digits {
            if !(1..=18).contains(&digits) {
                return Err(Error::bad_request("digits must be between 1 and 18"));
            }
        }
        if let Some(mode) = &change.mode {
            match mode {
                NumberMode::Pattern { regex } => {
                    if regex.trim().is_empty() {
                        return Err(Error::bad_request("a pattern needs a pattern"));
                    }
                    db::full_match(regex)?;
                }
                NumberMode::Script { script } => {
                    self.scripts().resolve(script)?;
                }
                _ => {}
            }
        }
        let change = change.clone();
        self.mutate(move |state| {
            let mut numbering = state.eco_numbering.0.clone();
            if let Some(prefix) = change.prefix {
                numbering.prefix = prefix.trim().to_string();
            }
            if let Some(digits) = change.digits {
                numbering.digits = digits;
            }
            if let Some(next) = change.next {
                numbering.next = next.max(1);
            }
            if let Some(mode) = change.mode {
                numbering.mode = mode;
            }
            if numbering.mode == NumberMode::Counter && numbering.next > numbering.capacity() {
                return Err(Error::bad_request(format!(
                    "the next number {} does not fit in {} digits",
                    numbering.next, numbering.digits
                )));
            }
            state.eco_numbering.0 = numbering.clone();
            Ok(numbering)
        })
    }

    // -- creating and editing ----------------------------------------------

    /// Open a new change order in Draft.
    pub fn create_eco(&self, user: &User, spec: &NewEco) -> Result<ChangeOrder, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("a change order is made by the author group"));
        }
        let title = check_text("title", &spec.title, MAX_TITLE)?;
        if title.is_empty() {
            return Err(Error::bad_request("a change order needs a title"));
        }
        let description = check_text("description", &spec.description, MAX_TEXT)?;
        let reason = check_text("reason", &spec.reason, MAX_TEXT)?;
        let priority = priority(&spec.priority)?;
        let numbering = self.read(|state| state.eco_numbering.0.clone());
        let given = self.eco_number(user, &numbering, &spec.number, spec)?;
        let author = user.id.clone();
        self.mutate(move |state| {
            if state.eco_numbering.mode != numbering.mode {
                return Err(Error::conflict("change-order numbering changed meanwhile — try again"));
            }
            let taken = |state: &State, number: &str| state.change_orders.iter().any(|e| e.number.eq_ignore_ascii_case(number));
            let number = match given {
                Some(number) => number,
                None => {
                    let current = state.eco_numbering.0.clone();
                    let mut sequence = current.next;
                    let mut number = current.format_number(sequence);
                    while sequence <= current.capacity() && taken(state, &number) {
                        sequence += 1;
                        number = current.format_number(sequence);
                    }
                    if sequence > current.capacity() {
                        return Err(Error::conflict(format!(
                            "change-order numbering is exhausted: {} digits cannot spell {sequence}",
                            current.digits
                        )));
                    }
                    state.eco_numbering.0.next = sequence + 1;
                    number
                }
            };
            if taken(state, &number) {
                return Err(Error::conflict(format!("change order {number} already exists")));
            }
            let stamp = now();
            let eco = ChangeOrder {
                id: crate::auth::new_id(),
                number,
                title,
                description,
                reason,
                priority,
                state: EcoState::Draft,
                items: Vec::new(),
                created_by: author,
                created_at: stamp,
                modified_at: stamp,
                released_by: None,
                released_at: None,
                cancelled_by: None,
                cancelled_at: None,
                reviews: Vec::new(),
                comments: Vec::new(),
            };
            state.change_orders.push(eco.clone());
            Ok(eco)
        })
    }

    /// Change a change order's title, description, reason or priority —
    /// while it is still open.
    pub fn update_eco(&self, user: &User, key: &str, change: &EcoChange) -> Result<ChangeOrder, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("a change order is edited by the author group"));
        }
        let title = change.title.as_deref().map(|t| check_text("title", t, MAX_TITLE)).transpose()?;
        if title.as_deref() == Some("") {
            return Err(Error::bad_request("a change order needs a title"));
        }
        let description = change.description.as_deref().map(|t| check_text("description", t, MAX_TEXT)).transpose()?;
        let reason = change.reason.as_deref().map(|t| check_text("reason", t, MAX_TEXT)).transpose()?;
        let priority = change.priority.as_deref().map(priority).transpose()?;
        self.mutate(move |state| {
            let eco = find_mut(state, key)?;
            if !eco.state.is_open() {
                return Err(Error::conflict(format!("{} is {} and cannot change", eco.number, eco.state.as_str())));
            }
            if let Some(t) = title {
                eco.title = t;
            }
            if let Some(d) = description {
                eco.description = d;
            }
            if let Some(r) = reason {
                eco.reason = r;
            }
            if let Some(p) = priority {
                eco.priority = p;
            }
            eco.modified_at = now();
            Ok(eco.clone())
        })
    }

    /// Add an item to a Draft change order.
    pub fn add_eco_item(&self, user: &User, key: &str, item: &NewItem) -> Result<ChangeOrder, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("a change order is edited by the author group"));
        }
        let action = action(&item.action)?;
        let note = check_text("note", &item.note, MAX_TITLE)?;
        let item = item.clone();
        self.mutate(move |state| {
            let part = state.part_by_id_or_number(&item.part).ok_or_else(|| Error::not_found(format!("part '{}'", item.part)))?;
            let revision = bom::find_revision(part, &item.revision)
                .ok_or_else(|| Error::not_found(format!("revision '{}' of {}", item.revision, part.number)))?;
            let name = format!("{} rev {}", part.number, revision.label);
            match action {
                EcoAction::Release if !revision.lifecycle.is_editable() => {
                    return Err(Error::conflict(format!(
                        "{name} is {} — only a revision in work can be released",
                        revision.lifecycle.as_str()
                    )))
                }
                EcoAction::Obsolete if !matches!(revision.lifecycle, Lifecycle::Released | Lifecycle::Superseded) => {
                    return Err(Error::conflict(format!(
                        "{name} is {} — only a released or superseded revision can be obsoleted",
                        revision.lifecycle.as_str()
                    )))
                }
                _ => {}
            }
            let (part_id, revision_id) = (part.id.clone(), revision.id.clone());
            let this = find(state, key).ok_or_else(|| Error::not_found("change order"))?;
            let this_id = this.id.clone();
            if let Some(other) = holder(state, &revision_id, &this_id) {
                return Err(Error::conflict(format!("{name} is already in {}", other.number)));
            }
            let eco = find_mut(state, &this_id)?;
            if eco.state != EcoState::Draft {
                return Err(Error::conflict(format!(
                    "{} is {} — items change only in a draft; withdraw it first",
                    eco.number,
                    eco.state.as_str()
                )));
            }
            if eco.item(&revision_id).is_some() {
                return Err(Error::conflict(format!("{name} is already in this change order")));
            }
            if action == EcoAction::Release
                && eco.items.iter().any(|i| i.part_id == part_id && i.action == EcoAction::Release)
            {
                return Err(Error::conflict(format!(
                    "this change order already releases a revision of {} — one release per part",
                    name.split(" rev ").next().unwrap_or("")
                )));
            }
            eco.items.push(EcoItem { part_id, revision_id, action, note, replaced: None });
            eco.modified_at = now();
            Ok(eco.clone())
        })
    }

    /// Take an item out of a Draft change order.
    pub fn remove_eco_item(&self, user: &User, key: &str, revision_id: &str) -> Result<ChangeOrder, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("a change order is edited by the author group"));
        }
        self.mutate(move |state| {
            let eco = find_mut(state, key)?;
            if eco.state != EcoState::Draft {
                return Err(Error::conflict(format!(
                    "{} is {} — items change only in a draft",
                    eco.number,
                    eco.state.as_str()
                )));
            }
            let before = eco.items.len();
            eco.items.retain(|i| i.revision_id != revision_id);
            if eco.items.len() == before {
                return Err(Error::not_found("that item"));
            }
            eco.modified_at = now();
            Ok(eco.clone())
        })
    }

    // -- review --------------------------------------------------------------

    /// Submit a Draft change order: open a review round with the change-order
    /// rule. A round approved on opening (0 approvals) approves the order.
    pub fn submit_eco(&self, user: &User, key: &str, extra_reviewers: Option<&Value>, note: &str) -> Result<Vec<String>, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("submitting a change order needs the author group"));
        }
        let eco = self.read(|state| find(state, key).cloned()).ok_or_else(|| Error::not_found("change order"))?;
        if eco.state != EcoState::Draft {
            return Err(Error::conflict(format!("{} is {}, not a draft", eco.number, eco.state.as_str())));
        }
        let mut rule = self.read(|state| state.settings.eco_review_rule.clone());
        if let Some(extra) = extra_reviewers {
            for reviewer in self.read(|state| review::parse_reviewers(state, extra))? {
                if !rule.reviewers.contains(&reviewer) {
                    rule.reviewers.push(reviewer);
                }
            }
        }
        let mut round = review::open_round(&rule, "settings:eco", &user.id, now());
        let input = json!({
            "kind": "eco",
            "eco": eco,
            "rule": {
                "required_approvals": round.required_approvals,
                "reviewers": round.reviewers,
                "due": round.due,
                "allow_self_approval": round.allow_self_approval,
                "source": "settings:eco",
            },
            "user": scripting::user_json(user),
        });
        self.shape_round(user, &mut round, &input)?;
        let (id, author, note) = (eco.id.clone(), user.clone(), note.to_string());
        self.mutate(move |state| {
            let eco = find_mut(state, &id)?;
            if eco.state != EcoState::Draft {
                return Err(Error::conflict(format!("{} is {}, not a draft", eco.number, eco.state.as_str())));
            }
            if eco.items.is_empty() {
                return Err(Error::conflict(format!("{} names no revisions yet — add what it changes first", eco.number)));
            }
            eco.state = if round.status == ReviewStatus::Approved { EcoState::Approved } else { EcoState::InReview };
            eco.reviews.push(round);
            if !note.trim().is_empty() {
                review::push_comment(&mut eco.comments, &author, &note, "", now())?;
            }
            eco.modified_at = now();
            Ok(())
        })?;
        Ok(self.eco_event(user, "opened", &eco.id, None))
    }

    /// Ask `reviewers.js` about a round about to open, and apply its answer.
    pub(crate) fn shape_round(&self, user: &User, round: &mut crate::model::Review, input: &Value) -> Result<(), Error> {
        match scripting::run(self, user, scripting::REVIEWERS, "reviewers", input)? {
            HookResult::Absent => Ok(()),
            HookResult::Refused(failure) => Err(Error::conflict(format!("{}: {}", scripting::REVIEWERS, failure.message))),
            HookResult::Returned(success) => match &success.value {
                Value::Null => Ok(()),
                Value::Object(answer) => {
                    if let Some(reviewers) = answer.get("reviewers") {
                        round.reviewers = self
                            .read(|state| review::parse_reviewers(state, reviewers))
                            .map_err(|e| Error::conflict(format!("{}: {}", scripting::REVIEWERS, e.message)))?;
                    }
                    if let Some(required) = answer.get("required_approvals") {
                        round.required_approvals = required
                            .as_u64()
                            .filter(|n| *n <= u64::from(review::MAX_APPROVALS))
                            .ok_or_else(|| {
                                Error::conflict(format!(
                                    "{}: required_approvals must be a whole number up to {}",
                                    scripting::REVIEWERS,
                                    review::MAX_APPROVALS
                                ))
                            })? as u32;
                    }
                    if let Some(due) = answer.get("due") {
                        round.due = due.as_u64().filter(|d| *d > 0);
                    }
                    if let Some(own) = answer.get("allow_self_approval").and_then(Value::as_bool) {
                        round.allow_self_approval = own;
                    }
                    round.rule_source.push_str("+script");
                    review::refresh(round, "");
                    Ok(())
                }
                other => Err(Error::conflict(format!(
                    "{}: reviewers() must return {{ reviewers, required_approvals, due }} or null — it returned {other}",
                    scripting::REVIEWERS
                ))),
            },
        }
    }

    /// Approve or reject a change order under review. A rejection ends the
    /// round and returns it to Draft; the approval that meets the rule
    /// approves it.
    pub fn decide_eco(&self, user: &User, key: &str, verdict: Verdict, comment: &str) -> Result<Vec<String>, Error> {
        let (decider, comment) = (user.clone(), comment.to_string());
        let id = self.mutate(move |state| {
            let eco = find_mut(state, key)?;
            if !matches!(eco.state, EcoState::InReview | EcoState::Approved) {
                return Err(Error::conflict(format!("{} is {}, not in review", eco.number, eco.state.as_str())));
            }
            let number = eco.number.clone();
            let snapshot = eco.clone();
            let current = review::eco_subject_hash(state, &snapshot);
            let eco = find_mut(state, key)?;
            let Some(round) = eco.reviews.last_mut().filter(|r| r.is_live()) else {
                return Err(Error::conflict(format!("{number} has no review open")));
            };
            review::record_decision(round, &decider, verdict, &comment, now(), &current)?;
            eco.state = match round.status {
                ReviewStatus::Rejected => EcoState::Draft,
                ReviewStatus::Approved => EcoState::Approved,
                _ => EcoState::InReview,
            };
            eco.modified_at = now();
            Ok(eco.id.clone())
        })?;
        Ok(self.eco_event(user, verdict.as_str(), &id, None))
    }

    /// Change a change order's live round.
    pub fn change_eco_review(&self, user: &User, key: &str, change: &RoundChange) -> Result<Vec<String>, Error> {
        let (editor, change) = (user.clone(), change.clone());
        let id = self.mutate(move |state| {
            let eco = find(state, key).ok_or_else(|| Error::not_found("change order"))?;
            let Some(mut round) = eco.reviews.last().cloned() else {
                return Err(Error::conflict("this change order has no review"));
            };
            let current = review::eco_subject_hash(state, eco);
            review::change_round(state, &mut round, &editor, &change, &current)?;
            let eco = find_mut(state, key)?;
            if eco.state.is_open() && eco.state != EcoState::Draft {
                eco.state = if round.status == ReviewStatus::Approved { EcoState::Approved } else { EcoState::InReview };
            }
            if let Some(last) = eco.reviews.last_mut() {
                *last = round;
            }
            eco.modified_at = now();
            Ok(eco.id.clone())
        })?;
        Ok(self.eco_event(user, "updated", &id, None))
    }

    /// Pull a change order under review back to Draft, ending its round.
    pub fn withdraw_eco(&self, user: &User, key: &str) -> Result<Vec<String>, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("withdrawing a change order needs the author group"));
        }
        let by = user.id.clone();
        let id = self.mutate(move |state| {
            let eco = find_mut(state, key)?;
            if !matches!(eco.state, EcoState::InReview | EcoState::Approved) {
                return Err(Error::conflict(format!("{} is {}, not in review", eco.number, eco.state.as_str())));
            }
            if let Some(round) = eco.reviews.last_mut() {
                review::withdraw(round, &by, now());
            }
            eco.state = EcoState::Draft;
            eco.modified_at = now();
            Ok(eco.id.clone())
        })?;
        Ok(self.eco_event(user, "withdrawn", &id, None))
    }

    /// Cancel an open change order. Its revisions are left as they are.
    pub fn cancel_eco(&self, user: &User, key: &str) -> Result<Vec<String>, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("cancelling a change order needs the author group"));
        }
        let by = user.id.clone();
        let id = self.mutate(move |state| {
            let eco = find_mut(state, key)?;
            if !eco.state.is_open() {
                return Err(Error::conflict(format!("{} is already {}", eco.number, eco.state.as_str())));
            }
            let stamp = now();
            if let Some(round) = eco.reviews.last_mut() {
                review::withdraw(round, &by, stamp);
            }
            eco.state = EcoState::Cancelled;
            eco.cancelled_by = Some(by);
            eco.cancelled_at = Some(stamp);
            eco.modified_at = stamp;
            Ok(eco.id.clone())
        })?;
        Ok(self.eco_event(user, "cancelled", &id, None))
    }

    /// Add a comment to a change order's discussion.
    pub fn comment_on_eco(&self, user: &User, key: &str, body: &str, parent: &str) -> Result<(crate::model::Comment, Vec<String>), Error> {
        let (author, body, parent) = (user.clone(), body.to_string(), parent.to_string());
        let (comment, id) = self.mutate(move |state| {
            let eco = find_mut(state, key)?;
            let comment = review::push_comment(&mut eco.comments, &author, &body, &parent, now())?;
            Ok((comment, eco.id.clone()))
        })?;
        let warnings = self.eco_event(user, "comment", &id, Some(&comment));
        Ok((comment, warnings))
    }

    // -- release -------------------------------------------------------------

    /// Release an approved change order: every item applied in ONE store
    /// transaction, or none of them.
    pub fn release_eco(&self, user: &User, key: &str) -> Result<Vec<String>, Error> {
        if !user.can_checkin() {
            return Err(Error::forbidden("releasing a change order needs the check-in group"));
        }
        // On a snapshot first, so a hook is never asked about a release that
        // cannot happen.
        let eco = self.read(|state| find(state, key).cloned()).ok_or_else(|| Error::not_found("change order"))?;
        self.read(|state| check_releasable(state, &eco))?;
        let found = self.read(|state| problems(state, &eco));
        if !found.is_empty() {
            return Err(Error::conflict(format!("{} cannot release: {}", eco.number, found.join("; "))));
        }
        let eco_summary = json!({ "id": eco.id, "number": eco.number, "title": eco.title, "reason": eco.reason });
        let mut seen: Vec<(String, String, String)> = Vec::new();
        for item in eco.items.iter().filter(|i| i.action == EcoAction::Release) {
            let (part, revision) = self.snapshot(&item.part_id, &item.revision_id)?;
            let mut input = self.read(|state| db::release_input(state, &part, &revision, user));
            input["eco"] = eco_summary.clone();
            match scripting::run(self, user, scripting::BEFORE_RELEASE, "beforeRelease", &input)? {
                HookResult::Absent | HookResult::Returned(_) => {}
                HookResult::Refused(failure) => {
                    return Err(Error::conflict(format!(
                        "{} cannot release: {} refused {} rev {}: {}",
                        eco.number,
                        scripting::BEFORE_RELEASE,
                        part.number,
                        revision.label,
                        failure.message
                    )))
                }
            }
            seen.push((item.part_id.clone(), item.revision_id.clone(), revision.content_hash.clone()));
        }

        let (releaser, eco_id) = (user.id.clone(), eco.id.clone());
        self.mutate(move |state| {
            // Everything again, INSIDE the lock, against the state as it is.
            let eco = find(state, &eco_id).cloned().ok_or_else(|| Error::not_found("change order"))?;
            check_releasable(state, &eco)?;
            let found = problems(state, &eco);
            if !found.is_empty() {
                return Err(Error::conflict(format!("{} cannot release: {}", eco.number, found.join("; "))));
            }
            for (part_id, revision_id, hash) in &seen {
                let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
                let revision = part.revision(revision_id).ok_or_else(|| Error::not_found("revision"))?;
                if &revision.content_hash != hash {
                    return Err(Error::conflict(format!(
                        "{} rev {} changed while {} was checking it — release {} again",
                        part.number,
                        revision.label,
                        scripting::BEFORE_RELEASE,
                        eco.number
                    )));
                }
            }
            let stamp = now();
            // Releases first — each supersedes its part's older release —
            // then obsoletes, which may land on a revision just superseded.
            let mut replaced: Vec<(String, Option<String>)> = Vec::new();
            for item in eco.items.iter().filter(|i| i.action == EcoAction::Release) {
                let part = state.parts.get_mut(&item.part_id).ok_or_else(|| Error::not_found("part"))?;
                replaced.push((item.revision_id.clone(), part.current_release().map(|r| r.id.clone())));
                for other in part.revisions.iter_mut() {
                    if other.id != item.revision_id && other.lifecycle == Lifecycle::Released {
                        other.lifecycle = Lifecycle::Superseded;
                    }
                }
                let revision = part.revision_mut(&item.revision_id).ok_or_else(|| Error::not_found("revision"))?;
                lifecycle::check_transition(revision.lifecycle, Lifecycle::Released).map_err(Error::conflict)?;
                review::close_on_release(revision, &releaser, stamp);
                revision.lifecycle = Lifecycle::Released;
                revision.released_by = Some(releaser.clone());
                revision.released_at = Some(stamp);
                revision.lock = None;
            }
            for item in eco.items.iter().filter(|i| i.action == EcoAction::Obsolete) {
                let (number, revision) = db::find_revision_mut(state, &item.part_id, &item.revision_id)?;
                lifecycle::check_transition(revision.lifecycle, Lifecycle::Obsolete)
                    .map_err(|e| Error::conflict(format!("{number} rev {}: {e}", revision.label)))?;
                revision.lifecycle = Lifecycle::Obsolete;
            }
            // The index rows of every part the order moved (the feed names them).
            for item in eco.items.iter() {
                state.touch_part(&item.part_id);
            }
            let eco = find_mut(state, &eco_id)?;
            if let Some(round) = eco.reviews.last_mut() {
                round.closed_by = Some(releaser.clone());
                round.closed_at = Some(stamp);
            }
            for item in eco.items.iter_mut() {
                if let Some((_, before)) = replaced.iter().find(|(id, _)| *id == item.revision_id) {
                    item.replaced = before.clone();
                }
            }
            eco.state = EcoState::Released;
            eco.released_by = Some(releaser);
            eco.released_at = Some(stamp);
            eco.modified_at = stamp;
            Ok(())
        })?;

        let mut warnings = Vec::new();
        for item in eco.items.iter().filter(|i| i.action == EcoAction::Release) {
            let (part, revision) = self.snapshot(&item.part_id, &item.revision_id)?;
            let mut input = self.read(|state| db::release_input(state, &part, &revision, user));
            input["eco"] = eco_summary.clone();
            match scripting::run(self, user, scripting::AFTER_RELEASE, "afterRelease", &input) {
                Ok(HookResult::Absent) | Ok(HookResult::Returned(_)) => {}
                Ok(HookResult::Refused(failure)) => warnings.push(format!(
                    "{} rev {} released, but {} failed: {}",
                    part.number,
                    revision.label,
                    scripting::AFTER_RELEASE,
                    failure.message
                )),
                Err(error) => warnings.push(format!(
                    "{} rev {} released, but {} could not run: {}",
                    part.number,
                    revision.label,
                    scripting::AFTER_RELEASE,
                    error.message
                )),
            }
        }
        warnings.extend(self.eco_event(user, "released", &eco.id, None));
        Ok(warnings)
    }

    // -- reading -------------------------------------------------------------

    /// Change orders, newest first, optionally only those in `state_filter`.
    pub fn list_ecos(&self, state_filter: &str) -> Vec<EcoRow> {
        self.read(|state| {
            state
                .change_orders
                .iter()
                .rev()
                .filter(|e| match state_filter.trim() {
                    "" => true,
                    "open" => e.state.is_open(),
                    other => e.state.as_str() == other,
                })
                .map(|e| row(state, e))
                .collect()
        })
    }

    /// The open change orders a revision could be added to: Drafts.
    pub fn draft_ecos(&self) -> Vec<EcoRow> {
        self.list_ecos("draft")
    }

    /// One change order as its page shows it.
    pub fn eco_view(&self, viewer: &User, key: &str) -> Result<EcoView, Error> {
        let eco = self.read(|state| find(state, key).cloned()).ok_or_else(|| Error::not_found("change order"))?;
        // Catalog changes come from the audit log, outside the store lock.
        let mut attribute_changes: Vec<Vec<AttributeChange>> = Vec::new();
        for item in &eco.items {
            attribute_changes.push(if item.action == EcoAction::Release {
                // Before the release: since the part's current release. After
                // it: between the release it replaced and this one.
                let (since, until) = if eco.state == EcoState::Released {
                    let since = self.read(|state| {
                        let part = state.part(&item.part_id)?;
                        part.revision(item.replaced.as_deref()?)?.released_at
                    });
                    (since, eco.released_at)
                } else {
                    (self.read(|state| state.part(&item.part_id)?.current_release()?.released_at), None)
                };
                self.attribute_changes_between(&item.part_id, since, until)
            } else {
                Vec::new()
            });
        }
        self.read(|state| {
            let releasing = releasing(&eco);
            let items = eco
                .items
                .iter()
                .zip(attribute_changes)
                .map(|(item, changes)| item_view(state, &eco, item, &releasing, changes))
                .collect();
            let current = review::eco_subject_hash(state, &eco);
            let mut rounds: Vec<ReviewView> = eco.reviews.iter().map(|r| review::view(state, r, viewer, &current)).collect();
            let current = rounds.pop();
            rounds.reverse();
            Ok(EcoView {
                eco: row(state, &eco),
                description: eco.description.clone(),
                reason: eco.reason.clone(),
                released_by: eco.released_by.as_deref().map(|id| name_of(state, id)),
                cancelled_by: eco.cancelled_by.as_deref().map(|id| name_of(state, id)),
                cancelled_at: eco.cancelled_at,
                items,
                current,
                history: rounds,
                comments: review::comment_views(state, &eco.comments),
                rule: state.settings.eco_review_rule.clone(),
                problems: if eco.state.is_open() { problems(state, &eco) } else { Vec::new() },
                can_edit: viewer.can_author() && eco.state == EcoState::Draft,
                can_release: viewer.can_checkin() && eco.state == EcoState::Approved,
                can_comment: review::may_comment(viewer),
            })
        })
    }

    /// Catalog-value changes to a part between two times (from the start,
    /// or to now, when one is missing), from the audit log.
    fn attribute_changes_between(&self, part_id: &str, since: Option<u64>, until: Option<u64>) -> Vec<AttributeChange> {
        let filter = crate::audit::AuditFilter {
            kind: "part".into(),
            entity: part_id.to_string(),
            since,
            until,
            limit: Some(200),
            ..Default::default()
        };
        let events = self.audit_query(&filter).unwrap_or_default();
        let mut out = Vec::new();
        for event in events.into_iter().rev() {
            for field in ["attributes", "category"] {
                if let Some(change) = event.changes.get(field) {
                    out.push(AttributeChange {
                        at: event.at,
                        by: event.actor.username.clone(),
                        field: field.to_string(),
                        before: change.before.clone(),
                        after: change.after.clone(),
                    });
                }
            }
        }
        out
    }

    /// Tell the `review-event` hook about a change order's event.
    fn eco_event(&self, user: &User, event: &str, eco_id: &str, comment: Option<&crate::model::Comment>) -> Vec<String> {
        let input = self.read(|state| {
            let eco = find(state, eco_id)?;
            Some(json!({
                "event": event,
                "subject": {
                    "kind": "eco",
                    "id": eco.id,
                    "number": eco.number,
                    "title": eco.title,
                    "state": eco.state.as_str(),
                },
                "review": eco.reviews.last(),
                "comment": comment,
                "user": scripting::user_json(user),
            }))
        });
        match input {
            Some(input) => review::hook_warning(self, user, &input),
            None => Vec::new(),
        }
    }
}

/// A change order may release only when approved, with its rule still met by
/// approvals given on its items' documents as they are now (D8).
fn check_releasable(state: &State, eco: &ChangeOrder) -> Result<(), Error> {
    let current = review::eco_subject_hash(state, eco);
    if eco.state != EcoState::Approved {
        // Back in review because an item was saved after approval: say so.
        let stale = eco.reviews.last().filter(|r| r.is_live()).map(|r| review::stale_clause(r, &current)).unwrap_or_default();
        return Err(Error::conflict(format!(
            "{} is {} — a change order releases once it is approved{stale}",
            eco.number,
            eco.state.as_str()
        )));
    }
    match eco.reviews.last() {
        Some(round) if round.is_live() && round.is_met(&current) => {}
        Some(round) => {
            return Err(Error::conflict(format!(
                "{}'s review is not approved{}",
                eco.number,
                review::stale_clause(round, &current)
            )))
        }
        None => return Err(Error::conflict(format!("{}'s review is not approved", eco.number))),
    }
    if eco.items.is_empty() {
        return Err(Error::conflict(format!("{} names no revisions", eco.number)));
    }
    Ok(())
}

fn item_view(
    state: &State,
    eco: &ChangeOrder,
    item: &EcoItem,
    releasing: &BTreeSet<String>,
    attribute_changes: Vec<AttributeChange>,
) -> ItemView {
    let part = state.part(&item.part_id);
    let revision = part.and_then(|p| p.revision(&item.revision_id));
    // What a release replaces as current: once released, the revision it
    // recorded; before, the part's release now, unless that is the item.
    let replaces = match (part, item.action) {
        (Some(p), EcoAction::Release) if eco.state == EcoState::Released => {
            item.replaced.as_deref().and_then(|id| p.revision(id))
        }
        (Some(p), EcoAction::Release) => p.current_release().filter(|r| r.id != item.revision_id),
        _ => None,
    };
    let bom_diff = match (part, revision, replaces) {
        (Some(p), Some(r), Some(old)) if !r.uses.is_empty() || !old.uses.is_empty() => Some(bom::diff(state, p, old, r)),
        _ => None,
    };
    ItemView {
        part_id: item.part_id.clone(),
        number: part.map(|p| p.number.clone()).unwrap_or_default(),
        name: part.map(|p| p.name.clone()).unwrap_or_default(),
        revision_id: item.revision_id.clone(),
        label: revision.map(|r| r.label.clone()).unwrap_or_default(),
        lifecycle: revision.map(|r| r.lifecycle.as_str()).unwrap_or(""),
        action: item.action,
        note: item.note.clone(),
        replaces: replaces.map(|r| r.label.clone()),
        replaces_id: replaces.map(|r| r.id.clone()),
        document_changed: match (revision, replaces) {
            (Some(r), Some(old)) => Some(r.content_hash != old.content_hash),
            _ => None,
        },
        content_hash: revision.map(|r| r.content_hash.clone()).unwrap_or_default(),
        previous_hash: replaces.map(|r| r.content_hash.clone()).unwrap_or_default(),
        bom_diff,
        attribute_changes,
        problem: if eco.state.is_open() {
            item_problem(state, eco, item, releasing).unwrap_or_default()
        } else {
            String::new()
        },
    }
}
