//! Replace everywhere: swap one part (or one revision of it) for another in
//! every assembly that uses it, driven by where-used.
//!
//! # What it touches
//!
//! Only DIRECT parents — the revisions whose own uses list names the old part.
//! Grandparents use the parent, not the old part, and a parent's new revision
//! is theirs to pick up (or not) through their own references.
//!
//! By default every CURRENT parent revision is a candidate: one in work, or a
//! part's current release. Superseded and obsolete revisions are history and
//! are never rewritten. The caller may narrow the set to named revisions.
//!
//! # How each parent is written (replace-everywhere defaults)
//!
//! - **A parent revision in work** is edited in place, in one locked write. If
//!   another user holds its lock the row is refused; if nobody does, the write
//!   happens without taking the lock (it is a check-out, write and check-in in
//!   one transaction). A lock the caller holds stays held.
//! - **A released parent** gets a NEW revision (the suggested label), which
//!   starts from the release like any new revision and is left checked in.
//!   If that part already has a revision in work, the released row is SKIPPED:
//!   the revision in work is where the part is being changed, and it is a row
//!   of its own when it still uses the old part. Nothing is merged into it.
//!
//! # Which lines change
//!
//! A line naming the old part changes when it names the revision being
//! replaced — any revision, when none is given. A FLOATING line (no pinned
//! revision) changes only when the whole part is being replaced; replacing
//! one revision leaves floating lines alone, since they follow the part's
//! releases and never pointed at that revision in particular. The new line
//! keeps the quantity, unit, find number, reference and notes, and names the
//! replacement pinned to the revision asked for, or floating when none is.
//!
//! Each parent is its own write, so one refusal leaves the others done; the
//! report says what happened to each, like a family Generate. A dry run
//! computes the same report from the store as it stands and writes nothing.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::bom;
use crate::db::{now, Db, State};
use crate::eco::{NewEco, NewItem};
use crate::model::{Lifecycle, Part, Revision, Use, User};
use crate::Error;

/// What the caller asks for.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct ReplaceRequest {
    /// The revision of the OLD part to replace, by id or label. Empty: every
    /// revision of it (the whole part).
    #[serde(default)]
    pub from_revision: String,
    /// The replacement part, by id or number. Empty: the old part itself (a
    /// revision swap).
    #[serde(default)]
    pub to_part: String,
    /// The replacement's revision, by id or label. Empty: floating.
    #[serde(default)]
    pub to_revision: String,
    /// Parent revisions to touch, by readable part/revision document key. Empty: every current direct parent.
    #[serde(default)]
    pub parents: Vec<String>,
    /// Compute the report without writing.
    #[serde(default)]
    pub dry_run: bool,
    /// Put every written revision into a change order.
    #[serde(default)]
    pub eco: Option<EcoTarget>,
}

/// An existing change order by id or number, or a new one.
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum EcoTarget {
    Existing(String),
    New { new: NewEco },
}

/// One changed line: what it named, and what it will name.
#[derive(Debug, Clone, Serialize)]
pub struct LineChange {
    pub find_number: String,
    pub quantity: f64,
    pub unit: String,
    pub from: String,
    pub to: String,
}

/// What happened (or would happen) to one parent revision.
#[derive(Debug, Clone, Serialize)]
pub struct ReplaceRow {
    pub part_id: String,
    pub number: String,
    pub name: String,
    /// The parent revision where-used found.
    pub parent_revision_id: String,
    pub parent_revision: String,
    pub parent_state: String,
    /// `edit` (in place), `new-revision`, or empty when nothing is written.
    pub plan: String,
    /// `written`, `created`, `would-write`, `would-create`, `skipped`, `refused`.
    pub status: String,
    pub reason: String,
    /// The revision written (or to be written): the parent itself for an
    /// edit, the new one for a new revision.
    pub revision_id: String,
    pub revision: String,
    pub lines: Vec<LineChange>,
    /// Floating lines of the old part left alone.
    pub floating_left: usize,
    /// What adding the written revision to the change order did: `added`,
    /// `already in it`, or why it could not be added.
    pub eco: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct ReplaceReport {
    pub dry_run: bool,
    pub from_part: String,
    pub from_revision: String,
    pub to_part: String,
    pub to_revision: String,
    pub rows: Vec<ReplaceRow>,
    pub written: usize,
    pub skipped: usize,
    pub refused: usize,
    /// The change order the written revisions went into, if any.
    pub eco_number: String,
    pub eco_id: String,
}

/// The resolved request: ids, and the names a report prints.
struct Target {
    old_part: String,
    old_number: String,
    /// Empty: the whole part.
    from_revision: String,
    from_label: String,
    new_part: String,
    new_number: String,
    /// Empty: floating.
    to_revision: String,
    to_label: String,
}

fn resolve_target(state: &State, part_key: &str, request: &ReplaceRequest) -> Result<Target, Error> {
    let old = state.part_by_id_or_number(part_key).ok_or_else(|| Error::not_found("part"))?;
    let (from_revision, from_label) = if request.from_revision.trim().is_empty() {
        (String::new(), String::new())
    } else {
        let r = bom::find_revision(old, &request.from_revision)
            .ok_or_else(|| Error::bad_request(format!("{} has no revision '{}'", old.number, request.from_revision.trim())))?;
        (r.id.clone(), r.label.clone())
    };
    let new = if request.to_part.trim().is_empty() {
        old
    } else {
        state
            .part_by_id_or_number(request.to_part.trim())
            .ok_or_else(|| Error::bad_request(format!("there is no part '{}'", request.to_part.trim())))?
    };
    let (to_revision, to_label) = if request.to_revision.trim().is_empty() {
        (String::new(), String::new())
    } else {
        let r = bom::find_revision(new, &request.to_revision)
            .ok_or_else(|| Error::bad_request(format!("{} has no revision '{}'", new.number, request.to_revision.trim())))?;
        (r.id.clone(), r.label.clone())
    };
    if new.id == old.id && to_revision.is_empty() {
        return Err(Error::bad_request(format!(
            "name the revision of {} to use instead — replacing {} with itself, floating, changes nothing",
            old.number, old.number
        )));
    }
    if new.id == old.id && !from_revision.is_empty() && to_revision == from_revision {
        return Err(Error::bad_request("the replacement is the revision being replaced"));
    }
    Ok(Target {
        old_part: old.id.clone(),
        old_number: old.number.clone(),
        from_revision,
        from_label,
        new_part: new.id.clone(),
        new_number: new.number.clone(),
        to_revision,
        to_label,
    })
}

/// Whether `line` is one this replacement changes (see the module header).
fn changes(target: &Target, line: &Use) -> bool {
    if line.part != target.old_part {
        return false;
    }
    if target.from_revision.is_empty() {
        // The whole part: pinned and floating lines alike — except a line
        // that already names exactly the replacement.
        !(line.part == target.new_part && line.revision == target.to_revision)
    } else {
        line.revision == target.from_revision
    }
}

fn floating_left(target: &Target, revision: &Revision) -> usize {
    if target.from_revision.is_empty() {
        return 0;
    }
    revision.uses.iter().filter(|u| u.part == target.old_part && u.revision.is_empty()).count()
}

fn describe(state: &State, part_id: &str, revision_id: &str) -> String {
    let Some(part) = state.part(part_id) else { return part_id.to_string() };
    if revision_id.is_empty() {
        format!("{} (floating)", part.number)
    } else {
        let label = part.revision(revision_id).map(|r| r.label.clone()).unwrap_or_else(|| "(deleted)".into());
        format!("{} rev {label}", part.number)
    }
}

/// The swapped list as the JSON [`bom::check_uses`] reads, and the lines it changes.
fn swapped(state: &State, target: &Target, uses: &[Use]) -> (Value, Vec<LineChange>) {
    let mut out = Vec::with_capacity(uses.len());
    let mut changed = Vec::new();
    for line in uses {
        let (part, revision) = if changes(target, line) {
            changed.push(LineChange {
                find_number: line.find_number.clone(),
                quantity: line.quantity,
                unit: line.unit.clone(),
                from: describe(state, &line.part, &line.revision),
                to: describe(state, &target.new_part, &target.to_revision),
            });
            (target.new_part.clone(), target.to_revision.clone())
        } else {
            (line.part.clone(), line.revision.clone())
        };
        out.push(serde_json::json!({
            "part": part,
            "revision": revision,
            "quantity": line.quantity,
            "unit": line.unit,
            "find_number": line.find_number,
            "reference": line.reference,
            "notes": line.notes,
        }));
    }
    (Value::Array(out), changed)
}

/// Whether a parent revision is one the default set covers.
fn is_current(revision: &Revision) -> bool {
    revision.lifecycle.is_editable() || revision.lifecycle == Lifecycle::Released
}

/// Plan one parent row from the store as it stands. `None` for the revision
/// to write means the row is skipped or refused (the row says why).
fn plan_row(state: &State, user: &User, target: &Target, parent: &Part, revision: &Revision) -> ReplaceRow {
    let (list, lines) = swapped(state, target, &revision.uses);
    let mut row = ReplaceRow {
        part_id: parent.id.clone(),
        number: parent.number.clone(),
        name: parent.name.clone(),
        parent_revision_id: revision.id.clone(),
        parent_revision: revision.label.clone(),
        parent_state: revision.lifecycle.as_str().to_string(),
        plan: String::new(),
        status: String::new(),
        reason: String::new(),
        revision_id: String::new(),
        revision: String::new(),
        lines,
        floating_left: floating_left(target, revision),
        eco: String::new(),
    };
    let refuse = |row: &mut ReplaceRow, status: &str, reason: String| {
        row.status = status.to_string();
        row.reason = reason;
    };
    if row.lines.is_empty() {
        let why = if row.floating_left > 0 {
            format!("its only use of {} is floating, which follows the part's releases", target.old_number)
        } else {
            format!("it no longer uses {} {}", target.old_number, target.from_label).trim_end().to_string()
        };
        refuse(&mut row, "skipped", why);
        return row;
    }
    if !is_current(revision) {
        refuse(&mut row, "skipped", format!("revision {} is {} — history is not rewritten", revision.label, revision.lifecycle.as_str()));
        return row;
    }
    if revision.lifecycle.is_editable() {
        row.plan = "edit".into();
        row.revision_id = revision.id.clone();
        row.revision = revision.label.clone();
        if let Some(lock) = revision.lock.as_ref().filter(|l| l.user_id != user.id) {
            let holder = state.user(&lock.user_id).map(|u| u.username.clone()).unwrap_or_else(|| "another user".into());
            refuse(&mut row, "refused", format!("revision {} is checked out by {holder}", revision.label));
            return row;
        }
    } else {
        // Released: a new revision, unless the part is already being changed.
        if let Some(open) = parent.open_draft() {
            refuse(
                &mut row,
                "skipped",
                format!("revision {} is already in work — the part is changed there", open.label),
            );
            return row;
        }
        row.plan = "new-revision".into();
        row.revision = crate::db::suggest_label(parent);
    }
    if let Err(error) = bom::check_uses(state, &parent.id, &list) {
        refuse(&mut row, "refused", error.message);
        return row;
    }
    row.status = if row.plan == "edit" { "would-write" } else { "would-create" }.into();
    row
}

impl Db {
    /// Replace everywhere (see the module header).
    pub fn replace_everywhere(&self, user: &User, part_key: &str, request: &ReplaceRequest) -> Result<ReplaceReport, Error> {
        if !user.can_author() {
            return Err(Error::forbidden("replacing a part in assemblies needs the author group"));
        }
        let (target, plans) = self.read(|state| -> Result<_, Error> {
            let target = resolve_target(state, part_key, request)?;
            let old = state.part(&target.old_part).ok_or_else(|| Error::not_found("part"))?;
            let wanted: Vec<&str> = request.parents.iter().map(|p| p.trim()).filter(|p| !p.is_empty()).collect();
            let from = old.revision(&target.from_revision);
            let found = bom::where_used(state, old, from, 1);
            let mut plans = Vec::new();
            let mut seen = std::collections::BTreeSet::new();
            for line in &found.lines {
                let key = crate::identity::document_key(&line.part_id, &line.revision_id);
                if !seen.insert(key.clone()) {
                    continue;
                }
                if !wanted.is_empty() && !wanted.contains(&key.as_str()) {
                    continue;
                }
                if wanted.is_empty() && !line.current {
                    continue;
                }
                let parent = state.part(&line.part_id).ok_or_else(|| Error::not_found("part"))?;
                let revision = parent.revision(&line.revision_id).ok_or_else(|| Error::not_found("revision"))?;
                plans.push(plan_row(state, user, &target, parent, revision));
            }
            // A floating use of one revision is not found by a revision-scoped
            // where-used when it resolves elsewhere; name the ones asked for.
            for id in &wanted {
                if seen.contains(*id) {
                    continue;
                }
                return Err(Error::bad_request(format!("revision '{id}' does not use {}", target.old_number)));
            }
            Ok((target, plans))
        })?;

        let mut report = ReplaceReport {
            dry_run: request.dry_run,
            from_part: target.old_number.clone(),
            from_revision: target.from_label.clone(),
            to_part: target.new_number.clone(),
            to_revision: target.to_label.clone(),
            rows: Vec::new(),
            written: 0,
            skipped: 0,
            refused: 0,
            eco_number: String::new(),
            eco_id: String::new(),
        };
        if request.dry_run {
            for row in plans {
                match row.status.as_str() {
                    "skipped" => report.skipped += 1,
                    "refused" => report.refused += 1,
                    _ => {}
                }
                report.rows.push(row);
            }
            return Ok(report);
        }

        // The change order first, so a bad one refuses before anything is
        // written.
        let eco = match &request.eco {
            None => None,
            Some(EcoTarget::Existing(key)) => {
                let (id, number) = self.read(|state| {
                    crate::eco::find(state, key.trim())
                        .map(|e| (e.id.clone(), e.number.clone()))
                        .ok_or_else(|| Error::not_found(format!("change order '{}'", key.trim())))
                })?;
                Some((id, number))
            }
            Some(EcoTarget::New { new }) => {
                let eco = self.create_eco(user, new)?;
                Some((eco.id, eco.number))
            }
        };
        if let Some((id, number)) = &eco {
            report.eco_id = id.clone();
            report.eco_number = number.clone();
        }

        for mut row in plans {
            if row.status == "would-write" || row.status == "would-create" {
                let result = if row.plan == "edit" {
                    self.replace_in(user, &target, &row.part_id, &row.revision_id).map(|lines| {
                        row.status = "written".into();
                        row.lines = lines;
                    })
                } else {
                    self.replace_in_new_revision(user, &target, &row.part_id, &row.parent_revision_id).map(
                        |(revision, lines)| {
                            row.status = "created".into();
                            row.revision_id = revision.id;
                            row.revision = revision.label;
                            row.lines = lines;
                        },
                    )
                };
                if let Err(error) = result {
                    row.status = "refused".into();
                    row.reason = error.message;
                }
            }
            match row.status.as_str() {
                "written" | "created" => {
                    report.written += 1;
                    if let Some((eco_id, _)) = &eco {
                        row.eco = self.replace_into_eco(user, eco_id, &row);
                    }
                }
                "skipped" => report.skipped += 1,
                _ => report.refused += 1,
            }
            report.rows.push(row);
        }
        Ok(report)
    }

    /// Swap the lines in a revision in work, in one locked write. The checks
    /// the plan made are made again here: the store may have moved.
    fn replace_in(&self, user: &User, target: &Target, part_id: &str, revision_id: &str) -> Result<Vec<LineChange>, Error> {
        let user_id = user.id.clone();
        self.mutate(|state| {
            let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
            let revision = part.revision(revision_id).ok_or_else(|| Error::not_found("revision"))?;
            if !revision.lifecycle.is_editable() {
                return Err(Error::conflict(format!(
                    "revision {} is now {}",
                    revision.label,
                    revision.lifecycle.as_str()
                )));
            }
            if let Some(lock) = revision.lock.as_ref().filter(|l| l.user_id != user_id) {
                let holder = state.user(&lock.user_id).map(|u| u.username.clone()).unwrap_or_else(|| "another user".into());
                return Err(Error::conflict(format!("revision {} is checked out by {holder}", revision.label)));
            }
            let (list, lines) = swapped(state, target, &revision.uses);
            if lines.is_empty() {
                return Err(Error::conflict(format!("revision {} no longer uses {}", revision.label, target.old_number)));
            }
            let uses = bom::check_uses(state, part_id, &list)?;
            let (_, revision) = crate::db::find_revision_mut(state, part_id, revision_id)?;
            revision.uses = uses;
            revision.modified_at = now();
            Ok(lines)
        })
    }

    /// Start a new revision of a released parent and swap the lines in it. If
    /// the swap is refused, the new revision is deleted again, so a refused
    /// row leaves nothing behind.
    fn replace_in_new_revision(
        &self,
        user: &User,
        target: &Target,
        part_id: &str,
        released_id: &str,
    ) -> Result<(Revision, Vec<LineChange>), Error> {
        // The plan saw no revision in work; one may have started since.
        self.read(|state| {
            let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
            if let Some(open) = part.open_draft() {
                return Err(Error::conflict(format!("revision {} is already in work — the part is changed there", open.label)));
            }
            if part.current_release().map(|r| r.id.as_str()) != Some(released_id) {
                return Err(Error::conflict("the part's current release changed meanwhile"));
            }
            Ok(())
        })?;
        let revision = self.create_revision(user, part_id)?;
        match self.replace_in(user, target, part_id, &revision.id) {
            Ok(lines) => Ok((revision, lines)),
            Err(error) => {
                if let Err(undo) = self.delete_draft(user, part_id, &revision.id) {
                    eprintln!("brep-plm: replace: could not remove the new revision after a refusal: {}", undo.message);
                }
                Err(error)
            }
        }
    }

    /// Add a written revision to the change order. A refusal here does not
    /// undo the write; the row says why it is not in the change order.
    fn replace_into_eco(&self, user: &User, eco_id: &str, row: &ReplaceRow) -> String {
        let already = self.read(|state| {
            crate::eco::find(state, eco_id).is_some_and(|e| e.item(&row.part_id, &row.revision_id).is_some())
        });
        if already {
            return "already in it".into();
        }
        let item = NewItem {
            part: row.part_id.clone(),
            revision: row.revision_id.clone(),
            action: "release".into(),
            note: String::new(),
        };
        match self.add_eco_item(user, eco_id, &item) {
            Ok(_) => "added".into(),
            Err(error) => error.message,
        }
    }
}
