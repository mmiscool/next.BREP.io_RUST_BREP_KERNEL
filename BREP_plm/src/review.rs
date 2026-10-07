//! Review and approval (round 9 item 4).
//!
//! Moving a revision to `InReview` opens a review round ([`Review`]) on it:
//! who may decide, how many approvals the release needs, an optional due date.
//! Reviewers approve, or reject with a comment; a rejection sends the revision
//! back to Draft and ends the round, and the author submits again for a new
//! one. Release is refused until the round's rule is met. Anyone who can read
//! a part reads its discussion ([`Comment`]).
//!
//! # The rule
//!
//! The rule comes from the settings ([`Settings::review_rule`]) unless an
//! override covers the part ([`Settings::review_overrides`]): the NEAREST
//! category with one — the part's own, then its parent's, up to the root —
//! then the part's type. The default needs 0 approvals, so until an
//! administrator sets a rule a draft releases exactly as it did before
//! reviews existed, and a submitted revision's round is approved the moment
//! it opens.
//!
//! The rule is COPIED into the round when it opens. Changing the settings
//! changes the next round, never one under way; an administrator or the
//! check-in group can change a round's reviewers, required approvals and due
//! date directly.
//!
//! # Hooks
//!
//! * `reviewers.js` — `reviewers(input)` runs when a revision is submitted,
//!   outside the store lock, and may return `{ reviewers, required_approvals,
//!   due, allow_self_approval }` to replace any of them, `null` for no opinion,
//!   or throw to refuse the submission.
//! * `review-event.js` — `reviewEvent(input)` runs after every review event
//!   commits (opened, approve, reject, withdrawn, comment, updated). It
//!   cannot undo anything: a throw comes back as a warning. This is the
//!   notification hook — a webhook, a chat message, an email relay — since
//!   scripts may make HTTP requests.
//!
//! The release gate itself stays in `beforeRelease`, which sees the revision
//! and so its `reviews`.

use serde::Serialize;
use serde_json::{json, Value};

use crate::catalog;
use crate::db::{self, now, Db, State};
use crate::lifecycle;
use crate::model::{
    groups, Comment, Decision, Lifecycle, Part, Review, ReviewOverride, ReviewRule, ReviewStatus, ReviewerKind,
    ReviewerRef, Revision, Settings, User, Verdict,
};
use crate::scripting::{self, HookResult};
use crate::Error;

/// An override for a part type.
pub const SCOPE_PART_TYPE: &str = "part_type";
/// An override for a category and everything under it.
pub const SCOPE_CATEGORY: &str = "category";

/// The longest comment the server keeps.
pub const MAX_COMMENT: usize = 10_000;

/// The most approvals a rule may ask for — more is a typo.
pub const MAX_APPROVALS: u32 = 50;

// ===========================================================================
// The rule
// ===========================================================================

/// The rule for `part`, and what decided it: `category:<id>`,
/// `part-type:<id>` or `settings`.
pub fn rule_for(settings: &Settings, categories: &[crate::model::Category], part: &Part) -> (ReviewRule, String) {
    let find = |scope: &str, id: &str| -> Option<&ReviewOverride> {
        settings
            .review_overrides
            .iter()
            .find(|o| o.scope == scope && o.id.eq_ignore_ascii_case(id))
    };
    if let Ok(chain) = catalog::chain(categories, &part.category) {
        // Root first; the nearest category wins, so walk it backwards.
        for category in chain.iter().rev() {
            if let Some(found) = find(SCOPE_CATEGORY, &category.id) {
                return (found.rule.clone(), format!("category:{}", category.id));
            }
        }
    }
    if let Some(found) = find(SCOPE_PART_TYPE, &part.part_type) {
        return (found.rule.clone(), format!("part-type:{}", part.part_type));
    }
    (settings.review_rule.clone(), "settings".to_string())
}

/// A new round from `rule`, opened by `by`.
pub fn open_round(rule: &ReviewRule, source: &str, by: &str, at: u64) -> Review {
    let mut review = Review {
        id: crate::auth::new_id(),
        status: ReviewStatus::Open,
        opened_by: by.to_string(),
        opened_at: at,
        reviewers: rule.reviewers.clone(),
        required_approvals: rule.required_approvals,
        allow_self_approval: rule.allow_self_approval,
        due: (rule.due_days > 0).then(|| at + u64::from(rule.due_days) * 86_400),
        rule_source: source.to_string(),
        decisions: Vec::new(),
        closed_by: None,
        closed_at: None,
    };
    // No decisions yet: only a rule needing none is met, whatever the hash.
    refresh(&mut review, "");
    review
}

/// Set a live round's status from its decisions, judged on the subject as it
/// is now (`current`, its content hash): a rejection rejects it, a rule met
/// by current approvals approves it, and otherwise it is open. A round
/// approved on a document that has since been saved goes back to open.
pub fn refresh(review: &mut Review, current: &str) {
    if !review.is_live() {
        return;
    }
    review.status = if review.decisions.iter().any(|d| d.verdict == Verdict::Reject) {
        ReviewStatus::Rejected
    } else if review.is_met(current) {
        ReviewStatus::Approved
    } else {
        ReviewStatus::Open
    };
}

/// What a change order's round decides on: every item's revision and its
/// document's content hash, in a stable order, hashed. Saving any item's
/// document — or changing the items — changes it, so approvals given before
/// go stale as a revision's do.
pub fn eco_subject_hash(state: &State, eco: &crate::model::ChangeOrder) -> String {
    let mut lines: Vec<String> = eco
        .items
        .iter()
        .map(|item| {
            let hash = state
                .part(&item.part_id)
                .and_then(|p| p.revision(&item.revision_id))
                .map(|r| r.content_hash.clone())
                .unwrap_or_default();
            serde_json::json!([item.part_id, item.revision_id, hash]).to_string()
        })
        .collect();
    lines.sort();
    crate::auth::content_hash(&lines.join("\n"))
}

/// A revision's document hash changed (a save, a bake result, a generated
/// document): re-judge its live round, and every live change-order round
/// naming it, on what the reviewers would now be approving. A change order
/// whose round was approved goes back to in review until its approvals are
/// renewed (and forward again if the document returns to what they saw).
pub fn document_changed(state: &mut State, part_id: &str, revision_id: &str) {
    if let Some(revision) = state.parts.get_mut(part_id).and_then(|p| p.revision_mut(revision_id)) {
        let hash = revision.content_hash.clone();
        if let Some(review) = revision.live_review_mut() {
            refresh(review, &hash);
        }
    }
    let affected: Vec<usize> = state
        .change_orders
        .iter()
        .enumerate()
        .filter(|(_, eco)| eco.state.is_open() && eco.items.iter().any(|i| i.part_id == part_id && i.revision_id == revision_id))
        .filter(|(_, eco)| eco.reviews.last().is_some_and(|r| r.is_live()))
        .map(|(i, _)| i)
        .collect();
    for index in affected {
        let hash = eco_subject_hash(state, &state.change_orders[index]);
        let eco = &mut state.change_orders[index];
        if let Some(round) = eco.reviews.last_mut() {
            refresh(round, &hash);
            if matches!(eco.state, crate::model::EcoState::Approved | crate::model::EcoState::InReview) {
                eco.state = if round.status == ReviewStatus::Approved {
                    crate::model::EcoState::Approved
                } else {
                    crate::model::EcoState::InReview
                };
            }
        }
    }
}

/// Whether `user` is one of the round's reviewers — named, in a named group,
/// or, when it names nobody, in the check-in group. Says nothing about
/// whether their approval would COUNT ([`Review::approvers`]).
pub fn is_reviewer(review: &Review, user: &User) -> bool {
    if !user.active {
        return false;
    }
    if review.reviewers.is_empty() {
        return user.can_checkin();
    }
    review.reviewers.iter().any(|r| match r.kind {
        ReviewerKind::User => r.id == user.id,
        ReviewerKind::Group => user.in_group(&r.id),
    })
}

/// Whether `user` may take part in a discussion: signed in, and more than a
/// viewer or a bake worker.
pub fn may_comment(user: &User) -> bool {
    user.active && user.groups.iter().any(|g| g != groups::VIEWER && g != groups::WORKER)
}

/// Reviewers as the API and scripts give them — `"user:ada"`,
/// `"group:quality"`, a bare username, or `{ kind, id }` with a user's id or
/// username — as refs to users that exist and groups by name.
pub fn parse_reviewers(state: &State, value: &Value) -> Result<Vec<ReviewerRef>, Error> {
    let Some(items) = value.as_array() else {
        return Err(Error::bad_request("reviewers must be a list, like [\"user:ada\", \"group:quality\"]"));
    };
    let mut out: Vec<ReviewerRef> = Vec::new();
    for item in items {
        let (kind, id) = match item {
            Value::String(text) => match text.trim().split_once(':') {
                Some((kind, id)) => (kind.trim().to_ascii_lowercase(), id.trim().to_string()),
                None => ("user".to_string(), text.trim().to_string()),
            },
            Value::Object(map) => (
                map.get("kind").and_then(Value::as_str).unwrap_or("user").trim().to_ascii_lowercase(),
                map.get("id").and_then(Value::as_str).unwrap_or("").trim().to_string(),
            ),
            other => return Err(Error::bad_request(format!("{other} is not a reviewer"))),
        };
        if id.is_empty() {
            return Err(Error::bad_request("a reviewer needs a user or a group"));
        }
        let reviewer = match kind.as_str() {
            "user" => {
                let user = state
                    .user(&id)
                    .or_else(|| state.user_by_name(&id))
                    .ok_or_else(|| Error::bad_request(format!("there is no user '{id}'")))?;
                ReviewerRef { kind: ReviewerKind::User, id: user.id.clone() }
            }
            "group" => {
                db::check_label(&id).map_err(|_| Error::bad_request(format!("'{id}' is not a group name")))?;
                ReviewerRef { kind: ReviewerKind::Group, id }
            }
            other => return Err(Error::bad_request(format!("'{other}' is not a reviewer kind — use user or group"))),
        };
        if !out.contains(&reviewer) {
            out.push(reviewer);
        }
    }
    Ok(out)
}

/// Rewrite the reviewer lists in a settings object as `{ kind, id }`, so a
/// rule may be written with `"user:ada"` and `"group:quality"` shorthands.
pub fn normalize_settings_json(state: &State, settings: &mut Value) -> Result<(), Error> {
    let mut rules: Vec<&mut Value> = Vec::new();
    let Some(object) = settings.as_object_mut() else { return Ok(()) };
    let mut overrides = None;
    for (key, value) in object.iter_mut() {
        match key.as_str() {
            "review_rule" | "eco_review_rule" => rules.push(value),
            "review_overrides" => overrides = Some(value),
            _ => {}
        }
    }
    if let Some(Value::Array(items)) = overrides {
        rules.extend(items.iter_mut().filter_map(|o| o.get_mut("rule")));
    }
    for rule in rules {
        if let Some(reviewers) = rule.get_mut("reviewers") {
            let parsed = parse_reviewers(state, reviewers)?;
            *reviewers = serde_json::to_value(parsed).map_err(Error::internal)?;
        }
    }
    Ok(())
}

/// Check a rule and resolve its user reviewers to ids, so a rule typed with
/// usernames is stored with ids and survives a rename.
pub fn check_rule(state: &State, rule: &mut ReviewRule) -> Result<(), Error> {
    if rule.required_approvals > MAX_APPROVALS {
        return Err(Error::bad_request(format!("a review needs at most {MAX_APPROVALS} approvals")));
    }
    rule.reviewers = parse_reviewers(state, &serde_json::to_value(&rule.reviewers).map_err(Error::internal)?)?;
    Ok(())
}

/// Check the settings' review rule and overrides against the store, as
/// [`Db::update_settings`] stores them.
pub fn check_settings(state: &State, settings: &mut Settings) -> Result<(), Error> {
    check_rule(state, &mut settings.review_rule)?;
    check_rule(state, &mut settings.eco_review_rule)?;
    let mut seen: Vec<(String, String)> = Vec::new();
    for o in &mut settings.review_overrides {
        o.id = o.id.trim().to_string();
        match o.scope.as_str() {
            SCOPE_PART_TYPE => {
                let kind = state
                    .part_types
                    .iter()
                    .find(|t| t.id.eq_ignore_ascii_case(&o.id))
                    .ok_or_else(|| Error::bad_request(format!("a review override names part type '{}', which does not exist", o.id)))?;
                o.id = kind.id.clone();
            }
            SCOPE_CATEGORY => {
                let category = catalog::find(&state.categories, &o.id)
                    .ok_or_else(|| Error::bad_request(format!("a review override names category '{}', which does not exist", o.id)))?;
                o.id = category.id.clone();
            }
            other => {
                return Err(Error::bad_request(format!(
                    "a review override's scope is part_type or category, not '{other}'"
                )))
            }
        }
        let key = (o.scope.clone(), o.id.to_ascii_lowercase());
        if seen.contains(&key) {
            return Err(Error::bad_request(format!("two review overrides name {} '{}'", o.scope, o.id)));
        }
        seen.push(key);
        check_rule(state, &mut o.rule)?;
    }
    Ok(())
}

/// Refuse a release that the revision's review does not allow, or say
/// nothing. `rule` is the rule for the part as it stands.
pub fn release_gate(revision: &Revision, rule: &ReviewRule, number: &str) -> Result<(), Error> {
    let current = revision.content_hash.as_str();
    match revision.reviews.last().filter(|r| r.is_live()) {
        Some(review) if review.is_met(current) => Ok(()),
        Some(review) => Err(Error::conflict(format!(
            "{number} rev {} needs {} approval(s) before it can release, and has {}{}",
            revision.label,
            review.required_approvals,
            review.approvers(current).len(),
            stale_clause(review, current)
        ))),
        None if rule.required_approvals > 0 => Err(Error::conflict(format!(
            "{number} rev {} needs {} approval(s) before it can release — submit it for review first",
            revision.label, rule.required_approvals
        ))),
        None => Ok(()),
    }
}

/// Who may submit a revision for review (operator ruling, 2026-09-25): the user holding
/// its lock, or any author while nobody does. A revision someone else has checked out is
/// theirs to submit — they may be about to save — and the refusal names them.
pub fn submit_lock_gate(state: &State, user: &User, number: &str, revision: &Revision) -> Result<(), Error> {
    match revision.lock.as_ref().filter(|lock| lock.user_id != user.id) {
        Some(lock) => {
            let holder = state.user(&lock.user_id).map(|u| u.username.clone()).unwrap_or_else(|| "another user".into());
            Err(Error::conflict(format!(
                "{number} rev {} is checked out by {holder} — only {holder} can submit it for review (or check it in first)",
                revision.label
            )))
        }
        None => Ok(()),
    }
}

/// `" — N approval(s) were given on an older version and need renewing"`
/// when a round has approvals that no longer count, else nothing: the tail
/// of a release refusal, so the person releasing knows why approvals they
/// can see do not count.
pub fn stale_clause(review: &Review, current: &str) -> String {
    match review.stale_approvers(current).len() {
        0 => String::new(),
        n => format!(" — {n} approval(s) were given on an older version and need renewing"),
    }
}

/// End the live round of a revision that just released.
pub fn close_on_release(revision: &mut Revision, by: &str, at: u64) {
    if let Some(review) = revision.live_review_mut() {
        review.closed_by = Some(by.to_string());
        review.closed_at = Some(at);
    }
}

/// End the live round of a revision pulled back to Draft by its author.
pub fn withdraw(review: &mut Review, by: &str, at: u64) {
    if review.is_live() {
        review.status = ReviewStatus::Withdrawn;
        review.closed_by = Some(by.to_string());
        review.closed_at = Some(at);
    }
}

/// Record `user`'s verdict on a live round, with every rule a decision is
/// held to. The caller moves the subject back to Draft on a rejection.
/// `current` is the subject's content hash now: the decision is recorded as
/// given on it, and counts only while it stays the same (D8).
pub fn record_decision(
    review: &mut Review,
    user: &User,
    verdict: Verdict,
    comment: &str,
    at: u64,
    current: &str,
) -> Result<(), Error> {
    if !review.is_live() {
        return Err(Error::conflict("this review is over — submit again for a new one"));
    }
    if !is_reviewer(review, user) {
        return Err(Error::forbidden(if review.reviewers.is_empty() {
            "this review is decided by the check-in group".to_string()
        } else {
            "you are not one of this review's reviewers".to_string()
        }));
    }
    if user.id == review.opened_by && !review.allow_self_approval && verdict == Verdict::Approve {
        return Err(Error::forbidden(
            "you submitted this for review, and your own approval does not count here",
        ));
    }
    let comment = comment.trim();
    if verdict == Verdict::Reject && comment.is_empty() {
        return Err(Error::bad_request("a rejection needs a comment saying what to change"));
    }
    if comment.chars().count() > MAX_COMMENT {
        return Err(Error::bad_request(format!("a comment is at most {MAX_COMMENT} characters")));
    }
    // Approving again is refused only when the standing approval is on THIS
    // version: renewing a stale approval is exactly what is wanted.
    let latest = review.decisions.iter().rev().find(|d| d.user_id == user.id);
    if verdict == Verdict::Approve && latest.is_some_and(|d| d.verdict == Verdict::Approve && d.is_current(current)) {
        return Err(Error::conflict("you have already approved this version"));
    }
    review.decisions.push(Decision {
        user_id: user.id.clone(),
        verdict,
        comment: comment.to_string(),
        at,
        content_hash: Some(current.to_string()),
    });
    refresh(review, current);
    if verdict == Verdict::Reject {
        review.closed_by = Some(user.id.clone());
        review.closed_at = Some(at);
    }
    Ok(())
}

/// Add a comment to `comments`, answering `parent` when it is given.
pub fn push_comment(comments: &mut Vec<Comment>, user: &User, body: &str, parent: &str, at: u64) -> Result<Comment, Error> {
    if !may_comment(user) {
        return Err(Error::forbidden("commenting needs more than the viewer group"));
    }
    let body = body.trim();
    if body.is_empty() {
        return Err(Error::bad_request("a comment needs some text"));
    }
    if body.chars().count() > MAX_COMMENT {
        return Err(Error::bad_request(format!("a comment is at most {MAX_COMMENT} characters")));
    }
    let parent = parent.trim();
    if !parent.is_empty() && !comments.iter().any(|c| c.id == parent) {
        return Err(Error::not_found("the comment being answered"));
    }
    let comment = Comment {
        id: crate::auth::new_id(),
        author: user.id.clone(),
        at,
        body: body.to_string(),
        parent: parent.to_string(),
    };
    comments.push(comment.clone());
    Ok(comment)
}

/// What a submitter, an administrator or the check-in group may change on
/// a live round.
#[derive(Debug, Clone, Default)]
pub struct RoundChange {
    pub reviewers: Option<Value>,
    pub required_approvals: Option<u32>,
    /// Unix seconds; 0 clears it.
    pub due: Option<u64>,
}

/// Apply `change` to a live round, as `user`; `current` is the subject's
/// content hash, which the round's status is re-judged on.
pub fn change_round(state: &State, review: &mut Review, user: &User, change: &RoundChange, current: &str) -> Result<(), Error> {
    if !review.is_live() {
        return Err(Error::conflict("this review is over"));
    }
    let manager = user.can_checkin();
    if !manager && user.id != review.opened_by {
        return Err(Error::forbidden("a review is changed by its submitter or the check-in group"));
    }
    if let Some(required) = change.required_approvals {
        if !manager {
            return Err(Error::forbidden("changing how many approvals a review needs takes the check-in group"));
        }
        if required > MAX_APPROVALS {
            return Err(Error::bad_request(format!("a review needs at most {MAX_APPROVALS} approvals")));
        }
        review.required_approvals = required;
    }
    if let Some(reviewers) = &change.reviewers {
        review.reviewers = parse_reviewers(state, reviewers)?;
    }
    if let Some(due) = change.due {
        review.due = (due > 0).then_some(due);
    }
    refresh(review, current);
    Ok(())
}

// ===========================================================================
// What the page shows
// ===========================================================================

/// A reviewer with their name, and what they have said.
#[derive(Debug, Serialize)]
pub struct ReviewerView {
    pub kind: ReviewerKind,
    pub id: String,
    pub name: String,
    /// For a user: `approve`, `reject` or empty. For a group: the members
    /// who approved.
    pub verdict: String,
    pub approved_by: Vec<String>,
}

/// A decision with the reviewer's name.
#[derive(Debug, Serialize)]
pub struct DecisionView {
    pub user_id: String,
    pub name: String,
    pub verdict: Verdict,
    pub comment: String,
    pub at: u64,
    /// The subject's content hash it was given on; `null` for a decision
    /// recorded before decisions carried one.
    pub content_hash: Option<String>,
    /// Given on the subject as it is now. `false`: "on an older version" —
    /// an approval that no longer counts until it is renewed.
    pub current: bool,
}

/// A review round as the page shows it.
#[derive(Debug, Serialize)]
pub struct ReviewView {
    pub id: String,
    pub status: &'static str,
    pub opened_by: String,
    pub opened_at: u64,
    pub required_approvals: u32,
    /// Approvals that count: given on the subject as it is now.
    pub approvals: usize,
    /// Standing approvals given on an older version (or before decisions
    /// carried a hash): they count again only when renewed.
    pub stale_approvals: usize,
    pub met: bool,
    pub allow_self_approval: bool,
    pub due: Option<u64>,
    pub overdue: bool,
    pub rule_source: String,
    pub reviewers: Vec<ReviewerView>,
    pub decisions: Vec<DecisionView>,
    pub closed_by: Option<String>,
    pub closed_at: Option<u64>,
    /// Whether the person looking may approve or reject it now.
    pub can_decide: bool,
    /// Whether they may change its reviewers, due date (and, for the
    /// check-in group, required approvals).
    pub can_change: bool,
    pub live: bool,
}

/// A comment with its author's name.
#[derive(Debug, Serialize)]
pub struct CommentView {
    pub id: String,
    pub author: String,
    pub author_id: String,
    pub at: u64,
    pub body: String,
    pub parent: String,
}

fn name_of(state: &State, id: &str) -> String {
    state
        .user(id)
        .map(|u| if u.display_name.is_empty() { u.username.clone() } else { u.display_name.clone() })
        .unwrap_or_else(|| id.to_string())
}

/// A round as the page shows it; `current` is the subject's content hash now,
/// which decides which approvals count and which are on an older version.
pub fn view(state: &State, review: &Review, viewer: &User, current: &str) -> ReviewView {
    let approvers = review.approvers(current);
    let latest = |user_id: &str| -> String {
        review
            .decisions
            .iter()
            .rev()
            .find(|d| d.user_id == user_id)
            .map(|d| d.verdict.as_str().to_string())
            .unwrap_or_default()
    };
    let reviewers = review
        .reviewers
        .iter()
        .map(|r| match r.kind {
            ReviewerKind::User => ReviewerView {
                kind: r.kind,
                id: r.id.clone(),
                name: name_of(state, &r.id),
                verdict: latest(&r.id),
                approved_by: Vec::new(),
            },
            ReviewerKind::Group => ReviewerView {
                kind: r.kind,
                id: r.id.clone(),
                name: r.id.clone(),
                verdict: String::new(),
                approved_by: approvers
                    .iter()
                    .filter(|id| state.user(id).is_some_and(|u| u.in_group(&r.id)))
                    .map(|id| name_of(state, id))
                    .collect(),
            },
        })
        .collect();
    let live = review.is_live();
    // An approval on an older version does not stop them deciding again.
    let already_approved = review
        .decisions
        .iter()
        .rev()
        .find(|d| d.user_id == viewer.id)
        .is_some_and(|d| d.verdict == Verdict::Approve && d.is_current(current));
    ReviewView {
        id: review.id.clone(),
        status: review.status.as_str(),
        opened_by: name_of(state, &review.opened_by),
        opened_at: review.opened_at,
        required_approvals: review.required_approvals,
        approvals: approvers.len(),
        stale_approvals: review.stale_approvers(current).len(),
        met: review.is_met(current),
        allow_self_approval: review.allow_self_approval,
        due: review.due,
        overdue: live && review.due.is_some_and(|due| due < now()),
        rule_source: review.rule_source.clone(),
        reviewers,
        decisions: review
            .decisions
            .iter()
            .map(|d| DecisionView {
                user_id: d.user_id.clone(),
                name: name_of(state, &d.user_id),
                verdict: d.verdict,
                comment: d.comment.clone(),
                at: d.at,
                content_hash: d.content_hash.clone(),
                current: d.is_current(current),
            })
            .collect(),
        closed_by: review.closed_by.as_deref().map(|id| name_of(state, id)),
        closed_at: review.closed_at,
        can_decide: live && is_reviewer(review, viewer) && !already_approved,
        can_change: live && (viewer.can_checkin() || viewer.id == review.opened_by),
        live,
    }
}

pub fn comment_views(state: &State, comments: &[Comment]) -> Vec<CommentView> {
    comments
        .iter()
        .map(|c| CommentView {
            id: c.id.clone(),
            author: name_of(state, &c.author),
            author_id: c.author.clone(),
            at: c.at,
            body: c.body.clone(),
            parent: c.parent.clone(),
        })
        .collect()
}

/// One revision's review panel: the current round, the ones before it, the
/// discussion, and the rule a new round would open with.
#[derive(Debug, Serialize)]
pub struct RevisionReview {
    pub part_id: String,
    pub number: String,
    pub revision_id: String,
    pub label: String,
    pub lifecycle: &'static str,
    pub current: Option<ReviewView>,
    pub history: Vec<ReviewView>,
    pub comments: Vec<CommentView>,
    /// The rule a submission would open a round with now.
    pub rule: ReviewRule,
    pub rule_source: String,
    pub can_comment: bool,
}

/// One entry in a person's review inbox.
#[derive(Debug, Clone, Serialize)]
pub struct InboxItem {
    /// `revision` or `eco`.
    pub kind: &'static str,
    /// The page link's target: a part id or a change order id.
    pub target: String,
    /// A revision's id; empty for a change order.
    pub revision_id: String,
    /// `CPART000000001 rev B`, or a change order's number.
    pub title: String,
    pub name: String,
    pub status: &'static str,
    pub opened_by: String,
    pub opened_at: u64,
    pub due: Option<u64>,
    pub overdue: bool,
    pub approvals: usize,
    pub required_approvals: u32,
}

/// What is waiting on a person, and what they have waiting on others.
#[derive(Debug, Default, Serialize)]
pub struct Inbox {
    /// Live rounds they may decide and have not approved yet.
    pub waiting_on_me: Vec<InboxItem>,
    /// Live rounds they submitted.
    pub submitted: Vec<InboxItem>,
    /// `waiting_on_me.len()`, for the badge.
    pub count: usize,
}

/// A live round's inbox entry.
pub fn inbox_item(
    state: &State,
    review: &Review,
    kind: &'static str,
    target: &str,
    revision_id: &str,
    title: String,
    name: &str,
    current: &str,
) -> InboxItem {
    InboxItem {
        kind,
        target: target.to_string(),
        revision_id: revision_id.to_string(),
        title,
        name: name.to_string(),
        status: review.status.as_str(),
        opened_by: name_of(state, &review.opened_by),
        opened_at: review.opened_at,
        due: review.due,
        overdue: review.due.is_some_and(|due| due < now()),
        approvals: review.approvers(current).len(),
        required_approvals: review.required_approvals,
    }
}

/// Where `review` goes in `user`'s inbox. A reviewer whose approval went
/// stale is waiting again.
pub fn file_item(inbox: &mut Inbox, review: &Review, user: &User, current: &str, item: impl FnOnce() -> InboxItem) {
    if !review.is_live() {
        return;
    }
    let mine = review.opened_by == user.id;
    let approved = review
        .decisions
        .iter()
        .rev()
        .find(|d| d.user_id == user.id)
        .is_some_and(|d| d.verdict == Verdict::Approve && d.is_current(current));
    let may_decide = is_reviewer(review, user) && !approved && !(mine && !review.allow_self_approval);
    if !(mine || may_decide) {
        return;
    }
    let item = item();
    if may_decide {
        inbox.waiting_on_me.push(item.clone());
    }
    if mine {
        inbox.submitted.push(item);
    }
}

// ===========================================================================
// The store's review operations on revisions
// ===========================================================================

/// What a submission may ask for beyond the rule: more reviewers, a due date.
#[derive(Debug, Clone, Default)]
pub struct Submission {
    /// Reviewers ADDED to the rule's.
    pub reviewers: Option<Value>,
    /// Unix seconds.
    pub due: Option<u64>,
    /// A comment posted with the submission.
    pub note: String,
}

impl Db {
    /// Submit a draft for review: open a round and move it to `InReview`.
    ///
    /// The rule is found, and the `reviewers` hook asked, on a snapshot and
    /// OUTSIDE the store lock; the locked write then re-checks that the move is
    /// still allowed. After it commits the `review-event` hook is told.
    pub fn submit_for_review(
        &self,
        user: &User,
        part_id: &str,
        revision_id: &str,
        submission: &Submission,
    ) -> Result<Vec<String>, Error> {
        lifecycle::check_enabled(&self.settings(), Lifecycle::InReview)?;
        let (part, revision) = self.snapshot(part_id, revision_id)?;
        lifecycle::check_transition(revision.lifecycle, Lifecycle::InReview).map_err(Error::conflict)?;
        self.read(|state| submit_lock_gate(state, user, &part.number, &revision))?;
        let (mut rule, mut source) = self.read(|state| rule_for(&state.settings, &state.categories, &part));
        if let Some(extra) = &submission.reviewers {
            let added = self.read(|state| parse_reviewers(state, extra))?;
            for reviewer in added {
                if !rule.reviewers.contains(&reviewer) {
                    rule.reviewers.push(reviewer);
                }
            }
        }
        let mut round = open_round(&rule, &source, &user.id, now());
        if let Some(due) = submission.due.filter(|d| *d > 0) {
            round.due = Some(due);
        }
        let input = json!({
            "part": part,
            "revision": revision,
            "rule": {
                "required_approvals": round.required_approvals,
                "reviewers": round.reviewers,
                "due": round.due,
                "allow_self_approval": round.allow_self_approval,
                "source": source,
            },
            "user": scripting::user_json(user),
        });
        match scripting::run(self, user, scripting::REVIEWERS, "reviewers", &input)? {
            HookResult::Absent => {}
            HookResult::Refused(failure) => {
                return Err(Error::conflict(format!("{}: {}", scripting::REVIEWERS, failure.message)))
            }
            HookResult::Returned(success) => {
                if let Value::Object(answer) = &success.value {
                    if let Some(reviewers) = answer.get("reviewers") {
                        round.reviewers = self
                            .read(|state| parse_reviewers(state, reviewers))
                            .map_err(|e| Error::conflict(format!("{}: {}", scripting::REVIEWERS, e.message)))?;
                    }
                    if let Some(required) = answer.get("required_approvals") {
                        round.required_approvals = required
                            .as_u64()
                            .filter(|n| *n <= u64::from(MAX_APPROVALS))
                            .ok_or_else(|| {
                                Error::conflict(format!(
                                    "{}: required_approvals must be a whole number up to {MAX_APPROVALS}",
                                    scripting::REVIEWERS
                                ))
                            })? as u32;
                    }
                    if let Some(due) = answer.get("due") {
                        round.due = due.as_u64().filter(|d| *d > 0);
                    }
                    if let Some(own) = answer.get("allow_self_approval").and_then(Value::as_bool) {
                        round.allow_self_approval = own;
                    }
                    source.push_str("+script");
                    round.rule_source = source.clone();
                    refresh(&mut round, "");
                } else if !success.value.is_null() {
                    return Err(Error::conflict(format!(
                        "{}: reviewers() must return {{ reviewers, required_approvals, due }} or null — it returned {}",
                        scripting::REVIEWERS,
                        success.value
                    )));
                }
            }
        }

        let note = submission.note.clone();
        let author = user.clone();
        self.mutate(move |state| {
            lifecycle::check_enabled(&state.settings, Lifecycle::InReview)?;
            // Again inside the lock: someone may have checked it out since.
            let (number, snapshot) = {
                let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
                (part.number.clone(), part.revision(revision_id).cloned().ok_or_else(|| Error::not_found("revision"))?)
            };
            submit_lock_gate(state, &author, &number, &snapshot)?;
            let (_, revision) = db::find_revision_mut(state, part_id, revision_id)?;
            lifecycle::check_transition(revision.lifecycle, Lifecycle::InReview).map_err(Error::conflict)?;
            revision.lifecycle = Lifecycle::InReview;
            revision.reviews.push(round);
            if !note.trim().is_empty() {
                push_comment(&mut revision.comments, &author, &note, "", now())?;
            }
            state.touch_part(part_id);
            Ok(())
        })?;
        Ok(self.review_event(user, "opened", part_id, revision_id, None))
    }

    /// Approve or reject a revision's live round. A rejection ends the round
    /// and sends the revision back to Draft.
    pub fn decide_revision(
        &self,
        user: &User,
        part_id: &str,
        revision_id: &str,
        verdict: Verdict,
        comment: &str,
    ) -> Result<Vec<String>, Error> {
        let decider = user.clone();
        let comment = comment.to_string();
        self.mutate(move |state| {
            let (number, revision) = db::find_revision_mut(state, part_id, revision_id)?;
            if revision.lifecycle != Lifecycle::InReview {
                return Err(Error::conflict(format!(
                    "{number} rev {} is {}, not in review",
                    revision.label,
                    revision.lifecycle.as_str()
                )));
            }
            let current = revision.content_hash.clone();
            let Some(review) = revision.live_review_mut() else {
                return Err(Error::conflict(format!("{number} rev {} has no review open", revision.label)));
            };
            record_decision(review, &decider, verdict, &comment, now(), &current)?;
            if verdict == Verdict::Reject {
                lifecycle::check_transition(revision.lifecycle, Lifecycle::Draft).map_err(Error::conflict)?;
                revision.lifecycle = Lifecycle::Draft;
            }
            state.touch_part(part_id);
            Ok(())
        })?;
        Ok(self.review_event(user, verdict.as_str(), part_id, revision_id, None))
    }

    /// Change a revision's live round: its reviewers, due date, and (for the
    /// check-in group) how many approvals it needs.
    pub fn change_revision_review(
        &self,
        user: &User,
        part_id: &str,
        revision_id: &str,
        change: &RoundChange,
    ) -> Result<Vec<String>, Error> {
        let editor = user.clone();
        let change = change.clone();
        self.mutate(move |state| {
            let snapshot = state.part(part_id).and_then(|p| p.revision(revision_id)).and_then(|r| r.review()).cloned();
            let Some(mut review) = snapshot else {
                return Err(Error::conflict("this revision has no review"));
            };
            let current = state
                .part(part_id)
                .and_then(|p| p.revision(revision_id))
                .map(|r| r.content_hash.clone())
                .unwrap_or_default();
            change_round(state, &mut review, &editor, &change, &current)?;
            let (_, revision) = db::find_revision_mut(state, part_id, revision_id)?;
            if let Some(last) = revision.reviews.last_mut() {
                *last = review;
            }
            Ok(())
        })?;
        Ok(self.review_event(user, "updated", part_id, revision_id, None))
    }

    /// Add a comment to a revision's discussion.
    pub fn comment_on_revision(
        &self,
        user: &User,
        part_id: &str,
        revision_id: &str,
        body: &str,
        parent: &str,
    ) -> Result<(Comment, Vec<String>), Error> {
        let author = user.clone();
        let body = body.to_string();
        let parent = parent.to_string();
        let comment = self.mutate(move |state| {
            let (_, revision) = db::find_revision_mut(state, part_id, revision_id)?;
            push_comment(&mut revision.comments, &author, &body, &parent, now())
        })?;
        let warnings = self.review_event(user, "comment", part_id, revision_id, Some(&comment));
        Ok((comment, warnings))
    }

    /// One revision's review panel, as `viewer` sees it.
    pub fn revision_review(&self, viewer: &User, part_id: &str, revision_id: &str) -> Result<RevisionReview, Error> {
        self.read(|state| {
            let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
            let revision = part.revision(revision_id).ok_or_else(|| Error::not_found("revision"))?;
            let (rule, rule_source) = rule_for(&state.settings, &state.categories, part);
            let mut rounds: Vec<ReviewView> =
                revision.reviews.iter().map(|r| view(state, r, viewer, &revision.content_hash)).collect();
            let current = rounds.pop();
            rounds.reverse();
            Ok(RevisionReview {
                part_id: part.id.clone(),
                number: part.number.clone(),
                revision_id: revision.id.clone(),
                label: revision.label.clone(),
                lifecycle: revision.lifecycle.as_str(),
                current,
                history: rounds,
                comments: comment_views(state, &revision.comments),
                rule,
                rule_source,
                can_comment: may_comment(viewer),
            })
        })
    }

    /// What is waiting on `user`, and what they submitted.
    pub fn inbox(&self, user: &User) -> Inbox {
        self.read(|state| {
            let mut inbox = Inbox::default();
            for part in &state.parts {
                for revision in &part.revisions {
                    let Some(review) = revision.review() else { continue };
                    file_item(&mut inbox, review, user, &revision.content_hash, || {
                        inbox_item(
                            state,
                            review,
                            "revision",
                            &part.id,
                            &revision.id,
                            format!("{} rev {}", part.number, revision.label),
                            &part.name,
                            &revision.content_hash,
                        )
                    });
                }
            }
            for eco in &state.change_orders {
                let Some(review) = eco.reviews.last() else { continue };
                let current = eco_subject_hash(state, eco);
                file_item(&mut inbox, review, user, &current, || {
                    inbox_item(state, review, "eco", &eco.id, "", eco.number.clone(), &eco.title, &current)
                });
            }
            inbox.waiting_on_me.sort_by_key(|i| (i.due.unwrap_or(u64::MAX), i.opened_at));
            inbox.submitted.sort_by_key(|i| std::cmp::Reverse(i.opened_at));
            inbox.count = inbox.waiting_on_me.len();
            inbox
        })
    }

    /// Tell the `review-event` hook that a revision was pulled out of review,
    /// when that ended a round.
    pub(crate) fn review_withdrawn(&self, user: &User, part_id: &str, revision_id: &str) -> Vec<String> {
        let withdrawn = self.read(|state| {
            state
                .part(part_id)
                .and_then(|p| p.revision(revision_id))
                .and_then(|r| r.review())
                .is_some_and(|r| r.status == ReviewStatus::Withdrawn)
        });
        if withdrawn {
            self.review_event(user, "withdrawn", part_id, revision_id, None)
        } else {
            Vec::new()
        }
    }

    /// Tell the `review-event` hook about something that just committed on a
    /// revision's review. It cannot undo it: a failure is a warning.
    fn review_event(
        &self,
        user: &User,
        event: &str,
        part_id: &str,
        revision_id: &str,
        comment: Option<&Comment>,
    ) -> Vec<String> {
        let input = self.read(|state| {
            let part = state.part(part_id)?;
            let revision = part.revision(revision_id)?;
            Some(json!({
                "event": event,
                "subject": {
                    "kind": "revision",
                    "part_id": part.id,
                    "number": part.number,
                    "name": part.name,
                    "revision_id": revision.id,
                    "label": revision.label,
                    "lifecycle": revision.lifecycle.as_str(),
                },
                "review": revision.review(),
                "comment": comment,
                "user": scripting::user_json(user),
            }))
        });
        let Some(input) = input else { return Vec::new() };
        hook_warning(self, user, &input)
    }
}

/// Run the `review-event` hook with `input`, returning its failure as a
/// warning.
pub fn hook_warning(db: &Db, user: &User, input: &Value) -> Vec<String> {
    match scripting::run(db, user, scripting::REVIEW_EVENT, "reviewEvent", input) {
        Ok(HookResult::Absent) | Ok(HookResult::Returned(_)) => Vec::new(),
        Ok(HookResult::Refused(failure)) => {
            vec![format!("done, but {} failed: {}", scripting::REVIEW_EVENT, failure.message)]
        }
        Err(error) => vec![format!("done, but {} could not run: {}", scripting::REVIEW_EVENT, error.message)],
    }
}
