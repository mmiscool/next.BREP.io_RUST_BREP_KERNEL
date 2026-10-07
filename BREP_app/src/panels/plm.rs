//! Lifecycle in the CAD UI: the revision picker and the checkout verbs for a
//! document opened from the PLM.
//!
//! # What it decides
//!
//! - **Session editing is always allowed.** [`access_for`] turns a revision's
//!   server view into save permissions: saving to that revision requires a
//!   draft or in-review revision checked out by this user. Released revisions,
//!   another user's checkout, and pending permissions only restrict saving.
//!   The session copy stays editable and can be saved to another destination.
//! - **The verbs** a revision offers ([`verbs`]): Check out on an editable
//!   revision nobody holds; Check in on one this user holds; Break lock only to
//!   the check-in group, on one someone else holds; Release on an editable one to
//!   the check-in group; New revision to authors. The server decides every one
//!   of them again, and its refusal is shown as it sent it.
//! - **Which revision opens** (D13): the NEWEST, drafts included — the one
//!   created last ([`newest`]); the picker lets the user choose another.
//!
//! # How it talks to the server
//!
//! Through [`PlmLifecycle`], one method per route, so the panel is tested
//! against a fake and the real client (`crate::plm::client`) implements the
//! same trait. Every call returns a [`PlmFuture`]; the panel keeps at most one
//! in flight and polls it each frame ([`PlmPanel::poll`]), because a panel
//! draws synchronously. Each answer re-reads the part, so what the picker shows
//! is always the server's word.
//!
//! # With no server
//!
//! None of this exists: the shell builds a [`PlmPanel`] only for a document
//! that came from a PLM, and a file-based document has none.

use std::task::{Context, Poll, Waker};

use eframe::egui;
use serde::Deserialize;

use crate::document::Access;
use crate::plm::PlmFuture;

/// The `client_id` the app takes a lock under, so a lock says which client of
/// a user holds it (the server defaults to `"web"`).
pub const CLIENT_ID: &str = "brep-app";

// ===========================================================================
// What the server answers (`GET /api/parts/:id`), read tolerantly
// ===========================================================================

/// One part and its revisions, as `GET /api/parts/:id` answers. Fields this
/// app does not read are ignored; a missing one takes its default.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct PartDetail {
    pub id: String,
    pub number: String,
    pub name: String,
    /// `normal`, `family` or `template`.
    pub document_class: String,
    /// Creation order, oldest first.
    pub revision_views: Vec<RevisionView>,
    /// The label a new revision gets when nobody types one.
    pub suggested_label: String,
    /// Whether another revision may start while one is in work.
    pub multiple_open_drafts: bool,
}

/// One revision with its lock, review, change order, bake and provenance.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct RevisionView {
    pub id: String,
    pub label: String,
    /// `draft`, `inreview`, `released`, `superseded` or `obsolete`.
    pub lifecycle: String,
    /// A draft or in review: its document may change, under a lock.
    pub editable: bool,
    pub created_at: u64,
    pub modified_at: u64,
    /// Who holds the lock, as a name a person reads.
    pub locked_by: Option<String>,
    pub locked_by_me: bool,
    pub locked_at: Option<u64>,
    pub released_by: Option<String>,
    pub released_at: Option<u64>,
    /// `part/<part>/rev/<revision>` — the store key its document lives at.
    pub document_key: String,
    /// `authored`, `imported` or `generated`.
    pub origin: String,
    pub family: Option<Provenance>,
    pub template: Option<Provenance>,
    /// `pending`, `claimed`, `failed` or `done`; absent when never queued.
    pub bake: Option<String>,
    pub bake_error: String,
    /// A generated member whose document changed since Generate wrote it.
    pub hand_edited: bool,
    /// Lines in its uses list: more than none makes it an assembly.
    pub uses: usize,
    pub review: Option<ReviewSummary>,
    pub comments: usize,
    pub eco: Option<EcoRef>,
}

/// Where a generated or spun-out revision came from.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct Provenance {
    pub part_id: String,
    pub number: String,
    pub revision_label: String,
}

/// The current review round, in brief.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct ReviewSummary {
    pub status: String,
    pub live: bool,
    pub approvals: usize,
    pub required_approvals: u32,
    pub may_decide: bool,
}

/// The open change order naming a revision.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct EcoRef {
    pub id: String,
    pub number: String,
    pub state: String,
    /// `release` or `obsolete`.
    pub action: String,
}

/// A new revision, as `POST /api/parts/:id/revisions` answers.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct NewRevision {
    pub id: String,
    pub label: String,
}

/// One line of `GET /api/parts/:id/history`, the part's audit trail. A change
/// the app made with a token is logged as the token's owner, `via: token`.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct HistoryEvent {
    pub id: u64,
    pub at: u64,
    pub actor: HistoryActor,
    pub action: String,
    pub entity: HistoryEntity,
    pub detail: String,
}

#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct HistoryActor {
    pub username: String,
    /// `session`, `token` or `system`.
    pub via: String,
}

#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct HistoryEntity {
    pub kind: String,
    pub id: String,
    pub label: String,
}

/// What the signed-in user may do, from `/api/me`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Rights {
    pub can_author: bool,
    pub can_checkin: bool,
}

// ===========================================================================
// The server, as the panel needs it
// ===========================================================================

/// The lifecycle routes, one method each. Errors are the server's sentence.
pub trait PlmLifecycle {
    /// What the signed-in user may do: `/api/me`'s `can_author` and
    /// `can_checkin`, resolved by the server from the groups.
    fn rights(&self) -> PlmFuture<Rights>;
    /// `GET /api/parts/:id`.
    fn part(&self, part: &str) -> PlmFuture<PartDetail>;
    /// `POST /api/parts/:id/revisions/:rev/checkout` with [`CLIENT_ID`].
    fn checkout(&self, part: &str, revision: &str) -> PlmFuture<()>;
    /// `POST …/checkin`; `force` breaks someone else's lock (check-in group).
    fn checkin(&self, part: &str, revision: &str, force: bool) -> PlmFuture<()>;
    /// `POST …/state` with `{"to": to}`; answers the release's warnings.
    fn set_state(&self, part: &str, revision: &str, to: &str) -> PlmFuture<Vec<String>>;
    /// `POST /api/parts/:id/revisions` with `{"label"}`; empty takes the
    /// server's suggestion.
    fn create_revision(&self, part: &str, label: &str) -> PlmFuture<NewRevision>;
    /// `GET /api/parts/:id/history`, newest first.
    fn history(&self, part: &str) -> PlmFuture<Vec<HistoryEvent>>;
}

// ===========================================================================
// The rules, as pure functions
// ===========================================================================

/// The revision that opens when a part is opened by number (D13): the one
/// created last, drafts included. Ties keep the later one in the list, which
/// is creation order.
pub fn newest(part: &PartDetail) -> Option<&RevisionView> {
    part.revision_views
        .iter()
        .enumerate()
        .max_by_key(|(i, r)| (r.created_at, *i))
        .map(|(_, r)| r)
}

/// Whether the session copy may be saved to `revision`, and if not, why.
pub fn access_for(revision: &RevisionView) -> Access {
    let label = &revision.label;
    if !revision.editable {
        let state = match revision.lifecycle.as_str() {
            "released" => "released",
            "superseded" => "superseded",
            "obsolete" => "obsolete",
            other => other,
        };
        return Access::read_only(format!("revision {label} is {state} and cannot be saved to"));
    }
    match (&revision.locked_by, revision.locked_by_me) {
        (Some(_), true) => Access::Editable,
        (Some(who), false) => Access::read_only(format!("revision {label} is checked out by {who}")),
        (None, _) => Access::read_only(format!("revision {label} is not checked out — saving requires checkout")),
    }
}

/// A lifecycle action the panel offers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verb {
    CheckOut,
    CheckIn,
    BreakLock,
    Release,
    NewRevision,
    /// Open the review pane's submit form (S4 owns the form and the route).
    Submit,
    /// Open the review pane on the revision's current round.
    Review,
    /// A release was refused and a change order holds the revision: open it.
    OpenChangeOrder,
}

impl Verb {
    /// The verb's hit key in the PLM pane: `plm/lifecycle:verb:<key>`.
    pub fn key(self) -> &'static str {
        match self {
            Verb::CheckOut => "check-out",
            Verb::CheckIn => "check-in",
            Verb::BreakLock => "break-lock",
            Verb::Release => "release",
            Verb::NewRevision => "new-revision",
            Verb::Submit => "submit",
            Verb::Review => "review",
            Verb::OpenChangeOrder => "open-change-order",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Verb::CheckOut => "Check out",
            Verb::CheckIn => "Check in",
            Verb::BreakLock => "Break lock",
            Verb::Release => "Release",
            Verb::NewRevision => "New revision",
            Verb::Submit => "Submit for review",
            Verb::Review => "Review…",
            Verb::OpenChangeOrder => "Open change order",
        }
    }
}

/// The verbs `revision` of `part` offers to a user with `rights`, in the order
/// the panel draws them.
pub fn verbs(part: &PartDetail, revision: &RevisionView, rights: Rights) -> Vec<Verb> {
    let mut out = Vec::new();
    if revision.editable {
        match (&revision.locked_by, revision.locked_by_me) {
            (None, _) if rights.can_author => out.push(Verb::CheckOut),
            (Some(_), true) => out.push(Verb::CheckIn),
            (Some(_), false) if rights.can_checkin => out.push(Verb::BreakLock),
            _ => {}
        }
        // Held by someone else, a release would land on their work in progress.
        let held_by_other = revision.locked_by.is_some() && !revision.locked_by_me;
        if rights.can_checkin && !held_by_other {
            out.push(Verb::Release);
        }
    }
    let open_draft = part.revision_views.iter().any(|r| r.editable);
    if rights.can_author && (part.multiple_open_drafts || !open_draft) {
        out.push(Verb::NewRevision);
    }
    let live_review = revision.review.as_ref().is_some_and(|r| r.live);
    // Only the lock holder submits, or any author while nobody holds it (ruling 2026-09-25).
    let held_by_other = revision.locked_by.is_some() && !revision.locked_by_me;
    if revision.editable && revision.lifecycle == "draft" && rights.can_author && !live_review && !held_by_other {
        out.push(Verb::Submit);
    }
    if revision.review.is_some() {
        out.push(Verb::Review);
    }
    out
}

/// One line of the picker: the label and everything the server says about the
/// revision that a person deciding which one to open needs.
pub fn describe(revision: &RevisionView) -> String {
    let mut parts = vec![format!("{} — {}", revision.label, revision.lifecycle)];
    match (&revision.locked_by, revision.locked_by_me) {
        (Some(_), true) => parts.push("checked out by you".into()),
        (Some(who), false) => parts.push(format!("checked out by {who}")),
        (None, _) => {}
    }
    if let Some(review) = &revision.review {
        if review.live {
            parts.push(format!("in review {}/{}", review.approvals, review.required_approvals));
        } else {
            parts.push(format!("review {}", review.status));
        }
    }
    if let Some(eco) = &revision.eco {
        parts.push(format!("{} ({} {})", eco.number, eco.action, eco.state));
    }
    if let Some(bake) = &revision.bake {
        if revision.bake_error.is_empty() {
            parts.push(format!("bake {bake}"));
        } else {
            parts.push(format!("bake {bake}: {}", revision.bake_error));
        }
    }
    if revision.hand_edited {
        parts.push("hand-edited".into());
    }
    if revision.uses > 0 {
        parts.push(format!("{} uses", revision.uses));
    }
    if revision.origin != "authored" && !revision.origin.is_empty() {
        parts.push(revision.origin.clone());
    }
    if let Some(family) = &revision.family {
        parts.push(format!("from family {} {}", family.number, family.revision_label));
    }
    if let Some(template) = &revision.template {
        parts.push(format!("from template {} {}", template.number, template.revision_label));
    }
    parts.join(" · ")
}

// ===========================================================================
// The panel
// ===========================================================================

/// A request in flight, polled once a frame.
struct Pending<T> {
    future: PlmFuture<T>,
}

impl<T> Pending<T> {
    fn new(future: PlmFuture<T>) -> Self {
        Self { future }
    }

    fn poll(&mut self) -> Option<Result<T, String>> {
        let mut cx = Context::from_waker(Waker::noop());
        match self.future.as_mut().poll(&mut cx) {
            Poll::Ready(result) => Some(result),
            Poll::Pending => None,
        }
    }
}

/// What is in flight.
enum Work {
    Part(Pending<PartDetail>),
    /// A verb; on success the part is re-read.
    Verb(Verb, Pending<String>),
    History(Pending<Vec<HistoryEvent>>),
}

/// What happened this frame that the shell acts on.
#[derive(Debug, Clone, PartialEq)]
pub enum PanelEvent {
    /// The shown revision's permission to save changed.
    Access(Access),
    /// The user chose another revision to open (its store key).
    Open(String),
    /// Open the review pane on a revision: its submit form, or its round
    /// (`panels::plm_review`, S4).
    OpenReview { part: String, revision: String },
    /// Open a change order, by id — the one holding a revision whose release
    /// was refused.
    OpenChangeOrder { eco: String },
}

/// The lifecycle panel for ONE document opened from the PLM.
pub struct PlmPanel {
    part_id: String,
    /// The revision the document shows.
    revision_id: String,
    rights: Rights,
    part: Option<PartDetail>,
    history: Option<Vec<HistoryEvent>>,
    work: Option<Work>,
    /// The last sentence to show: a refusal, a release warning, a success.
    message: Option<String>,
    /// Typed into the New revision field; empty takes the suggestion.
    new_label: String,
    /// The access last reported, so an unchanged one is not re-sent.
    reported: Option<Access>,
    /// A new revision was started: open the newest once the part is re-read.
    open_newest_after: bool,
    /// New revision was pressed in the pane: show what uses the part, then
    /// start only on the user's confirmation.
    confirm_new: bool,
    /// The last Release was refused, so a change order holding the revision is
    /// offered ([`Verb::OpenChangeOrder`]).
    release_refused: bool,
    /// Events a press raised without a server call, handed out by the next poll.
    queued: Vec<PanelEvent>,
}

impl PlmPanel {
    /// A panel for `part_id` showing `revision_id`, which asks for the part at
    /// once.
    pub fn new(server: &dyn PlmLifecycle, part_id: &str, revision_id: &str, rights: Rights) -> Self {
        let mut panel = Self {
            part_id: part_id.to_string(),
            revision_id: revision_id.to_string(),
            rights,
            part: None,
            history: None,
            work: None,
            message: None,
            new_label: String::new(),
            reported: None,
            open_newest_after: false,
            confirm_new: false,
            release_refused: false,
            queued: Vec::new(),
        };
        panel.work = Some(Work::Part(Pending::new(server.part(part_id))));
        panel
    }

    pub(crate) fn matches_document(&self, part: &str, revision: &str) -> bool {
        self.part_id == part && self.revision_id == revision
    }

    pub fn part(&self) -> Option<&PartDetail> {
        self.part.as_ref()
    }

    pub fn revision(&self) -> Option<&RevisionView> {
        self.part.as_ref()?.revision_views.iter().find(|r| r.id == self.revision_id)
    }

    pub fn message(&self) -> Option<&str> {
        self.message.as_deref()
    }

    pub fn busy(&self) -> bool {
        self.work.is_some()
    }

    fn refreshing(&self) -> bool {
        self.part.is_some() && matches!(self.work, Some(Work::Part(_)))
    }

    /// Keep the revision's actions mounted during read-only background refreshes.
    /// Mutating requests still suppress duplicate actions.
    pub fn offered(&self) -> Vec<Verb> {
        match (&self.part, self.revision()) {
            (Some(part), Some(revision)) if self.work.is_none() || self.refreshing() => {
                let mut offered = verbs(part, revision, self.rights);
                if self.release_refused && revision.eco.is_some() {
                    offered.push(Verb::OpenChangeOrder);
                }
                offered
            }
            _ => Vec::new(),
        }
    }

    /// Start `verb` on the shown revision. Ignored while a request is in flight
    /// or when the revision does not offer it.
    pub fn press(&mut self, server: &dyn PlmLifecycle, verb: Verb) {
        if !self.offered().contains(&verb) {
            return;
        }
        if self.refreshing() { self.work = None; }
        let (part, rev) = (self.part_id.clone(), self.revision_id.clone());
        // The review pane and the change order pane are S4's: these open them.
        match verb {
            Verb::Submit | Verb::Review => {
                self.queued.push(PanelEvent::OpenReview { part, revision: rev });
                return;
            }
            Verb::OpenChangeOrder => {
                if let Some(eco) = self.revision().and_then(|r| r.eco.clone()) {
                    self.queued.push(PanelEvent::OpenChangeOrder { eco: eco.id });
                }
                return;
            }
            _ => {}
        }
        self.release_refused = false;
        let done = |label: &'static str| move |_| label.to_string();
        let future: PlmFuture<String> = match verb {
            Verb::CheckOut => map(server.checkout(&part, &rev), done("Checked out")),
            Verb::CheckIn => map(server.checkin(&part, &rev, false), done("Checked in")),
            Verb::BreakLock => map(server.checkin(&part, &rev, true), done("Lock broken")),
            Verb::Release => map(server.set_state(&part, &rev, "released"), |warnings: Vec<String>| {
                if warnings.is_empty() { "Released".to_string() } else { format!("Released — {}", warnings.join("; ")) }
            }),
            Verb::NewRevision => map(server.create_revision(&part, self.new_label.trim()), |r: NewRevision| {
                format!("Revision {} started", r.label)
            }),
            Verb::Submit | Verb::Review | Verb::OpenChangeOrder => unreachable!("opened above"),
        };
        self.message = None;
        self.work = Some(Work::Verb(verb, Pending::new(future)));
    }

    /// Re-read the part: another client may have changed the revision (a
    /// submit, a release, a broken lock). Ignored while a request is in flight.
    pub fn refresh(&mut self, server: &dyn PlmLifecycle) {
        if self.work.is_none() {
            self.work = Some(Work::Part(Pending::new(server.part(&self.part_id))));
        }
    }

    /// Ask for the part's history.
    pub fn load_history(&mut self, server: &dyn PlmLifecycle) {
        if self.refreshing() { self.work = None; }
        if self.work.is_none() {
            self.work = Some(Work::History(Pending::new(server.history(&self.part_id))));
        }
    }

    pub fn history(&self) -> Option<&[HistoryEvent]> {
        self.history.as_deref()
    }

    /// Drive the request in flight one step. Returns what the shell must act
    /// on: the document's new access, or a revision to open.
    pub fn poll(&mut self, server: &dyn PlmLifecycle) -> Vec<PanelEvent> {
        let mut events = std::mem::take(&mut self.queued);
        let Some(work) = self.work.as_mut() else { return events };
        match work {
            Work::Part(pending) => match pending.poll() {
                None => return events,
                Some(Ok(part)) => {
                    self.part = Some(part);
                    self.work = None;
                }
                Some(Err(error)) => {
                    self.message = Some(error);
                    self.work = None;
                }
            },
            Work::Verb(verb, pending) => {
                let verb = *verb;
                match pending.poll() {
                    None => return events,
                    Some(result) => {
                        let new_revision = verb == Verb::NewRevision;
                        self.work = Some(Work::Part(Pending::new(server.part(&self.part_id))));
                        match result {
                            Ok(sentence) => {
                                self.message = Some(sentence);
                                if new_revision {
                                    self.new_label.clear();
                                }
                            }
                            Err(error) => {
                                self.release_refused = verb == Verb::Release;
                                self.message = Some(error);
                            }
                        }
                        // A new revision is opened once the part is re-read.
                        self.open_newest_after = new_revision;
                        return events;
                    }
                }
            }
            Work::History(pending) => match pending.poll() {
                None => return events,
                Some(result) => {
                    self.work = None;
                    match result {
                        Ok(history) => self.history = Some(history),
                        Err(error) => self.message = Some(error),
                    }
                }
            },
        }
        if std::mem::take(&mut self.open_newest_after) {
            if let Some(revision) = self.part.as_ref().and_then(newest) {
                if revision.id != self.revision_id {
                    events.push(PanelEvent::Open(revision.document_key.clone()));
                }
            }
        }
        if let Some(revision) = self.revision() {
            let access = access_for(revision);
            if self.reported.as_ref() != Some(&access) {
                self.reported = Some(access.clone());
                events.push(PanelEvent::Access(access));
            }
        }
        events
    }

    /// Poll, then draw. Returns what the shell must act on, as [`Self::poll`].
    pub fn ui(&mut self, ui: &mut egui::Ui, server: &dyn PlmLifecycle) -> Vec<PanelEvent> {
        let mut events = self.poll(server);
        let mut hits = std::collections::HashMap::new();
        events.extend(self.draw(ui, server, &mut crate::panels::plm_host::SectionHits::new("lifecycle", &mut hits)));
        events
    }

    /// Draw the panel without polling — for a host that polls every panel
    /// itself ([`crate::panels::plm_host::PlmHost::sync`]). Returns the picks
    /// made on this frame (another revision to open); a pressed verb's answer
    /// arrives through the next [`Self::poll`].
    ///
    /// Hit keys (under `plm/`): `lifecycle:revision:<label>` for each picker
    /// row, `lifecycle:verb:<Verb::key>` for each verb, `lifecycle:new-label`,
    /// `lifecycle:history:load`.
    pub fn draw(
        &mut self,
        ui: &mut egui::Ui,
        server: &dyn PlmLifecycle,
        hits: &mut crate::panels::plm_host::SectionHits<'_>,
    ) -> Vec<PanelEvent> {
        self.draw_with(ui, server, hits, &mut |_| {})
    }

    /// [`Self::draw`], with `before_new_revision` drawn in the New revision
    /// confirmation — the host shows where the part is used there, so a user
    /// sees what a change reaches before starting one (S6's where-used).
    ///
    /// Extra hit keys: `lifecycle:new-revision:start`, `lifecycle:new-revision:cancel`.
    pub fn draw_with(
        &mut self,
        ui: &mut egui::Ui,
        server: &dyn PlmLifecycle,
        hits: &mut crate::panels::plm_host::SectionHits<'_>,
        before_new_revision: &mut dyn FnMut(&mut egui::Ui),
    ) -> Vec<PanelEvent> {
        let mut events = Vec::new();
        if self.busy() {
            ui.ctx().request_repaint();
        }
        let Some(part) = self.part.clone() else {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Reading the part from the PLM…");
            });
            if let Some(message) = &self.message {
                ui.colored_label(ui.visuals().error_fg_color, message);
            }
            return events;
        };
        ui.heading(format!("{} {}", part.number, part.name));
        if let Some(revision) = self.revision() {
            let access = access_for(revision);
            match &access {
                Access::Editable => ui.label(format!("Revision {} — checked out by you", revision.label)),
                Access::ReadOnly { reason } => ui.label(format!("Read-only: {reason}")),
            };
        }
        ui.separator();
        ui.label("Revisions");
        for revision in part.revision_views.iter().rev() {
            let shown = revision.id == self.revision_id;
            let response = ui.selectable_label(shown, describe(revision));
            hits.put(&format!("revision:{}", revision.label), response.rect);
            if response.clicked() && !shown {
                events.push(PanelEvent::Open(revision.document_key.clone()));
            }
        }
        ui.separator();
        let offered = self.offered();
        ui.horizontal_wrapped(|ui| {
            for verb in offered.iter().copied().filter(|v| *v != Verb::NewRevision) {
                let label = match (verb, self.revision().and_then(|r| r.eco.as_ref())) {
                    (Verb::OpenChangeOrder, Some(eco)) => format!("Open change order {}", eco.number),
                    _ => verb.label().to_string(),
                };
                let button = ui.button(label);
                hits.put(&format!("verb:{}", verb.key()), button.rect);
                if button.clicked() {
                    self.press(server, verb);
                }
            }
        });
        if offered.contains(&Verb::NewRevision) {
            ui.horizontal(|ui| {
                let field = ui.add(egui::TextEdit::singleline(&mut self.new_label).hint_text(part.suggested_label.as_str()).desired_width(80.0));
                hits.put("new-label", field.rect);
                let button = ui.button(format!("{}…", Verb::NewRevision.label()));
                hits.put(&format!("verb:{}", Verb::NewRevision.key()), button.rect);
                if button.clicked() {
                    self.confirm_new = true;
                }
            });
            if self.confirm_new {
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.label("Before starting a new revision: where this part is used");
                    before_new_revision(ui);
                    ui.horizontal(|ui| {
                        let label = if self.new_label.trim().is_empty() { part.suggested_label.clone() } else { self.new_label.trim().to_string() };
                        let start = ui.button(format!("Start revision {label}"));
                        hits.put("new-revision:start", start.rect);
                        if start.clicked() {
                            self.confirm_new = false;
                            self.press(server, Verb::NewRevision);
                        }
                        let cancel = ui.button("Cancel");
                        hits.put("new-revision:cancel", cancel.rect);
                        if cancel.clicked() {
                            self.confirm_new = false;
                        }
                    });
                });
            }
        } else {
            self.confirm_new = false;
        }
        ui.horizontal(|ui| {
            let refresh = ui.add_enabled(!self.busy() || self.refreshing(), egui::Button::new("Refresh"));
            hits.put("refresh", refresh.rect);
            if refresh.clicked() {
                self.refresh(server);
            }
            if self.busy() && !self.refreshing() {
                ui.spinner();
            }
        });
        if let Some(message) = &self.message {
            ui.label(message);
        }
        ui.separator();
        egui::CollapsingHeader::new("History").id_salt("plm-lifecycle-history").show(ui, |ui| {
            match &self.history {
                None => {
                    let button = ui.button("Load history");
                    hits.put("history:load", button.rect);
                    if button.clicked() {
                        self.load_history(server);
                    }
                }
                Some(events) if events.is_empty() => {
                    ui.label("No changes recorded.");
                }
                Some(events) => {
                    for event in events {
                        let what = if event.entity.label.is_empty() { &event.entity.kind } else { &event.entity.label };
                        ui.label(format!("{} {} {} ({})", event.actor.username, event.action, what, event.actor.via));
                    }
                }
            }
        });
        events
    }
}

// ===========================================================================
// Over the PLM client
// ===========================================================================

/// The routes over [`crate::plm::client::PlmClient`]. An `Rc`, because each
/// answer is a future the panel keeps across frames, and it must hold the
/// client that sends it.
impl PlmLifecycle for std::rc::Rc<crate::plm::client::PlmClient> {
    fn rights(&self) -> PlmFuture<Rights> {
        #[derive(Deserialize, Default)]
        #[serde(default)]
        struct Can {
            can_author: bool,
            can_checkin: bool,
        }
        let client = self.clone();
        Box::pin(async move {
            let can: Can = json(client.call("GET", "/api/me", None).await)?;
            Ok(Rights { can_author: can.can_author, can_checkin: can.can_checkin })
        })
    }

    fn part(&self, part: &str) -> PlmFuture<PartDetail> {
        let (client, path) = (self.clone(), crate::plm::identity::part_path(&part));
        Box::pin(async move { json(client.call("GET", &path, None).await) })
    }

    fn checkout(&self, part: &str, revision: &str) -> PlmFuture<()> {
        let (client, path) = (self.clone(), format!("{}/checkout", crate::plm::identity::revision_path(&part, &revision)));
        let body = serde_json::json!({ "client_id": CLIENT_ID }).to_string().into_bytes();
        Box::pin(async move { unit(client.call("POST", &path, Some(body)).await) })
    }

    fn checkin(&self, part: &str, revision: &str, force: bool) -> PlmFuture<()> {
        let (client, path) = (self.clone(), format!("{}/checkin", crate::plm::identity::revision_path(&part, &revision)));
        let body = serde_json::json!({ "force": force }).to_string().into_bytes();
        Box::pin(async move { unit(client.call("POST", &path, Some(body)).await) })
    }

    fn set_state(&self, part: &str, revision: &str, to: &str) -> PlmFuture<Vec<String>> {
        #[derive(Deserialize, Default)]
        #[serde(default)]
        struct Answer {
            warnings: Vec<String>,
        }
        let (client, path) = (self.clone(), format!("{}/state", crate::plm::identity::revision_path(&part, &revision)));
        let body = serde_json::json!({ "to": to }).to_string().into_bytes();
        Box::pin(async move { json::<Answer>(client.call("POST", &path, Some(body)).await).map(|a| a.warnings) })
    }

    fn create_revision(&self, part: &str, label: &str) -> PlmFuture<NewRevision> {
        let (client, path) = (self.clone(), format!("{}/revisions", crate::plm::identity::part_path(&part)));
        let body = serde_json::json!({ "label": label }).to_string().into_bytes();
        Box::pin(async move { json(client.call("POST", &path, Some(body)).await) })
    }

    fn history(&self, part: &str) -> PlmFuture<Vec<HistoryEvent>> {
        let (client, path) = (self.clone(), format!("{}/history", crate::plm::identity::part_path(&part)));
        Box::pin(async move { json(client.call("GET", &path, None).await) })
    }
}

/// A 2xx answer's JSON body, or the server's sentence.
fn json<T: for<'de> Deserialize<'de>>(
    answer: Result<crate::plm::PlmResponse, crate::plm::client::PlmError>,
) -> Result<T, String> {
    let response = answer.map_err(|e| e.to_string())?;
    serde_json::from_slice(&response.body).map_err(|e| format!("the PLM answered in a shape this app does not read: {e}"))
}

fn unit(answer: Result<crate::plm::PlmResponse, crate::plm::client::PlmError>) -> Result<(), String> {
    answer.map(|_| ()).map_err(|e| e.to_string())
}

/// `future` with its value mapped.
fn map<T: 'static, U: 'static>(future: PlmFuture<T>, f: impl FnOnce(T) -> U + 'static) -> PlmFuture<U> {
    Box::pin(async move { future.await.map(f) })
}

