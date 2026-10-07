//! Review, change orders and the inbox (plm-cad-integration-todo §3 S4).
//!
//! What a PLM's review process looks like from the CAD app:
//!
//! * **A revision's review** ([`ReviewPanel`] on a [`Subject::Revision`]): the
//!   live round — who must decide, who has, how many approvals it has of how
//!   many it needs, when it is due — its history and its comments. A draft
//!   with no live round offers **Submit for review** with extra reviewers, a
//!   due date and a note. A reviewer who may decide gets **Approve** and
//!   **Reject** with a comment. Anyone who may comment can.
//! * **A change order** ([`Subject::ChangeOrder`]): its items (each revision it
//!   releases or obsoletes, and what would stop it), its problems, its review,
//!   its comments, and **Release** through the order. With the server's
//!   `eco_holds_revisions` on, a revision an open order names releases only
//!   this way: the lifecycle panel's refused Release offers the order, found
//!   through the revision view's `eco` (the refusal's sentence names only its
//!   number), and emits [`PlmReviewEvent::OpenChangeOrder`].
//! * **The inbox badge** ([`InboxBadge`]): `GET /api/inbox`'s `count`, asked
//!   again every [`INBOX_POLL_SECS`], beside the PLM connection indicator. It
//!   opens a list of what waits on the user and what they submitted; a row
//!   opens its review or its change order.
//! * **A save during review warns** ([`save_warning`], D8) and saves.
//!
//! Every refusal is shown in the server's own words, as every PLM refusal is:
//! a release the review gate refuses ("… needs 1 approval(s) before it can
//! release …") is the lifecycle panel's Release answering `409`, and needs
//! nothing of its own.
//!
//! Everything goes through [`PlmReview`], which S1's client implements
//! ([`PlmReviewClient`]). Each request's future is held and polled every frame
//! with a waker that repaints, as S7's panel does. Nothing here is constructed
//! without a server.

use crate::panels::plm_parts::Pending;
use crate::plm::PlmFuture;
use eframe::egui;
use std::collections::HashMap;
use std::task::Waker;

/// How often the inbox badge asks again, in seconds. A decision someone else
/// makes shows within this; the badge also asks after every action of the
/// user's own.
pub const INBOX_POLL_SECS: f64 = 60.0;

// --- the wire ------------------------------------------------------------------
//
// These mirror the server's answers (`BREP_plm/src/review.rs` and `eco.rs`,
// through `api/review.rs` and `api/eco.rs`) and are pinned against the REAL
// router by the golden tests below. Every field the app does not need is left
// out; every field it reads defaults, so a server adding one breaks nothing.

/// A reviewer of a round: a user, or a group any of whose members may decide.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Reviewer {
    /// `user` or `group`.
    #[serde(default)]
    pub kind: String,
    #[serde(default)]
    pub id: String,
    #[serde(default)]
    pub name: String,
    /// A user's `approve`, `reject`, or empty (not yet).
    #[serde(default)]
    pub verdict: String,
    /// A group's members who approved.
    #[serde(default)]
    pub approved_by: Vec<String>,
}

/// One decision in a round.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Decision {
    #[serde(default)]
    pub name: String,
    /// `approve` or `reject`.
    #[serde(default)]
    pub verdict: String,
    #[serde(default)]
    pub comment: String,
    #[serde(default)]
    pub at: u64,
    /// The document's content hash the decision was made on; `None` when it
    /// was recorded before decisions carried one.
    #[serde(default)]
    pub content_hash: Option<String>,
    /// `Some(false)`: made on an older version of the document (D8: an
    /// approval goes stale when the document changes).
    #[serde(default)]
    pub current: Option<bool>,
}

/// A review round, as the server's `ReviewView` describes it to the person
/// asking.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Round {
    #[serde(default)]
    pub id: String,
    /// `open`, `approved`, `rejected`, `withdrawn`, `released`, …
    #[serde(default)]
    pub status: String,
    #[serde(default)]
    pub opened_by: String,
    #[serde(default)]
    pub opened_at: u64,
    #[serde(default)]
    pub required_approvals: u32,
    /// Approvals given on the document as it is now.
    #[serde(default)]
    pub approvals: usize,
    /// Standing approvals given on an older version: they need renewing.
    #[serde(default)]
    pub stale_approvals: usize,
    /// Enough current approvals and no rejection.
    #[serde(default)]
    pub met: bool,
    #[serde(default)]
    pub due: Option<u64>,
    #[serde(default)]
    pub overdue: bool,
    #[serde(default)]
    pub reviewers: Vec<Reviewer>,
    #[serde(default)]
    pub decisions: Vec<Decision>,
    /// Whether the person asking may approve or reject it now.
    #[serde(default)]
    pub can_decide: bool,
    #[serde(default)]
    pub can_change: bool,
    /// Still deciding, or approved and not yet used by a release.
    #[serde(default)]
    pub live: bool,
}

/// A comment on a revision or a change order.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Comment {
    #[serde(default)]
    pub id: String,
    #[serde(default)]
    pub author: String,
    #[serde(default)]
    pub at: u64,
    #[serde(default)]
    pub body: String,
    /// The comment this answers; empty for a top-level one.
    #[serde(default)]
    pub parent: String,
}

/// The rule a submission would open a round with now.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Rule {
    #[serde(default)]
    pub required_approvals: u32,
    #[serde(default)]
    pub due_days: u32,
    #[serde(default)]
    pub allow_self_approval: bool,
}

/// `GET /api/parts/:id/revisions/:rev/review`.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct RevisionReview {
    #[serde(default)]
    pub part_id: String,
    #[serde(default)]
    pub number: String,
    #[serde(default)]
    pub revision_id: String,
    #[serde(default)]
    pub label: String,
    /// `draft`, `inreview`, `released`, …
    #[serde(default)]
    pub lifecycle: String,
    #[serde(default)]
    pub current: Option<Round>,
    #[serde(default)]
    pub history: Vec<Round>,
    #[serde(default)]
    pub comments: Vec<Comment>,
    #[serde(default)]
    pub rule: Rule,
    #[serde(default)]
    pub can_comment: bool,
}

/// One row of the inbox: a live round.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct InboxItem {
    /// `revision` or `eco`.
    #[serde(default)]
    pub kind: String,
    /// A part id, or a change order's id.
    #[serde(default)]
    pub target: String,
    /// A revision's id; empty for a change order.
    #[serde(default)]
    pub revision_id: String,
    /// `CPART000000001 rev B`, or a change order's number.
    #[serde(default)]
    pub title: String,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub status: String,
    #[serde(default)]
    pub due: Option<u64>,
    #[serde(default)]
    pub overdue: bool,
    #[serde(default)]
    pub approvals: usize,
    #[serde(default)]
    pub required_approvals: u32,
}

impl InboxItem {
    /// What opening this row does.
    pub fn event(&self) -> Option<PlmReviewEvent> {
        match self.kind.as_str() {
            "revision" => Some(PlmReviewEvent::OpenReview { part: self.target.clone(), revision: self.revision_id.clone() }),
            "eco" => Some(PlmReviewEvent::OpenChangeOrder { eco: self.target.clone() }),
            _ => None,
        }
    }

    /// One line of the list.
    pub fn describe(&self) -> String {
        let mut line = format!("{} {} — {} {}/{}", self.title, self.name, self.status, self.approvals, self.required_approvals);
        if let Some(due) = self.due {
            line.push_str(&format!(", due {}", date(due)));
        }
        if self.overdue {
            line.push_str(" — OVERDUE");
        }
        line
    }
}

/// `GET /api/inbox`.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Inbox {
    /// Live rounds the user may decide and has not approved yet.
    #[serde(default)]
    pub waiting_on_me: Vec<InboxItem>,
    /// Live rounds the user submitted.
    #[serde(default)]
    pub submitted: Vec<InboxItem>,
    /// `waiting_on_me.len()`: the badge.
    #[serde(default)]
    pub count: usize,
}

/// One revision a change order releases or obsoletes.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct ChangeOrderItem {
    #[serde(default)]
    pub part_id: String,
    #[serde(default)]
    pub number: String,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub revision_id: String,
    #[serde(default)]
    pub label: String,
    #[serde(default)]
    pub lifecycle: String,
    /// `release` or `obsolete`.
    #[serde(default)]
    pub action: String,
    #[serde(default)]
    pub note: String,
    /// Why it could not be applied now; empty when nothing stops it.
    #[serde(default)]
    pub problem: String,
}

/// `GET /api/ecos/:id`.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct ChangeOrder {
    #[serde(default)]
    pub id: String,
    #[serde(default)]
    pub number: String,
    #[serde(default)]
    pub title: String,
    /// `draft`, `open`, `inreview`, `approved`, `released`, `cancelled`, …
    #[serde(default)]
    pub state: String,
    #[serde(default)]
    pub priority: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub reason: String,
    #[serde(default)]
    pub items: Vec<ChangeOrderItem>,
    #[serde(default)]
    pub current: Option<Round>,
    #[serde(default)]
    pub history: Vec<Round>,
    #[serde(default)]
    pub comments: Vec<Comment>,
    /// Every problem that would stop the release now.
    #[serde(default)]
    pub problems: Vec<String>,
    #[serde(default)]
    pub can_edit: bool,
    #[serde(default)]
    pub can_release: bool,
    #[serde(default)]
    pub can_comment: bool,
}

/// What every mutating review route answers: `{ ok, seq, warnings }`. A
/// warning is a `review-event` hook that failed after the event committed.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Done {
    #[serde(default)]
    pub warnings: Vec<String>,
}

/// Who holds a revision's lock, from `GET /api/parts/:id`'s `revision_views`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Lock {
    /// The holder's name; `None` when nobody has it checked out.
    pub holder: Option<String>,
    /// Whether the holder is the person asking.
    pub mine: bool,
}

impl Lock {
    /// Why the person asking may not submit this revision for review, in the
    /// server's own words (operator ruling: the lock holder submits, or any
    /// author while nobody holds it). `None` when they may.
    pub fn submit_block(&self, number: &str, label: &str) -> Option<String> {
        match (&self.holder, self.mine) {
            (Some(holder), false) => Some(format!(
                "{number} rev {label} is checked out by {holder} — only {holder} can submit it for review (or check it in first)"
            )),
            _ => None,
        }
    }
}

// --- requests --------------------------------------------------------------------

/// `POST /api/parts/:id/revisions/:rev/submit`'s body. Empty fields are left
/// out, so a bare submission is `{}` and takes the rule as it stands.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize)]
pub struct SubmitRequest {
    /// Added to the rule's: `user:<name>` or `group:<name>`.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub reviewers: Vec<String>,
    /// Unix seconds.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub due: Option<u64>,
    /// Posted as a comment with the submission.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub note: String,
}

impl SubmitRequest {
    /// Read the form. `reviewers` is a list separated by commas or spaces; a
    /// bare name is a user, `group:quality` a group. `due_days` is whole days
    /// from `now` (unix seconds), empty for none.
    pub fn from_form(reviewers: &str, due_days: &str, note: &str, now: u64) -> Result<Self, String> {
        let reviewers = reviewers
            .split([',', ' ', '\n', '\t'])
            .map(str::trim)
            .filter(|r| !r.is_empty())
            .map(|r| if r.contains(':') { r.to_string() } else { format!("user:{r}") })
            .collect();
        let due = match due_days.trim() {
            "" => None,
            days => {
                let days: u64 = days.parse().map_err(|_| format!("due in `{days}` days: give a whole number of days"))?;
                if days == 0 {
                    return Err("due in 0 days: give at least 1, or leave it empty for no due date".into());
                }
                Some(now + days * 86_400)
            }
        };
        Ok(Self { reviewers, due, note: note.trim().to_string() })
    }
}

/// A reviewer's answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    Approve,
    Reject,
}

impl Verdict {
    pub fn as_str(self) -> &'static str {
        match self {
            Verdict::Approve => "approve",
            Verdict::Reject => "reject",
        }
    }
}

/// What this panel asks the shell to do. The lifecycle panel (`panels::plm`,
/// S3) raises the same two, under the same names, from its Submit for review
/// verb and a refused Release: a rename on either side is a red test.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlmReviewEvent {
    /// Open the review of a revision.
    OpenReview { part: String, revision: String },
    /// Open a change order, by its id.
    OpenChangeOrder { eco: String },
}

impl PlmReviewEvent {
    /// The event of the lifecycle panel's (`panels::plm`, S3) that opens a
    /// pane of this one; `None` for its others. The match is exhaustive on
    /// purpose: a variant added there is a compile error here until someone
    /// decides whether it opens a review.
    pub fn from_lifecycle(event: &crate::panels::plm::PanelEvent) -> Option<Self> {
        use crate::panels::plm::PanelEvent;
        match event {
            PanelEvent::OpenReview { part, revision } => Some(Self::OpenReview { part: part.clone(), revision: revision.clone() }),
            PanelEvent::OpenChangeOrder { eco } => Some(Self::OpenChangeOrder { eco: eco.clone() }),
            PanelEvent::Access(_) | PanelEvent::Open(_) => None,
        }
    }
}

/// D8: a save to a revision in review is allowed, and warned about. `None`
/// for any other state. `label` is the revision's label.
///
/// The warning says what the server does (the operator's D8 ruling): each
/// decision records the document's content hash, and only approvals of the
/// document as it is count, so a save sends the round back for renewal.
pub fn save_warning(label: &str, lifecycle: &str) -> Option<String> {
    (lifecycle == "inreview").then(|| {
        format!(
            "Revision {label} is in review. This save changes the document its reviewers are deciding on: \
             approvals given so far will need renewing."
        )
    })
}

// --- the server seam -------------------------------------------------------------

/// Everything this panel asks of the PLM. S1's client implements it
/// ([`PlmReviewClient`]); the tests implement it with a fake and, in the
/// golden tests, with the real router.
pub trait PlmReview {
    fn review(&self, part: &str, revision: &str) -> PlmFuture<RevisionReview>;
    /// Who holds `revision`'s lock (`GET /api/parts/:id`).
    fn lock(&self, part: &str, revision: &str) -> PlmFuture<Lock>;
    fn submit(&self, part: &str, revision: &str, request: &SubmitRequest) -> PlmFuture<Done>;
    fn decide(&self, part: &str, revision: &str, verdict: Verdict, comment: &str) -> PlmFuture<Done>;
    fn comment(&self, part: &str, revision: &str, body: &str) -> PlmFuture<Done>;
    fn inbox(&self) -> PlmFuture<Inbox>;
    fn change_order(&self, eco: &str) -> PlmFuture<ChangeOrder>;
    fn submit_change_order(&self, eco: &str, note: &str) -> PlmFuture<Done>;
    fn decide_change_order(&self, eco: &str, verdict: Verdict, comment: &str) -> PlmFuture<Done>;
    fn comment_change_order(&self, eco: &str, body: &str) -> PlmFuture<Done>;
    fn release_change_order(&self, eco: &str) -> PlmFuture<Done>;
}

/// [`PlmReview`] over S1's client.
pub struct PlmReviewClient {
    pub client: std::rc::Rc<crate::plm::client::PlmClient>,
}

impl PlmReviewClient {
    fn json<T: serde::de::DeserializeOwned + 'static>(&self, method: &'static str, path: String, body: Option<serde_json::Value>) -> PlmFuture<T> {
        let client = self.client.clone();
        let body = body.map(|b| serde_json::to_vec(&b).expect("a JSON value serialises"));
        Box::pin(async move {
            let response = client.call(method, &path, body).await.map_err(|error| error.to_string())?;
            serde_json::from_slice(&response.body)
                .map_err(|error| format!("the PLM answered {path} with something this app cannot read: {error}"))
        })
    }

    fn revision_path(part: &str, revision: &str, rest: &str) -> String {
        format!("/api/parts/{}/revisions/{}{rest}", segment(part), segment(revision))
    }
}

impl PlmReview for PlmReviewClient {
    fn review(&self, part: &str, revision: &str) -> PlmFuture<RevisionReview> {
        self.json("GET", Self::revision_path(part, revision, "/review"), None)
    }
    fn lock(&self, part: &str, revision: &str) -> PlmFuture<Lock> {
        let view: PlmFuture<serde_json::Value> = self.json("GET", format!("/api/parts/{}", segment(part)), None);
        let revision = revision.to_string();
        Box::pin(async move {
            let view = view.await?;
            let row = view["revision_views"]
                .as_array()
                .and_then(|rows| rows.iter().find(|r| r["id"] == revision.as_str()))
                .ok_or_else(|| format!("the PLM's part has no revision {revision}"))?;
            Ok(Lock { holder: row["locked_by"].as_str().map(str::to_string), mine: row["locked_by_me"].as_bool().unwrap_or(false) })
        })
    }
    fn submit(&self, part: &str, revision: &str, request: &SubmitRequest) -> PlmFuture<Done> {
        let body = serde_json::to_value(request).expect("a submission serialises");
        self.json("POST", Self::revision_path(part, revision, "/submit"), Some(body))
    }
    fn decide(&self, part: &str, revision: &str, verdict: Verdict, comment: &str) -> PlmFuture<Done> {
        let body = serde_json::json!({ "verdict": verdict.as_str(), "comment": comment });
        self.json("POST", Self::revision_path(part, revision, "/review/decision"), Some(body))
    }
    fn comment(&self, part: &str, revision: &str, body: &str) -> PlmFuture<Done> {
        self.json("POST", Self::revision_path(part, revision, "/comments"), Some(serde_json::json!({ "body": body })))
    }
    fn inbox(&self) -> PlmFuture<Inbox> {
        self.json("GET", "/api/inbox".into(), None)
    }
    fn change_order(&self, eco: &str) -> PlmFuture<ChangeOrder> {
        self.json("GET", format!("/api/ecos/{}", segment(eco)), None)
    }
    fn submit_change_order(&self, eco: &str, note: &str) -> PlmFuture<Done> {
        self.json("POST", format!("/api/ecos/{}/submit", segment(eco)), Some(serde_json::json!({ "note": note })))
    }
    fn decide_change_order(&self, eco: &str, verdict: Verdict, comment: &str) -> PlmFuture<Done> {
        let body = serde_json::json!({ "verdict": verdict.as_str(), "comment": comment });
        self.json("POST", format!("/api/ecos/{}/review/decision", segment(eco)), Some(body))
    }
    fn comment_change_order(&self, eco: &str, body: &str) -> PlmFuture<Done> {
        self.json("POST", format!("/api/ecos/{}/comments", segment(eco)), Some(serde_json::json!({ "body": body })))
    }
    fn release_change_order(&self, eco: &str) -> PlmFuture<Done> {
        self.json("POST", format!("/api/ecos/{}/release", segment(eco)), Some(serde_json::json!({})))
    }
}

/// A path segment, percent-encoded outside the unreserved set.
fn segment(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for byte in text.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => out.push(byte as char),
            other => out.push_str(&format!("%{other:02X}")),
        }
    }
    out
}

/// `YYYY-MM-DD` (UTC) for unix seconds.
pub fn date(unix: u64) -> String {
    // Howard Hinnant's civil_from_days.
    let z = (unix / 86_400) as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!("{year:04}-{month:02}-{day:02}")
}

/// One line for a round: status, approvals against the requirement, due.
pub fn describe_round(round: &Round) -> String {
    let mut line = format!("{} — {} of {} approval(s)", round.status, round.approvals, round.required_approvals);
    if round.met {
        line.push_str(", met");
    }
    if round.stale_approvals > 0 {
        line.push_str(&format!(", {} given on an older version and need renewing", round.stale_approvals));
    }
    if let Some(due) = round.due {
        line.push_str(&format!(", due {}", date(due)));
    }
    if round.overdue {
        line.push_str(" — OVERDUE");
    }
    line
}

// --- the panel -------------------------------------------------------------------

/// What a [`ReviewPanel`] shows.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Subject {
    Revision { part: String, revision: String },
    ChangeOrder { eco: String },
}

impl Subject {
    /// The subject a [`PlmReviewEvent`] opens.
    pub fn of(event: &PlmReviewEvent) -> Self {
        match event {
            PlmReviewEvent::OpenReview { part, revision } => Subject::Revision { part: part.clone(), revision: revision.clone() },
            PlmReviewEvent::OpenChangeOrder { eco } => Subject::ChangeOrder { eco: eco.clone() },
        }
    }
}

/// What the server said about the subject.
#[derive(Clone, Debug, PartialEq)]
pub enum Loaded {
    Revision(RevisionReview),
    ChangeOrder(ChangeOrder),
}

/// The last thing an action said: a refusal in the server's words, or what a
/// success warned about.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Message {
    Refused(String),
    Done(String),
}

/// A revision's review, or a change order, with what the user may do to it.
pub struct ReviewPanel {
    pub subject: Subject,
    loaded: Option<Loaded>,
    load: Option<Pending<Loaded>>,
    /// A revision's lock, which decides whether Submit is offered.
    lock: Option<Lock>,
    lock_load: Option<Pending<Lock>>,
    action: Option<Pending<Done>>,
    message: Option<Message>,
    // the forms
    reviewers: String,
    due_days: String,
    note: String,
    decision_comment: String,
    new_comment: String,
    /// Set by an action the user took, so the shell can refresh the inbox.
    acted: bool,
    hits: HashMap<String, egui::Rect>,
}

impl ReviewPanel {
    /// A panel on `subject`, asking for it at once.
    pub fn new(subject: Subject, server: &dyn PlmReview) -> Self {
        let mut panel = Self {
            subject,
            loaded: None,
            load: None,
            lock: None,
            lock_load: None,
            action: None,
            message: None,
            reviewers: String::new(),
            due_days: String::new(),
            note: String::new(),
            decision_comment: String::new(),
            new_comment: String::new(),
            acted: false,
            hits: HashMap::new(),
        };
        panel.reload(server);
        panel
    }

    pub fn loaded(&self) -> Option<&Loaded> {
        self.loaded.as_ref()
    }

    /// The revision's lock, once read.
    pub fn lock(&self) -> Option<&Lock> {
        self.lock.as_ref()
    }

    pub fn message(&self) -> Option<&Message> {
        self.message.as_ref()
    }

    /// Whether a request is out.
    pub fn busy(&self) -> bool {
        self.load.as_ref().is_some_and(Pending::is_open)
            || self.action.as_ref().is_some_and(Pending::is_open)
            || self.lock_load.as_ref().is_some_and(Pending::is_open)
    }

    /// Whether the user acted since the last call (the inbox should ask again).
    pub fn take_acted(&mut self) -> bool {
        std::mem::take(&mut self.acted)
    }

    /// The published hit rects, by key.
    pub fn hits(&self) -> &HashMap<String, egui::Rect> {
        &self.hits
    }

    pub fn reload(&mut self, server: &dyn PlmReview) {
        let future: PlmFuture<Loaded> = match &self.subject {
            Subject::Revision { part, revision } => {
                self.lock_load = Some(Pending::new(server.lock(part, revision)));
                let request = server.review(part, revision);
                Box::pin(async move { request.await.map(Loaded::Revision) })
            }
            Subject::ChangeOrder { eco } => {
                let request = server.change_order(eco);
                Box::pin(async move { request.await.map(Loaded::ChangeOrder) })
            }
        };
        self.load = Some(Pending::new(future));
    }

    fn act(&mut self, request: PlmFuture<Done>) {
        self.message = None;
        self.acted = true;
        self.action = Some(Pending::new(request));
    }

    /// Take whatever answered since the last frame. Returns whether anything did.
    pub fn poll(&mut self, waker: &Waker, server: &dyn PlmReview) -> bool {
        let mut changed = false;
        if let Some(outcome) = self.action.as_mut().and_then(|p| p.poll(waker)) {
            self.action = None;
            changed = true;
            match outcome {
                Ok(done) => {
                    self.message = (!done.warnings.is_empty()).then(|| Message::Done(done.warnings.join("\n")));
                    self.decision_comment.clear();
                    self.new_comment.clear();
                    self.note.clear();
                    self.reload(server);
                }
                Err(sentence) => self.message = Some(Message::Refused(sentence)),
            }
        }
        if let Some(outcome) = self.lock_load.as_mut().and_then(|p| p.poll(waker)) {
            self.lock_load = None;
            changed = true;
            // Unknown (the read failed) offers the form; the server still judges.
            self.lock = outcome.ok();
        }
        if let Some(outcome) = self.load.as_mut().and_then(|p| p.poll(waker)) {
            self.load = None;
            changed = true;
            match outcome {
                Ok(loaded) => self.loaded = Some(loaded),
                Err(sentence) => self.message = Some(Message::Refused(sentence)),
            }
        }
        changed
    }

    // --- the actions, which the UI and a script both reach ----------------------

    /// Submit the revision (from the form), or the change order (with the note).
    pub fn submit(&mut self, server: &dyn PlmReview, now: u64) {
        match self.subject.clone() {
            Subject::Revision { part, revision } => match SubmitRequest::from_form(&self.reviewers, &self.due_days, &self.note, now) {
                Ok(request) => self.act(server.submit(&part, &revision, &request)),
                Err(problem) => self.message = Some(Message::Refused(problem)),
            },
            Subject::ChangeOrder { eco } => {
                let note = self.note.trim().to_string();
                self.act(server.submit_change_order(&eco, &note));
            }
        }
    }

    pub fn decide(&mut self, server: &dyn PlmReview, verdict: Verdict) {
        let comment = self.decision_comment.trim().to_string();
        match self.subject.clone() {
            Subject::Revision { part, revision } => self.act(server.decide(&part, &revision, verdict, &comment)),
            Subject::ChangeOrder { eco } => self.act(server.decide_change_order(&eco, verdict, &comment)),
        }
    }

    pub fn post_comment(&mut self, server: &dyn PlmReview) {
        let body = self.new_comment.trim().to_string();
        if body.is_empty() {
            return;
        }
        match self.subject.clone() {
            Subject::Revision { part, revision } => self.act(server.comment(&part, &revision, &body)),
            Subject::ChangeOrder { eco } => self.act(server.comment_change_order(&eco, &body)),
        }
    }

    /// Release a change order: every item it names, together.
    pub fn release_change_order(&mut self, server: &dyn PlmReview) {
        if let Subject::ChangeOrder { eco } = self.subject.clone() {
            self.act(server.release_change_order(&eco));
        }
    }

    pub fn set_form(&mut self, reviewers: &str, due_days: &str, note: &str) {
        self.reviewers = reviewers.into();
        self.due_days = due_days.into();
        self.note = note.into();
    }

    pub fn set_decision_comment(&mut self, comment: &str) {
        self.decision_comment = comment.into();
    }

    pub fn set_new_comment(&mut self, body: &str) {
        self.new_comment = body.into();
    }

    // --- drawing -------------------------------------------------------------

    /// Draw the panel. Returns what the user asked the shell to open (a change
    /// order's item opens that revision's review).
    pub fn show(&mut self, ui: &mut egui::Ui, server: &dyn PlmReview) -> Vec<PlmReviewEvent> {
        let waker = repaint_waker(ui.ctx());
        self.poll(&waker, server);
        self.hits.clear();
        let mut events = Vec::new();
        // `recovery::wall_clock`, not `SystemTime::now`: std's clock PANICS on
        // wasm32 ("time not implemented on this platform"), which killed the
        // hosted app the first time this pane drew (verify_plm_review.mjs).
        let now = crate::recovery::wall_clock() as u64;

        match &self.message {
            Some(Message::Refused(sentence)) => {
                ui.colored_label(ui.visuals().error_fg_color, sentence);
            }
            Some(Message::Done(warnings)) => {
                ui.colored_label(ui.visuals().warn_fg_color, warnings);
            }
            None => {}
        }
        if self.busy() {
            ui.spinner();
        }
        let Some(loaded) = self.loaded.clone() else {
            return events;
        };
        match &loaded {
            Loaded::Revision(review) => {
                ui.heading(format!("{} rev {} — {}", review.number, review.label, review.lifecycle));
                self.show_round(ui, server, review.current.as_ref());
                if review.current.as_ref().map_or(true, |r| !r.live) && review.lifecycle == "draft" {
                    match self.lock.as_ref().and_then(|l| l.submit_block(&review.number, &review.label)) {
                        Some(block) => {
                            ui.separator();
                            let why = ui.label(block);
                            self.hits.insert("submit:blocked".into(), why.rect);
                        }
                        None => self.show_submit_form(ui, server, &review.rule, now),
                    }
                }
                self.show_history(ui, &review.history);
                self.show_comments(ui, server, &review.comments, review.can_comment);
            }
            Loaded::ChangeOrder(order) => {
                ui.heading(format!("{} — {} ({})", order.number, order.title, order.state));
                if !order.description.is_empty() {
                    ui.label(&order.description);
                }
                for item in &order.items {
                    let mut text = format!("{} rev {} — {} ({})", item.number, item.label, item.action, item.lifecycle);
                    if !item.problem.is_empty() {
                        text.push_str(&format!(" — {}", item.problem));
                    }
                    let row = ui.selectable_label(false, text);
                    self.hits.insert(format!("eco:item:{}", crate::plm::identity::document_key(&item.part_id, &item.revision_id)), row.rect);
                    if row.double_clicked() {
                        events.push(PlmReviewEvent::OpenReview { part: item.part_id.clone(), revision: item.revision_id.clone() });
                    }
                }
                for problem in &order.problems {
                    ui.colored_label(ui.visuals().warn_fg_color, problem);
                }
                self.show_round(ui, server, order.current.as_ref());
                let live = order.current.as_ref().is_some_and(|r| r.live);
                ui.horizontal(|ui| {
                    if !live && order.can_edit && matches!(order.state.as_str(), "draft" | "open") {
                        let note = ui.add(egui::TextEdit::singleline(&mut self.note).hint_text("Note for the reviewers"));
                        self.hits.insert("eco:note".into(), note.rect);
                        let submit = ui.button("Submit for review");
                        self.hits.insert("eco:submit".into(), submit.rect);
                        if submit.clicked() {
                            self.submit(server, now);
                        }
                    }
                    if order.can_release {
                        let release = ui.button(format!("Release through {}", order.number));
                        self.hits.insert("eco:release".into(), release.rect);
                        if release.clicked() {
                            self.release_change_order(server);
                        }
                    }
                });
                self.show_history(ui, &order.history);
                self.show_comments(ui, server, &order.comments, order.can_comment);
            }
        }
        events
    }

    fn show_round(&mut self, ui: &mut egui::Ui, server: &dyn PlmReview, round: Option<&Round>) {
        let Some(round) = round else {
            ui.label("No review open.");
            return;
        };
        ui.label(describe_round(round));
        for reviewer in &round.reviewers {
            let verdict = match (reviewer.kind.as_str(), reviewer.verdict.as_str()) {
                ("group", _) if !reviewer.approved_by.is_empty() => format!("approved by {}", reviewer.approved_by.join(", ")),
                (_, "") => "waiting".to_string(),
                (_, verdict) => verdict.to_string(),
            };
            ui.label(format!("  {} ({}): {verdict}", reviewer.name, reviewer.kind));
        }
        for decision in &round.decisions {
            let mut line = format!("  {} {} on {}", decision.name, decision.verdict, date(decision.at));
            if decision.current == Some(false) {
                line.push_str(" (on an older version)");
            }
            if !decision.comment.is_empty() {
                line.push_str(&format!(": {}", decision.comment));
            }
            ui.label(line);
        }
        if round.can_decide {
            let comment = ui.add(egui::TextEdit::singleline(&mut self.decision_comment).hint_text("Comment with your decision"));
            self.hits.insert("decide:comment".into(), comment.rect);
            ui.horizontal(|ui| {
                let approve = ui.button("Approve");
                self.hits.insert("decide:approve".into(), approve.rect);
                let reject = ui.button("Reject");
                self.hits.insert("decide:reject".into(), reject.rect);
                if approve.clicked() {
                    self.decide(server, Verdict::Approve);
                } else if reject.clicked() {
                    self.decide(server, Verdict::Reject);
                }
            });
        }
    }

    fn show_submit_form(&mut self, ui: &mut egui::Ui, server: &dyn PlmReview, rule: &Rule, now: u64) {
        ui.separator();
        ui.label(if rule.required_approvals == 0 {
            "Submit for review. This part's rule needs no approval to release.".to_string()
        } else {
            format!("Submit for review. This part's rule needs {} approval(s) to release.", rule.required_approvals)
        });
        let reviewers = ui.add(egui::TextEdit::singleline(&mut self.reviewers).hint_text("More reviewers: ada, group:quality"));
        self.hits.insert("submit:reviewers".into(), reviewers.rect);
        let due_hint = if rule.due_days > 0 { format!("Due in days (the rule: {})", rule.due_days) } else { "Due in days".to_string() };
        let due = ui.add(egui::TextEdit::singleline(&mut self.due_days).hint_text(due_hint));
        self.hits.insert("submit:due".into(), due.rect);
        let note = ui.add(egui::TextEdit::multiline(&mut self.note).hint_text("Note for the reviewers").desired_rows(2));
        self.hits.insert("submit:note".into(), note.rect);
        let go = ui.add_enabled(self.action.is_none(), egui::Button::new("Submit for review"));
        self.hits.insert("submit:go".into(), go.rect);
        if go.clicked() {
            self.submit(server, now);
        }
    }

    fn show_history(&mut self, ui: &mut egui::Ui, history: &[Round]) {
        if history.is_empty() {
            return;
        }
        ui.collapsing(format!("Earlier rounds ({})", history.len()), |ui| {
            for round in history {
                ui.label(format!("{} — opened {}", describe_round(round), date(round.opened_at)));
            }
        });
    }

    fn show_comments(&mut self, ui: &mut egui::Ui, server: &dyn PlmReview, comments: &[Comment], can_comment: bool) {
        ui.separator();
        for comment in comments {
            let indent = if comment.parent.is_empty() { "" } else { "    " };
            ui.label(format!("{indent}{} ({}): {}", comment.author, date(comment.at), comment.body));
        }
        if can_comment {
            ui.horizontal(|ui| {
                let field = ui.add(egui::TextEdit::singleline(&mut self.new_comment).hint_text("Add a comment"));
                self.hits.insert("comment:body".into(), field.rect);
                let post = ui.button("Post");
                self.hits.insert("comment:post".into(), post.rect);
                if post.clicked() {
                    self.post_comment(server);
                }
            });
        }
    }
}

// --- the section in the PLM pane -------------------------------------------------

/// This panel's section of the PLM pane (`panels::plm_host`, S3): the one
/// review or change order open now. The lifecycle panel's Submit for review,
/// Review… and Open change order, an inbox row, and a change order's item all
/// open it through [`Self::open`].
#[derive(Default)]
pub struct ReviewSection {
    panel: Option<ReviewPanel>,
    /// This frame's rects: the pane's, and Close.
    hits: HashMap<String, egui::Rect>,
}

impl ReviewSection {
    pub fn new() -> Self {
        Self::default()
    }

    /// Show `event`'s subject. The same subject again re-reads it rather than
    /// losing what was typed into its forms.
    pub fn open(&mut self, event: &PlmReviewEvent, server: &dyn PlmReview) {
        let subject = Subject::of(event);
        match &mut self.panel {
            Some(panel) if panel.subject == subject => panel.reload(server),
            _ => self.panel = Some(ReviewPanel::new(subject, server)),
        }
    }

    pub fn panel(&self) -> Option<&ReviewPanel> {
        self.panel.as_ref()
    }

    pub fn panel_mut(&mut self) -> Option<&mut ReviewPanel> {
        self.panel.as_mut()
    }

    pub fn close(&mut self) {
        self.panel = None;
    }

    /// This frame's widget rects, keyed without a prefix (the host adds its
    /// section's).
    pub fn hits(&self) -> &HashMap<String, egui::Rect> {
        &self.hits
    }

    /// Whether the open pane has a request out.
    pub fn busy(&self) -> bool {
        self.panel.as_ref().is_some_and(ReviewPanel::busy)
    }

    /// Take what answered since the last frame (the host drives this every
    /// frame, so an answer lands even while the pane is scrolled away).
    pub fn poll(&mut self, server: &dyn PlmReview) {
        if let Some(panel) = self.panel.as_mut() {
            panel.poll(std::task::Waker::noop(), server);
        }
    }

    /// What is open and what it says, for tests (no state blob: nothing draws
    /// the section since the pane split moved review management to the web app).
    pub fn state_json(&self) -> serde_json::Value {
        let Some(panel) = &self.panel else { return serde_json::Value::Null };
        let subject = match &panel.subject {
            Subject::Revision { part, revision } => serde_json::json!({ "kind": "revision", "part": part, "revision": revision }),
            Subject::ChangeOrder { eco } => serde_json::json!({ "kind": "eco", "eco": eco }),
        };
        let round = |r: &Round| serde_json::json!({
            "status": r.status, "approvals": r.approvals, "stale": r.stale_approvals, "required": r.required_approvals,
            "met": r.met, "live": r.live, "canDecide": r.can_decide,
        });
        let (lifecycle, current, comments) = match panel.loaded() {
            Some(Loaded::Revision(v)) => (Some(v.lifecycle.clone()), v.current.as_ref().map(round), v.comments.len()),
            Some(Loaded::ChangeOrder(o)) => (Some(o.state.clone()), o.current.as_ref().map(round), o.comments.len()),
            None => (None, None, 0),
        };
        let message = match panel.message() {
            Some(Message::Refused(s)) => serde_json::json!({ "refused": s }),
            Some(Message::Done(s)) => serde_json::json!({ "warned": s }),
            None => serde_json::Value::Null,
        };
        serde_json::json!({ "subject": subject, "state": lifecycle, "round": current, "comments": comments, "message": message, "busy": panel.busy() })
    }

    /// Whether the user acted since the last call, so the shell asks the inbox
    /// again ([`InboxBadge::refresh`]).
    pub fn take_acted(&mut self) -> bool {
        self.panel.as_mut().is_some_and(ReviewPanel::take_acted)
    }

    /// Draw the open pane, if any. An event it raises (a change order's item)
    /// opens here too.
    pub fn ui(&mut self, ui: &mut egui::Ui, server: &dyn PlmReview) {
        self.hits.clear();
        let Some(panel) = self.panel.as_mut() else {
            ui.label("Nothing open. Submit or review a revision from the lifecycle section, or open the inbox.");
            return;
        };
        let events = panel.show(ui, server);
        self.hits.extend(panel.hits().iter().map(|(k, r)| (k.clone(), *r)));
        let close = ui.button("Close");
        self.hits.insert("close".into(), close.rect);
        if close.clicked() {
            self.panel = None;
        }
        for event in events {
            self.open(&event, server);
        }
    }
}

// --- the inbox badge --------------------------------------------------------------

/// `GET /api/inbox`'s count beside the PLM connection indicator, and the list
/// it opens.
#[derive(Default)]
pub struct InboxBadge {
    request: Option<Pending<Inbox>>,
    inbox: Option<Inbox>,
    /// `ctx` time of the last ask; `None` asks on the next frame.
    asked_at: Option<f64>,
    open: bool,
    problem: Option<String>,
    hits: HashMap<String, egui::Rect>,
}

impl InboxBadge {
    pub fn new() -> Self {
        Self::default()
    }

    /// What waits on the user, once known.
    pub fn count(&self) -> Option<usize> {
        self.inbox.as_ref().map(|i| i.count)
    }

    pub fn inbox(&self) -> Option<&Inbox> {
        self.inbox.as_ref()
    }

    /// Whether an ask is out: the idle contract waits for it
    /// (`automation::cmd_frame`), so a script reads the count it asked for.
    pub fn busy(&self) -> bool {
        self.request.is_some() || (self.asked_at.is_none() && self.inbox.is_some())
    }

    /// Why the last ask failed, in the server's words.
    pub fn problem(&self) -> Option<&str> {
        self.problem.as_deref()
    }

    pub fn hits(&self) -> &HashMap<String, egui::Rect> {
        &self.hits
    }

    /// The badge's count and rows, for tests; null until the first answer (no
    /// state blob: the toolbar links to the web inbox and never draws the badge).
    pub fn state_json(&self) -> String {
        let Some(inbox) = &self.inbox else { return "null".into() };
        let rows = |items: &[InboxItem]| items.iter().map(|i| serde_json::json!({ "kind": i.kind, "title": i.title, "target": i.target, "revision": i.revision_id, "key": format!("plm:inbox:row:{}:{}", crate::plm::identity::segment(&i.target), crate::plm::identity::segment(&i.revision_id)) })).collect::<Vec<_>>();
        serde_json::json!({ "count": inbox.count, "open": self.open, "waitingOnMe": rows(&inbox.waiting_on_me), "submitted": rows(&inbox.submitted), "problem": self.problem }).to_string()
    }

    /// Ask again on the next frame — after the user decided or submitted.
    pub fn refresh(&mut self) {
        self.asked_at = None;
    }

    /// Ask if it is time (`now` in seconds, the frame's clock), and take an
    /// answer that arrived. Returns the seconds until the next ask, for the
    /// shell's `request_repaint_after`.
    pub fn tick(&mut self, now: f64, waker: &Waker, server: &dyn PlmReview) -> f64 {
        if let Some(outcome) = self.request.as_mut().and_then(|p| p.poll(waker)) {
            self.request = None;
            match outcome {
                Ok(inbox) => {
                    self.inbox = Some(inbox);
                    self.problem = None;
                }
                Err(sentence) => self.problem = Some(sentence),
            }
        }
        let due = self.asked_at.map_or(true, |at| now - at >= INBOX_POLL_SECS);
        if due && self.request.is_none() {
            self.asked_at = Some(now);
            self.request = Some(Pending::new(server.inbox()));
            // A resolved answer (a test's, or a cache's) is taken this frame.
            return self.tick(now, waker, server);
        }
        self.asked_at.map_or(0.0, |at| (at + INBOX_POLL_SECS - now).max(0.0))
    }

    /// Draw the badge, and its list when open. Returns the row the user
    /// opened, if any.
    pub fn show(&mut self, ui: &mut egui::Ui, server: &dyn PlmReview) -> Vec<PlmReviewEvent> {
        let waker = repaint_waker(ui.ctx());
        let now = ui.ctx().input(|i| i.time);
        let next = self.tick(now, &waker, server);
        ui.ctx().request_repaint_after(std::time::Duration::from_secs_f64(next.max(1.0)));
        self.hits.clear();
        let text = match self.count() {
            Some(0) | None => "Inbox".to_string(),
            Some(n) => format!("Inbox ({n})"),
        };
        let mut button = egui::Button::new(text);
        if self.count().unwrap_or(0) > 0 {
            button = button.fill(ui.visuals().selection.bg_fill);
        }
        let badge = ui.add(button);
        let badge = match &self.problem {
            Some(problem) => badge.on_hover_text(problem),
            None => badge,
        };
        self.hits.insert("plm:inbox".into(), badge.rect);
        if badge.clicked() {
            self.open = !self.open;
        }
        let mut events = Vec::new();
        if self.open {
            let inbox = self.inbox.clone().unwrap_or_default();
            egui::Window::new("Review inbox").id(egui::Id::new("plm_review:inbox:window")).open(&mut self.open).vscroll(true).show(ui.ctx(), |ui| {
                for (heading, items) in [("Waiting on you", &inbox.waiting_on_me), ("You submitted", &inbox.submitted)] {
                    ui.strong(heading);
                    if items.is_empty() {
                        ui.label("  nothing");
                    }
                    for item in items {
                        let row = ui.selectable_label(false, item.describe());
                        self.hits.insert(format!("plm:inbox:row:{}:{}", crate::plm::identity::segment(&item.target), crate::plm::identity::segment(&item.revision_id)), row.rect);
                        if row.clicked() {
                            events.extend(item.event());
                        }
                    }
                }
            });
        }
        // Opening a row closes the list: what it opened is in the PLM pane.
        if !events.is_empty() {
            self.open = false;
        }
        events
    }
}

/// A waker that asks egui for a frame — a transport wakes it when an answer
/// arrives, and the next frame's poll takes it.
fn repaint_waker(ctx: &egui::Context) -> Waker {
    struct Repaint(egui::Context);
    impl std::task::Wake for Repaint {
        fn wake(self: std::sync::Arc<Self>) {
            self.0.request_repaint();
        }
    }
    Waker::from(std::sync::Arc::new(Repaint(ctx.clone())))
}

