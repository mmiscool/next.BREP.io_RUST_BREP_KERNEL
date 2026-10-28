//! Where the PLM panels live in the shell (plm-cad-integration-todo §3 S3 part 2).
//!
//! # The shape every PLM panel uses
//!
//! - **One dock pane**, [`crate::panels::dock::PaneKind::Plm`], titled "PLM". It
//!   is in the tree like every side pane and shown only while the session is on
//!   a PLM ([`in_plm_session`], a [`crate::workbench::PanelCondition`]). A
//!   file-based session never sees it.
//! - **One host**, [`PlmHost`], owned by the shell. It holds every PLM panel as
//!   a field and draws each as a section of that pane. A slice that adds a panel
//!   adds a field, a section in [`PlmHost::ui`], and the routing of its events
//!   in [`PlmHost::route`] — nothing in `dock.rs` or `app.rs`.
//! - **The client** is the store's ([`crate::store::ModelStore::plm_client`]),
//!   handed in each frame by [`PlmHost::sync`]. Panels never build one.
//! - **Events between panels** go through the host: the lifecycle panel's
//!   `OpenReview` is the review section's to act on; an `Open` of another
//!   revision is the shell's, returned in [`HostActions`].
//!
//! # What the lifecycle section does to a document
//!
//! PLM permissions govern saving to the revision. The session copy stays
//! editable while permissions load, without checkout, and after release.
//! Checkout refreshes a clean copy from the store; unsaved session edits
//! are preserved through every permission change.

use std::collections::HashMap;
use std::rc::Rc;

use eframe::egui;

use crate::document::{Access, Documents};
use crate::panels::plm::{access_with, PanelEvent, PlmLifecycle, PlmPanel, Rights};
use crate::plm::client::PlmClient;
use crate::plm::PlmFuture;
use crate::store::ModelStore;


/// A link to the configured PLM web app, for management outside CAD.
pub fn web_url(store: &dyn ModelStore, route: &str) -> Option<String> {
    let session = store.plm_session()?;
    let url = match session.status {
        crate::plm::connection::Status::Connected { url, .. }
        | crate::plm::connection::Status::NotConnected { url, .. } => url,
    };
    Some(format!("{}/{route}", url.trim_end_matches('/')))
}

pub fn web_link(ui: &mut egui::Ui, label: &str, url: String) -> egui::Response {
    ui.add(egui::Hyperlink::from_label_and_url(label, url).open_in_new_tab(true))
}

thread_local! {
    /// Whether the session is on a PLM, as of the last [`PlmHost::sync`] — what
    /// the PLM pane's [`crate::workbench::PanelCondition`] reads, since a
    /// panel condition sees only the engine.
    static IN_SESSION: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// The PLM pane's panel condition: shown only while the session is on a PLM.
pub fn in_plm_session(_: &crate::workbench::ButtonState) -> bool {
    IN_SESSION.with(|flag| flag.get())
}

/// The access a PLM document has before the server has said anything.
pub fn pending_access() -> Access {
    Access::read_only("the revision is being read from the PLM")
}

/// A write-behind failure as the user reads it (`None`: no toast of its own).
/// The PLM server not answering is one plain sentence naming the document as
/// its tab does, not the transport's words; a preference written then
/// (`@recent_documents`) says nothing, as the pane already said the server
/// is not reachable. Anything else is shown as it came.
pub fn plain_write_failure(message: &str) -> Option<String> {
    const UNREACHABLE: &str = "' was NOT saved: the PLM server did not answer";
    // A `401` (`PlmError::SignIn`): the sign-in ended, not the network.
    const SIGNED_OUT: &str = "' was NOT saved: sign in again";
    let (end, why) = match (message.find(UNREACHABLE), message.find(SIGNED_OUT)) {
        (Some(end), _) => (end, "the PLM server is not reachable. It stays unsaved in its tab: Save again when the server is back."),
        (None, Some(end)) => (end, "your PLM sign-in has ended. It stays unsaved in its tab: sign in again (Settings › PLM), then Save."),
        (None, None) => return Some(message.to_string()),
    };
    if !message.starts_with('\'') {
        return Some(message.to_string());
    }
    let name = &message[1..end];
    if name.starts_with('@') {
        return None;
    }
    let shown = revision_of(name).map_or_else(|| name.to_string(), |(part, revision)| format!("{part}/{revision}"));
    Some(format!("{shown} was NOT saved: {why}"))
}

fn capitalised(text: &str) -> String {
    let mut chars = text.chars();
    chars.next().map_or_else(String::new, |first| first.to_uppercase().collect::<String>() + chars.as_str())
}

/// `part/<part>/rev/<revision>` read out of a document name as the explorer
/// spells it (`/models/part/<p>/rev/<r>.nbrep`, or the bare key). Only a PLM
/// session may read names this way: the shape alone is not proof.
pub fn revision_of(name: &str) -> Option<(String, String)> {
    let start = name.find("part/")?;
    let key = &name[start..];
    let key = key.strip_suffix(".nbrep").or_else(|| key.strip_suffix(".fbrep"))
        .or_else(|| key.strip_suffix(".tbrep")).unwrap_or(key);
    let crate::store::DocumentIdentity::Revision { part: part_id, revision: revision_id } =
        crate::store::DocumentIdentity::parse_revision_key(key)? else { return None };
    Some((part_id, revision_id))
}

/// The name to open another revision of the same part under: `name` with its
/// revision swapped, so the explorer's prefix and the class's extension stay.
/// Every PLM revision `library` holds, at any depth and once each, that the
/// store's index says is checked out to the user, deepest first; the
/// panel's own revision is not one of them.
/// Each is named as a Save names it (`plate (CPART000000003 rev A)`).
fn held_parts(library: &serde_json::Value, store: &dyn ModelStore, panel: &PlmPanel) -> Vec<String> {
    fn walk(
        library: &serde_json::Value,
        store: &dyn ModelStore,
        panel: &PlmPanel,
        seen: &mut std::collections::HashSet<String>,
        out: &mut Vec<String>,
    ) {
        for (name, entry) in library.as_object().into_iter().flatten() {
            walk(&entry["document"]["partsLibrary"], store, panel, seen, out);
            let Some(key) = entry["sourceKey"].as_str().filter(|key| !key.is_empty()) else { continue };
            if !seen.insert(key.to_string()) {
                continue;
            }
            let Some((part, revision)) = crate::plm::bom::revision_of_document(key) else { continue };
            if panel.matches_document(&part, &revision) {
                continue;
            }
            let held = store
                .plm_revision(&crate::plm::identity::document_key(&part, &revision))
                .is_some_and(|row| row.locked_by_me);
            if held {
                out.push(crate::document::owned_save::shown(key, std::slice::from_ref(name)));
            }
        }
    }
    let mut out = Vec::new();
    walk(library, store, panel, &mut std::collections::HashSet::new(), &mut out);
    out
}

pub fn sibling_revision(name: &str, key: &str) -> Option<String> {
    let start = name.find("part/")?;
    let (_, new_revision) = revision_of(key)?;
    let (_, old_revision) = revision_of(name)?;
    let old_revision = crate::plm::identity::segment(&old_revision);
    let new_revision = crate::plm::identity::segment(&new_revision);
    let tail = &name[start..];
    let at = tail.rfind(&old_revision)?;
    Some(format!("{}{}{}{}", &name[..start], &tail[..at], new_revision, &tail[at + old_revision.len()..]))
}

/// The PLM pane's sections, in the order they draw. Each id is the section's
/// hit-key prefix (`plm/<id>:…`) and its header's id, so both stay stable as
/// sections are added. A slice adding a section adds its id here.
/// The PLM pane's published keys (`__brepPlmHit`, `plm/<key>`): each section's own keys,
/// under the section's id, and every section's header. One family per section, so a
/// section that grows a widget stays documented.
pub static HIT_KEYS: &[crate::automation::hit_keys::HitKeyDoc] = &[
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "panel:clip", meaning: "the visible PLM pane region", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "section:", meaning: "section:lifecycle, the revision browser and checkout", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "lifecycle:", meaning: "lifecycle:revision (the Open revision dropdown; choosing an entry, lifecycle:revision#<label>, opens that revision); lifecycle:verb:check-out acquires its lock; lifecycle:held:check-in opens the PLM page that checks in the parts this assembly uses that are checked out to you", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "follow:", meaning: "another client saved this tab's revision while it holds edits: follow:keep keeps them, follow:take takes their version; for edited copies in an assembly whose revisions moved, follow:copies-keep / follow:copies-take", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "superseded:", meaning: "an assembly uses a superseded or obsolete PLM revision: superseded:update replaces it with the current release (mates kept, as Replace keeps them), superseded:keep leaves it; on an assembly that keeps its revisions (released, held by someone), superseded:new-revision makes its next revision and updates it there", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "web:", meaning: "management links opening the PLM web app in another tab: summary, revisions, attachments, structure, history, parts, workspace, reviews", command: None },
];

pub const SECTIONS: &[(&str, &str)] = &[(LIFECYCLE, "Revision")];

/// The uses-list section (S5): shown only while the active assembly's parts
/// list on the PLM disagrees with its document (Replace everywhere), with the
/// offer to re-point.
pub const USES: &str = "uses";
/// The parts section (S7): the catalog browser (Insert) and New part.
pub const PARTS: &str = "parts";
/// The import section (S13): adopt a folder of documents.
pub const IMPORT: &str = "import";

/// The where-used section: every assembly that uses the part, every level up
/// (S6's `panels::bom_plm::WhereUsedSection`). Its lines are also drawn in the
/// lifecycle section's New revision confirmation.
pub const WHERE_USED: &str = "where-used";

/// The review section: a revision's review round (S4).
pub const REVIEW: &str = "review";

/// The lifecycle section: the revision picker and the checkout verbs (S3).
pub const LIFECYCLE: &str = "lifecycle";

/// The files section: the part's and revision's attachments, and this
/// document's drawing PDF or STEP attached to the revision (S8).
pub const ATTACHMENTS: &str = "attachments";
/// The workspace section: the user's folders of links and files, and open by number (S14).
/// The same browser is File > Open in a PLM session.
pub const WORKSPACE: &str = "workspace";

/// The pane's widget rects, published as `__brepPlmHit` (`plm/<key>`).
pub type Hits = HashMap<String, egui::Rect>;

/// One section's view of [`Hits`]: every key it puts is prefixed with the
/// section's id, so two sections can never collide.
pub struct SectionHits<'a> {
    prefix: &'static str,
    map: &'a mut Hits,
}

impl<'a> SectionHits<'a> {
    pub fn new(prefix: &'static str, map: &'a mut Hits) -> Self {
        Self { prefix, map }
    }

    pub fn put(&mut self, key: &str, rect: egui::Rect) {
        self.map.insert(format!("{}:{key}", self.prefix), rect);
    }
}

/// A collapsible section of the PLM pane with a stable id: open by default,
/// its header's rect published as `plm/section:<id>`.
pub fn section<R>(ui: &mut egui::Ui, hits: &mut Hits, id: &'static str, add: impl FnOnce(&mut egui::Ui, &mut SectionHits<'_>) -> R) -> Option<R> {
    let title = SECTIONS.iter().find(|(key, _)| *key == id).map_or(id, |(_, title)| *title);
    let mut inner = Hits::new();
    let shown = egui::CollapsingHeader::new(title)
        .id_salt(("plm-section", id))
        .default_open(true)
        .show(ui, |ui| add(ui, &mut SectionHits::new(id, &mut inner)));
    hits.insert(format!("section:{id}"), shown.header_response.rect);
    hits.extend(inner);
    shown.body_returned
}

/// What the shell must do after [`PlmHost::sync`].
#[derive(Debug, Default, PartialEq)]
pub struct HostActions {
    /// Open these document names (another revision the user picked, or the
    /// one a New revision started).
    pub open: Vec<String>,
    /// For the names in `open` that a workspace folder's link opened: that
    /// folder `(id, name)` (`crate::document::Document::plm_folder`).
    pub opened_from: Vec<(String, (String, String))>,
    /// A section asked for a file: open the file chooser for (tag, title),
    /// and hand what it delivers to [`PlmHost::deliver_picked`].
    pub pick_file: Option<(String, String)>,
    /// The user decided, submitted, commented or released: ask the review
    /// inbox again (`ToolbarPanel::refresh_inbox`).
    pub refresh_inbox: bool,
    /// Files a section downloaded (an attachment, a workspace file): hand
    /// each to the file dialog's export door, which asks where on desktop.
    pub save_files: Vec<crate::panels::file::SaveRequest>,
    /// The server refused the session's sign-in (revoked under Linked
    /// desktops, expired): the shell opens the Settings PLM tab to sign in
    /// again, naming why.
    pub sign_in_again: Option<String>,
}

/// The file-chooser tag of the workspace section.
pub const PICK_WORKSPACE: &str = "plm:workspace";
/// The file-chooser tag prefix of the attachments section: `plm:attachments:<document id>`.
pub const PICK_ATTACHMENTS: &str = "plm:attachments:";

/// Every PLM panel, and the one client they share.
#[derive(Default)]
pub struct PlmHost {
    web_base: String,
    client: Option<Rc<PlmClient>>,
    rights: Option<Rights>,
    rights_pending: Option<PlmFuture<Rights>>,
    /// The lifecycle panel of each open PLM document, by document id.
    lifecycle: HashMap<u64, PlmPanel>,
    /// The PLM parts the active assembly uses that are checked out to the
    /// user ([`held_parts`]), named for the pane.
    held: Vec<String>,
    /// Per document, the PLM parts it has changed and not saved, with what the
    /// index said of each (editable; who else holds it): one released or
    /// taken by someone else meanwhile is told at once ([`Self::watch_changed`]).
    changed_rows: HashMap<u64, std::collections::BTreeMap<String, (bool, Option<String>)>>,
    /// Events a panel raised that another section acts on. S4's review section
    /// drains `OpenReview` here.
    pub routed: Vec<PanelEvent>,
    /// What the user picked while the pane drew (another revision to open),
    /// applied by the next [`Self::sync`], which knows the store.
    picked: Vec<PanelEvent>,
    /// This frame's widget rects ([`Self::hits_json`]).
    hits: Hits,
    /// The file lane's save count last seen ([`Self::sync`]'s `saves`).
    saves_seen: Option<u64>,
    /// Documents just saved whose revision is being re-read, to warn (D8) from
    /// the server's current word rather than a view another client may have
    /// moved on.
    saved_pending: Vec<u64>,
    /// Where the active document's part is used (S6).
    where_used: crate::panels::bom_plm::WhereUsedSection,
    /// The files section (S8): one attachments panel per open PLM document.
    attachments: crate::panels::plm_attachments::AttachmentsSection,
    /// The workspace section (S14).
    workspace: crate::panels::plm_workspace::WorkspaceBrowser,
    /// What the workspace asked for while the pane drew — document keys to open, a
    /// file to save — applied by the next [`Self::sync`], which knows the store.
    workspace_asked: Vec<crate::panels::plm_workspace::WorkspaceOutcome>,
    /// Following other clients live: the change feed, asked once per
    /// interval (`panels::plm_follow`).
    follower: crate::panels::plm_follow::Follower,
    /// When this host first synced, the follower's clock.
    started: Option<web_time::Instant>,
    /// The feed moved since the shell last asked ([`Self::take_followed`]).
    followed: bool,
    /// The re-point check on open (S5) and the answers given while the pane
    /// drew, applied by the next [`Self::sync`].
    repoint: crate::panels::plm_import::RepointCheck,
    cached: crate::panels::plm_import::CachedCopyCheck,
    /// Copies pinned to a superseded revision, and the answers given while the pane drew.
    superseded: crate::panels::plm_import::SupersededCheck,
    superseded_asked: Vec<(u64, bool)>,
    superseded_new: Vec<u64>,
    /// Open tabs follow their revisions when another client saves them
    /// (`panels::plm_tab_follow`), and the answers given while the pane drew.
    tab_follow: crate::panels::plm_tab_follow::TabFollow,
    tab_follow_asked: Vec<(u64, bool)>,
    /// The answers about edited copies whose revisions moved (`cached`).
    copies_asked: Vec<(u64, bool)>,
    repoint_asked: Vec<(u64, crate::panels::plm_import::RepointChoice)>,
    /// The parts section (S7), and what it asked for while it drew.
    parts: crate::panels::plm_parts::PlmPartsPanel,
    parts_asked: Vec<crate::panels::plm_parts::PlmPartsOutcome>,
    /// Parts picked for Insert, by store name and number, waiting for their
    /// document to be readable (an index-hydrated store loads it on demand).
    inserting: Vec<(String, String)>,
    /// A New part's first revision being written with the active document;
    /// its store name is opened when it lands.
    uploading: Option<(crate::panels::plm_parts::Pending<()>, String)>,
    /// The import section (S13).
    import: crate::panels::plm_import::PlmImportPanel,
    /// The review section (S4).
    review: crate::panels::plm_review::ReviewSection,
    /// Reviews asked for — by the lifecycle section, the
    /// toolbar's inbox — opened by the next [`Self::sync`], which has the client.
    review_asked: Vec<crate::panels::plm_review::PlmReviewEvent>,
    /// See [`Self::signed_out_here`].
    quiet_sign_out: bool,
}

impl PlmHost {
    pub fn new() -> Self {
        Self::default()
    }

    /// Once a frame, before the dock draws: take the store's client, give each
    /// PLM document its lifecycle panel, drive every panel's request, and apply
    /// what they answer to the documents.
    ///
    /// `saves` is the file lane's save count (`FileDialog::save_generation`):
    /// when it moves, the active document was just saved, and a save of a
    /// revision in review warns (D8, S4's `plm_review::save_warning`) — it is
    /// never refused.
    pub fn sync(&mut self, store: &dyn ModelStore, docs: &mut Documents, saves: u64) -> HostActions {
        self.web_base = web_url(store, "").unwrap_or_default();
        let client = store.plm_client();
        IN_SESSION.with(|flag| flag.set(client.is_some()));
        let Some(client) = client else {
            self.client = None;
            self.lifecycle.clear();
            return HostActions::default();
        };
        if !self.client.as_ref().is_some_and(|c| Rc::ptr_eq(c, &client)) {
            self.client = Some(client.clone());
            self.rights = None;
            self.rights_pending = Some(client.rights());
            self.lifecycle.clear();
            self.workspace = crate::panels::plm_workspace::WorkspaceBrowser::new();
            self.repoint = crate::panels::plm_import::RepointCheck::new();
            self.cached = crate::panels::plm_import::CachedCopyCheck::new();
            self.superseded = Default::default();
            self.superseded_asked.clear();
            self.superseded_new.clear();
            self.tab_follow = Default::default();
            self.tab_follow_asked.clear();
            self.copies_asked.clear();
            self.parts = crate::panels::plm_parts::PlmPartsPanel::new();
            self.import = crate::panels::plm_import::PlmImportPanel::new();
            self.review = crate::panels::plm_review::ReviewSection::new();
        }
        self.sync_sections(&client, store, docs);
        let mut actions = self.sync_with(&client, store, docs, saves);
        // A new revision made to use the current releases opens next to its source.
        for (source, key) in self.superseded.take_opened() {
            match docs.iter().find(|d| d.id() == source).and_then(|d| d.name()).and_then(|name| sibling_revision(name, &key)) {
                Some(name) => actions.open.push(name),
                None => actions.open.push(key),
            }
        }
        // The review section (S4): what the lifecycle section and the
        // toolbar's inbox asked to open, and the open pane's answers.
        let reviews = crate::panels::plm_review::PlmReviewClient { client: client.clone() };
        for event in std::mem::take(&mut self.review_asked) {
            self.review.open(&event, &reviews);
        }
        self.review.poll(&reviews);
        actions.refresh_inbox = self.review.take_acted();
        // Follow other clients: when anything moved on the server, re-read
        // every open PLM document's part (a panel already reading is skipped).
        let now = self.started.get_or_insert_with(web_time::Instant::now).elapsed().as_secs_f64();
        if let Some(moved) = self.follower.tick(now, &client) {
            self.parts.refresh_catalog(&crate::panels::plm_parts::PlmPartCatalog::new(client.clone()));
            if self.workspace.entries.is_some() {
                self.workspace.reload(&crate::panels::plm_workspace::PlmWorkspaces { client: client.clone() });
            }
            for panel in self.lifecycle.values_mut() {
                panel.refresh(&client);
            }
            // The store's held index rows (the BOM's PLM columns): exactly the
            // revisions the feed named, the whole index only when stale.
            store.refresh_plm_index(&moved.keys, moved.stale);
            self.tab_follow.moved(&client, docs, &moved.keys, moved.stale, store.pending_writes() > 0);
            self.cached.moved(docs, &moved.keys, moved.stale);
            self.followed = true;
        }
        // Said once when the server stops answering, and once when it is
        // back, with what is still to be saved, rather than one transport
        // error per failed write.
        let reach = self.follower.take_reach();
        let quiet = self.quiet_sign_out && matches!(reach, Some(crate::panels::plm_follow::Reach::SignedOut));
        if quiet {
            self.quiet_sign_out = false;
        }
        if let Some(reach) = reach.filter(|_| !quiet) {
            use crate::panels::plm_follow::Reach;
            use brep_render::engine_state::NoticeSeverity;
            let unsaved = docs.iter().filter(|d| d.name().is_some_and(|n| revision_of(n).is_some()) && d.is_dirty()).count();
            let (severity, text) = match reach {
                Reach::SignedOut => {
                    let why = "your PLM sign-in has ended (it was signed out or revoked on the server)".to_string();
                    actions.sign_in_again = Some(why.clone());
                    (
                        NoticeSeverity::Warning,
                        format!("{}. Sign in again in Settings › PLM: your work stays in the open tabs, and Save writes it once you are signed in.", capitalised(&why)),
                    )
                }
                Reach::SignedIn if unsaved > 0 => (
                    NoticeSeverity::Info,
                    format!("Signed in to the PLM again: Save to write the {unsaved} tab(s) with unsaved changes."),
                ),
                Reach::SignedIn => (NoticeSeverity::Info, "Signed in to the PLM again.".to_string()),
                Reach::Lost => (
                    NoticeSeverity::Warning,
                    "The PLM server is not reachable. Your work stays in the open tabs; a Save made now reaches nothing and the tab stays unsaved.".to_string(),
                ),
                Reach::Back if unsaved > 0 => (
                    NoticeSeverity::Info,
                    format!("The PLM server is reachable again: Save to write the {unsaved} tab(s) with unsaved changes."),
                ),
                Reach::Back => (NoticeSeverity::Info, "The PLM server is reachable again.".to_string()),
            };
            docs.engine_mut().push_notice_as(severity, text);
        }
        actions
    }

    /// A Save's write found the server unreachable: say so when it answers
    /// again, with the tabs still to save.
    pub fn note_unreachable(&mut self) {
        self.follower.note_unreachable();
    }

    /// Open a review in the review section (the toolbar's
    /// inbox rows). Opened by the next [`Self::sync`].
    /// A PLM part the active assembly has changed (not saved yet) that was
    /// released, or checked out by someone else, since: told now, while the
    /// user is still working on it, rather than at Save (which then makes a
    /// new revision for it, or keeps the change in the assembly).
    fn watch_changed(&mut self, docs: &mut Documents, store: &dyn ModelStore) {
        let open: Vec<u64> = docs.iter().map(|d| d.id()).collect();
        self.changed_rows.retain(|id, _| open.contains(id));
        let active = docs.active_id();
        let mut now = std::collections::BTreeMap::new();
        for key in docs.owned_modified_keys_of(active) {
            let Some((part, revision)) = crate::plm::bom::revision_of_document(&key) else { continue };
            let Some(row) = store.plm_revision(&crate::plm::identity::document_key(&part, &revision)) else { continue };
            let other = row.locked_by.clone().filter(|_| !row.locked_by_me);
            now.insert(key, (row.editable, other, row.lifecycle, row.part_name));
        }
        let before = self.changed_rows.remove(&active).unwrap_or_default();
        let mut told = Vec::new();
        for (key, (editable, other, lifecycle, name)) in &now {
            let Some((was_editable, was_other)) = before.get(key) else { continue };
            let shown = crate::document::owned_save::shown(key, std::slice::from_ref(name));
            if *was_editable && !editable {
                told.push(format!("{shown} was {lifecycle} on the PLM while you were changing it here: Save makes a new revision of it for your change"));
            } else if was_other.is_none() {
                if let Some(who) = other {
                    told.push(format!("{shown} was checked out by {who} while you were changing it here: Save keeps your change in this assembly until it can be written"));
                }
            }
        }
        for text in told {
            docs.tab_engine_mut().push_notice_as(brep_render::engine_state::NoticeSeverity::Warning, text);
        }
        self.changed_rows.insert(active, now.into_iter().map(|(key, (editable, other, ..))| (key, (editable, other))).collect());
    }

    pub fn open_review(&mut self, event: crate::panels::plm_review::PlmReviewEvent) {
        self.review_asked.push(event);
    }

    /// The review section's state, for tests (nothing draws the section since
    /// the pane split: review management is the web app's; no state blob).
    pub fn review_json(&self) -> String {
        self.review.state_json().to_string()
    }

    /// The feed moved since the last call: the shell asks the inbox again.
    pub fn take_followed(&mut self) -> bool {
        std::mem::take(&mut self.followed)
    }

    /// Seconds until the pane asks the feed again (the shell repaints then);
    /// `None` outside a PLM session.
    pub fn next_follow_in(&self) -> Option<f64> {
        let now = self.started?.elapsed().as_secs_f64();
        self.client.as_ref().map(|_| self.follower.next_in(now))
    }

    /// This machine signed out of the PLM (Settings › PLM › Forget): the
    /// refusal the feed meets next is expected, so it raises no prompt.
    pub fn signed_out_here(&mut self) {
        self.quiet_sign_out = true;
    }

    /// Ask the feed at the next sync instead of at the interval (scripts).
    pub fn follow_now(&mut self) {
        self.follower.ask_now();
    }

    /// The follower, for the state blob and tests.
    pub fn follower(&self) -> &crate::panels::plm_follow::Follower {
        &self.follower
    }

    /// [`Self::sync`] over any server — the tests' fake, or the real client.
    pub fn sync_with(&mut self, server: &dyn PlmLifecycle, store: &dyn ModelStore, docs: &mut Documents, saves: u64) -> HostActions {
        let mut actions = HostActions::default();
        // The file chooser exists natively (this machine's files) and in the
        // browser (the page's file input): every section may ask for a file.
        self.attachments.can_pick_files = true;
        self.workspace.can_pick_files = true;
        if let Some(id) = self.attachments.take_pick_request() {
            actions.pick_file = Some((format!("{PICK_ATTACHMENTS}{id}"), "Attach a file".into()));
        }
        // What the workspace section asked for while the pane drew: a link's document to
        // open (under the name the store spells it), a file to save.
        if self.workspace.pins.is_none() {
            self.workspace.pins = Some(crate::panels::plm_workspace::load_pins(store));
        }
        for asked in std::mem::take(&mut self.workspace_asked) {
            if let Some(pins) = &asked.pins {
                if let Err(e) = crate::panels::plm_workspace::save_pins(store, pins) {
                    self.workspace.notice = Some(format!("could not save the pinned folders: {e}"));
                }
            }
            if asked.pick_file {
                actions.pick_file = Some((PICK_WORKSPACE.into(), "Add a file to this folder".into()));
            }
            if let Some(key) = asked.open {
                let name = crate::panels::plm_workspace::document_name(store, &key);
                if let Some(folder) = asked.open_folder {
                    actions.opened_from.push((name.clone(), folder));
                }
                actions.open.push(name);
            }
            if let Some((name, bytes)) = asked.downloaded {
                actions.save_files.push(crate::panels::file::SaveRequest::bytes("Save the file", name, bytes));
            }
        }
        let saved = self.saves_seen.is_some_and(|seen| seen != saves);
        self.saves_seen = Some(saves);
        if saved {
            self.saved_pending.push(docs.active().id());
        }
        for id in &self.saved_pending {
            if let Some(panel) = self.lifecycle.get_mut(id) {
                panel.refresh(server);
            }
        }
        if let Some(pending) = self.rights_pending.as_mut() {
            let mut cx = std::task::Context::from_waker(std::task::Waker::noop());
            if let std::task::Poll::Ready(answer) = pending.as_mut().poll(&mut cx) {
                self.rights_pending = None;
                // A refusal leaves the rights empty: the panel offers nothing,
                // and the server would refuse every verb anyway.
                self.rights = Some(answer.unwrap_or_default());
            }
        }
        let open: Vec<(u64, String)> = docs.iter().filter_map(|d| Some((d.id(), d.name()?.to_string()))).collect();
        // Picks from the pane last frame belong to the document that was active.
        let active = docs.active().id();
        if let Some((_, name)) = open.iter().find(|(id, _)| *id == active).cloned() {
            for event in std::mem::take(&mut self.picked) {
                self.apply(event, active, &name, store, docs, &mut actions);
            }
        }
        self.lifecycle.retain(|id, panel| open.iter().any(|(open_id, name)| {
            open_id == id && revision_of(name).is_some_and(|(part, revision)| panel.matches_document(&part, &revision))
        }));
        for (id, name) in &open {
            let Some((part, revision)) = revision_of(name) else { continue };
            if !self.lifecycle.contains_key(id) {
                let Some(rights) = self.rights else {
                    // Pending rights restrict saving, never session editing.
                    set_access(docs, *id, pending_access());
                    continue;
                };
                // A successful Save As already knows its destination is checked out.
                // Keep that permission while fetching metadata, so stale cached bytes
                // cannot replace the session copy as though this were a new checkout.
                if !docs.iter().find(|d| d.id() == *id).is_some_and(crate::document::Document::has_plm_save_access) {
                    set_access(docs, *id, pending_access());
                }
                self.lifecycle.insert(*id, PlmPanel::new(server, &part, &revision, rights));
            }
            let Some(panel) = self.lifecycle.get_mut(id) else { continue };
            for event in panel.poll(server) {
                self.apply(event, *id, name, store, docs, &mut actions);
            }
        }
        // The PLM parts the active assembly uses that are checked out to the
        // user: its Saves check them out, and nothing else gives them back.
        let active = docs.active_id();
        self.held = match self.lifecycle.get(&active) {
            Some(panel) => held_parts(docs.tab_engine().history.parts_library(), store, panel),
            None => Vec::new(),
        };
        self.watch_changed(docs, store);
        let settled: Vec<u64> = self
            .saved_pending
            .iter()
            .copied()
            .filter(|id| self.lifecycle.get(id).is_none_or(|p| !p.busy()))
            .collect();
        self.saved_pending.retain(|id| !settled.contains(id));
        for id in settled {
            self.warn_saved_in_review(docs, id);
        }
        // The files section's exports and downloads need the documents
        // mutably, which drawing does not have.
        actions.save_files.extend(self.attachments.sync(docs));
        self.route();
        actions
    }

    /// The S5 / S7 / S13 sections' store-and-document work for this frame:
    /// the re-point check and its answers, Insert and New part, the import.
    fn sync_sections(&mut self, client: &Rc<PlmClient>, store: &dyn ModelStore, docs: &mut Documents) {
        let waker = std::task::Waker::noop();
        self.parts.browser.poll(waker);
        // The workspace's requests are driven here too, not only while its section is
        // drawn: a request another section started (a New part linked into the folder)
        // must land with the pane hidden.
        {
            let plm = crate::panels::plm_workspace::PlmWorkspaces { client: client.clone() };
            let mut outcome = crate::panels::plm_workspace::WorkspaceOutcome::default();
            self.workspace.poll(waker, &plm, &mut outcome);
            if outcome != crate::panels::plm_workspace::WorkspaceOutcome::default() {
                self.workspace_asked.push(outcome);
            }
        }
        self.repoint.sync(client, docs, waker);
        let writing = store.pending_writes() > 0;
        self.cached.sync(client, store, docs, waker, writing);
        self.superseded.sync(client, store, docs, waker);
        for (id, update) in std::mem::take(&mut self.superseded_asked) {
            self.superseded.answer(docs, store, id, update);
        }
        for id in std::mem::take(&mut self.superseded_new) {
            self.superseded.make_new_revision(client, docs, id);
        }
        for (id, take) in std::mem::take(&mut self.copies_asked) {
            self.cached.answer(docs, id, take);
        }
        self.tab_follow.poll(docs, waker, writing);
        for (id, take) in std::mem::take(&mut self.tab_follow_asked) {
            self.tab_follow.answer(docs, id, take);
        }
        for (id, choice) in std::mem::take(&mut self.repoint_asked) {
            if let Err(problem) = self.repoint.answer(docs, id, choice) {
                docs.engine_mut().push_notice(format!("Re-point: {problem}"));
            }
        }
        for outcome in std::mem::take(&mut self.parts_asked) {
            // "Link <number> to workspace" from the catalog: into the folder the
            // workspace section shows, following the newest revision.
            if let Some(row) = &outcome.link {
                let plm = crate::panels::plm_workspace::PlmWorkspaces { client: client.clone() };
                self.workspace.link(&plm, &row.id, "");
            }
            if let Some(row) = outcome.insert {
                let key = crate::plm::identity::document_key(&row.id, &row.latest_revision_id);
                self.inserting.push((crate::panels::plm_workspace::document_name(store, &key), row.number));
            }
            if let Some(created) = outcome.created {
                // D9: the SERVER linked the new part into the workspace folder the
                // user is in (the request's `workspace_folder`), following its newest
                // revision; show it there.
                let plm = crate::panels::plm_workspace::PlmWorkspaces { client: client.clone() };
                self.workspace.reload(&plm);
                if let Some(key) = created.first_revision_key() {
                    let (client, mut document) = (client.clone(), docs.engine().history_request_json());
                    if docs.engine().history_len() == 0 {
                        // New part adopts a COPY, including when the source is
                        // read-only. Seed the destination without editing that source.
                        let mut engine = docs.spawn_engine();
                        if let Err(problem) = engine.load_model_and_fit(&document) {
                            docs.engine_mut().push_notice(format!("The new part could not be prepared: {problem}"));
                            continue;
                        }
                        crate::document::seed_new_part_datum(&mut engine);
                        document = engine.history_request_json();
                    }
                    let at = key.clone();
                    let write: crate::plm::PlmFuture<()> =
                        Box::pin(async move { crate::plm::adoption::load_into_revision(&client, &at, &document).await });
                    self.uploading = Some((crate::panels::plm_parts::Pending::new(write), crate::panels::plm_workspace::document_name(store, &key)));
                }
            }
        }
        self.inserting.retain(|(name, number)| match crate::store::read_now(store, name) {
            crate::store::ReadNow::Ready(contents) => {
                let doc = docs.active_mut();
                if !doc.access().is_editable() {
                    doc.engine.push_notice(format!("Insert {number}: check this document out first"));
                    return false;
                }
                let inserted = doc.engine.insert_component(brep_render::engine_state::ComponentInsert::New {
                    name: number,
                    source_key: name,
                    source_signature: &crate::panels::parts_library::document_signature(&contents),
                    document_json: &contents,
                });
                if let Err(problem) = inserted {
                    doc.engine.push_notice(format!("Insert {number}: {problem}"));
                }
                false
            }
            // Listed but not loaded: the read just asked for it.
            crate::store::ReadNow::Loading => true,
            crate::store::ReadNow::Absent => {
                docs.engine_mut().push_notice(format!("Insert {number}: its newest revision has no document yet"));
                false
            }
        });
        if let Some(answer) = self.uploading.as_mut().and_then(|(pending, _)| pending.poll(waker)) {
            let (_, name) = self.uploading.take().unwrap();
            match answer {
                Ok(()) => docs.engine_mut().push_notice_as(
                    brep_render::engine_state::NoticeSeverity::Info,
                    format!("This document is now the new part's first revision: {name}"),
                ),
                Err(problem) => docs.engine_mut().push_notice(format!("The part was created, but this document was not written into it: {problem}")),
            }
        }
        let server = crate::plm::connection::parse_label(&store.backend_label()).map(|(url, _)| url).unwrap_or_default();
        // The folder is read from this machine's files, never the PLM's.
        self.import.sync(store, store.local_files(), Some(client.clone()), &server, waker);
    }

    fn apply(
        &mut self,
        event: PanelEvent,
        id: u64,
        name: &str,
        store: &dyn ModelStore,
        docs: &mut Documents,
        actions: &mut HostActions,
    ) {
        match event {
            PanelEvent::Access(access) => {
                let was_editable = docs.iter().find(|d| d.id() == id).is_some_and(|d| d.save_access().is_editable());
                set_access(docs, id, access.clone());
                let unsaved_edits = docs.iter().find(|d| d.id() == id).is_some_and(|doc| doc.is_dirty());
                if access.is_editable() && !was_editable && !unsaved_edits && reload(docs, id, name, store) {
                    // The reload takes the stored document, whose embedded copies
                    // the open's cached-copy check had refreshed: check them
                    // again, or the refresh is silently undone.
                    self.cached.recheck(id);
                }
            }
            PanelEvent::Moved(change) => {
                if let Some(doc) = docs.iter_mut().find(|d| d.id() == id) {
                    let (severity, text) = moved_notice(&doc.title(), doc.is_dirty(), &change);
                    doc.engine.push_notice_as(severity, text);
                }
            }
            PanelEvent::Open(key) => {
                if let Some(sibling) = sibling_revision(name, &key) {
                    actions.open.push(sibling);
                }
            }
            other => self.routed.push(other),
        }
    }

    /// D8: the active document was saved; if its revision is in review, say
    /// what that save means for the reviewers.
    fn warn_saved_in_review(&self, docs: &mut Documents, id: u64) {
        let Some(revision) = self.lifecycle.get(&id).and_then(PlmPanel::revision) else { return };
        let Some(warning) = crate::panels::plm_review::save_warning(&revision.label, &revision.lifecycle) else { return };
        if let Some(doc) = docs.iter_mut().find(|d| d.id() == id) {
            doc.engine.push_notice_as(brep_render::engine_state::NoticeSeverity::Warning, warning);
        }
    }

    /// Hand the routed events to whichever section acts on them: the
    /// lifecycle section's Submit for review, Review…
    /// open the review section. Run at the end of every [`Self::sync_with`].
    pub fn route(&mut self) {
        for event in std::mem::take(&mut self.routed) {
            if let Some(event) = crate::panels::plm_review::PlmReviewEvent::from_lifecycle(&event) {
                self.review_asked.push(event);
            }
        }
    }

    /// Whether a PLM request any section made is still unanswered: the idle
    /// contract waits for it (`automation::cmd_frame`), and the shell keeps
    /// repainting until it lands.
    pub fn busy(&self) -> bool {
        self.rights_pending.is_some()
            || self.lifecycle.values().any(PlmPanel::busy)
            || self.attachments.busy()
            || self.workspace.busy()
            || self.follower.busy()
            || self.repoint.busy()
            || self.cached.busy()
            || self.superseded.busy()
            || self.tab_follow.busy()
            || self.parts.busy()
            || !self.inserting.is_empty()
            || self.uploading.is_some()
            || self.import.running()
            || self.where_used.busy()
            || self.review.busy()
            || !self.review_asked.is_empty()
    }

    /// Open the parts section on its New part form: Save As → "A new part" (D9). The
    /// form writes the active document into the new part's first revision, and the
    /// server links the part into the workspace folder shown.
    pub fn open_new_part(&mut self) {
        self.parts.tab = crate::panels::plm_parts::Tab::NewPart;
    }

    /// The lifecycle panel of document `id`, if it is a PLM document.
    pub fn lifecycle(&self, id: u64) -> Option<&PlmPanel> {
        self.lifecycle.get(&id)
    }

    /// The same, to press a verb on (the automation layer, tests).
    pub fn lifecycle_mut(&mut self, id: u64) -> Option<&mut PlmPanel> {
        self.lifecycle.get_mut(&id)
    }

    /// Draw the PLM pane for the active document.
    pub fn ui(&mut self, ui: &mut egui::Ui, docs: &Documents) {
        self.hits.clear();
        // The pane's visible region: `click_widget` wheels the pane until a
        // widget is inside it, so a section below the fold is reachable.
        self.hits.insert("panel:clip".into(), ui.clip_rect());
        let Some(client) = self.client.clone() else {
            ui.label("Not connected to a PLM.");
            return;
        };
        let active = docs.active_id();
        if self.tab_follow.asking(active) {
            ui.colored_label(ui.visuals().warn_fg_color, "Another client saved this revision while you have unsaved edits here.");
            ui.horizontal(|ui| {
                let keep = ui.button("Keep my edits");
                self.hits.insert("follow:keep".into(), keep.rect);
                if keep.clicked() {
                    self.tab_follow_asked.push((active, false));
                }
                let take = ui.button("Take their version");
                self.hits.insert("follow:take".into(), take.rect);
                if take.on_hover_text("Discards your unsaved edits in this tab").clicked() {
                    self.tab_follow_asked.push((active, true));
                }
            });
            ui.separator();
        }
        if self.superseded.asking(active) {
            let found = self.superseded.found(active);
            let lines: Vec<String> = found
                .iter()
                .map(|s| format!("{}: {} uses {} rev {}; rev {} is current.", s.paths.join(", "), s.number, s.lifecycle, s.from, s.to))
                .collect();
            ui.colored_label(ui.visuals().warn_fg_color, lines.join("\n"));
            ui.horizontal(|ui| {
                let update = ui.button(if found.len() == 1 { format!("Update to rev {}", found[0].to) } else { "Update them".to_string() });
                self.hits.insert("superseded:update".into(), update.rect);
                if update.on_hover_text("Replace with the current release; mates are kept as Replace keeps them. One undo step per part.").clicked() {
                    self.superseded_asked.push((active, true));
                }
                let keep = ui.button("Not now");
                self.hits.insert("superseded:keep".into(), keep.rect);
                if keep.clicked() {
                    self.superseded_asked.push((active, false));
                }
            });
            ui.separator();
        }
        if let Some(why) = self.superseded.keeps(active).map(str::to_string) {
            let found = self.superseded.found(active);
            let to: Vec<String> = found.iter().map(|s| format!("{} rev {}", s.number, s.to)).collect();
            ui.colored_label(
                ui.visuals().warn_fg_color,
                format!("This assembly keeps the revisions it was built with ({why}). {} superseded.", if found.len() == 1 { "One part it uses is" } else { "Parts it uses are" }),
            );
            if self.superseded.creating(active) {
                ui.horizontal(|ui| { ui.spinner(); ui.label("Making its next revision…"); });
            } else {
                let make = ui.button(format!("Make a new revision using {}", to.join(", ")));
                self.hits.insert("superseded:new-revision".into(), make.rect);
                if make.on_hover_text("Its next revision, checked out to you, opens with the current revisions in place; this one is not changed").clicked() {
                    self.superseded_new.push(active);
                }
            }
            ui.separator();
        }
        if let Some(paths) = self.cached.asking(active) {
            ui.colored_label(
                ui.visuals().warn_fg_color,
                format!("{} changed on the PLM, and you have unsaved edits to it in this assembly.", paths.join(", ")),
            );
            ui.horizontal(|ui| {
                let keep = ui.button("Keep my edits");
                self.hits.insert("follow:copies-keep".into(), keep.rect);
                if keep.clicked() {
                    self.copies_asked.push((active, false));
                }
                let take = ui.button("Take their version");
                self.hits.insert("follow:copies-take".into(), take.rect);
                if take.on_hover_text("Discards your unsaved edits to those parts in this assembly").clicked() {
                    self.copies_asked.push((active, true));
                }
            });
            ui.separator();
        }
        let mut target = None;
        section(ui, &mut self.hits, LIFECYCLE, |ui, hits| {
            let Some(panel) = self.lifecycle.get_mut(&active) else {
                ui.label("Open a PLM part using File → Open.");
                return;
            };
            if let Some(part) = panel.part().cloned() {
                ui.heading(format!("{} · {}", part.number, part.name));
                if let Some(revision) = panel.revision() {
                    target = Some((part.id.clone(), revision.id.clone()));
                    match access_with(revision, panel.rights()) {
                        Access::Editable => { ui.label("Checked out by you"); }
                        Access::ReadOnly { reason } => { ui.colored_label(ui.visuals().warn_fg_color, format!("Session edits allowed; saving unavailable: {reason}")); }
                    }
                }
                // The revisions as one dropdown (revision, its open entries
                // revision#<label>); choosing another opens it.
                ui.horizontal(|ui| {
                    ui.label("Open revision");
                    let choices: Vec<(usize, &str, &str)> =
                        part.revision_views.iter().enumerate().map(|(i, revision)| (i, revision.label.as_str(), revision.label.as_str())).collect();
                    let open = part.revision_views.iter().position(|revision| panel.revision().is_some_and(|r| r.id == revision.id));
                    let mut chosen = open.unwrap_or(usize::MAX);
                    let mut local = std::collections::HashMap::new();
                    let (changed, _) = crate::form::enum_dropdown(ui, "revision", &choices, &mut chosen, None, Some(&mut local));
                    for (key, rect) in local {
                        hits.put(&key, rect);
                    }
                    if changed && Some(chosen) != open {
                        if let Some(revision) = part.revision_views.get(chosen) {
                            self.picked.push(PanelEvent::Open(revision.document_key.clone()));
                        }
                    }
                });
                if panel.offered().contains(&crate::panels::plm::Verb::CheckOut) {
                    let button = ui.button("Check out");
                    hits.put("verb:check-out", button.rect);
                    if button.clicked() { panel.press(&client, crate::panels::plm::Verb::CheckOut); }
                }
                // What its Saves checked out stays checked out to the user
                // (and locked to everyone else) until it is checked in on the
                // PLM, where the assembly checks in with its children.
                if !self.held.is_empty() {
                    let count = self.held.len();
                    ui.colored_label(
                        ui.visuals().warn_fg_color,
                        format!(
                            "{count} part{} this assembly uses {} checked out to you: {}. Check {} in with the assembly on the PLM (Check in with its children).",
                            if count == 1 { "" } else { "s" },
                            if count == 1 { "is" } else { "are" },
                            self.held.join(", "),
                            if count == 1 { "it" } else { "them" },
                        ),
                    );
                    if let Some(revision) = panel.revision() {
                        let route = format!("#/part/{}/{}/summary", crate::plm::identity::segment(&part.id), crate::plm::identity::segment(&revision.id));
                        let link = web_link(ui, "Check in on the PLM", format!("{}{route}", self.web_base));
                        hits.put("held:check-in", link.rect);
                    }
                }
            }
            if panel.busy() { ui.spinner(); ui.ctx().request_repaint(); }
            if let Some(message) = panel.message() { ui.label(message); }
        });
        ui.separator();
        ui.label("Manage in PLM");
        if let Some((part, revision)) = target {
            let route = format!("#/part/{}/{}", crate::plm::identity::segment(&part), crate::plm::identity::segment(&revision));
            for (label, tab) in [("Part details, check-in and release", "summary"), ("Revisions", "revisions"), ("Attachments", "attachments"), ("BOM and where used", "structure"), ("History", "history")] {
                let link = web_link(ui, label, format!("{}{route}/{tab}", self.web_base));
                self.hits.insert(format!("web:{tab}"), link.rect);
            }
        }
        for (label, route) in [("Parts and new parts", "parts"), ("Workspace", "workspace"), ("Reviews and approvals", "reviews")] {
            let link = web_link(ui, label, format!("{}#/{route}", self.web_base));
            self.hits.insert(format!("web:{route}"), link.rect);
        }
    }

    pub fn pick_tags(&self) -> Vec<String> {
        let mut tags = vec![PICK_WORKSPACE.to_string()];
        tags.extend(self.attachments.panel_ids().into_iter().map(|id| format!("{PICK_ATTACHMENTS}{id}")));
        tags
    }

    /// The file the chooser delivered for `tag`: staged on the attachments
    /// panel that asked, or added to the workspace folder shown.
    pub fn deliver_picked(&mut self, tag: &str, file: crate::panels::file::PickedFile) {
        let Some(client) = self.client.clone() else { return };
        if tag == PICK_WORKSPACE {
            let plm = crate::panels::plm_workspace::PlmWorkspaces { client };
            self.workspace.add_file(&plm, &file.name, file.bytes);
        } else if let Some(id) = tag.strip_prefix(PICK_ATTACHMENTS).and_then(|id| id.parse::<u64>().ok()) {
            self.attachments.stage_picked(id, &file.name, file.bytes);
        }
    }

    /// `__brepPlmHit`: the pane's widget rects.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// `__brepPlmDoc`: the active document's standing, for scripts. Null outside
    /// a PLM session.
    pub fn state_json(&self, docs: &Documents) -> String {
        if self.client.is_none() {
            return "null".into();
        }
        let doc = docs.active();
        let access = match doc.save_access() {
            Access::Editable => serde_json::json!({ "editable": doc.access().is_editable(), "canSave": true }),
            Access::ReadOnly { reason } => serde_json::json!({ "editable": doc.access().is_editable(), "canSave": false, "reason": reason }),
        };
        let panel = self.lifecycle.get(&doc.id());
        let revision = panel.and_then(|p| p.revision()).map(|r| {
            serde_json::json!({
                "id": r.id, "label": r.label, "lifecycle": r.lifecycle,
                "lockedBy": r.locked_by, "lockedByMe": r.locked_by_me,
            })
        });
        serde_json::json!({
            "access": access,
            "plmDocument": panel.is_some(),
            "revision": revision,
            "busy": panel.is_some_and(|p| p.busy()),
            "message": panel.and_then(|p| p.message()),
            "offered": panel.map(|p| p.offered().iter().filter(|v| **v == crate::panels::plm::Verb::CheckOut).map(|v| v.key()).collect::<Vec<_>>()).unwrap_or_default(),
            // The PLM parts this assembly uses that are checked out to the
            // user (Check in with parts gives them back).
            "held": self.held,
            // The Files section (S8): what is staged, listed, refused.
            "files": self.attachments.state_json(doc.id()),
            // The Parts list (S5): whether this document disagrees with its
            // revision's uses list, and how many re-points are offered.
            "uses": {
                "checking": self.repoint.busy(),
                "disagrees": self.repoint.disagreement(doc.id()).is_some(),
                "repoints": self.repoint.disagreement(doc.id()).map_or(0, |d| d.repoints.len()),
            },
            // Embedded copies older than their PLM revisions, refreshed on open.
            "cachedCopies": { "checking": self.cached.busy(), "refreshed": self.cached.refreshed(doc.id()), "unread": self.cached.unread(doc.id()), "live": self.cached.live_json(doc.id()) },
            // Copies pinned to a superseded or obsolete revision (`plm::superseded`).
            "superseded": self.superseded.state_json(doc.id()),
            // Requests the session's PLM client has sent: an operation's cost is
            // this read before and after it.
            "requestsSent": self.client.as_ref().map(|c| c.requests_sent()),
            "requestsByRoute": self.client.as_ref().map(|c| c.requests_by_route()),
            // Another client saved this tab's revision (`panels::plm_tab_follow`).
            "followTab": self.tab_follow.state_json(doc.id()),
            // The Parts section (S7).
            "parts": self.parts.state_json(),
            "workspaceThumbnails": self.workspace.thumbnail_entries(),
            // The Import section (S13).
            "import": self.import.state_json(),
            // The Where used section (S6): the part's assemblies, every level.
            "whereUsed": self.where_used.state_json(),
        })
        .to_string()
    }
}

/// The one notice a change on the PLM under an open tab gets: what moved,
/// and, when the tab holds unsaved edits, what Save will do with them now.
fn moved_notice(title: &str, unsaved: bool, change: &crate::panels::plm::Moved) -> (brep_render::engine_state::NoticeSeverity, String) {
    use crate::panels::plm::Moved;
    use brep_render::engine_state::NoticeSeverity::{Info, Warning};
    let kept = "your unsaved changes are only in this tab";
    match change {
        // The checkout went away (checked in on the web, or a lock broken
        // and maybe taken): say so now, not at the next Save.
        Moved::LockLost { holder, by } => {
            let how = match by {
                crate::panels::plm::LockGone::BrokenBy(user) => format!(" ({user} broke your lock)"),
                crate::panels::plm::LockGone::You => " (you checked it in on the PLM)".to_string(),
                crate::panels::plm::LockGone::Unknown => String::new(),
            };
            let next = match (holder, unsaved) {
                (Some(user), true) => format!("{user} has it checked out now; Save offers Save As to keep your changes as a new part"),
                (None, true) => "Save offers to check it out again".to_string(),
                (Some(user), false) => format!("{user} has it checked out now"),
                (None, false) => "check it out again to save to it".to_string(),
            };
            if unsaved {
                (Warning, format!("{title} is no longer checked out to you{how}, and {kept}: {next}"))
            } else {
                (Info, format!("{title} is no longer checked out to you{how}: {next}"))
            }
        }
        Moved::CheckedOutBy(user) if unsaved => {
            (Warning, format!("{user} checked out {title}, and {kept}: Save offers Save As to keep them as a new part"))
        }
        Moved::CheckedOutBy(user) => (Info, format!("{user} checked out {title}")),
        Moved::Lifecycle { to, by } => {
            let by = by.as_deref().map(|user| format!(" by {user}")).unwrap_or_default();
            let what = match to.as_str() {
                "released" => format!("{title} was released{by}"),
                "obsolete" => format!("{title} was made obsolete"),
                "superseded" => format!("{title} was superseded by a newer release"),
                other => format!("{title} is {other} now"),
            };
            if unsaved {
                (Warning, format!("{what}, and {kept}: Save offers Save As to keep them as a new revision"))
            } else {
                (Info, format!("{what}: it is read-only now"))
            }
        }
        Moved::Renamed(name) => (Info, format!("{title} is named \"{name}\" on the PLM now")),
    }
}

fn set_access(docs: &mut Documents, id: u64, access: Access) {
    if let Some(doc) = docs.iter_mut().find(|d| d.id() == id) {
        if doc.save_access() != access || !doc.access().is_editable() {
            doc.set_plm_save_access(access);
        }
    }
}

/// Re-read a document that just became editable, if the store holds other
/// bytes than it shows: someone saved it since it was opened.
/// Take the stored `name` into tab `id` when it differs; `true` when it did.
fn reload(docs: &mut Documents, id: u64, name: &str, store: &dyn ModelStore) -> bool {
    let Some(text) = store.read(name) else { return false };
    let Some(doc) = docs.iter_mut().find(|d| d.id() == id) else { return false };
    // The bytes it was opened from: the open's own runs have written into the
    // tab since (snapshots, a first mate solve), so the compare below always
    // differs, and the reload ran the whole document a second time per open.
    if doc.holds_stored(&text) {
        return false;
    }
    let same = serde_json::from_str::<serde_json::Value>(&text).ok()
        == serde_json::from_str::<serde_json::Value>(&doc.engine.history.request_json()).ok();
    if same {
        return false;
    }
    if doc.engine.set_history_json(&text).is_ok() {
        doc.mark_clean();
        doc.note_opened_from(&text);
        return true;
    }
    false
}

