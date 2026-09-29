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
//! A document opened from the PLM (its name carries a `part/<p>/rev/<r>` key,
//! and only while the store IS a PLM) gets a [`PlmPanel`] the frame it
//! appears. Until the server has answered, the document is read-only ("reading
//! the revision from the PLM"): nothing is editable before the server says it
//! may be. From then on the panel's access is the document's
//! ([`crate::document::Document::set_access`]). When access turns editable (a
//! checkout), the document is re-read from the store: someone may have saved
//! since it was opened, and the lock is now this user's.

use std::collections::HashMap;
use std::rc::Rc;

use eframe::egui;

use crate::document::{Access, Documents};
use crate::panels::plm::{access_for, PanelEvent, PlmLifecycle, PlmPanel, Rights};
use crate::plm::client::PlmClient;
use crate::plm::PlmFuture;
use crate::store::ModelStore;

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

/// `part/<part>/rev/<revision>` read out of a document name as the explorer
/// spells it (`/models/part/<p>/rev/<r>.nbrep`, or the bare key). Only a PLM
/// session may read names this way: the shape alone is not proof.
pub fn revision_of(name: &str) -> Option<(String, String)> {
    let start = name.find("part/")?;
    let key = &name[start..];
    let parts: Vec<&str> = key.split('/').collect();
    match parts.as_slice() {
        ["part", part, "rev", revision] if !part.is_empty() && !revision.is_empty() => {
            let revision = revision.split('.').next().unwrap_or(revision);
            (!revision.is_empty()).then(|| (part.to_string(), revision.to_string()))
        }
        _ => None,
    }
}

/// The name to open another revision of the same part under: `name` with its
/// revision swapped, so the explorer's prefix and the class's extension stay.
pub fn sibling_revision(name: &str, key: &str) -> Option<String> {
    let start = name.find("part/")?;
    let (_, new_revision) = revision_of(key)?;
    let (_, old_revision) = revision_of(name)?;
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
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "panel:clip", meaning: "the visible region of the PLM pane (click_widget scrolls the pane until a widget is inside it)", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "section:", meaning: "a section's header, by id: section:lifecycle, section:where-used, section:uses, section:attachments, section:workspace, section:parts, section:import", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "lifecycle:", meaning: "the revision section (S3): lifecycle:revision:<label>, lifecycle:verb:<check-out|check-in|break-lock|release|new-revision|submit|review|open-change-order>, lifecycle:new-label, lifecycle:new-revision:where-used:<key>, lifecycle:history:load", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "where-used:", meaning: "the where-used section: its rows and controls", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "uses:", meaning: "the parts-list section (S5): re-point the occurrences a Replace everywhere moved", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "attachments:", meaning: "the files section (S8): its rows, download, attach", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "workspace:", meaning: "the workspace section (S14), the workspace browser's keys under workspace:ws:… (entry:<name>, crumb:<i>, open-part / open-revision / open-go, new-folder, link-*, sel:*, version:<n>:*, pin-folder, pinned:<i>, paste, reload, owner)", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "review:", meaning: "the review section (S4): the round, decisions, discussion, and the change order's", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "parts:", meaning: "the parts section (S7): the catalog browser, Insert and New part", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "plm", prefix: "import:", meaning: "the import section (S13): import a folder of files", command: None },
];

pub const SECTIONS: &[(&str, &str)] = &[
    (LIFECYCLE, "Revision"),
    (WHERE_USED, "Where used"),
    (USES, "Parts list"),
    (REVIEW, "Review"),
    (ATTACHMENTS, "Files"),
    (WORKSPACE, "Workspace"),
    (IMPORT, "Import a folder"),
    // LAST: the catalog grows with every page loaded (200 rows is thousands of
    // points), and a section drawn under it would be buried.
    (PARTS, "Parts"),
];

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

/// The review section: a revision's review round or a change order (S4).
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
    /// A section asked for a file: open the file chooser for (tag, title),
    /// and hand what it delivers to [`PlmHost::deliver_picked`].
    pub pick_file: Option<(String, String)>,
    /// The user decided, submitted, commented or released: ask the review
    /// inbox again (`ToolbarPanel::refresh_inbox`).
    pub refresh_inbox: bool,
}

/// The file-chooser tag of the workspace section.
pub const PICK_WORKSPACE: &str = "plm:workspace";
/// The file-chooser tag prefix of the attachments section: `plm:attachments:<document id>`.
pub const PICK_ATTACHMENTS: &str = "plm:attachments:";

/// Every PLM panel, and the one client they share.
#[derive(Default)]
pub struct PlmHost {
    client: Option<Rc<PlmClient>>,
    rights: Option<Rights>,
    rights_pending: Option<PlmFuture<Rights>>,
    /// The lifecycle panel of each open PLM document, by document id.
    lifecycle: HashMap<u64, PlmPanel>,
    /// Events a panel raised that another section acts on. S4's review section
    /// drains `OpenReview` / `OpenChangeOrder` here.
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
    /// Reviews and change orders asked for — by the lifecycle section, the
    /// toolbar's inbox — opened by the next [`Self::sync`], which has the client.
    review_asked: Vec<crate::panels::plm_review::PlmReviewEvent>,
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
            self.parts = crate::panels::plm_parts::PlmPartsPanel::new();
            self.import = crate::panels::plm_import::PlmImportPanel::new();
            self.review = crate::panels::plm_review::ReviewSection::new();
        }
        self.sync_sections(&client, store, docs);
        let mut actions = self.sync_with(&client, store, docs, saves);
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
            self.followed = true;
        }
        actions
    }

    /// Open a review or a change order in the review section (the toolbar's
    /// inbox rows). Opened by the next [`Self::sync`].
    pub fn open_review(&mut self, event: crate::panels::plm_review::PlmReviewEvent) {
        self.review_asked.push(event);
    }

    /// `__brepPlmReview`: the review section's state, for scripts.
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
                actions.open.push(crate::panels::plm_workspace::document_name(store, &key));
            }
            if let Some((name, bytes)) = asked.downloaded {
                self.workspace.notice = Some(match store.export_file_named_bytes(&name, &bytes) {
                    Ok(()) => format!("saved {name}"),
                    Err(e) => format!("could not save {name}: {e}"),
                });
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
        self.lifecycle.retain(|id, _| open.iter().any(|(open_id, _)| open_id == id));
        for (id, name) in &open {
            let Some((part, revision)) = revision_of(name) else { continue };
            if !self.lifecycle.contains_key(id) {
                let Some(rights) = self.rights else {
                    // Until the rights are known the document stays read-only.
                    set_access(docs, *id, pending_access());
                    continue;
                };
                set_access(docs, *id, pending_access());
                self.lifecycle.insert(*id, PlmPanel::new(server, &part, &revision, rights));
            }
            let Some(panel) = self.lifecycle.get_mut(id) else { continue };
            for event in panel.poll(server) {
                self.apply(event, *id, name, store, docs, &mut actions);
            }
        }
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
        // The files section's exports and downloads need the documents and the
        // store mutably, which drawing does not have.
        self.attachments.sync(store, docs);
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
                let key = format!("part/{}/rev/{}", row.id, row.latest_revision_id);
                self.inserting.push((crate::panels::plm_workspace::document_name(store, &key), row.number));
            }
            if let Some(created) = outcome.created {
                // D9: the SERVER linked the new part into the workspace folder the
                // user is in (the request's `workspace_folder`), following its newest
                // revision; show it there.
                let plm = crate::panels::plm_workspace::PlmWorkspaces { client: client.clone() };
                self.workspace.reload(&plm);
                if let Some(key) = created.first_revision_key() {
                    let (client, document) = (client.clone(), docs.engine().history_request_json());
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
                let was_editable = docs.iter().find(|d| d.id() == id).is_some_and(|d| d.access().is_editable());
                set_access(docs, id, access.clone());
                if access.is_editable() && !was_editable {
                    reload(docs, id, name, store);
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
    /// lifecycle section's Submit for review, Review… and Open change order
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
        let active = docs.active().id();
        let (lifecycle, hits, picked, where_used) = (&mut self.lifecycle, &mut self.hits, &mut self.picked, &mut self.where_used);
        let part = lifecycle.get(&active).and_then(|p| p.part()).map(|p| p.id.clone());
        let mut confirm_hits = Hits::new();
        section(ui, hits, LIFECYCLE, |ui, hits| match lifecycle.get_mut(&active) {
            // Drawn, not polled: `sync` polls every panel, so what a request
            // answers is applied there, once. New revision confirms first,
            // showing where the part is used.
            Some(panel) => {
                let mut before = |ui: &mut egui::Ui| {
                    if let Some(part) = &part {
                        let mut put = |key: &str, rect: egui::Rect| {
                            confirm_hits.insert(format!("{LIFECYCLE}:new-revision:where-used:{key}"), rect);
                        };
                        where_used.draw(ui, &client, part, &mut put);
                    }
                };
                picked.extend(panel.draw_with(ui, &client, hits, &mut before));
            }
            None => {
                ui.label("This document is not from the PLM.");
            }
        });
        hits.extend(confirm_hits);
        if let Some(part) = &part {
            section(ui, hits, WHERE_USED, |ui, hits| {
                where_used.draw(ui, &client, part, &mut |key, rect| hits.put(key, rect));
            });
        }
        self.uses_section(ui, active);
        let (review, hits) = (&mut self.review, &mut self.hits);
        section(ui, hits, REVIEW, |ui, hits| {
            let reviews = crate::panels::plm_review::PlmReviewClient { client: client.clone() };
            review.ui(ui, &reviews);
            for (key, rect) in review.hits() {
                hits.put(key, *rect);
            }
        });
        let target = docs.active().name().and_then(revision_of);
        // Exports are attached as `<part number>-<revision label>`, once the
        // lifecycle section has read them.
        let stem = self
            .lifecycle
            .get(&active)
            .and_then(|p| Some(format!("{}-{}", p.part()?.number, p.revision()?.label)))
            .unwrap_or_else(|| "document".into());
        let plm = crate::panels::plm_attachments::PlmAttachments { client: client.clone() };
        let (attachments, hits) = (&mut self.attachments, &mut self.hits);
        section(ui, hits, ATTACHMENTS, |ui, hits| {
            let doc = target.as_ref().map(|(part, revision)| crate::panels::plm_attachments::SectionDoc {
                id: active,
                part,
                revision,
                stem: stem.clone(),
            });
            attachments.ui(ui, doc, &plm, &mut |key, rect| hits.put(key, rect));
        });
        let (workspace, asked) = (&mut self.workspace, &mut self.workspace_asked);
        section(ui, &mut self.hits, WORKSPACE, |ui, hits| {
            let plm = crate::panels::plm_workspace::PlmWorkspaces { client: client.clone() };
            // A file dropped while the pointer is over this pane goes into the folder shown.
            if ui.rect_contains_pointer(ui.max_rect()) {
                for (name, bytes) in crate::panels::plm_workspace::dropped(ui.ctx()) {
                    workspace.add_file(&plm, &name, bytes);
                }
            }
            let outcome = workspace.show(ui, &plm, "");
            for (key, rect) in &workspace.hits {
                hits.put(key, *rect);
            }
            if outcome != crate::panels::plm_workspace::WorkspaceOutcome::default() {
                asked.push(outcome);
            }
        });
        let import = &mut self.import;
        section(ui, &mut self.hits, IMPORT, |ui, hits| {
            import.draw(ui, true);
            for (key, rect) in &import.hits {
                hits.put(key.strip_prefix("plm_import:").unwrap_or(key), *rect);
            }
        });
        // Another user's workspace is read-only: a new part goes to the top of the user's own.
        self.parts.workspace_folder =
            if self.workspace.read_only() { String::new() } else { self.workspace.current_folder().to_string() };
        let (parts, parts_asked) = (&mut self.parts, &mut self.parts_asked);
        section(ui, &mut self.hits, PARTS, |ui, hits| {
            ui.weak("Double-click a part to insert its newest revision here. New part makes this document its first revision.");
            let outcome = parts.show(ui, &crate::panels::plm_parts::PlmPartCatalog::new(client.clone()));
            for (key, rect) in &parts.hits {
                hits.put(key.strip_prefix("plm_parts:").unwrap_or(key), *rect);
            }
            if outcome != crate::panels::plm_parts::PlmPartsOutcome::default() {
                parts_asked.push(outcome);
            }
        });
    }

    /// Draw the parts-list section when the active document has a
    /// disagreement to answer (drawn first, under the revision, because the
    /// next save would undo the swap).
    fn uses_section(&mut self, ui: &mut egui::Ui, active: u64) {
        let Some(found) = self.repoint.disagreement(active).cloned() else {
            return;
        };
        let asked = &mut self.repoint_asked;
        section(ui, &mut self.hits, USES, |ui, hits| {
            let mut local = HashMap::new();
            let names = |id: &str| id.to_string();
            if let Some(choice) = crate::panels::plm_import::RepointPrompt::show(ui, &found, &names, &mut local) {
                asked.push((active, choice));
            }
            for (key, rect) in &local {
                hits.put(key.strip_prefix("plm_import:").unwrap_or(key), *rect);
            }
        });
    }

    /// The tags a pick of this host's may be waiting under.
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
        let access = match doc.access() {
            Access::Editable => serde_json::json!({ "editable": true }),
            Access::ReadOnly { reason } => serde_json::json!({ "editable": false, "reason": reason }),
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
            "offered": panel.map(|p| p.offered().iter().map(|v| v.key()).collect::<Vec<_>>()).unwrap_or_default(),
            // The Files section (S8): what is staged, listed, refused.
            "files": self.attachments.state_json(doc.id()),
            // The Parts list (S5): whether this document disagrees with its
            // revision's uses list, and how many re-points are offered.
            "uses": {
                "checking": self.repoint.busy(),
                "disagrees": self.repoint.disagreement(doc.id()).is_some(),
                "repoints": self.repoint.disagreement(doc.id()).map_or(0, |d| d.repoints.len()),
            },
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

fn set_access(docs: &mut Documents, id: u64, access: Access) {
    if let Some(doc) = docs.iter_mut().find(|d| d.id() == id) {
        if doc.access() != access {
            doc.set_access(access);
        }
    }
}

/// Re-read a document that just became editable, if the store holds other
/// bytes than it shows: someone saved it since it was opened.
fn reload(docs: &mut Documents, id: u64, name: &str, store: &dyn ModelStore) {
    let Some(text) = store.read(name) else { return };
    let Some(doc) = docs.iter_mut().find(|d| d.id() == id) else { return };
    let same = serde_json::from_str::<serde_json::Value>(&text).ok()
        == serde_json::from_str::<serde_json::Value>(&doc.engine.history.request_json()).ok();
    if same {
        return;
    }
    if doc.engine.set_history_json(&text).is_ok() {
        doc.mark_clean();
    }
}

