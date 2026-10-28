//! Adoption (S13) and the uses-list re-point (S5), as the user meets them.
//!
//! * **Import a folder** ([`PlmImportPanel`]): read every model document under
//!   an explorer folder, show what the import will do (parts by class, the
//!   assemblies whose uses lists will be published, the documents it refuses,
//!   and every `sourceKey` pointing outside the folder), then run it against
//!   the PLM, with progress and **Stop**. The ledger is saved under
//!   `@plm_import` for this server and folder after every step, so a stopped
//!   or crashed import resumes where it was, and an import of a finished
//!   folder sends nothing. The folder is only ever read.
//! * **Re-point** ([`RepointPrompt`]): on opening an assembly revision whose
//!   uses list no longer matches its document (someone ran Replace everywhere),
//!   offer to move the occurrences to what the list names. Otherwise the next
//!   save would republish the list from the document and quietly undo the
//!   swap.
//!
//! Both are PLM-only: constructed with a signed-in client, never in a file
//! session.

use crate::panels::plm_parts::Pending;
use crate::plm::adoption::{
    plan_adoption, run_import, walk_tree, AdoptionPlan, ImportLedgers, ImportNumbering, ImportReport, Ledger,
    SourceDocument,
};
use crate::plm::client::PlmClient;
use crate::plm::uses::{Disagreement, Repoint};
use crate::store::ModelStore;
use eframe::egui;
use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::rc::Rc;
use std::task::Waker;

/// What the folder holds and what the import will do with it.
pub struct Planned {
    pub folder: String,
    pub documents: Vec<SourceDocument>,
    pub plan: AdoptionPlan,
    /// Every `sourceKey` in the folder's documents, spelled the explorer's
    /// way, worked out once so the running import needs no store.
    canonical: HashMap<String, String>,
}

/// A run in flight: its answer, the ledger as it moves, and Stop.
struct Running {
    answer: Pending<ImportReport>,
    ledger: Rc<RefCell<Ledger>>,
    stop: Rc<Cell<bool>>,
    saved: Ledger,
}

/// The import panel.
pub struct PlmImportPanel {
    pub folder: String,
    pub numbering: ImportNumbering,
    pub planned: Option<Planned>,
    pub report: Option<ImportReport>,
    pub problem: Option<String>,
    running: Option<Running>,
    /// What `draw` was asked for, done by the next `sync`.
    asked: Option<Ask>,
    /// The ledger to show: the run's as it moves, else what the store holds
    /// for this folder and server.
    shown: Option<Ledger>,
    /// Whether this session has local folders to read (see [`Self::sync`]).
    local: bool,
    pub hits: HashMap<String, egui::Rect>,
}

/// A click `draw` records for `sync`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ask {
    Plan,
    Start,
    Stop,
}

impl Default for PlmImportPanel {
    fn default() -> Self {
        Self {
            folder: String::new(),
            numbering: ImportNumbering { part_type: "component".into(), number_from_name: false },
            planned: None,
            report: None,
            problem: None,
            running: None,
            asked: None,
            shown: None,
            local: true,
            hits: HashMap::new(),
        }
    }
}

impl PlmImportPanel {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn running(&self) -> bool {
        self.running.is_some()
    }

    /// Read the folder and plan. Reads only.
    pub fn plan(&mut self, store: &dyn ModelStore) {
        self.report = None;
        self.problem = None;
        match walk_tree(store, &self.folder) {
            Ok(documents) => {
                let mut canonical = HashMap::new();
                for document in &documents {
                    let Ok(value) = serde_json::from_str::<serde_json::Value>(&document.contents) else {
                        continue;
                    };
                    collect_keys(&value, &mut |key| {
                        canonical.entry(key.to_string()).or_insert_with(|| store.canonical_identity(key));
                    });
                }
                let lookup = |key: &str| canonical.get(key).cloned().unwrap_or_else(|| key.to_string());
                let plan = plan_adoption(&documents, &lookup);
                self.planned = Some(Planned { folder: self.folder.clone(), documents, plan, canonical });
            }
            Err(problem) => {
                self.planned = None;
                self.problem = Some(problem);
            }
        }
    }

    /// Start (or resume) the import of the planned folder into `server`.
    pub fn start(&mut self, store: &dyn ModelStore, client: Rc<PlmClient>, server: &str) {
        let Some(planned) = &self.planned else {
            return;
        };
        let ledger = ImportLedgers::load(store).ledger(server, &planned.folder);
        let shared = Rc::new(RefCell::new(ledger.clone()));
        let stop = Rc::new(Cell::new(false));
        let (plan, documents, canonical) = (planned.plan.clone(), planned.documents.clone(), planned.canonical.clone());
        let numbering = self.numbering.clone();
        let (progress, stopping) = (shared.clone(), stop.clone());
        let future = async move {
            let lookup = |key: &str| canonical.get(key).cloned().unwrap_or_else(|| key.to_string());
            let mut ledger = ledger;
            let report = run_import(&client, &plan, &documents, &mut ledger, &lookup, &numbering, &mut |moved| {
                *progress.borrow_mut() = moved.clone();
                !stopping.get()
            })
            .await;
            Ok(report)
        };
        self.report = None;
        self.running = Some(Running {
            answer: Pending::new(Box::pin(future)),
            ledger: shared,
            stop,
            saved: Ledger::default(),
        });
    }

    /// Ask the run to stop after the step in flight.
    pub fn stop(&mut self) {
        if let Some(running) = &self.running {
            running.stop.set(true);
        }
    }

    /// Fold in the run's progress: save a ledger that moved, take the report
    /// when it ends.
    pub fn poll(&mut self, store: &dyn ModelStore, server: &str, waker: &Waker) {
        let Some(running) = &mut self.running else {
            return;
        };
        let answer = running.answer.poll(waker);
        let now = running.ledger.borrow().clone();
        if now != running.saved {
            if let Some(planned) = &self.planned {
                let mut ledgers = ImportLedgers::load(store);
                if let Err(problem) = ledgers.save(store, server, &planned.folder, &now) {
                    self.problem = Some(format!("the import's progress could not be saved: {problem}"));
                }
            }
            running.saved = now;
        }
        if let Some(answer) = answer {
            self.running = None;
            match answer {
                Ok(report) => self.report = Some(report),
                Err(problem) => self.problem = Some(problem),
            }
        }
    }

    /// The store's half of a frame, for a host whose drawing has no store:
    /// seed the folder from the explorer, act on what `draw` was asked last
    /// frame, fold in the run's progress (saving the ledger), and refresh the
    /// shown ledger.
    ///
    /// `files` is the store the folder is READ from — this machine's own file
    /// store, never the PLM's (`ModelStore::local_files` in a PLM session);
    /// `store` holds the ledger. `None`: this session has no local folders
    /// (the browser), and the panel says so.
    pub fn sync(&mut self, store: &dyn ModelStore, files: Option<&dyn ModelStore>, client: Option<Rc<PlmClient>>, server: &str, waker: &Waker) {
        self.local = files.is_some();
        if self.folder.is_empty() {
            if let Some(files) = files {
                self.folder = files.browser_location();
            }
        }
        match self.asked.take() {
            Some(Ask::Plan) => match files {
                Some(files) => self.plan(files),
                None => self.problem = Some("importing a folder needs the desktop app: this session has no local folders".into()),
            },
            Some(Ask::Start) => {
                if let Some(client) = client {
                    self.start(store, client, server);
                }
            }
            Some(Ask::Stop) => self.stop(),
            None => {}
        }
        self.poll(store, server, waker);
        self.shown = match (&self.running, &self.planned) {
            (Some(running), _) => Some(running.ledger.borrow().clone()),
            (None, Some(planned)) => Some(ImportLedgers::load(store).ledger(server, &planned.folder)),
            (None, None) => None,
        };
    }

    /// `{folder, documents, parts, assemblies, refused, external, running,
    /// report, problem}` — for scripts, through the PLM pane's state.
    pub fn state_json(&self) -> serde_json::Value {
        let plan = self.planned.as_ref().map(|p| &p.plan);
        serde_json::json!({
            "folder": self.folder,
            "local": self.local,
            "documents": self.planned.as_ref().map_or(0, |p| p.documents.len()),
            "parts": plan.map_or(0, |p| p.parts.len()),
            "assemblies": plan.map_or(0, |p| p.parts.iter().filter(|x| x.assembly).count()),
            "refused": plan.map_or(0, |p| p.refused.len()),
            "external": plan.map_or(0, |p| p.external.len()),
            "running": self.running(),
            "report": self.report.as_ref().map(|r| serde_json::json!({
                "minted": r.minted, "uploaded": r.uploaded, "published": r.published,
                "refused": r.refused.iter().map(|(d, why)| format!("{d}: {why}")).collect::<Vec<_>>(),
            })),
            "problem": self.problem,
        })
    }

    /// Draw the panel from what `sync` left, recording what the user asks
    /// for; the next `sync` does it. `can_import` is whether a PLM is signed
    /// in — without one the panel says so and imports nothing.
    pub fn draw(&mut self, ui: &mut egui::Ui, can_import: bool) {
        self.hits.clear();
        if !self.local {
            ui.weak("Importing a folder of documents runs in the desktop app: this session has no local folders.");
            return;
        }
        ui.horizontal(|ui| {
            ui.label("Folder");
            let field = ui.add_enabled(!self.running(), egui::TextEdit::singleline(&mut self.folder));
            self.hits.insert("plm_import:folder".into(), field.rect);
            let plan = ui.add_enabled(!self.running(), egui::Button::new("Read folder"));
            self.hits.insert("plm_import:plan".into(), plan.rect);
            if plan.clicked() {
                self.asked = Some(Ask::Plan);
            }
        });
        ui.horizontal(|ui| {
            ui.label("Part type");
            ui.add_enabled(!self.running(), egui::TextEdit::singleline(&mut self.numbering.part_type).desired_width(120.0));
            ui.add_enabled(
                !self.running(),
                egui::Checkbox::new(&mut self.numbering.number_from_name, "number each part by its file name"),
            );
        });
        if let Some(planned) = &self.planned {
            let plan = &planned.plan;
            let assemblies = plan.parts.iter().filter(|p| p.assembly).count();
            ui.label(format!(
                "{} document{} → {} part{}, {} with a uses list",
                planned.documents.len(),
                plural(planned.documents.len()),
                plan.parts.len(),
                plural(plan.parts.len()),
                assemblies
            ));
            for (document, reason) in &plan.refused {
                ui.colored_label(ui.visuals().warn_fg_color, format!("not imported: {document} — {reason}"));
            }
            for (document, key) in &plan.external {
                ui.colored_label(
                    ui.visuals().warn_fg_color,
                    format!("{document} uses '{key}', which is not in this folder: its uses list will be refused"),
                );
            }
            if let Some(ledger) = &self.shown {
                let minted = ledger.entries.len();
                let uploaded = ledger.entries.values().filter(|e| e.uploaded).count();
                let published = ledger.entries.values().filter(|e| e.published).count();
                ui.weak(format!(
                    "on this server: {minted}/{} minted, {uploaded}/{} uploaded, {published}/{assemblies} uses lists",
                    plan.parts.len(),
                    plan.parts.len()
                ));
            }
        }
        ui.horizontal(|ui| match (can_import, self.running()) {
            (_, true) => {
                ui.spinner();
                let stop = ui.button("Stop");
                self.hits.insert("plm_import:stop".into(), stop.rect);
                if stop.clicked() {
                    self.asked = Some(Ask::Stop);
                }
            }
            (true, false) => {
                let run = ui.add_enabled(self.planned.is_some(), egui::Button::new("Import"));
                self.hits.insert("plm_import:run".into(), run.rect);
                if run.clicked() {
                    self.asked = Some(Ask::Start);
                }
            }
            (false, false) => {
                ui.weak("Sign in to a PLM to import.");
            }
        });
        if let Some(report) = &self.report {
            ui.label(format!(
                "minted {}, uploaded {}, published {}",
                report.minted, report.uploaded, report.published
            ));
            for (document, sentence) in &report.refused {
                ui.colored_label(ui.visuals().error_fg_color, format!("{document}: {sentence}"));
            }
        }
        if let Some(problem) = &self.problem {
            ui.colored_label(ui.visuals().error_fg_color, problem);
        }
    }

    /// Both halves in one call, for a caller that has the store while it
    /// draws. The second `sync` acts on this frame's click at once, and polls
    /// a run started THIS frame — only a poll registers its waker.
    pub fn show(&mut self, ui: &mut egui::Ui, store: &dyn ModelStore, client: Option<Rc<PlmClient>>, server: &str) {
        let waker = repaint_waker(ui.ctx());
        self.sync(store, Some(store), client.clone(), server, &waker);
        self.draw(ui, client.is_some());
        self.sync(store, Some(store), client, server, &waker);
    }
}

fn collect_keys(document: &serde_json::Value, found: &mut dyn FnMut(&str)) {
    let Some(library) = document.get("partsLibrary").and_then(serde_json::Value::as_object) else {
        return;
    };
    for entry in library.values() {
        if let Some(key) = entry.get("sourceKey").and_then(serde_json::Value::as_str) {
            if !key.is_empty() {
                found(key);
            }
        }
        if let Some(nested) = entry.get("document") {
            collect_keys(nested, found);
        }
    }
}

fn plural(count: usize) -> &'static str {
    if count == 1 {
        ""
    } else {
        "s"
    }
}

fn repaint_waker(ctx: &egui::Context) -> Waker {
    struct Repaint(egui::Context);
    impl std::task::Wake for Repaint {
        fn wake(self: std::sync::Arc<Self>) {
            self.0.request_repaint();
        }
    }
    Waker::from(std::sync::Arc::new(Repaint(ctx.clone())))
}

// --- the re-point prompt --------------------------------------------------------------

/// The answer to the re-point prompt.
#[derive(Clone, Debug, PartialEq)]
pub enum RepointChoice {
    /// Move these occurrences (`plm::uses::repoint` each, then Update
    /// Components brings the new parts in).
    Repoint(Vec<Repoint>),
    /// Keep the document as it is. Its next save republishes the list from
    /// the document, which undoes the swap — the prompt says so.
    KeepDocument,
}

/// The prompt shown on opening an assembly whose uses list and document
/// disagree. `names` turns a part id into what a person reads (its number).
pub struct RepointPrompt;

impl RepointPrompt {
    pub fn show(
        ui: &mut egui::Ui,
        found: &Disagreement,
        names: &dyn Fn(&str) -> String,
        hits: &mut HashMap<String, egui::Rect>,
    ) -> Option<RepointChoice> {
        ui.label("This assembly's parts list on the PLM no longer matches the document:");
        for repoint in &found.repoints {
            ui.label(format!(
                "•  {} ({}) → {}",
                names(&repoint.from.0),
                repoint.occurrences.join(", "),
                names(&repoint.to.0)
            ));
        }
        for placed in &found.unlisted {
            ui.weak(format!("•  placed here but not on the list: {} ×{}", names(&placed.line.part), placed.line.quantity));
        }
        for listed in &found.unplaced {
            ui.weak(format!("•  on the list but not placed: {} ×{}", names(&listed.part), listed.quantity));
        }
        let mut choice = None;
        ui.horizontal(|ui| {
            if !found.repoints.is_empty() {
                let button = ui.button("Re-point to match the PLM");
                hits.insert("plm_import:repoint:apply".into(), button.rect);
                if button.clicked() {
                    choice = Some(RepointChoice::Repoint(found.repoints.clone()));
                }
            }
            let keep = ui.button("Keep the document");
            hits.insert("plm_import:repoint:keep".into(), keep.rect);
            if keep.clicked() {
                choice = Some(RepointChoice::KeepDocument);
            }
        });
        ui.weak("Keeping it means the next save publishes the parts list from the document again.");
        choice
    }
}

// --- the check on open ------------------------------------------------------------------

/// The store key a document name carries on a PLM store.
pub fn revision_key_of(name: &str) -> Option<String> {
    crate::plm::uses::revision_key_in(name)
}

/// S5's check on open: every PLM assembly document is compared once with
/// its revision's uses list, and a disagreement (Replace everywhere swapped a
/// child in the LIST) is offered to the user as a re-point before their next
/// save would republish the list from the document and undo the swap.
#[derive(Default)]
pub struct RepointCheck {
    /// Documents already checked (or being checked), by document id.
    checked: std::collections::HashSet<u64>,
    pending: HashMap<u64, Pending<Disagreement>>,
    /// What each document disagrees about, until the user answers.
    found: HashMap<u64, Disagreement>,
    pub hits: HashMap<String, egui::Rect>,
}

impl RepointCheck {
    pub fn new() -> Self {
        Self::default()
    }

    /// Once a frame: start a check for each newly opened PLM assembly, and
    /// fold in the answers. `waker` repaints.
    pub fn sync(&mut self, client: &Rc<PlmClient>, docs: &crate::document::Documents, waker: &Waker) {
        let open: Vec<u64> = docs.iter().map(|d| d.id()).collect();
        self.checked.retain(|id| open.contains(id));
        self.pending.retain(|id, _| open.contains(id));
        self.found.retain(|id, _| open.contains(id));
        for doc in docs.iter() {
            if self.checked.contains(&doc.id()) {
                continue;
            }
            let Some(key) = doc.name().and_then(revision_key_of) else {
                continue;
            };
            self.checked.insert(doc.id());
            let Ok(document) = serde_json::from_str::<serde_json::Value>(&doc.engine.history_request_json()) else {
                continue;
            };
            if !crate::plm::uses::publishes_on_save(&document) {
                continue;
            }
            let client = client.clone();
            self.pending.insert(
                doc.id(),
                Pending::new(Box::pin(async move { crate::plm::uses::check_on_open(&client, &key, &document).await })),
            );
        }
        let answered: Vec<(u64, Result<Disagreement, String>)> = self
            .pending
            .iter_mut()
            .filter_map(|(id, pending)| pending.poll(waker).map(|answer| (*id, answer)))
            .collect();
        for (id, answer) in answered {
            self.pending.remove(&id);
            // A failed check is not a disagreement: nothing is offered, and
            // the save republishes as it always does.
            if let Ok(found) = answer {
                if !found.is_empty() {
                    self.found.insert(id, found);
                }
            }
        }
    }

    pub fn busy(&self) -> bool {
        !self.pending.is_empty()
    }

    /// What document `id` disagrees about, if anything is still unanswered.
    pub fn disagreement(&self, id: u64) -> Option<&Disagreement> {
        self.found.get(&id)
    }

    /// Apply the user's answer to document `id`. A re-point edits the
    /// document (one undoable edit, dirty until saved; the save republishes
    /// the list, now matching the server's) — Update Components then brings
    /// the new parts in.
    pub fn answer(&mut self, docs: &mut crate::document::Documents, id: u64, choice: RepointChoice) -> Result<(), String> {
        self.found.remove(&id);
        let RepointChoice::Repoint(repoints) = choice else {
            return Ok(());
        };
        let Some(doc) = docs.iter_mut().find(|d| d.id() == id) else {
            return Ok(());
        };
        let mut document: serde_json::Value =
            serde_json::from_str(&doc.engine.history_request_json()).map_err(|e| e.to_string())?;
        for repoint in &repoints {
            crate::plm::uses::repoint(&mut document, repoint, &crate::plm::uses::plm_identity);
        }
        doc.engine.edit_document_json(&document.to_string()).map(|_| ())
    }

    /// Draw the prompt for the active document, if it has one; apply the
    /// answer the frame it is given. Part ids are shown as given (the host
    /// may pass a namer that turns them into numbers).
    pub fn ui(&mut self, ui: &mut egui::Ui, docs: &mut crate::document::Documents, names: &dyn Fn(&str) -> String) -> Result<(), String> {
        self.hits.clear();
        let id = docs.active().id();
        let Some(found) = self.found.get(&id).cloned() else {
            return Ok(());
        };
        match RepointPrompt::show(ui, &found, names, &mut self.hits) {
            Some(choice) => self.answer(docs, id, choice),
            None => Ok(()),
        }
    }
}

// --- cached copies, refreshed on open ----------------------------------------------------

/// The other check on open: an assembly's embedded copies of PLM parts are a
/// cache of their revisions ([`crate::plm::cached_copies`]). Each PLM document
/// is compared once, when it opens; the copies older than their revisions are
/// replaced as part of the open ([`crate::document::Document::absorb_open_write`]:
/// not an unsaved change), with one notice naming them.
///
/// And while it is open: when the change feed names a revision an open
/// document embeds, at any depth ([`Self::moved`]), it is compared again. The
/// copies the user has not edited follow their revisions, through the chain
/// (a part saved elsewhere refreshes the sub that holds it), in the document
/// and its clean baseline alike, so the refresh is no unsaved change of its
/// own. A copy the user edited is never replaced unasked: the pane asks, Keep
/// my edits (the default) or Take their version. Never while a write of ours
/// is in flight: the feed names our own keys before their writes land.
#[derive(Default)]
pub struct CachedCopyCheck {
    checked: std::collections::HashSet<u64>,
    /// Each read in flight: the revisions read, or taken from unchanged copies.
    pending: HashMap<u64, Pending<HashMap<String, serde_json::Value>>>,
    /// The revisions each open took from its copies instead of reading
    /// (state, for automation).
    unread: HashMap<u64, usize>,
    /// The copies each document had refreshed, by document id.
    refreshed: HashMap<u64, Vec<String>>,
    /// Documents due a live check, with the revisions the feed named
    /// (empty: it could not say which).
    due: HashMap<u64, Vec<String>>,
    /// Documents whose read in flight is a live check, as `due`.
    live: HashMap<u64, Vec<String>>,
    /// Edited copies whose revisions moved, while the pane asks: the copies'
    /// paths and the revisions read.
    asking: HashMap<u64, (Vec<String>, HashMap<String, serde_json::Value>)>,
    /// Revisions whose change the user chose not to take, per document: the
    /// revision's text when they did, so only a further change asks again.
    kept: HashMap<u64, HashMap<String, String>>,
    /// Live refreshes since the session began (state, for automation).
    updated: u64,
    /// Documents re-checked after a reload ([`Self::recheck`]): their open
    /// already said what it refreshed and what is kept, so this check is quiet.
    quiet: std::collections::HashSet<u64>,
}

impl CachedCopyCheck {
    pub fn new() -> Self {
        Self::default()
    }

    /// Once a frame: start a read of the revisions each newly opened PLM
    /// document's copies name, and apply the answers.
    pub fn sync(&mut self, client: &Rc<PlmClient>, store: &dyn ModelStore, docs: &mut crate::document::Documents, waker: &Waker, writing: bool) {
        let open: Vec<u64> = docs.iter().map(|d| d.id()).collect();
        self.checked.retain(|id| open.contains(id));
        self.pending.retain(|id, _| open.contains(id));
        self.unread.retain(|id, _| open.contains(id));
        self.refreshed.retain(|id, _| open.contains(id));
        self.due.retain(|id, _| open.contains(id));
        self.live.retain(|id, _| open.contains(id));
        self.asking.retain(|id, _| open.contains(id));
        self.kept.retain(|id, _| open.contains(id));
        self.quiet.retain(|id| open.contains(id));
        if !writing {
            for (id, keys) in std::mem::take(&mut self.due) {
                if self.pending.contains_key(&id) {
                    self.due.insert(id, keys);
                } else {
                    self.checked.remove(&id);
                    self.live.insert(id, keys);
                }
            }
        }
        for doc in docs.iter() {
            if self.checked.contains(&doc.id()) || doc.name().and_then(revision_key_of).is_none() {
                continue;
            }
            self.checked.insert(doc.id());
            // Restored from crash recovery: its copies hold the user's unsaved
            // edits, which a refresh from the revisions would throw away (and
            // take into the clean baseline, so the tab read saved). The Save
            // writes them; the follower still reports what others change.
            if doc.is_recovered() {
                continue;
            }
            let Ok(document) = serde_json::from_str::<serde_json::Value>(&doc.engine.history_request_json()) else {
                continue;
            };
            if !document["partsLibrary"].as_object().is_some_and(|l| !l.is_empty()) {
                continue;
            }
            // Copies that are their revisions' stored bytes, as the index
            // lists them, stand in for them. Not those of the revisions the feed just
            // named: the held index may not have their new rows yet.
            let mut unchanged = crate::plm::cached_copies::unchanged(&document, &|key| {
                store.plm_revision(key).map(|entry| entry.content_hash)
            });
            // Except the ones the feed named for our own save: the index row
            // holds what the server answered for our write, so a copy with
            // those bytes is the revision (another writer since would have
            // given the row a newer hash, and it is read).
            match self.live.get(&doc.id()) {
                Some(named) if named.is_empty() => unchanged.clear(),
                Some(named) => unchanged.retain(|key, _| !named.contains(key) || store.plm_written_by_us(key)),
                None => {}
            }
            self.unread.insert(doc.id(), unchanged.len());
            let client = client.clone();
            self.pending.insert(
                doc.id(),
                Pending::new(Box::pin(async move { Ok(crate::plm::cached_copies::fetch_changed(&client, &document, unchanged).await) })),
            );
        }
        let answered: Vec<(u64, Result<HashMap<String, serde_json::Value>, String>)> = self
            .pending
            .iter_mut()
            .filter_map(|(id, pending)| pending.poll(waker).map(|answer| (*id, answer)))
            .collect();
        for (id, answer) in answered {
            self.pending.remove(&id);
            if let Some(moved) = self.live.remove(&id) {
                if writing {
                    // Read while a write of ours was going out: it may predate it.
                    self.due.insert(id, moved);
                    continue;
                }
                if let (Ok(sources), Some(doc)) = (answer, docs.iter_mut().find(|d| d.id() == id)) {
                    if let Some(paths) = self.follow(doc, &sources) {
                        self.asking.insert(id, (paths, sources));
                    }
                }
                continue;
            }
            let (Ok(sources), Some(doc)) = (answer, docs.iter_mut().find(|d| d.id() == id)) else {
                continue;
            };
            // Refreshed against the document as it is NOW: the user may have
            // edited it while the revisions were being read.
            let Ok(mut document) = serde_json::from_str::<serde_json::Value>(&doc.engine.history_request_json()) else {
                continue;
            };
            let before = document.clone();
            let quiet = self.quiet.remove(&id);
            // Copies a Save could not write keep the user's change: say so, and
            // what to do, every time the document opens.
            for (key, why) in crate::document::owned_save::unsaved_edits(&document).into_iter().filter(|_| !quiet) {
                let name = crate::document::owned_save::first_copy(&document, &key, &mut Vec::new())
                    .map_or_else(|| key.clone(), |(names, _)| names.join("/"));
                doc.engine.push_notice(format!(
                    "{name} has changes that are not saved to its PLM item yet ({why}): {}. They are kept in this assembly until then",
                    crate::plm::cached_copies::way_forward(&why)
                ));
            }
            let replaced = crate::plm::cached_copies::refresh(&mut document, &sources);
            if replaced.is_empty() {
                continue;
            }
            let key = doc.edit_key();
            match doc.engine.edit_document_json(&document.to_string()) {
                // Taken as part of the open, not as an edit: the refresh adds
                // no unsaved change of its own (the tab is as dirty as the same
                // open with current copies). Nothing is written to the PLM.
                Ok(_) => {
                    doc.absorb_open_write(&before, &document, key);
                    // The refreshed copies re-heal on the run this edit causes.
                    doc.arm_open_settle();
                    if !quiet { doc.engine.push_notice(format!(
                        "Updated {} from {} PLM revision{}: the copies saved in this assembly were older",
                        replaced.join(", "),
                        if replaced.len() == 1 { "its" } else { "their" },
                        if replaced.len() == 1 { "" } else { "s" }
                    )); }
                }
                Err(problem) => doc.engine.push_notice(format!("Could not update {} from the PLM: {problem}", replaced.join(", "))),
            }
            self.refreshed.insert(id, replaced);
        }
    }

    /// Tab `id` was reloaded from its stored document: compare its copies
    /// again, as on open (the earlier check's refresh went with the reload).
    pub fn recheck(&mut self, id: u64) {
        self.quiet.insert(id);
        self.checked.remove(&id);
        self.pending.remove(&id);
        self.refreshed.remove(&id);
        self.live.remove(&id);
    }

    /// The feed moved: every open document that embeds a revision it named
    /// (every checked one when it could not say which) is due a live check.
    pub fn moved(&mut self, docs: &crate::document::Documents, keys: &[String], stale: bool) {
        let named: std::collections::BTreeSet<String> = keys.iter().filter_map(|k| revision_key_of(k)).collect();
        for doc in docs.iter() {
            if !self.checked.contains(&doc.id()) {
                continue;
            }
            let Ok(document) = serde_json::from_str::<serde_json::Value>(&doc.engine.history_request_json()) else {
                continue;
            };
            let embedded = crate::plm::cached_copies::revision_keys(&document);
            let hit: Vec<String> = embedded.into_iter().filter(|k| named.contains(k)).collect();
            if stale || !hit.is_empty() {
                self.due.entry(doc.id()).or_default().extend(hit);
            }
        }
    }

    /// A live check answered: refresh what the user has not edited, in the
    /// document and its baseline alike. The paths of edited copies whose
    /// revisions moved, when there are any, for the pane to ask about.
    fn follow(&mut self, doc: &mut crate::document::Document, sources: &HashMap<String, serde_json::Value>) -> Option<Vec<String>> {
        use crate::plm::cached_copies::{moved_revisions, paths_of, refresh_except};
        // A tab never saved (restored after a crash) has no baseline to tell
        // the user's edits by: it is left as it is.
        let mut baseline = doc.saved_document()?;
        // The history's own document: `history_request_json` adds the session's
        // workbench and thumbnail, which an edit written back would stamp into
        // the history (and from there into the clean baseline).
        let mut current: serde_json::Value = serde_json::from_str(&doc.engine.history.request_json()).ok()?;
        let edited: std::collections::BTreeSet<String> = crate::document::owned_save::owned_changes(&baseline, &current)
            .writes
            .into_iter()
            .filter_map(|w| revision_key_of(&w.key))
            .collect();
        // What changed on the PLM since the tab was last clean, by part (the
        // washer, not the subs that hold it); the edited ones among them are asked about.
        let moved = moved_revisions(&baseline, sources);
        let changed = paths_of(&current, &moved);
        let kept = self.kept.get(&doc.id());
        let ask: std::collections::BTreeSet<String> = moved
            .intersection(&edited)
            .filter(|key| kept.and_then(|k| k.get(*key)) != sources.get(*key).map(|s| s.to_string()).as_ref())
            .cloned()
            .collect();
        let asked = paths_of(&current, &ask);
        let replaced = refresh_except(&mut current, sources, &edited);
        if !replaced.is_empty() {
            refresh_except(&mut baseline, sources, &edited);
            let clean = !doc.is_dirty();
            let key = doc.edit_key();
            match doc.engine.edit_document_json(&current.to_string()) {
                Ok(_) => {
                    doc.rebase_saved_document(&baseline, key);
                    if clean {
                        // The refreshed copies re-heal on the run this edit
                        // causes; in a clean tab that run is not the user's.
                        doc.arm_open_settle();
                    }
                    self.updated += 1;
                    let what = if changed.is_empty() { replaced.join(", ") } else { changed.join(", ") };
                    doc.engine.push_notice(format!(
                        "{what} changed on the PLM (saved by another client): updated {} in this assembly",
                        replaced.join(", ")
                    ));
                }
                Err(problem) => doc.engine.push_notice(format!("Could not update {} from the PLM: {problem}", replaced.join(", "))),
            }
        }
        (!asked.is_empty()).then_some(asked)
    }

    /// The edited copies the pane asks about for document `id`.
    pub fn asking(&self, id: u64) -> Option<&[String]> {
        self.asking.get(&id).map(|(paths, _)| paths.as_slice())
    }

    /// The answer for document `id`: `true` takes their version of the edited
    /// copies (the edits in them are discarded), `false` keeps the edits
    /// (asked again only when the PLM moves again).
    pub fn answer(&mut self, docs: &mut crate::document::Documents, id: u64, take_theirs: bool) {
        let Some((paths, sources)) = self.asking.remove(&id) else { return };
        if !take_theirs {
            let kept = self.kept.entry(id).or_default();
            for (key, source) in &sources {
                kept.insert(key.clone(), source.to_string());
            }
            return;
        }
        let Some(doc) = docs.iter_mut().find(|d| d.id() == id) else { return };
        let (Some(mut baseline), Ok(mut current)) =
            (doc.saved_document(), serde_json::from_str::<serde_json::Value>(&doc.engine.history.request_json()))
        else {
            return;
        };
        let replaced = crate::plm::cached_copies::refresh(&mut current, &sources);
        if replaced.is_empty() {
            return;
        }
        crate::plm::cached_copies::refresh(&mut baseline, &sources);
        let key = doc.edit_key();
        match doc.engine.edit_document_json(&current.to_string()) {
            Ok(_) => {
                doc.rebase_saved_document(&baseline, key);
                if !doc.is_dirty() {
                    // The discarded edits were the only ones: the run this edit
                    // causes re-heals the copies, which is not the user's change.
                    doc.arm_open_settle();
                }
                // An edit whose mates moved other components: those poses
                // re-solve back, but not to the same bits. Once they have, and
                // nothing else differs, the tab is clean again.
                doc.arm_converge();
                self.updated += 1;
                doc.engine.push_notice(format!(
                    "{} now {} the other client's version; your edits there were discarded",
                    paths.join(", "),
                    if paths.len() == 1 { "shows" } else { "show" }
                ));
            }
            Err(problem) => doc.engine.push_notice(format!("Could not update {} from the PLM: {problem}", replaced.join(", "))),
        }
    }

    pub fn busy(&self) -> bool {
        !self.pending.is_empty()
    }

    /// Live state for automation: refreshes so far, and the copies asked about.
    pub fn live_json(&self, id: u64) -> serde_json::Value {
        serde_json::json!({
            "updated": self.updated,
            "due": self.due.contains_key(&id) || self.live.contains_key(&id),
            "asking": self.asking(id).unwrap_or(&[]),
        })
    }

    /// The copies document `id` had refreshed when it opened.
    pub fn refreshed(&self, id: u64) -> &[String] {
        self.refreshed.get(&id).map_or(&[], Vec::as_slice)
    }

    /// How many revisions document `id`'s last check took from its copies
    /// instead of reading them.
    pub fn unread(&self, id: u64) -> usize {
        self.unread.get(&id).copied().unwrap_or(0)
    }
}

// --- copies pinned to a superseded revision -------------------------------------------

/// One embedded revision that is no longer current, and the release that is.
#[derive(Debug, Clone)]
pub struct Superseded {
    /// The pinned revision's store key, and the current release's.
    pub key: String,
    pub current_key: String,
    pub number: String,
    pub from: String,
    pub lifecycle: String,
    pub to: String,
    /// The current release's document, for the update.
    text: String,
    /// Where it is placed, as library paths (`sub-b/plate`).
    pub paths: Vec<String>,
}

/// The check on open for copies pinned to a superseded or obsolete revision
/// (`plm::superseded`). Once per open, off its critical path: the pinned
/// revisions' lifecycles come from the store index the session already holds
/// (no request); only a part that IS superseded is read, once however often
/// it is placed, for its current release and that release's document. One
/// notice names what is stale. The pane offers Update only where the document
/// may change (a draft no one else holds); a released assembly, or one held by
/// someone else, pins its revisions by design and is only told.
#[derive(Default)]
pub struct SupersededCheck {
    checked: std::collections::HashSet<u64>,
    /// Frames spent waiting for the index to hold every pinned revision.
    waits: HashMap<u64, u32>,
    pending: HashMap<u64, Pending<Vec<Superseded>>>,
    found: HashMap<u64, Vec<Superseded>>,
    /// Documents whose Update question is still open.
    asking: std::collections::HashSet<u64>,
    updated: u64,
    /// Revision reads made, for the cost report (state).
    reads: u64,
    /// Why a document keeps its revisions (released, held by someone), for the pane.
    keeps: HashMap<u64, String>,
    /// "Make a new revision using B", by the document it was asked from.
    creating: HashMap<u64, Pending<(String, String)>>,
    /// New revisions the user made to move on: updated as soon as their own
    /// check finds the stale copies (the user already chose), as store keys.
    auto: std::collections::HashSet<String>,
    /// Made revisions to open, with the document they came from.
    opened: Vec<(u64, String)>,
    /// The revisions each checked document pins, with whether each read
    /// superseded at its check: one that turns superseded (or obsolete) while
    /// the document is open checks it again, so the user is told without
    /// reopening it.
    pinned: HashMap<u64, Vec<(String, bool)>>,
}

/// How many frames the check waits for the index to hold every revision a
/// document names before checking with what it has (~10 s at 50 ms).
const INDEX_WAIT_FRAMES: u32 = 200;

impl SupersededCheck {
    pub fn sync(&mut self, client: &Rc<PlmClient>, store: &dyn ModelStore, docs: &mut crate::document::Documents, waker: &Waker) {
        use crate::plm::superseded::{current_release, is_superseded, pinned_keys};
        let open: Vec<u64> = docs.iter().map(|d| d.id()).collect();
        self.checked.retain(|id| open.contains(id));
        self.waits.retain(|id, _| open.contains(id));
        self.pending.retain(|id, _| open.contains(id));
        self.found.retain(|id, _| open.contains(id));
        self.asking.retain(|id| open.contains(id));
        self.keeps.retain(|id, _| open.contains(id));
        self.creating.retain(|id, _| open.contains(id));
        self.pinned.retain(|id, _| open.contains(id));
        // A newer revision released (or one retired) since the check: once
        // the change feed has the pinned row's new lifecycle, check again.
        let moved: Vec<u64> = self
            .pinned
            .iter()
            .filter(|(id, keys)| {
                !self.pending.contains_key(id)
                    && keys.iter().any(|(key, was)| !was && store.plm_revision(key).is_some_and(|e| is_superseded(&e.lifecycle)))
            })
            .map(|(id, _)| *id)
            .collect();
        for id in moved {
            self.pinned.remove(&id);
            self.checked.remove(&id);
            self.waits.remove(&id);
        }
        let made: Vec<(u64, Result<(String, String), String>)> =
            self.creating.iter_mut().filter_map(|(id, pending)| pending.poll(waker).map(|answer| (*id, answer))).collect();
        for (id, answer) in made {
            self.creating.remove(&id);
            let Some(doc) = docs.iter_mut().find(|d| d.id() == id) else { continue };
            match answer {
                Ok((key, text)) => {
                    // The store mirror learns the new revision before it is opened
                    // (as Save As → new revision does), or the open reads it absent.
                    store.remember_saved_document(&store.canonical_identity(&key), &text);
                    store.refresh_plm_index(std::slice::from_ref(&key), false);
                    if let Some(key) = revision_key_of(&key) {
                        self.auto.insert(key.clone());
                        self.opened.push((id, key));
                    }
                    doc.engine.push_notice_as(
                        brep_render::engine_state::NoticeSeverity::Info,
                        "Made a new revision of this assembly, checked out to you: opening it with the current revisions",
                    );
                }
                Err(problem) => doc.engine.push_notice(format!("Could not make a new revision of this assembly: {problem}")),
            }
        }
        let mut apply_now = Vec::new();
        for doc in docs.iter() {
            let id = doc.id();
            if self.checked.contains(&id) || self.pending.contains_key(&id) || doc.name().and_then(revision_key_of).is_none() {
                continue;
            }
            let Ok(document) = serde_json::from_str::<serde_json::Value>(&doc.engine.history.request_json()) else { continue };
            let keys = pinned_keys(&document);
            let entries: Vec<(String, Option<crate::plm::client::IndexEntry>)> =
                keys.into_iter().map(|k| { let e = store.plm_revision(&k); (k, e) }).collect();
            if entries.iter().any(|(_, e)| e.is_none()) {
                let waited = self.waits.entry(id).or_default();
                *waited += 1;
                if *waited < INDEX_WAIT_FRAMES {
                    continue;
                }
            }
            self.checked.insert(id);
            self.pinned.insert(
                id,
                entries.iter().map(|(key, entry)| (key.clone(), entry.as_ref().is_some_and(|e| is_superseded(&e.lifecycle)))).collect(),
            );
            // Stale revisions, grouped by part: each part is read once.
            let mut by_part: std::collections::BTreeMap<String, Vec<(String, crate::plm::client::IndexEntry)>> = Default::default();
            for (key, entry) in entries {
                if let Some(entry) = entry.filter(|e| is_superseded(&e.lifecycle)) {
                    by_part.entry(entry.part_id.clone()).or_default().push((key, entry));
                }
            }
            if by_part.is_empty() {
                continue;
            }
            self.reads += 2 * by_part.len() as u64;
            let client = client.clone();
            self.pending.insert(
                id,
                Pending::new(Box::pin(async move {
                    let mut out = Vec::new();
                    for (part, stale) in by_part {
                        let path = format!("/api/parts/{}", crate::plm::identity::segment(&part));
                        let answer = client.call("GET", &path, None).await.map_err(|e| e.to_string())?;
                        let view: serde_json::Value = serde_json::from_slice(&answer.body).map_err(|e| e.to_string())?;
                        let Some((revision, label)) = current_release(&view) else {
                            // Nothing of it is released now: no update to
                            // offer, but the user is told (`current_key` empty).
                            for (key, entry) in stale {
                                out.push(Superseded {
                                    key,
                                    current_key: String::new(),
                                    number: entry.part_number.clone(),
                                    from: entry.revision_label.clone(),
                                    lifecycle: entry.lifecycle.clone(),
                                    to: String::new(),
                                    text: String::new(),
                                    paths: Vec::new(),
                                });
                            }
                            continue;
                        };
                        let current_key = crate::plm::identity::document_key(&part, &revision);
                        let Some(bytes) = client.get_document(&current_key).await.map_err(|e| e.to_string())? else { continue };
                        let text = String::from_utf8(bytes).map_err(|e| e.to_string())?;
                        for (key, entry) in stale {
                            if key == current_key {
                                continue;
                            }
                            out.push(Superseded {
                                key,
                                current_key: current_key.clone(),
                                number: entry.part_number.clone(),
                                from: entry.revision_label.clone(),
                                lifecycle: entry.lifecycle.clone(),
                                to: label.clone(),
                                text: text.clone(),
                                paths: Vec::new(),
                            });
                        }
                    }
                    Ok(out)
                })),
            );
        }
        let answered: Vec<(u64, Result<Vec<Superseded>, String>)> =
            self.pending.iter_mut().filter_map(|(id, pending)| pending.poll(waker).map(|answer| (*id, answer))).collect();
        for (id, answer) in answered {
            self.pending.remove(&id);
            let Some(doc) = docs.iter_mut().find(|d| d.id() == id) else { continue };
            let mut found = match answer {
                Ok(found) => found,
                Err(problem) => {
                    doc.engine.push_notice(format!("Could not check this assembly's PLM revisions: {problem}"));
                    continue;
                }
            };
            let Ok(document) = serde_json::from_str::<serde_json::Value>(&doc.engine.history.request_json()) else { continue };
            // Named by where they are PLACED: an entry no occurrence shows (a
            // Replace inside a sub-assembly leaves the old one behind) is not used.
            for item in &mut found {
                let mut names: Vec<String> = Vec::new();
                for placement in crate::plm::superseded::occurrences_of(&document, &item.key) {
                    let name = placement.name();
                    if !names.contains(&name) {
                        names.push(name);
                    }
                }
                item.paths = names;
            }
            found.retain(|item| !item.paths.is_empty());
            // A part with no released revision has nothing to update to.
            let (retired, rest): (Vec<Superseded>, Vec<Superseded>) = found.into_iter().partition(|item| item.current_key.is_empty());
            found = rest;
            for item in &retired {
                doc.engine.push_notice_as(
                    brep_render::engine_state::NoticeSeverity::Warning,
                    format!(
                        "{}: {} uses {} rev {}, and no revision of it is released now: replace it with another part, or have a revision of it released",
                        item.paths.join(", "),
                        item.number,
                        item.lifecycle,
                        item.from
                    ),
                );
            }
            if found.is_empty() {
                continue;
            }
            // Only a draft no one else holds may change; a released assembly
            // (or one in review, or held by someone else) keeps its revisions
            // by design and is told how to move on.
            let own_key = doc.name().and_then(revision_key_of);
            // A revision this check made (a draft, checked out to this user) may
            // not be in the index yet: it is editable, and the user chose it.
            let made_here = own_key.as_ref().is_some_and(|k| self.auto.contains(k));
            let own = own_key.as_ref().and_then(|key| store.plm_revision(key));
            let keeps = match &own {
                _ if made_here => None,
                Some(e) if e.lifecycle == "draft" && (e.locked_by.is_none() || e.locked_by_me) => None,
                Some(e) if e.lifecycle == "draft" => Some(format!("it is checked out by {}", e.locked_by.clone().unwrap_or_default())),
                Some(e) => Some(format!("it is {}", e.lifecycle)),
                None => Some("its PLM revision could not be read".to_string()),
            };
            for item in &found {
                let tail = keeps.as_ref().map_or(String::new(), |why| {
                    format!(". This assembly keeps rev {} ({why}); make a new revision of it to use rev {}", item.from, item.to)
                });
                doc.engine.push_notice_as(
                    brep_render::engine_state::NoticeSeverity::Warning,
                    format!("{}: {} uses {} rev {}; {} is current{tail}", item.paths.join(", "), item.number, item.lifecycle, item.from, item.to),
                );
            }
            match &keeps {
                // A revision the user made to move on: no second question.
                None if own_key.as_ref().is_some_and(|k| self.auto.remove(k)) => apply_now.push(id),
                None => {
                    self.asking.insert(id);
                }
                Some(why) => {
                    self.keeps.insert(id, why.clone());
                }
            }
            self.found.insert(id, found);
        }
        for id in apply_now {
            self.answer(docs, store, id, true);
        }
    }

    /// Why document `id` keeps its stale revisions (it is released, or held by
    /// someone else), when it does.
    pub fn keeps(&self, id: u64) -> Option<&str> {
        self.keeps.get(&id).map(String::as_str).filter(|_| !self.found(id).is_empty())
    }

    pub fn creating(&self, id: u64) -> bool {
        self.creating.contains_key(&id)
    }

    /// The opt-in way on from a document that keeps its revisions: make its
    /// next revision (a draft, checked out to this user) holding the document
    /// as it is, open it, and update the stale copies there; the user saves it.
    pub fn make_new_revision(&mut self, client: &Rc<PlmClient>, docs: &crate::document::Documents, id: u64) {
        if self.creating.contains_key(&id) {
            return;
        }
        let Some(doc) = docs.iter().find(|d| d.id() == id) else { return };
        let Some((part, _)) = doc.name().and_then(crate::panels::plm_host::revision_of) else { return };
        let text = doc.engine.history_request_json();
        let client = client.clone();
        self.creating.insert(
            id,
            Pending::new(Box::pin(async move {
                let key = crate::panels::plm_parts::save_as_new_revision(&client, &part, "", &text).await?;
                Ok((key, text))
            })),
        );
    }

    /// The revisions made since the last call, to open, with the document each came from.
    pub fn take_opened(&mut self) -> Vec<(u64, String)> {
        std::mem::take(&mut self.opened)
    }

    /// The stale revisions found in document `id`.
    pub fn found(&self, id: u64) -> &[Superseded] {
        self.found.get(&id).map_or(&[], Vec::as_slice)
    }

    pub fn asking(&self, id: u64) -> bool {
        self.asking.contains(&id) && !self.found(id).is_empty()
    }

    /// The answer for document `id`: `true` replaces every stale placement with
    /// the current release (top-level ones through Replace, ones inside a
    /// sub-assembly through its definition, so mates are kept as Replace keeps
    /// them); `false` leaves it (the mark stays).
    pub fn answer(&mut self, docs: &mut crate::document::Documents, store: &dyn ModelStore, id: u64, update: bool) {
        use crate::panels::replace_component::{replace, replace_nested, ReplaceSource};
        self.asking.remove(&id);
        if !update {
            return;
        }
        let Some(found) = self.found.remove(&id) else { return };
        let Some(doc) = docs.iter_mut().find(|d| d.id() == id) else { return };
        for item in &found {
            store.remember_saved_document(&store.canonical_identity(&item.current_key), &item.text);
            let Ok(document) = serde_json::from_str::<serde_json::Value>(&doc.engine.history.request_json()) else { continue };
            let placements = crate::plm::superseded::occurrences_of(&document, &item.key);
            let source = ReplaceSource::Stored(item.current_key.clone());
            let top: Vec<String> = placements.iter().filter(|p| p.owners.is_empty()).map(|p| p.path.clone()).collect();
            let mut result = Ok(());
            if !top.is_empty() {
                result = replace(&mut doc.engine, store, &top, &source).map(|_| ());
            }
            // Inside sub-assemblies: each defining chain once (it edits the
            // definition, which every placement of that sub-assembly shows).
            let mut chains: Vec<&Vec<String>> = Vec::new();
            for placement in placements.iter().filter(|p| !p.owners.is_empty()) {
                if result.is_err() || chains.contains(&&placement.owners) {
                    continue;
                }
                chains.push(&placement.owners);
                result = replace_nested(&mut doc.engine, store, &placement.path, &source, true).map(|_| ());
            }
            match result {
                Ok(()) => {
                    self.updated += 1;
                    doc.engine.push_notice_as(
                        brep_render::engine_state::NoticeSeverity::Info,
                        format!("{}: {} updated from rev {} to rev {}", item.paths.join(", "), item.number, item.from, item.to),
                    );
                }
                Err(problem) => doc.engine.push_notice(format!("{}: could not update {} to rev {}: {problem}", item.paths.join(", "), item.number, item.to)),
            }
        }
    }

    pub fn busy(&self) -> bool {
        !self.pending.is_empty() || !self.creating.is_empty()
    }

    pub fn state_json(&self, id: u64) -> serde_json::Value {
        serde_json::json!({
            "checked": self.checked.contains(&id) && !self.pending.contains_key(&id),
            "superseded": self.found(id).iter().map(|s| serde_json::json!({
                "paths": s.paths, "number": s.number, "from": s.from, "lifecycle": s.lifecycle, "to": s.to,
            })).collect::<Vec<_>>(),
            "asking": self.asking(id),
            "keeps": self.keeps(id),
            "creating": self.creating(id),
            "updated": self.updated,
            "reads": self.reads,
        })
    }
}

