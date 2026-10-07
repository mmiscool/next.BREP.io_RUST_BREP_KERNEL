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

