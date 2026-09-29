//! The `--kicad-library` window: importing KiCad parts, and nothing else.
//!
//! `brep-app --kicad-library` opens THIS window instead of the CAD window. It
//! is the only flag that does: every other launch runs the same full
//! `eframe::run_native` shell. There is no viewport, no history tree and no
//! workbench here — a part is read, reviewed, and written to the model store as
//! a document the CAD window then opens.
//!
//! # Two sources, one chain
//!
//! * **Download** — KiCad's own libraries, fetched file by file from GitLab at
//!   a pinned 9.x tag ([`super::kicad_remote`]) into a cache laid out like a
//!   KiCad install.
//! * **Your KiCad install** — the folders already on this machine, found the
//!   way the in-app dialog finds them ([`KicadLibrary::load`]).
//!
//! Both end in the SAME call: `part_from_symbol` follows the symbol to its
//! footprint to its `.step`, and [`import_part`] writes the IMPORT3D feature,
//! the `symbol` and `pads` blocks and the `Pins` port group. The only
//! difference is which [`KicadFiles`] answers, so a message about a missing
//! model reads identically whether the file was missing from a disk or from a
//! repository.
//!
//! # One part, or all of them
//!
//! The download source also offers a BULK import: every symbol of one library,
//! of the libraries the search box matches, or of all 223. That is
//! [`super::kicad_bulk`] on a worker thread, reported here as a progress bar and
//! stopped by a button, and it can be stopped and restarted as often as the user
//! likes — what it finished is written to a ledger beside the cache, and a later
//! run skips it without downloading or converting it again.
//!
//! # Its own state machine
//!
//! The in-app dialog ([`super::kicad_import::KicadImport`]) is driven by the
//! file explorer and reads the chain on the UI thread, which a download cannot
//! do. This window therefore keeps its own [`Stage`], reads every part on a
//! worker ([`read_part`]) and reuses the dialog's chain, notes and review
//! wording rather than its staging. The stage transitions are plain methods
//! with no egui in them, so the suite drives the whole window headless.

#![cfg(not(target_arch = "wasm32"))]

use super::kicad_bulk::{
    AppStoreWriter, BulkRun, Ledger, PartWriter, Progress, StoreWriter, Sweep,
};
use super::kicad_import::{
    read_symbol_library, DiskFiles, Folder, FootprintChoice, Imported, KicadFiles, KicadLibrary,
    KicadPart,
};
use super::kicad_remote::{
    default_cache_root, http_fetcher, parse_tree, read_part, symbol_libraries, Fetcher, PartReader,
    PartSource, RemoteFiles, Repo, DEFAULT_TAG,
};
use crate::store::{default_model_store, ModelStore};
use brep_ecad_core::Symbol;
use eframe::egui;
use std::sync::Arc;

/// The variables KiCad names its SYMBOL folder by, newest first. The importer's
/// own library record holds footprints and 3D models only, because a symbol
/// reaches it as a file the user opened; this window lists a folder of them, so
/// it needs the third one.
/// The API's page cap, and so the size of a page that has another after it.
const PAGE: usize = 100;

const SYMBOL_VARS: &[&str] = &[
    "KICAD9_SYMBOL_DIR",
    "KICAD8_SYMBOL_DIR",
    "KICAD7_SYMBOL_DIR",
    "KICAD6_SYMBOL_DIR",
];

/// Where the symbol libraries of a local install are: the variable when it
/// names a folder, else `symbols` beside the footprints folder, which is how
/// KiCad lays its share directory out.
pub fn symbols_folder(env: &dyn Fn(&str) -> Option<String>, footprints: &str, files: &dyn KicadFiles) -> String {
    for var in SYMBOL_VARS {
        if let Some(path) = env(var).filter(|path| files.is_dir(path)) {
            return path;
        }
    }
    std::path::Path::new(footprints)
        .parent()
        .map(|dir| dir.join("symbols").to_string_lossy().into_owned())
        .unwrap_or_else(|| "symbols".into())
}

/// The `*.kicad_sym` library names in `dir`, without their extension, sorted.
/// The local twin of [`symbol_libraries`].
pub fn local_symbol_libraries(dir: &str, files: &dyn KicadFiles) -> Vec<String> {
    symbol_libraries(&files.list(dir))
}

/// A document name for a part: the symbol's own name, with anything a file name
/// cannot hold replaced. `Timer:NE555D` becomes `NE555D`.
pub fn document_name(library_id: &str) -> String {
    let bare = library_id.rsplit(':').next().unwrap_or(library_id);
    let cleaned: String = bare
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' || c == '.' { c } else { '_' })
        .collect();
    let trimmed = cleaned.trim_matches(['_', '.']).to_owned();
    if trimmed.is_empty() {
        "kicad-part".into()
    } else {
        trimmed
    }
}

// ============================================================================
// The local source
// ============================================================================

/// A symbol library being read off the UI thread. KiCad 9.0.9.1's largest is
/// `MCU_ST_STM32H7.kicad_sym` at **15.4 MB** and its `MCU_ST_STM32F4` is 6.6 MB
/// (measured against gitlab.com), so neither the download nor the parse belongs
/// on the thread drawing the window — the same reason the chain does not.
struct LibraryReader {
    answer: std::sync::mpsc::Receiver<Result<(Vec<Symbol>, Vec<String>), String>>,
    library: String,
}

/// A KiCad install on disk, as a [`PartSource`].
pub struct LocalSource {
    library: KicadLibrary,
}

impl LocalSource {
    pub fn new(library: KicadLibrary) -> Self {
        Self { library }
    }
}

impl KicadFiles for LocalSource {
    fn read(&self, path: &str) -> Option<Vec<u8>> {
        DiskFiles.read(path)
    }
    fn is_file(&self, path: &str) -> bool {
        DiskFiles.is_file(path)
    }
    fn is_dir(&self, path: &str) -> bool {
        DiskFiles.is_dir(path)
    }
    fn list(&self, dir: &str) -> Vec<String> {
        DiskFiles.list(dir)
    }
}

impl PartSource for LocalSource {
    fn library(&self) -> KicadLibrary {
        self.library.clone()
    }
}

// ============================================================================
// The window's state
// ============================================================================

/// Which source the window is reading.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// KiCad's repositories, at [`DEFAULT_TAG`].
    Download,
    /// This machine's KiCad install.
    Install,
}

impl Source {
    fn label(self) -> &'static str {
        match self {
            Source::Download => "Download KiCad's library",
            Source::Install => "Use my KiCad install",
        }
    }
}

/// Where the window is.
pub enum Stage {
    /// Neither source chosen yet.
    Choose,
    /// A source chosen: pick one symbol library out of its list.
    Libraries,
    /// A library being read, on a worker.
    Opening { library: String },
    /// A library read: pick one of its symbols.
    Symbols { library: String, symbols: Vec<Symbol>, warnings: Vec<String> },
    /// A symbol chosen and its chain being read, on a worker.
    Reading { symbol: String },
    /// A part read: review it and save it.
    Review { part: Box<KicadPart>, warnings: Vec<String>, downloaded: Vec<String> },
    /// Every symbol of one or more libraries being imported, on a worker.
    Bulk,
}

/// The `--kicad-library` window.
pub struct KicadLibraryWindow {
    pub stage: Stage,
    source: Source,
    /// The install's three folders, editable.
    symbols_dir: String,
    footprints_dir: String,
    models_dir: String,
    /// The download's cache root and tag, editable.
    cache_dir: String,
    tag: String,
    /// The library names of the chosen source, once listed.
    libraries: Vec<String>,
    /// The listing in flight, for the download source.
    index: Option<std::sync::mpsc::Receiver<Result<String, String>>>,
    /// Which page of that listing it is. KiCad 9.0.9.1 has 233 entries at the
    /// root of the symbols repository and the API caps a page at 100, so the
    /// list is only complete after the third — a listing that stopped at the
    /// first would silently be missing two thirds of the libraries.
    index_page: u32,
    /// The chain in flight.
    reader: Option<PartReader>,
    /// The symbol library in flight.
    library_reader: Option<LibraryReader>,
    search: String,
    /// The warnings the chosen library carried, held from [`Self::read_symbol`]
    /// until the worker answers in a later frame.
    pending_warnings: Vec<String>,
    /// The document the next save writes.
    document: String,
    pub status: String,
    store: Box<dyn ModelStore>,
    /// Every part written this session, newest last — the window's own record
    /// of its work, since there is no viewport to show it in.
    pub saved: Vec<String>,
    /// The sweep in flight, and the view of it the frame loop draws.
    pub bulk: Option<BulkRun>,
    /// Whether the next sweep redoes what the ledger has already settled.
    force_reimport: bool,
    /// What the tag's ledger held the last time it was read, for the line above
    /// the library list. Recomputed when a source is chosen and when a sweep
    /// stops, not every frame: it is a file read.
    ledger_line: String,
    /// How a sweep fetches. The app's is `http_fetcher`; the suite substitutes a
    /// stub so a whole sweep runs with no network.
    new_fetcher: Arc<dyn Fn() -> Fetcher + Send + Sync>,
    /// Where a sweep writes. The app's is the ordinary model store, built inside
    /// the worker; the suite substitutes an in-memory one so a sweep never
    /// touches the user's documents.
    new_writer: Arc<dyn Fn() -> Box<dyn PartWriter + Send> + Send + Sync>,
}

impl KicadLibraryWindow {
    /// Build the window against the app's own model store.
    pub fn new() -> Self {
        Self::with_store(default_model_store())
    }

    /// Build it against `store` — the seam the suite writes into memory through.
    pub fn with_store(store: Box<dyn ModelStore>) -> Self {
        let env = |var: &str| std::env::var(var).ok().filter(|value| !value.is_empty());
        let library = KicadLibrary::load(store.as_ref(), &env, &DiskFiles);
        let symbols = symbols_folder(&env, &library.footprints.path, &DiskFiles);
        Self {
            stage: Stage::Choose,
            source: Source::Download,
            symbols_dir: symbols,
            footprints_dir: library.footprints.path.clone(),
            models_dir: library.models.path.clone(),
            cache_dir: default_cache_root().to_string_lossy().into_owned(),
            tag: DEFAULT_TAG.into(),
            libraries: Vec::new(),
            index: None,
            index_page: 0,
            reader: None,
            library_reader: None,
            search: String::new(),
            pending_warnings: Vec::new(),
            document: String::new(),
            status: String::new(),
            store,
            saved: Vec::new(),
            bulk: None,
            force_reimport: false,
            ledger_line: String::new(),
            new_fetcher: Arc::new(http_fetcher),
            new_writer: Arc::new(|| Box::new(AppStoreWriter)),
        }
    }

    /// Point a sweep at another fetcher and another place to write — the seam the
    /// suite drives a whole sweep through, with no network and no documents in
    /// the user's config folder.
    pub fn sweep_through(
        &mut self,
        fetch: Arc<dyn Fn() -> Fetcher + Send + Sync>,
        write: Arc<dyn Fn() -> Box<dyn PartWriter + Send> + Send + Sync>,
    ) {
        self.new_fetcher = fetch;
        self.new_writer = write;
    }

    /// The install's folders as the chain wants them.
    fn install_library(&self) -> KicadLibrary {
        let folder = |path: &str| Folder { path: path.to_owned(), from: format!("your KiCad install ({path})") };
        KicadLibrary { footprints: folder(&self.footprints_dir), models: folder(&self.models_dir) }
    }

    /// A fresh cache over the download source's folders.
    fn remote(&self) -> RemoteFiles {
        RemoteFiles::new(&self.cache_dir, &self.tag, http_fetcher())
    }

    /// Choose a source and list its symbol libraries. The install lists a
    /// folder here and now; the download asks GitLab and answers in a later
    /// frame.
    pub fn choose(&mut self, source: Source, ctx: Option<&egui::Context>) {
        self.source = source;
        self.stage = Stage::Libraries;
        self.libraries.clear();
        self.search.clear();
        // Drop whatever the other source had in flight. A listing page landing
        // after the switch would extend the new source's list with the old
        // one's names; a chain landing after it would put the window in a
        // Review of a part nobody asked for any more. Dropping the receiver is
        // the "nobody waits" case the workers already expect.
        self.index = None;
        self.reader = None;
        self.library_reader = None;
        // A sweep is NOT dropped here: it owns no stage of its own to be
        // confused by, its worker writes documents and ledger lines that are
        // complete either way, and a user who switches source to look at
        // something should not silently lose an hour of importing. Its progress
        // keeps arriving and `Stage::Bulk` is still reachable.
        self.refresh_ledger_line();
        match source {
            Source::Install => {
                self.libraries = local_symbol_libraries(&self.symbols_dir, &DiskFiles);
                self.status = if self.libraries.is_empty() {
                    format!("no .kicad_sym library in {}", self.symbols_dir)
                } else {
                    format!("{} symbol libraries in {}", self.libraries.len(), self.symbols_dir)
                };
            }
            Source::Download => {
                self.status = format!("listing KiCad {}'s symbol libraries\u{2026}", self.tag);
                self.index_page = 0;
                self.next_index_page(ctx);
            }
        }
    }

    /// Ask for the page after the last one taken.
    fn next_index_page(&mut self, ctx: Option<&egui::Context>) {
        self.index_page += 1;
        let url = super::kicad_remote::tree_url(Repo::Symbols, &self.tag, "", self.index_page);
        self.index = ctx.map(|ctx| crate::http::fetch_text(ctx, url));
    }

    /// One page of the download's listing arrived. A FULL page means another
    /// follows; a short one ends the walk. Split out so the suite hands it a
    /// body without a network.
    pub fn take_index(&mut self, body: Result<String, String>, ctx: Option<&egui::Context>) {
        self.index = None;
        match body.and_then(|body| parse_tree(&body)) {
            Ok(names) => {
                let more = names.len() >= PAGE;
                self.libraries.extend(symbol_libraries(&names));
                self.libraries.sort();
                if more && self.index_page < 64 {
                    self.status = format!(
                        "listing KiCad {}'s symbol libraries\u{2026} {} so far",
                        self.tag,
                        self.libraries.len()
                    );
                    self.next_index_page(ctx);
                } else {
                    self.status = format!(
                        "{} symbol libraries in KiCad {} (type to search)",
                        self.libraries.len(),
                        self.tag
                    );
                }
            }
            Err(error) => self.status = format!("the library list was not read: {error}"),
        }
    }

    /// Read one symbol library on a worker: from the install's folder, or
    /// downloaded into the cache. See [`LibraryReader`] for why this is not
    /// done in the frame.
    pub fn open_library(&mut self, library: &str) {
        let file = format!("{library}.kicad_sym");
        let bytes: Box<dyn FnOnce() -> Result<Vec<u8>, String> + Send> = match self.source {
            Source::Install => {
                let path = std::path::Path::new(&self.symbols_dir).join(&file);
                Box::new(move || std::fs::read(&path).map_err(|error| format!("{}: {error}", path.display())))
            }
            Source::Download => {
                let (root, tag, wanted) = (self.cache_dir.clone(), self.tag.clone(), file.clone());
                Box::new(move || {
                    let remote = RemoteFiles::new(root, tag, http_fetcher());
                    remote.prepare().and_then(|()| remote.fetch_file(Repo::Symbols, &wanted))
                })
            }
        };
        let (send, answer) = std::sync::mpsc::channel();
        let named = file.clone();
        std::thread::spawn(move || {
            let _ = send.send(bytes().and_then(|bytes| read_symbol_library(&named, &bytes)));
        });
        self.library_reader = Some(LibraryReader { answer, library: library.to_owned() });
        self.status = format!("reading {library}\u{2026}");
        self.stage = Stage::Opening { library: library.to_owned() };
    }

    /// Follow one symbol's chain on a worker. A symbol that names no footprint
    /// imports with its symbol alone: this window offers no footprint chooser,
    /// because the download's would be one repository listing per library.
    pub fn read_symbol(&mut self, symbol: Symbol) {
        let name = symbol.library_id.clone();
        self.pending_warnings = match &self.stage {
            Stage::Symbols { warnings, .. } => warnings.clone(),
            _ => Vec::new(),
        };
        self.document = document_name(&name);
        self.status = format!("reading {name}\u{2026}");
        self.reader = Some(match self.source {
            Source::Install => read_part(symbol, FootprintChoice::Named, LocalSource::new(self.install_library())),
            Source::Download => {
                let remote = self.remote();
                let _ = remote.prepare();
                read_part(symbol, FootprintChoice::Named, remote)
            }
        });
        self.stage = Stage::Reading { symbol: name };
    }

    /// The worker's answer, once it has one.
    pub fn poll(&mut self, ctx: Option<&egui::Context>) {
        if let Some(run) = &mut self.bulk {
            let was_running = !run.progress.finished;
            let moved = run.poll();
            let stopped = moved && was_running && run.progress.finished;
            if !run.progress.finished {
                if let Some(ctx) = ctx {
                    ctx.request_repaint_after(std::time::Duration::from_millis(250));
                }
            }
            if stopped {
                // It has stopped: say how, and re-read the ledger it wrote so the
                // library list's summary is the new truth.
                let line = run.progress.line();
                self.status = format!("import {line}");
                self.refresh_ledger_line();
            }
        }
        if let Some(index) = &self.index {
            if let Ok(body) = index.try_recv() {
                self.take_index(body, ctx);
            }
        }
        if let Some(reader) = &self.library_reader {
            match reader.answer.try_recv() {
                Ok(Ok((symbols, warnings))) => {
                    let library = reader.library.clone();
                    self.library_reader = None;
                    self.status = format!("{library} holds {} symbols", symbols.len());
                    self.search.clear();
                    self.stage = Stage::Symbols { library, symbols, warnings };
                }
                Ok(Err(error)) => {
                    self.status = format!("{} was not read: {error}", reader.library);
                    self.library_reader = None;
                    self.stage = Stage::Libraries;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {
                    if let Some(ctx) = ctx {
                        ctx.request_repaint_after(std::time::Duration::from_millis(100));
                    }
                }
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.status = format!("{} was not read: the reader stopped", reader.library);
                    self.library_reader = None;
                    self.stage = Stage::Libraries;
                }
            }
        }
        let Some(reader) = &self.reader else {
            return;
        };
        match reader.take() {
            Some((part, downloaded)) => {
                self.reader = None;
                self.status = if downloaded.is_empty() {
                    "read".into()
                } else {
                    format!("downloaded {} files", downloaded.len())
                };
                let warnings = std::mem::take(&mut self.pending_warnings);
                self.stage = Stage::Review { part: Box::new(part), warnings, downloaded };
            }
            None => {
                if let Some(ctx) = ctx {
                    ctx.request_repaint_after(std::time::Duration::from_millis(100));
                }
            }
        }
    }

    /// Write the reviewed part as a document, through the ordinary model store.
    /// The part goes into a FRESH engine every time, so two saves in one
    /// session never share a history.
    pub fn save(&mut self) -> Result<Imported, String> {
        let Stage::Review { part, .. } = &self.stage else {
            return Err("nothing has been read yet".into());
        };
        let name = self.document.trim();
        if name.is_empty() {
            return Err("the part needs a name".into());
        }
        let name = name.to_owned();
        // The same call the sweep makes, so a part imported by hand and the same
        // part imported by a sweep are the same document.
        let imported = super::kicad_bulk::write_part(&StoreWriter(self.store.as_ref()), &name, part)?;
        self.saved.push(name.clone());
        self.status = format!("saved {name}: {}", imported.summary);
        self.stage = Stage::Libraries;
        Ok(imported)
    }

    /// Where the window is, in words — the line the suite and the status bar
    /// both read.
    pub fn stage_name(&self) -> &'static str {
        match self.stage {
            Stage::Choose => "choose",
            Stage::Libraries => "libraries",
            Stage::Opening { .. } => "opening",
            Stage::Symbols { .. } => "symbols",
            Stage::Reading { .. } => "reading",
            Stage::Review { .. } => "review",
            Stage::Bulk => "bulk",
        }
    }

    /// The ledger's own summary for the tag and cache folder now named, read off
    /// disk. Empty when nothing has ever been swept into this folder.
    pub fn refresh_ledger_line(&mut self) {
        if self.source != Source::Download {
            self.ledger_line.clear();
            return;
        }
        let ledger = Ledger::read(std::path::Path::new(&self.cache_dir), &self.tag);
        let (done, retry) = ledger.counts();
        if done == 0 && retry == 0 {
            self.ledger_line = format!("nothing imported into {} yet", self.cache_dir);
            return;
        }
        let grades: Vec<String> = ledger
            .grades()
            .into_iter()
            .filter(|(_, count)| *count > 0)
            .map(|(grade, count)| format!("{count} {}", grade.as_str()))
            .collect();
        let torn = if ledger.dropped() > 0 {
            format!(", {} unreadable line(s) ignored", ledger.dropped())
        } else {
            String::new()
        };
        self.ledger_line = format!(
            "KiCad {}: {done} parts already imported ({}), {retry} to retry{torn} \u{2014} {}",
            self.tag,
            grades.join(", "),
            ledger.path().display()
        );
    }

    /// Start a sweep over `libraries`, resuming from the tag's ledger unless the
    /// force box is ticked.
    pub fn start_bulk(&mut self, libraries: Vec<String>) {
        if libraries.is_empty() {
            self.status = "there is no library to import".into();
            return;
        }
        if self.bulk.as_ref().is_some_and(|run| !run.finished()) {
            self.status = "a sweep is already running".into();
            return;
        }
        let plan = Sweep {
            cache_root: std::path::PathBuf::from(&self.cache_dir),
            tag: self.tag.clone(),
            libraries,
            force: self.force_reimport,
        };
        let wanted = plan.libraries.len();
        match super::kicad_bulk::start(plan, (self.new_fetcher)(), (self.new_writer)()) {
            Ok(run) => {
                self.status = format!(
                    "importing {wanted} librar{} into {}; {} parts already done{}",
                    if wanted == 1 { "y" } else { "ies" },
                    self.cache_dir,
                    run.resumed,
                    if self.force_reimport {
                        ", and every one of them again"
                    } else {
                        ", which are skipped"
                    }
                );
                self.bulk = Some(run);
                self.stage = Stage::Bulk;
            }
            Err(error) => self.status = format!("the import did not start: {error}"),
        }
    }

    /// The sweep's latest snapshot, for the suite and the status strip.
    pub fn bulk_progress(&self) -> Option<&Progress> {
        self.bulk.as_ref().map(|run| &run.progress)
    }

    /// The libraries on offer, narrowed by the search field.
    fn matching(&self) -> Vec<String> {
        let needle = self.search.to_lowercase();
        self.libraries
            .iter()
            .filter(|name| name.to_lowercase().contains(&needle))
            .cloned()
            .collect()
    }
}

impl eframe::App for KicadLibraryWindow {
    /// eframe 0.35 hands the app a `Ui` filling the window, so the status strip
    /// is a bottom panel INSIDE it and everything else fills what is left.
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.poll(Some(&ctx));
        egui::Panel::bottom("kicad-library-status").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| ui.weak(&self.status));
            if !self.saved.is_empty() {
                ui.weak(format!("saved this session: {}", self.saved.join(", ")));
            }
        });
        ui.heading("Import from KiCad");
        ui.add_space(4.0);
        self.show_source(ui);
        ui.separator();
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            self.show_stage(ui);
        });
    }
}

impl KicadLibraryWindow {
    /// The two paths, and the folders whichever one is chosen reads.
    fn show_source(&mut self, ui: &mut egui::Ui) {
        let mut chosen = None;
        ui.horizontal(|ui| {
            for source in [Source::Download, Source::Install] {
                let picked = self.source == source && !matches!(self.stage, Stage::Choose);
                if ui.selectable_label(picked, source.label()).clicked() {
                    chosen = Some(source);
                }
            }
        });
        match self.source {
            Source::Download => {
                egui::Grid::new("kicad-library-remote").num_columns(2).show(ui, |ui| {
                    ui.label("KiCad release");
                    ui.text_edit_singleline(&mut self.tag);
                    ui.end_row();
                    ui.label("Downloaded to");
                    ui.text_edit_singleline(&mut self.cache_dir);
                    ui.end_row();
                });
                ui.weak(
                    "Symbols, footprints and 3D models come from KiCad's own repositories, \
                     one file per part, into a folder laid out like a KiCad install.",
                );
            }
            Source::Install => {
                egui::Grid::new("kicad-library-local").num_columns(2).show(ui, |ui| {
                    for (label, field) in [
                        ("Symbols", &mut self.symbols_dir),
                        ("Footprints", &mut self.footprints_dir),
                        ("3D models", &mut self.models_dir),
                    ] {
                        ui.label(label);
                        ui.text_edit_singleline(field);
                        ui.end_row();
                    }
                });
            }
        }
        if let Some(source) = chosen {
            let ctx = ui.ctx().clone();
            self.choose(source, Some(&ctx));
        }
    }

    fn show_stage(&mut self, ui: &mut egui::Ui) {
        let mut open = None;
        let mut read = None;
        let mut sweep = None;
        // The bulk controls and the search field are drawn BEFORE the match, which
        // borrows the stage: both need `&mut self`.
        if matches!(self.stage, Stage::Libraries) && self.source == Source::Download {
            sweep = self.show_bulk_controls(ui);
            ui.separator();
        }
        if matches!(self.stage, Stage::Libraries | Stage::Symbols { .. }) {
            ui.text_edit_singleline(&mut self.search);
        }
        match &self.stage {
            Stage::Choose => {
                ui.weak("Choose a source above.");
            }
            Stage::Libraries => {
                let bulk = self.source == Source::Download;
                for name in self.matching() {
                    ui.horizontal(|ui| {
                        if ui.selectable_label(false, &name).clicked() {
                            open = Some(name.clone());
                        }
                        // One library is how a user samples a sweep before
                        // committing to all 165 of them, and it is the only
                        // size of sweep that has been measured end to end.
                        if bulk && ui.small_button("import all of it").clicked() {
                            sweep = Some(vec![name.clone()]);
                        }
                    });
                }
                if self.libraries.is_empty() {
                    ui.weak("No symbol library here yet.");
                }
            }
            Stage::Symbols { library, symbols, .. } => {
                ui.label(format!("{library}: {} symbols", symbols.len()));
                let needle = self.search.to_lowercase();
                for symbol in symbols {
                    if !symbol.library_id.to_lowercase().contains(&needle) {
                        continue;
                    }
                    let row = ui.selectable_label(
                        false,
                        format!("{}  ({} pins)", symbol.library_id, symbol.pins.len()),
                    );
                    if row.clicked() {
                        read = Some(symbol.clone());
                    }
                }
            }
            Stage::Opening { library } => {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label(format!("reading {library}\u{2026}"));
                });
            }
            Stage::Reading { symbol } => {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label(format!("reading {symbol} and its 3D model\u{2026}"));
                });
            }
            Stage::Review { part, warnings, downloaded } => {
                show_review(ui, part, warnings, downloaded);
            }
            Stage::Bulk => {
                if let Some(run) = &self.bulk {
                    show_sweep(ui, run);
                } else {
                    ui.weak("No import has been started.");
                }
            }
        }
        if matches!(self.stage, Stage::Bulk) {
            ui.separator();
            let running = self.bulk.as_ref().is_some_and(|run| !run.finished());
            let stopping = self.bulk.as_ref().is_some_and(BulkRun::stopping);
            ui.horizontal(|ui| {
                if running {
                    if ui.add_enabled(!stopping, egui::Button::new("Stop")).clicked() {
                        if let Some(run) = &self.bulk {
                            run.stop();
                        }
                        self.status = "stopping after the part being imported \u{2014} \
                                       it is finished and recorded first"
                            .into();
                    }
                    if stopping {
                        ui.weak("stopping\u{2026}");
                    }
                } else if ui.button("Back to the libraries").clicked() {
                    self.stage = Stage::Libraries;
                }
            });
        }
        if matches!(self.stage, Stage::Review { .. }) {
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("Save as");
                ui.text_edit_singleline(&mut self.document);
                if ui.button("Save part").clicked() {
                    if let Err(error) = self.save() {
                        self.status = format!("not saved: {error}");
                    }
                }
                if ui.button("Back").clicked() {
                    self.stage = Stage::Libraries;
                }
            });
        }
        if let Some(libraries) = sweep {
            self.start_bulk(libraries);
        }
        if let Some(name) = open {
            self.open_library(&name);
        }
        if let Some(symbol) = read {
            self.read_symbol(symbol);
        }
    }

    /// The bulk row above the library list: what the ledger already holds, the
    /// force box, and the button that starts the whole sweep. Answers with the
    /// libraries to sweep when one was asked for.
    fn show_bulk_controls(&mut self, ui: &mut egui::Ui) -> Option<Vec<String>> {
        let mut wanted = None;
        let matching = self.matching();
        let running = self.bulk.as_ref().is_some_and(|run| !run.finished());
        ui.horizontal_wrapped(|ui| {
            let all = egui::Button::new(format!("Import every part of all {} libraries", self.libraries.len()));
            if ui.add_enabled(!running && !self.libraries.is_empty(), all).clicked() {
                wanted = Some(self.libraries.clone());
            }
            if matching.len() != self.libraries.len() && !matching.is_empty() {
                let some = egui::Button::new(format!("\u{2026}or just the {} matching", matching.len()));
                if ui.add_enabled(!running, some).clicked() {
                    wanted = Some(matching);
                }
            }
            ui.checkbox(&mut self.force_reimport, "Force reimport")
                .on_hover_text(
                    "Redo parts this cache has already imported, instead of skipping them. \
                     Off, an import carries on where the last one stopped.",
                );
            if running && ui.button("Show the import in progress").clicked() {
                self.stage = Stage::Bulk;
            }
        });
        ui.weak(&self.ledger_line);
        ui.weak(
            "An import can be stopped and started again as often as you like: what it finished \
             is written down, and a later run skips it without downloading or converting it \
             again. A part with no footprint or no 3D model still becomes a part, and says so.",
        );
        wanted
    }
}

/// The progress of a sweep: two bars, because the only honest denominator is
/// libraries — how many symbols a library holds is unknown until its
/// `.kicad_sym` has been downloaded and parsed.
fn show_sweep(ui: &mut egui::Ui, run: &BulkRun) {
    let progress = &run.progress;
    ui.add(egui::ProgressBar::new(progress.fraction()).show_percentage());
    if progress.symbols_total > 0 {
        let within = progress.symbols_done as f32 / progress.symbols_total as f32;
        ui.add(
            egui::ProgressBar::new(within.clamp(0.0, 1.0))
                .desired_height(6.0)
                .text(format!(
                    "{}: {}/{}",
                    progress.library, progress.symbols_done, progress.symbols_total
                )),
        );
    }
    ui.add_space(4.0);
    egui::Grid::new("kicad-library-sweep").num_columns(2).show(ui, |ui| {
        for (label, value) in [
            ("Library", format!("{} of {}", progress.libraries_done.min(progress.libraries_total), progress.libraries_total)),
            ("Now", if progress.symbol.is_empty() { progress.library.clone() } else { progress.symbol.clone() }),
            ("Imported", progress.imported.to_string()),
            ("Already done", progress.skipped.to_string()),
            ("Failed, to retry later", progress.failed.to_string()),
            ("Downloaded", format!("{} files, {:.1} MB", progress.files, progress.bytes as f64 / 1_048_576.0)),
            ("Recorded in", run.ledger.display().to_string()),
        ] {
            ui.label(label);
            ui.label(value);
            ui.end_row();
        }
        if progress.libraries_failed > 0 {
            ui.label("Libraries not read");
            ui.label(progress.libraries_failed.to_string());
            ui.end_row();
        }
    });
    if let Some(error) = &progress.error {
        ui.colored_label(egui::Color32::from_rgb(220, 120, 60), error);
    }
    if !progress.notes.is_empty() {
        ui.add_space(4.0);
        ui.weak("What went wrong, most recent last \u{2014} a later import retries exactly these:");
        for note in &progress.notes {
            ui.weak(note);
        }
    }
    if progress.finished && !progress.cancelled && progress.error.is_none() {
        ui.add_space(4.0);
        ui.label("Finished.");
    }
}

/// What the save will write, and every note the chain left.
fn show_review(ui: &mut egui::Ui, part: &KicadPart, warnings: &[String], downloaded: &[String]) {
    egui::Grid::new("kicad-library-review").num_columns(2).show(ui, |ui| {
        ui.label("Symbol");
        ui.label(
            part.symbol
                .as_ref()
                .map_or("none".into(), |s| format!("{} ({} pins)", s.library_id, s.pins.len())),
        );
        ui.end_row();
        ui.label("Pads");
        ui.label(match (&part.footprint, &part.footprint_file) {
            (Some(f), Some(file)) => format!("{} ({} pads) from {file}", f.name, f.pads.len()),
            (Some(f), None) => format!("{} ({} pads)", f.name, f.pads.len()),
            (None, _) => "none".into(),
        });
        ui.end_row();
        ui.label("3D model");
        ui.label(match &part.model {
            Some(model) => format!("{} ({} bodies)", model.step_file, model.bodies.len()),
            None => "none".into(),
        });
        ui.end_row();
        if let Some(symbol) = &part.symbol {
            ui.label("Connection points");
            ui.label(format!(
                "{} in the part's `Pins` group, {}",
                symbol.pins.len(),
                if part.footprint.is_some() { "each at its pin's pad" } else { "at the part origin" }
            ));
            ui.end_row();
        }
        if !downloaded.is_empty() {
            ui.label("Downloaded");
            ui.label(downloaded.join("\n"));
            ui.end_row();
        }
    });
    for note in warnings.iter().chain(&part.notes) {
        ui.colored_label(ui.visuals().warn_fg_color, format!("\u{2022} {note}"));
    }
}

