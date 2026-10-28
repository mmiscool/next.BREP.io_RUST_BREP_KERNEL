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
//! * **Clone** — KiCad's own three library repositories, cloned with git at a
//!   pinned 9.x tag ([`brep_ecad_core::kicad_git`]) into folders laid out like a
//!   KiCad install, then read exactly as an install is. When git is missing or
//!   GitLab cannot be reached the window says so and switches to the install
//!   source, whose three folder fields are where the user names their own.
//! * **Your KiCad install** — the folders already on this machine, found the
//!   way the in-app dialog finds them ([`KicadLibrary::load`]).
//!
//! Both end in the SAME call: `part_from_symbol` follows the symbol to its
//! footprint to its `.step`, and [`import_part`](crate::panels::kicad_import::import_part) writes the IMPORT3D feature,
//! the `symbol` and `pads` blocks and the `Pins` port group. The only
//! difference is which folders it reads.
//!
//! # One part, or all of them
//!
//! Either source also offers a BULK import: every symbol of one library,
//! of the libraries the search box matches, or of all 223. That is
//! [`super::kicad_bulk`] on a worker thread, reported here as a progress bar and
//! stopped by a button, and it can be stopped and restarted as often as the user
//! likes — what it finished is written to a ledger beside the clones, and a
//! later run skips it without converting it again.
//!
//! # Its own state machine
//!
//! The in-app dialog ([`super::kicad_import::KicadImport`]) is driven by the
//! file explorer and reads the chain on the UI thread, which a clone and a
//! library's worth of STEP parsing cannot share. This window therefore keeps its own [`Stage`], reads every part on a
//! worker ([`read_part`]) and reuses the dialog's chain, notes and review
//! wording rather than its staging. The stage transitions are plain methods
//! with no egui in them, so the suite drives the whole window headless.

#![cfg(not(target_arch = "wasm32"))]

use super::kicad_bulk::{
    AppStoreWriter, BulkRun, Ledger, PartWriter, Progress, StoreWriter, Sweep,
};
use super::kicad_import::{
    part_from_symbol, read_symbol_library, DiskFiles, Folder, FootprintChoice, Imported, KicadFiles,
    KicadLibrary, KicadPart,
};
use crate::store::{default_model_store, ModelStore};
use brep_ecad_core::kicad_git::{self, Repo};
use brep_ecad_core::Symbol;
use eframe::egui;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// The release a clone reads; the window's tag field starts here.
pub const DEFAULT_TAG: &str = kicad_git::TAG;

/// Where clones and the bulk ledger live when the user names no folder: beside
/// the app's own state (the model store's `<config>/brep-app`), so they
/// survive a session — `%APPDATA%` on Windows.
pub fn default_cache_root() -> PathBuf {
    crate::plm::config::app_config_dir().join("kicad-library")
}

/// The `*.kicad_sym` library names among `names`, without their extension,
/// sorted.
pub fn symbol_libraries(names: &[String]) -> Vec<String> {
    let mut found: Vec<String> = names
        .iter()
        .filter_map(|name| name.strip_suffix(".kicad_sym"))
        .map(str::to_owned)
        .collect();
    found.sort();
    found
}

/// The variables KiCad names its SYMBOL folder by, newest first. The importer's
/// own library record holds footprints and 3D models only, because a symbol
/// reaches it as a file the user opened; this window lists a folder of them, so
/// it needs the third one.
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

/// Where one import's files come from, for [`read_part`].
pub trait PartSource: KicadFiles + Send + 'static {
    /// The folders the chain follows a footprint and a model through.
    fn library(&self) -> KicadLibrary;
}

/// A part being read off the UI thread: the chain and the STEP parse.
pub struct PartReader {
    answer: std::sync::mpsc::Receiver<KicadPart>,
}

impl PartReader {
    /// The answer, once the worker has one. `None` while it works; a worker
    /// that died without one answers with a note rather than hanging.
    pub fn take(&self) -> Option<KicadPart> {
        match self.answer.try_recv() {
            Ok(answer) => Some(answer),
            Err(std::sync::mpsc::TryRecvError::Empty) => None,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => Some(KicadPart {
                notes: vec!["the part reader stopped without an answer".into()],
                ..KicadPart::default()
            }),
        }
    }
}

/// Follow `symbol`'s chain through `source` on a worker thread: the footprint,
/// the 3D model, and the model's STEP parsed (529 ms for a real SOIC-8).
pub fn read_part(symbol: Symbol, choice: FootprintChoice, source: impl PartSource) -> PartReader {
    let (send, answer) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let library = source.library();
        let part = part_from_symbol(symbol, &choice, &library, &source).parsed();
        // A window closed meanwhile has dropped the receiver; nobody waits.
        let _ = send.send(part);
    });
    PartReader { answer }
}

/// KiCad's three repositories being cloned on a worker.
pub struct CloneRun {
    updates: std::sync::mpsc::Receiver<CloneUpdate>,
    cancel: Arc<AtomicBool>,
    /// git's latest progress line.
    pub line: String,
}

enum CloneUpdate {
    Line(String),
    Done(Result<(), String>),
}

impl CloneRun {
    /// Clone each of the three that is not under `root` yet, from `base`.
    fn start(base: String, tag: String, root: PathBuf) -> Self {
        let (send, updates) = std::sync::mpsc::channel();
        let cancel = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&cancel);
        std::thread::spawn(move || {
            let mut result = kicad_git::git_version().map(|_| ());
            for repo in Repo::ALL {
                if result.is_err() {
                    break;
                }
                let mut report = |line: &str| {
                    let _ = send.send(CloneUpdate::Line(format!("{}: {line}", repo.project())));
                };
                let dest = root.join(repo.install_folder());
                result = kicad_git::clone(&repo.url(&base), &tag, &dest, &mut report, &flag).map(|_| ());
            }
            let _ = send.send(CloneUpdate::Done(result));
        });
        Self { updates, cancel, line: "starting git".into() }
    }

    /// Stop git; the repository in hand is removed, those already cloned stay.
    pub fn stop(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    pub fn stopping(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }
}

// ============================================================================
// The window's state
// ============================================================================

/// Which source the window is reading.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// KiCad's repositories, cloned at the window's tag.
    Download,
    /// This machine's KiCad install.
    Install,
}

impl Source {
    fn label(self) -> &'static str {
        match self {
            Source::Download => "Clone KiCad's library",
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
    Review { part: Box<KicadPart>, warnings: Vec<String> },
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
    /// Where clones and the bulk ledger go, and the release cloned; editable.
    /// The clones are `<cache_dir>/<tag>/{symbols,footprints,3dmodels}`.
    cache_dir: String,
    tag: String,
    /// Where the clone source clones from; the suite names a local repository.
    clone_base: String,
    /// The library names of the chosen source, once listed.
    libraries: Vec<String>,
    /// The clone in flight.
    pub cloning: Option<CloneRun>,
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
            clone_base: kicad_git::BASE_URL.into(),
            libraries: Vec::new(),
            cloning: None,
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
            new_writer: Arc::new(|| Box::new(AppStoreWriter)),
        }
    }

    /// Point a sweep at another place to write — the seam the suite drives a
    /// whole sweep through, with no documents in the user's config folder.
    pub fn sweep_through(&mut self, write: Arc<dyn Fn() -> Box<dyn PartWriter + Send> + Send + Sync>) {
        self.new_writer = write;
    }

    /// Clone from `base` and into `cache_dir` instead — the suite's local
    /// repositories and temp folder.
    pub fn clone_from(&mut self, base: &str, cache_dir: &Path) {
        self.clone_base = base.to_owned();
        self.cache_dir = cache_dir.to_string_lossy().into_owned();
    }

    /// Where the clones of the window's tag are.
    fn clone_root(&self) -> PathBuf {
        Path::new(&self.cache_dir).join(&self.tag)
    }

    /// The symbols, footprints and 3D-model folders the chosen source reads.
    fn folders(&self) -> (String, String, String) {
        match self.source {
            Source::Install => (self.symbols_dir.clone(), self.footprints_dir.clone(), self.models_dir.clone()),
            Source::Download => {
                let root = self.clone_root();
                let [symbols, footprints, models] =
                    Repo::ALL.map(|repo| root.join(repo.install_folder()).to_string_lossy().into_owned());
                (symbols, footprints, models)
            }
        }
    }

    /// The chosen source's folders as the chain wants them.
    fn library(&self) -> KicadLibrary {
        let (_, footprints, models) = self.folders();
        let from = match self.source {
            Source::Install => "your KiCad install",
            Source::Download => "the clone of KiCad's library",
        };
        let folder = |path: String| Folder { from: format!("{from} ({path})"), path };
        KicadLibrary { footprints: folder(footprints), models: folder(models) }
    }

    /// Choose a source and list its symbol libraries. The install lists a
    /// folder here and now; the clone source lists its clone, cloning it first
    /// on a worker when it is not there yet.
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
        self.reader = None;
        self.library_reader = None;
        // A sweep is NOT dropped here: it owns no stage of its own to be
        // confused by, its worker writes documents and ledger lines that are
        // complete either way, and a user who switches source to look at
        // something should not silently lose an hour of importing. Its progress
        // keeps arriving and `Stage::Bulk` is still reachable.
        self.refresh_ledger_line();
        let cloned = Repo::ALL.iter().all(|repo| self.clone_root().join(repo.install_folder()).is_dir());
        if source == Source::Download && !cloned {
            if self.cloning.is_none() {
                self.cloning = Some(CloneRun::start(self.clone_base.clone(), self.tag.clone(), self.clone_root()));
            }
            self.status = format!(
                "cloning KiCad {}'s libraries into {} with git (about 5.6 GB, once)\u{2026}",
                self.tag,
                self.clone_root().display()
            );
            if let Some(ctx) = ctx {
                ctx.request_repaint_after(std::time::Duration::from_millis(100));
            }
            return;
        }
        self.list_libraries();
    }

    /// List the chosen source's symbol folder.
    fn list_libraries(&mut self) {
        let (symbols, _, _) = self.folders();
        self.libraries = local_symbol_libraries(&symbols, &DiskFiles);
        self.status = if self.libraries.is_empty() {
            format!("no .kicad_sym library in {symbols}")
        } else {
            format!("{} symbol libraries in {symbols} (type to search)", self.libraries.len())
        };
    }

    /// The clone's news, once per frame: git's progress, then either the
    /// library list or why it failed — and the folder fields to fill instead.
    fn poll_clone(&mut self) {
        let Some(run) = &mut self.cloning else {
            return;
        };
        let mut done = None;
        let mut gone = false;
        loop {
            match run.updates.try_recv() {
                Ok(CloneUpdate::Line(line)) => run.line = line,
                Ok(CloneUpdate::Done(result)) => done = Some(result),
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    gone = true;
                    break;
                }
            }
        }
        let done = match done {
            Some(done) => done,
            None if gone => Err("the clone stopped without saying why".into()),
            None => return,
        };
        self.cloning = None;
        match done {
            Ok(()) if self.source == Source::Download => self.list_libraries(),
            Ok(()) => self.status = format!("KiCad {}'s libraries are cloned", self.tag),
            Err(error) => {
                self.source = Source::Install;
                self.stage = Stage::Choose;
                self.status = format!(
                    "KiCad's libraries were not cloned: {error}. Install git, or name the folders of \
                     a KiCad library on this machine above and choose \"{}\".",
                    Source::Install.label()
                );
            }
        }
    }

    /// Read one symbol library on a worker. See `LibraryReader` for why this
    /// is not done in the frame.
    pub fn open_library(&mut self, library: &str) {
        let file = format!("{library}.kicad_sym");
        let path = Path::new(&self.folders().0).join(&file);
        let (send, answer) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let read = std::fs::read(&path).map_err(|error| format!("{}: {error}", path.display()));
            let _ = send.send(read.and_then(|bytes| read_symbol_library(&file, &bytes)));
        });
        self.library_reader = Some(LibraryReader { answer, library: library.to_owned() });
        self.status = format!("reading {library}\u{2026}");
        self.stage = Stage::Opening { library: library.to_owned() };
    }

    /// Follow one symbol's chain on a worker. A symbol that names no footprint
    /// imports with its symbol alone: this window offers no footprint chooser.
    pub fn read_symbol(&mut self, symbol: Symbol) {
        let name = symbol.library_id.clone();
        self.pending_warnings = match &self.stage {
            Stage::Symbols { warnings, .. } => warnings.clone(),
            _ => Vec::new(),
        };
        self.document = document_name(&name);
        self.status = format!("reading {name}\u{2026}");
        self.reader = Some(read_part(symbol, FootprintChoice::Named, LocalSource::new(self.library())));
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
        if self.cloning.is_some() {
            self.poll_clone();
            if let (Some(ctx), Some(_)) = (ctx, &self.cloning) {
                ctx.request_repaint_after(std::time::Duration::from_millis(250));
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
            Some(part) => {
                self.reader = None;
                self.status = "read".into();
                let warnings = std::mem::take(&mut self.pending_warnings);
                self.stage = Stage::Review { part: Box::new(part), warnings };
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
            cache_root: PathBuf::from(&self.cache_dir),
            tag: self.tag.clone(),
            symbols_dir: PathBuf::from(self.folders().0),
            library: self.library(),
            libraries,
            force: self.force_reimport,
        };
        let wanted = plan.libraries.len();
        match super::kicad_bulk::start(plan, Box::new(DiskFiles), (self.new_writer)()) {
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
        // The two sources as one dropdown (the form's enum field): nothing
        // shown until one is chosen.
        let mut chosen = None;
        ui.horizontal(|ui| {
            ui.label("Library");
            let choices = [
                (Some(Source::Download), "download", Source::Download.label()),
                (Some(Source::Install), "install", Source::Install.label()),
            ];
            let mut picked = (!matches!(self.stage, Stage::Choose)).then_some(self.source);
            let (changed, _) = crate::form::enum_dropdown(ui, "library:source", &choices, &mut picked, None, None);
            if changed {
                chosen = picked;
            }
        });
        match self.source {
            Source::Download => {
                let idle = self.cloning.is_none();
                egui::Grid::new("kicad-library-remote").num_columns(2).show(ui, |ui| {
                    ui.label("KiCad release");
                    ui.add_enabled(idle, egui::TextEdit::singleline(&mut self.tag));
                    ui.end_row();
                    ui.label("Cloned into");
                    ui.add_enabled(idle, egui::TextEdit::singleline(&mut self.cache_dir));
                    ui.end_row();
                });
                ui.weak(
                    "Symbols, footprints and 3D models are cloned with git from KiCad's own \
                     repositories, once (about 5.6 GB), into folders laid out like a KiCad install. \
                     git must be installed; without it, use your own KiCad folders instead.",
                );
                if let Some(run) = &self.cloning {
                    let stopping = run.stopping();
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label(&run.line);
                        if ui.add_enabled(!stopping, egui::Button::new("Stop")).clicked() {
                            run.stop();
                        }
                    });
                }
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
        if matches!(self.stage, Stage::Libraries) && !self.libraries.is_empty() {
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
                for name in self.matching() {
                    ui.horizontal(|ui| {
                        if ui.selectable_label(false, &name).clicked() {
                            open = Some(name.clone());
                        }
                        // One library is how a user samples a sweep before
                        // committing to all 165 of them, and it is the only
                        // size of sweep that has been measured end to end.
                        if ui.small_button("import all of it").clicked() {
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
            Stage::Review { part, warnings } => {
                show_review(ui, part, warnings);
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
             is written down, and a later run skips it without converting it again. A part with no footprint or no 3D model still becomes a part, and says so.",
        );
        wanted
    }
}

/// The progress of a sweep: two bars, because the only honest denominator is
/// libraries — how many symbols a library holds is unknown until its
/// `.kicad_sym` has been parsed.
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
fn show_review(ui: &mut egui::Ui, part: &KicadPart, warnings: &[String]) {
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
    });
    for note in warnings.iter().chain(&part.notes) {
        ui.colored_label(ui.visuals().warn_fg_color, format!("\u{2022} {note}"));
    }
}

