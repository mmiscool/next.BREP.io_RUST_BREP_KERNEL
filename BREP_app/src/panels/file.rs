//! File dialog — a **reusable modal** for model-document management: **New /
//! Open / Save / Save As** of the MODEL, driven from the toolbar. Mirrors the
//! shape of [`crate::palette::Palette`]: the caller owns ONE [`FileDialog`],
//! calls [`FileDialog::dispatch`] when a toolbar file button is clicked (which
//! either acts immediately or opens the modal in a mode), and calls
//! [`FileDialog::show`] every frame to draw the (possibly open) modal.
//!
//! The model document IS the engine-owned history: `EngineState` serializes it
//! with `history_request_json()` (the `.nbrep` recipe) and loads it with
//! `set_history_json` / `load_model_and_fit`. This dialog keeps NO model state
//! and no document IDENTITY — the name and the clean baseline live on
//! [`Document`], and the open set on [`Documents`]. It holds only transient UI
//! buffers (the name field, a status line, the open modal) and drives the
//! documents + the [`ModelStore`] seam.
//!
//! # New and Open never discard anything now
//!
//! Both ADD A TAB: New pushes an empty document, Open pushes the opened one (or
//! focuses the tab already holding it). Nothing is replaced, so neither needs
//! the "discard unsaved changes?" prompt they used to raise — the only place
//! unsaved work can still be lost is CLOSING a tab, which is where that
//! confirmation now lives ([`Mode::ConfirmClose`]).
//!
//! Persistence crosses the unified [`ModelStore`] seam — the ONE platform
//! exception. The same embeddable explorer renders on web and desktop. The web
//! backend lists localStorage models and can Upload; desktop lists the app's
//! models directory. No OS-native file dialog is used.

use crate::automation::hit_keys::HitKeyDoc;
use crate::document::{Document, Documents, EMPTY_DOCUMENT};
use crate::document_class::{self, DocumentClass};
use crate::family_table::{self, FamilySource};
use crate::template::{self, TemplateInput};
use crate::panels::parts_library::document_signature;
use crate::panels::file_explorer::{self, FileExplorer, FileExplorerOptions};
use crate::panels::kicad_import::{self, DiskFiles, KicadImport, KicadKind};
use crate::store::{model_display_name, sibling_identity, ModelStore, PlaceKind, MODEL_EXTENSIONS};
use brep_render::engine_state::{
    ComponentInsert, EngineState, PartSink, StepAssemblyImport, StepAssemblyProbe,
    StepAssemblyReport, StepProbeOutcome,
};
use brep_render::runner::MeshImportFormat;
use eframe::egui;
use std::collections::{BTreeMap, HashMap, HashSet};

/// The file operation a toolbar button requests. The shell maps a clicked
/// toolbar button to one of these and hands it to [`FileDialog::dispatch`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FileAction {
    New,
    /// New, of a chosen class: an empty family seed or template in a new tab
    /// (the toolbar's New menu). `NewOfClass(Normal)` is plain New.
    NewOfClass(DocumentClass),
    Open,
    Save,
    SaveAs,
    /// Import a CAD or mesh file FROM the user's filesystem, appending it to the
    /// model rather than replacing it. STL/OBJ use RANSAC reconstruction.
    Import,
    /// Export the model TO the user's filesystem in a chosen format (STEP /
    /// IGES / STL / OBJ / JSON).
    Export,
    /// Export the sheet-metal FLAT PATTERN (unfold) as a 2D vector file
    /// (DXF / SVG). Opens the flat-pattern export modal.
    ExportFlatPattern,
    /// Insert an ASSEMBLY COMPONENT: opens the component selector — existing
    /// parts-library entries first, then the model store's Open list (+ Upload)
    /// — and routes the chosen document through the engine's insert flow
    /// (`add_part_to_library` → an ACOMP instance referencing the returned
    /// effective part name). Dispatched when the palette picks `ACOMP`.
    InsertComponent,
    /// Import a part from KiCad into the open part: a `.kicad_sym` symbol (the
    /// Symbol workbench) or a `.kicad_mod` footprint (the Pads workbench), with
    /// the footprint and 3D model KiCad's chain leads to
    /// ([`crate::panels::kicad_import`]).
    ImportKicad(KicadKind),
}

/// Which modal (if any) the dialog is currently showing.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// Open a saved model through the common explorer.
    Open,
    /// Prompt for a name and save under it.
    SaveAs,
    /// Confirm discarding unsaved changes before CLOSING a document tab —
    /// the ONE place unsaved work can still be lost now that New and Open
    /// both add a tab. `pending_close` holds the tab index.
    ConfirmClose,
    /// Choose an export format (STEP / IGES / STL / OBJ / JSON) for the current model.
    Export,
    /// Choose a STEP / IGES / STL / OBJ file through the common explorer.
    Import,
    /// Choose a 2D vector format (DXF / SVG) for the sheet-metal flat pattern.
    FlatPattern,
    /// Pick a part to insert as an assembly component (library entries + the
    /// stored-model list + Upload).
    InsertComponent,
    /// A `.step` upload whose product structure the probe found: choose whether
    /// to keep that structure (parts + component instances) or flatten it to
    /// bodies. Backed by [`FileDialog::pending_step_import`].
    StepAssembly,
    /// A KiCad import: its file through the common explorer, then its own
    /// stages ([`KicadImport`]).
    Kicad,
    /// Insert component chose a FAMILY: pick one of its members, which is
    /// what gets placed ([`FileDialog::pick_member`]).
    PickMember,
    /// Insert component chose a TEMPLATE: name the new part and set its
    /// inputs; the spun-out part is what gets placed ([`FileDialog::spin_out`]).
    SpinOut,
    /// The active document is a family MEMBER and the user just changed it:
    /// save it as a new part, open the family, or undo ([`FileDialog::member_prompt`]).
    MemberEdit,
    /// The file chooser: ANY file on this machine, for a panel that asked
    /// ([`FileDialog::request_pick_file`]). Browses the local files even in a
    /// PLM session, whose own explorer shows the server's documents.
    PickFile,
}

/// The file-chooser tag of the Open modal's workspace (a PLM session's Open).
const PICK_OPEN_WORKSPACE: &str = "file:open-workspace";

/// A file the chooser delivered ([`FileDialog::take_picked_file`]).
#[derive(Clone, Debug, PartialEq)]
pub struct PickedFile {
    /// The file's own name, extension included (`photo.jpg`).
    pub name: String,
    /// Where it was on this machine; `None` in the browser, which never says.
    pub path: Option<String>,
    pub bytes: Vec<u8>,
}

/// What an explorer folder is FOR. The explorer's location lives in the
/// store and every modal shares it, so each purpose remembers where the user
/// last was and the modal opens there. With one shared folder, a KiCad import
/// from `…/kicad/symbols` sent the next Save As and Insert component into the
/// KiCad library (ecad-reaudit-2026-09-23, B8).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Purpose {
    /// The user's own documents: Open, Save As, Insert component.
    Models,
    /// Foreign CAD and mesh files: Import.
    Imports,
    /// KiCad symbol and footprint libraries.
    Kicad,
}

impl Purpose {
    const ALL: [Purpose; 3] = [Purpose::Models, Purpose::Imports, Purpose::Kicad];

    fn slug(self) -> &'static str {
        match self {
            Purpose::Models => "models",
            Purpose::Imports => "imports",
            Purpose::Kicad => "kicad",
        }
    }
}

impl Mode {
    /// The folder this modal browses, for the modals that browse one.
    fn purpose(self) -> Option<Purpose> {
        match self {
            Mode::Open | Mode::SaveAs | Mode::InsertComponent => Some(Purpose::Models),
            Mode::Import => Some(Purpose::Imports),
            Mode::Kicad => Some(Purpose::Kicad),
            Mode::ConfirmClose
            | Mode::Export
            | Mode::FlatPattern
            | Mode::StepAssembly
            | Mode::PickMember
            | Mode::SpinOut
            | Mode::MemberEdit
            | Mode::PickFile => None,
        }
    }
}

/// The store a purpose's explorer browses. Import and the KiCad import pick
/// a file from THIS MACHINE: in a PLM session the session store is the server's documents, so
/// Import browses the local files the session keeps beside it
/// ([`ModelStore::local_files`]); on a file store they are the same store.
fn browse_store(purpose: Purpose, store: &dyn ModelStore) -> &dyn ModelStore {
    match purpose {
        Purpose::Imports | Purpose::Kicad => store.local_files().unwrap_or(store),
        Purpose::Models => store,
    }
}

/// The store's models folder, where a purpose not yet used starts, and where
/// the Export dialog's files go on desktop. `None` where the store has no
/// such place (the browser), which leaves the location alone.
fn models_folder(store: &dyn ModelStore) -> Option<String> {
    store.browser_places().into_iter().find(|place| place.kind == PlaceKind::Models).map(|place| place.location)
}

/// A probed `.step` upload waiting on the user's choice — the state that makes
/// the §3.9 modal work across frames (egui draws every frame; the click can
/// land many frames after the upload).
///
/// The PARSED assembly itself is NOT here: it lives in the engine's stash
/// (`EngineState::probe_step_assembly` put it there), and the import consumes
/// that stash. This holds only what the prompt says and the `text` the flat
/// lane needs — the two lanes that re-import from source rather than from the
/// parse ("Import as bodies", and the structured lane's own failure fallback).
struct PendingStepImport {
    /// The uploaded file's name — the prompt's subject and the status line's.
    name: String,
    /// The file text. The engine's stash holds the PARSE, not the source, so
    /// the flat lane's input has to be kept here.
    text: String,
    /// The counts the prompt shows, from the same walk the import runs.
    probe: StepAssemblyProbe,
    /// The §3.9 checkbox: flatten the sub-assembly tree to leaf occurrences
    /// instead of building nested rigid sub-assembly documents. Only shown (and
    /// only meaningful) when `probe.nested_depth > 1`; a depth-1 file imports
    /// identically either way.
    flatten: bool,
}

/// A `.step` upload whose structure probe is RUNNING on the document's
/// background runner (native thread / browser worker — the parse builds every
/// product's bodies and takes seconds on a real assembly, so it left the UI
/// thread). Resolved by [`FileDialog::poll_step_probe`] into either the §3.9
/// choice ([`PendingStepImport`]) or the flat lane.
struct PendingStepProbe {
    /// The engine's probe id, so a stale answer (a superseded upload) is ignored.
    id: u64,
    name: String,
    /// The file text, kept for the flat lane the outcome may route to.
    text: String,
}

/// What the user clicked in the §3.9 assembly-choice modal.
#[derive(Clone, Copy, PartialEq, Eq)]
enum StepChoice {
    /// Keep the structure: parts-library entries + one component per occurrence.
    Assembly,
    /// Today's flat lane, unchanged: one IMPORT3D feature carrying the text.
    Bodies,
    /// Import nothing, and drop the parse (Esc / click-outside land here too).
    Cancel,
}

/// What Insert component lists, and so the only files it inserts: parts, and
/// the families and templates that each lead to a part.
const INSERT_EXTENSIONS: &[&str] = MODEL_EXTENSIONS;

/// One row of the member picker: a family row and where its member is.
struct MemberChoice {
    part_number: String,
    description: String,
    /// The member's store identity; `None` when the part number cannot be a
    /// file name.
    identity: Option<String>,
    /// Whether that file exists (Generate has written it).
    exists: bool,
    /// On a PLM store: why the member cannot be placed (not generated, waiting
    /// for its bake, failed it), as the server's family view says.
    why_not: Option<String>,
}

/// An armed member picker: the family Insert component chose.
struct PickMember {
    /// The family's file name, for the heading.
    family_file: String,
    rows: Vec<MemberChoice>,
    /// On a PLM store the members are parts: the list comes from the server
    /// (`loading` until it has answered), and a chosen member's document is
    /// read from it.
    plm: bool,
    loading: bool,
}

/// An armed template spin-out: the template Insert component chose, and the
/// user's answers so far.
struct SpinOut {
    template_file: String,
    template: serde_json::Value,
    inputs: Vec<TemplateInput>,
    /// The new part's name (no extension).
    name: String,
    /// Input name -> the value text the user typed or chose.
    values: BTreeMap<String, String>,
    /// Where the new part goes: the folder of the assembly being edited, or
    /// the models folder when that assembly has never been saved.
    assembly: Option<String>,
    /// On a PLM store: the template part the SERVER spins out, and the new
    /// part's number (empty lets a counter part type number it).
    plm_template: Option<String>,
    number: String,
}

/// PLM work the dialog waits on (plm-cad-integration-todo S9), polled each
/// frame so the web build works the same.
enum PlmWork {
    /// The member picker's list.
    Members(crate::panels::plm_family::Pending<Vec<crate::plm::family::Member>>),
    /// A chosen member's (or a spun-out part's) document, to place: the key,
    /// the text, and the part number it is named by.
    Place(crate::panels::plm_family::Pending<(String, String, String)>),
    /// Open the family a member came from: its newest revision's key.
    OpenFamily(crate::panels::plm_family::Pending<String>),
}

/// The hand-edit prompt's subject: which tab, and the family it came from.
struct MemberPrompt {
    doc_id: u64,
    source: FamilySource,
    /// The family file's identity, beside the member.
    family_identity: String,
    /// The prompt came from plain Save rather than from an edit.
    from_save: bool,
}

/// Make the engine's document a `class` document: the class field, and none
/// of the other classes' blocks (a template saved as a part stops asking for
/// inputs; a family saved as a template drops its table). No undo step: this
/// is what the new FILE is, written as it is saved.
fn adopt_class(engine: &mut EngineState, class: DocumentClass) {
    document_class::set_document_class(engine, class);
    let drop: &[&str] = match class {
        DocumentClass::Normal => &[family_table::FAMILY_TABLE_KEY, template::TEMPLATE_INPUTS_KEY],
        DocumentClass::Family => &[template::TEMPLATE_INPUTS_KEY],
        DocumentClass::Template => &[family_table::FAMILY_TABLE_KEY],
    };
    for key in drop {
        if engine.history.document_block(key).is_some() {
            engine.history.set_document_block_no_undo(key, None);
        }
    }
}

/// Where a spun-out part is written: `<name>.nbrep` in the assembly's folder,
/// or the bare file name (the models folder) when the assembly has none.
fn spin_out_target(spin: &SpinOut) -> String {
    let file = DocumentClass::Normal.file_name(spin.name.trim());
    match &spin.assembly {
        Some(assembly) => sibling_identity(assembly, &file),
        None => file,
    }
}

/// `"s"` unless there is exactly one — the difference between "1 parts" and a
/// sentence a user believes.
fn plural(count: usize) -> &'static str {
    if count == 1 {
        ""
    } else {
        "s"
    }
}

/// The document name an imported assembly's unnamed products are stemmed from:
/// the file's base name without its STEP extension, so a nameless product reads
/// `bracket-assy-part-7`, not `bracket-assy.step-part-7`.
fn step_document_name(file_name: &str) -> String {
    let base = file_name.rsplit(['/', '\\']).next().unwrap_or(file_name);
    let cut = base
        .rfind('.')
        .filter(|dot| is_step_name(&base[*dot..]))
        .unwrap_or(base.len());
    let stem = &base[..cut];
    if stem.is_empty() {
        base.to_string()
    } else {
        stem.to_string()
    }
}

/// The §3.9 outcome line: `imported bracket-assy.step — 7 parts, 23 components
/// (2 mirrored instances baked)`, plus the tail an imperfect import owes the
/// user. Every qualifier is reported ONLY when it happened, so a clean import
/// reads clean — but a partial one never reads as a whole one:
///
/// * `baked_nonrigid` — occurrences whose mirror/scale was baked into a part of
///   their own (§3.4), which is why the part count can exceed the file's;
/// * `failed_products` — products that did not encode, skipped and counted;
/// * `flat_fallback` — the user asked for an assembly and got bodies. Said
///   plainly, never silently (the dialog lane reports this through the Err
///   branch instead, which is the only way it can reach the user from here);
/// * `first_error` — the one thing that explains the rest.
/// The STEP-assembly import's [`PartSink`]: writes each unique part document to
/// the model store and hands back the identity it was stored under, so an
/// imported part carries a REAL `sourceKey` and there is no second kind of part.
///
/// # The destination
///
/// `browser_write` at the explorer's CURRENT location, under
/// `{assembly}-{part}` — the convention `panels::step_parts` already uses for a
/// single STEP part (that panel lets the user pick the folder first; the
/// assembly modal inherits wherever the explorer is pointing).
///
/// The kernel plan's alternative — a `{assembly}/{part}.nbrep` SUB-FOLDER —
/// is not reachable through this door: every `browser_write` implementation
/// flattens the name to a single file (native takes `file_name()`, web takes
/// `model_display_name`), so a sub-path would silently collapse. Writing the
/// assembly name into the FILE name keeps a 50-part import grouped in one
/// listing without a folder convention this seam cannot express. Creating and
/// navigating into a folder as a side effect of an import is the prompt this
/// lane would have to grow, and it is not bolted on here.
///
/// A failed write is per part: that part stays embedded-only (`None`) and the
/// import continues. Losing one part's FILE is recoverable; losing the import
/// is not.
///
/// # Known limit of a flat name
///
/// `taken` is per IMPORT, so re-importing the same file reuses the same names —
/// which is what makes cross-import dedup work (same key, same signature, the
/// resident entry is reused). But two DIFFERENT assemblies whose document names
/// sanitize to the same stem (`as1-ug` and `as1_ug`) write to each other's file
/// names. The second import wins the file; the first assembly's entry then
/// disagrees with it, so it badges outdated and the write-through guard refuses
/// to clobber it — visible and recoverable (the embedded document is intact),
/// but two assemblies sharing one name. The fix is the per-assembly sub-folder
/// `browser_write` cannot express; see the kernel plan's §3.5.
struct StorePartSink<'a> {
    store: &'a dyn ModelStore,
    /// Prefixes every file name, so one import's parts sort together.
    prefix: String,
    /// File names already claimed this import — two STEP products can sanitize
    /// to the same name, and the second must not overwrite the first's file
    /// (that would leave two entries pointing at one document).
    taken: std::collections::HashSet<String>,
    written: usize,
    failures: Vec<String>,
}

impl<'a> StorePartSink<'a> {
    fn new(store: &'a dyn ModelStore, prefix: &str) -> Self {
        Self {
            store,
            prefix: sanitize_file_stem(prefix),
            taken: std::collections::HashSet::new(),
            written: 0,
            failures: Vec::new(),
        }
    }
}

impl PartSink for StorePartSink<'_> {
    fn store_part(&mut self, part_name: &str, document_json: &str) -> Option<String> {
        let base = format!("{}-{}", self.prefix, sanitize_file_stem(part_name));
        let mut name = base.clone();
        let mut suffix = 2;
        while !self.taken.insert(name.clone()) {
            name = format!("{base}-{suffix}");
            suffix += 1;
        }
        match self.store.browser_write(&name, document_json) {
            Ok(identity) => {
                self.written += 1;
                Some(identity)
            }
            Err(error) => {
                self.failures.push(format!("{name}: {error}"));
                None
            }
        }
    }
}

/// A STEP product name reduced to something every store backend can hold as a
/// file name: ASCII word characters, `.`, `-` kept; everything else (spaces,
/// slashes, the `(mirrored)` parentheses this crate appends) becomes `_`. Runs
/// of `_` collapse and the ends are trimmed, so a name is readable rather than
/// a row of underscores. Empty input yields `part`.
fn sanitize_file_stem(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    for ch in name.chars() {
        if ch.is_ascii_alphanumeric() || ch == '.' || ch == '-' {
            out.push(ch);
        } else if !out.ends_with('_') {
            out.push('_');
        }
    }
    let trimmed = out.trim_matches('_');
    if trimmed.is_empty() {
        "part".to_string()
    } else {
        trimmed.to_string()
    }
}

fn assembly_import_message(name: &str, report: &StepAssemblyReport) -> String {
    let mut message = format!(
        "imported {name} \u{2014} {} part{}, {} component{}",
        report.parts,
        plural(report.parts),
        report.instances,
        plural(report.instances)
    );
    if report.baked_nonrigid > 0 {
        message.push_str(&format!(
            " ({} mirrored instance{} baked)",
            report.baked_nonrigid,
            plural(report.baked_nonrigid)
        ));
    }
    if report.failed_products > 0 {
        message.push_str(&format!(
            "; {} product{} could not be built",
            report.failed_products,
            plural(report.failed_products)
        ));
    }
    if report.flat_fallback {
        message.push_str("; the structure was NOT used \u{2014} the bodies came in flat");
    }
    if let Some(error) = &report.first_error {
        message.push_str(&format!(" [{error}]"));
    }
    message
}

/// Whether an imported file name is a STEP file (`.step` / `.stp`, any case) —
/// the routing key in [`FileDialog::show`]. Model documents arrive
/// extension-stripped (web) or as a stored model name, so this
/// never mis-routes a `.nbrep` file.
fn is_step_name(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    lower.ends_with(".step") || lower.ends_with(".stp")
}

/// Whether an imported file name is an IGES file (`.iges` / `.igs`, any case) —
/// the sibling routing key of [`is_step_name`].
fn is_iges_name(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    lower.ends_with(".iges") || lower.ends_with(".igs")
}

fn is_stl_name(name: &str) -> bool {
    name.to_ascii_lowercase().ends_with(".stl")
}

fn is_obj_name(name: &str) -> bool {
    name.to_ascii_lowercase().ends_with(".obj")
}

/// Whether an imported file name is a 3MF package. It takes the STL lane: the
/// same reconstruction preview, the same tolerances, the same accept.
fn is_3mf_name(name: &str) -> bool {
    name.to_ascii_lowercase().ends_with(".3mf")
}

/// The reusable file dialog — transient UI buffers + the current document
/// identity; the model itself lives in `EngineState`'s history.
pub struct FileDialog {
    /// Reusable browser body shared by Open, Save As, and ACOMP selection.
    explorer: FileExplorer,
    /// In a PLM session the Open modal is the workspace browser instead (S14, D12). It
    /// lives here so the folder the user was in is where Open starts next time.
    workspace: crate::panels::plm_workspace::WorkspaceBrowser,
    /// The name field buffer, used by the Save As modal.
    name_buf: String,
    /// Last-action status line, surfaced inside the modal.
    pub(crate) status: String,
    /// Whether a modal is currently open.
    open: bool,
    /// The open modal's mode (only meaningful while `open`).
    mode: Mode,
    /// Set on open so the Save As text input grabs focus on the next frame.
    want_focus: bool,
    /// A pending real-file pick belongs to the INSERT-COMPONENT flow (the
    /// modal's Upload fired there): the next completed `take_import` routes to
    /// the component insert instead of Open's load-document.
    pending_component_import: bool,
    /// The tab index a pending [`Mode::ConfirmClose`] will close on "Discard".
    pending_close: Option<usize>,
    /// A probed STEP assembly waiting on the §3.9 choice modal. `Some` means the
    /// ENGINE is holding a parsed assembly for us (with every product's solids
    /// resident), so every exit from that modal must end its life: import it,
    /// discard it, or be superseded by the next upload.
    pending_step_import: Option<PendingStepImport>,
    /// A `.step` upload whose probe has not answered yet (see
    /// [`PendingStepProbe`]); the status line says "reading…" meanwhile.
    pending_step_probe: Option<PendingStepProbe>,
    /// A mesh file waiting to open its reconstruction preview, with the reader
    /// the picker routed it to (STL or 3MF).
    pending_stl_import: Option<(MeshImportFormat, String, Vec<u8>)>,
    /// Bumped on every SUCCESSFUL store save (plain Save / Save As / native
    /// Save-As) — one half of the update-components staleness key (saving a
    /// part's source must re-check the outdated badges without a reload).
    save_generation: u64,
    /// Per-frame widget hit-rects for the headed verifier (wasm only).
    hits: HashMap<String, egui::Rect>,
    /// The KiCad import's stages after its file.
    kicad: KicadImport,
    /// The last folder of each [`Purpose`], taken from the store's explorer
    /// location every frame its modal is open.
    folders: HashMap<Purpose, String>,
    /// The file chooser: which panel asked (its tag) and the modal's title,
    /// the folder it was last in (on the LOCAL files), and what it delivered.
    pick_request: Option<(String, String)>,
    pick_folder: Option<String>,
    picked: Option<(String, PickedFile)>,
    /// Save As → "A new part" in a PLM session: the shell opens the PLM pane's New part
    /// form (S7), which writes this document into the new part's first revision.
    plm_new_part: bool,
    /// Save As → "A new revision of this part": the write in flight.
    plm_new_revision: Option<crate::panels::plm_parts::Pending<String>>,
    /// The class Save As writes (its class chooser).
    save_class: DocumentClass,
    /// The armed member picker ([`Mode::PickMember`]).
    pick_member: Option<PickMember>,
    plm_work: Option<PlmWork>,
    /// The armed template spin-out ([`Mode::SpinOut`]).
    spin_out: Option<SpinOut>,
    /// The armed hand-edit prompt ([`Mode::MemberEdit`]).
    member_prompt: Option<MemberPrompt>,
    /// Tabs whose user chose "Save as a new part": they may keep editing
    /// without being asked again; plain Save still asks.
    member_ack: HashSet<u64>,
    /// The last Generate's report, for the automation blob.
    last_generate: Option<family_table::GenerateReport>,
}

impl FileDialog {
    /// A closed dialog with empty buffers. Document identity (name + clean
    /// baseline) lives on [`Document`], so there is nothing to seed here.
    pub fn new() -> Self {
        Self {
            explorer: FileExplorer::new(),
            workspace: crate::panels::plm_workspace::WorkspaceBrowser::new(),
            name_buf: String::new(),
            status: String::new(),
            open: false,
            mode: Mode::Open,
            want_focus: false,
            pending_component_import: false,
            pending_close: None,
            pending_step_import: None,
            pending_step_probe: None,
            pending_stl_import: None,
            save_generation: 0,
            hits: HashMap::new(),
            kicad: KicadImport::new(),
            folders: HashMap::new(),
            pick_request: None,
            pick_folder: None,
            picked: None,
            plm_new_part: false,
            plm_new_revision: None,
            save_class: DocumentClass::Normal,
            pick_member: None,
            plm_work: None,
            spin_out: None,
            member_prompt: None,
            member_ack: HashSet::new(),
            last_generate: None,
        }
    }

    /// The dialog's own work in flight, for the status bar's working
    /// indicator: a KiCad part's 3D model being parsed off the UI thread.
    pub fn busy_activities(&self) -> Vec<crate::panels::busy::Activity> {
        let mut out = Vec::new();
        if self.kicad.reading() {
            out.push(crate::panels::busy::Activity::new("kicadRead", "kicadRead", "Reading the KiCad part's 3D model"));
        }
        out
    }

    /// The monotonic successful-save counter — the shell feeds it to the
    /// update-components checker as half its staleness key.
    pub fn save_generation(&self) -> u64 {
        self.save_generation
    }

    // --- dispatch: a toolbar button was clicked -------------------------------

    /// Act on a toolbar file button. New acts immediately (it adds a tab, so
    /// there is nothing to discard); all browsing flows use the same in-app
    /// modal on both platforms.
    pub fn dispatch(&mut self, action: FileAction, docs: &mut Documents, store: &dyn ModelStore) {
        match action {
            FileAction::New => self.new_document(docs, DocumentClass::Normal),
            FileAction::NewOfClass(class) => self.new_document(docs, class),
            FileAction::Open => {
                self.use_folder(Purpose::Models, store);
                self.open_modal(Mode::Open);
            }
            FileAction::Save => match docs.active().name().map(str::to_string) {
                // A family member is not saved over by hand: the prompt says
                // why and offers the two ways that are.
                Some(name) if self.arm_member_prompt(docs, &name, true) => {}
                // A named document saves straight to its name.
                Some(name) => {
                    let _ = self.save_to(docs, store, name);
                }
                // An unnamed document falls through to Save As.
                None => self.dispatch(FileAction::SaveAs, docs, store),
            },
            FileAction::SaveAs => {
                self.name_buf = document_class::strip_class_extension(&Self::document_name(docs)).to_string();
                self.save_class = document_class::document_class(docs.engine());
                self.use_folder(Purpose::Models, store);
                self.open_modal(Mode::SaveAs);
            }
            FileAction::Import => {
                self.use_folder(Purpose::Imports, browse_store(Purpose::Imports, store));
                self.open_modal(Mode::Import);
            }
            FileAction::Export => {
                self.name_buf = Self::document_name(docs);
                self.open_modal(Mode::Export);
            }
            FileAction::ExportFlatPattern => {
                self.name_buf = Self::document_name(docs);
                self.open_modal(Mode::FlatPattern);
            }
            // ALWAYS the in-app modal (never the native picker directly): the
            // "existing library entries first" list is an in-app concept; the
            // modal's own Upload button drives the platform picker when the
            // store supports interchange.
            FileAction::InsertComponent => {
                self.use_folder(Purpose::Models, store);
                self.open_modal(Mode::InsertComponent);
            }
            FileAction::ImportKicad(kind) => {
                self.kicad.start(kind, store);
                self.use_folder(Purpose::Kicad, browse_store(Purpose::Kicad, store));
                self.open_modal(Mode::Kicad);
            }
        }
    }

    /// Ask for a file (plm-cad-integration-todo S8 / S14's "Add file…"). `tag`
    /// names the asker, which takes the file with [`Self::take_picked_file`];
    /// `title` heads the chooser. Natively the chooser is this dialog's own
    /// explorer over the machine's files (one behaviour on every platform, and
    /// drivable by scripts); in the browser it is the page's file input.
    /// `Err` where neither exists.
    pub fn request_pick_file(&mut self, store: &dyn ModelStore, tag: &str, title: &str) -> Result<(), String> {
        self.picked = None;
        if let Some(local) = store.local_files() {
            let arrived = self.pick_folder.as_deref().is_some_and(|folder| local.browser_navigate(folder).is_ok());
            if !arrived {
                let _ = local.browser_home();
            }
            self.pick_request = Some((tag.to_string(), title.to_string()));
            self.status.clear();
            self.open_modal(Mode::PickFile);
            return Ok(());
        }
        store.begin_pick_file()?;
        self.pick_request = Some((tag.to_string(), title.to_string()));
        Ok(())
    }

    /// The file the chooser delivered for `tag`, once.
    pub fn take_picked_file(&mut self, tag: &str) -> Option<PickedFile> {
        if self.picked.as_ref().is_some_and(|(asker, _)| asker == tag) {
            return self.picked.take().map(|(_, file)| file);
        }
        None
    }

    /// Whether a pick for `tag` is still open (the chooser is up, or the
    /// browser's is and has not answered).
    pub fn picking(&self, tag: &str) -> bool {
        self.pick_request.as_ref().is_some_and(|(asker, _)| asker == tag)
    }

    /// The chooser: the shared explorer over the LOCAL files, every file shown.
    fn show_pick_file(&mut self, ctx: &egui::Context, store: &dyn ModelStore) {
        let Some(local) = store.local_files() else {
            self.open = false;
            self.pick_request = None;
            return;
        };
        let title = self.pick_request.as_ref().map(|(_, title)| title.clone()).unwrap_or_else(|| "Choose a file".into());
        let modal = egui::Modal::new(egui::Id::new("brep-file-pick-file")).show(ctx, |ui| {
            file_explorer::dialog_body(ui, |ui| {
                ui.heading(&title);
                ui.add_space(4.0);
                file_explorer::dialog_footer(ui, "pickfile", |ui| {
                    if !self.status.is_empty() {
                        ui.add_space(4.0);
                        ui.weak(&self.status);
                    }
                });
                let options = FileExplorerOptions {
                    hit_prefix: "pickfile",
                    empty_label: "(this folder is empty)",
                    row_icon: "\u{1F5CE}",
                    current: None,
                    allow_delete: false,
                    allow_import: false,
                    import_label: "",
                    import_hit: "pickfile:upload",
                    show_cancel: true,
                    confirm_label: Some("Choose"),
                    extensions: &[],
                };
                let output = self.explorer.show_store(ui, local, options);
                self.record_explorer_hits(&output.hits);
                output
            })
        });
        let should_close = modal.should_close();
        let output = modal.inner;
        self.pick_folder = Some(local.browser_location());
        if let Some(path) = output.activated {
            match local.read_external_file(&path) {
                Some(bytes) => {
                    let name = std::path::Path::new(&path)
                        .file_name()
                        .map(|n| n.to_string_lossy().into_owned())
                        .unwrap_or_else(|| path.clone());
                    if let Some((tag, _)) = self.pick_request.take() {
                        self.status = format!("chose {name}");
                        self.picked = Some((tag, PickedFile { name, path: Some(path), bytes }));
                    }
                    self.open = false;
                }
                None => self.status = format!("'{path}' could not be read"),
            }
        } else if output.cancel || should_close {
            self.pick_request = None;
            self.status.clear();
            self.open = false;
        }
    }

    /// Move the explorer to where `purpose` was last, or to the models folder
    /// the first time. A remembered folder that is gone falls back to the
    /// models folder too.
    fn use_folder(&mut self, purpose: Purpose, store: &dyn ModelStore) {
        let remembered = self.folders.get(&purpose).cloned();
        let arrived = remembered.is_some_and(|folder| store.browser_navigate(&folder).is_ok());
        if !arrived {
            if let Some(models) = models_folder(store) {
                let _ = store.browser_navigate(&models);
            }
        }
    }

    /// Where the open modal's explorer is is where its purpose was last.
    fn remember_folder(&mut self, store: &dyn ModelStore) {
        if let Some(purpose) = self.mode.purpose() {
            self.folders.insert(purpose, browse_store(purpose, store).browser_location());
        }
    }

    /// Open the modal in `mode` (and focus its input next frame).
    fn open_modal(&mut self, mode: Mode) {
        // Every opening is a new choice: the explorer is shared by every mode,
        // so a highlight left by the last one (Import's `Timer.kicad_sym`) would
        // otherwise be this one's confirm target (Insert component's).
        self.explorer.clear_selection();
        self.mode = mode;
        self.open = true;
        self.want_focus = true;
    }

    // --- per-frame draw -------------------------------------------------------

    /// Draw the modal (if open) and pick up any completed async import. Called
    /// every frame by the shell with a ctx-level handle (the modal is ctx-level,
    /// like the command palette).
    pub fn show(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        self.hits.clear();
        self.poll_plm_work(ctx, docs, store);
        self.poll_open_workspace_pick(store);
        // The browser's file chooser answered a pick.
        if let Some(file) = store.take_picked_file() {
            if let Some((tag, _)) = self.pick_request.take() {
                self.picked = Some((tag, PickedFile { name: file.name, path: None, bytes: file.bytes }));
            }
        }

        // A STEP probe the runner has answered since last frame resolves into
        // the assembly choice or the flat import now, before drawing.
        self.poll_step_probe(docs.engine_mut());

        // A completed async browser upload is picked up
        // here, before drawing, so the model is live for this frame. Routed by
        // extension: a STEP file (`.step`/`.stp`) is APPENDED to the model as an
        // IMPORT3D feature; anything else is a `.nbrep` model document loaded
        // (replacing the model). The model lanes deliver an extension-less name
        // (web, stripped by `model_display_name`) or a stored model name, so only
        // real STEP files route here.
        if let Some(imported) = store.take_import() {
            // A new delivery SUPERSEDES an armed assembly choice. The modal
            // describes a parse this routing is about to replace (a STEP probe
            // drops the previous stash by contract, and a document load drops it
            // outright), so leaving the choice armed would offer the user a
            // button whose stash is already gone.
            self.cancel_pending_step_import(docs.engine_mut());
            let name = imported.name;
            let bytes = imported.bytes;
            // CAD/mesh imports target the active document. STL waits for
            // preview acceptance; model documents open in a separate tab.
            if KicadKind::of_file(&name).is_some() {
                self.take_kicad_file(&name, &bytes, store);
                return;
            } else if is_step_name(&name) {
                self.import_text(docs.engine_mut(), &name, &bytes, Self::import_step);
            } else if is_iges_name(&name) {
                self.import_text(docs.engine_mut(), &name, &bytes, Self::import_iges);
            } else if is_stl_name(&name) {
                self.stage_mesh_preview(MeshImportFormat::Stl, name, bytes);
            } else if is_3mf_name(&name) {
                self.stage_mesh_preview(MeshImportFormat::ThreeMf, name, bytes);
            } else if is_obj_name(&name) {
                self.import_obj(docs.engine_mut(), &name, &bytes);
            } else if std::mem::take(&mut self.pending_component_import) {
                // The insert-component modal's Upload fired this pick: the
                // chosen document becomes a parts-library entry + an instance,
                // NOT a new tab — or, for a family or a template, the prompt
                // that leads to one.
                match String::from_utf8(bytes) {
                    Ok(contents) => {
                        let assembly = docs.active().name().map(str::to_string);
                        if !self.begin_class_insert(store, &name, &contents, assembly) {
                            self.insert_component_document(docs.engine_mut(), &name, &contents)
                        }
                        if self.pick_member.is_some() || self.spin_out.is_some() {
                            return;
                        }
                    }
                    Err(_) => self.status = format!("open failed: {name} is not UTF-8 text"),
                }
            } else {
                match String::from_utf8(bytes) {
                    Ok(contents) => self.load_document(docs, &name, &contents, store.plm_client().is_some()),
                    Err(_) => self.status = format!("open failed: {name} is not UTF-8 text"),
                }
            }
            self.close_after_import();
        }

        // A family member the user has just started to change raises the
        // hand-edit prompt (unless they already chose "Save as a new part").
        if !self.open {
            self.watch_member_edit(docs);
        }

        if !self.open {
            return;
        }
        self.remember_folder(store);

        match self.mode {
            Mode::ConfirmClose => self.show_confirm_close(ctx, docs),
            Mode::SaveAs => self.show_save_as(ctx, docs, store),
            Mode::Open => self.show_open(ctx, docs, store),
            Mode::Import => self.show_import(ctx, docs.engine_mut(), store),
            Mode::Export => self.show_export(ctx, docs, store),
            Mode::FlatPattern => self.show_flat_pattern(ctx, docs, store),
            Mode::InsertComponent => self.show_insert_component(ctx, docs, store),
            Mode::PickMember => self.show_pick_member(ctx, docs.engine_mut(), store),
            Mode::SpinOut => self.show_spin_out(ctx, docs.engine_mut(), store),
            Mode::MemberEdit => self.show_member_edit(ctx, docs, store),
            Mode::PickFile => self.show_pick_file(ctx, store),
            Mode::StepAssembly => self.show_step_assembly(ctx, docs.engine_mut(), store),
            Mode::Kicad => self.show_kicad(ctx, docs.engine_mut(), store),
        }
    }

    /// Close the modal after a completed import — unless the import ARMED the
    /// §3.9 assembly choice, in which case the modal stays open and switches to
    /// it. Both import routing sites (the async-upload poll and the Import
    /// modal's own pick) end here, so neither can close a dialog the probe just
    /// raised.
    fn close_after_import(&mut self) {
        if self.pending_step_import.is_some() {
            self.open_modal(Mode::StepAssembly);
        } else {
            self.open = false;
        }
    }

    /// Abandon an armed assembly choice and the engine stash behind it. The ONE
    /// place the app-side pending state and the engine-side parse are dropped
    /// together — they are two halves of one thing, and a half-drop is either a
    /// dialog with no stash or solids nobody will ever consume.
    fn cancel_pending_step_import(&mut self, state: &mut EngineState) {
        if self.pending_step_import.take().is_some() {
            state.discard_probed_step_assembly();
        }
    }

    /// Common CAD/mesh browser. Desktop enumerates files in the application's
    /// models directory; web shows the same explorer shell with an Upload action.
    fn show_import(
        &mut self,
        ctx: &egui::Context,
        state: &mut EngineState,
        store: &dyn ModelStore,
    ) {
        const EXTENSIONS: &[&str] =
            &["step", "stp", "iges", "igs", "stl", "3mf", "obj", "kicad_sym", "kicad_mod"];
        let modal = egui::Modal::new(egui::Id::new("brep-file-import")).show(ctx, |ui| {
            file_explorer::dialog_body(ui, |ui| {
                ui.heading("Import CAD file");
                ui.add_space(4.0);
                file_explorer::dialog_footer(ui, "import", |ui| {
                    if !self.status.is_empty() {
                        ui.add_space(4.0);
                        ui.weak(&self.status);
                    }
                });
                let options = FileExplorerOptions {
                    hit_prefix: "import",
                    empty_label: "(no STEP, IGES, STL, 3MF, OBJ, or KiCad files)",
                    row_icon: "\u{1F5CE}",
                    current: None,
                    allow_delete: false,
                    allow_import: store.supports_file_interchange(),
                    import_label: "Upload\u{2026}",
                    import_hit: "import:upload",
                    show_cancel: true,
                    confirm_label: Some("Import"),
                    extensions: EXTENSIONS,
                };
                let output = self.explorer.show_store(ui, browse_store(Purpose::Imports, store), options);
                self.record_explorer_hits(&output.hits);
                output
            })
        });
        let should_close = modal.should_close();
        let output = modal.inner;
        if let Some(name) = output.activated {
            match browse_store(Purpose::Imports, store).read_external_file(&name) {
                Some(bytes) if KicadKind::of_file(&name).is_some() => {
                    // Not an import yet: the KiCad dialog takes it from here.
                    self.take_kicad_file(&name, &bytes, store);
                    return;
                }
                Some(bytes) if is_step_name(&name) => {
                    self.import_text(state, &name, &bytes, Self::import_step)
                }
                Some(bytes) if is_iges_name(&name) => {
                    self.import_text(state, &name, &bytes, Self::import_iges)
                }
                Some(bytes) if is_stl_name(&name) => {
                    self.stage_mesh_preview(MeshImportFormat::Stl, name, bytes)
                }
                Some(bytes) if is_3mf_name(&name) => {
                    self.stage_mesh_preview(MeshImportFormat::ThreeMf, name, bytes)
                }
                Some(bytes) if is_obj_name(&name) => self.import_obj(state, &name, &bytes),
                Some(_) => self.status = format!("unsupported import file: {name}"),
                None => self.status = format!("import failed: '{name}' not found"),
            }
            self.close_after_import();
        } else if output.import {
            match store.begin_import_filtered(("CAD / mesh", EXTENSIONS)) {
                Ok(()) => {
                    self.status =
                        "choose a STEP, IGES, STL, 3MF, OBJ, or KiCad file\u{2026}".into();
                    self.open = false;
                }
                Err(e) => self.status = format!("import failed: {e}"),
            }
        } else if output.cancel || should_close {
            self.open = false;
        }
    }

    /// A `.kicad_sym` or `.kicad_mod` arrived (picked in Import, in the KiCad
    /// explorer, or uploaded): the KiCad dialog reads it and goes on to its
    /// next stage. `name` is the file's full path on desktop, which is where a
    /// footprint's relative model path starts from.
    fn take_kicad_file(&mut self, name: &str, bytes: &[u8], store: &dyn ModelStore) {
        if !(self.open && self.mode == Mode::Kicad) {
            if let Some(kind) = KicadKind::of_file(name) {
                self.kicad.start(kind, store);
            }
        }
        self.kicad.take_file(name, bytes, &DiskFiles);
        self.open_modal(Mode::Kicad);
    }

    /// The KiCad import: the common explorer for its file, then its own
    /// stages. Hit keys are `kicad:*`.
    fn show_kicad(&mut self, ctx: &egui::Context, state: &mut EngineState, store: &dyn ModelStore) {
        if !self.kicad.wants_file() {
            match self.kicad.show(ctx, state, store, &mut self.hits) {
                kicad_import::Outcome::Open => {}
                kicad_import::Outcome::Closed => self.open = false,
                kicad_import::Outcome::Imported(imported) => {
                    // The modal closes on the click, so the toasts are what the
                    // user sees: the summary, then each note.
                    self.status = imported.summary.clone();
                    state.push_notice_as(brep_render::engine_state::NoticeSeverity::Info, imported.summary);
                    const SHOWN: usize = 5;
                    let hidden = imported.notes.len().saturating_sub(SHOWN);
                    for note in imported.notes.into_iter().take(SHOWN) {
                        state.push_notice_as(brep_render::engine_state::NoticeSeverity::Warning, note);
                    }
                    if hidden > 0 {
                        state.push_notice_as(
                            brep_render::engine_state::NoticeSeverity::Warning,
                            format!("and {hidden} more notes on this import"),
                        );
                    }
                    self.open = false;
                }
            }
            return;
        }
        let extension = self.kicad.kind().extension();
        let heading = match self.kicad.kind() {
            KicadKind::Symbol => "Import a symbol from KiCad",
            KicadKind::Footprint => "Import a footprint from KiCad",
        };
        let extensions = [extension];
        let modal = egui::Modal::new(egui::Id::new("brep-file-kicad")).show(ctx, |ui| {
            file_explorer::dialog_body(ui, |ui| {
                ui.heading(heading);
                ui.add_space(4.0);
                file_explorer::dialog_footer(ui, "kicad", |ui| {
                    let status = if self.kicad.status.is_empty() { &self.status } else { &self.kicad.status };
                    if !status.is_empty() {
                        ui.add_space(4.0);
                        ui.weak(status);
                    }
                });
                let empty_label = format!("(no .{extension} files)");
                let options = FileExplorerOptions {
                    hit_prefix: "kicad",
                    empty_label: &empty_label,
                    row_icon: "\u{1F5CE}",
                    current: None,
                    allow_delete: false,
                    allow_import: store.supports_file_interchange(),
                    import_label: "Upload\u{2026}",
                    import_hit: "kicad:upload",
                    show_cancel: true,
                    confirm_label: Some("Open"),
                    extensions: &extensions,
                };
                let output = self.explorer.show_store(ui, browse_store(Purpose::Kicad, store), options);
                self.record_explorer_hits(&output.hits);
                output
            })
        });
        let should_close = modal.should_close();
        let output = modal.inner;
        if let Some(name) = output.activated {
            match browse_store(Purpose::Kicad, store).read_external_file(&name) {
                Some(bytes) => self.take_kicad_file(&name, &bytes, store),
                None => self.kicad.status = format!("import failed: '{name}' not found"),
            }
        } else if output.import {
            match store.begin_import_filtered(("KiCad", &extensions)) {
                Ok(()) => {
                    self.status = format!("choose a .{extension} file\u{2026}");
                    self.open = false;
                }
                Err(e) => self.kicad.status = format!("import failed: {e}"),
            }
        } else if output.cancel || should_close {
            self.open = false;
        }
    }

    /// CLOSE tab `index` with the unsaved-changes contract: a DIRTY document
    /// prompts to discard first (the confirm modal); a clean one closes
    /// immediately. The tab strip's `\u{2715}` routes here — it is the only
    /// door through which unsaved work can be dropped, so it is the only one
    /// that asks.
    pub fn request_close(&mut self, docs: &mut Documents, index: usize) {
        let dirty = docs.get(index).is_some_and(Document::is_dirty);
        if dirty {
            self.pending_close = Some(index);
            self.open_modal(Mode::ConfirmClose);
        } else {
            docs.close(index);
        }
    }

    /// The **Discard unsaved changes?** confirmation shown before closing a
    /// dirty document tab. Keeps the verifier's `confirm:discard` /
    /// `confirm:cancel` hit keys — the same two buttons, one door further on.
    fn show_confirm_close(&mut self, ctx: &egui::Context, docs: &mut Documents) {
        // A tab closed / reordered under an open prompt (there is no such path
        // today, but the index is only meaningful while it resolves).
        let Some(title) = self
            .pending_close
            .and_then(|index| docs.get(index))
            .map(Document::title)
        else {
            self.pending_close = None;
            self.open = false;
            return;
        };
        let mut discard = false;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-confirm-close")).show(ctx, |ui| {
            ui.set_width(340.0);
            ui.heading("Discard unsaved changes?");
            ui.add_space(4.0);
            ui.label(format!("\"{title}\" has unsaved changes. Close it?"));
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                let d = ui.button("Discard and close");
                self.hit("confirm:discard", &d);
                if d.clicked() {
                    discard = true;
                }
                let c = ui.button("Cancel");
                self.hit("confirm:cancel", &c);
                if c.clicked() {
                    cancel = true;
                }
            });
        });
        if discard {
            if let Some(index) = self.pending_close.take() {
                docs.close(index);
            }
            self.open = false;
        } else if cancel || modal.should_close() {
            self.pending_close = None;
            self.open = false;
        }
    }

    /// The §3.9 **assembly choice**, raised when a `.step` upload's probe found a
    /// product structure:
    ///
    /// > **"bracket-assy.step" contains an assembly** — 7 parts, 23 instances.
    /// > [ Import as assembly ] [ Import as bodies ] [ Cancel ]
    ///
    /// Default (and first) is **Import as assembly**. "Import as bodies" is
    /// today's flat lane, unchanged. Cancel — and Esc / click-outside, which
    /// egui folds into `should_close` — import nothing and DROP the parse: the
    /// engine is holding every product's solids until one of these three lands.
    ///
    /// **Flatten sub-assemblies** (§3.9) is offered only when the file actually
    /// HAS sub-assemblies (`nested_depth > 1`) — on a single-level file both
    /// lanes produce the identical document, so a box there would teach the user
    /// a distinction that does not exist. Unchecked (the default) keeps the
    /// tree; checked flattens to leaf occurrences, which is the right answer for
    /// a deep or pathological file and stores a part reused across two levels
    /// once rather than once per level.
    fn show_step_assembly(
        &mut self,
        ctx: &egui::Context,
        state: &mut EngineState,
        store: &dyn ModelStore,
    ) {
        // Nothing armed means nothing to choose about (a supersession raced the
        // draw): close rather than render an empty prompt.
        let Some(pending) = self.pending_step_import.as_ref() else {
            self.open = false;
            return;
        };
        let name = pending.name.clone();
        let probe = pending.probe;
        let mut flatten = pending.flatten;
        let mut choice: Option<StepChoice> = None;
        let modal = egui::Modal::new(egui::Id::new("brep-file-step-assembly")).show(ctx, |ui| {
            ui.set_width(360.0);
            ui.heading(format!("\"{name}\" contains an assembly"));
            ui.add_space(4.0);
            ui.label(format!(
                "{} part{}, {} instance{}{}.",
                probe.parts,
                plural(probe.parts),
                probe.instances,
                plural(probe.instances),
                match probe.nested_depth {
                    0 | 1 => String::new(),
                    depth => format!(", {depth} levels deep"),
                }
            ));
            if probe.nested_depth > 1 {
                ui.add_space(4.0);
                let check = ui.checkbox(&mut flatten, "Flatten sub-assemblies");
                self.hit("stepassembly:flatten", &check);
                check.on_hover_text(
                    "Off: each sub-assembly becomes one rigid component you can \
                     expand in the structure tree.\nOn: every part is placed \
                     directly in this document at its world position.",
                );
            }
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                let a = ui.button("Import as assembly");
                self.hit("stepassembly:assembly", &a);
                if a.clicked() {
                    choice = Some(StepChoice::Assembly);
                }
                let b = ui.button("Import as bodies");
                self.hit("stepassembly:bodies", &b);
                if b.clicked() {
                    choice = Some(StepChoice::Bodies);
                }
                let c = ui.button("Cancel");
                self.hit("stepassembly:cancel", &c);
                if c.clicked() {
                    choice = Some(StepChoice::Cancel);
                }
            });
        });
        // The box must survive the frames between the tick and the click.
        if let Some(pending) = self.pending_step_import.as_mut() {
            pending.flatten = flatten;
        }
        // Esc / click-outside is a Cancel, not a no-op: the stash must not
        // outlive the prompt that was going to consume it.
        let choice = choice.or_else(|| modal.should_close().then_some(StepChoice::Cancel));
        let Some(choice) = choice else { return };
        match choice {
            StepChoice::Assembly => self.step_assembly_import(state, store),
            StepChoice::Bodies => self.step_assembly_bodies(state),
            StepChoice::Cancel => self.step_assembly_cancel(state),
        }
        self.open = false;
    }

    /// Save As asked for a new PLM part: the shell opens the PLM pane's New part form.
    /// Once: true the frame after it was chosen.
    pub fn take_plm_new_part(&mut self) -> bool {
        std::mem::take(&mut self.plm_new_part)
    }

    /// **Save As** in a PLM session (D9): there are no file names on the PLM, so the
    /// question is a new part, or a new revision of this part (S7's [`SaveAsChooser`]).
    /// A new revision holds this document as it is now, current edits included, and
    /// opens checked out to this user.
    ///
    /// [`SaveAsChooser`]: crate::panels::plm_parts::SaveAsChooser
    fn show_save_as_plm(
        &mut self,
        ctx: &egui::Context,
        docs: &mut Documents,
        store: &dyn ModelStore,
        client: std::rc::Rc<crate::plm::client::PlmClient>,
    ) {
        use crate::panels::plm_parts::{save_as_new_revision, Pending, SaveAsChooser, SaveAsDecision};
        let identity = docs
            .active()
            .name()
            .and_then(crate::plm::uses::revision_key_in)
            .and_then(|key| crate::store::DocumentIdentity::parse_revision_key(&key));
        let choices = SaveAsChooser::choices(identity.as_ref());
        let saving = self.plm_new_revision.is_some();
        let mut chosen = None;
        let mut cancel = false;
        let mut choice_hits = HashMap::new();
        let modal = egui::Modal::new(egui::Id::new("brep-file-saveas")).show(ctx, |ui| {
            ui.heading("Save as");
            ui.add_space(4.0);
            if saving {
                ui.label("Saving a new revision…");
            } else {
                chosen = SaveAsChooser::show(ui, &choices, &mut choice_hits);
            }
            if !self.status.is_empty() {
                ui.weak(&self.status);
            }
            let button = ui.add_enabled(!saving, egui::Button::new("Cancel"));
            self.hits.insert("saveas:cancel".into(), button.rect);
            cancel = button.clicked();
        });
        // S7's keys are `plm_parts:saveas:…`: in this dialog they are the file panel's
        // `saveas:` family.
        for (key, rect) in choice_hits {
            self.hits.insert(key.strip_prefix("plm_parts:").unwrap_or(&key).to_string(), rect);
        }
        match chosen {
            Some(SaveAsDecision::NewPart) => {
                self.plm_new_part = true;
                self.status = "fill in the new part in the PLM pane".into();
                self.open = false;
            }
            Some(SaveAsDecision::NewRevision { part }) => {
                let document = docs.engine().history_request_json();
                let future: crate::plm::PlmFuture<String> =
                    Box::pin(async move { save_as_new_revision(&client, &part, "", &document).await });
                self.status.clear();
                self.plm_new_revision = Some(Pending::new(future));
            }
            None => {}
        }
        if let Some(pending) = self.plm_new_revision.as_mut() {
            let waker = {
                struct Repaint(egui::Context);
                impl std::task::Wake for Repaint {
                    fn wake(self: std::sync::Arc<Self>) {
                        self.0.request_repaint();
                    }
                }
                std::task::Waker::from(std::sync::Arc::new(Repaint(ctx.clone())))
            };
            if let Some(answer) = pending.poll(&waker) {
                self.plm_new_revision = None;
                match answer {
                    Ok(key) => {
                        let name = store.canonical_identity(&key);
                        self.open_document(docs, store, &name);
                        self.status = format!("saved as a new revision: {name}");
                        self.open = false;
                    }
                    Err(problem) => self.status = problem,
                }
            } else {
                ctx.request_repaint();
            }
        }
        if (cancel || modal.should_close()) && self.plm_new_revision.is_none() {
            self.open = false;
        }
    }

    /// The **Save As** name prompt.
    fn show_save_as(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        // A PLM session saves as a new part or a new revision, not under a file name.
        if let Some(client) = store.plm_client() {
            self.show_save_as_plm(ctx, docs, store, client);
            return;
        }
        let current = docs.active().name().map(str::to_string);
        let mut do_save = false;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-saveas")).show(ctx, |ui| {
            file_explorer::dialog_body(ui, |ui| {
                ui.heading("Save model as");
                ui.add_space(4.0);
                // The name field and its buttons are the dialog's footer: pinned
                // under the browser, which fills whatever is left.
                file_explorer::dialog_footer(ui, "saveas", |ui| {
                    ui.label("File name");
                    let field = ui.add(
                        egui::TextEdit::singleline(&mut self.name_buf)
                            .hint_text("model name")
                            .desired_width(f32::INFINITY),
                    );
                    self.hit("field:name", &field);
                    if self.want_focus {
                        field.request_focus();
                        self.want_focus = false;
                    }
                    let enter =
                        field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    // The class the file is saved as. A part's class is fixed
                    // for its life, so Save As is the one place it is chosen:
                    // saving a part as a Family or a Template makes a new file
                    // of that class.
                    ui.horizontal(|ui| {
                        ui.label("Save as");
                        for class in DocumentClass::ALL {
                            let text = format!("{} ({})", class.label(), class.extension());
                            let chip = ui.selectable_label(self.save_class == class, text);
                            self.hit(&format!("saveas:class:{}", class.slug()), &chip);
                            if chip.clicked() {
                                self.save_class = class;
                            }
                        }
                    });
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        let save = ui.button("Save");
                        self.hit("save", &save);
                        if save.clicked() || enter {
                            do_save = true;
                        }
                        let c = ui.button("Cancel");
                        self.hit("cancel", &c);
                        if c.clicked() {
                            cancel = true;
                        }
                    });
                    if !self.status.is_empty() {
                        ui.add_space(4.0);
                        ui.weak(&self.status);
                    }
                });
                let options = FileExplorerOptions {
                    hit_prefix: "saveas:file",
                    empty_label: "(no saved models)",
                    row_icon: "\u{1F5CE}",
                    current: current.as_deref(),
                    allow_delete: false,
                    allow_import: false,
                    import_label: "",
                    import_hit: "saveas:upload",
                    show_cancel: false,
                    confirm_label: None,
                    extensions: MODEL_EXTENSIONS,
                };
                let output = self.explorer.show_store(ui, store, options);
                self.record_explorer_hits(&output.hits);
                // Selecting (or double-clicking) a stored model fills the name
                // field so it can be overwritten; `picked`/`activated` are
                // one-shot so it never clobbers a name the user then types. The
                // field itself was drawn above, so the explorer asks for the
                // repaint that shows the new name.
                if let Some(name) = output.picked.or(output.activated) {
                    self.name_buf =
                        document_class::strip_class_extension(&model_display_name(&name)).to_string();
                    if let Some(class) = DocumentClass::of_name(&name) {
                        self.save_class = class;
                    }
                }
            })
        });
        if do_save {
            let name = self.save_class.file_name(self.name_buf.trim());
            if self.save_to_browser(docs, store, name) {
                self.open = false;
            }
        } else if cancel || modal.should_close() {
            self.open = false;
        }
    }

    /// The **Export** chooser: pick a format (STEP / IGES / STL / OBJ / GLB) and
    /// write the current model to the user's filesystem under `<name>.<ext>`
    /// through the store's format-typed interchange. STEP and IGES serialize the
    /// exact NURBS topology; STL, OBJ and GLB are the display mesh (ASCII
    /// facets, one indexed object, and binary glTF 2.0 with a mesh per body).
    /// No solids → a clear status line,
    /// nothing written. A second row offers the BOM (CSV / JSON), enabled only
    /// while the document has components or harness wires.
    fn show_export(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        // BOM gating: `component_ids` scans the history (cheap, no kernel
        // session), and a harness wire is a BOM line of its own, so a
        // harness-only document exports too. A rolled-back/failed ACOMP can
        // still enable the buttons — the export's own loud "no components or
        // harness wires" error covers that gap.
        let has_bom = !docs.engine().component_ids().is_empty()
            || !docs.engine().wire_harness_state().connections.is_empty();
        // The drawing sheets of THIS document: the sheet button writes the open
        // one (else the first), so it is offered only when there is one to
        // write — the BOM row's gating, for the same reason.
        let has_sheets = !docs.engine().sheet_state().sheets.is_empty();
        // The board's manufacturing files: offered once the PCB has parts on
        // its board, the BOM row's gating for the same reason.
        let has_board = super::fabrication_export::has_board(docs.engine());
        let has_schematic = super::fabrication_export::has_schematic(docs.engine());
        let mut chosen: Option<&'static str> = None;
        let mut bom_chosen: Option<&'static str> = None;
        let mut fab_chosen: Option<super::fabrication_export::Output> = None;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-export")).show(ctx, |ui| {
            ui.set_width(320.0);
            ui.heading("Export model");
            ui.add_space(4.0);
            Self::preview_export_line(ui, docs);
            let field = ui.add(
                egui::TextEdit::singleline(&mut self.name_buf)
                    .hint_text("file name")
                    .desired_width(f32::INFINITY),
            );
            self.hit("field:name", &field);
            if self.want_focus {
                field.request_focus();
                self.want_focus = false;
            }
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                let step = ui.button("STEP (.step)");
                self.hit("export:step", &step);
                if step.clicked() {
                    chosen = Some("step");
                }
                let iges = ui.button("IGES (.igs)");
                self.hit("export:iges", &iges);
                if iges.clicked() {
                    chosen = Some("iges");
                }
                let stl = ui.button("STL (.stl)");
                self.hit("export:stl", &stl);
                if stl.clicked() {
                    chosen = Some("stl");
                }
                let obj = ui.button("OBJ (.obj)");
                self.hit("export:obj", &obj);
                if obj.clicked() {
                    chosen = Some("obj");
                }
                let glb = ui.button("GLB (.glb)");
                self.hit("export:glb", &glb);
                if glb.clicked() {
                    chosen = Some("glb");
                }
                // The full model RECIPE (`.nbrep`) — for saving a document to
                // disk / sharing a failing model for a bug report. Re-openable via
                // Open / Import.
                let json = ui.button("Model (.nbrep)");
                self.hit("export:json", &json);
                if json.clicked() {
                    chosen = Some("json");
                }
                let c = ui.button("Cancel");
                self.hit("cancel", &c);
                if c.clicked() {
                    cancel = true;
                }
            });
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                // The BOM: one row per parts-library entry with the live
                // instance count, then one per harness wire with its cut
                // length. Disabled while the document has neither.
                let csv = ui.add_enabled(has_bom, egui::Button::new("BOM (CSV)"));
                self.hit("export:bomcsv", &csv);
                if csv.clicked() {
                    bom_chosen = Some("csv");
                }
                let json = ui.add_enabled(has_bom, egui::Button::new("BOM (JSON)"));
                self.hit("export:bomjson", &json);
                if json.clicked() {
                    bom_chosen = Some("json");
                }
                // The DRAWING SHEET: the paper as an SVG of lines and text.
                // Not the sheet-METAL flat pattern, which is its own dialog.
                let sheet = ui
                    .add_enabled(has_sheets, egui::Button::new("Sheet (SVG)"))
                    .on_disabled_hover_text("Add a drawing sheet in the Drawing workbench first");
                self.hit("export:sheetsvg", &sheet);
                if sheet.clicked() {
                    chosen = Some("sheetsvg");
                }
                // EVERY sheet as one PDF: a page per sheet, in sheet order, each
                // at its own paper size — the drawing set, not the page on
                // screen. The SVG beside it stays one file per sheet.
                let pdf = ui
                    .add_enabled(has_sheets, egui::Button::new("Sheets (PDF)"))
                    .on_hover_text("Every drawing sheet of the document as one PDF, a page per sheet")
                    .on_disabled_hover_text("Add a drawing sheet in the Drawing workbench first");
                self.hit("export:sheetpdf", &pdf);
                if pdf.clicked() {
                    chosen = Some("sheetpdf");
                }
            });
            ui.horizontal(|ui| {
                // The same drawing set with the LIVE 3D model: a 3D box over
                // every placement marked "Live 3D in PDF", and a last page
                // given over to the model with a button per saved PMI view.
                // Its own file, so the plain drawing set is never replaced.
                let pdf3d = ui
                    .add_enabled(has_sheets, egui::Button::new("Sheets + 3D (PDF)"))
                    .on_hover_text("The drawing set with the live 3D model: Acrobat and Foxit turn and zoom it and switch PMI views; other PDF readers show the drawings")
                    .on_disabled_hover_text("Add a drawing sheet in the Drawing workbench first");
                self.hit("export:sheetpdf3d", &pdf3d);
                if pdf3d.clicked() {
                    chosen = Some("sheetpdf3d");
                }
            });
            ui.horizontal(|ui| {
                // The PCB's manufacturing files. The zip is what a board house
                // takes; the two CSVs are also inside it, and on their own for
                // an assembler that asks for them separately.
                use super::fabrication_export::Output;
                let no_board = "Place the schematic's parts on the PCB board first";
                for (key, label, hover, output) in [
                    ("export:fabrication", "Fabrication (zip)", "Gerber X2 layers, Excellon drills, pick and place, the parts BOM and a README of the layers, in one zip for the board house", Output::Bundle),
                    ("export:pickplace", "Pick & place (CSV)", "One row per placed part: Designator, Val, Package, Mid X, Mid Y, Rotation, Layer (JLCPCB's columns), in mm from the board's lower-left corner", Output::PickPlace),
                    ("export:ecadbom", "Parts BOM (CSV)", "The electronic parts: Comment (the value), Designator, Footprint, Quantity (JLCPCB's columns), one line per value and footprint", Output::Bom),
                    ("export:kicadnet", "Netlist (KiCad)", "The PCB schematic's nets as a KiCad .net file, the netlist Pcbnew and other tools read: every part with its value and footprint, every net with its pins, under the names the Connectivity panel lists", Output::Netlist),
                ] {
                    let (enabled, why) = match output {
                        Output::Netlist => (has_schematic, "Put parts on the PCB schematic first"),
                        _ => (has_board, no_board),
                    };
                    let button = ui
                        .add_enabled(enabled, egui::Button::new(label))
                        .on_hover_text(hover)
                        .on_disabled_hover_text(why);
                    self.hit(key, &button);
                    if button.clicked() {
                        fab_chosen = Some(output);
                    }
                }
            });
            if !self.status.is_empty() {
                ui.add_space(4.0);
                ui.weak(&self.status);
            }
        });
        if let Some(format) = chosen {
            if self.export_as(docs, store, format) {
                // The `.nbrep` recipe is the document; every other format is
                // of the geometry, which a family-row preview has replaced.
                if format != "json" {
                    self.say_previewed_export(docs);
                }
                self.open = false;
            }
        } else if let Some(format) = bom_chosen {
            if self.export_bom_as(docs, store, format) {
                self.open = false;
            }
        } else if let Some(output) = fab_chosen {
            if self.export_fabrication_as(docs, store, output) {
                self.open = false;
            }
        } else if cancel || modal.should_close() {
            self.open = false;
        }
    }

    /// The **Flat pattern** chooser: pick a 2D vector format (DXF R12 / SVG) and
    /// write the sheet-metal body's unfolded flat pattern to the user's filesystem
    /// under `<name>.<ext>`. The unfold runs TRANSIENTLY in the engine (no feature,
    /// no history change). A part with no sheet-metal body reports it in the status
    /// line AND queues a toast (see [`Self::export_flat_pattern_as`]).
    fn show_flat_pattern(
        &mut self,
        ctx: &egui::Context,
        docs: &mut Documents,
        store: &dyn ModelStore,
    ) {
        let mut chosen: Option<&'static str> = None;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-flatpattern")).show(ctx, |ui| {
            ui.set_width(340.0);
            ui.heading("Export flat pattern");
            ui.add_space(2.0);
            Self::preview_export_line(ui, docs);
            ui.weak("Unfolds the sheet-metal body to a 2D vector file.");
            ui.add_space(4.0);
            let field = ui.add(
                egui::TextEdit::singleline(&mut self.name_buf)
                    .hint_text("file name")
                    .desired_width(f32::INFINITY),
            );
            self.hit("field:name", &field);
            if self.want_focus {
                field.request_focus();
                self.want_focus = false;
            }
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                let dxf = ui.button("DXF (.dxf)");
                self.hit("flat:dxf", &dxf);
                if dxf.clicked() {
                    chosen = Some("dxf");
                }
                let svg = ui.button("SVG (.svg)");
                self.hit("flat:svg", &svg);
                if svg.clicked() {
                    chosen = Some("svg");
                }
                let c = ui.button("Cancel");
                self.hit("cancel", &c);
                if c.clicked() {
                    cancel = true;
                }
            });
            if !self.status.is_empty() {
                ui.add_space(4.0);
                ui.weak(&self.status);
            }
        });
        if let Some(format) = chosen {
            if self.export_flat_pattern_as(docs, store, format) {
                self.say_previewed_export(docs);
                self.open = false;
            }
        } else if cancel || modal.should_close() {
            self.open = false;
        }
    }

    /// The **Open** browser: the saved-model list (+ Upload where the platform
    /// supports real-file interchange).
    fn show_open(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        // A PLM session opens from the user's workspace, not from a list of store keys.
        if let Some(client) = store.plm_client() {
            self.show_open_workspace(ctx, docs, store, client);
            return;
        }
        let current = docs.active().name().map(str::to_string);
        let modal = egui::Modal::new(egui::Id::new("brep-file-open")).show(ctx, |ui| {
            file_explorer::dialog_body(ui, |ui| {
                ui.heading("Open model");
                ui.add_space(4.0);
                file_explorer::dialog_footer(ui, "open", |ui| {
                    if !self.status.is_empty() {
                        ui.add_space(4.0);
                        ui.weak(&self.status);
                    }
                });
                let mut options = FileExplorerOptions::open(current.as_deref());
                options.allow_import = store.supports_file_interchange();
                let output = self.explorer.show_store(ui, store, options);
                self.record_explorer_hits(&output.hits);
                output
            })
        });
        let should_close = modal.should_close();
        let output = modal.inner;
        if let Some(name) = output.activated {
            self.open_document(docs, store, &name);
            self.open = false;
        } else if let Some(name) = output.remove {
            let _ = store.remove(&name);
            self.status = format!("removed {name}");
            // A tab holding the removed document keeps its content but loses its
            // file: it becomes an untitled document, so a later Save asks where
            // to put it rather than silently recreating what was deleted.
            if let Some(index) = docs.index_of(&name) {
                if let Some(doc) = docs.get_mut(index) {
                    doc.set_name(None);
                }
            }
        } else if output.import {
            // Fire the platform picker; the file arrives via take_import() and is
            // loaded on a later frame (the modal closes now).
            match store.begin_import() {
                Ok(()) => {
                    self.status = "choose a file…".into();
                    self.open = false;
                }
                Err(e) => self.status = format!("import failed: {e}"),
            }
        } else if output.cancel || should_close {
            self.open = false;
        }
    }

    /// The **Open** browser in a PLM session: the workspace (S14) — folders of links and
    /// files, other users' workspaces read-only, and Open by number. A link opens its
    /// part's document; a file dropped on the modal is added to the folder shown; a
    /// downloaded file goes out through the store's export lane.
    fn show_open_workspace(
        &mut self,
        ctx: &egui::Context,
        docs: &mut Documents,
        store: &dyn ModelStore,
        client: std::rc::Rc<crate::plm::client::PlmClient>,
    ) {
        use crate::panels::plm_workspace::{document_name, dropped, load_pins, save_pins, PlmWorkspaces};
        let plm = PlmWorkspaces { client };
        if self.workspace.pins.is_none() {
            self.workspace.pins = Some(load_pins(store));
        }
        for (name, bytes) in dropped(ctx) {
            self.workspace.add_file(&plm, &name, bytes);
        }
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-open")).show(ctx, |ui| {
            file_explorer::dialog_body(ui, |ui| {
                ui.heading("Open from the PLM");
                ui.add_space(4.0);
                file_explorer::dialog_footer(ui, "open", |ui| {
                    if !self.status.is_empty() {
                        ui.add_space(4.0);
                        ui.weak(&self.status);
                    }
                    let button = ui.button("Cancel");
                    self.hits.insert("open:cancel".into(), button.rect);
                    cancel = button.clicked();
                });
                self.workspace.show(ui, &plm, "open:")
            })
        });
        let hits: Vec<(String, egui::Rect)> = self.workspace.hits.iter().map(|(k, r)| (k.clone(), *r)).collect();
        self.record_explorer_hits(&hits);
        let should_close = modal.should_close();
        let outcome = modal.inner;
        if let Some(pins) = &outcome.pins {
            if let Err(e) = save_pins(store, pins) {
                self.status = format!("could not save the pinned folders: {e}");
            }
        }
        if let Some((name, bytes)) = outcome.downloaded {
            self.status = match store.export_file_named_bytes(&name, &bytes) {
                Ok(()) => format!("saved {name}"),
                Err(e) => format!("could not save {name}: {e}"),
            };
        }
        if outcome.pick_file {
            // The chooser takes the modal over; the file comes back to
            // `poll_open_workspace_pick`, which reopens Open on the folder.
            if let Err(error) = self.request_pick_file(store, PICK_OPEN_WORKSPACE, "Add a file to this folder") {
                self.status = error;
            }
            return;
        }
        if let Some(key) = outcome.open {
            let name = document_name(store, &key);
            self.open_document(docs, store, &name);
            self.open = false;
        } else if cancel || should_close {
            self.open = false;
        }
    }

    /// A file picked for the Open modal's workspace: added to the folder shown,
    /// and the Open modal comes back.
    fn poll_open_workspace_pick(&mut self, store: &dyn ModelStore) {
        let Some(file) = self.take_picked_file(PICK_OPEN_WORKSPACE) else { return };
        if let Some(client) = store.plm_client() {
            let plm = crate::panels::plm_workspace::PlmWorkspaces { client };
            self.workspace.add_file(&plm, &file.name, file.bytes);
            self.open_modal(Mode::Open);
        }
    }

    /// The **Insert component** selector: the EXISTING parts-library entries
    /// first (instant re-insert — no store read), then the model store's
    /// document list, + Upload where the platform supports real-file
    /// interchange. REUSES the Open modal's machinery (the same list +
    /// `take_import` poll), routed to the engine's insert flow.
    fn show_insert_component(
        &mut self,
        ctx: &egui::Context,
        docs: &mut Documents,
        store: &dyn ModelStore,
    ) {
        let assembly = docs.active().name().map(str::to_string);
        let state = docs.engine_mut();
        let library = state.parts_library_names();
        let modal = egui::Modal::new(egui::Id::new("brep-file-insert-component")).show(ctx, |ui| {
            file_explorer::dialog_body(ui, |ui| {
                ui.heading("Insert component");
                ui.add_space(4.0);
                file_explorer::dialog_footer(ui, "insert", |ui| {
                    if !self.status.is_empty() {
                        ui.add_space(4.0);
                        ui.weak(&self.status);
                    }
                });
                let mut chosen_library = None;
                if !library.is_empty() {
                    ui.weak("In this document (parts library)");
                    for name in &library {
                        let row = ui.add_sized(
                            egui::vec2(ui.available_width(), 18.0),
                            crate::icon_text::icon_button(ui, &format!("\u{25A3} {name}"))
                                .frame(false),
                        );
                        self.hit(&format!("insert:lib:{name}"), &row);
                        if row.clicked() {
                            chosen_library = Some(name.clone());
                        }
                    }
                    ui.add_space(4.0);
                }
                let options = FileExplorerOptions {
                    hit_prefix: "insert:model",
                    empty_label: "(no saved models)",
                    row_icon: "\u{1F5CE}",
                    current: None,
                    allow_delete: false,
                    allow_import: store.supports_file_interchange(),
                    import_label: "Upload\u{2026}",
                    import_hit: "insert:upload",
                    show_cancel: true,
                    confirm_label: Some("Insert"),
                    extensions: INSERT_EXTENSIONS,
                };
                let output = self.explorer.show_store(ui, store, options);
                self.record_explorer_hits(&output.hits);
                (chosen_library, output)
            })
        });
        let should_close = modal.should_close();
        let (chosen_library, output) = modal.inner;
        if let Some(part_name) = chosen_library {
            // An already-inserted library part: skip the store read entirely.
            match state.insert_component(ComponentInsert::Existing { part_name: &part_name }) {
                Ok(id) => {
                    self.status = format!("inserted {part_name} ({id})");
                    self.open = false;
                }
                Err(e) => self.status = format!("insert failed: {e}"),
            }
        } else if let Some(name) = output.activated.filter(|name| {
            // Only a file the list OFFERS is a component: a part document. A
            // highlight left by another purpose (Import's `.kicad_sym`) is not
            // one, and reached the parts library as a non-document.
            let listed = store
                .browser_entries(INSERT_EXTENSIONS)
                .iter()
                .any(|entry| !entry.is_dir && entry.identity == *name);
            if !listed {
                self.status = format!(
                    "insert refused: {name} is not a part, family or template document (.nbrep / .fbrep / .tbrep)"
                );
            }
            listed
        }) {
            match store.read(&name) {
                // A family or a template is never placed itself: it opens
                // the prompt that leads to the part that is.
                Some(contents) if self.begin_class_insert(store, &name, &contents, assembly) => {}
                Some(contents) => {
                    self.insert_component_document(state, &name, &contents);
                    self.open = false;
                }
                None => self.status = format!("insert failed: '{name}' not found"),
            }
        } else if output.import {
            match store.begin_import() {
                Ok(()) => {
                    // The picked file routes to the component insert (not Open).
                    self.pending_component_import = true;
                    self.status = "choose a part file…".into();
                    self.open = false;
                }
                Err(e) => self.status = format!("insert failed: {e}"),
            }
        } else if output.cancel || should_close {
            self.open = false;
        }
    }

    /// Insert a part DOCUMENT as an assembly component: library-add (dedup by
    /// sourceKey + content signature; the RETURNED effective name is what the
    /// instance references) + an ACOMP feature, through the engine's one insert
    /// flow. The first instance of an empty assembly is written `isFixed:true`.
    /// `sourceSignature` is written with [`document_signature`] — the ONE
    /// signature fn — so the update-components comparison reads a freshly
    /// inserted, unchanged part as up-to-date.
    fn insert_component_document(&mut self, state: &mut EngineState, name: &str, contents: &str) {
        self.insert_component_named(state, name, &model_display_name(name), contents);
    }

    /// [`Self::insert_component_document`] under a given name: a PLM member
    /// is named by its part number, which its key does not carry.
    fn insert_component_named(&mut self, state: &mut EngineState, name: &str, display: &str, contents: &str) {
        let display = display.to_string();
        match state.insert_component(ComponentInsert::New {
            name: &display,
            source_key: name,
            source_signature: &document_signature(contents),
            document_json: contents,
        }) {
            Ok(id) => self.status = format!("inserted {display} ({id})"),
            Err(e) => self.status = format!("insert failed: {e}"),
        }
    }

    // --- model operations -----------------------------------------------------

    /// The name to save under: the field if non-empty, else the active
    /// document's display name, else `"untitled"`.
    fn effective_name(&self, docs: &Documents) -> String {
        let field = self.name_buf.trim();
        if !field.is_empty() {
            field.to_string()
        } else {
            Self::document_name(docs)
        }
    }

    /// The ACTIVE document's display name, what Save As and Export start
    /// their name field from: the file's stem, never the full path a saved
    /// document is named by (whose `/`s became `_`s in every exported file
    /// name), and never the field's last contents, which belong to whichever
    /// tab last used it.
    fn document_name(docs: &Documents) -> String {
        docs.active().name().map(model_display_name).unwrap_or_else(|| "untitled".into())
    }

    /// **New** — an empty model in a NEW TAB. Nothing is replaced, so there is
    /// nothing to confirm.
    fn new_document(&mut self, docs: &mut Documents, class: DocumentClass) {
        let mut engine = docs.spawn_engine();
        let _ = engine.set_history_json(EMPTY_DOCUMENT);
        // Before `Document::new`, so the class is part of the clean baseline.
        document_class::set_document_class(&mut engine, class);
        docs.open_document(Document::new(engine));
        self.name_buf.clear();
        self.status = match class {
            DocumentClass::Normal => "new (empty) model".into(),
            _ => format!("new (empty) {}", class.label().to_ascii_lowercase()),
        };
    }

    /// Refuse a save that would give TWO open tabs the same store identity —
    /// "focus the tab holding this document" has no answer then, and the second
    /// save would silently overwrite the first tab's file. `true` = go ahead.
    ///
    /// Compared by DISPLAY name, not raw identity: on desktop an open document
    /// carries the full path it was loaded from while the Save As field holds a
    /// bare name, so an identity compare would never match and the clobber would
    /// happen before anything noticed. Two same-stemmed files in different
    /// folders are refused too — stricter than strictly necessary, and the side
    /// to err on when the alternative is overwriting another tab's file.
    fn name_is_free(&mut self, docs: &Documents, name: &str) -> bool {
        let display = model_display_name(name);
        let taken = docs.iter().enumerate().any(|(index, doc)| {
            index != docs.active_index()
                && doc
                    .name()
                    .is_some_and(|open| model_display_name(open) == display)
        });
        if taken {
            self.status = format!("'{display}' is already open in another tab");
        }
        !taken
    }

    /// Write the active document's request JSON through the store under `name`.
    /// Returns `true` on success (so the caller can close the modal).
    fn save_to(&mut self, docs: &mut Documents, store: &dyn ModelStore, name: String) -> bool {
        let name = name.trim().to_string();
        if name.is_empty() {
            self.status = "enter a name to save".into();
            return false;
        }
        if !self.name_is_free(docs, &name) {
            return false;
        }
        let document = docs.engine().history_request_json();
        crate::plm::thumbnail::stage_save(store, &name, &document, docs.engine(), docs.active_id());
        match store.write(&name, &document) {
            Ok(()) => {
                // The document keeps the raw identity (a full path when a
                // native dialog chose it); the name field shows the bare name.
                self.name_buf = model_display_name(&name);
                let doc = docs.active_mut();
                doc.set_name(Some(name.clone()));
                doc.mark_clean();
                self.save_generation += 1;
                self.status = format!("saved {name}");
                true
            }
            Err(e) => {
                self.status = format!("save failed: {e}");
                false
            }
        }
    }

    /// Save As through the explorer's current directory, then retain the
    /// backend's returned identity (an absolute path on desktop or a virtual
    /// `/models/...` path in the browser) for subsequent plain Save commands.
    fn save_to_browser(
        &mut self,
        docs: &mut Documents,
        store: &dyn ModelStore,
        name: String,
    ) -> bool {
        let name = name.trim().to_string();
        if name.is_empty() {
            self.status = "enter a name to save".into();
            return false;
        }
        if !self.name_is_free(docs, &name) {
            return false;
        }
        // Save As makes a NEW file: its class is the one the name carries
        // (the chooser put it there), the blocks of the other classes go, and
        // a family member's stamp goes — the copy is a part of its own, which
        // is what "Save as a new part" means.
        let class = DocumentClass::of_name(&name).unwrap_or_default();
        adopt_class(docs.engine_mut(), class);
        if docs.engine().history.document_block(family_table::FAMILY_SOURCE_KEY).is_some() {
            docs.engine_mut()
                .history
                .set_document_block_no_undo(family_table::FAMILY_SOURCE_KEY, None);
        }
        match store.browser_write(&name, &docs.engine().history_request_json()) {
            Ok(identity) => {
                self.name_buf = model_display_name(&identity);
                let doc = docs.active_mut();
                self.member_ack.remove(&doc.id());
                doc.set_name(Some(identity.clone()));
                doc.mark_clean();
                self.save_generation += 1;
                self.status = format!("saved {identity}");
                true
            }
            Err(e) => {
                self.status = format!("save failed: {e}");
                false
            }
        }
    }

    /// **Open** — read a stored document into its own tab (or focus the tab
    /// already holding it). The one door every open lane uses: File>Open, the
    /// Edit-Part flow, and the session restore's siblings.
    pub fn open_document(&mut self, docs: &mut Documents, store: &dyn ModelStore, name: &str) {
        if docs.focus_named(name) {
            self.status = format!("{name} is already open");
            return;
        }
        match crate::store::read_now(store, name) {
            crate::store::ReadNow::Ready(contents) => self.load_document(docs, name, &contents, store.plm_client().is_some()),
            // An index-hydrated PLM store lists what it has not loaded yet;
            // the read above asked for it.
            crate::store::ReadNow::Loading => {
                self.status = format!("'{name}' is still loading — open it again in a moment")
            }
            crate::store::ReadNow::Absent => self.status = format!("open failed: '{name}' not found"),
        }
    }

    /// Serialize the current model in `format` (`"step"` | `"iges"` | `"stl"` |
    /// `"obj"` | `"glb"` | `"json"`) and
    /// write it through the store's format-typed interchange under `<name>.<ext>`.
    /// `"json"` is the full model RECIPE (`.nbrep`, `history_request_json`) —
    /// the exact document Open/Import consume, for sharing a failing model. Returns
    /// `true` on success (so the caller can close the modal); a guard message and
    /// `false` when there is nothing to export or the engine/store errs.
    /// An export of the geometry taken while a family row is previewed is of
    /// THAT row: say so on the status line and in a toast, so a user does not
    /// ship the wrong size without knowing.
    fn say_previewed_export(&mut self, docs: &mut Documents) {
        let Some(label) = docs.engine().expression_preview().map(|p| p.label.clone()) else {
            return;
        };
        let note = format!("exported {label} (preview): the family's own values are not in this file");
        self.status = format!("{} — {label} (preview)", self.status);
        docs.engine_mut().push_notice(note);
    }

    /// The line the export dialogs show while a family row is previewed.
    fn preview_export_line(ui: &mut egui::Ui, docs: &Documents) {
        if let Some(label) = docs.engine().expression_preview().map(|p| p.label.clone()) {
            let text = format!(
                "Previewing {label}: STEP, IGES, meshes, drawings and flat patterns export {label}, \
                 not the family's own values. The .nbrep recipe keeps the family's own."
            );
            ui.add(egui::Label::new(egui::RichText::new(text).color(ui.visuals().warn_fg_color)).wrap());
            ui.add_space(4.0);
        }
    }

    fn export_as(&mut self, docs: &mut Documents, store: &dyn ModelStore, format: &str) -> bool {
        let name = self.effective_name(docs);
        // GLB is BINARY, so it takes the store's byte-shaped interchange rather
        // than the text one below — a `.glb` routed through a `&str` would be
        // mangled the moment a chunk length happened not to be valid UTF-8.
        if format == "glb" {
            // The document's name becomes the root NODE's, as it becomes the
            // root PRODUCT's on a STEP export.
            return match docs.engine().export_glb_bytes(&name) {
                Ok(bytes) => match store.export_file_named_bytes(&format!("{name}.glb"), &bytes) {
                    Ok(()) => {
                        self.status = format!("exported {name}.glb");
                        true
                    }
                    Err(e) => {
                        self.status = format!("export failed: {e}");
                        false
                    }
                },
                Err(e) => {
                    self.status = format!("export failed: {e}");
                    false
                }
            };
        }
        let state = docs.engine_mut();
        // A drawing sheet's SVG writes `<name>-<sheet>.svg`: a document can
        // carry several sheets, and one file per export must not overwrite the
        // last. The PDF is the whole drawing SET, one page per sheet, so it is
        // the document's own `<name>.pdf`. It takes the BYTE lane — it is a
        // byte format, and both lanes type the file from its extension, so the
        // user gets `application/pdf` either way.
        // The 3D drawing set is `<name>-3d.pdf`, beside the plain one rather
        // than over it.
        if format == "sheetsvg" || format == "sheetpdf" || format == "sheetpdf3d" {
            let pdf = format != "sheetsvg";
            let three_d = format == "sheetpdf3d";
            let (file, written) = if pdf {
                let file = if three_d { format!("{name}-3d.pdf") } else { format!("{name}.pdf") };
                let bytes = if three_d { state.export_document_pdf_3d() } else { state.export_document_pdf() };
                let written = bytes.and_then(|bytes| store.export_file_named_bytes(&file, &bytes).map(|()| ()));
                (file, written)
            } else {
                let sheet_name = state
                    .resolve_sheet("")
                    .ok()
                    .and_then(|id| state.sheet_state().find_sheet(&id).map(|sheet| sheet.name.clone()))
                    .unwrap_or_default();
                let file = format!("{name}-{}.svg", slug(&sheet_name));
                let written = state.resolve_sheet("").and_then(|id| {
                    state
                        .export_sheet_svg(&id)
                        .and_then(|text| store.export_file_named(&file, &text).map(|()| ()))
                });
                (file, written)
            };
            let pages = state.sheet_state().sheets.len() + usize::from(three_d);
            return match written {
                Ok(()) => {
                    self.status = if pdf {
                        format!("exported {file} ({pages} page{})", if pages == 1 { "" } else { "s" })
                    } else {
                        format!("exported {file}")
                    };
                    true
                }
                Err(e) => {
                    self.status = format!("export failed: {e}");
                    false
                }
            };
        }
        let (text, ext) = match format {
            "stl" => (state.export_stl_text(), "stl"),
            "obj" => (state.export_obj_text(), "obj"),
            "iges" => (state.export_iges_text(), "igs"),
            "json" => (Ok(state.history_request_json()), "nbrep"),
            // The document's name becomes the root PRODUCT's — an assembly
            // exports as `<name>` with its parts named under it.
            _ => (state.export_step_text_named(&name), "step"),
        };
        match text {
            Ok(contents) => match store.export_file_named(&format!("{name}.{ext}"), &contents) {
                Ok(()) => {
                    self.status = format!("exported {name}.{ext}");
                    true
                }
                Err(e) => {
                    self.status = format!("export failed: {e}");
                    false
                }
            },
            Err(e) => {
                self.status = format!("export failed: {e}");
                false
            }
        }
    }

    /// Serialize the sheet-metal flat pattern in `format` (`"dxf"` | `"svg"`) and
    /// write it through the store under `<name>.<ext>`. Returns `true` on success
    /// (so the caller can close the modal). On failure the message goes to the
    /// status line AND is queued as a toast (the existing engine notice path), so
    /// a "no sheet-metal body in the part" error is surfaced prominently.
    fn export_flat_pattern_as(
        &mut self,
        docs: &mut Documents,
        store: &dyn ModelStore,
        format: &str,
    ) -> bool {
        let name = self.effective_name(docs);
        let state = docs.engine_mut();
        let (text, ext) = match format {
            "svg" => (state.export_flat_pattern_svg(), "svg"),
            _ => (state.export_flat_pattern_dxf(), "dxf"),
        };
        match text {
            Ok(contents) => match store.export_file_named(&format!("{name}.{ext}"), &contents) {
                Ok(()) => {
                    self.status = format!("exported {name}.{ext}");
                    true
                }
                Err(e) => {
                    self.status = format!("flat-pattern export failed: {e}");
                    false
                }
            },
            Err(e) => {
                self.status = format!("flat-pattern export failed: {e}");
                state.push_notice(format!("Flat pattern: {e}"));
                false
            }
        }
    }

    /// Serialize the assembly BOM in `format` (`"csv"` | `"json"`) and write it
    /// through the store under `<name>.bom.<ext>` (a compound extension naming
    /// the content, like the `.nbrep` recipe). On failure the message goes
    /// to the status line AND is queued as a toast — the flat-pattern error
    /// pattern — so a componentless document reports loudly.
    fn export_bom_as(
        &mut self,
        docs: &mut Documents,
        store: &dyn ModelStore,
        format: &str,
    ) -> bool {
        let name = self.effective_name(docs);
        let state = docs.engine_mut();
        let (text, ext) = match format {
            "json" => (state.export_bom_json(), "bom.json"),
            _ => (state.export_bom_csv(), "bom.csv"),
        };
        match text {
            Ok(contents) => match store.export_file_named(&format!("{name}.{ext}"), &contents) {
                Ok(()) => {
                    self.status = format!("exported {name}.{ext}");
                    true
                }
                Err(e) => {
                    self.status = format!("BOM export failed: {e}");
                    false
                }
            },
            Err(e) => {
                self.status = format!("BOM export failed: {e}");
                state.push_notice(format!("BOM export: {e}"));
                false
            }
        }
    }

    /// Write one of the PCB's manufacturing outputs through the store under
    /// [`Output::file_name`](super::fabrication_export::Output::file_name).
    /// The bundle's warnings (design-rule findings, silkscreen characters
    /// the font lacks) keep the dialog OPEN with the warnings on its status
    /// line, and go to a toast: the files are complete, but the user should
    /// read why before ordering boards. Returns `true` to close the dialog.
    fn export_fabrication_as(
        &mut self,
        docs: &mut Documents,
        store: &dyn ModelStore,
        output: super::fabrication_export::Output,
    ) -> bool {
        use super::fabrication_export::{self as fab, Output};
        let name = self.effective_name(docs);
        let file = output.file_name(&name);
        let state = docs.engine_mut();
        let (written, warnings) = match output {
            Output::Bundle => match fab::bundle(state, &name) {
                Ok(bundle) => (
                    store.export_file_named_bytes(&file, &bundle.zip()).map(|()| bundle.files.len()),
                    bundle.warnings,
                ),
                Err(e) => (Err(e), vec![]),
            },
            _ => (
                fab::text(state, output, &name).and_then(|text| store.export_file_named(&file, &text).map(|()| 1)),
                vec![],
            ),
        };
        match written {
            Ok(count) => {
                // Say WHERE: the dialog has no folder of its own, and on desktop
                // the files go to the models folder, not beside the board.
                let into = models_folder(store).map(|folder| format!(" into {folder}")).unwrap_or_default();
                self.status = if output == Output::Bundle {
                    format!("exported {file} ({count} files){into}")
                } else {
                    format!("exported {file}{into}")
                };
                if warnings.is_empty() {
                    state.push_notice_as(
                        brep_render::engine_state::NoticeSeverity::Info,
                        format!("{}: {}", output.label(), self.status),
                    );
                    return true;
                }
                self.status = format!("{}. Before ordering: {}", self.status, warnings.join(" "));
                state.push_notice_as(
                    brep_render::engine_state::NoticeSeverity::Warning,
                    format!("{}: {}", output.label(), warnings.join(" ")),
                );
                false
            }
            Err(e) => {
                self.status = format!("{} export failed: {e}", output.label());
                state.push_notice(format!("{} export: {e}", output.label()));
                false
            }
        }
    }

    /// Route an imported STEP file: **probe first**. The probe IS the parse —
    /// it stashes the structure in the engine for the import to consume — so a
    /// structured file arms the choice modal and is imported on the click,
    /// never parsed a second time.
    ///
    /// Everything else goes straight to today's flat lane, unchanged:
    ///
    /// * `Ok(None)` — no product structure. A part file must NEVER see the
    ///   dialog, and its behaviour stays byte-for-byte what it was.
    /// * `Err` — text the Part 21 parser refuses. Handing it to the flat lane
    ///   keeps the failure wording exactly today's (the two share the same
    ///   `ISO-10303-21` guard), rather than inventing a second one.
    fn import_step(&mut self, state: &mut EngineState, name: &str, contents: &str) {
        // A previous file's armed choice cannot survive this one: the probe
        // below REPLACES the engine's stash on every outcome (including the
        // structureless one), so an app-side pending left standing would offer
        // a button whose parse is already gone. A probe still running for an
        // earlier upload is superseded the same way (its answer is ignored).
        self.pending_step_import = None;
        let id = state.submit_step_probe(contents);
        self.pending_step_probe = Some(PendingStepProbe {
            id,
            name: name.to_string(),
            text: contents.to_string(),
        });
        self.status = format!("reading \"{name}\"\u{2026}");
        // The synchronous Inline runner (tests, headless) has answered inside
        // the submit; a background runner answers on a later frame's poll.
        self.poll_step_probe(state);
    }

    /// Resolve an answered STEP probe: structure arms the §3.9 choice, no
    /// structure (or a parse the flat lane will refuse with its own wording)
    /// takes the flat lane. A probe the engine no longer holds — a document
    /// switch or a cancelled run dropped it — is reported and forgotten.
    fn poll_step_probe(&mut self, state: &mut EngineState) {
        while let Some((id, outcome)) = state.take_step_probe() {
            let Some(pending) = self.pending_step_probe.as_ref() else {
                continue;
            };
            if pending.id != id {
                continue; // an earlier upload's answer; a newer probe replaced it
            }
            let PendingStepProbe { name, text, .. } = self.pending_step_probe.take().unwrap();
            match outcome {
                StepProbeOutcome::Structure(probe) => {
                    self.status = format!(
                        "\"{name}\" contains an assembly — {} part{}, {} instance{}",
                        probe.parts,
                        plural(probe.parts),
                        probe.instances,
                        plural(probe.instances)
                    );
                    self.pending_step_import = Some(PendingStepImport {
                        name,
                        text,
                        probe,
                        // Default = keep the tree (§3.9). Flattening is the escape
                        // hatch for a deep or pathological file, not the norm.
                        flatten: false,
                    });
                    // The routing site's `close_after_import` ran frames ago,
                    // while the probe was still out, and closed the modal: the
                    // choice is raised HERE, when the answer lands. (Under the
                    // synchronous Inline runner both run in the same call, and
                    // the second open is idempotent.)
                    self.open_modal(Mode::StepAssembly);
                }
                StepProbeOutcome::Flat | StepProbeOutcome::Failed(_) => {
                    self.import_step_flat(state, &name, &text);
                }
            }
        }
        if self.pending_step_probe.is_some() && !state.step_probes_pending() {
            // Nothing is running and no answer came: the engine dropped the
            // probe (document switch / cancel). Say so rather than reading
            // "reading…" forever.
            let pending = self.pending_step_probe.take().unwrap();
            self.status = format!("import of \"{}\" was cancelled", pending.name);
        }
    }

    /// Append an imported STEP file to the model (an IMPORT3D feature), then treat
    /// the enlarged model as dirty (an import is an edit, not an Open — the model
    /// keeps its current name / save baseline). THE flat lane, reached from a
    /// structureless file, an unparseable one, and the dialog's "Import as
    /// bodies".
    fn import_step_flat(&mut self, state: &mut EngineState, name: &str, contents: &str) {
        match state.import_step_feature(contents) {
            // Framing is deferred to the engine (`pending_fit`): under the native
            // thread / wasm worker runner the body is not resident yet, so framing
            // here would frame the empty scene. See [`EngineState::pending_fit`].
            Ok(_) => self.status = format!("imported {name}"),
            Err(e) => self.status = format!("import failed: {e}"),
        }
    }

    /// **Import as assembly** — consume the probe's stash into parts-library
    /// entries + one component per occurrence, in the engine's single batch.
    ///
    /// An `Err` here means the structured lane produced nothing, and the engine
    /// returns BEFORE it touches history when that happens — so the honest
    /// answer is the one A6's contract prescribes: re-run the flat import with
    /// the text this dialog is still holding, and say plainly that the assembly
    /// the user asked for came in as bodies. Silence there would leave them
    /// believing they have a structure tree that does not exist.
    fn step_assembly_import(&mut self, state: &mut EngineState, store: &dyn ModelStore) {
        let Some(pending) = self.pending_step_import.take() else {
            return;
        };
        // Read BEFORE the import: afterwards its own ACOMP features make the
        // answer unconditionally "yes" and the workbench switch never fires.
        let was_assembly = state.history_has_assembly();
        let doc_name = step_document_name(&pending.name);
        // The §3.9 checkbox. A depth-1 file imports identically either way, so
        // an un-shown box costs nothing.
        let opts = StepAssemblyImport {
            nested: !pending.flatten,
        };
        // Every unique part is written to the store as its own document, at the
        // browser's current location and under `{assembly}-{part}` — the same
        // `browser_write` door + "wherever the explorer is pointing" convention
        // the STEP parts-library import uses (`panels::step_parts`). So an
        // imported part is a part like any other: Open Part opens it, Update
        // Components tracks it, a part edit writes through to it.
        let mut sink = StorePartSink::new(store, &doc_name);
        let message = match state.import_probed_step_assembly(&doc_name, opts, &mut sink) {
            Ok(report) => {
                if !was_assembly {
                    self.enter_assembly_workbench(state);
                }
                // The store side effect is REPORTED, never silent: a 50-part
                // import writes 50 files the user did not individually ask for.
                for failure in &sink.failures {
                    state.push_notice(format!("part not saved — {failure}"));
                }
                let saved = sink.written;
                let base = assembly_import_message(&pending.name, &report);
                match (saved, sink.failures.len()) {
                    (0, _) => base,
                    (saved, 0) => format!("{base}; saved {saved} part file(s)"),
                    (saved, failed) => {
                        format!("{base}; saved {saved} part file(s), {failed} could not be saved")
                    }
                }
            }
            Err(error) => match state.import_step_feature(&pending.text) {
                Ok(_) => format!(
                    "imported {} as bodies — the assembly structure could not be built ({error})",
                    pending.name
                ),
                Err(flat) => format!("import failed: {flat}"),
            },
        };
        // The status line only renders inside an OPEN modal and this one closes
        // on the click, so the toast is the half the user actually sees.
        self.status = message.clone();
        state.push_notice(message);
    }

    /// **Import as bodies** — today's flat lane, unchanged. Drops the probe's
    /// stash first: the user chose the text, so the parsed assembly (and every
    /// product's solids it is holding resident) has no consumer left.
    fn step_assembly_bodies(&mut self, state: &mut EngineState) {
        let Some(pending) = self.pending_step_import.take() else {
            return;
        };
        state.discard_probed_step_assembly();
        self.import_step_flat(state, &pending.name, &pending.text);
    }

    /// **Cancel** — import nothing and drop the parse (§3.9).
    fn step_assembly_cancel(&mut self, state: &mut EngineState) {
        let Some(pending) = self.pending_step_import.take() else {
            return;
        };
        state.discard_probed_step_assembly();
        self.status = format!("import cancelled: {}", pending.name);
    }

    /// Switch to the **Assembly** workbench so a freshly imported structure is
    /// actually reachable: the Assembly Structure tree and the Constraints panel
    /// are CLAIMED by that workbench, so an assembly imported under Modeling
    /// would land with its structure invisible.
    ///
    /// No-op when the active workbench already shows those panels: the
    /// predicate asks whether the BOM panel is visible, which is true for
    /// Assembly itself, for Wire harness (it lists the assembly panels too),
    /// and for "All", whose users must not be yanked out of it. Applied
    /// through the same settings seam the toolbar dropdown and the saved-
    /// workbench restore use, and like the restore NOT persisted to the settings
    /// blob: that blob stays the user's boot preference, and a document's
    /// workbench is session-scoped.
    fn enter_assembly_workbench(&mut self, state: &mut EngineState) {
        if crate::workbench::panel_visible(
            &state.settings.workbench,
            crate::workbench::assembly::BOM_PANEL_ID,
            &crate::workbench::ButtonState::of(state),
        ) {
            return;
        }
        let _ = state.apply_settings_json(
            &serde_json::json!({ "workbench": crate::workbench::assembly::ASSEMBLY.id })
                .to_string(),
        );
    }

    /// Append an imported IGES file to the model (an IMPORT3D feature) — the
    /// IGES sibling of [`Self::import_step`].
    fn import_iges(&mut self, state: &mut EngineState, name: &str, contents: &str) {
        match state.import_iges_feature(contents) {
            // Framing deferred to the engine (`pending_fit`) — see `import_step`.
            Ok(_) => self.status = format!("imported {name}"),
            Err(e) => self.status = format!("import failed: {e}"),
        }
    }

    fn stage_mesh_preview(
        &mut self,
        format: MeshImportFormat,
        name: String,
        contents: Vec<u8>,
    ) {
        self.pending_stl_import = Some((format, name, contents));
        self.status.clear();
    }

    /// Selecting an STL or a 3MF starts a preview session; only its Accept
    /// action edits the destination document.
    pub fn take_stl_import(&mut self) -> Option<(MeshImportFormat, String, Vec<u8>)> {
        self.pending_stl_import.take()
    }

    fn import_obj(&mut self, state: &mut EngineState, name: &str, contents: &[u8]) {
        match state.import_obj_bytes_feature(contents) {
            Ok(_) => self.status = format!("reconstructing {name} in background…"),
            Err(e) => self.status = format!("import failed: {e}"),
        }
    }

    fn import_text(
        &mut self,
        state: &mut EngineState,
        name: &str,
        bytes: &[u8],
        importer: fn(&mut Self, &mut EngineState, &str, &str),
    ) {
        match std::str::from_utf8(bytes) {
            Ok(contents) => importer(self, state, name, contents),
            Err(_) => self.status = format!("import failed: {name} is not UTF-8 text"),
        }
    }

    /// Load a model document's contents into a NEW TAB (roll to the last
    /// feature + zoom-to-fit), clean from the start. A document that fails to
    /// load leaves no tab behind — the engine it was loading into is dropped
    /// with its runner.
    fn load_document(&mut self, docs: &mut Documents, name: &str, contents: &str, plm: bool) {
        let mut engine = docs.spawn_engine();
        match engine.load_model_and_fit(contents) {
            Ok(_) => {
                // The extension names the class on the file system; the
                // document field follows it, before `Document::new` takes the
                // clean baseline so a hand-renamed file does not open dirty.
                // On a PLM store the name is a revision key the explorer spells
                // with `.nbrep` whatever the part is, so the document's own
                // field is the class (the part's fixed `document_class`).
                if let Some(class) = DocumentClass::of_name(name).filter(|_| !plm) {
                    document_class::set_document_class(&mut engine, class);
                }
                // The document keeps the raw identity — the full path when a
                // native dialog picked the file, so plain Save writes back to
                // it; the name field shows only the bare display name.
                let mut doc = Document::new(engine);
                doc.set_name(Some(name.to_string()));
                docs.open_document(doc);
                self.name_buf = model_display_name(name);
                self.status = format!("opened {name}");
            }
            Err(e) => self.status = format!("open failed: {e}"),
        }
    }

    // --- document classes: families, templates, members ------------------------

    /// The session's PLM client and the revision key `identity` names, when
    /// the store IS the PLM and `identity` is one of its revisions.
    fn plm_revision(&self, store: &dyn ModelStore, identity: &str) -> Option<(std::rc::Rc<crate::plm::client::PlmClient>, String)> {
        let client = store.plm_client()?;
        let key = crate::plm::family::revision_key(identity)?;
        Some((client, key))
    }

    /// Poll the PLM work in flight (S9) and act on what finished.
    fn poll_plm_work(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        let Some(work) = self.plm_work.as_mut() else { return };
        ctx.request_repaint_after(std::time::Duration::from_millis(30));
        match work {
            PlmWork::Members(pending) => {
                let Some(result) = pending.poll() else { return };
                self.plm_work = None;
                let Some(pick) = self.pick_member.as_mut() else { return };
                pick.loading = false;
                match result {
                    Ok(members) => {
                        pick.rows = members
                            .into_iter()
                            .map(|member| MemberChoice {
                                why_not: member.not_placeable(),
                                exists: member.not_placeable().is_none(),
                                identity: member.key(),
                                description: member.name.clone(),
                                part_number: member.number,
                            })
                            .collect();
                    }
                    Err(error) => self.status = format!("the members could not be read: {error}"),
                }
            }
            PlmWork::Place(pending) => {
                let Some(result) = pending.poll() else { return };
                self.plm_work = None;
                match result {
                    Ok((key, text, number)) => {
                        self.insert_component_named(docs.engine_mut(), &key, &number, &text);
                        self.pick_member = None;
                        self.spin_out = None;
                        self.open = false;
                    }
                    Err(error) => self.status = error,
                }
            }
            PlmWork::OpenFamily(pending) => {
                let Some(result) = pending.poll() else { return };
                self.plm_work = None;
                match result {
                    Ok(key) => self.open_document(docs, store, &key),
                    Err(error) => self.status = format!("the family could not be opened: {error}"),
                }
            }
        }
    }

    /// Route an Insert-component choice by its class. A FAMILY arms the member
    /// picker and a TEMPLATE the spin-out prompt; both return `true` (handled,
    /// the modal switches). A normal part returns `false` and is inserted as
    /// it always was. The class is the file's extension, or the document's own
    /// field when the name carries none (a browser upload's bare name).
    fn begin_class_insert(
        &mut self,
        store: &dyn ModelStore,
        identity: &str,
        contents: &str,
        assembly: Option<String>,
    ) -> bool {
        let document: serde_json::Value = serde_json::from_str(contents).unwrap_or_default();
        let class = DocumentClass::of_name(identity)
            .filter(|class| *class != DocumentClass::Normal)
            .unwrap_or_else(|| DocumentClass::of_document(&document));
        let file = crate::store::file_name_of(identity);
        match class {
            DocumentClass::Normal => false,
            DocumentClass::Family if self.plm_revision(store, identity).is_some() => {
                let (client, key) = self.plm_revision(store, identity).unwrap_or_else(|| unreachable!());
                let Some((part, _)) = crate::plm::family::key_ids(&key).map(|(p, r)| (p.to_string(), r.to_string())) else {
                    return false;
                };
                self.plm_work = Some(PlmWork::Members(crate::panels::plm_family::Pending::new(Box::pin(async move {
                    crate::plm::family::family_view(&client, &part).await.map(|view| view.members()).map_err(|e| e.to_string())
                }))));
                self.pick_member = Some(PickMember { family_file: file, rows: Vec::new(), plm: true, loading: true });
                self.status.clear();
                self.open_modal(Mode::PickMember);
                true
            }
            DocumentClass::Family => {
                let table = family_table::read_table(contents);
                let rows = table
                    .rows
                    .iter()
                    .map(|row| {
                        let identity = family_table::member_identity(Some(identity), &row.part_number);
                        let exists = identity.as_deref().is_some_and(|id| store.read(id).is_some());
                        MemberChoice {
                            part_number: row.part_number.clone(),
                            description: row.description.clone(),
                            identity,
                            exists,
                            why_not: None,
                        }
                    })
                    .collect();
                self.pick_member = Some(PickMember { family_file: file, rows, plm: false, loading: false });
                self.status.clear();
                self.open_modal(Mode::PickMember);
                true
            }
            DocumentClass::Template => {
                let inputs = template::inputs_of(&document);
                let values = inputs
                    .iter()
                    .map(|input| (input.name.clone(), template::default_value(&document, input)))
                    .collect();
                let stem = document_class::strip_class_extension(&file).to_string();
                let plm_template = self
                    .plm_revision(store, identity)
                    .and_then(|(_, key)| crate::plm::family::key_ids(&key).map(|(part, _)| part.to_string()));
                self.spin_out = Some(SpinOut {
                    template_file: file,
                    template: document,
                    inputs,
                    name: format!("{stem}-1"),
                    values,
                    assembly,
                    plm_template,
                    number: String::new(),
                });
                self.status.clear();
                self.open_modal(Mode::SpinOut);
                true
            }
        }
    }

    /// The **member picker**: a family is never placed itself. Each table row
    /// is a button; one whose member file exists inserts that `.nbrep` as a
    /// normal component, one that was never generated says so.
    fn show_pick_member(&mut self, ctx: &egui::Context, state: &mut EngineState, store: &dyn ModelStore) {
        let Some(pick) = self.pick_member.take() else {
            self.open = false;
            return;
        };
        let mut chosen = None;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-pick-member")).show(ctx, |ui| {
            ui.set_width(420.0);
            ui.heading(format!("Insert a member of {}", pick.family_file));
            ui.add_space(2.0);
            ui.weak("A family is never placed itself: choose the member to place.");
            ui.add_space(6.0);
            if pick.loading {
                ui.label("Reading the family's members from the PLM\u{2026}");
            } else if pick.rows.is_empty() {
                ui.label("This family's table has no rows yet.");
            }
            egui::ScrollArea::vertical().max_height(320.0).show(ui, |ui| {
                for (index, row) in pick.rows.iter().enumerate() {
                    ui.horizontal(|ui| {
                        let text = if row.description.trim().is_empty() {
                            row.part_number.clone()
                        } else {
                            format!("{} \u{2014} {}", row.part_number, row.description)
                        };
                        let button = ui.add_enabled(
                            row.exists,
                            egui::Button::new(text).min_size(egui::vec2(220.0, 0.0)),
                        );
                        self.hit(&format!("member:{}", row.part_number), &button);
                        if button.clicked() {
                            chosen = Some(index);
                        }
                        if let Some(why) = &row.why_not {
                            ui.weak(format!("({why})"));
                        } else if row.identity.is_none() {
                            ui.weak("(the part number is not a file name)");
                        } else if !row.exists {
                            ui.weak("(not generated yet: open the family and Generate)");
                        }
                    });
                }
            });
            ui.add_space(6.0);
            if !self.status.is_empty() {
                ui.weak(&self.status);
                ui.add_space(4.0);
            }
            let c = ui.button("Cancel");
            self.hit("member:cancel", &c);
            cancel = c.clicked();
        });
        let should_close = modal.should_close();
        if let (Some(index), true) = (chosen, pick.plm) {
            let row = &pick.rows[index];
            if let (Some(client), Some(key)) = (store.plm_client(), row.identity.clone()) {
                self.status = format!("reading {} from the PLM\u{2026}", row.part_number);
                self.plm_work = Some(PlmWork::Place(fetch_document(client, key, row.part_number.clone())));
            }
        } else if let Some(index) = chosen {
            let row = &pick.rows[index];
            let identity = row.identity.clone().unwrap_or_default();
            match store.read(&identity) {
                Some(contents) => {
                    self.insert_component_document(state, &identity, &contents);
                    self.open = false;
                    return;
                }
                None => self.status = format!("'{}' has not been generated yet ({identity})", row.part_number),
            }
        } else if cancel || should_close {
            self.status.clear();
            self.open = false;
            return;
        }
        self.pick_member = Some(pick);
    }

    /// The **template spin-out** prompt: a template is never placed itself.
    /// The user names the new part and sets each marked input (a fixed list
    /// is offered as buttons; limits are checked as they type), then Create
    /// writes the specialised copy as `<name>.nbrep` beside the assembly and
    /// places it. A name that already exists is refused, never overwritten.
    fn show_spin_out(&mut self, ctx: &egui::Context, state: &mut EngineState, store: &dyn ModelStore) {
        let Some(mut spin) = self.spin_out.take() else {
            self.open = false;
            return;
        };
        let mut create = false;
        let mut cancel = false;
        let problems: Vec<Option<String>> = spin
            .inputs
            .iter()
            .map(|input| {
                let text = spin.values.get(&input.name).map(String::as_str).unwrap_or("");
                template::value_problem(&spin.template, input, text)
            })
            .collect();
        let name_problem = template::name_problem(&spin.name);
        let modal = egui::Modal::new(egui::Id::new("brep-file-spin-out")).show(ctx, |ui| {
            ui.set_width(440.0);
            ui.heading(format!("New part from {}", spin.template_file));
            ui.add_space(2.0);
            ui.weak("A template is never placed itself: this makes a new part from it and places that.");
            ui.add_space(6.0);
            egui::Grid::new("brep-spin-out-grid").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
                ui.label("New part name");
                let field = ui.add(egui::TextEdit::singleline(&mut spin.name).desired_width(240.0));
                self.hit("spinout:name", &field);
                ui.end_row();
                if spin.plm_template.is_some() {
                    ui.label("Part number");
                    let field = ui.add(
                        egui::TextEdit::singleline(&mut spin.number)
                            .hint_text("empty: the part type numbers it")
                            .desired_width(240.0),
                    );
                    self.hit("spinout:number", &field);
                    ui.end_row();
                }
                for (input, problem) in spin.inputs.iter().zip(&problems) {
                    ui.label(input.shown());
                    ui.vertical(|ui| {
                        let value = spin.values.entry(input.name.clone()).or_default();
                        if input.choices.is_empty() {
                            let field = ui.add(egui::TextEdit::singleline(value).desired_width(240.0));
                            self.hit(&format!("spinout:input:{}", input.name), &field);
                        } else {
                            ui.horizontal_wrapped(|ui| {
                                for choice in &input.choices {
                                    let chip = ui.selectable_label(value.trim() == choice.trim(), choice);
                                    self.hit(&format!("spinout:choice:{}:{}", input.name, choice.trim()), &chip);
                                    if chip.clicked() {
                                        *value = choice.trim().to_string();
                                    }
                                }
                            });
                        }
                        let limits = match (input.min, input.max) {
                            (Some(min), Some(max)) => format!("{min} to {max}"),
                            (Some(min), None) => format!("at least {min}"),
                            (None, Some(max)) => format!("at most {max}"),
                            (None, None) => String::new(),
                        };
                        if !limits.is_empty() {
                            ui.weak(limits);
                        }
                        if let Some(problem) = problem {
                            ui.colored_label(ui.visuals().warn_fg_color, problem);
                        }
                    });
                    ui.end_row();
                }
            });
            if spin.inputs.is_empty() {
                ui.weak("This template marks no inputs; the copy is the template as it is.");
            }
            ui.add_space(4.0);
            let file = crate::store::file_name_of(&spin_out_target(&spin));
            ui.weak(match &spin.assembly {
                _ if spin.plm_template.is_some() => {
                    "The PLM makes the new part from the template and bakes it; it is placed as its first revision.".to_string()
                }
                Some(assembly) => format!(
                    "Saved as {file}, in the folder of {}",
                    crate::store::file_name_of(assembly)
                ),
                None => format!("The assembly is not saved yet: the part goes to the models folder as {file}"),
            });
            if let Some(problem) = &name_problem {
                ui.colored_label(ui.visuals().warn_fg_color, problem);
            }
            if !self.status.is_empty() {
                ui.weak(&self.status);
            }
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                let ok = name_problem.is_none() && problems.iter().all(Option::is_none) && self.plm_work.is_none();
                let c = ui.add_enabled(ok, egui::Button::new("Create and insert"));
                self.hit("spinout:create", &c);
                create = c.clicked();
                let x = ui.button("Cancel");
                self.hit("spinout:cancel", &x);
                cancel = x.clicked();
            });
        });
        let should_close = modal.should_close();
        if create {
            match self.create_spin_out(state, store, &spin) {
                Ok(()) => {
                    self.open = false;
                    return;
                }
                Err(error) => self.status = error,
            }
        } else if cancel || should_close {
            self.status.clear();
            self.open = false;
            return;
        }
        self.spin_out = Some(spin);
    }

    /// Write the spun-out part and place it. `Err` is the status line.
    fn create_spin_out(&mut self, state: &mut EngineState, store: &dyn ModelStore, spin: &SpinOut) -> Result<(), String> {
        if let (Some(template_part), Some(client)) = (spin.plm_template.clone(), store.plm_client()) {
            // The server makes the part; it is placed when it answers. The
            // dialog stays open until then, saying so.
            let (name, number, values) = (spin.name.trim().to_string(), spin.number.trim().to_string(), spin.values.clone());
            self.plm_work = Some(PlmWork::Place(crate::panels::plm_family::Pending::new(Box::pin(async move {
                let spun = crate::plm::family::spin_out(&client, &template_part, &name, &number, &values)
                    .await
                    .map_err(|e| e.to_string())?;
                let bytes = client.get_document(&spun.key).await.map_err(|e| e.to_string())?;
                let text = bytes.map(|b| String::from_utf8_lossy(&b).into_owned()).ok_or_else(|| {
                    format!("{} was made, but has no document yet", spun.number)
                })?;
                Ok((spun.key, text, spun.number))
            }))));
            return Err("making the part on the PLM\u{2026}".into());
        }
        if let Some(problem) = template::name_problem(&spin.name) {
            return Err(problem);
        }
        let target = spin_out_target(spin);
        if store.read(&target).is_some() {
            return Err(format!("'{target}' already exists: choose another name"));
        }
        let copy = template::spin_out(&spin.template, &spin.template_file, &spin.values)?;
        let text = copy.to_string();
        store.write(&target, &text).map_err(|error| format!("could not save {target}: {error}"))?;
        self.save_generation += 1;
        self.insert_component_document(state, &target, &text);
        if spin.assembly.is_none() {
            self.status = format!(
                "{} — saved to the models folder as {target}, because the assembly is not saved yet",
                self.status
            );
        }
        Ok(())
    }

    /// Arm the hand-edit prompt when the active tab is a family member. `true`
    /// when it was armed (the caller does not go on to write).
    fn arm_member_prompt(&mut self, docs: &Documents, name: &str, from_save: bool) -> bool {
        let Some(source) = family_table::engine_family_source(docs.engine()) else {
            return false;
        };
        self.member_prompt = Some(MemberPrompt {
            doc_id: docs.active().id(),
            family_identity: sibling_identity(name, &source.family),
            source,
            from_save,
        });
        self.status.clear();
        self.open_modal(Mode::MemberEdit);
        true
    }

    /// Raise the hand-edit prompt the frame a family member first shows an
    /// unsaved change — the moment the user "starts to change it".
    fn watch_member_edit(&mut self, docs: &Documents) {
        let doc = docs.active();
        if !doc.dirty_marker() || self.member_ack.contains(&doc.id()) {
            return;
        }
        if let Some(name) = doc.name().map(str::to_string) {
            self.arm_member_prompt(docs, &name, false);
        }
    }

    /// Undo the active tab back to its saved state (bounded), so a member the
    /// user declined to change is left exactly as generated.
    fn revert_member_edit(docs: &mut Documents) {
        for _ in 0..64 {
            if !docs.active().is_dirty() || !docs.engine().history.can_undo() {
                break;
            }
            docs.engine_mut().undo();
        }
    }

    /// The **hand-edit prompt** on a family member: a member is the family's
    /// output, so a change belongs either in a NEW part or in the family.
    fn show_member_edit(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        let Some(prompt) = self.member_prompt.take() else {
            self.open = false;
            return;
        };
        if docs.active().id() != prompt.doc_id {
            self.open = false;
            return;
        }
        let mut choice = None;
        let plm = docs.active().name().and_then(|name| self.plm_revision(store, name));
        let modal = egui::Modal::new(egui::Id::new("brep-file-member-edit")).show(ctx, |ui| {
            ui.set_width(440.0);
            ui.heading("This part is a family member");
            ui.add_space(4.0);
            ui.label(format!(
                "'{}' was generated from the family '{}'. A member is written by its family, not edited by hand.",
                prompt.source.part_number, prompt.source.family
            ));
            ui.add_space(4.0);
            ui.weak(if prompt.from_save {
                "Save it as a new part, or open the family and make the change there."
            } else {
                "Keep the change in a new part, or open the family and make it there."
            });
            if !self.status.is_empty() {
                ui.add_space(4.0);
                ui.weak(&self.status);
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let a = ui.button("Save as a new part\u{2026}");
                self.hit("member-edit:saveas", &a);
                if a.clicked() {
                    choice = Some("saveas");
                }
                // On a PLM only someone who may edit the family is offered it.
                let offer_family = match &plm {
                    Some((client, _)) => client.me().is_some_and(|me| me.groups.iter().any(|g| g == "author" || g == "admin")),
                    None => true,
                };
                if offer_family {
                    let b = ui.button("Open the family");
                    self.hit("member-edit:family", &b);
                    if b.clicked() {
                        choice = Some("family");
                    }
                }
                let c = ui.button(if prompt.from_save { "Cancel" } else { "Cancel (undo the change)" });
                self.hit("member-edit:cancel", &c);
                if c.clicked() {
                    choice = Some("cancel");
                }
            });
        });
        let choice = choice.or_else(|| modal.should_close().then_some("cancel"));
        match choice {
            Some("saveas") => {
                // The tab may keep changing; Save As strips the stamp from
                // the copy it writes. Plain Save still asks.
                self.member_ack.insert(prompt.doc_id);
                self.status.clear();
                self.dispatch(FileAction::SaveAs, docs, store);
            }
            Some("family") if plm.is_some() => {
                let (client, _) = plm.unwrap_or_else(|| unreachable!());
                let number = document_class::strip_class_extension(&prompt.source.family).to_string();
                Self::revert_member_edit(docs);
                self.status = format!("opening the family {number}\u{2026}");
                self.plm_work = Some(PlmWork::OpenFamily(crate::panels::plm_family::Pending::new(Box::pin(async move {
                    let head = crate::plm::family::part_head(&client, &number).await.map_err(|e| e.to_string())?;
                    // The newest revision, drafts included (D13).
                    let newest = head.revisions.last().ok_or_else(|| format!("the family {number} has no revision"))?;
                    Ok(format!("part/{}/rev/{}", head.id, newest.id))
                }))));
                self.open = false;
            }
            Some("family") => {
                if store.read(&prompt.family_identity).is_none() {
                    self.status = format!("the family file is not there: {}", prompt.family_identity);
                    self.member_prompt = Some(prompt);
                    return;
                }
                Self::revert_member_edit(docs);
                self.open = false;
                self.open_document(docs, store, &prompt.family_identity);
            }
            Some(_) => {
                if !prompt.from_save {
                    Self::revert_member_edit(docs);
                }
                self.status.clear();
                self.open = false;
            }
            None => self.member_prompt = Some(prompt),
        }
    }

    /// Run the family's Generate on the ACTIVE document (a family seed) and
    /// keep the report for the automation blob. The one door the automation
    /// command and the table editor's button share.
    /// Generate the active family. On a PLM store (the document is a
    /// `part/…/rev/…` revision and the session has a client) the server
    /// writes the members from the family's SAVED revision: a family with
    /// unsaved edits is refused unless `save` is set, which saves it first
    /// (plm-cad-integration-todo S9). Everywhere else this is the file-system
    /// Generate, and `save` is ignored.
    pub fn generate_active_family(
        &mut self,
        docs: &mut Documents,
        store: &dyn ModelStore,
        save: bool,
    ) -> Result<family_table::GenerateReport, String> {
        let plm_key = docs.active().name().and_then(crate::plm::family::revision_key);
        let report = match (store.plm_client(), plm_key) {
            (Some(client), Some(key)) => self.generate_on_plm(docs, store, &client, &key, save)?,
            _ => family_table::generate_family(store, docs.active().name(), &docs.engine().history_request_json()),
        };
        Ok(self.record_generate(report))
    }

    /// The PLM half of [`Self::generate_active_family`]: blocks this thread
    /// while the members are built and the server answers (native; the web
    /// build's editor polls instead).
    #[cfg(not(target_arch = "wasm32"))]
    fn generate_on_plm(
        &mut self,
        docs: &mut Documents,
        store: &dyn ModelStore,
        client: &std::rc::Rc<crate::plm::client::PlmClient>,
        key: &str,
        save: bool,
    ) -> Result<family_table::GenerateReport, String> {
        let json = docs.engine().history_request_json();
        if docs.active().is_dirty() {
            if !save {
                return Err(crate::plm::family::UNSAVED_FAMILY.into());
            }
            store.write(key, &json)?;
            docs.active_mut().mark_clean();
            self.save_generation += 1;
            // The write is behind: wait for it to land before the server reads it.
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(60);
            while store.pending_writes() > 0 {
                if std::time::Instant::now() > deadline {
                    return Err("the family's save did not reach the server within 60 s".into());
                }
                crate::plm::native::run_pending();
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
            if let Some(error) = store.take_persistence_errors().into_iter().next() {
                return Err(format!("the family was not saved, so nothing was generated: {error}"));
            }
        }
        crate::plm::native::block_on_for(crate::plm::family::generate(client, key, &json), std::time::Duration::from_secs(300))
            .ok_or("the PLM server did not answer Generate within 5 minutes")?
            .map_err(|e| e.to_string())
    }

    #[cfg(target_arch = "wasm32")]
    fn generate_on_plm(
        &mut self,
        _: &mut Documents,
        _: &dyn ModelStore,
        _: &std::rc::Rc<crate::plm::client::PlmClient>,
        _: &str,
        _: bool,
    ) -> Result<family_table::GenerateReport, String> {
        Err("in the browser, Generate a PLM family from the family table's Generate button".into())
    }

    fn record_generate(&mut self, report: family_table::GenerateReport) -> family_table::GenerateReport {
        if report.rows.iter().any(|row| row.status() == "written") {
            self.save_generation += 1;
        }
        self.status = format!("generate: {}", report.summary());
        self.last_generate = Some(report.clone());
        report
    }

    // --- verifier hooks (wasm only) -------------------------------------------

    /// The published state for the headed verifier: current name, dirty flag,
    /// backend label, the stored-document list, and the modal's open/mode.
    pub fn file_state_json(&self, docs: &Documents, store: &dyn ModelStore) -> String {
        serde_json::json!({
            "name": docs.active().name(),
            "nameBuf": self.name_buf,
            "selected": self.explorer.selected(),
            "location": store.browser_location(),
            // The last folder of each purpose (Open/Save As/Insert, Import,
            // KiCad), which is where that modal opens next.
            "folders": Purpose::ALL
                .iter()
                .map(|purpose| (purpose.slug().to_string(), serde_json::json!(self.folders.get(purpose))))
                .collect::<serde_json::Map<_, _>>(),
            // The CACHED marker, not `is_dirty()`: this blob is published every
            // frame, and `is_dirty()` answers by serializing the whole document
            // — for a document whose history holds an imported model that is
            // hundreds of KB of JSON built and thrown away per frame, on the
            // thread that also has to answer the pointer. `Document::refresh_dirty_marker`
            // exists for exactly this and is refreshed once per frame; the tab
            // strip's dot already reads it, so the two cannot disagree.
            "dirty": docs.active().dirty_marker(),
            "backend": store.backend_label(),
            "interchange": store.supports_file_interchange(),
            "list": store.list(),
            "status": self.status,
            "open": self.open,
            "mode": match self.mode {
                Mode::Open => "open",
                Mode::SaveAs => "saveas",
                Mode::ConfirmClose => "confirmclose",
                Mode::Export => "export",
                Mode::Import => "import",
                Mode::FlatPattern => "flatpattern",
                Mode::InsertComponent => "insertcomponent",
                Mode::StepAssembly => "stepassembly",
                Mode::Kicad => "kicad",
                Mode::PickMember => "pickmember",
                Mode::SpinOut => "spinout",
                Mode::MemberEdit => "memberedit",
                Mode::PickFile => "pickfile",
            },
            "pickFile": self.pick_request.as_ref().map(|(tag, title)| serde_json::json!({ "tag": tag, "title": title })),
            "picked": self.picked.as_ref().map(|(tag, file)| serde_json::json!({
                "tag": tag, "name": file.name, "path": file.path, "size": file.bytes.len(),
            })),
            // The active document's class (from its own `documentClass`
            // field) and, for a family member, its stamp.
            "class": document_class::document_class(docs.engine()).slug(),
            "saveClass": self.save_class.slug(),
            "familySource": family_table::engine_family_source(docs.engine()).map(|source| serde_json::json!({
                "family": source.family,
                "partNumber": source.part_number,
                "valuesHash": source.values_hash,
                "historyHash": source.history_hash,
            })),
            // The armed member picker: each row, where its member is, and
            // whether Generate has written it.
            "pickMember": self.pick_member.as_ref().map(|pick| serde_json::json!({
                "family": pick.family_file,
                "plm": pick.plm,
                "loading": pick.loading,
                "rows": pick.rows.iter().map(|row| serde_json::json!({
                    "partNumber": row.part_number,
                    "identity": row.identity,
                    "exists": row.exists,
                    "whyNot": row.why_not,
                })).collect::<Vec<_>>(),
            })),
            "plmBusy": self.plm_work.is_some(),
            // The armed template spin-out: the answers so far, where the new
            // part will go, and what (if anything) blocks Create.
            "spinOut": self.spin_out.as_ref().map(|spin| serde_json::json!({
                "template": spin.template_file,
                "name": spin.name,
                "values": spin.values,
                "inputs": spin.inputs.iter().map(|input| input.name.clone()).collect::<Vec<_>>(),
                "target": spin_out_target(spin),
                "problems": spin.inputs.iter().filter_map(|input| template::value_problem(
                    &spin.template,
                    input,
                    spin.values.get(&input.name).map(String::as_str).unwrap_or(""),
                )).chain(template::name_problem(&spin.name)).collect::<Vec<_>>(),
            })),
            "memberPrompt": self.member_prompt.as_ref().map(|prompt| serde_json::json!({
                "partNumber": prompt.source.part_number,
                "family": prompt.source.family,
                "familyIdentity": prompt.family_identity,
                "fromSave": prompt.from_save,
            })),
            "generate": self.last_generate.as_ref().map(|report| serde_json::json!({
                "summary": report.summary(),
                "rows": report.to_json(),
            })),
            "kicad": (self.open && self.mode == Mode::Kicad).then(|| self.kicad.state_json()),
            // The armed §3.9 assembly choice: the file it belongs to and the
            // counts the prompt is showing, so the headed verifier can confirm
            // the dialog appeared with the numbers the import will deliver.
            // A `.step` upload whose structure probe is still running on the
            // background runner (the "reading…" state).
            "probing": self.pending_step_probe.as_ref().map(|pending| pending.name.clone()),
            "stepAssembly": self.pending_step_import.as_ref().map(|pending| {
                serde_json::json!({
                    "name": pending.name,
                    "parts": pending.probe.parts,
                    "instances": pending.probe.instances,
                    "nestedDepth": pending.probe.nested_depth,
                    // The §3.9 checkbox: shown only when there is a tree to
                    // flatten, and OFF by default (import keeps the tree).
                    "flatten": pending.flatten,
                })
            }),
        })
        .to_string()
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Record a widget's screen rect for the headed verifier (wasm only; a no-op
    /// elsewhere so the dialog code reads the same on both targets).
    fn hit(&mut self, key: &str, resp: &egui::Response) {
        self.hits.insert(key.to_string(), resp.rect);
    }

    fn record_explorer_hits(&mut self, hits: &[(String, egui::Rect)]) {
        self.hits.extend(hits.iter().cloned());
    }
}


/// A sheet's name as a file-name fragment: spaces and separators become
/// hyphens, so `Sheet 1` exports as `<document>-Sheet-1.svg` and cannot
/// reach the store as a path.
fn slug(name: &str) -> String {
    let cleaned: String = name
        .chars()
        .map(|c| if c.is_alphanumeric() { c } else { '-' })
        .collect();
    let trimmed = cleaned.trim_matches('-').to_string();
    if trimmed.is_empty() { "sheet".into() } else { trimmed }
}
/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "file", prefix: "field:name", meaning: "the document name field", command: None },
    HitKeyDoc { panel: "file", prefix: "save", meaning: "confirm save / save-as", command: None },
    HitKeyDoc { panel: "file", prefix: "cancel", meaning: "close the modal", command: None },
    HitKeyDoc { panel: "file", prefix: "confirm:cancel", meaning: "keep the dirty document open", command: None },
    HitKeyDoc { panel: "file", prefix: "confirm:discard", meaning: "discard the dirty document", command: None },
    HitKeyDoc { panel: "file", prefix: "export:", meaning: "export in a format (export:step, export:iges, export:stl, export:obj, export:glb, export:json, export:bomcsv, export:bomjson, the open DRAWING SHEET as export:sheetsvg, every drawing sheet as ONE multi-page PDF as export:sheetpdf, and the same drawing set with the live 3D model as export:sheetpdf3d; for a document whose PCB board has parts, its manufacturing files: the Gerber/drill/pick-and-place/BOM zip as export:fabrication, the pick-and-place CSV as export:pickplace, the electronics BOM CSV as export:ecadbom)", command: None },
    HitKeyDoc { panel: "file", prefix: "flat:", meaning: "export the flat pattern (flat:dxf, flat:svg)", command: None },
    HitKeyDoc { panel: "file", prefix: "stepassembly:", meaning: "a STEP-with-structure import choice (assembly, bodies, flatten, cancel)", command: None },
    HitKeyDoc { panel: "file", prefix: "insert:lib:", meaning: "insert a parts-library entry (insert:lib:name)", command: None },
    HitKeyDoc { panel: "file", prefix: "filesystem:", meaning: "an entry of the file explorer", command: None },
    HitKeyDoc { panel: "file", prefix: "saveas:class:", meaning: "Save As's class chooser: saveas:class:normal (.nbrep), saveas:class:family (.fbrep), saveas:class:template (.tbrep)", command: None },
    HitKeyDoc { panel: "file", prefix: "member:", meaning: "the member picker Insert component raises for a family: member:<part number> places that member (disabled until Generate has written it), member:cancel", command: None },
    HitKeyDoc { panel: "file", prefix: "spinout:", meaning: "the template spin-out prompt: spinout:name, spinout:input:<name> (a value field), spinout:choice:<name>:<value> (a fixed-list input), spinout:create, spinout:cancel", command: None },
    HitKeyDoc { panel: "file", prefix: "member-edit:", meaning: "the hand-edit prompt on a family member: member-edit:saveas, member-edit:family (open the family), member-edit:cancel (undo the change)", command: None },
    HitKeyDoc { panel: "file", prefix: "pickfile:", meaning: "the file chooser (any file on this machine, for a panel's Add file…): the explorer's key set under the `pickfile` prefix — pickfile:<file name> (a row), pickfile:confirm, pickfile:path-edit-toggle, …", command: Some("file_pick") },
    HitKeyDoc { panel: "file", prefix: "kicad:", meaning: "the KiCad import: its explorer (kicad:<file name>, kicad:confirm, kicad:upload and the explorer's other keys), then kicad:symbol:<library:name>, kicad:next, kicad:footprint:<library:name>, kicad:use-footprint, kicad:no-footprint, kicad:search, kicad:field:footprints, kicad:field:models, kicad:apply-folders, kicad:import, kicad:cancel", command: None },
];

/// Read `key`'s document from the PLM, for placing it.
fn fetch_document(
    client: std::rc::Rc<crate::plm::client::PlmClient>,
    key: String,
    number: String,
) -> crate::panels::plm_family::Pending<(String, String, String)> {
    crate::panels::plm_family::Pending::new(Box::pin(async move {
        let bytes = client.get_document(&key).await.map_err(|e| e.to_string())?;
        let text = bytes
            .map(|b| String::from_utf8_lossy(&b).into_owned())
            .ok_or_else(|| format!("{key} has no document yet"))?;
        Ok((key, text, number))
    }))
}
