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
//! confirmation now lives (`Mode::ConfirmClose`).
//!
//! Persistence crosses the unified [`ModelStore`] seam — the ONE platform
//! exception. The same embeddable explorer renders on web and desktop. The web
//! backend lists localStorage models and can Upload; desktop lists the app's
//! models directory. No OS-native file dialog is used.

mod plm_save_as;

use crate::automation::hit_keys::HitKeyDoc;
use crate::document::{Document, Documents};
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
    /// Confirm recovery of a missing component source from its embedded part.
    CreateComponentPart,
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
    /// The assembly Save's questions about the documents it owns
    /// ([`crate::document::owned_save`]): one row per document, every row
    /// defaulting to yes ([`FileDialog::owned_prompt`]).
    OwnedSave,
    /// A part was saved under a new name while open assemblies place it:
    /// point them at the new file too? Yes by default ([`FileDialog::retarget`]).
    Retarget,
    /// The Save-file chooser: WHERE an export goes, on desktop. A format
    /// button (or a panel's Export) produced the file; this picks its folder
    /// through the explorer over the local files and its name through a
    /// field, then writes it ([`FileDialog::save_file`]). The browser never
    /// sees it: there an export is a download, and the browser asks.
    SaveFile,
}

/// A file a panel wants handed to the user: the one door every UI export
/// takes ([`FileDialog::request_save_file`]). On desktop it opens the
/// Save-file chooser (`Mode::SaveFile`); in the browser it is a download.
#[derive(Clone, Debug, PartialEq)]
pub struct SaveRequest {
    /// The chooser's heading: "Export STEP", "Export the script".
    pub title: String,
    /// The file's full name, extension included; the chooser's name field
    /// starts from it and the browser downloads under it.
    pub file_name: String,
    pub body: SaveBody,
}

/// An export's content. Text keeps the browser's string lane (its `Blob`
/// takes a string directly, see [`ModelStore::export_file_named`]); on
/// desktop both are bytes on disk.
#[derive(Clone, Debug, PartialEq)]
pub enum SaveBody {
    Text(String),
    Bytes(Vec<u8>),
}

impl SaveBody {
    fn as_bytes(&self) -> &[u8] {
        match self {
            SaveBody::Text(text) => text.as_bytes(),
            SaveBody::Bytes(bytes) => bytes,
        }
    }
}

impl SaveRequest {
    pub fn text(title: impl Into<String>, file_name: impl Into<String>, text: String) -> Self {
        Self { title: title.into(), file_name: file_name.into(), body: SaveBody::Text(text) }
    }

    pub fn bytes(title: impl Into<String>, file_name: impl Into<String>, bytes: Vec<u8>) -> Self {
        Self { title: title.into(), file_name: file_name.into(), body: SaveBody::Bytes(bytes) }
    }
}

/// An export produced and waiting for its folder: the Save-file chooser's
/// payload, with what the write then says. Only the file is held; the
/// dialog that produced it has closed (a format chooser) or never had a
/// modal (a panel's Export button).
struct PendingSave {
    request: SaveRequest,
    /// What the status says after the file name: " (13 files)", " (2 pages)".
    detail: String,
    /// The export's name for its toast ("Netlist", "Fabrication"): the PCB's
    /// manufacturing outputs raise one, named by their output, on every
    /// platform. Every other export's toast on desktop is the generic one.
    label: Option<String>,
    /// The bundle's warnings (design-rule findings): the chooser stays open
    /// on them after the write, as the format dialog did, and they go to a
    /// warning toast.
    warnings: Vec<String>,
    /// A family row was previewed: say so after the write
    /// ([`FileDialog::say_previewed_export`]).
    preview: bool,
}

/// How a produced export left the dialog ([`FileDialog::deliver`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Delivery {
    /// Written (desktop, no chooser) or downloaded (browser): done.
    Done,
    /// The Save-file chooser is up with it; the write happens on its Save.
    Chooser,
    /// Not delivered; the status line says why.
    Failed,
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
    /// Where exports go: the Save-file chooser's folder, on the local files.
    Exports,
}

impl Purpose {
    const ALL: [Purpose; 4] = [Purpose::Models, Purpose::Imports, Purpose::Kicad, Purpose::Exports];

    fn slug(self) -> &'static str {
        match self {
            Purpose::Models => "models",
            Purpose::Imports => "imports",
            Purpose::Kicad => "kicad",
            Purpose::Exports => "exports",
        }
    }
}

impl Mode {
    /// The folder this modal browses, for the modals that browse one.
    fn purpose(self) -> Option<Purpose> {
        match self {
            Mode::Open | Mode::SaveAs | Mode::InsertComponent | Mode::CreateComponentPart => {
                Some(Purpose::Models)
            }
            Mode::Import => Some(Purpose::Imports),
            Mode::Kicad => Some(Purpose::Kicad),
            Mode::SaveFile => Some(Purpose::Exports),
            Mode::ConfirmClose
            | Mode::Export
            | Mode::FlatPattern
            | Mode::StepAssembly
            | Mode::PickMember
            | Mode::SpinOut
            | Mode::MemberEdit
            | Mode::OwnedSave
            | Mode::Retarget
            | Mode::PickFile => None,
        }
    }
}

/// The store a purpose's explorer browses. Import, the KiCad import and the
/// Save-file chooser deal in files on THIS MACHINE: in a PLM session the session
/// store is the server's documents, so they browse the local files the session
/// keeps beside it ([`ModelStore::local_files`]); on a file store they are the
/// same store.
fn browse_store(purpose: Purpose, store: &dyn ModelStore) -> &dyn ModelStore {
    match purpose {
        Purpose::Imports | Purpose::Kicad | Purpose::Exports => store.local_files().unwrap_or(store),
        Purpose::Models => store,
    }
}

/// The store's models folder, where a purpose not yet used starts (the
/// Save-file chooser's exports included). `None` where the store has no
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

/// "Also point these assemblies at the new file?" ([`Mode::Retarget`]).
struct Retarget {
    /// The part's previous name, as the assemblies spell their copies of it.
    old: String,
    new_key: String,
    /// The part as it was saved under the new name.
    text: String,
    tabs: Vec<(u64, String)>,
}

/// A Save waiting for its answers ([`Mode::OwnedSave`]).
struct OwnedPrompt {
    plan: crate::document::owned_save::OwnedSavePlan,
    /// The name the tab is being saved under.
    name: String,
    /// Save As through the explorer (`save_to_browser`), not a plain Save.
    browser: bool,
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
/// The replace-component lineage stamp: a Save As under a NEW identity
/// appends the identity the tab was saved under (and, for a family member,
/// its family) to the document's `derivedFrom` chain. Silent: it is part of
/// what the copy is, not an edit. A never-saved document records nothing.
pub(crate) fn stamp_save_as_lineage(docs: &mut Documents, store: &dyn ModelStore, new_name: &str) {
    let Some(old) = docs.active().name().map(str::to_string) else { return };
    let old = store.canonical_identity(&old);
    if old == store.canonical_identity(new_name) {
        return;
    }
    let engine = docs.tab_engine_mut();
    let mut document = serde_json::json!({});
    for key in [brep_render::engine_state::replace_component::DERIVED_FROM_KEY, family_table::FAMILY_SOURCE_KEY] {
        if let Some(block) = engine.history.document_block(key) {
            document[key] = block.clone();
        }
    }
    brep_render::engine_state::replace_component::stamp_derived_from(&mut document, &old);
    engine.history.set_document_block_no_undo(
        brep_render::engine_state::replace_component::DERIVED_FROM_KEY,
        Some(document[brep_render::engine_state::replace_component::DERIVED_FROM_KEY].clone()),
    );
}

fn adopt_class(engine: &mut EngineState, class: DocumentClass) {
    document_class::set_document_class(engine, class);
    let drop: &[&str] = match class {
        DocumentClass::Normal => &[family_table::FAMILY_TABLE_KEY, template::TEMPLATE_INPUTS_KEY],
        DocumentClass::Family => &[template::TEMPLATE_INPUTS_KEY],
        DocumentClass::Template => &[family_table::FAMILY_TABLE_KEY],
        DocumentClass::Instructions => &[family_table::FAMILY_TABLE_KEY, template::TEMPLATE_INPUTS_KEY],
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
/// A document title as a file stem: path separators and the characters a
/// file name cannot hold become `-`; leading and trailing dots, spaces and
/// dashes go.
fn file_stem_of_title(title: &str) -> String {
    let stem: String = title.chars().map(|c| if c.is_control() || "/\\:*?\"<>|".contains(c) { '-' } else { c }).collect();
    stem.trim_matches(|c: char| c == '.' || c == ' ' || c == '-').to_string()
}

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
    /// On a PLM store a part is an ITEM, not a file at a location: the
    /// browser's `/models/<name>` key is one the PLM cannot hold. The parts
    /// stay embedded and become items when the assembly is saved (the Save's
    /// promotion, `document::owned_save`, at every depth, one item per
    /// content), so nothing is written here and `deferred` counts them.
    plm: bool,
    deferred: usize,
}

impl<'a> StorePartSink<'a> {
    fn new(store: &'a dyn ModelStore, prefix: &str) -> Self {
        Self {
            store,
            prefix: sanitize_file_stem(prefix),
            taken: std::collections::HashSet::new(),
            written: 0,
            failures: Vec::new(),
            plm: store.plm_client().is_some(),
            deferred: 0,
        }
    }
}

impl PartSink for StorePartSink<'_> {
    fn store_part(&mut self, part_name: &str, document_json: &str) -> Option<String> {
        if self.plm {
            self.deferred += 1;
            return None;
        }
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
/// The Save dialog's row for the document being saved itself, when it is a
/// PLM revision this user cannot save yet: check it out (default yes), or
/// `Blocked` when nobody can say yes (checked out by someone else, released).
/// `None` when it can be saved as it is, or is not a PLM revision.
fn top_checkout_row(docs: &Documents, store: &dyn ModelStore, name: &str) -> Option<crate::document::owned_save::Planned> {
    use crate::document::owned_save::{Issue, OwnedWrite, Planned};
    let client = store.plm_client()?;
    let (part, revision) = crate::panels::plm_host::revision_of(name)?;
    let doc = docs.active();
    if matches!(doc.save_access(), crate::document::Access::Editable) && doc.has_plm_save_access() {
        return None;
    }
    let viewer = client.can_author() == Some(false);
    let issue = match store.plm_revision(&crate::plm::identity::document_key(&part, &revision)) {
        Some(entry) if entry.editable && entry.locked_by_me => return None,
        _ if viewer => Issue::Blocked(crate::document::owned_save::VIEW_ONLY.into()),
        Some(entry) if !entry.editable => {
            Issue::Blocked(format!("PLM revision {} is {} and not editable", entry.revision_label, entry.lifecycle))
        }
        Some(entry) if entry.locked_by.is_some() => Issue::Blocked(format!("checked out by {}", entry.locked_by.unwrap_or_default())),
        _ => Issue::CheckOut { part, revision, newer: false },
    };
    let write = OwnedWrite { key: name.to_string(), names: Vec::new(), depth: 0, text: String::new(), synced: String::new(), variants: Vec::new() };
    Some(Planned { write, issue: Some(issue), proceed: true, choice: 0, edited_last: None })
}

/// A checkout or new revision the Save asked for failed AFTER the assembly
/// `id` was written, and its copies were just marked: the stored assembly
/// holds the change without the mark, so the next open (here or on another
/// client) would refresh the copy and lose it. Write the mark into it.
fn write_late_marks(docs: &mut Documents, store: &dyn ModelStore, id: u64, was_clean: bool) {
    if let Err(error) = docs.write_unsaved_marks(store, id, was_clean) {
        docs.engine_mut().push_notice(format!("Save the assembly again: the change it keeps could not be marked in it ({error})"));
    }
}

/// One PLM item to make per distinct new key of `plan`'s promotions:
/// `(new key it replaces, part name, part type)`. The type is the document's
/// own kind, as the PLM import wizard picks it: `assembly` when it places
/// components, else `component`.
fn plm_promotion_requests(current: &serde_json::Value, plan: &crate::document::owned_save::OwnedSavePlan) -> Vec<(String, String, String)> {
    let mut requests: Vec<(String, String, String)> = Vec::new();
    for promotion in &plan.promotions {
        if requests.iter().any(|(key, ..)| *key == promotion.new_key) {
            continue;
        }
        let mut document = current;
        for name in &promotion.names {
            document = &document["partsLibrary"][name]["document"];
        }
        let part = promotion.names.last().cloned().unwrap_or_default();
        requests.push((promotion.new_key.clone(), part, crate::plm::promote::part_type_of(document).to_string()));
    }
    requests
}

/// Every promotion of `plan` takes the PLM key made for its new key.
fn assign_plm_keys(plan: &mut crate::document::owned_save::OwnedSavePlan, keys: &[String]) {
    let mut order: Vec<String> = Vec::new();
    for promotion in &plan.promotions {
        if !order.contains(&promotion.new_key) {
            order.push(promotion.new_key.clone());
        }
    }
    for promotion in &mut plan.promotions {
        if let Some(key) = order.iter().position(|k| *k == promotion.new_key).and_then(|i| keys.get(i)) {
            promotion.new_key = key.clone();
        }
    }
}

pub(crate) fn sanitize_file_stem(name: &str) -> String {
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

mod component_part;

/// A Save's new PLM revisions, in flight ([`FileDialog::poll_owned_revisions`]).
struct OwnedRevisions {
    /// The assembly tab whose copies re-point to them.
    doc: u64,
    /// Deepest first, as planned.
    requests: Vec<crate::document::owned_save::CheckoutRequest>,
    /// Each request's new key and the text written there, or why not.
    future: crate::plm::PlmFuture<Vec<Result<(String, String), String>>>,
}

/// The Save that follows a Save's new revisions ([`FileDialog::poll_owned_checkouts`]).
struct OwnedFollowup {
    doc: u64,
    /// The new revisions' keys: the Save waits until the index has each.
    keys: Vec<String>,
    /// Frames waited for the index; past the cap the Save goes ahead and
    /// asks about what the index still lacks.
    polls: u32,
}

struct PendingInstructionsPdf {
    document: u64,
    name: String,
    preview: bool,
    job: brep_render::engine_state::InstructionsPdfExport,
}

/// The reusable file dialog — transient UI buffers + the current document
/// identity; the model itself lives in `EngineState`'s history.
pub struct FileDialog {
    /// Demand-loaded open, scoped to the requesting tab. A newer open or
    /// navigation cancels it so a delayed response cannot steal focus.
    pending_open: Option<(String, u64)>,
    /// A cold component read belongs to the assembly tab that requested it.
    pending_insert: Option<(String, u64)>,
    component_part: Option<component_part::Recovery>,
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
    /// A step-images or offline-player export held until its geometry or tool
    /// pictures arrive. PDFs use their own incremental job below.
    pending_instructions_export: Option<String>,
    /// PDF preparation is polled between frames; its document and name stay pinned.
    instructions_pdf: Option<PendingInstructionsPdf>,
    /// A mesh file waiting to open its reconstruction preview, with the reader
    /// the picker routed it to (STL or 3MF).
    pending_stl_import: Option<(MeshImportFormat, String, Vec<u8>)>,
    /// Bumped on every SUCCESSFUL store save (plain Save / Save As / native
    /// Save-As) — one half of the update-components staleness key (saving a
    /// part's source must re-check the outdated badges without a reload).
    save_generation: u64,
    plm_save_as: Option<plm_save_as::SaveAs>,
    /// The hand-edit prompt chose "Save as a new part" on a PLM: the Save As
    /// it opens starts on the NEW PART destination (never on overwriting the
    /// generated member or its revision). Consumed when that Save As opens.
    member_new_part: bool,
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
    /// The class Save As writes (its class chooser).
    save_class: DocumentClass,
    /// The armed member picker ([`Mode::PickMember`]).
    pick_member: Option<PickMember>,
    plm_work: Option<PlmWork>,
    /// The armed template spin-out ([`Mode::SpinOut`]).
    spin_out: Option<SpinOut>,
    /// The armed hand-edit prompt ([`Mode::MemberEdit`]).
    member_prompt: Option<MemberPrompt>,
    /// An assembly Save waiting for the answers to its questions
    /// ([`Mode::OwnedSave`]).
    owned_prompt: Option<OwnedPrompt>,
    /// The last assembly Save's owned documents, for automation: the open
    /// question (`prompt`) or what was done (`report`).
    owned_state: serde_json::Value,
    /// PLM items being checked out by an assembly Save; each is written when
    /// its checkout lands.
    owned_checkouts: Vec<(crate::document::owned_save::CheckoutRequest, crate::plm::PlmFuture<()>)>,
    /// The new revisions a Save asked for, made one after another deepest
    /// first (a holder's revision names the revisions made under it): each
    /// request with its new key and written text, or why not.
    owned_revisions: Option<OwnedRevisions>,
    /// The tab a Save made new revisions for: saved again once every request
    /// has landed and the PLM index knows the new revisions, so the documents
    /// holding them name them (and are not taken for missing files).
    owned_followup: Option<OwnedFollowup>,
    /// A Save of a PLM assembly held while the PLM makes an item for each part
    /// that has no document of its own; the answer is each item's key, in the
    /// order of [`plm_promotion_requests`].
    /// A Save held while its PLM items are made, with where they are linked
    /// ("your workspace folder 'X'" or "the top of your workspace").
    /// A Save held while PLM items are made: its prompt, and the items' keys
    /// with the workspace folder they were linked in (`(id, name)`; `None`:
    /// the top).
    #[allow(clippy::type_complexity)]
    owned_promoting: Option<(OwnedPrompt, crate::plm::PlmFuture<(Vec<String>, Option<(String, String)>)>)>,
    /// A Save held until the stored documents it writes have loaded (a PLM
    /// session lists every revision but reads one's bytes only when asked):
    /// `(tab id, name, keys, frames waited)`. Then the Save is planned again.
    owned_loading: Option<(u64, String, Vec<String>, u32)>,
    /// The held Save is being made again: plan it with what loaded, and do
    /// not hold it a second time.
    owned_loaded: bool,
    /// The Save's own document, a PLM revision that was not checked out to
    /// this user: written once its checkout lands. `(tab id, signature it
    /// was saved at, request, checkout)`.
    owned_top: Option<(u64, String, crate::document::owned_save::CheckoutRequest, crate::plm::PlmFuture<()>)>,
    /// The open assemblies that place a part just saved under a new name,
    /// waiting for the answer to "point them at the new file too?".
    retarget: Option<Retarget>,
    /// "Check out without asking again in this session" (the Save dialog's
    /// checkbox): checkout rows no longer ask.
    checkout_remembered: bool,
    /// Tabs whose user chose "Save as a new part": they may keep editing
    /// without being asked again; plain Save still asks.
    member_ack: HashSet<u64>,
    /// The last Generate's report, for the automation blob.
    last_generate: Option<family_table::GenerateReport>,
    /// The export the Save-file chooser holds ([`Mode::SaveFile`]), and the
    /// chooser's own name field — its own, so the Export dialog's stem
    /// (`name_buf`) is what the next export starts from again.
    save_file: Option<PendingSave>,
    save_name: String,
}

impl FileDialog {
    /// A closed dialog with empty buffers. Document identity (name + clean
    /// baseline) lives on [`Document`], so there is nothing to seed here.
    pub fn new() -> Self {
        Self {
            pending_open: None,
            pending_insert: None,
            component_part: None,
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
            pending_instructions_export: None,
            instructions_pdf: None,
            pending_stl_import: None,
            save_generation: 0,
            plm_save_as: None,
            member_new_part: false,
            hits: HashMap::new(),
            kicad: KicadImport::new(),
            folders: HashMap::new(),
            pick_request: None,
            pick_folder: None,
            picked: None,
            plm_new_part: false,
            save_class: DocumentClass::Normal,
            pick_member: None,
            plm_work: None,
            spin_out: None,
            member_prompt: None,
            owned_prompt: None,
            owned_state: serde_json::Value::Null,
            owned_checkouts: Vec::new(),
            owned_revisions: None,
            owned_followup: None,
            owned_promoting: None,
            owned_loading: None,
            owned_loaded: false,
            owned_top: None,
            retarget: None,
            checkout_remembered: false,
            member_ack: HashSet::new(),
            last_generate: None,
            save_file: None,
            save_name: String::new(),
        }
    }

    /// The dialog's own work in flight, for the status bar's working
    /// indicator: a KiCad part's 3D model being parsed off the UI thread.
    pub fn busy_activities(&self) -> Vec<crate::panels::busy::Activity> {
        let mut out = Vec::new();
        if let Some(export) = &self.instructions_pdf {
            let (done, total) = export.job.progress();
            out.push(crate::panels::busy::Activity::new("instructionsPdf", "instructionsPdf", format!("Preparing instructions PDF: {done}/{total} pictures")));
        }
        if self.owned_loading.is_some() {
            out.push(crate::panels::busy::Activity::new("saveReading", "saveReading", "Reading parts from the PLM before saving"));
        }
        if self.kicad.reading() {
            out.push(crate::panels::busy::Activity::new("kicadRead", "kicadRead", "Reading the KiCad part's 3D model"));
        }
        out
    }

    pub fn instructions_pdf_pending(&self) -> bool {
        self.instructions_pdf.is_some()
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
        if matches!(action, FileAction::New | FileAction::NewOfClass(_) | FileAction::Open | FileAction::Import) {
            self.pending_open = None;
            self.pending_insert = None;
        }
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
                // A PLM revision saves like any document: a checkout it
                // needs is a row of the one Save dialog (see `top_checkout_row`).
                // A named document saves straight to its name.
                Some(name) => {
                    let _ = self.save_to(docs, store, name);
                }
                // An unnamed document falls through to Save As.
                None => self.dispatch(FileAction::SaveAs, docs, store),
            },
            FileAction::SaveAs => {
                self.name_buf = Self::proposed_name(docs);
                self.save_class = document_class::document_class(docs.engine());
                self.use_folder(Purpose::Models, store);
                self.open_modal(Mode::SaveAs);
            }
            FileAction::Import => {
                self.use_folder(Purpose::Imports, browse_store(Purpose::Imports, store));
                self.open_modal(Mode::Import);
            }
            // The stem without the class extension, like Save As: a family's
            // STEP is `bolt-family.step`, not `bolt-family.fbrep.step`.
            FileAction::Export => {
                self.name_buf = Self::proposed_name(docs);
                self.open_modal(Mode::Export);
            }
            FileAction::ExportFlatPattern => {
                self.name_buf = Self::proposed_name(docs);
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
        self.pending_insert = None;
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
        self.poll_pending_open(ctx, docs, store);
        self.poll_pending_insert(ctx, docs, store);
        self.poll_plm_save_as(ctx, docs, store);
        self.poll_plm_work(ctx, docs, store);
        self.poll_open_workspace_pick(store);
        self.poll_owned_checkouts(ctx, docs, store);
        self.poll_owned_promotions(ctx, docs, store);
        self.poll_owned_loading(ctx, docs, store);
        if self.retarget.is_some() && !self.open {
            self.open_modal(Mode::Retarget);
        }
        // The browser's file chooser answered a pick.
        if let Some(file) = store.take_picked_file() {
            if let Some((tag, _)) = self.pick_request.take() {
                self.picked = Some((tag, PickedFile { name: file.name, path: None, bytes: file.bytes }));
            }
        }

        // A STEP probe the runner has answered since last frame resolves into
        // the assembly choice or the flat import now, before drawing.
        self.poll_step_probe(docs.engine_mut());
        self.poll_instructions_pdf(ctx, docs, store);
        // An instructions export held for the exact solids runs once they
        // have landed (the busy line says "Loading exact geometry" meanwhile).
        if self.pending_instructions_export.is_some() && !docs.engine().topology_pending() && crate::plm::tool_pictures::pending() == 0 {
            if let Some(format) = self.pending_instructions_export.take() {
                self.export_as(docs, store, &format);
            }
        }

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
                    Ok(contents) => self.load_document(docs, &name, &contents, store),
                    Err(_) => self.status = format!("open failed: {name} is not UTF-8 text"),
                }
            }
            self.close_after_import();
        }

        // A family member the user has just started to change raises the
        // hand-edit prompt (unless they already chose "Save as a new part").
        // On every store (integrator ruling 2026-10-03): on a PLM the
        // prompt's "Save as a new part" hands off to the PLM Save As with a
        // NEW PART preselected, so the generated member is never written
        // over by hand.
        if !self.open {
            self.watch_member_edit(docs);
        }

        if !self.open {
            return;
        }
        self.remember_folder(store);

        match self.mode {
            Mode::CreateComponentPart => self.show_create_component_part(ctx, docs, store),
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
            Mode::OwnedSave => self.show_owned_save(ctx, docs, store),
            Mode::Retarget => self.show_retarget(ctx, docs),
            Mode::PickFile => self.show_pick_file(ctx, store),
            Mode::SaveFile => self.show_save_file(ctx, docs, store),
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
                        egui::TextEdit::singleline(&mut self.name_buf).margin(crate::viewport::touch::text_edit_margin(ui)).min_size(crate::viewport::touch::target_size(ui.ctx(), egui::Vec2::ZERO))
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
                        // One dropdown (the form's enum field): saveas:class, its
                        // open entries saveas:class#<slug>.
                        let texts: Vec<String> = DocumentClass::ALL.iter().map(|class| format!("{} ({})", class.label(), class.extension())).collect();
                        let choices: Vec<(DocumentClass, &str, &str)> =
                            DocumentClass::ALL.iter().zip(&texts).map(|(class, text)| (*class, class.slug(), text.as_str())).collect();
                        crate::form::enum_dropdown(ui, "saveas:class", &choices, &mut self.save_class, None, Some(&mut self.hits));
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

    /// The **Export** chooser: pick a format (STEP / IGES / STL / OBJ / GLB /
    /// 3MF) and write the current model to the user's filesystem under
    /// `<name>.<ext>` through the store's format-typed interchange. STEP and
    /// IGES serialize the exact NURBS topology; STL, OBJ, GLB and 3MF are the
    /// display mesh (ASCII facets, one indexed object, binary glTF 2.0 with a
    /// mesh per body, and a 3MF package with an object per body and the
    /// document's colours as base materials).
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
        let has_instructions = docs.engine().is_instructions();
        // The board's manufacturing files: offered once the PCB has parts on
        // its board, the BOM row's gating for the same reason.
        let has_board = super::fabrication_export::has_board(docs.engine());
        let has_schematic = super::fabrication_export::has_schematic(docs.engine());
        let mut chosen: Option<&'static str> = None;
        let mut bom_chosen: Option<&'static str> = None;
        let mut fab_chosen: Option<super::fabrication_export::Output> = None;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-export")).show(ctx, |ui| {
            // Wide enough for the 3D formats on one line; each group wraps
            // rather than widening the modal (lane U audit item 7).
            ui.set_width(440.0);
            ui.heading("Export model");
            ui.add_space(4.0);
            Self::preview_export_line(ui, docs);
            let field = ui.add(
                egui::TextEdit::singleline(&mut self.name_buf).margin(crate::viewport::touch::text_edit_margin(ui)).min_size(crate::viewport::touch::target_size(ui.ctx(), egui::Vec2::ZERO))
                    .hint_text("file name")
                    .desired_width(f32::INFINITY),
            );
            self.hit("field:name", &field);
            if self.want_focus {
                field.request_focus();
                self.want_focus = false;
            }
            // One headed, wrapping row per kind of output, and Cancel on its
            // own row below them all — it used to sit among the 3D formats,
            // with "Sheets + 3D (PDF)" orphaned on a line of its own.
            let group = |ui: &mut egui::Ui, title: &str| {
                ui.add_space(6.0);
                ui.label(egui::RichText::new(title).weak());
            };
            group(ui, "3D model");
            ui.horizontal_wrapped(|ui| {
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
                // The slicer format: the display mesh as a 3MF package, one
                // object per body, with the document's colours carried as
                // base materials so a multi-colour print arrives painted.
                let three_mf = ui
                    .button("3MF (.3mf)")
                    .on_hover_text("For slicers: one object per body, with the model's colours");
                self.hit("export:3mf", &three_mf);
                if three_mf.clicked() {
                    chosen = Some("3mf");
                }
                // The full model RECIPE (`.nbrep`) — for saving a document to
                // disk / sharing a failing model for a bug report. Re-openable via
                // Open / Import.
                let json = ui.button("Model (.nbrep)");
                self.hit("export:json", &json);
                if json.clicked() {
                    chosen = Some("json");
                }
            });
            group(ui, "Bill of materials");
            ui.horizontal_wrapped(|ui| {
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
            });
            group(ui, "Drawing");
            ui.horizontal_wrapped(|ui| {
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
            group(ui, "Electronics");
            ui.horizontal_wrapped(|ui| {
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
            group(ui, "Instructions");
            ui.horizontal_wrapped(|ui| {
                // The printed instructions on the page template the Instructions
                // pane sets, and every step's picture as PNG and SVG in one zip.
                let not_instructions = "Open manufacturing instructions first (New from assembly in the Instructions workbench)";
                let pdf = ui
                    .add_enabled(has_instructions, egui::Button::new("PDF"))
                    .on_hover_text("The cover, the parts list, the tools list and the steps, on the page template set in the Instructions pane")
                    .on_disabled_hover_text(not_instructions);
                self.hit("export:instructionspdf", &pdf);
                if pdf.clicked() {
                    chosen = Some("instructionspdf");
                }
                let images = ui
                    .add_enabled(has_instructions, egui::Button::new("Step images"))
                    .on_hover_text("Every step's picture as PNG and SVG, in one zip")
                    .on_disabled_hover_text(not_instructions);
                self.hit("export:instructionsimages", &images);
                if images.clicked() {
                    chosen = Some("instructionsimages");
                }
                let html = ui
                    .add_enabled(has_instructions, egui::Button::new("Player (HTML)"))
                    .on_hover_text("The interactive player in one self-contained page: Next/Back, a live 3D view per step and a step checklist, with no server")
                    .on_disabled_hover_text(not_instructions);
                self.hit("export:instructionshtml", &html);
                if html.clicked() {
                    chosen = Some("instructionshtml");
                }
            });
            if !self.status.is_empty() {
                ui.add_space(4.0);
                ui.weak(&self.status);
            }
            ui.add_space(6.0);
            ui.separator();
            let c = ui.button("Cancel");
            self.hit("cancel", &c);
            if c.clicked() {
                cancel = true;
            }
        });
        // A delivered export closes this dialog; one the Save-file chooser
        // took over has switched the modal to the chooser, which stays up.
        if let Some(format) = chosen {
            if self.export_as(docs, store, format) && self.mode == Mode::Export {
                self.open = false;
            }
        } else if let Some(format) = bom_chosen {
            if self.export_bom_as(docs, store, format) && self.mode == Mode::Export {
                self.open = false;
            }
        } else if let Some(output) = fab_chosen {
            if self.export_fabrication_as(docs, store, output) && self.mode == Mode::Export {
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
                egui::TextEdit::singleline(&mut self.name_buf).margin(crate::viewport::touch::text_edit_margin(ui)).min_size(crate::viewport::touch::target_size(ui.ctx(), egui::Vec2::ZERO))
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
            if self.export_flat_pattern_as(docs, store, format) && self.mode == Mode::FlatPattern {
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
    /// part's document. Workspace changes are linked to the web app; downloads
    /// go out through the store's export lane.
    fn show_open_workspace(
        &mut self,
        ctx: &egui::Context,
        docs: &mut Documents,
        store: &dyn ModelStore,
        client: std::rc::Rc<crate::plm::client::PlmClient>,
    ) {
        use crate::panels::plm_workspace::{document_name, load_pins, save_pins, PlmWorkspaces};
        let plm = PlmWorkspaces { client };
        if self.workspace.pins.is_none() {
            self.workspace.pins = Some(load_pins(store));
        }
        self.workspace.browse_only = true;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-open")).show(ctx, |ui| {
            file_explorer::dialog_body(ui, |ui| {
                ui.heading("Open from the PLM");
                if let Some(url) = crate::panels::plm_host::web_url(store, "#/workspace") {
                    crate::panels::plm_host::web_link(ui, "Manage workspace in PLM", url);
                }
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
        let library = docs.engine_mut().parts_library_names();
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
            match docs.engine_mut().insert_component(ComponentInsert::Existing { part_name: &part_name }) {
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
            self.request_component_document(docs, store, &name);
            if self.pending_insert.is_some() {
                ctx.request_repaint_after(std::time::Duration::from_millis(100));
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

    fn request_component_document(&mut self, docs: &mut Documents, store: &dyn ModelStore, name: &str) {
        self.pending_insert = None;
        match crate::store::read_now(store, name) {
            crate::store::ReadNow::Ready(contents) => self.finish_component_document(docs, store, name, &contents),
            crate::store::ReadNow::Loading => {
                self.pending_insert = Some((name.to_owned(), docs.active_id()));
                self.status = format!("'{name}' is still loading");
            }
            crate::store::ReadNow::Absent => self.status = format!("insert failed: '{name}' not found"),
        }
    }

    fn finish_component_document(&mut self, docs: &mut Documents, store: &dyn ModelStore, name: &str, contents: &str) {
        let assembly = docs.active().name().map(str::to_string);
        if !self.begin_class_insert(store, name, contents, assembly) {
            self.insert_component_document(docs.engine_mut(), name, contents);
            self.open = false;
        }
    }

    fn poll_pending_insert(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        let Some((name, requester)) = self.pending_insert.take() else { return };
        if !self.open || self.mode != Mode::InsertComponent || docs.active_id() != requester {
            return;
        }
        match crate::store::read_now(store, &name) {
            crate::store::ReadNow::Ready(contents) => self.finish_component_document(docs, store, &name, &contents),
            crate::store::ReadNow::Loading => {
                self.pending_insert = Some((name, requester));
                ctx.request_repaint_after(std::time::Duration::from_millis(100));
            }
            crate::store::ReadNow::Absent => self.status = format!("insert failed: '{name}' not found"),
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
    /// document's display name, else its untitled name (`"untitled"`, `"untitled 2"`, …).
    fn effective_name(&self, docs: &Documents) -> String {
        let field = self.name_buf.trim();
        if !field.is_empty() {
            field.to_string()
        } else {
            Self::proposed_name(docs)
        }
    }

    /// The name Save As and every export start from: the active document's
    /// stem, or, for an instructions tab not saved yet, the instructions'
    /// own title (`valve`), so its player, PDF and pictures are
    /// `valve.html`, `valve.pdf`, … rather than `untitled 2.html`.
    fn proposed_name(docs: &Documents) -> String {
        if docs.active().name().is_none() {
            if let Ok(Some(block)) = docs.engine().instructions() {
                let title = file_stem_of_title(&block.front.title);
                if !title.is_empty() {
                    return title;
                }
            }
        }
        document_class::strip_class_extension(&Self::document_name(docs)).to_string()
    }

    /// The ACTIVE document's display name, what Save As and Export start
    /// their name field from: the file's stem, never the full path a saved
    /// document is named by (whose `/`s became `_`s in every exported file
    /// name), and never the field's last contents, which belong to whichever
    /// tab last used it.
    fn document_name(docs: &Documents) -> String {
        docs.active().name().map(model_display_name).unwrap_or_else(|| docs.active().untitled_title())
    }

    /// **New** — a fresh document in a NEW TAB. Nothing is replaced, so there is
    /// nothing to confirm.
    fn new_document(&mut self, docs: &mut Documents, class: DocumentClass) {
        docs.open_document(Document::fresh(docs.spawn_engine(), class));
        self.name_buf.clear();
        self.status = match class {
            DocumentClass::Normal => "new model".into(),
            _ => format!("new {}", class.label().to_ascii_lowercase()),
        };
    }

    /// Refuse a save that would give TWO open tabs the same store identity —
    /// "focus the tab holding this document" has no answer then, and the second
    /// save would silently overwrite the first tab's file. `true` = go ahead.
    ///
    /// PLM documents compare their full part/revision key in either spelling.
    /// Local documents compare DISPLAY names: on desktop an open document
    /// carries the full path it was loaded from while the Save As field holds a
    /// bare name, so an identity compare would never match and the clobber would
    /// happen before anything noticed. Two same-stemmed files in different
    /// folders are refused too — stricter than strictly necessary, and the side
    /// to err on when the alternative is overwriting another tab's file.
    fn name_is_free(&mut self, docs: &mut Documents, name: &str) -> bool {
        let display = model_display_name(name);
        let target_revision = crate::plm::uses::revision_key_in(name);
        let taken = docs.iter().enumerate().any(|(index, doc)| {
            index != docs.active_index()
                && doc
                    .name()
                    .is_some_and(|open| {
                        match (&target_revision, crate::plm::uses::revision_key_in(open)) {
                            (Some(target), Some(other)) => *target == other,
                            (None, None) => model_display_name(open) == display,
                            _ => false,
                        }
                    })
        });
        if taken {
            let identity = target_revision.as_deref().unwrap_or(&display);
            self.status = format!("Save refused: '{identity}' is already open in another tab");
            docs.engine_mut().push_notice(&self.status);
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
        // A Work part edited in place: Save writes the ASSEMBLY (its tab),
        // with the part's latest state pushed into it first.
        docs.flush_work_part();
        let mut plan = docs.plan_owned_save(store, &name);
        // A document the Save writes whose bytes have not been read yet (the
        // plan's read asked for them): wait for them rather than refuse it
        // "still loading", which left a held part unsaved on the first Save.
        if !std::mem::take(&mut self.owned_loaded) {
            let loading: Vec<String> = plan
                .rows
                .iter()
                .map(|row| row.write.key.clone())
                .filter(|key| store.residency(key) == crate::store::Residency::OnDisk)
                .collect();
            for key in &loading {
                // A row the plan settled without its bytes (a checkout) asked
                // for none: ask now, so the wait is only as long as the load.
                let _ = store.read(key);
            }
            if !loading.is_empty() {
                self.status = format!("Reading {} part(s) from the PLM before saving…", loading.len());
                self.owned_loading = Some((docs.active_id(), name, loading, 0));
                return false;
            }
        }
        match top_checkout_row(docs, store, &name) {
            Some(row) if matches!(row.issue, Some(crate::document::owned_save::Issue::Blocked(_))) => {
                // The revision this tab is cannot take the Save (another user
                // holds it, or it is released): the changes exist only in the
                // tab, so offer the way to keep them instead of a dead end (owner
                // rule: prompts that default to proceed). Held by someone else,
                // the Save As starts on a new part, as a new revision would sit
                // beside their draft; not editable, on its next revision.
                let Some(crate::document::owned_save::Issue::Blocked(why)) = &row.issue else { unreachable!() };
                // A viewer has nowhere on the PLM to save to: Save As would
                // offer destinations that are all refused.
                if why == crate::document::owned_save::VIEW_ONLY {
                    self.status = format!(
                        "{} is not saved: {why}. Your changes are only in this tab: Export keeps a copy as a file.",
                        docs.active().title()
                    );
                    docs.engine_mut().push_notice_as(brep_render::engine_state::NoticeSeverity::Warning, self.status.clone());
                    return false;
                }
                let held = why.starts_with("checked out by");
                self.status = format!(
                    "{} is not saved: {why}. Your changes are only in this tab: Save As keeps them as {}, or Save again once it can be written.",
                    docs.active().title(),
                    if held { "a new part" } else { "a new revision" }
                );
                docs.engine_mut().push_notice_as(brep_render::engine_state::NoticeSeverity::Warning, self.status.clone());
                let status = self.status.clone();
                self.member_new_part = held;
                self.dispatch(FileAction::SaveAs, docs, store);
                self.status = status;
                return false;
            }
            Some(row) => plan.rows.insert(0, row),
            None => {}
        }
        if self.hold_for_plm_items(docs, store, plan.clone(), &name, false) {
            return false;
        }
        if self.asks(&plan) {
            self.ask_owned_save(plan, name, false);
            return false;
        }
        self.commit_save(docs, store, name, plan)
    }

    /// Whether `plan` needs the Save dialog: a question, unless the only
    /// questions are checkouts the user said not to ask about again.
    fn asks(&self, plan: &crate::document::owned_save::OwnedSavePlan) -> bool {
        use crate::document::owned_save::Issue;
        plan.rows
            .iter()
            .any(|row| row.asks() && !(self.checkout_remembered && matches!(row.issue, Some(Issue::CheckOut { newer: false, .. }))))
    }

    /// A PLM assembly's parts with no document get PLM items, not files: start
    /// making them and hold the Save until they exist (`true`).
    fn hold_for_plm_items(&mut self, docs: &Documents, store: &dyn ModelStore, plan: crate::document::owned_save::OwnedSavePlan, name: &str, browser: bool) -> bool {
        let Some(client) = store.plm_client() else { return false };
        if plan.promotions.is_empty() || crate::panels::plm_host::revision_of(name).is_none() {
            return false;
        }
        let current: serde_json::Value = serde_json::from_str(&docs.tab_engine().history_request_json()).unwrap_or_default();
        let requests = plm_promotion_requests(&current, &plan);
        let count = requests.len();
        // Linked into the workspace folder the assembly was opened from;
        // opened any other way (by number, from a search), the one folder of
        // the user's workspace that links it; else the top of the workspace.
        let opened = docs.active().plm_folder().cloned();
        let part = crate::panels::plm_host::revision_of(name).map(|(part, _)| part);
        let future = async move {
            let folder = match (opened, part) {
                (Some(folder), _) => Some(folder),
                (None, Some(part)) => crate::plm::promote::linking_folder(&client, &part).await.filter(|(id, _)| !id.is_empty()),
                (None, None) => None,
            };
            let keys = crate::plm::promote::make_items(client, requests, folder.as_ref().map_or(String::new(), |(id, _)| id.clone())).await?;
            Ok((keys, folder))
        };
        self.owned_promoting = Some((OwnedPrompt { plan, name: name.to_string(), browser }, Box::pin(future)));
        self.status = format!("Making PLM items for {count} part(s) with no document of their own…");
        true
    }

    /// Continue a Save held until its documents loaded: plan it again once
    /// none is waiting, or after ~20 s (the Save then says which part is still
    /// loading, as before). A load that failed is asked again every ~2 s.
    fn poll_owned_loading(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        const LOAD_WAIT_FRAMES: u32 = 400;
        let Some((_, _, keys, frames)) = self.owned_loading.as_mut() else { return };
        if *frames < LOAD_WAIT_FRAMES && keys.iter().any(|key| store.residency(key) == crate::store::Residency::OnDisk) {
            *frames += 1;
            if *frames % 40 == 0 {
                for key in keys.iter() {
                    let _ = store.read(key);
                }
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(50));
            return;
        }
        let Some((doc, name, _, _)) = self.owned_loading.take() else { return };
        if doc != docs.active_id() {
            self.status = "Save cancelled because the active document changed".into();
            return;
        }
        self.owned_loaded = true;
        self.save_to(docs, store, name);
    }

    /// Continue a Save held by [`Self::hold_for_plm_items`] once its items
    /// exist; a failure saves nothing and says why (never into the assembly).
    fn poll_owned_promotions(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        let Some((_, future)) = self.owned_promoting.as_mut() else { return };
        let mut cx = std::task::Context::from_waker(std::task::Waker::noop());
        let std::task::Poll::Ready(result) = std::future::Future::poll(future.as_mut(), &mut cx) else {
            ctx.request_repaint_after(std::time::Duration::from_millis(50));
            return;
        };
        let Some((mut prompt, _)) = self.owned_promoting.take() else { return };
        if prompt.plan.doc_id != docs.active_id() {
            self.status = "Save cancelled because the active document changed".into();
            return;
        }
        match result {
            Ok((keys, folder)) => {
                let place = folder.map_or_else(|| "the top of your workspace".to_string(), |(_, name)| format!("your workspace folder '{name}'"));
                docs.engine_mut().push_notice_as(
                    brep_render::engine_state::NoticeSeverity::Info,
                    format!("Made {} PLM item(s) for parts with no document, linked in {place}", keys.len()),
                );
                assign_plm_keys(&mut prompt.plan, &keys);
                if self.asks(&prompt.plan) {
                    self.ask_owned_save(prompt.plan, prompt.name, prompt.browser);
                } else if prompt.browser {
                    self.commit_save_browser(docs, store, prompt.name, prompt.plan);
                } else {
                    self.commit_save(docs, store, prompt.name, prompt.plan);
                }
            }
            Err(error) => {
                self.status = format!("Save stopped: the PLM could not make an item for a part with no document: {error}");
                docs.engine_mut().push_notice(&self.status);
            }
        }
    }

    /// Hold a Save until the user answers its questions about the documents
    /// the assembly owns.
    fn ask_owned_save(&mut self, plan: crate::document::owned_save::OwnedSavePlan, name: String, browser: bool) {
        self.owned_state = serde_json::json!({"prompt": plan.to_json()});
        self.owned_prompt = Some(OwnedPrompt { plan, name, browser });
        self.open_modal(Mode::OwnedSave);
    }

    /// Write the documents the assembly owns (with the user's answers), then
    /// return the Save summary to fold into the status line.
    fn save_owned(&mut self, docs: &mut Documents, store: &dyn ModelStore, plan: crate::document::owned_save::OwnedSavePlan) -> String {
        let (report, checkouts) = docs.execute_owned_save(store, plan);
        if let Some(client) = store.plm_client() {
            use crate::panels::plm::PlmLifecycle;
            let (revisions, checkouts): (Vec<_>, Vec<_>) = checkouts.into_iter().partition(|r| r.new_revision);
            for request in checkouts {
                let future = client.checkout(&request.part, &request.revision);
                self.owned_checkouts.push((request, future));
            }
            if !revisions.is_empty() {
                // Deepest first, as the Save planned them: each text names the
                // revisions made before it.
                let client = client.clone();
                let requests = revisions.clone();
                let future: crate::plm::PlmFuture<Vec<Result<(String, String), String>>> = Box::pin(async move {
                    let mut done: Vec<(String, String, String)> = Vec::new();
                    let mut answers = Vec::new();
                    for request in &requests {
                        let text = crate::document::owned_save::retarget_text(&request.text, &done);
                        match crate::panels::plm_parts::save_as_new_revision(&client, &request.part, "", &text).await {
                            Ok(new_key) => {
                                done.push((request.key.clone(), new_key.clone(), text.clone()));
                                answers.push(Ok((new_key, text)));
                            }
                            Err(why) => answers.push(Err(why)),
                        }
                    }
                    Ok(answers)
                });
                self.owned_revisions = Some(OwnedRevisions { doc: docs.active_id(), requests: revisions, future });
            }
        }
        self.owned_state = serde_json::json!({"report": report.to_json()});
        let summary = report.summary();
        if !summary.is_empty() {
            docs.engine_mut().push_notice_as(brep_render::engine_state::NoticeSeverity::Info, format!("Save: {summary}"));
        }
        summary
    }

    /// The new revisions a Save asked for, once their chain has answered:
    /// each one's copies re-point to it (in order, so a holder's re-point
    /// keeps the ones beneath it), a failed one stays marked.
    fn poll_owned_revisions(&mut self, docs: &mut Documents, store: &dyn ModelStore) {
        let Some(revisions) = self.owned_revisions.as_mut() else { return };
        let mut cx = std::task::Context::from_waker(std::task::Waker::noop());
        let std::task::Poll::Ready(result) = std::future::Future::poll(revisions.future.as_mut(), &mut cx) else { return };
        let OwnedRevisions { doc, requests, .. } = self.owned_revisions.take().unwrap();
        let answers = result.unwrap_or_else(|why| requests.iter().map(|_| Err(why.clone())).collect());
        for (request, answer) in requests.iter().zip(answers) {
            match answer {
                Ok((new_key, text)) => match docs.retarget_revised_copies(doc, &request.key, &new_key, &text) {
                    Ok(()) => {
                        // As Save As → new revision: the mirror holds it now and
                        // the index learns it, or the next Save reads it absent.
                        store.remember_saved_document(&store.canonical_identity(&new_key), &text);
                        store.refresh_plm_index(std::slice::from_ref(&new_key), false);
                        docs.engine_mut().push_notice_as(
                            brep_render::engine_state::NoticeSeverity::Info,
                            format!(
                                "Made a new revision of {} and saved the change into it: {}",
                                request.shown,
                                crate::document::owned_save::shown(&new_key, &[])
                            ),
                        );
                        let followup = self.owned_followup.get_or_insert(OwnedFollowup { doc, keys: Vec::new(), polls: 0 });
                        followup.keys.push(new_key);
                    }
                    Err(why) => docs.engine_mut().push_notice(format!("'{new_key}' holds the change, but the assembly could not name it: {why}")),
                },
                Err(why) => {
                    docs.engine_mut().push_notice(format!("{} NOT saved: {why} — the change is kept in the assembly", request.shown));
                    let was_clean = docs.iter().find(|d| d.id() == doc).is_some_and(|d| !d.is_dirty());
                    if docs.mark_unsaved_copies(doc, &request.key, &format!("a new revision could not be made: {why}")) {
                        write_late_marks(docs, store, doc, was_clean);
                    }
                }
            }
        }
    }

    /// Write the PLM items whose checkout landed since last frame.
    fn poll_owned_checkouts(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        if let Some((_, _, _, future)) = self.owned_top.as_mut() {
            let mut cx = std::task::Context::from_waker(std::task::Waker::noop());
            match std::future::Future::poll(future.as_mut(), &mut cx) {
                std::task::Poll::Ready(result) => {
                    if let Some((id, signature, request, _)) = self.owned_top.take() {
                        let saved = crate::document::owned_save::finish_checkout(docs.engine_mut(), store, &request, result);
                        if saved {
                            if let Some(doc) = docs.iter_mut().find(|d| d.id() == id) {
                                doc.mark_saved_signature(signature);
                                doc.set_plm_save_access(crate::document::Access::Editable);
                            }
                            self.save_generation += 1;
                            remember_document(store, &request.key);
                            self.status = format!("saved {}", request.key);
                        } else {
                            self.status = format!("{} NOT saved: the checkout or the write failed; your edits are kept", request.key);
                        }
                    }
                }
                std::task::Poll::Pending => ctx.request_repaint_after(std::time::Duration::from_millis(50)),
            }
        }
        self.poll_owned_revisions(docs, store);
        if self.owned_checkouts.is_empty() && self.owned_revisions.is_none() {
            // The follow-up Save also waits for the assembly's own checkout.
            if self.owned_top.is_some() {
                return;
            }
            if let Some(followup) = self.owned_followup.as_mut() {
                // ~10 s at the 50 ms repaint below.
                const INDEX_WAIT_POLLS: u32 = 200;
                if followup.polls < INDEX_WAIT_POLLS && followup.keys.iter().any(|key| store.plm_revision(key).is_none()) {
                    followup.polls += 1;
                    ctx.request_repaint_after(std::time::Duration::from_millis(50));
                    return;
                }
            }
            if let Some(OwnedFollowup { doc: id, .. }) = self.owned_followup.take() {
                // Every new revision landed: save the assembly again so the
                // documents holding them (and the assembly itself) name them.
                if docs.active_id() == id {
                    if let Some(name) = docs.active().name().map(str::to_string) {
                        self.save_to(docs, store, name);
                    }
                } else {
                    docs.engine_mut().push_notice("Save the assembly again: it names the new revisions now");
                }
            }
            return;
        }
        let mut cx = std::task::Context::from_waker(std::task::Waker::noop());
        let mut i = 0;
        while i < self.owned_checkouts.len() {
            match std::future::Future::poll(self.owned_checkouts[i].1.as_mut(), &mut cx) {
                std::task::Poll::Ready(result) => {
                    let (request, _) = self.owned_checkouts.remove(i);
                    let failure = result.as_ref().err().cloned();
                    let id = docs.active_id();
                    if crate::document::owned_save::finish_checkout(docs.engine_mut(), store, &request, result) {
                        if docs.clear_unsaved_copies(id, &request.key) {
                            // A change kept by an earlier Save is written now; the
                            // assembly was written before it landed and still marks
                            // it: save the assembly again, as for new revisions.
                            self.owned_followup.get_or_insert(OwnedFollowup { doc: id, keys: Vec::new(), polls: 0 });
                        }
                    } else {
                        // Kept in the assembly: marked, so it reads unsaved, the
                        // next Save asks again and no refresh replaces it.
                        let why = failure.unwrap_or_else(|| "the write after the checkout failed".into());
                        let was_clean = docs.iter().find(|d| d.id() == id).is_some_and(|d| !d.is_dirty());
                        if docs.mark_unsaved_copies(id, &request.key, &format!("checkout failed: {why}")) {
                            write_late_marks(docs, store, id, was_clean);
                        }
                    }
                }
                std::task::Poll::Pending => i += 1,
            }
        }
        if !self.owned_checkouts.is_empty() || self.owned_revisions.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(50));
        }
    }

    /// After the owned documents: a part that should have been given a file
    /// of its own and could not be stops the Save BEFORE the assembly is
    /// written — written now, it would keep that part only inside the
    /// assembly. Everything else already saved stays saved, the tab stays
    /// unsaved, and Save can simply be pressed again.
    fn promotion_failed(&mut self, docs: &mut Documents) -> bool {
        let failed: Vec<String> = self.owned_state["report"]["failedPromotions"]
            .as_array()
            .map(|keys| keys.iter().filter_map(|k| k.as_str().map(str::to_string)).collect())
            .unwrap_or_default();
        if failed.is_empty() {
            return false;
        }
        self.status = format!(
            "Save stopped: could not create {} — the assembly was not written, so no part is kept only inside it. Fix the problem and Save again.",
            failed.join(", ")
        );
        docs.engine_mut().push_notice(&self.status);
        true
    }

    /// The Save itself, once nothing is left to ask: the owned documents
    /// first (deepest first), then the active tab.
    fn commit_save(&mut self, docs: &mut Documents, store: &dyn ModelStore, name: String, plan: crate::document::owned_save::OwnedSavePlan) -> bool {
        use crate::document::owned_save::{CheckoutRequest, Issue};
        let top = plan.rows.first().filter(|row| row.write.key == name).cloned();
        if let Some(row) = &top {
            if !row.proceed {
                self.status = format!("Save cancelled: {name} was not checked out");
                return false;
            }
        }
        let summary = self.save_owned(docs, store, plan);
        if self.promotion_failed(docs) {
            return false;
        }
        let document = docs.tab_engine().history_request_json();
        if let (Some(row), Some(client)) = (top, store.plm_client()) {
            if let Some(Issue::CheckOut { part, revision, .. }) = row.issue {
                use crate::panels::plm::PlmLifecycle;
                let signature = docs.active().save_signature();
                let future = client.checkout(&part, &revision);
                let shown = crate::document::owned_save::shown(&name, &[]);
                let request = CheckoutRequest { key: name.clone(), part, revision, text: document, shown: shown.clone(), new_revision: false };
                self.owned_top = Some((docs.active_id(), signature, request, future));
                self.status = if summary.is_empty() {
                    format!("checking out and saving {shown}")
                } else {
                    format!("checking out and saving {shown}; {summary}")
                };
                return true;
            }
        }
        crate::plm::thumbnail::stage_save(store, &name, &document, docs.tab_engine(), docs.active_id());
        match store.write(&name, &document) {
            Ok(()) => {
                // The document keeps the raw identity (a full path when a
                // native dialog chose it); the name field shows the bare name.
                self.name_buf = model_display_name(&name);
                let doc = docs.active_mut();
                doc.set_name(Some(name.clone()));
                doc.mark_clean();
                self.save_generation += 1;
                remember_document(store, &name);
                self.status = if summary.is_empty() { format!("saved {name}") } else { format!("saved {name}; {summary}") };
                true
            }
            Err(e) => {
                self.status = format!("save failed: {e}");
                docs.engine_mut().push_notice(&self.status);
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
        docs.flush_work_part();
        let plan = docs.plan_owned_save(store, &name);
        if self.hold_for_plm_items(docs, store, plan.clone(), &name, true) {
            return false;
        }
        if plan.needs_asking() {
            self.ask_owned_save(plan, name, true);
            return false;
        }
        self.commit_save_browser(docs, store, name, plan)
    }

    /// [`Self::save_to_browser`] once nothing is left to ask.
    fn commit_save_browser(&mut self, docs: &mut Documents, store: &dyn ModelStore, name: String, plan: crate::document::owned_save::OwnedSavePlan) -> bool {
        let summary = self.save_owned(docs, store, plan);
        if self.promotion_failed(docs) {
            return false;
        }
        let class = DocumentClass::of_name(&name).unwrap_or_default();
        adopt_class(docs.tab_engine_mut(), class);
        // The copy remembers what it was saved from (before the family stamp
        // goes: the chain keeps the family too), so Replace component knows
        // an edited variation is still that part's relative.
        stamp_save_as_lineage(docs, store, &name);
        if docs.tab_engine().history.document_block(family_table::FAMILY_SOURCE_KEY).is_some() {
            docs.tab_engine_mut()
                .history
                .set_document_block_no_undo(family_table::FAMILY_SOURCE_KEY, None);
        }
        let previous = docs.active().name().map(str::to_string);
        let saved_text = docs.tab_engine().history_request_json();
        match store.browser_write(&name, &saved_text) {
            Ok(identity) => {
                // Open assemblies that place this part under its OLD name:
                // ask once whether they follow it to the new file.
                if let Some(old) = previous.filter(|old| store.canonical_identity(old) != store.canonical_identity(&identity)) {
                    let tabs = docs.tabs_holding(&old, docs.active_id());
                    if !tabs.is_empty() {
                        self.retarget = Some(Retarget { old, new_key: identity.clone(), text: saved_text.clone(), tabs });
                    }
                }
                // The recent list's picture, under the identity the backend
                // chose (the name the recent row will carry).
                crate::plm::thumbnail::stage_recent(&identity, docs.tab_engine(), docs.active_id());
                self.name_buf = model_display_name(&identity);
                let doc = docs.active_mut();
                self.member_ack.remove(&doc.id());
                doc.set_name(Some(identity.clone()));
                doc.mark_clean();
                self.save_generation += 1;
                remember_document(store, &identity);
                self.status = if summary.is_empty() { format!("saved {identity}") } else { format!("saved {identity}; {summary}") };
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
        self.pending_open = None;
        if docs.focus_named(name) {
            remember_document(store, name);
            self.status = format!("{name} is already open");
            return;
        }
        match crate::store::read_now(store, name) {
            crate::store::ReadNow::Ready(contents) => self.load_document(docs, name, &contents, store),
            // An index-hydrated PLM store lists what it has not loaded yet;
            // the read above asked for it.
            crate::store::ReadNow::Loading => {
                self.pending_open = Some((name.to_owned(), docs.active_id()));
                self.status = format!("'{name}' is still loading");
            }
            crate::store::ReadNow::Absent => self.status = format!("open failed: '{name}' not found"),
        }
    }

    /// Complete the common File>Open / Edit-Part demand read on a later frame.
    /// Store reads are already deduplicated while a request is in flight.
    fn poll_pending_open(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        let Some((name, requester)) = self.pending_open.take() else { return };
        if docs.active_id() != requester {
            return;
        }
        if docs.focus_named(&name) {
            self.status = format!("{name} is already open");
            return;
        }
        match crate::store::read_now(store, &name) {
            crate::store::ReadNow::Ready(contents) => {
                self.load_document(docs, &name, &contents, store);
            }
            crate::store::ReadNow::Loading => {
                self.pending_open = Some((name, requester));
                ctx.request_repaint_after(std::time::Duration::from_millis(100));
            }
            crate::store::ReadNow::Absent => {
                self.status = format!("open failed: '{name}' not found");
            }
        }
    }

    /// Serialize the current model in `format` (`"step"` | `"iges"` | `"stl"` |
    /// `"obj"` | `"glb"` | `"3mf"` | `"json"`) and
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

    fn poll_instructions_pdf(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        let Some(mut export) = self.instructions_pdf.take() else { return };
        if export.document != docs.view_id() {
            self.status = "PDF export cancelled: the active document changed".into();
            docs.engine_mut().push_notice(self.status.clone());
            return;
        }
        match docs.engine_mut().poll_instructions_pdf(&mut export.job) {
            Ok(None) => {
                self.instructions_pdf = Some(export);
                ctx.request_repaint_after(std::time::Duration::from_millis(16));
            }
            Ok(Some((bytes, pages))) => {
                let pending = PendingSave {
                    request: SaveRequest::bytes("Export the instructions", format!("{}.pdf", export.name), bytes),
                    detail: format!(" ({pages} page{})", if pages == 1 { "" } else { "s" }),
                    label: None, warnings: vec![], preview: export.preview,
                };
                self.deliver(docs, store, pending);
            }
            Err(error) => {
                self.status = format!("PDF export failed: {error}");
                docs.engine_mut().push_notice(self.status.clone());
            }
        }
    }

    fn export_as(&mut self, docs: &mut Documents, store: &dyn ModelStore, format: &str) -> bool {
        let name = self.effective_name(docs);
        // The `.nbrep` recipe is the document; every other format is of the
        // geometry, which a family-row preview has replaced — said once the
        // file is delivered.
        let preview = format != "json";
        let export = |request: SaveRequest| PendingSave { request, detail: String::new(), label: None, warnings: vec![], preview };
        // GLB and 3MF are BINARY, so they take the store's byte-shaped
        // interchange rather than the text one below — a `.glb` routed through
        // a `&str` would be mangled the moment a chunk length happened not to
        // be valid UTF-8, and a 3MF is a ZIP.
        // The offline player: one page, named for the document, the tools'
        // pictures fetched from the library and inlined (the door is held
        // while they load; without a session the page carries none).
        if format == "instructionshtml" {
            let ids = linked_tool_ids(docs.engine());
            if let Some(client) = store.plm_client().filter(|_| !ids.is_empty()) {
                crate::plm::tool_pictures::request(&client, &ids);
                if crate::plm::tool_pictures::pending_of(&ids) > 0 {
                    self.pending_instructions_export = Some(format.to_string());
                    self.status = "loading the tool pictures for the export…".into();
                    return true;
                }
            }
            let made = docs.engine().instructions_player_data().map(|mut data| {
                data.inline_tool_pictures(&crate::plm::tool_pictures::fetched(&ids));
                brep_instructions::player::offline_html(&data, brep_instructions::player::Mode::Shop)
            });
            return match made {
                Ok(html) => {
                    let pending = export(SaveRequest::text("Export the offline player", format!("{name}.html"), html));
                    self.deliver(docs, store, pending) != Delivery::Failed
                }
                Err(e) => {
                    self.status = format!("export failed: {e}");
                    false
                }
            };
        }
        // The instructions' PDF and their step pictures: byte formats, named
        // for the document (`<name>.pdf`, `<name>-steps.zip`).
        if format == "instructionspdf" {
            if self.instructions_pdf.is_some() {
                self.status = "an instructions PDF is already being prepared".into();
                return false;
            }
            return match docs.engine_mut().begin_instructions_pdf() {
                Ok(job) => {
                    self.instructions_pdf = Some(PendingInstructionsPdf { document: docs.view_id(), name, preview, job });
                    self.status = "preparing the instructions PDF…".into();
                    true
                }
                Err(error) => { self.status = format!("export failed: {error}"); false }
            };
        }
        if format == "instructionsimages" {
            // The pictures are drawn from the exact solids: asked of the
            // runner first, and on a runner that cannot be waited on the
            // export is held until they land.
            if !docs.engine_mut().instructions_topology_ready() {
                self.pending_instructions_export = Some(format.to_string());
                self.status = "loading the exact geometry for the export…".into();
                return true;
            }
            let made = super::instructions::images_zip(docs.engine_mut()).map(|bytes| (bytes, String::new()));
            return match made {
                Ok((bytes, detail)) => {
                    let (title, file) = ("Export the step images", format!("{name}-steps.zip"));
                    let pending = PendingSave { detail, ..export(SaveRequest::bytes(title, file, bytes)) };
                    self.deliver(docs, store, pending) != Delivery::Failed
                }
                Err(e) => {
                    self.status = format!("export failed: {e}");
                    false
                }
            };
        }
        if format == "glb" || format == "3mf" {
            // The document's name becomes the root NODE's (GLB) or the model's
            // Title (3MF), as it becomes the root PRODUCT's on a STEP export.
            let written = if format == "glb" {
                docs.engine().export_glb_bytes(&name)
            } else {
                docs.engine().export_3mf_bytes(&name)
            };
            return match written {
                Ok(bytes) => {
                    let title = if format == "glb" { "Export GLB" } else { "Export 3MF" };
                    let request = SaveRequest::bytes(title, format!("{name}.{format}"), bytes);
                    self.deliver(docs, store, export(request)) != Delivery::Failed
                }
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
            let (file, made) = if pdf {
                let file = if three_d { format!("{name}-3d.pdf") } else { format!("{name}.pdf") };
                let bytes = if three_d { state.export_document_pdf_3d() } else { state.export_document_pdf() };
                (file, bytes.map(SaveBody::Bytes))
            } else {
                let sheet_name = state
                    .resolve_sheet("")
                    .ok()
                    .and_then(|id| state.sheet_state().find_sheet(&id).map(|sheet| sheet.name.clone()))
                    .unwrap_or_default();
                let file = format!("{name}-{}.svg", slug(&sheet_name));
                let text = state.resolve_sheet("").and_then(|id| state.export_sheet_svg(&id));
                (file, text.map(SaveBody::Text))
            };
            let pages = state.sheet_state().sheets.len() + usize::from(three_d);
            return match made {
                Ok(body) => {
                    let detail = if pdf {
                        format!(" ({pages} page{})", if pages == 1 { "" } else { "s" })
                    } else {
                        String::new()
                    };
                    let title = if pdf { "Export the drawing set" } else { "Export the drawing sheet" };
                    let request = SaveRequest { title: title.into(), file_name: file, body };
                    let pending = PendingSave { detail, ..export(request) };
                    self.deliver(docs, store, pending) != Delivery::Failed
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
            Ok(contents) => {
                let title = if format == "json" { "Export the model".to_string() } else { format!("Export {}", ext.to_ascii_uppercase()) };
                let request = SaveRequest::text(title, format!("{name}.{ext}"), contents);
                self.deliver(docs, store, export(request)) != Delivery::Failed
            }
            Err(e) => {
                self.status = format!("export failed: {e}");
                false
            }
        }
    }

    /// Hand a produced export to the user — the one door every export takes.
    /// Desktop (a store with local files): the Save-file chooser, which the
    /// modal switches to; the write happens on its Save. Browser: the
    /// download, now. What a written export says (status, toasts, the
    /// preview note) is [`Self::say_written`], on both roads.
    fn deliver(&mut self, docs: &mut Documents, store: &dyn ModelStore, pending: PendingSave) -> Delivery {
        if let Some(local) = store.local_files() {
            self.save_name = pending.request.file_name.clone();
            self.save_file = Some(pending);
            self.use_folder(Purpose::Exports, local);
            self.status.clear();
            self.open_modal(Mode::SaveFile);
            return Delivery::Chooser;
        }
        let file = pending.request.file_name.clone();
        let written = match &pending.request.body {
            SaveBody::Text(text) => store.export_file_named(&file, text),
            SaveBody::Bytes(bytes) => store.export_file_named_bytes(&file, bytes),
        };
        match written {
            Ok(()) => {
                self.say_written(docs, &pending, &file, None);
                Delivery::Done
            }
            Err(e) => {
                self.status = match &pending.label {
                    Some(label) => format!("{label} export failed: {e}"),
                    None => format!("export failed: {e}"),
                };
                Delivery::Failed
            }
        }
    }

    /// What a written export says: the status line ("exported <file>
    /// [detail] [into <folder>]"), the family-preview note, and the toast —
    /// a labelled export's own (the PCB outputs name themselves: "Netlist:
    /// …"), a warning one carrying the bundle's warnings, or on desktop,
    /// where the chooser closes on the write and the status with it, the
    /// generic "exported … into …". Returns whether the dialog should STAY
    /// open: it does on warnings, so they are read before boards are ordered.
    fn say_written(&mut self, docs: &mut Documents, pending: &PendingSave, file: &str, folder: Option<&str>) -> bool {
        let into = folder.map(|folder| format!(" into {folder}")).unwrap_or_default();
        self.status = format!("exported {file}{}{into}", pending.detail);
        if pending.preview {
            self.say_previewed_export(docs);
        }
        use brep_render::engine_state::NoticeSeverity;
        let state = docs.engine_mut();
        match &pending.label {
            Some(label) if !pending.warnings.is_empty() => {
                let warnings = pending.warnings.join(" ");
                self.status = format!("{}. Before ordering: {warnings}", self.status);
                state.push_notice_as(NoticeSeverity::Warning, format!("{label}: {warnings}"));
                true
            }
            Some(label) => {
                state.push_notice_as(NoticeSeverity::Info, format!("{label}: {}", self.status));
                false
            }
            None => {
                if folder.is_some() {
                    state.push_notice_as(NoticeSeverity::Info, self.status.clone());
                }
                false
            }
        }
    }

    /// A panel's export (the JavaScript editor's script, a plugin bundle, a
    /// PLM download): hand `request` to the user through the export door.
    /// Desktop: the Save-file chooser opens. Browser: the download, now.
    /// `Err` only when the download could not start.
    pub fn request_save_file(&mut self, docs: &mut Documents, store: &dyn ModelStore, request: SaveRequest) -> Result<(), String> {
        let pending = PendingSave { request, detail: String::new(), label: None, warnings: vec![], preview: false };
        match self.deliver(docs, store, pending) {
            Delivery::Failed => Err(std::mem::take(&mut self.status)),
            Delivery::Done | Delivery::Chooser => Ok(()),
        }
    }

    /// The **Save-file** chooser ([`Mode::SaveFile`]): the explorer over the
    /// local files picks the FOLDER, the field the NAME, and Save writes the
    /// held export there. Shaped like Save As (the name field and buttons
    /// pinned under the browser), over the exports' own remembered folder,
    /// which starts in the models folder.
    fn show_save_file(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        let Some(local) = store.local_files() else {
            self.save_file = None;
            self.open = false;
            return;
        };
        let title = self.save_file.as_ref().map(|pending| pending.request.title.clone()).unwrap_or_else(|| "Save file".into());
        let pending = self.save_file.is_some();
        // The explorer lists the files of the export's own kind beside the
        // folders, so a name to overwrite can be picked; a name without an
        // extension lists every file.
        let extension = std::path::Path::new(self.save_name.trim())
            .extension()
            .map(|extension| extension.to_string_lossy().into_owned());
        let extensions: Vec<&str> = extension.iter().map(String::as_str).collect();
        let mut do_save = false;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-file-savefile")).show(ctx, |ui| {
            file_explorer::dialog_body(ui, |ui| {
                ui.heading(&title);
                ui.add_space(4.0);
                file_explorer::dialog_footer(ui, "savefile", |ui| {
                    ui.label("File name");
                    let field = ui.add(
                        egui::TextEdit::singleline(&mut self.save_name).margin(crate::viewport::touch::text_edit_margin(ui)).min_size(crate::viewport::touch::target_size(ui.ctx(), egui::Vec2::ZERO))
                            .hint_text("file name")
                            .desired_width(f32::INFINITY),
                    );
                    self.hit("savefile:name", &field);
                    if self.want_focus {
                        field.request_focus();
                        self.want_focus = false;
                    }
                    let enter = field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        // Written already (the chooser stays up on a warning):
                        // nothing is left to save, only to read and close.
                        let save = ui.add_enabled(pending, egui::Button::new("Save"));
                        self.hit("save", &save);
                        if save.clicked() || (enter && pending) {
                            do_save = true;
                        }
                        let c = ui.button(if pending { "Cancel" } else { "Close" });
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
                    hit_prefix: "savefile",
                    empty_label: "(no files of this kind here)",
                    row_icon: "\u{1F5CE}",
                    current: None,
                    allow_delete: false,
                    allow_import: false,
                    import_label: "",
                    import_hit: "savefile:upload",
                    show_cancel: false,
                    confirm_label: None,
                    extensions: &extensions,
                };
                let output = self.explorer.show_store(ui, local, options);
                self.record_explorer_hits(&output.hits);
                // A row fills the name field (to overwrite that file); the
                // row's identity is its full path on desktop, so just its
                // name. One-shot, so it never clobbers a name typed after.
                if let Some(identity) = output.picked.or(output.activated) {
                    self.save_name = std::path::Path::new(&identity)
                        .file_name()
                        .map(|name| name.to_string_lossy().into_owned())
                        .unwrap_or(identity);
                    ui.ctx().request_repaint();
                }
            })
        });
        if do_save {
            if self.save_file(docs, store) {
                self.open = false;
            }
        } else if cancel || modal.should_close() {
            if self.save_file.take().is_some() {
                self.status = "export cancelled: nothing written".into();
            }
            self.open = false;
        }
    }

    /// The Save-file chooser's Save: write the held export under the name
    /// field, where the explorer is, and remember that folder for the next
    /// export. `true` once written, so the chooser closes — unless the export
    /// carried warnings, which keep it open to be read (what the format
    /// dialog did before the chooser existed).
    pub(crate) fn save_file(&mut self, docs: &mut Documents, store: &dyn ModelStore) -> bool {
        let Some(local) = store.local_files() else {
            return false;
        };
        let name = self.save_name.trim().to_string();
        if name.is_empty() {
            self.status = "enter a file name".into();
            return false;
        }
        let Some(pending) = self.save_file.take() else {
            return false;
        };
        match local.browser_write_bytes(&name, pending.request.body.as_bytes()) {
            Ok(path) => {
                self.folders.insert(Purpose::Exports, local.browser_location());
                let path = std::path::Path::new(&path);
                let folder = path.parent().map(|folder| folder.display().to_string()).unwrap_or_else(|| local.browser_location());
                let file = path.file_name().map(|file| file.to_string_lossy().into_owned()).unwrap_or(name);
                !self.say_written(docs, &pending, &file, Some(&folder))
            }
            Err(e) => {
                self.status = format!("export failed: {e}");
                self.save_file = Some(pending);
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
            Ok(contents) => {
                let request = SaveRequest::text("Export the flat pattern", format!("{name}.{ext}"), contents);
                let pending = PendingSave { request, detail: String::new(), label: None, warnings: vec![], preview: true };
                self.deliver(docs, store, pending) != Delivery::Failed
            }
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
            Ok(contents) => {
                let request = SaveRequest::text("Export the bill of materials", format!("{name}.{ext}"), contents);
                let pending = PendingSave { request, detail: String::new(), label: None, warnings: vec![], preview: false };
                self.deliver(docs, store, pending) != Delivery::Failed
            }
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
        let made = match output {
            Output::Bundle => fab::bundle(state, &name).map(|bundle| {
                let detail = format!(" ({} files)", bundle.files.len());
                (SaveBody::Bytes(bundle.zip()), detail, bundle.warnings)
            }),
            _ => fab::text(state, output, &name).map(|text| (SaveBody::Text(text), String::new(), vec![])),
        };
        match made {
            Ok((body, detail, warnings)) => {
                // The output names itself in its toast (Netlist, Pick & place,
                // BOM, Fabrication), and the chooser's status says WHERE the
                // files went: on desktop that is the folder the user chose.
                let request = SaveRequest { title: format!("Export: {}", output.label()), file_name: file, body };
                let pending = PendingSave { request, detail, label: Some(output.label().to_string()), warnings, preview: false };
                match self.deliver(docs, store, pending) {
                    Delivery::Done => !self.status.contains("Before ordering"),
                    Delivery::Chooser => true,
                    Delivery::Failed => {
                        docs.engine_mut().push_notice(format!("{} export: {}", output.label(), self.status));
                        false
                    }
                }
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
                    (0, _) if sink.deferred > 0 => format!(
                        "{base}; its {} part documents become PLM items when you save the assembly",
                        sink.deferred
                    ),
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
    fn load_document(&mut self, docs: &mut Documents, name: &str, contents: &str, store: &dyn ModelStore) {
        let plm = store.plm_client().is_some();
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
                doc.note_opened_from(contents);
                docs.open_document(doc);
                self.name_buf = model_display_name(name);
                self.status = format!("opened {name}");
                remember_document(store, name);
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
            DocumentClass::Normal | DocumentClass::Instructions => false,
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
                let field = ui.add(egui::TextEdit::singleline(&mut spin.name).margin(crate::viewport::touch::text_edit_margin(ui)).min_size(crate::viewport::touch::target_size(ui.ctx(), egui::Vec2::ZERO)).desired_width(240.0));
                self.hit("spinout:name", &field);
                ui.end_row();
                if spin.plm_template.is_some() {
                    ui.label("Part number");
                    let field = ui.add(
                        egui::TextEdit::singleline(&mut spin.number).margin(crate::viewport::touch::text_edit_margin(ui)).min_size(crate::viewport::touch::target_size(ui.ctx(), egui::Vec2::ZERO))
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
                            let field = ui.add(egui::TextEdit::singleline(value).margin(crate::viewport::touch::text_edit_margin(ui)).min_size(crate::viewport::touch::target_size(ui.ctx(), egui::Vec2::ZERO)).desired_width(240.0));
                            self.hit(&format!("spinout:input:{}", input.name), &field);
                        } else {
                            // The input's choices as one dropdown: spinout:choice:<name>,
                            // its open entries spinout:choice:<name>#<choice>.
                            let trimmed: Vec<String> = input.choices.iter().map(|choice| choice.trim().to_string()).collect();
                            let choices: Vec<(String, &str, &str)> = trimmed.iter().map(|choice| (choice.clone(), choice.as_str(), choice.as_str())).collect();
                            let mut chosen = value.trim().to_string();
                            let (changed, _) = crate::form::enum_dropdown(ui, &format!("spinout:choice:{}", input.name), &choices, &mut chosen, None, Some(&mut self.hits));
                            if changed {
                                *value = chosen;
                            }
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
    /// when it was armed (the caller does not go on to write). Raise the
    /// hand-edit prompt the frame a family member first shows an unsaved
    /// change — the moment the user "starts to change it".
    fn watch_member_edit(&mut self, docs: &Documents) {
        let doc = docs.active();
        if !doc.dirty_marker() || self.member_ack.contains(&doc.id()) {
            return;
        }
        if let Some(name) = doc.name().map(str::to_string) {
            self.arm_member_prompt(docs, &name, false);
        }
    }

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

    /// The assembly Save's ONE dialog about the documents it owns: a row per
    /// document that needs a decision, each ticked (yes) by default, so Enter
    /// saves everything. An unticked row is not written and its change stays
    /// in the assembly. Cancel saves nothing.
    fn show_owned_save(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        let Some(mut prompt) = self.owned_prompt.take() else {
            self.open = false;
            return;
        };
        if docs.active_id() != prompt.plan.doc_id {
            self.status = "Save cancelled because the active document changed".into();
            self.open = false;
            return;
        }
        let mut choice = None;
        let modal = egui::Modal::new(egui::Id::new("brep-file-owned-save")).show(ctx, |ui| {
            ui.set_width(520.0);
            ui.heading("Save the parts this assembly changed");
            ui.add_space(4.0);
            ui.label("Saving writes these documents too. Untick any you do not want written; its change stays in the assembly.");
            ui.add_space(6.0);
            for (i, row) in prompt.plan.rows.iter_mut().enumerate() {
                if !row.asks() {
                    if let Some(crate::document::owned_save::Issue::Blocked(_)) = row.issue {
                        ui.weak(row.question());
                    }
                    continue;
                }
                let question = row.question();
                let tick = ui.checkbox(&mut row.proceed, question);
                self.hit(&format!("owned:row:{i}"), &tick);
                if row.issue == Some(crate::document::owned_save::Issue::Conflict) && row.proceed {
                    ui.indent(("owned-conflict", i), |ui| {
                        // Which copy to keep, as one dropdown: owned:choice:<row>, its
                        // open entries owned:choice:<row>#<variant>.
                        let labels: Vec<(String, String)> = row
                            .write
                            .variants
                            .iter()
                            .enumerate()
                            .map(|(j, (names, _))| {
                                let last = if row.edited_last == Some(j) { " (edited last)" } else { "" };
                                (j.to_string(), format!("keep the copy in {}{last}", names.join(" > ")))
                            })
                            .collect();
                        let choices: Vec<(usize, &str, &str)> = labels.iter().enumerate().map(|(j, (id, label))| (j, id.as_str(), label.as_str())).collect();
                        crate::form::enum_dropdown(ui, &format!("owned:choice:{i}"), &choices, &mut row.choice, None, Some(&mut self.hits));
                    });
                }
            }
            if prompt.plan.rows.iter().any(|row| matches!(row.issue, Some(crate::document::owned_save::Issue::CheckOut { .. }))) {
                ui.add_space(4.0);
                let remember = ui.checkbox(&mut self.checkout_remembered, "Check out without asking again in this session");
                self.hit("owned:remember", &remember);
            }
            for promotion in &prompt.plan.promotions {
                let part = promotion.names.last().map_or("", String::as_str);
                ui.weak(if promotion.recreates() {
                    format!("{part}: its file is missing — re-created as {}", promotion.new_key)
                } else {
                    format!("{part}: saved to a new part file {}", promotion.new_key)
                });
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let save = ui.button("Save");
                self.hit("owned:save", &save);
                if save.clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    choice = Some(true);
                }
                let cancel = ui.button("Cancel");
                self.hit("owned:cancel", &cancel);
                if cancel.clicked() {
                    choice = Some(false);
                }
            });
        });
        let choice = choice.or_else(|| modal.should_close().then_some(false));
        self.owned_state = serde_json::json!({"prompt": prompt.plan.to_json()});
        match choice {
            Some(true) => {
                self.open = false;
                let OwnedPrompt { plan, name, browser } = prompt;
                if browser {
                    self.commit_save_browser(docs, store, name, plan);
                } else {
                    self.commit_save(docs, store, name, plan);
                }
            }
            Some(false) => {
                self.status = "Save cancelled".into();
                self.owned_state = serde_json::Value::Null;
                self.open = false;
            }
            None => self.owned_prompt = Some(prompt),
        }
    }

    /// "Also point these assemblies at the new file?" — one question for every
    /// open assembly that places the part just saved under a new name. Yes
    /// (the default, Enter) moves each of their copies to the new file as one
    /// undoable edit per assembly; No leaves them on the old file.
    fn show_retarget(&mut self, ctx: &egui::Context, docs: &mut Documents) {
        let Some(retarget) = self.retarget.as_ref() else {
            self.open = false;
            return;
        };
        let mut choice = None;
        let new_name = model_display_name(&retarget.new_key);
        let old_name = model_display_name(&retarget.old);
        let titles: Vec<String> = retarget.tabs.iter().map(|(_, title)| title.clone()).collect();
        egui::Modal::new(egui::Id::new("brep-file-retarget")).show(ctx, |ui| {
            ui.set_width(480.0);
            ui.heading("Point the assemblies at the new file?");
            ui.label(format!(
                "{} {} '{old_name}'. Point {} at '{new_name}' too?",
                titles.join(", "),
                if titles.len() == 1 { "places" } else { "place" },
                if titles.len() == 1 { "it" } else { "them" },
            ));
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let yes = ui.button(format!("Yes, use '{new_name}'"));
                self.hit("retarget:yes", &yes);
                if yes.clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    choice = Some(true);
                }
                let no = ui.button(format!("No, keep '{old_name}'"));
                self.hit("retarget:no", &no);
                if no.clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    choice = Some(false);
                }
            });
        });
        let Some(yes) = choice else { return };
        let Some(retarget) = self.retarget.take() else { return };
        self.open = false;
        if !yes {
            self.status = format!("kept the assemblies on '{old_name}'");
            return;
        }
        let mut moved = Vec::new();
        for (id, title) in &retarget.tabs {
            match docs.retarget_copies(*id, &retarget.old, &retarget.new_key, &retarget.text) {
                Ok(n) if n > 0 => moved.push(title.clone()),
                Ok(_) => {}
                Err(error) => docs.engine_mut().push_notice(format!("{title}: could not point it at '{new_name}': {error}")),
            }
        }
        self.status = format!("{} now {} '{new_name}' (unsaved)", moved.join(", "), if moved.len() == 1 { "uses" } else { "use" });
        docs.engine_mut().push_notice_as(brep_render::engine_state::NoticeSeverity::Info, self.status.clone());
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
                // the copy it writes. Plain Save still asks. On a PLM the
                // Save As opens on "A new part".
                self.member_ack.insert(prompt.doc_id);
                self.member_new_part = store.plm_client().is_some();
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
                    Ok(crate::plm::identity::document_key(&head.id, &newest.id))
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
        let json = docs.tab_engine().history_request_json();
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
            "ownedSave": self.owned_state,
            // What an assembly Save still has out after its status says
            // "saved": checkouts (the assembly's own included), new revisions, and the Save that names them.
            // 0 = the PLM holds everything this Save will write.
            "ownedInFlight": self.owned_checkouts.len()
                + self.owned_revisions.as_ref().map_or(0, |r| r.requests.len())
                + usize::from(self.owned_followup.is_some())
                + usize::from(self.owned_top.is_some()),
            "retarget": self.retarget.as_ref().map(|r| serde_json::json!({
                "from": r.old,
                "to": r.new_key,
                "tabs": r.tabs.iter().map(|(_, title)| title.clone()).collect::<Vec<_>>(),
            })),
            "open": self.open,
            "mode": match self.mode {
                Mode::CreateComponentPart => "createcomponentpart",
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
                Mode::OwnedSave => "ownedsave",
                Mode::Retarget => "retarget",
                Mode::PickFile => "pickfile",
                Mode::SaveFile => "savefile",
            },
            // The Save-file chooser's export: its title, the name field, the
            // folder the explorer is in (where Save writes) and the size.
            "saveFile": self.save_file.as_ref().map(|pending| serde_json::json!({
                "title": pending.request.title,
                "name": self.save_name,
                "folder": store.local_files().map(|local| local.browser_location()),
                "bytes": pending.request.body.as_bytes().len(),
            })),
            "pickFile": self.pick_request.as_ref().map(|(tag, title)| serde_json::json!({ "tag": tag, "title": title })),
            "picked": self.picked.as_ref().map(|(tag, file)| serde_json::json!({
                "tag": tag, "name": file.name, "path": file.path, "size": file.bytes.len(),
            })),
            // The active document's class (from its own `documentClass`
            // field) and, for a family member, its stamp.
            "class": document_class::document_class(docs.engine()).slug(),
            "saveClass": self.save_class.slug(),
            "plmSaveAs": self.plm_save_as.as_ref().map(|state| state.state_json()),
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
            // What the PLM workspace browser (the Open dialog) says went wrong.
            "workspaceProblem": self.workspace.problem_text(),
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
    HitKeyDoc { panel: "file", prefix: "export:", meaning: "export in a format (export:step, export:iges, export:stl, export:obj, export:glb, export:3mf, export:json, export:bomcsv, export:bomjson, the open DRAWING SHEET as export:sheetsvg, every drawing sheet as ONE multi-page PDF as export:sheetpdf, and the same drawing set with the live 3D model as export:sheetpdf3d; for a document whose PCB board has parts, its manufacturing files: the Gerber/drill/pick-and-place/BOM zip as export:fabrication, the pick-and-place CSV as export:pickplace, the electronics BOM CSV as export:ecadbom; for manufacturing instructions, the printed PDF as export:instructionspdf, every step's PNG and SVG as one zip as export:instructionsimages, and the self-contained offline player as export:instructionshtml)", command: None },
    HitKeyDoc { panel: "file", prefix: "flat:", meaning: "export the flat pattern (flat:dxf, flat:svg)", command: None },
    HitKeyDoc { panel: "file", prefix: "savefile:", meaning: "the Save-file chooser a desktop export opens after its format click, for WHERE the file goes: savefile:name is the file name field, and its explorer over this machine's files publishes the explorer's key set under the `savefile` prefix (savefile:<file name> a row to overwrite, savefile:crumb:<i>, savefile:place:<label>, savefile:up, savefile:path-edit, …); `save` writes the file where the explorer is, `cancel` drops it. In the browser an export is a download and the chooser never opens", command: None },
    HitKeyDoc { panel: "file", prefix: "stepassembly:", meaning: "a STEP-with-structure import choice (assembly, bodies, flatten, cancel)", command: None },
    HitKeyDoc { panel: "file", prefix: "insert:lib:", meaning: "insert a parts-library entry (insert:lib:name)", command: None },
    HitKeyDoc { panel: "file", prefix: "filesystem:", meaning: "an entry of the file explorer", command: None },
    HitKeyDoc { panel: "file", prefix: "saveas:", meaning: "PLM Save As: saveas:destination (one dropdown; its open entries saveas:destination#new-revision, #new-part, #existing), destination fields (part, revision, name, number), save and cancel", command: None },
    HitKeyDoc { panel: "file", prefix: "saveas:class", meaning: "Save As's class dropdown: saveas:class, its open entries saveas:class#normal (.nbrep), saveas:class#family (.fbrep), saveas:class#template (.tbrep)", command: None },
    HitKeyDoc { panel: "file", prefix: "member:", meaning: "the member picker Insert component raises for a family: member:<part number> places that member (disabled until Generate has written it), member:cancel", command: None },
    HitKeyDoc { panel: "file", prefix: "spinout:", meaning: "the template spin-out prompt: spinout:name, spinout:input:<name> (a value field), spinout:choice:<name> (a fixed-list input's dropdown; its open entries spinout:choice:<name>#<value>), spinout:create, spinout:cancel", command: None },
    HitKeyDoc { panel: "file", prefix: "retarget:", meaning: "after a part is saved under a new name, point the open assemblies that place it at the new file (retarget:yes, the default) or keep them on the old one (retarget:no)", command: None },
    HitKeyDoc { panel: "file", prefix: "owned:", meaning: "the assembly Save's questions about the documents it owns: owned:save (save, every ticked row), owned:cancel (save nothing), owned:row:<i> (tick/untick row i), owned:choice:<i> (which copy of a document changed differently to keep, a dropdown; its open entries owned:choice:<i>#<j>), owned:remember (check out without asking again in this session)", command: None },
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

/// Recent documents are global store identities, never document content.
use crate::store::RECENT_DOCUMENTS_KEY;
pub fn recent_documents(store: &dyn ModelStore) -> Vec<String> {
    store
        .read(RECENT_DOCUMENTS_KEY)
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default()
}
fn remember_document(store: &dyn ModelStore, name: &str) {
    let mut recent = recent_documents(store);
    recent.retain(|item| item != name);
    recent.insert(0, name.into());
    recent.truncate(20);
    let _ = store.write(
        RECENT_DOCUMENTS_KEY,
        &serde_json::to_string(&recent).unwrap(),
    );
}

#[derive(Default)]
pub struct FileMenuOutcome {
    pub action: Option<FileAction>,
    pub recent: Option<String>,
    /// The Plugins row under Settings was clicked: open the plugin manager.
    pub plugins: bool,
    /// The JavaScript row under Settings was clicked: open the editor.
    pub javascript: bool,
}

/// The side of a recent row's thumbnail square, in points.
pub const RECENT_THUMB_SIDE: f32 = crate::plm::thumbnail::RECENT_SIDE;

/// The neutral square a recent document without a picture yet shows: the
/// same size as a thumbnail, never a gap. Drawn through the SVG loader the
/// menu's artwork already uses.
const RECENT_PLACEHOLDER_SVG: &str = r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 40"><rect x="1.5" y="1.5" width="37" height="37" rx="4" fill="#8a8a8a" fill-opacity="0.16" stroke="#8a8a8a" stroke-opacity="0.45"/></svg>"##;

/// Shared File menu for both appearances: the workbench picker on the left,
/// the document commands on the right, the recent documents (with their
/// thumbnails) in one column under both. Workbench selection uses apply +
/// store.
pub fn file_menu(
    ui: &mut egui::Ui,
    state: &mut EngineState,
    store: &dyn ModelStore,
    settings_open: &mut bool,
    properties_open: &mut bool,
    hits: &mut HashMap<String, egui::Rect>,
) -> FileMenuOutcome {
    use crate::panels::toolbar_button;
    let mut outcome = FileMenuOutcome::default();
    let presentation = crate::workbench::file_presentation;
    egui_extras::install_image_loaders(ui.ctx());
    let menu = ui.menu_button("File", |ui| {
        // Plugin workbenches declare no artwork of their own; they share the
        // plugin manager's glyph.
        let plugin_glyph = presentation("workbench:manage:plugins").glyph;
        let dynamic = crate::workbench::owned_workbenches(state);
        let workbench_rows: Vec<(&str, &str)> = crate::workbench::WORKBENCHES
            .iter()
            .map(|w| (w.glyph, w.label))
            .chain(dynamic.iter().map(|w| (plugin_glyph, w.label.as_str())))
            .collect();
        let file_rows: Vec<_> = crate::workbench::FILE_COMMANDS
            .iter()
            .map(|p| (p.glyph, p.label()))
            .collect();
        // Fixed measured widths — one per column — avoid feeding the popup's
        // previous width back through full-width rows while egui settles its
        // area. The recent rows span both columns.
        let left_width = toolbar_button::menu_width_salted(ui, "workbench-measure", &workbench_rows);
        let right_width = toolbar_button::menu_width(ui, &file_rows);
        let gap = ui.spacing().item_spacing.x;
        let compact = crate::viewport::touch::compact(ui.ctx());
        let width = if compact { left_width.max(right_width).min((ui.ctx().content_rect().width() - 32.0).max(100.0)) }
            else { left_width + gap + right_width };
        let (left_width, right_width) = if compact { (width, width) } else { (left_width, right_width) };
        ui.set_width(width);
        let max_height = ui.ctx().content_rect().height() * 0.8;
        ui.set_max_height(max_height);
        // The whole body sits in one scroll area so a window too short for
        // the columns themselves still shows a menu that fits; on any taller
        // window the columns never scroll and only the recent list does, up
        // to the room left under the cap (`cap_bottom`).
        let cap_bottom = ui.max_rect().bottom();
        let body = egui::ScrollArea::vertical()
            .id_salt("file-menu-body")
            .auto_shrink([false, true])
            .max_height(max_height)
            .show(ui, |ui| {
                let columns = |ui: &mut egui::Ui| {
                    // LEFT: the workbench picker.
                    ui.vertical(|ui| {
                        ui.set_width(left_width);
                        ui.weak("Workbench");
                        let current = crate::workbench::resolved_id(state, &state.settings.workbench);
                        if current != state.settings.workbench {
                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new(format!(
                                        "Unavailable workbench: {}; using Modeling",
                                        state.settings.workbench
                                    ))
                                    .weak(),
                                )
                                .wrap(),
                            );
                        }
                        let options = crate::workbench::WORKBENCHES
                            .iter()
                            .map(|w| (w.id, w.label, w.glyph))
                            .chain(dynamic.iter().map(|w| (w.id.as_str(), w.label.as_str(), plugin_glyph)));
                        for (id, label, glyph) in options {
                            let row = toolbar_button::menu_row(ui, glyph, label, current == id, left_width);
                            hits.insert(format!("workbench:item:{id}"), row.rect);
                            if row.clicked() {
                                let _ = state
                                    .apply_settings_json(&serde_json::json!({"workbench": id}).to_string());
                                let _ = store.write(crate::store::SETTINGS_KEY, &state.settings_json());
                                ui.close();
                            }
                        }
                    });
                    // RIGHT: everything else, in the order the one-column menu had.
                    ui.vertical(|ui| {
                        ui.set_width(right_width);
                        ui.weak("Document");
                        for (class, p) in DocumentClass::ALL
                            .into_iter()
                            .zip(crate::workbench::NEW_DOCUMENT_COMMANDS)
                        {
                            let row = toolbar_button::menu_row(ui, p.glyph, p.label(), false, right_width);
                            hits.insert(p.id.into(), row.rect);
                            if class == DocumentClass::Normal {
                                hits.insert("file:new".into(), row.rect);
                                hits.insert("file:newmenu".into(), row.rect);
                            }
                            if row.clicked() {
                                outcome.action = Some(FileAction::NewOfClass(class));
                                ui.close();
                            }
                        }
                        for (id, action) in [
                            ("file:open", FileAction::Open),
                            ("file:save", FileAction::Save),
                            ("file:saveas", FileAction::SaveAs),
                            ("file:import", FileAction::Import),
                            ("file:export", FileAction::Export),
                        ] {
                            let p = presentation(id);
                            let row = toolbar_button::menu_row(ui, p.glyph, p.label(), false, right_width);
                            hits.insert(id.into(), row.rect);
                            if row.clicked() {
                                outcome.action = Some(action);
                                ui.close();
                            }
                        }
                        let properties = toolbar_button::menu_row(
                            ui,
                            presentation("properties").glyph,
                            presentation("properties").label(),
                            *properties_open,
                            right_width,
                        );
                        hits.insert("properties".into(), properties.rect);
                        if properties.clicked() {
                            *properties_open = !*properties_open;
                            ui.close();
                        }
                        let settings = toolbar_button::menu_row(
                            ui,
                            presentation("settings").glyph,
                            presentation("settings").label(),
                            *settings_open,
                            right_width,
                        );
                        hits.insert("settings".into(), settings.rect);
                        if settings.clicked() {
                            *settings_open = !*settings_open;
                            ui.close();
                        }
                        // Plugin management sits under Settings: the plugin manager
                        // and the JavaScript editor, the same rows in both styles.
                        for (id, flag) in [
                            ("workbench:manage:plugins", &mut outcome.plugins),
                            ("workbench:manage:javascript", &mut outcome.javascript),
                        ] {
                            let row = toolbar_button::menu_row(
                                ui,
                                presentation(id).glyph,
                                presentation(id).label(),
                                false,
                                right_width,
                            );
                            hits.insert(id.into(), row.rect);
                            if row.clicked() {
                                *flag = true;
                                ui.close();
                            }
                        }
                    });
                };
                if compact { ui.vertical(columns); } else { ui.horizontal_top(columns); }

                // BELOW BOTH: the recent documents, one column, a thumbnail each. The
                // list scrolls under the 80% cap; the columns above it never do.
                let recent = recent_documents(store);
                if !recent.is_empty() {
                    ui.separator();
                    let caption = ui.weak("Recent Documents");
                    hits.insert("file:recent".into(), caption.rect);
                    let thumbs = crate::plm::thumbnail::recent_textures(ui.ctx(), store);
                    let side = egui::Vec2::splat(RECENT_THUMB_SIDE);
                    let row_height = RECENT_THUMB_SIDE + 2.0 * ui.spacing().button_padding.y;
                    // The room left under the cap; at least two rows even when the
                    // columns already fill it (then the outer area scrolls).
                    let room = cap_bottom - ui.cursor().top();
                    let list_height = room.max(2.0 * row_height + ui.spacing().item_spacing.y);
                    egui::ScrollArea::vertical()
                        .id_salt("file-recent")
                        .auto_shrink([false, true])
                        .max_height(list_height)
                        .show(ui, |ui| {
                            for name in &recent {
                                let image = match thumbs.get(name) {
                                    Some((_, texture)) => egui::Image::from_texture((texture.id(), side)),
                                    None => egui::Image::new(egui::ImageSource::Bytes {
                                        uri: "bytes://brep-recent-thumb/placeholder.svg".into(),
                                        bytes: egui::load::Bytes::Static(RECENT_PLACEHOLDER_SVG.as_bytes()),
                                    }),
                                }
                                .fit_to_exact_size(side);
                                let row = ui
                                    .add(
                                        egui::Button::new((image, model_display_name(name)))
                                            .wrap_mode(egui::TextWrapMode::Truncate)
                                            .min_size(egui::vec2(width, row_height)),
                                    )
                                    .on_hover_text(name);
                                hits.insert(format!("file:recent:{name}"), row.rect);
                                let padding = ui.spacing().button_padding;
                                let thumb = egui::Rect::from_min_size(
                                    row.rect.min + egui::vec2(padding.x, (row.rect.height() - RECENT_THUMB_SIDE) / 2.0),
                                    side,
                                );
                                hits.insert(format!("file:recent:thumb:{name}"), thumb);
                                if row.clicked() {
                                    outcome.recent = Some(name.clone());
                                    ui.close();
                                }
                            }
                        });
                }
            });
        if compact { hits.insert("file:clip".into(), body.inner_rect); }
    });
    hits.insert("file:menu".into(), menu.response.rect);
    // Retain the historic menu-opener key for coordinate automation.
    hits.insert("workbench".into(), menu.response.rect);
    outcome
}


/// The library ids of the tools the open instructions use (the tools whose
/// pictures an export fetches).
pub fn linked_tool_ids(engine: &EngineState) -> Vec<String> {
    engine
        .instructions()
        .ok()
        .flatten()
        .map(|block| block.tools_list().into_iter().filter_map(|(tool, _)| tool.tool_id).collect())
        .unwrap_or_default()
}
