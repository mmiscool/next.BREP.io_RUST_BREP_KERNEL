//! Import a part from KiCad: its symbol, its pads and its 3D model, as ONE edit.
//!
//! The Symbol workbench imports a `.kicad_sym` symbol and the Pads workbench a
//! `.kicad_mod` footprint (eCAD plan, decision 9). The host reads the files and
//! eCAD parses them (`brep_ecad_core::kicad`). A symbol import follows KiCad's chain:
//!
//! 1. the symbol's `Footprint` property names `library:name`, found as
//!    `<footprints>/<library>.pretty/<name>.kicad_mod`; a generic symbol leaves it
//!    empty and offers `ki_fp_filters` patterns, and the user chooses among the
//!    footprints they match;
//! 2. the footprint's `(model …)` entry names a `.wrl` in KiCad's 3D library,
//!    usually under `${KICAD9_3DMODEL_DIR}`;
//! 3. BREP imports STEP, not VRML, so the model read is the `.step` of the same
//!    name beside it.
//!
//! Every link that fails is a stated outcome, not an error: the part imports
//! with what was found, and a note names the file that was looked for. KiCad's
//! own library tables (`fp-lib-table`) are not read; a footprint library is found
//! only under the footprints folder, by its nickname.
//!
//! # The part's frame
//!
//! The model and the ports are placed in KiCad's 3D frame of the footprint: x as
//! the footprint's, y the footprint's NEGATED, z up from the board's top copper,
//! millimetres. That is where KiCad's own 3D view and STEP exporter put a
//! footprint's model ([`kicad::model_placement`]), so a pad at `(x, y)` µm in the
//! pads block is at `(x / 1000, −y / 1000, 0)` mm in the part.
//!
//! # One undo
//!
//! The model becomes one IMPORT3D feature, each symbol pin one connection point
//! at its pad, and the symbol and pads are written as the part's `symbol` and
//! `pads` blocks. The features go in with ONE `add_features` (one checkpoint),
//! and the blocks after it without a checkpoint of their own, so one Ctrl+Z takes
//! the whole import away. A block written BEFORE the checkpoint would be inside
//! the snapshot undo restores and survive the undo (`model_io.rs` explains the
//! same trap for PMI).
//!
//! The model is imported here, not by the pipeline: KiCad turns a model by
//! `Rz(−rz)·Ry(−ry)·Rx(−rx)`, and BREP's transform param by `Rx·Ry·Rz` with the
//! angles as given, so KiCad's numbers cannot be handed to a transform feature.
//! The STEP bodies are moved by KiCad's own matrix and sealed into the feature's
//! `nativeBrep` payload, the lane a STEP-assembly part rides on, and the numbers
//! they were placed by are kept in its `persistentData.kicadModel`.

use crate::store::{ModelStore, KICAD_LIBRARY_KEY};
use brep_render::brep_kernel as kernel;
use brep_render::engine_state::EngineState;
use brep_ecad_core::board::{Footprint, Model};
use brep_ecad_core::kicad::{self, FootprintLink};
use brep_ecad_core::Symbol;
use serde_json::{json, Value};
use std::path::Path;

/// Which KiCad file an import starts from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KicadKind {
    /// A `.kicad_sym` library: a symbol, and through it the footprint and model.
    Symbol,
    /// A `.kicad_mod` footprint: pads and the model.
    Footprint,
}

impl KicadKind {
    pub fn extension(self) -> &'static str {
        match self {
            KicadKind::Symbol => "kicad_sym",
            KicadKind::Footprint => "kicad_mod",
        }
    }

    /// The kind a file name's extension names, if any.
    pub fn of_file(name: &str) -> Option<Self> {
        let lower = name.to_ascii_lowercase();
        [KicadKind::Symbol, KicadKind::Footprint]
            .into_iter()
            .find(|kind| lower.ends_with(&format!(".{}", kind.extension())))
    }
}

// ============================================================================
// Reading the user's files
// ============================================================================

/// The files an import reads: the user's KiCad library on disk. A trait so the
/// tests can hand the chain a library without one being installed.
pub trait KicadFiles {
    /// Whether this build can read the user's files at all. The web build
    /// cannot: the chain stops at the file the user uploaded.
    fn available(&self) -> bool {
        true
    }
    fn read(&self, path: &str) -> Option<Vec<u8>>;
    fn is_file(&self, path: &str) -> bool;
    fn is_dir(&self, path: &str) -> bool;
    /// The names of `dir`'s entries.
    fn list(&self, dir: &str) -> Vec<String>;
}

/// The filesystem, for the native build.
pub struct DiskFiles;

impl KicadFiles for DiskFiles {
    fn available(&self) -> bool {
        !cfg!(target_arch = "wasm32")
    }
    fn read(&self, path: &str) -> Option<Vec<u8>> {
        std::fs::read(path).ok()
    }
    fn is_file(&self, path: &str) -> bool {
        Path::new(path).is_file()
    }
    fn is_dir(&self, path: &str) -> bool {
        Path::new(path).is_dir()
    }
    fn list(&self, dir: &str) -> Vec<String> {
        std::fs::read_dir(dir)
            .into_iter()
            .flatten()
            .flatten()
            .map(|entry| entry.file_name().to_string_lossy().into_owned())
            .collect()
    }
}

fn join(dir: &str, name: &str) -> String {
    Path::new(dir).join(name).to_string_lossy().into_owned()
}

fn parent(path: &str) -> Option<String> {
    Path::new(path)
        .parent()
        .map(|dir| dir.to_string_lossy().into_owned())
        .filter(|dir| !dir.is_empty())
}

fn stem(path: &str) -> String {
    let name = path.rsplit(['/', '\\']).next().unwrap_or(path);
    name.rsplit_once('.').map_or(name, |(stem, _)| stem).to_owned()
}

// ============================================================================
// Where the library is
// ============================================================================

/// The variables KiCad names its footprint folder by, newest first.
const FOOTPRINT_VARS: &[&str] = &[
    "KICAD9_FOOTPRINT_DIR",
    "KICAD8_FOOTPRINT_DIR",
    "KICAD7_FOOTPRINT_DIR",
    "KICAD6_FOOTPRINT_DIR",
];
/// The variables KiCad names its 3D-model folder by, newest first.
/// `KISYS3DMOD` is the name before KiCad 6.
const MODEL_VARS: &[&str] = &[
    "KICAD9_3DMODEL_DIR",
    "KICAD8_3DMODEL_DIR",
    "KICAD7_3DMODEL_DIR",
    "KICAD6_3DMODEL_DIR",
    "KISYS3DMOD",
];

/// Where a KiCad install keeps its libraries, `(footprints, 3D models)`, for
/// this platform. The first is also what a message names when none exists.
fn install_defaults() -> &'static [(&'static str, &'static str)] {
    if cfg!(target_os = "windows") {
        &[(
            r"C:\Program Files\KiCad\9.0\share\kicad\footprints",
            r"C:\Program Files\KiCad\9.0\share\kicad\3dmodels",
        )]
    } else if cfg!(target_os = "macos") {
        &[(
            "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints",
            "/Applications/KiCad/KiCad.app/Contents/SharedSupport/3dmodels",
        )]
    } else {
        &[
            ("/usr/share/kicad/footprints", "/usr/share/kicad/3dmodels"),
            ("/usr/local/share/kicad/footprints", "/usr/local/share/kicad/3dmodels"),
            (
                "/var/lib/flatpak/app/org.kicad.KiCad/current/active/files/share/kicad/footprints",
                "/var/lib/flatpak/app/org.kicad.KiCad/current/active/files/share/kicad/3dmodels",
            ),
        ]
    }
}

/// One library folder and where it came from, in words the dialog shows.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Folder {
    pub path: String,
    pub from: String,
}

/// The user's KiCad library: the folder holding the `*.pretty` footprint
/// libraries and the folder holding the `*.3dshapes` models.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct KicadLibrary {
    pub footprints: Folder,
    pub models: Folder,
}

impl KicadLibrary {
    /// The folders a fresh session uses: KiCad's variable when it is set to a
    /// folder, else the first install default that exists, else the first
    /// install default (so a message can name where it looked).
    pub fn seeded(env: &dyn Fn(&str) -> Option<String>, files: &dyn KicadFiles) -> Self {
        let seed = |vars: &[&str], pick: fn(&(&'static str, &'static str)) -> &'static str| -> Folder {
            for var in vars {
                if let Some(path) = env(var).filter(|path| files.is_dir(path)) {
                    return Folder { path, from: format!("from {var}") };
                }
            }
            let defaults = install_defaults();
            match defaults.iter().map(pick).find(|path| files.is_dir(path)) {
                Some(path) => Folder { path: path.into(), from: "KiCad's install folder".into() },
                None => Folder {
                    path: pick(&defaults[0]).into(),
                    from: format!("not found: {} unset, and no KiCad install here", vars.join(", ")),
                },
            }
        };
        Self {
            footprints: seed(FOOTPRINT_VARS, |(footprints, _)| footprints),
            models: seed(MODEL_VARS, |(_, models)| models),
        }
    }

    /// The folders the user last chose (the [`KICAD_LIBRARY_KEY`] record), each
    /// falling back to the seed when it is unset or no longer a folder. A stored
    /// folder that is gone is SAID: the source names it and what is used instead.
    pub fn load(
        store: &dyn ModelStore,
        env: &dyn Fn(&str) -> Option<String>,
        files: &dyn KicadFiles,
    ) -> Self {
        let mut library = Self::seeded(env, files);
        let stored: Value = store
            .read(KICAD_LIBRARY_KEY)
            .and_then(|text| serde_json::from_str(&text).ok())
            .unwrap_or(Value::Null);
        for (key, folder, what) in [
            ("footprints", &mut library.footprints, "footprint"),
            ("models", &mut library.models, "3D-model"),
        ] {
            let Some(path) = stored.get(key).and_then(Value::as_str).filter(|p| !p.is_empty()) else {
                continue;
            };
            if files.is_dir(path) {
                *folder = Folder { path: path.into(), from: "your saved folder".into() };
            } else {
                folder.from = format!(
                    "your saved {what} folder {path} no longer exists, so this is {}",
                    folder.from
                );
            }
        }
        library
    }

    /// Remember the folders for the next session.
    pub fn save(&self, store: &dyn ModelStore) -> Result<(), String> {
        let record = json!({ "footprints": self.footprints.path, "models": self.models.path });
        store.write(KICAD_LIBRARY_KEY, &record.to_string())
    }

    /// Where footprint `library:name` is: `<footprints>/<library>.pretty/<name>.kicad_mod`.
    pub fn footprint_file(&self, library: &str, name: &str) -> String {
        join(&join(&self.footprints.path, &format!("{library}.pretty")), &format!("{name}.kicad_mod"))
    }

    /// A model path with KiCad's variables replaced: any of its 3D-model
    /// variables is the models folder, and `KIPRJMOD` the folder the footprint
    /// was read from. A relative path is relative to that folder too. Any other
    /// variable is refused by name.
    pub fn model_file(&self, path: &str, footprint_dir: Option<&str>) -> Result<String, String> {
        let mut out = String::new();
        let mut rest = path;
        while let Some(start) = rest.find('$') {
            let close = match rest[start + 1..].chars().next() {
                Some('{') => '}',
                Some('(') => ')',
                _ => {
                    out.push_str(&rest[..=start]);
                    rest = &rest[start + 1..];
                    continue;
                }
            };
            let Some(end) = rest[start + 2..].find(close) else {
                return Err(format!("the model path {path} has an unclosed variable"));
            };
            let name = &rest[start + 2..start + 2 + end];
            let value = if MODEL_VARS.contains(&name) || is_model_var(name) {
                self.models.path.clone()
            } else if name == "KIPRJMOD" {
                footprint_dir.map(str::to_owned).ok_or_else(|| {
                    format!("the model path {path} uses ${{KIPRJMOD}}, and there is no project folder")
                })?
            } else {
                return Err(format!(
                    "the model path {path} uses ${{{name}}}, which names no KiCad 3D-model folder"
                ));
            };
            out.push_str(&rest[..start]);
            out.push_str(&value);
            rest = &rest[start + 3 + end..];
        }
        out.push_str(rest);
        if Path::new(&out).is_relative() {
            if let Some(dir) = footprint_dir {
                return Ok(join(dir, &out));
            }
        }
        Ok(out)
    }
}

/// `KICAD<n>_3DMODEL_DIR` for any version `n`.
fn is_model_var(name: &str) -> bool {
    name.strip_prefix("KICAD")
        .and_then(|rest| rest.strip_suffix("_3DMODEL_DIR"))
        .is_some_and(|version| !version.is_empty() && version.chars().all(|c| c.is_ascii_digit()))
}

// ============================================================================
// Following the chain
// ============================================================================

/// A model's `.step`, read but not yet parsed. Parsing is the slow half: the
/// real KiCad 9.0.0 SOIC-8 model (254 KB) took 591 ms in a release build, so
/// the dialog parses on a reader thread ([`parse_step`]) and says so.
#[derive(Clone)]
pub struct StepSource {
    /// The `(model …)` entry as the footprint gives it.
    pub model: Model,
    /// The `.step` that was read.
    pub step_file: String,
    pub text: String,
}

/// The 3D model a footprint names, read and parsed.
#[derive(Clone)]
pub struct ModelFile {
    /// The `(model …)` entry as the footprint gives it.
    pub model: Model,
    /// The `.step` that was read.
    pub step_file: String,
    /// The file's bodies, in its own frame; placed at import.
    pub bodies: Vec<kernel::BrepSolid>,
    pub appearances: Vec<kernel::BodyAppearance>,
}

/// Everything one import brings, and what it could not.
#[derive(Clone, Default)]
pub struct KicadPart {
    pub symbol: Option<Symbol>,
    pub footprint: Option<Footprint>,
    /// Where the footprint was read from, when it came from the library.
    pub footprint_file: Option<String>,
    pub model: Option<ModelFile>,
    /// The model's `.step` while it waits to be parsed ([`KicadPart::parsed`]).
    pub step: Option<StepSource>,
    /// What the user is told: every link that failed, naming the file looked
    /// for, and the parsers' own warnings.
    pub notes: Vec<String>,
}

/// The symbols of a `.kicad_sym` library, named `nickname:symbol` after the
/// file, with the importer's warnings.
pub fn read_symbol_library(file: &str, bytes: &[u8]) -> Result<(Vec<Symbol>, Vec<String>), String> {
    let text = std::str::from_utf8(bytes).map_err(|_| format!("{file} is not UTF-8 text"))?;
    let report = kicad::import_library(text, &stem(file)).map_err(|error| format!("{file}: {error}"))?;
    if report.symbols.is_empty() {
        return Err(format!("{file} holds no symbol this importer reads"));
    }
    Ok((report.symbols, report.warnings))
}

/// How the footprint of a symbol import is found.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FootprintChoice {
    /// The one the symbol's `Footprint` property names.
    Named,
    /// One the user chose, `(library, name)`.
    Chosen(String, String),
    /// None: the part imports with its symbol only.
    Skip,
}

/// A symbol import: the symbol, then its footprint as `choice` says, then the
/// footprint's model.
pub fn part_from_symbol(
    symbol: Symbol,
    choice: &FootprintChoice,
    library: &KicadLibrary,
    files: &dyn KicadFiles,
) -> KicadPart {
    let id = symbol.library_id.clone();
    let mut part = KicadPart { symbol: Some(symbol), ..KicadPart::default() };
    let wanted = match (choice, kicad::footprint_link(part.symbol.as_ref().unwrap())) {
        (FootprintChoice::Chosen(lib, name), _) => Some((lib.clone(), name.clone())),
        (FootprintChoice::Skip, _) => {
            part.notes.push(format!("{id}: no footprint chosen, so the part imports with its symbol only"));
            None
        }
        (FootprintChoice::Named, FootprintLink::Named { library: lib, name }) if lib.is_empty() => {
            part.notes.push(format!(
                "{id} names footprint '{name}' without a library, so it cannot be found; \
                 the part imports with its symbol only"
            ));
            None
        }
        (FootprintChoice::Named, FootprintLink::Named { library: lib, name }) => Some((lib, name)),
        (FootprintChoice::Named, FootprintLink::Chosen { .. }) => {
            part.notes.push(format!("{id} names no footprint, so the part imports with its symbol only"));
            None
        }
    };
    let Some((lib, name)) = wanted else {
        return part;
    };
    if !files.available() {
        part.notes.push(format!(
            "this build cannot read your KiCad library, so footprint {lib}:{name} is not followed"
        ));
        return part;
    }
    let file = library.footprint_file(&lib, &name);
    let Some(bytes) = files.read(&file) else {
        part.notes.push(format!("footprint {lib}:{name} not found: looked for {file}"));
        return part;
    };
    match read_footprint(&file, &bytes) {
        Ok((footprint, warnings)) => {
            part.notes.extend(warnings);
            // The symbol keeps `library:name`, KiCad's footprint id, so a netlist
            // names a footprint Pcbnew can find (the third eCAD audit, B10).
            if let Some(symbol) = &mut part.symbol { kicad::assign_footprint(symbol, &lib, &name); }
            part.footprint = Some(footprint);
            part.footprint_file = Some(file);
            attach_model(&mut part, library, files);
        }
        Err(error) => part.notes.push(error),
    }
    part
}

/// A footprint import: the pads of `file`, then its model.
pub fn part_from_footprint(
    file: &str,
    bytes: &[u8],
    library: &KicadLibrary,
    files: &dyn KicadFiles,
) -> Result<KicadPart, String> {
    let (footprint, warnings) = read_footprint(file, bytes)?;
    let mut part = KicadPart {
        footprint: Some(footprint),
        footprint_file: Some(file.into()),
        notes: warnings,
        ..KicadPart::default()
    };
    attach_model(&mut part, library, files);
    Ok(part)
}

fn read_footprint(file: &str, bytes: &[u8]) -> Result<(Footprint, Vec<String>), String> {
    let text = std::str::from_utf8(bytes).map_err(|_| format!("{file} is not UTF-8 text"))?;
    let (footprint, warnings) =
        kicad::import_footprint(text).map_err(|error| format!("footprint {file}: {error}"))?;
    Ok((footprint, warnings.into_iter().map(|w| format!("footprint {}: {w}", stem(file))).collect()))
}

/// The last two links: the footprint's `(model …)` entry, then the `.step`
/// beside the `.wrl` it names.
fn attach_model(part: &mut KicadPart, library: &KicadLibrary, files: &dyn KicadFiles) {
    let footprint = part.footprint.as_ref().expect("a footprint to follow");
    let Some(model) = footprint.model.clone() else {
        part.notes.push(format!(
            "footprint {} names no 3D model, so the part imports without one",
            footprint.name
        ));
        return;
    };
    if !files.available() {
        part.notes.push(format!(
            "this build cannot read your KiCad library, so the 3D model {} is not read",
            model.path
        ));
        return;
    }
    let beside = part.footprint_file.as_deref().and_then(parent);
    let step_file = match library.model_file(&model.step_path(), beside.as_deref()) {
        Ok(file) => file,
        Err(error) => {
            part.notes.push(format!("3D model not read: {error}"));
            return;
        }
    };
    let Some(bytes) = files.read(&step_file) else {
        let named = library.model_file(&model.path, beside.as_deref()).unwrap_or_default();
        part.notes.push(if named != step_file && files.is_file(&named) {
            format!(
                "3D model: the library has {named} but no {step_file} beside it, and BREP imports \
                 STEP, not VRML; the part imports without a model"
            )
        } else {
            format!("3D model not found: looked for {step_file}; the part imports without a model")
        });
        return;
    };
    match String::from_utf8(bytes) {
        Ok(text) => part.step = Some(StepSource { model, step_file, text }),
        Err(_) => part.notes.push(format!(
            "3D model {step_file} is not UTF-8 text; the part imports without a model"
        )),
    }
}

/// Parse a model's `.step` into its bodies: the slow half of the chain, which
/// the dialog runs on a reader thread.
pub fn parse_step(source: &StepSource) -> Result<ModelFile, String> {
    let (bodies, appearances) = kernel::import_step_with_appearance(&source.text).map_err(|error| {
        format!(
            "3D model {} did not import ({error}); the part imports without a model",
            source.step_file
        )
    })?;
    Ok(ModelFile { model: source.model.clone(), step_file: source.step_file.clone(), bodies, appearances })
}

impl KicadPart {
    /// Take a parse's outcome: the model, or a note saying why there is none.
    pub fn settle_model(&mut self, parsed: Result<ModelFile, String>) {
        self.step = None;
        match parsed {
            Ok(model) => self.model = Some(model),
            Err(note) => self.notes.push(note),
        }
    }

    /// The part with its model's `.step` parsed here and now.
    pub fn parsed(mut self) -> Self {
        if let Some(step) = self.step.take() {
            let outcome = parse_step(&step);
            self.settle_model(outcome);
        }
        self
    }
}

/// The footprints a generic symbol's patterns offer, `(library, name)`, from
/// every `*.pretty` folder under the footprints folder, sorted.
pub fn footprint_candidates(
    filters: &[String],
    library: &KicadLibrary,
    files: &dyn KicadFiles,
) -> Vec<(String, String)> {
    let mut found = Vec::new();
    for entry in files.list(&library.footprints.path) {
        let Some(lib) = entry.strip_suffix(".pretty") else {
            continue;
        };
        for file in files.list(&join(&library.footprints.path, &entry)) {
            let Some(name) = file.strip_suffix(".kicad_mod") else {
                continue;
            };
            if kicad::footprint_filter_matches(filters, lib, name) {
                found.push((lib.to_owned(), name.to_owned()));
            }
        }
    }
    found.sort();
    found
}

// ============================================================================
// The import itself
// ============================================================================

/// What an import did, for the status line and the toast.
#[derive(Clone, Debug, Default)]
pub struct Imported {
    pub summary: String,
    pub notes: Vec<String>,
}

/// Where a pad centre is in the part: `(x / 1000, −y / 1000, 0)` mm (see the
/// module doc for the frame).
pub fn pad_point(footprint: &Footprint, index: usize) -> [f64; 3] {
    let at = footprint.pads[index].at;
    [f64::from(at.x) / 1000., -f64::from(at.y) / 1000., 0.]
}

/// A new connection point's direction: +Z, up out of the board, the way a lead
/// or a wire leaves a pad. A point's direction is its transform's rotated +X
/// (`feature_pipeline/ports.rs`), and `Ry(-90 deg)` turns +X onto +Z.
const PORT_UP: [f64; 3] = [0., -90., 0.];
/// The port group a KiCad import declares its pins' points in, and its purpose:
/// these points ARE the part's pads, so the group is a `pcb` one.
const IMPORT_PORT: &str = "Pins";
const IMPORT_PURPOSE: &str = "pcb";

/// Write `part` into the open part document as ONE undoable edit.
///
/// Refused, with nothing written, when the part already has the block this
/// import would write: a KiCad import does not replace a symbol or pads.
pub fn import_part(state: &mut EngineState, part: &KicadPart) -> Result<Imported, String> {
    let parsed;
    let part = if part.step.is_some() {
        parsed = part.clone().parsed();
        &parsed
    } else {
        part
    };
    if part.symbol.is_none() && part.footprint.is_none() {
        return Err("nothing to import".into());
    }
    if part.symbol.is_some() && state.history.symbol_block().is_some() {
        return Err("this part already has a symbol, and a KiCad import does not replace one".into());
    }
    if part.footprint.is_some() && state.history.pads_block().is_some() {
        return Err("this part already has pads, and a KiCad import does not replace them".into());
    }
    let symbol_block = part.symbol.as_ref().map(serde_json::to_value).transpose().map_err(|e| e.to_string())?;
    let pads_block = part.footprint.as_ref().map(serde_json::to_value).transpose().map_err(|e| e.to_string())?;

    let mut notes = part.notes.clone();
    let mut features = Vec::new();
    let mut summary = Vec::new();
    if let Some(symbol) = &part.symbol {
        summary.push(format!("symbol {} ({} pins)", symbol.library_id, symbol.pins.len()));
    }
    if let Some(footprint) = &part.footprint {
        summary.push(format!("{} pads", footprint.pads.len()));
    }
    if let Some(model) = &part.model {
        features.push(model_feature(state, model)?);
        summary.push(format!("3D model ({} bodies)", model.bodies.len()));
    }
    let mut ports_block = None;
    if let Some(symbol) = &part.symbol {
        let (block, port_notes) = port_points(state, symbol, part.footprint.as_ref());
        notes.extend(port_notes);
        let added = block["points"].as_array().map_or(0, Vec::len);
        if added > 0 {
            summary.push(format!("{added} connection points"));
            ports_block = Some(block);
        }
    }

    // The one checkpoint: the features' when there are any, else the first
    // block's own. Every later write falls after it.
    let mut checkpointed = !features.is_empty();
    if checkpointed {
        state.add_features(&features);
    }
    // Before the symbol, so the symbol write's own pin follow finds each pin's
    // point already declared and adds none.
    if let Some(block) = ports_block {
        let mut blocks = state
            .history
            .ports_block()
            .and_then(|value| value.as_array().cloned())
            .unwrap_or_default();
        blocks.push(block);
        let blocks = Some(serde_json::Value::Array(blocks));
        if checkpointed {
            state.history.set_ports_block_no_undo(blocks);
        } else {
            state.history.set_ports_block(blocks, None);
            checkpointed = true;
        }
    }
    if let Some(block) = symbol_block {
        if checkpointed {
            state.history.set_symbol_block_no_undo(Some(block));
        } else {
            state.history.set_symbol_block(Some(block), None);
            checkpointed = true;
        }
    }
    if let Some(block) = pads_block {
        if checkpointed {
            state.history.set_pads_block_no_undo(Some(block));
        } else {
            state.history.set_pads_block(Some(block), None);
        }
    }

    // Writing the symbol ran History's pin/point binding. Each pin found the
    // point declared for it above (a pin pairs with the point carrying its
    // name), so it added none; what it could not bind, it says.
    if let Some(hold) = state.history.pin_port_hold() {
        notes.push(format!("the pins were not bound to connection points: {hold}"));
    }
    if let Some(report) = state.history.pin_point_report() {
        notes.extend(report.problems.iter().map(|problem| problem.message()));
    }
    Ok(Imported { summary: format!("imported {}", summary.join(", ")), notes })
}

/// The IMPORT3D feature for the model: its bodies moved by KiCad's matrix and
/// sealed, under their final names, into a `nativeBrep` payload.
fn model_feature(state: &mut EngineState, file: &ModelFile) -> Result<Value, String> {
    let transform = kernel::AffineTransform::new(kicad::model_placement(&file.model))
        .map_err(|error| format!("3D model {}: {error}", file.step_file))?;
    // A negative scale mirrors, and a mirror must reverse the faces or the
    // solid turns inside out (`transform_brep` refuses it otherwise).
    let mirrored = transform.determinant3() < 0.0;
    let bodies = file
        .bodies
        .iter()
        .map(|body| kernel::transform_brep(body, transform, mirrored))
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| format!("3D model {}: {error}", file.step_file))?;
    let id = state.next_feature_id(&brep_render::features::feature_short_name("IMPORT3D"));
    // The file's colours are stamped into the payload, not into the open
    // document's metadata: the feature's run merges them back from there.
    let payload = {
        let _isolation = kernel::IsolatedSceneMetadata::begin();
        kernel::native_import_payload_with_appearance(&id, &bodies, &file.appearances)
    }
    .map_err(|error| format!("3D model {}: {error}", file.step_file))?;
    let model = &file.model;
    Ok(json!({
        "type": "IMPORT3D",
        "inputParams": { "id": id, "nativeBrep": payload },
        "persistentData": {
            "kicadModel": {
                "path": model.path,
                "stepFile": file.step_file,
                "offset": model.offset,
                "scale": model.scale,
                "rotation": model.rotation,
            }
        },
    }))
}

/// One connection point per symbol pin, NAMED with the pin's label, at the
/// FIRST pad carrying that label — as one declared port group. A pin whose
/// name the part already declares keeps that point where it is. Returns the
/// group and what the user is told: a pin on several pads, a pin on none.
fn port_points(
    state: &EngineState,
    symbol: &Symbol,
    footprint: Option<&Footprint>,
) -> (Value, Vec<String>) {
    let existing = state.history.declared_points();
    let mut points = Vec::new();
    let mut notes = Vec::new();
    if footprint.is_none() && !symbol.pins.is_empty() {
        notes.push("no pads were imported, so every new connection point sits at the part origin".into());
    }
    for pin in &symbol.pins {
        let label = &pin.number;
        if let Some(point) = existing.iter().find(|point| &point.point == label) {
            notes.push(format!(
                "pin '{label}' takes the part's existing connection point {}, which stays where it is",
                point.address()
            ));
            continue;
        }
        let mut position = [0., 0., 0.];
        if let Some(footprint) = footprint {
            let pads: Vec<usize> =
                (0..footprint.pads.len()).filter(|&i| &footprint.pads[i].number == label).collect();
            match pads.as_slice() {
                [] => notes.push(format!("pin '{label}' matches no pad, so its point sits at the part origin")),
                [first, rest @ ..] => {
                    position = pad_point(footprint, *first);
                    if !rest.is_empty() {
                        notes.push(format!(
                            "pin '{label}' is on {} pads; its point is on the first, at ({}, {}) mm",
                            pads.len(),
                            position[0],
                            position[1]
                        ));
                    }
                }
            }
        }
        points.push(json!({
            "name": label,
            "transform": { "position": position, "rotationEuler": PORT_UP }
        }));
    }
    (json!({ "name": IMPORT_PORT, "purpose": IMPORT_PURPOSE, "points": points }), notes)
}

// ============================================================================
// The dialog
// ============================================================================

/// Where a reviewed part came from, so new library folders can re-read it.
#[derive(Clone)]
enum Origin {
    Symbol { symbol: Symbol, choice: FootprintChoice },
    Footprint { file: String, bytes: Vec<u8> },
}

enum Stage {
    /// Choosing the file; the file dialog's explorer draws this one.
    File,
    /// A library holding several symbols: choose one.
    Symbol { file: String, symbols: Vec<Symbol>, warnings: Vec<String>, selected: Option<usize> },
    /// A generic symbol: choose its footprint among those its patterns offer.
    Footprint {
        symbol: Symbol,
        warnings: Vec<String>,
        filters: Vec<String>,
        candidates: Vec<(String, String)>,
        selected: Option<usize>,
    },
    /// Everything read: the user confirms.
    Review {
        origin: Origin,
        warnings: Vec<String>,
        part: KicadPart,
        /// The reader parsing the model's `.step`, while it runs.
        reading: Option<Reader>,
        /// The symbol and its file's warnings when the chain STOPPED at the
        /// footprint folder (a generic symbol, and no folder to offer
        /// footprints from). New folders then re-run the chain from the
        /// symbol, which offers the footprints the review could not; reading
        /// the review again would only repeat "no footprint folder".
        rechoose: Option<(Symbol, Vec<String>)>,
    },
}

/// What a click in a stage asks for, done once the stage is drawn.
enum Step {
    ChooseSymbol(Symbol, Vec<String>),
    Review(Origin, Vec<String>),
}

/// A `.step` being parsed off the UI thread: the answer arrives on `answer`.
struct Reader {
    answer: std::sync::mpsc::Receiver<Result<ModelFile, String>>,
    /// The file's size, for the progress line.
    bytes: usize,
}

impl Reader {
    /// Start parsing `source`. The native build parses on a thread of its own;
    /// the web build never gets here (it cannot read a KiCad library, so a part
    /// there has no `.step` to parse) and would parse in place.
    fn start(source: StepSource) -> Self {
        let bytes = source.text.len();
        let (send, answer) = std::sync::mpsc::channel();
        #[cfg(not(target_arch = "wasm32"))]
        std::thread::spawn(move || {
            // A dialog cancelled meanwhile has dropped the receiver; nobody waits.
            let _ = send.send(parse_step(&source));
        });
        #[cfg(target_arch = "wasm32")]
        let _ = send.send(parse_step(&source));
        Self { answer, bytes }
    }
}

/// What a frame of the dialog came to.
pub enum Outcome {
    Open,
    Closed,
    Imported(Imported),
}

/// The KiCad import modal's state, owned by the file dialog, which draws its
/// file-choosing stage with its own explorer.
pub struct KicadImport {
    kind: KicadKind,
    stage: Stage,
    library: Option<KicadLibrary>,
    footprints_buf: String,
    models_buf: String,
    search: String,
    pub status: String,
}

fn process_env(var: &str) -> Option<String> {
    std::env::var(var).ok().filter(|value| !value.is_empty())
}

impl KicadImport {
    pub fn new() -> Self {
        Self {
            kind: KicadKind::Symbol,
            stage: Stage::File,
            library: None,
            footprints_buf: String::new(),
            models_buf: String::new(),
            search: String::new(),
            status: String::new(),
        }
    }

    pub fn kind(&self) -> KicadKind {
        self.kind
    }

    /// Begin an import of `kind`: the file comes next.
    pub fn start(&mut self, kind: KicadKind, store: &dyn ModelStore) {
        self.kind = kind;
        self.stage = Stage::File;
        self.search.clear();
        self.status.clear();
        self.set_library(KicadLibrary::load(store, &process_env, &DiskFiles));
    }

    fn set_library(&mut self, library: KicadLibrary) {
        self.footprints_buf = library.footprints.path.clone();
        self.models_buf = library.models.path.clone();
        self.library = Some(library);
    }

    /// Whether the file is still to be chosen.
    pub fn wants_file(&self) -> bool {
        matches!(self.stage, Stage::File)
    }

    /// The chosen file arrived: parse it and go on to what it needs next.
    /// `file` is its full path where the store has one, which is where a
    /// footprint's relative model path starts from.
    pub fn take_file(&mut self, file: &str, bytes: &[u8], files: &dyn KicadFiles) {
        let Some(kind) = KicadKind::of_file(file) else {
            self.status = format!("{file} is not a .kicad_sym or .kicad_mod file");
            return;
        };
        self.kind = kind;
        let library = self.library.clone().unwrap_or_else(|| KicadLibrary::seeded(&process_env, files));
        self.library = Some(library.clone());
        match kind {
            KicadKind::Symbol => match read_symbol_library(file, bytes) {
                Ok((mut symbols, warnings)) if symbols.len() == 1 => {
                    self.choose_symbol(symbols.remove(0), warnings, files)
                }
                Ok((symbols, warnings)) => {
                    self.search.clear();
                    self.stage = Stage::Symbol { file: file.into(), symbols, warnings, selected: None };
                }
                Err(error) => self.status = format!("import failed: {error}"),
            },
            KicadKind::Footprint => {
                let origin = Origin::Footprint { file: file.into(), bytes: bytes.to_vec() };
                self.review(origin, Vec::new(), files);
            }
        }
    }

    /// A symbol is chosen: follow its footprint, or offer the footprints its
    /// patterns match.
    fn choose_symbol(&mut self, symbol: Symbol, warnings: Vec<String>, files: &dyn KicadFiles) {
        let library = self.library.clone().expect("a library once a file is taken");
        match kicad::footprint_link(&symbol) {
            FootprintLink::Chosen { filters } if files.available() && files.is_dir(&library.footprints.path) => {
                let candidates = footprint_candidates(&filters, &library, files);
                self.search.clear();
                self.stage = Stage::Footprint { symbol, warnings, filters, candidates, selected: None };
            }
            FootprintLink::Chosen { .. } => {
                let rechoose = (symbol.clone(), warnings.clone());
                let mut warnings = warnings;
                warnings.push(if files.available() {
                    format!(
                        "no footprint folder at {}, so no footprint can be chosen",
                        library.footprints.path
                    )
                } else {
                    "this build cannot read your KiCad library, so no footprint can be chosen".into()
                });
                self.review(Origin::Symbol { symbol, choice: FootprintChoice::Skip }, warnings, files);
                if let Stage::Review { rechoose: slot, .. } = &mut self.stage {
                    *slot = Some(rechoose);
                }
            }
            FootprintLink::Named { .. } => {
                self.review(Origin::Symbol { symbol, choice: FootprintChoice::Named }, warnings, files)
            }
        }
    }

    fn review(&mut self, origin: Origin, warnings: Vec<String>, files: &dyn KicadFiles) {
        let library = self.library.clone().expect("a library once a file is taken");
        let part = match &origin {
            Origin::Symbol { symbol, choice } => part_from_symbol(symbol.clone(), choice, &library, files),
            Origin::Footprint { file, bytes } => match part_from_footprint(file, bytes, &library, files) {
                Ok(part) => part,
                Err(error) => {
                    self.status = format!("import failed: {error}");
                    return;
                }
            },
        };
        let mut part = part;
        let reading = part.step.take().map(Reader::start);
        self.stage = Stage::Review { origin, warnings, part, reading, rechoose: None };
    }

    /// Take the reader's answer if it has come. While it has not, the dialog
    /// asks for another frame, so the answer shows without the pointer moving.
    fn poll_reader(&mut self, ctx: &egui::Context) {
        let Stage::Review { part, reading: Some(reader), .. } = &mut self.stage else {
            return;
        };
        match reader.answer.try_recv() {
            Ok(parsed) => part.settle_model(parsed),
            Err(std::sync::mpsc::TryRecvError::Empty) => {
                ctx.request_repaint_after(std::time::Duration::from_millis(50));
                return;
            }
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                part.notes.push("the 3D model's reader stopped without an answer; the part imports without a model".into())
            }
        }
        if let Stage::Review { reading, .. } = &mut self.stage {
            *reading = None;
        }
    }

    /// Whether the model's `.step` is still being parsed.
    pub fn reading(&self) -> bool {
        matches!(self.stage, Stage::Review { reading: Some(_), .. })
    }


    /// Where the dialog is, for the published file state: its kind, its
    /// stage, and what a review would import.
    pub fn state_json(&self) -> Value {
        let stage = match &self.stage {
            Stage::File => "file",
            Stage::Symbol { .. } => "symbol",
            Stage::Footprint { .. } => "footprint",
            Stage::Review { .. } => "review",
        };
        let review = self.reviewed().map(|part| {
            json!({
                "symbol": part.symbol.as_ref().map(|s| s.library_id.clone()),
                "pads": part.footprint.as_ref().map(|f| f.pads.len()),
                "footprintFile": part.footprint_file,
                "stepFile": part.model.as_ref().map(|m| m.step_file.clone()),
                "notes": part.notes,
                "reading": self.reading(),
            })
        });
        json!({
            "kind": self.kind.extension(),
            "stage": stage,
            "status": self.status,
            "footprints": self.library.as_ref().map(|l| l.footprints.path.clone()),
            "models": self.library.as_ref().map(|l| l.models.path.clone()),
            "review": review,
        })
    }

    /// The part the dialog would import now, once it has been read.
    pub fn reviewed(&self) -> Option<&KicadPart> {
        match &self.stage {
            Stage::Review { part, .. } => Some(part),
            _ => None,
        }
    }

    /// Import the reviewed part; the dialog closes on success.
    pub fn confirm(&mut self, state: &mut EngineState) -> Result<Imported, String> {
        let Stage::Review { part, warnings, reading, .. } = &self.stage else {
            return Err("nothing has been read yet".into());
        };
        if reading.is_some() {
            return Err("the 3D model is still being read".into());
        }
        let mut imported = import_part(state, part)?;
        imported.notes.splice(0..0, warnings.iter().cloned());
        self.stage = Stage::File;
        Ok(imported)
    }

    /// Draw the stages after the file: the symbol list, the footprint list,
    /// the review. Hit keys are `kicad:*` (see `file::HIT_KEYS`).
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        state: &mut EngineState,
        store: &dyn ModelStore,
        hits: &mut std::collections::HashMap<String, egui::Rect>,
    ) -> Outcome {
        self.show_with(ctx, state, store, hits, &DiskFiles)
    }

    /// [`Self::show`] reading the library through `files`.
    ///
    /// The layout is three bands. The folders, the stage's question and its
    /// search field sit at the top; the stage's buttons (Next, Use this
    /// footprint, Import) sit in the footer with Cancel; only the LIST between
    /// them scrolls. With Next laid out after the list, a 100-row search put it
    /// at y = 2540 in a 960-point window (ecad-reaudit-2026-09-23, B3).
    fn show_with(
        &mut self,
        ctx: &egui::Context,
        state: &mut EngineState,
        store: &dyn ModelStore,
        hits: &mut std::collections::HashMap<String, egui::Rect>,
        files: &dyn KicadFiles,
    ) -> Outcome {
        self.poll_reader(ctx);
        let reading = self.reading();
        let mut outcome = Outcome::Open;
        let mut step: Option<Step> = None;
        let mut hit = |key: &str, response: &egui::Response| {
            hits.insert(format!("kicad:{key}"), response.rect);
        };
        let modal = egui::Modal::new(egui::Id::new("brep-kicad-import")).show(ctx, |ui| {
            super::file_explorer::dialog_body(ui, |ui| {
                ui.heading("Import from KiCad");
                ui.add_space(4.0);
                super::file_explorer::dialog_footer(ui, "kicad", |ui| {
                    if !self.status.is_empty() {
                        ui.add_space(4.0);
                        ui.weak(&self.status);
                    }
                    ui.horizontal(|ui| {
                        let cancel = ui.button("Cancel");
                        hit("cancel", &cancel);
                        if cancel.clicked() {
                            outcome = Outcome::Closed;
                        }
                        step = step.take().or(self.show_buttons(ui, &mut hit));
                        if matches!(self.stage, Stage::Review { .. }) {
                            let import = ui.add_enabled(!reading, egui::Button::new("Import"));
                            hit("import", &import);
                            if import.clicked() {
                                match self.confirm(state) {
                                    Ok(imported) => outcome = Outcome::Imported(imported),
                                    Err(error) => self.status = format!("import refused: {error}"),
                                }
                            }
                        }
                    });
                });
                self.show_library(ui, store, files, &mut hit);
                ui.separator();
                self.show_question(ui, &mut hit);
                egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                    step = step.take().or(self.show_list(ui, &mut hit));
                });
            })
        });
        match step {
            Some(Step::ChooseSymbol(symbol, warnings)) => self.choose_symbol(symbol, warnings, files),
            Some(Step::Review(origin, warnings)) => self.review(origin, warnings, files),
            None => {}
        }
        if modal.should_close() && matches!(outcome, Outcome::Open) {
            outcome = Outcome::Closed;
        }
        outcome
    }

    /// The two library folders, where each came from, and Apply.
    fn show_library(
        &mut self,
        ui: &mut egui::Ui,
        store: &dyn ModelStore,
        files: &dyn KicadFiles,
        hit: &mut impl FnMut(&str, &egui::Response),
    ) {
        let Some(library) = self.library.clone() else {
            return;
        };
        // Rows, not a grid: where each folder came from can name a path, and a
        // grid cell is as wide as its widest content ([`path_label`]).
        ui.horizontal(|ui| {
            ui.label("Footprints");
            let field = ui.text_edit_singleline(&mut self.footprints_buf);
            hit("field:footprints", &field);
        });
        path_label(ui, &library.footprints.from);
        ui.horizontal(|ui| {
            ui.label("3D models");
            let field = ui.text_edit_singleline(&mut self.models_buf);
            hit("field:models", &field);
        });
        path_label(ui, &library.models.from);
        let changed = self.footprints_buf != library.footprints.path || self.models_buf != library.models.path;
        let apply = ui.add_enabled(changed, egui::Button::new("Use these folders"));
        hit("apply-folders", &apply);
        if apply.clicked() {
            self.apply_folders(store, files);
        }
    }

    /// Take the typed folders, remember them, and read the chain again from
    /// them without the user starting over.
    fn apply_folders(&mut self, store: &dyn ModelStore, files: &dyn KicadFiles) {
        let Some(library) = self.library.clone() else {
            return;
        };
        let chosen = |path: &str, old: &Folder| {
            if path == old.path {
                old.clone()
            } else if files.is_dir(path) {
                Folder { path: path.into(), from: "your saved folder".into() }
            } else {
                Folder { path: path.into(), from: format!("{path} is not a folder") }
            }
        };
        let library = KicadLibrary {
            footprints: chosen(&self.footprints_buf, &library.footprints),
            models: chosen(&self.models_buf, &library.models),
        };
        self.status = match library.save(store) {
            Ok(()) => String::new(),
            Err(error) => format!("the folders were not saved: {error}"),
        };
        self.library = Some(library);
        // Read the chain again from the new folders: from the symbol when
        // the chain stopped at the footprint folder, else from the review.
        match &mut self.stage {
            Stage::Review { rechoose: Some((symbol, warnings)), .. } => {
                let (symbol, warnings) = (symbol.clone(), warnings.clone());
                self.choose_symbol(symbol, warnings, files);
            }
            Stage::Review { origin, warnings, .. } => {
                let (origin, warnings) = (origin.clone(), warnings.clone());
                self.review(origin, warnings, files);
            }
            Stage::Footprint { symbol, warnings, .. } => {
                let (symbol, warnings) = (symbol.clone(), warnings.clone());
                self.choose_symbol(symbol, warnings, files);
            }
            Stage::File | Stage::Symbol { .. } => {}
        }
    }

    /// The stage's question and its search field: above the list, so they
    /// stay put while it scrolls.
    fn show_question(&mut self, ui: &mut egui::Ui, hit: &mut impl FnMut(&str, &egui::Response)) {
        match &self.stage {
            Stage::File | Stage::Review { .. } => return,
            Stage::Symbol { file, symbols, .. } => {
                // The file's NAME: its folder is where the user just was.
                let name = file.rsplit(['/', '\\']).next().unwrap_or(file);
                ui.label(format!("{name} holds {} symbols. Choose one.", symbols.len()))
                    .on_hover_text(file.as_str());
            }
            Stage::Footprint { symbol, filters, .. } => {
                ui.label(format!(
                    "{} names no footprint. It offers footprints matching {}.",
                    symbol.library_id,
                    if filters.is_empty() { "anything".to_string() } else { filters.join(" ") }
                ));
            }
        }
        let search = ui.add(
            egui::TextEdit::singleline(&mut self.search).hint_text("search: an exact name comes first"),
        );
        hit("search", &search);
    }

    /// The stage's rows, best match first. A double click on a row chooses
    /// it and goes on, as the stage's button would.
    fn show_list(&mut self, ui: &mut egui::Ui, hit: &mut impl FnMut(&str, &egui::Response)) -> Option<Step> {
        let needle = self.search.clone();
        match &mut self.stage {
            Stage::File => {
                ui.weak("Choose a file.");
                None
            }
            Stage::Symbol { symbols, warnings, selected, .. } => {
                let mut opened = None;
                for index in ranked(symbols.iter().map(|s| s.library_id.as_str()), &needle) {
                    let symbol = &symbols[index];
                    let row = ui.selectable_label(
                        *selected == Some(index),
                        format!("{}  ({} pins)", symbol.library_id, symbol.pins.len()),
                    );
                    hit(&format!("symbol:{}", symbol.library_id), &row);
                    if row.clicked() {
                        *selected = Some(index);
                    }
                    if opens(&row) {
                        opened = Some(Step::ChooseSymbol(symbol.clone(), warnings.clone()));
                    }
                }
                opened
            }
            Stage::Footprint { symbol, warnings, candidates, selected, .. } => {
                let labels: Vec<String> = candidates.iter().map(|(lib, name)| format!("{lib}:{name}")).collect();
                let mut opened = None;
                for (shown, index) in ranked(labels.iter().map(String::as_str), &needle).into_iter().enumerate() {
                    if shown == 500 {
                        ui.weak("\u{2026} type to narrow the list");
                        break;
                    }
                    let label = &labels[index];
                    let row = ui.selectable_label(*selected == Some(index), label);
                    hit(&format!("footprint:{label}"), &row);
                    if row.clicked() {
                        *selected = Some(index);
                    }
                    if opens(&row) {
                        let (lib, name) = candidates[index].clone();
                        let choice = FootprintChoice::Chosen(lib, name);
                        opened = Some(Step::Review(Origin::Symbol { symbol: symbol.clone(), choice }, warnings.clone()));
                    }
                }
                if candidates.is_empty() {
                    ui.weak("No footprint in the library matches.");
                }
                opened
            }
            Stage::Review { warnings, part, reading, .. } => {
                show_review(ui, part, warnings, reading.as_ref().map(|reader| reader.bytes));
                None
            }
        }
    }

    /// The stage's own buttons, in the footer beside Cancel.
    fn show_buttons(
        &mut self,
        ui: &mut egui::Ui,
        hit: &mut impl FnMut(&str, &egui::Response),
    ) -> Option<Step> {
        match &self.stage {
            Stage::Symbol { symbols, warnings, selected, .. } => {
                let next = ui.add_enabled(selected.is_some(), egui::Button::new("Next"));
                hit("next", &next);
                let symbol = selected.map(|index| symbols[index].clone())?;
                next.clicked().then(|| Step::ChooseSymbol(symbol, warnings.clone()))
            }
            Stage::Footprint { symbol, warnings, candidates, selected, .. } => {
                let use_it = ui.add_enabled(selected.is_some(), egui::Button::new("Use this footprint"));
                hit("use-footprint", &use_it);
                let skip = ui.button("No footprint");
                hit("no-footprint", &skip);
                let choice = if use_it.clicked() {
                    let (lib, name) = candidates[selected.unwrap()].clone();
                    FootprintChoice::Chosen(lib, name)
                } else if skip.clicked() {
                    FootprintChoice::Skip
                } else {
                    return None;
                };
                Some(Step::Review(Origin::Symbol { symbol: symbol.clone(), choice }, warnings.clone()))
            }
            Stage::File | Stage::Review { .. } => None,
        }
    }
}

/// Whether a click on a list row opens it: a double click, or a triple one.
/// egui counts clicks by TIME alone, not place, so a click in the search field
/// a moment before makes the row's double click its triple: the audit's
/// double click on `Device:R` came within 0.3 s of another click and never
/// read as double (ecad-reaudit-2026-09-23, B3).
fn opens(row: &egui::Response) -> bool {
    row.double_clicked() || row.triple_clicked()
}

/// The indices of `ids` that `needle` matches, best first: an exact name, then
/// a name that starts with it, then a name that holds it anywhere; the file's
/// order within each. An id is `library:name`, and the needle is matched
/// against both the whole id and the name after its `:` (a leading `:` in the
/// needle anchors it to the name), ignoring case. Before this ranking a search
/// for "R" put `Device:R` some 250 rows down, under every symbol holding an r.
fn ranked<'a>(ids: impl Iterator<Item = &'a str>, needle: &str) -> Vec<usize> {
    let needle = needle.trim().to_lowercase();
    let bare = needle.strip_prefix(':').unwrap_or(&needle);
    let mut found: Vec<(u8, usize)> = ids
        .enumerate()
        .filter_map(|(index, id)| {
            let id = id.to_lowercase();
            let name = id.rsplit_once(':').map_or(id.as_str(), |(_, name)| name);
            let rank = if needle.is_empty() {
                2
            } else if id == needle || name == bare {
                0
            } else if id.starts_with(&needle) || name.starts_with(bare) {
                1
            } else if id.contains(&needle) {
                2
            } else {
                return None;
            };
            Some((rank, index))
        })
        .collect();
    found.sort();
    found.into_iter().map(|(_, index)| index).collect()
}

/// A file path in the width the dialog has: one line, cut at the end with an
/// ellipsis, the whole path on hover. Laid out at its natural width, a
/// 203-character footprint path made the review 2046 points wide, and Import
/// and Cancel went off a 1400-point window.
fn path_label(ui: &mut egui::Ui, path: &str) {
    ui.add(egui::Label::new(egui::RichText::new(path).weak()).truncate())
        .on_hover_text(path);
}

/// What the import will write, and every note, before the user confirms. The
/// short facts sit in a grid; each file PATH gets a line of its own under its
/// fact, as wide as the dialog and cut to fit it ([`path_label`]), because a
/// grid cell is as wide as its widest content.
fn show_review(ui: &mut egui::Ui, part: &KicadPart, warnings: &[String], reading: Option<usize>) {
    let fact = |ui: &mut egui::Ui, name: &str, value: String| {
        ui.horizontal_wrapped(|ui| {
            ui.label(egui::RichText::new(name).strong());
            ui.label(value);
        });
    };
    fact(
        ui,
        "Symbol",
        part.symbol.as_ref().map_or("none".into(), |s| format!("{} ({} pins)", s.library_id, s.pins.len())),
    );
    fact(
        ui,
        "Pads",
        part.footprint.as_ref().map_or("none".into(), |f| format!("{} ({} pads)", f.name, f.pads.len())),
    );
    if let (Some(_), Some(file)) = (&part.footprint, &part.footprint_file) {
        path_label(ui, file);
    }
    match (&part.model, reading) {
        (_, Some(bytes)) => {
            ui.horizontal_wrapped(|ui| {
                ui.label(egui::RichText::new("3D model").strong());
                ui.spinner();
                ui.label(format!("reading the model's STEP file ({} KB)\u{2026}", bytes.div_ceil(1024)));
            });
        }
        (Some(m), None) => {
            fact(ui, "3D model", format!("{} bodies", m.bodies.len()));
            path_label(ui, &m.step_file);
            ui.weak(format!(
                "offset {:?} mm, rotation {:?}°, scale {:?}",
                m.model.offset, m.model.rotation, m.model.scale
            ));
        }
        (None, None) => fact(ui, "3D model", "none".into()),
    }
    if let Some(symbol) = &part.symbol {
        fact(
            ui,
            "Connection points",
            format!(
                "{} in the part's `Pins` group, {}",
                symbol.pins.len(),
                if part.footprint.is_some() { "each at its pin's pad" } else { "at the part origin" }
            ),
        );
    }
    let notes: Vec<&String> = warnings.iter().chain(&part.notes).collect();
    if !notes.is_empty() {
        ui.add_space(6.0);
        for note in notes {
            // A note names the file it looked for, so it WRAPS (it is a
            // sentence, not a path to scan), breaking inside a long path too.
            ui.add(
                egui::Label::new(
                    egui::RichText::new(format!("\u{2022} {note}")).color(ui.visuals().warn_fg_color),
                )
                .wrap_mode(egui::TextWrapMode::Wrap),
            );
        }
    }
}

