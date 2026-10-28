//! The KiCad library from a session: search the symbols and footprints on
//! disk, then add one as a component of the open assembly. User report
//! 20261024T213219Z-8cc7efda: an agent could only reach the library by
//! clicking through the Import dialog's stages, and the "Add part to
//! assembly" picker offers stored parts, never the library.
//!
//! The part is made the way the dialog makes it (`kicad_import::part_from_*`,
//! `kicad_bulk::write_part` into the models folder) and enters the assembly
//! through the one insert flow (`EngineState::insert_component`): a PCB is an
//! assembly, and a symbol or footprint enters it only as a component.
//!
//! Native only: it reads the library off the filesystem. Nothing here
//! downloads a library (the KiCad 9 libraries are ~5.6 GB of git); the
//! folders come from the arguments, else the same places the dialog looks.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, Outcome, Phase};
use crate::panels::kicad_import::{self, DiskFiles, Folder, FootprintChoice, KicadFiles, KicadLibrary};
use brep_ecad_core::kicad::{self, FootprintLink};
use serde::Deserialize;
use serde_json::{json, Value};
use std::collections::BTreeMap;

/// The folders a call may name instead of the configured ones.
struct Folders<'a> {
    symbols: &'a Option<String>,
    footprints: &'a Option<String>,
    models: &'a Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SearchArgs {
    /// Words that must all appear in the id (`Library:Name`, case-insensitive),
    /// e.g. `NE555`, `soic 8`. A `Library:` prefix keeps to that library
    /// (`Device:R`, `Package_SO:`). Empty lists everything, up to `limit`.
    #[serde(default)]
    pub query: String,
    /// `symbol` (default), `footprint` or `both`.
    #[serde(default)]
    pub kind: Option<String>,
    /// Only the footprints this symbol's `ki_fp_filters` offer (`Device:R`):
    /// how a generic symbol's footprint is chosen.
    #[serde(default)]
    pub for_symbol: Option<String>,
    /// Most matches returned per kind. Default 25.
    #[serde(default)]
    pub limit: Option<usize>,
    /// The folder of `*.kicad_sym` symbol libraries. Default: KiCad's
    /// `KICAD9_SYMBOL_DIR` (or 8/7/6), else `symbols` beside the footprints folder.
    #[serde(default)]
    pub symbols: Option<String>,
    /// The folder of `*.pretty` footprint libraries. Default: the folder last
    /// chosen in the KiCad import dialog, else `KICAD9_FOOTPRINT_DIR` (…), else
    /// KiCad's install folder.
    #[serde(default)]
    pub footprints: Option<String>,
    /// The folder of `*.3dshapes` models. Default as for `footprints`.
    #[serde(default)]
    pub models: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddArgs {
    /// The symbol, `Library:Name` as `kicad_library_search` lists it. Give this,
    /// `footprint`, or both.
    #[serde(default)]
    pub symbol: Option<String>,
    /// The footprint, `Library:Name`. With a symbol it overrides the one the
    /// symbol names (and is required when the symbol is generic); alone it
    /// adds a pads-only part.
    #[serde(default)]
    pub footprint: Option<String>,
    /// Add a generic symbol with no footprint at all (a sheet-only part).
    #[serde(default)]
    pub no_footprint: bool,
    /// The part document's name in the models folder. Default: the id with its
    /// library, `Device:R` → `Device_R`. An existing document of that name is
    /// replaced.
    #[serde(default)]
    pub name: Option<String>,
    /// The symbols folder, as for `kicad_library_search`.
    #[serde(default)]
    pub symbols: Option<String>,
    /// The footprints folder, as for `kicad_library_search`.
    #[serde(default)]
    pub footprints: Option<String>,
    /// The 3D-models folder, as for `kicad_library_search`.
    #[serde(default)]
    pub models: Option<String>,
}

/// The library as this call sees it: the configured folders with the call's
/// overrides on top, and the symbols folder.
struct Library {
    folders: KicadLibrary,
    symbols: Folder,
}

fn library(ctx: &mut Ctx<'_>, overrides: Folders<'_>) -> Library {
    let env = |name: &str| std::env::var(name).ok();
    let (_, _, store) = ctx.app.file_docs_store();
    let mut folders = KicadLibrary::load(store, &env, &DiskFiles);
    let given = |path: &Option<String>| {
        path.as_ref().filter(|p| !p.trim().is_empty()).map(|p| Folder { path: p.trim().to_owned(), from: "this call".into() })
    };
    if let Some(folder) = given(overrides.footprints) {
        folders.footprints = folder;
    }
    if let Some(folder) = given(overrides.models) {
        folders.models = folder;
    }
    let symbols = given(overrides.symbols).unwrap_or_else(|| Folder {
        path: crate::panels::kicad_library::symbols_folder(&env, &folders.footprints.path, &DiskFiles),
        from: "KICAD9_SYMBOL_DIR (or 8/7/6), else beside the footprints folder".into(),
    });
    Library { folders, symbols }
}

fn folder_json(folder: &Folder) -> Value {
    json!({ "path": folder.path, "from": folder.from, "exists": DiskFiles.is_dir(&folder.path) })
}

fn folders_json(library: &Library) -> Value {
    json!({
        "symbols": folder_json(&library.symbols),
        "footprints": folder_json(&library.folders.footprints),
        "models": folder_json(&library.folders.models),
    })
}

/// A missing folder, said with what to do about it.
fn missing(what: &str, folder: &Folder, argument: &str) -> String {
    format!(
        "no KiCad {what} folder at {} ({}); pass `{argument}` with the folder that holds KiCad's {what} libraries \
         (a KiCad install's share/kicad/{argument}, or a clone of gitlab.com/kicad/libraries/kicad-{argument})",
        folder.path, folder.from
    )
}

/// A query split into its library (from a `Library:` prefix) and its words.
struct Query {
    library: Option<String>,
    words: Vec<String>,
}

impl Query {
    fn parse(text: &str) -> Self {
        let text = text.trim();
        let (library, rest) = match text.split_once(':') {
            Some((library, rest)) if !library.trim().is_empty() && !library.contains(char::is_whitespace) => {
                (Some(library.trim().to_lowercase()), rest)
            }
            _ => (None, text),
        };
        Self { library, words: rest.split_whitespace().map(str::to_lowercase).collect() }
    }

    fn library_matches(&self, library: &str) -> bool {
        self.library.as_ref().map_or(true, |wanted| wanted.eq_ignore_ascii_case(library))
    }

    /// `None` when the entry does not match; else its rank, lower first: the
    /// exact name, then a name the first word starts, then the rest.
    fn rank(&self, library: &str, name: &str) -> Option<u8> {
        if !self.library_matches(library) {
            return None;
        }
        let id = format!("{library}:{name}").to_lowercase();
        if !self.words.iter().all(|word| id.contains(word.as_str())) {
            return None;
        }
        let name = name.to_lowercase();
        Some(match self.words.first() {
            Some(first) if self.words.len() == 1 && name == *first => 0,
            Some(first) if name.starts_with(first.as_str()) => 1,
            _ => 2,
        })
    }
}

/// The names of a `.kicad_sym` library's symbols, read off the text without
/// parsing it: each `(symbol "…"` one level inside `(kicad_symbol_lib`. A
/// unit's own `(symbol "R_0_1"` sits a level deeper and is not listed.
fn symbol_names(text: &str) -> Vec<String> {
    let bytes = text.as_bytes();
    let mut names = Vec::new();
    let mut depth = 0usize;
    let mut index = 0usize;
    while index < bytes.len() {
        match bytes[index] {
            b'"' => {
                index += 1;
                while index < bytes.len() && bytes[index] != b'"' {
                    index += if bytes[index] == b'\\' { 2 } else { 1 };
                }
            }
            b'(' => {
                if depth == 1 && text[index + 1..].starts_with("symbol") {
                    let after = &text[index + 7..];
                    if after.starts_with(char::is_whitespace) {
                        let after = after.trim_start();
                        if let Some(rest) = after.strip_prefix('"') {
                            names.push(quoted(rest));
                        }
                    }
                }
                depth += 1;
            }
            b')' => depth = depth.saturating_sub(1),
            _ => {}
        }
        index += 1;
    }
    names
}

/// The text of a quoted string whose opening quote is already consumed, up
/// to its closing quote, with `\"` and `\\` unescaped.
fn quoted(rest: &str) -> String {
    let mut out = String::new();
    let mut chars = rest.chars();
    while let Some(c) = chars.next() {
        match c {
            '"' => break,
            '\\' => out.extend(chars.next()),
            c => out.push(c),
        }
    }
    out
}

/// Read and parse one symbol library, `Library:Name` ids.
fn read_symbols(library: &Library, name: &str) -> Result<Vec<brep_ecad_core::Symbol>, String> {
    let file = format!("{name}.kicad_sym");
    let path = std::path::Path::new(&library.symbols.path).join(&file);
    let bytes = std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    kicad_import::read_symbol_library(&file, &bytes).map(|(symbols, _)| symbols)
}

fn split_id(id: &str) -> Result<(&str, &str), String> {
    match id.split_once(':') {
        Some((library, name)) if !library.is_empty() && !name.is_empty() => Ok((library, name)),
        _ => Err(format!("'{id}' is not a KiCad id: give `Library:Name`, as `kicad_library_search` lists it")),
    }
}

fn find_symbol(library: &Library, id: &str) -> Result<brep_ecad_core::Symbol, String> {
    let (lib, name) = split_id(id)?;
    if !DiskFiles.is_dir(&library.symbols.path) {
        return Err(missing("symbol", &library.symbols, "symbols"));
    }
    let symbols = read_symbols(library, lib)?;
    symbols
        .into_iter()
        .find(|symbol| symbol.library_id == id)
        .ok_or_else(|| format!("no symbol {id}: {lib}.kicad_sym has no symbol named {name}"))
}

fn link_json(symbol: &brep_ecad_core::Symbol) -> Value {
    match kicad::footprint_link(symbol) {
        FootprintLink::Named { library, name } if library.is_empty() => json!({ "footprint": name, "footprintFilters": [] }),
        FootprintLink::Named { library, name } => json!({ "footprint": format!("{library}:{name}"), "footprintFilters": [] }),
        FootprintLink::Chosen { filters } => json!({ "footprint": null, "footprintFilters": filters }),
    }
}

fn search_symbols(library: &Library, query: &Query, limit: usize) -> Result<(Vec<Value>, usize, usize), String> {
    if !DiskFiles.is_dir(&library.symbols.path) {
        return Err(missing("symbol", &library.symbols, "symbols"));
    }
    let libraries: Vec<String> = crate::panels::kicad_library::local_symbol_libraries(&library.symbols.path, &DiskFiles)
        .into_iter()
        .filter(|lib| query.library_matches(lib))
        .collect();
    let mut hits: Vec<(u8, String, String)> = Vec::new();
    for lib in &libraries {
        let path = std::path::Path::new(&library.symbols.path).join(format!("{lib}.kicad_sym"));
        let Ok(text) = std::fs::read_to_string(&path) else { continue };
        for name in symbol_names(&text) {
            if let Some(rank) = query.rank(lib, &name) {
                hits.push((rank, lib.clone(), name));
            }
        }
    }
    hits.sort();
    let total = hits.len();
    hits.truncate(limit);
    // Details for what is returned, parsing each library once.
    let mut parsed: BTreeMap<String, Option<Vec<brep_ecad_core::Symbol>>> = BTreeMap::new();
    let rows = hits
        .into_iter()
        .map(|(_, lib, name)| {
            let id = format!("{lib}:{name}");
            let symbols = parsed.entry(lib.clone()).or_insert_with(|| read_symbols(library, &lib).ok());
            let mut row = json!({ "id": id, "library": lib, "name": name });
            if let Some(symbol) = symbols.as_ref().and_then(|all| all.iter().find(|s| s.library_id == id)) {
                row["description"] = json!(symbol.description);
                row["reference"] = json!(symbol.reference_prefix);
                row["pins"] = json!(symbol.pins.len());
                if let (Some(row), Value::Object(link)) = (row.as_object_mut(), link_json(symbol)) {
                    row.extend(link);
                }
            }
            row
        })
        .collect();
    Ok((rows, total, libraries.len()))
}

fn search_footprints(
    library: &Library,
    query: &Query,
    filters: Option<&[String]>,
    limit: usize,
) -> Result<(Vec<Value>, usize, usize), String> {
    let root = &library.folders.footprints.path;
    if !DiskFiles.is_dir(root) {
        return Err(missing("footprint", &library.folders.footprints, "footprints"));
    }
    let mut libraries: Vec<String> = DiskFiles
        .list(root)
        .into_iter()
        .filter_map(|entry| entry.strip_suffix(".pretty").map(str::to_owned))
        .filter(|lib| query.library_matches(lib))
        .collect();
    libraries.sort();
    let mut hits: Vec<(u8, String, String)> = Vec::new();
    for lib in &libraries {
        for file in DiskFiles.list(&std::path::Path::new(root).join(format!("{lib}.pretty")).to_string_lossy()) {
            let Some(name) = file.strip_suffix(".kicad_mod") else { continue };
            if filters.is_some_and(|filters| !kicad::footprint_filter_matches(filters, lib, name)) {
                continue;
            }
            if let Some(rank) = query.rank(lib, name) {
                hits.push((rank, lib.clone(), name.to_owned()));
            }
        }
    }
    hits.sort();
    let total = hits.len();
    hits.truncate(limit);
    let rows = hits
        .into_iter()
        .map(|(_, lib, name)| json!({ "id": format!("{lib}:{name}"), "library": lib, "name": name }))
        .collect();
    Ok((rows, total, libraries.len()))
}

fn kicad_library_search(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SearchArgs = parse_args(args)?;
    let library = library(ctx, Folders { symbols: &a.symbols, footprints: &a.footprints, models: &a.models });
    let query = Query::parse(&a.query);
    let limit = a.limit.unwrap_or(25).max(1);
    let kind = a.kind.as_deref().unwrap_or(if a.for_symbol.is_some() { "footprint" } else { "symbol" });
    let (want_symbols, want_footprints) = match kind {
        "symbol" => (true, false),
        "footprint" => (false, true),
        "both" => (true, true),
        other => return Err(format!("kind '{other}': give `symbol`, `footprint` or `both`")),
    };
    let mut out = json!({ "folders": folders_json(&library) });
    if want_symbols {
        let (rows, total, scanned) = search_symbols(&library, &query, limit)?;
        out["symbols"] = json!(rows);
        out["symbolMatches"] = json!(total);
        out["symbolLibraries"] = json!(scanned);
    }
    if want_footprints {
        let filters = match &a.for_symbol {
            Some(id) => match kicad::footprint_link(&find_symbol(&library, id)?) {
                FootprintLink::Chosen { filters } => Some(filters),
                // A symbol that names its footprint offers only that one.
                FootprintLink::Named { library: lib, name } => Some(vec![format!("{lib}:{name}")]),
            },
            None => None,
        };
        let (rows, total, scanned) = search_footprints(&library, &query, filters.as_deref(), limit)?;
        out["footprints"] = json!(rows);
        out["footprintMatches"] = json!(total);
        out["footprintLibraries"] = json!(scanned);
    }
    Ok(Outcome::Done(out))
}

fn kicad_part_add(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddArgs = parse_args(args)?;
    let library = library(ctx, Folders { symbols: &a.symbols, footprints: &a.footprints, models: &a.models });
    let chosen = match &a.footprint {
        Some(id) => {
            let (lib, name) = split_id(id)?;
            if !DiskFiles.is_dir(&library.folders.footprints.path) {
                return Err(missing("footprint", &library.folders.footprints, "footprints"));
            }
            let file = library.folders.footprint_file(lib, name);
            if !DiskFiles.is_file(&file) {
                return Err(format!("no footprint {id}: looked for {file}"));
            }
            Some((lib.to_owned(), name.to_owned(), file))
        }
        None => None,
    };
    let part = match (&a.symbol, chosen) {
        (Some(id), chosen) => {
            let symbol = find_symbol(&library, id)?;
            let choice = match (chosen, kicad::footprint_link(&symbol)) {
                (Some((lib, name, _)), _) => FootprintChoice::Chosen(lib, name),
                (None, _) if a.no_footprint => FootprintChoice::Skip,
                (None, FootprintLink::Named { .. }) => FootprintChoice::Named,
                (None, FootprintLink::Chosen { filters }) => {
                    return Err(format!(
                        "{id} is a generic symbol that names no footprint; pick one with \
                         `kicad_library_search` {{ for_symbol: \"{id}\" }} (its filters: {}) and pass it as \
                         `footprint`, or pass `no_footprint: true` for a sheet-only part",
                        if filters.is_empty() { "none".to_owned() } else { filters.join(", ") }
                    ));
                }
            };
            kicad_import::part_from_symbol(symbol, &choice, &library.folders, &DiskFiles)
        }
        (None, Some((_, _, file))) => {
            let bytes = DiskFiles.read(&file).ok_or_else(|| format!("{file} could not be read"))?;
            kicad_import::part_from_footprint(&file, &bytes, &library.folders, &DiskFiles)?
        }
        (None, None) => return Err("give `symbol`, `footprint`, or both".into()),
    };
    let part = part.parsed();
    let id = a.symbol.as_deref().or(a.footprint.as_deref()).unwrap_or_default();
    let name = a
        .name
        .as_deref()
        .map(str::trim)
        .filter(|name| !name.is_empty())
        .map(str::to_owned)
        .unwrap_or_else(|| crate::panels::kicad_bulk::bulk_document_name(id));

    // The part document the dialog would have saved, in the models folder, so
    // `component_update` finds its source later.
    let mut fresh = brep_render::engine_state::EngineState::new();
    let imported = kicad_import::import_part(&mut fresh, &part)?;
    let document = fresh.history_request_json();
    let (_, docs, store) = ctx.app.file_docs_store();
    store.write(&name, &document)?;
    let component = docs.engine_mut().insert_component(brep_render::engine_state::ComponentInsert::New {
        name: &crate::store::model_display_name(&name),
        source_key: &name,
        source_signature: &brep_render::engine_state::document_signature(&document),
        document_json: &document,
    })?;
    Ok(Outcome::Done(json!({
        "id": component,
        "document": name,
        "summary": imported.summary,
        "symbol": part.symbol.as_ref().map(|s| s.library_id.clone()),
        "footprint": part.footprint.as_ref().map(|f| f.name.clone()),
        "pads": part.footprint.as_ref().map_or(0, |f| f.pads.len()),
        "model": part.model.as_ref().map(|m| m.step_file.clone()),
        "notes": imported.notes,
    })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "kicad_library_search", group: "ecad", doc: "Search the KiCad library on disk for symbols (`kind: symbol`, the default) and footprints (`footprint`, or `both`) by words in their `Library:Name` id; a `Library:` prefix keeps to one library, and `for_symbol` lists the footprints a generic symbol's filters offer. Symbol rows carry the description, reference, pin count and the footprint the symbol names (or its `footprintFilters` when it names none). `folders` says which folders were read and whether each exists: pass `symbols` / `footprints` / `models` to read others. Nothing is downloaded. Then add a match with `kicad_part_add`.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<SearchArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(kicad_library_search) },
    CommandSpec { name: "kicad_part_add", group: "ecad", doc: "Add a KiCad library part to the open assembly as a component, the way Import → KiCad then Add part to assembly does: the symbol (`Library:Name`), its footprint (the one it names, or `footprint` for a generic symbol or to override), and the footprint's STEP model become a part document saved in the models folder (`name`, default `Library_Name`), which is inserted through the assembly's insert flow (an ACOMP feature; the first is grounded). A footprint alone adds a pads-only part. On the PCB workbench a part with pads is seated on the board and its sheet the next frame; on the Diagram workbench it is offered as a card in the Assembly parts pane (`ecadparts/parts:card:<reference>`, `hit_rects`) and placed by dragging the card onto `ecad/canvas` (`drag_widget`). Returns the ACOMP id, the document, pad count, model file and the import's notes (each link it could not follow).", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AddArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(kicad_part_add) },
];

