//! Documents: new / load / save-as-text / import / export / tabs. Files never
//! cross this boundary — the host reads and writes them and passes content.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::document::Document;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LoadArgs {
    /// A `.nbrep` document as text.
    pub json: String,
    /// The document's display name (the tab title). Optional.
    #[serde(default)]
    pub name: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ImportFormat {
    Step,
    Stl,
    Obj,
    Iges,
    /// A 3MF package. Named for the extension, not the Rust variant, so the
    /// wire value is the one a caller types.
    #[serde(rename = "3mf")]
    ThreeMf,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ImportArgs {
    pub format: ImportFormat,
    /// The file's name, for the feature label.
    pub name: String,
    /// File content, base64.
    pub base64: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ExportFormat {
    /// The native document (`history_request_json`).
    Brep,
    Step,
    Stl,
    Obj,
    Iges,
    /// Binary glTF 2.0 of the display mesh. A BINARY format, so it comes back
    /// in `base64` rather than `text`.
    Glb,
    /// Sheet-metal flat pattern.
    Dxf,
    Svg,
    /// The OPEN drawing sheet as SVG (the first sheet when none is open) —
    /// a different drawing from `svg`, which is the sheet-METAL flat pattern.
    SheetSvg,
    /// EVERY drawing sheet of the document as one PDF, one page per sheet in
    /// sheet order, each at its own paper size. Binary, so it comes back in
    /// `base64`, with the page count in `pages`.
    SheetPdf,
    /// The same drawing set with the live 3D model (Sheets + 3D): PDF 1.7, a
    /// 3D box over every placement marked `threeD` and a last page given over
    /// to the model, what it carries read back in `model3d`.
    #[serde(rename = "sheet_pdf_3d")]
    SheetPdf3d,
    /// The PCB's fabrication bundle: Gerber X2 layers, Excellon drills, the
    /// pick-and-place and parts-BOM CSVs and a README, in one zip. Binary, so
    /// `base64`, with each file in `files` and what the files hold, counted
    /// back from them, in `readBack`.
    Fabrication,
    /// The pick-and-place (centroid) CSV alone, JLCPCB CPL columns.
    PickPlaceCsv,
    /// The electronics BOM CSV alone: references grouped by value and
    /// footprint, JLCPCB BOM columns.
    EcadBomCsv,
    /// The PCB schematic's netlist as KiCad writes one (`.net`, export
    /// version "E"): every part and every net with its pins, under the
    /// Connectivity panel's net names.
    KicadNetlist,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExportArgs {
    pub format: ExportFormat,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IndexArgs {
    pub index: usize,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CloseArgs {
    pub index: usize,
    /// Close even if the document has unsaved changes.
    #[serde(default)]
    pub discard: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PartAttributeArgs {
    /// The attribute name, exactly as the BOM column configuration writes it
    /// after `part.` (`Part_Number`, `Material`, a custom `Alloy_Temper`).
    pub field: String,
    /// The value to store. A string, a number, or `null` / `""` to REMOVE the
    /// attribute — the last one removed takes the empty record with it, so a
    /// document that was never annotated saves exactly as before.
    pub value: Value,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct Tab {
    pub index: usize,
    pub title: String,
    pub name: Option<String>,
    pub dirty: bool,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct Tabs {
    pub active: usize,
    pub tabs: Vec<Tab>,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct Exported {
    pub format: String,
    /// The document as text — empty for a BINARY format, which arrives in
    /// `base64` instead.
    pub text: String,
    /// The file's length in bytes, whichever field carries it.
    pub bytes: usize,
    /// The file's bytes, base64, for a BINARY format (`glb`). Absent otherwise,
    /// which is how a caller tells the two apart without knowing the format.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base64: Option<String>,
    /// How many pages the file has, for a paged format (`sheet_pdf`: one per
    /// sheet). Absent otherwise.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pages: Option<usize>,
    /// Every text string a drawing-sheet file shows, in file order, READ BACK
    /// from the bytes written (`sheet_svg`: each `<text>`; `sheet_pdf`: each
    /// shown string, WinAnsi decoded — so a `⌀` reads `Ø` and a U+2212 minus a
    /// hyphen, which is what the PDF carries). Absent for every other format.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub texts: Option<Vec<String>>,
    /// `sheet_pdf` carrying a live 3D model: what it carries, READ BACK from
    /// the bytes written. Absent for a file with no 3D.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model3d: Option<Model3dSummary>,
    /// `fabrication`: every file in the zip, in archive order, READ BACK from
    /// the archive written. Absent for every other format.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub files: Option<Vec<ZipEntry>>,
    /// `fabrication`: what the board's files hold, counted from their text.
    #[serde(rename = "readBack", skip_serializing_if = "Option::is_none")]
    pub read_back: Option<FabricationReadBack>,
    /// `fabrication`: problems to fix before ordering boards (design-rule
    /// findings, silkscreen characters the font lacks). The files are complete
    /// either way. Absent for every other format.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warnings: Option<Vec<String>>,
    /// The CSV formats: data rows, not counting the header.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rows: Option<usize>,
}

/// One file of a fabrication zip, from the archive's central directory.
#[derive(Serialize, schemars::JsonSchema)]
pub struct ZipEntry {
    pub name: String,
    pub bytes: usize,
}

/// A fabrication bundle's contents, counted from its files.
#[derive(Serialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase")]
pub struct FabricationReadBack {
    /// Pad and via flashes on the top and bottom copper.
    pub top_flashes: usize,
    pub bottom_flashes: usize,
    /// Drill hits, plated and non-plated.
    pub plated_holes: usize,
    pub unplated_holes: usize,
    /// The Edge_Cuts drawing's width and height in millimetres.
    pub outline_mm: [f64; 2],
    /// Pick-and-place rows and parts-BOM lines.
    pub placements: usize,
    pub bom_lines: usize,
}

/// A sheet PDF's 3D content as read back from the file.
#[derive(Serialize, schemars::JsonSchema)]
pub struct Model3dSummary {
    /// 3D boxes: one per live placement, plus the model page's.
    pub annotations: usize,
    /// Saved views in the model's view list.
    pub views: usize,
    /// The model page's view buttons.
    pub links: usize,
    /// The embedded U3D stream's size, decoded.
    #[serde(rename = "u3dBytes")]
    pub u3d_bytes: usize,
}

fn tabs(ctx: &Ctx<'_>) -> Tabs {
    let docs = &ctx.app.docs;
    Tabs {
        active: docs.active_index(),
        tabs: docs
            .iter()
            .enumerate()
            .map(|(i, d)| Tab { index: i, title: d.title(), name: d.name().map(str::to_string), dirty: d.is_dirty() })
            .collect(),
    }
}

fn doc_new(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let mut engine = ctx.app.docs.spawn_engine();
    let _ = engine.set_history_json(crate::document::EMPTY_DOCUMENT);
    let index = ctx.app.docs.open_document(Document::new(engine));
    Ok(Outcome::Done(json!({ "index": index })))
}

fn doc_load(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: LoadArgs = parse_args(args)?;
    let mut engine = ctx.app.docs.spawn_engine();
    let report = engine.load_model_and_fit(&a.json)?;
    // The name's extension names the class, as File > Open does.
    if let Some(class) = a.name.as_deref().and_then(crate::document_class::DocumentClass::of_name) {
        crate::document_class::set_document_class(&mut engine, class);
    }
    let mut doc = Document::new(engine);
    doc.set_name(a.name.clone());
    doc.mark_clean();
    let index = ctx.app.docs.open_document(doc);
    Ok(Outcome::Done(json!({ "index": index, "name": a.name, "report": parse(&report) })))
}

fn doc_json(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let text = ctx.app.docs.engine().history_request_json();
    Ok(Outcome::Done(json!({ "document": parse(&text), "bytes": text.len() })))
}

fn part_attributes_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let attributes = ctx.app.docs.engine().document_part_attributes();
    Ok(Outcome::Done(json!({ "attributes": attributes })))
}

fn part_attribute_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PartAttributeArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    engine.set_document_part_attribute(&a.field, a.value)?;
    Ok(Outcome::Done(json!({ "attributes": engine.document_part_attributes() })))
}

fn doc_import(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ImportArgs = parse_args(args)?;
    let bytes = base64_decode(&a.base64)?;
    let engine = ctx.app.docs.engine_mut();
    let report = match a.format {
        ImportFormat::Step => engine.import_step_feature(&String::from_utf8_lossy(&bytes))?,
        ImportFormat::Iges => engine.import_iges_feature(&String::from_utf8_lossy(&bytes))?,
        ImportFormat::Stl => engine.import_stl_feature(&bytes)?,
        ImportFormat::Obj => engine.import_obj_bytes_feature(&bytes)?,
        ImportFormat::ThreeMf => engine.import_3mf_bytes_feature(&bytes)?,
    };
    Ok(Outcome::Done(json!({ "name": a.name, "report": parse(&report) })))
}

fn doc_export(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ExportArgs = parse_args(args)?;
    // Files and the root PRODUCT take the document's DISPLAY name, as the
    // UI's Export does (its name field starts from `model_display_name`). A
    // document saved through the explorer is named by its full path, and the
    // raw name made the fabrication zip's members
    // `_tmp_…_kicad_symbols_amp-board-F_Cu.gtl` (ecad-reaudit-2026-09-23, B8).
    let document_name = ctx
        .app
        .docs
        .active()
        .name()
        .map(crate::store::model_display_name)
        .unwrap_or_else(|| "Part".to_string());
    // The BINARY formats take their own exit: the bytes never become a lossy
    // `String`, and the caller reads them from `base64`. A PDF is ASCII as this
    // writer emits it, but it is a byte format and goes out as one — the field
    // the caller reads must not depend on what happens to be in the file.
    let binary: Option<(&str, Vec<u8>, Option<usize>)> = match a.format {
        ExportFormat::Glb => {
            Some(("glb", ctx.app.docs.engine().export_glb_bytes(&document_name)?, None))
        }
        ExportFormat::SheetPdf => {
            let engine = ctx.app.docs.engine_mut();
            let pages = engine.sheet_state().sheets.len();
            Some(("sheet_pdf", engine.export_document_pdf()?, Some(pages)))
        }
        ExportFormat::SheetPdf3d => {
            let engine = ctx.app.docs.engine_mut();
            let pages = engine.sheet_state().sheets.len() + 1;
            Some(("sheet_pdf_3d", engine.export_document_pdf_3d()?, Some(pages)))
        }
        ExportFormat::Fabrication => {
            let bundle = crate::panels::fabrication_export::bundle(ctx.app.docs.engine(), &document_name)?;
            let raw = bundle.zip();
            let r = bundle.read_back();
            let exported = Exported {
                format: "fabrication".into(),
                text: String::new(),
                bytes: raw.len(),
                base64: Some(base64_encode(&raw)),
                pages: None,
                texts: None,
                model3d: None,
                files: Some(
                    brep_ecad_core::fabrication::zip_entries(&raw)
                        .into_iter()
                        .map(|(name, bytes)| ZipEntry { name, bytes })
                        .collect(),
                ),
                read_back: Some(FabricationReadBack {
                    top_flashes: r.top_flashes,
                    bottom_flashes: r.bottom_flashes,
                    plated_holes: r.plated_holes,
                    unplated_holes: r.unplated_holes,
                    outline_mm: [r.outline_mm.0, r.outline_mm.1],
                    placements: r.placements,
                    bom_lines: r.bom_lines,
                }),
                warnings: Some(bundle.warnings),
                rows: None,
            };
            return serde_json::to_value(exported).map(Outcome::Done).map_err(|e| e.to_string());
        }
        _ => None,
    };
    if let Some((format, raw, pages)) = binary {
        let pdf = format.starts_with("sheet_pdf");
        let texts = pdf.then(|| brep_render::sheets::pdf::text_strings(&raw));
        let model3d = pdf
            .then(|| brep_render::sheets::pdf3d::summary(&raw))
            .flatten()
            .map(|s| Model3dSummary { annotations: s.annotations, views: s.views, links: s.links, u3d_bytes: s.u3d_bytes });
        let exported = Exported {
            format: format.into(),
            text: String::new(),
            bytes: raw.len(),
            base64: Some(base64_encode(&raw)),
            pages,
            texts,
            model3d,
            files: None,
            read_back: None,
            warnings: None,
            rows: None,
        };
        return serde_json::to_value(exported).map(Outcome::Done).map_err(|e| e.to_string());
    }
    let engine = ctx.app.docs.engine_mut();
    let (format, text) = match a.format {
        ExportFormat::Brep => ("brep", engine.history_request_json()),
        ExportFormat::Step => ("step", engine.export_step_text_named(&document_name)?),
        ExportFormat::Stl => ("stl", engine.export_stl_text()?),
        ExportFormat::Obj => ("obj", engine.export_obj_text()?),
        ExportFormat::Iges => ("iges", engine.export_iges_text()?),
        ExportFormat::Dxf => ("dxf", engine.export_flat_pattern_dxf()?),
        ExportFormat::Svg => ("svg", engine.export_flat_pattern_svg()?),
        ExportFormat::SheetSvg => {
            let sheet = engine.resolve_sheet("")?;
            ("sheet_svg", engine.export_sheet_svg(&sheet)?)
        }
        ExportFormat::PickPlaceCsv => {
            ("pick_place_csv", crate::panels::fabrication_export::csv(engine, crate::panels::fabrication_export::Output::PickPlace)?)
        }
        ExportFormat::EcadBomCsv => {
            ("ecad_bom_csv", crate::panels::fabrication_export::csv(engine, crate::panels::fabrication_export::Output::Bom)?)
        }
        ExportFormat::KicadNetlist => {
            ("kicad_netlist", crate::panels::fabrication_export::text(engine, crate::panels::fabrication_export::Output::Netlist, &document_name)?)
        }
        ExportFormat::Glb | ExportFormat::SheetPdf | ExportFormat::SheetPdf3d | ExportFormat::Fabrication => {
            unreachable!("the binary lane returned above")
        }
    };
    let bytes = text.len();
    let texts = (format == "sheet_svg").then(|| brep_render::sheets::svg::text_strings(&text));
    let rows = format.ends_with("_csv").then(|| text.lines().count().saturating_sub(1));
    serde_json::to_value(Exported { format: format.into(), text, bytes, base64: None, pages: None, texts, model3d: None, files: None, read_back: None, warnings: None, rows }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn docs_list(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    serde_json::to_value(tabs(ctx)).map(Outcome::Done).map_err(|e| e.to_string())
}

fn doc_activate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: IndexArgs = parse_args(args)?;
    if a.index >= ctx.app.docs.len() {
        return Err(format!("no document {} ({} open)", a.index, ctx.app.docs.len()));
    }
    ctx.app.docs.activate(a.index);
    serde_json::to_value(tabs(ctx)).map(Outcome::Done).map_err(|e| e.to_string())
}

fn doc_close(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: CloseArgs = parse_args(args)?;
    let Some(doc) = ctx.app.docs.get(a.index) else {
        return Err(format!("no document {} ({} open)", a.index, ctx.app.docs.len()));
    };
    if doc.is_dirty() && !a.discard {
        return Err(format!("document {} has unsaved changes; pass discard: true to close it anyway", a.index));
    }
    ctx.app.docs.close(a.index);
    serde_json::to_value(tabs(ctx)).map(Outcome::Done).map_err(|e| e.to_string())
}

fn doc_mark_clean(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.docs.active_mut().mark_clean();
    Ok(Outcome::Done(json!({})))
}

pub(crate) fn parse(text: &str) -> Value {
    serde_json::from_str(text).unwrap_or_else(|_| Value::String(text.to_string()))
}

/// Standard base64, padded — the inverse of [`base64_decode`], for handing a
/// BINARY export back through a JSON result. No dependency.
pub(crate) fn base64_encode(bytes: &[u8]) -> String {
    const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity((bytes.len() + 2) / 3 * 4);
    for chunk in bytes.chunks(3) {
        let b = [chunk[0], *chunk.get(1).unwrap_or(&0), *chunk.get(2).unwrap_or(&0)];
        let n = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        for slot in 0..4 {
            if slot <= chunk.len() {
                out.push(ALPHABET[((n >> (18 - slot * 6)) & 0x3f) as usize] as char);
            } else {
                out.push('=');
            }
        }
    }
    out
}

/// Standard base64 (with or without padding); no dependency.
pub(crate) fn base64_decode(s: &str) -> Result<Vec<u8>, String> {
    fn val(c: u8) -> Result<u32, String> {
        Ok(match c {
            b'A'..=b'Z' => (c - b'A') as u32,
            b'a'..=b'z' => (c - b'a') as u32 + 26,
            b'0'..=b'9' => (c - b'0') as u32 + 52,
            b'+' | b'-' => 62,
            b'/' | b'_' => 63,
            _ => return Err(format!("invalid base64 byte {c:#x}")),
        })
    }
    let mut out = Vec::with_capacity(s.len() * 3 / 4);
    let mut acc: u32 = 0;
    let mut bits = 0;
    for &c in s.as_bytes() {
        if c == b'=' || c == b'\n' || c == b'\r' || c == b' ' {
            continue;
        }
        acc = (acc << 6) | val(c)?;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push(((acc >> bits) & 0xff) as u8);
        }
    }
    Ok(out)
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "doc_new", group: "document", doc: "Open a new, empty document as a tab and make it active.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(doc_new) },
    CommandSpec { name: "doc_load", group: "document", doc: "Open a `.nbrep` document (passed as text) in a new tab, run its history and fit the camera. Returns the run report.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<LoadArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(doc_load) },
    CommandSpec { name: "doc_json", group: "document", doc: "The active document as `.nbrep` (the history request with metadata).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(doc_json) },
    CommandSpec { name: "doc_import", group: "document", doc: "Append an Import 3D Model feature from STEP / IGES text or STL / OBJ / 3MF bytes (base64). STL, OBJ and 3MF go through mesh reconstruction on the runner; a 3MF is read to the 3MF core specification and its build instances are flattened first.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ImportArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(doc_import) },
    CommandSpec { name: "doc_export", group: "document", doc: "Export the active document: brep (native JSON), step, stl, obj, iges, the sheet-metal flat pattern as dxf / svg, or the open drawing sheet as sheet_svg, all as `text`; glb (binary glTF 2.0 of the display mesh) and EVERY drawing sheet as sheet_pdf (one PDF, a page per sheet in sheet order, with the count in `pages`) and the same set with the live 3D model as sheet_pdf_3d (PDF 1.7: a 3D box over every placement marked `threeD` and a last page given over to the model, read back in `model3d`) as `base64`. The two sheet formats also answer `texts`: every text string the file shows, read back from the bytes written. For a document whose PCB board has parts: fabrication (one zip of Gerber X2 layers, Excellon drills, the pick-and-place and parts-BOM CSVs and a README) as `base64`, with its `files` read back from the archive's directory, `readBack` (top/bottom copper flashes, plated/unplated drill hits, the outline's size in mm and the CSVs' row counts, counted from the files) and `warnings`; and the CSVs alone as pick_place_csv (JLCPCB CPL columns) and ecad_bom_csv (JLCPCB BOM columns), as `text` with their data `rows`. For a document whose PCB schematic has parts: kicad_netlist, its KiCad netlist (`.net`, export version \"E\": the parts and every net with its pins, under the Connectivity panel's net names) as `text`.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<ExportArgs>, result_schema: schema_of::<Exported>, handler: Handler::App(doc_export) },
    CommandSpec { name: "part_attributes_get", group: "document", doc: "The active document's own BOM part attributes (`partAttributes`) — the record the toolbar's Part Properties dialog edits, and the one an assembly's BOM reads off this part.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(part_attributes_get) },
    CommandSpec { name: "part_attribute_set", group: "document", doc: "Write ONE of the active document's own BOM part attributes (Part_Number, Material, Mass, or any custom field); a null or empty value removes it. One undo step per field.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<PartAttributeArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(part_attribute_set) },
    CommandSpec { name: "docs_list", group: "document", doc: "The open document tabs and which is active.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Tabs>, handler: Handler::App(docs_list) },
    CommandSpec { name: "doc_activate", group: "document", doc: "Make the tab at `index` the active document.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<IndexArgs>, result_schema: schema_of::<Tabs>, handler: Handler::App(doc_activate) },
    CommandSpec { name: "doc_close", group: "document", doc: "Close the tab at `index`; refuses a dirty document unless `discard` is true.", phase: Phase::Mutate, annotations: Annotations::DESTRUCTIVE, args_schema: schema_of::<CloseArgs>, result_schema: schema_of::<Tabs>, handler: Handler::App(doc_close) },
    CommandSpec { name: "doc_mark_clean", group: "document", doc: "Mark the active document as saved (the host wrote `doc_json` to disk).", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(doc_mark_clean) },
];

