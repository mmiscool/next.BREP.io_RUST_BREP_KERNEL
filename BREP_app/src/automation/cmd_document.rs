//! Documents: new / load / save-as-text / import / export / tabs. Files never
//! cross this boundary — the host reads and writes them and passes content.
use crate::automation::command::{app_command, parse_args, schema_of, CommandSpec, Ctx, Empty, NoArgs, Outcome};
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
    /// A 3MF package of the display mesh, one object per body with the
    /// document's colours as base materials. Binary (a ZIP), so `base64`.
    /// Named for the extension, as the import format is.
    #[serde(rename = "3mf")]
    ThreeMf,
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
    /// The assembly's BOM as CSV — the File ▸ Export BOM file: one row per
    /// part with its quantity, material, density (g/cm³), volume (mm³), mass
    /// and total mass (kg), all read off the PART, then the harness wires.
    BomCsv,
    /// The same BOM as a JSON array.
    BomJson,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExportArgs {
    pub format: ExportFormat,
    /// STEP or IGES: export just these resident solids of a part document
    /// (default: every solid). A STEP subset leaves its PMI out.
    #[serde(default)]
    pub solids: Option<Vec<String>>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IndexArgs {
    pub index: usize,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NewArgs {
    /// A display label for the new, untitled tab (never a file name).
    #[serde(default)]
    pub title: Option<String>,
    /// Make the new tab the visible one (default). False opens it behind the
    /// visible document, which keeps its panels and camera.
    #[serde(default = "yes")]
    pub activate: bool,
}

fn yes() -> bool {
    true
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CloseArgs {
    /// The tab's position. One of `index` and `id` is required.
    #[serde(default)]
    pub index: Option<usize>,
    /// The tab's public handle (`docs_list`'s `id`): closes that document
    /// wherever its tab now is.
    #[serde(default)]
    pub id: Option<String>,
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
    /// The document's public handle: a command's `context.documentId`.
    pub id: String,
    /// The committed revision (decimal string): a writing command's
    /// `context.expectedRevision`.
    pub revision: String,
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
    /// The file's bytes, base64, for a BINARY format (`glb`, `3mf`). Absent otherwise,
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
    /// `bom_csv` / `bom_json`: every row READ BACK from the text written —
    /// the JSON array as is, or each CSV line as `{header: field}` (strings,
    /// as the file has them). Absent for every other format.
    #[serde(rename = "bomRows", skip_serializing_if = "Option::is_none")]
    pub bom_rows: Option<Vec<Value>>,
}

/// A BOM export's rows read back from its text (see `Exported::bom_rows`).
fn bom_read_back(format: &str, text: &str) -> Option<Vec<Value>> {
    match format {
        "bom_json" => return serde_json::from_str(text).ok(),
        "bom_csv" => {}
        _ => return None,
    }
    // Minimal RFC-4180: quoted fields with doubled quotes, as the writer quotes.
    fn fields(line: &str) -> Vec<String> {
        let mut out = Vec::new();
        let mut field = String::new();
        let mut quoted = false;
        let mut chars = line.chars().peekable();
        while let Some(c) = chars.next() {
            match c {
                '"' if quoted && chars.peek() == Some(&'"') => {
                    field.push('"');
                    chars.next();
                }
                '"' => quoted = !quoted,
                ',' if !quoted => out.push(std::mem::take(&mut field)),
                other => field.push(other),
            }
        }
        out.push(field);
        out
    }
    let mut lines = text.lines();
    let header = fields(lines.next()?);
    Some(
        lines
            .map(|line| {
                Value::Object(header.iter().cloned().zip(fields(line).into_iter().map(Value::String)).collect())
            })
            .collect(),
    )
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
            .map(|(i, d)| Tab {
                index: i,
                title: d.title(),
                name: d.name().map(str::to_string),
                dirty: d.is_dirty(),
                id: d.handle().to_string(),
                revision: d.committed_revision().to_string(),
            })
            .collect(),
    }
}

fn doc_new(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NewArgs = parse_args(args)?;
    let mut doc = Document::fresh(ctx.app.docs.spawn_engine(), crate::document_class::DocumentClass::Normal);
    doc.set_label(a.title.filter(|t| !t.trim().is_empty()));
    let index = if a.activate { ctx.app.docs.open_document(doc) } else { ctx.app.docs.open_document_behind(doc) };
    Ok(Outcome::Done(json!({ "index": index, "context": opened_context(ctx, index) })))
}

// Successful doc_load JSON, including the nullable name and the context
// returned by opened_context. Keep the engine's report as its native JSON.
fn doc_load_result_schema() -> Value {
    json!({"type":"object","properties":{
        "index":{"type":"integer","minimum":0},
        "name":{"type":["string","null"]},
        "report":{"description":"The engine's native model-load report."},
        "context":{"anyOf":[{"type":"null"},{"type":"object","properties":{
            "documentId":{"type":"string"},"expectedRevision":{"type":"string"}
        },"required":["documentId","expectedRevision"],"additionalProperties":false}]}
    },"required":["index","name","report","context"],"additionalProperties":false})
}

// ImportArgs::name is String, unlike LoadArgs::name. The import report is
// native JSON and can vary with the imported format and reconstruction path.
fn doc_import_result_schema() -> Value {
    json!({"type":"object","properties":{
        "name":{"type":"string"},"report":{"description":"The engine's native import report."}
    },"required":["name","report"],"additionalProperties":false})
}

fn doc_load(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: LoadArgs = parse_args(args)?;
    let mut engine = ctx.app.docs.spawn_engine();
    let report = engine.load_model_and_fit(&a.json)?;
    // The name's extension names the class, as File > Open does.
    if let Some(class) = a.name.as_deref().and_then(crate::document_class::DocumentClass::of_name) {
        crate::document_class::set_document_class(&mut engine, class);
    }
    // `Document::new` takes the clean baseline, as File > Open's does.
    let mut doc = Document::new(engine);
    doc.set_name(a.name.clone());
    let index = ctx.app.docs.open_document(doc);
    Ok(Outcome::Done(json!({ "index": index, "name": a.name, "report": parse(&report), "context": opened_context(ctx, index) })))
}

/// The context a just-opened document's first write names.
fn opened_context(ctx: &Ctx<'_>, index: usize) -> Value {
    ctx.app.docs.get(index).map_or(Value::Null, |d| {
        json!({ "documentId": d.handle(), "expectedRevision": d.committed_revision().to_string() })
    })
}

// Document readback carries authored JSON rather than a closed kernel DTO.
// The stable envelope is typed, while plugin fields and custom attributes stay
// intact. opened_context can return null when the tab cannot be resolved.
fn doc_new_result_schema() -> Value {
    json!({"type":"object","properties":{
        "index":{"type":"integer","minimum":0},
        "context":{"anyOf":[{"type":"null"},{"type":"object","properties":{
            "documentId":{"type":"string"},"expectedRevision":{"type":"string"}
        },"required":["documentId","expectedRevision"],"additionalProperties":false}]}
    },"required":["index","context"],"additionalProperties":false})
}
fn doc_json_result_schema() -> Value {
    json!({"type":"object","properties":{
        "document":{"description":"Complete authored document JSON, including plugin and user-defined fields."},
        "bytes":{"type":"integer","minimum":0,"description":"UTF-8 byte length of the serialized native document."}
    },"required":["document","bytes"],"additionalProperties":false})
}
fn document_attributes_result_schema() -> Value {
    json!({"type":"object","properties":{
        "attributes":{"type":"object","additionalProperties":true,"description":"Document BOM attributes; values may be arbitrary authored JSON."}
    },"required":["attributes"],"additionalProperties":false})
}

fn doc_json(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    // The TAB's document (the assembly while a Work part is edited in place;
    // pushes are live and `wait_idle` waits for them, so it is current).
    let text = ctx.app.docs.tab_engine().history_request_json();
    Ok(Outcome::Done(json!({ "document": parse(&text), "bytes": text.len() })))
}

/// The open document's own attributes with its mass reading, or `None`
/// while a measurement is still in flight.
fn document_mass_reply(engine: &mut brep_render::engine_state::EngineState) -> Option<Value> {
    let report = engine.document_mass();
    if report.status == brep_render::engine_state::part_mass::MassStatus::Pending {
        return None;
    }
    Some(json!({
        "attributes": engine.document_part_attributes(),
        "record": engine.document_bom_record(),
        "massProperties": report.to_json(),
    }))
}

fn part_attributes_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    if let Some(reply) = document_mass_reply(ctx.app.docs.engine_mut()) {
        return Ok(Outcome::Done(reply));
    }
    let handle = ctx.app.docs.active().handle().to_string();
    Ok(Outcome::Poll(Box::new(move |ctx: &mut Ctx<'_>| {
        let Some(engine) = ctx.app.docs.engine_of_handle_mut(&handle) else {
            return Some(Err("document_unknown: the document closed while it was being measured".into()));
        };
        document_mass_reply(engine).map(|reply| Ok(Outcome::Done(reply)))
    })))
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
    if a.solids.is_some() && !matches!(a.format, ExportFormat::Step | ExportFormat::Iges) {
        return Err("`solids` selects what a STEP or IGES export writes; other formats export the whole document".into());
    }
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
        ExportFormat::ThreeMf => {
            Some(("3mf", ctx.app.docs.engine().export_3mf_bytes(&document_name)?, None))
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
                bom_rows: None,
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
            bom_rows: None,
        };
        return serde_json::to_value(exported).map(Outcome::Done).map_err(|e| e.to_string());
    }
    let engine = ctx.app.docs.engine_mut();
    // A BOM's masses are measured off the parts' geometry: wait for them.
    if matches!(a.format, ExportFormat::BomCsv | ExportFormat::BomJson) && engine.bom_mass_pending() {
        let handle = ctx.app.docs.active().handle().to_string();
        return Ok(Outcome::Poll(Box::new(move |ctx: &mut Ctx<'_>| {
            let Some(engine) = ctx.app.docs.engine_of_handle_mut(&handle) else {
                return Some(Err("document_unknown: the document closed while its BOM was being measured".into()));
            };
            if engine.bom_mass_pending() {
                return None;
            }
            let format = if matches!(a.format, ExportFormat::BomCsv) { "bom_csv" } else { "bom_json" };
            let text = match a.format {
                ExportFormat::BomCsv => engine.export_bom_csv(),
                _ => engine.export_bom_json(),
            };
            Some(text.map(|text| bom_exported(format, text)))
        })));
    }
    let (format, text) = match a.format {
        ExportFormat::Brep => ("brep", engine.history_request_json()),
        ExportFormat::Step => ("step", engine.export_step_text_selected(&document_name, a.solids.as_deref())?),
        ExportFormat::Stl => ("stl", engine.export_stl_text()?),
        ExportFormat::Obj => ("obj", engine.export_obj_text()?),
        ExportFormat::Iges => ("iges", engine.export_iges_text_selected(&document_name, a.solids.as_deref())?),
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
        ExportFormat::BomCsv => ("bom_csv", engine.export_bom_csv()?),
        ExportFormat::BomJson => ("bom_json", engine.export_bom_json()?),
        ExportFormat::KicadNetlist => {
            ("kicad_netlist", crate::panels::fabrication_export::text(engine, crate::panels::fabrication_export::Output::Netlist, &document_name)?)
        }
        ExportFormat::Glb | ExportFormat::ThreeMf | ExportFormat::SheetPdf | ExportFormat::SheetPdf3d | ExportFormat::Fabrication => {
            unreachable!("the binary lane returned above")
        }
    };
    let bytes = text.len();
    let texts = (format == "sheet_svg").then(|| brep_render::sheets::svg::text_strings(&text));
    let rows = format.ends_with("_csv").then(|| text.lines().count().saturating_sub(1));
    let bom_rows = bom_read_back(format, &text);
    serde_json::to_value(Exported { format: format.into(), text, bytes, base64: None, pages: None, texts, model3d: None, files: None, read_back: None, warnings: None, rows, bom_rows }).map(Outcome::Done).map_err(|e| e.to_string())
}

/// A BOM export answered after its measurements landed (see `doc_export`).
fn bom_exported(format: &str, text: String) -> Outcome {
    let rows = format.ends_with("_csv").then(|| text.lines().count().saturating_sub(1));
    let bom_rows = bom_read_back(format, &text);
    let exported = Exported { format: format.into(), bytes: text.len(), text, base64: None, pages: None, texts: None, model3d: None, files: None, read_back: None, warnings: None, bom_rows, rows };
    Outcome::Done(serde_json::to_value(exported).unwrap_or(Value::Null))
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
    let index = match (a.index, a.id.as_deref()) {
        (_, Some(id)) => ctx.app.docs.index_of_handle(id).ok_or_else(|| "document_unknown: no open document has that handle".to_string())?,
        (Some(index), None) => index,
        (None, None) => return Err("pass `index` or `id`".into()),
    };
    let Some(doc) = ctx.app.docs.get(index) else {
        return Err(format!("no document {index} ({} open)", ctx.app.docs.len()));
    };
    if doc.is_dirty() && !a.discard {
        return Err(format!("document {index} has unsaved changes; pass discard: true to close it anyway"));
    }
    ctx.app.docs.close(index);
    serde_json::to_value(tabs(ctx)).map(Outcome::Done).map_err(|e| e.to_string())
}

/// Close the app as its window's close button does: unsaved work is written
/// for recovery, and the close waits for the store's writes to land
/// (`__brepClosing`; the headless host itself stays up).
fn app_close(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.close_asked = true;
    Ok(Outcome::Done(json!({})))
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
/// BINARY export back through a JSON result.
pub(crate) fn base64_encode(bytes: &[u8]) -> String {
    use base64::Engine as _;
    base64::engine::general_purpose::STANDARD.encode(bytes)
}

/// Base64 as a caller may hand it in: the standard or the URL-safe alphabet,
/// with or without padding, wrapped across lines or spaced. Anything else is
/// refused naming what is wrong.
pub(crate) fn base64_decode(s: &str) -> Result<Vec<u8>, String> {
    use base64::engine::{DecodePaddingMode, GeneralPurpose, GeneralPurposeConfig};
    use base64::Engine as _;
    let config = GeneralPurposeConfig::new()
        .with_decode_padding_mode(DecodePaddingMode::Indifferent)
        .with_decode_allow_trailing_bits(true);
    let compact: String = s.chars().filter(|c| !matches!(c, '\n' | '\r' | ' ' | '\t')).collect();
    let alphabet = if compact.contains(['-', '_']) { &base64::alphabet::URL_SAFE } else { &base64::alphabet::STANDARD };
    GeneralPurpose::new(alphabet, config).decode(compact).map_err(|e| format!("invalid base64: {e}"))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("document", "app_close", "Close the app as its window's close button does: unsaved documents are written for recovery at once (the next start offers them), and the close waits until the store's writes (a Save just made, the recovery blob) have landed, at most 30 s. Watch `__brepClosing` {closing, closed, pending}; the headless host stays up, so follow with session_restart.", NoArgs, app_close, Mutate, MUTATE),
    app_command!("document", "doc_new", "Open a new document as a tab and make it active. Parts start with a datum (D) and its XY/XZ/YZ reference planes. `title` labels the untitled tab; `activate: false` opens it behind the visible document.", NewArgs, doc_new, Mutate, MUTATE, doc_new_result_schema),
    app_command!("document", "doc_load", "Open a `.nbrep` document (passed as text) in a new tab, run its history and fit the camera. Returns the run report.", LoadArgs, doc_load, Mutate, MUTATE, doc_load_result_schema),
    app_command!("document", "doc_json", "The active document as `.nbrep` (the history request with metadata).", NoArgs, doc_json, Read, READ, doc_json_result_schema),
    app_command!("document", "doc_import", "Append an Import 3D Model feature from STEP / IGES text or STL / OBJ / 3MF bytes (base64). STL, OBJ and 3MF go through mesh reconstruction on the runner; a 3MF is read to the 3MF core specification and its build instances are flattened first.", ImportArgs, doc_import, Mutate, MUTATE, doc_import_result_schema),
    app_command!("document", "doc_export", "Export the active document: brep (native JSON), step, stl, obj, iges, the sheet-metal flat pattern as dxf / svg, or the open drawing sheet as sheet_svg, all as `text`; glb (binary glTF 2.0 of the display mesh), 3mf (a 3MF package of the display mesh: one object per body, the document's colours as base materials) and EVERY drawing sheet as sheet_pdf (one PDF, a page per sheet in sheet order, with the count in `pages`) and the same set with the live 3D model as sheet_pdf_3d (PDF 1.7: a 3D box over every placement marked `threeD` and a last page given over to the model, read back in `model3d`) as `base64`. The two sheet formats also answer `texts`: every text string the file shows, read back from the bytes written. For a document whose PCB board has parts: fabrication (one zip of Gerber X2 layers, Excellon drills, the pick-and-place and parts-BOM CSVs and a README) as `base64`, with its `files` read back from the archive's directory, `readBack` (top/bottom copper flashes, plated/unplated drill hits, the outline's size in mm and the CSVs' row counts, counted from the files) and `warnings`; and the CSVs alone as pick_place_csv (JLCPCB CPL columns) and ecad_bom_csv (JLCPCB BOM columns), as `text` with their data `rows`. For a document with components or harness wires: bom_csv / bom_json, the assembly BOM the File menu exports (part rows carry material, densityGcm3, volumeMm3, massKg and totalMassKg read off each PART; waits for the measurements). For a document whose PCB schematic has parts: kicad_netlist, its KiCad netlist (`.net`, export version \"E\": the parts and every net with its pins, under the Connectivity panel's net names) as `text`.", ExportArgs, doc_export, Read, READ, schema_of::<Exported>),
    app_command!("document", "part_attributes_get", "The active document's own BOM part attributes (`partAttributes`) — the record the toolbar's Part Properties dialog edits, and the one an assembly's BOM reads off this part — with `record` (Mass in kg and Volume in mm³ computed from the document's geometry and density, Density in g/cm³ resolved from the Material when none is typed) and `massProperties`, the reading behind it. Waits for the measurement.", NoArgs, part_attributes_get, Read, READ, super::assembly_output_schema::part_attributes),
    app_command!("document", "part_attribute_set", "Write ONE of the active document's own BOM part attributes (Part_Number, Material, Density in g/cm³, or any custom field); a null or empty value removes it. One undo step per field. Mass is computed from the geometry and refused while there is any; Volume is never written.", PartAttributeArgs, part_attribute_set, Mutate, MUTATE, document_attributes_result_schema),
    app_command!("document", "docs_list", "The open document tabs and which is active. Each tab's `id` and `revision` are the document context any command accepts at the root of its arguments, `context: {documentId: id, expectedRevision: revision}`: checked right before the command runs, it must name the visible document (else `document_not_active`; a tab with a Work part edited in place answers `document_busy`), a write's `expectedRevision` must be the current revision (else `revision_conflict`), and an object result answers with `context: {documentId, expectedRevision}` as it stands after the command. Errors begin with that stable code. A geometry run the command starts moves the revision again when it lands.", NoArgs, docs_list, Read, READ, schema_of::<Tabs>),
    app_command!("document", "doc_activate", "Make the tab at `index` the active document.", IndexArgs, doc_activate, Mutate, MUTATE_NOWAIT, schema_of::<Tabs>),
    app_command!("document", "doc_close", "Close the tab at `index`, or the document whose handle is `id` wherever its tab is; refuses a dirty document unless `discard` is true.", CloseArgs, doc_close, Mutate, DESTRUCTIVE, schema_of::<Tabs>),
    app_command!("document", "doc_mark_clean", "Mark the active document as saved (the host wrote `doc_json` to disk).", NoArgs, doc_mark_clean, Mutate, MUTATE_NOWAIT),
];

