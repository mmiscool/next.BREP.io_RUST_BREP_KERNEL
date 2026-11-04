//! Document classes (`.nbrep` / `.fbrep` / `.tbrep`): a family's table and
//! its Generate, a template's input marks, and a read-back of a stored
//! document's class and stamps. The dialogs (member picker, template spin-out,
//! hand-edit prompt) are driven by their hit keys; these are the doors with no
//! pointer, and the read-back a script checks the files through.
use crate::automation::command::{app_command, parse_args, schema_of, Annotations, CommandSpec, Ctx, Handler, NoArgs, Outcome, Phase};
use crate::document_class::{self, DocumentClass};
use crate::family_table::{self, FamilyTable};
use crate::template::{self, TemplateInput};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TableArgs {
    /// The whole table: `{ columns: [{name, label}], rows: [{partNumber,
    /// revision, description, values: {column: "source"}}] }`.
    pub table: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InputsArgs {
    /// The marks: `[{ name, label?, min?, max?, choices? }]`. Empty clears them.
    pub inputs: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NewArgs {
    /// `normal` (a part, `.nbrep`), `family` (`.fbrep`) or `template` (`.tbrep`).
    pub class: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StoreArgs {
    /// A store identity (a path, or a name the store resolves into its models
    /// folder: `M3x10` / `bolt.fbrep`).
    pub name: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StoreWriteArgs {
    /// The file's name, extension included. A model extension (`.nbrep`,
    /// `.fbrep`, `.tbrep`) makes it a DOCUMENT of the store, listed by Open;
    /// any other name is handed to the user as a file (a download in the
    /// browser, a file in the app's models folder natively).
    pub name: String,
    /// The content, for a text file (a document, STEP, SVG, CSV, …).
    #[serde(default)]
    pub text: Option<String>,
    /// The content, for a binary file (a GLB, a PDF, a zip), as base64.
    #[serde(default)]
    pub base64: Option<String>,
    /// Replace a document that already exists; refused otherwise.
    #[serde(default)]
    pub overwrite: bool,
}

/// What `store_write` does with a name: keep it as one of the store's
/// documents, or hand it to the user as a file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StoreLane {
    Document,
    Export,
}

/// The lane a name takes — the store's own rule: a model extension is a
/// document, everything else is a file for the user (an export never shows up
/// in Open as if it were a part).
pub fn store_lane(name: &str) -> StoreLane {
    let ext = name.rsplit_once('.').map(|(_, e)| e.to_ascii_lowercase()).unwrap_or_default();
    if crate::store::MODEL_EXTENSIONS.contains(&ext.as_str()) {
        StoreLane::Document
    } else {
        StoreLane::Export
    }
}

fn doc_new_of_class(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NewArgs = parse_args(args)?;
    let class = DocumentClass::ALL
        .into_iter()
        .find(|class| class.slug() == a.class)
        .ok_or_else(|| format!("class `{}`: expected normal, family or template", a.class))?;
    let doc = crate::document::Document::fresh(ctx.app.docs.spawn_engine(), class);
    let index = ctx.app.docs.open_document(doc);
    Ok(Outcome::Done(json!({ "index": index, "class": class.slug() })))
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct GenerateArgs {
    /// On a PLM store: save a family with unsaved changes first ("Save and
    /// generate"). Without it such a family is refused, since the server
    /// generates from the saved revision. Ignored on the file system.
    #[serde(default)]
    pub save: bool,
}

fn family_generate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: GenerateArgs = parse_args(args)?;
    let (file, docs, store) = ctx.app.file_docs_store();
    if !document_class::is_family(docs.engine()) {
        return Err("the active document is not a family seed (.fbrep)".into());
    }
    let before = store.mutation_generation();
    let answer = move |report: &family_table::GenerateReport, after: u64| {
        Outcome::Done(json!({
            "summary": report.summary(),
            "rows": report.to_json(),
            // Every write that landed moves the store's generation by one, so
            // `writes` is the number of files Generate actually wrote.
            "writes": after - before,
        }))
    };
    let key = match file.begin_generate_active_family(docs, store, a.save)? {
        Ok(report) => return Ok(answer(&report, store.mutation_generation())),
        Err(key) => key,
    };
    plm_generate(key, answer)
}

/// The PLM half, frame by frame: wait for the family's save to reach the
/// server, then run the server's Generate as a frame-thread task and answer
/// when it does. Nothing here blocks a frame.
#[cfg(not(target_arch = "wasm32"))]
fn plm_generate(key: String, answer: impl Fn(&family_table::GenerateReport, u64) -> Outcome + Send + 'static) -> Result<Outcome, String> {
    use std::sync::{Arc, Mutex};
    let saved_by = web_time::Instant::now() + std::time::Duration::from_secs(60);
    let answered_by = saved_by + std::time::Duration::from_secs(300);
    let slot: Arc<Mutex<Option<Result<family_table::GenerateReport, String>>>> = Arc::new(Mutex::new(None));
    let mut started = false;
    Ok(Outcome::Poll(Box::new(move |ctx: &mut Ctx<'_>| {
        let (file, docs, store) = ctx.app.file_docs_store();
        if !started {
            if store.pending_writes() > 0 {
                return (web_time::Instant::now() > saved_by).then(|| Err("the family's save did not reach the server within 60 s".into()));
            }
            if let Some(error) = store.take_persistence_errors().into_iter().next() {
                return Some(Err(format!("the family was not saved, so nothing was generated: {error}")));
            }
            let client = store.plm_client()?;
            let json = docs.tab_engine().history_request_json();
            let (key, out) = (key.clone(), slot.clone());
            crate::plm::native::spawn(async move {
                let result = crate::plm::family::generate(&client, &key, &json).await.map_err(|e| e.to_string());
                *out.lock().unwrap() = Some(result);
            });
            started = true;
        }
        match slot.lock().unwrap().take() {
            Some(Ok(report)) => {
                let report = file.record_generate_report(report);
                Some(Ok(answer(&report, store.mutation_generation())))
            }
            Some(Err(error)) => Some(Err(error)),
            None => (web_time::Instant::now() > answered_by).then(|| Err("the PLM server did not answer Generate within 5 minutes".into())),
        }
    })))
}

#[cfg(target_arch = "wasm32")]
fn plm_generate(_key: String, _answer: impl Fn(&family_table::GenerateReport, u64) -> Outcome + Send + 'static) -> Result<Outcome, String> {
    Err("in the browser, Generate a PLM family from the family table's Generate button".into())
}

fn family_table_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine();
    Ok(Outcome::Done(json!({
        "class": document_class::document_class(engine).slug(),
        "table": serde_json::to_value(family_table::engine_table(engine)).unwrap_or(Value::Null),
    })))
}

fn family_table_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: TableArgs = parse_args(args)?;
    let table: FamilyTable = serde_json::from_value(a.table).map_err(|e| format!("table: {e}"))?;
    let engine = ctx.app.docs.engine_mut();
    if !document_class::is_family(engine) {
        return Err("the active document is not a family seed (.fbrep)".into());
    }
    family_table::apply_table(engine, &table, None);
    Ok(Outcome::Done(json!({ "rows": table.rows.len(), "columns": table.columns.len() })))
}

fn template_inputs_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine();
    Ok(Outcome::Done(json!({
        "class": document_class::document_class(engine).slug(),
        "inputs": serde_json::to_value(template::engine_inputs(engine)).unwrap_or(Value::Null),
    })))
}

fn template_inputs_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: InputsArgs = parse_args(args)?;
    let inputs: Vec<TemplateInput> = serde_json::from_value(a.inputs).map_err(|e| format!("inputs: {e}"))?;
    let engine = ctx.app.docs.engine_mut();
    if !document_class::is_template(engine) {
        return Err("the active document is not a template (.tbrep)".into());
    }
    template::apply_inputs(engine, &inputs, None);
    Ok(Outcome::Done(json!({ "inputs": inputs.len() })))
}

fn store_document(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StoreArgs = parse_args(args)?;
    let (_, _, store) = ctx.app.file_docs_store();
    let generation = store.mutation_generation();
    let Some(text) = store.read(&a.name) else {
        return Ok(Outcome::Done(json!({ "name": a.name, "exists": false, "storeGeneration": generation })));
    };
    let document: Value = serde_json::from_str(&text).unwrap_or(Value::Null);
    let hash = format!("{:016x}", brep_render::brep_kernel::stable_json_hash(&document));
    Ok(Outcome::Done(json!({
        "name": a.name,
        "exists": true,
        "bytes": text.len(),
        "contentHash": hash,
        // The class by the file's extension, and by the document's own field.
        "classByName": DocumentClass::of_name(&a.name).map(DocumentClass::slug),
        "class": DocumentClass::of_document(&document).slug(),
        "expressions": document.get("expressions"),
        "partAttributes": document.get(brep_render::engine_state::PART_ATTRIBUTES),
        "familySource": document.get(family_table::FAMILY_SOURCE_KEY),
        "templateSource": document.get(template::TEMPLATE_SOURCE_KEY),
        "hasFamilyTable": document.get(family_table::FAMILY_TABLE_KEY).is_some(),
        "hasTemplateInputs": document.get(template::TEMPLATE_INPUTS_KEY).is_some(),
        "storeGeneration": generation,
    })))
}

fn store_list(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let (_, _, store) = ctx.app.file_docs_store();
    Ok(Outcome::Done(json!({
        "backend": store.backend_label(),
        "names": store.list(),
        "storeGeneration": store.mutation_generation(),
    })))
}

fn store_read(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StoreArgs = parse_args(args)?;
    let (_, _, store) = ctx.app.file_docs_store();
    match store.read(&a.name) {
        Some(text) => Ok(Outcome::Done(json!({ "name": a.name, "exists": true, "bytes": text.len(), "text": text }))),
        None => Ok(Outcome::Done(json!({ "name": a.name, "exists": false }))),
    }
}

fn store_write(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StoreWriteArgs = parse_args(args)?;
    let (_, _, store) = ctx.app.file_docs_store();
    let lane = store_lane(&a.name);
    let (text, bytes) = match (a.text, a.base64) {
        (Some(_), Some(_)) => return Err("give `text` or `base64`, not both".into()),
        (Some(t), None) => (Some(t), None),
        (None, Some(b)) => (None, Some(super::cmd_document::base64_decode(&b)?)),
        (None, None) => return Err("missing the content: `text` or `base64`".into()),
    };
    let written = match lane {
        StoreLane::Document => {
            let text = match (text, bytes) {
                (Some(t), _) => t,
                (None, Some(b)) => String::from_utf8(b).map_err(|_| "a document is text; this base64 is not UTF-8".to_string())?,
                (None, None) => unreachable!(),
            };
            if !a.overwrite && store.read(&a.name).is_some() {
                return Err(format!("{} exists; pass overwrite: true", a.name));
            }
            let len = text.len();
            store.write(&a.name, &text)?;
            len
        }
        StoreLane::Export => match (text, bytes) {
            (Some(t), _) => {
                store.export_file_named(&a.name, &t)?;
                t.len()
            }
            (None, Some(b)) => {
                store.export_file_named_bytes(&a.name, &b)?;
                b.len()
            }
            (None, None) => unreachable!(),
        },
    };
    Ok(Outcome::Done(json!({
        "name": a.name,
        "bytes": written,
        "lane": match lane { StoreLane::Document => "document", StoreLane::Export => "export" },
        "storeGeneration": store.mutation_generation(),
    })))
}

fn store_remove(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StoreArgs = parse_args(args)?;
    let (_, _, store) = ctx.app.file_docs_store();
    let existed = store.read(&a.name).is_some();
    store.remove(&a.name)?;
    Ok(Outcome::Done(json!({ "name": a.name, "existed": existed, "storeGeneration": store.mutation_generation() })))
}

// These schemas describe the successful native payload. The MCP adapter adds
// frame/notices/settlement; the producers below remain the wire authority.
fn result_object(properties: Value) -> Value {
    let required: Vec<_> = properties.as_object().unwrap().keys().cloned().collect();
    json!({"type":"object","properties":properties,"required":required})
}

fn count_schema() -> Value { json!({"type":"integer","minimum":0}) }
fn class_schema() -> Value {
    json!({"enum":DocumentClass::KNOWN.map(DocumentClass::slug)})
}

// Keep derived DTO references rooted in the enclosing result, not under a
// property: JSON Schema's local $refs resolve from the document root.
fn with_derived_property<T: schemars::JsonSchema>(mut result: Value, name: &str) -> Value {
    let mut derived = schema_of::<T>();
    let root = derived.as_object_mut().unwrap();
    root.remove("$schema");
    if let Some(defs) = root.remove("$defs") { result["$defs"] = defs; }
    result["properties"][name] = derived;
    result
}

fn doc_new_of_class_result_schema() -> Value {
    result_object(json!({"index":count_schema(),"class":{"enum":DocumentClass::ALL.map(DocumentClass::slug)}}))
}
fn family_generate_result_schema() -> Value {
    result_object(json!({"summary":{"type":"string"},"writes":count_schema(),"rows":{
        "type":"array","items":result_object(json!({"partNumber":{"type":"string"},
        "status":{"enum":["written","skipped","failed"]},"reason":{"type":["string","null"]}}))
    }}))
}
fn family_table_get_result_schema() -> Value {
    with_derived_property::<Option<FamilyTableSchema>>(result_object(json!({"class":class_schema(),"table":{}})), "table")
}

// The table's JSON Schema. The table type is the kernel's (`brep_kernel::family`,
// shared with the PLM server), which derives no schema: these three describe
// exactly what `serde_json::to_value(&FamilyTable)` emits, and
// `the_family_table_schema_describes_the_table` pins both the schema text and a
// round trip of a full table through them.
#[derive(serde::Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase")]
#[schemars(rename = "FamilyTable")]
struct FamilyTableSchema {
    columns: Vec<FamilyColumnSchema>,
    rows: Vec<FamilyRowSchema>,
}

#[derive(serde::Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase")]
#[schemars(rename = "FamilyColumn")]
struct FamilyColumnSchema {
    /// The expression name this column drives (e.g. `length`).
    name: String,
    /// Optional display label; empty = show `name`.
    #[serde(default)]
    label: String,
}

#[derive(serde::Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "camelCase")]
#[schemars(rename = "FamilyRow")]
struct FamilyRowSchema {
    /// Typed by the user; the member's file name (`<partNumber>.nbrep`).
    part_number: String,
    /// Kept for the PLM; ignored on the file system.
    #[serde(default)]
    revision: String,
    #[serde(default)]
    description: String,
    /// Column name -> expression source text for this row ("12", "m * 2").
    #[serde(default)]
    values: std::collections::BTreeMap<String, String>,
}
fn family_table_set_result_schema() -> Value {
    result_object(json!({"rows":count_schema(),"columns":count_schema()}))
}
fn template_inputs_get_result_schema() -> Value {
    with_derived_property::<Option<Vec<TemplateInput>>>(result_object(json!({"class":class_schema(),"inputs":{}})), "inputs")
}
fn template_inputs_set_result_schema() -> Value {
    result_object(json!({"inputs":count_schema()}))
}
fn store_document_result_schema() -> Value {
    let absent = result_object(json!({"name":{"type":"string"},"exists":{"const":false},"storeGeneration":count_schema()}));
    let present = result_object(json!({
        "name":{"type":"string"},"exists":{"const":true},"bytes":count_schema(),
        "contentHash":{"type":"string"},"classByName":{"anyOf":[class_schema(),{"type":"null"}]},
        "class":class_schema(),"expressions":{},"partAttributes":{},"familySource":{},"templateSource":{},
        "hasFamilyTable":{"type":"boolean"},"hasTemplateInputs":{"type":"boolean"},"storeGeneration":count_schema()
    }));
    json!({"type":"object","oneOf":[absent,present]})
}
fn store_list_result_schema() -> Value {
    result_object(json!({"backend":{"type":"string"},"names":{"type":"array","items":{"type":"string"}},"storeGeneration":count_schema()}))
}
fn store_read_result_schema() -> Value {
    json!({"type":"object","oneOf":[
        result_object(json!({"name":{"type":"string"},"exists":{"const":false}})),
        result_object(json!({"name":{"type":"string"},"exists":{"const":true},"bytes":count_schema(),"text":{"type":"string"}}))
    ]})
}
fn store_write_result_schema() -> Value {
    result_object(json!({"name":{"type":"string"},"bytes":count_schema(),"lane":{"enum":["document","export"]},"storeGeneration":count_schema()}))
}
fn store_remove_result_schema() -> Value {
    result_object(json!({"name":{"type":"string"},"existed":{"type":"boolean"},"storeGeneration":count_schema()}))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("document", "doc_new_of_class", "Open a new document of a class — `normal` (a part), `family` (a family seed, `.fbrep`) or `template` (`.tbrep`) — as a tab and make it active: the toolbar's New menu. Parts start with a datum (D). The class is in the document before it has a file.", NewArgs, doc_new_of_class, Mutate, MUTATE, doc_new_of_class_result_schema),
    app_command!("document", "family_generate", "Generate the ACTIVE family seed (.fbrep): write every table row as `<part number>.nbrep` beside the family file, the family's model with the row's values baked into its expressions and a `familySource` stamp. An unchanged row is skipped; a row that cannot be written is reported and the rest still generate. Returns each row's outcome, a summary, and `writes` — how many files the store actually wrote. The same Generate the table editor's button runs. On a PLM store (the family is a `part/…/rev/…` revision) the SERVER writes each member as a part, from the family's saved revision: the app builds and bakes every member first and sends the ones that build, each row's revision column is the revision written, and the rows come back written, skipped (unchanged) or failed with the server's sentence. A family with unsaved changes is refused there unless `save: true`.", GenerateArgs, family_generate, Mutate, MUTATE, family_generate_result_schema),
    app_command!("document", "family_table_get", "The active document's class and its family table (`familyTable`; empty when it has none).", NoArgs, family_table_get, Read, READ, family_table_get_result_schema),
    app_command!("document", "family_table_set", "Replace the active family seed's table as one undoable edit. Refused on a document that is not a family.", TableArgs, family_table_set, Mutate, MUTATE_NOWAIT, family_table_set_result_schema),
    app_command!("document", "template_inputs_get", "The active document's class and its template input marks (`templateInputs`).", NoArgs, template_inputs_get, Read, READ, template_inputs_get_result_schema),
    app_command!("document", "template_inputs_set", "Replace the active template's input marks as one undoable edit (what the Expressions pane's Template inputs grid edits). Refused on a document that is not a template.", InputsArgs, template_inputs_set, Mutate, MUTATE_NOWAIT, template_inputs_set_result_schema),
    app_command!("document", "store_document", "Read one document back from the app's store: whether it exists, its class (by extension and by its own field), its expressions, part attributes, family or template stamp, a content hash, and the store's mutation generation (which every landed write moves by one).", StoreArgs, store_document, Read, READ, store_document_result_schema),
    app_command!("document", "store_list", "The names of the documents in the app's store — what Open lists — with the store's label (a models folder natively, browser storage or a PLM server in the browser) and its mutation generation.", NoArgs, store_list, Read, READ, store_list_result_schema),
    app_command!("document", "store_read", "One stored document's text by name (`exists: false` when there is none). This is the browser's `document_open` lane: the server reads the file, the page reads its store.", StoreArgs, store_read, Read, READ, store_read_result_schema),
    app_command!("document", "store_write", "Write a file by name: a model extension (`.nbrep` / `.fbrep` / `.tbrep`) makes it a document of the store — refused if one exists unless `overwrite` — and any other name is handed to the user as a file (a browser download; the app's models folder natively), so an export never lands in Open as a part. `text` or `base64` carries the content. This is the browser's `document_save` / `document_export` lane.", StoreWriteArgs, store_write, Mutate, MUTATE_NOWAIT, store_write_result_schema),
    CommandSpec { name: "store_remove", group: "document", doc: "Delete a stored document by name (`Ok` when it is already gone).", phase: Phase::Mutate, annotations: Annotations { read_only: false, destructive: true, idempotent: true, waits: false }, args_schema: schema_of::<StoreArgs>, result_schema: store_remove_result_schema, handler: Handler::App(store_remove) },
];

