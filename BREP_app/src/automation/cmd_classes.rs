//! Document classes (`.nbrep` / `.fbrep` / `.tbrep`): a family's table and
//! its Generate, a template's input marks, and a read-back of a stored
//! document's class and stamps. The dialogs (member picker, template spin-out,
//! hand-edit prompt) are driven by their hit keys; these are the doors with no
//! pointer, and the read-back a script checks the files through.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
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

fn doc_new_of_class(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NewArgs = parse_args(args)?;
    let class = DocumentClass::ALL
        .into_iter()
        .find(|class| class.slug() == a.class)
        .ok_or_else(|| format!("class `{}`: expected normal, family or template", a.class))?;
    let mut engine = ctx.app.docs.spawn_engine();
    let _ = engine.set_history_json(crate::document::EMPTY_DOCUMENT);
    // Before `Document::new`, so the class is part of the clean baseline —
    // what the toolbar's New menu does.
    document_class::set_document_class(&mut engine, class);
    let index = ctx.app.docs.open_document(crate::document::Document::new(engine));
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
    let report = file.generate_active_family(docs, store, a.save)?;
    let after = store.mutation_generation();
    Ok(Outcome::Done(json!({
        "summary": report.summary(),
        "rows": report.to_json(),
        // Every write that landed moves the store's generation by one, so
        // `writes` is the number of files Generate actually wrote.
        "writes": after - before,
    })))
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

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "doc_new_of_class", group: "document", doc: "Open a new, empty document of a class — `normal` (a part), `family` (a family seed, `.fbrep`) or `template` (`.tbrep`) — as a tab and make it active: the toolbar's New menu. The class is in the document before it has a file.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NewArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(doc_new_of_class) },
    CommandSpec { name: "family_generate", group: "document", doc: "Generate the ACTIVE family seed (.fbrep): write every table row as `<part number>.nbrep` beside the family file, the family's model with the row's values baked into its expressions and a `familySource` stamp. An unchanged row is skipped; a row that cannot be written is reported and the rest still generate. Returns each row's outcome, a summary, and `writes` — how many files the store actually wrote. The same Generate the table editor's button runs. On a PLM store (the family is a `part/…/rev/…` revision) the SERVER writes each member as a part, from the family's saved revision: the app builds and bakes every member first and sends the ones that build, each row's revision column is the revision written, and the rows come back written, skipped (unchanged) or failed with the server's sentence. A family with unsaved changes is refused there unless `save: true`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<GenerateArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(family_generate) },
    CommandSpec { name: "family_table_get", group: "document", doc: "The active document's class and its family table (`familyTable`; empty when it has none).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(family_table_get) },
    CommandSpec { name: "family_table_set", group: "document", doc: "Replace the active family seed's table as one undoable edit. Refused on a document that is not a family.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<TableArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(family_table_set) },
    CommandSpec { name: "template_inputs_get", group: "document", doc: "The active document's class and its template input marks (`templateInputs`).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(template_inputs_get) },
    CommandSpec { name: "template_inputs_set", group: "document", doc: "Replace the active template's input marks as one undoable edit (what the Expressions pane's Template inputs grid edits). Refused on a document that is not a template.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<InputsArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(template_inputs_set) },
    CommandSpec { name: "store_document", group: "document", doc: "Read one document back from the app's store: whether it exists, its class (by extension and by its own field), its expressions, part attributes, family or template stamp, a content hash, and the store's mutation generation (which every landed write moves by one).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<StoreArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(store_document) },
];
