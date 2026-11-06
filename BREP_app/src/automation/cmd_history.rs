//! The feature history: add / edit / delete / reorder / roll / undo / redo,
//! expressions, and the run report. Every mutation returns at once with the
//! engine's report; the host applies the idle contract (§6) before answering
//! the tool call.
use crate::automation::command::{app_command, parse_args, schema_of, CommandSpec, Ctx, NoArgs, Outcome};
use crate::automation::cmd_document::parse;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StageArgs {
    /// One batch item `{type, params, id?, persistent_data?}`, seeded and validated as `feature_add_many` seeds its items.
    pub item: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FeatureAddArgs {
    /// A complete feature `{type, inputParams: {id, …}, persistentData?}`; the host seeds defaults and validates.
    pub feature: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FeatureAddManyArgs {
    pub features: Vec<Value>,
    pub parts: Option<Vec<Value>>,
    pub constraints: Option<Vec<Value>>,
    pub expressions: Option<String>,
    pub metadata: Option<Value>,
    #[serde(default)]
    pub rollback_on_error: bool,
    #[serde(default)]
    pub structured: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IdArgs {
    pub id: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SetParamsArgs {
    pub id: String,
    /// The feature's complete new `inputParams` object.
    pub input_params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SetPersistentArgs {
    pub id: String,
    /// The key inside `persistentData` to replace (e.g. `sketch`).
    pub key: String,
    pub value: Value,
}

/// Address one feature by id OR by history index. The index lane is the only
/// way to reach a feature whose `inputParams` carry no `id`.
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DeleteArgs {
    /// The feature's id (`E3`). Give this or `index`.
    #[serde(default)]
    pub id: Option<String>,
    /// The feature's position in the history. Give this or `id`.
    #[serde(default)]
    pub index: Option<usize>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ReorderArgs {
    pub index: usize,
    /// Move up (towards index 0) or down.
    pub up: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RollArgs {
    /// Feature index to roll to (the feature executes, later ones do not).
    pub index: usize,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct GroupEndArgs {
    /// The token `history_group_begin` answered.
    pub token: u64,
    /// Restore the document as it was when the group opened (the group's
    /// edits are taken back and leave no undo step).
    #[serde(default)]
    pub rollback: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NextIdArgs {
    /// The id base, normally the feature's shortName (`E` → `E7`).
    pub base: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExpressionsArgs {
    /// The Expressions panel script (`name = expr;` statements).
    pub script: String,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct Report {
    /// The engine's run report (`featureErrors`, `featureNotes`, `featureFulfilment` — a
    /// feature id → `{requested, applied, rejected, summary}` object for every feature
    /// that succeeded on fewer references than it was asked —, `featureRefusals` — a
    /// feature id → the kernel's typed refusal `{class, …payload, stage, message}`, plus
    /// `step` (`rotation` / `translation`) when the feature records which motion
    /// refused, for every feature that failed with one, the key absent when none did —,
    /// `featureApproximations` — a feature id → `[{code, body, measured, bar, volume_bound,
    /// edges, message, summary}]` array for every SUCCESSFUL feature that carries a measured
    /// approximation (the STEP import's `import.shell_closure`), the key absent when none
    /// does —, `unresolved`, `displayErrors`, timings…).
    pub report: Value,
    pub step: usize,
    pub history_len: usize,
}

/// Native batch replies have a legacy report, a preflight rejection, or an
/// evaluated transaction. Keep this union at the root so MCP wrappers can
/// extend the legacy branch without relocating local schema references.
fn feature_add_many_result_schema() -> Value {
    // OPEN objects (no `additionalProperties: false`), unlike `schema_dsl`.
    fn open_object(properties: Value, required: &[&str]) -> Value {
        json!({"type":"object","properties":properties,"required":required})
    }
    let strings = json!({"type":"array","items":{"type":"string"}});
    let indices = json!({"type":"array","items":{"type":"integer","minimum":0}});
    let string_map = json!({"type":"object","additionalProperties":{"type":"string"}});
    let outputs = json!({"type":"object","additionalProperties":strings});
    let mut legacy = schema_of::<Report>();
    let mut defs = legacy.as_object_mut().unwrap().remove("$defs").unwrap_or(json!({}));
    legacy.as_object_mut().unwrap().remove("$schema");
    // Rejected input is echoed before native validation: alias/id need not
    // have their successful string types (and missing values become null).
    let identity = json!({
        "position":{"type":"integer","minimum":0},"alias":{},"id":{},
        "status":{"enum":["evaluated","failed","not_evaluated"]},"committed":{"type":"boolean"}
    });
    let mut item = identity.clone();
    item.as_object_mut().unwrap().extend(json!({
        "dependencies":indices,"dependencyFailures":indices,
        "dependencyFailure":open_object(json!({"part":{"type":"string"}}), &["part"]),
        "error":{"anyOf":[strings,open_object(json!({"category":{"type":"string"},"message":{"type":"string"},"path":{"type":"string"}}), &["category","message","path"])]},
        "generatedSolids":strings,
        "generatedReferences":{"type":"array","items":open_object(json!({"solid":{"type":"string"},"faces":strings,"edges":strings}), &["solid","faces","edges"])},
        "diagnostic":open_object(json!({
            "operation":{"type":"string"},"owningComponent":{"type":["string","null"]},
            "operands":{"type":"object"},"affectedTopology":{},
            "category":{"type":["string","null"]},"kernelClass":{},"detailId":{"type":"string"}
        }), &["operation","owningComponent","operands","affectedTopology","category","kernelClass","detailId"])
    }).as_object().unwrap().clone());
    defs["BatchFeatureResult"] = open_object(item, &["position","alias","id","status","committed"]);
    let items = json!({"type":"array","items":{"$ref":"#/$defs/BatchFeatureResult"}});
    let mut part = identity.clone();
    part["items"] = items.clone();
    part["outputs"] = outputs.clone();
    let parts = json!({"type":"array","items":open_object(part, &["position","alias","id","status","committed"])});
    let mut constraint = identity.clone();
    constraint["error"] = json!({"type":["string","null"]});
    // Assembly diagnostics are engine/plugin report values, not a fixed DTO.
    constraint["diagnostics"] = json!({"type":["object","null"]});
    constraint["dependencyFailures"] = indices;
    let constraints = json!({"type":"array","items":open_object(constraint, &["position","alias","id","status","committed","error","diagnostics","dependencyFailures"])});
    let transaction = open_object(json!({
        "validationAtomic":{"const":true},"geometryAtomic":{"type":"boolean"},
        "idCountersRolledBack":{"type":"boolean"},"runtimeGenerationRolledBack":{"const":false}
    }), &["validationAtomic","geometryAtomic","idCountersRolledBack","runtimeGenerationRolledBack"]);
    let common = json!({
        "success":{"type":"boolean"},"items":items,"partResults":parts,"constraintResults":constraints,
        "aliases":string_map,"partAliases":{"type":"object","additionalProperties":string_map},"diagnosticsTool":{"const":"geometry_diagnostics"},
        "rolledBack":{"type":"boolean"},"transaction":transaction
    });
    let required = ["success","items","partResults","constraintResults","aliases","partAliases","diagnosticsTool","rolledBack","transaction"];
    let mut normal = common.clone();
    normal["outputs"] = outputs;
    normal["constraintStatuses"] = json!({"type":"array","items":{"type":"object"}});
    normal["dof"] = json!({"type":["object","null"]});
    let mut normal_required = required.to_vec();
    normal_required.extend(["outputs","constraintStatuses","dof"]);
    let mut failed_part = common;
    failed_part["success"] = json!({"const":false});
    failed_part["rolledBack"] = json!({"const":true});
    let mut rejected_item = identity;
    rejected_item["status"] = json!({"enum":["failed","not_evaluated"]});
    rejected_item["committed"] = json!({"const":false});
    let rejection = open_object(json!({
        "success":{"const":false},"status":{"const":"rejected"},"modelChanged":{"const":false},
        "errors":{"type":"array","items":open_object(json!({"path":{"type":"string"},"message":{"type":"string"}}), &["path","message"])},
        "items":{"type":"array","items":open_object(rejected_item, &["position","alias","id","status","committed"])}
    }), &["success","status","modelChanged","errors","items"]);
    let mut early_failure = open_object(failed_part, &required);
    // The early failed-part return omits these fields. Without this guard a
    // malformed rolled-back normal result could match the weaker branch.
    early_failure["not"] = json!({"anyOf":[
        {"required":["outputs"]},{"required":["constraintStatuses"]},{"required":["dof"]}
    ]});
    json!({"type":"object","anyOf":[legacy,rejection,open_object(normal,&normal_required),early_failure],"$defs":defs})
}

// Stable history envelopes; document-authored JSON and native reports remain
// extensible. Keep these factories together for batched contract validation.
fn history_object(properties: Value) -> Value {
    let required: Vec<String> = properties.as_object().unwrap().keys().cloned().collect();
    json!({"type":"object","properties":properties,"required":required})
}

fn history_listing_result_schema() -> Value {
    let feature = history_object(json!({
        "index":{"type":"integer","minimum":0},"type":{"type":"string"},"id":{"type":"string"}
    }));
    history_object(json!({
        "listing":history_object(json!({"step":{"type":"integer","minimum":0},"features":{"type":"array","items":feature}})),
        "report":{},"step":{"type":"integer","minimum":0},
        "model":history_object(json!({
            "step":{"type":"integer","minimum":0},"solidCount":{"type":"integer","minimum":0},"totalTris":{"type":"integer","minimum":0},
            "solids":{"type":"array","items":history_object(json!({
                "name":{"type":"string"},"tris":{"type":"integer","minimum":0},
                "min":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3},
                "max":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3}
            }))}
        })),"idle":{"type":"boolean"},
        "selectedFeature":{"type":["string","null"]},"can_undo":{"type":"boolean"},"can_redo":{"type":"boolean"}
    }))
}

fn feature_params_result_schema() -> Value {
    history_object(json!({
        "id":{"type":"string"},"index":{"type":"integer","minimum":0},
        "type":{"type":["string","null"]},"inputParams":{},"persistentData":{}
    }))
}

fn feature_stage_result_schema() -> Value {
    history_object(json!({"id":{"type":"string"},"staged":{"const":true}}))
}

fn feature_stage_poll_result_schema() -> Value {
    let mut success = history_object(json!({
        "id":{"type":"string"},"committed":{"const":true},"status":{"const":"evaluated"},
        "generatedSolids":{"type":"array","items":{"type":"string"}},"rolledBack":{"const":false}
    }));
    success["properties"]["interleavedEdit"] = json!({"const":true});
    let mut failure = history_object(json!({
        "id":{"type":"string"},"committed":{"const":false},"status":{"const":"failed"},
        "generatedSolids":{"type":"array","items":{"type":"string"},"maxItems":0},
        "error":{"type":"string"},"rolledBack":{"const":true}
    }));
    failure["properties"]["interleavedEdit"] = json!({"const":true});
    json!({"type":"object","anyOf":[
        history_object(json!({"pending":{"const":true}})),success,failure
    ]})
}

fn run_cancel_result_schema() -> Value {
    history_object(json!({"cancelled":{"type":"boolean"}}))
}
fn history_group_begin_result_schema() -> Value {
    history_object(json!({"token":{"type":"integer","minimum":0}}))
}
fn history_group_end_result_schema() -> Value {
    history_object(json!({"rolledBack":{"type":"boolean"}}))
}
fn next_feature_id_result_schema() -> Value {
    history_object(json!({"id":{"type":"string"}}))
}
fn expressions_get_result_schema() -> Value {
    // parse() returns decoded JSON when source happens to be JSON, otherwise
    // its original string. Configurator is copied from the authored document.
    history_object(json!({"expressions":{},"configurator":{},
        "variables":{"type":"array","items":history_object(json!({"name":{"type":"string"},"expr":{"type":"string"}}))}
    }))
}

fn report(ctx: &Ctx<'_>, report: &str) -> Result<Outcome, String> {
    if let Some(error) = parse(report).get("error").and_then(Value::as_str) {
        // A refusal returned instead of the engine's stored evaluation report
        // must not be replaced by a stale success during settling. Rebuild
        // failures retain report.error (undo may already have applied).
        let current = parse(&ctx.app.docs.engine().history_report_json());
        if error.starts_with("invalid_arguments:") || current.get("error").and_then(Value::as_str) != Some(error) {
            return Err(error.to_string());
        }
    }
    let engine = ctx.app.docs.engine();
    serde_json::to_value(Report { report: parse(report), step: engine.history_rollback(), history_len: engine.history_len() })
        .map(Outcome::Done)
        .map_err(|e| e.to_string())
}

fn index_of(ctx: &Ctx<'_>, id: &str) -> Result<usize, String> {
    ctx.app.docs.engine().history.index_of(id).ok_or_else(|| format!("no feature with id `{id}`"))
}

fn feature_add(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: FeatureAddArgs = parse_args(args)?;
    let text = serde_json::to_string(&a.feature).map_err(|e| e.to_string())?;
    let r = ctx.app.docs.engine_mut().add_feature(&text)?;
    let index = ctx.app.docs.engine().history_len().saturating_sub(1);
    let id = ctx.app.docs.engine().feature_id_at(index);
    let mut out = report(ctx, &r)?;
    if let Outcome::Done(v) = &mut out {
        v["index"] = json!(index);
        v["id"] = json!(id);
    }
    Ok(out)
}

/// Stage one feature and return at once (`{id, staged: true}`): its run goes
/// to the runner, and [`feature_stage_result`] finishes it — committed as one
/// undo checkpoint, or restored exactly — on a later call. No frame waits for
/// the geometry, and no host request is held open while it evaluates.
fn feature_stage(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: StageArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().stage_feature_begin(&a.item)?;
    Ok(Outcome::Done(json!({ "id": id, "staged": true })))
}

/// The staged feature's outcome: `{pending: true}` while its run is in
/// flight, then its row `{id, committed, status, generatedSolids, error?,
/// rolledBack}` (once; the stage is then over).
fn feature_stage_result(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    match engine.stage_feature_poll() {
        Some(row) => row.map(Outcome::Done),
        None if engine.stage_feature_pending() => Ok(Outcome::Done(json!({ "pending": true }))),
        None => Err("no feature is staged on this document".into()),
    }
}

fn feature_add_many(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: FeatureAddManyArgs = parse_args(args.clone())?;
    if a.structured || a.rollback_on_error || a.parts.is_some() || a.constraints.is_some() || a.expressions.is_some() || a.metadata.is_some() {
        return match ctx.app.docs.engine_mut().author_batch(&args) {
            Ok(result) => Ok(Outcome::Done(result)),
            Err(error) => {
                let position = error.strip_prefix("features[").and_then(|s| s.split_once(']')).and_then(|(i,_)| i.parse::<usize>().ok());
                let path = error.split(": ").next().unwrap_or("batch");
                let items: Vec<Value> = a.features.iter().enumerate().map(|(i,f)| json!({"position":i,"alias":f["alias"],"id":f.get("id").or_else(|| f.get("inputParams").and_then(|p| p.get("id"))),"status":if position == Some(i) {"failed"} else {"not_evaluated"},"committed":false})).collect();
                Ok(Outcome::Done(json!({"success":false,"status":"rejected","modelChanged":false,"errors":[{"path":path,"message":error}],"items":items})))
            }
        };
    }
    let r = ctx.app.docs.engine_mut().add_features(&a.features);
    report(ctx, &r)
}

fn geometry_diagnostics(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: IdArgs = parse_args(args)?;
    ctx.app.docs.engine().geometry_diagnostics(&a.id).map(Outcome::Done).ok_or_else(|| format!("no geometry diagnostics for {}", a.id))
}

fn feature_params(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: IdArgs = parse_args(args)?;
    let index = index_of(ctx, &a.id)?;
    let engine = ctx.app.docs.engine();
    Ok(Outcome::Done(json!({
        "id": a.id,
        "index": index,
        "type": engine.feature_type_at(index),
        "inputParams": parse(&engine.feature_params_json(index)),
        "persistentData": engine.history.feature_persistent_data(index),
    })))
}

fn feature_set_params(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SetParamsArgs = parse_args(args)?;
    let text = serde_json::to_string(&a.input_params).map_err(|e| e.to_string())?;
    let r = ctx.app.docs.engine_mut().update_feature_params(&a.id, &text)?;
    report(ctx, &r)
}

fn feature_set_persistent(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SetPersistentArgs = parse_args(args)?;
    let index = index_of(ctx, &a.id)?;
    let engine = ctx.app.docs.engine_mut();
    engine.history.set_feature_persistent_field(index, &a.key, a.value);
    let step = engine.history_rollback();
    let r = engine.roll_to(step);
    report(ctx, &r)
}

fn feature_delete(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: DeleteArgs = parse_args(args)?;
    // Resolve to an index either way, so both lanes refuse an absent feature
    // instead of silently re-running an unchanged history.
    let index = match (&a.id, a.index) {
        (Some(id), None) => index_of(ctx, id)?,
        (None, Some(index)) => {
            let len = ctx.app.docs.engine().history_len();
            if index >= len {
                return Err(format!("index {index} out of range ({len} features)"));
            }
            index
        }
        _ => return Err("give exactly one of `id` or `index`".into()),
    };
    let r = ctx.app.docs.engine_mut().delete_feature_at(index);
    report(ctx, &r)
}

fn feature_reorder(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ReorderArgs = parse_args(args)?;
    let len = ctx.app.docs.engine().history_len();
    if a.index >= len {
        return Err(format!("index {} out of range ({len} features)", a.index));
    }
    let r = ctx.app.docs.engine_mut().reorder_feature(a.index, a.up);
    report(ctx, &r)
}

fn roll_to(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: RollArgs = parse_args(args)?;
    let len = ctx.app.docs.engine().history_len();
    if a.index >= len {
        return Err(format!("index {} out of range ({len} features)", a.index));
    }
    let r = ctx.app.docs.engine_mut().roll_to(a.index);
    report(ctx, &r)
}

fn undo(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    if !ctx.app.docs.engine().can_undo() {
        return Err("nothing to undo".into());
    }
    let r = ctx.app.docs.engine_mut().undo();
    report(ctx, &r)
}

fn redo(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    if !ctx.app.docs.engine().can_redo() {
        return Err("nothing to redo".into());
    }
    let r = ctx.app.docs.engine_mut().redo();
    report(ctx, &r)
}

fn history_listing(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(listing_json(ctx)))
}

/// What `history_listing` answers (also the body of a `settle` reply).
pub(crate) fn listing_json(ctx: &Ctx<'_>) -> Value {
    let engine = ctx.app.docs.engine();
    json!({
        "listing": parse(&engine.history_listing_json()),
        "report": parse(&engine.history_report_json()),
        "step": engine.history_rollback(),
        "selectedFeature": engine.selected_feature(),
        "can_undo": engine.can_undo(),
        "can_redo": engine.can_redo(),
        "model": parse(&ctx.app.model_signature_json()),
        "idle": crate::automation::cmd_frame::frame_info(ctx).idle,
    })
}

fn run_cancel(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let cancelled = ctx.app.docs.engine_mut().cancel_run();
    Ok(Outcome::Done(json!({ "cancelled": cancelled })))
}

fn history_group_begin(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({ "token": ctx.app.docs.engine_mut().history_group_begin() })))
}

fn history_group_end(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: GroupEndArgs = parse_args(args)?;
    let rolled_back = ctx.app.docs.engine_mut().history_group_end(a.token, a.rollback)?;
    Ok(Outcome::Done(json!({ "rolledBack": rolled_back })))
}

fn next_feature_id(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NextIdArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().next_feature_id(&a.base);
    Ok(Outcome::Done(json!({ "id": id })))
}

fn expressions_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine();
    Ok(Outcome::Done(json!({
        "expressions": parse(&engine.expressions_json()),
        "variables": parse(&engine.expression_variables_json()),
        "configurator": parse(&engine.configurator_json()),
    })))
}

fn expressions_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ExpressionsArgs = parse_args(args)?;
    let r = ctx.app.docs.engine_mut().set_expressions(&a.script);
    report(ctx, &r)
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("history", "geometry_diagnostics", "Full kernel diagnostics by detailId from structured batches or interference checks (retained after rollback). Use __report__ for the last batch rebuild report. A new batch replaces the diagnostic store.", IdArgs, geometry_diagnostics, Read, READ, super::diagnostic_output_schema::geometry_diagnostics_schema),
    app_command!("history", "feature_add", "Append a feature to the history and run it; the rollback moves to the new feature. Returns its index and id with the run report.", FeatureAddArgs, feature_add, Mutate, MUTATE, schema_of::<Report>),
    app_command!("history", "feature_stage", "Stage ONE feature (a `feature_add_many` item) without blocking: the recipe, metadata and parts library are snapshotted, the feature is appended and its run handed to the runner, and the reply `{id, staged: true}` comes at once. `feature_stage_result` finishes it. One staged feature per document at a time.", StageArgs, feature_stage, Mutate, MUTATE_NOWAIT, feature_stage_result_schema),
    app_command!("history", "feature_stage_result", "The staged feature's outcome: `{pending: true}` while its run is in flight, then `{id, committed, status, generatedSolids, error?, rolledBack}` — committed as one undo step, or the snapshot restored exactly when it failed. Answered once; the stage then ends.", NoArgs, feature_stage_result, Read, MUTATE_NOWAIT, feature_stage_poll_result_schema),
    app_command!("history", "feature_add_many", "Append a feature batch. The legacy path uses one rebuild; structured batches support native parts, aliases, constraints, evaluated per-item results and optional geometry rollback.", FeatureAddManyArgs, feature_add_many, Mutate, MUTATE, feature_add_many_result_schema),
    app_command!("history", "feature_params", "A feature's type, index, inputParams and persistentData by id.", IdArgs, feature_params, Read, READ, feature_params_result_schema),
    app_command!("history", "feature_set_params", "Replace a feature's inputParams and rerun from it.", SetParamsArgs, feature_set_params, Mutate, MUTATE, schema_of::<Report>),
    app_command!("history", "feature_set_persistent", "Replace one key of a feature's persistentData (e.g. a sketch's `sketch` block) and rerun.", SetPersistentArgs, feature_set_persistent, Mutate, MUTATE, schema_of::<Report>),
    app_command!("history", "feature_delete", "Delete a feature by `id` or by history `index` (exactly one) and rerun; the rollback clamps to the new length. The index lane reaches a feature that carries no id.", DeleteArgs, feature_delete, Mutate, DESTRUCTIVE, schema_of::<Report>),
    app_command!("history", "feature_reorder", "Swap the feature at `index` with its neighbour (up = towards 0); the rollback moves to the target.", ReorderArgs, feature_reorder, Mutate, MUTATE, schema_of::<Report>),
    app_command!("history", "roll_to", "Execute the history up to and including feature `index`.", RollArgs, roll_to, Mutate, MUTATE, schema_of::<Report>),
    app_command!("history", "undo", "Undo the last history edit and rerun.", NoArgs, undo, Mutate, MUTATE, schema_of::<Report>),
    app_command!("history", "redo", "Redo the last undone history edit and rerun.", NoArgs, redo, Mutate, MUTATE, schema_of::<Report>),
    app_command!("history", "history_listing", "The feature list `{step, features:[{index,type,id}]}`, the last run report, the rollback step, the feature selected in the tree (`selectedFeature`, null when none) and undo/redo availability.", NoArgs, history_listing, Read, READ, history_listing_result_schema),
    app_command!("history", "run_cancel", "Cancel the in-flight history run, if any.", NoArgs, run_cancel, Mutate, MUTATE_NOWAIT, run_cancel_result_schema),
    app_command!("history", "history_group_begin", "Open an undo group on the document: every edit until `history_group_end` is ONE undo step (a script's run). Answers `{token}`. A group already open is replaced.", NoArgs, history_group_begin, Mutate, MUTATE_NOWAIT, history_group_begin_result_schema),
    app_command!("history", "history_group_end", "Close the undo group `token`. With `rollback: true` the group's edits are taken back (the document as it was when the group opened, no undo step left) and the history re-runs. Answers `{rolledBack}`.", GroupEndArgs, history_group_end, Mutate, MUTATE_NOWAIT, history_group_end_result_schema),
    app_command!("history", "next_feature_id", "Reserve the next unique feature id for a base (the feature's shortName), the way the add palette does.", NextIdArgs, next_feature_id, Mutate, MUTATE_NOWAIT, next_feature_id_result_schema),
    app_command!("history", "expressions_get", "The Expressions panel script, the variables it defines, and the configurator values.", NoArgs, expressions_get, Read, READ, expressions_get_result_schema),
    app_command!("history", "expressions_set", "Replace the Expressions script and rerun the history.", ExpressionsArgs, expressions_set, Mutate, MUTATE, schema_of::<Report>),
];
