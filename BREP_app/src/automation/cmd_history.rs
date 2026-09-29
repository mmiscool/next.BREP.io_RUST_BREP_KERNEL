//! The feature history: add / edit / delete / reorder / roll / undo / redo,
//! expressions, and the run report. Every mutation returns at once with the
//! engine's report; the host applies the idle contract (§6) before answering
//! the tool call.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::automation::cmd_document::parse;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

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
    /// `unresolved`, `displayErrors`, timings…).
    pub report: Value,
    pub step: usize,
    pub history_len: usize,
}

fn report(ctx: &Ctx<'_>, report: &str) -> Result<Outcome, String> {
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

fn feature_add_many(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: FeatureAddManyArgs = parse_args(args)?;
    let r = ctx.app.docs.engine_mut().add_features(&a.features);
    report(ctx, &r)
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
    let engine = ctx.app.docs.engine();
    Ok(Outcome::Done(json!({
        "listing": parse(&engine.history_listing_json()),
        "report": parse(&engine.history_report_json()),
        "step": engine.history_rollback(),
        "selectedFeature": engine.selected_feature(),
        "can_undo": engine.can_undo(),
        "can_redo": engine.can_redo(),
    })))
}

fn run_cancel(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let cancelled = ctx.app.docs.engine_mut().cancel_run();
    Ok(Outcome::Done(json!({ "cancelled": cancelled })))
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
    CommandSpec { name: "feature_add", group: "history", doc: "Append a feature to the history and run it; the rollback moves to the new feature. Returns its index and id with the run report.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<FeatureAddArgs>, result_schema: schema_of::<Report>, handler: Handler::App(feature_add) },
    CommandSpec { name: "feature_add_many", group: "history", doc: "Append several features with one run.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<FeatureAddManyArgs>, result_schema: schema_of::<Report>, handler: Handler::App(feature_add_many) },
    CommandSpec { name: "feature_params", group: "history", doc: "A feature's type, index, inputParams and persistentData by id.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<IdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(feature_params) },
    CommandSpec { name: "feature_set_params", group: "history", doc: "Replace a feature's inputParams and rerun from it.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<SetParamsArgs>, result_schema: schema_of::<Report>, handler: Handler::App(feature_set_params) },
    CommandSpec { name: "feature_set_persistent", group: "history", doc: "Replace one key of a feature's persistentData (e.g. a sketch's `sketch` block) and rerun.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<SetPersistentArgs>, result_schema: schema_of::<Report>, handler: Handler::App(feature_set_persistent) },
    CommandSpec { name: "feature_delete", group: "history", doc: "Delete a feature by `id` or by history `index` (exactly one) and rerun; the rollback clamps to the new length. The index lane reaches a feature that carries no id.", phase: Phase::Mutate, annotations: Annotations::DESTRUCTIVE, args_schema: schema_of::<DeleteArgs>, result_schema: schema_of::<Report>, handler: Handler::App(feature_delete) },
    CommandSpec { name: "feature_reorder", group: "history", doc: "Swap the feature at `index` with its neighbour (up = towards 0); the rollback moves to the target.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ReorderArgs>, result_schema: schema_of::<Report>, handler: Handler::App(feature_reorder) },
    CommandSpec { name: "roll_to", group: "history", doc: "Execute the history up to and including feature `index`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<RollArgs>, result_schema: schema_of::<Report>, handler: Handler::App(roll_to) },
    CommandSpec { name: "undo", group: "history", doc: "Undo the last history edit and rerun.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Report>, handler: Handler::App(undo) },
    CommandSpec { name: "redo", group: "history", doc: "Redo the last undone history edit and rerun.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Report>, handler: Handler::App(redo) },
    CommandSpec { name: "history_listing", group: "history", doc: "The feature list `{step, features:[{index,type,id}]}`, the last run report, the rollback step, the feature selected in the tree (`selectedFeature`, null when none) and undo/redo availability.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(history_listing) },
    CommandSpec { name: "run_cancel", group: "history", doc: "Cancel the in-flight history run, if any.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(run_cancel) },
    CommandSpec { name: "next_feature_id", group: "history", doc: "Reserve the next unique feature id for a base (the feature's shortName), the way the add palette does.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NextIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(next_feature_id) },
    CommandSpec { name: "expressions_get", group: "history", doc: "The Expressions panel script, the variables it defines, and the configurator values.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(expressions_get) },
    CommandSpec { name: "expressions_set", group: "history", doc: "Replace the Expressions script and rerun the history.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ExpressionsArgs>, result_schema: schema_of::<Report>, handler: Handler::App(expressions_set) },
];
