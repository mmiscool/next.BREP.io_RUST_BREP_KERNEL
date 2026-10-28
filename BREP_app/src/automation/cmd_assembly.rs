//! The ASSEMBLY workbench: components, mate constraints, the BOM's attribute
//! columns, and the interference check.
//!
//! One command per operation the Assembly panels perform, calling the SAME
//! engine methods they do (`panels::assembly_components`,
//! `panels::assembly_constraints`, `panels::bom`, `panels::interference`), so
//! an agent and a person edit one assembly through one seam. Nothing here
//! enumerates constraint kinds: `assembly_constraint_catalogue` hands back the
//! kernel's own schema catalogue, which is what `assembly_add_constraint`
//! validates against.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddConstraintArgs {
    /// The constraint type id from `assembly_constraint_catalogue`, e.g. `coincident`.
    #[serde(rename = "type")]
    pub constraint_type: String,
    /// The constraint's `inputParams`; keys come from the catalogue entry's schema.
    #[serde(default)]
    pub params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateConstraintArgs {
    pub id: String,
    /// The constraint's complete new `inputParams`.
    pub params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ConstraintIdArgs {
    pub id: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ConstraintEnabledArgs {
    pub id: String,
    pub enabled: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ConstraintMoveArgs {
    pub id: String,
    /// The constraint's new position in the list.
    pub index: usize,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ComponentInsertArgs {
    /// A parts-library entry name (`assembly_parts_library` lists them).
    pub part_name: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ReplaceComponentArgs {
    /// The ACOMP feature id of the occurrence to replace, or a NESTED
    /// occurrence path (`ACOMP3:ACOMP1`) for a part inside a sub-assembly, or a
    /// Pattern Component instance (`APAT1.ACOMP2.i3`): its seed is replaced, and
    /// with it every instance (the report says so).
    pub id: String,
    /// Replace every occurrence that shows the same part (one undo step);
    /// for a nested path, every placement of it in that sub-assembly.
    #[serde(default, rename = "allInstances")]
    pub all_instances: bool,
    /// The replacement as a stored part document (its store name, as
    /// `component_insert`'s file flow names it). Give this or `partName`.
    #[serde(default)]
    pub source: Option<String>,
    /// The replacement as a parts-library entry the assembly already carries.
    #[serde(default, rename = "partName")]
    pub part_name: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ReplaceRepair {
    /// The constraint id.
    pub id: String,
    /// The unresolved reference (as the report lists it).
    pub from: String,
    /// The entity to use instead (a namespaced ref; equal to `from` confirms
    /// the same-name match).
    pub to: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ReplaceRepairArgs {
    pub repairs: Vec<ReplaceRepair>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ReplaceAcceptArgs {
    /// Constraint ids whose pre-ticked suggestions to accept (omit for all).
    #[serde(default)]
    pub ids: Vec<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ComponentFixedArgs {
    /// The owning ACOMP feature id.
    pub id: String,
    pub fixed: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ComponentIdArgs {
    /// The owning ACOMP feature id.
    pub id: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ComponentSelectArgs {
    /// ACOMP feature ids; the selection becomes exactly these components.
    pub ids: Vec<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PartAttributeArgs {
    /// A parts-library entry name.
    pub part_name: String,
    pub key: String,
    /// The value; `null` or `""` removes the key.
    pub value: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PartNameArgs {
    pub part_name: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OccurrenceAttributeArgs {
    /// The ACOMP feature ids to write; several ids is ONE undo step (the packed
    /// BOM row's fan-out).
    pub ids: Vec<String>,
    pub key: String,
    /// The value; `null` or `""` removes the key.
    pub value: Value,
}

/// The Auto Constraints knobs — every field optional, so `{}` runs every rule
/// at the kernel's default tolerances.
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InferConstraintsArgs {
    /// Which constraint types to infer (omit for every type the kernel offers —
    /// `assembly_inferable_types` lists them).
    #[serde(default)]
    pub types: Option<Vec<String>>,
    /// Gap tolerance in mm for "coplanar" / "collinear" / "same point"
    /// (default 0.001).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Angular tolerance in degrees for "parallel" (default 0.1).
    #[serde(default, rename = "angleToleranceDeg")]
    pub angle_tolerance_deg: Option<f64>,
}

impl InferConstraintsArgs {
    /// The kernel's `InferOptions` body, with unset fields left to its defaults.
    fn options(&self) -> String {
        let mut body = serde_json::Map::new();
        if let Some(types) = &self.types {
            body.insert("types".into(), json!(types));
        }
        if let Some(tolerance) = self.tolerance {
            body.insert("tolerance".into(), json!(tolerance));
        }
        if let Some(angle) = self.angle_tolerance_deg {
            body.insert("angleToleranceDeg".into(), json!(angle));
        }
        Value::Object(body).to_string()
    }
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateComponentsArgs {
    /// Refresh the outdated entries (the button). False only counts them.
    #[serde(default)]
    pub run: bool,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct ConstraintAdded {
    /// The id the kernel minted for the new constraint.
    pub id: String,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct ComponentAdded {
    /// The ACOMP feature id the insert appended.
    pub id: String,
}

/// Refuse before the kernel is asked, when the document has no assembly
/// session at all.
///
/// The kernel's assembly exports report their errors as a `JsValue`, which a
/// NATIVE build cannot construct: an `Err` out of one of them aborts the
/// process rather than returning. The panels never reach that path because
/// they only offer ids the list already holds — a command takes whatever it is
/// handed, so it has to do the same check itself. Every mutating command below
/// goes through this and [`require_constraint`].
fn require_assembly(ctx: &mut Ctx<'_>) -> Result<(), String> {
    if !ctx.app.docs.engine().history_has_assembly() {
        return Err("this document has no assembly: insert a component first (`component_insert`)".into());
    }
    Ok(())
}

/// Refuse an id the assembly does not hold, for the same reason.
fn require_constraint(ctx: &mut Ctx<'_>, id: &str) -> Result<(), String> {
    require_assembly(ctx)?;
    let state = ctx.app.docs.engine_mut().assembly_state_value();
    // A constraint entry carries its id at `inputParams.id` — that is what
    // `ConstraintEntry::id()` reads and the only place the serialized block has
    // one. Looking for a TOP-LEVEL `id` here matched nothing, so every command
    // behind this guard (remove / update / enable / move) refused every id it
    // was given. Verified against a build predating the runner-reply change, so
    // it is not a regression of it.
    let known = state["constraints"]
        .as_array()
        .into_iter()
        .flatten()
        .any(|c| c["inputParams"]["id"].as_str() == Some(id));
    if !known {
        return Err(format!("no assembly constraint `{id}` (see assembly_state)"));
    }
    Ok(())
}

fn assembly_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    Ok(Outcome::Done(json!({
        "hasAssembly": engine.history_has_assembly(),
        "modelRevision": engine.applied_generation(),
        "evaluationStatus": if engine.run_pending() { "pending" } else { "evaluated" },
        "state": engine.assembly_state_value(),
        "statuses": engine.assembly_statuses_value(),
        "dof": engine.assembly_dof_value(),
    })))
}

fn assembly_constraint_catalogue(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let _ = ctx;
    Ok(Outcome::Done(json!({ "constraints": brep_render::brep_kernel::constraint_schema_catalogue() })))
}

fn assembly_add_constraint(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddConstraintArgs = parse_args(args)?;
    require_assembly(ctx)?;
    let params = serde_json::to_string(&a.params).map_err(|e| e.to_string())?;
    let id = ctx.app.docs.engine_mut().assembly_add_constraint(&a.constraint_type, &params)?;
    serde_json::to_value(ConstraintAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn assembly_update_constraint(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateConstraintArgs = parse_args(args)?;
    require_constraint(ctx, &a.id)?;
    let params = serde_json::to_string(&a.params).map_err(|e| e.to_string())?;
    ctx.app.docs.engine_mut().assembly_update_constraint(&a.id, &params)?;
    Ok(Outcome::Done(json!({})))
}

fn assembly_remove_constraint(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ConstraintIdArgs = parse_args(args)?;
    require_constraint(ctx, &a.id)?;
    ctx.app.docs.engine_mut().assembly_remove_constraint(&a.id)?;
    Ok(Outcome::Done(json!({})))
}

fn assembly_set_constraint_enabled(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ConstraintEnabledArgs = parse_args(args)?;
    require_constraint(ctx, &a.id)?;
    ctx.app.docs.engine_mut().assembly_set_constraint_enabled(&a.id, a.enabled)?;
    Ok(Outcome::Done(json!({})))
}

fn assembly_move_constraint(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ConstraintMoveArgs = parse_args(args)?;
    require_constraint(ctx, &a.id)?;
    ctx.app.docs.engine_mut().assembly_move_constraint(&a.id, a.index)?;
    Ok(Outcome::Done(json!({})))
}

fn assembly_solve(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    require_assembly(ctx)?;
    ctx.app.docs.engine_mut().assembly_run_solve()?;
    let engine = ctx.app.docs.engine_mut();
    Ok(Outcome::Done(json!({ "statuses": engine.assembly_statuses_value(), "dof": engine.assembly_dof_value() })))
}

fn assembly_parts_library(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({ "parts": ctx.app.docs.engine_mut().parts_library_names() })))
}

fn component_insert(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ComponentInsertArgs = parse_args(args)?;
    let id = ctx
        .app
        .docs
        .engine_mut()
        .insert_component(brep_render::engine_state::ComponentInsert::Existing { part_name: &a.part_name })?;
    serde_json::to_value(ComponentAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn assembly_replace_component(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ReplaceComponentArgs = parse_args(args)?;
    // A Work part edited in context is finished first, as the menu does.
    let (_, docs, _) = ctx.app.file_docs_store();
    crate::panels::replace_component::finish_work_part_for_replace(docs)?;
    require_assembly(ctx)?;
    let source = match (a.source, a.part_name) {
        (Some(source), None) => crate::panels::replace_component::ReplaceSource::Stored(source),
        (None, Some(part)) => crate::panels::replace_component::ReplaceSource::Library(part),
        _ => return Err("give exactly one of `source` (a stored part) or `partName` (a library entry)".into()),
    };
    let (_, docs, store) = ctx.app.file_docs_store();
    let engine = docs.engine_mut();
    if a.id.contains(':') {
        // A part inside a sub-assembly, by its occurrence path.
        let part = crate::panels::replace_component::replace_nested(engine, store, &a.id, &source, a.all_instances)?;
        return Ok(Outcome::Done(json!({ "partName": part, "components": [a.id], "report": engine.replace_report() })));
    }
    // A Pattern Component instance stands for its seed (the engine replaces
    // the seed and notes why): every instance of the SEED's part.
    let ids = if a.all_instances {
        let seed = brep_render::brep_kernel::pattern_component_instance(&a.id).map_or(a.id.clone(), |(_, seed, _)| seed);
        let mut ids = crate::panels::replace_component::instances_of(engine, &seed);
        if seed != a.id {
            ids.insert(0, a.id.clone());
        }
        ids
    } else {
        vec![a.id.clone()]
    };
    let part = crate::panels::replace_component::replace(engine, store, &ids, &source)?;
    Ok(Outcome::Done(json!({ "partName": part, "components": ids, "report": engine.replace_report() })))
}

fn assembly_replace_report(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    Ok(Outcome::Done(json!({
        "report": engine.replace_report(),
        "unresolved": crate::panels::replace_component::held_mates(engine),
    })))
}

fn assembly_replace_repair(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ReplaceRepairArgs = parse_args(args)?;
    require_assembly(ctx)?;
    let repairs: Vec<(String, String, String)> = a.repairs.into_iter().map(|r| (r.id, r.from, r.to)).collect();
    let engine = ctx.app.docs.engine_mut();
    crate::panels::replace_component::repair(engine, &repairs)?;
    Ok(Outcome::Done(json!({ "unresolved": crate::panels::replace_component::held_mates(engine) })))
}

fn assembly_replace_accept(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ReplaceAcceptArgs = parse_args(args)?;
    require_assembly(ctx)?;
    let engine = ctx.app.docs.engine_mut();
    let accepted = crate::panels::replace_component::accept_suggestions(engine, &a.ids)?;
    Ok(Outcome::Done(json!({ "accepted": accepted, "unresolved": crate::panels::replace_component::held_mates(engine) })))
}

fn assembly_replace_delete(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ConstraintIdArgs = parse_args(args)?;
    require_assembly(ctx)?;
    let engine = ctx.app.docs.engine_mut();
    crate::panels::replace_component::delete_held(engine, &a.id)?;
    Ok(Outcome::Done(json!({ "unresolved": crate::panels::replace_component::held_mates(engine) })))
}

fn component_info(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ComponentIdArgs = parse_args(args)?;
    let info = ctx
        .app
        .docs
        .engine()
        .component_info(&a.id)
        .ok_or_else(|| format!("no component feature '{}'", a.id))?;
    let engine = ctx.app.docs.engine();
    let snapshot = (!engine.run_pending()).then(|| engine.assembly_components().iter().find(|c| c.id == a.id)).flatten();
    let pose = snapshot.map(|c| brep_render::brep_kernel::transform_to_pose_params(&c.affine()));
    let report: Value = serde_json::from_str(&engine.history_report_json()).unwrap_or_default();
    let authored = engine.history.index_of(&a.id).and_then(|i| engine.history.feature_params(i)).map(|p| p["transform"].clone());
    Ok(Outcome::Done(json!({
        "id": info.id,
        "partName": info.part_name,
        "translate": pose.as_ref().map(|p| &p["translate"]),
        "rotateEulerDeg": pose.as_ref().map(|p| &p["rotateEulerDeg"]),
        "placement": {
            "status": if engine.run_pending() { "pending" } else if snapshot.is_some() { "evaluated" } else { "not_evaluated" },
            "modelRevision":engine.applied_generation(),
            "authored": snapshot.map(|c| c.authored_transform.clone()).or(authored),
            "requestedMatrix": snapshot.and_then(|c| c.requested_transform),
            "evaluatedMatrix": snapshot.map(|c| c.transform),
            "solverAdjusted": snapshot.and_then(|c| c.requested_transform.map(|r| r.iter().zip(c.transform).any(|(a,b)| (a-b).abs() > 1e-10))),
            "frame": "assembly", "lengthUnit": "mm", "angleUnit": "degree", "matrixLayout": "row_major",
            "errors": report["featureErrors"].as_array().into_iter().flatten().filter(|e| e.as_str().is_some_and(|e| e.starts_with(&format!("{}:", a.id)))).collect::<Vec<_>>(),
        },
        "fixed": info.fixed,
        "members": info.members,
        "attributes": engine.occurrence_attributes(&a.id),
        "displayName": engine.occurrence_attributes(&a.id).get("Name").cloned().unwrap_or(json!(info.part_name)),
    })))
}

fn component_set_fixed(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ComponentFixedArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().set_component_fixed(&a.id, a.fixed)?;
    Ok(Outcome::Done(json!({})))
}

fn component_select(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ComponentSelectArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().select_components(&a.ids);
    Ok(Outcome::Done(json!({ "selected": a.ids })))
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InterferenceArgs {
    pub components: Option<Vec<String>>,
    /// Maximum component pairs per page, from 1 to 4096 (default 4096).
    pub budget: Option<usize>,
    pub continuation_token: Option<String>,
    /// Near-miss threshold in mm (default 0 = off): non-interfering pairs
    /// within it report `minDistance` (display tessellation) and `nearMiss`.
    pub clearance_mm: Option<f64>,
}

fn interference_check(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: InterferenceArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().interference_page(a.components, a.budget.unwrap_or(4096), a.continuation_token, a.clearance_mm).map(Outcome::Done)
}

fn assembly_inferable_types(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({
        "types": ctx.app.docs.engine().assembly_inferable_types(),
    })))
}

fn assembly_infer_constraints(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: InferConstraintsArgs = parse_args(args)?;
    Ok(Outcome::Done(
        ctx.app.docs.engine_mut().assembly_infer_constraints(&a.options()),
    ))
}

fn assembly_apply_inferred_constraints(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: InferConstraintsArgs = parse_args(args)?;
    Ok(Outcome::Done(
        ctx.app
            .docs
            .engine_mut()
            .assembly_apply_inferred_constraints(&a.options()),
    ))
}

fn component_update(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateComponentsArgs = parse_args(args)?;
    let (outdated, missing, refreshed) = ctx.app.update_components(a.run)?;
    Ok(Outcome::Done(json!({ "outdated": outdated, "missing": missing, "refreshed": refreshed })))
}

fn bom_part_attributes(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PartNameArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({ "attributes": ctx.app.docs.engine().part_attributes(&a.part_name) })))
}

fn bom_set_part_attribute(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PartAttributeArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().set_part_attribute(&a.part_name, &a.key, a.value)?;
    Ok(Outcome::Done(json!({})))
}

fn bom_occurrence_attributes(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ComponentIdArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({ "attributes": ctx.app.docs.engine().occurrence_attributes(&a.id) })))
}

fn bom_set_occurrence_attribute(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: OccurrenceAttributeArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().set_occurrence_attribute(&a.ids, &a.key, a.value)?;
    Ok(Outcome::Done(json!({})))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "assembly_state", group: "assembly", doc: "The assembly: component instances, constraints, per-constraint solve statuses and the remaining degrees of freedom. `hasAssembly` is false for a document with no components.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_state) },
    CommandSpec { name: "assembly_constraint_catalogue", group: "assembly", doc: "Every mate constraint the kernel offers, with its parameter schema — the types `assembly_add_constraint` takes.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_constraint_catalogue) },
    CommandSpec { name: "assembly_add_constraint", group: "assembly", doc: "Add a mate constraint and (when auto-solve is on) re-solve. Returns the minted constraint id.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AddConstraintArgs>, result_schema: schema_of::<ConstraintAdded>, handler: Handler::App(assembly_add_constraint) },
    CommandSpec { name: "assembly_update_constraint", group: "assembly", doc: "Replace a constraint's inputParams and re-solve.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateConstraintArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_update_constraint) },
    CommandSpec { name: "assembly_remove_constraint", group: "assembly", doc: "Delete a constraint and re-solve.", phase: Phase::Mutate, annotations: Annotations::DESTRUCTIVE, args_schema: schema_of::<ConstraintIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_remove_constraint) },
    CommandSpec { name: "assembly_set_constraint_enabled", group: "assembly", doc: "Suppress or re-enable one constraint and re-solve.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ConstraintEnabledArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_set_constraint_enabled) },
    CommandSpec { name: "assembly_move_constraint", group: "assembly", doc: "Move a constraint to a new position in the list and re-solve (solve order is list order).", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ConstraintMoveArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_move_constraint) },
    CommandSpec { name: "assembly_solve", group: "assembly", doc: "Run the constraint solver now (the manual Solve the panel offers when auto-solve is off) and return the statuses and remaining DOF.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_solve) },
    CommandSpec { name: "assembly_parts_library", group: "assembly", doc: "The document's parts-library entry names — what `component_insert` places.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_parts_library) },
    CommandSpec { name: "component_insert", group: "assembly", doc: "Place another instance of a parts-library entry: appends an ACOMP feature with an identity transform and runs it. The document's FIRST component is grounded. Returns the feature id; move it with `feature_set_params` on its `transform`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ComponentInsertArgs>, result_schema: schema_of::<ComponentAdded>, handler: Handler::App(component_insert) },
    CommandSpec { name: "assembly_replace_component", group: "assembly", doc: "Replace the part an occurrence shows (or, with `allInstances`, every occurrence of that part) with a stored part (`source`) or a library entry (`partName`), keeping the occurrence's id, pose and attributes. One undo step. Mates are re-resolved by face name only when the parts share lineage (same family, or one saved from the other); otherwise, or when a name is missing or changed type, the mate is kept DISABLED with a `replace` marker and a suggestion. `report` is null until the run lands: read it with `assembly_replace_report`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ReplaceComponentArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_replace_component) },
    CommandSpec { name: "assembly_replace_report", group: "assembly", doc: "The last replace's report (`{components, from, to, lineage, kept, unresolved, notes}`) and every mate still carrying a replace marker at ANY depth (`unresolved`; a sub-assembly's mate is listed as `<occurrence path>:<id>` with namespaced refs; markers survive save/reopen).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_replace_report) },
    CommandSpec { name: "assembly_replace_repair", group: "assembly", doc: "Point held mate references at picked entities, one undo step, at any depth (ids and refs as `assembly_replace_report` lists them; a sub-assembly's mate takes a ref inside that sub-assembly). A mate with nothing left held is re-enabled (if it was enabled before the replace) and loses its marker. Delete one with `assembly_replace_delete`.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ReplaceRepairArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_replace_repair) },
    CommandSpec { name: "assembly_replace_delete", group: "assembly", doc: "Delete a mate a replace is holding, at any depth: a top-level id (`CONC1`) or a sub-assembly's mate by its occurrence path (`ACOMP1:CONC1`, which edits that sub-assembly's definition). One undo step.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ConstraintIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_replace_delete) },
    CommandSpec { name: "assembly_replace_accept", group: "assembly", doc: "Accept the PRE-TICKED suggestions (the entity whose geometry is the old one's) of the given held mates, or of every held mate at every depth, as one undo step.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ReplaceAcceptArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_replace_accept) },
    CommandSpec { name: "component_info", group: "assembly", doc: "One component instance with authored expressions, requested and solver-adjusted evaluated placement matrices, frame/units, evaluation status, attributes and member names. Numeric legacy fields describe evaluated geometry; failed or pending placement is null.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<ComponentIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(component_info) },
    CommandSpec { name: "component_set_fixed", group: "assembly", doc: "Ground or unground a component (its ACOMP `isFixed`) and re-run.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ComponentFixedArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(component_set_fixed) },
    CommandSpec { name: "component_select", group: "assembly", doc: "Select exactly these component instances (their member solids) — the selection a constraint's reference pick reads.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<ComponentSelectArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(component_select) },
    CommandSpec { name: "interference_check", group: "assembly", doc: "Check component pairs with an explicit pair budget and revision-bound continuation; clearance_mm adds a near-miss gap check (nearMisses, minDistance). Only allClear=true establishes complete clearance; zero-volume contact candidates remain unverified without a distance witness.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<InterferenceArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(interference_check) },
    CommandSpec { name: "assembly_inferable_types", group: "assembly", doc: "The constraint types automatic inference can produce, each with what placement it reads as that constraint — the rows the Auto Constraints dialog lists.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_inferable_types) },
    CommandSpec { name: "assembly_infer_constraints", group: "assembly", doc: "Read the components' current placement as constraints WITHOUT creating any: the candidates (type, elements, the measurement each was read from) plus what was skipped. The look-before-you-leap half of Auto Constraints — an imported STEP assembly is fully posed and completely unconstrained, and this says what would hold it there.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<InferConstraintsArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_infer_constraints) },
    CommandSpec { name: "assembly_apply_inferred_constraints", group: "assembly", doc: "Create every constraint `assembly_infer_constraints` accepts and solve the batch as ONE undo step. Pairs that already carry a constraint are left alone, so running it twice creates nothing the second time.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<InferConstraintsArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(assembly_apply_inferred_constraints) },
    CommandSpec { name: "component_update", group: "assembly", doc: "Check the parts library against the saved source documents: how many entries are outdated and which have no source. `run: true` refreshes the outdated ones and re-runs (the Constraints header's Update components button); entries with no source are skipped, never fatal.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateComponentsArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(component_update) },
    CommandSpec { name: "bom_part_attributes", group: "assembly", doc: "One parts-library entry's BOM attributes (the `part.` columns).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<PartNameArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(bom_part_attributes) },
    CommandSpec { name: "bom_set_part_attribute", group: "assembly", doc: "Write one BOM attribute on a PART — shared by every occurrence of it. `null` removes the key.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<PartAttributeArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(bom_set_part_attribute) },
    CommandSpec { name: "bom_occurrence_attributes", group: "assembly", doc: "One component occurrence's BOM attributes (the `occurrence.` columns).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<ComponentIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(bom_occurrence_attributes) },
    CommandSpec { name: "bom_set_occurrence_attribute", group: "assembly", doc: "Write one BOM attribute across component OCCURRENCES — one undo step however many ids. `null` removes the key.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<OccurrenceAttributeArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(bom_set_occurrence_attribute) },
];
