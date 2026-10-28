//! A multi-axis finish's three-axis SOURCE (lane C17, the blade row): the
//! generator tilts an explicitly designated finishing operation's path and
//! never guesses one (`brep_cam::dispatch`). The designation is kept in the
//! document beside the operation (`CamDocument::multi_axis_sources`, like a
//! setup's orientation); the operation form shows it as a choice of the
//! eligible operations, `cam_update_operation {patch: {sourceOperation}}`
//! sets it, and the generation job carries the source's current result, or
//! its definition for the worker to generate first.

use std::hash::{Hash, Hasher};

use brep_cam::model::{Operation, Strategy};
use brep_cam::program::OperationInput;
use brep_render::engine_state::EngineState;
use brep_render::style::{FieldKind, FormField};
use serde_json::Value;

use crate::document::cam::CamDocument;

/// The form / patch key.
pub const FIELD: &str = "sourceOperation";

/// True for the strategies the generator accepts as a source: the
/// self-contained three-axis finishing ones (dispatch's list).
pub fn is_source_strategy(strategy: &Strategy) -> bool {
    matches!(
        strategy,
        Strategy::Parallel(_)
            | Strategy::Scallop(_)
            | Strategy::Pencil(_)
            | Strategy::Radial(_)
            | Strategy::Spiral(_)
            | Strategy::MorphedSpiral(_)
            | Strategy::Project(_)
            | Strategy::Morph(_)
            | Strategy::Contour3D(_)
            | Strategy::SteepShallow(_)
            | Strategy::Horizontal(_)
            | Strategy::Ramp(_)
    )
}

/// Whether `source` may feed the multi-axis finish `op`: a distinct enabled
/// three-axis finishing operation with the same setup, tool and targets.
pub fn eligible(op: &Operation, source: &Operation) -> bool {
    source.id != op.id && source.enabled && source.setup == op.setup && source.tool == op.tool && source.targets == op.targets && is_source_strategy(&source.strategy)
}

/// The operations that may feed `op_id`, in document order.
pub fn eligible_sources(doc: &CamDocument, op_id: &str) -> Vec<String> {
    let Some(op) = doc.operation(op_id) else { return Vec::new() };
    doc.operations.iter().filter(|s| eligible(op, s)).map(|s| s.id.to_string()).collect()
}

/// A multi-axis finish's extra form fields: its source, and what an
/// unreachable point does (`strategy.unreachable`: the model omits the
/// default, so the strategy walk cannot see it). None for other strategies.
pub fn form_fields(doc: &CamDocument, op: &Operation) -> Vec<FormField> {
    let mut fields: Vec<FormField> = form_field(doc, op).into_iter().collect();
    if matches!(op.strategy, Strategy::MultiAxisFinish(_)) {
        fields.push(FormField {
            path: vec!["strategy".to_string(), "unreachable".to_string()],
            label: "Unreachable points".to_string(),
            group: "Passes".to_string(),
            kind: FieldKind::Enum { variants: brep_cam::model::UnreachablePolicy::VARIANTS.iter().map(|v| v.to_string()).collect() },
        });
    }
    fields
}

/// The form's Source field for a multi-axis finish (none for other strategies).
pub fn form_field(doc: &CamDocument, op: &Operation) -> Option<FormField> {
    if !matches!(op.strategy, Strategy::MultiAxisFinish(_)) {
        return None;
    }
    let mut variants = vec![String::new()];
    variants.extend(eligible_sources(doc, op.id.as_str()));
    if let Some(current) = doc.multi_axis_source(op.id.as_str()) {
        if !variants.iter().any(|v| v == current) {
            variants.push(current.to_string());
        }
    }
    Some(FormField { path: vec![FIELD.to_string()], label: "Three-axis source operation".to_string(), group: "Passes".to_string(), kind: FieldKind::Enum { variants } })
}

/// Put the designation (and a multi-axis finish's unreachable policy) into
/// the form's params.
pub fn merge_params(doc: &CamDocument, op_id: &str, params: &mut Value) {
    if let Some(obj) = params.as_object_mut() {
        obj.insert(FIELD.to_string(), Value::String(doc.multi_axis_source(op_id).unwrap_or_default().to_string()));
    }
    if let Some(Strategy::MultiAxisFinish(p)) = doc.operation(op_id).map(|o| &o.strategy) {
        let word = serde_json::to_value(p.unreachable).unwrap_or(Value::Null);
        if let Some(strategy) = params.get_mut("strategy").and_then(Value::as_object_mut) {
            strategy.entry("unreachable").or_insert(word);
        }
    }
}

/// Apply `patch`'s `sourceOperation` (absent: nothing; empty or null: clear).
/// Refused by field name, applying nothing, when `op_id` is not a
/// multi-axis finish or the source is not eligible (the eligible ids listed).
pub fn apply_source(patch: &Value, doc: &mut CamDocument, op_id: &str) -> Result<(), String> {
    let Some(value) = patch.get(FIELD) else { return Ok(()) };
    let source = match value {
        Value::Null => None,
        Value::String(s) if s.trim().is_empty() => None,
        Value::String(s) => Some(s.trim().to_string()),
        other => return Err(format!("{FIELD}: expected an operation id, got {other}")),
    };
    let op = doc.operation(op_id).ok_or_else(|| format!("no operation `{op_id}`"))?;
    let unchanged = doc.multi_axis_source(op_id).map(str::to_string) == source;
    if !matches!(op.strategy, Strategy::MultiAxisFinish(_)) {
        if source.is_none() || unchanged {
            return Ok(());
        }
        return Err(format!("{FIELD}: only a multiAxisFinish operation takes a source (`{op_id}` is {})", op.strategy.kind_name()));
    }
    if let Some(id) = &source {
        let ok = doc.operation(id).map_or(false, |s| eligible(op, s));
        if !ok && !unchanged {
            let list = eligible_sources(doc, op_id);
            let list = if list.is_empty() { "none: add a three-axis finishing operation with the same setup, tool and targets".to_string() } else { list.join(", ") };
            return Err(format!("{FIELD}: `{id}` cannot feed `{op_id}` (a distinct enabled three-axis finishing operation with the same setup, tool and targets; eligible: {list})"));
        }
    }
    doc.set_multi_axis_source(op_id, source);
    Ok(())
}

/// What the generation job carries for `op`: `(source, source_operation)`.
/// Not a multi-axis finish: nothing. Otherwise the designation is required
/// and must still be eligible; a current result rides as the resolved
/// source path, else the definition does and the worker generates it.
pub fn job_source(state: &EngineState, doc: &CamDocument, op: &Operation) -> Result<(Option<OperationInput>, Option<Operation>), String> {
    if !matches!(op.strategy, Strategy::MultiAxisFinish(_)) {
        return Ok((None, None));
    }
    let op_id = op.id.as_str();
    let id = doc.multi_axis_source(op_id).ok_or_else(|| {
        let list = eligible_sources(doc, op_id);
        let hint = if list.is_empty() { "add a three-axis finishing operation with the same setup, tool and targets".to_string() } else { format!("eligible: {}", list.join(", ")) };
        format!("{op_id}: a multi-axis finish tilts a designated three-axis source operation: pick its source operation ({hint})")
    })?;
    let source = doc.operation(id).ok_or_else(|| format!("{op_id}: its source operation `{id}` is gone"))?;
    if !eligible(op, source) {
        return Err(format!("{op_id}: its source operation `{id}` no longer matches (a distinct enabled three-axis finishing operation with the same setup, tool and targets)"));
    }
    let current = matches!(crate::panels::cam_generate::status_of(state, doc, id), crate::panels::cam_generate::OpStatus::Ok { .. });
    let path = state.cam.results.get(id).and_then(|r| r.toolpath.clone()).filter(|_| current);
    match (path, doc.tool(source.tool.as_str())) {
        (Some(toolpath), Some(tool)) => Ok((Some(OperationInput { operation: source.clone(), tool: tool.clone(), toolpath }), None)),
        _ => Ok((None, Some(source.clone()))),
    }
}

/// Fold the designation and the source's DEFINITION (operation and tool)
/// into a finish's hash, so re-pointing it or editing the source makes the
/// finish stale. Not the source's full inputs hash: that folds in the
/// results of the operations before the source (C7's starting stock), which
/// include the finish itself when it comes first in the setup, and the two
/// would make each other stale forever.
pub fn hash_source(_state: &EngineState, doc: &CamDocument, op_id: &str, hasher: &mut impl Hasher) {
    match doc.multi_axis_source(op_id) {
        Some(id) => {
            id.hash(hasher);
            let source = doc.operation(id);
            serde_json::to_string(&source).unwrap_or_default().hash(hasher);
            serde_json::to_string(&source.and_then(|s| doc.tool(s.tool.as_str()))).unwrap_or_default().hash(hasher);
        }
        None => 0u8.hash(hasher),
    }
}

