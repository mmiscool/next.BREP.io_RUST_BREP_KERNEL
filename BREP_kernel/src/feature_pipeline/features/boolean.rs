//! B — Boolean. The standalone Boolean feature (no base primitive of its own):
//! it resolves already-resident operands BY NAME from the scene-map and applies a
//! boolean between a `targetSolid` and a set of tool solids.
//!
//! Semantics:
//!
//! - `inputParams.targetSolid` — a `reference_selection` (a solid NAME, or a
//!   one-element array of names).
//! - `inputParams.boolean = { operation, targets, mergeCoplanarFaces }` — the
//!   operation and the OTHER solids (tools) to combine with the target.
//!
//! Dispatch:
//! - `operation == NONE` or no tools → no-op.
//! - **UNION / INTERSECT**: base = target, fold each tool in against the target.
//!   The result takes the FIRST TOOL's name (the boolean names its result after
//!   the first target, and here the targets are the tools); `removed = [target,
//!   ...tools]`.
//! - **SUBTRACT**: pre-union the tools (a single tool is used directly), then
//!   subtract that union from the target. The result takes the TARGET's name (the
//!   target is the sole boolean target); `removed = [...tools,
//!   target]` (the tool-union is a non-scene intermediate, dropped).
//!
//! Name fidelity (contract rule 2): FACE names propagate through
//! `boolean_operation` automatically (`operand_face_names`) and stand. The
//! derived EDGE names are re-stamped from the RESULT's own adjacency by
//! `common::register_added`, exactly as every other feature registers its
//! output.
//!
//! They have to be. `boolean_operation` names each new intersection edge after
//! the two faces that SUPPORTED it at imprint time, which is not the pair it
//! ends up between: a tool cap coplanar with a target face names the cut's rim
//! after a face the result does not contain (`Box_NY|Pin_B`), and a target face
//! the cut SPLIT lends its pre-split name to both fragments' edges at once
//! (`Box_PX|Pin_S` twice, so the scene map keeps one and a pick of the other
//! silently resolves to it). Neither name survives the next feature either —
//! anything that runs after this one re-stamps the same edges — so leaving them
//! made an edge's identity depend on where the timeline was cut.
//!
//! The result solid name is the established convention above; the loop reuses a
//! removed operand's name for the result (`SceneMap::apply` removes-then-adds).

use std::collections::HashSet;

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult};
use crate::{BooleanOperation, BooleanOptions, BrepSolid};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

/// The `boolean` param (`{ operation, targets, mergeCoplanarFaces }`), parsed the
/// same way `common::read_boolean_param` parses it for the primitive fold.
struct BooleanParam {
    operation: String,
    targets: Vec<String>,
    merge_coplanar_faces: bool,
}


/// `inputParams.targetSolid` → the primary target name (array → first entry).
fn read_target_name(ctx: &FeatureContext) -> Option<String> {
    match ctx.param("targetSolid")? {
        serde_json::Value::Array(items) => items.iter().find_map(common::reference_name),
        other => common::reference_name(other),
    }
}

fn read_boolean_param(ctx: &FeatureContext) -> BooleanParam {
    let boolean = ctx.param("boolean");
    let operation = boolean
        .and_then(|value| value.get("operation"))
        .and_then(|value| value.as_str())
        .unwrap_or("NONE")
        .to_uppercase();
    let targets = boolean
        .and_then(|value| value.get("targets"))
        .and_then(|value| value.as_array())
        .map(|array| array.iter().filter_map(common::reference_name).collect())
        .unwrap_or_default();
    let merge_coplanar_faces = boolean
        .and_then(|value| value.get("mergeCoplanarFaces"))
        .and_then(|value| value.as_bool())
        .unwrap_or(true);
    BooleanParam {
        operation,
        targets,
        merge_coplanar_faces,
    }
}

/// Fold the boolean over resolved targets: start from `base`
/// and, for each `(name, handle)`, run `result = op(target, result)` — the array
/// entry is the FIRST operand, the running result the SECOND (`x.op(y)` =
/// `boolean_operation(x, y, op)`). Registers only the running-result operand for the
/// two-solid borrow and frees it each step — no leaked intermediates.
fn fold_boolean(
    base: BrepSolid,
    targets: &[(String, u32)],
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<BrepSolid, FeatureRefusal> {
    let mut current = base;
    for (name, handle) in targets {
        let operand = crate::register_solid_value(current);
        let folded = crate::with_two_registered_solids(*handle, operand, |target, running| {
            crate::boolean_operation(target, running, operation, options)
        });
        crate::free_registered_solid(operand);
        // The boolean's class rides through the feature's wrap: a degeneracy
        // refused by the kernel is a degeneracy at the feature boundary.
        current = folded.map_err(|error: crate::KernelRefusal| {
            error.with_message(|error| format!("boolean {operation:?} on '{name}' failed: {error}"))
        })?;
    }
    Ok(current)
}

/// Union many tools into one: seed = clone(tools[0]);
/// then `result = boolean_operation(result, tools[i], Union)` for i ≥ 1 (running
/// result FIRST, next tool SECOND — the opposite operand order from
/// [`fold_boolean`]). The tool-union ALWAYS merges coplanar faces: the user's
/// `mergeCoplanarFaces` flag does not reach this union (it passes only
/// `{featureID, owningFeatureID}`), so [`BooleanOptions::default`]
/// (merge = true) is used here. Only called with ≥ 2 tools.
fn union_tools(tools: &[(String, u32)]) -> Result<BrepSolid, FeatureRefusal> {
    let options = BooleanOptions::default();
    let mut current = crate::with_registered_solid_str(tools[0].1, |solid| Ok(solid.clone()))?;
    for (name, handle) in &tools[1..] {
        let running = crate::register_solid_value(current);
        let folded = crate::with_two_registered_solids(running, *handle, |accumulated, tool| {
            crate::boolean_operation(accumulated, tool, BooleanOperation::Union, &options)
        });
        crate::free_registered_solid(running);
        current = folded.map_err(|error: crate::KernelRefusal| {
            error.with_message(|error| format!("tool union at '{name}' failed: {error}"))
        })?;
    }
    Ok(current)
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    let boolean = read_boolean_param(ctx);
    // No-op: NONE / no tools leaves the scene unchanged.
    if boolean.operation == "NONE" || boolean.targets.is_empty() {
        return Ok(result);
    }
    let operation = common::parse_boolean_operation(&boolean.operation)?;

    // Resolve target + every tool BY NAME against the live scene-map, accumulating
    // ALL misses into `unresolved` (contract rule 1: a miss is a structured entry
    // the caller repairs + re-dispatches, never a panic). One pass gives the caller the complete
    // repair list before we decide whether the op can run.
    let target = match read_target_name(ctx) {
        Some(name) => match ctx.scene.resolve_solid(&name) {
            Some(handle) => Some((name, handle)),
            None => {
                result.unresolved.push(name);
                None
            }
        },
        None => None,
    };

    // Order-preserving dedup of tool names via a `seen` set.
    let mut tools: Vec<(String, u32)> = Vec::new();
    let mut seen: HashSet<String> = HashSet::new();
    for name in &boolean.targets {
        if !seen.insert(name.clone()) {
            continue;
        }
        match ctx.scene.resolve_solid(name) {
            Some(handle) => tools.push((name.clone(), handle)),
            None => result.unresolved.push(name.clone()),
        }
    }

    // Need a resolved target AND at least one resolved tool to proceed; otherwise
    // no-op with `unresolved` populated (the caller repairs + re-dispatches).
    let Some((target_name, target_handle)) = target else {
        return Ok(result);
    };
    if tools.is_empty() {
        return Ok(result);
    }

    // Only the final subtract / fold honors the user's mergeCoplanarFaces.
    let options = BooleanOptions {
        merge_coplanar_faces: boolean.merge_coplanar_faces,
        ..BooleanOptions::default()
    };

    let (solid, result_name, removed): (BrepSolid, String, Vec<String>) = match operation {
        BooleanOperation::Subtract => {
            // Subtract the (unioned) tools from the target: `target - toolUnion` =
            // `boolean_operation(target, toolUnion, Subtract)`.
            let solid = if tools.len() == 1 {
                // Single tool: subtract it directly.
                crate::with_two_registered_solids(target_handle, tools[0].1, |target, tool| {
                    crate::boolean_operation(target, tool, BooleanOperation::Subtract, &options)
                })
                .map_err(|error: crate::KernelRefusal| {
                    error.with_message(|error| format!("boolean SUBTRACT failed: {error}"))
                })?
            } else {
                let union = union_tools(&tools)?;
                let union_handle = crate::register_solid_value(union);
                let folded = crate::with_two_registered_solids(target_handle, union_handle, |target, tool_union| {
                    crate::boolean_operation(target, tool_union, BooleanOperation::Subtract, &options)
                });
                crate::free_registered_solid(union_handle);
                folded.map_err(|error: crate::KernelRefusal| {
                    error.with_message(|error| format!("boolean SUBTRACT failed: {error}"))
                })?
            };
            // The target is the sole boolean target → result takes the target's name;
            // removed = [...tools, target] (the tool-union
            // is a non-scene intermediate, dropped — removal is by scene name).
            let mut removed: Vec<String> = tools.iter().map(|(name, _)| name.clone()).collect();
            removed.push(target_name.clone());
            (solid, target_name, removed)
        }
        _ => {
            // UNION / INTERSECT: base = target, fold each tool in.
            let base = crate::with_registered_solid_str(target_handle, |solid| Ok(solid.clone()))?;
            let solid = fold_boolean(base, &tools, operation, &options)?;
            // The targets are the tools → result takes the FIRST TOOL's name;
            // removed = [target, ...tools]
            // (the duplicate target is a removal no-op).
            let result_name = tools[0].0.clone();
            let mut removed = vec![target_name];
            removed.extend(tools.iter().map(|(name, _)| name.clone()));
            (solid, result_name, removed)
        }
    };

    // Face names propagated by `boolean_operation` stand; the derived EDGE names
    // are re-stamped from the RESULT's adjacency like every other feature's
    // (`common::register_added`) — see the module note on name fidelity.
    result.added.push(common::register_added(solid, &result_name));
    result.removed = removed;
    Ok(result)
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected solid drives `targetSolid`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.solids > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "B",
    "shortName": "B",
    "longName": "Boolean",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the boolean feature"
        },
        "targetSolid": {
            "type": "reference_selection",
            "selectionFilter": [
                "SOLID"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Primary target solid"
        },
        "boolean": {
            "type": "boolean_operation",
            "default_value": {
                "targets": [],
                "operation": "UNION",
                "mergeCoplanarFaces": true
            },
            "hint": "Operation + other solids (as tools)"
        }
    }
})
}

