//! The constraint run lifecycle: validate → map → ONE `solve_assembly`
//! call → status/diagnostics merge → pose write-back.

use std::collections::BTreeMap;

use super::mapping::{self, ConstraintFailure, MappedConstraint, ResolvedElement};
use super::{constraints, AssemblyState};
use crate::feature_pipeline::component::update_component_transform;
use crate::feature_pipeline::{Env, SceneMap};
use crate::{
    solve_assembly, AssemblyBody, AssemblyMate, AssemblySolveOptions,
};

/// Satisfied-vs-adjusted gate, relative to the constraint pair's coordinate
/// scale. Distances are model units; direction residuals are dimensionless
/// but bounded by the same gate (the LM converges orders below it).
const SATISFIED_REL_TOL: f64 = 1e-6;
/// The both-fixed validity check runs a zero-unknown solve at this (looser)
/// relative tolerance — grounded geometry only has to TOUCH, not converge.
const FIXED_CHECK_TOLERANCE: f64 = 1e-6;

/// What one constraint run produced beyond the merged `persistentData`.
#[derive(Debug, Default)]
pub struct LifecycleOutcome {
    /// Component id → the solved `inputParams.transform` JSON (the generic
    /// pose write-back for the owning ACOMP feature).
    pub pose_updates: BTreeMap<String, serde_json::Value>,
    /// Component id → grounded flag write-back (`inputParams.isFixed`), from
    /// Fixed constraints.
    pub fixed_updates: BTreeMap<String, bool>,
    /// Scene solids whose geometry MOVED in the write-back (`(name, handle)`)
    /// — the display lane must re-tessellate these even when their producing
    /// feature replayed clean this run.
    pub moved_solids: Vec<(String, u32)>,
    /// The whole-solve report (`ok`, solver diagnostics or `error`).
    pub report: serde_json::Value,
}

impl LifecycleOutcome {
    pub fn empty() -> Self {
        Self {
            report: serde_json::json!({ "ok": true, "mates": 0 }),
            ..Self::default()
        }
    }
}

/// Which lane a constraint landed in after validate+map.
enum Lane {
    /// Terminal status already written (disabled/unknown/duplicate/failure/
    /// fixed-type/both-fixed).
    Settled,
    /// Mapped into the solve: indexes into the global mate list.
    Solved {
        mates: Vec<usize>,
        mapped: MappedConstraint,
        scale: f64,
    },
}

/// Run the whole lifecycle over `state` against the live scene. Every entry
/// gets a status; the scene's components are re-posed in place on success.
pub fn run_constraints(
    state: &mut AssemblyState,
    scene: &mut SceneMap,
    env: &Env,
) -> LifecycleOutcome {
    let mut outcome = LifecycleOutcome::default();

    // --- Validate: disabled / unknown / incomplete / duplicates -----------
    let mut duplicate_of: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    for (index, entry) in state.constraints.iter().enumerate() {
        if !entry.enabled {
            continue;
        }
        let Some(def) = constraints::constraint_type(&entry.constraint_type) else {
            continue;
        };
        if def.duplicate_family {
            let elements = entry.elements();
            if (def.min_elements..=def.max_elements).contains(&elements.len()) {
                duplicate_of
                    .entry(mapping::pair_signature(&elements))
                    .or_default()
                    .push(index);
            }
        }
    }
    duplicate_of.retain(|_, members| members.len() > 1);

    let mut lanes: Vec<Lane> = Vec::with_capacity(state.constraints.len());
    let mut mates: Vec<AssemblyMate> = Vec::new();
    let mut mate_sides: Vec<(String, String)> = Vec::new();
    let mut bodies: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();

    // Fixed constraints ground components FIRST, so every later fixed-flag
    // read (body building, both-fixed checks) sees them.
    for entry in &state.constraints {
        if entry.enabled && entry.constraint_type == "fixed" {
            let elements = entry.elements();
            if elements.len() != 1 {
                continue; // marked incomplete in the main pass below
            }
            if let Ok(resolved) = mapping::resolve_element(scene, &elements[0]) {
                scene.set_component_fixed(&resolved.component, true);
                outcome.fixed_updates.insert(resolved.component, true);
            } // a resolve failure writes its status in the main pass
        }
    }

    // --- Per-entry validate + resolve + map --------------------------------
    for index in 0..state.constraints.len() {
        let entry = &mut state.constraints[index];
        if !entry.enabled {
            entry.set_status("disabled", "Constraint disabled.", false);
            lanes.push(Lane::Settled);
            continue;
        }
        let Some(def) = constraints::constraint_type(&entry.constraint_type) else {
            entry.set_status(
                "error",
                format!("Unknown constraint type: {}", entry.constraint_type),
                false,
            );
            lanes.push(Lane::Settled);
            continue;
        };
        let elements = entry.elements();
        if !(def.min_elements..=def.max_elements).contains(&elements.len()) {
            let wanted = match (def.min_elements, def.max_elements) {
                (1, 1) => "1 element".to_string(),
                (min, max) if min == max => format!("{min} elements"),
                (min, max) if max == min + 1 => format!("{min} or {max} elements"),
                (min, max) => format!("{min} to {max} elements"),
            };
            entry.set_status("incomplete", format!("Select {wanted}."), false);
            lanes.push(Lane::Settled);
            continue;
        }

        // Duplicate marking: every member of an overlapping-family
        // signature group is marked and skipped.
        if def.duplicate_family {
            let signature = mapping::pair_signature(&elements);
            if let Some(members) = duplicate_of.get(&signature) {
                let others: Vec<(String, String)> = members
                    .iter()
                    .filter(|&&other| other != index)
                    .map(|&other| {
                        let peer = &state.constraints[other];
                        let label = constraints::constraint_type(&peer.constraint_type)
                            .map(|def| def.label)
                            .unwrap_or(peer.constraint_type.as_str());
                        (label.to_string(), peer.id().to_string())
                    })
                    .collect();
                let entry = &mut state.constraints[index];
                let mut message = String::from("Duplicate constraint selections:");
                for (label, id) in &others {
                    message.push_str(&format!(" conflicts with {label} constraint {id}."));
                }
                entry.set_status("duplicate", message, false);
                entry.set_persistent(
                    "duplicateConstraintIDs",
                    serde_json::Value::Array(
                        others
                            .iter()
                            .map(|(_, id)| serde_json::Value::String(id.clone()))
                            .collect(),
                    ),
                );
                entry.set_persistent(
                    "duplicateSignature",
                    serde_json::Value::String(signature),
                );
                lanes.push(Lane::Settled);
                continue;
            }
        }

        // Fixed: grounded above — settle the status.
        if entry.constraint_type == "fixed" {
            let entry = &mut state.constraints[index];
            match mapping::resolve_element(scene, &elements[0]) {
                Ok(resolved) => {
                    entry.set_status(
                        "satisfied",
                        format!("Component {} is fixed.", resolved.component),
                        true,
                    );
                }
                Err(failure) => entry.set_status(failure.status, failure.message, false),
            }
            lanes.push(Lane::Settled);
            continue;
        }

        // Resolve every element; typed failures settle the constraint.
        let resolved: Result<Vec<ResolvedElement>, ConstraintFailure> = elements
            .iter()
            .map(|name| mapping::resolve_element(scene, name))
            .collect();
        let entry = &mut state.constraints[index];
        let resolved = match resolved {
            Ok(resolved) => resolved,
            Err(failure) => {
                entry.set_status(failure.status, failure.message, false);
                lanes.push(Lane::Settled);
                continue;
            }
        };
        // A constraint whose every element sits on ONE rigid body cannot move
        // anything (the per-type mapper decides how MANY bodies it wants
        // beyond that — center insists on exactly two).
        let participants: std::collections::BTreeSet<&str> = resolved
            .iter()
            .map(|element| element.component.as_str())
            .collect();
        if participants.len() < 2 {
            let component = resolved
                .first()
                .map(|element| element.component.as_str())
                .unwrap_or("");
            entry.set_status(
                "invalid-selection",
                format!(
                    "{} belong to component '{component}' — constraints act between two components.",
                    if resolved.len() == 2 { "Both selections" } else { "All the selections" }
                ),
                false,
            );
            lanes.push(Lane::Settled);
            continue;
        }

        let mapped = match mapping::map_constraint(entry, &resolved, env) {
            Ok(mapped) => mapped,
            Err(failure) => {
                entry.set_status(failure.status, failure.message, false);
                lanes.push(Lane::Settled);
                continue;
            }
        };
        let scale = mapping::elements_scale(&resolved);

        // Every participant grounded: validate in place (a zero-unknown solve
        // evaluates the exact mate residuals) — satisfied or blocked, never
        // fed to the main solve (an unsatisfiable grounded mate would fail
        // the whole pass).
        let all_fixed = participants
            .iter()
            .all(|id| scene.component_fixed(id).unwrap_or(false));
        if all_fixed {
            let entry = &mut state.constraints[index];
            let check = check_fixed_pair(scene, &mapped);
            match check {
                Ok(()) => entry.set_status(
                    "satisfied",
                    "Satisfied (both components are fixed).",
                    true,
                ),
                Err(_) => entry.set_status(
                    "blocked",
                    "Blocked: every participating component is fixed.",
                    false,
                ),
            }
            lanes.push(Lane::Settled);
            continue;
        }

        // Register bodies + mates for the single global solve.
        bodies.extend(participants.iter().map(|id| id.to_string()));
        let mut indexes = Vec::with_capacity(mapped.mates.len());
        for mapped_mate in &mapped.mates {
            indexes.push(mates.len());
            mate_sides.push((mapped_mate.body_a.clone(), mapped_mate.body_b.clone()));
            mates.push(AssemblyMate {
                id: Some(state.constraints[index].id().to_string()),
                body_a: 0, // patched below once the body table is final
                body_b: 0,
                kind: mapped_mate.kind.clone(),
            });
        }
        lanes.push(Lane::Solved {
            mates: indexes,
            mapped,
            scale,
        });
    }

    // --- The single solve ---------------------------------------------------
    if mates.is_empty() {
        outcome.report = serde_json::json!({ "ok": true, "mates": 0 });
        return outcome;
    }
    let body_inputs: Vec<AssemblyBody> = bodies
        .iter()
        .map(|id| body_from_component(scene, id))
        .collect();
    let body_index: BTreeMap<&str, usize> = bodies
        .iter()
        .enumerate()
        .map(|(position, id)| (id.as_str(), position))
        .collect();
    for (mate, (side_a, side_b)) in mates.iter_mut().zip(&mate_sides) {
        mate.body_a = body_index[side_a.as_str()];
        mate.body_b = body_index[side_b.as_str()];
    }

    match solve_assembly(&body_inputs, &mates, &AssemblySolveOptions::default()) {
        Ok(solution) => {
            // Per-constraint statuses + diagnostics.
            for (index, lane) in lanes.iter().enumerate() {
                let Lane::Solved { mates: indexes, mapped, scale } = lane else {
                    continue;
                };
                let residual = indexes
                    .iter()
                    .map(|&mate_index| solution.mate_residuals[mate_index].residual.abs())
                    .fold(0.0f64, f64::max);
                let tolerance = SATISFIED_REL_TOL * scale.max(1.0);
                let entry = &mut state.constraints[index];
                let note = mapped
                    .note
                    .as_deref()
                    .map(|note| format!(" {note}."))
                    .unwrap_or_default();
                if residual <= tolerance {
                    entry.set_status(
                        "satisfied",
                        format!("Satisfied (residual {residual:.2e}).{note}"),
                        true,
                    );
                } else {
                    entry.set_status(
                        "adjusted",
                        format!("Adjusting: residual {residual:.2e} above tolerance.{note}"),
                        false,
                    );
                }
                entry.set_persistent("error", serde_json::json!(residual));
                entry.set_persistent(
                    "kernel",
                    serde_json::json!({
                        "residual": residual,
                        "strategy": solution.strategy,
                        "iterations": solution.iterations,
                        "dof": solution.dof,
                        "rank": solution.rank,
                        "redundant": solution.redundant,
                        "status": solution.status,
                    }),
                );
                // First-solve initialization commits ONLY on solve success.
                for (key, value) in &mapped.pending_params {
                    if let Some(map) = entry.input_params.as_object_mut() {
                        map.insert(key.clone(), value.clone());
                    }
                }
                for (key, value) in &mapped.pending_persistent {
                    entry.set_persistent(key, value.clone());
                }
            }

            // Pose write-back: renormalized solver poses re-pose the scene
            // components in place and export the generic feature update.
            // Unmoved bodies (satisfied constraints) write NOTHING — a pose
            // update dirties the owning ACOMP feature's fingerprint, so a
            // no-motion solve must not churn the cache every run.
            let mut write_back_error: Option<String> = None;
            for pose in &solution.poses {
                let Some(record) = scene.resolve_component(&pose.id) else {
                    continue;
                };
                if record.fixed {
                    continue;
                }
                match mapping::pose_to_transform(pose.rotation, pose.translation) {
                    Ok(transform) => {
                        let moved = transform
                            .elements
                            .iter()
                            .zip(record.transform.elements.iter())
                            .any(|(new, old)| (new - old).abs() > 1e-12);
                        if !moved {
                            continue;
                        }
                        if let Err(error) = update_component_transform(scene, &pose.id, transform) {
                            write_back_error =
                                Some(format!("pose write-back for '{}': {error}", pose.id));
                            continue;
                        }
                        outcome
                            .pose_updates
                            .insert(pose.id.clone(), mapping::transform_to_pose_params(&transform));
                        outcome
                            .moved_solids
                            .extend(scene.component_solids(&pose.id));
                    }
                    Err(error) => {
                        write_back_error =
                            Some(format!("pose write-back for '{}': {error}", pose.id));
                    }
                }
            }

            outcome.report = match write_back_error {
                Some(error) => serde_json::json!({ "ok": false, "error": error }),
                None => serde_json::json!({
                    "ok": true,
                    "mates": mates.len(),
                    "strategy": solution.strategy,
                    "iterations": solution.iterations,
                    "maxResidual": solution.max_residual,
                    "dof": solution.dof,
                    "rank": solution.rank,
                    "redundant": solution.redundant,
                    "status": solution.status,
                    "bodies": bodies.iter().collect::<Vec<_>>(),
                    "movedComponents": outcome
                        .pose_updates
                        .keys()
                        .collect::<Vec<_>>(),
                    // The display-lane seam: geometry that moved IN PLACE this
                    // solve. A reused (cache-replayed) feature's solid can be
                    // among these — the renderer must re-tessellate them even
                    // though the producing feature reported `reused`.
                    "movedSolids": outcome
                        .moved_solids
                        .iter()
                        .map(|(name, handle)| serde_json::json!({ "name": name, "handle": handle }))
                        .collect::<Vec<_>>(),
                }),
            };
        }
        Err(error) => {
            // Conflict / non-convergence fails the WHOLE mapped set — never a
            // stale 'satisfied' from a previous run.
            for (index, lane) in lanes.iter().enumerate() {
                if matches!(lane, Lane::Solved { .. }) {
                    state.constraints[index].set_status("error", error.clone(), false);
                }
            }
            outcome.report = serde_json::json!({ "ok": false, "error": error });
        }
    }
    outcome
}

/// A grounded body for the solve, from the component record's rigid pose.
fn body_from_component(scene: &SceneMap, id: &str) -> AssemblyBody {
    let record = scene.resolve_component(id).expect("registered body");
    AssemblyBody {
        id: id.to_string(),
        fixed: record.fixed,
        rotation: mapping::matrix_to_quaternion(&record.transform),
        translation: [
            record.transform.elements[3],
            record.transform.elements[7],
            record.transform.elements[11],
        ],
    }
}

/// Both-fixed validity: a ZERO-unknown solve evaluates the exact mate
/// residuals against the grounded poses — `Ok` means the geometry already
/// satisfies the mates (within the loose check tolerance), `Err` means the
/// constraint cannot be met without moving a grounded component.
fn check_fixed_pair(scene: &SceneMap, mapped: &MappedConstraint) -> Result<(), String> {
    let mut ids: Vec<&str> = Vec::new();
    for mapped_mate in &mapped.mates {
        for id in [&mapped_mate.body_a, &mapped_mate.body_b] {
            if !ids.contains(&id.as_str()) {
                ids.push(id);
            }
        }
    }
    let bodies: Vec<AssemblyBody> = ids
        .iter()
        .map(|id| {
            let mut body = body_from_component(scene, id);
            body.fixed = true;
            body
        })
        .collect();
    let mates: Vec<AssemblyMate> = mapped
        .mates
        .iter()
        .map(|mapped_mate| AssemblyMate {
            id: None,
            body_a: ids.iter().position(|id| *id == mapped_mate.body_a).unwrap(),
            body_b: ids.iter().position(|id| *id == mapped_mate.body_b).unwrap(),
            kind: mapped_mate.kind.clone(),
        })
        .collect();
    let options = AssemblySolveOptions {
        tolerance: FIXED_CHECK_TOLERANCE,
        ..Default::default()
    };
    solve_assembly(&bodies, &mates, &options).map(|_| ())
}
