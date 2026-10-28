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
/// Flexible sub-assembly instances lend their occurrences and their own mates
/// to the solve; the mates leave `state` again as nested rows.
pub fn run_constraints(
    state: &mut AssemblyState,
    scene: &mut SceneMap,
    env: &Env,
) -> LifecycleOutcome {
    let outcome = solve_state(state, scene, env);
    super::flexible::collect(state, scene);
    outcome
}

fn solve_state(
    state: &mut AssemblyState,
    scene: &mut SceneMap,
    env: &Env,
) -> LifecycleOutcome {
    let mut outcome = LifecycleOutcome::default();
    let Prepared { lanes, mut mates, mate_sides, bodies } = prepare(state, scene, env, &mut outcome);

    // --- The single solve ---------------------------------------------------
    if mates.is_empty() {
        outcome.report = serde_json::json!({ "ok": true, "mates": 0 });
        return outcome;
    }
    let body_inputs = solve_bodies(scene, &bodies, &mut mates, &mate_sides);

    // A gear or rack turned by a typed edit since the last solve DRIVES its
    // partner: first solve with the driving side held (the drag's rule); if
    // that cannot hold, the free solve.
    let drivers: Vec<usize> = mates
        .iter()
        .filter_map(|mate| {
            let pose = |body: usize| (body_inputs[body].rotation, body_inputs[body].translation);
            match mate.kind.driver(pose(mate.body_a), pose(mate.body_b))? {
                true => Some(mate.body_a),
                false => Some(mate.body_b),
            }
        })
        .filter(|&body| !body_inputs[body].fixed)
        .collect();
    let led = (!drivers.is_empty())
        .then(|| {
            let held: Vec<AssemblyBody> = body_inputs
                .iter()
                .enumerate()
                .map(|(i, body)| AssemblyBody { fixed: body.fixed || drivers.contains(&i), ..body.clone() })
                .collect();
            solve_assembly(&held, &mates, &AssemblySolveOptions::default()).ok()
        })
        .flatten();
    let result = match led {
        Some(solution) => Ok(solution),
        None => solve_assembly(&body_inputs, &mates, &AssemblySolveOptions::default()),
    };

    match result {
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
                store_mechanical_references(entry, indexes, &mates, &solution);
            }

            // Pose write-back: renormalized solver poses re-pose the scene
            // components in place and export the generic feature update.
            // Unmoved bodies (satisfied constraints) write NOTHING — a pose
            // update dirties the owning ACOMP feature's fingerprint, so a
            // no-motion solve must not churn the cache every run.
            let mut write_back_error: Option<String> = None;
            // Outer bodies first: a flexible occurrence moves with the body
            // that places it, so it takes its own pose after that one.
            let mut poses: Vec<_> = solution.poses.iter().collect();
            poses.sort_by_key(|pose| pose.id.matches(':').count());
            for pose in poses {
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
                        // A flexible occurrence's pose persists on its
                        // instance, in the instance frame.
                        let persisted = super::flexible::occurrence_local(scene, &pose.id, &transform)
                            .unwrap_or(transform);
                        outcome
                            .pose_updates
                            .insert(pose.id.clone(), mapping::transform_to_pose_params(&persisted));
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
                    // Per component, what the mates leave free (fixed
                    // components are not listed; an absent map = not analysed).
                    "bodyFreedom": solution
                        .body_freedom
                        .iter()
                        .map(|freedom| (freedom.id.clone(), serde_json::json!({
                            "rotations": freedom.rotations,
                            "translations": freedom.translations,
                            // Which: the axis lines it can turn about and the
                            // directions it can slide (world).
                            "axes": freedom.axes,
                            "slides": freedom.slides,
                        })))
                        .collect::<serde_json::Map<String, serde_json::Value>>(),
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
            // stale 'satisfied' from a previous run — and nothing moves. When
            // a minimal conflicting set can be named, its members read
            // `conflict` and the rest of the set reads as unsolved because of it.
            let solved: Vec<usize> = (0..lanes.len())
                .filter(|&index| matches!(lanes[index], Lane::Solved { .. }))
                .collect();
            let seeds = lanes_named(&state.constraints, &solved, &worst_mate_ids(&error));
            let conflict = minimal_conflict(&body_inputs, &mates, &lanes, &solved, &seeds);
            match conflict {
                // One constraint that fails on its own conflicts with nothing:
                // the solve could not reach it from the current poses (e.g. a
                // face asked to turn exactly about-face). Named as such, with
                // the solver's residual, never as a conflict.
                Some(members) if members.len() == 1 => {
                    let error = not_a_conflict(&error);
                    let stuck = members[0];
                    let stuck_id = state.constraints[stuck].id().to_string();
                    for &index in &solved {
                        let entry = &mut state.constraints[index];
                        if index == stuck {
                            entry.set_status(
                                "error",
                                format!("Did not converge from the current position: {error}. Move the component nearer the solution, or Flip it."),
                                false,
                            );
                        } else {
                            entry.set_status("error", format!("Not solved: {stuck_id} did not converge."), false);
                        }
                    }
                    outcome.report = serde_json::json!({
                        "ok": false,
                        "error": format!("{stuck_id} did not converge: {error}"),
                        "nonConvergent": stuck_id,
                    });
                }
                Some(members) => {
                    let ids: Vec<String> =
                        members.iter().map(|&index| state.constraints[index].id().to_string()).collect();
                    let newest = members
                        .iter()
                        .copied()
                        .max_by_key(|&index| (id_counter(state.constraints[index].id()), index))
                        .expect("a conflict has members");
                    let newest_id = state.constraints[newest].id().to_string();
                    for &index in &solved {
                        let entry = &mut state.constraints[index];
                        if members.contains(&index) {
                            let others: Vec<String> =
                                ids.iter().filter(|id| id.as_str() != entry.id()).cloned().collect();
                            entry.set_status(
                                "conflict",
                                format!(
                                    "Conflicts with {}: these constraints cannot all hold. Suppress one to solve (suggested: {newest_id}).",
                                    others.join(", ")
                                ),
                                false,
                            );
                            entry.set_persistent("conflictWith", serde_json::json!(others));
                            entry.set_persistent("suppressSuggested", serde_json::json!(index == newest));
                        } else {
                            entry.set_status(
                                "error",
                                format!("Not solved: constraints {} conflict.", ids.join(", ")),
                                false,
                            );
                        }
                    }
                    outcome.report = serde_json::json!({
                        "ok": false,
                        "error": format!("conflicting constraints {}: {error}", ids.join(", ")),
                        "conflict": ids,
                        "suggestSuppress": newest_id,
                    });
                }
                None => {
                    let error = not_a_conflict(&error);
                    for &index in &solved {
                        state.constraints[index].set_status("error", error.clone(), false);
                    }
                    outcome.report = serde_json::json!({ "ok": false, "error": error });
                }
            }
        }
    }
    outcome
}

/// Re-capture every mechanical mate's stored references at `poses` (component
/// id → pose; components not named keep their scene pose) — the live drag's
/// release, whose poses the mates already hold. `true` when anything was
/// written. Mapping runs on a clone; only the persistent references change.
pub fn rebase_mechanical(
    state: &mut AssemblyState,
    scene: &mut SceneMap,
    env: &Env,
    poses: &BTreeMap<String, crate::AffineTransform>,
) -> bool {
    let mut probe = state.clone();
    let mut outcome = LifecycleOutcome::default();
    let Prepared { lanes, mut mates, mate_sides, bodies } = prepare(&mut probe, scene, env, &mut outcome);
    let body_inputs = solve_bodies(scene, &bodies, &mut mates, &mate_sides);
    let pose = |body: &AssemblyBody| match poses.get(&body.id) {
        Some(transform) => (
            mapping::matrix_to_quaternion(transform),
            [transform.elements[3], transform.elements[7], transform.elements[11]],
        ),
        None => (body.rotation, body.translation),
    };
    let mut changed = false;
    for (index, lane) in lanes.iter().enumerate() {
        let Lane::Solved { mates: indexes, .. } = lane else { continue };
        for &mi in indexes {
            let mate = &mates[mi];
            let mut kind = mate.kind.clone();
            if kind.references().is_none() {
                continue;
            }
            kind.rebase(pose(&body_inputs[mate.body_a]), pose(&body_inputs[mate.body_b]));
            if let Some(values) = kind.references() {
                let entry = &mut state.constraints[index];
                entry.set_persistent("mechanicalReference", serde_json::json!(values));
                entry.set_persistent(
                    "mechanicalSignature",
                    serde_json::json!(constraints::mechanical::signature(entry)),
                );
                changed = true;
            }
        }
    }
    changed
}

/// Persist a mechanical mate's references at the SOLVED poses
/// (`mechanicalReference`, keyed by `mechanicalSignature`), so the next run
/// maps from them: a typed edit that turns one gear is then coupled to the
/// other by that run's solve, instead of being re-held where it was typed.
fn store_mechanical_references(
    entry: &mut super::ConstraintEntry,
    indexes: &[usize],
    mates: &[AssemblyMate],
    solution: &crate::AssemblySolution,
) {
    for &index in indexes {
        let mate = &mates[index];
        let mut kind = mate.kind.clone();
        if kind.references().is_none() {
            continue;
        }
        let pose = |body: usize| (solution.poses[body].rotation, solution.poses[body].translation);
        kind.rebase(pose(mate.body_a), pose(mate.body_b));
        if let Some(values) = kind.references() {
            entry.set_persistent("mechanicalReference", serde_json::json!(values));
            entry.set_persistent(
                "mechanicalSignature",
                serde_json::json!(constraints::mechanical::signature(entry)),
            );
        }
    }
}

/// The most constraints a failed solve is searched for a minimal conflict:
/// the deletion filter costs one solve per constraint.
const CONFLICT_SEARCH_LIMIT: usize = 96;

/// A minimal conflicting subset of the `solved` lanes (deletion filter: drop
/// each constraint in turn, oldest first, while the rest still fails), or
/// `None` when no searched set fails (a non-convergence that is not a
/// conflict) or none small enough could be found.
///
/// Above [`CONFLICT_SEARCH_LIMIT`] constraints the search is SCOPED: it
/// starts from `seeds` (the constraints the failed solve named worst) and
/// grows rings of the constraints touching their movable bodies (never
/// through a fixed body, which would take in everything) until that subset
/// fails on its own, then filters inside it. One conflicting pair in a
/// 200-mate assembly used to mark all 200 rows "error" and name nothing.
fn minimal_conflict(
    bodies: &[AssemblyBody],
    mates: &[AssemblyMate],
    lanes: &[Lane],
    solved: &[usize],
    seeds: &[usize],
) -> Option<Vec<usize>> {
    if solved.is_empty() {
        return None;
    }
    let options = AssemblySolveOptions { diagnostics: false, ..Default::default() };
    let lane_mates = |index: usize| -> &[usize] {
        match &lanes[index] {
            Lane::Solved { mates: indexes, .. } => indexes,
            Lane::Settled => &[],
        }
    };
    let fails = |set: &[usize]| -> bool {
        let subset: Vec<AssemblyMate> =
            set.iter().flat_map(|&index| lane_mates(index).iter()).map(|&mate| mates[mate].clone()).collect();
        solve_assembly(bodies, &subset, &options).is_err()
    };
    let candidates: Vec<usize> = if solved.len() <= CONFLICT_SEARCH_LIMIT {
        if !fails(solved) {
            return None;
        }
        solved.to_vec()
    } else {
        let movable_bodies = |set: &[usize]| -> std::collections::BTreeSet<usize> {
            set.iter()
                .flat_map(|&index| lane_mates(index).iter())
                .flat_map(|&mate| [Some(mates[mate].body_a), Some(mates[mate].body_b), mates[mate].body_c])
                .flatten()
                .filter(|&body| !bodies[body].fixed)
                .collect()
        };
        let mut set: Vec<usize> = seeds.iter().copied().filter(|index| solved.contains(index)).collect();
        loop {
            if set.is_empty() || set.len() > CONFLICT_SEARCH_LIMIT {
                return None;
            }
            if fails(&set) {
                break set;
            }
            let reach = movable_bodies(&set);
            let grown: Vec<usize> = solved
                .iter()
                .copied()
                .filter(|&index| {
                    set.contains(&index)
                        || lane_mates(index).iter().any(|&mate| {
                            [Some(mates[mate].body_a), Some(mates[mate].body_b), mates[mate].body_c]
                                .into_iter()
                                .flatten()
                                .any(|body| reach.contains(&body))
                        })
                })
                .collect();
            if grown.len() == set.len() {
                return None;
            }
            set = grown;
        }
    };
    let mut set = candidates.clone();
    for &candidate in &candidates {
        let trial: Vec<usize> = set.iter().copied().filter(|&index| index != candidate).collect();
        if !trial.is_empty() && fails(&trial) {
            set = trial;
        }
    }
    Some(set)
}

/// The constraint ids a solver failure names as its worst mates (`…; worst
/// mates: TALN3 (residual 5.000e0), DIST2 (residual …)`, the solver's own
/// wording, `assembly_solver::numerics::finalize`).
fn worst_mate_ids(error: &str) -> Vec<String> {
    let Some((_, list)) = error.rsplit_once("worst mates: ") else {
        return Vec::new();
    };
    list.split("), ")
        .filter_map(|item| item.split_once(" (residual").map(|(id, _)| id.trim().to_string()))
        .filter(|id| !id.is_empty())
        .collect()
}

/// The `solved` lane indexes whose constraint ids are among `ids`.
fn lanes_named(constraints: &[super::ConstraintEntry], solved: &[usize], ids: &[String]) -> Vec<usize> {
    solved.iter().copied().filter(|&index| ids.iter().any(|id| id == constraints[index].id())).collect()
}

/// The solver's failure without its "conflicting mates (over-constrained)"
/// cause, for a failure the conflict search found NOT to be a conflict (one
/// constraint that fails alone, or a set whose parts all solve) or could not
/// search. That cause is read off the Jacobian's rank deficit at the point
/// the solve stopped, and a stuck start (no gradient toward the solution) is
/// rank-deficient too: the label sent users deleting mates that were fine.
pub(super) fn not_a_conflict(error: &str) -> String {
    error.replacen("conflicting mates (over-constrained)", "the solve did not reach a solution from here", 1)
}

/// The persistent counter a constraint id was minted with (`TALN3` → 3), for
/// "most recently added"; ids without one sort first.
fn id_counter(id: &str) -> u64 {
    let digits: String = id.chars().rev().take_while(|c| c.is_ascii_digit()).collect();
    digits.chars().rev().collect::<String>().parse().unwrap_or(0)
}

/// The live-drag dry run: solve the mates with the `drag` components seeded at
/// their dragged poses and return the solved pose of every movable body (a
/// dragged component no mate reaches keeps its dragged pose). `unfix` members
/// count as free for this solve only. `reference` holds the last good live
/// poses: the mechanical mates (gear, rack and pinion, screw) are re-based at
/// them, so each frame couples only the motion since the last good frame and
/// a long drag never wraps a spin; bodies without one use their session pose.
///
/// Two phases. First the dragged components are HELD at the drag pose and
/// everything else solves around them, so a dragged part leads (a gear turns
/// its partner, a slider pushes its linkage). When that cannot hold, they are
/// freed and the solve projects the drag onto the mates (a part pulled off its
/// face stays on it). NOTHING is written: the state is a clone, the scene's
/// fixed flags are restored and no resident solid is re-posed. `Err` is a
/// drag the mates cannot hold either way (the caller keeps its last good
/// pose).
///
/// With `explain`, a drag the held phase could not hold also names the
/// constraints that refused it: a minimal set that cannot hold with the
/// dragged components at the drag pose (the conflict search, one solve per
/// constraint, so a live drag frame never asks). Empty when the held phase
/// held, or when no such set can be named.
pub fn drag_solve(
    state: &AssemblyState,
    scene: &mut SceneMap,
    env: &Env,
    drag: &BTreeMap<String, crate::AffineTransform>,
    unfix: &[String],
    reference: &BTreeMap<String, crate::AffineTransform>,
    explain: bool,
) -> Result<(BTreeMap<String, crate::AffineTransform>, Vec<String>), String> {
    let restore: Vec<(String, bool)> = unfix
        .iter()
        .filter_map(|id| Some((id.clone(), scene.component_fixed(id)?)))
        .collect();
    for (id, _) in &restore {
        scene.set_component_fixed(id, false);
    }
    let mut state = state.clone();
    let mut outcome = LifecycleOutcome::default();
    let Prepared { lanes, mut mates, mate_sides, bodies } =
        prepare(&mut state, scene, env, &mut outcome);
    let mut body_inputs = solve_bodies(scene, &bodies, &mut mates, &mate_sides);
    for (id, fixed) in &restore {
        scene.set_component_fixed(id, *fixed);
    }
    let pose = |body: &AssemblyBody| match reference.get(&body.id) {
        Some(transform) => (
            mapping::matrix_to_quaternion(transform),
            [transform.elements[3], transform.elements[7], transform.elements[11]],
        ),
        None => (body.rotation, body.translation),
    };
    for mate in &mut mates {
        let (a, b) = (pose(&body_inputs[mate.body_a]), pose(&body_inputs[mate.body_b]));
        mate.kind.rebase(a, b);
    }
    for body in &mut body_inputs {
        // A flexible instance's occurrence rides its dragged instance: seed it
        // at the drag delta composed onto its current pose (overriding the
        // warm start), so the child's own mates start satisfied instead of
        // fighting the drag.
        let carried = body.id.split_once(':').and_then(|(owner, _)| {
            let target = drag.get(owner).filter(|_| !drag.contains_key(&body.id))?;
            // The last good live frame when there is one, else the session.
            let now = |id: &str| reference.get(id).copied().or_else(|| Some(scene.resolve_component(id)?.transform));
            let owner_now = now(owner)?;
            let occurrence_now = now(&body.id)?;
            let delta = crate::feature_pipeline::component::compose(target, &owner_now.rigid_inverse().ok()?).ok()?;
            crate::feature_pipeline::component::compose(&delta, &occurrence_now).ok()
        });
        if let Some(transform) = drag.get(&body.id).or(carried.as_ref()) {
            body.fixed = false;
            body.rotation = mapping::matrix_to_quaternion(transform);
            body.translation = [transform.elements[3], transform.elements[7], transform.elements[11]];
        } else if let Some(transform) = reference.get(&body.id).filter(|_| !body.fixed) {
            // Warm start: the partners begin where the last good frame left them.
            body.rotation = mapping::matrix_to_quaternion(transform);
            body.translation = [transform.elements[3], transform.elements[7], transform.elements[11]];
        }
    }
    let mut poses = drag.clone();
    if mates.is_empty() {
        return Ok((poses, Vec::new()));
    }
    let held: Vec<AssemblyBody> = body_inputs
        .iter()
        .map(|body| AssemblyBody { fixed: body.fixed || drag.contains_key(&body.id), ..body.clone() })
        .collect();
    // A drag reads poses only: the DOF diagnostics are skipped.
    let free = AssemblySolveOptions { diagnostics: false, ..Default::default() };
    let lead = AssemblySolveOptions { max_iterations: DRAG_LEAD_MAX_ITERATIONS, ..free };
    let mut blocking = Vec::new();
    let solution = match solve_assembly(&held, &mates, &lead) {
        Ok(solution) => solution,
        Err(error) => {
            if explain {
                let solved: Vec<usize> = (0..lanes.len())
                    .filter(|&index| matches!(lanes[index], Lane::Solved { .. }))
                    .collect();
                let seeds = lanes_named(&state.constraints, &solved, &worst_mate_ids(&error));
                blocking = minimal_conflict(&held, &mates, &lanes, &solved, &seeds)
                    .unwrap_or_default()
                    .iter()
                    .map(|&index| state.constraints[index].id().to_string())
                    .collect();
            }
            solve_assembly(&body_inputs, &mates, &free)?
        }
    };
    for (pose, body) in solution.poses.iter().zip(&body_inputs) {
        if body.fixed {
            continue;
        }
        poses.insert(pose.id.clone(), mapping::pose_to_transform(pose.rotation, pose.translation)?);
    }
    Ok((poses, blocking))
}

/// One frame of a motion study: the session state with some mates driven to
/// a value and some switched on or off (`overrides`: constraint id →
/// `(param, value)` and/or `enabled`), solved from the previous frame's poses
/// (`reference`: the warm start, and where mechanical mates re-base, as a
/// live drag's frames do). NOTHING is written: the overrides go on a clone,
/// which is dropped. `Err` names a frame the mates cannot hold, or an
/// override on a constraint the session does not have.
pub fn frame_solve(
    state: &AssemblyState,
    scene: &mut SceneMap,
    env: &Env,
    overrides: &BTreeMap<String, FrameOverride>,
    reference: &BTreeMap<String, crate::AffineTransform>,
) -> Result<BTreeMap<String, crate::AffineTransform>, String> {
    let mut state = state.clone();
    for (id, change) in overrides {
        let entry = state
            .constraints
            .iter_mut()
            .find(|entry| entry.id() == id)
            .ok_or_else(|| format!("frame solve: unknown constraint '{id}'"))?;
        if let Some((param, value)) = &change.value {
            if !value.is_finite() {
                return Err(format!("frame solve: '{id}' {param} is not a number"));
            }
            entry.input_params[param.as_str()] = serde_json::json!(value);
        }
        if let Some(enabled) = change.enabled {
            entry.enabled = enabled;
        }
    }
    let (poses, _) = drag_solve(&state, scene, env, &BTreeMap::new(), &[], reference, false)?;
    Ok(poses)
}

/// What a study does to one mate for a frame ([`frame_solve`]).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FrameOverride {
    /// The param driven and its value (`("angle", 370.0)`).
    pub value: Option<(String, f64)>,
    pub enabled: Option<bool>,
}

/// The held phase of a drag solve gives up early: a drag that cannot be held
/// usually shows it within a few steps, and the free phase still runs.
const DRAG_LEAD_MAX_ITERATIONS: usize = 50;

/// Everything the single solve consumes, after validate + resolve + map.
struct Prepared {
    lanes: Vec<Lane>,
    mates: Vec<AssemblyMate>,
    mate_sides: Vec<(String, String, Option<String>)>,
    bodies: std::collections::BTreeSet<String>,
}

/// Validate, resolve and map every entry: settled lanes get their terminal
/// status here, solved lanes contribute bodies and mates (body indexes are
/// patched by [`solve_bodies`]). Mapped mate geometry is body-LOCAL, so the
/// result does not depend on where the bodies currently are.
fn prepare(
    state: &mut AssemblyState,
    scene: &mut SceneMap,
    env: &Env,
    outcome: &mut LifecycleOutcome,
) -> Prepared {
    super::flexible::expand(state, scene);
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
    let mut mate_sides: Vec<(String, String, Option<String>)> = Vec::new();
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

        // Duplicate marking: of the overlapping-family mates on the same
        // selections, the FIRST holds and every later one is marked and
        // skipped. (All of them used to be skipped, so adding a duplicate
        // silently switched the original mate off and the part lost it.)
        if def.duplicate_family {
            let signature = mapping::pair_signature(&elements);
            if let Some(members) = duplicate_of.get(&signature).filter(|members| members[0] != index) {
                let others: Vec<(String, String)> = members
                    .iter()
                    .take(1)
                    .map(|&other| {
                        let peer = &state.constraints[other];
                        let label = constraints::constraint_type(&peer.constraint_type)
                            .map(|def| def.label)
                            .unwrap_or(peer.constraint_type.as_str());
                        (label.to_string(), peer.id().to_string())
                    })
                    .collect();
                let entry = &mut state.constraints[index];
                let mut message = String::new();
                for (label, id) in &others {
                    message.push_str(&format!(
                        "Duplicate of {label} constraint {id} on the same selections: it has no effect while {id} holds. Delete it, or change its selections."
                    ));
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
            mate_sides.push((
                mapped_mate.body_a.clone(),
                mapped_mate.body_b.clone(),
                mapped_mate.body_c.clone(),
            ));
            mates.push(AssemblyMate {
                id: Some(state.constraints[index].id().to_string()),
                body_a: 0, // patched below once the body table is final
                body_b: 0,
                body_c: None,
                kind: mapped_mate.kind.clone(),
            });
        }
        lanes.push(Lane::Solved {
            mates: indexes,
            mapped,
            scale,
        });
    }

    Prepared { lanes, mates, mate_sides, bodies }
}

/// The solver's body table (sorted ids, grounded flags and current poses),
/// patching every mate's body indexes to it.
fn solve_bodies(
    scene: &SceneMap,
    bodies: &std::collections::BTreeSet<String>,
    mates: &mut [AssemblyMate],
    mate_sides: &[(String, String, Option<String>)],
) -> Vec<AssemblyBody> {
    let body_index: BTreeMap<&str, usize> = bodies
        .iter()
        .enumerate()
        .map(|(position, id)| (id.as_str(), position))
        .collect();
    for (mate, (side_a, side_b, side_c)) in mates.iter_mut().zip(mate_sides) {
        mate.body_a = body_index[side_a.as_str()];
        mate.body_b = body_index[side_b.as_str()];
        mate.body_c = side_c.as_ref().map(|side| body_index[side.as_str()]);
    }
    bodies.iter().map(|id| body_from_component(scene, id)).collect()
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
        for id in [&mapped_mate.body_a, &mapped_mate.body_b].into_iter().chain(mapped_mate.body_c.as_ref()) {
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
            body_c: mapped_mate
                .body_c
                .as_ref()
                .map(|side| ids.iter().position(|id| *id == side.as_str()).unwrap()),
            kind: mapped_mate.kind.clone(),
        })
        .collect();
    let options = AssemblySolveOptions {
        tolerance: FIXED_CHECK_TOLERANCE,
        ..Default::default()
    };
    solve_assembly(&bodies, &mates, &options).map(|_| ())
}
