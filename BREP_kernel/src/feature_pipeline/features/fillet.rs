//! Rolling-ball edge blends, with optional taper through `radiusEnd`.
//!
//! Edge and face selections resolve to one solid and are blended together,
//! including shared convex corners. Cross-solid selections return no changes.
//!
//! The result retains the target's name. Generated faces use the shared
//! `{featureID || 'F'}:BLEND:{edgeName}` naming scheme, with corner patches
//! under `{base}:CORNER:{adjacent edge names}`.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult};
use crate::{KernelRefusal, KernelStage};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let selection = common::resolve_blend_selection(ctx)?;
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    result.unresolved = selection.unresolved;

    // A cross-solid selection is a SOFT abort (warn + return empty).
    if selection.multi_solid {
        return Ok(result);
    }
    // A selection in which NOTHING resolves is refused by name.  Until
    // 2026-09-26 the misses rode along in `unresolved` and the feature
    // returned success for a document that blended no edge at all
    // (`Box_PY|Box_NZ[0]`, the faces in the wrong order, on the through-hole
    // case document): a no-op reported as a result.  A selection in which SOME
    // names resolve still blends those and reports the misses — that is the
    // engine's contract (rule 1: a miss is recorded in `unresolved`, annotated
    // with the name that replaced it, never a no-op with no signal), and two
    // reported documents rely on it: the 2026-09-10 mouth-over-wall and
    // collapsed-fillet reports both select `Box_PY|Box_PZ[0]`, which their own
    // r = 18 blend consumed, beside the live mouth edge.  Whether that partial
    // should refuse too is a contract question, not this feature's alone.
    let Some(mut target) = selection.target else {
        if result.unresolved.is_empty() {
            return Ok(result);
        }
        let mut refused = ctx.fail(unresolved_refusal("fillet", &result.unresolved));
        refused.unresolved = result.unresolved;
        return Ok(refused);
    };
    // The feature proceeds on the references that resolved: a miss is
    // reported as a typed partial fulfilment beside `unresolved`, never
    // silently (the engine annotates both with the rename hint).
    result.note_partial_resolution(&common::reference_names(ctx.param("edges")));

    // A picked SEAM (an edge with the same face on both sides: the line
    // where a cylinder's surface closes on itself) has no corner to round.
    // It is skipped with a note, and the rest of the selection is blended;
    // before 2026-11-16 the blend refused the whole selection ("edge borders
    // 1 faces") over the seam a user picked with the cylinder's edges.
    let seams = drop_seam_edges(&mut target)?;
    if !seams.is_empty() {
        result.notes.push(format!(
            "Fillet: {} closes a face on itself (a seam, not a corner) and was skipped",
            seams.join(", ")
        ));
        if target.edge_points.is_empty() {
            return Ok(result);
        }
    }

    let radius = ctx.number("radius")?;
    // A non-positive / non-finite radius is a soft no-op.
    if !(radius.is_finite() && radius > 0.0) {
        return Ok(result);
    }

    // `radiusEnd > 0 && |radiusEnd − radius| > 1e-9` selects the VARIABLE
    // (tapered) fillet.
    let taper = common::optional_number(ctx, "radiusEnd")?
        .filter(|value| value.is_finite() && *value > 0.0 && (*value - radius).abs() > 1e-9);

    // "Follow tangent": every edge picked AS AN EDGE grows to its whole G1
    // tangent chain on the solid (a picked face keeps meaning its own
    // boundary edges, unchanged), derived
    // afresh on every run from the resolved references, so replay re-reads
    // it by name and never stores it. A key the document does not carry
    // reads as OFF: documents written before the option exist are unchanged;
    // a new feature takes the schema's `true`.
    let followed = if follow_tangent(ctx) { follow_tangent_chains(&mut target)? } else { Vec::new() };
    if !followed.is_empty() {
        let requested = common::reference_names(ctx.param("edges"));
        let fulfilment = result.fulfilment.get_or_insert_with(|| crate::feature_pipeline::Fulfilment {
            applied: requested.clone(),
            requested,
            ..Default::default()
        });
        fulfilment.followed = followed;
    }

    // Names parallel the sampled edge points; the base also names corner patches.
    let blend_base = common::blend_face_base(&ctx.id);
    let blend_names: Vec<String> = target
        .edge_names
        .iter()
        .map(|edge_name| common::blend_face_name(&ctx.id, edge_name))
        .collect();
    let build_blend = || crate::with_registered_solid_typed(target.handle, |solid| match taper {
        Some(radius_end) => crate::fillet_edges_variable(
            solid,
            &target.edge_points,
            Some(&blend_names),
            &[(0.0, radius), (1.0, radius_end)],
            false,
            Some(&blend_base),
        ),
        // Through the options call, so a sliver the round swallows is healed
        // where the plain build refuses (`fillet_edges_with_options`). The
        // chains are already followed above as the option asks, and the
        // pinch keeps its ridge: users get what they got, plus that heal.
        None => crate::fillet_edges_with_options(
            solid,
            &target.edge_points,
            Some(&blend_names),
            radius,
            crate::FilletOptions { follow_tangent_chains: false, refuse_pinch: false },
            Some(&blend_base),
        ),
    })
    // The blend's refusal keeps its class through the feature's wrap.
    .map_err(|error| error.with_message(|error| format!("fillet failed: {error}")));
    // "Wrap sharp corners": the build runs with the ball rolled round each
    // unselected sharp edge two selected concave blends meet over on a curved
    // shared face. It must never do worse than the option off: when a corner
    // it took up could not be closed (a refusal, or a body the acceptance
    // rejected, which sends the ladder on to another composition), or the
    // build failed, the selection is built again without it and a note says so.
    let blended = if wrap_corners(ctx) {
        let (wrapped, outcome) = brep_blend::blend::with_wrap_sharp_corners(build_blend);
        // A closure built on a body some later stage threw away counts in
        // `outcome.wrapped`; the returned body must carry one itself.
        let carries = |body: &crate::BrepSolid| {
            brep_blend::blend::body_carries_wrapped_corner(body, &outcome, 1e-9 * (1.0 + crate::solid_scale(body)))
        };
        if outcome.candidates == 0 || (outcome.failed == 0 && wrapped.as_ref().is_ok_and(carries)) {
            // A document that a later feature built on this selection's sharp
            // seam (a second fillet picking the miter between two of these
            // blends, saved while that corner could not be wrapped) keeps the
            // seam: wrapping would remove the edge the later feature names.
            let kept = wrapped.as_ref().map(|body| seams_named_later(ctx, &target.edge_names, body)).unwrap_or_default();
            if kept.is_empty() {
                wrapped
            } else {
                result.notes.push(format!(
                    "Wrap sharp corners: a later feature references {}, the sharp seam wrapping would remove, so the fillet was built without the option; remove that reference to wrap the corner",
                    kept.join(", ")
                ));
                build_blend()
            }
        } else {
            result.notes.push(
                "Wrap sharp corners: a sharp-edge corner of this selection could not be wrapped, so the fillet was built without the option"
                    .to_string(),
            );
            build_blend()
        }
    } else {
        build_blend()
    };
    let blended = match blended {
        Ok(blended) => blended,
        Err(refusal) => {
            // A constant-radius selection that ends inside a tangent chain
            // and refused is rerun ONCE along the open chain; the plain
            // refusal stands unless that rerun builds.
            let rerun = if taper.is_none() { along_tangent_chain(ctx, &target, radius, &blend_base) } else { None };
            let Some((blended, extended)) = rerun else { return Err(refusal.into()) };
            let requested = common::reference_names(ctx.param("edges"));
            let fulfilment = result.fulfilment.get_or_insert_with(|| crate::feature_pipeline::Fulfilment {
                applied: requested.clone(),
                requested,
                ..Default::default()
            });
            fulfilment.extended = extended;
            blended
        }
    };

    result.added.push(common::register_added(blended, &target.name));
    result.removed.push(target.name);
    Ok(result)
}

/// The `wrapCorners` input: only an explicit `true` turns it on.
fn wrap_corners(ctx: &FeatureContext) -> bool {
    ctx.param("wrapCorners").and_then(|value| value.as_bool()).unwrap_or(false)
}

/// A fillet descriptor with "Wrap sharp corners" on: the history walk hands
/// it the later references to its own blend faces and edges.
pub(crate) fn wraps_corners(descriptor: &crate::feature_pipeline::FeatureDescriptor) -> bool {
    matches!(descriptor.feature_type.as_str(), "F" | "FILLET")
        && descriptor.input_params.get("wrapCorners").and_then(|value| value.as_bool()).unwrap_or(false)
}

/// The later references (`ctx.later_inputs`) that name the seam between two
/// of this selection's blends where `wrapped` carries the corner patch that
/// replaced it: `{base}:CORNER:{a}+{b}` stands where the miter edge
/// `{base}:{a}|{base}:{b}[k]` (either order) would be.
fn seams_named_later(ctx: &FeatureContext, edge_names: &[String], wrapped: &crate::BrepSolid) -> Vec<String> {
    let Some(later) = ctx.later_inputs.filter(|later| !later.is_empty()) else { return Vec::new() };
    let base = common::blend_face_base(&ctx.id);
    let faces: std::collections::HashSet<&str> =
        wrapped.shells.iter().flat_map(|shell| &shell.faces).filter_map(|face| face.name.as_deref()).collect();
    let mut kept = Vec::new();
    for (i, a) in edge_names.iter().enumerate() {
        for b in &edge_names[i + 1..] {
            if !faces.contains(format!("{base}:CORNER:{a}+{b}").as_str()) && !faces.contains(format!("{base}:CORNER:{b}+{a}").as_str()) {
                continue;
            }
            let (fa, fb) = (common::blend_face_name(&ctx.id, a), common::blend_face_name(&ctx.id, b));
            let seams = [format!("{fa}|{fb}["), format!("{fb}|{fa}[")];
            // Exactly the edge's name, `…[k]` with nothing after it, as a
            // reference spells it.
            let is_seam = |name: &str| {
                seams.iter().any(|seam| {
                    name.strip_prefix(seam.as_str()).and_then(|rest| rest.strip_suffix(']')).is_some_and(|index| {
                        !index.is_empty() && index.bytes().all(|byte| byte.is_ascii_digit())
                    })
                })
            };
            kept.extend(later.iter().filter(|name| is_seam(name)).cloned());
        }
    }
    kept.sort();
    kept.dedup();
    kept
}

/// The `followTangent` input: only an explicit `true` turns it on.
fn follow_tangent(ctx: &FeatureContext) -> bool {
    ctx.param("followTangent").and_then(|value| value.as_bool()).unwrap_or(false)
}

/// Append the tangent-chain edges of `target`'s edge picks to it, keeping its
/// points, names and ids parallel; returns the added edges' names. An edge a
/// picked face already gathered is not added twice: the walk treats every
/// selected edge as taken.
/// Remove the target's seam edges (the same face on both sides), returning
/// their names.
fn drop_seam_edges(target: &mut common::BlendTarget) -> Result<Vec<String>, FeatureRefusal> {
    let seams: Vec<u64> = crate::with_registered_solid_str(target.handle, |solid| {
        Ok(target
            .edge_ids
            .iter()
            .copied()
            .filter(|edge_id| {
                let mut faces = solid
                    .shells
                    .iter()
                    .flat_map(|shell| &shell.faces)
                    .filter(|face| face.loops.iter().any(|lp| lp.coedges.iter().any(|coedge| coedge.edge_id == *edge_id)));
                let uses: usize = solid
                    .shells
                    .iter()
                    .flat_map(|shell| &shell.faces)
                    .flat_map(|face| &face.loops)
                    .flat_map(|lp| &lp.coedges)
                    .filter(|coedge| coedge.edge_id == *edge_id)
                    .count();
                faces.next().is_some() && faces.next().is_none() && uses == 2
            })
            .collect())
    })
    .map_err(|error| KernelRefusal::internal(KernelStage::Collect, "seam_edges", error))?;
    let mut names = Vec::new();
    let mut index = 0;
    while index < target.edge_ids.len() {
        if seams.contains(&target.edge_ids[index]) {
            let id = target.edge_ids.remove(index);
            target.edge_points.remove(index);
            names.push(target.edge_names.remove(index));
            target.picked_edge_ids.retain(|picked| *picked != id);
        } else {
            index += 1;
        }
    }
    Ok(names)
}

fn follow_tangent_chains(target: &mut common::BlendTarget) -> Result<Vec<String>, FeatureRefusal> {
    let expansion = crate::with_registered_solid_str(target.handle, |solid| {
        common::tangent_chain_expansion(solid, &target.picked_edge_ids, &target.edge_ids)
    })
    .map_err(|error| KernelRefusal::internal(KernelStage::Collect, "tangent_chain", error))?;
    let mut added = Vec::with_capacity(expansion.len());
    for (id, point, name) in expansion {
        target.edge_ids.push(id);
        target.edge_points.push(point);
        target.edge_names.push(name.clone());
        added.push(name);
    }
    Ok(added)
}

/// The fallback for a selection that refused while ending inside a tangent
/// chain: the same blend with the chain's unselected edges added
/// (`common::open_tangent_chain_extension`). `None` when no selected edge
/// runs on into one, or when the rerun refuses too; otherwise the body and
/// the added edges' names.
fn along_tangent_chain(
    ctx: &FeatureContext,
    target: &common::BlendTarget,
    radius: f64,
    blend_base: &str,
) -> Option<(crate::BrepSolid, Vec<String>)> {
    crate::with_registered_solid_typed(target.handle, |solid| {
        let extension = common::open_tangent_chain_extension(solid, &target.edge_ids)
            .map_err(|error| KernelRefusal::internal(KernelStage::Collect, "tangent_chain", error))?;
        if extension.is_empty() {
            return Err(KernelRefusal::internal(KernelStage::Collect, "tangent_chain", "no tangent continuation"));
        }
        let mut points = target.edge_points.clone();
        let mut names: Vec<String> = target.edge_names.iter().map(|name| common::blend_face_name(&ctx.id, name)).collect();
        let mut extended = Vec::with_capacity(extension.len());
        for (_, point, name) in extension {
            points.push(point);
            names.push(common::blend_face_name(&ctx.id, &name));
            extended.push(name);
        }
        crate::fillet_edges(solid, &points, Some(&names), radius, false, Some(blend_base)).map(|body| (body, extended))
    })
    .ok()
}

/// The refusal for a blend selection none of whose references resolve on
/// the model, shared with the chamfer feature: an invalid input, class
/// `unresolved_reference`, typed at the feature boundary.
pub(super) fn unresolved_refusal(feature: &str, unresolved: &[String]) -> KernelRefusal {
    let names: Vec<String> = unresolved.iter().map(|name| format!("`{name}`")).collect();
    KernelRefusal::input(
        KernelStage::Collect,
        "unresolved_reference",
        format!(
            "{feature}: none of the {} selected reference{} resolves on this model ({}); a blend \
             of nothing is not a result, so the selection is refused — repair the reference{} \
             and rerun",
            unresolved.len(),
            if unresolved.len() == 1 { "" } else { "s" },
            names.join(", "),
            if unresolved.len() == 1 { "" } else { "s" },
        ),
    )
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces/edges drive `edges`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0 || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "F",
    "shortName": "F",
    "longName": "Fillet",
    "ribbonPath": "Home/Blend/Fillet",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the fillet feature"
        },
        "edges": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "EDGE"
            ],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": null,
            "hint": "Select faces (or an edge) to fillet along shared edges"
        },
        "radius": {
            "type": "number",
            "step": 0.1,
            "default_value": 1,
            "hint": "Fillet radius"
        },
        "radiusEnd": {
            "type": "number",
            "step": 0.1,
            "default_value": 0,
            "hint": "Optional end radius for a VARIABLE (tapered) fillet. 0 = constant radius. When > 0 and different from radius, each selected edge tapers from radius at its start to this value at its end (kernel-exact path; corners are not rounded in variable mode)."
        },
        "followTangent": {
            "type": "boolean",
            "default_value": true,
            "label": "Follow tangent",
            "hint": "Grow each selected edge to its whole tangent (G1) chain: pick one edge of a rounded rectangle and the fillet runs around the whole loop. A chain stops at a sharp corner and at a vertex where more than one edge continues it tangentially. The added edges are listed on the feature. New fillets start with it on; a fillet saved before the option existed reads as off."
        },
        "wrapCorners": {
            "type": "boolean",
            "default_value": false,
            "label": "Wrap sharp corners",
            "hint": "Where two selected inside (concave) fillets meet over an unselected sharp outside edge on a curved face (a round boss standing across a block's edge), roll the fillet round that edge instead of meeting in a sharp seam: the corner is closed by the surface the rolling ball sweeps while it stays on the curved face and pivots on the sharp edge. Off by default, which keeps the sharp seam. When a corner cannot be wrapped the fillet is built without the option and a note says so."
        }
    }
})
}

