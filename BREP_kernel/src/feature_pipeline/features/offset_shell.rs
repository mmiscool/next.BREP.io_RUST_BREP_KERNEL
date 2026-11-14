//! O.S — Offset Shell. Hollow a solid by removing a set of OPENING faces and
//! offsetting the retained face supports by a signed distance.
//!
//! Semantics:
//!
//! - `inputParams.faces` — ONE `reference_selection` admitting FACE and SOLID
//!   names (owner 2026-11-01): the faces are the OPENINGS to remove; a solid
//!   names the body to shell.
//! - `inputParams.distance` — the signed offset applied to the retained face
//!   supports. POSITIVE = INWARD (the kernel's own
//!   `open_top_box_offsets_to_exact_hollow_volume` proves +0.5 hollows a 4³ box
//!   to 32.5); NEGATIVE = OUTWARD (the retained faces grow away from the
//!   material and the opening walls are the opening planes extended by the
//!   thickness — `outward_shell_open_top_box_has_exact_grown_wall_volume`);
//!   the kernel op takes the signed value verbatim.
//!   A solid alone builds the CLOSED hollow (outer shell plus inner void,
//!   sharp joins; `offset_shell_closed`); with opening faces they must lie on
//!   it. Two solids, faces on two solids, or faces off the named solid refuse
//!   by name. Nothing picked is the panel's empty state: no shell, and a
//!   `NothingSelected` fulfilment says so. A document saved with the former
//!   separate `inputParams.solid` (2026-10-31) folds it into `faces` on read.
//! - `inputParams.replaceOriginalSolid` — `!= false` (default true) removes the
//!   source solid, leaving only the shell.
//!
//! Resolution (contract rule 1): every opening-face name resolves against the
//! live scene-map by exact match. ALL-OR-NOTHING — a single miss (or an
//! empty selection, or a zero/non-finite distance) yields an
//! empty result (with `unresolved` populated on a name miss) so the caller repairs +
//! re-dispatches, matching the established soft no-op / throw-then-repair split. The
//! kernel's "cannot produce a shell" refusal (every face selected) is likewise a
//! soft no-op. Any OTHER kernel error is a hard
//! failure that halts the loop (the caller rethrows).
//!
//! Name fidelity (contract rule 2): the result face names are derived from the
//! kernel's `face_images` — which the kernel returns POSITIONALLY PARALLEL to the
//! solid's shell/face iteration order — NOT from kernel-carried `face.name`, so
//! this replicates the offset-shell naming byte-for-byte:
//! a Source image keeps its source face name, an
//! Offset image appends `_Offset`, an opening Wall inherits its removed face's
//! name, and collisions dedup as `base`, `base_1`, `base_2`, …. A shell that
//! REPLACES its source keeps the source solid's name (since 2026-11-07,
//! when the name changed on every shell); one that keeps the source is
//! `${targetName}_${featureId}`, and — like every copying feature (mirror,
//! transform copy) — suffixes its face and edge names with `::{featureId}`
//! (`cube_NZ::shell`), since the kept source still owns the plain names and
//! the scene refuses two live solids sharing one. A replacing shell also answers to that
//! former name (`solid_aliases`), which documents saved before 2026-11-07
//! spell in later features. One wall is named differently on purpose: the
//! ring a through-bore exposes at an opening (the thickness between the hole
//! wall and its offset) is named after the HOLE WALL (`Pin_S` → `Pin_S_1`),
//! not the opening — the ring is the bore's, there is one per bore end, and
//! the kernel assigns that provenance whichever construction built the ring
//! (`offset_shell::claim_bore_end_rings`), so a saved reference to it survives.

use std::collections::HashMap;

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult};
use crate::OffsetFaceRole;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

/// True for the derived `{faceA}|{faceB}[n]` edge-name shape, optionally with
/// a deterministic `_k` split-fragment suffix. Authored names never match.
fn is_derived_edge_name(name: &str) -> bool {
    if !name.contains('|') {
        return false;
    }
    let Some(bracket) = name.rfind('[') else {
        return false;
    };
    let rest = &name[bracket + 1..];
    let Some(close) = rest.find(']') else {
        return false;
    };
    let index = &rest[..close];
    if index.is_empty() || !index.chars().all(|c| c.is_ascii_digit()) {
        return false;
    }
    let tail = &rest[close + 1..];
    tail.is_empty()
        || (tail.len() > 1
            && tail.starts_with('_')
            && tail[1..].chars().all(|c| c.is_ascii_digit()))
}

/// Unique-name dedup: first occurrence = base, then
/// `base_1`, `base_2`, ….
fn unique_name(counts: &mut HashMap<String, usize>, base: String) -> String {
    let count = counts.entry(base.clone()).or_insert(0);
    let name = if *count == 0 {
        base.clone()
    } else {
        format!("{base}_{count}")
    };
    *count += 1;
    name
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // Distance: no-op on NaN / 0 (a soft warn,
    // not a throw). Treat a missing / malformed
    // / zero / non-finite distance as an empty result, not a hard error.
    let distance = match ctx.number("distance") {
        Ok(distance) if distance.is_finite() && distance != 0.0 => distance,
        _ => return Ok(result),
    };

    // ONE reference field (owner 2026-11-01): `faces` admits FACE and SOLID
    // names. Faces are the openings; a solid names the body to shell, which
    // with no opening face asks for the CLOSED hollow (lane O row 34).
    // Documents saved with the separate `solid` field (2026-10-31 to
    // 2026-11-01) fold it in here.
    let mut names = common::unique_reference_names(ctx.param("faces"));
    if let Some(legacy) = common::first_reference_name(ctx.param("solid")) {
        if !names.contains(&legacy) {
            names.push(legacy);
        }
    }
    if names.is_empty() {
        // Nothing picked yet (the panel's empty state): no shell, and the
        // result says so instead of reading as a plain success.
        result.fulfilment = Some(crate::feature_pipeline::Fulfilment {
            requested: Vec::new(),
            applied: Vec::new(),
            rejected: vec![crate::feature_pipeline::Rejected {
                name: "faces".to_string(),
                kind: crate::feature_pipeline::RejectedKind::NothingSelected,
                reason: "names no opening face and no solid, so nothing was shelled".to_string(),
            }],
            extended: Vec::new(),
            followed: Vec::new(),
        });
        return Ok(result);
    }
    // Resolve every name BY NAME against the live scene-map: an exact face
    // name, then a solid name, then a one-face container (`resolve_face`'s
    // fallback), so a solid is never read as a container of the same name.
    // ALL-OR-NOTHING: any miss records `unresolved` and produces no shell (a
    // partial opening set is a different shell than requested).
    let mut opening = Vec::new();
    let mut face_handles = Vec::new();
    let mut solids: Vec<(String, u32)> = Vec::new();
    for name in &names {
        let exact_face = ctx.scene.faces.get(name).copied();
        let solid = exact_face.is_none().then(|| ctx.scene.resolve_solid(name)).flatten();
        if let Some(handle) = solid {
            if !solids.iter().any(|(_, known)| *known == handle) {
                solids.push((name.clone(), handle));
            }
        } else if let Some(face) = exact_face.or_else(|| ctx.scene.resolve_face(name)) {
            if !face_handles.contains(&face.handle) {
                face_handles.push(face.handle);
            }
            opening.push(face);
        } else {
            result.unresolved.push(name.clone());
        }
    }
    if !result.unresolved.is_empty() {
        return Ok(result);
    }
    if solids.len() > 1 {
        return Err(crate::KernelRefusal::input(
            crate::KernelStage::Collect,
            "offset_shell_two_solids",
            format!(
                "offset_shell: the selection names {} solids ({}); a shell is built from one solid",
                solids.len(),
                solids.iter().map(|(name, _)| format!("`{name}`")).collect::<Vec<_>>().join(", ")
            ),
        )
        .into());
    }
    if face_handles.len() > 1 {
        return Err(crate::KernelRefusal::input(
            crate::KernelStage::Collect,
            "offset_shell_faces_on_two_solids",
            format!(
                "offset_shell: the opening faces lie on {} different solids; pick openings on one solid",
                face_handles.len()
            ),
        )
        .into());
    }
    let handle = match (solids.first(), face_handles.first()) {
        (Some((solid_name, named)), Some(on_faces)) if named != on_faces => {
            return Err(crate::KernelRefusal::input(
                crate::KernelStage::Collect,
                "offset_shell_faces_off_solid",
                format!("offset_shell: the opening faces are not on the named solid `{solid_name}`"),
            )
            .into())
        }
        (Some((_, named)), _) => *named,
        (None, Some(on_faces)) => *on_faces,
        (None, None) => unreachable!("every name resolved to a face or a solid"),
    };
    let opening_ids = opening.iter().map(|face| face.face_id).collect::<Vec<u64>>();

    // The target solid's scene name — the caller names the result `${targetSolid.name}_…`.
    let target_name = ctx
        .scene
        .solids
        .iter()
        .find(|(_, resident)| **resident == handle)
        .map(|(name, _)| name.clone())
        .ok_or_else(|| {
            crate::KernelRefusal::internal(
                crate::KernelStage::Collect,
                "offset_shell_no_resident_solid",
                "offset_shell: opening faces resolve to no scene-resident solid",
            )
        })?;

    // Short borrow: capture the source face id → name map and run the op in one
    // borrow, then release the registry (contract handle discipline).
    let (source_names, record) = crate::with_registered_solid_typed(handle, |solid| {
        let source_names: HashMap<u64, String> = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter_map(|face| face.name.as_ref().map(|name| (face.id, name.clone())))
            .collect();
        let record = if opening_ids.is_empty() {
            crate::offset_shell_closed(solid, distance)
        } else {
            crate::offset_shell(solid, &opening_ids, distance)
        };
        Ok((source_names, record))
    })?;
    let record = match record {
        Ok(record) => record,
        // Every face selected → no support left to offset: warn + no-op. Keyed
        // on the refusal's class and slug, never its text.
        Err(error) if crate::is_offset_shell_no_retained_face(&error) => return Ok(result),
        // The opening-wall machinery still cannot close a watertight shell around
        // SOME openings whose boundary is a complex CURVED (non-ruled) cut — e.g. a
        // tangent rim fillet (A3c). (Sphere-carved, cone-crater, partial-arc, and
        // cylinder/cone-lateral openings shell watertight inward; sphere-gouged
        // and conic-crater openings, adjacent openings, through-bores and
        // corner-blended edge fillets also shell watertight OUTWARD.) When the
        // wall along such a rim is neither ruled nor planar the completion
        // leaves the shell open (a non-integral genus / unwelded-rim refusal);
        // give the user an ACTIONABLE message + the reorder workaround instead
        // of the raw genus dump.
        // Keyed on the class: the assembly's `NonIntegralGenus` (carried
        // through every cause the shell puts in front of it) or the unwelded
        // curved rim (`is_offset_shell_open_curved_rim`).
        Err(error) if crate::is_offset_shell_open_curved_rim(&error) => {
            return Err(error
                .with_message(|error| {
                    format!(
                        "offset shell could not close a watertight shell around the selected opening. \
                         Some openings carved by a complex CURVED cut (e.g. a rim-tangent fillet) \
                         are not yet supported — the wall along such a rim is neither ruled nor \
                         planar. Fix: run the offset shell EARLIER in the history, before that cut \
                         (reorder it above the curved boolean). (kernel detail: {error})"
                    )
                })
                .into());
        }
        // Any other kernel refusal is a genuine failure (the caller rethrows).
        Err(error) => return Err(error.into()),
    };

    let mut shell = record.solid;
    let images = record.face_images;

    // Faithful naming: walk the shell's faces in iteration order (the kernel
    // returns `face_images` positionally parallel to this order — do NOT re-index
    // by `face.id`, that mapping is already applied) and stamp each from its
    // image's role + source-face name.
    let mut counts: HashMap<String, usize> = HashMap::new();
    let mut face_index = 0usize;
    for shell_record in &mut shell.shells {
        for face in &mut shell_record.faces {
            let image = images.get(face_index).ok_or_else(|| {
                "offset_shell: result face has no matching image (provenance lost)".to_string()
            })?;
            let source_name = source_names
                .get(&image.source_face_id)
                .cloned()
                .unwrap_or_else(|| format!("{target_name}_Face"));
            let base = match image.role {
                OffsetFaceRole::Offset => format!("{source_name}_Offset"),
                OffsetFaceRole::Source | OffsetFaceRole::Wall => source_name,
            };
            face.name = Some(unique_name(&mut counts, base));
            face_index += 1;
        }
    }

    let feature_id = if ctx.id.is_empty() {
        ctx.feature_type.as_str()
    } else {
        ctx.id.as_str()
    };
    // `replaceOriginalSolid !== false` (default true) consumes the source
    // solid, and the shell takes its place under the SAME name, as every other
    // in-place feature does (fillet, chamfer, push face): removed-then-added.
    let replace = ctx
        .param("replaceOriginalSolid")
        .and_then(|value| value.as_bool())
        .unwrap_or(true);
    let legacy_name = format!("{target_name}_{feature_id}");
    let new_name = if replace { target_name.clone() } else { legacy_name.clone() };
    // The offset op splits opening-rim edges, so PROPAGATED derived names come
    // through as stale `{A}|{B}[0]_2` fragments while references (and the caller
    // display) use names derived from the FINAL face adjacency. Clear the
    // derived-format names (authored names are kept) and let register_added's
    // stamper re-derive every unnamed edge — without that stamp no post-shell
    // edge reference (revolve axis / fillet on the shell rim) can resolve.
    for edge in &mut shell.edges {
        if edge.name.as_deref().is_some_and(is_derived_edge_name) {
            edge.name = None;
        }
    }
    // A shell beside its kept source would carry the source's face names
    // (`Source` images) and fail the scene's naming contract; namespace it
    // the way the copy features do.
    if !replace {
        common::namespace_copy_names(&mut shell, feature_id);
    }
    result.added.push(common::register_added(shell, &new_name));
    if replace {
        result.solid_aliases.push((legacy_name, target_name.clone()));
        result.removed.push(target_name);
    }
    Ok(result)
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces drive `faces`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "O.S",
    "shortName": "O.S",
    "longName": "Offset Shell",
    "ribbonPath": "Home/Modeling/Offset Shell",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier used when naming the generated solid and faces"
        },
        "distance": {
            "type": "number",
            "default_value": 1,
            "hint": "Signed distance applied to the retained BREP face supports."
        },
        "faces": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "SOLID"
            ],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": [],
            "hint": "Pick the faces to remove as shell openings, or a solid alone to shell it closed (a hollow with an inner void). Faces and their solid together shell that solid with those openings."
        },
        "replaceOriginalSolid": {
            "type": "boolean",
            "label": "REPLACE ORIGINAL SOLID",
            "default_value": true,
            "hint": "When enabled, remove the source solid and leave only the shell result in the scene."
        }
    }
})
}

