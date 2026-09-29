//! LOFT — Loft.
//!
//! # Sections resolve FROM THE SCENE (headless)
//!
//! The `extrude.rs` / `revolve.rs` template: the `profiles` `reference_selection`
//! (a `multiple` selection, ORDERED) names 2+ sections, each either a SKETCH
//! whose profile the SKETCH feature already extracted into the scene
//! (`SceneMap::resolve_profile`) or a resident solid FACE turned into the same
//! `SketchProfile` by `face_profile::face_profile` — the two-step extrude,
//! revolve, sweep, path_sweep and sheet-metal tab all run, and the reason
//! `profiles`' filter is `["SKETCH","FACE"]`. A face section consumes nothing
//! and leaves its source solid resident. Each section is that profile's OUTER
//! loop curves (world space); all sections must share the same curve count (a
//! loft rails one curve per station across every section). No marshaled
//! `inputParams.sections`, no caller round-trip.
//!
//! Sections whose planes face OPPOSITE ways carry opposite world windings — the
//! natural "loft between two bodies" pick is one body's top face and another's
//! bottom. `loft_profile_brep` normalizes winding per section against its
//! predecessor, so those rail correctly instead of twisting into a bow-tie; see
//! `construction/loft_topology/basic.rs`.
//!
//! # Name fidelity (contract rule 2)
//!
//! `loft_profile_brep` emits faces UNNAMED, in order `[side faces (INPUT curve
//! order — the builder reverses them back after winding normalization), START cap
//! (= first section), END cap (= last section)]`. We stamp the naming convention
//! byte-for-byte:
//!   - side `i`  → `${edgeName[i]}_LF` from the FIRST section's edge names
//!                 (missing → the `"EDGE"` falsy fallback);
//!   - START cap → `${base}_START`, END cap → `${base}_END`, where
//!                 `base = featureID || "Loft"` (headless-world choice, consistent
//!                 with extrude/revolve `{id}_START/_END`).
//! Loft face names carry NO feature-id prefix on the sides (unlike sweep).
//!
//! # Deferred paths (loud errors, never a silent geometry drop)
//!
//! - a `guide` that resolves to no curve, or to a chain of more than one segment
//!   (the guided loft rides ONE spine curve).
//!
//! A resolvable `guide` runs the §5.8 guided loft: `loft_profile_brep_guided`,
//! or `loft_profile_brep_guided_frame` when `rotateToGuide` is set. Both
//! delegate skins, caps, winding and validation to `loft_profile_brep` and emit
//! one side face per curve index whatever the station count, so the face order
//! this file names is guided-or-not identical.
//! Holed/multi-region profiles loft: every section must share ONE region/loop
//! structure (same region count, same per-region hole count, same per-loop
//! curve count — loud error otherwise); each region lofts across the sections
//! into its own solid (its k-th hole lofts into a cutter subtracted via
//! `common::subtract_region_holes`, `{id}:HOLE:L{loopId}` faces keyed by the hole
//! loop's own stable identity) and the region
//! solids union into ONE result.
//! The schema carries NO `referencePoints`, `reverseFirstLoop`, `loftType` or
//! `guideCurves`. An earlier version of this comment claimed the first two were
//! "consumed UPSTREAM when the sketch sections are ordered" — nothing in the
//! repository ever read any of the four, upstream or here, so the dialog was
//! offering four controls that could not affect the result: a start-vertex
//! picker, a reverse toggle, a one-option dropdown, and a guide-curve picker
//! whose own hint said "(unused)". Section ORDER comes from the `profiles`
//! selection order and each region's own loop ordering; the guided loft is
//! driven by `guide` + `rotateToGuide`, which ARE read (§5.8).

use serde_json::Value;

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult, SketchProfile};
use crate::{
    loft_profile_brep, loft_profile_brep_guided, loft_profile_brep_guided_frame, NurbsCurve,
};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    // GUIDE (§5.8): the spine follows a named curve instead of the straight
    // centroid-to-centroid path. `loft_profile_brep_guided` (translation) and
    // `loft_profile_brep_guided_frame` (`rotateToGuide` — sections rotate into
    // the guide's moving frame) both delegate their skins, shared edges, planar
    // end caps, winding normalization and validation to `loft_profile_brep`, and
    // emit ONE side face per curve index whatever the station count. So the face
    // order the naming below stamps — `[sides…, START, END]` — is the same
    // guided or not, and nothing downstream changes.
    let guide = resolve_guide(ctx)?;
    let rotate_to_guide = matches!(ctx.param("rotateToGuide"), Some(Value::Bool(true)));

    // Ordered section references (a `multiple` reference_selection). Need >= 2.
    // Normalize each `{sketch}:FACE` display-sheet pick to the sketch profile base.
    let names: Vec<String> = common::reference_names(ctx.param("profiles"))
        .into_iter()
        .map(common::normalize_profile_alias)
        .collect();
    if names.len() < 2 {
        return Err(format!(
            "loft: need at least 2 section profiles, got {}",
            names.len()
        ).into());
    }

    // Resolve each section's FULL profile (outer + hole loops). A SKETCH profile
    // first; otherwise a resident FACE becomes one via `face_profile` — the
    // extrude.rs / revolve.rs / sweep.rs / path_sweep.rs two-step, and the reason
    // `profiles`' filter is `["SKETCH","FACE"]`. `from_face` skips the
    // consumed-sketch bookkeeping (a face profile has no sketch behind it).
    let mut section_profiles: Vec<SketchProfile> = Vec::with_capacity(names.len());
    let mut from_face: Vec<bool> = Vec::with_capacity(names.len());
    for name in &names {
        let (profile, face_sourced) = match ctx.scene.resolve_profile(name) {
            Some(profile) => (profile.clone(), false),
            None => match ctx.scene.resolve_face(name) {
                Some(face) => (
                    super::face_profile::face_profile(face)
                        .map_err(|error| format!("loft: {error}"))?,
                    true,
                ),
                None => {
                    return Err(format!(
                        "loft: profile '{name}' not found (no sketch profile or resident face)"
                    ).into());
                }
            },
        };
        let outer = profile
            .regions
            .first()
            .and_then(|region| region.first())
            .ok_or_else(|| format!("loft: profile '{name}' has no outer loop"))?;
        if outer.curves.len() < 2 {
            return Err(format!(
                "loft: profile '{name}' outer loop needs >= 2 curves, got {}",
                outer.curves.len()
            ).into());
        }
        section_profiles.push(profile);
        from_face.push(face_sourced);
    }

    // Every section must share ONE region/loop structure: the same region
    // count, per region the same hole count, and per loop the same curve count
    // (a loft rails one curve per station across every section — for each
    // region's outer wall AND for each lofted hole cutter).
    let region_count = section_profiles[0].regions.len();
    for (index, profile) in section_profiles.iter().enumerate() {
        if profile.regions.len() != region_count {
            return Err(format!(
                "loft: every section must have the same loop structure; section 0 has \
                 {region_count} region(s), section {index} ('{}') has {}",
                names[index],
                profile.regions.len()
            ).into());
        }
    }
    for region_index in 0..region_count {
        let loop_count = section_profiles[0].regions[region_index].len();
        for (index, profile) in section_profiles.iter().enumerate() {
            if profile.regions[region_index].len() != loop_count {
                return Err(format!(
                    "loft: every section must have the same loop structure; in region \
                     {region_index} section 0 has {} hole loop(s), section {index} ('{}') has {}",
                    loop_count - 1,
                    names[index],
                    profile.regions[region_index].len() - 1
                ).into());
            }
        }
        for loop_index in 0..loop_count {
            let count = section_profiles[0].regions[region_index][loop_index].curves.len();
            for (index, profile) in section_profiles.iter().enumerate() {
                let got = profile.regions[region_index][loop_index].curves.len();
                if got != count {
                    return Err(format!(
                        "loft: all sections must have the same curve count in loop {loop_index}; \
                         section 0 has {count}, section {index} ('{}') has {got}",
                        names[index]
                    ).into());
                }
            }
        }
    }

    // Base name: `featureID || "Loft"`.
    let base = if ctx.id.is_empty() { "Loft" } else { ctx.id.as_str() };

    // Per-loop cap names keyed off each region's STABLE loop identity (taken from
    // the FIRST section, the one whose loops the walls are named from), with the
    // un-keyed `{base}_START/_END` as the containers (see `common::CapNames`).
    let caps = common::CapNames::new(base, &section_profiles[0].regions);

    // One lofted solid per REGION (railed across every section), unioned below.
    let mut region_solids = Vec::with_capacity(region_count);
    for region_index in 0..region_count {
        let first_region = &section_profiles[0].regions[region_index];
        let first_edge_names: Vec<Option<String>> = first_region[0].edge_names.clone();
        let sections: Vec<Vec<NurbsCurve>> = section_profiles
            .iter()
            .map(|profile| profile.regions[region_index][0].curves.clone())
            .collect();
        let side_count = sections[0].len();

        // Exact kernel loft (unnamed faces), order `[sides…, START, END]`.
        let mut solid = loft_sections(&sections, guide.as_ref(), rotate_to_guide)?;

        // Author the established name convention onto the emitted faces.
        let mut face_names: Vec<String> = Vec::with_capacity(side_count + 2);
        for index in 0..side_count {
            let edge = first_edge_names
                .get(index)
                .cloned()
                .flatten()
                .unwrap_or_else(|| "EDGE".to_string());
            face_names.push(format!("{edge}_LF"));
        }
        let (start, end) = &caps.per_loop[region_index];
        face_names.push(start.clone());
        face_names.push(end.clone());

        let faces = &mut solid
            .shells
            .get_mut(0)
            .ok_or("loft builder produced no shell")?
            .faces;
        if faces.len() != face_names.len() {
            return Err(format!(
                "loft builder produced {} faces, expected {} ({} sides + 2 caps)",
                faces.len(),
                face_names.len(),
                side_count
            ).into());
        }
        for (face, name) in faces.iter_mut().zip(&face_names) {
            face.name = Some(name.clone());
        }

        // Region holes: loft the k-th hole loop ACROSS the sections into a
        // cutter and subtract (containment from the FIRST section's region).
        // The cutter's end caps land coplanar with the outer loft's caps; the
        // exact boolean's coplanar merge handles them.
        if first_region.len() > 1 {
            solid = common::subtract_region_holes(
                solid,
                &section_profiles[0],
                first_region,
                "loft",
                base,
                None,
                &mut |loop_index, _depth| {
                    let hole_sections: Vec<Vec<NurbsCurve>> = section_profiles
                        .iter()
                        .map(|profile| profile.regions[region_index][loop_index].curves.clone())
                        .collect();
                    loft_sections(&hole_sections, guide.as_ref(), rotate_to_guide)
                },
            )?;
        }
        region_solids.push(solid);
    }
    let solid = common::union_region_solids(region_solids)
        .map_err(|error| format!("loft: {error}"))?;

    // Role metadata: sidewall/cap convention, from the actual named faces.
    common::stamp_sweep_roles_multi(&solid, "_LF", &caps.starts(), &caps.ends());

    // Optional boolean fold + result.
    let mut result = common::finalize_solid_grouped(ctx, solid, base, &caps.containers());
    if result.error.is_some() {
        return Ok(result);
    }
    // Consume each referenced sketch (default consumeProfileSketch) — signal the caller to
    // drop its display group. Sketches register no scene solid, so this is a
    // display-only signal.
    for (index, name) in names.iter().enumerate() {
        // A FACE section has no sketch to consume — its source solid stays resident.
        if from_face[index] {
            continue;
        }
        common::consume_sketch(ctx, name, &mut result);
    }
    Ok(result)
}

/// True iff a usable `guide` is present — ANY non-null, non-empty value (a `{name}`
/// reference OR an old marshaled curve object both count). Absent / `null` / empty
/// string / empty (or all-null) array → no guide. Deliberately broader than
/// `first_reference_name` so a non-name-shaped guide can't slip through un-deferred.
fn has_guide(ctx: &FeatureContext) -> bool {
    match ctx.param("guide") {
        None | Some(serde_json::Value::Null) => false,
        Some(serde_json::Value::String(text)) => !text.trim().is_empty(),
        Some(serde_json::Value::Array(items)) => items.iter().any(|value| !value.is_null()),
        Some(_) => true,
    }
}

/// The `guide` reference resolved to its ONE spine curve, or `None` when no
/// guide is selected. A named-but-unresolvable guide, or one that resolves to a
/// multi-segment chain, is a loud error — never a silent un-guided loft.
fn resolve_guide(ctx: &FeatureContext) -> Result<Option<NurbsCurve>, String> {
    if !has_guide(ctx) {
        return Ok(None);
    }
    let name = common::first_reference_name(ctx.param("guide"))
        .ok_or("loft: `guide` is set but names nothing selectable")?;
    let chain = common::resolve_path(ctx, &name).map_err(|error| format!("loft: {error}"))?;
    match chain.len() {
        1 => Ok(Some(chain.into_iter().next().expect("one curve"))),
        0 => Err(format!("loft: guide '{name}' resolved to no curves").into()),
        other => Err(format!(
            "loft: guide '{name}' is a {other}-segment chain; the guided loft rides ONE spine \
             curve — pick a single edge, or a sketch whose open chain is one segment"
        ).into()),
    }
}

/// Loft one stack of sections: straight when there is no guide, along the guide
/// otherwise, rotating into the guide's moving frame when `rotate_to_guide`.
/// Every branch emits the same face order (see the call site).
fn loft_sections(
    sections: &[Vec<NurbsCurve>],
    guide: Option<&NurbsCurve>,
    rotate_to_guide: bool,
) -> Result<crate::BrepSolid, crate::KernelRefusal> {
    match (guide, rotate_to_guide) {
        (None, _) => loft_profile_brep(sections),
        (Some(guide), false) => loft_profile_brep_guided(sections, guide, None),
        (Some(guide), true) => loft_profile_brep_guided_frame(sections, guide, None),
    }
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected profiles (faces/sketches) drive `profiles`, edges the guides.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.has_profile() || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "LOFT",
    "shortName": "LOFT",
    "longName": "Loft",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the loft feature"
        },
        "profiles": {
            "type": "reference_selection",
            "selectionFilter": [
                "SKETCH",
                "FACE"
            ],
            "multiple": true,
            "default_value": [],
            "hint": "Select 2+ profiles (faces) to loft"
        },
        "consumeProfileSketch": {
            "type": "boolean",
            "default_value": true,
            "hint": "Remove referenced sketches after creating the loft. Turn off to keep them in the scene."
        },
        "guide": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE"
            ],
            "multiple": false,
            "default_value": null,
            "label": "Guide curve",
            "hint": "Optional single guide curve (§5.8): the loft's spine follows it, bending the sections along the guide instead of the straight centroid-to-centroid path."
        },
        "rotateToGuide": {
            "type": "boolean",
            "default_value": false,
            "label": "Rotate sections to guide",
            "hint": "With a guide: rotate the sections into the guide's moving frame (like a sweep) instead of keeping their own orientation. Sections are still hit exactly at their own stations."
        },
        "boolean": common::optional_boolean_schema()
    }
})
}

