//! E — Extrude. Ported from the retired `Extrude` feature.
//!
//! A closed planar profile swept `distance` along the profile-plane normal —
//! straight, two-sided (`distanceBack`), or drafted (`draftAngle`: a tapered
//! frustum lofted by the kernel).
//!
//! Profile source (post-cutover): the `profile` `reference_selection` names a
//! SKETCH whose profile the SKETCH feature already extracted into the scene
//! (`SceneMap::resolve_profile`), OR a resident solid FACE — resolved via
//! `scene.resolve_face` and turned into the same [`SketchProfile`] shape by
//! `face_profile::face_profile`. Either way
//! extrude reads resolved loops + a plane frame — no marshaled curves, no
//! round-trip. A face profile consumes nothing (there is no sketch to remove).
//!
//! Name fidelity (contract rule 2), the standard (sketch-profile) extrude naming:
//! - each outer-loop curve -> sidewall `${featureID}:${edgeName}_SW` (the
//!   profile's carried `{sketchId}:G{gid}` names; face-profile edges carry the
//!   source edge name, fallback `EDGE`), in input-curve order;
//! - caps `${featureID}:${sketchId}:PROFILE:L{loopId}_START` (the base cap, at
//!   `-distanceBack` along the normal) and `..._END` (at `+distance`), ONE PAIR
//!   PER PROFILE REGION, keyed by the region's outer loop's stable identity (see
//!   the sketch feature's `loop_ids`). The key is what stops a cap name from
//!   moving to a different island when a DIFFERENT loop is added to or removed
//!   from the sketch — before it, every region stamped the same name and the
//!   post-union dedup pass disambiguated them positionally. The un-keyed
//!   `${featureID}:${sketchId}:PROFILE_START/_END` is registered as the CONTAINER
//!   for all of them, so a reference stored before per-loop naming still resolves
//!   (to the one cap of a single-loop sketch, to all of them otherwise). For a
//!   FACE profile the cap base is `${featureID}:${faceName}` (the base-name
//!   rule's `:PROFILE`-suffix branch strips-then-reappends, so the face name
//!   always passes through verbatim) and, being single-region with no sketch loop
//!   behind it, it keeps the UN-KEYED spelling.
//!
//! DRAFT path (`draftAngle` != 0, in DEGREES; positive tapers INWARD → smaller
//! far cap — the draft sign convention): the
//! kernel `extrude_profile_brep_draft` builds the outer loop's EXACT drafted
//! walls directly — tilted planes for lines, cone patches for circular arcs,
//! exact conic wall∧wall junction edges — with the far section the exact
//! in-plane miter offset (d = distance·tanθ; lines re-intersect at corners,
//! arcs shrink concentrically). An offset that exceeds the local feature size
//! (miter/arc intersections vanish) is a loud kernel Err. TWO-SIDED draft
//! (`distanceBack` > 0) is the MOLD / PARTING-LINE form: the sketch plane is
//! the parting plane and positive draft tapers inward moving away from it in
//! EACH direction (one leg by `distance`, one by `distanceBack`, unioned; the
//! wall-normal break leaves a real ridge edge at the sketch plane). Draft
//! names follow the established DRAFT convention, NOT the sweep one: walls
//! `${edgeName}_E` (no feature-id prefix; a two-sided draft has one wall per
//! leg per edge), caps `${featureID}_START` (at −distanceBack) /
//! `${featureID}_END` (at +distance) (plain feature id even for a FACE
//! profile — the draft path never used a source-profile base name).
//! Combinations the retired draft path never supported are LOUD errors here
//! instead of the retired silent fallback to a straight extrude: draft + hole
//! loops (the retired path dropped them on the floor), draft + multiple regions (the retired path could
//! not ring-order them), draft + zero total span.
//!
//! Two-sided extrude (`distanceBack`, the schema DEFAULT): the profile plane is
//! translated `-distanceBack` along the normal and the solid swept the total
//! `distance + distanceBack` — the two-sided sweep
//! behavior. The anti-z-fighting boolean biases (±1e-5 nudges)
//! are NOT ported: the exact kernel boolean handles coplanar faces.
//!
//! Each profile REGION extrudes to its own prism (outer loop swept, hole loops
//! subtracted as through prisms — the sheet-metal cutout pattern) and the
//! region prisms are unioned into ONE solid (the established multi-region
//! behavior). Hole cutter faces are stamped `{id}:HOLE:L{loopId}` from the hole
//! loop's own identity, for the same reason the caps are keyed.
//! The optional `boolean` param folds via `common::finalize_solid`.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult, SketchProfile};
use crate::{extrude_profile_brep, extrude_profile_brep_draft, BrepSolid, NurbsCurve, Vec3};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    // Draft angle in DEGREES (`draftAngle`); 0 / absent = straight extrude.
    let draft_deg = optional_number(ctx, "draftAngle")?;

    // A committed sketch is picked as `{sketch}:FACE` (its render-side display sheet);
    // normalize to the sketch PROFILE base — see `common::normalize_profile_alias`.
    let name = common::normalize_profile_alias(
        common::first_reference_name(ctx.param("profile")).ok_or("extrude: missing `profile` reference selection")?,
    );
    // Sketch profile first; otherwise a resident FACE becomes a profile via
    // `face_profile` (same shape, same downstream path). `from_face` gates the
    // cap-name base and skips the consumed-sketch bookkeeping.
    let mut face_profile_storage: Option<SketchProfile> = None;
    let (profile, from_face): (&SketchProfile, bool) = match ctx.scene.resolve_profile(&name) {
        Some(profile) => (profile, false),
        None => match ctx.scene.resolve_face(&name) {
            Some(face) => {
                let built = super::face_profile::face_profile(face)
                    .map_err(|error| format!("extrude: {error}"))?;
                (&*face_profile_storage.insert(built), true)
            }
            None => {
                return Err(format!(
                    "extrude: profile '{name}' not found (no sketch profile or resident face)"
                ).into());
            }
        },
    };

    // Two-sided span: sweep from `-distanceBack` to `+distance` along the plane
    // normal (the two-sided `{distance, distanceBack}` sweep behavior; back is the
    // schema default 10, so this IS the standard extrude).
    let distance = ctx.number("distance")?;
    let back = optional_number(ctx, "distanceBack")?;
    let direction = profile.z_axis;

    let (solid, caps) = if draft_deg.abs() > 1e-12 {
        build_drafted(ctx, profile, from_face, &name, direction, distance, back, draft_deg)?
    } else {
        build_straight(ctx, profile, from_face, &name, direction, distance, back)?
    };

    // Optional boolean fold + result. The cap CONTAINERS ride along so the
    // un-keyed `{cap_base}_START/_END` a pre-per-loop-naming model stored still
    // resolves — to the one cap on a single-loop sketch, to all of them on a
    // multi-loop one.
    let mut result = common::finalize_solid_grouped(ctx, solid, &ctx.id, &caps.containers());
    if result.error.is_some() {
        return Ok(result);
    }
    // Consume the referenced sketch (default true) — signal the caller to drop its display
    // group. Sketches register no scene solid, so this is a no-op in the scene-map
    // (`SceneMap::apply` ignores a removed name it doesn't hold). A FACE profile
    // has nothing to consume — the source solid stays resident.
    if !from_face && common::consume_profile_sketch(ctx) {
        let sketch_name = name
            .strip_suffix(":PROFILE")
            .unwrap_or(&name)
            .to_string();
        if !result.removed.contains(&sketch_name) {
            result.removed.push(sketch_name);
        }
    }
    Ok(result)
}

/// STRAIGHT / two-sided extrude: one prism per region (outer swept, holes
/// subtracted as through prisms), unioned into one solid, with sweep
/// names (`{id}:{edge}_SW` walls, `{cap_base}_START/_END` caps) + role stamps.
fn build_straight(
    ctx: &FeatureContext,
    profile: &SketchProfile,
    from_face: bool,
    name: &str,
    direction: Vec3,
    distance: f64,
    back: f64,
) -> Result<(BrepSolid, common::CapNames), FeatureRefusal> {
    let total = distance + back;
    if total.abs() <= 1e-9 {
        return Err("extrude: total extrusion length (distance + distanceBack) is zero".into());
    }
    let dir = direction.normalized()?;
    let base_offset = dir.scale(-back);

    // Names byte-match the established Sweep: walls `{id}:{edge}_SW`, caps
    // `{cap_base}_START/_END` (base cap at −back, END at +distance) where
    // `cap_base` is `{id}:{sketchBase}:PROFILE` for a sketch profile and
    // `{id}:{faceName}` for a face profile (the source-profile base name).
    let tag = if ctx.id.is_empty() {
        String::new()
    } else {
        format!("{}:", ctx.id)
    };
    let cap_base = if from_face {
        format!("{tag}{name}")
    } else {
        let sketch_base = name.strip_suffix(":PROFILE").unwrap_or(name);
        format!("{tag}{sketch_base}:PROFILE")
    };

    // Per-loop cap names keyed off each region's STABLE loop identity, with the
    // un-keyed `{cap_base}_START/_END` registered as the containers standing for
    // all of them (see `common::CapNames`). Keying is what stops a cap name from
    // moving to a different island when a DIFFERENT loop is added or removed.
    let caps = common::CapNames::new(&cap_base, &profile.regions);

    // One prism per REGION (outer + its holes), unioned below into one solid.
    let mut region_solids = Vec::with_capacity(profile.regions.len());
    for (region_index, region) in profile.regions.iter().enumerate() {
        let outer = region.first().ok_or("extrude: profile has no outer loop")?;
        let count = outer.curves.len();
        if count < 2 {
            return Err(format!("extrude: outer loop needs >= 2 curves, got {count}").into());
        }
        let outer_curves: Vec<NurbsCurve> = if back.abs() > 1e-12 {
            outer
                .curves
                .iter()
                .map(|curve| common::translate_curve(curve, base_offset))
                .collect::<Result<_, _>>()?
        } else {
            outer.curves.clone()
        };

        let mut solid = extrude_profile_brep(&outer_curves, direction, total)?;

        // Face order (sweep_topology.rs): [sidewalls in INPUT order..., bottom, top].
        {
            let faces = &mut solid
                .shells
                .get_mut(0)
                .ok_or("extrude builder produced no shell")?
                .faces;
            if faces.len() != count + 2 {
                return Err(format!(
                    "extrude builder produced {} faces, expected {}",
                    faces.len(),
                    count + 2
                ).into());
            }
            for (index, face) in faces.iter_mut().take(count).enumerate() {
                let raw = outer
                    .edge_names
                    .get(index)
                    .cloned()
                    .flatten()
                    .unwrap_or_else(|| "EDGE".to_string());
                face.name = Some(format!("{tag}{raw}_SW"));
            }
            let (start, end) = &caps.per_loop[region_index];
            faces[count].name = Some(start.clone());
            faces[count + 1].name = Some(end.clone());
        }
        // Subtract each hole loop as a through prism (translated to the same
        // base). The cutter is named from the hole loop's OWN stable identity, so
        // adding or removing one hole never renames the walls of another (the cap
        // bug's hole-wall sibling — the old name counted holes positionally
        // across regions).
        for (loop_index, hole) in region.iter().enumerate().skip(1) {
            let hole_curves: Vec<NurbsCurve> = if back.abs() > 1e-12 {
                hole.curves
                    .iter()
                    .map(|curve| common::translate_curve(curve, base_offset))
                    .collect::<Result<_, _>>()?
            } else {
                hole.curves.clone()
            };
            solid = common::subtract_hole_prism(
                "extrude",
                solid,
                &hole_curves,
                direction,
                total,
                &ctx.id,
                &common::hole_key(hole, loop_index),
            )?;
        }
        region_solids.push(solid);
    }
    let solid = common::union_region_solids(region_solids)
        .map_err(|error| format!("extrude: {error}"))?;

    // Role metadata (the established Sweep sidewall/cap convention the PMI dimension UI
    // reads) — stamped from the faces that actually survived hole subtraction.
    common::stamp_sweep_roles_multi(&solid, "_SW", &caps.starts(), &caps.ends());
    Ok((solid, caps))
}

/// DRAFTED extrude (`draft_deg` != 0, degrees; positive tapers INWARD): the
/// kernel builds the outer loop's exact drafted walls directly
/// (`extrude_profile_brep_draft` — planes for lines, cone patches for arcs,
/// exact conic junction edges). Single region, no holes — the combinations the
/// the retired draft path never supported are loud errors (see the module doc).
///
/// TWO-SIDED (`distanceBack` > 0) uses MOLD / PARTING-LINE semantics (the
/// SolidWorks two-direction draft): the sketch plane is the parting plane and
/// positive draft tapers INWARD moving away from it in EACH direction — one
/// drafted leg along +normal by `distance`, one along −normal by
/// `distanceBack`, both at the same draft angle, unioned across the shared
/// profile-plane cross-section. The internal coplanar caps merge away; the
/// wall-normal break at the sketch plane remains as a REAL ridge edge.
///
/// NAMING IS THE STRAIGHT PATH'S, unchanged by draft (2026-08-18 decision —
/// the legacy separate `_E`/`{id}_START` draft convention renamed every face
/// when draftAngle toggled, breaking downstream references): walls
/// `{id}:{edge}_SW` in input order, caps `{cap_base}_START` (at
/// −distanceBack) / `{cap_base}_END` (at +distance) where `cap_base` is
/// `{id}:{sketchBase}:PROFILE` for a sketch profile and `{id}:{faceName}`
/// for a face profile. A zero `distanceBack` is the plain one-sided path.
#[allow(clippy::too_many_arguments)]
fn build_drafted(
    ctx: &FeatureContext,
    profile: &SketchProfile,
    from_face: bool,
    name: &str,
    direction: Vec3,
    distance: f64,
    back: f64,
    draft_deg: f64,
) -> Result<(BrepSolid, common::CapNames), FeatureRefusal> {
    if back < -1e-12 {
        return Err("extrude: draft (`draftAngle`) needs `distanceBack` >= 0".into());
    }
    let two_sided = back > 1e-12;
    if !two_sided && distance.abs() <= 1e-9 {
        return Err("extrude: draft (`draftAngle`) needs a non-zero `distance`".into());
    }
    if two_sided && distance < -1e-9 {
        return Err(
            "extrude: two-sided draft (`draftAngle` + `distanceBack`) needs `distance` >= 0 — \
             both spans leave the sketch plane in opposite directions"
                .into(),
        );
    }
    // Several disjoint REGIONS draft the way they extrude: one prism each,
    // unioned. Holes are a different problem and still refuse — a drafted hole
    // cutter has to taper the OTHER way and over-run both ends, and getting that
    // sign wrong yields a watertight, validating, WRONG solid.
    if let Some(holed) = profile.regions.iter().find(|region| region.len() > 1) {
        return Err(format!(
            "extrude: draft (`draftAngle`) does not support profiles with holes \
             ({} hole loop(s) in the profile); several separate regions do draft",
            holed.len() - 1
        ).into());
    }

    // Straight-path naming bases (see build_straight): `{id}:` tag, cap_base
    // from the profile source. feature_base only seeds the builder's internal
    // labels and the two-sided internal-cap sentinel.
    let feature_base = if ctx.id.is_empty() { "Extrude" } else { ctx.id.as_str() };
    let tag = if ctx.id.is_empty() {
        String::new()
    } else {
        format!("{}:", ctx.id)
    };
    let cap_base = if from_face {
        format!("{tag}{name}")
    } else {
        let sketch_base = name.strip_suffix(":PROFILE").unwrap_or(name);
        format!("{tag}{sketch_base}:PROFILE")
    };
    // The straight path's per-loop cap names, unchanged by draft (the 2026-08-18
    // decision: naming must not move when `draftAngle` toggles) and keyed per
    // REGION exactly as `build_straight` keys them, so a two-region profile names
    // its caps the same whether or not it is drafted.
    let caps = common::CapNames::new(&cap_base, &profile.regions);

    // One drafted prism per REGION, unioned below — `build_straight`'s own shape,
    // which has always looped regions and unioned via `union_region_solids`.
    let mut region_solids = Vec::with_capacity(profile.regions.len());
    for (region_index, region) in profile.regions.iter().enumerate() {
        let outer = region.first().ok_or("extrude: profile has no outer loop")?;
        let count = outer.curves.len();
        if count < 2 {
            return Err(format!("extrude: outer loop needs >= 2 curves, got {count}").into());
        }
        let (start_name, end_name) = caps.per_loop[region_index].clone();

        // One drafted leg: build + stamp draft names — walls `{edge}_E` in input
        // order, `near_name` on the profile-plane cap, `far_name` on the far cap.
        let drafted_leg = |leg_direction: Vec3,
                           leg_distance: f64,
                           near_name: &str,
                           far_name: &str|
         -> Result<BrepSolid, String> {
            let mut solid = extrude_profile_brep_draft(
                &outer.curves,
                leg_direction,
                leg_distance,
                draft_deg.to_radians(),
                Some(feature_base),
            )
            .map_err(|error| format!("extrude: {error}"))?;
            // Builder face order (sweep_topology.rs): [walls in INPUT curve
            // order…, START cap (profile plane), END cap (offset far section)].
            let faces = &mut solid
                .shells
                .get_mut(0)
                .ok_or("extrude draft builder produced no shell")?
                .faces;
            if faces.len() != count + 2 {
                return Err(format!(
                    "extrude draft builder produced {} faces, expected {}",
                    faces.len(),
                    count + 2
                ).into());
            }
            for (index, face) in faces.iter_mut().take(count).enumerate() {
                let raw = outer
                    .edge_names
                    .get(index)
                    .cloned()
                    .flatten()
                    .unwrap_or_else(|| "EDGE".to_string());
                face.name = Some(format!("{tag}{raw}_SW"));
            }
            faces[count].name = Some(near_name.to_string());
            faces[count + 1].name = Some(far_name.to_string());
            Ok(solid)
        };

        let solid = if !two_sided {
            // One-sided: caps `{featureID}_START` (profile plane) / `_END` (far).
            drafted_leg(direction, distance, &start_name, &end_name)?
        } else if distance.abs() <= 1e-9 {
            // Back-only: one leg along −normal by `back`; its far cap sits at
            // −distanceBack (START), the profile-plane cap at +distance = 0 (END).
            drafted_leg(direction.scale(-1.0), back, &end_name, &start_name)?
        } else {
            // Two-sided: both legs taper inward away from the parting plane; the
            // shared profile-plane caps are internal and must merge away in the
            // union, leaving the ridge edge at the sketch plane.
            let internal_name = format!("{feature_base}_PARTING_INTERNAL");
            let front = drafted_leg(direction, distance, &internal_name, &end_name)?;
            let back_leg = drafted_leg(direction.scale(-1.0), back, &internal_name, &start_name)?;
            let solid = common::union_region_solids(vec![front, back_leg])
                .map_err(|error| format!("extrude: two-sided draft {error}"))?;
            let survivor = solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .find(|face| {
                    face.name
                        .as_deref()
                        .is_some_and(|name| name.starts_with(&internal_name))
                });
            if survivor.is_some() {
                return Err(
                    "extrude: two-sided draft union kept an internal profile-plane cap — \
                     the two legs did not glue across the sketch plane"
                        .into(),
                );
            }
            solid
        };
        region_solids.push(solid);
    }
    let solid = common::union_region_solids(region_solids)
        .map_err(|error| format!("extrude: draft {error}"))?;

    // Same sidewall/cap role convention as the straight path, keyed on the
    // draft `_E` suffix (kernel-side scene metadata, like every other port).
    common::stamp_sweep_roles_multi(&solid, "_SW", &caps.starts(), &caps.ends());
    // The two-sided union carries BOTH legs' walls; the boolean uniquifies the
    // duplicate names to `{...}_SW_1`, which `stamp_sweep_roles_multi`'s suffix match
    // misses — those are sidewalls all the same.
    for face in solid.shells.iter().flat_map(|shell| shell.faces.iter()) {
        if let Some(name) = face.name.as_deref() {
            if let Some((stem, tail)) = name.rsplit_once('_') {
                if stem.ends_with("_SW") && tail.chars().all(|c| c.is_ascii_digit()) {
                    common::stamp_face_role(name, "sidewall", "SIDEWALL");
                }
            }
        }
    }
    Ok((solid, caps))
}


/// `key` evaluated as a number, 0.0 when absent/null (schema-optional numbers).
fn optional_number(ctx: &FeatureContext, key: &str) -> Result<f64, String> {
    match ctx.param(key) {
        None | Some(serde_json::Value::Null) => Ok(0.0),
        Some(_) => ctx.number(key),
    }
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected profile (face or sketch) drives `profile`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.has_profile()
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "E",
    "shortName": "E",
    "longName": "Extrude",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the extrude feature"
        },
        "profile": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "SKETCH"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Select the profile to extrude"
        },
        "consumeProfileSketch": {
            "type": "boolean",
            "default_value": true,
            "hint": "Remove the referenced sketch after creating the extrusion. Turn off to keep it in the scene."
        },
        "distance": {
            "type": "number",
            "default_value": 10,
            "hint": "Extrude distance when no path is provided"
        },
        "distanceBack": {
            "type": "number",
            "default_value": 10,
            "hint": "Optional backward extrude distance (two-sided extrude)"
        },
        "draftAngle": {
            "type": "number",
            "default_value": 0,
            "hint": "Draft/taper angle in degrees (positive tapers inward → smaller far cap). Line/arc profiles, single region, no holes; with distanceBack the sketch plane is the parting plane and both sides taper inward away from it; 0 = straight extrude."
        },
        "boolean": common::optional_boolean_schema()
    }
})
}

