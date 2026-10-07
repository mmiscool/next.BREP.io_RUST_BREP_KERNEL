//! SWP — Path Sweep. Rewired to resolve its profile + path FROM THE SCENE by name
//! (the headless cutover), mirroring `extrude.rs` / `revolve.rs`.
//!
//! # What this feature is (audit finding)
//!
//! The DEDICATED swept-surface op — NOT the `Sweep` (SW) feature, which only
//! extrudes along a chord and ignores twist. This feature sweeps a closed
//! planar profile along a path via `sweep_profile_along_chain`, which folds the
//! twisted and anchored single-curve builders behind one call and sweeps a
//! MULTI-segment path as one continuous tube. This port honors twist exactly
//! the same way.
//!
//! # SWP or SW — which sweep a path belongs to
//!
//! The two features now accept the SAME path selection (a whole sketch, or
//! connected edges chained head-to-tail), and they divide on the path's SHAPE
//! rather than on how it was picked:
//!   - **SWP (here)** drives the profile along the path on a
//!     rotation-minimizing frame, so the profile turns to follow a curve. A
//!     TANGENT-CONTINUOUS run is skinned between sampled stations as one tube.
//!   - **SW** translates the profile along each segment's chord and unions the
//!     portions, so the profile never rotates — and it refuses a CURVED segment.
//! A CORNER is no longer the line between them. An OPEN path of STRAIGHT
//! segments meeting at corners is MITRED here: the section is swept along each
//! segment and both sweeps are trimmed to the joint's bisector plane, where they
//! share one face loop — inner side folded, outer side extended, no bulkhead
//! between them, a picture-frame corner. SW's `pathAlign` mode mitres the same
//! path the same way (the construction is the shared builder's); the two still
//! differ in WHERE they put the section, which is this feature's whole point:
//! SWP transplants it onto the path, SW's `pathAlign` carries it from where it
//! was drawn.
//! A CLOSED cornered path is the whole FRAME: its closing joint is mitred like
//! every other, and the result has no caps at all (see below — a frame and a ring
//! are the same capless shape reached by the two lanes).
//! What is still refused, by name: a corner at a CURVED segment (a curve has no
//! single direction for a bisector plane to bisect, and no lane mixes mitring
//! with skinning), a closed cornered loop that CROSSES ITSELF, a SPATIAL frame
//! whose counter-twist has no room (below), a bend too TIGHT for
//! the section across it — on a frame, the two mitres of one side OVERLAPPING —
//! and a COLLAPSED bend at or past the 168.522° fold angle.
//!
//! # A CLOSED path is a RING — or, cornered, a FRAME
//!
//! A path whose last segment ends where its first begins — classified as such
//! when it is resolved (`common::resolve_sweep_path`) — sweeps as one CAPLESS
//! ring: the stations go round the loop with the first and last the same station,
//! and the sections skin through the CLOSED loft, so the result is one face per
//! profile edge, no caps, `V − E + F = 0` and genus 1. A CORNERED closed path is
//! the same capless topology reached by the MITRE lane instead — `n` walls per
//! profile edge for `n` segments, one shared loop per joint, still no caps and
//! still genus 1 — which is why the cap names are dropped on `path.closed`
//! rather than on the lane. The volume is Pappus — the
//! section's area times the distance its centroid travels, which under
//! `Transplant` IS the path length, since the centroid rides the path.
//!
//! A PLANAR loop's frame turns only about the plane normal, so it returns to
//! itself and the section meets itself at the seam. A SPATIAL loop's frame
//! returns ROLLED by the path's holonomy — the solid angle its tangent
//! indicatrix encloses, modulo 2π — and the builder closes it with the smallest
//! counter-twist: laid down linearly in arc length on a ring, shared over the
//! sides by length on a frame (each share rolled over the middle half of its
//! side, so the mitres stay exact). `sweep_closure` reports the holonomy and the
//! twist applied. A frame is refused, by name, where that roll has no room.
//!
//! A requested `twistAngle` on a closed path rides on that counter-twist, over the
//! whole lap, so after one lap the section comes back turned by exactly the
//! requested angle. It builds only when that turn carries the section onto itself:
//! a whole number of turns, or a step of the section's own rotational symmetry
//! about the path (90° for a centred square), read curve for curve as drawn.
//! Anything else is refused by name by the builder
//! ([`crate::SWEEP_TWIST_CLOSURE_REFUSAL`]), quoting the nearest twists that close.
//! Each loop is swept in a call of its own, so a HOLE loop is asked the same
//! question about the same path axis: an off-centre hole closes only on whole
//! turns, and a hole sharing the outer loop's symmetry closes on its steps. Under
//! a fractional turn each wall lands on the next profile edge's curve after the lap
//! and is still one face, named for the edge it leaves from.
//!
//! # Headless input contract (post-cutover)
//!
//! Both inputs resolve from the live scene, by name (no marshaled curves):
//! - `profile` `reference_selection` names a SKETCH whose profile the SKETCH
//!   feature already extracted (`SceneMap::resolve_profile`). Each region's
//!   OUTER loop `.curves` are a swept ring; its `.edge_names` stamp the sidewall
//!   names. Hole loops sweep along the SAME path (same twist, anchored to the
//!   FIRST region outer's placement) into cutters subtracted via
//!   `common::subtract_region_holes`; further regions sweep anchored too and the
//!   region solids union into ONE result. A profile that resolves to a resident
//!   solid FACE becomes a `SketchProfile` via `face_profile::face_profile` —
//!   the extrude.rs / revolve.rs / sweep.rs two-step, and the reason the
//!   `profile` field's filter is `["SKETCH","FACE"]`. A face profile has no
//!   sketch behind it, so nothing is consumed and the source solid stays
//!   resident.
//! - `path` `reference_selection` (`multiple`) resolves via
//!   `common::resolve_path_chain` — SW's resolver — to an ordered, head-to-tail
//!   run of named world curves: a whole SKETCH contributes its published chain,
//!   individual sketch segments and resident solid EDGES resolve one curve each
//!   and chain by endpoint coincidence, and an edge running the "wrong" way is
//!   reversed. The FIRST pick seeds the direction. The whole run sweeps as ONE
//!   tube under a SINGLE rotation-minimizing frame, so the frame carries across
//!   a joint rather than restarting per segment; a one-segment run delegates to
//!   the single-curve builders and is byte-identical to what it always emitted.
//!
//! # Name fidelity (contract rule 2)
//!
//! Side faces in INPUT-curve order, then bottom cap = START, top cap = END (the
//! loft order `sweep_profile_along_path` produces):
//!   - side `i`  → `${edgeName}_SWP`  (missing name → the `"EDGE"` fallback)
//!   - START cap → `${featureID}_START`
//!   - END cap   → `${featureID}_END`
//! There is NO `${id}:` feature tag here (unlike SW) — the
//! names are bare. Solid name `featureID || "PathSweep"`.
//!
//! A RING has no caps, so its faces are the side walls and NOTHING else — the cap
//! names are not merely unused, they are not registered, because a container
//! standing for a face that does not exist is a name a later feature can select
//! and nothing can resolve.
//!
//! The optional `boolean` param folds via `common::finalize_solid`; the referenced
//! sketch is consumed (`common::consume_sketch`).

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult, SketchProfile};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    // --- Profile: resolve the SKETCH profile from the scene by name.
    // Normalize a `{sketch}:FACE` display-sheet pick to the sketch profile base.
    let profile_name = common::normalize_profile_alias(
        common::first_reference_name(ctx.param("profile"))
            .ok_or("path sweep: missing `profile` reference selection")?,
    );
    // Sketch profile first; otherwise a resident FACE becomes a profile via
    // `face_profile` (the extrude.rs / revolve.rs / sweep.rs two-step, same
    // `SketchProfile` shape and the same downstream path). `from_face` skips the
    // consumed-sketch bookkeeping — a face profile has no sketch behind it.
    let mut face_profile_storage: Option<SketchProfile> = None;
    let (profile, from_face): (&SketchProfile, bool) =
        match ctx.scene.resolve_profile(&profile_name) {
            Some(profile) => (profile, false),
            None => match ctx.scene.resolve_face(&profile_name) {
                Some(face) => {
                    let built = super::face_profile::face_profile(face)
                        .map_err(|error| format!("path sweep: {error}"))?;
                    (&*face_profile_storage.insert(built), true)
                }
                None => {
                    return Err(format!(
                        "path sweep: profile '{profile_name}' not found (no sketch profile or resident face)"
                    ).into());
                }
            },
        };
    let first_outer = profile
        .regions
        .first()
        .and_then(|region| region.first())
        .ok_or("path sweep: profile has no outer loop")?;

    // --- Path: a whole sketch chain and/or individually picked edges, ordered
    //     head-to-tail into ONE connected run — the same resolution SW uses, so
    //     the two sweeps accept the same selection. The whole run is swept as
    //     one continuous tube under a single rotation-minimizing frame; the
    //     builder refuses a run with a CORNER in it, because skinning between
    //     stations would round the corner off rather than mitre it (SW is the
    //     feature for a cornered path). A missing selection is a hard error.
    let path_names = common::reference_names(ctx.param("path"));
    if path_names.is_empty() {
        return Err("path sweep: requires a path edge selection".into());
    }
    // The resolved chain AND its classification — closure, the plane it lies in if
    // it lies in one, and every joint's continuity — computed once here; the
    // builder's corner refusal and its closed-ring lane both read it.
    let path = common::resolve_sweep_path(ctx, &path_names)
        .map_err(|error| format!("path sweep: {error}"))?;

    let feature_id = if ctx.id.is_empty() {
        "PathSweep"
    } else {
        ctx.id.as_str()
    };

    // Twist: degrees → radians (a non-finite `twistAngle` → 0). The gate for the
    // twisted builder is plain truthiness on the RADIANS, i.e. `!= 0.0` exactly
    // (NOT an epsilon).
    let twist_degrees = common::number_or_default(ctx, "twistAngle", 0.0);
    let twist_radians = twist_degrees * std::f64::consts::PI / 180.0;

    // The builder re-centers a swept loop's own centroid onto the path; every
    // loop OTHER than the first region's outer (hole loops, and the outers of
    // further regions) must instead keep its in-plane offset relative to that
    // first outer — they sweep ANCHORED to its placement anchor.
    let anchor = crate::profile_anchor(&first_outer.curves)
        .map_err(|error| format!("path sweep: {error}"))?;

    // Per-loop cap names keyed off each region's STABLE loop identity, with the
    // un-keyed `{feature_id}_START/_END` as the containers standing for all of
    // them (see `common::CapNames`).
    let caps = common::CapNames::new(feature_id, &profile.regions);

    // One swept solid per REGION, unioned below.
    let mut region_solids = Vec::with_capacity(profile.regions.len());
    // The station lane's measured bounds, one per swept loop (`sweep.stations`).
    let mut approximations: Vec<crate::Approximation> = Vec::new();
    // Per region: the section is WIDER than its own bend and the kernel built the
    // swept ENVELOPE (a revolve) instead of skinning a tube. Read off the same
    // recognition the builder selects the lane on, so names and geometry agree.
    let mut envelope_regions = vec![false; profile.regions.len()];
    for (region_index, region) in profile.regions.iter().enumerate() {
        let outer = region.first().ok_or("path sweep: profile has no outer loop")?;
        if outer.curves.len() < 2 {
            return Err(format!(
                "path sweep: requires a closed profile with at least two boundary curves, got {}",
                outer.curves.len()
            ).into());
        }
        // Region 0 places on its OWN centroid; every later region rides region
        // 0's anchor so its in-plane offset survives (the same rule hole loops
        // follow below). `sweep_profile_along_chain` folds the twisted and
        // anchored variants into one call and delegates a ONE-segment chain
        // straight back to the single-curve builders, so an ordinary
        // single-edge path sweep emits exactly the geometry it always has.
        let envelope = matches!(
            crate::recognize_swept_envelope(
                &outer.curves,
                &path,
                crate::SectionPlacement::Transplant,
            ),
            Ok(Ok(_))
        );
        envelope_regions[region_index] = envelope;
        if envelope && region.len() > 1 {
            return Err(format!(
                "path sweep: region {} bends TIGHTER than its own section, so its swept solid is \
                 the envelope trimmed at its own self-intersection — a revolve, not a skinned tube \
                 — and a HOLE loop through one is not built: the hole's own envelope would have to \
                 be trimmed against the outer one, which nothing solves. Sweep the outer loop \
                 alone, or ease the bend",
                region_index + 1
            ).into());
        }
        // The station lane's MEASURED BOUND rides into the result's
        // `approximations` (`sweep.stations`); the mitre and envelope lanes are
        // exact and report none.
        let swept = crate::sweep_profile_along_chain_reported(
            &outer.curves,
            &path,
            twist_radians,
            Some(feature_id),
            (region_index != 0).then_some(anchor),
            crate::SectionPlacement::Transplant,
            "A corner between two STRAIGHT segments is MITRED — both sweeps trimmed to the joint's bisector plane, sharing one loop there — but a corner at a CURVED segment is not built by either sweep feature, because a curve has no single direction for a bisector plane to bisect. Split the path at that corner and sweep each run separately, or round the corner so the joint is tangent-continuous.",
            None,
        )
        .map_err(|error| format!("path sweep: {error}"))?;
        let mut solid = swept.solid;
        if let Some(report) = swept.report {
            approximations.push(report.approximation(if region_index == 0 {
                feature_id.to_string()
            } else {
                format!("{feature_id} region {}", region_index + 1)
            }));
        }

        // Face names: `[${edgeName}_SWP..., ${id}_START, ${id}_END]` (INPUT-curve
        // order). A MITRED path — a cornered polyline, built as one exact piece
        // per segment joined on each joint's bisector plane — has a wall per
        // (path segment × profile edge) instead, in that order, so each one is
        // keyed by its segment as `${edgeName}:${segment}_SWP` with the un-keyed
        // `${edgeName}_SWP` registered as the CONTAINER standing for that profile
        // edge's wall along every segment. Without the key several faces would
        // share one name and a downstream reference would resolve to whichever
        // the boolean happened to order first.
        let side_count = outer.curves.len();
        let mitred = path.cornered_polyline();
        let wall_base = |index: usize| -> String {
            outer
                .edge_names
                .get(index)
                .cloned()
                .flatten()
                .filter(|name| !name.trim().is_empty())
                .unwrap_or_else(|| "EDGE".to_string())
        };
        let mut face_names: Vec<String> = Vec::with_capacity(side_count + 2);
        let caps_here = if path.closed { 0 } else { 2 };
        let envelope_walls = solid
            .shells
            .first()
            .map_or(0, |shell| shell.faces.len())
            .saturating_sub(caps_here);
        if envelope {
            // The envelope's walls are REVOLVED arcs of the section circle, not
            // the section's own edges. Crossing the axis leaves ONE wall — the
            // image of the whole boundary — which takes edge 0's name, and the
            // other edges' containers list it (registered below). A section only
            // TANGENT to the axis leaves one wall per profile arc, one name each.
            let walls = if envelope_walls == 1 { 1 } else { side_count };
            // Only the ONE-wall envelope folds the other edges' names into it.
            envelope_regions[region_index] = walls == 1;
            for index in 0..walls {
                face_names.push(format!("{}_SWP", wall_base(index)));
            }
        } else if mitred {
            for segment in 0..path.len() {
                for index in 0..side_count {
                    face_names.push(format!("{}:{}_SWP", wall_base(index), path.name(segment)));
                }
            }
        } else {
            for index in 0..side_count {
                face_names.push(format!("{}_SWP", wall_base(index)));
            }
        }
        // A CLOSED path sweeps a capless RING: the closed loft emits one face per
        // profile edge, in the same INPUT-curve order, and no caps — so there is
        // no cap face to name and no cap NAME to publish.
        if !path.closed {
            let (start, end) = &caps.per_loop[region_index];
            face_names.push(start.clone());
            face_names.push(end.clone());
        }

        let faces = &mut solid
            .shells
            .get_mut(0)
            .ok_or("path sweep: builder produced no shell")?
            .faces;
        if faces.len() != face_names.len() {
            return Err(format!(
                "path sweep: builder produced {} faces, expected {} ({} {} + {})",
                faces.len(),
                face_names.len(),
                if mitred {
                    format!("{} x {side_count}", path.len())
                } else {
                    side_count.to_string()
                },
                if mitred { "mitred walls" } else { "sides" },
                if path.closed { "no caps (a closed path)" } else { "2 caps" }
            ).into());
        }
        for (face, name) in faces.iter_mut().zip(&face_names) {
            face.name = Some(name.clone());
        }

        // Region holes: sweep each hole loop along the SAME path with the SAME
        // twist into an anchored cutter and subtract. Cutter caps land coplanar
        // with the outer's caps; the exact boolean's coplanar merge handles them.
        if region.len() > 1 {
            solid = common::subtract_region_holes(
                solid,
                profile,
                region,
                "path sweep",
                feature_id,
                None,
                &mut |loop_index, _depth| {
                    let hole_curves = &region[loop_index].curves;
                    crate::sweep_profile_along_chain_reported(
                        hole_curves,
                        &path,
                        twist_radians,
                        None,
                        Some(anchor),
                        crate::SectionPlacement::Transplant,
                        "A corner between two STRAIGHT segments is MITRED — both sweeps trimmed to the joint's bisector plane, sharing one loop there — but a corner at a CURVED segment is not built by either sweep feature, because a curve has no single direction for a bisector plane to bisect. Split the path at that corner and sweep each run separately, or round the corner so the joint is tangent-continuous.",
                        None,
                    )
                    .map(|swept| {
                        if let Some(report) = swept.report {
                            approximations.push(report.approximation(format!("{feature_id} hole {loop_index}")));
                        }
                        swept.solid
                    })
                },
            )?;
        }
        region_solids.push(solid);
    }
    let solid = common::union_region_solids(region_solids)
        .map_err(|error| format!("path sweep: {error}"))?;

    // Role metadata: sidewall/cap convention, from the actual named faces. A RING
    // has no caps, so it offers none and registers no cap container.
    let (cap_starts, cap_ends) = if path.closed {
        (Vec::new(), Vec::new())
    } else {
        (caps.starts(), caps.ends())
    };
    common::stamp_sweep_roles_multi(&solid, "_SWP", &cap_starts, &cap_ends);

    let mut containers = if path.closed {
        Vec::new()
    } else {
        caps.containers()
    };
    // A MITRED path names a wall per segment, so the un-keyed `${edgeName}_SWP`
    // becomes the CONTAINER standing for all of them — the same relationship the
    // per-loop caps have with their un-keyed names, and the spelling a reference
    // stored before the path was given a corner already uses.
    // An ENVELOPE region's one revolved wall is named for profile edge 0; every
    // other edge's `_SWP` name is registered as a container standing for it, so
    // a reference to any edge's wall resolves to the face that is there.
    for (region_index, region) in profile.regions.iter().enumerate() {
        if !envelope_regions[region_index] {
            continue;
        }
        let Some(outer) = region.first() else {
            continue;
        };
        let base = |index: usize| {
            outer
                .edge_names
                .get(index)
                .cloned()
                .flatten()
                .filter(|name| !name.trim().is_empty())
                .unwrap_or_else(|| "EDGE".to_string())
        };
        let wall = format!("{}_SWP", base(0));
        for index in 1..outer.curves.len() {
            let name = format!("{}_SWP", base(index));
            if name != wall {
                containers.push((name, vec![wall.clone()]));
            }
        }
    }
    if path.cornered_polyline() {
        for region in &profile.regions {
            let Some(outer) = region.first() else {
                continue;
            };
            for index in 0..outer.curves.len() {
                let base = outer
                    .edge_names
                    .get(index)
                    .cloned()
                    .flatten()
                    .filter(|name| !name.trim().is_empty())
                    .unwrap_or_else(|| "EDGE".to_string());
                let members = (0..path.len())
                    .map(|segment| format!("{base}:{}_SWP", path.name(segment)))
                    .collect();
                containers.push((format!("{base}_SWP"), members));
            }
        }
    }
    let mut result = common::finalize_solid_grouped(ctx, solid, feature_id, &containers);
    if result.error.is_some() {
        return Ok(result);
    }
    result.approximations.extend(approximations);
    // A FACE profile has no sketch to consume — the source solid stays resident.
    if !from_face {
        common::consume_sketch(ctx, &profile_name, &mut result);
    }
    Ok(result)
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected profile drives `profile`, edges `path`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.has_profile() || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SWP",
    "shortName": "SWP",
    "longName": "Path Sweep",
    "ribbonPath": "Home/Sweep/Path Sweep",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the path sweep feature"
        },
        "profile": {
            "type": "reference_selection",
            "selectionFilter": [
                "SKETCH",
                "FACE"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Select a single closed planar profile (sketch or face) to sweep"
        },
        "consumeProfileSketch": {
            "type": "boolean",
            "default_value": true,
            "hint": "Remove the referenced sketch after creating the sweep. Turn off to keep it in the scene."
        },
        "path": {
            "type": "reference_selection",
            "selectionFilter": [
                "SKETCH",
                "EDGE"
            ],
            "multiple": true,
            "default_value": null,
            "hint": "Sweep path: pick a whole sketch, or one or more connected edges — they are chained head-to-tail, with the FIRST pick setting the direction the sweep runs. A smooth (tangent-continuous) run is swept as one tube; a run of STRAIGHT segments meeting at corners is MITRED on each joint's bisector plane. A CLOSED run sweeps with no end caps — a smooth one as a ring, a cornered one as the whole mitred FRAME. A closed loop that is not flat comes back from one lap rolled by the path's own holonomy, and the sweep closes it with the smallest counter-twist, spread evenly along the path. A corner at a curved segment, a closed loop that crosses itself, a bend too tight for the section, and a fold at or past 168.5 degrees are refused by name."
        },
        "twistAngle": {
            "type": "number",
            "default_value": 0,
            "label": "Twist (deg)",
            "hint": "Rotate the profile about the path tangent, linearly in arc length, by this total angle (§5.7). 0 = plain sweep. On a CLOSED path the twist must carry the profile onto itself after one lap: a whole number of turns, or a step of the profile's own symmetry about the path (every 90 degrees for a square centred on it). Other angles are refused, naming the nearest that close."
        },
        "boolean": common::optional_boolean_schema()
    }
})
}

