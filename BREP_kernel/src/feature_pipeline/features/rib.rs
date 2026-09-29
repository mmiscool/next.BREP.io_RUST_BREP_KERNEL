//! RIB — Rib. Backed by the Rust `rib_from_profile`.
//!
//! # What this feature does
//!
//! It reproduces SolidWorks' RIB: an OPEN sketch chain is thickened into a thin
//! wall and grown until it LANDS ON THE PART, then unioned in — so the result
//! REPLACES the target (`removed: [target]`).
//!
//! # Headless contract (post-cutover)
//!
//! Both scene inputs resolve BY NAME, no marshaled geometry:
//! - `profile` names a SKETCH whose ordered OPEN chain the SKETCH feature already
//!   published as a path (`scene.resolve_path`, via `common::resolve_path`) — an
//!   ordered `Vec<NurbsCurve>` of the profile's line segments. A single resident
//!   EDGE also resolves (its curve); a lone edge publishes no plane, so a STRAIGHT
//!   one has no rib plane to work in and errors clearly.
//! - `targetSolid` names the resident solid to fuse into (`scene.resolve_solid`).
//!   A miss is a structured `unresolved` entry (contract rule 1), NOT a halt.
//!
//! # Extrusion direction — SolidWorks' two, and they are not interchangeable
//!
//! `extrusionDirection` is SolidWorks' control of the same name, and it decides
//! WHICH AXIS CARRIES WHICH:
//! - `PARALLEL_TO_SKETCH` (the default, and SolidWorks' own): the material grows
//!   PARALLEL to the sketch plane, thickness applied NORMAL to it. A line drawn
//!   between two walls becomes a thin fin standing ON its sketch plane — the rib
//!   or gusset everybody means.
//! - `NORMAL_TO_SKETCH`: the material grows NORMAL to the plane, thickness applied
//!   IN it. The chain is thickened inside its own plane and that ribbon is driven
//!   off the plane.
//!
//! Getting these the wrong way round does not tilt a rib slightly — it lays down
//! the fin that should stand up, which is what this feature did before.
//!
//! # The profile plane
//!
//! Both directions need the profile's PLANE. When the profile is a SKETCH that is
//! the plane it was AUTHORED on, published as a named frame under the sketch id
//! (`common::sketch_plane_frame`) — the same frame `SM.CF` and the hole feature
//! take. Only a FRAMELESS profile (a resident solid edge) falls back to deriving
//! it from the chain's bends, which is sign-ambiguous and undefined for a chain
//! with no bend. Taking the sketch's own plane is what makes a SINGLE-SEGMENT
//! sketch a legal rib profile.
//!
//! # Which side the material goes
//!
//! `direction` is SolidWorks' "flip material side" for the axis the extrusion
//! direction chose: `NORMAL` = `+axis`, `-NORMAL` = `-axis`, and `AUTO` picks the
//! side the part is on. For a parallel rib AUTO MARCHES from points along the
//! chain and takes the side that meets material — not a centroid test, which
//! happily aims a rib out of a shelled box's open face.
//!
//! # Up To Next — there is no depth
//!
//! SolidWorks' rib has exactly one end condition and no depth field: the rib grows
//! until it meets the next faces, and the feature FAILS when any part of it meets
//! nothing. `rib_from_profile` reproduces that by cutting an over-long sweep with
//! the part itself, so the rib's far end IS the part's faces, whatever shape they
//! are. A `depth` in a saved document is ignored.
//!
//! # V1 limitation (replicated): OPEN STRAIGHT-POLYLINE profiles only
//!
//! Arcs/curves and closed loops are rejected. Here: a
//! non-degree-1 (arc) profile curve errors clearly; `rib_from_profile` itself
//! rejects a closed chain, and a collinear chain with no plane to fall back on.
//!
//! # Name fidelity (contract rule 2)
//!
//! Every face the rib makes is named from the feature id and the input it came
//! from, never from the order the kernel's booleans met it (`crate::RibNames`
//! holds the table). With `{id}` the feature id and `{segment}` the sketch
//! segment a profile curve came from (`{sketchId}:G{gid}`, the name the sketch
//! publishes for it — resolved against the OWNING sketch, so a segment pick and a
//! whole-sketch pick name the same faces):
//!
//! - Parallel to Sketch: the two walls `{id}:A` (the sketch normal's negative
//!   side) and `{id}:B`, the top `{id}:{segment}_TOP` per segment, the ends
//!   `{id}:START` / `{id}:END` at the chain's start and end;
//! - Normal to Sketch: the walls `{id}:{segment}_A` / `_B` per segment (`_B` on
//!   the left of the chain seen from the side the sketch normal points to), the
//!   top on the sketch plane `{id}:TOP`, the ends `{id}:START` / `{id}:END`.
//!
//! A part face the rib SPLITS keeps its name on its first fragment in side order
//! (the rib's A side, straddling, B side) and then along the chain; every other
//! fragment is `{id}:{name}_{side}{k}`, `k` its place in its side's count with the
//! kept fragment included (`_B1`, `_A2`, …) — never by size, which an unrelated
//! hole moves. The
//! result goes through `common::register_added`, so its derived edge names are
//! stamped from the result's own faces here rather than by whichever feature
//! consumes it next.
//!
//! It is a MODIFY, not a create: the fused solid is registered under the TARGET's
//! own name and the target is consumed, so the scene sees the same body with a
//! rib on it rather than a new body named after the feature. That is the same
//! removed-then-added-under-one-name shape chamfer, fillet, hole and the boolean
//! fold all use. The source sketch is consumed too, by default
//! (`consumeProfileSketch`).

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult};
use crate::{
    rib_from_profile, NurbsCurve, PointClass, RibExtrusion, RibNames, SolidClassifier, Vec3,
};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // Resolve the target BY NAME first (before the profile). A miss is a structured
    // `unresolved` entry (contract rule 1: the caller repairs + re-dispatches), NOT a hard
    // error that halts the loop.
    let (target_name, target_handle) = match read_target_name(ctx) {
        Some(name) => match ctx.scene.resolve_solid(&name) {
            Some(handle) => (name, handle),
            None => {
                result.unresolved.push(name);
                return Ok(result);
            }
        },
        // No target selected at all: hard error.
        None => return Err("rib: select a target SOLID to fuse the rib into".into()),
    };

    // Resolve the profile BY NAME to its ordered OPEN chain (the sketch's path).
    // Normalize a `{sketch}:FACE` display-sheet pick to the sketch profile base.
    let profile_name = common::normalize_profile_alias(
        common::first_reference_name(ctx.param("profile"))
            .ok_or("rib: select an open sketch chain (polyline) for the profile")?,
    );
    let profile =
        common::resolve_path(ctx, &profile_name).map_err(|error| format!("rib: {error}"))?;
    if profile.is_empty() {
        return Err(format!("rib: profile '{profile_name}' has no curves").into());
    }
    // V1: OPEN straight-polyline profiles only — reject arcs (degree > 1), matching
    // the `edgeIsStraight` throw. `rib_from_profile` rejects a CLOSED chain and
    // a collinear one.
    if profile.iter().any(|curve| curve.degree != 1) {
        return Err(
            "rib: the profile must be a single OPEN polyline chain (arcs and closed loops \
             are not supported in V1)"
                .into(),
        );
    }

    // A non-finite value → 0; `rib_from_profile` enforces > 0.
    let thickness = common::number_or_default(ctx, "thickness", 0.0);

    // The profile's PLANE, from the sketch that published it. `None` only for a
    // frameless profile (a resident solid edge), where the chain's own bends are
    // the sole source and `rib_from_profile` derives them itself.
    let plane_normal = common::sketch_plane_frame(ctx, std::slice::from_ref(&profile_name))
        .map(|frame| frame.z_axis);

    let extrusion = read_extrusion(ctx)?;
    // Extrude direction, derived headlessly from the sketch and the extrusion mode.
    let extrude_dir =
        resolve_extrude_dir(ctx, &profile, plane_normal, extrusion, target_handle)?;

    let names = RibNames {
        feature: if ctx.id.is_empty() { "RIB".to_string() } else { ctx.id.clone() },
        segments: profile_segment_names(ctx, &profile_name, profile.len()),
    };

    // Thicken + grow up to the part. Short borrow — no held registry.
    let fused = crate::with_registered_solid_typed(target_handle, |target| {
        rib_from_profile(
            target,
            &profile,
            thickness,
            extrude_dir,
            plane_normal,
            extrusion,
            &names,
        )
    })?;

    // A rib MODIFIES its target — it does not create a body. So the fused solid
    // keeps the TARGET's name, the way every other target-consuming feature does
    // (chamfer/fillet via `common::resolve_blend_selection`, the boolean fold via
    // `common::finalize_solid`, hole): removed-then-added under the same name, so
    // every downstream reference to that body still resolves and the scene tree
    // shows the part the user ribbed rather than a new body called `RIB3`.
    result.added.push(common::register_added(fused, &target_name));
    // The rib REPLACES the target (removed: [target]).
    result.removed = vec![target_name];
    // ...and (by default) consumes the source sketch for display cleanup
    // (sketches register no scene solid, so `apply`
    // no-ops on the removed name). A viewport pick on the sketch's drawn segment
    // stores `{sketchId}:G{gid}`, so consume the OWNING sketch — the name the
    // display knows — not the segment.
    common::consume_sketch(ctx, &common::sketch_base_name(ctx, &profile_name), &mut result);
    Ok(result)
}

/// The source name of each profile curve, index-aligned with the chain
/// `common::resolve_path` resolved: the per-segment names the OWNING sketch
/// published (`{sketchId}:G{gid}`), so picking the sketch or one of its segments
/// names the rib's faces identically. A profile with no published names (a
/// resident solid edge) is named by the reference itself.
fn profile_segment_names(ctx: &FeatureContext, profile_name: &str, count: usize) -> Vec<String> {
    let owner = common::sketch_base_name(ctx, profile_name);
    let published = ctx
        .scene
        .resolve_path_segment_names(profile_name)
        .or_else(|| ctx.scene.resolve_path_segment_names(&owner));
    (0..count)
        .map(|index| {
            published
                .and_then(|names| names.get(index).cloned().flatten())
                .filter(|name| !name.trim().is_empty())
                .unwrap_or_else(|| {
                    if count == 1 {
                        profile_name.to_string()
                    } else {
                        format!("{profile_name}[{index}]")
                    }
                })
        })
        .collect()
}

/// The requested SolidWorks **Extrusion Direction**. Absent means
/// `PARALLEL_TO_SKETCH` — SolidWorks' own default, and the one everybody draws a
/// rib for.
fn read_extrusion(ctx: &FeatureContext) -> Result<RibExtrusion, String> {
    let raw = ctx
        .param("extrusionDirection")
        .and_then(|value| value.as_str())
        .map(|text| text.trim().to_uppercase())
        .filter(|text| !text.is_empty());
    match raw.as_deref() {
        None | Some("PARALLEL_TO_SKETCH") => Ok(RibExtrusion::ParallelToSketch),
        Some("NORMAL_TO_SKETCH") => Ok(RibExtrusion::NormalToSketch),
        Some(other) => Err(format!(
            "rib: unknown extrusion direction '{other}' (expected PARALLEL_TO_SKETCH or \
             NORMAL_TO_SKETCH)"
        ).into()),
    }
}

/// Resolve the direction the rib GROWS: the axis comes from the extrusion
/// direction — across the chain INSIDE the plane for `PARALLEL_TO_SKETCH`, along
/// the plane normal for `NORMAL_TO_SKETCH` — and `direction` picks which way
/// along it (SolidWorks' "flip material side").
fn resolve_extrude_dir(
    ctx: &FeatureContext,
    profile: &[NurbsCurve],
    plane_normal: Option<Vec3>,
    extrusion: RibExtrusion,
    target_handle: u32,
) -> Result<Vec3, String> {
    // The sketch's own plane when it published one, else the chain's bends.
    let np = match plane_normal {
        Some(normal) => normal,
        None => profile_normal(profile)?,
    };
    let axis = match extrusion {
        // Parallel to Sketch: the rib grows ACROSS the chain, inside the plane —
        // the direction a gusset fills toward the corner it braces.
        RibExtrusion::ParallelToSketch => {
            let vertices = chain_vertices(profile)?;
            let chord = vertices
                .last()
                .expect("a chain has vertices")
                .sub(vertices[0]);
            np.cross(chord).normalized().map_err(|_| {
                "rib: the profile's chord is degenerate, so a Parallel-to-Sketch rib has no \
                 direction to grow in"
                    .to_string()
            })?
        }
        RibExtrusion::NormalToSketch => np,
    };

    let mode = ctx
        .param("direction")
        .and_then(|value| value.as_str())
        .map(|text| text.trim().to_uppercase())
        .filter(|text| !text.is_empty())
        .unwrap_or_else(|| "AUTO".to_string());

    match mode.as_str() {
        "NORMAL" => Ok(axis),
        "-NORMAL" => Ok(axis.scale(-1.0)),
        // AUTO: drive the rib toward the part — the side it can actually land on.
        "AUTO" => auto_material_side(profile, axis, extrusion, target_handle),
        other => Err(format!(
            "rib: unknown extrude direction option '{other}' (expected AUTO, NORMAL, or -NORMAL)"
        ).into()),
    }
}

/// AUTO — which way along `axis` the material goes.
///
/// A rib has to LAND on the part (its only end condition is Up To Next), so the
/// side to grow into is the side the part is actually on. March out from points
/// along the chain in both directions and take the side that meets material from
/// more of them, ties broken by meeting it sooner: a cheap stand-in for
/// SolidWorks' hemispherical scan that gives the same answer on the shapes a rib
/// is drawn for, and one that cannot pick the open side of a shelled box the way
/// a centroid test can.
///
/// `NORMAL_TO_SKETCH` keeps the centroid rule it has always used — that mode
/// drives a ribbon off its plane, where "toward the part's middle" is exactly
/// right and the chain lies flat against nothing.
fn auto_material_side(
    profile: &[NurbsCurve],
    axis: Vec3,
    extrusion: RibExtrusion,
    target_handle: u32,
) -> Result<Vec3, String> {
    if matches!(extrusion, RibExtrusion::NormalToSketch) {
        let profile_centroid = chain_centroid(profile)?;
        let solid_centroid = target_vertex_centroid(target_handle)?;
        let to_solid = solid_centroid.sub(profile_centroid);
        return Ok(if axis.dot(to_solid) < 0.0 {
            axis.scale(-1.0)
        } else {
            axis
        });
    }

    let samples = chain_samples(profile)?;
    crate::with_registered_solid_str(target_handle, |solid| {
        let reach = crate::spatial::Aabb::from_points(
            solid.vertices.iter().map(|vertex| vertex.point),
        )
        .diagonal();
        if !(reach > 0.0) || !reach.is_finite() {
            return Err("rib: the target solid has no extent to aim the rib at".into());
        }
        let classifier = SolidClassifier::new(solid, 1e-6)?;
        let steps = 48;
        let mut best: Option<(usize, f64, Vec3)> = None;
        for signed in [axis, axis.scale(-1.0)] {
            let mut hits = 0usize;
            let mut total = 0.0;
            for sample in &samples {
                for step in 1..=steps {
                    let distance = reach * step as f64 / steps as f64;
                    let point = sample.add(signed.scale(distance));
                    if classifier.classify(point)?.class == PointClass::In {
                        hits += 1;
                        total += distance;
                        break;
                    }
                }
            }
            let mean = if hits == 0 {
                f64::INFINITY
            } else {
                total / hits as f64
            };
            let better = match best {
                None => true,
                Some((best_hits, best_mean, _)) => {
                    hits > best_hits || (hits == best_hits && mean < best_mean)
                }
            };
            if better {
                best = Some((hits, mean, signed));
            }
        }
        match best {
            Some((hits, _, direction)) if hits > 0 => Ok(direction),
            _ => Err(
                "rib: neither side of the profile reaches the part, so the rib has nothing to \
                 land on; move the sketch or set `direction` explicitly"
                    .into(),
            ),
        }
    })
}

/// Points spread along the chain — the probes AUTO marches from.
fn chain_samples(profile: &[NurbsCurve]) -> Result<Vec<Vec3>, String> {
    let mut samples = Vec::new();
    for curve in profile {
        let [start, end] = curve.domain()?;
        for step in 1..=3 {
            samples.push(curve.evaluate(start + (end - start) * step as f64 / 4.0)?);
        }
    }
    if samples.is_empty() {
        return Err("rib: profile has no points to aim from".into());
    }
    Ok(samples)
}

/// The ordered chain vertices (segment endpoints) of the profile curves.
fn chain_vertices(profile: &[NurbsCurve]) -> Result<Vec<Vec3>, String> {
    let mut vertices = Vec::with_capacity(profile.len() + 1);
    for (index, curve) in profile.iter().enumerate() {
        let [start, end] = curve.domain()?;
        let v_start = curve.evaluate(start)?;
        let v_end = curve.evaluate(end)?;
        if index == 0 {
            vertices.push(v_start);
        }
        vertices.push(v_end);
    }
    Ok(vertices)
}

/// Newell-style plane normal from the open chain's bends — the FALLBACK for a
/// profile that publishes no plane of its own (a resident solid edge; every sketch
/// publishes one, see [`build`]). Errors on a collinear chain (no unique plane —
/// the kernel's own `np` derivation errors the same way).
fn profile_normal(profile: &[NurbsCurve]) -> Result<Vec3, String> {
    let vertices = chain_vertices(profile)?;
    if vertices.len() < 3 {
        return Err(
            "rib: a straight (single-segment) profile from an edge has no plane; pick a SKETCH \
             chain (a sketch publishes the plane it was drawn on) or bend the chain"
                .into(),
        );
    }
    let mut normal = Vec3::default();
    for i in 1..vertices.len() - 1 {
        let a = vertices[i].sub(vertices[i - 1]);
        let b = vertices[i + 1].sub(vertices[i]);
        normal = normal.add(a.cross(b));
    }
    normal.normalized().map_err(|_| {
        "rib: profile is collinear and publishes no plane of its own; its plane normal cannot be \
         determined (pick a SKETCH chain instead)"
            .to_string()
    })
}

/// The profile centroid = the average of the chain vertices.
fn chain_centroid(profile: &[NurbsCurve]) -> Result<Vec3, String> {
    let vertices = chain_vertices(profile)?;
    if vertices.is_empty() {
        return Err("rib: empty profile has no centroid".into());
    }
    let mut sum = Vec3::default();
    for vertex in &vertices {
        sum = sum.add(*vertex);
    }
    Ok(sum.scale(1.0 / vertices.len() as f64))
}

/// The target's vertex centroid = the average of its BREP vertex points.
fn target_vertex_centroid(handle: u32) -> Result<Vec3, String> {
    crate::with_registered_solid_str(handle, |solid| {
        if solid.vertices.is_empty() {
            return Err("rib: target solid has no vertices for AUTO extrude direction".into());
        }
        let mut sum = Vec3::default();
        for vertex in &solid.vertices {
            sum = sum.add(vertex.point);
        }
        Ok(sum.scale(1.0 / solid.vertices.len() as f64))
    })
}

/// `inputParams.targetSolid` → the target name (array → first entry).
fn read_target_name(ctx: &FeatureContext) -> Option<String> {
    match ctx.param("targetSolid")? {
        serde_json::Value::Array(items) => items.iter().find_map(common::reference_name),
        other => common::reference_name(other),
    }
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected solid drives `targetSolid`, a sketch/edge `profile`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.solids > 0 || probe.sketches > 0 || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "RIB",
    "shortName": "RIB",
    "longName": "Rib",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the rib feature"
        },
        "targetSolid": {
            "type": "reference_selection",
            "selectionFilter": [
                "SOLID"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Select the part the rib fuses into"
        },
        "profile": {
            "type": "reference_selection",
            "selectionFilter": [
                "SKETCH",
                "EDGE"
            ],
            "multiple": true,
            "default_value": [],
            "hint": "Select an OPEN sketch chain (polyline) to thicken into a rib"
        },
        "thickness": {
            "type": "number",
            "default_value": 2,
            "step": 0.1,
            "hint": "Total rib wall thickness (offset ±thickness/2 about the profile)"
        },
        "extrusionDirection": {
            "type": "options",
            "options": [
                "PARALLEL_TO_SKETCH",
                "NORMAL_TO_SKETCH"
            ],
            "default_value": "PARALLEL_TO_SKETCH",
            "hint": "Parallel to sketch: the rib grows across the sketch with its thickness normal to the plane (a gusset). Normal to sketch: the chain is thickened in its plane and driven off it"
        },
        "direction": {
            "type": "options",
            "options": [
                "AUTO",
                "NORMAL",
                "-NORMAL"
            ],
            "default_value": "AUTO",
            "hint": "Which side the material goes: AUTO grows toward the part; NORMAL/-NORMAL force the ± side"
        },
        "consumeProfileSketch": {
            "type": "boolean",
            "default_value": true,
            "hint": "Remove the referenced sketch after creating the rib"
        }
    }
})
}

