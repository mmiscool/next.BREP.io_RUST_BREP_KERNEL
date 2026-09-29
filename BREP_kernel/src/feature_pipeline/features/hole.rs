//! H — Hole.
//!
//! A hole builds a cutter tool per placement point, places each one at the point
//! oriented so the tool axis points INTO the target, and SUBTRACTS the cutters
//! from the target solid.
//!
//! # Placement resolution (post-cutover, scene-resolved)
//!
//! The schema's `face` param (label "Placement
//! (sketch)") names ONE SKETCH. The feature drills at the sketch's
//! non-construction points along the sketch plane normal flipped INTO the target.
//! Post-flip the pipeline receives the RAW params, so
//! all of that resolves HERE from the scene:
//!
//! - **centers** — the referenced sketch publishes every solved point as
//!   `{sketchId}:P{pid}` (world space) with its construction flag. The scene-map
//!   indexes points by exact name only, so the sketch's points are prefix-scanned
//!   (`{sketchId}:P…`, via `SceneMap::model_points_with_prefix`) — safe because
//!   the prefix ends in `:P`, which another sketch id cannot spell into ("S1:P"
//!   never prefixes "S10:P…"). CONSTRUCTION points are skipped by that flag (a
//!   ◐ point, a materialized external reference); every model point drills,
//!   `P0` included — the app seeds a new sketch with no points, so the first
//!   point placed IS `P0`. Points sort by numeric pid (a deterministic subtract
//!   order — the map iterates randomly) and coincident positions dedupe at 1e-7.
//! - **axis** — the sketch's published plane frame normal
//!   (`SceneMap::resolve_frame(sketchId)` → `z_axis`, after normalizing the
//!   `{sketch}:FACE` / `{sketch}:PROFILE` aliases to the bare id like every
//!   other sketch consumer), then flipped so it points
//!   INTO the target using the target's
//!   bbox (the "center" is the placement bbox center,
//!   the headless stand-in for the sketch-group bbox center).
//! - **per-point face prefix** — the point's own scene name `{sketchId}:P{pid}`,
//!   used as the face-name prefix, so the
//!   walls land as `{sketchId}:P{pid}_Hole_S` etc.
//! - `diameter`, `depth`, `throughAll`, the CSK/CBORE params, and
//!   `boolean = { operation, targets, mergeCoplanarFaces }` read as before:
//!   `targets[0]` is the cut target (contract rule 1: EXACT-match resolution; a
//!   miss is an `unresolved` entry, never a panic). `operation` defaults
//!   `SUBTRACT`; a hole is always a cut. A missing/unresolved placement SKETCH
//!   is a HARD error (the extrude profile precedent — no silent air-drill).
//!
//! # Hole types (`holeType`, default `SIMPLE`)
//!
//! - **SIMPLE** — a plain cylinder cutter (`make_cylinder_brep`, radius = d/2,
//!   height = depth).
//! - **THREADED** — SYMBOLIC only. Per the audit, `effectiveThreadMode` is
//!   hardcoded `SYMBOLIC`, so a modeled helix is NEVER built: the cutter is a
//!   plain cylinder core of the crest diameter, identical to SIMPLE. (NPT taper
//!   is not modeled.)
//! - **COUNTERSINK / COUNTERBORE** — a revolved profile (`revolve_profile_brep_named`,
//!   full turn about `+Y`), the exact hole-tool profiles.
//!
//! # Name fidelity (contract rule 2)
//!
//! The master cutter's faces are stamped `${id}_MASTER_HOLE…`; each placed copy is
//! re-prefixed to its point's name (`{sketchId}:P{pid}`). The wall/cap suffixes
//! match the hole-tool profiles:
//! SIMPLE `_Hole_{S,B,T}` (in `make_cylinder_brep` order `[side, bottom, top]`;
//! `_B` is the ENTRY cap, `_T` the drill tip). The subtract PROPAGATES those names
//! onto the result (via `operand_face_names`), so this feature does NOT re-stamp
//! post-boolean. The result reuses the TARGET's solid name (`removed = [target]`;
//! the cutters were never scene-resident).

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureResult};
use crate::{
    boolean_operation, make_cylinder_brep, make_line, revolve_profile_brep_named, transform_brep,
    AffineTransform, BooleanOperation, BooleanOptions, BrepSolid, NurbsCurve, Vec3,
};

/// The back-offset: pull the cutter base slightly back
/// along `-normal` so the entry cut is a clean through-intersection, not tangent.
const BACK_OFFSET: f64 = 1e-3;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

/// One resolved placement: a world center and its scene point name (the per-hole
/// face prefix).
struct Placement {
    position: Vec3,
    name: String,
}

/// The `boolean` param (`{ targets, operation,
/// mergeCoplanarFaces }`). Only the FIRST target is the cut target.
struct BooleanParam {
    operation: String,
    target: Option<String>,
    merge_coplanar_faces: bool,
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // --- Target resolution (contract rule 1: exact match, miss -> unresolved) ---
    let boolean = read_boolean_param(ctx);
    if boolean.operation == "NONE" {
        // A NONE boolean leaves the scene unchanged (a no-op).
        return Ok(result);
    }
    let Some(target_name) = boolean.target.clone() else {
        // No target selected — pass through (the caller repairs / re-dispatches).
        return Ok(result);
    };
    let Some(target_handle) = ctx.scene.resolve_solid(&target_name) else {
        result.unresolved.push(target_name);
        return Ok(result);
    };

    // --- Placement sketch: `face` names the SKETCH whose points are the centers
    //     and whose plane normal is the drill axis ---
    // The SAME sketch is spelled three ways and all three must resolve: the bare
    // id, `{sketch}:FACE` (what a viewport click on the committed sketch's
    // display sheet stores) and the older `{sketch}:PROFILE`. Hole used to match
    // EXACTLY, so the commonest of the three — the viewport click — failed with
    // "placement sketch not found" while the sketch's resolved frame sat in the
    // scene under the bare id. A resident solid FACE is spelled
    // `{solid}|{face}[n]` and carries neither suffix (see
    // `common::normalize_profile_alias`), so stripping them collides with nothing.
    let picked = common::first_reference_name(ctx.param("face"))
        .ok_or("hole: requires a sketch selection (`face` names the placement sketch)")?;
    let aliased = common::normalize_profile_alias(picked);
    let sketch_name = aliased
        .strip_suffix(":PROFILE")
        .map(str::to_string)
        .unwrap_or(aliased);
    let frame = ctx.scene.resolve_frame(&sketch_name).ok_or_else(|| {
        format!("hole: placement sketch '{sketch_name}' not found (no resolved sketch frame in the scene)")
    })?;
    let placements = resolve_placements(ctx, &sketch_name);
    if placements.is_empty() {
        return Err("hole: no placement points (each hole needs a sketch center)".into());
    }
    let sketch_normal = frame
        .z_axis
        .normalized()
        .map_err(|_| "hole: sketch plane normal is zero-length".to_string())?;

    // Target bbox: drives the into-target normal flip AND the through-all reach.
    let target_bbox = with_solid_bbox(target_handle)?;
    let normal = flip_into_target(sketch_normal, &placements, target_bbox);

    // --- Sizes (numeric params may be expression strings) ---
    let hole_type = ctx
        .string("holeType")
        .unwrap_or_else(|| "SIMPLE".to_string())
        .to_uppercase();
    let diameter = ctx.number("diameter").unwrap_or(0.0).abs();
    let radius = (diameter * 0.5).max(1e-4);
    let depth_input = ctx.number("depth").unwrap_or(0.0).abs();
    let through_all = ctx
        .param("throughAll")
        .and_then(|value| value.as_bool())
        .unwrap_or(false);

    // Through-all: drill at least 1.5x the target's bbox diagonal (legacy:956), so the
    // cutter fully clears the target regardless of the entry point.
    let straight_depth = if through_all {
        let diag = target_bbox
            .map(|(min, max)| max.sub(min).length())
            .unwrap_or(0.0);
        let reach = if diag > 0.0 { diag * 1.5 } else { 50.0 };
        depth_input.max(reach)
    } else {
        depth_input
    };
    if straight_depth <= 0.0 {
        return Err("hole: straight depth must be positive (set `depth` or `throughAll`)".into());
    }

    // --- Master cutter, built once in local frame (base at origin, axis +Y) ---
    let master_prefix = if ctx.id.is_empty() {
        "MASTER_HOLE".to_string()
    } else {
        format!("{}_MASTER_HOLE", ctx.id)
    };
    let master = build_master_cutter(&hole_type, radius, straight_depth, ctx, &master_prefix)?;

    // --- Place a cutter at every point, oriented so +Y -> normal ---
    let mut cutters: Vec<BrepSolid> = Vec::with_capacity(placements.len());
    for placement in &placements {
        let origin = placement.position.sub(normal.scale(BACK_OFFSET));
        let transform = AffineTransform::new(placement_matrix(normal, origin)?)?;
        let mut placed = transform_brep(&master, transform, false)?;
        // Re-prefix faces master_prefix -> the point's name.
        reprefix_faces(&mut placed, &master_prefix, &placement.name);
        cutters.push(placed);
    }

    // --- Subtract every cutter from the target (SEQUENTIAL, not union-first) ---
    // Hole cutters are disjoint, so a
    // union-first pass is fragile; subtract each tool in turn instead.
    let options = BooleanOptions {
        merge_coplanar_faces: boolean.merge_coplanar_faces,
        ..BooleanOptions::default()
    };
    let solid = subtract_cutters(target_handle, cutters, &options)?;

    // Names already propagated by `boolean_operation` — collect, do NOT re-stamp.
    let face_names = common::collect_face_names(&solid);
    let edge_names = common::collect_edge_names(&solid);
    let handle = crate::register_solid_value(solid);
    result.added.push(AddedSolid {
        handle,
        // SUBTRACT: the result reuses the TARGET's name (applyBoolean targets ==
        // [target]); the scene-map removes-then-adds so the name is reused cleanly.
        name: target_name.clone(),
        face_names,
        edge_names,
        ..AddedSolid::default()
    });
    result.removed = vec![target_name];
    Ok(result)
}

/// Resolve the referenced sketch's published MODEL points (`{sketchId}:P{pid}`)
/// into ordered, deduped placements. Construction points are skipped by their
/// published flag (see [`SceneMap::model_points_with_prefix`]) — never by id: the
/// app seeds a new sketch with NO points, so the first point a user places is
/// `P0`, and an id rule would silently drop that first hole (a one-point sketch
/// drilled nothing at all). Sorts by numeric pid (non-numeric ids after,
/// lexicographic) for a deterministic subtract order; dedupes coincident
/// positions at 1e-7 like the `dedupePlacementsByPosition`.
fn resolve_placements(ctx: &FeatureContext, sketch: &str) -> Vec<Placement> {
    let prefix = format!("{sketch}:P");
    let mut entries: Vec<(String, Vec3)> = ctx
        .scene
        .model_points_with_prefix(&prefix)
        .into_iter()
        .filter(|(name, _)| name.len() > prefix.len())
        .collect();
    let sort_key = |name: &str| -> (u8, i64, String) {
        let pid = &name[prefix.len()..];
        match pid.parse::<i64>() {
            Ok(number) => (0, number, String::new()),
            Err(_) => (1, 0, pid.to_string()),
        }
    };
    entries.sort_by(|(a, _), (b, _)| sort_key(a).cmp(&sort_key(b)));

    // Coincident-point dedupe (tolerance 1e-7, bucketed by rounding).
    let quantize = |value: f64| (value * 1e7).round() as i64;
    let mut seen = std::collections::HashSet::new();
    let mut placements = Vec::with_capacity(entries.len());
    for (name, position) in entries {
        if seen.insert((quantize(position.x), quantize(position.y), quantize(position.z))) {
            placements.push(Placement { position, name });
        }
    }
    placements
}

/// Flip `normal` so it points INTO the target: clamp
/// the placement-bbox center into the target's bbox; if the center already lies
/// inside (zero clamp delta), aim at the bbox center instead; flip on a negative
/// dot. No bbox (a degenerate target) leaves the sketch normal untouched.
fn flip_into_target(normal: Vec3, placements: &[Placement], bbox: Option<(Vec3, Vec3)>) -> Vec3 {
    let Some((min, max)) = bbox else {
        return normal;
    };
    let mut lo = placements[0].position;
    let mut hi = placements[0].position;
    for placement in placements {
        let p = placement.position;
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let center = lo.add(hi).scale(0.5);
    let clamped = Vec3::new(
        center.x.clamp(min.x, max.x),
        center.y.clamp(min.y, max.y),
        center.z.clamp(min.z, max.z),
    );
    let mut to_target = clamped.sub(center);
    if to_target.length_squared() < 1e-12 {
        to_target = min.add(max).scale(0.5).sub(center);
    }
    if to_target.length_squared() > 1e-10 && normal.dot(to_target) < 0.0 {
        normal.scale(-1.0)
    } else {
        normal
    }
}

/// Subtract every `cutter` from the scene target in sequence, keeping the running
/// result as a LOCAL owned solid. Only the operand(s) needed for each two-solid
/// borrow are registered, and each is freed immediately — no leaked intermediates,
/// and the SCENE target handle is never freed here (the removal loop owns it).
fn subtract_cutters(
    target_handle: u32,
    cutters: Vec<BrepSolid>,
    options: &BooleanOptions,
) -> Result<BrepSolid, String> {
    let mut current: Option<BrepSolid> = None;
    for cutter in cutters {
        let cutter_handle = crate::register_solid_value(cutter);
        let folded = match current.take() {
            None => crate::with_two_registered_solids(target_handle, cutter_handle, |target, tool| {
                boolean_operation(target, tool, BooleanOperation::Subtract, options)
            }),
            Some(running) => {
                let running_handle = crate::register_solid_value(running);
                let folded =
                    crate::with_two_registered_solids(running_handle, cutter_handle, |run, tool| {
                        boolean_operation(run, tool, BooleanOperation::Subtract, options)
                    });
                crate::free_registered_solid(running_handle);
                folded
            }
        };
        crate::free_registered_solid(cutter_handle);
        current = Some(folded.map_err(|error| format!("hole subtract failed: {error}"))?);
    }
    current.ok_or_else(|| "hole: no cutter produced".to_string())
}

/// Build the cutter for `hole_type` in the local frame (base at the origin, axis
/// `+Y`), with faces named `${master_prefix}…` in hole-tool order.
fn build_master_cutter(
    hole_type: &str,
    radius: f64,
    straight_depth: f64,
    ctx: &FeatureContext,
    master_prefix: &str,
) -> Result<BrepSolid, String> {
    match hole_type {
        // THREADED is SYMBOLIC (plain cylinder core) — identical build to SIMPLE.
        "SIMPLE" | "THREADED" => build_simple_cutter(radius, straight_depth, master_prefix),
        "COUNTERSINK" => {
            let sink_dia = ctx.number("countersinkDiameter").unwrap_or(0.0).abs();
            let sink_angle = ctx.number("countersinkAngle").unwrap_or(82.0).clamp(1.0, 179.0);
            build_countersink_cutter(radius, straight_depth, sink_dia, sink_angle, master_prefix)
        }
        "COUNTERBORE" => {
            let bore_dia = ctx.number("counterboreDiameter").unwrap_or(0.0).abs();
            let bore_depth = ctx.number("counterboreDepth").unwrap_or(0.0).abs();
            build_counterbore_cutter(radius, straight_depth, bore_dia, bore_depth, master_prefix)
        }
        other => Err(format!("hole: unknown holeType '{other}'")),
    }
}

/// SIMPLE / THREADED(symbolic): a plain cylinder, faces `_Hole_{S,B,T}` in
/// `make_cylinder_brep` order `[side, bottom, top]` (the hole-tool else-branch).
fn build_simple_cutter(
    radius: f64,
    straight_depth: f64,
    master_prefix: &str,
) -> Result<BrepSolid, String> {
    let mut solid = make_cylinder_brep(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        radius,
        straight_depth,
    )?;
    stamp_faces(&mut solid, master_prefix, &["_Hole_S", "_Hole_B", "_Hole_T"])?;
    Ok(solid)
}

/// COUNTERSINK: the revolved COUNTERSINK hole-tool profile.
fn build_countersink_cutter(
    radius: f64,
    straight_depth: f64,
    sink_dia: f64,
    sink_angle: f64,
    master_prefix: &str,
) -> Result<BrepSolid, String> {
    let sink_radius = radius.max(sink_dia * 0.5);
    let angle_rad = sink_angle * std::f64::consts::PI / 180.0;
    let sink_height = (sink_radius - radius) / (angle_rad * 0.5).tan();
    if !(sink_height > 0.0) {
        return Err(
            "hole: countersink has no recess (countersinkDiameter must exceed diameter)".into(),
        );
    }
    let core_depth = (straight_depth - sink_height).max(0.0);
    // Profile in the z=0 half-plane: x = radial, y = axial (built identically upstream).
    let (profile, side_names) = if core_depth <= 0.0 {
        (
            vec![
                make_line(pt(0.0, 0.0), pt(sink_radius, 0.0))?,
                make_line(pt(sink_radius, 0.0), pt(radius, sink_height))?,
                make_line(pt(radius, sink_height), pt(0.0, sink_height))?,
                make_line(pt(0.0, sink_height), pt(0.0, 0.0))?,
            ],
            vec![
                named(master_prefix, "_CSK_T"),
                named(master_prefix, "_CSK_S"),
                named(master_prefix, "_CSK_B"),
                None,
            ],
        )
    } else {
        (
            vec![
                make_line(pt(0.0, 0.0), pt(sink_radius, 0.0))?,
                make_line(pt(sink_radius, 0.0), pt(radius, sink_height))?,
                make_line(pt(radius, sink_height), pt(radius, straight_depth))?,
                make_line(pt(radius, straight_depth), pt(0.0, straight_depth))?,
                make_line(pt(0.0, straight_depth), pt(0.0, 0.0))?,
            ],
            vec![
                named(master_prefix, "_CSK_T"),
                named(master_prefix, "_CSK_S"),
                named(master_prefix, "_Hole_S"),
                named(master_prefix, "_Hole_B"),
                None,
            ],
        )
    };
    revolve_full_turn(&profile, &side_names)
}

/// COUNTERBORE: the revolved COUNTERBORE hole-tool profile.
fn build_counterbore_cutter(
    radius: f64,
    straight_depth: f64,
    bore_dia: f64,
    bore_depth: f64,
    master_prefix: &str,
) -> Result<BrepSolid, String> {
    if !(bore_depth > 0.0) {
        return Err("hole: counterbore has no recess (set counterboreDepth)".into());
    }
    let bore_radius = radius.max(bore_dia * 0.5);
    let core_depth = (straight_depth - bore_depth).max(0.0);
    let (profile, side_names) = if core_depth <= 0.0 {
        (
            vec![
                make_line(pt(0.0, 0.0), pt(bore_radius, 0.0))?,
                make_line(pt(bore_radius, 0.0), pt(bore_radius, bore_depth))?,
                make_line(pt(bore_radius, bore_depth), pt(0.0, bore_depth))?,
                make_line(pt(0.0, bore_depth), pt(0.0, 0.0))?,
            ],
            vec![
                named(master_prefix, "_CBore_T"),
                named(master_prefix, "_CBore_S"),
                named(master_prefix, "_CBore_B"),
                None,
            ],
        )
    } else {
        (
            vec![
                make_line(pt(0.0, 0.0), pt(bore_radius, 0.0))?,
                make_line(pt(bore_radius, 0.0), pt(bore_radius, bore_depth))?,
                make_line(pt(bore_radius, bore_depth), pt(radius, bore_depth))?,
                make_line(pt(radius, bore_depth), pt(radius, straight_depth))?,
                make_line(pt(radius, straight_depth), pt(0.0, straight_depth))?,
                make_line(pt(0.0, straight_depth), pt(0.0, 0.0))?,
            ],
            vec![
                named(master_prefix, "_CBore_T"),
                named(master_prefix, "_CBore_S"),
                named(master_prefix, "_Shoulder"),
                named(master_prefix, "_Hole_S"),
                named(master_prefix, "_Hole_B"),
                None,
            ],
        )
    };
    revolve_full_turn(&profile, &side_names)
}

/// A revolve profile point in the z=0 half-plane (x = radial, y = axial).
fn pt(radial: f64, axial: f64) -> Vec3 {
    Vec3::new(radial, axial, 0.0)
}

fn named(prefix: &str, suffix: &str) -> Option<String> {
    Some(format!("{prefix}{suffix}"))
}

/// Full-turn revolve of `profile` about `+Y` through the origin, with the FINAL
/// input-aligned `side_names` (the closing axis segment carries `None` and the
/// kernel skips it; a full turn emits no caps).
fn revolve_full_turn(
    profile: &[NurbsCurve],
    side_names: &[Option<String>],
) -> Result<BrepSolid, String> {
    revolve_profile_brep_named(
        profile,
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        std::f64::consts::TAU,
        side_names,
        &[],
    )
}

/// Stamp `${prefix}${suffix[i]}` on shell-0's faces, IN ORDER (the builder emits a
/// fixed face order).
fn stamp_faces(solid: &mut BrepSolid, prefix: &str, suffixes: &[&str]) -> Result<(), String> {
    let faces = &mut solid
        .shells
        .get_mut(0)
        .ok_or("hole cutter produced no shell")?
        .faces;
    if faces.len() != suffixes.len() {
        return Err(format!(
            "hole cutter produced {} faces, expected {}",
            faces.len(),
            suffixes.len()
        ));
    }
    for (face, suffix) in faces.iter_mut().zip(suffixes) {
        face.name = Some(format!("{prefix}{suffix}"));
    }
    Ok(())
}

/// Replace `master_prefix` with `hole_prefix` on every face whose name starts with
/// it. Faces without the prefix are untouched.
fn reprefix_faces(solid: &mut BrepSolid, master_prefix: &str, hole_prefix: &str) {
    if master_prefix == hole_prefix {
        return;
    }
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            if let Some(name) = &face.name {
                if let Some(rest) = name.strip_prefix(master_prefix) {
                    face.name = Some(format!("{hole_prefix}{rest}"));
                }
            }
        }
    }
}

/// The placement affine (row-major, as [`AffineTransform::new`] reads): local `+Y`
/// (the cutter axis) maps to `normal`; local origin maps to `origin`. Columns are
/// the orthonormal basis `[x, up=normal, z]`, a proper rotation (det +1), ported
/// exactly from the `buildBasisFromNormal` (including the `|up.y| < 0.9` ref
/// switch that keeps `x` well-conditioned near the poles).
fn placement_matrix(normal: Vec3, origin: Vec3) -> Result<[f64; 16], String> {
    let up = normal.normalized()?;
    let reference = if up.y.abs() < 0.9 {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let mut x = reference.cross(up);
    if x.length_squared() < 1e-12 {
        x = Vec3::new(1.0, 0.0, 0.0);
    }
    let x = x.normalized()?;
    let z = x.cross(up).normalized()?;
    Ok([
        x.x, up.x, z.x, origin.x, //
        x.y, up.y, z.y, origin.y, //
        x.z, up.z, z.z, origin.z, //
        0.0, 0.0, 0.0, 1.0,
    ])
}

/// The exact-vertex AABB of a resident solid (`None` for a vertexless solid).
/// Approximates the tessellated `Box3.setFromObject`; the through-all 1.5x
/// margin covers the curved slack.
fn with_solid_bbox(handle: u32) -> Result<Option<(Vec3, Vec3)>, String> {
    crate::with_registered_solid_str(handle, |solid| {
        let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for vertex in &solid.vertices {
            let p = vertex.point;
            min = Vec3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
            max = Vec3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
        }
        if !min.x.is_finite() {
            return Ok(None);
        }
        Ok(Some((min, max)))
    })
}

/// Read the `boolean` param (`{ operation, targets, mergeCoplanarFaces }`).
fn read_boolean_param(ctx: &FeatureContext) -> BooleanParam {
    let boolean = ctx.param("boolean");
    let operation = boolean
        .and_then(|value| value.get("operation"))
        .and_then(|value| value.as_str())
        .unwrap_or("SUBTRACT")
        .to_uppercase();
    let target = boolean
        .and_then(|value| value.get("targets"))
        .and_then(|value| value.as_array())
        .and_then(|array| {
            array.iter().find_map(|entry| {
                entry
                    .as_str()
                    .or_else(|| entry.get("name").and_then(|name| name.as_str()))
            })
        })
        .map(|name| name.trim().to_string())
        .filter(|name| !name.is_empty());
    let merge_coplanar_faces = boolean
        .and_then(|value| value.get("mergeCoplanarFaces"))
        .and_then(|value| value.as_bool())
        .unwrap_or(true);
    BooleanParam {
        operation,
        target,
        merge_coplanar_faces,
    }
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected placement sketch drives `face`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.sketches > 0
}

/// Dialog field-visibility hook (see [`crate::feature_pipeline::feature_hidden_params`]).
/// Given the dialog's CURRENT param values, return the param keys that do not
/// apply and should be hidden. The shared form engine calls this every frame, so
/// it covers both the first display and every field change with one pure function.
///
/// The recess parameters only apply to their own hole type, and the straight
/// depth is meaningless once the hole cuts through all — so the simple/threaded
/// hole shows neither recess, and a through hole drops its depth field.
pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
    let countersink = ["countersinkDiameter", "countersinkAngle"];
    let counterbore = ["counterboreDiameter", "counterboreDepth"];
    let hole_type = values.get("holeType").and_then(serde_json::Value::as_str).unwrap_or("SIMPLE");
    let mut hidden: Vec<&'static str> = match hole_type {
        "COUNTERSINK" => counterbore.to_vec(),
        "COUNTERBORE" => countersink.to_vec(),
        // SIMPLE, THREADED, or anything unrecognized: no recess at all.
        _ => countersink.into_iter().chain(counterbore).collect(),
    };
    // The straight depth is ignored when the hole goes through all (its own hint
    // says so) — hide it rather than let the user dial in a value that does nothing.
    if values.get("throughAll").and_then(serde_json::Value::as_bool).unwrap_or(false) {
        hidden.push("depth");
    }
    hidden
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "H",
    "shortName": "H",
    "longName": "Hole",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the hole feature"
        },
        "face": {
            "type": "reference_selection",
            "label": "Placement sketch",
            "selectionFilter": [
                "SKETCH"
            ],
            "multiple": false,
            "minSelections": 1,
            "default_value": null,
            "hint": "Select a sketch to place the hole"
        },
        "holeType": {
            "type": "options",
            "label": "Hole type",
            "options": [
                "SIMPLE",
                "COUNTERSINK",
                "COUNTERBORE",
                "THREADED"
            ],
            "default_value": "SIMPLE",
            "hint": "Choose the hole style"
        },
        "diameter": {
            "type": "number",
            "label": "Diameter",
            "default_value": 6,
            "min": 0,
            "step": 0.1,
            "hint": "Straight hole diameter"
        },
        "depth": {
            "type": "number",
            "label": "Depth",
            "default_value": 10,
            "min": 0,
            "step": 0.1,
            "hint": "Straight portion depth (ignored when Through All)"
        },
        "throughAll": {
            "type": "boolean",
            "label": "Through all",
            "default_value": false,
            "hint": "Cut through the entire target thickness"
        },
        "countersinkDiameter": {
            "type": "number",
            "label": "Countersink diameter",
            "default_value": 10,
            "min": 0,
            "step": 0.1,
            "hint": "Major diameter of the countersink"
        },
        "countersinkAngle": {
            "type": "number",
            "label": "Countersink angle (deg)",
            "default_value": 82,
            "min": 1,
            "max": 179,
            "step": 1,
            "hint": "Included angle of the countersink"
        },
        "counterboreDiameter": {
            "type": "number",
            "label": "Counterbore diameter",
            "default_value": 10,
            "min": 0,
            "step": 0.1,
            "hint": "Major diameter of the counterbore"
        },
        "counterboreDepth": {
            "type": "number",
            "label": "Counterbore depth",
            "default_value": 3,
            "min": 0,
            "step": 0.1,
            "hint": "Depth of the counterbore recess"
        },
        "boolean": {
            "type": "boolean_operation",
            "label": "Boolean",
            "default_value": {
                "targets": [],
                "operation": "SUBTRACT",
                "mergeCoplanarFaces": true
            },
            "hint": "Targets to cut; defaults to the selected body"
        }
    }
})
}

