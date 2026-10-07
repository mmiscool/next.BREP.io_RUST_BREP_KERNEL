//! SM.CUTOUT removes sketch regions from a sheet-metal body's [`SheetTree`].
//! Cuts become [`Hole`]s on the affected flats, so later features and flat
//! patterns retain them.
//!
//! Parameters:
//! - `sheet` names the target sheet body.
//! - `profile` names a sketch or its `{sketch}:FACE` alias. Each region cuts
//!   independently; inner loops preserve material islands. Solid/face cutters
//!   are unsupported.
//! - `forwardDistance` / `backDistance` measure depth from the sketch plane
//!   along ±normal. Zero or absent means through-all on that side.
//! - `keepTool` publishes separate cutter solids as `{id}:CUTTER` or
//!   `{id}:CUTTER:{n}`. The profile sketch is consumed on success.
//!
//! Folded flat placements map profile loops and depth intervals into each
//! parallel flat's local frame. Rational control points and weights are
//! preserved. A region may cut several parallel flats, including both plies
//! of a hem, and may overhang an outline to form an edge notch.
//!
//! Rejected cuts include regions that reach a nonparallel flat, land on no
//! flat, or enter a bend-wedge footprint. Bend checks use the outer radius
//! and a widened depth window. A tool that threads a bend arc while missing
//! both adjacent slabs and cutting another flat can escape these checks.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::sheet_metal::evaluate::{FlatPlacement, WedgeStrip};
use crate::feature_pipeline::sheet_metal::{self, Flat, Hole};
use crate::feature_pipeline::{FeatureContext, FeatureResult, ProfileLoop, SketchProfile};
use crate::{NurbsCurve, Vec3};

/// |sketch_normal · flat_normal| above this = the planes are parallel. Frames
/// come from chained exact rotations, so true-parallel dots sit within ~1e-14
/// of 1; anything below is a genuinely tilted plane (oblique guard territory).
const PARALLEL_DOT: f64 = 1.0 - 1e-9;

/// Lower bound on |sketch_normal · flat_normal| for a manufacturable OBLIQUE cut.
/// The through-cut footprint spreads by `t·tan(θ)` where `cos(θ) = |s|`, i.e. as
/// `1/|s|`; below this the tool comes in almost tangent to the sheet and the
/// footprint blows up past several thicknesses. At `|s| = 0.25` the tilt is
/// ~75.5° off perpendicular (`tan θ ≈ 3.9`), so the footprint spreads ~3.9× the
/// sheet thickness — the practical grazing limit. Anything shallower is a loud
/// grazing error.
const MIN_OBLIQUE: f64 = 0.25;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let Some(body_name) = common::sheet_body_name(ctx) else {
        return Ok(FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone()));
    };
    let Some(handle) = ctx.scene.resolve_solid(&body_name) else {
        let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
        result.unresolved.push(body_name);
        return Ok(result);
    };
    let Some(mut tree) = sheet_metal::get_tree(handle) else {
        return Err("sheet-metal cutout: target is not a sheet-metal body".into());
    };

    // --- Resolve the cut profile (sketch only, headless) ---
    let profile_name = common::normalize_profile_alias(
        common::first_reference_name(ctx.param("profile"))
            .ok_or("sheet-metal cutout: missing `profile` reference selection")?,
    );
    let profile = match ctx.scene.resolve_profile(&profile_name) {
        Some(profile) => profile,
        None => {
            if ctx.scene.resolve_face(&profile_name).is_some() {
                return Err(format!(
                    "sheet-metal cutout: profile '{profile_name}' is a resident solid face — \
                     face/solid cutters are not supported; select a sketch profile"
                ));
            }
            if ctx.scene.resolve_solid(&profile_name).is_some() {
                return Err(format!(
                    "sheet-metal cutout: profile '{profile_name}' is a solid body — \
                     solid-tool cutouts are not supported; select a sketch profile"
                ));
            }
            return Err(format!(
                "sheet-metal cutout: profile '{profile_name}' not found (no sketch profile)"
            ));
        }
    };
    if profile.regions.is_empty() {
        return Err(format!(
            "sheet-metal cutout: profile '{profile_name}' has no closed regions to cut with"
        ));
    }

    // --- Depths + tool retention ---
    // 0 / absent = through-all on that side (see module docs for the divergence).
    let forward = common::optional_number(ctx, "forwardDistance")?
        .unwrap_or(0.0)
        .max(0.0);
    let back = common::optional_number(ctx, "backDistance")?
        .unwrap_or(0.0)
        .max(0.0);
    let keep_tool = ctx
        .param("keepTool")
        .and_then(|value| value.as_bool())
        .unwrap_or(false);

    // --- Map every profile region onto the folded flats ---
    let placements = sheet_metal::evaluate::folded_flat_placements(&tree)?;
    let half_t = tree.thickness * 0.5;
    let mut cuts: Vec<(String, Hole)> = Vec::new();
    for (region_index, region) in profile.regions.iter().enumerate() {
        let outer = region
            .first()
            .ok_or("sheet-metal cutout: profile region has no outer loop")?;
        // The outer loop sampled in the SKETCH plane's own (u, v) — the oblique
        // guard's frame of reference.
        let sketch_poly = sample_loop_poly(outer, |p| {
            let d = p.sub(profile.origin);
            [d.dot(profile.x_axis), d.dot(profile.y_axis)]
        })?;
        let mut landed = false;
        for placement in &placements {
            let s = profile.z_axis.dot(placement.w);
            if s.abs() >= PARALLEL_DOT {
                let side = if s >= 0.0 { 1.0 } else { -1.0 };
                // Exact rigid map of the loop into the flat frame + the sketch
                // plane's offset along the flat normal.
                let (mapped_outer, plane_z) = loop_to_flat(placement, outer)?;
                let (z_min, z_max) = depth_span(plane_z, side, forward, back);
                let flat_poly = sample_loop_poly(outer, |p| {
                    let d = p.sub(placement.origin);
                    [d.dot(placement.u), d.dot(placement.v)]
                })?;
                // Bend-crossing guard: the loop must not enter a bend-wedge
                // footprint. z-gated by a widened window (the wedge never
                // reaches farther than 2×reach past the slab), otherwise
                // depth-independent — conservative by design.
                for strip in &placement.wedge_strips {
                    let window = half_t + 2.0 * strip.reach;
                    if span_overlaps(z_min, z_max, -window, window)
                        && polygon_overlaps_strip(&flat_poly, strip)
                    {
                        return Err(format!(
                            "sheet-metal cutout: profile region {region_index} crosses bend \
                             '{}' at flat '{}' — cutting across a bend arc is not supported; \
                             keep each cut inside one flat",
                            strip.bend_id, placement.id
                        ));
                    }
                }
                if !span_overlaps(z_min, z_max, -half_t, half_t) {
                    continue; // the cut depth never reaches this flat's material
                }
                if !polygons_overlap(&flat_poly, &placement.outline) {
                    continue; // the loop misses this flat's footprint
                }
                let islands = region[1..]
                    .iter()
                    .map(|island| loop_to_flat(placement, island).map(|(curves, _)| curves))
                    .collect::<Result<Vec<_>, _>>()?;
                cuts.push((
                    placement.id.clone(),
                    Hole {
                        outer: mapped_outer,
                        islands,
                        z_min,
                        z_max,
                    },
                ));
                landed = true;
            } else if oblique_tool_reaches_slab(
                profile, &sketch_poly, forward, back, placement, half_t,
            ) {
                // The tool reaches a NON-parallel flat. An OBLIQUE cut in the
                // manufacturable band gets a perpendicular-walled through hole
                // sized for the angled tool (the two-footprint union); a
                // near-tangent (grazing) tilt is a loud error.
                if s.abs() < MIN_OBLIQUE {
                    return Err(format!(
                        "sheet-metal cutout: profile region {region_index}'s oblique cut grazes \
                         flat '{}' at {:.1}° from perpendicular — a near-tangent oblique cut is \
                         unmanufacturable (its footprint spreads past several sheet thicknesses); \
                         sketch the cut closer to perpendicular to the flat",
                        placement.id,
                        s.abs().clamp(0.0, 1.0).acos().to_degrees()
                    ));
                }
                let hole = oblique_hole(
                    profile, region, placement, half_t, forward, back, region_index,
                )?;
                // Bend-crossing guard runs against the UNION footprint (an
                // oblique through cut fills the whole slab z-range, so the wedge
                // z-gate always passes — check the polygon unconditionally).
                let union_poly = sample_flat_curves(&hole.outer)?;
                for strip in &placement.wedge_strips {
                    if polygon_overlaps_strip(&union_poly, strip) {
                        return Err(format!(
                            "sheet-metal cutout: profile region {region_index} crosses bend \
                             '{}' at flat '{}' — cutting across a bend arc is not supported; \
                             keep each cut inside one flat",
                            strip.bend_id, placement.id
                        ));
                    }
                }
                if !polygons_overlap(&union_poly, &placement.outline) {
                    continue; // the union footprint misses this flat's material
                }
                cuts.push((placement.id.clone(), hole));
                landed = true;
            }
        }
        if !landed {
            return Err(format!(
                "sheet-metal cutout: profile region {region_index} does not land on any flat \
                 of '{body_name}' — the loop must overlap a flat and the cut span must reach \
                 its material (depth 0 = through-all)"
            ));
        }
    }

    // --- Bake the holes into the tree and re-evaluate ---
    for (flat_id, hole) in cuts {
        tree.root.find_by_id_mut(&flat_id)
            .ok_or_else(|| format!("sheet-metal cutout: flat `{flat_id}` vanished from the tree"))?
            .holes
            .push(hole);
    }
    let added = sheet_metal::register_folded(tree.clone(), &body_name)?;
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    result.added.push(added);
    result.removed.push(body_name);
    // Sheet-metal contract: the profile sketch is ALWAYS consumed.
    common::consume_sketch_always(&profile_name, &mut result);

    // --- keepTool: add the cutter solid(s), one per region, never unioned ---
    if keep_tool {
        let cutters = build_region_cutters(ctx, profile, forward, back, &placements, half_t, &tree)?;
        for (name, cutter) in cutters {
            result.added.push(common::register_added(cutter, &name));
        }
    }
    Ok(result)
}

// ===========================================================================
// Depth spans
// ===========================================================================

/// The flat-local z window a cut removes. `plane_z` is the sketch plane's offset
/// along the flat normal; `side` = sign(sketch_normal · flat_normal). `forward`
/// extends along +sketch normal (= `side`·flat z), `back` along −; `0` = open
/// (through-all) on that side.
fn depth_span(plane_z: f64, side: f64, forward: f64, back: f64) -> (Option<f64>, Option<f64>) {
    let fwd = (forward > 0.0).then(|| plane_z + side * forward);
    let bck = (back > 0.0).then(|| plane_z - side * back);
    if side >= 0.0 {
        (bck, fwd) // (z_min, z_max)
    } else {
        (fwd, bck)
    }
}

/// Does the (possibly half-open) span `[z_min, z_max]` overlap `[lo, hi]`?
fn span_overlaps(z_min: Option<f64>, z_max: Option<f64>, lo: f64, hi: f64) -> bool {
    let eps = 1e-9 * (hi - lo).abs().max(1.0);
    let a = z_min.unwrap_or(f64::NEG_INFINITY);
    let b = z_max.unwrap_or(f64::INFINITY);
    a < hi - eps && b > lo + eps && b > a + eps
}

// ===========================================================================
// Exact loop mapping
// ===========================================================================

/// Rigidly map one profile loop's WORLD curves into a flat's local frame:
/// control point `p` → `((p−origin)·u, (p−origin)·v, 0)`, weights kept — the
/// same exact rational-curve-preserving map as [`common::profile_loop_uv_curves`]
/// but into the FLAT's frame instead of the profile's own. Returns the curves
/// plus the loop plane's offset along the flat normal (all control points of a
/// parallel-plane loop share it; a spread is a hard error).
fn loop_to_flat(
    placement: &FlatPlacement,
    profile_loop: &ProfileLoop,
) -> Result<(Vec<NurbsCurve>, f64), String> {
    let mut curves = Vec::with_capacity(profile_loop.curves.len());
    let mut z_lo = f64::INFINITY;
    let mut z_hi = f64::NEG_INFINITY;
    for curve in &profile_loop.curves {
        let mut control_points = Vec::with_capacity(curve.control_points.len());
        for cp in &curve.control_points {
            let delta = cp.point()?.sub(placement.origin);
            let z = delta.dot(placement.w);
            z_lo = z_lo.min(z);
            z_hi = z_hi.max(z);
            control_points.push(crate::Vec4::from_point(
                Vec3::new(delta.dot(placement.u), delta.dot(placement.v), 0.0),
                cp.w,
            ));
        }
        curves.push(NurbsCurve::new(
            curve.degree,
            curve.knots.clone(),
            control_points,
        )?);
    }
    if !(z_hi - z_lo <= 1e-6 * z_hi.abs().max(z_lo.abs()).max(1.0)) {
        return Err(format!(
            "sheet-metal cutout: profile loop is not planar against flat '{}' (z spread {})",
            placement.id,
            z_hi - z_lo
        ));
    }
    Ok((curves, (z_lo + z_hi) * 0.5))
}

/// Sample a profile loop into a 2D polygon through `map` (16 points per curve —
/// generous, because the overlap decision biases PERMISSIVE: a false-positive
/// hole is a boolean no-op, a false negative silently skips material).
fn sample_loop_poly(
    profile_loop: &ProfileLoop,
    map: impl Fn(Vec3) -> [f64; 2],
) -> Result<Vec<[f64; 2]>, String> {
    let mut poly = Vec::new();
    for curve in &profile_loop.curves {
        let [t0, t1] = curve.domain()?;
        for step in 0..16 {
            let t = t0 + (t1 - t0) * (step as f64) / 16.0;
            poly.push(map(curve.evaluate(t)?));
        }
    }
    if poly.len() < 3 {
        return Err("sheet-metal cutout: degenerate profile loop".into());
    }
    Ok(poly)
}

// ===========================================================================
// Guards
// ===========================================================================

/// Does the region's tool (the loop prism along the sketch normal, clamped by
/// the depth span) reach a NON-parallel flat's slab? Exact in-plane: the slab's
/// silhouette in the sketch frame is the union of its faces' projections
/// (outline at ±t/2 + the side quads), each an exact affine image. Conservative
/// in depth: the sketch-z interval of the slab corners vs the span (depth varies
/// across the footprint, so a blind tool stopping just short of an oblique flat
/// may still trip this — a loud false error, never a silent wrong cut).
fn oblique_tool_reaches_slab(
    profile: &SketchProfile,
    sketch_poly: &[[f64; 2]],
    forward: f64,
    back: f64,
    placement: &FlatPlacement,
    half_t: f64,
) -> bool {
    let n = placement.outline.len();
    if n < 3 {
        return false;
    }
    let corner = |i: usize, z: f64| {
        let p = placement.outline[i % n];
        placement
            .origin
            .add(placement.u.scale(p[0]))
            .add(placement.v.scale(p[1]))
            .add(placement.w.scale(z))
    };
    let to_sketch = |p: Vec3| {
        let d = p.sub(profile.origin);
        ([d.dot(profile.x_axis), d.dot(profile.y_axis)], d.dot(profile.z_axis))
    };
    // Depth interval of the slab along the sketch normal vs the cut span.
    let mut s_lo = f64::INFINITY;
    let mut s_hi = f64::NEG_INFINITY;
    for i in 0..n {
        for z in [-half_t, half_t] {
            let (_, sz) = to_sketch(corner(i, z));
            s_lo = s_lo.min(sz);
            s_hi = s_hi.max(sz);
        }
    }
    let z_min = (back > 0.0).then_some(-back);
    let z_max = (forward > 0.0).then_some(forward);
    if !span_overlaps(z_min, z_max, s_lo, s_hi) {
        return false;
    }
    // Silhouette: outline at -t/2, at +t/2, and every side quad, projected.
    let bottom: Vec<[f64; 2]> = (0..n).map(|i| to_sketch(corner(i, -half_t)).0).collect();
    let top: Vec<[f64; 2]> = (0..n).map(|i| to_sketch(corner(i, half_t)).0).collect();
    if polygons_overlap(sketch_poly, &bottom) || polygons_overlap(sketch_poly, &top) {
        return true;
    }
    for i in 0..n {
        let quad = vec![
            to_sketch(corner(i, -half_t)).0,
            to_sketch(corner(i + 1, -half_t)).0,
            to_sketch(corner(i + 1, half_t)).0,
            to_sketch(corner(i, half_t)).0,
        ];
        if polygons_overlap(sketch_poly, &quad) {
            return true;
        }
    }
    false
}

// ===========================================================================
// Oblique cut (sketch plane tilted to the flat)
// ===========================================================================

/// Build the perpendicular-walled THROUGH hole for an OBLIQUE cut on `placement`.
///
/// The manufacturing intent (a domain expert's method): the angled tool must
/// clear the sheet with the hole's walls kept PERPENDICULAR to the sheet. The
/// tool pierces the flat's TOP and BOTTOM face planes at two different in-plane
/// footprints; the hole is the perpendicular prism over their 2D UNION (subtract
/// two perpendicular solids ⇒ subtract their union, `X\A\B = X\(A∪B)`).
///
/// Method, exactly the specified mechanism (no Minkowski sweep, no ideal swept
/// hole — the sub-tolerance waist under-coverage is accepted):
/// 1. THROUGH-only gate: the cut's depth interval along the sketch normal must
///    pierce BOTH face planes across the whole footprint (blind ⇒ loud error).
/// 2. Affine-project each loop's control points along the sketch normal onto the
///    top and bottom planes, expressed in the flat `(u, v)`. The map is affine,
///    so weights are unchanged and a circle projects to an EXACT ellipse.
/// 3. UNION the two footprints via the existing 3D boolean: extrude each
///    projected region (outer minus its islands) into a perpendicular prism,
///    union the two, and read the union's perpendicular top cross-section back
///    out as the hole's outer loop + island loops. Islands / tangency /
///    merged-vs-disjoint all fall out of the boolean.
/// 4. DISJOINT footprints ⇒ loud "too steep" error (the tool cannot pass).
fn oblique_hole(
    profile: &SketchProfile,
    region: &[ProfileLoop],
    placement: &FlatPlacement,
    half_t: f64,
    forward: f64,
    back: f64,
    region_index: usize,
) -> Result<Hole, String> {
    let sw = profile.z_axis.dot(placement.w); // = s; |sw| >= MIN_OBLIQUE (caller-gated)
    let outer = region
        .first()
        .ok_or("sheet-metal cutout: profile region has no outer loop")?;

    // --- 1. THROUGH-cut gate. The cut interval along +sketch-normal is
    //        [-back, +forward] with 0 = open (±∞). Every boundary sample's hit on
    //        BOTH face planes must fall inside it, or the oblique cut is blind. ---
    let s_min = if back > 0.0 { -back } else { f64::NEG_INFINITY };
    let s_max = if forward > 0.0 { forward } else { f64::INFINITY };
    let eps = 1e-9 * half_t.max(1.0);
    for curve in &outer.curves {
        let [t0, t1] = curve.domain()?;
        for step in 0..16 {
            let t = t0 + (t1 - t0) * (step as f64) / 16.0;
            let p = curve.evaluate(t)?;
            let d_w = p.sub(placement.origin).dot(placement.w);
            let s_top = (half_t - d_w) / sw;
            let s_bot = (-half_t - d_w) / sw;
            let hits_through = s_top >= s_min - eps
                && s_top <= s_max + eps
                && s_bot >= s_min - eps
                && s_bot <= s_max + eps;
            if !hits_through {
                return Err(format!(
                    "sheet-metal cutout: profile region {region_index} on flat '{}': oblique \
                     cutout must pass through the sheet (a partial-depth oblique cut is not \
                     supported)",
                    placement.id
                ));
            }
        }
    }

    // --- 2. Affine footprints on the top (+half_t) and bottom (-half_t) planes. ---
    let top_outer = project_loop_to_face(profile, outer, placement, half_t)?;
    let bot_outer = project_loop_to_face(profile, outer, placement, -half_t)?;
    let top_islands = region[1..]
        .iter()
        .map(|island| project_loop_to_face(profile, island, placement, half_t))
        .collect::<Result<Vec<_>, _>>()?;
    let bot_islands = region[1..]
        .iter()
        .map(|island| project_loop_to_face(profile, island, placement, -half_t))
        .collect::<Result<Vec<_>, _>>()?;

    // --- 4. DISJOINT gate (pre-boolean): the two outer footprints must overlap. ---
    let top_poly = sample_flat_curves(&top_outer)?;
    let bot_poly = sample_flat_curves(&bot_outer)?;
    if !polygons_overlap(&top_poly, &bot_poly) {
        return Err(format!(
            "sheet-metal cutout: profile region {region_index} on flat '{}': cut angle too \
             steep for sheet thickness — the tool cannot pass through",
            placement.id
        ));
    }

    // --- 3. UNION the two perpendicular prisms and read the cross-section back. ---
    // Prism height: the footprint bbox diagonal, so the union solid stays
    // well-conditioned (aspect ratio ~1). Only the perpendicular cross-section
    // shape is read back — the absolute height is irrelevant.
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in top_poly.iter().chain(bot_poly.iter()) {
        lo[0] = lo[0].min(p[0]);
        lo[1] = lo[1].min(p[1]);
        hi[0] = hi[0].max(p[0]);
        hi[1] = hi[1].max(p[1]);
    }
    let height = (hi[0] - lo[0]).hypot(hi[1] - lo[1]).max(1e-3);

    let region_a = footprint_prism(&top_outer, &top_islands, height)?;
    let region_b = footprint_prism(&bot_outer, &bot_islands, height)?;
    let union = common::union_region_solids(vec![region_a, region_b])
        .map_err(|e| format!("sheet-metal cutout: oblique footprint union failed: {e}"))?;
    // Belt-and-suspenders disjoint check: a tangent-only overlap slips past the
    // 2D pre-check but yields two shells here — same loud "too steep" error.
    if union.shells.len() != 1 {
        return Err(format!(
            "sheet-metal cutout: profile region {region_index} on flat '{}': cut angle too \
             steep for sheet thickness — the tool cannot pass through",
            placement.id
        ));
    }

    let (outer_curves, islands) = cross_section_loops(&union, height)?;
    Ok(Hole {
        outer: outer_curves,
        islands,
        z_min: Some(-half_t),
        z_max: Some(half_t),
    })
}

/// Affine-project one profile loop along the sketch normal onto the flat's face
/// plane at flat-normal offset `offset` (±half_t), expressed in the flat `(u, v)`
/// at local `z = 0`. For a control point `p`, `s = (offset − (p−origin)·w) / (
/// z_axis·w)`, `hit = p + s·z_axis`, `(u, v) = ((hit−origin)·u, (hit−origin)·v)`.
/// The map is AFFINE, so mapping control points and keeping weights preserves the
/// rational curve exactly — a circle becomes a true ellipse (no densify-refit).
fn project_loop_to_face(
    profile: &SketchProfile,
    profile_loop: &ProfileLoop,
    placement: &FlatPlacement,
    offset: f64,
) -> Result<Vec<NurbsCurve>, String> {
    let sw = profile.z_axis.dot(placement.w); // |sw| >= MIN_OBLIQUE
    let mut curves = Vec::with_capacity(profile_loop.curves.len());
    for curve in &profile_loop.curves {
        let mut control_points = Vec::with_capacity(curve.control_points.len());
        for cp in &curve.control_points {
            let p = cp.point()?;
            let s = (offset - p.sub(placement.origin).dot(placement.w)) / sw;
            let hit = p.add(profile.z_axis.scale(s));
            let d = hit.sub(placement.origin);
            control_points.push(crate::Vec4::from_point(
                Vec3::new(d.dot(placement.u), d.dot(placement.v), 0.0),
                cp.w,
            ));
        }
        curves.push(NurbsCurve::new(
            curve.degree,
            curve.knots.clone(),
            control_points,
        )?);
    }
    Ok(curves)
}

/// A perpendicular prism over one footprint region: the outer `(u, v)` loop
/// extruded along +z over `[0, height]`, each island loop carved out (its prism
/// overhangs both caps so the carve is a clean clearance subtract). Curves are
/// interpreted in local `(x = u, y = v, z)`, so the read-back cross-section is
/// already in the flat frame.
fn footprint_prism(
    outer: &[NurbsCurve],
    islands: &[Vec<NurbsCurve>],
    height: f64,
) -> Result<crate::BrepSolid, String> {
    let mut solid = extrude_footprint(outer, 0.0, height)?;
    for island in islands {
        if island.is_empty() {
            continue;
        }
        let island_solid = extrude_footprint(island, -0.5 * height, 2.0 * height)?;
        solid = common::subtract_solid(solid, island_solid)?;
    }
    Ok(solid)
}

/// Extrude a closed flat `(u, v, 0)` loop into a prism spanning `[z_start,
/// z_start + z_len]` along +z. A lone closed curve (a full circle/ellipse) is
/// split at mid-domain first — `extrude_profile_brep` needs ≥ 2 curves.
fn extrude_footprint(
    curves: &[NurbsCurve],
    z_start: f64,
    z_len: f64,
) -> Result<crate::BrepSolid, String> {
    let drop = Vec3::new(0.0, 0.0, z_start);
    let mut profile = curves
        .iter()
        .map(|curve| common::translate_curve(curve, drop))
        .collect::<Result<Vec<_>, _>>()?;
    if profile.len() == 1 {
        let single = profile.pop().expect("one curve");
        let domain = single.domain()?;
        let (first, second) = single.split((domain[0] + domain[1]) * 0.5)?;
        profile.push(first);
        profile.push(second);
    }
    crate::extrude_profile_brep(&profile, Vec3::new(0.0, 0.0, 1.0), z_len).map_err(String::from)
}

/// Read the union prism's perpendicular TOP cross-section back out as closed
/// `(u, v)` loops: the outer boundary (largest |area|) + island loops. The top
/// cap is the single planar face all of whose loop edges sit at `z = z_top`
/// (coplanar-merge fuses the two overlapping caps into one, so the boundary has
/// no interior/overlap edges); each loop's edge curves are dropped to `z = 0`
/// and chained head-to-tail.
fn cross_section_loops(
    solid: &crate::BrepSolid,
    z_top: f64,
) -> Result<(Vec<NurbsCurve>, Vec<Vec<NurbsCurve>>), String> {
    let tol = 1e-6 * z_top.abs().max(1.0);
    let edge_by_id: std::collections::HashMap<u64, &crate::topology::EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();

    // Every face whose loop edges ALL sit in the top plane. After coplanar merge
    // there is exactly one (its loops = outer + islands); 0 or >1 means the
    // boolean did not merge the cap — a loud failure, never a partial hole.
    let mut cap_faces = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            if face.loops.is_empty() {
                continue;
            }
            let mut all_top = true;
            'face: for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    let Some(edge) = edge_by_id.get(&coedge.edge_id) else {
                        all_top = false;
                        break 'face;
                    };
                    for cp in &edge.curve.control_points {
                        if (cp.point()?.z - z_top).abs() > tol {
                            all_top = false;
                            break 'face;
                        }
                    }
                }
            }
            if all_top {
                cap_faces.push(face);
            }
        }
    }
    if cap_faces.len() != 1 {
        return Err(format!(
            "sheet-metal cutout: oblique footprint union produced {} top-cap faces (expected \
             1) — the boolean did not merge the union cross-section",
            cap_faces.len()
        ));
    }

    // Chain each loop into a closed head-to-tail curve chain in (u, v, 0). The
    // boolean trims a shared curve by NARROWING the edge's `[t0, t1]` range (not
    // by rebuilding control points), so read the TRIMMED subcurve — the full
    // curve would give the untrimmed tips and leave a gap at every trim vertex.
    let mut loops: Vec<Vec<NurbsCurve>> = Vec::new();
    for loop_record in &cap_faces[0].loops {
        let bag = loop_record
            .coedges
            .iter()
            .map(|coedge| {
                let edge = edge_by_id
                    .get(&coedge.edge_id)
                    .ok_or("sheet-metal cutout: cross-section coedge lost its edge")?;
                project_curve_uv(&subcurve(&edge.curve, edge.t0, edge.t1)?)
            })
            .collect::<Result<Vec<_>, _>>()?;
        loops.push(chain_loop(bag)?);
    }

    // Largest |area| = outer boundary; the rest are islands (holes in the cut).
    let mut best = 0usize;
    let mut best_area = -1.0;
    for (index, chain) in loops.iter().enumerate() {
        let area = loop_area(chain)?.abs();
        if area > best_area {
            best_area = area;
            best = index;
        }
    }
    let outer = loops.remove(best);
    Ok((outer, loops))
}

/// The subcurve of `curve` over the edge parameter range `[t0, t1]` (the boolean
/// trims by narrowing this range, keeping the full curve), via `split`. Boundary
/// hits (range == full domain) short-circuit to a clone.
fn subcurve(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<NurbsCurve, String> {
    let [d0, d1] = curve.domain()?;
    let tol = 1e-9 * (d1 - d0).abs().max(1.0);
    let (a, b) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
    let a = a.max(d0);
    let b = b.min(d1);
    // Drop the head before `a`, keeping the right part [a, d1].
    let right = if a > d0 + tol { curve.split(a)?.1 } else { curve.clone() };
    // Drop the tail after `b`, keeping the left part [a, b].
    let [r0, r1] = right.domain()?;
    if b > r0 + tol && b < r1 - tol {
        Ok(right.split(b)?.0)
    } else {
        Ok(right)
    }
}

/// Drop a 3D top-cap edge curve to the flat plane: each control point `(x, y, z)`
/// → `(x, y, 0)`, weights unchanged (the walls are perpendicular, so `x, y` are
/// already the flat `(u, v)`).
fn project_curve_uv(curve: &NurbsCurve) -> Result<NurbsCurve, String> {
    let mut control_points = Vec::with_capacity(curve.control_points.len());
    for cp in &curve.control_points {
        let p = cp.point()?;
        control_points.push(crate::Vec4::from_point(Vec3::new(p.x, p.y, 0.0), cp.w));
    }
    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
}

/// Assemble a bag of 2D curves into ONE closed head-to-tail chain, reversing
/// curves as needed (endpoint match within a size-relative tolerance). A gap ⇒
/// loud error. A lone already-closed curve passes through unchanged.
fn chain_loop(mut bag: Vec<NurbsCurve>) -> Result<Vec<NurbsCurve>, String> {
    if bag.is_empty() {
        return Err("sheet-metal cutout: empty oblique cross-section loop".into());
    }
    let endpoints = |curve: &NurbsCurve| -> Result<([f64; 2], [f64; 2]), String> {
        let [t0, t1] = curve.domain()?;
        let a = curve.evaluate(t0)?;
        let b = curve.evaluate(t1)?;
        Ok(([a.x, a.y], [b.x, b.y]))
    };
    let close = |a: [f64; 2], b: [f64; 2]| {
        let scale = a[0].abs().max(a[1].abs()).max(b[0].abs()).max(b[1].abs()).max(1.0);
        (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-6 * scale
    };
    let mut chain = vec![bag.remove(0)];
    let (_, mut end) = endpoints(&chain[0])?;
    while !bag.is_empty() {
        let mut pick = None;
        for (index, curve) in bag.iter().enumerate() {
            let (a, b) = endpoints(curve)?;
            if close(a, end) {
                pick = Some((index, false, b));
                break;
            }
            if close(b, end) {
                pick = Some((index, true, a));
                break;
            }
        }
        let (index, reversed, new_end) =
            pick.ok_or("sheet-metal cutout: oblique cross-section loop has a gap")?;
        let curve = bag.remove(index);
        chain.push(if reversed { curve.reversed()? } else { curve });
        end = new_end;
    }
    Ok(chain)
}

/// Shoelace signed area of a curve chain (sampled), in its own `(x, y)`.
fn loop_area(curves: &[NurbsCurve]) -> Result<f64, String> {
    let poly = sample_flat_curves(curves)?;
    let mut area = 0.0;
    let n = poly.len();
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        area += a[0] * b[1] - b[0] * a[1];
    }
    Ok(area * 0.5)
}

/// Sample a chain of flat `(x, y, 0)` curves into a 2D polygon (16 pts/curve).
fn sample_flat_curves(curves: &[NurbsCurve]) -> Result<Vec<[f64; 2]>, String> {
    let mut poly = Vec::new();
    for curve in curves {
        let [t0, t1] = curve.domain()?;
        for step in 0..16 {
            let t = t0 + (t1 - t0) * (step as f64) / 16.0;
            let p = curve.evaluate(t)?;
            poly.push([p.x, p.y]);
        }
    }
    if poly.len() < 3 {
        return Err("sheet-metal cutout: degenerate oblique footprint loop".into());
    }
    Ok(poly)
}

/// Does `poly` enter the bend-wedge strip? The strip rectangle sits beyond the
/// directed edge `a -> b` on its right, inset a hair off the edge itself so a
/// legal cut TOUCHING the fold line (numerically) does not trip it.
fn polygon_overlaps_strip(poly: &[[f64; 2]], strip: &WedgeStrip) -> bool {
    let dx = strip.b[0] - strip.a[0];
    let dy = strip.b[1] - strip.a[1];
    let len = (dx * dx + dy * dy).sqrt();
    if len <= 1e-12 || strip.reach <= 0.0 {
        return false;
    }
    let out = [dy / len, -dx / len]; // right of a -> b
    let inset = 1e-7 * strip.reach.max(1.0);
    let rect = vec![
        [strip.a[0] + out[0] * inset, strip.a[1] + out[1] * inset],
        [strip.b[0] + out[0] * inset, strip.b[1] + out[1] * inset],
        [strip.b[0] + out[0] * strip.reach, strip.b[1] + out[1] * strip.reach],
        [strip.a[0] + out[0] * strip.reach, strip.a[1] + out[1] * strip.reach],
    ];
    polygons_overlap(poly, &rect)
}

// ===========================================================================
// 2D polygon primitives
// ===========================================================================

/// Even-odd point-in-polygon (ray cast toward +x).
fn point_in_polygon(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let n = poly.len();
    let mut inside = false;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let x = a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
            if x > p[0] {
                inside = !inside;
            }
        }
    }
    inside
}

fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Segment intersection including endpoint/collinear touches (touch counts as
/// intersecting — permissive for the cut decision, conservative for guards).
fn segments_intersect(p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], p4: [f64; 2]) -> bool {
    let len1 = ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2)).sqrt();
    let len2 = ((p4[0] - p3[0]).powi(2) + (p4[1] - p3[1]).powi(2)).sqrt();
    let eps = 1e-12 * (len1 * len2).max(1e-12);
    let d1 = orient(p3, p4, p1);
    let d2 = orient(p3, p4, p2);
    let d3 = orient(p1, p2, p3);
    let d4 = orient(p1, p2, p4);
    if ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps))
        && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))
    {
        return true;
    }
    let tol = 1e-9 * len1.max(len2).max(1.0);
    let on_seg = |a: [f64; 2], b: [f64; 2], p: [f64; 2]| {
        p[0] >= a[0].min(b[0]) - tol
            && p[0] <= a[0].max(b[0]) + tol
            && p[1] >= a[1].min(b[1]) - tol
            && p[1] <= a[1].max(b[1]) + tol
    };
    (d1.abs() <= eps && on_seg(p3, p4, p1))
        || (d2.abs() <= eps && on_seg(p3, p4, p2))
        || (d3.abs() <= eps && on_seg(p1, p2, p3))
        || (d4.abs() <= eps && on_seg(p1, p2, p4))
}

/// Do two polygons overlap (share any area or touch)? Vertex containment either
/// way, or any pair of boundary segments intersecting.
fn polygons_overlap(a: &[[f64; 2]], b: &[[f64; 2]]) -> bool {
    if a.len() < 3 || b.len() < 3 {
        return false;
    }
    if a.iter().any(|p| point_in_polygon(*p, b)) || b.iter().any(|p| point_in_polygon(*p, a)) {
        return true;
    }
    for i in 0..a.len() {
        let a1 = a[i];
        let a2 = a[(i + 1) % a.len()];
        for j in 0..b.len() {
            if segments_intersect(a1, a2, b[j], b[(j + 1) % b.len()]) {
                return true;
            }
        }
    }
    false
}

// ===========================================================================
// keepTool cutters
// ===========================================================================

/// Build one WORLD-space cutter solid per profile region: the region's outer
/// loop extruded along the sketch normal over the depth span (open sides clamp
/// past the whole folded sheet), islands carved out. Named `{id}:CUTTER`, or
/// `{id}:CUTTER:{n}` (1-based) when there are several regions.
fn build_region_cutters(
    ctx: &FeatureContext,
    profile: &SketchProfile,
    forward: f64,
    back: f64,
    placements: &[FlatPlacement],
    half_t: f64,
    tree: &sheet_metal::SheetTree,
) -> Result<Vec<(String, crate::BrepSolid)>, String> {
    // The folded sheet's extent along the sketch normal — the clamp for
    // through-all sides (a margin of one thickness past the farthest face).
    let mut s_lo = f64::INFINITY;
    let mut s_hi = f64::NEG_INFINITY;
    for placement in placements {
        for p in &placement.outline {
            for z in [-half_t, half_t] {
                let world = placement
                    .origin
                    .add(placement.u.scale(p[0]))
                    .add(placement.v.scale(p[1]))
                    .add(placement.w.scale(z));
                let sz = world.sub(profile.origin).dot(profile.z_axis);
                s_lo = s_lo.min(sz);
                s_hi = s_hi.max(sz);
            }
        }
    }
    let margin = tree.thickness;
    let bottom = if back > 0.0 { -back } else { s_lo - margin };
    let top = if forward > 0.0 { forward } else { s_hi + margin };
    if !(top > bottom + 1e-12) {
        return Ok(Vec::new());
    }

    let mut cutters = Vec::new();
    for (region_index, region) in profile.regions.iter().enumerate() {
        let outer = region
            .first()
            .ok_or("sheet-metal cutout: profile region has no outer loop")?;
        // Keyed off the region's STABLE loop identity, not its position, so
        // adding or removing a DIFFERENT cutout region never renames this one's
        // walls (the same fix as the swept features' caps). A single-region
        // profile keeps the bare `{id}:CUTTER` spelling it has always had.
        let name = if profile.regions.len() == 1 {
            format!("{}:CUTTER", ctx.id)
        } else {
            format!("{}:CUTTER:{}", ctx.id, common::hole_key(outer, region_index))
        };
        let mut cutter = extrude_world_loop(&outer.curves, profile.z_axis, bottom, top, &name)?;
        for island in &region[1..] {
            // Extend the island prism half a thickness past both cutter caps so
            // the carve is a clean clearance subtract.
            let island_prism = extrude_world_loop(
                &island.curves,
                profile.z_axis,
                bottom - tree.thickness * 0.5,
                top + tree.thickness * 0.5,
                &name,
            )?;
            cutter = common::subtract_solid(cutter, island_prism)?;
        }
        cutters.push((name, cutter));
    }
    Ok(cutters)
}

/// Extrude a closed WORLD loop along `direction` from offset `bottom` to `top`
/// (both measured along `direction` from the loop's own plane), stamping `name`
/// on every face.
fn extrude_world_loop(
    curves: &[NurbsCurve],
    direction: Vec3,
    bottom: f64,
    top: f64,
    name: &str,
) -> Result<crate::BrepSolid, String> {
    let offset = direction.scale(bottom);
    let profile = curves
        .iter()
        .map(|curve| common::translate_curve(curve, offset))
        .collect::<Result<Vec<_>, _>>()?;
    let mut solid = crate::extrude_profile_brep(&profile, direction, top - bottom)?;
    for face in solid
        .shells
        .iter_mut()
        .flat_map(|shell| shell.faces.iter_mut())
    {
        if face.name.is_none() {
            face.name = Some(name.to_string());
        }
    }
    Ok(solid)
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected solid drives `sheet`, a face/sketch `profile`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.solids > 0 || probe.has_profile()
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SM.CUTOUT",
    "shortName": "SM.CUTOUT",
    "longName": "SM Cutout",
    "ribbonPath": "Home/Sheet Metal/Cutout",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the sheet metal cutout"
        },
        "sheet": {
            "type": "reference_selection",
            "selectionFilter": [
                "SOLID"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Target sheet metal solid to cut."
        },
        "profile": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "SKETCH"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Sketch profile to cut with (always consumed). Each sketch region cuts; inner loops are islands that keep their material."
        },
        "forwardDistance": {
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "Blind cut depth forward of the sketch plane (along the sketch normal). 0 = cut through everything on that side."
        },
        "backDistance": {
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "Blind cut depth behind the sketch plane (against the sketch normal). 0 = cut through everything on that side."
        },
        "keepTool": {
            "type": "boolean",
            "default_value": false,
            "hint": "Also keep the generated cutting tool solid(s) in the scene ({id}:CUTTER)."
        }
    }
})
}

