//! §5.9 Sheet → solid THICKEN (Golovanov): turn an open surface patch —
//! either its full parameter rectangle or a region TRIMMED by pcurve loops —
//! into a slab solid bounded by the sheet offset on one or both sides plus
//! ruled side walls along every boundary pcurve.
//!
//! The offset carriers come from `offset_surface` (§3.15 equidistant
//! surface), which preserves the source degree / knots / weights — planes
//! stay exact planes and cylinders / spheres / tori stay exact rational
//! carriers.  Because the two sheets share one basis, each side wall is the
//! HOMOGENEOUS ruling between corresponding boundary curves: with equal
//! per-column weights the ruling evaluates to the pointwise segment
//! (1−w)·B(s) + w·T(s), so wall pcurves are exact parameter lines, a planar
//! rectangle thickens to an EXACT box, and a circular hole in a planar sheet
//! grows an EXACT cylindrical tube.
//!
//! Trim loops follow the FaceRecord convention (`validate_uv_wire`): the
//! outer loop runs counter-clockwise in (u, v) and hole loops run clockwise,
//! exactly as they would appear on a `same_sense = true` face.  Each hole
//! adds one handle: the result of thickening a sheet with h holes is a
//! genus-h solid, and the Euler check V − E + F − H = 2(1 − genus) closes.
//!
//! Where the equidistant surface would degenerate over the SELECTED REGION —
//! a concave principal curvature radius smaller than the offset distance folds
//! the offset through its evolute — the builder refuses with an honest `Err`
//! instead of assembling silent garbage.  The question is asked about the
//! trimmed region and not about the whole carrier: a cone whose apex, or a
//! torus whose inner equator, lies outside the selected trim offsets perfectly
//! well over the sheet actually being thickened.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::offset_carve::{carve_folded_trim, dropped_sweep as carve_dropped_sweep};
use crate::offset_regularity::{scan_offset_regularity, ScanBudget, TrimRegion};
use crate::sweep_topology::parameter_line;
use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::image_curve::{affine_image_curve, image_curve_pair};
use crate::{make_arc, make_line, NurbsCurve, NurbsSurface, Vec3};

#[path = "thicken/band.rs"]
pub(crate) mod band;

/// The fold factor at or below which the equidistant sheet has collapsed.  The
/// offset-shell carrier lane refuses at the same number on the same condition
/// (`offset/offset_shell/carriers.rs`), which is why both read it through the
/// shared scan rather than each keeping a copy of the arithmetic.  The free-form
/// push reads this one (`edit/direct_edit/face_offset_freeform.rs`).
pub(crate) const FOLD_FACTOR: f64 = 1e-6;

/// Refuse offsets that fold through the sheet's evolute INSIDE THE SELECTED
/// TRIM: over the trimmed region, and for every requested signed offset distance
/// d, the per-direction area factor 1 − d·κ must stay positive, or the
/// equidistant surface self-intersects there (concave curvature radius ≤ offset
/// distance).
///
/// The region, not the rectangle.  A carrier that folds where the trim does not
/// reach — a cone whose apex lies outside the selected window, a torus whose
/// inner equator does — offsets perfectly well over the sheet actually being
/// thickened, and used to be refused because the gate swept the whole parameter
/// domain.  A fold that IS inside the trim must not be hidden by sparse
/// sampling either, which is why the sampling density comes from the trim's own
/// knot spans and the offset distance rather than from a fixed 33 × 33 grid; see
/// [`crate::offset_regularity`].
fn ensure_offsets_regular(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    distances: &[f64],
) -> Result<(), KernelRefusal> {
    let region = TrimRegion::from_pcurve_loops(surface, loops)
        .map_err(|error| error.with_message(|error| format!("thickenSheet: {error}")))?;
    let scan = scan_offset_regularity(surface, &region, distances, FOLD_FACTOR, ScanBudget::SHEET)
        .map_err(|error| error.with_message(|error| format!("thickenSheet: {error}")))?;
    let Some(worst) = scan.worst.filter(|worst| worst.factor <= FOLD_FACTOR) else {
        return Ok(());
    };
    let where_3d = match surface.evaluate(worst.u, worst.v) {
        Ok(point) => format!(
            " — 3D ({:.6}, {:.6}, {:.6})",
            point.x, point.y, point.z
        ),
        Err(_) => String::new(),
    };
    Err(KernelRefusal::unsupported(KernelStage::Refine, "thicken_fold_inside_trim", format!(
        "thickenSheet: offset by {:.6} self-intersects — inside the selected trim the sheet's \
         concave curvature radius {:.6} at (u={:.6}, v={:.6}){} is not larger than the offset \
         distance (fold factor {:.3e})",
        worst.displacement,
        worst.radius(),
        worst.u,
        worst.v,
        where_3d,
        worst.factor
    )))
}

/// Parameter-space validation of the trim loops: closure, degeneracy, pinches
/// and winding.
///
/// Split out of the construction pass so it runs BEFORE the regularity gate,
/// which now reads the loops as a REGION: a loop that does not close has no
/// region to judge, and its own refusal is the one worth reporting.
fn validate_trim_loops(
    loops: &[Vec<NurbsCurve>],
    uv_tolerance: f64,
    minimum_area: f64,
) -> Result<(), KernelRefusal> {
    for (loop_index, loop_curves) in loops.iter().enumerate() {
        let count = loop_curves.len();
        let mut starts = Vec::with_capacity(count);
        let mut ends = Vec::with_capacity(count);
        for curve in loop_curves {
            let [q0, q1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            starts.push(curve.evaluate(q0).or_refuse(KernelStage::Refine, "evaluate")?);
            ends.push(curve.evaluate(q1).or_refuse(KernelStage::Refine, "evaluate")?);
        }
        for index in 0..count {
            let next_index = (index + 1) % count;
            let gap = planar_gap(ends[index], starts[next_index]);
            if gap > uv_tolerance {
                return Err(KernelRefusal::input(KernelStage::Collect, "thicken_open_loop", format!(
                    "thickenSheet: loop {loop_index} is open — pcurve {index} ends at \
                     (u={:.6}, v={:.6}) but pcurve {next_index} starts at (u={:.6}, v={:.6}) \
                     (parameter-space gap {gap:.3e})",
                    ends[index].x, ends[index].y, starts[next_index].x, starts[next_index].y
                )));
            }
        }
        if count == 1 {
            let [q0, q1] = loop_curves[0].domain().or_refuse(KernelStage::Refine, "domain")?;
            let middle = loop_curves[0].evaluate((q0 + q1) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
            if planar_gap(middle, starts[0]) <= uv_tolerance {
                return Err(KernelRefusal::input(KernelStage::Collect, "thicken_degenerate_loop", format!(
                    "thickenSheet: loop {loop_index} is degenerate (zero parameter-space extent)"
                )));
            }
        } else {
            for index in 0..count {
                if planar_gap(ends[index], starts[index]) <= uv_tolerance {
                    return Err(KernelRefusal::input(KernelStage::Collect, "thicken_pinched_loop", format!(
                        "thickenSheet: pcurve {index} of loop {loop_index} closes on itself \
                         inside a multi-curve loop (pinched loop)"
                    )));
                }
            }
        }
        let mut area = 0.0;
        for curve in loop_curves {
            area += pcurve_signed_area(curve)?;
        }
        if loop_index == 0 {
            if area <= minimum_area {
                return Err(KernelRefusal::input(KernelStage::Collect, "thicken_outer_winding", format!(
                    "thickenSheet: outer loop must run counter-clockwise in (u, v) \
                     (signed area {area:.3e})"
                )));
            }
        } else if area >= -minimum_area {
            return Err(KernelRefusal::input(KernelStage::Collect, "thicken_hole_winding", format!(
                "thickenSheet: hole loop {loop_index} must run clockwise in (u, v) \
                 (signed area {area:.3e})"
            )));
        }
    }
    Ok(())
}

/// Equidistant sheet moved `distance` along the parametrization normal
/// n = Su × Sv (positive = +n side).  `offset_surface`'s positive distance
/// moves OPPOSITE the face normal, hence the negation.
///
/// A RE-PARAMETERIZED carrier is refused rather than used.  Everything this
/// builder does downstream rests on one identity: the offset shares the sheet's
/// basis, so the SAME `(u, v)` names the pointwise offset of the same source
/// point and the trim pcurves transfer verbatim.  `offset_surface`'s apex-cone
/// pinch retrim deliberately breaks that identity — it pulls an inverted sample
/// row back to the offset's own apex so the cavity ends in a proper degenerate
/// row — and with the extension inactive (as it always is here) that retrim is
/// the only thing that can report [`OffsetSurfaceLane::Reparameterised`].
///
/// The sheet's own TRIM is what decides whether that retrim fires at all, and
/// this is where it is handed over: the carrier is built as a face carrying the
/// selected loops, so `offset_surface` can ask whether the trim reaches past the
/// pinch before it re-parameterizes anything.  A sheet whose carrier folds
/// OUTSIDE its trim used to be re-parameterized anyway and shipped a sheared
/// slab that validated while being the wrong solid; it now keeps the pointwise
/// offset, and the refusal below is left standing for the case the retrim really
/// is about — a trim that DOES reach the pinch and would carry a sheared
/// correspondence into the walls.
fn offset_sheet(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    distance: f64,
) -> Result<NurbsSurface, KernelRefusal> {
    if distance == 0.0 {
        return Ok(surface.clone());
    }
    let carrier = FaceRecord {
        id: 1,
        surface: surface.clone(),
        same_sense: true,
        loops: loops
            .iter()
            .enumerate()
            .map(|(loop_index, loop_curves)| LoopRecord {
                id: loop_index as u64 + 1,
                coedges: loop_curves
                    .iter()
                    .enumerate()
                    .map(|(index, pcurve)| CoedgeRecord {
                        // A carrier built only to be measured: nothing reads
                        // these ids or this edge, only the pcurves, which are
                        // the trim `offset_surface` has to see.
                        id: index as u64 + 1,
                        edge_id: index as u64 + 1,
                        forward: true,
                        pcurve: pcurve.clone(),
                    })
                    .collect(),
            })
            .collect(),
        name: None,
    };
    let (offset, lane) = crate::offset::offset_surface_with_lane(
        &carrier,
        -distance,
        &crate::CarrierExtension::uniform(0.0),
    )?;
    if matches!(lane, crate::OffsetSurfaceLane::Reparameterised) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "thicken_reparameterised_carrier", format!(
            "thickenSheet: the offset carrier for distance {distance:.6} was re-parameterized \
             (the apex-cone pinch retrim pulled a sample row back to the offset's own apex), so \
             the trim's pcurves no longer name the pointwise offset of the selected region — the \
             selected trim reaches past the pinch and offsetting only its own sub-domain is not \
             yet supported"
        )));
    }
    Ok(offset)
}

/// Ruled wall between corresponding boundary curves of the bottom and top
/// sheets.  Requires the shared basis that `offset_surface` guarantees
/// (same degree, knots, and per-column weights); with equal weights the
/// homogeneous ruling evaluates to the exact pointwise segment
/// (1−w)·bottom(s) + w·top(s).
fn ruled_wall(bottom: &NurbsCurve, top: &NurbsCurve) -> Result<NurbsSurface, KernelRefusal> {
    if bottom.degree != top.degree
        || bottom.knots.len() != top.knots.len()
        || bottom
            .knots
            .iter()
            .zip(&top.knots)
            .any(|(a, b)| (a - b).abs() > 1e-12)
        || bottom
            .control_points
            .iter()
            .zip(&top.control_points)
            .any(|(a, b)| (a.w - b.w).abs() > 1e-9)
    {
        return Err(KernelRefusal::internal(KernelStage::Refine, "thicken_basis_mismatch", "thickenSheet: internal error — offset sheet basis mismatch"));
    }
    let rows = bottom
        .control_points
        .iter()
        .zip(&top.control_points)
        .map(|(b, t)| vec![*b, *t])
        .collect();
    NurbsSurface::new(
        bottom.degree,
        1,
        bottom.knots.clone(),
        vec![0.0, 0.0, 1.0, 1.0],
        rows,
    )
    .or_refuse(KernelStage::Refine, "new")
}

const GAUSS_X: [f64; 8] = [
    -0.9602898564975363,
    -0.7966664774136267,
    -0.525532409916329,
    -0.18343464249564978,
    0.18343464249564978,
    0.525532409916329,
    0.7966664774136267,
    0.9602898564975363,
];
const GAUSS_W: [f64; 8] = [
    0.10122853629037669,
    0.22238103445337445,
    0.31370664587788727,
    0.362683783378362,
    0.362683783378362,
    0.31370664587788727,
    0.22238103445337445,
    0.10122853629037669,
];

/// Green's-theorem signed-area contribution ∮ (x·y' − y·x')/2 of one pcurve,
/// integrated per knot span with 8-point Gauss.  Summed over a closed loop
/// this is the loop's signed (u, v) area — the orientation oracle for the
/// outer-CCW / hole-CW convention.
fn pcurve_signed_area(curve: &NurbsCurve) -> Result<f64, KernelRefusal> {
    let [q0, q1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let mut breaks = vec![q0];
    for &knot in &curve.knots {
        if knot > q0 + 1e-12 && knot < q1 - 1e-12 && (knot - breaks[breaks.len() - 1]).abs() > 1e-12
        {
            breaks.push(knot);
        }
    }
    breaks.push(q1);
    let mut area = 0.0;
    for pair in breaks.windows(2) {
        let half = (pair[1] - pair[0]) * 0.5;
        let middle = (pair[1] + pair[0]) * 0.5;
        for index in 0..GAUSS_X.len() {
            let derivatives = curve.derivatives(middle + half * GAUSS_X[index], 1).or_refuse(KernelStage::Refine, "derivatives")?;
            let point = derivatives[0];
            let tangent = derivatives[1];
            area += GAUSS_W[index] * half * 0.5 * (point.x * tangent.y - point.y * tangent.x);
        }
    }
    Ok(area)
}

/// Distance between two pcurve evaluations in the (u, v) plane (the z slot
/// of a parameter-space curve is dead weight).
fn planar_gap(first: Vec3, second: Vec3) -> f64 {
    let du = first.x - second.x;
    let dv = first.y - second.y;
    (du * du + dv * dv).sqrt()
}

/// 3D images of one boundary pcurve on the bottom and top sheets, plus the
/// edge parameter range they are represented over and whether the stored
/// loop direction runs with increasing edge parameter.
struct BoundaryImages {
    bottom: NurbsCurve,
    top: NurbsCurve,
    t0: f64,
    t1: f64,
    /// Stored pcurve direction == increasing edge parameter.
    dir: bool,
}

/// Build the bottom/top 3D images of one boundary pcurve.
///
/// * Affine sheets take ANY rational pcurve (exact homogeneous mapping).
/// * Curved sheets take iso-parameter LINE segments (u = const or
///   v = const): the image is the shared-basis isocurve of each sheet,
///   trimmed to the segment's parameter range.  A degree-1 equal-weight
///   pcurve maps its parameter linearly onto the iso parameter, so the
///   validator's fraction-matched pcurve consistency check is exact.
/// * Anything else on a curved sheet goes to the GENERAL image ladder
///   ([`crate::image_curve::image_curve_pair`]): the composed curve-on-surface
///   is sampled and fitted on both sheets over ONE parameter set, which is what
///   makes the pair basis-identical for [`ruled_wall`].  The transfer is exact
///   in the sense that matters here — unlike a push against a FIXED neighbour,
///   both sheets move together, so the image of the shared trim IS the
///   boundary, not an approximation of some other curve.  The ladder refuses
///   with its measured deviation rather than returning an unvalidated fit.
fn boundary_images(
    base_affine: bool,
    bottom: &NurbsSurface,
    top: &NurbsSurface,
    pcurve: &NurbsCurve,
    eps_u: f64,
    eps_v: f64,
    fit_tolerance: f64,
) -> Result<BoundaryImages, KernelRefusal> {
    if base_affine {
        let [q0, q1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        return Ok(BoundaryImages {
            bottom: affine_image_curve(bottom, pcurve).or_refuse(KernelStage::Refine, "affine_image_curve")?,
            top: affine_image_curve(top, pcurve).or_refuse(KernelStage::Refine, "affine_image_curve")?,
            t0: q0,
            t1: q1,
            dir: true,
        });
    }
    if pcurve.degree == 1 && pcurve.control_points.len() == 2 {
        let first = pcurve.control_points[0];
        let second = pcurve.control_points[1];
        if (first.w - second.w).abs() <= 1e-12 {
            let (ua, va) = (first.x / first.w, first.y / first.w);
            let (ub, vb) = (second.x / second.w, second.y / second.w);
            if (ua - ub).abs() <= eps_u && (va - vb).abs() > eps_v {
                let u_constant = (ua + ub) * 0.5;
                return Ok(BoundaryImages {
                    bottom: bottom.iso_curve_u(u_constant).or_refuse(KernelStage::Refine, "iso_curve_u")?,
                    top: top.iso_curve_u(u_constant).or_refuse(KernelStage::Refine, "iso_curve_u")?,
                    t0: va.min(vb),
                    t1: va.max(vb),
                    dir: vb > va,
                });
            }
            if (va - vb).abs() <= eps_v && (ua - ub).abs() > eps_u {
                let v_constant = (va + vb) * 0.5;
                return Ok(BoundaryImages {
                    bottom: bottom.iso_curve_v(v_constant).or_refuse(KernelStage::Refine, "iso_curve_v")?,
                    top: top.iso_curve_v(v_constant).or_refuse(KernelStage::Refine, "iso_curve_v")?,
                    t0: ua.min(ub),
                    t1: ua.max(ub),
                    dir: ub > ua,
                });
            }
        }
    }
    // The general trim: no closed form, so fit the composed curve on both
    // sheets and let the ladder measure itself.
    let (bottom_image, top_image) =
        image_curve_pair(bottom, top, pcurve, fit_tolerance, "thickenSheet").or_refuse(KernelStage::Refine, "image_curve_pair")?;
    let forward = bottom_image.t0 <= bottom_image.t1;
    Ok(BoundaryImages {
        bottom: bottom_image.curve,
        top: top_image.curve,
        t0: bottom_image.t0.min(bottom_image.t1),
        t1: bottom_image.t0.max(bottom_image.t1),
        dir: forward,
    })
}

/// §5.9 THICKEN a TRIMMED sheet region into a closed solid.
///
/// `loops` are parameter-space curves on `surface` exactly as FaceRecord
/// loops store them: the outer loop first, running counter-clockwise in
/// (u, v), followed by optional hole loops running clockwise (the
/// `same_sense = true` convention of `validate_uv_wire`).  Each loop's
/// pcurves must chain tip-to-tail and close; a loop may also be a single
/// closed pcurve (e.g. a rational circle).
///
/// * `symmetric = false`: the solid occupies the space between the sheet and
///   its offset at signed `thickness` along the sheet normal n = Su × Sv
///   (negative thickness grows the solid on the −n side).
/// * `symmetric = true`: the material splits evenly, |thickness|/2 on each
///   side of the sheet (the sheet becomes the mid-surface).
///
/// The caps are the bottom/top offset sheets trimmed by the SAME pcurve
/// loops (the offset shares the sheet's basis, so pcurves transfer
/// verbatim); every boundary pcurve contributes one ruled side wall between
/// its bottom and top 3D images, ruled pointwise at equal pcurve parameter —
/// for an offset pair that ruling runs along the surface normal, so walls
/// are exact wherever the full-domain walls were.  Hole loops produce inner
/// wall tubes and each adds one handle: `genus = loops.len() - 1`.
///
/// Every boundary edge is a single EdgeRecord shared by exactly two coedges
/// (cap + wall); wall-to-wall junction edges are likewise shared.
///
/// A general (non-iso) pcurve on a CURVED sheet is no longer refused: its two
/// 3D images come from [`crate::image_curve::image_curve_pair`], which fits the
/// composed curve-on-surface on both sheets over ONE shared parameter set (the
/// basis identity [`ruled_wall`] requires) and refuses only when the fit misses
/// its measured tolerance.  Unlike a push against a FIXED neighbour, both
/// sheets here move together, so the image of the shared trim IS the boundary.
///
/// Returns one BODY per regular piece of the selected trim.  That is one body
/// for a trim the offset is regular over, and one for a trim the fold locus
/// splits in two (the folded half is dropped) — but a fold BAND across the trim
/// is bounded by TWO locus branches and leaves TWO regular pieces, and each of
/// those is its own body: they share no edge, because the dropped band lies
/// between them.
///
/// Refuses (Err) on: zero/non-finite thickness, closed sheets, open or
/// misoriented loops, pinched loops, a general pcurve image that cannot be
/// fitted to tolerance, and offsets that fold through the evolute INSIDE THE
/// SELECTED TRIM (a fold the trim excludes is not this sheet's problem) that
/// the carve cannot divide the trim along.
pub fn thicken_trimmed_sheet(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    thickness: f64,
    symmetric: bool,
) -> Result<Vec<BrepSolid>, KernelRefusal> {
    if !thickness.is_finite() || thickness.abs() <= 1e-12 {
        return Err(KernelRefusal::input(KernelStage::Collect, "thicken_thickness", "thickenSheet: thickness must be a nonzero finite value"));
    }
    if loops.is_empty() || loops.iter().any(|loop_curves| loop_curves.is_empty()) {
        return Err(KernelRefusal::input(KernelStage::Collect, "thicken_no_loop", "thickenSheet: at least one non-empty pcurve loop is required"));
    }
    let (distance_bottom, distance_top) = if symmetric {
        (-thickness.abs() * 0.5, thickness.abs() * 0.5)
    } else if thickness > 0.0 {
        (0.0, thickness)
    } else {
        (thickness, 0.0)
    };
    // --- The fold BAND on a FULL revolution, before the closed-sheet refusal
    //     below: this lane's sheet IS a seamed one, and the reason it can be
    //     built where a general closed sheet cannot is that the whole question
    //     is planar. A thicken is the UNION of the sheet's normal segments; over
    //     a folded band those segments cross the axis and come out at the
    //     azimuth half a turn away, which a FULL revolution covers itself. The
    //     union is then the half-plane region (the swept sector truncated at the
    //     axis, plus the reflected lemon) revolved — `offset/thicken/band.rs`.
    //
    //     `None` is "not this lane" and changes nothing for any other sheet: the
    //     recognition wants a revolution with a circular generatrix whose offset
    //     really does reach the axis, in a BAND strictly inside the trim.
    if let Some(recognized) = band::recognize(surface, loops, distance_bottom, distance_top)? {
        return Ok(vec![band::build(&recognized)?]);
    }
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    if closed_u || closed_v {
        return Err(KernelRefusal::unsupported(KernelStage::Collect, "thicken_closed_sheet", "thickenSheet: closed sheets are not supported (split the patch at its seam first)"));
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let uv_tolerance = 1e-7 * (u1 - u0).max(v1 - v0);
    let minimum_area = 1e-10 * (u1 - u0) * (v1 - v0);
    // Loop validation runs BEFORE the regularity gate: the gate now reads the
    // trim as a REGION, and an open or misoriented loop is that loop's refusal
    // to report, not a fold's.  The construction pass below re-derives the same
    // endpoints it needs and no longer re-checks them.
    validate_trim_loops(loops, uv_tolerance, minimum_area)?;
    // A fold covering PART of the selected trim is CARVED out of it: the fold
    // locus is traced, the trim split along it, and the folded side dropped.
    // What is left is a region the offset is regular over, so the gate below
    // has nothing to refuse — and the carve has already verified that with the
    // same scan, judged a band off the level so the new boundary itself does
    // not vote.  A region that folds EVERYWHERE, or one whose fold only the
    // refinement pass found, is not a carve; the gate keeps its refusal.
    let carved = match carve_folded_trim(
        surface,
        loops,
        &[distance_bottom, distance_top],
        FOLD_FACTOR,
        ScanBudget::SHEET,
    ) {
        Ok(carved) => carved,
        Err(why) => {
            // A carve that cannot be made does NOT become this feature's
            // refusal.  The sheet still folds inside its trim, and the message
            // a caller has always got for that is the gate's — naming the
            // curvature radius and the `(u, v)`.  The carve's own reason rides
            // along so the cause is not lost, but it never replaces the
            // refusal, and it never turns a fold into a build.
            ensure_offsets_regular(surface, loops, &[distance_bottom, distance_top]).map_err(|refusal| {
                refusal.with_message(|refusal| format!("{refusal}; the fold could not be carved out either: {why}"))
            })?;
            return Err(why.with_message(|why| format!("thickenSheet: {why}")));
        }
    };
    let regions: Vec<Vec<Vec<NurbsCurve>>> = match &carved {
        Some(trim) => {
            let mut regions = Vec::with_capacity(trim.kept.len());
            for piece in &trim.kept {
                let kept = piece
                    .iter()
                    .map(|carved_loop| carved_loop.pcurves.clone())
                    .collect::<Vec<_>>();
                // The carved loops go through the SAME loop validation the
                // selected ones did: a division that leaves an open or pinched
                // loop is this module's refusal to report, not a defect to
                // build on.
                validate_trim_loops(&kept, uv_tolerance, minimum_area)?;
                regions.push(kept);
            }
            regions
        }
        None => {
            ensure_offsets_regular(surface, loops, &[distance_bottom, distance_top])?;
            vec![loops.to_vec()]
        }
    };
    // One BODY per regular piece. They are bodies and not shells of one solid
    // because they share no EDGE: the dropped band lies between them, and a
    // shell is edge-connected by definition — `brep/soundness.rs` reads exactly
    // that, and its vertex-fan detector already treats two cones meeting at
    // their apex as two pieces. Where a division leaves ONE piece this is a
    // one-element vector and nothing about the result changes.
    let mut bodies = Vec::with_capacity(regions.len());
    for region_loops in &regions {
        bodies.push(thicken_one_region(
            surface,
            region_loops,
            distance_bottom,
            distance_top,
        )?);
    }

    // THE RULE the carve rests on: a carve may drop a part of the trim only
    // when that part's own normals sweep material the kept construction
    // contains. Dropping the folded part is sound when the material it would
    // have contributed is already inside what was built, and unsound when it is
    // not — and that is a property of the geometry, not of how many pieces the
    // division left.
    //
    // The shape it catches: a torus sheet grown OUTWARD past its tube radius.
    // At the inner equator the outward normal points AT the axis, so the
    // segment reaches the axis part way along and comes out the far side,
    // sweeping material at the opposite azimuth that no kept piece holds. A
    // volume measured against the kept pieces alone is then fitted to the
    // construction rather than to the solid the sheet sweeps — which is the one
    // outcome this family of code exists to prevent, and it validates and
    // measures cleanly without this check.
    //
    // The shape it admits: an apex cone shelled through its base, where the
    // dropped sliver sweeps INWARD into material the result keeps.
    if let Some(trim) = &carved {
        for swept in carve_dropped_sweep(
            surface,
            &trim.dropped,
            &[distance_bottom, distance_top],
            ScanBudget::SHEET,
        )? {
            if bodies.iter().any(|body| {
                crate::classify_point(swept.point, body, uv_tolerance)
                    .map(|classification| classification.class != crate::PointClass::Out)
                    .unwrap_or(false)
            }) {
                continue;
            }
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "thicken_fold_drop_sweeps_material", format!(
                "thickenSheet: the fold covers part of the selected trim, and the part that \
                 would be dropped sweeps material the result does not contain — the normal at \
                 (u={:.6}, v={:.6}) reaches ({:.6}, {:.6}, {:.6}) at {:.0}% of the {:.6} offset, \
                 which is outside every body built from the {} kept piece(s). A carve may drop a \
                 part of the trim only when that part's own normals sweep material the kept \
                 construction contains; here they leave it, so the kept pieces bound less than \
                 the sheet sweeps and their volume is a form fitted to the construction. \
                 Refused rather than measured against it",
                swept.uv[0],
                swept.uv[1],
                swept.point.x,
                swept.point.y,
                swept.point.z,
                swept.fraction * 100.0,
                swept.displacement,
                bodies.len()
            )));
        }
    }
    Ok(bodies)
}

/// Build ONE slab body over one region's loops: both offset caps, the ruled
/// side walls between them, and the junction edges that join them.
///
/// Split out of [`thicken_trimmed_sheet`] because a fold BAND across a trim
/// leaves TWO regular pieces and each is its own body; everything above this
/// point decides WHICH regions to build, everything below builds one.
fn thicken_one_region(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    distance_bottom: f64,
    distance_top: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;

    let bottom = offset_sheet(surface, loops, distance_bottom)?;
    let top = offset_sheet(surface, loops, distance_top)?;
    // Each sheet's fit is refined by its own measured deviation, so the two can
    // leave on different nets; the ruled walls pair their iso curves control
    // point for control point.
    let (bottom, top) = crate::offset::unify_offset_sheet_bases(&bottom, &top)?;
    let eps_u = 1e-9 * (u1 - u0);
    let eps_v = 1e-9 * (v1 - v0);
    let base_affine = surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")?;
    // Fit-accuracy bar for the general image ladder.  `intersection_fit` is
    // the kernel's single NAMED "maximum geometric error accepted while
    // fitting" field (`geometry/tolerance.rs`), sized to this sheet — an
    // accuracy target, deliberately not the validator's loose
    // `pcurve_acceptance` identity band, which would admit a fit that misses
    // the true boundary by 2.5% of the model.
    let sheet_points = bottom
        .control_points
        .iter()
        .flatten()
        .map(|control| control.point().or_refuse(KernelStage::Refine, "point"))
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    let fit_tolerance =
        crate::KernelTolerances::for_scale(crate::model_scale(sheet_points), 1e-7).intersection_fit;

    let mut vertices: Vec<VertexRecord> = Vec::new();
    let mut edges: Vec<EdgeRecord> = Vec::new();
    let mut faces: Vec<FaceRecord> = Vec::new();
    let mut top_cap_loops: Vec<LoopRecord> = Vec::new();
    let mut bottom_cap_loops: Vec<LoopRecord> = Vec::new();
    let mut bottom_junction_points: Vec<Vec3> = Vec::new();
    let mut next_id = 1u64;

    for loop_curves in loops.iter() {
        let count = loop_curves.len();

        // Junction parameters; the checks these used to feed now run in
        // `validate_trim_loops`, ahead of the regularity gate.
        let mut starts = Vec::with_capacity(count);
        for curve in loop_curves {
            let [q0, _] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            starts.push(curve.evaluate(q0).or_refuse(KernelStage::Refine, "evaluate")?);
        }

        // ---- Junction vertices: junction j = start of pcurve j. ----
        let mut bottom_vertex_ids = Vec::with_capacity(count);
        let mut top_vertex_ids = Vec::with_capacity(count);
        let mut bottom_points = Vec::with_capacity(count);
        let mut top_points = Vec::with_capacity(count);
        for start in &starts {
            let bottom_point = bottom.evaluate(start.x, start.y).or_refuse(KernelStage::Refine, "evaluate")?;
            let top_point = top.evaluate(start.x, start.y).or_refuse(KernelStage::Refine, "evaluate")?;
            vertices.push(VertexRecord {
                id: next_id,
                point: bottom_point,
            });
            bottom_vertex_ids.push(next_id);
            next_id += 1;
            vertices.push(VertexRecord {
                id: next_id,
                point: top_point,
            });
            top_vertex_ids.push(next_id);
            next_id += 1;
            bottom_points.push(bottom_point);
            top_points.push(top_point);
            bottom_junction_points.push(bottom_point);
        }

        // ---- Boundary edges on both sheets + vertical junction edges. ----
        let mut images = Vec::with_capacity(count);
        for curve in loop_curves {
            images.push(boundary_images(
                base_affine,
                &bottom,
                &top,
                curve,
                eps_u,
                eps_v,
                fit_tolerance,
            )?);
        }
        let mut bottom_edge_ids = Vec::with_capacity(count);
        let mut top_edge_ids = Vec::with_capacity(count);
        for (index, image) in images.iter().enumerate() {
            let next_index = (index + 1) % count;
            let (start_j, end_j) = if image.dir {
                (index, next_index)
            } else {
                (next_index, index)
            };
            edges.push(EdgeRecord {
                id: next_id,
                curve: image.bottom.clone(),
                t0: image.t0,
                t1: image.t1,
                start_vertex_id: bottom_vertex_ids[start_j],
                end_vertex_id: bottom_vertex_ids[end_j],
                degenerate: false,
                name: None,
            });
            bottom_edge_ids.push(next_id);
            next_id += 1;
            edges.push(EdgeRecord {
                id: next_id,
                curve: image.top.clone(),
                t0: image.t0,
                t1: image.t1,
                start_vertex_id: top_vertex_ids[start_j],
                end_vertex_id: top_vertex_ids[end_j],
                degenerate: false,
                name: None,
            });
            top_edge_ids.push(next_id);
            next_id += 1;
        }
        let mut vertical_edge_ids = Vec::with_capacity(count);
        for junction in 0..count {
            edges.push(EdgeRecord {
                id: next_id,
                curve: make_line(bottom_points[junction], top_points[junction]).or_refuse(KernelStage::Refine, "make_line")?,
                t0: 0.0,
                t1: 1.0,
                start_vertex_id: bottom_vertex_ids[junction],
                end_vertex_id: top_vertex_ids[junction],
                degenerate: false,
                name: None,
            });
            vertical_edge_ids.push(next_id);
            next_id += 1;
        }

        // ---- One ruled wall per boundary pcurve. ----
        //
        // The wall's s parameter is the edge parameter; its loop traverses
        // the bottom edge ALONG the stored loop direction.  With the top
        // sheet at the larger offset, W_w = Δd·n with Δd > 0, so the wall's
        // natural normal W_s × W_w points to the RIGHT of the walk — which
        // is outward for a CCW outer loop (material on the left) AND for a
        // CW hole loop (material on the left, void on the right).  Hence
        // same_sense = dir uniformly, with the loop winding to match.
        for (index, image) in images.iter().enumerate() {
            let next_index = (index + 1) % count;
            let wall = ruled_wall(&image.bottom, &image.top)?;
            let (s_start, s_end) = if image.dir {
                (image.t0, image.t1)
            } else {
                (image.t1, image.t0)
            };
            let mut coedges = Vec::with_capacity(4);
            for (edge_id, forward, pcurve) in [
                (
                    bottom_edge_ids[index],
                    image.dir,
                    parameter_line(s_start, 0.0, s_end, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
                ),
                (
                    vertical_edge_ids[next_index],
                    true,
                    parameter_line(s_end, 0.0, s_end, 1.0).or_refuse(KernelStage::Refine, "parameter_line")?,
                ),
                (
                    top_edge_ids[index],
                    !image.dir,
                    parameter_line(s_end, 1.0, s_start, 1.0).or_refuse(KernelStage::Refine, "parameter_line")?,
                ),
                (
                    vertical_edge_ids[index],
                    false,
                    parameter_line(s_start, 1.0, s_start, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
                ),
            ] {
                coedges.push(CoedgeRecord {
                    id: next_id,
                    edge_id,
                    forward,
                    pcurve,
                });
                next_id += 1;
            }
            let loop_id = next_id;
            next_id += 1;
            faces.push(FaceRecord {
                id: next_id,
                surface: wall,
                same_sense: image.dir,
                loops: vec![LoopRecord {
                    id: loop_id,
                    coedges,
                }],
                name: None,
            });
            next_id += 1;
        }

        // ---- Cap loops: verbatim pcurves on top, reversed on bottom. ----
        let mut top_coedges = Vec::with_capacity(count);
        for (index, image) in images.iter().enumerate() {
            top_coedges.push(CoedgeRecord {
                id: next_id,
                edge_id: top_edge_ids[index],
                forward: image.dir,
                pcurve: loop_curves[index].clone(),
            });
            next_id += 1;
        }
        top_cap_loops.push(LoopRecord {
            id: next_id,
            coedges: top_coedges,
        });
        next_id += 1;
        let mut bottom_coedges = Vec::with_capacity(count);
        for index in (0..count).rev() {
            bottom_coedges.push(CoedgeRecord {
                id: next_id,
                edge_id: bottom_edge_ids[index],
                forward: !images[index].dir,
                pcurve: loop_curves[index].reversed().or_refuse(KernelStage::Refine, "reversed")?,
            });
            next_id += 1;
        }
        bottom_cap_loops.push(LoopRecord {
            id: next_id,
            coedges: bottom_coedges,
        });
        next_id += 1;
    }

    // Coincident junction vertices — v1's coincident-corner refusal
    // generalized: a repeated junction point pinches the boundary into a
    // non-manifold vertex (this also catches a hole touching the rim).
    // The junction ring's own extent, not its distance from the world origin:
    // the same sheet modelled 5 m out must be judged degenerate on the same
    // evidence.  For a two-junction boundary this is the pair's separation, so
    // the test still answers "coincident" only at a true collapse.
    let scale = crate::model_scale(bottom_junction_points.iter().copied());
    for first in 0..bottom_junction_points.len() {
        for second in first + 1..bottom_junction_points.len() {
            if bottom_junction_points[first]
                .sub(bottom_junction_points[second])
                .length()
                <= 1e-7 * scale
            {
                return Err(KernelRefusal::input(KernelStage::Collect, "thicken_degenerate_boundary", "thickenSheet: sheet boundary is degenerate (coincident junction vertices)"));
            }
        }
    }

    // Caps: outward is +n on the top sheet, −n on the bottom; the given
    // loop orientation matches same_sense = true, its reversal the bottom.
    faces.push(FaceRecord {
        id: next_id,
        surface: top,
        same_sense: true,
        loops: top_cap_loops,
        name: None,
    });
    next_id += 1;
    faces.push(FaceRecord {
        id: next_id,
        surface: bottom,
        same_sense: false,
        loops: bottom_cap_loops,
        name: None,
    });
    next_id += 1;

    let shell_id = next_id;
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: next_id + 1,
        vertices,
        edges,
        shells: vec![ShellRecord {
            id: shell_id,
            faces,
        }],
        genus: loops.len() as i64 - 1,
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "thicken_validate", format!(
            "thickenSheet: assembled solid failed validation: {issues:?}"
        )));
    }
    let volume = crate::solid_signed_volume(&solid).or_refuse(KernelStage::Refine, "solid_signed_volume")?;
    if volume <= 0.0 {
        return Err(KernelRefusal::internal(KernelStage::Validate, "thicken_orientation", format!(
            "thickenSheet: internal orientation error (signed volume {volume})"
        )));
    }
    // The sheet's two offset faces can reach past the source's curvature
    // radius and cross each other, or one of them can fold through itself,
    // and the body still validates and still measures a volume. This is the
    // lane's single exit and the one place that is asked — once per BODY,
    // which is what a carved trim needs: two pieces that share no edge have to
    // be sound each on its own, and a scan over their union would have to know
    // they are separate to avoid reading the gap between them.
    crate::accept_sound(solid, "thickenSheet")
}

/// §5.9 THICKEN a sheet (an open surface patch over its full parameter
/// domain) into a closed solid.
///
/// Thin wrapper over [`thicken_trimmed_sheet`] passing the full-domain
/// rectangle as the (counter-clockwise) outer loop.  The result is one
/// genus-0 body with 8 vertices, 12 edges, and 6 faces (offset caps + four
/// ruled walls), oriented outward and validated — or, where a fold BAND
/// crosses the domain, one such body per regular piece.  Refuses (Err) on:
/// zero/non-finite thickness, closed sheets (split at the seam first),
/// degenerate boundaries, and offsets that would self-intersect because a
/// concave curvature radius is smaller than the offset distance.
pub fn thicken_face_sheet(
    surface: &NurbsSurface,
    thickness: f64,
    symmetric: bool,
) -> Result<Vec<BrepSolid>, KernelRefusal> {
    if !thickness.is_finite() || thickness.abs() <= 1e-12 {
        return Err(KernelRefusal::input(KernelStage::Collect, "thicken_thickness", "thickenSheet: thickness must be a nonzero finite value"));
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let rectangle = vec![
        parameter_line(u0, v0, u1, v0).or_refuse(KernelStage::Refine, "parameter_line")?,
        parameter_line(u1, v0, u1, v1).or_refuse(KernelStage::Refine, "parameter_line")?,
        parameter_line(u1, v1, u0, v1).or_refuse(KernelStage::Refine, "parameter_line")?,
        parameter_line(u0, v1, u0, v0).or_refuse(KernelStage::Refine, "parameter_line")?,
    ];
    thicken_trimmed_sheet(surface, &[rectangle], thickness, symmetric)
}

