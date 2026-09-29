use super::*;

/// Per-coedge (u,v) parameter offset that unwraps a loop onto a periodic
/// surface's covering plane, marking where the loop crosses an IMPLICIT seam.
///
/// A periodic face whose exporter dropped the seam ruling leaves consecutive
/// coedges a whole period apart in parameter space (an explicit seam edge, by
/// contrast, connects the two sides in-domain and produces no such jump). Only
/// those whole-period jumps at coedge boundaries are folded out; adding
/// `offsets[i]` to coedge `i`'s pcurve samples turns the otherwise
/// self-intersecting boundary into a single monotone polygon the mass
/// integrator and tessellator can process over a tiled cell grid / with the
/// domain-wrapping evaluator.
///
/// Returns all-zero offsets when the loop is already continuous (the common
/// case — callers can skip the unwrapped path). The offsets are cumulative,
/// starting at zero on coedge 0, so the loop's own orientation is preserved.
pub fn loop_seam_offsets(
    coedges: &[CoedgeRecord],
    closed_u: bool,
    closed_v: bool,
    u_period: f64,
    v_period: f64,
) -> Result<Vec<[f64; 2]>, String> {
    let mut offsets = Vec::with_capacity(coedges.len());
    let fold = |from: f64, to: f64, period: f64| -> f64 {
        if period > 0.0 {
            ((from - to) / period).round() * period
        } else {
            0.0
        }
    };
    let mut offset = [0.0f64, 0.0f64];
    let mut previous_end: Option<[f64; 2]> = None;
    let mut first_start: Option<[f64; 2]> = None;
    for coedge in coedges {
        let [d0, d1] = coedge.pcurve.domain()?;
        let start = coedge.pcurve.evaluate(d0)?;
        first_start.get_or_insert([start.x, start.y]);
        // Advance the running offset across the gap between the previous
        // coedge's end and this coedge's start; a whole-period jump there is an
        // implicit seam hop.
        if let Some(end) = previous_end {
            if closed_u {
                offset[0] += fold(end[0], start.x, u_period);
            }
            if closed_v {
                offset[1] += fold(end[1], start.y, v_period);
            }
        }
        offsets.push(offset);
        let finish = coedge.pcurve.evaluate(d1)?;
        previous_end = Some([finish.x, finish.y]);
    }
    // The loop must return to its starting branch: fold the closing gap
    // (last coedge end -> first coedge start) and confirm the net offset is
    // zero. A NON-zero net is a full-period wrap (a rim that circles the whole
    // surface, not a band that rides a seam) — flattening it would tear the
    // covering-plane polygon open, so decline to unwrap (all-zero offsets keep
    // the raw in-domain handling).
    if let (Some(end), Some(start)) = (previous_end, first_start) {
        let mut closing = offset;
        if closed_u {
            closing[0] += fold(end[0], start[0], u_period);
        }
        if closed_v {
            closing[1] += fold(end[1], start[1], v_period);
        }
        let period = u_period.abs().max(v_period.abs()).max(1.0);
        if closing[0].abs() > 1e-6 * period || closing[1].abs() > 1e-6 * period {
            return Ok(vec![[0.0, 0.0]; coedges.len()]);
        }
    }
    Ok(offsets)
}

/// Classification of a DOUBLY-periodic (torus) face whose trim is a full-wrap
/// BAND bounded by exactly two loops: a constant-cross-level SEAM RIM sitting on
/// a cross-domain extreme, and a single-valued WAVY CUT (a full-wrap loop whose
/// cross level varies but crosses each periodic level exactly once). The rim's
/// implicit iso-line and the wavy cut never close a 2D region on their own, so
/// the raw winding integrator collapses the band to a sliver, the tessellator
/// sprays the seamed chains across the domain, and the point classifier reads
/// the wrong side — this is the doubly-periodic analogue of the singly-periodic
/// wall bounded by two implicit rim circles (`mass_properties::
/// singly_periodic_wall_polygons`). See [`analyze_doubly_periodic_seam_band`].
#[derive(Clone, Copy, Debug)]
pub struct SeamBand {
    /// True when the periodic (full-wrap) parameter is u; the cross parameter is
    /// then v (and vice versa).
    pub p_is_u: bool,
    /// Loop index of the constant-cross-level seam rim.
    pub rim_index: usize,
    /// Loop index of the single-valued wavy cut.
    pub cut_index: usize,
    /// Cross level of the rim (an extreme of the cross domain, q0 or q1, for
    /// the classic variant; the rim's own in-domain level for the in-domain-rim
    /// variant below).
    pub rim_level: f64,
    /// The material band is the cross strip ABOVE the cut, `q ∈ [cut(p), q_hi]`,
    /// when true; the strip BELOW it, `q ∈ [q_lo, cut(p)]`, when false. Both are
    /// in-domain because the rim sits at the opposite cross extreme.
    pub material_above_cut: bool,
    /// Cross level of the FLAT closure edge of the material polygon — the side
    /// opposite the wavy cut. For the classic rim-at-extreme variant this is the
    /// material-side cross extreme (q1 above the cut, q0 below it — the rim's
    /// OTHER seam representation); for the in-domain-rim variant it is the rim's
    /// own level (no lift — the material band sits strictly between rim and cut).
    pub flat_level: f64,
}

/// True only for a whole doubly-periodic face represented by one or more
/// collapsed parameter-space loops and no finite trim loop. Some STEP writers
/// encode an untrimmed torus as a sole `VERTEX_LOOP`; that point is not a hole,
/// and the material region is the entire surface domain. This deliberately
/// excludes the common fillet-band case with two real rims plus an extra
/// collapsed puncture.
pub(crate) fn doubly_periodic_has_only_collapsed_loops(face: &FaceRecord) -> Result<bool, String> {
    if face.loops.is_empty() || face.surface.closed_directions()? != (true, true) {
        return Ok(false);
    }
    let [u0, u1] = face.surface.domain_u()?;
    let [v0, v1] = face.surface.domain_v()?;
    let u_tolerance = 1e-9 * (u1 - u0).abs().max(1.0);
    let v_tolerance = 1e-9 * (v1 - v0).abs().max(1.0);
    for loop_record in &face.loops {
        if loop_record.coedges.is_empty() {
            return Ok(false);
        }
        let mut bounds = [
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ];
        for coedge in &loop_record.coedges {
            let [t0, t1] = coedge.pcurve.domain()?;
            for index in 0..=4 {
                let point = coedge
                    .pcurve
                    .evaluate(t0 + (t1 - t0) * index as f64 / 4.0)?;
                bounds[0] = bounds[0].min(point.x);
                bounds[1] = bounds[1].max(point.x);
                bounds[2] = bounds[2].min(point.y);
                bounds[3] = bounds[3].max(point.y);
            }
        }
        if bounds[1] - bounds[0] > u_tolerance || bounds[3] - bounds[2] > v_tolerance {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Detect the [`SeamBand`] signature of a doubly-periodic face from its loops
/// pre-sampled into (u,v) points in coedge/traversal order (one `Vec` per loop).
/// `domain` is `[u0,u1,v0,v1]`; `same_sense` is the face's sense flag. Returns
/// None for anything that is not this exact configuration — every other face
/// keeps its existing region determination unchanged.
///
/// The material side is chosen by the SAME "material lies to the LEFT of the
/// boundary traversal" rule the kernel uses everywhere else
/// (`mass_properties::biperiodic_band_range`, `watertight_tessellation::
/// close_periodic_trim_dir`): the between-rim-and-cut strip is kept unless the
/// senses conclusively name the complement across the cross seam.
pub fn analyze_doubly_periodic_seam_band(
    loops_uv: &[Vec<[f64; 2]>],
    domain: [f64; 4],
    same_sense: bool,
) -> Option<SeamBand> {
    if loops_uv.len() != 2 {
        return None;
    }
    let [u0, u1, v0, v1] = domain;
    for p_is_u in [true, false] {
        let (p0, p1) = if p_is_u { (u0, u1) } else { (v0, v1) };
        let (q0, q1) = if p_is_u { (v0, v1) } else { (u0, u1) };
        let period = p1 - p0;
        let q_extent = (q1 - q0).abs();
        if !(period > 0.0) || !(q_extent > 0.0) {
            continue;
        }
        let coord = |pt: &[f64; 2]| -> (f64, f64) {
            if p_is_u {
                (pt[0], pt[1])
            } else {
                (pt[1], pt[0])
            }
        };
        // Per-loop shape in the (periodic p, cross q) frame.
        struct Shape {
            full_wrap: bool,
            is_rim: bool,
            single_valued: bool,
            level: f64,
            dir: i32,
            qmin: f64,
            qmax: f64,
        }
        let mut shapes: Vec<Shape> = Vec::with_capacity(2);
        for points in loops_uv {
            if points.len() < 2 {
                return None;
            }
            let (mut qmin, mut qmax) = (f64::INFINITY, f64::NEG_INFINITY);
            for pt in points {
                let (_p, q) = coord(pt);
                qmin = qmin.min(q);
                qmax = qmax.max(q);
            }
            // Net winding and single-valuedness of the unwrapped periodic
            // parameter. Single-valued (each level crossed once) means the total
            // variation barely exceeds the net wrap: a there-and-back helical
            // ribbon has total variation >= 2 periods, while a monotone wrap with
            // sampling jitter stays just above one. Measured by total variation
            // rather than a strict sign test so a WAVY cut (small local u
            // reversals from the wave) still reads as single-valued.
            let mut net = 0.0;
            let mut total_variation = 0.0;
            for pair in points.windows(2) {
                let mut delta = coord(&pair[1]).0 - coord(&pair[0]).0;
                if delta > 0.5 * period {
                    delta -= period;
                } else if delta < -0.5 * period {
                    delta += period;
                }
                net += delta;
                total_variation += delta.abs();
            }
            // Backward (against-net) travel as a fraction of a period; 0 for a
            // perfectly monotone wrap.
            let backward_fraction = 0.5 * (total_variation - net.abs()) / period;
            let single_valued = backward_fraction <= 0.1;
            let dir = if net > 0.25 * period {
                1
            } else if net < -0.25 * period {
                -1
            } else {
                0
            };
            shapes.push(Shape {
                full_wrap: (net.abs() - period).abs() <= 0.25 * period,
                is_rim: (qmax - qmin) <= 0.05 * q_extent,
                single_valued,
                level: 0.5 * (qmin + qmax),
                dir,
                qmin,
                qmax,
            });
        }
        // Both loops must wrap the full period exactly once.
        if !shapes.iter().all(|s| s.full_wrap) {
            continue;
        }
        // Exactly one is a constant-level rim; the other a single-valued cut.
        let (rim_index, cut_index) = match (shapes[0].is_rim, shapes[1].is_rim) {
            (true, false) => (0, 1),
            (false, true) => (1, 0),
            // Two clean rims are the `biperiodic_band_range` case; two wavy cuts
            // are ambiguous. Neither is handled here.
            _ => continue,
        };
        if !shapes[cut_index].single_valued {
            continue;
        }
        // The rim must sit on a cross-domain EXTREME so BOTH candidate bands stay
        // in-domain (mirrors `singly_periodic_wall_polygons`).
        let tol = 0.02 * q_extent;
        let rim_at_lo = (shapes[rim_index].level - q0).abs() <= tol;
        let rim_at_hi = (shapes[rim_index].level - q1).abs() <= tol;
        if !(rim_at_lo || rim_at_hi) {
            // IN-DOMAIN-RIM VARIANT (case 08:10-genus, faces 1604/1616): an
            // imported band between a perfectly FLAT rim at an interior cross
            // level (v=0.5) and a SCALLOPED full-wrap cut (v=0.75 dipping to
            // ~0.678) — the dip breaks the two-clean-rims path's is_rim gate
            // and the interior rim level breaks the extreme gate here, so the
            // face fell to the raw even-odd whose verdict across the band is
            // meaningless (u-winding loop polygons enclose nothing). Accept the
            // configuration ONLY when every guess-free condition holds:
            //   - the rim is EXACTLY flat (1e-3 of the cross extent — no
            //     mid-hull ambiguity for the polygon's flat closure);
            //   - rim and cut are separable and BOTH in-domain;
            //   - the winding senses conclusively put the material BETWEEN rim
            //     and cut (the in-domain strip). A complement verdict would
            //     wrap the material out-of-domain across the cross seam —
            //     decline (fail-soft to the previous behavior) rather than
            //     build a chart we cannot represent flat.
            // The flat closure then sits at the rim's OWN level (flat_level =
            // rim_level), not at a domain extreme. Escape hatch:
            // BREP_BAND_INDOMAIN_RIM=0 restores the unconditional decline.
            if std::env::var("BREP_BAND_INDOMAIN_RIM").as_deref() == Ok("0") {
                continue;
            }
            let rim_level = shapes[rim_index].level;
            if (shapes[rim_index].qmax - shapes[rim_index].qmin) > 1e-3 * q_extent {
                continue;
            }
            let margin = 1e-6 * q_extent;
            let rim_below_cut = rim_level + margin < shapes[cut_index].qmin;
            let rim_above_cut = rim_level - margin > shapes[cut_index].qmax;
            if !(rim_below_cut || rim_above_cut) {
                continue;
            }
            let in_domain = |shape: &Shape| {
                shape.qmin >= q0 - margin && shape.qmax <= q1 + margin
            };
            if !(in_domain(&shapes[rim_index]) && in_domain(&shapes[cut_index])) {
                continue;
            }
            let (lower_index, upper_index) = if shapes[0].level <= shapes[1].level {
                (0, 1)
            } else {
                (1, 0)
            };
            let (lower_dir, upper_dir) = (shapes[lower_index].dir, shapes[upper_index].dir);
            if lower_dir == 0 || upper_dir == 0 || lower_dir == upper_dir {
                continue;
            }
            let between_is_ccw_uv = if p_is_u { lower_dir > 0 } else { lower_dir < 0 };
            if between_is_ccw_uv != same_sense {
                continue; // material would be the out-of-domain wrapped complement
            }
            return Some(SeamBand {
                p_is_u,
                rim_index,
                cut_index,
                rim_level,
                material_above_cut: rim_above_cut,
                flat_level: rim_level,
            });
        }
        // Rim and cut must be separable (no interleave in q).
        let margin = 1e-6 * q_extent;
        let separable = if rim_at_lo {
            shapes[rim_index].level + margin < shapes[cut_index].qmin
        } else {
            shapes[rim_index].level - margin > shapes[cut_index].qmax
        };
        if !separable {
            continue;
        }
        // Material side: conclusive opposite senses required (else decline — a
        // wavy cut is too easy to mis-side to guess). Reuse the lower-ring rule.
        let (lower_index, upper_index) = if shapes[0].level <= shapes[1].level {
            (0, 1)
        } else {
            (1, 0)
        };
        let (lower_dir, upper_dir) = (shapes[lower_index].dir, shapes[upper_index].dir);
        if lower_dir == 0 || upper_dir == 0 || lower_dir == upper_dir {
            continue;
        }
        let between_rims_is_ccw_uv = if p_is_u { lower_dir > 0 } else { lower_dir < 0 };
        let complement = between_rims_is_ccw_uv != same_sense;
        // rim at lo: between = strip below cut; complement = strip above cut.
        // rim at hi: between = strip above cut; complement = strip below cut.
        let material_above_cut = if rim_at_lo { complement } else { !complement };
        return Some(SeamBand {
            p_is_u,
            rim_index,
            cut_index,
            rim_level: if rim_at_lo { q0 } else { q1 },
            material_above_cut,
            flat_level: if material_above_cut { q1 } else { q0 },
        });
    }
    None
}

/// Build the material-region (u,v) polygon for a detected [`SeamBand`], from the
/// same per-loop uv samples passed to [`analyze_doubly_periodic_seam_band`]. The
/// polygon is the wavy cut on one side and the flat rim iso-line (at the cross
/// extreme bounding the material band — the same physical seam circle) on the
/// other, closed across the two private wrap-seam edges: one simple loop the
/// winding integrator and the point classifier can both consume, so they agree
/// on the material side. The tessellator builds the same topology separately
/// because it must carry authoritative edge-sample 3D positions for welding.
pub fn seam_band_uv_polygon(
    loops_uv: &[Vec<[f64; 2]>],
    domain: [f64; 4],
    band: &SeamBand,
) -> Vec<[f64; 2]> {
    seam_band_uv_polygon_with(loops_uv, |point| *point, |point| point, domain, band)
}

/// [`seam_band_uv_polygon`] over samples that carry a payload — the mass
/// integrator tags each sample with the pcurve parameter it came from so the
/// polygon's chords keep their arcs. `uv` reads a sample's position; `synthetic`
/// makes the vertices this function invents (the flat extensions to the seam
/// and the two rim corners). Every consecutive pair of the cut loop's samples
/// stays consecutive in the result (the loop is rotated and possibly reversed,
/// never re-sorted).
pub fn seam_band_uv_polygon_with<P: Copy>(
    loops: &[Vec<P>],
    uv: impl Fn(&P) -> [f64; 2],
    synthetic: impl Fn([f64; 2]) -> P,
    domain: [f64; 4],
    band: &SeamBand,
) -> Vec<P> {
    let [u0, u1, v0, v1] = domain;
    let (p0, p1) = if band.p_is_u { (u0, u1) } else { (v0, v1) };
    let (q0, _q1) = if band.p_is_u { (v0, v1) } else { (u0, u1) };
    let coord = |pt: &[f64; 2]| -> (f64, f64) {
        if band.p_is_u {
            (pt[0], pt[1])
        } else {
            (pt[1], pt[0])
        }
    };
    let mk = |p: f64, q: f64| -> [f64; 2] {
        if band.p_is_u {
            [p, q]
        } else {
            [q, p]
        }
    };
    let period = p1 - p0;
    // Wavy cut ordered ascending in the periodic parameter. Rotate the loop to
    // start just after its full-wrap seam jump (rather than SORTING, which
    // scrambles the wave's local reversals) so the boundary is monotone and its
    // in-domain samples stay in traversal order, then extend it flat to the seam
    // on each side (a full-wrap loop sampled raw stops a sample shy of it).
    let mut cut: Vec<P> = ascending_wrap_boundary(&loops[band.cut_index], |p| coord(&uv(p)));
    let eps = 1e-6 * period;
    if cut.first().map_or(true, |c| coord(&uv(c)).0 > p0 + eps) {
        let q = cut.first().map(|c| coord(&uv(c)).1).unwrap_or(q0);
        cut.insert(0, synthetic(mk(p0, q)));
    }
    if cut.last().map_or(true, |c| coord(&uv(c)).0 < p1 - eps) {
        let q = cut.last().map(|c| coord(&uv(c)).1).unwrap_or(q0);
        cut.push(synthetic(mk(p1, q)));
    }
    let q_flat = band.flat_level;
    let mut polygon: Vec<P> = Vec::with_capacity(cut.len() + 2);
    polygon.extend(cut);
    polygon.push(synthetic(mk(p1, q_flat)));
    polygon.push(synthetic(mk(p0, q_flat)));
    polygon
}

/// Rotate a full-wrap loop (sampled points, coedge/traversal order) to start
/// immediately AFTER its one big periodic-seam jump, returning the samples in
/// ascending periodic order. Rotating (not sorting) keeps every consecutive
/// pair a real edge chord — the wave's small local reversals survive but no two
/// originally-nonadjacent samples become adjacent — which is what lets the
/// tessellator's analogous boundary weld crack-free.
fn ascending_wrap_boundary<P: Copy>(points: &[P], coord: impl Fn(&P) -> (f64, f64)) -> Vec<P> {
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }
    // Largest consecutive (cyclic) jump in the periodic parameter is the seam.
    let (mut seam, mut worst) = (0usize, -1.0);
    for i in 0..n {
        let a = coord(&points[i]).0;
        let b = coord(&points[(i + 1) % n]).0;
        let d = (b - a).abs();
        if d > worst {
            worst = d;
            seam = (i + 1) % n;
        }
    }
    let mut rotated: Vec<P> = (0..n).map(|k| points[(seam + k) % n]).collect();
    if rotated.first().map(|c| coord(c).0) > rotated.last().map(|c| coord(c).0) {
        rotated.reverse();
    }
    rotated
}
