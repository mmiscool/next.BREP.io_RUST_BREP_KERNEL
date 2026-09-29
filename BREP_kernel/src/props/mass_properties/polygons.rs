use super::*;
use crate::topology::CoedgeRecord;

/// The pcurve arc behind one chord of a [`TrimPolygon`]: coedge
/// `coedge_index` of loop `loop_index`, from parameter `t0` at the chord's
/// start to `t1` at its end (`t1 < t0` when the loop was reversed), shifted by
/// `offset` on the covering plane.
#[derive(Clone, Copy, Debug)]
pub(super) struct ChordArc {
    pub loop_index: usize,
    pub coedge_index: usize,
    pub t0: f64,
    pub t1: f64,
    pub offset: [f64; 2],
}

/// One trim loop as the integrator consumes it: the chord polygon plus, per
/// chord, the pcurve arc it stands in for. `arcs[k]` belongs to the chord from
/// `points[k]` to `points[(k + 1) % n]`; `None` marks a segment with nothing
/// to correct against — a straight pcurve span, a synthetic side (band
/// rectangle, cap join, seam closure) or a jump between coedges that do not
/// meet. The integrator adds, per arc, the signed region between the arc and
/// its chord, so the chord count sets only the quadrature's work, never the
/// answer (`integrate_trimmed_polys`).
#[derive(Clone, Debug)]
pub(super) struct TrimPolygon {
    pub points: Vec<[f64; 2]>,
    pub arcs: Vec<Option<ChordArc>>,
    /// Where the last pushed coedge's pcurve ends (shifted): the point the next
    /// vertex has to land on for the chord leaving the last sample to be that
    /// pcurve's closing arc rather than a jump.
    pending_end: Option<[f64; 2]>,
    /// A vertex further than this from `pending_end` is a jump — a seam
    /// teleport is a whole period, a stitched pcurve gap is far smaller.
    jump_tolerance: f64,
}

fn is_straight_polyline(curve: &NurbsCurve) -> bool {
    let rational = curve
        .control_points
        .iter()
        .any(|point| (point.w - curve.control_points[0].w).abs() > 1e-12);
    curve.degree == 1 && !rational
}

impl TrimPolygon {
    pub(super) fn new(domain: [f64; 4]) -> Self {
        let [u0, u1, v0, v1] = domain;
        let jump_tolerance = 1e-3 * (u1 - u0).abs().min((v1 - v0).abs()).max(1e-300);
        Self {
            points: Vec::new(),
            arcs: Vec::new(),
            pending_end: None,
            jump_tolerance,
        }
    }

    pub(super) fn for_face(face: &FaceRecord) -> Result<Self, String> {
        let [u0, u1] = face.surface.domain_u()?;
        let [v0, v1] = face.surface.domain_v()?;
        Ok(Self::new([u0, u1, v0, v1]))
    }

    fn settle_pending(&mut self, next: [f64; 2]) {
        if let Some(end) = self.pending_end.take() {
            let gap = ((next[0] - end[0]).powi(2) + (next[1] - end[1]).powi(2)).sqrt();
            if gap > self.jump_tolerance {
                if let Some(last) = self.arcs.last_mut() {
                    *last = None;
                }
            }
        }
    }

    /// A vertex with no arc behind the chord that leaves it.
    pub(super) fn push_point(&mut self, point: [f64; 2]) {
        self.settle_pending(point);
        self.points.push(point);
        self.arcs.push(None);
    }

    /// Append one coedge's pcurve the way the integrator samples trims: a
    /// straight polyline at its knots (exact, no arcs), anything curved at
    /// 24..128 chords spread over its knot spans with the arc recorded behind
    /// every chord. A chord never straddles a knot, so each arc is one
    /// polynomial (or rational) piece and the sliver quadrature is exact on it.
    pub(super) fn push_coedge(
        &mut self,
        loop_index: usize,
        coedge_index: usize,
        coedge: &CoedgeRecord,
        offset: [f64; 2],
    ) -> Result<(), String> {
        let curve = &coedge.pcurve;
        let interior = interior_knots(&curve.knots, curve.degree);
        if is_straight_polyline(curve) {
            let [start, _] = curve.domain()?;
            for parameter in std::iter::once(start).chain(interior.iter().copied()) {
                let point = curve.evaluate(parameter)?;
                self.push_point([point.x + offset[0], point.y + offset[1]]);
            }
            return Ok(());
        }
        let sample_count = (24usize).max((interior.len() + 1) * 16).min(128);
        self.push_coedge_spans(loop_index, coedge_index, coedge, offset, sample_count, false)
    }

    /// Append about `target` chords of one coedge's pcurve, spread evenly over
    /// its knot spans (at least one per span, so no chord straddles a knot)
    /// and tagged with the arc behind each; a straight polyline contributes
    /// its knots exactly. The end point is pushed too when `include_end`.
    pub(super) fn push_coedge_spans(
        &mut self,
        loop_index: usize,
        coedge_index: usize,
        coedge: &CoedgeRecord,
        offset: [f64; 2],
        target: usize,
        include_end: bool,
    ) -> Result<(), String> {
        let curve = &coedge.pcurve;
        let spans = curve_breaks(curve)?;
        let [_, end] = curve.domain()?;
        if is_straight_polyline(curve) {
            let last = if include_end { spans.len() } else { spans.len() - 1 };
            for &parameter in &spans[..last] {
                let point = curve.evaluate(parameter)?;
                self.push_point([point.x + offset[0], point.y + offset[1]]);
            }
            return Ok(());
        }
        let span_count = spans.len().saturating_sub(1).max(1);
        let per_span = target.div_ceil(span_count).max(1);
        let mut previous: Option<(f64, [f64; 2])> = None;
        let mut emit = |this: &mut Self, t: f64| -> Result<(), String> {
            let point = curve.evaluate(t)?;
            let point = [point.x + offset[0], point.y + offset[1]];
            if let Some((t_prev, _)) = previous {
                if let Some(last) = this.arcs.last_mut() {
                    *last = Some(ChordArc { loop_index, coedge_index, t0: t_prev, t1: t, offset });
                }
            }
            this.settle_pending(point);
            this.points.push(point);
            this.arcs.push(None);
            previous = Some((t, point));
            Ok(())
        };
        for pair in spans.windows(2) {
            for index in 0..per_span {
                emit(self, pair[0] + (pair[1] - pair[0]) * index as f64 / per_span as f64)?;
            }
        }
        if include_end {
            emit(self, end)?;
        } else if let Some((t_prev, _)) = previous {
            // The chord leaving the last sample reaches the pcurve's end, which
            // the next vertex must supply.
            if let Some(last) = self.arcs.last_mut() {
                *last = Some(ChordArc { loop_index, coedge_index, t0: t_prev, t1: end, offset });
            }
            let point = curve.evaluate(end)?;
            self.pending_end = Some([point.x + offset[0], point.y + offset[1]]);
        }
        Ok(())
    }

    /// Close the loop: the chord back to the first vertex is the last pcurve's
    /// closing arc only if the first vertex is where that pcurve ends.
    pub(super) fn finish(&mut self) {
        if let Some(first) = self.points.first().copied() {
            self.settle_pending(first);
        }
        self.pending_end = None;
    }

    pub(super) fn reverse(&mut self) {
        self.finish();
        let n = self.points.len();
        if n == 0 {
            return;
        }
        let old = std::mem::take(&mut self.arcs);
        let swap = |arc: Option<ChordArc>| arc.map(|a| ChordArc { t0: a.t1, t1: a.t0, ..a });
        let mut arcs = Vec::with_capacity(n);
        for k in 0..n - 1 {
            arcs.push(swap(old[n - 2 - k]));
        }
        arcs.push(swap(old[n - 1]));
        self.arcs = arcs;
        self.points.reverse();
    }

    pub(super) fn shift_u(&mut self, du: f64) {
        for point in &mut self.points {
            point[0] += du;
        }
        for arc in self.arcs.iter_mut().flatten() {
            arc.offset[0] += du;
        }
        if let Some(end) = &mut self.pending_end {
            end[0] += du;
        }
    }

    /// Concatenate `other` after this loop's vertices; both joins are synthetic.
    pub(super) fn append(&mut self, mut other: TrimPolygon) {
        self.finish();
        other.finish();
        if let Some(last) = self.arcs.last_mut() {
            *last = None;
        }
        if let Some(last) = other.arcs.last_mut() {
            *last = None;
        }
        self.points.append(&mut other.points);
        self.arcs.append(&mut other.arcs);
    }
}

/// The chord polygons of `face`'s trim loops, as sampled for integration.
pub fn trim_polygons(face: &FaceRecord) -> Result<Vec<Vec<[f64; 2]>>, String> {
    Ok(trim_polygons_tagged(face)?
        .into_iter()
        .map(|polygon| polygon.points)
        .collect())
}

pub(super) fn trim_polygons_tagged(face: &FaceRecord) -> Result<Vec<TrimPolygon>, String> {
    let mut polygons = Vec::new();
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        let mut polygon = TrimPolygon::for_face(face)?;
        for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
            polygon.push_coedge(loop_index, coedge_index, coedge, [0.0, 0.0])?;
        }
        polygon.finish();
        polygons.push(polygon);
    }
    Ok(polygons)
}

/// Rebuild a sphere cap bounded by a varying full-period contact rim and the
/// collapsed opposite-winding rim at one parameter pole.  In the flat UV
/// plane both individual loops have zero enclosed area; joined across one cut
/// of the periodic cover they form the actual cap polygon.
pub(super) fn singly_periodic_sphere_cap_polygons(
    face: &FaceRecord,
    closed_u: bool,
    closed_v: bool,
    domain: [f64; 4],
) -> Result<Option<Vec<TrimPolygon>>, String> {
    if !matches!(
        face.surface.analytic(),
        Some(crate::AnalyticSurface::Sphere { .. })
    ) || (closed_u, closed_v) != (true, false)
        || face.loops.len() != 2
    {
        return Ok(None);
    }
    let [u0, u1, v0, v1] = domain;
    let period = u1 - u0;
    let v_span = v1 - v0;
    struct Rim {
        polygon: TrimPolygon,
        winding: f64,
        vmin: f64,
        vmax: f64,
    }
    let mut rims = Vec::with_capacity(2);
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        let mut polygon = TrimPolygon::new(domain);
        for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
            polygon.push_coedge_spans(loop_index, coedge_index, coedge, [0.0, 0.0], 64, true)?;
        }
        polygon.finish();
        let points = &polygon.points;
        if points.len() < 2 {
            return Ok(None);
        }
        let winding = points.last().unwrap()[0] - points[0][0];
        let (mut vmin, mut vmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for point in points {
            vmin = vmin.min(point[1]);
            vmax = vmax.max(point[1]);
        }
        rims.push(Rim {
            polygon,
            winding,
            vmin,
            vmax,
        });
    }
    if rims
        .iter()
        .any(|rim| (rim.winding.abs() - period).abs() > 0.05 * period)
        || rims[0].winding * rims[1].winding >= 0.0
    {
        return Ok(None);
    }
    let flat = |rim: &Rim| rim.vmax - rim.vmin <= 1e-6 * v_span;
    let pole = match (flat(&rims[0]), flat(&rims[1])) {
        (true, false) => &rims[0],
        (false, true) => &rims[1],
        _ => return Ok(None),
    };
    let pole_v = 0.5 * (pole.vmin + pole.vmax);
    if (pole_v - v0).abs() > 1e-6 * v_span && (pole_v - v1).abs() > 1e-6 * v_span {
        return Ok(None);
    }

    let (lower, upper) = if rims[0].vmax <= rims[1].vmin {
        (&rims[0], &rims[1])
    } else if rims[1].vmax <= rims[0].vmin {
        (&rims[1], &rims[0])
    } else {
        return Ok(None);
    };
    let mut lower_points = lower.polygon.clone();
    if lower.winding < 0.0 {
        lower_points.reverse();
    }
    let mut upper_points = upper.polygon.clone();
    if upper.winding > 0.0 {
        upper_points.reverse();
    }
    let shift = ((lower_points.points[0][0] - upper_points.points.last().unwrap()[0]) / period)
        .round()
        * period;
    upper_points.shift_u(shift);
    lower_points.append(upper_points);
    Ok(Some(vec![lower_points]))
}

/// Trim polygons with implicit torus seams unwrapped onto the covering plane.
/// Returns `(polygons, unwrapped)`; `unwrapped` is true iff any loop crossed an
/// implicit seam (and therefore carries out-of-domain parameters). Only attempts
/// the unwrap on a doubly-periodic surface — single-periodic seams the raw
/// integrator already handles are left in-domain.
pub(super) fn trim_polygons_unwrapped(
    face: &FaceRecord,
    closed_u: bool,
    closed_v: bool,
    u_period: f64,
    v_period: f64,
) -> Result<(Vec<TrimPolygon>, bool), String> {
    let mut polygons = Vec::new();
    let mut unwrapped = false;
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        let offsets = crate::topology::loop_seam_offsets(
            &loop_record.coedges,
            closed_u,
            closed_v,
            u_period,
            v_period,
        )?;
        let mut polygon = TrimPolygon::for_face(face)?;
        for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
            let offset = offsets[coedge_index];
            if offset[0] != 0.0 || offset[1] != 0.0 {
                unwrapped = true;
            }
            polygon.push_coedge(loop_index, coedge_index, coedge, offset)?;
        }
        polygon.finish();
        polygons.push(polygon);
    }
    Ok((polygons, unwrapped))
}

/// A singly-periodic (cylinder/cone) wall whose seam is IMPLICIT: the closed
/// wall is bounded by two full-wrap rim loops (iso-parametric lines at constant
/// cross-level) with NO seam edge connecting them — the two-rim-circles form
/// OCC/STEP emit for a full lateral face. This is the single-periodic analogue
/// of [`biperiodic_band_range`]: each rim is a degenerate iso line in parameter
/// space, so the raw trim integrates the band between them to ~zero (only the
/// in-band holes survive — a full cylinder wall collapses to a ~1000x-too-small
/// sliver, ABC 00000056 solid 2). We restrict to the unambiguous case where the
/// two rims sit at the cross-domain EXTREMES: the material region is then the
/// WHOLE surface (full periodic span × full cross span) minus any hole/notch
/// loops, with no between-rims-vs-complement choice to get wrong. Replacing the
/// two rim lines with a single full-domain band rectangle lets the existing
/// winding integrator fill the wall while the remaining loops subtract as holes.
///
/// Returns the rebuilt polygon set (band rectangle first, then the unwrapped
/// non-rim loops), or None when `face` is not a clean two-extreme-rim band —
/// every other periodic face keeps its existing integration path unchanged.
pub(super) fn singly_periodic_wall_polygons(
    face: &FaceRecord,
    closed_u: bool,
    closed_v: bool,
    domain: [f64; 4],
) -> Result<Option<Vec<TrimPolygon>>, String> {
    // Exactly one periodic direction (the doubly-periodic case is handled by
    // `biperiodic_band_integral` upstream).
    if closed_u == closed_v {
        return Ok(None);
    }
    let [u0, u1, v0, v1] = domain;
    let p_is_u = closed_u;
    let period = if p_is_u { u1 - u0 } else { v1 - v0 };
    if !(period > 0.0) {
        return Ok(None);
    }
    let (q_dom_lo, q_dom_hi) = if p_is_u { (v0, v1) } else { (u0, u1) };
    let q_extent = (q_dom_hi - q_dom_lo).abs();
    if !(q_extent > 0.0) {
        return Ok(None);
    }
    // (periodic, cross) coordinate split.
    let coord = |p: [f64; 2]| if p_is_u { (p[0], p[1]) } else { (p[1], p[0]) };

    let mut rim_indices: Vec<usize> = Vec::new();
    let mut rim_levels: Vec<f64> = Vec::new();
    for (index, loop_record) in face.loops.iter().enumerate() {
        // Sample the loop's pcurves in traversal order.
        let mut points: Vec<[f64; 2]> = Vec::new();
        for coedge in &loop_record.coedges {
            let [d0, d1] = coedge.pcurve.domain()?;
            let samples = 16;
            for k in 0..=samples {
                let t = d0 + (d1 - d0) * k as f64 / samples as f64;
                let p = coedge.pcurve.evaluate(t)?;
                points.push([p.x, p.y]);
            }
        }
        if points.len() < 2 {
            return Ok(None);
        }
        let (mut qmin, mut qmax) = (f64::INFINITY, f64::NEG_INFINITY);
        let mut net = 0.0;
        for pair in points.windows(2) {
            let (p0, q0) = coord(pair[0]);
            let (p1, _q1) = coord(pair[1]);
            qmin = qmin.min(q0);
            qmax = qmax.max(q0);
            let mut delta = p1 - p0;
            if delta > 0.5 * period {
                delta -= period;
            } else if delta < -0.5 * period {
                delta += period;
            }
            net += delta;
        }
        if let Some(last) = points.last() {
            let (_p, q) = coord(*last);
            qmin = qmin.min(q);
            qmax = qmax.max(q);
        }
        // A rim: nets a full period in the periodic direction and stays on a
        // near-constant cross-level (a true iso line).
        let is_rim = net.abs() > 0.75 * period && (qmax - qmin) <= 0.02 * q_extent;
        if is_rim {
            rim_indices.push(index);
            rim_levels.push(0.5 * (qmin + qmax));
        }
    }
    if rim_levels.len() != 2 {
        return Ok(None);
    }
    let (q_lo, q_hi) = if rim_levels[0] <= rim_levels[1] {
        (rim_levels[0], rim_levels[1])
    } else {
        (rim_levels[1], rim_levels[0])
    };
    // Require the rims at the cross-domain extremes: material is then the whole
    // wall (no between-rims/complement ambiguity). A sub-band (rims interior to
    // the cross domain) is declined here — the existing path keeps handling it.
    let tol = 0.02 * q_extent;
    if (q_lo - q_dom_lo).abs() > tol || (q_hi - q_dom_hi).abs() > tol {
        return Ok(None);
    }
    if (q_hi - q_lo) <= 0.5 * q_extent {
        return Ok(None);
    }

    // Band rectangle over the full periodic span × the rim cross-levels.
    let (r_ulo, r_uhi, r_vlo, r_vhi) = if p_is_u {
        (u0, u1, q_lo, q_hi)
    } else {
        (q_lo, q_hi, v0, v1)
    };
    let mut rectangle = TrimPolygon::new(domain);
    for corner in [
        [r_ulo, r_vlo],
        [r_uhi, r_vlo],
        [r_uhi, r_vhi],
        [r_ulo, r_vhi],
    ] {
        rectangle.push_point(corner);
    }
    let mut polygons: Vec<TrimPolygon> = vec![rectangle];
    // Keep the non-rim loops (holes / notches), unwrapping any implicit-seam hop
    // so a seam-straddling hole subtracts as one region.
    for (index, loop_record) in face.loops.iter().enumerate() {
        if rim_indices.contains(&index) {
            continue;
        }
        let offsets = crate::topology::loop_seam_offsets(
            &loop_record.coedges,
            closed_u,
            closed_v,
            u1 - u0,
            v1 - v0,
        )?;
        let mut polygon = TrimPolygon::new(domain);
        for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
            polygon.push_coedge(index, coedge_index, coedge, offsets[coedge_index])?;
        }
        polygon.finish();
        polygons.push(polygon);
    }
    Ok(Some(polygons))
}

/// A DOUBLY-periodic (torus) face whose trim is a full-wrap BAND bounded by a
/// constant-cross-level seam RIM (at a cross-domain extreme) and a single-valued
/// WAVY CUT. The raw winding integrator collapses it to a sliver (the rim is a
/// zero-area iso line and the cut's own polygon encloses only the thin strip it
/// bounds), so — mirroring [`singly_periodic_wall_polygons`] for the singly-
/// periodic wall — rebuild the material region as ONE simple (u,v) polygon:
/// the wavy cut as one boundary, the flat rim iso-line (at the OPPOSITE cross
/// extreme, which is the same physical seam circle) as the other, closed by the
/// two private wrap-seam edges. The winding integrator then fills the band.
///
/// Returns None for anything that is not this exact signature (see
/// [`crate::topology::analyze_doubly_periodic_seam_band`]) — every other face
/// keeps its existing integration path bit-for-bit.
pub(super) fn doubly_periodic_seam_band_polygons(
    face: &FaceRecord,
    domain: [f64; 4],
) -> Result<Option<Vec<TrimPolygon>>, String> {
    let (closed_u, closed_v) = face.surface.closed_directions()?;
    if !(closed_u && closed_v) {
        return Ok(None);
    }
    // Sample each loop's pcurves in traversal order; every sample remembers
    // the pcurve parameter it came from so the band polygon's chords keep
    // their arcs through the rotation below.
    type Tagged = ([f64; 2], Option<(usize, usize, f64)>);
    let mut loops_uv: Vec<Vec<[f64; 2]>> = Vec::with_capacity(face.loops.len());
    let mut loops_tagged: Vec<Vec<Tagged>> = Vec::with_capacity(face.loops.len());
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        let mut points: Vec<[f64; 2]> = Vec::new();
        let mut tagged: Vec<Tagged> = Vec::new();
        for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
            let [d0, d1] = coedge.pcurve.domain()?;
            let samples = 24;
            let curved = !is_straight_polyline(&coedge.pcurve);
            for k in 0..=samples {
                let t = d0 + (d1 - d0) * k as f64 / samples as f64;
                let p = coedge.pcurve.evaluate(t)?;
                points.push([p.x, p.y]);
                tagged.push(([p.x, p.y], curved.then_some((loop_index, coedge_index, t))));
            }
        }
        loops_uv.push(points);
        loops_tagged.push(tagged);
    }
    let Some(band) =
        crate::topology::analyze_doubly_periodic_seam_band(&loops_uv, domain, face.same_sense)
    else {
        return Ok(None);
    };
    let vertices = crate::topology::seam_band_uv_polygon_with(
        &loops_tagged,
        |vertex: &Tagged| vertex.0,
        |point| (point, None),
        domain,
        &band,
    );
    let mut polygon = TrimPolygon::new(domain);
    let n = vertices.len();
    for (k, (point, tag)) in vertices.iter().enumerate() {
        polygon.push_point(*point);
        // The chord to the next vertex is an arc only between two samples of
        // one pcurve; the rotation keeps consecutive samples adjacent, and a
        // synthetic vertex (seam extension, rim corner) carries no tag.
        let next = &vertices[(k + 1) % n];
        if let (Some((l0, c0, t0)), Some((l1, c1, t1))) = (tag, next.1) {
            if *l0 == l1 && *c0 == c1 {
                polygon.arcs[k] = Some(ChordArc {
                    loop_index: *l0,
                    coedge_index: *c0,
                    t0: *t0,
                    t1,
                    offset: [0.0, 0.0],
                });
            }
        }
    }
    Ok(Some(vec![polygon]))
}

pub(super) fn is_untrimmed(face: &FaceRecord) -> Result<bool, String> {
    if face.loops.len() != 1 {
        return Ok(false);
    }
    let ku = crate::KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
    let kv = crate::KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    let domain_area = (u1 - u0) * (v1 - v0);
    // The loop's own joints closed: this asks whether the trim IS the domain, and a
    // trim that misses its own joints by 8e-9 is still the domain. The bar is 1e-6 of
    // the DOMAIN area, so on a small parameter box (1.4e-3 on a STEP import) it is
    // 1.4e-9 — under the miss — and the face took the trimmed path though it is whole.
    Ok((closed_parameter_space_area_on_carrier(face)?.abs() - domain_area).abs() <= 1e-6 * domain_area)
}

/// Symmetric degree-5 triangle cubature (barycentric points and weights,
/// normalized to unit area) — Golovanov §8.10 Table 8.10.2.
pub(super) const TRIANGLE_CUBATURE: [([f64; 3], f64); 7] = [
    ([1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0], 0.225),
    (
        [0.101286507323456, 0.101286507323456, 0.797426985353087],
        0.125939180544827,
    ),
    (
        [0.101286507323456, 0.797426985353087, 0.101286507323456],
        0.125939180544827,
    ),
    (
        [0.797426985353087, 0.101286507323456, 0.101286507323456],
        0.125939180544827,
    ),
    (
        [0.470142064105115, 0.470142064105115, 0.059715871789770],
        0.132394152788506,
    ),
    (
        [0.470142064105115, 0.059715871789770, 0.470142064105115],
        0.132394152788506,
    ),
    (
        [0.059715871789770, 0.470142064105115, 0.470142064105115],
        0.132394152788506,
    ),
];

pub(super) fn polygon_signed_area(polygon: &[[f64; 2]]) -> f64 {
    let mut area = 0.0;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        area += a[0] * b[1] - b[0] * a[1];
    }
    area * 0.5
}

/// Sutherland–Hodgman clip of one loop against an axis-aligned cell.  A
/// loop that fully encloses the cell clips to the whole cell; a disjoint
/// loop clips to nothing — so clipping every loop handles containment,
/// holes, and partial coverage uniformly, with the loop's winding as the
/// contribution sign.
///
/// The four half-plane passes write into caller-owned buffers: the two working
/// vectors are ping-ponged for the whole call instead of allocated per pass. The integrator clips every trim loop
/// against every cell of a knot-conforming grid and then against every
/// quadtree child of every boundary cell, so the five vectors a call used to
/// allocate were the shape of its clipping cost.
pub(super) fn clip_polygon_to_cell_into(
    polygon: &[[f64; 2]],
    cell: [f64; 4],
    scratch: &mut Vec<[f64; 2]>,
    out: &mut Vec<[f64; 2]>,
) {
    let [u_low, u_high, v_low, v_high] = cell;
    out.clear();
    out.extend_from_slice(polygon);
    // (axis, bound, keep_below)
    for (axis, bound, keep_below) in [
        (0, u_low, false),
        (0, u_high, true),
        (1, v_low, false),
        (1, v_high, true),
    ] {
        if out.len() < 3 {
            out.clear();
            return;
        }
        let inside = |point: &[f64; 2]| {
            if keep_below {
                point[axis] <= bound
            } else {
                point[axis] >= bound
            }
        };
        scratch.clear();
        scratch.reserve(out.len() + 4);
        for index in 0..out.len() {
            let a = out[index];
            let b = out[(index + 1) % out.len()];
            let a_in = inside(&a);
            let b_in = inside(&b);
            if a_in {
                scratch.push(a);
            }
            if a_in != b_in {
                let t = (bound - a[axis]) / (b[axis] - a[axis]);
                let mut crossing = [0.0; 2];
                crossing[axis] = bound;
                crossing[1 - axis] = a[1 - axis] + t * (b[1 - axis] - a[1 - axis]);
                scratch.push(crossing);
            }
        }
        std::mem::swap(scratch, out);
    }
}

pub(super) fn cell_breaks(knots: &[f64], degree: usize, low: f64, high: f64) -> Vec<f64> {
    let mut breaks = vec![low, high];
    breaks.extend(interior_knots(knots, degree));
    breaks.sort_by(f64::total_cmp);
    breaks.dedup_by(|a, b| (*a - *b).abs() <= (high - low) * 1e-12);
    // Subdivide spans so the base grid has at least MINIMUM_CELLS cells
    // across the domain; boundary cells then refine adaptively.
    const MINIMUM_CELLS: usize = 8;
    let target = (high - low) / MINIMUM_CELLS as f64;
    let mut refined = Vec::with_capacity(breaks.len() * 2);
    for pair in breaks.windows(2) {
        refined.push(pair[0]);
        let span = pair[1] - pair[0];
        let pieces = (span / target).ceil().max(1.0) as usize;
        for piece in 1..pieces {
            refined.push(pair[0] + span * piece as f64 / pieces as f64);
        }
    }
    refined.push(high);
    refined
}

/// Tile one period's `cell_breaks` across `[lo, hi]` for a periodic direction
/// whose unwrapped trim ran past the domain end. Shifted copies of the base
/// breaks keep the knot-conforming grid on every covered period; a direction
/// that stayed in-domain yields exactly the base breaks.
pub(super) fn tiled_breaks(base: &[f64], lo: f64, hi: f64, period: f64) -> Vec<f64> {
    if period <= 0.0 || base.len() < 2 {
        return base.to_vec();
    }
    let origin = base[0];
    let first = ((lo - origin) / period).floor() as i64;
    let last = ((hi - origin) / period).ceil() as i64;
    let mut breaks = Vec::new();
    for tile in first..=last {
        let shift = tile as f64 * period;
        for &value in base {
            let shifted = value + shift;
            if breaks.last().map_or(true, |&previous: &f64| {
                (shifted - previous).abs() > 1e-12 * period
            }) {
                breaks.push(shifted);
            }
        }
    }
    breaks
}
