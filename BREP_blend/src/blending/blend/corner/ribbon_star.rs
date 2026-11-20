//! A convex vertex whose N stripes have no common inscribed ball is not
//! closed by a sphere.  Each stripe ends on a RIB — a curve on its own blend
//! surface from its rail/rail point on one mate to its rail/rail point on
//! the other — and the N ribs bound a hole.  Four ribs are filled here by a
//! bicubic Coons patch ([`crate::coons_with_ribbons`]) that interpolates the
//! ribs exactly and carries, along each, a G1 RIBBON: a cross-boundary
//! derivative field lying in that rib's blend surface's tangent plane, so
//! the patch is tangent-plane continuous to every blend across its rib.
//! The ribbon's end vectors are the neighbouring ribs' tangents at the
//! shared corner (the Coons corner condition), which lie in both blends'
//! tangent planes there because both blends touch the corner's face at the
//! rail/rail point; between the ends the field is the linear blend of those
//! two vectors projected into the tangent plane at each sample.  The
//! report's cross-derivative residual is what the patch ACHIEVED, and is
//! what a caller reads before calling the join G1.  Three ribs and five or
//! more are named, not built.

use super::*;

/// One rib of the hole: the edge stored in its stripe's loop-walk direction,
/// the blend surface it lies on and its pcurve there in that same direction.
pub(in crate::blend) struct RibbonRib<'a> {
    pub(in crate::blend) edge: EdgeRecord,
    pub(in crate::blend) surface: &'a NurbsSurface,
    pub(in crate::blend) pcurve: NurbsCurve,
}

/// What the fill measured.
pub(in crate::blend) struct RibbonFillReading {
    /// The Coons report's worst `|dS/dn − T|` along the ribs.
    pub(in crate::blend) cross_derivative_residual: f64,
    /// The worst `|T|` it is read against.
    pub(in crate::blend) cross_derivative_scale: f64,
    /// The twist disagreement at the corners the patch averaged.
    pub(in crate::blend) twist_discrepancy: f64,
}

/// Samples per rib for the ribbon fields.
const RIBBON_SAMPLES: usize = 64;
/// Control points of a fitted ribbon.
const RIBBON_CONTROLS: usize = 16;

/// A clamped cubic B-spline through `start` and `end` with ZERO derivative at
/// both (its second control point repeats the first, its penultimate the
/// last), its eight free control points fitted to `samples` at `parameters`
/// by least squares on uniform knots.
fn fit_ribbon(samples: &[Vec3], parameters: &[f64], start: Vec3, end: Vec3) -> Result<NurbsCurve, KernelRefusal> {
    let degree = 3usize;
    let m = RIBBON_CONTROLS;
    let (t0, t1) = (parameters[0], parameters[parameters.len() - 1]);
    let mut knots = vec![t0; degree + 1];
    let interior = m - degree - 1;
    for j in 1..=interior {
        knots.push(t0 + (t1 - t0) * j as f64 / (interior + 1) as f64);
    }
    knots.extend(std::iter::repeat_n(t1, degree + 1));
    let knot_vector = crate::KnotVector::new(knots.clone(), degree).or_refuse(KernelStage::Refine, "KnotVector")?;
    // Free controls P2 .. P_{m-3}; P0 = P1 = start, P_{m-2} = P_{m-1} = end.
    let free = m - 4;
    let mut normal = vec![vec![0.0f64; free]; free];
    let mut rhs = vec![Vec3::default(); free];
    for (sample, &parameter) in samples.iter().zip(parameters) {
        let span = knot_vector.find_span(parameter);
        let basis = knot_vector.basis_functions(span, parameter);
        let mut row = vec![0.0f64; m];
        for (offset, value) in basis.into_iter().enumerate() {
            row[span - degree + offset] = value;
        }
        let known = start.scale(row[0] + row[1]).add(end.scale(row[m - 2] + row[m - 1]));
        let residual = sample.sub(known);
        for a in 0..free {
            rhs[a] = rhs[a].add(residual.scale(row[2 + a]));
            for b in 0..free {
                normal[a][b] += row[2 + a] * row[2 + b];
            }
        }
    }
    let solve = |axis: fn(Vec3) -> f64| {
        fit::solve_dense(normal.clone(), rhs.iter().copied().map(axis).collect()).or_refuse(KernelStage::Refine, "solve_dense")
    };
    let (xs, ys, zs) = (solve(|p| p.x)?, solve(|p| p.y)?, solve(|p| p.z)?);
    let mut controls = Vec::with_capacity(m);
    controls.push(Vec4::from_point(start, 1.0));
    controls.push(Vec4::from_point(start, 1.0));
    for index in 0..free {
        controls.push(Vec4::from_point(Vec3::new(xs[index], ys[index], zs[index]), 1.0));
    }
    controls.push(Vec4::from_point(end, 1.0));
    controls.push(Vec4::from_point(end, 1.0));
    NurbsCurve::new(degree, knots, controls).or_refuse(KernelStage::Refine, "NurbsCurve::new")
}

/// Build the four-sided ribbon fill bounded by `ribs`, each stored in its
/// stripe's walk direction and used `forward = false` here (the patch walks
/// the hole the other way round, as the sphere patch does).  `outward` says
/// which way the material lies (a convex corner faces away from `center`).
pub(in crate::blend) fn build_ribbon_star_patch(
    ribs: &[RibbonRib<'_>],
    center: Vec3,
    outward: bool,
    name: Option<&str>,
    next_id: &mut dyn FnMut() -> u64,
) -> Result<(FaceRecord, RibbonFillReading), KernelRefusal> {
    let n = ribs.len();
    if n != 4 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Refine,
            "n_sided_corner",
            format!(
                "blend network: the no-common-ball star has {n} ribs; the ribbon fill is \
                 four-sided (three sides by a degenerate top curve and N above four are not in \
                 this lane)"
            ),
        ));
    }
    // Chain the ribs into a closed loop traversed OPPOSITE to their stored
    // direction: each successor's stored END meets the current rib's stored
    // START.
    let mut order = vec![0usize];
    let mut used = vec![false; n];
    used[0] = true;
    let mut chain_vertex = ribs[0].edge.start_vertex_id;
    for _ in 0..(n - 1) {
        let next = (0..n)
            .find(|&j| !used[j] && ribs[j].edge.end_vertex_id == chain_vertex)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "rib_chain",
                    "blend network: the star's ribs do not form a closed N-gon",
                )
            })?;
        used[next] = true;
        order.push(next);
        chain_vertex = ribs[next].edge.start_vertex_id;
    }
    if chain_vertex != ribs[0].edge.end_vertex_id {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "rib_loop",
            "blend network: the star's rib loop is not closed",
        ));
    }
    // The patch's traversal of rib `order[k]` runs from its stored end to its
    // stored start: corners c0 → c1 → c2 → c3 → c0.  The Coons constructor
    // takes the four boundaries in that CYCLIC order — S0 = c0→c1 is v = 0,
    // S1 = c1→c2 is u = 1, S2 = c2→c3 is v = 1 and S3 = c3→c0 is u = 0, the
    // last two running AGAINST the square's parameters — and reports any
    // boundary it had to turn to make the chain consistent (none here).
    let traversal = |k: usize| -> Result<(NurbsCurve, NurbsCurve), KernelRefusal> {
        let rib = &ribs[order[k]];
        Ok((
            rib.edge.curve.reversed().or_refuse(KernelStage::Refine, "reversed")?,
            rib.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?,
        ))
    };
    let (t0, p0) = traversal(0)?;
    let (t1, p1) = traversal(1)?;
    let (t2, p2) = traversal(2)?;
    let (t3, p3) = traversal(3)?;
    // (boundary curve, its pcurve on its blend, that blend)
    let boundaries: [(NurbsCurve, NurbsCurve, &NurbsSurface); 4] = [
        (t0, p0, ribs[order[0]].surface),
        (t1, p1, ribs[order[1]].surface),
        (t2, p2, ribs[order[2]].surface),
        (t3, p3, ribs[order[3]].surface),
    ];
    let derivative_at = |curve: &NurbsCurve, fraction: f64| -> Result<Vec3, KernelRefusal> {
        let [start, end] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let derivatives = curve
            .derivatives(start + (end - start) * fraction, 1)
            .or_refuse(KernelStage::Refine, "derivatives")?;
        // The ribbon is read against the boundary's own parameter, which
        // runs over its domain; the Coons constructor reads both on [0, 1].
        Ok(derivatives[1].scale(end - start))
    };
    // A ribbon is the patch's derivative across its boundary in the
    // INCREASING direction of the crossing parameter, parameterised over the
    // boundary as supplied.  At each corner that derivative IS the adjacent
    // boundary's tangent in the square's increasing direction: along S0
    // (v = 0) it is dS/dv, which at c0 is the u = 0 side walked c0 → c3, the
    // reverse of S3 at its end, and at c1 the u = 1 side walked c1 → c2, S1
    // at its start; along S1 (u = 1) dS/du is S0 at its end (c1) and, at c2,
    // the v = 1 side walked c3 → c2, the reverse of S2 at its start; along S2
    // (v = 1) dS/dv is S1 at its end (c2) and the reverse of S3 at its start
    // (c3); along S3 (u = 0) dS/du is the reverse of S2 at its end (c3) and
    // S0 at its start (c0).
    let ends: [[Vec3; 2]; 4] = [
        [derivative_at(&boundaries[3].0, 1.0)?.scale(-1.0), derivative_at(&boundaries[1].0, 0.0)?],
        [derivative_at(&boundaries[0].0, 1.0)?, derivative_at(&boundaries[2].0, 0.0)?.scale(-1.0)],
        [derivative_at(&boundaries[1].0, 1.0)?, derivative_at(&boundaries[3].0, 0.0)?.scale(-1.0)],
        [derivative_at(&boundaries[2].0, 1.0)?.scale(-1.0), derivative_at(&boundaries[0].0, 0.0)?],
    ];
    let mut ribbons: [Option<NurbsCurve>; 4] = [None, None, None, None];
    for k in 0..4 {
        let (curve, pcurve, surface) = &boundaries[k];
        let [start, end] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let [p_start, p_end] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mut samples: Vec<Vec3> = Vec::with_capacity(RIBBON_SAMPLES + 1);
        let mut parameters: Vec<f64> = Vec::with_capacity(RIBBON_SAMPLES + 1);
        for index in 0..=RIBBON_SAMPLES {
            let fraction = index as f64 / RIBBON_SAMPLES as f64;
            let vector = if index == 0 {
                ends[k][0]
            } else if index == RIBBON_SAMPLES {
                ends[k][1]
            } else {
                let uv = pcurve
                    .evaluate(p_start + (p_end - p_start) * fraction)
                    .or_refuse(KernelStage::Refine, "evaluate")?;
                let normal = raw_normal(surface, uv.x, uv.y)?;
                // A cubic Hermite between the end vectors with ZERO end
                // derivatives, so both ribbons at a corner carry the same
                // (zero) twist, then projected into the blend's tangent
                // plane; at the ends the projection changes nothing because
                // the rib leaves the corner along the other blend's ruling.
                let (s2, s3) = (fraction * fraction, fraction * fraction * fraction);
                let (h00, h01) = (2.0 * s3 - 3.0 * s2 + 1.0, -2.0 * s3 + 3.0 * s2);
                let blended = ends[k][0].scale(h00).add(ends[k][1].scale(h01));
                blended.sub(normal.scale(blended.dot(normal)))
            };
            samples.push(vector);
            parameters.push(start + (end - start) * fraction);
        }
        // Fitted with ZERO end derivatives built in (the field's own, and
        // what both ribbons at a corner must agree on): a derivative-free
        // interpolant reads a slope of its own at the ends and the twists
        // disagree by it (1.2e-2 at 48 samples, 2.5e-2 at 96 on the pyramid
        // apex); `interpolate_curve_with_end_tangents` is singular above
        // about twenty points and ill-conditioned below.
        ribbons[k] = Some(fit_ribbon(&samples, &parameters, ends[k][0], ends[k][1])?);
    }
    let [b0, b1, b2, b3] = boundaries;
    let (surface, report) = crate::coons_with_ribbons([b0.0, b1.0, b2.0, b3.0], ribbons).map_err(|refusal| {
        KernelRefusal::unsupported(
            KernelStage::Refine,
            "ribbon_fill",
            format!("blend network: the star's ribbon fill refused: {refusal}"),
        )
    })?;
    let ribbon_report = report.ribbons.as_ref().ok_or(KernelRefusal::internal(
        KernelStage::Refine,
        "ribbon_report",
        "blend network: the ribbon fill carried no ribbon report",
    ))?;
    // The cyclic order was supplied consistently, so the constructor had
    // nothing to reverse; a reversal here would mean the ribs' chain and the
    // square disagree, and the ribbons' end vectors with them.
    if report.reversed.iter().any(|reversed| *reversed) {
        return Err(KernelRefusal::internal(
            KernelStage::Refine,
            "ribbon_orientation",
            format!("blend network: the ribbon fill reversed boundaries {:?} of a cyclic chain", report.reversed),
        ));
    }
    let reading = RibbonFillReading {
        cross_derivative_residual: ribbon_report.cross_derivative_residual,
        cross_derivative_scale: ribbon_report.cross_derivative_scale,
        twist_discrepancy: ribbon_report.twist_discrepancy,
    };
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        eprintln!(
            "ribbon star fill: cross-derivative residual {:.3e} against scale {:.3e}, twist \
             discrepancy {:.3e}, corner tangent gap {:.3e}, boundary residual {:.3e}",
            reading.cross_derivative_residual,
            reading.cross_derivative_scale,
            reading.twist_discrepancy,
            ribbon_report.corner_tangent_gap,
            report.boundary_residual
        );
    }
    // The patch's loop: c0 → c1 along v = 0, c1 → c2 along u = 1, c2 → c3
    // along v = 1 (against b2), c3 → c0 along u = 0 (against b3); each rib
    // used `forward = false`.
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let lines = [
        crate::sweep_topology::parameter_line(u0, v0, u1, v0),
        crate::sweep_topology::parameter_line(u1, v0, u1, v1),
        crate::sweep_topology::parameter_line(u1, v1, u0, v1),
        crate::sweep_topology::parameter_line(u0, v1, u0, v0),
    ];
    let mut coedges = Vec::with_capacity(4);
    for (k, line) in lines.into_iter().enumerate() {
        coedges.push(CoedgeRecord {
            id: next_id(),
            edge_id: ribs[order[k]].edge.id,
            forward: false,
            pcurve: line.or_refuse(KernelStage::Refine, "parameter_line")?,
        });
    }
    // same_sense so the face normal points away from the centre for a convex
    // corner, toward it for a concave one.
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let middle = surface.evaluate(um, vm).or_refuse(KernelStage::Refine, "evaluate")?;
    let surface_normal = surface.normal(um, vm).or_refuse(KernelStage::Refine, "normal")?;
    let away = middle.sub(center).normalized().or_refuse(KernelStage::Refine, "normalized")?;
    let same_sense = (surface_normal.dot(away) > 0.0) == outward;
    let loop_id = next_id();
    Ok((
        FaceRecord {
            id: next_id(),
            surface,
            same_sense,
            loops: vec![LoopRecord { id: loop_id, coedges }],
            name: name.map(|value| value.to_string()),
        },
        reading,
    ))
}

/// The no-common-ball star with N ≠ 4 ribs closed by the N-sided fill
/// ([`crate::n_sided_fill`], Golovanov §3.5): N four-sided patches meeting at
/// a centre, each rib split at its parameter midpoint where two patches meet
/// it, one spoke edge from each midpoint to the centre. What the caller
/// commits: the new vertices and edges, each rib's split (so the stripe's
/// loop takes the two halves in place of the rib), and the faces.
pub(in crate::blend) struct NSidedStar {
    pub(in crate::blend) vertices: Vec<VertexRecord>,
    pub(in crate::blend) edges: Vec<EdgeRecord>,
    /// (the rib's edge id, its first half's id, its second half's id, the
    /// rib's pcurve split there: (first half, second half)) — halves in the
    /// rib's stored direction.
    pub(in crate::blend) splits: Vec<(u64, u64, u64, (NurbsCurve, NurbsCurve))>,
    pub(in crate::blend) faces: Vec<FaceRecord>,
    pub(in crate::blend) spoke_normal_angle: f64,
    pub(in crate::blend) tangent_normal_angle: f64,
}

/// The N-sided star: a no-common-ball star of 3, 5 or 6 ribs closed with the
/// N-sided fill ([`build_n_sided_star`]), and the laws eased to zero slope at
/// the stations they are held from. On by default; `BREP_BLEND_N_SIDED_STAR=0`
/// restores the refusal of an unequal tapered star and the laws held with a
/// slope break.
pub fn n_sided_star_on() -> bool {
    switch!(live "BREP_BLEND_N_SIDED_STAR", true)
}


/// `BREP_BLEND_N_SIDED_FOLD_REACH` (on by default; `=0` restores the fold
/// and spoke-bar refusals and the per-patch sense): a star whose N-sided fill
/// folds beside a side or misses the angle bars is filled again with its
/// corner-twist bumps shortened and its spokes kept 15 degrees off their sides
/// (`n_sided_fill_shortening_reach`), and every patch of a star takes the
/// sense of its most decisive one.
pub fn n_sided_fold_reach_on() -> bool {
    switch!(live "BREP_BLEND_N_SIDED_FOLD_REACH", true)
}

/// The refusal code of a star whose corner stands outside a non-adjacent
/// rib's blend ([`corner_outside_a_blend`]).
pub const N_SIDED_UNDERCUT_WHAT: &str = "n_sided_undercut";

/// The refusal code of a star whose N-sided fill folds in a patch: a limit
/// of the fill's spoke and centre layout, not of the geometry.
pub const N_SIDED_FOLD_WHAT: &str = "n_sided_fold";

/// Whether `refusal` is a star's that no smooth corner fill answers on any
/// lane ([`N_SIDED_UNDERCUT_WHAT`] or [`N_SIDED_FOLD_WHAT`]).
pub fn is_n_sided_undercut_refusal(refusal: &KernelRefusal) -> bool {
    matches!(&refusal.class, crate::RefusalClass::UnsupportedGeometry { what }
        if what == N_SIDED_UNDERCUT_WHAT || what == N_SIDED_FOLD_WHAT)
}

/// The worst normal angle (radians) the N-sided star's patches may make with
/// each other across a spoke or with a blend across a rib: the angle the
/// kernel's G1 joins are read at.
const N_SIDED_ANGLE_BAR: f64 = 1e-4;

/// Exact samples per side of the N-sided fill's tangent field.
const N_SIDED_FIELD_SAMPLES: usize = 128;

pub(in crate::blend) fn build_n_sided_star(
    ribs: &[RibbonRib<'_>],
    center: Vec3,
    outward: bool,
    name: Option<&str>,
    next_id: &mut dyn FnMut() -> u64,
) -> Result<NSidedStar, KernelRefusal> {
    let n = ribs.len();
    // Chain the ribs into the loop the fill walks, opposite to their stored
    // direction (as the four-sided fill does).
    let mut order = vec![0usize];
    let mut used = vec![false; n];
    used[0] = true;
    let mut chain_vertex = ribs[0].edge.start_vertex_id;
    for _ in 0..(n - 1) {
        let next = (0..n)
            .find(|&j| !used[j] && ribs[j].edge.end_vertex_id == chain_vertex)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "rib_chain", "blend network: the star's ribs do not form a closed N-gon"))?;
        used[next] = true;
        order.push(next);
        chain_vertex = ribs[next].edge.start_vertex_id;
    }
    if chain_vertex != ribs[0].edge.end_vertex_id {
        return Err(KernelRefusal::internal(KernelStage::Fragment, "rib_loop", "blend network: the star's rib loop is not closed"));
    }
    // Side k: rib order[k] walked end -> start; the fill's tangent field
    // across it: the blend's normal crossed with the side's tangent, in the
    // blend's tangent plane and across the rib.
    let mut sides = Vec::with_capacity(n);
    let mut fields = Vec::with_capacity(n);
    for &index in &order {
        let rib = &ribs[index];
        let side = rib.edge.curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        let pcurve = rib.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        let [s0, s1] = side.domain().or_refuse(KernelStage::Refine, "domain")?;
        let [p0, p1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mut samples = Vec::with_capacity(N_SIDED_FIELD_SAMPLES + 1);
        let mut parameters = Vec::with_capacity(N_SIDED_FIELD_SAMPLES + 1);
        for k in 0..=N_SIDED_FIELD_SAMPLES {
            let f = k as f64 / N_SIDED_FIELD_SAMPLES as f64;
            let t = s0 + (s1 - s0) * f;
            let tangent = side.derivatives(t, 1).or_refuse(KernelStage::Refine, "derivatives")?[1];
            let uv = pcurve.evaluate(p0 + (p1 - p0) * f).or_refuse(KernelStage::Refine, "evaluate")?;
            let normal = raw_normal(rib.surface, uv.x, uv.y)?;
            let across = normal.cross(tangent).normalized().or_refuse(KernelStage::Refine, "n_sided_field")?;
            samples.push(across);
            parameters.push(t);
        }
        // The field interpolated through every exact sample: a fit with
        // fewer poles than samples turned it up to 5.2e-3 rad off the true
        // normals (on a tapered box's star), and the fill matches
        // whatever planes it is handed.
        fields.push(crate::fit::interpolate_curve(&samples, 3, &parameters).or_refuse(KernelStage::Refine, "n_sided_field_fit")?);
        sides.push(side);
    }
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        // At each corner P_k (side k's start, side k-1's end): the two blends'
        // normals there and how far each side's end tangent leaves the other
        // blend's tangent plane.
        let at = |k: usize, start: bool| -> Result<(Vec3, Vec3, Vec3), KernelRefusal> {
            let rib = &ribs[order[k]];
            let side = rib.edge.curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
            let pcurve = rib.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
            let [s0, s1] = side.domain().or_refuse(KernelStage::Refine, "domain")?;
            let [p0, p1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let (t, q) = if start { (s0, p0) } else { (s1, p1) };
            let d = side.derivatives(t, 1).or_refuse(KernelStage::Refine, "derivatives")?;
            let uv = pcurve.evaluate(q).or_refuse(KernelStage::Refine, "evaluate")?;
            let normal = raw_normal(rib.surface, uv.x, uv.y)?.normalized().or_refuse(KernelStage::Refine, "normalized")?;
            let on_blend = rib.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?;
            Ok((d[0], d[1].normalized().or_refuse(KernelStage::Refine, "normalized")?, normal.scale(if on_blend.sub(d[0]).length() > 1e-6 { f64::NAN } else { 1.0 })))
        };
        for k in 0..n {
            let previous = (k + n - 1) % n;
            let (p_k, t_k, n_k) = at(k, true)?;
            let (p_prev, t_prev, n_prev) = at(previous, false)?;
            eprintln!(
                "n-sided corner {k}: gap {:.3e}, blend normals {:.3e} rad apart, side {k} start tangent . blend {previous} normal {:.3e}, side {previous} end tangent . blend {k} normal {:.3e}, at {p_k:?}",
                p_k.sub(p_prev).length(), n_k.dot(n_prev).clamp(-1.0, 1.0).acos(), t_k.dot(n_prev), t_prev.dot(n_k)
            );
        }
    }
    let (patches, report) = match crate::n_sided_fill(&sides, Some(&fields)) {
        Ok(filled) => filled,
        Err(refusal) => {
            // A loop the fill reads as folded (a re-entrant corner, a side
            // turning away from the centre) is, at a star of unequal radii,
            // a corner standing outside a non-adjacent rib's blend: no
            // smooth fill reaches it without an undercut lip. Say so (by
            // depth, not position: a far solid is filleted recentred).
            if matches!(refusal, crate::NSidedRefusal::ReentrantTangent { .. } | crate::NSidedRefusal::Centre { .. }) {
                if let Some((_, side, depth)) = corner_outside_a_blend(ribs, &order, &sides)? {
                    return Err(KernelRefusal::unsupported(KernelStage::Refine, N_SIDED_UNDERCUT_WHAT, format!(
                        "blend network: a corner of the star lies {depth:.3e} outside the blend of its rib {side} \
                         (the rounds meet at radii too unequal: a smooth corner fill would need an undercut lip; at a box \
                         corner of radii r, r and R this happens for r < (1 - 1/sqrt 2) R = 0.293 R)"
                    )));
                }
            }
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "n_sided_fill", format!(
                "blend network: the star's N-sided fill refused: {refusal:?}"
            )));
        }
    };
    // A fill that folds beside a side, or misses the angle bars, is tried
    // again with its corner-twist bumps reaching only as far as keeps the
    // across component and its spokes leaving at least 15 degrees off their
    // sides (`BREP_BLEND_N_SIDED_FOLD_REACH`); kept only when no patch of it
    // folds and it meets both bars, so a star that fills today is filled as
    // before.
    let meets_bars = |report: &crate::NSidedReport| {
        report.spoke_normal_angle <= N_SIDED_ANGLE_BAR && report.tangent_normal_angle.unwrap_or(f64::INFINITY) <= N_SIDED_ANGLE_BAR
    };
    let regular = |patches: &[crate::NurbsSurface]| patches.iter().all(|patch| crate::certify_regular(patch).is_ok());
    // The spokes are moved off their sides only when the shortened bumps
    // alone do not fill it: that moves the fill further (a 1 -> 3 star's by
    // about 0.02 in volume, against 0.002 for the bumps alone).
    let (patches, report) = if n_sided_fold_reach_on() && !(meets_bars(&report) && regular(&patches)) {
        let retries: [fn(&[crate::NurbsCurve], &[crate::NurbsCurve]) -> Result<(Vec<crate::NurbsSurface>, crate::NSidedReport), crate::NSidedRefusal>; 2] =
            [crate::n_sided_fill_shortening_reach, crate::n_sided_fill_shortening_reach_spokes_off_sides];
        let refilled = retries.into_iter().find_map(|retry| {
            retry(&sides, &fields)
                .ok()
                .filter(|refilled| meets_bars(&refilled.1) && regular(&refilled.0))
        });
        refilled.unwrap_or((patches, report))
    } else {
        (patches, report)
    };
    if report.reversed.iter().any(|reversed| *reversed) {
        return Err(KernelRefusal::internal(KernelStage::Refine, "n_sided_orientation", format!(
            "blend network: the N-sided fill reversed sides {:?} of a cyclic chain", report.reversed
        )));
    }
    let tangent_angle = report.tangent_normal_angle.unwrap_or(f64::INFINITY);
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        eprintln!(
            "n-sided star fill: {n} sides, spoke normal angle {:.3e}, tangent normal angle {:.3e}, boundary deviation {:.3e}, corner gap {:.3e}",
            report.spoke_normal_angle, tangent_angle, report.boundary_deviation, report.corner_gap
        );
    }
    if !(report.spoke_normal_angle <= N_SIDED_ANGLE_BAR && tangent_angle <= N_SIDED_ANGLE_BAR) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "n_sided_g1", format!(
            "blend network: the star's N-sided fill is not tangent-continuous: {:.3e} rad across a spoke, \
             {tangent_angle:.3e} rad against a blend (bar {N_SIDED_ANGLE_BAR:.1e})",
            report.spoke_normal_angle
        )));
    }
    // Each patch free of folds: the angle bars read the joins, not the
    // patches between them (a star whose rounds meet near the undercut onset,
    // at radii 1 against 3, met both bars and still turned back beside a rib).
    // A limit of this construction, not of the geometry: a smooth fill
    // exists there (the corner is inside every blend).
    for (k, patch) in patches.iter().enumerate() {
        if let Err(irregularity) = crate::certify_regular(patch) {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, N_SIDED_FOLD_WHAT, format!(
                "blend network: the star's N-sided fill folds in its patch {k} ({irregularity:?}): the current N-sided \
                 construction folds here (its spokes leave the ribs nearly along them, as at a box corner of radii r, r \
                 and R for about 0.293 R < r < 0.35 R); a different spoke and centre layout may fill it"
            )));
        }
    }
    // The star's own size (its ribs' ends and midpoints): a bar read against
    // it, not against the distance from the world origin, reads the same
    // wherever the star stands.
    let mut points = Vec::with_capacity(3 * n);
    for side in &sides {
        let [a, b] = side.domain().or_refuse(KernelStage::Refine, "domain")?;
        for t in [a, 0.5 * (a + b), b] {
            points.push(side.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?);
        }
    }
    let size = crate::model_scale(points);
    // Split each rib where the fill cut its side (the side's, reversed).
    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let mut splits = Vec::new();
    let mut midpoint_vertex = vec![0u64; n];
    // halves[k] = (first half of side k: its stored-direction SECOND half,
    // second half of side k: its stored-direction FIRST half)
    let mut halves = vec![(0u64, 0u64); n];
    for (k, &index) in order.iter().enumerate() {
        let rib = &ribs[index];
        let [t0, t1] = rib.edge.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        // where the fill cut side k (the rib reversed: side parameter s is
        // the rib's t0 + t1 - s), its midpoint or a knot right beside it
        let cut = report.split_parameters.get(k).copied().flatten().unwrap_or(0.5 * (t0 + t1));
        let tm = t0 + t1 - cut;
        let fraction = (tm - t0) / (t1 - t0);
        let point = rib.edge.curve.evaluate(tm).or_refuse(KernelStage::Refine, "evaluate")?;
        let (c1, c2) = rib.edge.curve.split(tm).or_refuse(KernelStage::Refine, "split")?;
        let [q0, q1] = rib.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let qm = q0 + fraction * (q1 - q0);
        let on_blend = rib.pcurve.evaluate(qm).or_refuse(KernelStage::Refine, "evaluate")?;
        let off = rib.surface.evaluate(on_blend.x, on_blend.y).or_refuse(KernelStage::Refine, "evaluate")?.sub(point).length();
        if off > 1e-7 * (1.0 + size) {
            return Err(KernelRefusal::internal(KernelStage::Refine, "n_sided_split", format!(
                "blend network: a rib's pcurve and edge part at its midpoint by {off:.3e}"
            )));
        }
        let pcurve_halves = rib.pcurve.split(qm).or_refuse(KernelStage::Refine, "split")?;
        let m = next_id();
        vertices.push(VertexRecord { id: m, point });
        midpoint_vertex[k] = m;
        let (e1, e2) = (next_id(), next_id());
        for (id, curve, start, end) in [(e1, c1, rib.edge.start_vertex_id, m), (e2, c2, m, rib.edge.end_vertex_id)] {
            let [a, b] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            edges.push(EdgeRecord { id, curve, t0: a, t1: b, start_vertex_id: start, end_vertex_id: end, degenerate: false, name: None });
        }
        splits.push((rib.edge.id, e1, e2, pcurve_halves));
        // Walked end -> start, side k's first half is the stored second half.
        halves[k] = (e2, e1);
    }
    let q = next_id();
    vertices.push(VertexRecord { id: q, point: report.centre });
    // Spoke k: M_k -> Q, patch k's u = 1 side.
    let mut spokes = vec![0u64; n];
    for k in 0..n {
        let surface = &patches[k];
        let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let _ = u0;
        let curve = surface.iso_curve_u(u1).or_refuse(KernelStage::Refine, "iso_curve_u")?;
        let [a, b] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let id = next_id();
        edges.push(EdgeRecord { id, curve, t0: a, t1: b, start_vertex_id: midpoint_vertex[k], end_vertex_id: q, degenerate: false, name: None });
        spokes[k] = id;
    }
    // Which way each patch faces: its normal against the way from the star's
    // centre to its middle. The patches meet tangentially across the spokes
    // with their normals on one side, so they face one way; on a long thin
    // patch the way from the centre runs nearly along the patch and its sign
    // is noise (a star of radii 1 against 3 turned three patches inside out,
    // a valid body 2.8 off in volume). With `BREP_BLEND_N_SIDED_FOLD_REACH`
    // the most decisive patch sets the sense for all of them.
    let mut leanings = Vec::with_capacity(n);
    for patch in &patches {
        let [u0, u1] = patch.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = patch.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
        let middle = patch.evaluate(um, vm).or_refuse(KernelStage::Refine, "evaluate")?;
        let normal = patch.normal(um, vm).or_refuse(KernelStage::Refine, "normal")?;
        let away = middle.sub(center).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        // the reading as it always was (the switch off), and its cosine
        // (which patch is most decisive)
        leanings.push((normal.dot(away), normal.dot(away) / normal.length().max(f64::MIN_POSITIVE)));
    }
    let decisive = leanings.iter().map(|&(_, cosine)| cosine).fold(0.0_f64, |best, cosine| if cosine.abs() > best.abs() { cosine } else { best });
    let mut faces = Vec::with_capacity(n);
    for k in 0..n {
        let surface = patches[k].clone();
        let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let previous = (k + n - 1) % n;
        let sides_of_patch = [
            (halves[k].0, false, (u0, v0, u1, v0)),
            (spokes[k], true, (u1, v0, u1, v1)),
            (spokes[previous], false, (u1, v1, u0, v1)),
            (halves[previous].1, false, (u0, v1, u0, v0)),
        ];
        let mut coedges = Vec::with_capacity(4);
        for (edge_id, forward, (a, b, c, d)) in sides_of_patch {
            coedges.push(CoedgeRecord {
                id: next_id(),
                edge_id,
                forward,
                pcurve: crate::sweep_topology::parameter_line(a, b, c, d).or_refuse(KernelStage::Refine, "parameter_line")?,
            });
        }
        let leaning = if n_sided_fold_reach_on() { decisive } else { leanings[k].0 };
        let same_sense = (leaning > 0.0) == outward;
        let loop_id = next_id();
        faces.push(FaceRecord {
            id: next_id(),
            surface,
            same_sense,
            loops: vec![LoopRecord { id: loop_id, coedges }],
            name: name.map(|value| if n == 1 { value.to_string() } else { format!("{value}[{k}]") }),
        });
    }
    Ok(NSidedStar {
        vertices,
        edges,
        splits,
        faces,
        spoke_normal_angle: report.spoke_normal_angle,
        tangent_normal_angle: tangent_angle,
    })
}

/// The first corner of a star's loop (side `k`'s start) standing on the air
/// side of a non-adjacent rib's blend, beyond 1e-6 of the star's size: the
/// point of the blend's surface nearest it, the surface's normal there, and
/// the side its bending (the direction it curves most, read 1e-3 of the
/// domain away) falls to, which is the material's. `(corner, side, depth)`.
fn corner_outside_a_blend(
    ribs: &[RibbonRib<'_>],
    order: &[usize],
    sides: &[NurbsCurve],
) -> Result<Option<(Vec3, usize, f64)>, KernelRefusal> {
    let n = sides.len();
    let mut corners = Vec::with_capacity(n);
    for side in sides {
        let [a, _] = side.domain().or_refuse(KernelStage::Refine, "domain")?;
        corners.push(side.evaluate(a).or_refuse(KernelStage::Refine, "evaluate")?);
    }
    let size = crate::model_scale(corners.iter().copied());
    for (k, corner) in corners.iter().enumerate() {
        for j in 0..n {
            if j == k || j == (k + n - 1) % n {
                continue;
            }
            let surface = ribs[order[j]].surface;
            let Ok(foot) = crate::project_point_to_surface(surface, *corner) else { continue };
            let normal = raw_normal(surface, foot.u, foot.v)?.normalized().or_refuse(KernelStage::Refine, "normalized")?;
            let base = surface.evaluate(foot.u, foot.v).or_refuse(KernelStage::Refine, "evaluate")?;
            let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
            let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
            let mut bend = 0.0_f64;
            for (du, dv) in [(1e-3 * (u1 - u0), 0.0), (0.0, 1e-3 * (v1 - v0))] {
                for sign in [-1.0, 1.0] {
                    let (u, v) = ((foot.u + sign * du).clamp(u0, u1), (foot.v + sign * dv).clamp(v0, v1));
                    let offset = surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?.sub(base).dot(normal);
                    if offset.abs() > bend.abs() {
                        bend = offset;
                    }
                }
            }
            let height = corner.sub(base).dot(normal);
            if bend != 0.0 && height * bend < 0.0 && height.abs() > 1e-6 * size {
                return Ok(Some((*corner, j, height.abs())));
            }
        }
    }
    Ok(None)
}
