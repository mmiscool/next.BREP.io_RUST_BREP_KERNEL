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
