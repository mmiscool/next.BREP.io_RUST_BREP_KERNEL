//! Certified full circular cap rims. Unsupported charts retain the marcher.
use super::*;


fn enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("BREP_ANALYTIC_CAP_RIM").ok().as_deref() != Some("0"))
}

/// Positive rational bases form a partition of unity. For identical bases,
/// this bounds the entire rational curve/patch difference, including between
/// samples, from the homogeneous control differences and denominator floor.
fn net_bound(a: &[Vec4], b: &[Vec4]) -> Option<f64> {
    if a.len() != b.len() || a.is_empty() { return None; }
    let origin = Vec3::new(a[0].x / a[0].w, a[0].y / a[0].w, a[0].z / a[0].w);
    if [origin.x, origin.y, origin.z].iter().any(|v| !v.is_finite()) { return None; }
    let mut floor = f64::INFINITY;
    let (mut numerator, mut denominator, mut reach) = (0.0_f64, 0.0_f64, 0.0_f64);
    for (a, b) in a.iter().zip(b) {
        if !(a.w > 0.0 && b.w > 0.0) || [a.x, a.y, a.z, a.w, b.x, b.y, b.z, b.w].iter().any(|v| !v.is_finite()) { return None; }
        floor = floor.min(a.w);
        let ah = Vec3::new(a.x, a.y, a.z).sub(origin.scale(a.w));
        let bh = Vec3::new(b.x, b.y, b.z).sub(origin.scale(b.w));
        if [ah.x, ah.y, ah.z, bh.x, bh.y, bh.z].iter().any(|v| !v.is_finite()) { return None; }
        numerator = numerator.max(ah.sub(bh).length());
        denominator = denominator.max((a.w - b.w).abs());
        reach = reach.max(bh.length() / b.w);
    }
    let bound = (numerator + reach * denominator) / floor;
    bound.is_finite().then_some(bound)
}

fn curve_matches(a: &NurbsCurve, b: &NurbsCurve, bar: f64) -> bool {
    a.degree == b.degree && a.knots == b.knots
        && net_bound(&a.control_points, &b.control_points).is_some_and(|gap| gap <= bar)
}

fn surface_matches(a: &NurbsSurface, b: &NurbsSurface, bar: f64) -> bool {
    a.degree_u == b.degree_u && a.degree_v == b.degree_v
        && a.knots_u == b.knots_u && a.knots_v == b.knots_v
        && a.control_points.len() == b.control_points.len()
        && a.control_points.iter().zip(&b.control_points).all(|(a, b)| a.len() == b.len())
        && net_bound(&a.control_points.iter().flatten().copied().collect::<Vec<_>>(),
                     &b.control_points.iter().flatten().copied().collect::<Vec<_>>()).is_some_and(|gap| gap <= bar)
}

/// Initial certificate: a full convex cylinder cap, an affine plane chart,
/// and a ring-torus (or conical chamfer) section wholly inside the carriers.
/// Neither analytic recognition alone nor sampled agreement is the certificate:
/// the stored cylinder net, plane net and edge are bounded against the exact
/// construction. All failures return to the original march and row builder.
pub(in crate::blend) fn construct(
    edge: &EdgeRecord, first: &BlendMate, second: &BlendMate,
    constant: Option<f64>, chamfer: bool, anchor: Option<f64>,
) -> Option<(Vec<Station>, FittedRows)> {
    if !enabled() { return None; }
    let radius = constant?;
    if !(radius.is_finite() && radius > 0.0 && first.rho.abs() == radius && second.rho.abs() == radius)
        || anchor != Some(0.0) || first.loop_index != 0 || second.loop_index != 0
        || second.face.loops.len() != 1 || second.face.loops[0].coedges.len() != 1
        || edge.start_vertex_id != edge.end_vertex_id || edge.t0 != 0.0 || edge.t1 != 1.0
    { return None; }
    let cylinder = &first.face.surface;
    let plane = &second.face.surface;
    let crate::AnalyticSurface::RuledRevolution { frame, rho0, rho1, height } = cylinder.analytic()? else { return None; };
    let crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. } = plane.analytic()? else { return None; };
    let bar = crate::KernelTolerances::for_scale(rho0.abs().max(height.abs()), 1e-7).convergence;
    if (rho0 - rho1).abs() > bar || !(*height > 0.0 && *rho0 > radius * 2.0)
        || cylinder.domain_u().ok()? != [0.0, 1.0] || cylinder.domain_v().ok()? != [0.0, 1.0]
        || plane.degree_u != 1 || plane.degree_v != 1
        || plane.control_points.len() != 2 || plane.control_points.iter().any(|r| r.len() != 2)
    { return None; }
    let axis = frame.axis;
    let radial = frame.x_axis;
    let normal = u_dir.cross(*v_dir).normalized().ok()?;
    if normal.cross(axis).length() > 1e-13 { return None; }
    let z0 = origin.sub(frame.origin).dot(axis);
    if z0.abs().min((z0 - height).abs()) > bar { return None; }
    let raw = raw_normal(cylinder, 0.0, 0.5).ok()?;
    if raw.cross(radial).length() > 1e-13 { return None; }
    let rc = rho0 + first.rho * raw.dot(radial);
    let zc = z0 + second.rho * normal.dot(axis);
    if !(rc > radius && rc < *rho0 && zc > 0.0 && zc < *height) { return None; }
    let point = |r: f64, z: f64| frame.origin.add(radial.scale(r)).add(axis.scale(z));
    let cylinder_line = crate::make_line(point(*rho0, 0.0), point(*rho0, *height)).ok()?;
    let rebuilt = crate::make_revolution(frame.origin, axis, &cylinder_line, std::f64::consts::TAU).ok()?;
    if !surface_matches(cylinder, &rebuilt, bar) { return None; }
    let [u0, u1] = plane.domain_u().ok()?;
    let [v0, v1] = plane.domain_v().ok()?;
    // Affine bilinear net with unit weights: its entire chart is certified.
    for (i, row) in plane.control_points.iter().enumerate() {
        for (j, cp) in row.iter().enumerate() {
            if cp.w != 1.0 { return None; }
            let p = origin.add(u_dir.scale([u0, u1][i])).add(v_dir.scale([v0, v1][j]));
            if p.sub(Vec3::new(cp.x, cp.y, cp.z)).length() > bar { return None; }
        }
    }
    let rim = cylinder.iso_curve_v(z0 / height).ok()?;
    // Reversed/shifted parameterizations are deliberately left to the marcher.
    if !curve_matches(&edge.curve, &rim, bar) { return None; }
    let (p1, p2, apex) = (point(*rho0, zc), point(rc, z0), point(*rho0, z0));
    let weight = std::f64::consts::FRAC_1_SQRT_2;
    let generatrix = if chamfer { crate::make_line(p1, p2).ok()? } else {
        NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![
            Vec4::from_point(p1, 1.0), Vec4::from_point(apex, weight), Vec4::from_point(p2, 1.0),
        ]).ok()?
    };
    let surface = crate::make_revolution(frame.origin, axis, &generatrix, std::f64::consts::TAU).ok()?;
    let cr = surface.iso_curve_v(0.0).ok()?;
    let cs = surface.iso_curve_v(1.0).ok()?;
    if !curve_matches(&cr, &cylinder.iso_curve_v(zc / height).ok()?, bar) { return None; }
    let cr_pcurve = crate::make_line(Vec3::new(0.0, zc / height, 0.0), Vec3::new(1.0, zc / height, 0.0)).ok()?;
    let (a, b, c) = (u_dir.dot(*u_dir), u_dir.dot(*v_dir), v_dir.dot(*v_dir));
    let det = a * c - b * b;
    if !(det > 0.0 && det.is_finite()) { return None; }
    let mut controls = Vec::with_capacity(cs.control_points.len());
    let mut back = Vec::with_capacity(cs.control_points.len());
    for cp in &cs.control_points {
        let p = Vec3::new(cp.x / cp.w, cp.y / cp.w, cp.z / cp.w).sub(*origin);
        let (x, y) = (p.dot(*u_dir), p.dot(*v_dir));
        let (u, v) = ((c * x - b * y) / det, (a * y - b * x) / det);
        controls.push(Vec4::from_point(Vec3::new(u, v, 0.0), cp.w));
        back.push(Vec4::from_point(origin.add(u_dir.scale(u)).add(v_dir.scale(v)), cp.w));
    }
    if net_bound(&cs.control_points, &back)? > bar { return None; }
    let cs_pcurve = NurbsCurve::new(cs.degree, cs.knots.clone(), controls).ok()?;
    // These are support/trim diagnostic stations, not fitted or solved samples.
    // The existing exit, overrun, topology, interference and soundness checks
    // still consume them and the exact rows.
    let mut stations = Vec::with_capacity(SEED_INTERVALS + 1);
    for k in 0..=SEED_INTERVALS {
        let t = k as f64 / SEED_INTERVALS as f64;
        let p1 = cr.evaluate(t).ok()?;
        let p2 = cs.evaluate(t).ok()?;
        let uv2 = cs_pcurve.evaluate(t).ok()?;
        let direction = p1.sub(frame.origin).sub(axis.scale(zc)).scale(1.0 / rho0);
        stations.push(Station {
            uv1: [t, zc / height], uv2: [uv2.x, uv2.y], p1, p2,
            center: frame.origin.add(direction.scale(rc)).add(axis.scale(zc)), weight,
            apex: frame.origin.add(direction.scale(*rho0)).add(axis.scale(z0)),
        });
    }
    crate::blend::record_count("count.closed.analytic_cap", 1);
    Some((stations, FittedRows {
        surface, cr, cs, cr_pcurve, cs_pcurve, u_domain: [0.0, 1.0],
        center: None, exact_extrusion: false, vertex_stations: None, revolution_axis: None,
    }))
}
