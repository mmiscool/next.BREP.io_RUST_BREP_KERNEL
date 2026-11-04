//! Certified circular rim ARCS between a plane and a coaxial cylinder: the
//! exact revolution stations in closed form, no Newton solve. Other geometry
//! marches as before.
//!
//! Lane N row 3 (2026-10-28). The stadium's two round ends are quarter-turn
//! blends along half-turn arcs between its top plane and its semicylinder
//! walls; the exact revolution lane (`exact_revolution_rows`) already builds
//! their rows as the carrier's own circle carried along, but only after
//! marching 65 stations each (1.8 ms of the 9.6 the C6 fillet reads on the
//! fast binary). When the plane is normal to the carrier's axis every station
//! is one closed form rotated: the ball's centre circle sits at
//! `(ρ₀ + ρ_c·s_c, z₀ + ρ_p·s_p)` for the cylinder's radius `ρ₀`, the rim's
//! height `z₀` and each mate's signed radius against its own raw normal, the
//! contacts at `(ρ₀, z_c)` and `(r_c, z₀)`, the apex on the rim itself, the
//! section a quarter circle (weight `1/√2`). The stations are built at the
//! march's own parameters (`open_station_parameters`) and handed to
//! `exact_revolution_rows`, whose witness holds every window station against
//! the rows it builds at `1e-9·(1 + scale)`; a station it refuses marches.
//!
//! The certificate, read from the stored records and never a sample: the
//! plane's net is affine (unit weights, no twist past the bar) with its
//! normal along the axis; the carrier is the analytic recogniser's cylinder
//! (a full `RuledRevolution` or a partial-sweep `Revolution` of a straight
//! generatrix along the axis — the stadium's extruded half-turn walls); the
//! edge is rational quadratic spans of the carrier's parallel circle at the
//! plane's height (each span's ends at `(ρ₀, z₀)`, its apex at `ρ₀ / w` for
//! its standard-form weight `w`). ON by default since lane N's promotion
//! (2026-10-28); `BREP_ANALYTIC_NETWORK_ARC=0` is the escape hatch;
//! `force_for_test` steers a test's own thread.
use super::*;
use super::analytic_network::CertifiedCylinder;


fn enabled() -> bool {
    switch!("BREP_ANALYTIC_NETWORK_ARC", true)
}

fn finite(point: Vec3) -> bool { point.x.is_finite() && point.y.is_finite() && point.z.is_finite() }

/// The plane mate's normal when its stored net is affine: two by two,
/// degree one, unit weights, the standard knots, the twist within `bar`.
fn affine_plane_normal(surface: &NurbsSurface, bar: f64) -> Option<Vec3> {
    if surface.degree_u != 1 || surface.degree_v != 1
        || surface.knots_u.len() != 4 || surface.knots_v.len() != 4
        || surface.control_points.len() != 2 || surface.control_points.iter().any(|row| row.len() != 2)
        || surface.control_points.iter().flatten().any(|p| p.w != 1.0 || !finite(Vec3::new(p.x, p.y, p.z)))
    {
        return None;
    }
    let point = |i: usize, j: usize| { let p = surface.control_points[i][j]; Vec3::new(p.x, p.y, p.z) };
    let origin = point(0, 0);
    let u = point(1, 0).sub(origin);
    let v = point(0, 1).sub(origin);
    let twist = point(1, 1).sub(origin).sub(u).sub(v);
    if !finite(twist) || !(twist.length() <= bar) {
        return None;
    }
    u.cross(v).normalized().ok()
}

/// The carrier's radius when it is a cylinder about `(origin, axis)`: a full
/// `RuledRevolution` whose two radii agree, or a partial `Revolution` of a
/// straight generatrix parallel to the axis.
fn coaxial_cylinder_radius(surface: &NurbsSurface, origin: Vec3, axis: Vec3, bar: f64) -> Option<f64> {
    let analytic = surface.analytic()?;
    let frame = analytic.frame()?;
    if frame.axis.cross(axis).length() > 1e-12 {
        return None;
    }
    let off = frame.origin.sub(origin);
    if off.sub(axis.scale(off.dot(axis))).length() > bar {
        return None;
    }
    match analytic {
        crate::AnalyticSurface::RuledRevolution { rho0, rho1, .. } => {
            ((rho0 - rho1).abs() <= bar && *rho0 > bar).then_some(*rho0)
        }
        crate::AnalyticSurface::Revolution { generatrix, .. } => {
            let (a, b) = generatrix.straight_segment(bar)?;
            let radial = |p: Vec3| { let d = p.sub(frame.origin); d.sub(frame.axis.scale(d.dot(frame.axis))).length() };
            let (ra, rb) = (radial(a), radial(b));
            ((ra - rb).abs() <= bar && ra > bar && b.sub(a).cross(frame.axis).length() <= bar * b.sub(a).length().max(1.0)).then_some(ra)
        }
        _ => None,
    }
}

/// Is `curve` rational quadratic spans of the circle of radius `rho` at
/// height `z` about `(origin, axis)`? Each span's ends on the circle and its
/// apex at `rho / w` for the span's standard-form weight, all within `bar`.
fn curve_is_parallel_circle(curve: &NurbsCurve, origin: Vec3, axis: Vec3, rho: f64, z: f64, bar: f64) -> bool {
    let controls = &curve.control_points;
    if curve.degree != 2 || controls.len() < 3 || controls.len() % 2 == 0 {
        return false;
    }
    let cylindrical = |p: Vec3| { let d = p.sub(origin); let h = d.dot(axis); (d.sub(axis.scale(h)).length(), h) };
    for span in 0..(controls.len() - 1) / 2 {
        let (a, apex, b) = (controls[2 * span], controls[2 * span + 1], controls[2 * span + 2]);
        if !(a.w > 0.0 && apex.w > 0.0 && b.w > 0.0) {
            return false;
        }
        let w = apex.w / (a.w * b.w).sqrt();
        if !(w > 0.0 && w < 1.0) {
            return false;
        }
        let Ok(points) = [a, b, apex].iter().map(|c| c.point()).collect::<Result<Vec<_>, _>>() else { return false };
        for (point, radius) in points.iter().zip([rho, rho, rho / w]) {
            let (r, h) = cylindrical(*point);
            if (r - radius).abs() > bar || (h - z).abs() > bar {
                return false;
            }
        }
    }
    true
}

/// The certified stations of an arc rim and the exact revolution rows through
/// them; `None` wherever the certificate does not hold (the marcher then
/// solves the stations as before).
pub(in crate::blend) fn construct(
    edge: &EdgeRecord, first: &BlendMate, second: &BlendMate,
    axis: (Vec3, Vec3), constant: Option<f64>, overshoot: f64, ends: [f64; 2],
) -> Option<CertifiedCylinder> {
    if !enabled() { return None; }
    construct_impl(edge, first, second, axis, constant, overshoot, ends, false)
}

pub(in crate::blend) fn construct_chamfer(
    edge: &EdgeRecord, first: &BlendMate, second: &BlendMate,
    axis: (Vec3, Vec3), constant: Option<f64>, overshoot: f64, ends: [f64; 2],
) -> Option<CertifiedCylinder> {
    if !super::analytic_network::chamfer_enabled() { return None; }
    construct_impl(edge, first, second, axis, constant, overshoot, ends, true)
}

fn construct_impl(
    edge: &EdgeRecord, first: &BlendMate, second: &BlendMate,
    axis: (Vec3, Vec3), constant: Option<f64>, overshoot: f64, ends: [f64; 2], chamfer: bool,
) -> Option<CertifiedCylinder> {
    let radius = constant?;
    let (origin, axis) = axis;
    if !(radius.is_finite() && radius > 0.0 && overshoot.is_finite() && overshoot >= 0.0)
        || first.rho.abs() != radius || second.rho.abs() != radius
        || first.loop_index != 0 || second.loop_index != 0
        || edge.start_vertex_id == edge.end_vertex_id
        || !(edge.t0.is_finite() && edge.t1.is_finite() && edge.t1 > edge.t0)
        || !finite(origin) || !finite(axis) || (axis.length() - 1.0).abs() > 1e-12
    {
        return None;
    }
    let scale = edge.curve.control_points.iter().filter_map(|c| c.point().ok()).fold(radius, |m, p| m.max(p.length()));
    let bar = crate::KernelTolerances::for_scale(scale, 1e-7).convergence;
    if !(bar.is_finite() && radius > bar) { return None; }
    // Which mate is the plane and which the cylinder.
    let plane_first = affine_plane_normal(&first.face.surface, bar).is_some();
    let (plane, cylinder) = if plane_first { (first, second) } else { (second, first) };
    let normal = affine_plane_normal(&plane.face.surface, bar)?;
    if normal.cross(axis).length() > 1e-13 { return None; }
    let rho0 = coaxial_cylinder_radius(&cylinder.face.surface, origin, axis, bar)?;
    let plane_origin = plane.face.surface.control_points[0][0];
    let z0 = Vec3::new(plane_origin.x, plane_origin.y, plane_origin.z).sub(origin).dot(axis);
    if !curve_is_parallel_circle(&edge.curve, origin, axis, rho0, z0, bar) { return None; }
    // The ball's centre circle, from each mate's raw normal: the cylinder's
    // is radial, the plane's axial; the sign is read once.
    let radial_of = |p: Vec3| { let d = p.sub(origin); d.sub(axis.scale(d.dot(axis))) };
    let cylinder_sign = {
        let ([u0, u1], [v0, v1]) = (cylinder.face.surface.domain_u().ok()?, cylinder.face.surface.domain_v().ok()?);
        let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
        let raw = raw_normal(&cylinder.face.surface, u, v).ok()?;
        let at = cylinder.face.surface.evaluate(u, v).ok()?;
        let outward = radial_of(at).normalized().ok()?;
        let along = raw.dot(outward);
        if (along.abs() - 1.0).abs() > 1e-12 { return None; }
        along.signum()
    };
    let plane_sign = {
        let raw = raw_normal(&plane.face.surface, 0.5, 0.5).ok()?;
        let along = raw.dot(axis);
        if (along.abs() - 1.0).abs() > 1e-12 { return None; }
        along.signum()
    };
    let rc = rho0 + cylinder.rho * cylinder_sign;
    let zc = z0 + plane.rho * plane_sign;
    if !(rc > bar) { return None; }
    // The march's own station parameters along the edge, the window's
    // stations at the edge's azimuths, the overshoot's extrapolated at the
    // window's mean spacing (`exact_revolution_rows` replaces them with the
    // rows' own sections).
    let (station_ts, vertex_indices) = crate::blend::edge::open_station_parameters(edge, overshoot, ends, SEED_INTERVALS, [false, false]);
    if station_ts.iter().any(|t| !t.is_finite()) || station_ts.windows(2).any(|p| !(p[1] > p[0])) || vertex_indices[0] >= vertex_indices[1] {
        return None;
    }
    let (lo, hi) = (vertex_indices[0], vertex_indices[1]);
    let x_axis = radial_of(edge.curve.evaluate(edge.t0).ok()?).normalized().ok()?;
    let y_axis = axis.cross(x_axis);
    let azimuth = |p: Vec3| { let r = radial_of(p); r.dot(y_axis).atan2(r.dot(x_axis)) };
    let mut thetas = vec![0.0f64; station_ts.len()];
    let mut previous = 0.0f64;
    for index in lo..=hi {
        let t = station_ts[index].clamp(edge.t0, edge.t1);
        let mut theta = azimuth(edge.curve.evaluate(t).ok()?);
        if index > lo {
            while theta - previous > std::f64::consts::PI { theta -= std::f64::consts::TAU; }
            while previous - theta > std::f64::consts::PI { theta += std::f64::consts::TAU; }
        }
        thetas[index] = theta;
        previous = theta;
    }
    let step = (thetas[hi] - thetas[lo]) / (hi - lo) as f64;
    if !(step.abs() > 0.0) || (lo..hi).any(|i| (thetas[i + 1] - thetas[i]) * step <= 0.0) { return None; }
    for index in 0..lo { thetas[index] = thetas[lo] - step * (lo - index) as f64; }
    for index in hi + 1..station_ts.len() { thetas[index] = thetas[hi] + step * (index - hi) as f64; }
    let at = |theta: f64, rho: f64, z: f64| origin.add(x_axis.scale(rho * theta.cos())).add(y_axis.scale(rho * theta.sin())).add(axis.scale(z));
    let mut stations = Vec::with_capacity(station_ts.len());
    for &theta in &thetas {
        let on_cylinder = at(theta, rho0, zc);
        let on_plane = at(theta, rc, z0);
        let uv = |surface: &NurbsSurface, point: Vec3| -> Option<[f64; 2]> {
            let foot = crate::project_point_to_surface(surface, point).ok()?;
            Some([foot.u, foot.v])
        };
        let (p1, p2, uv1, uv2) = if plane_first {
            (on_plane, on_cylinder, uv(&first.face.surface, on_plane)?, uv(&second.face.surface, on_cylinder)?)
        } else {
            (on_cylinder, on_plane, uv(&first.face.surface, on_cylinder)?, uv(&second.face.surface, on_plane)?)
        };
        stations.push(Station { uv1, uv2, p1, p2, center: at(theta, rc, zc), weight: std::f64::consts::FRAC_1_SQRT_2, apex: at(theta, rho0, z0) });
    }
    let built = crate::blend::edge::exact_revolution_rows(&stations, vertex_indices, chamfer, (origin, axis), radius, first, second);
    let crate::blend::edge::ExactRevolution { rows, parameters, stations } = match built {
        Ok(exact) => exact,
        Err(_) => {
            crate::blend::record_count("count.network.analytic_arc_declined", 1);
            return None;
        }
    };
    crate::blend::record_count(if chamfer { "count.network.analytic_chamfer_arc" } else { "count.network.analytic_arc" }, 1);
    Some(CertifiedCylinder { rows, stations, parameters, vertex_indices, station_ts })
}

