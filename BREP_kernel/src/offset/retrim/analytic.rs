//! Reconstruct analytic shell boundaries from the carriers, rather than from
//! section fits whose endpoints were welded after their trims were constructed.
use crate::{AnalyticSurface, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, Vec3};
use std::collections::{BTreeMap, BTreeSet};
#[path = "displacement.rs"]
mod displacement;

#[derive(Clone)]
enum Carrier {
    Plane(Vec3, Vec3),
    Sphere(Vec3, f64),
    Cylinder(crate::RevolutionFrame, f64),
}
impl Carrier {
    fn read(a: &AnalyticSurface) -> Option<Self> {
        match a {
            AnalyticSurface::Plane {
                origin,
                u_dir,
                v_dir,
                ..
            } => Some(Self::Plane(*origin, u_dir.cross(*v_dir).normalized().ok()?)),
            AnalyticSurface::Sphere { frame, radius } => Some(Self::Sphere(frame.origin, *radius)),
            AnalyticSurface::RuledRevolution {
                frame, rho0, rho1, ..
            } if (rho0 - rho1).abs() < 1e-12 * rho0.abs() => {
                Some(Self::Cylinder(frame.clone(), *rho0))
            }
            _ => None,
        }
    }
    fn constraint(&self, p: Vec3) -> Result<(f64, Vec3), String> {
        match self {
            Self::Plane(o, n) => Ok((p.sub(*o).dot(*n), *n)),
            Self::Sphere(o, r) => {
                let d = p.sub(*o);
                Ok((d.length() - r, d.normalized()?))
            }
            Self::Cylinder(f, r) => {
                let d = p.sub(f.origin);
                let radial = d.sub(f.axis.scale(d.dot(f.axis)));
                Ok((radial.length() - r, radial.normalized()?))
            }
        }
    }
}

fn junction(
    mut p: Vec3,
    carriers: &[&Carrier],
    accuracy: f64,
    movement: f64,
) -> Result<Vec3, String> {
    let anchor = p;
    for _ in 0..24 {
        let constraints = carriers
            .iter()
            .map(|c| c.constraint(p))
            .collect::<Result<Vec<_>, _>>()?;
        if constraints.iter().all(|(d, _)| d.abs() <= accuracy) {
            return Ok(p);
        }
        let mut rows = Vec::<Vec3>::new();
        let mut rhs = Vec::new();
        for (d, n) in &constraints {
            let mut perpendicular = *n;
            for row in &rows {
                perpendicular = perpendicular.sub(row.scale(perpendicular.dot(*row)));
            }
            if perpendicular.length() > 1e-7 {
                // Gram-Schmidt the equation as well as its normal.
                let mut value = -*d;
                for (row, b) in rows.iter().zip(&rhs) {
                    value -= n.dot(*row) * b;
                }
                let norm = perpendicular.length();
                rows.push(perpendicular.scale(1.0 / norm));
                rhs.push(value / norm);
            }
        }
        let step = rows
            .iter()
            .zip(&rhs)
            .fold(Vec3::default(), |v, (n, d)| v.add(n.scale(*d)));
        p = p.add(step);
        if p.sub(anchor).length() > movement {
            return Err("analytic junction exceeds assembly displacement".into());
        }
    }
    Err("analytic junction does not lie on all incident carriers".into())
}

fn angle(p: Vec3, origin: Vec3, x: Vec3, y: Vec3) -> f64 {
    let d = p.sub(origin);
    d.dot(y).atan2(d.dot(x))
}
fn sweep(start: f64, middle: f64, end: f64, closed: bool) -> f64 {
    let tau = std::f64::consts::TAU;
    let positive = (end - start).rem_euclid(tau);
    if closed {
        if (middle - start).rem_euclid(tau) < std::f64::consts::PI {
            tau
        } else {
            -tau
        }
    } else if (middle - start).rem_euclid(tau) <= positive {
        positive
    } else {
        positive - tau
    }
}

/// Reject unsupported/tangent branches without changing the caller's solid.
fn preflight(
    a: &Carrier,
    b: &Carrier,
    from: Vec3,
    to: Vec3,
    through: Vec3,
    closed: bool,
    movement: f64,
) -> bool {
    match (a, b) {
        (Carrier::Plane(_, n), Carrier::Plane(_, m)) => n.cross(*m).length() > 64.0 * f64::EPSILON,
        (Carrier::Sphere(c, r), Carrier::Plane(o, n))
        | (Carrier::Plane(o, n), Carrier::Sphere(c, r)) => {
            r * r - o.sub(*c).dot(*n).powi(2) > movement * movement
        }
        (Carrier::Cylinder(f, r), Carrier::Plane(o, n))
        | (Carrier::Plane(o, n), Carrier::Cylinder(f, r)) => {
            if n.dot(f.axis).abs() >= 64.0 * f64::EPSILON {
                return true;
            }
            let height = o.sub(f.origin).dot(*n);
            let radial = |p: Vec3| {
                let d = p.sub(f.origin);
                d.sub(f.axis.scale(d.dot(f.axis)))
            };
            // Parallel-plane sections have two disconnected generators. They
            // must be distinct and all witnesses must identify the same one.
            r * r - height * height > movement * movement
                && !closed
                && radial(from).sub(radial(to)).length() <= movement
                && radial(from).sub(radial(through)).length() <= movement
        }
        (Carrier::Cylinder(f, r), Carrier::Sphere(c, radius))
        | (Carrier::Sphere(c, radius), Carrier::Cylinder(f, r)) => {
            let start = angle(from, f.origin, f.x_axis, f.y_axis);
            let travel = sweep(
                start,
                angle(through, f.origin, f.x_axis, f.y_axis),
                angle(to, f.origin, f.x_axis, f.y_axis),
                closed,
            );
            let d = c.sub(f.origin);
            let x = d.dot(f.x_axis);
            let y = d.dot(f.y_axis);
            let disc = |t: f64| {
                radius * radius - r * r - x * x - y * y + 2.0 * r * (x * t.cos() + y * t.sin())
            };
            let lo = start.min(start + travel);
            let hi = start.max(start + travel);
            let mut minimum = disc(lo).min(disc(hi));
            let phase = y.atan2(x) + std::f64::consts::PI;
            for k in -3..=3 {
                let t = phase + k as f64 * std::f64::consts::TAU;
                if t >= lo && t <= hi {
                    minimum = minimum.min(disc(t));
                }
            }
            // Roots remain separated by more than twice the allowed movement;
            // endpoint and interior witnesses must select the same axial root.
            let sign = through.sub(*c).dot(f.axis).signum();
            minimum > movement * movement
                && sign != 0.0
                && [from, to]
                    .iter()
                    .all(|p| p.sub(*c).dot(f.axis) * sign > movement)
        }
        _ => false,
    }
}

fn section(
    a: &Carrier,
    b: &Carrier,
    from: Vec3,
    to: Vec3,
    through: Vec3,
    closed: bool,
    accuracy: f64,
) -> Result<NurbsCurve, String> {
    match (a, b) {
        (Carrier::Plane(..), Carrier::Plane(..)) => crate::make_line(from, to),
        (Carrier::Sphere(c, r), Carrier::Plane(o, n))
        | (Carrier::Plane(o, n), Carrier::Sphere(c, r)) => {
            let z = o.sub(*c).dot(*n);
            let center = c.add(n.scale(z));
            let radius = (r * r - z * z).sqrt();
            let radial = from.sub(center);
            let x = radial.sub(n.scale(radial.dot(*n))).normalized()?;
            let y = n.cross(x);
            let end = angle(to, center, x, y);
            let mid = angle(through, center, x, y);
            let travel = sweep(0.0, mid, end, closed);
            crate::make_arc(
                center,
                x,
                y.scale(travel.signum()),
                radius,
                0.0,
                travel.abs(),
            )
        }
        (Carrier::Cylinder(f, r), other) | (other, Carrier::Cylinder(f, r)) => {
            if let Carrier::Plane(_, n) = other {
                if n.dot(f.axis).abs() < 64.0 * f64::EPSILON {
                    return crate::make_line(from, to);
                }
            }
            // The perpendicular plane section is a rational circle, not a
            // sampled fit. Preserve this conic for downstream intersections.
            if let Carrier::Plane(o, n) = other {
                if n.cross(f.axis).length() < 64.0 * f64::EPSILON {
                    let center = f
                        .origin
                        .add(f.axis.scale(o.sub(f.origin).dot(*n) / f.axis.dot(*n)));
                    let x = from.sub(center).normalized()?;
                    let y = f.axis.cross(x);
                    let travel = sweep(
                        0.0,
                        angle(through, center, x, y),
                        angle(to, center, x, y),
                        closed,
                    );
                    return crate::make_arc(
                        center,
                        x,
                        y.scale(travel.signum()),
                        *r,
                        0.0,
                        travel.abs(),
                    );
                }
            }
            let start = angle(from, f.origin, f.x_axis, f.y_axis);
            let travel = sweep(
                start,
                angle(through, f.origin, f.x_axis, f.y_axis),
                angle(to, f.origin, f.x_axis, f.y_axis),
                closed,
            );
            let branch = if let Carrier::Sphere(c, _) = other {
                if through.sub(*c).dot(f.axis) >= 0.0 {
                    1.0
                } else {
                    -1.0
                }
            } else {
                1.0
            };
            let station = |t: f64| -> Result<Vec3, String> {
                let theta = start + travel * t;
                let base = f
                    .origin
                    .add(f.x_axis.scale(r * theta.cos()))
                    .add(f.y_axis.scale(r * theta.sin()));
                let z = match other {
                    Carrier::Plane(o, n) => o.sub(base).dot(*n) / f.axis.dot(*n),
                    Carrier::Sphere(c, radius) => {
                        let d = base.sub(*c);
                        let axial = d.dot(f.axis);
                        let disc = radius * radius - d.sub(f.axis.scale(axial)).length_squared();
                        if disc < 0.0 {
                            return Err("sphere/cylinder branch crosses a turning point".into());
                        }
                        -axial + branch * disc.sqrt()
                    }
                    _ => return Err("unsupported analytic section".into()),
                };
                Ok(base.add(f.axis.scale(z)))
            };
            let mut parameters = (0..=32).map(|i| i as f64 / 32.0).collect::<Vec<_>>();
            for _ in 0..18 {
                let mut points = parameters
                    .iter()
                    .map(|t| station(*t))
                    .collect::<Result<Vec<_>, _>>()?;
                points[0] = from;
                *points.last_mut().unwrap() = to;
                let curve = crate::interpolate_curve(&points, 3, &parameters)?;
                let mut inserts = Vec::new();
                for pair in parameters.windows(2) {
                    for f in [0.2113248654, 0.5, 0.7886751346] {
                        let t = pair[0] + (pair[1] - pair[0]) * f;
                        if curve.evaluate(t)?.sub(station(t)?).length() > accuracy {
                            inserts.push(t);
                        }
                    }
                }
                if inserts.is_empty() {
                    return Ok(curve);
                }
                if parameters.len() + inserts.len() > 8192 {
                    break;
                }
                parameters.extend(inserts);
                parameters.sort_by(f64::total_cmp);
                if parameters.len() > 8192 {
                    break;
                }
            }
            Err("analytic section exhausted construction budget".into())
        }
        _ => Err("unsupported analytic carrier pair".into()),
    }
}

pub(crate) fn reconstruct_analytic_shell(solid: &mut BrepSolid) -> Result<(), KernelRefusal> {
    let faces = solid
        .shells
        .iter()
        .flat_map(|s| &s.faces)
        .collect::<Vec<_>>();
    let Some(carriers) = faces
        .iter()
        .map(|f| f.surface.analytic().and_then(Carrier::read))
        .collect::<Option<Vec<_>>>()
    else {
        return Ok(());
    };
    if !carriers.iter().any(|c| matches!(c, Carrier::Sphere(..))) {
        return Ok(());
    }
    let mut incident = BTreeMap::<u64, BTreeSet<usize>>::new();
    for (i, f) in faces.iter().enumerate() {
        for co in f.loops.iter().flat_map(|l| &l.coedges) {
            incident.entry(co.edge_id).or_default().insert(i);
        }
    }
    // Only enter when every ordinary edge has a supported analytic pair.
    // Other sphere shells retain the general retrim path.
    for edge in &solid.edges {
        if edge.degenerate {
            continue;
        }
        if let Some(fs) = incident.get(&edge.id) {
            if fs.len() != 2 {
                continue;
            }
            let mut it = fs.iter();
            let (a, b) = (
                &carriers[*it.next().unwrap()],
                &carriers[*it.next().unwrap()],
            );
            if !matches!(
                (a, b),
                (Carrier::Plane(..), Carrier::Plane(..))
                    | (Carrier::Plane(..), Carrier::Sphere(..))
                    | (Carrier::Sphere(..), Carrier::Plane(..))
                    | (Carrier::Plane(..), Carrier::Cylinder(..))
                    | (Carrier::Cylinder(..), Carrier::Plane(..))
                    | (Carrier::Sphere(..), Carrier::Cylinder(..))
                    | (Carrier::Cylinder(..), Carrier::Sphere(..))
            ) {
                return Ok(());
            }
        }
    }
    let mut vertices = BTreeMap::<u64, BTreeSet<usize>>::new();
    for e in &solid.edges {
        if let Some(fs) = incident.get(&e.id) {
            for v in [e.start_vertex_id, e.end_vertex_id] {
                vertices.entry(v).or_default().extend(fs);
            }
        }
    }
    let scale = crate::solid_scale(solid);
    // Relative analytic boundary contract: 1e-12 of model extent. Allocate
    // one fifth to carrier sections/junctions and four fifths to their trim
    // images. Their triangle bound is the boundary contract; material probes
    // and strict volume oracles independently validate the resulting region.
    // This budget is not inferred from a measured volume discrepancy.
    let boundary_contract = scale * 1e-12;
    let accuracy = boundary_contract / 5.0;
    let movement = 2e-3_f64.max(scale * 5e-5);
    let fail = |s: String| KernelRefusal::internal(KernelStage::Refine, "analytic_shell_retrim", s);
    let mut points = BTreeMap::new();
    for v in &solid.vertices {
        if let Some(fs) = vertices.get(&v.id) {
            let cs = fs.iter().map(|i| &carriers[*i]).collect::<Vec<_>>();
            let point = junction(v.point, &cs, accuracy, movement).map_err(&fail)?;
            if point.sub(v.point).length() > movement {
                return Err(fail(
                    "analytic junction exceeds assembly displacement".into(),
                ));
            }
            points.insert(v.id, point);
        } else {
            points.insert(v.id, v.point);
        }
    }
    // Preflight every branch before constructing any replacement geometry.
    for e in &solid.edges {
        if e.degenerate {
            continue;
        }
        let Some(fs) = incident.get(&e.id) else {
            return Ok(());
        };
        if fs.len() != 2 {
            continue;
        }
        let mut it = fs.iter();
        let a = &carriers[*it.next().unwrap()];
        let b = &carriers[*it.next().unwrap()];
        let closed = e.start_vertex_id == e.end_vertex_id;
        let through = e
            .curve
            .evaluate(e.t0 + (e.t1 - e.t0) * if closed { 0.25 } else { 0.5 })
            .map_err(&fail)?;
        if !preflight(
            a,
            b,
            points[&e.start_vertex_id],
            points[&e.end_vertex_id],
            through,
            closed,
            movement,
        ) {
            return Ok(());
        }
    }
    let mut rebuilt = solid.clone();
    for v in &mut rebuilt.vertices {
        v.point = points[&v.id];
    }
    for e in &mut rebuilt.edges {
        if e.degenerate {
            continue;
        }
        let Some(fs) = incident.get(&e.id) else {
            continue;
        };
        if fs.len() != 2 {
            continue;
        }
        let mut it = fs.iter();
        let a = &carriers[*it.next().unwrap()];
        let b = &carriers[*it.next().unwrap()];
        // The antipodal midpoint of a closed rim cannot select orientation.
        let fraction = if e.start_vertex_id == e.end_vertex_id {
            0.25
        } else {
            0.5
        };
        let through = e
            .curve
            .evaluate(e.t0 + (e.t1 - e.t0) * fraction)
            .map_err(&fail)?;
        let replacement = section(
            a,
            b,
            points[&e.start_vertex_id],
            points[&e.end_vertex_id],
            through,
            e.start_vertex_id == e.end_vertex_id,
            accuracy,
        )
        .map_err(&fail)?;
        displacement::certify(&e.curve, e.t0, e.t1, &replacement, movement).map_err(&fail)?;
        e.curve = replacement;
        [e.t0, e.t1] = e.curve.domain().map_err(&fail)?;
    }
    let edges = rebuilt
        .edges
        .iter()
        .map(|e| (e.id, e))
        .collect::<BTreeMap<_, _>>();
    for face in rebuilt.shells.iter_mut().flat_map(|s| &mut s.faces) {
        for co in face.loops.iter_mut().flat_map(|l| &mut l.coedges) {
            let e = edges[&co.edge_id];
            if e.degenerate || incident[&e.id].len() != 2 {
                continue;
            }
            let station = |f: f64| {
                e.curve.evaluate(if co.forward {
                    e.t0 + (e.t1 - e.t0) * f
                } else {
                    e.t1 + (e.t0 - e.t1) * f
                })
            };
            co.pcurve = if face.surface.is_affine().map_err(&fail)? {
                crate::build_pcurve_on_surface_range(
                    &face.surface,
                    &e.curve,
                    e.t0,
                    e.t1,
                    co.forward,
                    accuracy * 4.0,
                )
                .map_err(&fail)?
            } else {
                crate::pcurve::analytic_station_pcurve(&face.surface, &station, accuracy * 4.0)
                    .map_err(|error| fail(format!("face {} edge {}: {error}", face.id, e.id)))?
            };
        }
    }
    *solid = rebuilt;
    Ok(())
}

