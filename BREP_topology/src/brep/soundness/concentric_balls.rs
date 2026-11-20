//! Concentric balls certificate (`BREP_SOUNDNESS_CONCENTRIC_BALLS`, default
//! on, `=0` restores the full scan; `BREP_SOUNDNESS_CONCENTRIC_BALLS_CHECK=1` runs the full scan as well,
//! returns ITS report, and says so on stderr when the scan flags a body the
//! certificate passed).
//!
//! A body of one shell whose faces are native balls (every stored bit what
//! `make_sphere_surface` builds, as the sphere certificate requires) about ONE
//! centre with distinct radii, and at most one plane, holds no crossing and no
//! fold, proved pair by pair on the built carriers:
//! - two concentric spheres of different radii have no point in common;
//! - a sphere and a plane meet in one circle (or not at all, or in a point,
//!   which declines); where that whole circle is a closed edge both faces
//!   bound, no other point of the two carriers is common to them; the ball
//!   face's trimmed region lies on one side of the plane, read exactly from its
//!   pcurves' control hull against the circle's chart latitude (a trim is never
//!   sampled), so its interior cannot reach through the plane;
//! - a native ball's chart is injective away from the seam and poles the full
//!   scan identifies too, with every trim control inside its domain; a plane's
//!   affine chart is injective.
//!
//! A ball shelled to a wall under a plane (brep-bench shell/sphere-open-at-a-cap,
//! shell/sphere-pinhole-opening) is the outer cap, the inner cap and the plane's
//! annulus, and a ball split by a plane is a cap and a disk: the scan
//! tessellated each to thousands of triangles, 15 to 50 ms a body. Anything
//! else declines to the scan.
use super::*;

macro_rules! decline {
    () => {{
        if std::env::var_os("BREP_DEBUG_CONCENTRIC_BALLS").is_some() {
            eprintln!("soundness.concentric_balls decline line={}", line!());
        }
        return false;
    }};
}

/// Recognition slack relative to the body's size.
const DELTA: f64 = 1e-12;

pub(super) fn enabled() -> bool {
    crate::switch!("BREP_SOUNDNESS_CONCENTRIC_BALLS", true)
}

pub(super) fn check_mode() -> bool {
    crate::switch!("BREP_SOUNDNESS_CONCENTRIC_BALLS_CHECK", false)
}


/// A ball net that is, to `DELTA` of its own size, the net
/// `make_sphere_surface` builds, carried by a rigid motion, possibly mirrored
/// (a native ball scaled about its centre and turned inside out, as a shell's
/// inner wall is): the same layout and knot bits, both pole rows one point
/// exactly, and every control and weight the canonical unit net's mapped
/// through the frame the net's own poles and equator give. Its chart is then
/// the native one's up to the slack, and injective as that one is.
fn rebuilt_ball(s: &NurbsSurface) -> Option<(Vec3, f64)> {
    if s.degree_u != 2 || s.degree_v != 2 || s.control_points.len() != 9 || s.control_points.iter().any(|row| row.len() != 5) {
        return None;
    }
    let canonical = crate::make_sphere_surface(Vec3::new(0.0, 0.0, 0.0), 1.0, Vec3::new(0.0, 0.0, 1.0)).ok()?;
    if s.knots_u != canonical.knots_u || s.knots_v != canonical.knots_v {
        return None;
    }
    for pole in [0, 4] {
        let first = s.control_points[0][pole].point().ok()?;
        if s.control_points.iter().any(|row| row[pole].point().map_or(true, |p| p.x.to_bits() != first.x.to_bits() || p.y.to_bits() != first.y.to_bits() || p.z.to_bits() != first.z.to_bits())) {
            return None;
        }
    }
    // A frame from the poles and two equator controls a quarter turn apart.
    let frame = |net: &NurbsSurface| -> Option<(Vec3, f64, [Vec3; 3])> {
        let (south, north) = (net.control_points[0][0].point().ok()?, net.control_points[0][4].point().ok()?);
        let centre = south.add(north).scale(0.5);
        let radius = north.sub(south).length() * 0.5;
        if !(radius > 0.0) {
            return None;
        }
        let up = north.sub(south).scale(0.5 / radius);
        let first = net.control_points[0][2].point().ok()?.sub(centre).scale(1.0 / radius);
        let second = net.control_points[2][2].point().ok()?.sub(centre).scale(1.0 / radius);
        Some((centre, radius, [first, second, up]))
    };
    let (centre, radius, [e1, e2, e3]) = frame(s)?;
    let (_, _, [c1, c2, c3]) = frame(&canonical)?;
    // The slack is the ball's own: a ball far from the origin, whose
    // coordinates' rounding alone would need a slack of its own size, declines.
    if centre.length() > 1e3 * radius {
        return None;
    }
    let slack = DELTA * (centre.length() + radius);
    // Orthonormal, to the slack.
    for (a, b) in [(e1, e2), (e1, e3), (e2, e3)] {
        if a.dot(b).abs() > DELTA {
            return None;
        }
    }
    for e in [e1, e2, e3] {
        if (e.length() - 1.0).abs() > DELTA {
            return None;
        }
    }
    for (row, fresh) in s.control_points.iter().zip(&canonical.control_points) {
        for (p, q) in row.iter().zip(fresh) {
            if (p.w - q.w).abs() > DELTA * q.w.abs() {
                return None;
            }
            let local = q.point().ok()?;
            let (x, y, z) = (local.dot(c1), local.dot(c2), local.dot(c3));
            let mapped = centre.add(e1.scale(radius * x)).add(e2.scale(radius * y)).add(e3.scale(radius * z));
            if p.point().ok()?.sub(mapped).length() > slack {
                return None;
            }
        }
    }
    Some((centre, radius))
}

/// Whether the ball face's whole trimmed region (radius `radius` about
/// `centre`) lies on one side of the plane through `origin` with unit
/// `normal`, read exactly. The plane must be square to the chart's axis (pole
/// to pole) to within a sine of 1e-12, so over the ball it stays within
/// `slack` of one height along the axis: the height of the plane's foot of the
/// centre, which no choice of the plane's chart origin moves. The height rises
/// monotonically with the chart's v, so a height is one parameter latitude;
/// the region lies in the convex hull of its pcurves' controls (positive
/// weights), so every control at or below the latitude of `height + slack`
/// (or every one at or above that of `height - slack`) puts the whole region
/// on that side, short of the plane by no more than `slack`. Then no interior
/// point of the face reaches through the plane, and the ball and the plane,
/// whose carriers meet only on that circle, cannot cross.
fn ball_face_on_one_side(face: &FaceRecord, centre: Vec3, radius: f64, normal: Vec3, origin: Vec3) -> bool {
    let s = &face.surface;
    let (Ok([u0, _]), Ok([v0, v1])) = (s.domain_u(), s.domain_v()) else { return false };
    let (Ok(south), Ok(north)) = (s.evaluate(u0, v0), s.evaluate(u0, v1)) else { return false };
    let Ok(axis) = north.sub(south).normalized() else { return false };
    let tilt = axis.cross(normal).length();
    if !(tilt <= DELTA) {
        return false;
    }
    let along = axis.dot(normal);
    let target = origin.sub(centre).dot(normal) * along;
    let slack = (tilt + DELTA) * radius;
    let height = |v: f64| s.evaluate(u0, v).map(|p| p.sub(centre).dot(axis));
    let latitude = |h: f64| -> Option<f64> {
        let (mut lo, mut hi) = (v0, v1);
        for _ in 0..80 {
            let mid = 0.5 * (lo + hi);
            if height(mid).ok()? < h {
                lo = mid
            } else {
                hi = mid
            }
        }
        Some(0.5 * (lo + hi))
    };
    let (Some(upper), Some(lower)) = (latitude(target + slack), latitude(target - slack)) else { return false };
    let mut below = true;
    let mut above = true;
    for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
        for control in &coedge.pcurve.control_points {
            if !(control.w > 0.0) {
                return false;
            }
            let v = control.y / control.w;
            below &= v <= upper;
            above &= v >= lower;
        }
    }
    below || above
}

/// Every trim control of `face` inside its carrier's parameter domain.
fn trims_inside_domain(face: &FaceRecord) -> bool {
    let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else { return false };
    let band = 1e-12 * (u1 - u0).abs().max(v1 - v0).max(1.0);
    face.loops.iter().flat_map(|loop_record| &loop_record.coedges).all(|coedge| {
        coedge.pcurve.control_points.iter().all(|control| {
            let w = control.w;
            if !(w > 0.0 && w.is_finite()) {
                return false;
            }
            let (x, y) = (control.x / w, control.y / w);
            x >= u0 - band && x <= u1 + band && y >= v0 - band && y <= v1 + band
        })
    })
}

pub(super) fn certifies(solid: &BrepSolid, options: SelfIntersectionOptions) -> bool {
    // Only the scanner's own scale-derived options: anything else the full
    // scan reads (or refuses) as asked.
    let expected = SelfIntersectionOptions::for_solid(solid);
    let readings = [options.chord_tolerance, options.straddle_band, options.min_crossing_length, options.weld];
    let same = |a: f64, b: f64| a.to_bits() == b.to_bits();
    if !readings.iter().all(|value| value.is_finite() && *value > 0.0)
        || !same(options.chord_tolerance, expected.chord_tolerance)
        || !same(options.straddle_band, expected.straddle_band)
        || !same(options.min_crossing_length, expected.min_crossing_length)
        || !same(options.weld, expected.weld)
        || options.max_triangles != expected.max_triangles
    {
        decline!();
    }
    let [shell] = solid.shells.as_slice() else { decline!() };
    let faces = &shell.faces;
    if faces.len() < 2 {
        decline!();
    }
    // (face index, centre, radius) and (face index, point, unit normal).
    let mut balls: Vec<(usize, Vec3, f64)> = Vec::new();
    let mut planes: Vec<(usize, Vec3, Vec3)> = Vec::new();
    for (index, face) in faces.iter().enumerate() {
        if let Some((centre, radius)) = sphere_certificate::native_ball_geometry(&face.surface).or_else(|| rebuilt_ball(&face.surface)) {
            if !trims_inside_domain(face) {
                decline!();
            }
            balls.push((index, centre, radius));
            continue;
        }
        match face.surface.analytic() {
            Some(crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) => {
                let Ok(normal) = u_dir.cross(*v_dir).normalized() else { decline!() };
                planes.push((index, *origin, normal));
            }
            _ => decline!(),
        }
    }
    if balls.is_empty() || planes.len() > 1 {
        decline!();
    }
    let size = balls.iter().map(|(_, centre, radius)| centre.length() + radius).fold(1.0, f64::max);
    let slack = DELTA * size;
    let centre = balls[0].1;
    for (i, (_, c, r)) in balls.iter().enumerate() {
        if c.sub(centre).length() > slack {
            decline!();
        }
        if balls[..i].iter().any(|(_, _, other)| (other - r).abs() <= 1e-9 * size) {
            decline!();
        }
    }
    if !solid.validate().is_empty() {
        decline!();
    }
    // Each ball meets the plane only along a closed edge both faces bound.
    if let Some(&(plane_index, origin, normal)) = planes.first() {
        let edges_of = |index: usize| -> std::collections::HashSet<u64> {
            faces[index].loops.iter().flat_map(|loop_record| &loop_record.coedges).map(|coedge| coedge.edge_id).collect()
        };
        let plane_edges = edges_of(plane_index);
        for &(ball_index, c, r) in &balls {
            let height = normal.dot(c.sub(origin));
            if height.abs() >= r * (1.0 + 1e-9) {
                continue; // the plane misses the sphere
            }
            let rho2 = r * r - height * height;
            if !(rho2 > (1e-6 * r) * (1e-6 * r)) {
                decline!(); // tangent or nearly: a point, not a circle
            }
            let (foot, rho) = (c.sub(normal.scale(height)), rho2.sqrt());
            let shared: Vec<u64> = edges_of(ball_index).intersection(&plane_edges).copied().collect();
            let on_circle = |edge: &crate::topology::EdgeRecord| -> bool {
                if edge.start_vertex_id != edge.end_vertex_id || edge.degenerate {
                    decline!();
                }
                (0..=32).all(|k| {
                    let t = edge.t0 + (edge.t1 - edge.t0) * k as f64 / 32.0;
                    edge.curve.evaluate(t).is_ok_and(|p| {
                        let w = p.sub(foot);
                        w.dot(normal).abs() <= 1e-9 * size && (w.length() - rho).abs() <= 1e-9 * size
                    })
                })
            };
            let closed_on_circle = shared.iter().filter(|id| solid.edges.iter().any(|edge| edge.id == **id && on_circle(edge))).count();
            if closed_on_circle != 1 {
                decline!();
            }
            if !ball_face_on_one_side(&faces[ball_index], c, r, normal, origin) {
                decline!();
            }
        }
    }
    true
}
