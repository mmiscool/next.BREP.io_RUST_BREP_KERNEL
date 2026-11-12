//! A body grown by at least the radius of a straight bore through it
//! (`BREP_OFFSET_BORE_DIMPLES`, default on; `=0` leaves the growth filled).
//!
//! The parallel body closes the bore, but not flat: each end keeps a dimple
//! where the rolling ball about the bore's rim reaches the axis, at
//! `√(d² − r²)` past the end face instead of `d`. The sharp offset loses the
//! bore once its offset radius `r − d` reaches zero, so the rim edge the
//! rounding would have rolled is gone and the grown body comes out filled
//! (a tube R 5 r 2 h 10 grown by 2: 2084.442 for 2079.6231064). The fill is
//! the parallel body of the bore-filled solid exactly, so each dimple is
//! carved from it: the solid of revolution between the rim's torus and the
//! grown end plane.
//!
//! The carve is exact only where nothing but the rim is within `d` of the
//! dimple. A bore qualifies when its cylinder faces are concave of radius at
//! most `d`, it is open at both ends onto end planes square to its axis that
//! are material out to `r + d` around the rim, every other face and edge
//! stands at least `d` from both dimples (a box test in the bore's frame,
//! conservative), and the grown body still fills the dimple. Anything else
//! leaves the grown body as it was.
use super::debug_enabled;
use crate::analytic_surface::AnalyticSurface;
use crate::topology::{BrepSolid, FaceRecord};
use crate::{boolean_operation, classify_point, make_arc, make_line, revolve_profile_brep, BooleanOperation, BooleanOptions, PointClass, Vec3};
use std::f64::consts::PI;

pub(super) fn bore_dimples_enabled() -> bool {
    switch!("BREP_OFFSET_BORE_DIMPLES", true)
}

/// A straight bore: a point on its axis, the unit axis, its radius and the
/// axial span its cylinder faces cover (from the point).
struct Bore {
    origin: Vec3,
    axis: Vec3,
    radius: f64,
    span: [f64; 2],
    faces: Vec<u64>,
}

/// The cylinder `surface` lies on, as (point on axis, unit axis, radius).
fn cylinder_of(surface: &crate::NurbsSurface) -> Option<(Vec3, Vec3, f64)> {
    match surface.analytic()? {
        AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. } => {
            ((rho0 - rho1).abs() <= 1e-9 * rho0.abs().max(1.0)).then_some((frame.origin, frame.axis, *rho0))
        }
        AnalyticSurface::Revolution { frame, generatrix, .. } => {
            // A partial sweep of a straight generatrix parallel to the axis.
            let mut radius: Option<f64> = None;
            for control in &generatrix.control_points {
                let p = control.point().ok()?;
                let d = p.sub(frame.origin);
                let rho = d.sub(frame.axis.scale(d.dot(frame.axis))).length();
                match radius {
                    None => radius = Some(rho),
                    Some(r) if (r - rho).abs() <= 1e-9 * r.max(1.0) => {}
                    _ => return None,
                }
            }
            radius.map(|r| (frame.origin, frame.axis, r))
        }
        _ => None,
    }
}

fn classify(solid: &BrepSolid, point: Vec3, scale: f64) -> Option<PointClass> {
    classify_point(point, solid, 1e-9 * scale).ok().map(|c| c.class)
}

/// Two unit vectors square to `axis` and to each other.
fn across(axis: Vec3) -> (Vec3, Vec3) {
    let seed = if axis.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let first = seed.sub(axis.scale(seed.dot(axis)));
    let first = first.scale(1.0 / first.length());
    (first, axis.cross(first))
}

/// The straight bores of `source` of radius at most `distance`, open at both ends.
fn bores(source: &BrepSolid, distance: f64, scale: f64) -> Vec<Bore> {
    let mut found: Vec<Bore> = Vec::new();
    for face in source.shells.iter().flat_map(|shell| &shell.faces) {
        let Some((origin, axis, radius)) = cylinder_of(&face.surface) else { continue };
        if !(radius > 0.0 && radius <= distance * (1.0 + 1e-9)) {
            continue;
        }
        let axial = |p: Vec3| p.sub(origin).dot(axis);
        let mut span = [f64::INFINITY, f64::NEG_INFINITY];
        // The face's extent along the axis, read at its edges' vertices (an edge's curve may
        // run past them, a seam on its untrimmed generator).
        for edge in face_edges(source, face) {
            for id in [edge.start_vertex_id, edge.end_vertex_id] {
                if let Some(vertex) = source.vertices.iter().find(|vertex| vertex.id == id) {
                    span = [span[0].min(axial(vertex.point)), span[1].max(axial(vertex.point))];
                }
            }
        }
        if !(span[1] > span[0]) {
            continue;
        }
        let same = found.iter_mut().find(|bore| {
            let off = origin.sub(bore.origin);
            bore.axis.cross(axis).length() <= 1e-9
                && off.sub(bore.axis.scale(off.dot(bore.axis))).length() <= 1e-9 * scale
                && (bore.radius - radius).abs() <= 1e-9 * scale
        });
        match same {
            Some(bore) => {
                let shift = origin.sub(bore.origin).dot(bore.axis);
                let sign = bore.axis.dot(axis).signum();
                let (a, b) = (shift + sign * span[0], shift + sign * span[1]);
                bore.span = [bore.span[0].min(a.min(b)), bore.span[1].max(a.max(b))];
                bore.faces.push(face.id);
            }
            None => found.push(Bore { origin, axis, radius, span, faces: vec![face.id] }),
        }
    }
    // Hollow along its whole length (concave, open at both ends): the axis is
    // outside the body at the middle and just past each end, and the body is
    // material just outside the cylinder at the middle.
    found.retain(|bore| {
        let (x, y) = across(bore.axis);
        let at = |t: f64| bore.origin.add(bore.axis.scale(t));
        let middle = 0.5 * (bore.span[0] + bore.span[1]);
        let length = bore.span[1] - bore.span[0];
        let eps = 1e-3 * length.min(bore.radius);
        let out = |p: Vec3| classify(source, p, scale) == Some(PointClass::Out);
        let inside = |p: Vec3| classify(source, p, scale) == Some(PointClass::In);
        out(at(middle))
            && out(at(bore.span[0] - eps))
            && out(at(bore.span[1] + eps))
            && [x, y, x.scale(-1.0), y.scale(-1.0)].iter().all(|r| inside(at(middle).add(r.scale(bore.radius + eps))))
    });
    found
}

fn face_edges<'a>(source: &'a BrepSolid, face: &FaceRecord) -> Vec<&'a crate::topology::EdgeRecord> {
    face.loops
        .iter()
        .flat_map(|lp| &lp.coedges)
        .filter_map(|coedge| source.edges.iter().find(|edge| edge.id == coedge.edge_id))
        .collect()
}

/// Whether the end of `bore` at axial `t` (outward `out` along the axis) is
/// flat at the rim: material just inside the end plane around it, empty just
/// past it.
fn flat_end(source: &BrepSolid, bore: &Bore, t: f64, out: Vec3, distance: f64, scale: f64) -> bool {
    let (x, y) = across(bore.axis);
    let centre = bore.origin.add(bore.axis.scale(t));
    let eps = 1e-4 * bore.radius.min(distance);
    // Local flatness at the rim; what lies further out is the clearance test's.
    for k in 1..=2 {
        let rho = bore.radius + 0.25 * bore.radius.min(distance) * k as f64 / 2.0;
        for direction in [x, y, x.scale(-1.0), y.scale(-1.0), x.add(y).scale(0.5f64.sqrt()), x.sub(y).scale(0.5f64.sqrt())] {
            let foot = centre.add(direction.scale(rho));
            if classify(source, foot.sub(out.scale(eps)), scale) != Some(PointClass::In)
                || classify(source, foot.add(out.scale(eps)), scale) != Some(PointClass::Out)
            {
                return false;
            }
        }
    }
    true
}

/// Whether every face and edge of `source` that is not the bore's own stands
/// at least `distance` from the dimple at the end `t` (outward `out`): the
/// dimple lies in the cylinder `ρ ≤ r`, `0 ≤ z ≤ d` past the end plane, and
/// each face is read on its whole untrimmed surface at dense samples, a sample
/// near the bar refined by a local search. The end face that owns the rim is
/// read by its edges other than the rim: the rim is the dimple's cause, and the
/// face's region is the end plane itself, at height zero under the dimple.
fn clear_of_others(source: &BrepSolid, bore: &Bore, t: f64, out: Vec3, distance: f64, scale: f64) -> bool {
    let (x, y) = across(bore.axis);
    let centre = bore.origin.add(bore.axis.scale(t));
    let gap = |p: Vec3| {
        let d = p.sub(centre);
        let (a, b, z) = (d.dot(x), d.dot(y), d.dot(out));
        let radial = ((a * a + b * b).sqrt() - bore.radius).max(0.0);
        let axial = (-z).max(z - distance).max(0.0);
        radial.hypot(axial)
    };
    let enough = distance * (1.0 - 1e-9);
    // The rim circle itself: every sampled point at the rim radius in the end plane.
    let on_rim = |edge: &crate::topology::EdgeRecord| {
        let Ok([t0, t1]) = edge.curve.domain() else { return false };
        (0..=16).all(|k| {
            edge.curve.evaluate(t0 + (t1 - t0) * k as f64 / 16.0).is_ok_and(|p| {
                let d = p.sub(centre);
                let (a, b, z) = (d.dot(x), d.dot(y), d.dot(out));
                z.abs() <= 1e-9 * scale && ((a * a + b * b).sqrt() - bore.radius).abs() <= 1e-9 * scale
            })
        })
    };
    // Each sample within reach of a violation (its gap under the bar plus the largest sample
    // spacing) is refined by a local search over its neighbouring parameter cell, so a face or
    // edge exactly `d` away (a plate's side at the dimple's reach) reads as clear.
    let edge_clear = |edge: &crate::topology::EdgeRecord| -> bool {
        let Ok([t0, t1]) = edge.curve.domain() else { return false };
        let n = 256;
        let step = (t1 - t0) / n as f64;
        let read = |t: f64| edge.curve.evaluate(t.clamp(t0, t1)).map_or(f64::NEG_INFINITY, gap);
        let points: Vec<Vec3> = (0..=n).filter_map(|k| edge.curve.evaluate(t0 + step * k as f64).ok()).collect();
        if points.len() != n + 1 {
            return false;
        }
        let spacing = points.windows(2).map(|pair| pair[1].sub(pair[0]).length()).fold(0.0f64, f64::max);
        (0..=n).all(|k| {
            let here = gap(points[k]);
            if here - spacing >= enough {
                return true;
            }
            // Golden section over the two neighbouring cells.
            let (mut a, mut b) = (t0 + step * (k as f64 - 1.0), t0 + step * (k as f64 + 1.0));
            let ratio = 0.5 * (5f64.sqrt() - 1.0);
            let mut best = here;
            for _ in 0..80 {
                let (c, d) = (b - ratio * (b - a), a + ratio * (b - a));
                let (fc, fd) = (read(c), read(d));
                best = best.min(fc).min(fd);
                if fc < fd { b = d } else { a = c }
            }
            best >= enough
        })
    };
    let surface_clear = |surface: &crate::NurbsSurface| -> bool {
        let (Ok([u0, u1]), Ok([v0, v1])) = (surface.domain_u(), surface.domain_v()) else { return false };
        let n = 64;
        let (du, dv) = ((u1 - u0) / n as f64, (v1 - v0) / n as f64);
        let read = |u: f64, v: f64| surface.evaluate(u.clamp(u0, u1), v.clamp(v0, v1)).map_or(f64::NEG_INFINITY, gap);
        let mut grid = Vec::with_capacity((n + 1) * (n + 1));
        for i in 0..=n {
            for j in 0..=n {
                match surface.evaluate(u0 + du * i as f64, v0 + dv * j as f64) {
                    Ok(p) => grid.push(p),
                    Err(_) => return false,
                }
            }
        }
        let at = |i: usize, j: usize| grid[i * (n + 1) + j];
        let mut spacing = 0.0f64;
        for i in 0..n {
            for j in 0..n {
                spacing = spacing.max(at(i, j).sub(at(i + 1, j + 1)).length()).max(at(i + 1, j).sub(at(i, j + 1)).length());
            }
        }
        (0..=n).all(|i| {
            (0..=n).all(|j| {
                let here = gap(at(i, j));
                if here - spacing >= enough {
                    return true;
                }
                // Pattern search from the sample, its step halving down to 1e-12 of a cell.
                let (mut u, mut v, mut best) = (u0 + du * i as f64, v0 + dv * j as f64, here);
                let (mut su, mut sv) = (du, dv);
                while su > 1e-12 * du {
                    let mut moved = false;
                    for (a, b) in [(su, 0.0), (-su, 0.0), (0.0, sv), (0.0, -sv)] {
                        let value = read(u + a, v + b);
                        if value < best {
                            best = value;
                            u = (u + a).clamp(u0, u1);
                            v = (v + b).clamp(v0, v1);
                            moved = true;
                        }
                    }
                    if !moved {
                        su *= 0.5;
                        sv *= 0.5;
                    }
                }
                best >= enough
            })
        })
    };
    for face in source.shells.iter().flat_map(|shell| &shell.faces) {
        if bore.faces.contains(&face.id) {
            continue;
        }
        // The end face that owns this rim lies in the end plane, so its patch covers the bore's
        // mouth; it is read by its other edges (its region past the rim is plane at height 0 under
        // the dimple, and its boundary elsewhere is what could come near). Every other face is
        // read by its whole untrimmed surface, which holds the face.
        let edges = face_edges(source, face);
        let owns_rim = matches!(face.surface.analytic(), Some(AnalyticSurface::Plane { .. })) && edges.iter().any(|edge| on_rim(edge));
        let clear = if owns_rim {
            edges.iter().filter(|edge| !on_rim(edge)).all(|edge| edge_clear(edge))
        } else {
            surface_clear(&face.surface)
        };
        if !clear {
            os_debug!("bore dimples: face {} stands within {distance} of the dimple at {t}", face.id);
            return false;
        }
    }
    true
}

/// The dimple at a rim of radius `r` (centre `centre`, end plane's outward
/// normal `out`) for a growth `d >= r`: between the rim's torus and the grown
/// end plane, revolved about the bore's axis.
fn dimple(centre: Vec3, out: Vec3, r: f64, d: f64) -> Result<BrepSolid, String> {
    let (x, _) = across(out);
    let at = |rho: f64, z: f64| centre.add(x.scale(rho)).add(out.scale(z));
    let low = (d * d - r * r).max(0.0).sqrt();
    let on_axis = make_line(at(0.0, low), at(0.0, d))?;
    let lid = make_line(at(0.0, d), at(r, d))?;
    // The rim's torus in this half-plane: about (r, 0) from straight up to the axis.
    let end = PI - (r / d).min(1.0).acos();
    let arc = make_arc(at(r, 0.0), x, out, d, 0.5 * PI, end)?;
    revolve_profile_brep(&[on_axis, lid, arc], centre, out, 2.0 * PI)
}

/// `grown` (`source` grown by `distance`) with the dimple at each end of every
/// qualifying bore carved, or `None` when there is none to carve or any step
/// declines.
pub(super) fn carve_bore_dimples(source: &BrepSolid, grown: &BrepSolid, distance: f64) -> Option<BrepSolid> {
    if !bore_dimples_enabled() || !(distance > 0.0) {
        return None;
    }
    let scale = crate::solid_scale(source).max(1.0);
    let mut voids = Vec::new();
    let found = bores(source, distance, scale);
    os_debug!("bore dimples: {} bore(s) of radius at most {distance}", found.len());
    for bore in found {
        for (t, out) in [(bore.span[1], bore.axis), (bore.span[0], bore.axis.scale(-1.0))] {
            if !flat_end(source, &bore, t, out, distance, scale) {
                os_debug!("bore dimples: the end at {t} is not flat at its rim");
                return None;
            }
            if !clear_of_others(source, &bore, t, out, distance, scale) {
                os_debug!("bore dimples: a bore of radius {} declined at its end {t}", bore.radius);
                return None;
            }
            let centre = bore.origin.add(bore.axis.scale(t));
            // Still filled: the dimple's middle on the axis is inside the growth.
            let low = (distance * distance - bore.radius * bore.radius).max(0.0).sqrt();
            let probe = centre.add(out.scale(0.5 * (low + distance)));
            if classify(grown, probe, scale) != Some(PointClass::In) {
                os_debug!("bore dimples: the growth is already carved at {probe:?}");
                return None;
            }
            voids.push(dimple(centre, out, bore.radius, distance).ok()?);
        }
    }
    if voids.is_empty() {
        return None;
    }
    let mut carved = grown.clone();
    for void in &voids {
        carved = match boolean_operation(&carved, void, BooleanOperation::Subtract, &BooleanOptions::default()) {
            Ok(next) => next,
            Err(error) => {
                os_debug!("bore dimples: the carve declined: {}", error.message);
                return None;
            }
        };
    }
    carved.validate().is_empty().then_some(carved)
}
