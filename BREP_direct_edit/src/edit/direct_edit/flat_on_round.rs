//! Deleting FLATS from a ROUND: `BREP_DELETE_FLAT_ON_ROUND` (default on; `=0`
//! restores the gap fill; `check` builds both and compares them).
//!
//! A shaft with flats milled along it, a cone frustum with a flat on its side,
//! a ball with its top cut flat: each is a primitive P (a cylinder or cone
//! between two cap planes square to its axis, or a whole ball) less the
//! material the flats cut away. When every face the delete keeps lies on P's
//! boundary (on its one round carrier, or on one of its two cap planes) and
//! every deleted face is planar and lies inside P, the body is P less pieces
//! bounded by those flats and P's own surface, so the healed body is P itself:
//! it is built directly, without the gap fill's booleans.
//!
//! Anything else is not this lane's: a hole, a groove or a pocket on the
//! round, a third cap plane, a concave round (a bore), a cap with an inner
//! loop, a second shell. The lane then declines and the gap fill answers as
//! before.
//!
//! Every kept face's id and name survive the heal (the delete's contract): P's
//! round face takes the id and name of the kept round face with the smallest
//! id, and each cap the id and name of the kept face on its plane with the
//! smallest id. Edges, coedges, loops and vertices are new, numbered past the
//! body's own.
use crate::analytic_surface::AnalyticSurface;
use crate::{make_cone_brep, make_cylinder_brep, make_sphere_brep, parameter_point_in_face, BrepSolid, FaceRecord, PolygonClass, Vec2, Vec3};
use std::collections::HashSet;

/// `BREP_DELETE_FLAT_ON_ROUND`: on by default (promoted 2026-11-22); `=0`
/// answers from the gap fill; `check` answers from the gap fill and reports
/// where the two disagree.
pub(crate) fn flat_on_round_mode() -> crate::env_read::Mode {
    crate::mode_switch!("BREP_DELETE_FLAT_ON_ROUND", true)
}

/// The round carrier: its axis line and its radius along it.
#[derive(Clone, Copy, Debug)]
enum Round {
    /// A cylinder or cone: `rho(t) = rho0 + slope · t`, `t` measured from
    /// `origin` along `axis`.
    Ruled { origin: Vec3, axis: Vec3, rho0: f64, slope: f64 },
    Ball { centre: Vec3, axis: Vec3, radius: f64 },
}

impl Round {
    fn of(face: &FaceRecord) -> Option<Round> {
        match face.surface.analytic()? {
            AnalyticSurface::RuledRevolution { frame, rho0, rho1, height } if *height > 0.0 => {
                Some(Round::Ruled { origin: frame.origin, axis: frame.axis, rho0: *rho0, slope: (rho1 - rho0) / height })
            }
            AnalyticSurface::Sphere { frame, radius } if *radius > 0.0 => {
                Some(Round::Ball { centre: frame.origin, axis: frame.axis, radius: *radius })
            }
            _ => None,
        }
    }

    /// The same carrier as `other`, to `tolerance`: one axis line and one
    /// radius along it.
    fn same(&self, other: &Round, tolerance: f64) -> bool {
        match (self, other) {
            (Round::Ruled { origin: o1, axis: a1, rho0: r1, slope: s1 }, Round::Ruled { origin: o2, axis: a2, .. }) => {
                if a1.cross(*a2).length() > 1e-12 {
                    return false;
                }
                let d = o2.sub(*o1);
                if d.sub(a1.scale(d.dot(*a1))).length() > tolerance {
                    return false;
                }
                // The radius at the other's origin and one unit further on.
                let t = d.dot(*a1);
                let step = if a1.dot(*a2) > 0.0 { 1.0 } else { -1.0 };
                [(t, 0.0), (t + step, 1.0)].iter().all(|&(at, along)| {
                    let mine = r1 + s1 * at;
                    (mine - other.radius_at(*o2, along)).abs() <= tolerance
                })
            }
            (Round::Ball { centre: c1, radius: r1, .. }, Round::Ball { centre: c2, radius: r2, .. }) => {
                c1.sub(*c2).length() <= tolerance && (r1 - r2).abs() <= tolerance
            }
            _ => false,
        }
    }

    /// A ruled carrier's radius `along` its own axis from `origin` (used by
    /// [`Round::same`]); a ball's radius.
    fn radius_at(&self, _origin: Vec3, along: f64) -> f64 {
        match self {
            Round::Ruled { rho0, slope, .. } => rho0 + slope * along,
            Round::Ball { radius, .. } => *radius,
        }
    }

    fn axis(&self) -> Vec3 {
        match self {
            Round::Ruled { axis, .. } | Round::Ball { axis, .. } => *axis,
        }
    }
}

/// Points of `face`: along every edge it uses, and on a grid over its own
/// trim's parameter box where they fall inside it.
fn face_points(solid: &BrepSolid, face: &FaceRecord) -> Option<Vec<Vec3>> {
    let mut points = Vec::new();
    let edges: HashSet<u64> = face.loops.iter().flat_map(|l| &l.coedges).map(|c| c.edge_id).collect();
    for edge in solid.edges.iter().filter(|edge| edges.contains(&edge.id) && !edge.degenerate) {
        for k in 0..=16 {
            points.push(edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 16.0).ok()?);
        }
    }
    let (mut u_low, mut u_high, mut v_low, mut v_high) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
        for control in &coedge.pcurve.control_points {
            let p = control.point().ok()?;
            u_low = u_low.min(p.x);
            u_high = u_high.max(p.x);
            v_low = v_low.min(p.y);
            v_high = v_high.max(p.y);
        }
    }
    if u_low < u_high && v_low < v_high {
        for i in 1..=9 {
            for j in 1..=9 {
                let (u, v) = (u_low + (u_high - u_low) * i as f64 / 10.0, v_low + (v_high - v_low) * j as f64 / 10.0);
                if matches!(parameter_point_in_face(face, Vec2 { x: u, y: v }, 1e-9).ok()?, PolygonClass::Inside) {
                    points.push(face.surface.evaluate(u, v).ok()?);
                }
            }
        }
    }
    (!points.is_empty()).then_some(points)
}

/// The healed body, when the delete of `face_ids` is the flats-on-a-round
/// case; `None` (decline) otherwise.
pub(crate) fn heal_flats_on_a_round(solid: &BrepSolid, face_ids: &[u64]) -> Option<BrepSolid> {
    if solid.shells.len() != 1 || face_ids.is_empty() {
        return None;
    }
    let scale = crate::solid_scale(solid).max(1.0);
    let tolerance = 1e-9 * scale;
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    let faces: Vec<&FaceRecord> = solid.shells[0].faces.iter().collect();
    if !selected.iter().all(|id| faces.iter().any(|face| face.id == *id)) {
        return None;
    }
    // The kept faces: one round carrier, and planes square to its axis.
    let mut round: Option<Round> = None;
    let mut round_faces: Vec<&FaceRecord> = Vec::new();
    let mut planes: Vec<(&FaceRecord, Vec3, Vec3)> = Vec::new();
    for face in faces.iter().filter(|face| !selected.contains(&face.id)) {
        match face.surface.analytic()? {
            AnalyticSurface::Plane { origin, u_dir, v_dir, .. } => {
                let mut normal = u_dir.cross(*v_dir).normalized().ok()?;
                if !face.same_sense {
                    normal = normal.scale(-1.0);
                }
                planes.push((face, *origin, normal));
            }
            _ => {
                let carrier = Round::of(face)?;
                match &round {
                    Some(first) if !first.same(&carrier, tolerance) => return None,
                    Some(_) => {}
                    None => round = Some(carrier),
                }
                round_faces.push(face);
            }
        }
    }
    let round = round?;
    let axis = round.axis();
    // Every kept round face bounds material on the axis side: its outward
    // normal points away from the axis (a bore's would point in).
    for face in &round_faces {
        let points = face_points(solid, face)?;
        let point = points[points.len() / 2];
        let foot = crate::project_point_to_surface(&face.surface, point).ok()?;
        let mut normal = face.surface.normal(foot.u, foot.v).ok()?;
        if !face.same_sense {
            normal = normal.scale(-1.0);
        }
        let away = match round {
            Round::Ruled { origin, .. } => {
                let d = foot.point.sub(origin);
                d.sub(axis.scale(d.dot(axis)))
            }
            Round::Ball { centre, .. } => foot.point.sub(centre),
        };
        if !(normal.dot(away) > 0.0) {
            return None;
        }
    }
    // The cap planes: square to the axis, two of them (none for a ball), the
    // first one's outward normal against the axis, the second's along it.
    let (low, high) = match round {
        Round::Ruled { origin, .. } => {
            let mut heights: Vec<(f64, bool)> = Vec::new();
            for (_, point, normal) in &planes {
                if normal.cross(axis).length() > 1e-12 {
                    return None;
                }
                let height = point.sub(origin).dot(axis);
                let up = normal.dot(axis) > 0.0;
                if !heights.iter().any(|(h, u)| (h - height).abs() <= tolerance && *u == up) {
                    heights.push((height, up));
                }
            }
            let [(a, a_up), (b, b_up)] = heights[..] else { return None };
            let ((low, low_up), (high, high_up)) = if a < b { ((a, a_up), (b, b_up)) } else { ((b, b_up), (a, a_up)) };
            if low_up || !high_up || !(high - low > tolerance) {
                return None;
            }
            (low, high)
        }
        Round::Ball { .. } => {
            if !planes.is_empty() {
                return None;
            }
            (0.0, 0.0)
        }
    };
    // Inside P (or on its boundary), to `tolerance`.
    let inside = |p: Vec3| -> bool {
        match round {
            Round::Ruled { origin, rho0, slope, .. } => {
                let d = p.sub(origin);
                let t = d.dot(axis);
                let r = d.sub(axis.scale(t)).length();
                t >= low - tolerance && t <= high + tolerance && r <= rho0 + slope * t + tolerance
            }
            Round::Ball { centre, radius, .. } => p.sub(centre).length() <= radius + tolerance,
        }
    };
    // Every kept face lies on P's boundary within its caps; every deleted
    // face is planar and inside P.
    for face in &faces {
        let points = face_points(solid, face)?;
        if !points.iter().all(|p| inside(*p)) {
            return None;
        }
        if selected.contains(&face.id) && !matches!(face.surface.analytic(), Some(AnalyticSurface::Plane { .. })) {
            return None;
        }
    }
    // A cap with an inner loop is a feature on it (a hole), not P's cap.
    if planes.iter().any(|(face, _, _)| face.loops.len() != 1) {
        return None;
    }
    // P, on the round's own axis.
    let mut healed = match round {
        Round::Ruled { origin, rho0, slope, .. } => {
            let base = origin.add(axis.scale(low));
            let (bottom, top) = (rho0 + slope * low, rho0 + slope * high);
            if !(bottom > tolerance && top > tolerance) {
                return None;
            }
            if (bottom - top).abs() <= tolerance {
                make_cylinder_brep(base, axis, bottom, high - low).ok()?
            } else {
                make_cone_brep(base, axis, bottom, top, high - low).ok()?
            }
        }
        Round::Ball { centre, axis, radius } => make_sphere_brep(centre, radius, axis).ok()?,
    };
    // Ids and names: each of P's faces takes a kept face's.
    let mut used: HashSet<u64> = HashSet::default();
    for face in healed.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        let keep: Option<&FaceRecord> = match face.surface.analytic() {
            Some(AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) => {
                let mut normal = u_dir.cross(*v_dir).normalized().ok()?;
                if !face.same_sense {
                    normal = normal.scale(-1.0);
                }
                let origin = *origin;
                planes
                    .iter()
                    .filter(|(_, point, kept_normal)| kept_normal.dot(normal) > 0.0 && point.sub(origin).dot(normal).abs() <= tolerance)
                    .map(|(kept, _, _)| *kept)
                    .min_by_key(|kept| kept.id)
            }
            _ => round_faces.iter().copied().min_by_key(|kept| kept.id),
        };
        let keep = keep?;
        if !used.insert(keep.id) {
            return None;
        }
        face.id = keep.id;
        face.name = keep.name.clone();
    }
    if used.len() != healed.shells[0].faces.len() {
        return None;
    }
    // Everything else numbered past the body's own.
    let next = solid
        .edges
        .iter()
        .map(|e| e.id)
        .chain(solid.vertices.iter().map(|v| v.id))
        .chain(solid.shells.iter().flat_map(|s| &s.faces).flat_map(|f| &f.loops).flat_map(|l| std::iter::once(l.id).chain(l.coedges.iter().map(|c| c.id))))
        .max()
        .unwrap_or(0)
        + 1;
    for vertex in &mut healed.vertices {
        vertex.id += next;
    }
    for edge in &mut healed.edges {
        edge.id += next;
        edge.start_vertex_id += next;
        edge.end_vertex_id += next;
    }
    for face in healed.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        for lp in &mut face.loops {
            lp.id += next;
            for coedge in &mut lp.coedges {
                coedge.id += next;
                coedge.edge_id += next;
            }
        }
    }
    healed.id = solid.id;
    healed.shells[0].id = solid.shells[0].id;
    (healed.validate().is_empty()).then_some(healed)
}
