//! A fillet selection that no ball fits: two parallel selected edges whose
//! blends overlap on the face between them, beyond which the two walls are
//! parallel. The blends must then swallow that face and roll on the walls,
//! and a ball touches two parallel walls only when its diameter is their
//! distance (the full round). Any other radius is no fillet of the selection;
//! the sequential fallback would otherwise build one blend and roll the next
//! over it (a rib 2 wide at radius 1.01: a ridge where the suite wants a
//! refusal).
use super::analyze::{edge_and_faces, face_plane_normal};
use super::edges::resolve_edge_by_point;
use crate::topology::{BrepSolid, FaceRecord};
use crate::{KernelRefusal, KernelStage, Vec3};


/// The refusal code: two blends overlap on a face between parallel walls.
pub const PARALLEL_WALL_LIMIT_WHAT: &str = "fillet_wider_than_parallel_walls";

fn is_plane(face: &FaceRecord) -> bool {
    let s = &face.surface;
    if !(s.degree_u == 1 && s.degree_v == 1 && s.control_points.len() == 2 && s.control_points[0].len() == 2) {
        return false;
    }
    // A bilinear patch is a plane only untwisted: its fourth corner in the
    // plane of the other three.
    let Some(c) = [s.control_points[0][0], s.control_points[0][1], s.control_points[1][0], s.control_points[1][1]]
        .iter()
        .map(|p| p.point().ok())
        .collect::<Option<Vec<_>>>()
    else {
        return false;
    };
    let (a, b) = (c[1].sub(c[0]), c[2].sub(c[0]));
    let Ok(n) = a.cross(b).normalized() else { return false };
    let size = a.length().max(b.length()).max(c[3].sub(c[0]).length());
    n.dot(c[3].sub(c[0])).abs() <= 1e-12 * size.max(1e-300)
}

/// `Some(refusal)` when two of `edge_points`' edges are such a pair at
/// `radius`: the caller's strict reading of a fillet selection. By default
/// the kernel builds such a pair as two balls each tangent to its own faces,
/// pinching to a ridge where they meet (a valid solid:
/// `the_radius_gate_leaves_pinching_and_star_selections_alone`, a 20-wide
/// cube top at r = 10.001 to 18); a caller that wants a ball touching every
/// face it rolls on (brep-conformance's `fillet-over-limit`) asks for this
/// refusal (`FilletOptions::refuse_pinch` in the kernel).
pub fn pinch_refusal(solid: &BrepSolid, edge_points: &[Vec3], radius: f64) -> Option<KernelRefusal> {
    if edge_points.len() < 2 || !(radius > 0.0) {
        return None;
    }
    // Each straight selected edge: (id, a point on it, its unit direction, its two planar faces).
    let mut edges = Vec::new();
    for point in edge_points {
        let id = resolve_edge_by_point(solid, *point).ok()?;
        let (edge, faces) = edge_and_faces(solid, id).ok()?;
        if edge.curve.degree != 1 || faces.len() != 2 || !faces.iter().all(|f| is_plane(f)) {
            continue;
        }
        let (a, b) = (edge.curve.evaluate(edge.t0).ok()?, edge.curve.evaluate(edge.t1).ok()?);
        let Ok(direction) = b.sub(a).normalized() else { continue };
        let middle = a.add(b).scale(0.5);
        edges.push((id, middle, direction, [faces[0].clone(), faces[1].clone()], [a, b]));
    }
    // The solid's own size (its box's diagonal), not its distance from the
    // origin: the reading is the same wherever the rib stands.
    let (lo, hi) = solid.vertices.iter().fold(([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]), |(lo, hi), v| {
        let p = [v.point.x, v.point.y, v.point.z];
        ([lo[0].min(p[0]), lo[1].min(p[1]), lo[2].min(p[2])], [hi[0].max(p[0]), hi[1].max(p[1]), hi[2].max(p[2])])
    });
    let scale = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt().max(1e-300);
    for i in 0..edges.len() {
        for j in i + 1..edges.len() {
            let (ia, pa, da, fa, ends_a) = &edges[i];
            let (ib, pb, db, fb, ends_b) = &edges[j];
            if ia == ib || da.cross(*db).length() > 1e-9 {
                continue;
            }
            // Side by side along their common direction: their spans overlap
            // (two edges at opposite ends of a long face never meet).
            let span = |ends: &[Vec3; 2]| {
                let (s0, s1) = (ends[0].dot(*da), ends[1].dot(*da));
                (s0.min(s1), s0.max(s1))
            };
            let ((a0, a1), (b0, b1)) = (span(ends_a), span(ends_b));
            if !(a1.min(b1) - a0.max(b0) > 0.0) {
                continue;
            }
            // The face both lie on, and each edge's other face (its wall).
            let Some((shared_a, shared_b)) = (0..2).flat_map(|x| (0..2).map(move |y| (x, y))).find(|&(x, y)| fa[x].id == fb[y].id) else { continue };
            let (shared, wall_a, wall_b) = (&fa[shared_a], &fa[1 - shared_a], &fb[1 - shared_b]);
            let (Ok((_, n_shared)), Ok((on_a, n_a)), Ok((on_b, n_b))) = (face_plane_normal(shared, *pa), face_plane_normal(wall_a, *pa), face_plane_normal(wall_b, *pb)) else { continue };
            // The walls parallel and facing away from each other.
            if !(n_a.dot(n_b) <= -1.0 + 1e-9) {
                continue;
            }
            // Each edge convex, and its blend's setback on the shared face:
            // r / tan(φ/2), φ the angle inside the material (π less the
            // normals' angle).
            let setback = |n_wall: Vec3| -> Option<f64> {
                let between = n_shared.dot(n_wall).clamp(-1.0, 1.0).acos();
                let inside = std::f64::consts::PI - between;
                (inside > 1e-9 && inside < std::f64::consts::PI - 1e-9).then(|| radius / (0.5 * inside).tan())
            };
            let (Some(sa), Some(sb)) = (setback(n_a), setback(n_b)) else { continue };
            // The face lies BETWEEN the edges: the point midway across them is on
            // it (a boss's two base edges share the plate's top face, but what
            // lies between them is the boss, not that face).
            let midway = pa.add(pb.sub(*pa).scale(0.5));
            let between = crate::project_point_to_surface(&shared.surface, midway)
                .ok()
                .filter(|q| q.distance <= 1e-9 * scale)
                .and_then(|q| crate::parameter_point_in_face(shared, crate::Vec2 { x: q.u, y: q.v }, 1e-9).ok())
                .is_some_and(|class| class == crate::PolygonClass::Inside);
            if !between {
                continue;
            }
            // The edges' distance across the shared face, and the walls' distance.
            let across = pb.sub(*pa);
            let width = across.sub(da.scale(across.dot(*da))).length();
            let walls = on_b.sub(on_a).dot(n_a).abs();
            if sa + sb <= width * (1.0 + 1e-9) + 1e-12 * scale {
                continue;
            }
            if (2.0 * radius - walls).abs() <= 1e-9 * walls.max(1e-300) + 1e-12 * scale {
                continue; // the full round: the ball touches both walls
            }
            // The input asks for what no ball can make: an invalid size,
            // not a limit of the kernel (as a fillet too large for its faces).
            return Some(KernelRefusal::input(
                KernelStage::Collect,
                PARALLEL_WALL_LIMIT_WHAT,
                format!(
                    "fillet: no ball of radius {radius} fits: edges {ia} and {ib} lie {width:.9} apart on face {}, their blends would need {:.9} of it, and the walls beyond are parallel {walls:.9} apart, so a ball rolling on both needs radius {:.9}",
                    shared.id,
                    sa + sb,
                    0.5 * walls
                ),
            ));
        }
    }
    None
}
