//! A chamfer too large for its faces: its setback on a planar face runs past
//! that face's far boundary. A chamfer plane cut beyond the face reaches into
//! the next wall, so what would be built is no chamfer of the edge; it is
//! refused as an input error, as a fillet too large for its faces is
//! (`support_extent`). Unlike the fillet's reading, a chamfer overrunning one
//! face into air is not admitted: its plane keeps cutting there.
use super::analyze::{edge_and_faces, into_face_direction};
use crate::topology::{BrepSolid, FaceRecord};
use crate::{KernelRefusal, KernelStage};

/// `BREP_CHAMFER_EXTENT` (on by default; `=0` lets an oversized chamfer
/// through as before).
pub(super) fn chamfer_extent_on() -> bool {
    crate::switch!("BREP_CHAMFER_EXTENT", true)
}

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

/// Refuse when the setback `distances[k]` along the edge's face `k` (in
/// [`edge_and_faces`] order, as the asymmetric chamfer reads them) leaves that
/// face at any of the edge's stations, past the band the surgery snaps
/// across. Planar faces only; anything this cannot read is admitted (no
/// refusal rests on a guess).
pub(super) fn check_chamfer_extent(solid: &BrepSolid, edge_id: u64, distances: [f64; 2], entry: &str) -> Result<(), KernelRefusal> {
    if !chamfer_extent_on() {
        return Ok(());
    }
    check_extent_by_station(solid, edge_id, &|k, _| distances[k], entry)
}

/// [`check_chamfer_extent`] with the setback on face `k` at the station
/// `fraction` of the edge's parameter (0 at its start) given by `setback`.
fn check_extent_by_station(solid: &BrepSolid, edge_id: u64, setback: &dyn Fn(usize, f64) -> f64, entry: &str) -> Result<(), KernelRefusal> {
    let Ok((edge, faces)) = edge_and_faces(solid, edge_id) else { return Ok(()) };
    if faces.len() != 2 || faces[0].id == faces[1].id {
        return Ok(());
    }
    let band = crate::blend::consumed_band(solid);
    const STATIONS: usize = 8;
    for (k, face) in faces.iter().enumerate() {
        if !is_plane(face) {
            continue;
        }
        let mut worst: Option<(f64, f64, f64)> = None; // (how far past, how far the face reaches, the setback)
        for step in 0..=STATIONS {
            // Stations inside the edge (its ends sit on the neighbouring faces' boundaries).
            let f = (step as f64 + 0.5) / (STATIONS + 1) as f64;
            let t = edge.t0 + (edge.t1 - edge.t0) * f;
            let d = setback(k, f);
            if !(d > 0.0) {
                continue;
            }
            let (Ok(point), Ok(tangent)) = (edge.curve.evaluate(t), edge.curve.unit_tangent(t, edge.t0, edge.t1)) else { continue };
            let Ok(into) = into_face_direction(face, point, tangent, point, tangent, 1e-3 * d.min(1.0)) else { continue };
            // On the face: on its carrier (a point past a planar patch
            // projects onto the patch's boundary, some way off) and inside
            // its trim.
            let on_face = |at: f64| -> Option<bool> {
                let probe = point.add(into.scale(at));
                let q = crate::project_point_to_surface(&face.surface, probe).ok()?;
                if q.distance > band {
                    return Some(false);
                }
                let class = crate::parameter_point_in_face(face, crate::Vec2 { x: q.u, y: q.v }, 1e-9).ok()?;
                Some(class != crate::PolygonClass::Outside)
            };
            let Some(fits) = on_face(d) else { continue };
            if !fits {
                // How far the face reaches along that line: back off until inside.
                let (mut lo, mut hi) = (0.0, d);
                for _ in 0..48 {
                    let m = 0.5 * (lo + hi);
                    if on_face(m).unwrap_or(false) { lo = m } else { hi = m }
                }
                let past = d - lo;
                if past > band && worst.is_none_or(|(w, _, _)| past > w) {
                    worst = Some((past, lo, d));
                }
            }
        }
        if let Some((_, reaches, d)) = worst {
            return Err(KernelRefusal::input(
                KernelStage::Classify,
                "support_extent",
                format!(
                    "{entry}: a setback of {d} is too large for the faces it must lie on — the chamfer would meet face {} \
                     {d:.6} from edge {edge_id}, but that face only reaches {reaches:.6} there; a chamfer plane cut past the \
                     face runs into the next wall, so what would be built is not a chamfer of this edge: reduce the distance",
                    face.id
                ),
            ));
        }
    }
    Ok(())
}

/// [`check_chamfer_extent`] for a rolling ball whose radius at the station
/// `fraction` is `radius(fraction)`: its contact on each planar face lies
/// `radius · cot(φ/2)` from the edge, φ the angle inside the material between
/// the faces (read at the edge's middle). Strict, as a chamfer's: a contact
/// past either face is no blend of the edge.
pub(super) fn check_ball_extent(solid: &BrepSolid, edge_id: u64, radius: &dyn Fn(f64) -> f64, entry: &str) -> Result<(), KernelRefusal> {
    let Ok((edge, faces)) = edge_and_faces(solid, edge_id) else { return Ok(()) };
    if faces.len() != 2 || !faces.iter().all(|f| is_plane(f)) {
        return Ok(());
    }
    let t = 0.5 * (edge.t0 + edge.t1);
    let Ok(point) = edge.curve.evaluate(t) else { return Ok(()) };
    let (Ok((_, n0)), Ok((_, n1))) = (super::analyze::face_plane_normal(faces[0], point), super::analyze::face_plane_normal(faces[1], point)) else { return Ok(()) };
    let inside = std::f64::consts::PI - n0.dot(n1).clamp(-1.0, 1.0).acos();
    if !(inside > 1e-9 && inside < std::f64::consts::PI - 1e-9) {
        return Ok(());
    }
    let cot = 1.0 / (0.5 * inside).tan();
    check_extent_by_station(solid, edge_id, &|_, f| radius(f) * cot, entry)
}
