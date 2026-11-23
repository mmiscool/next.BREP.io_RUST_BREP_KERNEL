//! A fillet between two faces that need not share an edge: a ball of `radius`
//! rolling while it touches both, what lies outside its path between its
//! contacts going, faces between them included.
//!
//! Built where both faces are planes and the faces between them (each sharing
//! an edge with both) are planes too, ending on planes square to the line the
//! two faces' planes meet in (a prism's chamfered or faceted corner): the
//! material between those faces and that line is put back (a triangular prism
//! on the line and the far edges of the faces between, united and its
//! coplanar faces merged), and the sharp edge that leaves on the line is
//! filleted at `radius`. Where the ball's contacts fall short of the faces
//! between (on them, not beyond), the ball does not touch the two faces alone
//! and the call is refused by name, as it is for any other configuration.
use super::*;

const ENTRY: &str = "fillet_faces";

fn unsupported(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{ENTRY}: {message}"))
}

/// The outward unit normal of a planar face, if it is one.
fn outward_plane_normal(face: &FaceRecord) -> Option<Vec3> {
    let s = &face.surface;
    if s.degree_u != 1 || s.degree_v != 1 {
        return None;
    }
    let pts: Vec<Vec3> = s.control_points.iter().flatten().filter_map(|c| c.point().ok()).collect();
    if pts.len() < 3 {
        return None;
    }
    let n = pts[1].sub(pts[0]).cross(pts[pts.len() - 1].sub(pts[0]));
    let n = if n.length() > 0.0 { n } else { pts[2].sub(pts[0]).cross(pts[1].sub(pts[0])) };
    let mut n = n.normalized().ok()?;
    let scale = pts.iter().map(|p| p.sub(pts[0]).length()).fold(0.0, f64::max).max(1.0);
    if pts.iter().any(|p| n.dot(p.sub(pts[0])).abs() > 1e-10 * scale) {
        return None;
    }
    // The carrier's own normal, turned by the face's sense.
    let [u0, u1] = s.domain_u().ok()?;
    let [v0, v1] = s.domain_v().ok()?;
    let carrier = s.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)).ok()?;
    if carrier.dot(n) < 0.0 {
        n = n.scale(-1.0);
    }
    Some(if face.same_sense { n } else { n.scale(-1.0) })
}

/// The face the point lies on.
fn face_at(solid: &BrepSolid, point: Vec3) -> Result<u64, KernelRefusal> {
    // Within a millionth of the solid's own size (its vertices' box), wherever it stands.
    let (lo, hi) = solid.vertices.iter().fold((Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY)), |(lo, hi), v| {
        (Vec3::new(lo.x.min(v.point.x), lo.y.min(v.point.y), lo.z.min(v.point.z)), Vec3::new(hi.x.max(v.point.x), hi.y.max(v.point.y), hi.z.max(v.point.z)))
    });
    let size = hi.sub(lo).length();
    let near = if size.is_finite() && size > 0.0 { 1e-6 * size } else { 1e-9 };
    let mut best: Option<(f64, u64)> = None;
    for face in solid.shells.iter().flat_map(|s| s.faces.iter()) {
        let Ok(q) = crate::project_point_to_surface(&face.surface, point) else { continue };
        if q.distance > near {
            continue;
        }
        let inside = crate::classification::parameter_point_in_face(face, crate::Vec2 { x: q.u, y: q.v }, 1e-9).map(|c| c != crate::classification::PolygonClass::Outside).unwrap_or(false);
        if inside && best.map_or(true, |b| q.distance < b.0) {
            best = Some((q.distance, face.id));
        }
    }
    best.map(|b| b.1).ok_or_else(|| KernelRefusal::input(KernelStage::Collect, "face_point", format!("{ENTRY}: no face of the solid holds the point {point:?}")))
}

fn face_vertices(solid: &BrepSolid, face: &FaceRecord) -> Vec<Vec3> {
    let mut ids: Vec<u64> = Vec::new();
    for c in face.loops.iter().flat_map(|l| l.coedges.iter()) {
        if let Some(e) = solid.edges.iter().find(|e| e.id == c.edge_id) {
            for v in [e.start_vertex_id, e.end_vertex_id] {
                if !ids.contains(&v) {
                    ids.push(v);
                }
            }
        }
    }
    ids.iter().filter_map(|id| solid.vertices.iter().find(|v| v.id == *id).map(|v| v.point)).collect()
}

fn face_edges(face: &FaceRecord) -> Vec<u64> {
    face.loops.iter().flat_map(|l| l.coedges.iter().map(|c| c.edge_id)).collect()
}

/// Rounds between the faces `point_a` and `point_b` lie on by a ball of `radius`
/// (see the module's account of what is built and what is refused).
pub fn fillet_faces(solid: &BrepSolid, point_a: Vec3, point_b: Vec3, radius: f64, name: Option<&str>) -> Result<BrepSolid, KernelRefusal> {
    let _operation = crate::blend::BlendOperation::enter();
    if !(radius > 0.0) || !radius.is_finite() {
        return Err(KernelRefusal::input(KernelStage::Collect, "fillet_faces_radius", format!("{ENTRY}: the radius must be positive")));
    }
    if solid.shells.len() != 1 {
        return Err(unsupported("fillet_faces_shells", "the solid has more than one shell".into()));
    }
    let (ia, ib) = (face_at(solid, point_a)?, face_at(solid, point_b)?);
    if ia == ib {
        return Err(KernelRefusal::input(KernelStage::Collect, "fillet_faces_same", format!("{ENTRY}: both points lie on one face")));
    }
    let faces = &solid.shells[0].faces;
    let (fa, fb) = (faces.iter().find(|f| f.id == ia).expect("face"), faces.iter().find(|f| f.id == ib).expect("face"));
    let (na, nb) = match (outward_plane_normal(fa), outward_plane_normal(fb)) {
        (Some(a), Some(b)) => (a, b),
        _ => return Err(unsupported("fillet_faces_planes", "both faces must be planes".into())),
    };
    let e = na.cross(nb);
    if e.length() < 1e-9 {
        return Err(unsupported("fillet_faces_parallel", "the faces' planes do not meet".into()));
    }
    let e = e.normalized().or_refuse(KernelStage::Classify, "normalized")?;
    let (va, vb) = (face_vertices(solid, fa), face_vertices(solid, fb));
    let (Some(&qa), Some(&qb)) = (va.first(), vb.first()) else {
        return Err(unsupported("fillet_faces_vertices", "a face has no vertices".into()));
    };
    // The line the planes meet in: K with na·K = na·qa, nb·K = nb·qb, e·K = e·(middle of the faces).
    let middle = va.iter().chain(&vb).fold(Vec3::new(0.0, 0.0, 0.0), |s, p| s.add(*p)).scale(1.0 / (va.len() + vb.len()) as f64);
    let (da, db, de) = (na.dot(qa), nb.dot(qb), e.dot(middle));
    let denominator = na.dot(nb.cross(e));
    let k = nb.cross(e).scale(da).add(e.cross(na).scale(db)).add(na.cross(nb).scale(de)).scale(1.0 / denominator);
    let scale = va.iter().chain(&vb).map(|p| p.sub(k).length()).fold(0.0, f64::max).max(1.0);
    let tol = 1e-9 * scale;
    // Each face's direction away from the line, in its plane.
    let away = |n: Vec3, verts: &[Vec3]| -> Option<Vec3> {
        let d = e.cross(n).normalized().ok()?;
        let reach = verts.iter().map(|p| d.dot(p.sub(k))).fold(0.0, |a: f64, b| if b.abs() > a.abs() { b } else { a });
        Some(if reach < 0.0 { d.scale(-1.0) } else { d })
    };
    let (Some(ua), Some(ub)) = (away(na, &va), away(nb, &vb)) else { return Err(unsupported("fillet_faces_planes", "the faces' directions from their line are undefined".into())) };
    // A convex corner: each face's outward normal points away from the other face.
    if na.dot(ub) > -1e-9 || nb.dot(ua) > -1e-9 {
        return Err(unsupported("fillet_faces_concave", "the faces meet in a concave corner; only a convex one is rounded here".into()));
    }
    // The faces between: those sharing an edge with each and running along
    // the line.
    let (ea, eb) = (face_edges(fa), face_edges(fb));
    // (running along the line: an end face shares an edge with each too, but
    // stands square to it)
    let between: Vec<&FaceRecord> = faces
        .iter()
        .filter(|f| f.id != ia && f.id != ib)
        .filter(|f| {
            let own = face_edges(f);
            own.iter().any(|x| ea.contains(x)) && own.iter().any(|x| eb.contains(x))
        })
        .filter(|f| outward_plane_normal(f).is_none_or(|n| n.dot(e).abs() < 1e-6))
        .collect();
    if between.is_empty() {
        return Err(unsupported("fillet_faces_between", "no face lies between the two (they share an edge: fillet that edge)".into()));
    }
    let mut far = [0.0f64; 2];
    let (mut tmin, mut tmax) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut corners: Vec<f64> = Vec::new();
    for face in &between {
        if outward_plane_normal(face).is_none() {
            return Err(unsupported("fillet_faces_between", "a face between the two is not a plane".into()));
        }
        for p in face_vertices(solid, face) {
            let r = p.sub(k);
            // Within the wedge between the two planes, toward the solid.
            if na.dot(r) > tol || nb.dot(r) > tol {
                return Err(unsupported("fillet_faces_between", "a face between the two reaches past their planes".into()));
            }
            if na.dot(r).abs() <= tol {
                far[0] = far[0].max(ua.dot(r));
            }
            if nb.dot(r).abs() <= tol {
                far[1] = far[1].max(ub.dot(r));
            }
            let t = e.dot(r);
            tmin = tmin.min(t);
            tmax = tmax.max(t);
            corners.push(t);
        }
    }
    // The faces between end on two planes square to the line.
    if corners.iter().any(|&t| (t - tmin).abs() > tol && (t - tmax).abs() > tol) || !(tmax - tmin > tol) {
        return Err(unsupported("fillet_faces_ends", "the faces between do not end on planes square to the line the two faces meet in".into()));
    }
    if !(far[0] > tol && far[1] > tol) {
        return Err(unsupported("fillet_faces_between", "the faces between do not reach each face".into()));
    }
    // The ball's contacts must clear the faces between, and the ball itself
    // must stand clear of each of their planes (its centre on the bisector,
    // `radius / sin(θ/2)` from the line, at least `radius` from each).
    let theta = ua.dot(ub).clamp(-1.0, 1.0).acos();
    let contact = radius / (0.5 * theta).tan();
    if !(contact > far[0].max(far[1]) * (1.0 + 1e-9)) {
        return Err(unsupported(
            "fillet_faces_contact",
            format!("a ball of radius {radius} touches the faces {contact:.6} from their line, not past the faces between ({:.6}); it would touch those too", far[0].max(far[1])),
        ));
    }
    let bisector = ua.add(ub).normalized().or_refuse(KernelStage::Classify, "normalized")?;
    let centre = k.add(bisector.scale(radius / (0.5 * theta).sin()));
    for face in &between {
        let n = outward_plane_normal(face).expect("planar");
        let on = face_vertices(solid, face).first().copied().unwrap_or(k);
        let clearance = n.dot(on.sub(centre));
        if !(clearance >= radius * (1.0 - 1e-9)) {
            return Err(unsupported(
                "fillet_faces_contact",
                format!("a ball of radius {radius} between the faces stands {clearance:.6} from a face between them; it would touch it, not pass it"),
            ));
        }
    }
    // Put back the material between the faces between and the line, merge the
    // coplanar faces that leaves, and round the sharp edge on the line.
    let base = k.add(e.scale(tmin));
    let triangle = [base, base.add(ua.scale(far[0])), base.add(ub.scale(far[1]))];
    let curves: Vec<NurbsCurve> = (0..3).map(|i| make_line(triangle[i], triangle[(i + 1) % 3]).or_refuse(KernelStage::Fragment, "make_line")).collect::<Result<_, _>>()?;
    let wedge = extrude_profile_brep(&curves, e, tmax - tmin)?;
    let united = boolean_operation(solid, &wedge, crate::BooleanOperation::Union, &Default::default())?;
    let merged = crate::merge_same_surface_faces(&united, 1e-7).unwrap_or(united);
    let on_line = k.add(e.scale(0.5 * (tmin + tmax)));
    let result = fillet_edges(&merged, &[on_line], None, radius, false, name)?;
    Ok(result)
}
