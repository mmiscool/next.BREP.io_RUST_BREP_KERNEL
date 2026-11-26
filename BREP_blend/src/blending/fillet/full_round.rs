//! A full round: the face `middle` lies on replaced by a round tangent to it
//! and to the faces `side_a` and `side_b` lie on, which stay where they are.
//!
//! Built on a prism (two planar end faces square to one direction, every other
//! face a plane along it, one loop each): the round is the arc of the circle
//! tangent to the three faces' lines in the section, from its contact on one
//! side through its contact on the middle face to its contact on the other, so
//! the section is the prism's with the middle face's segment and the parts of
//! the sides beyond the contacts replaced by that arc, extruded again between
//! the end faces. Anything else is refused by name.
use super::*;

const ENTRY: &str = "full_round";

fn unsupported(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{ENTRY}: {message}"))
}

/// A face's outward unit normal and a point on it, if it is a plane.
pub(super) fn plane_of(face: &FaceRecord) -> Option<(Vec3, Vec3)> {
    let s = &face.surface;
    if s.degree_u != 1 || s.degree_v != 1 {
        return None;
    }
    let pts: Vec<Vec3> = s.control_points.iter().flatten().filter_map(|c| c.point().ok()).collect();
    if pts.len() < 3 {
        return None;
    }
    let mut n = pts[1].sub(pts[0]).cross(pts[pts.len() - 1].sub(pts[0])).normalized().ok()?;
    let scale = pts.iter().map(|p| p.sub(pts[0]).length()).fold(0.0, f64::max).max(1.0);
    if pts.iter().any(|p| n.dot(p.sub(pts[0])).abs() > 1e-10 * scale) {
        return None;
    }
    let [u0, u1] = s.domain_u().ok()?;
    let [v0, v1] = s.domain_v().ok()?;
    if s.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)).ok()?.dot(n) < 0.0 {
        n = n.scale(-1.0);
    }
    Some((if face.same_sense { n } else { n.scale(-1.0) }, pts[0]))
}

fn face_at(solid: &BrepSolid, point: Vec3) -> Result<u64, KernelRefusal> {
    let size = solid.vertices.iter().map(|v| v.point.sub(solid.vertices[0].point).length()).fold(0.0, f64::max);
    let near = if size > 0.0 { 1e-6 * size } else { 1e-9 };
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

/// Replaces the face `middle` lies on by a round tangent to it and to the faces
/// `side_a` and `side_b` lie on (see the module's account).
pub fn full_round(solid: &BrepSolid, side_a: Vec3, middle: Vec3, side_b: Vec3, name: Option<&str>) -> Result<BrepSolid, KernelRefusal> {
    let _operation = crate::blend::BlendOperation::enter();
    if solid.shells.len() != 1 {
        return Err(unsupported("full_round_shells", "the solid has more than one shell".into()));
    }
    let faces = &solid.shells[0].faces;
    let ids = [face_at(solid, side_a)?, face_at(solid, middle)?, face_at(solid, side_b)?];
    if ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2] {
        return Err(KernelRefusal::input(KernelStage::Collect, "full_round_faces", format!("{ENTRY}: the three points must lie on three faces")));
    }
    let by_id = |id: u64| faces.iter().find(|f| f.id == id).expect("face");
    let planes: Vec<(Vec3, Vec3)> = ids.iter().map(|&id| plane_of(by_id(id))).collect::<Option<_>>().ok_or_else(|| unsupported("full_round_planes", "the three faces must be planes".into()))?;
    let e = planes[0].0.cross(planes[1].0).normalized().map_err(|_| unsupported("full_round_parallel", "the side and middle faces are parallel".into()))?;
    if planes[2].0.dot(e).abs() > 1e-9 {
        return Err(unsupported("full_round_prism", "the three faces do not run along one direction".into()));
    }
    // A prism along e: two end faces square to it, every other face along it.
    let mut ends: Vec<(usize, Vec3, Vec3)> = Vec::new();
    for (index, face) in faces.iter().enumerate() {
        let (n, on) = plane_of(face).ok_or_else(|| unsupported("full_round_prism", "the solid is not a prism: a face is not a plane".into()))?;
        if face.loops.len() != 1 {
            return Err(unsupported("full_round_prism", "the solid is not a prism: a face has a hole".into()));
        }
        if (n.dot(e).abs() - 1.0).abs() <= 1e-9 {
            ends.push((index, n, on));
        } else if n.dot(e).abs() > 1e-9 {
            return Err(unsupported("full_round_prism", "the solid is not a prism: a face is neither along the direction nor square to it".into()));
        }
    }
    if ends.len() != 2 || ends[0].1.dot(ends[1].1) > 0.0 {
        return Err(unsupported("full_round_prism", "the solid is not a prism: it has no two opposite end faces".into()));
    }
    // Each end face square to the direction exactly: all its vertices at one
    // height along it (a face sloped by a hair is not a prism's end).
    let size = solid.vertices.iter().map(|v| v.point.sub(solid.vertices[0].point).length()).fold(0.0, f64::max).max(1.0);
    for &(index, _, on) in &ends {
        let heights: Vec<f64> = faces[index]
            .loops
            .iter()
            .flat_map(|l| l.coedges.iter())
            .filter_map(|c| solid.edges.iter().find(|x| x.id == c.edge_id))
            .flat_map(|x| [x.start_vertex_id, x.end_vertex_id])
            .filter_map(|id| solid.vertices.iter().find(|v| v.id == id))
            .map(|v| v.point)
            .collect::<Vec<Vec3>>()
            .iter()
            .map(|p| e.dot(p.sub(on)))
            .collect();
        // (to a millionth of a millionth of the end face's own width, not the solid's length)
        let width = faces[index]
            .loops
            .iter()
            .flat_map(|l| l.coedges.iter())
            .filter_map(|c| solid.edges.iter().find(|x| x.id == c.edge_id))
            .filter_map(|x| Some(x.curve.evaluate(x.t0).ok()?.sub(x.curve.evaluate(x.t1).ok()?).length()))
            .fold(0.0, f64::max)
            .max(1e-300);
        if heights.iter().any(|h| h.abs() > 1e-12 * width) {
            return Err(unsupported("full_round_prism", "the solid is not a prism: an end face is not square to the direction".into()));
        }
    }
    // The end face facing −e is the profile's; the other is `length` along e.
    let (start, finish) = if ends[0].1.dot(e) < 0.0 { (ends[0], ends[1]) } else { (ends[1], ends[0]) };
    let length = e.dot(finish.2.sub(start.2));
    if !(length > 0.0) {
        return Err(unsupported("full_round_prism", "the end faces stand the wrong way".into()));
    }
    // The profile: the start face's loop as a polygon, every edge straight.
    let face = &faces[start.0];
    let mut points: Vec<Vec3> = Vec::new();
    let mut owners: Vec<u64> = Vec::new(); // the side face along each segment
    for coedge in &face.loops[0].coedges {
        let edge = solid.edges.iter().find(|x| x.id == coedge.edge_id).ok_or_else(|| unsupported("full_round_prism", "a loop edge is missing".into()))?;
        if edge.curve.degree != 1 || edge.curve.control_points.len() != 2 {
            return Err(unsupported("full_round_prism", "the solid is not a prism on a polygon".into()));
        }
        let a = edge.curve.evaluate(if coedge.forward { edge.t0 } else { edge.t1 }).or_refuse(KernelStage::Classify, "evaluate")?;
        points.push(a);
        let other = faces.iter().find(|f| f.id != face.id && f.loops.iter().flat_map(|l| l.coedges.iter()).any(|c| c.edge_id == edge.id)).map(|f| f.id).unwrap_or(0);
        owners.push(other);
    }
    let n = points.len();
    let Some(mi) = owners.iter().position(|&o| o == ids[1]) else { return Err(unsupported("full_round_faces", "the middle face does not meet the end face".into())) };
    let (prev, next) = ((mi + n - 1) % n, (mi + 1) % n);
    let (ia, ib) = if owners[prev] == ids[0] && owners[next] == ids[2] {
        (prev, next)
    } else if owners[prev] == ids[2] && owners[next] == ids[0] {
        (prev, next)
    } else {
        return Err(unsupported("full_round_faces", "the side faces do not both border the middle face".into()));
    };
    // In the section plane: the circle tangent to the three lines on the
    // material's side (each inward normal times the radius from its line).
    let inward = |k: usize| -> Vec3 {
        let id = owners[k];
        let (n, _) = plane_of(faces.iter().find(|f| f.id == id).expect("face")).expect("planar");
        n.scale(-1.0)
    };
    let on_line = |k: usize| points[k];
    let rows: Vec<(Vec3, Vec3)> = [ia, mi, ib].iter().map(|&k| (inward(k), on_line(k))).collect();
    // Solve nᵢ·c − ρ = nᵢ·pᵢ in the plane's 2D frame.
    let x_axis = rows[1].0.cross(e).normalized().or_refuse(KernelStage::Classify, "normalized")?;
    let y_axis = e.cross(x_axis);
    let origin = points[mi];
    let to2 = |v: Vec3| [v.dot(x_axis), v.dot(y_axis)];
    let m: Vec<[f64; 4]> = rows.iter().map(|(nv, p)| { let n2 = to2(*nv); [n2[0], n2[1], -1.0, n2[0] * to2(p.sub(origin))[0] + n2[1] * to2(p.sub(origin))[1]] }).collect();
    let det3 = |a: [[f64; 3]; 3]| a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    let a = [[m[0][0], m[0][1], m[0][2]], [m[1][0], m[1][1], m[1][2]], [m[2][0], m[2][1], m[2][2]]];
    let d = det3(a);
    if d.abs() < 1e-12 {
        return Err(unsupported("full_round_circle", "no circle touches the three faces".into()));
    }
    let column = |c: usize| { let mut b = a; for r in 0..3 { b[r][c] = m[r][3]; } det3(b) / d };
    let (cx, cy, rho) = (column(0), column(1), column(2));
    if !(rho > 0.0) {
        return Err(unsupported("full_round_circle", "the circle touching the three faces lies outside the solid".into()));
    }
    let centre = origin.add(x_axis.scale(cx)).add(y_axis.scale(cy));
    let touch = |k: usize| { let (nv, _) = rows[k]; centre.sub(nv.scale(rho)) };
    let (ta, tm, tb) = (touch(0), touch(1), touch(2));
    // Each contact on its own segment of the profile.
    let within = |p: Vec3, a: Vec3, b: Vec3| { let ab = b.sub(a); let f = p.sub(a).dot(ab) / ab.dot(ab); f >= -1e-9 && f <= 1.0 + 1e-9 };
    if !(within(ta, points[ia], points[(ia + 1) % n]) && within(tm, points[mi], points[(mi + 1) % n]) && within(tb, points[ib], points[(ib + 1) % n])) {
        return Err(unsupported("full_round_contacts", "the round's contacts do not all lie on their faces".into()));
    }
    // The round's arc stays clear of every segment the section keeps (a notch
    // or a wall reaching in under it would be crossed): no point of such a
    // segment lies inside the circle within the arc's own sweep.
    let (arc_u, arc_w, arc_sweep) = {
        let (ra, rm, rb) = (ta.sub(centre), tm.sub(centre), tb.sub(centre));
        let u = ra.normalized().or_refuse(KernelStage::Fragment, "normalized")?;
        let mut w = e.cross(u);
        let angle = |r: Vec3, w: Vec3| { let a = r.dot(w).atan2(r.dot(u)); if a < 0.0 { a + std::f64::consts::TAU } else { a } };
        if angle(rm, w) > angle(rb, w) {
            w = w.scale(-1.0);
        }
        let sweep = angle(rb, w);
        (u, w, sweep)
    };
    for k in 0..n {
        if k == ia || k == mi || k == ib {
            continue;
        }
        let (a, b) = (points[k], points[(k + 1) % n]);
        for j in 0..=512 {
            let p = a.add(b.sub(a).scale(j as f64 / 512.0));
            let r = p.sub(centre);
            if r.length() >= rho * (1.0 - 1e-9) {
                continue;
            }
            let at = r.dot(arc_w).atan2(r.dot(arc_u));
            let at = if at < 0.0 { at + std::f64::consts::TAU } else { at };
            if at > 1e-9 && at < arc_sweep - 1e-9 {
                return Err(unsupported("full_round_clearance", format!("the round of radius {rho:.6} would cross another face of the section")));
            }
        }
    }
    // The new section: ... points[ia] -> ta -> arc through tm -> tb -> points[ib+1] ...
    let mut curves: Vec<NurbsCurve> = Vec::new();
    let line = |p: Vec3, q: Vec3| make_line(p, q).or_refuse(KernelStage::Fragment, "make_line");
    let mut k = (ib + 1) % n;
    // From the far end of the second side round to the first side's start.
    while k != ia {
        let next_k = (k + 1) % n;
        curves.push(line(points[k], points[next_k])?);
        k = next_k;
    }
    if points[ia].sub(ta).length() > 1e-12 {
        curves.push(line(points[ia], ta)?);
    }
    // The arc from ta through tm to tb, about the centre in the section plane.
    let (ra, rm, rb) = (ta.sub(centre), tm.sub(centre), tb.sub(centre));
    let u = ra.normalized().or_refuse(KernelStage::Fragment, "normalized")?;
    let mut w = e.cross(u);
    let angle = |r: Vec3, w: Vec3| { let a = r.dot(w).atan2(r.dot(u)); if a < 0.0 { a + std::f64::consts::TAU } else { a } };
    if angle(rm, w) > angle(rb, w) {
        w = w.scale(-1.0);
    }
    let sweep = angle(rb, w);
    curves.push(crate::make_arc(centre, u, w, rho, 0.0, sweep).or_refuse(KernelStage::Fragment, "make_arc")?);
    let after = (ib + 1) % n;
    if tb.sub(points[after]).length() > 1e-12 {
        curves.push(line(tb, points[after])?);
    }
    let mut result = extrude_profile_brep(&curves, e, length)?;
    // Names: each planar face takes the name of the solid's face on its plane;
    // the round takes `name`.
    let old: Vec<(Vec3, Vec3, Option<String>)> = faces.iter().filter_map(|f| plane_of(f).map(|(n, on)| (n, on, f.name.clone()))).collect();
    for face in &mut result.shells[0].faces {
        match plane_of(face) {
            Some((n, on)) => {
                if let Some((_, _, old_name)) = old.iter().find(|(m, at, _)| m.dot(n) > 1.0 - 1e-9 && m.dot(on.sub(*at)).abs() <= 1e-9 * size) {
                    face.name = old_name.clone();
                }
            }
            None => {
                if let Some(name) = name {
                    face.name = Some(name.to_string());
                }
            }
        }
    }
    if result.validate().iter().any(|i| i.severity == "error") {
        return Err(KernelRefusal::internal(KernelStage::Validate, "full_round_result", format!("{ENTRY}: the rebuilt prism does not validate")));
    }
    Ok(result)
}
