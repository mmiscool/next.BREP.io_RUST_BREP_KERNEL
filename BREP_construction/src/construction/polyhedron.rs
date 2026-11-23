//! A closed polyhedron from its corners and its planar polygon faces.
//!
//! Each face is a plane patch sized to the face itself (its polygon and a
//! tenth of its extent around it), trimmed by the polygon's straight edges;
//! the faces are sewn along their shared edges. A patch sized to its face
//! keeps every quantity read through its parameters (area, moments) at the
//! precision of the face, wherever the face stands.

use crate::{face_from_surface_trimmed, make_line, solid_from_sheets, BrepSolid, KernelRefusal, KernelStage, NurbsSurface, OrRefuse, Vec3, Vec4};
use std::collections::HashMap;

/// The prefix of every refusal of this module.
pub const POLYHEDRON_REFUSAL: &str = "polyhedron";

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{POLYHEDRON_REFUSAL}: {message}"))
}

/// The closed polyhedron whose faces are the polygons `faces` over `points`:
/// each face a loop of point indices running counter-clockwise seen from
/// outside, flat (every corner within a billionth of the polyhedron's size
/// of its plane), its consecutive corners distinct.
///
/// Refuses fewer than four faces, a face of fewer than three corners, an
/// index past `points`, a face that is not flat or has no area, faces that
/// do not close (every edge must be used once each way) and faces that form
/// more than one closed part.
pub fn make_polyhedron_brep(points: &[Vec3], faces: &[Vec<usize>]) -> Result<BrepSolid, KernelRefusal> {
    if faces.len() < 4 {
        return Err(input("polyhedron_faces", format!("{} faces bound no solid", faces.len())));
    }
    if let Some(f) = faces.iter().position(|face| face.len() < 3) {
        return Err(input("polyhedron_face", format!("face {f} has fewer than three corners")));
    }
    if faces.iter().flatten().any(|&i| i >= points.len()) {
        return Err(input("polyhedron_index", "a face names a corner past the points".into()));
    }
    let used: Vec<Vec3> = faces.iter().flatten().map(|&i| points[i]).collect();
    let (mut low, mut high) = (used[0], used[0]);
    for p in &used {
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    let size = high.sub(low).length();
    if !(size.is_finite() && size > 0.0) {
        return Err(input("polyhedron_extent", "the corners have no extent".into()));
    }
    // A billionth of the size, and the round-off of the coordinates themselves.
    let reach = used.iter().fold(0.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs()));
    let flat = 1e-9 * size + 16.0 * f64::EPSILON * reach;
    let mut sides: HashMap<(usize, usize), usize> = HashMap::new();
    for (f, face) in faces.iter().enumerate() {
        if face.len() < 3 {
            return Err(input("polyhedron_face", format!("face {f} has fewer than three corners")));
        }
        for k in 0..face.len() {
            let (a, b) = (face[k], face[(k + 1) % face.len()]);
            if a == b || sides.insert((a, b), f).is_some() {
                return Err(input("polyhedron_closure", format!("face {f} repeats a corner or an edge another face runs the same way")));
            }
        }
    }
    if sides.keys().any(|&(a, b)| !sides.contains_key(&(b, a))) {
        return Err(input("polyhedron_closure", "the faces do not close: an edge has no face on its other side".into()));
    }
    // One polyhedron: every face reaches every other across edges (closed
    // parts apart could lie one through another).
    let mut reached = vec![false; faces.len()];
    let mut stack = vec![0usize];
    reached[0] = true;
    while let Some(f) = stack.pop() {
        let face = &faces[f];
        for k in 0..face.len() {
            let other = sides[&(face[(k + 1) % face.len()], face[k])];
            if !reached[other] {
                reached[other] = true;
                stack.push(other);
            }
        }
    }
    if reached.iter().any(|&r| !r) {
        return Err(input("polyhedron_parts", "the faces form more than one closed part".into()));
    }
    let line = |a: usize, b: usize| make_line(points[a], points[b]).or_refuse(KernelStage::Collect, "polyhedron.edge");
    let mut sheets = Vec::with_capacity(faces.len());
    for (f, face) in faces.iter().enumerate() {
        // Newell's normal: the polygon's area vector, along its outward normal.
        // Read about the face's own centre, so its size and not its place sets the round-off.
        let centre = face.iter().fold(Vec3::new(0.0, 0.0, 0.0), |s, &i| s.add(points[i])).scale(1.0 / face.len() as f64);
        let mut area = Vec3::new(0.0, 0.0, 0.0);
        for k in 0..face.len() {
            let (p, q) = (points[face[k]].sub(centre), points[face[(k + 1) % face.len()]].sub(centre));
            area = area.add(Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y)));
        }
        let length = area.length();
        if !(length > flat * flat) {
            return Err(input("polyhedron_face", format!("face {f} has no area")));
        }
        let n = area.scale(1.0 / length);
        if face.iter().any(|&i| points[i].sub(centre).dot(n).abs() > flat) {
            return Err(input("polyhedron_flat", format!("face {f} is not flat")));
        }
        // In-plane axes from the face's longest reach from its centre.
        let far = face.iter().map(|&i| points[i].sub(centre)).fold(Vec3::new(0.0, 0.0, 0.0), |a, b| if b.length() > a.length() { b } else { a });
        let e1 = far.sub(n.scale(far.dot(n))).scale(1.0 / far.length());
        let e2 = n.cross(e1);
        let (mut u0, mut u1, mut v0, mut v1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for &i in face {
            let d = points[i].sub(centre);
            (u0, u1, v0, v1) = (u0.min(d.dot(e1)), u1.max(d.dot(e1)), v0.min(d.dot(e2)), v1.max(d.dot(e2)));
        }
        let margin = 0.1 * (u1 - u0).max(v1 - v0);
        let (u0, u1, v0, v1) = (u0 - margin, u1 + margin, v0 - margin, v1 + margin);
        let corner = |u: f64, v: f64| Vec4::from_point(centre.add(e1.scale(u)).add(e2.scale(v)), 1.0);
        let plane = NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], vec![vec![corner(u0, v0), corner(u0, v1)], vec![corner(u1, v0), corner(u1, v1)]])
            .or_refuse(KernelStage::Collect, "polyhedron.face")?;
        let edges = (0..face.len()).map(|k| line(face[k], face[(k + 1) % face.len()])).collect::<Result<Vec<_>, _>>()?;
        sheets.push(face_from_surface_trimmed(&plane, &[edges])?);
    }
    solid_from_sheets(&sheets, flat)
}

