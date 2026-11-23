//! A symmetric CHAMFER of one straight edge of a CONVEX POLYHEDRON, as one
//! plane clip: `BREP_CHAMFER_PLANAR_CLIP` (default off; `check` builds both
//! this answer and the cutter composition's and compares them).
//!
//! A body whose faces are all planes, each with one loop, and which is exactly
//! the convex polyhedron of those planes, chamfered along an edge between two
//! of them is that polyhedron with one more plane: the chord plane through the
//! two lines where the rolling ball of the chamfer's `radius` touches the two
//! faces, `radius / tan(θ/2)` from the edge on each (θ the faces' dihedral
//! angle inside the material). It is built directly from the planes
//! (`polyhedron_of_planes`, `solid_from_polygons`), with no cutter and no
//! boolean.
//!
//! The clip is the chamfer only where it stays at the edge: it removes no face
//! of the body, leaves no two corners within the sliver band (a strip the
//! composition snaps onto its edge), and every corner of the new face lies within three setbacks of
//! the edge's segment, so it ends on the faces at the edge's ends, mitring with
//! them (the chamfers made there before it among them) as the cutter's does. Anything else declines and the composition answers.
//!
//! Every face the chamfer keeps keeps its id and name; the chamfer face takes
//! the next id past the body's faces, as the composition gives it, and the
//! chamfer's name. Edges, coedges, loops and vertices are new.
use crate::analytic_surface::AnalyticSurface;
use crate::{BrepSolid, Vec3};

/// `BREP_CHAMFER_PLANAR_CLIP`: on by default; `=0` answers from the
/// composition; `check` answers from the composition and reports where the
/// two disagree.
pub(crate) fn planar_clip_mode() -> crate::env_read::Mode {
    crate::mode_switch!("BREP_CHAMFER_PLANAR_CLIP", true)
}

/// A face's outward plane `(n, d)`, `n · p = d`, when it is a plane with one
/// loop.
fn outward_plane(face: &crate::FaceRecord) -> Option<(Vec3, f64)> {
    if face.loops.len() != 1 {
        return None;
    }
    match face.surface.analytic()? {
        AnalyticSurface::Plane { origin, u_dir, v_dir, .. } => {
            let mut normal = u_dir.cross(*v_dir).normalized().ok()?;
            if !face.same_sense {
                normal = normal.scale(-1.0);
            }
            Some((normal, normal.dot(*origin)))
        }
        _ => None,
    }
}

/// The body `solid` with the edge through `point` chamfered at `radius` by
/// one plane clip, or `None` (decline) where this lane does not answer.
pub(crate) fn chamfer_by_plane_clip(solid: &BrepSolid, point: Vec3, radius: f64, name: Option<&str>) -> Option<BrepSolid> {
    match clip(solid, point, radius, name) {
        Ok(body) => Some(body),
        Err(why) => {
            if std::env::var_os("BREP_DEBUG_PLANAR_CLIP").is_some() {
                eprintln!("planar clip declined: {why}");
            }
            None
        }
    }
}

fn clip(solid: &BrepSolid, point: Vec3, radius: f64, name: Option<&str>) -> Result<BrepSolid, &'static str> {
    if solid.shells.len() != 1 || !(radius > 0.0) || !radius.is_finite() {
        return Err("not one shell, or no radius");
    }
    let faces = &solid.shells[0].faces;
    let planes: Vec<(Vec3, f64)> = faces.iter().map(outward_plane).collect::<Option<_>>().ok_or("a face is not a plane with one loop")?;
    let scale = crate::solid_scale(solid).max(1.0);
    let tolerance = 1e-9 * scale;
    // The body IS the polyhedron of its planes: one face per plane, and its
    // corners the body's vertices.
    let (corners, loops) = brep_construction::solid_sweep_caps::polyhedron_of_planes(&planes).map_err(|_| "no polyhedron of the planes")?;
    if loops.len() != faces.len() || corners.len() != solid.vertices.len() {
        return Err("the body is not the polyhedron of its planes (counts)");
    }
    if !solid.vertices.iter().all(|v| corners.iter().any(|c| c.sub(v.point).length() <= tolerance)) {
        return Err("the body is not the polyhedron of its planes (corners)");
    }
    // The edge and its two faces.
    let edge_id = super::edges::resolve_edge_by_point(solid, point).map_err(|_| "no edge at the point")?;
    let edge = solid.edges.iter().find(|e| e.id == edge_id).ok_or("no edge")?;
    let users: Vec<usize> = faces
        .iter()
        .enumerate()
        .filter(|(_, f)| f.loops.iter().flat_map(|l| &l.coedges).any(|c| c.edge_id == edge_id))
        .map(|(index, _)| index)
        .collect();
    let [a, b] = users[..] else { return Err("the edge does not join two faces") };
    let start = solid.vertices.iter().find(|v| v.id == edge.start_vertex_id).ok_or("no start vertex")?.point;
    let end = solid.vertices.iter().find(|v| v.id == edge.end_vertex_id).ok_or("no end vertex")?.point;
    let along = end.sub(start).normalized().map_err(|_| "a degenerate edge")?;
    let (na, nb) = (planes[a].0, planes[b].0);
    // A straight edge on both planes.
    if [start, end].iter().any(|p| (na.dot(*p) - planes[a].1).abs() > tolerance || (nb.dot(*p) - planes[b].1).abs() > tolerance) {
        return Err("the edge is not on both planes");
    }
    // The dihedral angle inside the material and the ball's setback.
    let cosine = na.dot(nb).clamp(-1.0, 1.0);
    let inside = std::f64::consts::PI - cosine.acos();
    let setback = radius / (0.5 * inside).tan();
    if !(setback > tolerance) || !setback.is_finite() {
        return Err("no setback");
    }
    // Into face a, square to the edge: away from face b's plane.
    let mut into_a = along.cross(na).normalized().map_err(|_| "no direction into the face")?;
    if into_a.dot(nb) > 0.0 {
        into_a = into_a.scale(-1.0);
    }
    let chord = na.add(nb).normalized().map_err(|_| "the faces are opposite")?;
    let middle = start.add(end).scale(0.5);
    let offset = chord.dot(middle.add(into_a.scale(setback)));
    let mut clipped = planes.clone();
    clipped.push((chord, offset));
    let (corners, loops) = brep_construction::solid_sweep_caps::polyhedron_of_planes(&clipped).map_err(|_| "no clipped polyhedron")?;
    // No face removed: one face per plane, the chord's among them.
    if loops.len() != clipped.len() {
        return Err("the clip removes a face");
    }
    // No sliver: two corners within the sliver band of each other are a strip
    // the composition snaps onto its edge (or refuses), not a face to keep.
    let band = crate::KernelTolerances::for_solid(solid, 1e-7).sliver;
    for i in 0..corners.len() {
        for j in i + 1..corners.len() {
            if corners[i].sub(corners[j]).length() <= band {
                return Err("the clip leaves a sliver");
            }
        }
    }
    // The chord face stays at the edge: every corner of it within three
    // setbacks of the edge's segment (its long sides lie one setback off it;
    // its ends mitre with the faces there, the chamfers made before it among
    // them). A clip reaching further is not this edge's chamfer alone.
    let reach = 3.0 * setback + tolerance;
    let on = |p: Vec3, index: usize| (clipped[index].0.dot(p) - clipped[index].1).abs() <= tolerance;
    let chord_index = clipped.len() - 1;
    let chord_face = loops.iter().find(|l| l.iter().all(|&i| on(corners[i], chord_index))).ok_or("no chord face")?;
    let length = end.sub(start).length();
    for &corner in chord_face {
        let q = corners[corner].sub(start);
        let t = q.dot(along).clamp(0.0, length);
        if q.sub(along.scale(t)).length() > reach {
            return Err("the chord face reaches past the edge's ends");
        }
    }
    let polygons: Vec<Vec<Vec3>> = loops.iter().map(|l| l.iter().map(|&i| corners[i]).collect()).collect();
    let mut body = crate::solid_from_polygons(&polygons).map_err(|_| "the polygons do not sew")?;
    if body.shells.len() != 1 || body.shells[0].faces.len() != clipped.len() {
        return Err("the sewn body has another face count");
    }
    // Faces take the kept faces' ids and names; the chord the next id.
    let next_face = faces.iter().map(|f| f.id).max().ok_or("no faces")? + 1;
    let mut used = vec![false; clipped.len()];
    for face in &mut body.shells[0].faces {
        let (normal, level) = outward_plane(face).ok_or("a sewn face is not a plane")?;
        let index = (0..clipped.len()).find(|&i| !used[i] && clipped[i].0.dot(normal) > 1.0 - 1e-12 && (clipped[i].1 - level).abs() <= tolerance).ok_or("a sewn face matches no plane")?;
        used[index] = true;
        if index == chord_index {
            face.id = next_face;
            face.name = name.map(str::to_string);
        } else {
            face.id = faces[index].id;
            face.name = faces[index].name.clone();
        }
    }
    // Everything else numbered past the body's own.
    let next = solid
        .edges
        .iter()
        .map(|e| e.id)
        .chain(solid.vertices.iter().map(|v| v.id))
        .chain(faces.iter().flat_map(|f| &f.loops).flat_map(|l| std::iter::once(l.id).chain(l.coedges.iter().map(|c| c.id))))
        .max()
        .unwrap_or(0)
        + 1;
    for vertex in &mut body.vertices {
        vertex.id += next;
    }
    for edge in &mut body.edges {
        edge.id += next;
        edge.start_vertex_id += next;
        edge.end_vertex_id += next;
    }
    for face in &mut body.shells[0].faces {
        for lp in &mut face.loops {
            lp.id += next;
            for coedge in &mut lp.coedges {
                coedge.id += next;
                coedge.edge_id += next;
            }
        }
    }
    body.id = solid.id;
    body.shells[0].id = solid.shells[0].id;
    if body.validate().is_empty() { Ok(body) } else { Err("the clipped body does not validate") }
}
