//! A solid as a viewer draws it: each face meshed on its own with the exact
//! surface's outward normal at every vertex, and every edge as a polyline
//! with the faces it bounds. Built from the watertight tessellation (so the
//! faces' boundaries meet vertex for vertex and the mesh they make closes)
//! before any normal smoothing: each vertex's normal is read from its own
//! face's surface.
//!
//! A vertex where the face's normal is not continuous (a crease inside one
//! face, a pole, an apex) is given one copy per triangle, with the normal
//! along that triangle. A crease is told from curvature by three probes into
//! each triangle; one shallower than their second difference (about the
//! surface's curvature times the probes' step squared: below 1e-7 of a
//! radian for chords at a display tolerance) may share one side's normal.

use crate::{BrepSolid, KernelRefusal, KernelStage, Vec3};
use std::collections::HashMap;

/// One face of a [`SolidDisplay`]: its own vertices, the surface's outward
/// normal at each, and its triangles over them (counter-clockwise outward).
#[derive(Clone, Debug, Default)]
pub struct DisplayFace {
    /// The face's id in the solid.
    pub face_id: u64,
    pub positions: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
}

/// One edge of a [`SolidDisplay`]: its polyline (the same samples the faces'
/// boundaries use) and the indices into [`SolidDisplay::faces`] of the faces
/// it bounds, one per use.
#[derive(Clone, Debug, Default)]
pub struct DisplayEdge {
    /// The edge's id in the solid.
    pub edge_id: u64,
    pub points: Vec<Vec3>,
    pub faces: Vec<u32>,
}

/// A solid as a viewer draws it (see the module docs). Seams, where a face
/// meets itself, are kept apart from the edges between faces.
#[derive(Clone, Debug, Default)]
pub struct SolidDisplay {
    pub faces: Vec<DisplayFace>,
    pub edges: Vec<DisplayEdge>,
    pub seams: Vec<Vec<Vec3>>,
}

/// The face's unit normal at `q` on it (by its recorded sense), or None
/// where its surface's parameter derivatives give none (their cross product
/// zero or unread).
fn surface_normal(face: &crate::FaceRecord, q: Vec3) -> Option<Vec3> {
    let hit = crate::project_point_to_surface(&face.surface, q).ok()?;
    normal_at(face, hit.u, hit.v)
}

/// The face's unit normal at the parameters (u, v), by its recorded sense.
fn normal_at(face: &crate::FaceRecord, u: f64, v: f64) -> Option<Vec3> {
    let (_, su, sv) = face.surface.deriv1(u, v).ok()?;
    let cross = su.cross(sv);
    let length = cross.length();
    if !(length > 0.0 && length.is_finite()) {
        return None;
    }
    let n = cross.scale(1.0 / length);
    Some(if face.same_sense { n } else { n.scale(-1.0) })
}

/// How far (as a chord of unit normals) a vertex's shared normal may differ
/// from the normal carried back along a triangle beyond the probes' own
/// spread (see `solid_display`): rounding only.
const AGREE: f64 = 1e-9;

/// The display of `solid` with chords within `tolerance` of it.
pub fn solid_display(solid: &BrepSolid, tolerance: f64) -> Result<SolidDisplay, KernelRefusal> {
    if !(tolerance > 0.0 && tolerance.is_finite()) {
        return Err(KernelRefusal::input(KernelStage::Collect, "display_tolerance", "solid_display: the tolerance must be positive and finite"));
    }
    let failed = |message: String| KernelRefusal::internal(KernelStage::Refine, "display_tessellation", format!("solid_display: {message}"));
    let mesh = crate::tessellate_brep_watertight(solid, tolerance).map_err(failed)?;
    // The mesh's face ids number the faces in shell order.
    let faces: Vec<&crate::FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    let point = |i: u32| {
        let k = 3 * i as usize;
        Vec3::new(mesh.positions[k], mesh.positions[k + 1], mesh.positions[k + 2])
    };
    // Each face's triangles over the mesh's vertices, in mesh order.
    let mut order: Vec<usize> = Vec::new();
    let mut by_face: HashMap<usize, Vec<[u32; 3]>> = HashMap::new();
    for (t, corners) in mesh.indices.chunks_exact(3).enumerate() {
        let index = *mesh.face_ids.get(t).ok_or_else(|| failed(format!("triangle {t} names no face")))? as usize;
        if index >= faces.len() {
            return Err(failed(format!("triangle {t} names face {index} of {}", faces.len())));
        }
        by_face.entry(index).or_insert_with(|| {
            order.push(index);
            Vec::new()
        }).push([corners[0], corners[1], corners[2]]);
    }
    let probe_floor = 1e-5 * crate::solid_model_scale(solid).max(1e-300);
    let mut out = SolidDisplay::default();
    let mut slot_of_face: HashMap<usize, usize> = HashMap::new();
    for index in order {
        let face = faces[index];
        let triangles = &by_face[&index];
        slot_of_face.insert(index, out.faces.len());
        // Each triangle's winding normal (its area vector; the tessellation
        // orients the mesh outward, repairing a reversed sense).
        let winding: Vec<Vec3> = triangles.iter().map(|t| {
            let [a, b, c] = t.map(point);
            b.sub(a).cross(c.sub(a))
        }).collect();
        // Each shared vertex's surface normal, turned the way its triangles
        // face (per vertex, so a face whose surface folds keeps each part's).
        let mut shared: HashMap<u32, Option<Vec3>> = HashMap::new();
        for tri in triangles {
            for &v in tri {
                shared.entry(v).or_insert_with(|| surface_normal(face, point(v)));
            }
        }
        let mut lean: HashMap<u32, f64> = HashMap::new();
        for (t, tri) in triangles.iter().enumerate() {
            for &v in tri {
                if let Some(Some(n)) = shared.get(&v) {
                    *lean.entry(v).or_default() += winding[t].dot(*n);
                }
            }
        }
        for (v, n) in shared.iter_mut() {
            if let Some(n) = n {
                if lean.get(v).copied().unwrap_or(0.0) < 0.0 {
                    *n = n.scale(-1.0);
                }
            }
        }
        let mut display = DisplayFace { face_id: face.id, ..DisplayFace::default() };
        let mut local: HashMap<u32, u32> = HashMap::new();
        for (t, tri) in triangles.iter().enumerate() {
            let w = winding[t];
            let unit = if w.length() > 0.0 { Some(w.scale(1.0 / w.length())) } else { None };
            let centroid = tri.iter().map(|&v| point(v)).fold(Vec3::new(0.0, 0.0, 0.0), |m, p| m.add(p)).scale(1.0 / 3.0);
            let mut corners = [0u32; 3];
            for (k, &v) in tri.iter().enumerate() {
                let q = point(v);
                let normal = shared[&v];
                // The normal along this triangle at the vertex: read at three
                // points 1, 2 and 3 thousandths of the way to the centroid
                // (projected: far above the projector's floor, and on the
                // triangle's own side of a seam or a crease), carried back to
                // the vertex by the parabola through them, turned the way the
                // triangle faces. Its error is the normals' third difference
                // (a smooth surface's change of curvature over the step,
                // cubed); a crease, pole or apex differs by an angle that
                // does not shrink with the step.
                // The step: a thousandth of the way in, but never closer to
                // the vertex than 1e-5 of the model's scale (the projector
                // stops within 1e-7: a probe nearer than that could read the
                // vertex's own side), nor past a third of the way.
                let toward = centroid.sub(q);
                let reach = toward.length();
                let step = if reach > 0.0 { (1e-3f64).max((probe_floor / reach).min(1.0 / 9.0)) } else { 1e-3 };
                let face_normal = |k: f64| surface_normal(face, q.add(toward.scale(k))).map(|n| if unit.is_some_and(|u| n.dot(u) < 0.0) { n.scale(-1.0) } else { n });
                let probes = [face_normal(step), face_normal(2.0 * step), face_normal(3.0 * step)];
                let (along, slack) = match probes {
                    [Some(a), Some(b), Some(c)] => {
                        let back = a.scale(3.0).sub(b.scale(3.0)).add(c);
                        // The second difference bounds what the parabola
                        // misses by the step's next power.
                        let bend = a.sub(b.scale(2.0)).add(c).length();
                        (back.normalized().ok(), bend)
                    }
                    [Some(a), ..] => (Some(a), f64::INFINITY),
                    _ => (None, 0.0),
                };
                let agrees = match (normal, along) {
                    (Some(n), Some(a)) => n.sub(a).length() <= AGREE + slack,
                    (Some(_), None) => true,
                    (None, _) => false,
                };
                corners[k] = if agrees {
                    *local.entry(v).or_insert_with(|| {
                        display.positions.push(q);
                        display.normals.push(normal.expect("agrees"));
                        (display.positions.len() - 1) as u32
                    })
                } else {
                    // No normal here that this triangle bears out (an apex, a
                    // pole, a crease inside the face): its own copy of the
                    // vertex, with the normal just inside it.
                    let n = along.ok_or_else(|| failed(format!("face {} has no normal near {q:?}", face.id)))?;
                    display.positions.push(q);
                    display.normals.push(n);
                    (display.positions.len() - 1) as u32
                };
            }
            display.triangles.push(corners);
        }
        out.faces.push(display);
    }
    // Every edge's polyline, with the display faces it bounds; a face meeting
    // itself is a seam.
    let mut uses: HashMap<u64, Vec<u32>> = HashMap::new();
    for (index, face) in faces.iter().enumerate() {
        let Some(&slot) = slot_of_face.get(&index) else { continue };
        for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
            uses.entry(coedge.edge_id).or_default().push(slot as u32);
        }
    }
    for (edge_id, points) in crate::sample_edge_polylines(solid, tolerance).map_err(failed)? {
        let used = uses.remove(&edge_id).unwrap_or_default();
        if used.len() >= 2 && used.iter().all(|&f| f == used[0]) {
            out.seams.push(points);
        } else {
            out.edges.push(DisplayEdge { edge_id, points, faces: used });
        }
    }
    Ok(out)
}

