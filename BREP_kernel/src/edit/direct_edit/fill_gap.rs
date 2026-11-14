//! Deleting faces whose gap closes by FILLING: the last lane, after every
//! other has refused (`BREP_DELETE_FILL_GAP`, default on; `=0` leaves the
//! refusal).
//!
//! The heals before this one re-solve edges: a strip between two walls, a
//! patch capped where its rim lies whole in a face, a band, a corner blend, a
//! ring. Three plain selections fit none of them, and each refused:
//!
//! * an L-block's STEP, riser and tread together: the faces left beside them
//!   (the far side and the far end) have never shared an edge, so no pair of
//!   neighbours re-intersects along anything the selection touched;
//! * a half-round NOTCH through a block's side: one face, whose two straight
//!   edges lie in the same plane, so the "two walls" are one wall;
//! * an open SLOT, two flats and a half cylinder: the same, three faces deep.
//!
//! What "the faces round them run on until they close the gap" means for all
//! three is the same sentence: the material missing between the selection and
//! the planes of the faces that border it comes back. So that is what is
//! built. The gap's region is the box that holds the selected faces, cut to
//! the inner side of every bordering face's plane; the body united with it is
//! the answer IF, and only if, it is checked to be one:
//!
//! * every face that borders the selection is planar, and none of their planes
//!   cuts the box (this first slice builds only the box itself, each side that
//!   no bordering plane holds pushed a little way on into the body);
//! * after the union every sampled point of every selected face is strictly
//!   inside the body: the selection is gone, not merely covered at an edge;
//! * no face of the box is left showing except where it lies in a bordering
//!   face's plane: the fill has added no surface the body did not already have.
//!
//! A selection that removes material when healed (a boss, a concave round)
//! fails the second test, and one whose gap opens onto anything but its
//! bordering planes fails the third; both keep the refusal they came with.
use super::*;
use crate::classification::PolygonClass;
use crate::{boolean_operation, classify_point, make_box_brep, parameter_point_in_face, BooleanOperation, BooleanOptions, PointClass, Vec2};

fn fill_gap_enabled() -> bool {
    switch!("BREP_DELETE_FILL_GAP", true)
}

/// `solid` with `face_ids` deleted by filling the gap they bound, or `None`
/// where that is not provably the heal (see the module docs).
fn declined(step: u32) -> Option<BrepSolid> {
    if crate::debug_flag!("BREP_DEBUG_FILL_GAP") {
        eprintln!("delete fill gap: declined at step {step}");
    }
    None
}

pub(super) fn heal_by_filling_the_gap(solid: &BrepSolid, face_ids: &[u64]) -> Option<BrepSolid> {
    if !fill_gap_enabled() || face_ids.is_empty() {
        return declined(1);
    }
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let mut selected: HashSet<u64> = face_ids.iter().copied().collect();
    let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    let edges_of = |face: &FaceRecord| -> Vec<u64> { face.loops.iter().flat_map(|l| &l.coedges).map(|c| c.edge_id).collect() };
    // A wall the kernel keeps as two faces split at a seam (a half cylinder cut
    // from a block comes as two quarter turns) is one face to whoever picked it:
    // a curved face that borders the selection on the SAME surface as a selected
    // face is taken with it.
    let on_surface_of = |face: &FaceRecord, other: &FaceRecord| -> bool {
        let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else { return false };
        [(0.25, 0.25), (0.5, 0.5), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)].iter().all(|(a, b)| {
            face.surface
                .evaluate(u0 + (u1 - u0) * a, v0 + (v1 - v0) * b)
                .ok()
                .and_then(|point| crate::project_point_to_surface(&other.surface, point).ok())
                .is_some_and(|foot| foot.distance <= tolerance)
        })
    };
    loop {
        let rim: HashSet<u64> = faces.iter().filter(|f| selected.contains(&f.id)).flat_map(|f| edges_of(f)).collect();
        let more: Vec<u64> = faces
            .iter()
            .filter(|f| !selected.contains(&f.id) && f.surface.analytic().is_some_and(|a| !matches!(a, crate::analytic_surface::AnalyticSurface::Plane { .. })))
            .filter(|f| edges_of(f).iter().any(|edge| rim.contains(edge)))
            .filter(|f| faces.iter().any(|g| selected.contains(&g.id) && on_surface_of(f, g)))
            .map(|f| f.id)
            .collect();
        if more.is_empty() {
            break;
        }
        selected.extend(more);
    }
    let rim: HashSet<u64> = faces.iter().filter(|f| selected.contains(&f.id)).flat_map(|f| edges_of(f)).collect();
    if rim.is_empty() {
        return declined(2);
    }

    // The planes of the faces that border the selection, as (point, outward normal).
    let mut planes: Vec<(Vec3, Vec3)> = Vec::new();
    for face in faces.iter().filter(|f| !selected.contains(&f.id)) {
        if !edges_of(face).iter().any(|edge| rim.contains(edge)) {
            continue;
        }
        let Some(crate::analytic_surface::AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) = face.surface.analytic() else {
            return declined(3);
        };
        let mut normal = u_dir.cross(*v_dir).normalized().ok()?;
        if !face.same_sense {
            normal = normal.scale(-1.0);
        }
        planes.push((*origin, normal));
    }
    if planes.is_empty() {
        return declined(4);
    }

    // The box that holds the selection: its edges at 33 stations each, and the
    // points of each face's own region on a 9 × 9 grid of its surface (a curved
    // face bulges past its edges). The same grid points are the witnesses that
    // the face is gone.
    let (mut low, mut high) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    let take = |low: &mut [f64; 3], high: &mut [f64; 3], p: Vec3| {
        for (axis, value) in [p.x, p.y, p.z].into_iter().enumerate() {
            low[axis] = low[axis].min(value);
            high[axis] = high[axis].max(value);
        }
    };
    for edge in solid.edges.iter().filter(|edge| rim.contains(&edge.id) && !edge.degenerate) {
        for k in 0..=32 {
            take(&mut low, &mut high, edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 32.0).ok()?);
        }
    }
    let mut witnesses: Vec<Vec3> = Vec::new();
    for face in faces.iter().filter(|f| selected.contains(&f.id)) {
        let ([u0, u1], [v0, v1]) = (face.surface.domain_u().ok()?, face.surface.domain_v().ok()?);
        let before = witnesses.len();
        for i in 1..=9 {
            for j in 1..=9 {
                let (u, v) = (u0 + (u1 - u0) * i as f64 / 10.0, v0 + (v1 - v0) * j as f64 / 10.0);
                if matches!(parameter_point_in_face(face, Vec2 { x: u, y: v }, 1e-9).ok()?, PolygonClass::Inside) {
                    witnesses.push(face.surface.evaluate(u, v).ok()?);
                }
            }
        }
        if witnesses.len() == before {
            return declined(5);
        }
    }
    for &point in &witnesses {
        take(&mut low, &mut high, point);
    }
    if (0..3).any(|axis| !(high[axis] - low[axis] > 1e-6 * scale)) {
        return declined(6);
    }
    // This slice builds only the box: no bordering plane may cut into it.
    let outside = |p: Vec3, plane: &(Vec3, Vec3)| p.sub(plane.0).dot(plane.1);
    let cut_by_a_plane = |low: [f64; 3], high: [f64; 3]| {
        (0..8).any(|k| {
            let p = Vec3::new(
                if k & 1 == 0 { low[0] } else { high[0] },
                if k & 2 == 0 { low[1] } else { high[1] },
                if k & 4 == 0 { low[2] } else { high[2] },
            );
            planes.iter().any(|plane| outside(p, plane) > tolerance)
        })
    };
    if cut_by_a_plane(low, high) {
        return declined(7);
    }
    // Each side of the box that no bordering plane holds is pushed a little way
    // on, into the body: a curved face touches its own box along a line (a half
    // cylinder's crown), and a fill that only touches the body there has not
    // joined it. The last test below still requires every such side to lie
    // strictly inside the body.
    let reach = 0.05 * (0..3).map(|axis| high[axis] - low[axis]).fold(0.0f64, f64::max);
    for axis in 0..3 {
        let mut wider = low;
        wider[axis] -= reach;
        if !cut_by_a_plane(wider, high) {
            low = wider;
        }
        let mut wider = high;
        wider[axis] += reach;
        if !cut_by_a_plane(low, wider) {
            high = wider;
        }
    }
    let size = [high[0] - low[0], high[1] - low[1], high[2] - low[2]];

    let fill = make_box_brep(Vec3::new(low[0], low[1], low[2]), size[0], size[1], size[2]).ok()?;
    let united = boolean_operation(solid, &fill, BooleanOperation::Union, &BooleanOptions::default()).ok()?;
    if !united.validate().is_empty() {
        return declined(8);
    }
    let strictly_inside = |p: Vec3| classify_point(p, &united, tolerance).is_ok_and(|c| c.class == PointClass::In);
    // The selection is gone.
    if !witnesses.iter().all(|&p| strictly_inside(p)) {
        return declined(9);
    }
    // The fill shows nowhere but in a bordering plane.
    for axis in 0..3 {
        let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
        for end in [low[axis], high[axis]] {
            for i in 1..=5 {
                for j in 1..=5 {
                    let mut q = [0.0; 3];
                    q[axis] = end;
                    q[a] = low[a] + size[a] * i as f64 / 6.0;
                    q[b] = low[b] + size[b] * j as f64 / 6.0;
                    let q = Vec3::new(q[0], q[1], q[2]);
                    if planes.iter().any(|plane| outside(q, plane).abs() <= tolerance) {
                        continue;
                    }
                    if !strictly_inside(q) {
                        return declined(10);
                    }
                }
            }
        }
    }
    Some(united)
}
