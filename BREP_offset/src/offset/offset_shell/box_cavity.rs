//! An INWARD shell of a rectangular box whose cavity is empty
//! (`BREP_OFFSET_BOX_EMPTY_CAVITY`, default on; `=0` restores): the solid unchanged, every
//! face its own `Source` image, read exactly in front of the pipeline.
//!
//! Exact by the box's product form. For a point x inside the box, the foot of x
//! on any face's plane keeps x's other two coordinates, which lie inside that
//! face's rectangle, so x's distance to the trimmed face IS its distance to the
//! plane. The cavity, the points of the box farther than d from every retained
//! face, is then the box shrunk by d on each retained face only: a product of
//! three open intervals, empty exactly when one of them is. Along an axis whose
//! two faces are both retained that is a length of at most 2d, with one face
//! retained at most d, with none never.
//!
//! Cause (brep-bench shell/box-walls-meeting): the box 10 x 8 x 6 shelled by 4
//! open at its top, whose y walls' offsets meet in the plane y = 4. Every skin
//! reads shadowed; the pipeline then searches for a cavity (a 12³ grid of point
//! classifications and pattern-search climbs) and proves it empty on another
//! 12³ grid, 0.43 s on the host for a body it returns unchanged.
//!
//! Scope: one shell of six planar faces whose normals are three perpendicular
//! pairs; the eight vertices on the eight corners; twelve straight edges, each
//! joining two corners that differ along one axis; each face one loop of four
//! edges on its own plane; `validate()` clean; the box's centre classifies In;
//! every opening a face of the box. A cavity that is not empty declines to the
//! pipeline, which builds the shell. With `BREP_OFFSET_SHELL_EMPTY_CAVITY=0`
//! (the pipeline's own empty-cavity answer off) the lane stands down too.
use super::*;
use crate::{KernelRefusal, OrRefuse};

/// Recognition slack relative to the box's size.
const DELTA: f64 = 1e-12;

pub(super) fn enabled() -> bool {
    switch!("BREP_OFFSET_BOX_EMPTY_CAVITY", true)
}


macro_rules! decline {
    ($why:expr) => {{
        os_debug!("box empty cavity declined: {}", $why);
        return Ok(None);
    }};
}

/// The source unchanged when it is a box (see the module) whose cavity at
/// `distance` (> 0, inward) is empty; `None` declines to the pipeline.
pub(super) fn box_empty_cavity_shell(
    source: &BrepSolid,
    opening_face_ids: &[u64],
    distance: f64,
) -> Result<Option<OffsetShellResultRecord>, KernelRefusal> {
    // The answer is the empty cavity's (`BREP_OFFSET_SHELL_EMPTY_CAVITY`), read
    // exactly: with that switch off the shell refuses as before.
    if !enabled() || !super::pipeline::empty_cavity_enabled() || !(distance > MINIMUM_DISTANCE) || !distance.is_finite() || opening_face_ids.is_empty() {
        return Ok(None);
    }
    let [shell] = source.shells.as_slice() else { decline!("not one shell") };
    let faces = &shell.faces;
    if faces.len() != 6 || source.vertices.len() != 8 || source.edges.len() != 12 {
        decline!("not six faces, eight vertices and twelve edges");
    }
    if opening_face_ids.iter().any(|id| !faces.iter().any(|face| face.id == *id)) {
        decline!("an opening is not a face of the body");
    }
    // Each face's plane: its unit normal and a point on it.
    let mut planes = Vec::with_capacity(6);
    for face in faces {
        let Some(crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) = face.surface.analytic() else { decline!("a face is not planar") };
        let Ok(normal) = u_dir.cross(*v_dir).normalized() else { decline!("a degenerate plane") };
        planes.push((*origin, normal));
    }
    // Three axes from the first face of each parallel pair; every face's
    // normal along exactly one of them, square to the others.
    let mut axes: Vec<Vec3> = Vec::with_capacity(3);
    let mut axis_of = [0usize; 6];
    for (index, (_, normal)) in planes.iter().enumerate() {
        match axes.iter().position(|axis| axis.cross(*normal).length() <= DELTA) {
            Some(found) => axis_of[index] = found,
            None => {
                if axes.len() == 3 {
                    decline!("more than three face directions");
                }
                axis_of[index] = axes.len();
                axes.push(*normal);
            }
        }
    }
    if axes.len() != 3 || (0..3).any(|k| axis_of.iter().filter(|&&a| a == k).count() != 2) {
        decline!("the faces are not three parallel pairs");
    }
    if axes[0].dot(axes[1]).abs() > DELTA || axes[0].dot(axes[2]).abs() > DELTA || axes[1].dot(axes[2]).abs() > DELTA {
        decline!("the face directions are not square");
    }
    // Each pair's two plane levels along its axis.
    let level = |index: usize| planes[index].0.dot(axes[axis_of[index]]);
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for index in 0..6 {
        let (k, value) = (axis_of[index], level(index));
        low[k] = low[k].min(value);
        high[k] = high[k].max(value);
    }
    let size = (0..3).map(|k| high[k] - low[k]).fold(0.0f64, f64::max);
    if !(size > 0.0) || !size.is_finite() {
        decline!("no extent");
    }
    let slack = DELTA * size.max(planes.iter().map(|(origin, _)| origin.length()).fold(0.0, f64::max));
    if (0..3).any(|k| !(high[k] - low[k] > slack)) {
        decline!("a pair of faces on one plane");
    }
    // Which side of its pair each face is: low or high.
    let mut side = [false; 6];
    for index in 0..6 {
        let k = axis_of[index];
        let value = level(index);
        side[index] = (value - high[k]).abs() <= slack;
        if !side[index] && (value - low[k]).abs() > slack {
            decline!("a face between its pair's levels");
        }
    }
    for k in 0..3 {
        if (0..6).filter(|&index| axis_of[index] == k && side[index]).count() != 1 {
            decline!("a pair is not one low face and one high face");
        }
    }
    // Every vertex a corner: each coordinate at its pair's low or high level,
    // and all eight corners present.
    let corner_of = |point: Vec3| -> Option<usize> {
        let mut code = 0;
        for k in 0..3 {
            let value = point.dot(axes[k]);
            if (value - high[k]).abs() <= slack {
                code |= 1 << k;
            } else if (value - low[k]).abs() > slack {
                return None;
            }
        }
        Some(code)
    };
    let mut corners = HashMap::<u64, usize>::default();
    let mut seen = [false; 8];
    for vertex in &source.vertices {
        let Some(code) = corner_of(vertex.point) else { decline!("a vertex off the corners") };
        if seen[code] {
            decline!("two vertices on one corner");
        }
        seen[code] = true;
        corners.insert(vertex.id, code);
    }
    // Every edge a straight segment between two corners one axis apart.
    for edge in &source.edges {
        let (Some(&a), Some(&b)) = (corners.get(&edge.start_vertex_id), corners.get(&edge.end_vertex_id)) else { decline!("an edge off the corners") };
        if edge.degenerate || (a ^ b).count_ones() != 1 {
            decline!("an edge not along one axis");
        }
        let k = (a ^ b).trailing_zeros() as usize;
        let start = source.vertices.iter().find(|vertex| vertex.id == edge.start_vertex_id).map(|vertex| vertex.point);
        let end = source.vertices.iter().find(|vertex| vertex.id == edge.end_vertex_id).map(|vertex| vertex.point);
        let (Some(start), Some(end)) = (start, end) else { decline!("an edge's vertex is missing") };
        let reach = end.sub(start).dot(axes[k]);
        for control in &edge.curve.control_points {
            if !(control.w > 0.0) {
                decline!("an edge's weight is not positive");
            }
            let Ok(point) = control.point() else { decline!("an edge control cannot be read") };
            let off = point.sub(start);
            let along = off.dot(axes[k]);
            if off.sub(axes[k].scale(along)).length() > slack || along < reach.min(0.0) - slack || along > reach.max(0.0) + slack {
                decline!("an edge is not straight");
            }
        }
    }
    // Every face one loop of four edges, each on the face's own plane.
    let edge_by_id = source.edges.iter().map(|edge| (edge.id, edge)).collect::<HashMap<_, _>>();
    for (index, face) in faces.iter().enumerate() {
        let [loop_record] = face.loops.as_slice() else { decline!("a face has more than one loop") };
        if loop_record.coedges.len() != 4 {
            decline!("a face's loop is not four edges");
        }
        let (k, bit) = (axis_of[index], side[index]);
        for coedge in &loop_record.coedges {
            let Some(edge) = edge_by_id.get(&coedge.edge_id) else { decline!("a coedge's edge is missing") };
            for id in [edge.start_vertex_id, edge.end_vertex_id] {
                let code = corners[&id];
                if ((code >> k) & 1 == 1) != bit {
                    decline!("a face's edge is off its plane");
                }
            }
        }
    }
    if !source.validate().is_empty() {
        decline!("the body does not validate");
    }
    let centre = (0..3).fold(Vec3::new(0.0, 0.0, 0.0), |sum, k| sum.add(axes[k].scale(0.5 * (low[k] + high[k]))));
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-9 * scale;
    if classify_point(centre, source, tolerance * 10.0).or_refuse(KernelStage::Select, "classify_point")?.class != PointClass::In {
        decline!("the box's centre is not inside");
    }
    // The cavity: per axis, the interval shrunk by d on each retained face. It
    // is empty only to 1e-12 of the box's own extent, never to a tolerance
    // scaled by the body or by its distance from the world origin (1e-9 of a
    // 10^6 box would hide a cavity 1e-3 thick). A length read from levels far
    // from the origin is uncertain by what the recognition slack adds past
    // that band, and the uncertainty counts against emptiness: such a box on
    // the threshold declines to the pipeline.
    let band = DELTA * size;
    let uncertain = (slack - band).max(0.0);
    let empty = (0..3).any(|k| {
        let retained = (0..6).filter(|&index| axis_of[index] == k && !opening_face_ids.contains(&faces[index].id)).count();
        let length = high[k] - low[k];
        match retained {
            2 => length + uncertain <= 2.0 * distance + band,
            1 => length + uncertain <= distance + band,
            _ => false,
        }
    });
    if !empty {
        decline!("the cavity is not empty");
    }
    os_debug!("box empty cavity: an axis is no longer than the walls' reach, so the box is its own shell");
    let face_images = faces
        .iter()
        .map(|face| OffsetShellFaceImageRecord { role: OffsetFaceRole::Source, source_face_id: face.id })
        .collect();
    Ok(Some(OffsetShellResultRecord { solid: source.clone(), face_images }))
}
