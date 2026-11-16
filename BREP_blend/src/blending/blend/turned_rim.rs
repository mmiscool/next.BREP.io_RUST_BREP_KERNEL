//! A closed ridge written as a CHAIN of arcs because the two walls it joins
//! have their seams a hair apart: a cone stood on a cylinder and turned a few
//! microradians about the common axis. The union splits the rim at both seam
//! vertices, so one piece is a few 1e-6 long, and the per-edge blend of that
//! piece (a cutter swept over microradians, whose boolean has nothing to
//! resolve) refuses where the same ridge with its seams together, one closed
//! edge, builds.
//!
//! So when the ordinary path refuses such a selection, the body is
//! CANONICALISED: the side of the rim whose every face is a surface of
//! revolution about the rim's axis (or a plane square to it), with every
//! other edge a seam or a circle about that axis, is turned rigidly about the
//! axis so its seam vertex lands on the other wall's. That side maps onto
//! itself as a point set, so the turn is exact; it only moves where its seams
//! run. The rim's pieces then join into one closed edge (the long piece's id
//! and name), and the selection is that edge.
//!
//! The result keeps the turned side: its seam (and the seam vertex of every
//! circle on it) sits at the other wall's seam angle, a few microradians from
//! where it was. Every face, edge and vertex keeps its id and name, except
//! that the short rim piece and the turned side's rim vertex are gone (the
//! rim is consumed by the blend).
//!
//! Default ON: `BREP_FILLET_TURNED_RIM=0` (or `off`) turns it off.

use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord};
use crate::transform_topology::{transform_curve, transform_surface, AffineTransform};
use crate::{KernelRefusal, NurbsCurve, Vec3};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

/// Whether the turned-rim canonicalisation runs.
pub fn turned_rim_on() -> bool {
    switch!("BREP_FILLET_TURNED_RIM", true)
}

/// The canonicalised body and the selection on it: one point on the joined
/// rim and the first selected name.
pub struct TurnedRim {
    pub solid: BrepSolid,
    pub points: Vec<Vec3>,
    pub names: Option<Vec<String>>,
}

/// The (face id, loop index, coedge index) uses of every edge.
fn uses(solid: &BrepSolid) -> HashMap<u64, Vec<(u64, usize, usize)>> {
    let mut map: HashMap<u64, Vec<(u64, usize, usize)>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for (loop_index, loop_record) in face.loops.iter().enumerate() {
            for (index, coedge) in loop_record.coedges.iter().enumerate() {
                map.entry(coedge.edge_id).or_default().push((face.id, loop_index, index));
            }
        }
    }
    map
}

fn samples(edge: &EdgeRecord, count: usize) -> Option<Vec<Vec3>> {
    (0..=count)
        .map(|index| edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * index as f64 / count as f64).ok())
        .collect()
}

/// Whether every sample of `edge` lies on the circle about the axis line
/// (`origin`, `axis`) through its first sample.
fn on_axis_circle(edge: &EdgeRecord, origin: Vec3, axis: Vec3, tolerance: f64) -> bool {
    let Some(points) = samples(edge, 8) else { return false };
    let height = points[0].sub(origin).dot(axis);
    // Every control point in the plane square to the axis: the whole curve
    // is planar there (samples alone could pass a wavy boundary), and on a
    // face of revolution that plane cuts only circles.
    let planar = edge.curve.control_points.iter().all(|pole| {
        pole.point().is_ok_and(|point| (point.sub(origin).dot(axis) - height).abs() <= tolerance)
    });
    if !planar {
        return false;
    }
    let radius = |point: Vec3| point.sub(origin).sub(axis.scale(point.sub(origin).dot(axis))).length();
    let first = radius(points[0]);
    points.iter().all(|point| (point.sub(origin).dot(axis) - height).abs() <= tolerance && (radius(*point) - first).abs() <= tolerance)
}

/// Rotation by `angle` about the line through `origin` along unit `axis`.
fn rotation(origin: Vec3, axis: Vec3, angle: f64) -> Option<AffineTransform> {
    let (sin, cos) = angle.sin_cos();
    let (x, y, z) = (axis.x, axis.y, axis.z);
    let c1 = 1.0 - cos;
    let r = [
        [cos + x * x * c1, x * y * c1 - z * sin, x * z * c1 + y * sin],
        [y * x * c1 + z * sin, cos + y * y * c1, y * z * c1 - x * sin],
        [z * x * c1 - y * sin, z * y * c1 + x * sin, cos + z * z * c1],
    ];
    let moved = |row: [f64; 3]| row[0] * origin.x + row[1] * origin.y + row[2] * origin.z;
    #[rustfmt::skip]
    let elements = [
        r[0][0], r[0][1], r[0][2], origin.x - moved(r[0]),
        r[1][0], r[1][1], r[1][2], origin.y - moved(r[1]),
        r[2][0], r[2][1], r[2][2], origin.z - moved(r[2]),
        0.0, 0.0, 0.0, 1.0,
    ];
    AffineTransform::new(elements).ok()
}

/// The canonicalised body, or `None` when the selection is not that class
/// (or the body it would give does not validate).
pub fn turn_rim_seams_together(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    resolve: &dyn Fn(&BrepSolid, Vec3) -> Result<u64, KernelRefusal>,
) -> Option<TurnedRim> {
    if edge_points.len() < 2 {
        return None;
    }
    let mut selected: Vec<u64> = Vec::new();
    for point in edge_points {
        let id = resolve(solid, *point).ok()?;
        if !selected.contains(&id) {
            selected.push(id);
        }
    }
    if selected.len() != 2 {
        return None;
    }
    let edge = |id: u64| solid.edges.iter().find(|edge| edge.id == id);
    let rim: Vec<&EdgeRecord> = selected.iter().map(|id| edge(*id)).collect::<Option<_>>()?;
    // Two open arcs closing one cycle through two vertices.
    let (va, vb) = (rim[0].start_vertex_id, rim[0].end_vertex_id);
    if va == vb || rim.iter().any(|piece| piece.degenerate)
        || !((rim[1].start_vertex_id == va && rim[1].end_vertex_id == vb) || (rim[1].start_vertex_id == vb && rim[1].end_vertex_id == va))
    {
        return None;
    }
    // Both pieces between the same two faces, each face once.
    let uses = uses(solid);
    let faces_of = |id: u64| -> Option<[u64; 2]> {
        let list = uses.get(&id)?;
        (list.len() == 2 && list[0].0 != list[1].0).then(|| {
            let mut pair = [list[0].0, list[1].0];
            pair.sort_unstable();
            pair
        })
    };
    let pair = faces_of(rim[0].id)?;
    if faces_of(rim[1].id)? != pair {
        return None;
    }
    let face = |id: u64| solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == id);
    let frame = face(pair[0])?.surface.analytic()?.frame()?.clone();
    let (origin, axis) = (frame.origin, frame.axis.normalized().ok()?);
    let point_of = |id: u64| solid.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point);
    let (pa, pb) = (point_of(va)?, point_of(vb)?);
    let center = origin.add(axis.scale(pa.sub(origin).dot(axis)));
    let radius = pa.sub(center).length();
    // Relative to the rim, plus coordinate roundoff where the body sits:
    // never loose enough to pass an eccentric feature as coaxial.
    let tolerance = 1e-9 * (1.0 + radius) + 64.0 * f64::EPSILON * center.length();
    if radius <= tolerance || rim.iter().any(|piece| !on_axis_circle(piece, origin, axis, tolerance)) {
        return None;
    }
    // The faces reachable from `start` without crossing the rim.
    let rim_ids: HashSet<u64> = selected.iter().copied().collect();
    let side = |start: u64| -> HashSet<u64> {
        let mut seen: HashSet<u64> = [start].into_iter().collect();
        let mut stack = vec![start];
        while let Some(current) = stack.pop() {
            let Some(record) = face(current) else { continue };
            for coedge in record.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                if rim_ids.contains(&coedge.edge_id) {
                    continue;
                }
                for (other, _, _) in uses.get(&coedge.edge_id).into_iter().flatten() {
                    if seen.insert(*other) {
                        stack.push(*other);
                    }
                }
            }
        }
        seen
    };
    let sides = [side(pair[0]), side(pair[1])];
    if !sides[0].is_disjoint(&sides[1]) {
        return None;
    }
    // A side turns onto itself when every face is coaxial revolution or a
    // plane square to the axis, and every non-rim edge a seam or a coaxial
    // circle; it holds exactly one rim vertex, the one that moves.
    let symmetric = |faces: &HashSet<u64>| -> Option<(Vec<u64>, u64)> {
        let mut edges: Vec<u64> = Vec::new();
        for id in faces {
            let record = face(*id)?;
            let analytic = record.surface.analytic()?;
            let coaxial = match analytic.frame() {
                Some(other) => other.axis.cross(axis).length() <= 1e-12
                    && other.origin.sub(origin).sub(axis.scale(other.origin.sub(origin).dot(axis))).length() <= tolerance,
                None => match analytic {
                    crate::AnalyticSurface::Plane { u_dir, v_dir, .. } => {
                        u_dir.cross(*v_dir).normalized().is_ok_and(|normal| normal.cross(axis).length() <= 1e-12)
                    }
                    _ => false,
                },
            };
            if !coaxial {
                return None;
            }
            for coedge in record.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                if !rim_ids.contains(&coedge.edge_id) && !edges.contains(&coedge.edge_id) {
                    edges.push(coedge.edge_id);
                }
            }
        }
        let mut rim_vertex = None;
        for id in &edges {
            let record = edge(*id)?;
            // A seam: used exactly twice, both by one face (a one-use sheet
            // boundary is not one).
            let seam = uses.get(id).is_some_and(|list| list.len() == 2 && list[0].0 == list[1].0);
            if !seam && !(record.degenerate || on_axis_circle(record, origin, axis, tolerance)) {
                return None;
            }
            for vertex in [record.start_vertex_id, record.end_vertex_id] {
                if vertex == va || vertex == vb {
                    if rim_vertex.is_some_and(|known| known != vertex) {
                        return None;
                    }
                    rim_vertex = Some(vertex);
                }
            }
        }
        Some((edges, rim_vertex?))
    };
    // Turn the side with fewer faces that qualifies.
    let mut order = [0usize, 1];
    order.sort_by_key(|index| sides[*index].len());
    let (turned, (turned_edges, moving)) = order.iter().find_map(|index| symmetric(&sides[*index]).map(|found| (*index, found)))?;
    let (moving_point, staying) = if moving == vb { (pb, va) } else { (pa, vb) };
    let staying_point = point_of(staying)?;
    // The other side's edges must not reach the moving vertex.
    let other_side = &sides[1 - turned];
    for id in other_side {
        for coedge in face(*id)?.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let record = edge(coedge.edge_id)?;
            if !rim_ids.contains(&record.id) && (record.start_vertex_id == moving || record.end_vertex_id == moving) {
                return None;
            }
        }
    }
    let (from, to) = (moving_point.sub(center), staying_point.sub(center));
    let angle = from.cross(to).dot(axis).atan2(from.dot(to));
    let turn = rotation(origin, axis, angle)?;

    let mut body = solid.clone();
    let turned_faces = &sides[turned];
    for shell in &mut body.shells {
        for record in &mut shell.faces {
            if turned_faces.contains(&record.id) {
                record.surface = transform_surface(&record.surface, turn).ok()?;
            }
        }
    }
    let mut moved_vertices: HashSet<u64> = HashSet::default();
    for record in &mut body.edges {
        if turned_edges.contains(&record.id) {
            record.curve = transform_curve(&record.curve, turn).ok()?;
            moved_vertices.extend([record.start_vertex_id, record.end_vertex_id]);
        }
    }
    for vertex in &mut body.vertices {
        if vertex.id == moving {
            vertex.point = staying_point;
        } else if moved_vertices.contains(&vertex.id) {
            vertex.point = turn.point(vertex.point);
        }
    }
    body.vertices.retain(|vertex| vertex.id != moving);
    for record in &mut body.edges {
        for end in [&mut record.start_vertex_id, &mut record.end_vertex_id] {
            if *end == moving {
                *end = staying;
            }
        }
    }
    // The long piece (the larger sweep) carries the joined rim.
    let sweep = |piece: &EdgeRecord| -> Option<f64> {
        let points = samples(piece, 2)?;
        let (a, m, b) = (points[0].sub(center), points[1].sub(center), points[2].sub(center));
        let half = |p: Vec3, q: Vec3| p.cross(q).dot(axis).atan2(p.dot(q)).abs();
        Some(half(a, m) + half(m, b))
    };
    let (long, short) = if sweep(rim[0])? >= sweep(rim[1])? { (rim[0], rim[1]) } else { (rim[1], rim[0]) };
    // The joined rim runs from the staying vertex the way the long piece
    // leaves it.
    let leaves = |piece: &EdgeRecord| -> Option<Vec3> {
        let forward = piece.start_vertex_id == staying;
        let t = if forward { piece.t0 } else { piece.t1 };
        let tangent = piece.curve.derivatives(t, 1).ok()?[1];
        Some(if forward { tangent } else { tangent.scale(-1.0) })
    };
    let x_axis = staying_point.sub(center).scale(1.0 / radius);
    let mut y_axis = axis.cross(x_axis);
    if y_axis.dot(leaves(long)?) < 0.0 {
        y_axis = y_axis.scale(-1.0);
    }
    let circle: NurbsCurve = crate::make_arc(center, x_axis, y_axis, radius, 0.0, std::f64::consts::TAU).ok()?;
    let [t0, t1] = circle.domain().ok()?;
    let tangent_at_start = circle.derivatives(t0, 1).ok()?[1];
    for record in &mut body.edges {
        if record.id == long.id {
            record.curve = circle.clone();
            record.t0 = t0;
            record.t1 = t1;
            record.start_vertex_id = staying;
            record.end_vertex_id = staying;
            record.degenerate = false;
        }
    }
    body.edges.retain(|record| record.id != short.id);
    // Each face's run of rim coedges becomes one coedge along the joined rim:
    // its pcurve is the iso line from the run's first start to its last end
    // (both on the face's seam, the turned face's carried round with it).
    let pieces = |coedge: &CoedgeRecord| rim_ids.contains(&coedge.edge_id);
    for shell in &mut body.shells {
        for record in &mut shell.faces {
            for loop_record in &mut record.loops {
                let count = loop_record.coedges.len();
                let Some(first) = (0..count).find(|index| pieces(&loop_record.coedges[*index]) && !pieces(&loop_record.coedges[(*index + count - 1) % count])) else {
                    continue;
                };
                let mut run = Vec::new();
                let mut index = first;
                while pieces(&loop_record.coedges[index]) && run.len() < count {
                    run.push(index);
                    index = (index + 1) % count;
                }
                if run.len() != 2 {
                    return None;
                }
                let head = &loop_record.coedges[run[0]];
                let tail = &loop_record.coedges[run[1]];
                let [h0, _] = head.pcurve.domain().ok()?;
                let [_, t1_] = tail.pcurve.domain().ok()?;
                let uv_start = head.pcurve.evaluate(h0).ok()?;
                let uv_end = tail.pcurve.evaluate(t1_).ok()?;
                // Traversal direction at the run's start, on the original body.
                let head_edge = edge(head.edge_id)?;
                let along = {
                    let t = if head.forward { head_edge.t0 } else { head_edge.t1 };
                    let tangent = head_edge.curve.derivatives(t, 1).ok()?[1];
                    if head.forward { tangent } else { tangent.scale(-1.0) }
                };
                let joined = CoedgeRecord {
                    id: head.id,
                    edge_id: long.id,
                    forward: along.dot(tangent_at_start) > 0.0,
                    pcurve: crate::make_line(uv_start, uv_end).ok()?,
                };
                let mut coedges: Vec<CoedgeRecord> = Vec::with_capacity(count - 1);
                for (position, coedge) in loop_record.coedges.iter().enumerate() {
                    if position == run[0] {
                        coedges.push(joined.clone());
                    } else if position != run[1] {
                        coedges.push(coedge.clone());
                    }
                }
                loop_record.coedges = coedges;
            }
        }
    }
    body.mass_properties_cache = Default::default();
    if !body.validate().is_empty() {
        return None;
    }
    let point = circle.evaluate(t0 + 0.5 * (t1 - t0)).ok()?;
    let names = edge_names.and_then(|names| names.first().cloned()).map(|name| vec![name]);
    Some(TurnedRim { solid: body, points: vec![point], names })
}
