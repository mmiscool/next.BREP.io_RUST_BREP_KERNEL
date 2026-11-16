//! `BREP_OFFSET_PIPE_PARALLEL` (default on; `=0` restores the refusal): the
//! rounded GROWTH of a CONVEX body whose faces are planes, cylinders and
//! spheres and whose every sharp edge is a closed loop (no corners), built
//! directly: each face moved out by `d` under the exact affine map of its own
//! surface (a plane translated, a cylinder or sphere scaled about its axis or
//! centre), and each edge rounded by the PIPE of radius `d` about the SOURCE
//! edge itself, between the two moved faces.
//!
//! The rolling ball's centre on a convex edge of the grown body is `d` from
//! both grown faces, so it runs on the source edge; the round is the pipe
//! `c(t) + d f(s)` with `f` the arc from one face's unit normal `a(t)` to the
//! other's `b(t)`. With `k = a.b`, `f(s) = ((1 - s^2) a + 2 s (1 + k s) b) /
//! (1 + 2 k s + s^2)`: the rational quadratic with weights `(1, 1 + k, 2 + 2k)`
//! and middle point `(a + b) / (1 + k)`. Where `c` is rational of degree `p`
//! with weight `W`, and `a`, `b` are affine in `c` (true for planes, cylinders
//! and spheres), the pipe's homogeneous net times `W^3` is polynomial of
//! degree `3p` in `t` on every span of `c`: the pipe is an exact NURBS, read
//! here by interpolating each span at `3p + 1` stations.
//!
//! Every pcurve is exact as well: a moved face keeps its source surface's
//! parametrisation, so the source pcurves serve unchanged, and the pipe's
//! rails and seam are its own iso lines.
//!
//! Cause (brep-bench: a cylinder cut by a slanted plane, grown rounded):
//! refused at 45 degrees from d 1.3 and at 60 degrees at every distance. The
//! general blend rounding the grown sharp body's elliptic edge did not settle
//! ("rails stand 1.88e-6 off their carriers (bar 1e-6) after 190 stations and
//! 6 local rounds"; at 60 degrees its start "tangency Newton did not
//! converge"), although the round there is this exact pipe.
//!
//! It runs only where every other rounded-growth lane refused.
use super::*;
use super::carrier_rebuild::{junction_mate, junction_stations, JunctionStation};
use crate::{AnalyticSurface, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, Vec4};

pub(super) fn pipe_parallel_on() -> bool {
    switch!("BREP_OFFSET_PIPE_PARALLEL", true)
}

/// How a face's points move out by `d`, and its outward unit normal there.
#[derive(Clone, Copy)]
enum Mover {
    /// A plane: the constant outward normal.
    Plane { normal: Vec3 },
    /// A cylinder about (origin, axis), radius `radius`, outward `sign`
    /// (+1 when the material is inside it).
    Cylinder { origin: Vec3, axis: Vec3, radius: f64, sign: f64 },
    /// A sphere about `centre`.
    Sphere { centre: Vec3, radius: f64, sign: f64 },
}

impl Mover {
    fn normal(&self, x: Vec3) -> Vec3 {
        match *self {
            Mover::Plane { normal } => normal,
            Mover::Cylinder { origin, axis, radius, sign } => {
                let w = x.sub(origin);
                w.sub(axis.scale(w.dot(axis))).scale(sign / radius)
            }
            Mover::Sphere { centre, radius, sign } => x.sub(centre).scale(sign / radius),
        }
    }
    fn moved(&self, x: Vec3, distance: f64) -> Vec3 {
        x.add(self.normal(x).scale(distance))
    }
    /// Convex: a plane, or a cylinder or sphere with the material inside it.
    fn convex(&self) -> bool {
        match *self {
            Mover::Plane { .. } => true,
            Mover::Cylinder { sign, .. } | Mover::Sphere { sign, .. } => sign > 0.0,
        }
    }
}

fn moved_point(point: Vec4, mover: &Mover, distance: f64) -> Result<Vec4, KernelRefusal> {
    let cartesian = point.point().or_refuse(KernelStage::Refine, "point")?;
    Ok(Vec4::from_point(mover.moved(cartesian, distance), point.w))
}

fn moved_curve(curve: &NurbsCurve, mover: &Mover, distance: f64) -> Result<NurbsCurve, KernelRefusal> {
    let points = curve.control_points.iter().map(|p| moved_point(*p, mover, distance)).collect::<Result<Vec<_>, _>>()?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), points).or_refuse(KernelStage::Refine, "NurbsCurve::new")
}

fn moved_surface(surface: &NurbsSurface, mover: &Mover, distance: f64) -> Result<NurbsSurface, KernelRefusal> {
    let net = surface
        .control_points
        .iter()
        .map(|row| row.iter().map(|p| moved_point(*p, mover, distance)).collect::<Result<Vec<_>, _>>())
        .collect::<Result<Vec<_>, _>>()?;
    NurbsSurface::new(surface.degree_u, surface.degree_v, surface.knots_u.clone(), surface.knots_v.clone(), net)
        .or_refuse(KernelStage::Refine, "NurbsSurface::new")
}

/// The face's mover, its outward sense read off the face's own normal; `None`
/// for any other surface.
fn mover_of(face: &FaceRecord) -> Result<Option<Mover>, KernelRefusal> {
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let (u, v) = ((u0 + u1) * 0.5, (v0 + v1) * 0.5);
    let at = face.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
    let outward = super::face_normal(face, u, v)?;
    let mover = match face.surface.analytic() {
        Some(AnalyticSurface::Plane { u_dir, v_dir, .. }) => {
            let Ok(normal) = u_dir.cross(*v_dir).normalized() else { return Ok(None) };
            Mover::Plane { normal: if normal.dot(outward) >= 0.0 { normal } else { normal.scale(-1.0) } }
        }
        Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. }) if (rho0 - rho1).abs() <= 1e-12 * rho0.abs().max(1.0) && *rho0 > 0.0 => {
            let probe = Mover::Cylinder { origin: frame.origin, axis: frame.axis, radius: *rho0, sign: 1.0 };
            let sign = if probe.normal(at).dot(outward) >= 0.0 { 1.0 } else { -1.0 };
            Mover::Cylinder { origin: frame.origin, axis: frame.axis, radius: *rho0, sign }
        }
        Some(AnalyticSurface::Sphere { frame, radius }) => {
            let probe = Mover::Sphere { centre: frame.origin, radius: *radius, sign: 1.0 };
            let sign = if probe.normal(at).dot(outward) >= 0.0 { 1.0 } else { -1.0 };
            Mover::Sphere { centre: frame.origin, radius: *radius, sign }
        }
        _ => return Ok(None),
    };
    // The reading must agree with the face's own normal (a unit normal, the
    // right sense) or the face is not what the reading says.
    if (mover.normal(at).dot(outward) - 1.0).abs() > 1e-7 {
        return Ok(None);
    }
    Ok(Some(mover))
}

/// The coefficients of the degree-`n` Bernstein polynomial through `values`
/// at `stations` in [0, 1] (n + 1 of each): Gaussian elimination with partial
/// pivoting on the small collocation system.
fn bernstein_through(stations: &[f64], values: &[[f64; 4]]) -> Option<Vec<[f64; 4]>> {
    let n = stations.len() - 1;
    let binomial = |n: usize, k: usize| (0..k).fold(1.0f64, |acc, i| acc * (n - i) as f64 / (i + 1) as f64);
    let mut matrix: Vec<Vec<f64>> = stations
        .iter()
        .map(|&x| (0..=n).map(|i| binomial(n, i) * x.powi(i as i32) * (1.0 - x).powi((n - i) as i32)).collect())
        .collect();
    let mut rhs: Vec<[f64; 4]> = values.to_vec();
    for column in 0..=n {
        let pivot = (column..=n).max_by(|a, b| matrix[*a][column].abs().total_cmp(&matrix[*b][column].abs()))?;
        if matrix[pivot][column].abs() < 1e-14 {
            return None;
        }
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);
        for row in (column + 1)..=n {
            let factor = matrix[row][column] / matrix[column][column];
            for k in column..=n {
                matrix[row][k] -= factor * matrix[column][k];
            }
            for c in 0..4 {
                rhs[row][c] -= factor * rhs[column][c];
            }
        }
    }
    let mut out = vec![[0.0f64; 4]; n + 1];
    for row in (0..=n).rev() {
        let mut value = rhs[row];
        for k in (row + 1)..=n {
            for c in 0..4 {
                value[c] -= matrix[row][k] * out[k][c];
            }
        }
        for c in 0..4 {
            value[c] /= matrix[row][row];
        }
        out[row] = value;
    }
    Some(out)
}

/// The pipe of radius `distance` about `spine` (t over its domain) from the
/// normal field `first` (s = 0) to `second` (s = 1).
fn pipe_surface(spine: &NurbsCurve, first: &Mover, second: &Mover, distance: f64) -> Result<Option<NurbsSurface>, KernelRefusal> {
    let mut breaks: Vec<f64> = Vec::new();
    for &knot in &spine.knots {
        if breaks.last().is_none_or(|last| knot > *last) {
            breaks.push(knot);
        }
    }
    let degree = 3 * spine.degree;
    let mut rows: [Vec<Vec4>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    let mut knots_u = Vec::new();
    for (span, pair) in breaks.windows(2).enumerate() {
        let (lo, hi) = (pair[0], pair[1]);
        let stations: Vec<f64> = (0..=degree).map(|j| j as f64 / degree as f64).collect();
        let mut values: [Vec<[f64; 4]>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        for &x in &stations {
            let t = lo + (hi - lo) * x;
            let homogeneous = spine.evaluate_homogeneous(t).or_refuse(KernelStage::Refine, "evaluate_homogeneous")?;
            let weight = homogeneous.w;
            let c = homogeneous.point().or_refuse(KernelStage::Refine, "point")?;
            let (a, b) = (first.normal(c), second.normal(c));
            let k = a.dot(b);
            if !(k > -1.0 + 1e-9) {
                return Ok(None);
            }
            let cube = weight * weight * weight;
            let rows_here = [
                (c.add(a.scale(distance)), 1.0),
                (c.add(a.add(b).scale(distance / (1.0 + k))), 1.0 + k),
                (c.add(b.scale(distance)), 2.0 + 2.0 * k),
            ];
            for (row, (point, w)) in rows_here.iter().enumerate() {
                let total = w * cube;
                values[row].push([point.x * total, point.y * total, point.z * total, total]);
            }
        }
        for row in 0..3 {
            let Some(coefficients) = bernstein_through(&stations, &values[row]) else { return Ok(None) };
            let skip = usize::from(span > 0);
            for coefficient in coefficients.into_iter().skip(skip) {
                rows[row].push(Vec4 { x: coefficient[0], y: coefficient[1], z: coefficient[2], w: coefficient[3] });
            }
        }
        if span == 0 {
            knots_u.extend(std::iter::repeat_n(lo, degree + 1));
        }
        let end_multiplicity = if span + 2 == breaks.len() { degree + 1 } else { degree };
        knots_u.extend(std::iter::repeat_n(hi, end_multiplicity));
    }
    // Net indexed [u = t][v = s].
    let net = (0..rows[0].len()).map(|i| vec![rows[0][i], rows[1][i], rows[2][i]]).collect::<Vec<_>>();
    if net.iter().flatten().any(|p| !(p.w > 0.0)) {
        return Ok(None);
    }
    let surface = NurbsSurface::new(degree, 2, knots_u, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], net).or_refuse(KernelStage::Refine, "NurbsSurface::new")?;
    Ok(Some(surface))
}

/// The lane's answer, or `None` when it declines (the caller's refusal stands).
pub(super) fn pipe_parallel(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    // The pipeline's own floor: a distance it refuses as too small is not
    // grown here either.
    if !pipe_parallel_on() || !distance.is_finite() || distance <= MINIMUM_DISTANCE {
        return None;
    }
    match pipe_parallel_impl(source, distance) {
        Ok(Some((solid, sources))) => {
            // Each moved face is its source face's offset image; a pipe has
            // no single source face.
            let face_images = sources
                .into_iter()
                .map(|source_face_id| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: source_face_id.unwrap_or(0) })
                .collect();
            Some(OffsetShellResultRecord { solid, face_images })
        }
        Ok(None) => None,
        Err(error) => {
            os_debug!("pipe parallel lane declined: {error}");
            None
        }
    }
}

/// The grown solid and, per face in shell order, the source face it images
/// (`None` for a pipe).
fn pipe_parallel_impl(source: &BrepSolid, distance: f64) -> Result<Option<(BrepSolid, Vec<Option<u64>>)>, KernelRefusal> {
    let [shell] = source.shells.as_slice() else { return Ok(None) };
    let faces: Vec<&FaceRecord> = shell.faces.iter().collect();
    let edge_by_id: HashMap<u64, &EdgeRecord> = source.edges.iter().map(|edge| (edge.id, edge)).collect();
    let vertex_by_id: HashMap<u64, Vec3> = source.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    let mut movers: HashMap<u64, Mover> = HashMap::default();
    for face in &faces {
        let Some(mover) = mover_of(face)? else {
            os_debug!("pipe parallel lane: face {} is not a plane, cylinder or sphere", face.id);
            return Ok(None);
        };
        // Only a CONVEX body: every face convex here and every edge convex
        // below make a closed, connected, locally convex surface, which bounds
        // a convex body, whose parallel body never folds. A concave face (a
        // dimple, a bore) can fold a pipe through its own spine, and
        // `validate` does not see a surface pass through itself (cross review:
        // a cylinder less an off-axis ball grown past the ball's openings).
        if !mover.convex() {
            os_debug!("pipe parallel lane: face {} is concave", face.id);
            return Ok(None);
        }
        movers.insert(face.id, mover);
    }
    // Every edge: a seam (twice in one face) or a closed convex edge between
    // two faces, used once by each, forward by one and backward by the other.
    let mut uses: HashMap<u64, Vec<(u64, bool)>> = HashMap::default();
    for face in &faces {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            uses.entry(coedge.edge_id).or_default().push((face.id, coedge.forward));
        }
    }
    // (edge, face used forward, face used backward)
    let mut rounds: Vec<(u64, u64, u64)> = Vec::new();
    for (edge_id, edge_uses) in &uses {
        let Some(edge) = edge_by_id.get(edge_id) else { return Ok(None) };
        if edge.degenerate {
            return Ok(None);
        }
        match edge_uses.as_slice() {
            [(first, _), (second, _)] if first == second => {}
            [(first, first_forward), (second, second_forward)] if first_forward != second_forward => {
                if edge.start_vertex_id != edge.end_vertex_id {
                    os_debug!("pipe parallel lane: edge {edge_id} ends in corners");
                    return Ok(None);
                }
                let face = faces.iter().find(|face| face.id == *first).expect("used face");
                let coedge = face
                    .loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .find(|coedge| coedge.edge_id == *edge_id)
                    .expect("its use");
                let Some((mate, _)) = junction_mate(&faces, face, *edge_id) else { return Ok(None) };
                let stations = junction_stations(face, coedge, edge, mate)?;
                if stations.is_empty() || !stations.iter().all(JunctionStation::is_convex) {
                    os_debug!("pipe parallel lane: edge {edge_id} is not convex all along");
                    return Ok(None);
                }
                let (forward_face, backward_face) = if *first_forward { (*first, *second) } else { (*second, *first) };
                rounds.push((*edge_id, forward_face, backward_face));
            }
            _ => return Ok(None),
        }
    }
    if rounds.is_empty() {
        return Ok(None);
    }
    rounds.sort();

    // Ids.
    let mut next = 1u64;
    let mut take = || {
        next += 1;
        next
    };
    let mut vertices: Vec<VertexRecord> = Vec::new();
    let mut vertex_of: HashMap<(u64, u64), u64> = HashMap::default(); // (face, source vertex)
    let mut vertex = |face: u64, source_vertex: u64, take: &mut dyn FnMut() -> u64, vertices: &mut Vec<VertexRecord>| -> Result<u64, KernelRefusal> {
        if let Some(id) = vertex_of.get(&(face, source_vertex)) {
            return Ok(*id);
        }
        let point = vertex_by_id.get(&source_vertex).copied().ok_or_else(|| "pipe parallel: vertex missing".to_string()).or_refuse(KernelStage::Refine, "vertex")?;
        let id = take();
        vertices.push(VertexRecord { id, point: movers[&face].moved(point, distance) });
        vertex_of.insert((face, source_vertex), id);
        Ok(id)
    };
    let mut edges: Vec<EdgeRecord> = Vec::new();
    let mut edge_of: HashMap<(u64, u64), u64> = HashMap::default(); // (face, source edge) -> moved edge
    for face in &faces {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            if edge_of.contains_key(&(face.id, coedge.edge_id)) {
                continue;
            }
            let edge = edge_by_id[&coedge.edge_id];
            let start = vertex(face.id, edge.start_vertex_id, &mut take, &mut vertices)?;
            let end = vertex(face.id, edge.end_vertex_id, &mut take, &mut vertices)?;
            let id = take();
            edges.push(EdgeRecord {
                id,
                curve: moved_curve(&edge.curve, &movers[&face.id], distance)?,
                t0: edge.t0,
                t1: edge.t1,
                start_vertex_id: start,
                end_vertex_id: end,
                degenerate: false,
                name: None,
            });
            edge_of.insert((face.id, coedge.edge_id), id);
        }
    }
    // The moved faces: the source loops over the moved edges, pcurves as they are.
    let mut out_faces: Vec<FaceRecord> = Vec::new();
    let mut sources: Vec<Option<u64>> = Vec::new();
    for face in &faces {
        sources.push(Some(face.id));
        let loops = face
            .loops
            .iter()
            .map(|loop_record| LoopRecord {
                id: take(),
                coedges: loop_record
                    .coedges
                    .iter()
                    .map(|coedge| CoedgeRecord { id: take(), edge_id: edge_of[&(face.id, coedge.edge_id)], forward: coedge.forward, pcurve: coedge.pcurve.clone() })
                    .collect(),
            })
            .collect();
        out_faces.push(FaceRecord { id: take(), surface: moved_surface(&face.surface, &movers[&face.id], distance)?, same_sense: face.same_sense, loops, name: face.name.clone() });
    }
    // One pipe per closed edge, s = 0 on the face that walks the edge
    // backward (the pipe walks that rail forward), s = 1 on the other.
    let line = |a: f64, b: f64, c: f64, d: f64| crate::sweep_topology::parameter_line(a, b, c, d).or_refuse(KernelStage::Refine, "parameter_line");
    for (edge_id, forward_face, backward_face) in &rounds {
        let edge = edge_by_id[edge_id];
        let Some(surface) = pipe_surface(&edge.curve, &movers[backward_face], &movers[forward_face], distance)? else {
            os_debug!("pipe parallel lane: edge {edge_id}'s pipe could not be read");
            return Ok(None);
        };
        let (t0, t1) = (edge.t0, edge.t1);
        let rail_low = edge_of[&(*backward_face, *edge_id)];
        let rail_high = edge_of[&(*forward_face, *edge_id)];
        // The pipe must carry its rails: its iso lines s = 0, 1 against the
        // moved edges.
        for fraction in [0.0, 0.31, 0.62, 1.0] {
            let t = t0 + (t1 - t0) * fraction;
            for (s, rail) in [(0.0, rail_low), (1.0, rail_high)] {
                let on_pipe = surface.evaluate(t, s).or_refuse(KernelStage::Refine, "evaluate")?;
                let rail_edge = edges.iter().find(|e| e.id == rail).expect("rail");
                let on_rail = rail_edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
                if on_pipe.sub(on_rail).length() > 1e-9 * (1.0 + on_rail.length()) {
                    os_debug!("pipe parallel lane: edge {edge_id}'s pipe leaves its rail by {:.3e}", on_pipe.sub(on_rail).length());
                    return Ok(None);
                }
            }
        }
        let seam_curve = surface.iso_curve_u(t0).or_refuse(KernelStage::Refine, "iso_curve_u")?;
        let low_vertex = edges.iter().find(|e| e.id == rail_low).expect("rail").start_vertex_id;
        let high_vertex = edges.iter().find(|e| e.id == rail_high).expect("rail").start_vertex_id;
        let seam = take();
        edges.push(EdgeRecord { id: seam, curve: seam_curve, t0: 0.0, t1: 1.0, start_vertex_id: low_vertex, end_vertex_id: high_vertex, degenerate: false, name: None });
        let mut face = FaceRecord {
            id: take(),
            surface,
            same_sense: true,
            name: None,
            loops: vec![LoopRecord {
                id: take(),
                coedges: vec![
                    CoedgeRecord { id: take(), edge_id: rail_low, forward: true, pcurve: line(t0, 0.0, t1, 0.0)? },
                    CoedgeRecord { id: take(), edge_id: seam, forward: true, pcurve: line(t1, 0.0, t1, 1.0)? },
                    CoedgeRecord { id: take(), edge_id: rail_high, forward: false, pcurve: line(t1, 1.0, t0, 1.0)? },
                    CoedgeRecord { id: take(), edge_id: seam, forward: false, pcurve: line(t0, 1.0, t0, 0.0)? },
                ],
            }],
        };
        // The loop's winding is fixed by its neighbours; the face's sense
        // follows it, and its normal must then point away from the spine.
        face.same_sense = parameter_space_area(&face).or_refuse(KernelStage::Refine, "parameter_space_area")? > 0.0;
        let (tm, sm) = ((t0 + t1) * 0.5, 0.5);
        let outward = super::face_normal(&face, tm, sm)?;
        let spine = edge.curve.evaluate(tm).or_refuse(KernelStage::Refine, "evaluate")?;
        if outward.dot(face.surface.evaluate(tm, sm).or_refuse(KernelStage::Refine, "evaluate")?.sub(spine)) <= 0.0 {
            os_debug!("pipe parallel lane: edge {edge_id}'s pipe would face its spine");
            return Ok(None);
        }
        out_faces.push(face);
        sources.push(None);
    }
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: take(),
        genus: source.genus,
        kind: source.kind,
        vertices,
        edges,
        shells: vec![ShellRecord { id: take(), faces: out_faces }],
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        os_debug!("pipe parallel lane declined: {} validation issue(s), first {:?}", issues.len(), issues.first());
        return Ok(None);
    }
    os_debug!("pipe parallel lane built {} faces ({} pipes)", solid.shells[0].faces.len(), rounds.len());
    Ok(Some((solid, sources)))
}
