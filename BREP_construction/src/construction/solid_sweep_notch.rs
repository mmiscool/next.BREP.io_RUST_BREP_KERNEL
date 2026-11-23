//! A convex workpiece less a sweep that crosses one of its faces' planes and
//! no other, built directly: the sweep's faces on the workpiece's side of that
//! plane, turned inside out, and the face itself with the sweep's section in
//! its plane as a hole. No general boolean: the sweep's faces are read, split
//! only along a knot line that lies in the plane, and sewn to the face by the
//! edges they already share.
//!
//! What makes it exact (each tested, else `None` and the caller cuts the sweep
//! by the boolean):
//! - the workpiece is convex with planar faces (every vertex on the inner side
//!   of every face's plane; each face a parallelogram of a 1×1 net);
//! - exactly one plane F has sweep control points outside it; the sweep's kept
//!   faces lie strictly inside every other plane (their control points, which
//!   hold them, by the hull property), so their section in F lies inside F;
//! - each sweep face lies wholly on one side of F (its control points), or is
//!   split along a full-multiplicity knot row lying in F, whose two sides each
//!   lie on one side;
//! - the edges the kept faces use once lie in F and close into one loop that
//!   is shown simple (`section_is_simple`); that loop is the sweep's section in
//!   F, a hole in F's face;
//! - the result validates.

use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};
use std::collections::{HashMap, HashSet};

/// The switch for [`notch_less`] (default on; `=0` answers None).
pub fn notch_less_on() -> bool {
    crate::switch!("BREP_SWEEP_NOTCH", true)
}

/// The workpiece less the sweep, built directly where the sweep crosses one
/// face plane of a convex workpiece (module docs); `None` elsewhere, or with
/// `BREP_SWEEP_NOTCH=0`.
pub fn notch_less(workpiece: &BrepSolid, sweep: &BrepSolid) -> Option<BrepSolid> {
    if !notch_less_on() {
        return None;
    }
    // A decline below that names no reason reads as this one.
    DECLINED.with(|cell| cell.set("declined without a stated reason"));
    build(workpiece, sweep)
}

thread_local! {
    /// The last decline's reason, for a reader that asks why (the tests).
    static DECLINED: std::cell::Cell<&'static str> = const { std::cell::Cell::new("") };
}

/// Decline, saying why ([`last_decline`]).
fn declined<T>(why: &'static str) -> Option<T> {
    DECLINED.with(|cell| cell.set(why));
    None
}

/// Why the last [`notch_less`] on this thread declined, if it said.
pub fn last_decline() -> &'static str {
    DECLINED.with(|cell| cell.get())
}

struct Plane {
    point: Vec3,
    normal: Vec3,
}

impl Plane {
    fn height(&self, x: Vec3) -> f64 {
        x.sub(self.point).dot(self.normal)
    }
}

fn point(p: &Vec4) -> Option<Vec3> {
    (p.w > 0.0 && p.w.is_finite()).then(|| Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w))
}

/// A planar face's plane (outward) and its parallelogram chart, if it is one.
fn face_plane(face: &FaceRecord) -> Option<(Plane, [Vec3; 3], [f64; 4])> {
    let surface = &face.surface;
    if surface.degree_u != 1 || surface.degree_v != 1 || surface.control_points.len() != 2 || surface.control_points[0].len() != 2 {
        return None;
    }
    // Equal weights: a bilinear net of a parallelogram is then affine in its
    // parameters (unequal ones bend the parametrisation, not the plane).
    let w = surface.control_points[0][0].w;
    if surface.control_points.iter().flatten().any(|p| p.w != w) {
        return None;
    }
    let [u0, u1] = surface.domain_u().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    let p00 = surface.evaluate(u0, v0).ok()?;
    let p10 = surface.evaluate(u1, v0).ok()?;
    let p01 = surface.evaluate(u0, v1).ok()?;
    let p11 = surface.evaluate(u1, v1).ok()?;
    let (eu, ev) = (p10.sub(p00), p01.sub(p00));
    let size = eu.length().max(ev.length());
    if p11.sub(p10.add(ev)).length() > 1e-12 * size.max(1.0) {
        return None;
    }
    let normal = eu.cross(ev).normalized().ok()?;
    let normal = if face.same_sense { normal } else { normal.scale(-1.0) };
    Some((Plane { point: p00, normal }, [p00, eu, ev], [u0, u1, v0, v1]))
}

/// A world point of a parallelogram face's plane to its parameters.
fn chart_uv(chart: &[Vec3; 3], domain: &[f64; 4], x: Vec3) -> [f64; 2] {
    let [p00, eu, ev] = *chart;
    let d = x.sub(p00);
    let (a, b, c) = (eu.dot(eu), eu.dot(ev), ev.dot(ev));
    let (e, f) = (eu.dot(d), ev.dot(d));
    let det = a * c - b * b;
    let (alpha, beta) = ((c * e - b * f) / det, (a * f - b * e) / det);
    [domain[0] + alpha * (domain[1] - domain[0]), domain[2] + beta * (domain[3] - domain[2])]
}

/// A curve's control points' heights over a plane.
fn curve_heights(curve: &NurbsCurve, plane: &Plane) -> Option<Vec<f64>> {
    curve.control_points.iter().map(|p| point(p).map(|x| plane.height(x))).collect()
}

/// The first way a body's curves or surfaces are malformed, if any: a net not
/// rectangular, a degree at or past its points' count, knots that do not fit
/// (count, finite, non-decreasing).
fn malformed(body: &BrepSolid) -> Option<&'static str> {
    let knots_fit = |knots: &[f64], count: usize, degree: usize| {
        degree < count && knots.len() == count + degree + 1 && knots.iter().all(|k| k.is_finite()) && knots.windows(2).all(|w| w[0] <= w[1])
    };
    let curve_fits = |c: &NurbsCurve| knots_fit(&c.knots, c.control_points.len(), c.degree);
    for face in body.shells.iter().flat_map(|s| &s.faces) {
        let net = &face.surface.control_points;
        if net.is_empty() || net.iter().any(|row| row.is_empty() || row.len() != net[0].len()) {
            return Some("a face's net is not rectangular");
        }
        if !knots_fit(&face.surface.knots_u, net.len(), face.surface.degree_u) || !knots_fit(&face.surface.knots_v, net[0].len(), face.surface.degree_v) {
            return Some("a face's knots do not fit its net");
        }
        if face.loops.iter().flat_map(|l| &l.coedges).any(|c| !curve_fits(&c.pcurve)) {
            return Some("a pcurve's knots do not fit its points");
        }
    }
    if body.edges.iter().any(|e| !curve_fits(&e.curve)) {
        return Some("an edge curve's knots do not fit its points");
    }
    None
}

/// One kept sweep face: its surface (maybe one side of a split) and loop.
struct Kept {
    surface: NurbsSurface,
    same_sense: bool,
    coedges: Vec<CoedgeRecord>,
}

/// The new edge along a split line: (curve, start vertex, end vertex).
struct IsoEdge {
    id: u64,
    curve: NurbsCurve,
    start: u64,
    end: u64,
}

fn build(workpiece: &BrepSolid, sweep: &BrepSolid) -> Option<BrepSolid> {
    if workpiece.kind != crate::BodyKind::Solid || sweep.kind != crate::BodyKind::Solid {
        return declined("the workpiece and the sweep must be solids");
    }
    if workpiece.shells.len() != 1 || sweep.shells.len() != 1 || workpiece.shells[0].faces.iter().any(|f| f.loops.len() != 1) {
        return declined("one shell each, the workpiece's faces one loop each");
    }
    // Both bodies well formed in every curve and surface they hold, so that
    // nothing below reads past an array or sums a degree into an overflow.
    for body in [workpiece, sweep] {
        if let Some(why) = malformed(body) {
            return declined(why);
        }
    }
    // --- The workpiece: convex, planar.
    let mut planes = Vec::new();
    for face in &workpiece.shells[0].faces {
        let Some(plane) = face_plane(face) else { return declined("a workpiece face is not a parallelogram of equal weights") };
        planes.push(plane);
    }
    // The tolerance: of the workpiece's extent (what is kept of the sweep lies
    // inside it; what lies outside, however far, does not widen it), and the
    // rounding of a height read that far from the origin.
    let (lo, hi) = workpiece.vertices.iter().fold(
        (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY)),
        |(lo, hi), v| (Vec3::new(lo.x.min(v.point.x), lo.y.min(v.point.y), lo.z.min(v.point.z)), Vec3::new(hi.x.max(v.point.x), hi.y.max(v.point.y), hi.z.max(v.point.z))),
    );
    let extent = hi.sub(lo).length();
    let reach = lo.x.abs().max(lo.y.abs()).max(lo.z.abs()).max(hi.x.abs()).max(hi.y.abs()).max(hi.z.abs());
    if !(extent.is_finite() && extent > 0.0) {
        return declined("no extent");
    }
    let tol = 1e-9 * extent + 8.0 * f64::EPSILON * reach;
    for (plane, _, _) in &planes {
        if workpiece.vertices.iter().any(|v| plane.height(v.point) > tol) {
            return declined("the workpiece is not convex");
        }
    }
    // --- The one plane the sweep crosses.
    let sweep_points: Vec<Vec3> = sweep.shells[0]
        .faces
        .iter()
        .flat_map(|f| f.surface.control_points.iter().flatten())
        .map(point)
        .collect::<Option<_>>()?;
    let outside: Vec<usize> = (0..planes.len()).filter(|&j| sweep_points.iter().any(|&x| planes[j].0.height(x) > tol)).collect();
    let [cut] = outside.as_slice() else { return declined(if outside.is_empty() { "the sweep crosses no face plane" } else { "the sweep crosses more than one face plane" }) };
    let cut = *cut;
    let plane = &planes[cut].0;
    // --- The sweep's faces on the inner side of the plane.
    let mut kept: Vec<Kept> = Vec::new();
    let mut iso_edges: Vec<IsoEdge> = Vec::new();
    // Ids are renumbered past the workpiece's: room for that, or declined.
    const ROOM: u64 = u64::MAX / 8;
    let sweep_ids = sweep
        .vertices
        .iter()
        .map(|v| v.id)
        .chain(sweep.edges.iter().flat_map(|e| [e.id, e.start_vertex_id, e.end_vertex_id]))
        .chain(sweep.shells.iter().flat_map(|s| s.faces.iter().flat_map(|f| f.loops.iter().flat_map(|l| l.coedges.iter().map(|c| c.edge_id)))))
        .max()
        .unwrap_or(0);
    if sweep_ids >= ROOM {
        return declined("the sweep's ids leave no room to renumber");
    }
    let mut next_id = sweep_ids + 1;
    let edge_of: HashMap<u64, &EdgeRecord> = sweep.edges.iter().map(|e| (e.id, e)).collect();
    for face in &sweep.shells[0].faces {
        if face.loops.len() != 1 {
            return declined("a sweep face has more than one loop");
        }
        let heights: Vec<Vec<f64>> = face.surface.control_points.iter().map(|row| row.iter().map(|p| point(p).map(|x| plane.height(x))).collect::<Option<Vec<_>>>()).collect::<Option<_>>()?;
        let all = heights.iter().flatten();
        if all.clone().all(|&h| h <= tol) {
            kept.push(Kept { surface: face.surface.clone(), same_sense: face.same_sense, coedges: face.loops[0].coedges.clone() });
            continue;
        }
        if all.clone().all(|&h| h >= -tol) {
            continue;
        }
        let Some((surface, coedges)) = split_face(face, &heights, tol, &edge_of, &mut next_id, &mut iso_edges) else {
            return declined("a sweep face crossing the plane has no knot row in it to split along");
        };
        kept.push(Kept { surface, same_sense: face.same_sense, coedges });
    }
    if kept.is_empty() {
        return declined("nothing of the sweep lies inside the workpiece");
    }
    // A kept face meets the plane only along its boundary: its net's interior
    // points strictly inside (an interior point of the surface weighs at least
    // one of them, positively). A face lying in the plane, or touching it
    // inside, is not one the hole accounts for.
    for k in &kept {
        let net = &k.surface.control_points;
        if net.len() < 3 || net[0].len() < 3 {
            return declined("a kept face's net has no interior");
        }
        let interior = net[1..net.len() - 1].iter().flat_map(|row| &row[1..row.len() - 1]);
        if interior.clone().any(|p| point(p).is_none_or(|x| plane.height(x) >= -tol)) {
            return declined("a kept face reaches the plane inside its boundary");
        }
    }
    // Kept faces strictly inside every other plane (by their control points).
    for (j, (other, _, _)) in planes.iter().enumerate() {
        if j == cut {
            continue;
        }
        if kept.iter().flat_map(|k| k.surface.control_points.iter().flatten()).any(|p| point(p).is_none_or(|x| other.height(x) >= -tol)) {
            return declined("a kept face reaches another face plane of the workpiece");
        }
    }
    // --- The hole: edges the kept faces use once, in the plane, closed into one loop.
    let iso_of: HashMap<u64, &IsoEdge> = iso_edges.iter().map(|e| (e.id, e)).collect();
    let mut uses: HashMap<u64, Vec<bool>> = HashMap::new();
    for k in &kept {
        for c in &k.coedges {
            uses.entry(c.edge_id).or_default().push(c.forward);
        }
    }
    let degenerate = |id: u64| edge_of.get(&id).is_some_and(|e| e.degenerate);
    let mut hole: Vec<(u64, bool)> = Vec::new();
    for (&id, list) in &uses {
        if degenerate(id) {
            continue;
        }
        match list.as_slice() {
            [_, _] => {}
            [forward] => {
                let curve = match (edge_of.get(&id), iso_of.get(&id)) {
                    (Some(e), _) => &e.curve,
                    (None, Some(iso)) => &iso.curve,
                    _ => return declined("a hole edge is neither the sweep's nor a split's"),
                };
                if curve_heights(curve, plane)?.iter().any(|h| h.abs() > tol) {
                    return declined("an edge the kept faces use once does not lie in the plane");
                }
                hole.push((id, *forward));
            }
            _ => return declined("a kept edge is used more than twice"),
        }
    }
    let ends = |id: u64| -> Option<(u64, u64)> {
        match (edge_of.get(&id), iso_of.get(&id)) {
            (Some(e), _) => Some((e.start_vertex_id, e.end_vertex_id)),
            (None, Some(iso)) => Some((iso.start, iso.end)),
            _ => None,
        }
    };
    let curve_of = |id: u64| -> Option<&NurbsCurve> {
        match (edge_of.get(&id), iso_of.get(&id)) {
            (Some(e), _) => Some(&e.curve),
            (None, Some(iso)) => Some(&iso.curve),
            _ => None,
        }
    };
    // The hole uses each edge as the kept face did before it is turned.
    let mut by_start: HashMap<u64, (u64, bool, u64)> = HashMap::new();
    for &(id, forward) in &hole {
        let (a, b) = ends(id)?;
        let (from, to) = if forward { (a, b) } else { (b, a) };
        if by_start.insert(from, (id, forward, to)).is_some() {
            return declined("the hole's edges branch at a vertex");
        }
    }
    let first = *hole.iter().map(|(id, _)| id).min()?;
    let (start_vertex, _) = {
        let (a, b) = ends(first)?;
        let forward = hole.iter().find(|(id, _)| *id == first)?.1;
        if forward { (a, b) } else { (b, a) }
    };
    let mut ring: Vec<(u64, bool)> = Vec::new();
    let mut at = start_vertex;
    for _ in 0..hole.len() {
        let &(id, forward, to) = by_start.get(&at)?;
        ring.push((id, forward));
        at = to;
    }
    if at != start_vertex || ring.len() != hole.len() {
        return declined("the hole's edges do not close into one loop");
    }
    // Nothing else kept meets the plane: an edge not in the hole stays
    // strictly inside but at its ends, and an end (or a degenerate edge's
    // point) in the plane is one of the hole's vertices.
    let hole_vertices: HashSet<u64> = ring.iter().filter_map(|&(id, _)| ends(id)).flat_map(|(a, b)| [a, b]).collect();
    let vertex_at: HashMap<u64, Vec3> = sweep.vertices.iter().map(|v| (v.id, v.point)).collect();
    let in_hole: HashSet<u64> = ring.iter().map(|&(id, _)| id).collect();
    for id in uses.keys() {
        if in_hole.contains(id) {
            continue;
        }
        let Some(edge) = edge_of.get(id) else { return declined("a kept edge is not the sweep's") };
        let heights = curve_heights(&edge.curve, plane)?;
        let n = heights.len();
        if n < 2 {
            return declined("a kept edge's curve has one point");
        }
        if !edge.degenerate && heights[1..n - 1].iter().any(|&h| h >= -tol) {
            return declined("a kept edge reaches the plane outside the hole");
        }
        for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
            let at = vertex_at.get(&vertex)?;
            if plane.height(*at) >= -tol && !hole_vertices.contains(&vertex) {
                return declined("a kept edge reaches the plane outside the hole");
            }
        }
    }
    let ring_curves: Vec<NurbsCurve> = ring
        .iter()
        .map(|&(id, forward)| {
            let curve = curve_of(id)?.clone();
            if forward { Some(curve) } else { curve.reversed().ok() }
        })
        .collect::<Option<_>>()?;
    if !crate::solid_sweep::section_is_simple(&ring_curves, plane.normal, extent) {
        return declined("the hole's loop is not shown simple");
    }
    // --- Assemble: the workpiece, its cut face with the hole; the kept faces turned.
    let offset = workpiece
        .vertices
        .iter()
        .map(|v| v.id)
        .chain(workpiece.edges.iter().map(|e| e.id))
        .chain(workpiece.shells.iter().flat_map(|s| std::iter::once(s.id).chain(s.faces.iter().flat_map(|f| std::iter::once(f.id).chain(f.loops.iter().flat_map(|l| std::iter::once(l.id).chain(l.coedges.iter().map(|c| c.id))))))))
        .chain(std::iter::once(workpiece.id))
        .max()?;
    if offset >= ROOM {
        return declined("the workpiece's ids leave no room to renumber");
    }
    let offset = offset + 1;
    let mut fresh = offset + next_id + 1;
    let mut new_id = || {
        fresh += 1;
        fresh
    };
    let (_, chart, domain) = &planes[cut];
    let mut hole_coedges = Vec::with_capacity(ring.len());
    for (&(id, forward), curve) in ring.iter().zip(&ring_curves) {
        let points: Vec<Vec4> = curve
            .control_points
            .iter()
            .map(|p| {
                let x = point(p)?;
                let [u, v] = chart_uv(chart, domain, x);
                Some(Vec4 { x: u * p.w, y: v * p.w, z: 0.0, w: p.w })
            })
            .collect::<Option<_>>()?;
        let pcurve = NurbsCurve::new(curve.degree, curve.knots.clone(), points).ok()?;
        hole_coedges.push(CoedgeRecord { id: new_id(), edge_id: id + offset, forward, pcurve });
    }
    let mut faces = Vec::new();
    for (j, face) in workpiece.shells[0].faces.iter().enumerate() {
        let mut face = face.clone();
        if j == cut {
            face.loops.push(LoopRecord { id: new_id(), coedges: hole_coedges.clone() });
        }
        faces.push(face);
    }
    let mut used_edges: HashSet<u64> = HashSet::new();
    for k in &kept {
        let mut coedges = Vec::with_capacity(k.coedges.len());
        for c in k.coedges.iter().rev() {
            used_edges.insert(c.edge_id);
            coedges.push(CoedgeRecord { id: new_id(), edge_id: c.edge_id + offset, forward: !c.forward, pcurve: c.pcurve.reversed().ok()? });
        }
        faces.push(FaceRecord { id: new_id(), surface: k.surface.clone(), same_sense: !k.same_sense, loops: vec![LoopRecord { id: new_id(), coedges }], name: None });
    }
    let mut edges = workpiece.edges.clone();
    let mut used_vertices: HashSet<u64> = HashSet::new();
    for e in &sweep.edges {
        if used_edges.contains(&e.id) {
            used_vertices.insert(e.start_vertex_id);
            used_vertices.insert(e.end_vertex_id);
            let mut e = e.clone();
            e.id += offset;
            e.start_vertex_id += offset;
            e.end_vertex_id += offset;
            edges.push(e);
        }
    }
    for iso in &iso_edges {
        let [t0, t1] = iso.curve.domain().ok()?;
        used_vertices.insert(iso.start);
        used_vertices.insert(iso.end);
        edges.push(EdgeRecord { id: iso.id + offset, curve: iso.curve.clone(), t0, t1, start_vertex_id: iso.start + offset, end_vertex_id: iso.end + offset, degenerate: false, name: None });
    }
    let mut vertices = workpiece.vertices.clone();
    for v in &sweep.vertices {
        if used_vertices.contains(&v.id) {
            vertices.push(VertexRecord { id: v.id + offset, point: v.point });
        }
    }
    let holes = faces.iter().map(|f| f.loops.len().saturating_sub(1)).sum::<usize>() as i64;
    let euler = vertices.len() as i64 - edges.iter().filter(|e| !e.degenerate).count() as i64 + faces.len() as i64 - holes;
    let body = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: new_id(),
        vertices,
        edges,
        shells: vec![ShellRecord { id: new_id(), faces }],
        genus: (2 - euler) / 2,
        kind: workpiece.kind,
    };
    body.validate().is_empty().then_some(body)
}

/// One side of a sweep face split along a full-multiplicity knot row (u or v)
/// lying in the plane, the side on the plane's inner side; its loop: the
/// coedges on that side (a degenerate edge across the line cut to its side),
/// and a new edge along the line closing the gap. None where the face does
/// not split so.
fn split_face(
    face: &FaceRecord,
    heights: &[Vec<f64>],
    tol: f64,
    edge_of: &HashMap<u64, &EdgeRecord>,
    next_id: &mut u64,
    iso_edges: &mut Vec<IsoEdge>,
) -> Option<(NurbsSurface, Vec<CoedgeRecord>)> {
    let surface = &face.surface;
    for along_u in [true, false] {
        // Index rows (u) or columns (v) uniformly.
        let (degree, knots, count) = if along_u {
            (surface.degree_u, &surface.knots_u, surface.control_points.len())
        } else {
            (surface.degree_v, &surface.knots_v, surface.control_points[0].len())
        };
        let line = |r: usize| -> Vec<f64> {
            if along_u { heights[r].clone() } else { heights.iter().map(|row| row[r]).collect() }
        };
        for r in 1..count.saturating_sub(1) {
            // Row r interpolates: knots r+1 ..= r+degree all one value.
            if r + degree + 1 >= knots.len() || r < degree {
                continue;
            }
            let k = knots[r + 1];
            if !knots[r + 1..=r + degree].iter().all(|&x| x == k) || knots[r] == k || knots[r + degree + 1] == k {
                continue;
            }
            if line(r).iter().any(|h| h.abs() > tol) {
                continue;
            }
            let below = |range: std::ops::Range<usize>| range.flat_map(|i| line(i)).all(|h| h <= tol);
            let above = |range: std::ops::Range<usize>| range.flat_map(|i| line(i)).all(|h| h >= -tol);
            let keep_high = if below(r..count) && above(0..r + 1) {
                true
            } else if below(0..r + 1) && above(r..count) {
                false
            } else {
                continue;
            };
            // The kept side's surface, its domain the original's part.
            let (kept_knots, range): (Vec<f64>, std::ops::Range<usize>) = if keep_high {
                (std::iter::repeat_n(k, degree + 1).chain(knots[r + degree + 1..].iter().copied()).collect(), r..count)
            } else {
                (knots[..r + 1].iter().copied().chain(std::iter::repeat_n(k, degree + 1)).collect(), 0..r + 1)
            };
            let net: Vec<Vec<Vec4>> = if along_u {
                surface.control_points[range.clone()].to_vec()
            } else {
                surface.control_points.iter().map(|row| row[range.clone()].to_vec()).collect()
            };
            let kept_surface = if along_u {
                NurbsSurface::new(surface.degree_u, surface.degree_v, kept_knots, surface.knots_v.clone(), net).ok()?
            } else {
                NurbsSurface::new(surface.degree_u, surface.degree_v, surface.knots_u.clone(), kept_knots, net).ok()?
            };
            // The line as a curve: row r over the other parameter.
            let (other_degree, other_knots) = if along_u { (surface.degree_v, surface.knots_v.clone()) } else { (surface.degree_u, surface.knots_u.clone()) };
            let row: Vec<Vec4> = if along_u { surface.control_points[r].clone() } else { surface.control_points.iter().map(|row| row[r]).collect() };
            let iso = NurbsCurve::new(other_degree, other_knots, row).ok()?;
            let [s0, s1] = iso.domain().ok()?;
            // The coordinate across the line (u for a row), and along it.
            let across = |p: &Vec4| if along_u { p.x / p.w } else { p.y / p.w };
            let side = |x: f64| if keep_high { x >= k - 1e-12 } else { x <= k + 1e-12 };
            let mut kept_coedges: Vec<CoedgeRecord> = Vec::new();
            let mut cut_any = false;
            for c in &face.loops[0].coedges {
                let xs: Vec<f64> = c.pcurve.control_points.iter().map(across).collect();
                if xs.iter().all(|&x| side(x)) {
                    kept_coedges.push(c.clone());
                } else if xs.iter().any(|&x| side(x)) {
                    // Across the line: only a degenerate edge, a straight pcurve, is cut.
                    let edge = edge_of.get(&c.edge_id)?;
                    if !edge.degenerate || c.pcurve.degree != 1 || c.pcurve.control_points.len() != 2 {
                        return None;
                    }
                    let clamp = |p: Vec4| -> Vec4 {
                        let (u, v) = (p.x / p.w, p.y / p.w);
                        let (u, v) = if along_u { (if side(u) { u } else { k }, v) } else { (u, if side(v) { v } else { k }) };
                        Vec4 { x: u, y: v, z: 0.0, w: 1.0 }
                    };
                    let points = vec![clamp(c.pcurve.control_points[0]), clamp(c.pcurve.control_points[1])];
                    let pcurve = NurbsCurve::new(1, c.pcurve.knots.clone(), points).ok()?;
                    kept_coedges.push(CoedgeRecord { id: c.id, edge_id: c.edge_id, forward: c.forward, pcurve });
                    cut_any = true;
                }
            }
            let _ = cut_any;
            // The gap: the one place the loop does not join, closed along the line.
            let uv_end = |c: &CoedgeRecord| -> Option<[f64; 2]> {
                let [_, d1] = c.pcurve.domain().ok()?;
                let p = c.pcurve.evaluate(d1).ok()?;
                Some([p.x, p.y])
            };
            let uv_start = |c: &CoedgeRecord| -> Option<[f64; 2]> {
                let [d0, _] = c.pcurve.domain().ok()?;
                let p = c.pcurve.evaluate(d0).ok()?;
                Some([p.x, p.y])
            };
            let n = kept_coedges.len();
            if n == 0 {
                return None;
            }
            let mut gaps = Vec::new();
            for i in 0..n {
                let (a, b) = (uv_end(&kept_coedges[i])?, uv_start(&kept_coedges[(i + 1) % n])?);
                if (a[0] - b[0]).hypot(a[1] - b[1]) > 1e-9 {
                    gaps.push(i);
                }
            }
            let [gap] = gaps.as_slice() else { return None };
            let gap = *gap;
            let (a, b) = (uv_end(&kept_coedges[gap])?, uv_start(&kept_coedges[(gap + 1) % n])?);
            let along = |p: [f64; 2]| if along_u { p[1] } else { p[0] };
            let (from, to) = (along(a), along(b));
            // The gap spans the whole line, end to end.
            let forward = if (from - s0).abs() < 1e-9 && (to - s1).abs() < 1e-9 {
                true
            } else if (from - s1).abs() < 1e-9 && (to - s0).abs() < 1e-9 {
                false
            } else {
                return None;
            };
            // Its ends' vertices: where the coedges beside the gap end and start.
            let vertex_end = |c: &CoedgeRecord| -> Option<u64> {
                let e = edge_of.get(&c.edge_id)?;
                Some(if c.forward { e.end_vertex_id } else { e.start_vertex_id })
            };
            let vertex_start = |c: &CoedgeRecord| -> Option<u64> {
                let e = edge_of.get(&c.edge_id)?;
                Some(if c.forward { e.start_vertex_id } else { e.end_vertex_id })
            };
            let (va, vb) = (vertex_end(&kept_coedges[gap])?, vertex_start(&kept_coedges[(gap + 1) % n])?);
            let (start, end) = if forward { (va, vb) } else { (vb, va) };
            let id = *next_id;
            *next_id += 1;
            let line_uv = |s: f64| if along_u { Vec4 { x: k, y: s, z: 0.0, w: 1.0 } } else { Vec4 { x: s, y: k, z: 0.0, w: 1.0 } };
            let pcurve = NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![line_uv(from), line_uv(to)]).ok()?;
            let iso_id = *next_id;
            *next_id += 1;
            kept_coedges.insert(gap + 1, CoedgeRecord { id: iso_id, edge_id: id, forward, pcurve });
            iso_edges.push(IsoEdge { id, curve: iso, start, end });
            return Some((kept_surface, kept_coedges));
        }
    }
    None
}

