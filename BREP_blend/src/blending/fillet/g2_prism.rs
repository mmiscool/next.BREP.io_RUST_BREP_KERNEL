//! A blend of a straight edge between two planes, built on the solid directly
//! rather than by a cutter's boolean: the curvature-continuous (order 2)
//! section meets its faces with contact of order 2, which a cutter's boolean
//! reads as a tangent-curve intersection it does not take.
//!
//! The edge's two faces are planes; at each end the edge meets one planar end
//! face, and the end vertex has the edge, one edge along each side face and
//! nothing else (a prism's side edge). Then the blend is the section extruded
//! along the edge, between the contact lines it leaves on the two faces and the
//! end faces, each cut by the section carried along the edge onto it (exactly:
//! the carry is affine, so it maps the rational section's controls). The side
//! faces lose the strips between the edge and their contact lines, their edges
//! along the end faces shorten to the contacts, and each end face takes the
//! carried section in place of its corner. Anything else is not this lane
//! (`None`), for the cutter as before.
use super::*;
use crate::topology::{CoedgeRecord, VertexRecord};

fn planar_normal(face: &FaceRecord) -> Option<Vec3> {
    let s = &face.surface;
    if s.degree_u != 1 || s.degree_v != 1 || s.control_points.len() != 2 || s.control_points[0].len() != 2 {
        return None;
    }
    let p = |i: usize, j: usize| s.control_points[i][j].point().ok();
    let (a, b, c, d) = (p(0, 0)?, p(1, 0)?, p(0, 1)?, p(1, 1)?);
    let n = b.sub(a).cross(c.sub(a)).normalized().ok()?;
    let scale = b.sub(a).length().max(c.sub(a).length());
    if d.sub(a).dot(n).abs() > 1e-12 * scale.max(1.0) {
        return None;
    }
    // Parallelogram: the chart is affine, so a curve's controls map onto it.
    if d.sub(b.add(c).sub(a)).length() > 1e-12 * scale.max(1.0) {
        return None;
    }
    Some(n)
}

fn line_ends(edge: &EdgeRecord) -> Option<(Vec3, Vec3)> {
    let c = &edge.curve;
    if c.degree != 1 || c.control_points.len() != 2 {
        return None;
    }
    Some((c.evaluate(edge.t0).ok()?, c.evaluate(edge.t1).ok()?))
}

/// The (u, v) of `point` on a planar face's affine chart.
fn chart_uv(face: &FaceRecord, point: Vec3) -> Option<[f64; 2]> {
    let q = crate::project_point_to_surface(&face.surface, point).ok()?;
    (q.distance <= 1e-9 * (1.0 + point.length())).then_some([q.u, q.v])
}

fn line2(a: [f64; 2], b: [f64; 2]) -> Option<NurbsCurve> {
    make_line(Vec3::new(a[0], a[1], 0.0), Vec3::new(b[0], b[1], 0.0)).ok()
}

/// `curve` (3D, rational) mapped onto a planar face's affine chart, control by
/// control.
fn mapped_on_plane(face: &FaceRecord, curve: &NurbsCurve) -> Option<NurbsCurve> {
    let mut controls = Vec::with_capacity(curve.control_points.len());
    for c in &curve.control_points {
        let uv = chart_uv(face, c.point().ok()?)?;
        controls.push(crate::Vec4 { x: uv[0] * c.w, y: uv[1] * c.w, z: 0.0, w: c.w });
    }
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls).ok()
}

/// Where in the solid's faces an edge is used: (face index, loop index, coedge index).
fn uses(solid: &BrepSolid, edge_id: u64) -> Vec<(usize, usize, usize)> {
    let mut out = Vec::new();
    for (fi, face) in solid.shells[0].faces.iter().enumerate() {
        for (li, ring) in face.loops.iter().enumerate() {
            for (ci, c) in ring.coedges.iter().enumerate() {
                if c.edge_id == edge_id {
                    out.push((fi, li, ci));
                }
            }
        }
    }
    out
}

/// The coedge's start and end vertices as walked.
fn walked(edge: &EdgeRecord, forward: bool) -> (u64, u64) {
    if forward { (edge.start_vertex_id, edge.end_vertex_id) } else { (edge.end_vertex_id, edge.start_vertex_id) }
}

/// The blend of the straight edge `edge_id` between two planes whose section
/// (in the plane square to the edge through its start, from the contact on the
/// face `into_first` leads into to the one on the other) is `section`, built
/// directly; None where the edge and its ends are not of that kind.
pub(super) fn blend_prism_edge(solid: &BrepSolid, edge_id: u64, cross: &EdgeCross, section: &NurbsCurve, name: Option<&str>) -> Option<BrepSolid> {
    if solid.shells.len() != 1 || !cross.convex {
        return None;
    }
    let edge = solid.edges.iter().find(|e| e.id == edge_id)?.clone();
    let (p0, p1) = (edge.curve.evaluate(edge.t0).ok()?, edge.curve.evaluate(edge.t1).ok()?);
    let length = p1.sub(p0).length();
    let e = p1.sub(p0).normalized().ok()?;
    if p0.sub(cross.start).length() > 1e-9 * (1.0 + length) {
        return None;
    }
    let tol = 1e-9 * (1.0 + length);
    let faces = &solid.shells[0].faces;
    let edge_uses = uses(solid, edge_id);
    if edge_uses.len() != 2 || edge_uses[0].0 == edge_uses[1].0 {
        return None;
    }
    // The section's ends: t = 0 on the `into_first` face, t = 1 on the other.
    let [s0, s1] = section.domain().ok()?;
    let first = section.evaluate(s0).ok()?;
    let second = section.evaluate(s1).ok()?;
    // Which use is which face: the one whose plane holds the first contact.
    let mut sides = Vec::new();
    for &(fi, li, ci) in &edge_uses {
        let n = planar_normal(&faces[fi])?;
        sides.push((fi, li, ci, n));
    }
    let on_plane = |n: Vec3, x: Vec3| n.dot(x.sub(p0)).abs() <= tol * 10.0;
    let (side_first, side_second) = if on_plane(sides[0].3, first) && on_plane(sides[1].3, second) {
        (sides[0], sides[1])
    } else if on_plane(sides[1].3, first) && on_plane(sides[0].3, second) {
        (sides[1], sides[0])
    } else {
        return None;
    };
    let contact_offsets = [first.sub(p0), second.sub(p0)];
    // At each end vertex: the edges along the two side faces, and the end face.
    struct End {
        vertex: u64,
        point: Vec3,
        along: [u64; 2],
        end_face: usize,
        normal: Vec3,
        contacts: [Vec3; 2],
    }
    let mut ends = Vec::new();
    for (vertex, point) in [(edge.start_vertex_id, p0), (edge.end_vertex_id, p1)] {
        // The vertex has exactly three edges.
        let at: Vec<&EdgeRecord> = solid.edges.iter().filter(|x| x.start_vertex_id == vertex || x.end_vertex_id == vertex).collect();
        if at.len() != 3 {
            return None;
        }
        let mut along = [0u64; 2];
        for (slot, side) in [side_first, side_second].iter().enumerate() {
            let ring = &faces[side.0].loops[side.1];
            let n = ring.coedges.len();
            let (before, after) = (&ring.coedges[(side.2 + n - 1) % n], &ring.coedges[(side.2 + 1) % n]);
            let pick = [before, after].into_iter().find(|c| {
                c.edge_id != edge_id && solid.edges.iter().find(|x| x.id == c.edge_id).is_some_and(|x| x.start_vertex_id == vertex || x.end_vertex_id == vertex)
            })?;
            along[slot] = pick.edge_id;
        }
        // The end face: the third face at the vertex, holding both.
        let holders = |id: u64| uses(solid, id).into_iter().map(|u| u.0).collect::<Vec<_>>();
        let (ha, hb) = (holders(along[0]), holders(along[1]));
        let end_face = *ha.iter().find(|f| hb.contains(f) && **f != side_first.0 && **f != side_second.0)?;
        let normal = planar_normal(&faces[end_face])?;
        if normal.dot(e).abs() < 1e-6 {
            return None;
        }
        // Each contact line meets the end face where it crosses its plane.
        let mut contacts = [Vec3::new(0.0, 0.0, 0.0); 2];
        for k in 0..2 {
            let on_line = p0.add(contact_offsets[k]);
            let lambda = normal.dot(point.sub(on_line)) / normal.dot(e);
            contacts[k] = on_line.add(e.scale(lambda));
            // Strictly inside its edge along the side face, which is straight.
            let other = solid.edges.iter().find(|x| x.id == along[k])?;
            let (a, b) = line_ends(other)?;
            let (from, to) = if other.start_vertex_id == vertex { (a, b) } else { (b, a) };
            let run = to.sub(from);
            let f = contacts[k].sub(from).dot(run) / run.dot(run);
            let off = contacts[k].sub(from.add(run.scale(f))).length();
            if !(f > 1e-6 && f < 1.0 - 1e-6 && off <= 1e-7 * (1.0 + length)) {
                return None;
            }
        }
        ends.push(End { vertex, point, along, end_face, normal, contacts });
    }
    if ends[0].end_face == ends[1].end_face {
        return None;
    }
    // The four faces it edits hold nothing else: one loop each (a hole in
    // any of them, a bore beside the edge, is the cutter's to answer).
    let edited = [side_first.0, side_second.0, ends[0].end_face, ends[1].end_face];
    for f in edited {
        if faces[f].loops.len() != 1 {
            return None;
        }
    }
    // And no other face reaches into what the blend takes off: that region is
    // the convex prism on the corner and the two contacts between the end
    // faces, and every other face must lie wholly outside one of its five
    // planes (its controls do, whose hull holds it); anything else (a bore
    // that runs in toward the corner from another face) is the cutter's.
    {
        let (ca, cb) = (p0.add(contact_offsets[0]), p0.add(contact_offsets[1]));
        let chord_normal = {
            let across = cb.sub(ca);
            let n = e.cross(across).normalized().ok()?;
            if n.dot(p0.sub(ca)) > 0.0 { n.scale(-1.0) } else { n }
        };
        // (normal pointing out of the region, a point on the plane)
        let outward_of = |n: Vec3, on: Vec3, inside: Vec3| if n.dot(inside.sub(on)) > 0.0 { n.scale(-1.0) } else { n };
        let centroid = p0.add(ca).add(cb).scale(1.0 / 3.0).add(p1.sub(p0).scale(0.5));
        let planes = [
            (outward_of(side_first.3, p0, centroid), p0),
            (outward_of(side_second.3, p0, centroid), p0),
            (chord_normal, ca),
            (outward_of(ends[0].normal, ends[0].point, centroid), ends[0].point),
            (outward_of(ends[1].normal, ends[1].point, centroid), ends[1].point),
        ];
        let clear = tol * 10.0;
        for (index, face) in faces.iter().enumerate() {
            if edited.contains(&index) {
                continue;
            }
            let controls: Vec<Vec3> = face.surface.control_points.iter().flatten().filter_map(|c| c.point().ok()).collect();
            let separated = planes.iter().any(|(n, on)| controls.iter().all(|x| n.dot(x.sub(*on)) >= -clear));
            if !separated {
                return None;
            }
        }
    }
    // The section carried along the edge onto each end face (control by control).
    let carried = |end: &End| -> Option<NurbsCurve> {
        let mut controls = Vec::with_capacity(section.control_points.len());
        for c in &section.control_points {
            let x = c.point().ok()?;
            let lambda = end.normal.dot(end.point.sub(x)) / end.normal.dot(e);
            controls.push(crate::Vec4::from_point(x.add(e.scale(lambda)), c.w));
        }
        NurbsCurve::new(section.degree, section.knots.clone(), controls).ok()
    };
    let trims = [carried(&ends[0])?, carried(&ends[1])?];
    // The blend's carrier: the section extruded along the edge over the trims'
    // reach, v in [0, 1] for λ in [lo, hi].
    let reach = |curve: &NurbsCurve| -> Vec<f64> { curve.control_points.iter().filter_map(|c| c.point().ok()).map(|x| x.sub(p0).dot(e)).collect() };
    let all: Vec<f64> = reach(&trims[0]).into_iter().chain(reach(&trims[1])).collect();
    let margin = 0.02 * length;
    let (lo, hi) = (all.iter().copied().fold(f64::INFINITY, f64::min) - margin, all.iter().copied().fold(f64::NEG_INFINITY, f64::max) + margin);
    let net: Vec<Vec<crate::Vec4>> = section
        .control_points
        .iter()
        .map(|c| {
            let x = c.point().unwrap_or(p0);
            vec![crate::Vec4::from_point(x.add(e.scale(lo)), c.w), crate::Vec4::from_point(x.add(e.scale(hi)), c.w)]
        })
        .collect();
    let carrier = crate::NurbsSurface::new(section.degree, 1, section.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], net).ok()?;
    let v_of = |x: Vec3| (x.sub(p0).dot(e) - lo) / (hi - lo);

    let mut result = solid.clone();
    let mut next = result.vertices.iter().map(|v| v.id).chain(result.edges.iter().map(|x| x.id)).chain(result.shells.iter().flat_map(|s| s.faces.iter()).flat_map(|f| std::iter::once(f.id).chain(f.loops.iter().flat_map(|l| std::iter::once(l.id).chain(l.coedges.iter().map(|c| c.id)))))).max().unwrap_or(0) + 1;
    let mut id = || {
        next += 1;
        next
    };
    // New vertices: the four contacts.
    let mut contact_vertex = [[0u64; 2]; 2]; // [end][side]
    for (k, end) in ends.iter().enumerate() {
        for s in 0..2 {
            let v = id();
            result.vertices.push(VertexRecord { id: v, point: end.contacts[s] });
            contact_vertex[k][s] = v;
        }
    }
    // The edges along the side faces, shortened to the contacts.
    for (k, end) in ends.iter().enumerate() {
        for s in 0..2 {
            let edge = result.edges.iter_mut().find(|x| x.id == end.along[s])?;
            let (a, b) = line_ends(edge)?;
            let (start, finish) = if edge.start_vertex_id == end.vertex {
                edge.start_vertex_id = contact_vertex[k][s];
                (end.contacts[s], b)
            } else {
                edge.end_vertex_id = contact_vertex[k][s];
                (a, end.contacts[s])
            };
            edge.curve = make_line(start, finish).ok()?;
            edge.t0 = 0.0;
            edge.t1 = 1.0;
        }
    }
    // New edges: the contact lines (start end -> far end) and the trims (first side -> second side).
    let mut contact_line = [0u64; 2];
    for s in 0..2 {
        let x = id();
        result.edges.push(EdgeRecord { id: x, curve: make_line(ends[0].contacts[s], ends[1].contacts[s]).ok()?, t0: 0.0, t1: 1.0, start_vertex_id: contact_vertex[0][s], end_vertex_id: contact_vertex[1][s], degenerate: false, name: None });
        contact_line[s] = x;
    }
    let mut trim_edge = [0u64; 2];
    for k in 0..2 {
        let [d0, d1] = trims[k].domain().ok()?;
        let x = id();
        result.edges.push(EdgeRecord { id: x, curve: trims[k].clone(), t0: d0, t1: d1, start_vertex_id: contact_vertex[k][0], end_vertex_id: contact_vertex[k][1], degenerate: false, name: None });
        trim_edge[k] = x;
    }
    result.edges.retain(|x| x.id != edge_id);
    result.vertices.retain(|v| v.id != ends[0].vertex && v.id != ends[1].vertex);
    // Re-read every pcurve on the planar faces whose edges moved, and place the
    // new edges in their loops.
    let edge_of = |result: &BrepSolid, x: u64| result.edges.iter().find(|r| r.id == x).cloned();
    let moved: Vec<u64> = ends.iter().flat_map(|end| end.along).collect();
    let snapshot = result.clone();
    let mut blend_uses: Vec<(u64, bool)> = Vec::new(); // (edge, the neighbour's forward)
    for fi in 0..result.shells[0].faces.len() {
        let face_copy = snapshot.shells[0].faces[fi].clone();
        for li in 0..face_copy.loops.len() {
            let mut coedges = Vec::new();
            let ring = &face_copy.loops[li];
            let n = ring.coedges.len();
            for ci in 0..n {
                let c = &ring.coedges[ci];
                if c.edge_id == edge_id {
                    // The side face's contact line, walked as the edge was.
                    let s = if fi == side_first.0 { 0 } else { 1 };
                    let line = edge_of(&result, contact_line[s])?;
                    let (from, to) = if c.forward { (ends[0].contacts[s], ends[1].contacts[s]) } else { (ends[1].contacts[s], ends[0].contacts[s]) };
                    let pcurve = line2(chart_uv(&face_copy, from)?, chart_uv(&face_copy, to)?)?;
                    let _ = line;
                    coedges.push(CoedgeRecord { id: id(), edge_id: contact_line[s], forward: c.forward, pcurve });
                    blend_uses.push((contact_line[s], c.forward));
                    continue;
                }
                if moved.contains(&c.edge_id) {
                    let x = edge_of(&result, c.edge_id)?;
                    let (a, b) = line_ends(&x)?;
                    let (from, to) = if c.forward { (a, b) } else { (b, a) };
                    let pcurve = line2(chart_uv(&face_copy, from)?, chart_uv(&face_copy, to)?)?;
                    coedges.push(CoedgeRecord { id: c.id, edge_id: c.edge_id, forward: c.forward, pcurve });
                    // On an end face, the trim goes between its two moved edges at the corner.
                    if let Some(k) = ends.iter().position(|end| end.end_face == fi) {
                        let next_c = &ring.coedges[(ci + 1) % n];
                        if ends[k].along.contains(&c.edge_id) && ends[k].along.contains(&next_c.edge_id) && next_c.edge_id != c.edge_id {
                            // Walked from this edge's end (a contact) to the next edge's start.
                            let here = walked(&x, c.forward).1;
                            let forward = here == contact_vertex[k][0];
                            let trim = edge_of(&result, trim_edge[k])?;
                            let mapped = mapped_on_plane(&face_copy, &trim.curve)?;
                            let pcurve = if forward { mapped } else { mapped.reversed().ok()? };
                            coedges.push(CoedgeRecord { id: id(), edge_id: trim_edge[k], forward, pcurve });
                            blend_uses.push((trim_edge[k], forward));
                        }
                    }
                    continue;
                }
                coedges.push(c.clone());
            }
            result.shells[0].faces[fi].loops[li].coedges = coedges;
        }
    }
    if blend_uses.len() != 4 {
        return None;
    }
    // The blend face's loop: each new edge walked against its neighbour, chained.
    let uv_on_blend = |x: Vec3, u: f64| [u, v_of(x)];
    let mut pieces: Vec<(u64, bool, NurbsCurve, u64, u64)> = Vec::new(); // edge, forward, pcurve, from, to
    for &(x, neighbour_forward) in &blend_uses {
        let forward = !neighbour_forward;
        let record = edge_of(&result, x)?;
        let (from, to) = walked(&record, forward);
        let pcurve = if let Some(s) = contact_line.iter().position(|&c| c == x) {
            let u = if s == 0 { 0.0 } else { 1.0 };
            let (a, b) = if forward { (ends[0].contacts[s], ends[1].contacts[s]) } else { (ends[1].contacts[s], ends[0].contacts[s]) };
            line2(uv_on_blend(a, u), uv_on_blend(b, u))?
        } else {
            // A trim: at the section's own parameter, v read off its points.
            let [d0, d1] = record.curve.domain().ok()?;
            let samples = 48;
            let params: Vec<f64> = (0..=samples).map(|i| d0 + (d1 - d0) * i as f64 / samples as f64).collect();
            let points: Vec<Vec3> = params.iter().map(|&t| record.curve.evaluate(t).map(|x| Vec3::new(t, v_of(x), 0.0))).collect::<Result<_, _>>().ok()?;
            let fitted = crate::interpolate_curve(&points, 5, &params).ok()?;
            if forward { fitted } else { fitted.reversed().ok()? }
        };
        pieces.push((x, forward, pcurve, from, to));
    }
    let mut chain = vec![pieces.remove(0)];
    while !pieces.is_empty() {
        let last_to = chain.last()?.4;
        let at = pieces.iter().position(|p| p.3 == last_to)?;
        chain.push(pieces.remove(at));
    }
    if chain.last()?.4 != chain[0].3 {
        return None;
    }
    // Its sense: the loop's winding in (u, v); the carrier's normal must then
    // point out, toward the edge the blend replaces.
    let mut winding = 0.0f64;
    for (_, _, pcurve, _, _) in &chain {
        let [a, b] = pcurve.domain().ok()?;
        let mut last = pcurve.evaluate(a).ok()?;
        for k in 1..=32 {
            let next = pcurve.evaluate(a + (b - a) * k as f64 / 32.0).ok()?;
            winding += last.x * next.y - next.x * last.y;
            last = next;
        }
    }
    let same_sense = winding > 0.0;
    let mid = section.evaluate(0.5 * (s0 + s1)).ok()?;
    let d = carrier.derivatives(0.5 * (s0 + s1), v_of(mid), 1).ok()?;
    let normal = d[1][0].cross(d[0][1]);
    let outward = p0.sub(mid);
    if (normal.dot(outward) > 0.0) != same_sense {
        return None;
    }
    let coedges = chain.into_iter().map(|(x, forward, pcurve, _, _)| CoedgeRecord { id: id(), edge_id: x, forward, pcurve }).collect();
    let face_id = id();
    let loop_id = id();
    result.shells[0].faces.push(FaceRecord { id: face_id, surface: carrier, same_sense, loops: vec![crate::topology::LoopRecord { id: loop_id, coedges }], name: name.map(str::to_string) });
    result.mass_properties_cache = Default::default();
    if result.validate().iter().any(|issue| issue.severity == "error") {
        return None;
    }
    Some(result)
}
