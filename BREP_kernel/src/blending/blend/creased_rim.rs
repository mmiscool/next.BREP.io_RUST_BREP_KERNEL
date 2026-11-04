//! A closed rim whose only vertex is a CORNER of the rim and the crease of
//! its periodic mate's seam (refusal census 2026-10-26 row 2): a right prism
//! whose wall is ONE face whose profile closes with a C0 corner, so the wall
//! is creased along its own seam and the rim has a corner at its closing
//! point.
//!
//! The general closed-edge march anchors its first station ON that seam
//! meridian (`stations.rs::solve_anchored_start`), where the wall has two
//! one-sided normals, and the anchored Newton is singular; an unanchored lap
//! would have to cross the crease mid-march, where a periodic row fit cannot
//! follow the kink, because the ball's section there is not a two-surface
//! tangency station at all.  What the rim needs is the two-edge MITER at the
//! crease (`miter.rs`: two stripes meeting at a vertex whose third edge stays
//! sharp) and a flush join wherever the rim is smooth — exactly what the
//! stripe network builds for a prism whose wall is two faces.
//!
//! So the body is CANONICALISED before the selection is resolved: the creased
//! mate is split along a smooth meridian (the midpoint of its periodic
//! domain) into two faces, every edge of its loop that crosses that meridian
//! into two edges, and both pieces of a selected rim are selected.  The two
//! faces ride two OPEN carriers, the periodic surface's two halves (knot
//! insertion at the meridian): on the periodic carrier itself a march past
//! the crease wraps onto the far sheet and the into-material probe at the
//! crease projects onto whichever sheet is nearer, so the network's miter
//! could not even orient the sharp edge.  The network then sees a sharp
//! convex vertex at the crease (the miter) and a G1 vertex at the split (the
//! flush join), both of which it already constructs.  After the blend the
//! two wall faces are put back on the periodic carrier (their pcurves keep
//! its parameters) and merged across the split (`merge_faces_within`), so
//! the result carries the one wall the input carried; when the merge
//! declines, the split body — valid, with the wall in two faces — is the
//! answer, never a refusal.
//!
//! The class is narrow by construction: a selected CLOSED edge whose unit
//! tangents at its two ends differ (a corner at its one vertex), with a mate
//! whose loop uses one edge TWICE (the seam) at that vertex.  A smooth cap
//! rim on a cylinder keeps its tangent across the seam and is untouched; a
//! rim between two wall faces is not closed.

//! Default ON since lane R's promotion (2026-10-28); `BREP_CREASED_RIM_SPLIT=0`
//! (or `off`) restores the unsplit body, and `with_creased_rim_split_for_test`
//! forces either on a test's thread. Each rim piece carries the OPEN sub-curve
//! over its own range: kept on the closed rim's curve, a piece's extension
//! wrapped round the crease (closed curves wrap in `derivatives_extended`), the
//! stripe's overshoot followed the far side of lCount_12's 48° corner and
//! folded back over its own wall, and the crease miter refused at its seam fit
//! (1.08e-4 against 2e-6).

use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, Vec3, Vec4};

/// Whether the creased-rim canonicalisation runs: ON unless
/// `BREP_CREASED_RIM_SPLIT=0` (or `off`), or a test's forced value.  Off, the
/// creased rim refuses at the anchored seam start as it did before the split.
pub(crate) fn creased_rim_split_on() -> bool {
    switch!("BREP_CREASED_RIM_SPLIT", true)
}


/// The canonicalised body and selection, and what it takes to fold the
/// result back onto the caller's selection.
pub(crate) struct CreasedRimSplit {
    pub(crate) solid: BrepSolid,
    pub(crate) points: Vec<Vec3>,
    pub(crate) names: Option<Vec<String>>,
    /// The two faces each creased mate was split into and the periodic
    /// carrier they came off, to merge back.
    pub(crate) wall_pairs: Vec<(u64, u64, NurbsSurface)>,
}

impl CreasedRimSplit {
    /// The blended body with every split wall merged back into one face where
    /// the merge validates; a wall the merge declines stays in two faces.
    pub(crate) fn rejoin(&self, blended: BrepSolid) -> BrepSolid {
        let mut result = blended;
        for (a, b, carrier) in &self.wall_pairs {
            let present = |id: u64| result.shells.iter().flat_map(|shell| &shell.faces).any(|face| face.id == id);
            if !present(*a) || !present(*b) {
                continue;
            }
            // Both halves back on the periodic carrier: the halves were cut
            // from it by knot insertion, so every pcurve on them reads the
            // same point on it.
            let mut rejoined = result.clone();
            for face in rejoined.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
                if face.id == *a || face.id == *b {
                    face.surface = carrier.clone();
                }
            }
            if let Ok((merged, _)) = crate::merge_faces_within(&rejoined, 1e-7, &[*a, *b]) {
                if merged.validate().is_empty() {
                    result = merged;
                }
            }
        }
        result
    }
}

/// The surface cut in two at the parameter `at` of its u direction (`in_u`)
/// or its v direction: knot insertion to full multiplicity on every row of
/// the net, each half clamped there.  Both halves keep the parameters of the
/// whole, so a pcurve on the whole is a pcurve on the half its domain lies in.
fn split_surface(surface: &NurbsSurface, in_u: bool, at: f64) -> Result<(NurbsSurface, NurbsSurface), String> {
    let rows = surface.control_points.len();
    let columns = surface.control_points.first().map_or(0, |row| row.len());
    if rows == 0 || columns == 0 {
        return Err("creased rim: an empty control net".to_string());
    }
    let zero = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
    if in_u {
        let mut lows = Vec::with_capacity(columns);
        let mut highs = Vec::with_capacity(columns);
        for j in 0..columns {
            let points: Vec<Vec4> = surface.control_points.iter().map(|row| row[j]).collect();
            let (low, high) = NurbsCurve::new(surface.degree_u, surface.knots_u.clone(), points)?.split(at)?;
            lows.push(low);
            highs.push(high);
        }
        let build = |curves: &[NurbsCurve]| -> Result<NurbsSurface, String> {
            let n = curves[0].control_points.len();
            let mut net = vec![vec![zero; columns]; n];
            for (j, curve) in curves.iter().enumerate() {
                if curve.control_points.len() != n {
                    return Err("creased rim: the split rows disagree".to_string());
                }
                for (i, point) in curve.control_points.iter().enumerate() {
                    net[i][j] = *point;
                }
            }
            NurbsSurface::new(surface.degree_u, surface.degree_v, curves[0].knots.clone(), surface.knots_v.clone(), net)
        };
        Ok((build(&lows)?, build(&highs)?))
    } else {
        let mut lows = Vec::with_capacity(rows);
        let mut highs = Vec::with_capacity(rows);
        for row in &surface.control_points {
            let (low, high) = NurbsCurve::new(surface.degree_v, surface.knots_v.clone(), row.clone())?.split(at)?;
            lows.push(low);
            highs.push(high);
        }
        let build = |curves: &[NurbsCurve]| -> Result<NurbsSurface, String> {
            let n = curves[0].control_points.len();
            let mut net = Vec::with_capacity(rows);
            for curve in curves {
                if curve.control_points.len() != n {
                    return Err("creased rim: the split rows disagree".to_string());
                }
                net.push(curve.control_points.clone());
            }
            NurbsSurface::new(surface.degree_u, surface.degree_v, surface.knots_u.clone(), curves[0].knots.clone(), net)
        };
        Ok((build(&lows)?, build(&highs)?))
    }
}

/// A rim edge's corner at its one vertex, read from its own curve: the unit
/// tangents at the two ends.
fn closed_edge_has_corner(edge: &EdgeRecord) -> bool {
    let tangent = |t: f64| -> Option<Vec3> {
        let derivatives = edge.curve.derivatives(t, 1).ok()?;
        derivatives.get(1).copied()?.normalized().ok()
    };
    match (tangent(edge.t0), tangent(edge.t1)) {
        (Some(a), Some(b)) => a.dot(b) < 1.0 - 1e-6,
        _ => false,
    }
}

/// The vertices a loop traverses `coedge` between, in traversal order.
fn coedge_ends(solid: &BrepSolid, coedge: &CoedgeRecord) -> Result<(u64, u64), KernelRefusal> {
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == coedge.edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "creased_rim_edge", format!("creased rim: coedge {} names a missing edge {}", coedge.id, coedge.edge_id)))?;
    Ok(if coedge.forward { (edge.start_vertex_id, edge.end_vertex_id) } else { (edge.end_vertex_id, edge.start_vertex_id) })
}

/// The periodic direction of a seam coedge's pcurve: `true` when the seam
/// runs along v at constant u (the carrier is periodic in u).
fn seam_runs_in_v(coedge: &CoedgeRecord) -> Option<bool> {
    let [q0, q1] = coedge.pcurve.domain().ok()?;
    let a = coedge.pcurve.evaluate(q0).ok()?;
    let b = coedge.pcurve.evaluate(q1).ok()?;
    let (du, dv) = ((a.x - b.x).abs(), (a.y - b.y).abs());
    if du <= 1e-9 * (1.0 + dv) {
        Some(true)
    } else if dv <= 1e-9 * (1.0 + du) {
        Some(false)
    } else {
        None
    }
}

/// The face and loop holding a coedge on `edge_id` whose loop also uses
/// some edge twice at `vertex` (a seam-structured loop): the creased mate.
fn creased_mate<'a>(solid: &'a BrepSolid, edge_id: u64, vertex: u64) -> Option<(&'a FaceRecord, usize, u64)> {
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for (loop_index, loop_record) in face.loops.iter().enumerate() {
            if !loop_record.coedges.iter().any(|coedge| coedge.edge_id == edge_id) {
                continue;
            }
            for coedge in &loop_record.coedges {
                let uses = loop_record.coedges.iter().filter(|other| other.edge_id == coedge.edge_id).count();
                if uses != 2 {
                    continue;
                }
                let touches = solid
                    .edges
                    .iter()
                    .find(|edge| edge.id == coedge.edge_id)
                    .is_some_and(|edge| edge.start_vertex_id == vertex || edge.end_vertex_id == vertex);
                if touches {
                    return Some((face, loop_index, coedge.edge_id));
                }
            }
        }
    }
    None
}

/// Where a coedge's pcurve crosses the meridian `u = at` (or `v = at` when
/// `in_u` is false): the EDGE parameter and the point, by bisection on the
/// pcurve, or `None` when the pcurve stays on one side.
fn meridian_crossing(solid: &BrepSolid, coedge: &CoedgeRecord, in_u: bool, at: f64) -> Result<Option<(f64, Vec3, f64)>, KernelRefusal> {
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == coedge.edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "creased_rim_edge", format!("creased rim: coedge {} names a missing edge {}", coedge.id, coedge.edge_id)))?;
    let coordinate = |t: f64| -> Result<[f64; 2], KernelRefusal> {
        let uv = super::stations::edge_uv_on_face(coedge, edge, t)?;
        Ok(uv)
    };
    let side = |uv: [f64; 2]| if in_u { uv[0] - at } else { uv[1] - at };
    let (mut lo, mut hi) = (edge.t0, edge.t1);
    let (mut f_lo, mut f_hi) = (side(coordinate(lo)?), side(coordinate(hi)?));
    if f_lo == 0.0 || f_hi == 0.0 || (f_lo < 0.0) == (f_hi < 0.0) {
        return Ok(None);
    }
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        let f_mid = side(coordinate(mid)?);
        if (f_mid < 0.0) == (f_lo < 0.0) {
            lo = mid;
            f_lo = f_mid;
        } else {
            hi = mid;
            f_hi = f_mid;
        }
        if hi - lo <= 1e-15 * (1.0 + (edge.t1 - edge.t0).abs()) {
            break;
        }
    }
    let _ = f_hi;
    let t = 0.5 * (lo + hi);
    let uv = coordinate(t)?;
    let point = edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
    let other = if in_u { uv[1] } else { uv[0] };
    Ok(Some((t, point, other)))
}

/// Split every creased mate of a selected corner rim along a smooth meridian
/// and select both pieces of each such rim.  `None` when the selection holds
/// no rim of the class, so the caller's body and selection are used as they
/// came; a body the split cannot prepare (a mate with holes, a meridian the
/// loop crosses other than twice) is left alone the same way.
pub(crate) fn split_creased_rims(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    resolve: &dyn Fn(&BrepSolid, Vec3) -> Result<u64, KernelRefusal>,
) -> Result<Option<CreasedRimSplit>, KernelRefusal> {
    // The creased mates, each once, with the rims selected on them.
    let mut mates: Vec<(u64, usize, u64)> = Vec::new();
    let mut selected: Vec<u64> = Vec::with_capacity(edge_points.len());
    for point in edge_points {
        // A point no edge answers to is the ordinary path's refusal, not this one's.
        let Ok(edge_id) = resolve(solid, *point) else { return Ok(None) };
        selected.push(edge_id);
        let Some(edge) = solid.edges.iter().find(|edge| edge.id == edge_id) else { continue };
        if edge.degenerate || edge.start_vertex_id != edge.end_vertex_id || !closed_edge_has_corner(edge) {
            continue;
        }
        let Some((face, loop_index, seam)) = creased_mate(solid, edge_id, edge.start_vertex_id) else { continue };
        if !mates.iter().any(|(id, _, _)| *id == face.id) {
            mates.push((face.id, loop_index, seam));
        }
    }
    if mates.is_empty() {
        return Ok(None);
    }
    let mut result = solid.clone();
    let mut take_id = super::edge::fresh_id_source(solid);
    let mut wall_pairs = Vec::with_capacity(mates.len());
    // Every split edge and its two pieces, for the selection.
    let mut pieces: Vec<(u64, [u64; 2])> = Vec::new();
    for (face_id, loop_index, seam_id) in mates {
        let face = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == face_id)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "creased_rim_face", "creased rim: the mate vanished"))?
            .clone();
        if face.loops.len() != 1 {
            // A mate with holes: the meridian could cut through one.
            return Ok(None);
        }
        let seam_coedge = face.loops[loop_index]
            .coedges
            .iter()
            .find(|coedge| coedge.edge_id == seam_id)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "creased_rim_seam", "creased rim: the seam vanished"))?;
        let Some(in_u) = seam_runs_in_v(seam_coedge) else { return Ok(None) };
        let domain = if in_u { face.surface.domain_u() } else { face.surface.domain_v() }.or_refuse(KernelStage::Refine, "domain")?;
        let at = 0.5 * (domain[0] + domain[1]);
        // The crossings of the meridian, on the loop's own edges.
        let mut crossings: Vec<(u64, f64, Vec3, f64)> = Vec::new();
        for coedge in &face.loops[loop_index].coedges {
            if coedge.edge_id == seam_id {
                continue;
            }
            if let Some((t, point, other)) = meridian_crossing(&result, coedge, in_u, at)? {
                crossings.push((coedge.edge_id, t, point, other));
            }
        }
        if crossings.len() != 2 || crossings[0].0 == crossings[1].0 {
            return Ok(None);
        }
        // The chord: the meridian between the two crossings, on the carrier.
        let iso = if in_u { face.surface.iso_curve_u(at) } else { face.surface.iso_curve_v(at) }.or_refuse(KernelStage::Refine, "iso_curve")?;
        let (first, second) = if crossings[0].3 <= crossings[1].3 { (0, 1) } else { (1, 0) };
        let (lo, hi) = (crossings[first].3, crossings[second].3);
        let [iso0, iso1] = iso.domain().or_refuse(KernelStage::Refine, "domain")?;
        let span = iso1 - iso0;
        if hi - lo <= 1e-9 * span.abs() {
            return Ok(None);
        }
        let mut chord = iso.clone();
        if lo > iso0 + 1e-9 * span.abs() {
            chord = chord.split(lo).or_refuse(KernelStage::Refine, "split")?.1;
        }
        if hi < iso1 - 1e-9 * span.abs() {
            chord = chord.split(hi).or_refuse(KernelStage::Refine, "split")?.0;
        }
        let [c0, c1] = chord.domain().or_refuse(KernelStage::Refine, "domain")?;
        let band = crate::KernelTolerances::for_solid(&result, 1e-7).pcurve_consistency;
        for (parameter, index) in [(c0, first), (c1, second)] {
            let on_chord = chord.evaluate(parameter).or_refuse(KernelStage::Refine, "evaluate")?;
            if on_chord.sub(crossings[index].2).length() > band.max(1e-9) * 1e3 {
                return Ok(None);
            }
        }
        // The crossing vertices, and the edges split at them.
        let mut vertex_ids = [0u64; 2];
        for (slot, index) in [(0usize, first), (1, second)] {
            let id = take_id();
            result.vertices.push(VertexRecord { id, point: crossings[index].2 });
            vertex_ids[slot] = id;
            let sub = super::rollover_surgery::split_edge_at_vertices(&mut result, crossings[index].0, &[(id, crossings[index].2)], &mut take_id)?;
            if sub.len() != 2 {
                return Err(KernelRefusal::internal(KernelStage::Sew, "creased_rim_split", "creased rim: an edge split at one point into other than two pieces"));
            }
            // Each piece carries the OPEN sub-curve over its own range.  The
            // splitter keeps the closed rim's curve, and a closed curve's
            // extension WRAPS (`NurbsCurve::derivatives_extended`): the
            // stripe's march past the crease end then followed the rim round
            // the corner onto the far side, and on lCount_12's 48° corner its
            // overshoot folded back over its own wall (a seam point 1.4e-7 off
            // the stripe at u 0.20 and again on it, reversed, at u 0.25).  An
            // open piece extends along its end tangent, as every open edge
            // does.  Same parameters, so every pcurve keeps its map.
            for piece in &sub {
                let Some(record) = result.edges.iter_mut().find(|candidate| candidate.id == *piece) else { continue };
                let mut curve = record.curve.clone();
                let [c0, c1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
                let band = 1e-12 * (c1 - c0).abs().max(1.0);
                if record.t0 > c0 + band {
                    curve = curve.split(record.t0).or_refuse(KernelStage::Refine, "split")?.1;
                }
                if record.t1 < c1 - band {
                    curve = curve.split(record.t1).or_refuse(KernelStage::Refine, "split")?.0;
                }
                record.curve = curve;
            }
            pieces.push((crossings[index].0, [sub[0], sub[1]]));
        }
        let chord_edge_id = take_id();
        result.edges.push(EdgeRecord {
            id: chord_edge_id,
            curve: chord,
            t0: c0,
            t1: c1,
            start_vertex_id: vertex_ids[0],
            end_vertex_id: vertex_ids[1],
            degenerate: false,
            name: None,
        });
        let chord_pcurve = {
            let uv = |p: f64| if in_u { Vec4 { x: at, y: p, z: 0.0, w: 1.0 } } else { Vec4 { x: p, y: at, z: 0.0, w: 1.0 } };
            NurbsCurve::new(1, vec![c0, c0, c1, c1], vec![uv(c0), uv(c1)]).or_refuse(KernelStage::Refine, "new")?
        };
        // The loop, cut at the two crossing vertices into the two walks the
        // chord closes.
        let (shell_index, face_index) = result
            .shells
            .iter()
            .enumerate()
            .find_map(|(s, shell)| shell.faces.iter().position(|face| face.id == face_id).map(|f| (s, f)))
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "creased_rim_face", "creased rim: the mate vanished"))?;
        let coedges = result.shells[shell_index].faces[face_index].loops[loop_index].coedges.clone();
        let mut ends = Vec::with_capacity(coedges.len());
        for coedge in &coedges {
            ends.push(coedge_ends(&result, coedge)?);
        }
        let n = coedges.len();
        let Some(i) = (0..n).find(|k| ends[*k].1 == vertex_ids[0]) else { return Ok(None) };
        let Some(j) = (0..n).find(|k| ends[*k].1 == vertex_ids[1]) else { return Ok(None) };
        if i == j {
            return Ok(None);
        }
        let walk = |from: usize, to: usize| -> Vec<CoedgeRecord> {
            // Coedges after `from`, up to and including `to`, cyclically.
            let mut out = Vec::new();
            let mut k = (from + 1) % n;
            loop {
                out.push(coedges[k].clone());
                if k == to {
                    break;
                }
                k = (k + 1) % n;
            }
            out
        };
        // Walk A runs from vertex 0 to vertex 1; the chord closes it from
        // vertex 1 back to vertex 0 (reversed).  Walk B runs from vertex 1 to
        // vertex 0; the chord closes it forward.
        let mut loop_a = walk(i, j);
        loop_a.push(CoedgeRecord {
            id: take_id(),
            edge_id: chord_edge_id,
            forward: false,
            pcurve: chord_pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?,
        });
        let mut loop_b = walk(j, i);
        loop_b.push(CoedgeRecord { id: take_id(), edge_id: chord_edge_id, forward: true, pcurve: chord_pcurve.clone() });
        // The carrier's two halves, each loop on the half its own edges lie in
        // (read off a coedge of the loop other than the chord).
        let (low, high) = match split_surface(&face.surface, in_u, at) {
            Ok(halves) => halves,
            Err(_) => return Ok(None),
        };
        let side_of = |coedges: &[CoedgeRecord]| -> Option<bool> {
            let coedge = coedges.iter().find(|coedge| coedge.edge_id != chord_edge_id)?;
            let [q0, q1] = coedge.pcurve.domain().ok()?;
            let middle = coedge.pcurve.evaluate(0.5 * (q0 + q1)).ok()?;
            Some((if in_u { middle.x } else { middle.y }) < at)
        };
        let (Some(a_low), Some(b_low)) = (side_of(&loop_a), side_of(&loop_b)) else { return Ok(None) };
        if a_low == b_low {
            return Ok(None);
        }
        let (surface_a, surface_b) = if a_low { (low, high) } else { (high, low) };
        let new_face_id = take_id();
        let new_loop_id = take_id();
        let carrier = {
            let face = &mut result.shells[shell_index].faces[face_index];
            let carrier = std::mem::replace(&mut face.surface, surface_a);
            face.loops[loop_index].coedges = loop_a;
            let name = face.name.as_ref().map(|name| format!("{name}_2"));
            let sibling = FaceRecord {
                id: new_face_id,
                surface: surface_b,
                same_sense: face.same_sense,
                loops: vec![LoopRecord { id: new_loop_id, coedges: loop_b }],
                name,
            };
            result.shells[shell_index].faces.push(sibling);
            carrier
        };
        wall_pairs.push((face_id, new_face_id, carrier));
    }
    // Both pieces of every selected split rim are selected: the caller's point
    // resolves to one piece, the other piece's midpoint is appended.
    let mut points = edge_points.to_vec();
    let mut names = edge_names.map(|names| names.to_vec());
    let mut appended = 0usize;
    for (index, edge_id) in selected.iter().enumerate() {
        let Some((_, sub)) = pieces.iter().find(|(original, _)| original == edge_id) else { continue };
        let on = resolve(&result, edge_points[index])?;
        let other = if on == sub[0] { sub[1] } else if on == sub[1] { sub[0] } else { continue };
        let edge = result
            .edges
            .iter()
            .find(|edge| edge.id == other)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "creased_rim_piece", "creased rim: a piece vanished"))?;
        let midpoint = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).or_refuse(KernelStage::Refine, "evaluate")?;
        points.push(midpoint);
        if let Some(names) = names.as_mut() {
            let name = names.get(index).cloned().unwrap_or_default();
            names.push(name);
        }
        appended += 1;
    }
    if appended == 0 {
        return Ok(None);
    }
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        eprintln!(
            "creased rim: {} mate(s) split along a smooth meridian, selection {} -> {} edges",
            wall_pairs.len(),
            edge_points.len(),
            points.len()
        );
    }
    Ok(Some(CreasedRimSplit { solid: result, points, names, wall_pairs }))
}
