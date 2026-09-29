use super::*;

/// Coedge uses by edge ID, counting both uses of a seam independently.
pub(crate) fn edge_use_counts(solid: &BrepSolid) -> rustc_hash::FxHashMap<u64, usize> {
    let mut counts = rustc_hash::FxHashMap::default();
    for coedge in solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
    {
        *counts.entry(coedge.edge_id).or_default() += 1;
    }
    counts
}

/// Nearest successfully projected edge and its distance, clamped to the edge's
/// trimmed interval. Strict comparison preserves edge order when distances tie.
pub(crate) fn nearest_edge(solid: &BrepSolid, point: Vec3) -> Option<(u64, f64)> {
    let mut best: Option<(u64, f64)> = None;
    for edge in &solid.edges {
        let Ok(projection) = crate::project_point_to_curve(&edge.curve, point) else {
            continue;
        };
        let clamped = projection
            .u
            .clamp(edge.t0.min(edge.t1), edge.t0.max(edge.t1));
        let Ok(sample) = edge.curve.evaluate(clamped) else {
            continue;
        };
        let distance = sample.sub(point).length();
        if best.map(|(_, known)| distance < known).unwrap_or(true) {
            best = Some((edge.id, distance));
        }
    }
    best
}

/// Match edge endpoints within tolerance; `true` means reversed. If both
/// directions match, preserve the sewing code's preference for reversal.
pub(crate) fn edge_endpoint_alignment(
    first: &EdgeRecord,
    second: &EdgeRecord,
    points: &HashMap<u64, Vec3>,
    tolerance: f64,
) -> Option<bool> {
    let direct = points[&first.start_vertex_id]
        .sub(points[&second.start_vertex_id])
        .length()
        <= tolerance
        && points[&first.end_vertex_id]
            .sub(points[&second.end_vertex_id])
            .length()
            <= tolerance;
    let reversed = points[&first.start_vertex_id]
        .sub(points[&second.end_vertex_id])
        .length()
        <= tolerance
        && points[&first.end_vertex_id]
            .sub(points[&second.start_vertex_id])
            .length()
            <= tolerance;
    (direct || reversed).then_some(reversed)
}

/// Previous pcurve end and next pcurve start around the first use of an edge.
/// Missing edges or unevaluable neighbors yield `None`.
pub(crate) fn coedge_neighbor_endpoints<'a>(
    loops: impl IntoIterator<Item = &'a LoopRecord>,
    edge_id: u64,
) -> Option<(Vec3, Vec3)> {
    let loop_record = loops.into_iter().find(|loop_record| {
        loop_record
            .coedges
            .iter()
            .any(|coedge| coedge.edge_id == edge_id)
    })?;
    let count = loop_record.coedges.len();
    let index = loop_record
        .coedges
        .iter()
        .position(|coedge| coedge.edge_id == edge_id)?;
    let prev = &loop_record.coedges[(index + count - 1) % count];
    let next = &loop_record.coedges[(index + 1) % count];
    let prev_domain = prev.pcurve.domain().ok()?;
    let next_domain = next.pcurve.domain().ok()?;
    Some((
        prev.pcurve.evaluate(prev_domain[1]).ok()?,
        next.pcurve.evaluate(next_domain[0]).ok()?,
    ))
}

