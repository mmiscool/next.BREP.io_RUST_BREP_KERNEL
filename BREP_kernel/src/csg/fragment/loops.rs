use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
use super::types::{Chain, ChainSource};
use super::sampling::{point_segment_distance, subcurve_by_fraction};
pub(super) fn edge_for<'a>(solid: &'a BrepSolid, edge_id: u64) -> Result<&'a EdgeRecord, KernelRefusal> {
    solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", format!("fragment_face: missing edge {edge_id}")))
}

pub(super) fn vertex_point(solid: &BrepSolid, vertex_id: u64) -> Result<Vec3, KernelRefusal> {
    solid
        .vertices
        .iter()
        .find(|vertex| vertex.id == vertex_id)
        .map(|vertex| vertex.point)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", format!("fragment_face: missing vertex {vertex_id}")))
}

pub(super) fn pcurve_for<'a>(piece: &'a ImprintPieceRecord, face: FaceKey) -> Result<&'a NurbsCurve, KernelRefusal> {
    piece
        .pcurves
        .iter()
        .find(|pcurve| pcurve.operand == face.operand && pcurve.face_id == face.face_id)
        .map(|pcurve| &pcurve.pcurve)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", "fragment_face: imprint lacks face pcurve"))
}

pub(super) fn point_in_polygon(point: Vec2, polygon: &[Vec2]) -> bool {
    crate::polygon::parity_inside([point.x, point.y], polygon.len(), |i| [polygon[i].x, polygon[i].y])
}

/// PARITY INDEX (lane O row 17, 2026-10-28; on by default since the batch 4
/// promotion, `BREP_FRAGMENT_PARITY_INDEX=0` reads every edge). [`point_in_polygon`] toggles on exactly the edges whose
/// half-open y-span `[min, max)` holds the query's y, and the toggles commute,
/// so reading only those edges, each by the same predicate and the same
/// crossing expression, returns the same answer. The interior-point search
/// asks it once per polygon edge per pass (six passes at most), which on the
/// D1 bench case's 272-edge regions was the whole of `fragment.region
/// interior`; the index buckets the edges by y so a query reads the few edges
/// of its bucket. Every edge is registered in each bucket its closed y-span
/// touches, and the bucket map is monotone in y, so no contributing edge is
/// skipped.
pub(super) struct PolygonParityIndex<'a> {
    polygon: &'a [Vec2],
    low: f64,
    scale: f64,
    buckets: Vec<Vec<u32>>,
}
impl<'a> PolygonParityIndex<'a> {
    pub(super) fn new(polygon: &'a [Vec2]) -> Option<Self> {
        if polygon.len() < 64 || polygon.len() >= u32::MAX as usize {
            return None;
        }
        let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
        for point in polygon {
            if !(point.x.is_finite() && point.y.is_finite()) {
                return None;
            }
            low = low.min(point.y);
            high = high.max(point.y);
        }
        if !(high > low) {
            return None;
        }
        let count = ((polygon.len() as f64).sqrt().ceil() as usize).clamp(8, 256);
        let scale = count as f64 / (high - low);
        if !scale.is_finite() {
            return None;
        }
        let mut index = Self { polygon, low, scale, buckets: vec![Vec::new(); count] };
        for edge in 0..polygon.len() {
            let a = polygon[edge];
            let b = polygon[(edge + 1) % polygon.len()];
            let (first, last) = (index.bucket(a.y.min(b.y)), index.bucket(a.y.max(b.y)));
            for bucket in first..=last {
                index.buckets[bucket].push(edge as u32);
            }
        }
        Some(index)
    }
    fn bucket(&self, y: f64) -> usize {
        let at = ((y - self.low) * self.scale).floor();
        if at.is_nan() || at < 0.0 { 0 } else { (at as usize).min(self.buckets.len() - 1) }
    }
    pub(super) fn contains(&self, point: Vec2) -> bool {
        let mut inside = false;
        for &edge in &self.buckets[self.bucket(point.y)] {
            let index = edge as usize;
            let a = self.polygon[index];
            let b = self.polygon[(index + 1) % self.polygon.len()];
            if crate::polygon::crosses_ray([point.x, point.y], [a.x, a.y], [b.x, b.y]) {
                inside = !inside;
            }
        }
        inside
    }
}

pub(super) fn parity_index_enabled() -> bool {
    switch!("BREP_FRAGMENT_PARITY_INDEX", true)
}


/// Interior points of the region, best-clearance first.  The first entry
/// is the same point the single-point selection historically produced;
/// later entries are additional well-separated interior points used as
/// extra classification votes in boolean fragment selection.
pub(super) fn interior_points(
    outer: &[crate::CycleUse],
    holes: &[Vec<crate::CycleUse>],
    area: f64,
    count: usize,
) -> Vec<Vec2> {
    let polygon = |cycle: &[crate::CycleUse]| {
        cycle
            .iter()
            .map(|usage| {
                if usage.forward {
                    usage.piece.a
                } else {
                    usage.piece.b
                }
            })
            .collect::<Vec<_>>()
    };
    let outer_polygon = polygon(outer);
    let hole_polygons = holes.iter().map(|hole| polygon(hole)).collect::<Vec<_>>();
    interior_points_in_polygons(&outer_polygon, &hole_polygons, area, count)
}

/// [`interior_points`] kept clear of `strays`: closed there-and-back paths
/// along section pieces that bound no region. Each is read as one more hole
/// boundary, so the clearance every candidate is ranked by counts the distance
/// to it, and a point left within a fiftieth of the region's size of one is
/// dropped: off a tangent contact the two bodies part only quadratically, so a
/// point a hair off the piece still reads as lying on the other body.
pub(super) fn interior_points_clear_of(
    outer: &[crate::CycleUse],
    holes: &[Vec<crate::CycleUse>],
    strays: &[Vec<Vec2>],
    area: f64,
    count: usize,
) -> Vec<Vec2> {
    let polygon = |cycle: &[crate::CycleUse]| {
        cycle.iter().map(|usage| if usage.forward { usage.piece.a } else { usage.piece.b }).collect::<Vec<_>>()
    };
    let outer_polygon = polygon(outer);
    let mut hole_polygons = holes.iter().map(|hole| polygon(hole)).collect::<Vec<_>>();
    hole_polygons.extend(strays.iter().cloned());
    let band = area.abs().sqrt() * 0.02;
    let clear = |point: &Vec2| {
        strays.iter().all(|path| (0..path.len()).all(|i| point_segment_distance(*point, path[i], path[(i + 1) % path.len()]) > band))
    };
    let picked = interior_points_in_polygons(&outer_polygon, &hole_polygons, area, count);
    let kept: Vec<Vec2> = picked.iter().copied().filter(clear).collect();
    // A region so thin about its piece that no candidate clears the band keeps
    // the best-clearance point it has.
    if kept.is_empty() { picked.into_iter().take(1).collect() } else { kept }
}

/// A conservative nearest-segment index. Each leaf still evaluates the
/// original point/segment expression; boxes only discard edges that cannot
/// lower the current distance. Padding encloses roundoff in a+t*(b-a).
struct InteriorSegmentIndex {
    edges: Vec<(Vec2, Vec2)>,
    nodes: Vec<InteriorSegmentNode>,
}
struct InteriorSegmentNode {
    low: Vec2,
    high: Vec2,
    start: usize,
    end: usize,
    children: Option<[usize; 2]>,
}
impl InteriorSegmentIndex {
    fn new(edges: Vec<(Vec2, Vec2)>) -> Self {
        let mut index = Self { edges, nodes: Vec::new() };
        index.build(0, index.edges.len());
        index
    }
    fn build(&mut self, start: usize, end: usize) -> usize {
        let mut low = Vec2 { x: f64::INFINITY, y: f64::INFINITY };
        let mut high = Vec2 { x: f64::NEG_INFINITY, y: f64::NEG_INFINITY };
        for &(a, b) in &self.edges[start..end] {
            for p in [a, b] {
                low.x = low.x.min(p.x); low.y = low.y.min(p.y);
                high.x = high.x.max(p.x); high.y = high.y.max(p.y);
            }
        }
        let pad = 16.0 * f64::EPSILON * low.x.abs().max(low.y.abs()).max(high.x.abs()).max(high.y.abs()).max(1.0);
        low.x -= pad; low.y -= pad; high.x += pad; high.y += pad;
        let id = self.nodes.len();
        self.nodes.push(InteriorSegmentNode { low, high, start, end, children: None });
        if end - start > 8 {
            let x_axis = high.x - low.x >= high.y - low.y;
            self.edges[start..end].sort_by(|(a, b), (c, d)| {
                let first = if x_axis { a.x + b.x } else { a.y + b.y };
                let second = if x_axis { c.x + d.x } else { c.y + d.y };
                first.total_cmp(&second)
            });
            let middle = start + (end - start) / 2;
            let left = self.build(start, middle);
            let right = self.build(middle, end);
            self.nodes[id].children = Some([left, right]);
        }
        id
    }
    fn lower(&self, id: usize, point: Vec2) -> f64 {
        let node = &self.nodes[id];
        point.sub(Vec2 { x: point.x.clamp(node.low.x, node.high.x), y: point.y.clamp(node.low.y, node.high.y) }).length()
    }
    fn visit(&self, id: usize, point: Vec2, best: &mut f64) {
        if self.lower(id, point) > *best { return; }
        let node = &self.nodes[id];
        if let Some([left, right]) = node.children {
            let (first, second) = if self.lower(left, point) <= self.lower(right, point) { (left, right) } else { (right, left) };
            self.visit(first, point, best);
            self.visit(second, point, best);
        } else {
            for &(a, b) in &self.edges[node.start..node.end] {
                *best = best.min(point_segment_distance(point, a, b));
            }
        }
    }
    fn distance(&self, point: Vec2) -> f64 {
        let mut best = f64::INFINITY;
        self.visit(0, point, &mut best);
        best
    }
}


/// [`interior_points`] on the polygons themselves: each cycle use is one
/// polygon edge (its start is the polygon's vertex), so the loop split's
/// sampled loops read the same points the arrangement's cycles would.
pub(super) fn interior_points_in_polygons(
    outer_polygon: &[Vec2],
    hole_polygons: &[Vec<Vec2>],
    area: f64,
    count: usize,
) -> Vec<Vec2> {
    let mode = std::env::var("BREP_FRAGMENT_INTERIOR_INDEX").unwrap_or_else(|_| "on".into());
    let enabled = mode == "on" || mode == "1";
    if mode == "check" {
        let old = interior_points_in_polygons_indexed(outer_polygon, hole_polygons, area, count, false);
        let new = interior_points_in_polygons_indexed(outer_polygon, hole_polygons, area, count, true);
        let identical = old.len() == new.len() && old.iter().zip(&new).all(|(a, b)| a.x.to_bits() == b.x.to_bits() && a.y.to_bits() == b.y.to_bits());
        if let Ok(path) = std::env::var("BREP_INTERIOR_INDEX_CENSUS") {
            use std::io::Write;
            if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                let _ = writeln!(file, "fragment.interior_census identical={identical} edges={} holes={} points={}", outer_polygon.len(), hole_polygons.len(), old.len());
            }
        }
        return old;
    }
    interior_points_in_polygons_indexed(outer_polygon, hole_polygons, area, count, enabled)
}

pub(super) fn interior_points_in_polygons_indexed(
    outer_polygon: &[Vec2], hole_polygons: &[Vec<Vec2>], area: f64, count: usize, indexed: bool,
) -> Vec<Vec2> {
    let scale = area.abs().sqrt();
    let mut seen: Vec<(Vec2, f64)> = Vec::new();
    let select = switch!("BREP_FRAGMENT_INTERIOR_SELECT", true)
        && scale.is_finite() && scale <= 1e100
        && outer_polygon.iter().chain(hole_polygons.iter().flatten())
            .all(|p| p.x.is_finite() && p.y.is_finite() && p.x.abs().max(p.y.abs()) <= 1e100);
    // Candidate, exact clearance if already read, and distance to its own
    // generating edge. The last is an upper bound on the minimum over ALL
    // boundary edges, using the same point/segment expression as the index.
    let mut pending: Vec<(Vec2, Option<f64>, f64)> = Vec::new();
    let mut finite = true;
    let mut queries = 0usize;
    let mut pruned = 0usize;
    let mut best = None;
    let mut best_clearance = -1.0;
    let edges = |polygon: &[Vec2]| -> Vec<(Vec2, Vec2)> {
        (0..polygon.len()).map(|i| (polygon[i], polygon[(i + 1) % polygon.len()])).collect()
    };
    let outer_edges = edges(outer_polygon);
    let hole_edges: Vec<Vec<(Vec2, Vec2)>> = hole_polygons.iter().map(|hole| edges(hole)).collect();
    let edge_count = outer_edges.len() + hole_edges.iter().map(Vec::len).sum::<usize>();
    let index = (indexed && edge_count >= 64
        && std::iter::once(&outer_edges).chain(hole_edges.iter()).flatten().all(|(a, b)| a.x.is_finite() && a.y.is_finite() && b.x.is_finite() && b.y.is_finite()))
        .then(|| InteriorSegmentIndex::new(std::iter::once(&outer_edges).chain(hole_edges.iter()).flatten().copied().collect()));
    let parity = parity_index_enabled();
    let outer_parity = if parity { PolygonParityIndex::new(outer_polygon) } else { None };
    let hole_parity: Vec<Option<PolygonParityIndex<'_>>> = hole_polygons.iter()
        .map(|hole| if parity { PolygonParityIndex::new(hole) } else { None }).collect();
    let inside_outer = |point: Vec2| outer_parity.as_ref().map_or_else(|| point_in_polygon(point, outer_polygon), |index| index.contains(point));
    let inside_hole = |hole: usize, point: Vec2| hole_parity[hole].as_ref().map_or_else(|| point_in_polygon(point, &hole_polygons[hole]), |index| index.contains(point));
    let distance = |candidate: Vec2| match &index {
        Some(index) => index.distance(candidate),
        None => {
            let mut clearance = f64::INFINITY;
            for cycle in std::iter::once(&outer_edges).chain(hole_edges.iter()) {
                for (a, b) in cycle { clearance = clearance.min(point_segment_distance(candidate, *a, *b)); }
            }
            clearance
        }
    };
    for factor in [0.25, 0.1, 0.05, 0.01, 0.002, 4e-4] {
        let epsilon = scale * factor;
        for &(a, b) in &outer_edges {
            let direction = b.sub(a);
            let length = direction.length();
            if length <= 1e-12 {
                continue;
            }
            let candidate = a.add(b).scale(0.5).add(
                Vec2 {
                    x: -direction.y / length,
                    y: direction.x / length,
                }
                .scale(epsilon),
            );
            if !inside_outer(candidate)
                || (0..hole_polygons.len()).any(|hole| inside_hole(hole, candidate))
            {
                continue;
            }
            let upper = if select { point_segment_distance(candidate, a, b) } else { f64::INFINITY };
            if select && upper.is_finite() && upper <= best_clearance {
                pending.push((candidate, None, upper));
                pruned += 1;
                continue;
            }
            let clearance = distance(candidate);
            queries += 1;
            if select {
                finite &= clearance.is_finite();
                pending.push((candidate, Some(clearance), upper));
            } else {
                seen.push((candidate, clearance));
            }
            if clearance > best_clearance {
                best = Some(candidate);
                best_clearance = clearance;
            }
        }
        if best.is_some() && best_clearance >= epsilon * 0.5 {
            break;
        }
    }
    let Some(primary) = best else {
        return Vec::new();
    };
    let mut picked = vec![primary];
    let separation = (scale * 0.05).max(1e-12);
    // Repeated maxima among separated candidates yield the stable sort's
    // same sequence. A strict improvement preserves insertion order at ties.
    // Read a deferred clearance only if its bound can beat the current winner.
    if select && finite {
        while picked.len() < count {
            let mut next = None;
            let mut clearance = f64::NEG_INFINITY;
            for (candidate, cached, upper) in &mut pending {
                if (upper.is_finite() && *upper <= clearance)
                    || !picked.iter().all(|existing| existing.sub(*candidate).length() >= separation)
                { continue; }
                let value = *cached.get_or_insert_with(|| { queries += 1; distance(*candidate) });
                if value > clearance {
                    next = Some(*candidate);
                    clearance = value;
                }
            }
            let Some(candidate) = next else { break };
            picked.push(candidate);
        }
        if crate::profile::profile_enabled() {
            crate::imprint::profile_emit(format_args!("fragment.interior_select candidates={} queries={queries} primary_pruned={pruned} wanted={count}", pending.len()));
        }
        return picked;
    }
    if select {
        // Preserve the original partial_cmp behavior on non-finite readings.
        seen = pending.into_iter().map(|(point, value, _)|
            (point, value.unwrap_or_else(|| distance(point)))).collect();
    }
    seen.sort_by(|left, right| {
        right
            .1
            .partial_cmp(&left.1)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for (candidate, _) in seen {
        if picked.len() >= count {
            break;
        }
        if picked
            .iter()
            .all(|existing| existing.sub(candidate).length() >= separation)
        {
            picked.push(candidate);
        }
    }
    picked
}

/// The (chain, segment) of an arrangement use's parent segment, from the
/// segment's tag; a segment without a position reads as `u64::MAX`.
fn chain_segment(usage: &crate::CycleUse) -> (u64, u64) {
    let tag = &usage.piece.parent.tag;
    (tag_chain(tag).unwrap_or(u64::MAX), tag_seg(tag).unwrap_or(u64::MAX))
}

/// PACKED SEGMENT TAGS (lane O row 9, 2026-10-28; on by default since its
/// promotion, `BREP_FRAGMENT_PACKED_TAGS=0` writes the object tags).
/// Every arrangement piece and every cycle use clones its
/// parent segment, tag included; the `{"chain", "seg"}` object allocates a map
/// and two keys on each clone, and the D1 bench case's 272-segment faces paid
/// for it ~1 100 times a face. The packed tag is ONE JSON number,
/// `chain · 2^32 + seg`, which clones without allocating. The readers below
/// accept both encodings, and the values they return are the same, so the
/// arrangement and every fragment are unchanged.
pub(super) fn segment_tag(chain: usize, segment: usize) -> serde_json::Value {
    // Compared as u64: on wasm32 a usize cannot hold 1 << 32 (the shift
    // overflowed there, failing the web build), and every usize fits.
    if packed_tags_enabled() && chain < (1 << 20) && (segment as u64) < (1u64 << 32) {
        serde_json::Value::from(((chain as u64) << 32) | segment as u64)
    } else {
        serde_json::json!({ "chain": chain, "seg": segment })
    }
}

/// The chain a segment tag names, in either encoding.
pub(super) fn tag_chain(tag: &serde_json::Value) -> Option<u64> {
    match tag {
        serde_json::Value::Number(number) => number.as_u64().map(|packed| packed >> 32),
        _ => tag.get("chain").and_then(serde_json::Value::as_u64),
    }
}

/// The segment index a segment tag names, in either encoding.
pub(super) fn tag_seg(tag: &serde_json::Value) -> Option<u64> {
    match tag {
        serde_json::Value::Number(number) => number.as_u64().map(|packed| packed & 0xffff_ffff),
        _ => tag.get("seg").and_then(serde_json::Value::as_u64),
    }
}

fn packed_tags_enabled() -> bool {
    switch!("BREP_FRAGMENT_PACKED_TAGS", true)
}


/// A region's topological key (lane X row 9): the sorted (chain, segment)
/// list of its outer loop, then the sorted lists of its holes. It reads no
/// coordinate, so congruent regions are ordered the same whatever the last
/// bits of their areas.
pub(super) fn region_topology_key(outer: &[crate::CycleUse], holes: &[Vec<crate::CycleUse>]) -> (Vec<(u64, u64)>, Vec<Vec<(u64, u64)>>) {
    let mut outer_key: Vec<(u64, u64)> = outer.iter().map(chain_segment).collect();
    outer_key.sort_unstable();
    let mut hole_keys: Vec<Vec<(u64, u64)>> = holes
        .iter()
        .map(|hole| {
            let mut key: Vec<(u64, u64)> = hole.iter().map(chain_segment).collect();
            key.sort_unstable();
            key
        })
        .collect();
    hole_keys.sort_unstable();
    (outer_key, hole_keys)
}

fn chain_index(usage: &crate::CycleUse) -> Result<usize, KernelRefusal> {
    tag_chain(&usage.piece.parent.tag)
        .map(|value| value as usize)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", "fragment_face: arrangement lost chain tag"))
}

// Returns `Ok(None)` when the cycle is a spurious zero-area "antenna" region
// (a self-touching fold that encloses no material — see the seam-grazing note at
// the incomplete-run site); the caller drops that region instead of aborting.
/// A closed boundary curve touched at one point by a cut (a footprint's edge
/// tangent to a bore's rim: printmotor-assembly B517) gives the arrangement a
/// node there, so the region's outer cycle runs in along the cut, around the
/// whole closed curve and out again. The face still has that curve as a hole
/// of its own, and nothing else meets it, so the run is moved to `holes`,
/// starting at the chain's own start: the closed edge stays one edge on its
/// own vertex and the loop count is the surface's. Returns the runs moved.
/// `BREP_FRAGMENT_DETACH_TOUCHED_RING=0` keeps the single cycle.
pub(super) fn detach_touched_closed_boundaries(
    region: &mut crate::ArrangementRegion,
    chains: &[Chain],
    tolerance: f64,
) -> Result<usize, KernelRefusal> {
    if !switch!("BREP_FRAGMENT_DETACH_TOUCHED_RING", true) || region.outer.len() < 2 {
        return Ok(0);
    }
    let chain_of = |usage: &crate::CycleUse| chain_index(usage);
    let begin = |usage: &crate::CycleUse| if usage.forward { usage.piece.a } else { usage.piece.b };
    let end = |usage: &crate::CycleUse| if usage.forward { usage.piece.b } else { usage.piece.a };
    let count = region.outer.len();
    let mut start = None;
    for index in 0..count {
        if chain_of(&region.outer[(index + count - 1) % count])? != chain_of(&region.outer[index])? {
            start = Some(index);
            break;
        }
    }
    // One chain all the way round is an ordinary closed loop.
    let Some(start) = start else { return Ok(0) };
    let mut rotated = region.outer.clone();
    rotated.rotate_left(start);
    // A hole winds against the region's whole cycle. The cycle's own outer rim
    // winds with it and must stay (a bore a micron inside a cap's rim: moving
    // the rim would leave the bore as the outer loop).
    let signed_area = |uses: &[crate::CycleUse]| -> f64 {
        uses.iter().map(|usage| {
            let (from, to) = (begin(usage), end(usage));
            from.x * to.y - to.x * from.y
        }).sum::<f64>() / 2.0
    };
    let whole_area = signed_area(&rotated);
    let mut kept = Vec::with_capacity(count);
    let mut detached = Vec::new();
    let mut index = 0;
    while index < rotated.len() {
        let chain_id = chain_of(&rotated[index])?;
        let mut run_end = index + 1;
        while run_end < rotated.len() && chain_of(&rotated[run_end])? == chain_id {
            run_end += 1;
        }
        let run = &rotated[index..run_end];
        let closed_ring = chains.get(chain_id).is_some_and(|chain| {
            matches!(chain.source, ChainSource::Boundary { .. })
                && chain.start.sub(chain.end).length() <= tolerance
                && begin(&run[0]).sub(end(&run[run.len() - 1])).length() <= tolerance
                && {
                    let along: f64 = run.iter().map(|usage| usage.piece.b.sub(usage.piece.a).length()).sum();
                    let whole: f64 = chain.segments.iter().map(|(a, b)| b.sub(*a).length()).sum();
                    (along - whole).abs() <= tolerance * 4.0
                }
        });
        if closed_ring && run.len() < rotated.len() && signed_area(run) * whole_area < 0.0 {
            let chain = &chains[chain_id];
            let first = run.iter().position(|usage| begin(usage).sub(chain.start).length() <= tolerance);
            match first {
                Some(first) => {
                    let mut ring = run.to_vec();
                    ring.rotate_left(first);
                    detached.push(ring);
                }
                None => kept.extend_from_slice(run),
            }
        } else {
            kept.extend_from_slice(run);
        }
        index = run_end;
    }
    let moved = detached.len();
    if moved > 0 {
        region.outer = kept;
        region.holes.extend(detached);
    }
    Ok(moved)
}

pub(super) fn build_loop(
    cycle: &[crate::CycleUse],
    reverse: bool,
    chains: &[Chain],
    tolerance: f64,
    surface: &NurbsSurface,
) -> Result<Option<FragmentLoop>, KernelRefusal> {
    if loop_view_enabled() {
        let views = cycle
            .iter()
            .map(|usage| Ok(UseView { a: usage.piece.a, b: usage.piece.b, forward: usage.forward, chain: chain_index(usage)? }))
            .collect::<Result<Vec<_>, KernelRefusal>>()?;
        return build_loop_uses(views, reverse, chains, tolerance, surface);
    }
    build_loop_uses(cycle.to_vec(), reverse, chains, tolerance, surface)
}

/// What [`build_loop_uses`] reads of one arrangement use: its ends, its
/// direction and its chain. The cloned [`crate::CycleUse`] carries the piece's
/// whole parent segment and its `serde_json` tag, re-read for the chain on
/// every comparison; [`UseView`] copies the ends and parses the chain once.
trait LoopUse {
    fn forward(&self) -> bool;
    fn set_forward(&mut self, forward: bool);
    fn a(&self) -> Vec2;
    fn b(&self) -> Vec2;
    fn chain(&self) -> Result<usize, KernelRefusal>;
}

impl LoopUse for crate::CycleUse {
    fn forward(&self) -> bool { self.forward }
    fn set_forward(&mut self, forward: bool) { self.forward = forward; }
    fn a(&self) -> Vec2 { self.piece.a }
    fn b(&self) -> Vec2 { self.piece.b }
    fn chain(&self) -> Result<usize, KernelRefusal> { chain_index(self) }
}

#[derive(Clone, Copy)]
struct UseView {
    a: Vec2,
    b: Vec2,
    forward: bool,
    chain: usize,
}

impl LoopUse for UseView {
    fn forward(&self) -> bool { self.forward }
    fn set_forward(&mut self, forward: bool) { self.forward = forward; }
    fn a(&self) -> Vec2 { self.a }
    fn b(&self) -> Vec2 { self.b }
    fn chain(&self) -> Result<usize, KernelRefusal> { Ok(self.chain) }
}

/// LOOP VIEW (lane O row 8, 2026-10-28; on by default since its promotion,
/// `BREP_FRAGMENT_LOOP_VIEW=0` restores the cloned cycle):
/// [`build_loop`] reads its cycle through [`UseView`] instead of
/// cloning it. The loop it builds reads the same ends, directions and chains,
/// so every coedge, pcurve and source is the clone path's to the byte.
/// Measured on the filleted box's outward offset (D1), whose two arc regions
/// hold ~270 uses each: the clone and the per-comparison tag parse were
/// 0.43 of build_loop's 0.49 ms per attempt (cad-bot fast build, plain).
fn loop_view_enabled() -> bool {
    switch!("BREP_FRAGMENT_LOOP_VIEW", true)
}

/// `BREP_FRAGMENT_CLOSED_PARTIAL` (default on; `=0` restores the refusal):
/// see CLOSED PARTIAL in the loop builder.
fn closed_partial_enabled() -> bool {
    switch!("BREP_FRAGMENT_CLOSED_PARTIAL", true)
}


fn build_loop_uses<U: LoopUse>(
    mut uses: Vec<U>,
    reverse: bool,
    chains: &[Chain],
    tolerance: f64,
    surface: &NurbsSurface,
) -> Result<Option<FragmentLoop>, KernelRefusal> {
    if reverse {
        uses.reverse();
        for usage in &mut uses {
            usage.set_forward(!usage.forward());
        }
    }
    let mut start = 0;
    for index in 0..uses.len() {
        if uses[(index + uses.len() - 1) % uses.len()].chain()? != uses[index].chain()?
        {
            start = index;
            break;
        }
    }
    uses.rotate_left(start);
    let mut coedges = Vec::new();
    let mut index = 0;
    while index < uses.len() {
        let chain_id = uses[index].chain()?;
        let mut run_end_index = index + 1;
        while run_end_index < uses.len() && uses[run_end_index].chain()? == chain_id {
            run_end_index += 1;
        }
        let chain = chains
            .get(chain_id)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", "fragment_face: unknown arrangement chain"))?;
        let run = &uses[index..run_end_index];
        let run_start = if run[0].forward() {
            run[0].a()
        } else {
            run[0].b()
        };
        let last = &run[run.len() - 1];
        let run_end = if last.forward() {
            last.b()
        } else {
            last.a()
        };
        let epsilon = tolerance * 20.0;
        let chain_closed = chain.start.sub(chain.end).length() <= epsilon;
        let complete = if chain_closed {
            run_start.sub(run_end).length() <= epsilon
        } else {
            (run_start.sub(chain.start).length() <= epsilon
                && run_end.sub(chain.end).length() <= epsilon)
                || (run_start.sub(chain.end).length() <= epsilon
                    && run_end.sub(chain.start).length() <= epsilon)
        };
        if !complete || run.len() < 1usize.max(chain.count / 2) {
            if !chain_closed {
                let (carrier_pcurve, carrier_curve, lift) = match &chain.source {
                    ChainSource::Boundary { coedge, curve } => (&coedge.pcurve, curve, Vec2 { x: 0.0, y: 0.0 }),
                    ChainSource::Cut { pcurve, curve, lift, .. } => (pcurve, curve, *lift),
                };
                // Back onto the pcurve's own period: the arrangement may have
                // lifted the chain a whole period into the face's band.
                let run_start = run_start.sub(lift);
                let run_end = run_end.sub(lift);
                let start_projection = project_point_to_curve(
                    carrier_pcurve,
                    Vec3::new(run_start.x, run_start.y, 0.0),
                ).or_refuse(KernelStage::Fragment, "csg.fragment.loops")?;
                let end_projection =
                    project_point_to_curve(carrier_pcurve, Vec3::new(run_end.x, run_end.y, 0.0)).or_refuse(KernelStage::Fragment, "project_point_to_curve")?;
                let [p0, p1] = carrier_pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                let parameter_span = p1 - p0;
                let start_fraction = (start_projection.u - p0) / parameter_span;
                let end_fraction = (end_projection.u - p0) / parameter_span;
                let chord_tolerance = |point: Vec2| {
                    chain
                        .segments
                        .iter()
                        .min_by(|(a0, a1), (b0, b1)| {
                            point_segment_distance(point, *a0, *a1)
                                .total_cmp(&point_segment_distance(point, *b0, *b1))
                        })
                        .map(|(a, b)| b.sub(*a).length())
                        .unwrap_or(0.0)
                };
                let start_tolerance = (epsilon * 20.0).max(chord_tolerance(run_start) * 1.01);
                let end_tolerance = (epsilon * 20.0).max(chord_tolerance(run_end) * 1.01);
                if env_read!("BREP_DEBUG_FRAG").is_some() {
                    eprintln!(
                        "partial run: chain {chain_id} count={} run={} complete={complete} span={parameter_span:.6e} \
                         start(d={:.6e} tol={:.6e} f={:.9}) end(d={:.6e} tol={:.6e} f={:.9})",
                        chain.count,
                        run.len(),
                        start_projection.distance,
                        start_tolerance,
                        start_fraction,
                        end_projection.distance,
                        end_tolerance,
                        end_fraction,
                    );
                }
                if parameter_span > 1e-12
                    && start_projection.distance <= start_tolerance
                    && end_projection.distance <= end_tolerance
                    && (end_fraction - start_fraction).abs() > 1e-12
                {
                    let low = start_fraction.min(end_fraction).clamp(0.0, 1.0);
                    let high = start_fraction.max(end_fraction).clamp(0.0, 1.0);
                    let mut edge_curve = subcurve_by_fraction(carrier_curve, low, high)?;
                    let mut pcurve = subcurve_by_fraction(carrier_pcurve, low, high)?;
                    let forward = end_fraction > start_fraction;
                    let [partial_p0, partial_p1] = pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                    let endpoint_uv_start = pcurve.evaluate(partial_p0).or_refuse(KernelStage::Fragment, "evaluate")?;
                    let endpoint_uv_end = pcurve.evaluate(partial_p1).or_refuse(KernelStage::Fragment, "evaluate")?;
                    let desired_low = if forward { run_start } else { run_end };
                    let desired_high = if forward { run_end } else { run_start };
                    let endpoint_gap = surface
                        .evaluate(endpoint_uv_start.x, endpoint_uv_start.y).or_refuse(KernelStage::Fragment, "evaluate")?
                        .sub(surface.evaluate(desired_low.x, desired_low.y).or_refuse(KernelStage::Fragment, "evaluate")?)
                        .length()
                        .max(
                            surface
                                .evaluate(endpoint_uv_end.x, endpoint_uv_end.y).or_refuse(KernelStage::Fragment, "evaluate")?
                                .sub(surface.evaluate(desired_high.x, desired_high.y).or_refuse(KernelStage::Fragment, "evaluate")?)
                                .length(),
                        );
                    if endpoint_gap > 2e-3 {
                        let parameters = (0..=32)
                            .map(|sample| sample as f64 / 32.0)
                            .collect::<Vec<_>>();
                        let mut uv_points = parameters
                            .iter()
                            .map(|fraction| {
                                pcurve.evaluate(partial_p0 + (partial_p1 - partial_p0) * fraction)
                            })
                            .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Fragment, "csg.fragment.loops")?;
                        uv_points[0] = Vec3::new(desired_low.x, desired_low.y, 0.0);
                        *uv_points.last_mut().unwrap() =
                            Vec3::new(desired_high.x, desired_high.y, 0.0);
                        pcurve = interpolate_curve(&uv_points, 1, &parameters).or_refuse(KernelStage::Fragment, "interpolate_curve")?;
                        let edge_points = uv_points
                            .iter()
                            .map(|uv| surface.evaluate(uv.x, uv.y))
                            .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Fragment, "csg.fragment.loops")?;
                        edge_curve = interpolate_curve(&edge_points, 1, &parameters).or_refuse(KernelStage::Fragment, "interpolate_curve")?;
                    }
                    let [t0, t1] = edge_curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                    let start = edge_curve.evaluate(t0).or_refuse(KernelStage::Fragment, "evaluate")?;
                    let end = edge_curve.evaluate(t1).or_refuse(KernelStage::Fragment, "evaluate")?;
                    coedges.push(FragmentCoedge {
                        source: FragmentEdgeSource::Derived {
                            curve: edge_curve,
                            t0,
                            t1,
                            start,
                            end,
                        },
                        forward,
                        pcurve: if forward { pcurve } else { pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")? },
                    });
                    index = run_end_index;
                    continue;
                }
            }
            // CLOSED PARTIAL (`BREP_FRAGMENT_CLOSED_PARTIAL`, default on): a
            // CLOSED chain another chain of the face crosses is used in arcs,
            // one per region it bounds: a one-call imprint mints sections of
            // several members on one carrier, and they cross one another (the
            // shelled block's bore skin, the circle of radius 4 on the floor's
            // offset plane, crosses the wall skin's cut x = 19; the lens beyond
            // the cut used the arc backward and the face refused). The arc is
            // the carrier between the run's ends, in the run's direction; a run
            // over the chain's own start is cut there into two coedges, and the
            // regions on its two sides both cut it there.
            if chain_closed && !complete && closed_partial_enabled() {
                let (carrier_pcurve, carrier_curve, lift) = match &chain.source {
                    ChainSource::Boundary { coedge, curve } => (&coedge.pcurve, curve, Vec2 { x: 0.0, y: 0.0 }),
                    ChainSource::Cut { pcurve, curve, lift, .. } => (pcurve, curve, *lift),
                };
                let (from, to) = (run_start.sub(lift), run_end.sub(lift));
                let first = project_point_to_curve(carrier_pcurve, Vec3::new(from.x, from.y, 0.0)).or_refuse(KernelStage::Fragment, "project_point_to_curve")?;
                let last = project_point_to_curve(carrier_pcurve, Vec3::new(to.x, to.y, 0.0)).or_refuse(KernelStage::Fragment, "project_point_to_curve")?;
                let [p0, p1] = carrier_pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                let span = p1 - p0;
                let band = epsilon * 20.0;
                if span > 1e-12 && first.distance <= band && last.distance <= band {
                    let (f_start, f_end) = ((first.u - p0) / span, (last.u - p0) / span);
                    let forward = run[0].forward();
                    // The arcs in the run's direction, each as (low, high).
                    let arcs: Vec<(f64, f64)> = match (forward, f_end > f_start) {
                        (true, true) => vec![(f_start, f_end)],
                        (true, false) => vec![(f_start, 1.0), (0.0, f_end)],
                        (false, false) => vec![(f_end, f_start)],
                        (false, true) => vec![(0.0, f_start), (f_end, 1.0)],
                    };
                    let mut built = Vec::new();
                    // The run's own ends, where the arrangement crossed the
                    // other chains: every neighbour (the crossing chain's
                    // partial run, the other face's edge) ends there, so the
                    // arc does too. A fraction is the run end projected onto
                    // the carrier, a chord's sag (~1e-5) off it.
                    let (low_end, high_end) = if forward { (from, to) } else { (to, from) };
                    let arc_count = arcs.len();
                    for (arc_index, (low, high)) in arcs.into_iter().enumerate() {
                        if high - low <= 1e-12 {
                            continue;
                        }
                        let mut edge_curve = subcurve_by_fraction(carrier_curve, low, high)?;
                        let mut pcurve = subcurve_by_fraction(carrier_pcurve, low, high)?;
                        // Which of this arc's ends are run ends (not the seam).
                        let snap_low = if forward { arc_index == 0 } else { arc_index + 1 == arc_count };
                        let snap_high = if forward { arc_index + 1 == arc_count } else { arc_index == 0 };
                        for (snap, uv, at_start) in [(snap_low, low_end, true), (snap_high, high_end, false)] {
                            if !snap {
                                continue;
                            }
                            let point = surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Fragment, "evaluate")?;
                            for (curve, target) in [(&mut edge_curve, point), (&mut pcurve, Vec3::new(uv.x, uv.y, 0.0))] {
                                let index = if at_start { 0 } else { curve.control_points.len() - 1 };
                                let weight = curve.control_points[index].w;
                                curve.control_points[index] = crate::Vec4::from_point(target, weight);
                            }
                        }
                        let [t0, t1] = edge_curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                        let start = edge_curve.evaluate(t0).or_refuse(KernelStage::Fragment, "evaluate")?;
                        let end = edge_curve.evaluate(t1).or_refuse(KernelStage::Fragment, "evaluate")?;
                        built.push(FragmentCoedge {
                            source: FragmentEdgeSource::Derived { curve: edge_curve, t0, t1, start, end },
                            forward,
                            pcurve: if forward { pcurve } else { pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")? },
                        });
                    }
                    if !forward {
                        built.reverse();
                    }
                    if !built.is_empty() {
                        if env_read!("BREP_DEBUG_FRAG").is_some() {
                            eprintln!("closed partial: chain {chain_id} arc f={f_start:.9}..{f_end:.9} forward={forward} as {} coedge(s)", built.len());
                        }
                        coedges.extend(built);
                        index = run_end_index;
                        continue;
                    }
                }
            }
            // A run that departs from and returns to the SAME point along an
            // OPEN chain is a spurious zero-area "antenna" — a self-touching
            // fold, not a real boundary. This is exactly how a periodic-band
            // face's seam-grazing SSI cut manifests: near the u-seam the cut's
            // in-domain-clamped pcurve folds back on itself, and `arrange_segments`
            // carves off a degenerate micro-loop region bounded solely by that
            // fold (observed param area ~1e-9). That region encloses no material
            // and has no counterpart on the mating (non-periodic) face, so signal
            // the caller to DROP the whole region rather than aborting the entire
            // face's fragmentation. The real, full-area region that also borders
            // this chain still traverses it as one complete run and is unaffected.
            if !chain_closed && run_start.sub(run_end).length() <= epsilon {
                return Ok(None);
            }
            if env_read!("BREP_DEBUG_FRAG").is_some() {
                eprintln!(
                    "incomplete run: chain {chain_id} count={} closed={} start=({:.9},{:.9}) end=({:.9},{:.9}); run len={} start=({:.9},{:.9}) end=({:.9},{:.9}) epsilon={:.3e}",
                    chain.count, chain_closed, chain.start.x, chain.start.y, chain.end.x, chain.end.y,
                    run.len(), run_start.x, run_start.y, run_end.x, run_end.y, epsilon
                );
                for usage in run {
                    eprintln!("   use fwd={} a=({:.9},{:.9}) b=({:.9},{:.9})", usage.forward(), usage.a().x, usage.a().y, usage.b().x, usage.b().y);
                }
            }
            return Err(KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", format!(
                "fragment_face: chain {chain_id} fragmented into an incomplete run"
            )));
        }
        let chain_forward = run[0].forward();
        match &chain.source {
            ChainSource::Boundary { coedge: source, .. } => coedges.push(FragmentCoedge {
                source: FragmentEdgeSource::Boundary {
                    operand: 0, // replaced by caller after construction
                    edge_id: source.edge_id,
                },
                forward: if chain_forward {
                    source.forward
                } else {
                    !source.forward
                },
                pcurve: if chain_forward {
                    source.pcurve.clone()
                } else {
                    source.pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")?
                },
            }),
            ChainSource::Cut {
                piece_id,
                pcurve,
                shared_ring,
                ..
            } => {
                // CLOSED-RING SHARED SECTION (equator-tangent): a closed
                // piece marked shared_edge coincides with an existing
                // boundary RING, which the assembler's endpoint weld can
                // never unify (no distinct endpoints). Reference the owning
                // boundary edge DIRECTLY so both operands' fragments share
                // one edge record by construction. Open shared arcs keep the
                // landed endpoint-weld path unchanged.
                match shared_ring {
                    Some((operand, edge_id, aligned)) => {
                        if std::env::var("BREP_DEBUG_SHARED_SECTION").is_ok() {
                            eprintln!("fragment: SharedBoundary emit {}:{} aligned={} fwd={}", operand, edge_id, aligned, chain_forward);
                        }
                        coedges.push(FragmentCoedge {
                        source: FragmentEdgeSource::SharedBoundary { operand: *operand, edge_id: *edge_id },
                        forward: chain_forward == *aligned,
                        pcurve: if chain_forward {
                            pcurve.clone()
                        } else {
                            pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")?
                        },
                    })
                    }
                    None => coedges.push(FragmentCoedge {
                        source: FragmentEdgeSource::Imprint {
                            piece_id: *piece_id,
                        },
                        forward: chain_forward,
                        pcurve: if chain_forward {
                            pcurve.clone()
                        } else {
                            pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")?
                        },
                    }),
                }
            }
        }
        index = run_end_index;
    }
    Ok(Some(FragmentLoop { coedges }))
}

