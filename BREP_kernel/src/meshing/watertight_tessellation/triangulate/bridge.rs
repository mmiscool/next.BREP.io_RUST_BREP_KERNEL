use super::*;
use super::earclip::{crossing_parity_inside, point_in_triangle};

/// Bridge every hole into the outer loop (classic two-way bridge, vertices
/// duplicated), producing one simple polygon for ear clipping.
pub(in crate::watertight_tessellation) fn bridge_holes(outer: Vec<FaceVertex>, holes: Vec<Vec<FaceVertex>>) -> Vec<FaceVertex> {
    let mut polygon = outer;
    let mut remaining: Vec<Vec<FaceVertex>> = holes;
    // Bridge holes right-to-left so previously inserted bridges stay valid.
    remaining.sort_by(|a, b| {
        let max_u = |chain: &Vec<FaceVertex>| {
            chain
                .iter()
                .map(|vertex| vertex.uv[0])
                .fold(f64::NEG_INFINITY, f64::max)
        };
        let max_v = |chain: &Vec<FaceVertex>| {
            chain
                .iter()
                .map(|vertex| vertex.uv[1])
                .fold(f64::NEG_INFINITY, f64::max)
        };
        max_u(b)
            .total_cmp(&max_u(a))
            .then_with(|| max_v(b).total_cmp(&max_v(a)))
    });
    while !remaining.is_empty() {
        // A bridge is validated against the holes already merged into the
        // polygon, but it cannot see holes still in `remaining`. With a dense
        // grid of openings, blindly taking the first equal-u hole can therefore
        // run a corridor through a later one. Consider every currently valid
        // candidate and take the shortest bridge whose merged polygon remains
        // simple, growing the trim locally instead of laying long crossings.
        let mut selected: Option<(f64, usize, usize, usize, Vec<FaceVertex>)> = None;
        for (hole_index, hole) in remaining.iter().enumerate() {
            let hole_anchor = (0..hole.len())
                .max_by(|&a, &b| hole[a].uv[0].total_cmp(&hole[b].uv[0]))
                .unwrap_or(0);
            if let Some((target, merged)) = simple_hole_bridge(&polygon, hole, hole_anchor) {
                let a = hole[hole_anchor].uv;
                let b = polygon[target].uv;
                let distance = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
                // On very high-genus faces, evaluating every remaining hole at
                // every merge makes the global-simplicity proof quartic. The
                // deterministic right-to-left order is safe as soon as its
                // bridge also avoids every unconsumed hole chain; take that
                // first certified candidate. Smaller faces keep the historical
                // global shortest-bridge choice byte-for-byte.
                if remaining.len() >= 64
                    && bridge_avoids_other_holes(
                        a,
                        b,
                        &remaining,
                        hole_index,
                        bridge_epsilon(&polygon),
                    )
                {
                    selected = Some((distance, hole_index, hole_anchor, target, merged));
                    break;
                }
                if selected.as_ref().is_none_or(|best| distance < best.0) {
                    selected = Some((distance, hole_index, hole_anchor, target, merged));
                }
            }
        }
        // Degenerate vendor trims may offer no globally simple candidate. Keep
        // the old deterministic fallback in that case; the guarded ear clip is
        // still preferable to dropping the face altogether.
        let (_, hole_index, hole_anchor, target, merged) = selected.unwrap_or_else(|| {
            let hole = &remaining[0];
            let hole_anchor = (0..hole.len())
                .max_by(|&a, &b| hole[a].uv[0].total_cmp(&hole[b].uv[0]))
                .unwrap_or(0);
            let target = bridge_target(&polygon, hole, hole_anchor);
            let merged = merge_hole_bridge(&polygon, hole, hole_anchor, target);
            (f64::INFINITY, 0, hole_anchor, target, merged)
        });
        let hole = remaining.remove(hole_index);
        let anchor = hole[hole_anchor];
        if std::env::var("BREP_DEBUG_TESS_BRIDGE").is_ok() {
            eprintln!(
                "bridge: anchor=({:.3},{:.3}) -> polygon[{target}]=({:.3},{:.3}) len={}",
                anchor.uv[0],
                anchor.uv[1],
                polygon[target].uv[0],
                polygon[target].uv[1],
                polygon.len(),
            );
        }
        polygon = merged;
        if std::env::var("BREP_DEBUG_TESS_BRIDGE").is_ok() {
            eprintln!("   merged simple={}", polygon_is_simple(&polygon));
        }
    }
    polygon
}

fn bridge_epsilon(polygon: &[FaceVertex]) -> f64 {
    let total: f64 = (0..polygon.len())
        .map(|index| {
            let a = polygon[index].uv;
            let b = polygon[(index + 1) % polygon.len()].uv;
            ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt()
        })
        .sum();
    (total / polygon.len().max(1) as f64).powi(2) * 1e-12
}

/// A bridge certified against the current polygon must also avoid holes that
/// have not been merged yet. This is the missing precondition that made the
/// old sorted fast path unsafe on aligned grids.
fn bridge_avoids_other_holes(
    anchor: [f64; 2],
    target: [f64; 2],
    holes: &[Vec<FaceVertex>],
    selected_hole: usize,
    epsilon: f64,
) -> bool {
    for (hole_index, hole) in holes.iter().enumerate() {
        if hole_index == selected_hole {
            continue;
        }
        for edge in 0..hole.len() {
            let a = hole[edge].uv;
            let b = hole[(edge + 1) % hole.len()].uv;
            if segments_properly_cross(anchor, target, a, b, epsilon)
                || point_strictly_on_segment(a, anchor, target, epsilon)
            {
                return false;
            }
        }
    }
    true
}

fn merge_hole_bridge(
    polygon: &[FaceVertex],
    hole: &[FaceVertex],
    hole_anchor: usize,
    target: usize,
) -> Vec<FaceVertex> {
    let mut merged = Vec::with_capacity(polygon.len() + hole.len() + 2);
    merged.extend_from_slice(&polygon[..=target]);
    for offset in 0..=hole.len() {
        merged.push(hole[(hole_anchor + offset) % hole.len()]);
    }
    merged.extend_from_slice(&polygon[target..]);
    merged
}

/// Find a clear bridge that also preserves the ear clip's global simplicity
/// precondition. The usual nearest/Eberly target is overwhelmingly the right
/// answer, so alternatives are searched only when that target crosses an
/// already merged corridor.
fn simple_hole_bridge(
    polygon: &[FaceVertex],
    hole: &[FaceVertex],
    hole_anchor: usize,
) -> Option<(usize, Vec<FaceVertex>)> {
    let preferred = bridge_target(polygon, hole, hole_anchor);
    let merged = merge_hole_bridge(polygon, hole, hole_anchor, preferred);
    if polygon_is_simple(&merged) {
        return Some((preferred, merged));
    }

    let anchor = hole[hole_anchor].uv;
    let epsilon = bridge_epsilon(polygon);
    let mut candidates: Vec<(f64, usize)> = (0..polygon.len())
        .filter_map(|target| {
            let point = polygon[target].uv;
            let distance = (point[0] - anchor[0]).powi(2) + (point[1] - anchor[1]).powi(2);
            (target != preferred && distance > epsilon).then_some((distance, target))
        })
        .collect();
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
    for (_, target) in candidates {
        if !bridge_is_clear(polygon, hole, hole_anchor, target, epsilon) {
            continue;
        }
        let merged = merge_hole_bridge(polygon, hole, hole_anchor, target);
        if polygon_is_simple(&merged) {
            return Some((target, merged));
        }
    }
    None
}

/// VISIBLE bridge target for a hole anchor (Eberly's ear-clipping-with-holes
/// construction): cast a +u ray from the anchor, take the closest polygon
/// edge it pierces, then refine to any reflex vertex hiding inside the
/// (anchor, hit, candidate) triangle — the one with the smallest angle off
/// the ray wins. A nearest-vertex heuristic without the visibility test
/// (the previous code) let a bridge slice straight through an adjacent
/// hole's chain, and the self-intersecting merged polygon ear-clipped OVER
/// the holes (problemInbox 2026-08-05T01-07: a spider face's four quadrant
/// openings rendered filled solid).
///
/// Eberly's construction only guarantees that the returned vertex is VISIBLE,
/// not that it is close: when the pierced edge is a long diagonal, its max-u
/// endpoint can be the far corner of the whole face. On a plate whose holes
/// all sit along such a diagonal, every hole then bridged to that one corner —
/// four 300-unit corridors converging on a single vertex, which pinches the
/// merged polygon into slivers the ear clip cannot resolve; it stalls and fans,
/// spraying overlapping triangles over the face (problemInbox
/// 2026-08-06T19-00: an excavator arm's side plates meshed 1.9x their own
/// area). So the Eberly vertex is used as a guaranteed-visible FALLBACK, and
/// the target actually taken is the NEAREST vertex whose bridge is verified
/// clear — short bridges leave the polygon locally simple and ear-clippable.
pub(in crate::watertight_tessellation) fn bridge_target(polygon: &[FaceVertex], hole: &[FaceVertex], hole_anchor: usize) -> usize {
    let anchor = hole[hole_anchor].uv;
    nearest_clear_bridge_target(polygon, hole, hole_anchor)
        .unwrap_or_else(|| eberly_bridge_target(polygon, anchor))
}

/// Eberly's visible-vertex construction (see `bridge_target`).
pub(in crate::watertight_tessellation) fn eberly_bridge_target(polygon: &[FaceVertex], anchor: [f64; 2]) -> usize {
    let count = polygon.len();
    let mut best_hit: Option<(f64, usize)> = None; // (hit u, edge start index)
    for index in 0..count {
        let a = polygon[index].uv;
        let b = polygon[(index + 1) % count].uv;
        // Half-open span rule: robust when the ray passes exactly through
        // shared vertices (the collinear crossbar rows of real trims).
        let (low, high) = if a[1] <= b[1] { (a, b) } else { (b, a) };
        if !(low[1] <= anchor[1] && anchor[1] < high[1]) {
            continue;
        }
        let t = (anchor[1] - a[1]) / (b[1] - a[1]);
        let u_hit = a[0] + (b[0] - a[0]) * t;
        if u_hit < anchor[0] {
            continue;
        }
        if best_hit.is_none_or(|(best_u, _)| u_hit < best_u) {
            best_hit = Some((u_hit, index));
        }
    }
    let Some((u_hit, edge_index)) = best_hit else {
        // No pierced edge (degenerate trims): nearest right-ish vertex.
        return (0..count)
            .min_by(|&a, &b| {
                let cost = |vertex: &FaceVertex| {
                    let du = vertex.uv[0] - anchor[0];
                    let dv = vertex.uv[1] - anchor[1];
                    du * du + dv * dv + if du < 0.0 { 1e6 } else { 0.0 }
                };
                cost(&polygon[a]).total_cmp(&cost(&polygon[b]))
            })
            .unwrap_or(0);
    };
    let a = polygon[edge_index].uv;
    let b = polygon[(edge_index + 1) % count].uv;
    let candidate = if a[0] > b[0] {
        edge_index
    } else {
        (edge_index + 1) % count
    };
    let hit = [u_hit, anchor[1]];
    let candidate_uv = polygon[candidate].uv;
    let mut refined: Option<(f64, f64, usize)> = None; // (tan angle, dist², vertex)
    for index in 0..count {
        if index == candidate {
            continue;
        }
        let point = polygon[index].uv;
        let previous = polygon[(index + count - 1) % count].uv;
        let next = polygon[(index + 1) % count].uv;
        // Only REFLEX vertices (CCW polygon ⇒ negative turn) can occlude.
        let turn = (point[0] - previous[0]) * (next[1] - point[1])
            - (point[1] - previous[1]) * (next[0] - point[0]);
        if turn >= 0.0 {
            continue;
        }
        if point[0] <= anchor[0] {
            continue;
        }
        if !point_in_triangle(point, anchor, hit, candidate_uv) {
            continue;
        }
        let tangent = (point[1] - anchor[1]).abs() / (point[0] - anchor[0]);
        let distance = (point[0] - anchor[0]).powi(2) + (point[1] - anchor[1]).powi(2);
        if refined.is_none_or(|(best_tan, best_dist, _)| {
            tangent < best_tan || (tangent == best_tan && distance < best_dist)
        }) {
            refined = Some((tangent, distance, index));
        }
    }
    refined.map(|(_, _, index)| index).unwrap_or(candidate)
}

/// NEAREST polygon vertex the hole anchor can be bridged to without breaking
/// the merged polygon: the connecting segment must cross no polygon edge and no
/// edge of the hole's own chain (which is not part of the polygon yet), must
/// pass through no third vertex, and its midpoint must lie inside the polygon —
/// the same "validated diagonal" contract `find_split_diagonal` applies to a
/// stalled ear-clip ring. Returns None when nothing validates, leaving Eberly's
/// guaranteed-visible vertex in charge.
///
/// A vertex whose position ALREADY appears twice in the polygon is an existing
/// bridge's endpoint. Landing a second bridge there stacks three corridors on
/// one point, which is what pinches the ring-spider face's quadrant openings
/// (`inbox_20260805_ring_spider_face_tessellates_inside_its_trim`) — so those
/// are only considered when no fresh vertex validates.
fn nearest_clear_bridge_target(
    polygon: &[FaceVertex],
    hole: &[FaceVertex],
    hole_anchor: usize,
) -> Option<usize> {
    let anchor = hole[hole_anchor].uv;
    let count = polygon.len();
    if count < 3 {
        return None;
    }
    // Area epsilon on the polygon's own edge scale, matching ear_clip and
    // polygon_is_simple, so a proper crossing means a real one at this size.
    let mut total = 0.0f64;
    for index in 0..count {
        let a = polygon[index].uv;
        let b = polygon[(index + 1) % count].uv;
        total += ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
    }
    let epsilon = (total / count as f64).powi(2) * 1e-12;

    // Bridge endpoints are exact copies of the vertex they duplicate.
    let uv_key = |uv: [f64; 2]| {
        let bits = |value: f64| if value == 0.0 { 0 } else { value.to_bits() };
        (bits(uv[0]), bits(uv[1]))
    };
    let mut uv_counts: FxHashMap<(u64, u64), usize> = FxHashMap::default();
    for vertex in polygon {
        *uv_counts.entry(uv_key(vertex.uv)).or_default() += 1;
    }
    let carries_bridge: Vec<bool> = polygon
        .iter()
        .map(|vertex| uv_counts[&uv_key(vertex.uv)] > 1)
        .collect();

    // Candidates nearest-first, so the first one that validates is the answer
    // and the O(n) validation runs a handful of times rather than n.
    let mut candidates: Vec<(f64, usize)> = (0..count)
        .filter_map(|index| {
            let target = polygon[index].uv;
            let distance = (target[0] - anchor[0]).powi(2) + (target[1] - anchor[1]).powi(2);
            // The anchor's own position is no bridge at all.
            (distance > epsilon).then_some((distance, index))
        })
        .collect();
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0));

    // Pass 1 over fresh vertices only; pass 2 (existing bridge endpoints) runs
    // only if pass 1 found nothing.
    for allow_bridged in [false, true] {
        for &(_, index) in &candidates {
            if carries_bridge[index] != allow_bridged {
                continue;
            }
            if bridge_is_clear(polygon, hole, hole_anchor, index, epsilon) {
                return Some(index);
            }
        }
    }
    None
}

/// Whether the bridge from a hole anchor to `target` keeps the merged polygon
/// well formed (see `nearest_clear_bridge_target`).
pub(in crate::watertight_tessellation) fn bridge_is_clear(
    polygon: &[FaceVertex],
    hole: &[FaceVertex],
    hole_anchor: usize,
    target_index: usize,
    epsilon: f64,
) -> bool {
    let anchor = hole[hole_anchor].uv;
    let target = polygon[target_index].uv;
    let count = polygon.len();
    // No proper crossing with the polygon, and no third vertex sitting on the
    // bridge (a collinear pass-through is not a "proper" crossing but still
    // welds two chains into a degenerate corridor).
    for offset in 0..count {
        let e0 = offset;
        let e1 = (offset + 1) % count;
        if e0 != target_index && e1 != target_index {
            let p = polygon[e0].uv;
            let q = polygon[e1].uv;
            if segments_properly_cross(anchor, target, p, q, epsilon) {
                return false;
            }
        }
        if e0 != target_index && point_strictly_on_segment(polygon[e0].uv, anchor, target, epsilon)
        {
            return false;
        }
    }
    // The hole chain itself is not in the polygon yet; a bridge that cuts back
    // through it would make the merged polygon self-intersecting. The vertex
    // test is as load-bearing as the crossing test: a bridge that dives into the
    // hole's OWN interior has to come back out across this chain, and when it
    // exits exactly THROUGH a chain vertex the crossing is not "proper", so the
    // crossing test alone waves it through (problemInbox 2026-08-06T21-04: an
    // evenly sampled circular bolt hole puts a vertex exactly antipodal to its
    // own max-u anchor, so the corridor ran down the hole's diameter and the ear
    // clip filled half the hole).
    let hole_count = hole.len();
    for offset in 0..hole_count {
        let h0 = offset;
        let h1 = (offset + 1) % hole_count;
        if h0 != hole_anchor
            && h1 != hole_anchor
            && segments_properly_cross(anchor, target, hole[h0].uv, hole[h1].uv, epsilon)
        {
            return false;
        }
        if h0 != hole_anchor && point_strictly_on_segment(hole[h0].uv, anchor, target, epsilon) {
            return false;
        }
    }
    // Inside the region — not across a concavity, and not through another hole
    // that has already been bridged in.
    let midpoint = [(anchor[0] + target[0]) * 0.5, (anchor[1] + target[1]) * 0.5];
    point_in_polygon(polygon, midpoint)
}

/// True when `p` lies on segment `a`-`b` strictly between the endpoints.
fn point_strictly_on_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2], epsilon: f64) -> bool {
    if triangle_area(a, b, p).abs() > epsilon {
        return false;
    }
    let length2 = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
    if length2 <= epsilon {
        return false;
    }
    let t = ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / length2;
    let margin = (epsilon / length2).sqrt();
    t > margin && t < 1.0 - margin
}

/// Crossing-parity point-in-polygon test over a whole vertex chain.
pub(super) fn point_in_polygon(polygon: &[FaceVertex], p: [f64; 2]) -> bool {
    crossing_parity_inside(polygon.len(), |offset| polygon[offset].uv, p)
}

