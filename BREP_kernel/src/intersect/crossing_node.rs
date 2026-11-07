//! CROSSING-NODE SPLIT — cut a marched section at every passage through an
//! isolated tangent node (2026-11-06, `BREP_SSI_NODE_SPLIT`, default on
//! since the 2026-11-06 promotion, `=0` restores the march as it was).
//!
//! Where two carriers touch at one point with a SADDLE height difference
//! (the eigenvalues of `H = II_a − II_b` of opposite sign, see
//! `tangent_curve.rs`), the section is two smooth branches crossing there:
//! Viviani's figure eight on a ball and the cylinder tangent inside it, or
//! the two loops of a tee whose branch touches the main pipe's crown. The
//! marcher does not stop at the node. Its step direction `n_a × n_b`
//! vanishes only AT the node, and a predictor step carries the trace across
//! it and on along the same smooth branch, so the whole figure eight comes
//! back as ONE branch (brep-bench `tangent/viviani-common`: one branch of
//! 6368 stations, 76.40 long, the curve's whole length). The node is then an
//! interior station of that branch, twice. Each face needs the node as a
//! VERTEX where four half-branches meet, or the arrangement cannot carve
//! the regions that meet at the point. The run reached the node's vertex only where a
//! trim edge happened to pass through it (the tee: one passage split on the
//! main's seam, the other carried through), and the shell tore.
//!
//! This pass finds the nodes along each branch and splits the branch at
//! every passage:
//!
//! * **Witness.** A station where `|n̂_a × n̂_b|` has a local minimum below
//!   [`NODE_WITNESS_CROSS`]. Along a branch through a saddle node the
//!   crossing falls linearly to zero, so the station nearest the node is
//!   such a minimum. The gate is loose: the refinement and the saddle test
//!   below decide, and the node must lie within the witness's own chord.
//! * **Node.** The witness is refined onto `{S_a = S_b, n_a × n_b = 0}` and
//!   classified (`classify_tangent_contact_at`). Only a SADDLE that does not
//!   extend is a crossing node. An extended contact or a definite one is
//!   left alone, as before.
//! * **Passage.** A local minimum of the stations' distance to the node
//!   whose adjacent chord passes within the ambiguity radius
//!   `3 √(2 tol / κ)` of it (the band in which the carriers are within the
//!   tolerance of each other across the contact, so the corrector's stations
//!   there are noise; the same radius the imprint's node landing drops), or
//!   within a quarter of the chord where the march stepped coarsely past.
//! * **Cut.** The stations of the passage inside the ambiguity radius are
//!   dropped (with none inside it, the nearest station stays on its own side
//!   of the node), the exact node is inserted on both sides (its parameters
//!   taken from the refinement, shifted by a period where the carrier is
//!   closed so they continue the neighbouring station's), and the branch is
//!   split there. A closed branch becomes as many open branches as it has
//!   passages; an open one gains one piece per passage.
//!
//! Nothing else moves. A branch with no node, or a node with no passage
//! close enough, is returned unchanged.

use crate::surface_surface_intersection::SurfaceIntersectionCurve;
use crate::tangent_curve::classify_tangent_contact_at;
use crate::{NurbsSurface, Vec3};

/// `|n̂_a × n̂_b|` (about 14.5°) below which a local minimum along a branch is
/// refined as a possible node witness. Loose on purpose: along a branch
/// through a saddle node the crossing falls linearly with the distance, so a
/// march that stepped coarsely past the node reads it at its nearest station
/// only as low as that station's distance allows. The refinement and the
/// saddle test decide; a witness near no contact refines onto none.
const NODE_WITNESS_CROSS: f64 = 0.25;

/// Is the crossing-node split on (`BREP_SSI_NODE_SPLIT`, default on; `=0`
/// off)?
pub fn crossing_node_split_enabled() -> bool {
    crate::switch!("BREP_SSI_NODE_SPLIT", true)
}

/// One crossing node: its point, its parameters on both carriers, and the
/// ambiguity radius around it.
#[derive(Clone, Copy, Debug)]
struct Node {
    point: Vec3,
    params_a: [f64; 2],
    params_b: [f64; 2],
    radius: f64,
}

/// The periods of a carrier's closed directions (`0.0` where open).
fn periods(surface: &NurbsSurface) -> Result<[f64; 2], String> {
    let (closed_u, closed_v) = surface.closed_directions()?;
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    Ok([
        if closed_u { u1 - u0 } else { 0.0 },
        if closed_v { v1 - v0 } else { 0.0 },
    ])
}

/// `params` moved by whole periods to lie nearest `near`.
fn continue_params(params: [f64; 2], near: [f64; 2], periods: [f64; 2]) -> [f64; 2] {
    let mut out = params;
    for index in 0..2 {
        let period = periods[index];
        if period > 0.0 {
            out[index] += period * ((near[index] - params[index]) / period).round();
        }
    }
    out
}

fn crossing(first: &NurbsSurface, second: &NurbsSurface, a: [f64; 2], b: [f64; 2]) -> Option<f64> {
    let normal_a = first.normal(a[0], a[1]).ok()?;
    let normal_b = second.normal(b[0], b[1]).ok()?;
    Some(normal_a.cross(normal_b).length())
}

/// The crossing nodes along `curves`, deduplicated.
fn find_nodes(
    first: &NurbsSurface,
    second: &NurbsSurface,
    curves: &[SurfaceIntersectionCurve],
    tolerance: f64,
) -> Result<Vec<Node>, String> {
    let mut nodes: Vec<Node> = Vec::new();
    for curve in curves {
        let count = curve.points.len();
        if count < 3 || curve.tangent || curve.params_a.len() != count || curve.params_b.len() != count {
            continue;
        }
        let crossings: Vec<f64> = (0..count)
            .map(|index| {
                crossing(first, second, curve.params_a[index], curve.params_b[index]).unwrap_or(f64::INFINITY)
            })
            .collect();
        for index in 0..count {
            let value = crossings[index];
            if !(value < NODE_WITNESS_CROSS) {
                continue;
            }
            let before = if index > 0 { crossings[index - 1] } else if curve.closed { crossings[count - 2] } else { f64::INFINITY };
            let after = if index + 1 < count { crossings[index + 1] } else if curve.closed { crossings[1] } else { f64::INFINITY };
            if value > before || value > after {
                continue;
            }
            let witness = curve.points[index];
            if nodes.iter().any(|node| node.point.sub(witness).length() <= node.radius) {
                continue;
            }
            let Some(contact) = classify_tangent_contact_at(first, second, witness, tolerance)? else {
                continue;
            };
            if !contact.saddle || contact.is_extended() || contact.curvature_scale <= 0.0 {
                continue;
            }
            // The refinement must land near the witness: a contact found
            // elsewhere on the carriers is not this branch's node.
            let before = if index > 0 { index - 1 } else if curve.closed { count - 2 } else { index };
            let after = if index + 1 < count { index + 1 } else if curve.closed { 1 } else { index };
            let chord = curve.points[before]
                .sub(witness)
                .length()
                .max(curve.points[after].sub(witness).length());
            let reach = (1e-2 * (1.0 + witness.length())).max(chord);
            if contact.point.sub(witness).length() > reach {
                continue;
            }
            if nodes.iter().any(|node| node.point.sub(contact.point).length() <= tolerance * 10.0) {
                continue;
            }
            nodes.push(Node {
                point: contact.point,
                params_a: contact.params_a,
                params_b: contact.params_b,
                radius: 3.0 * (2.0 * tolerance / contact.curvature_scale).sqrt(),
            });
        }
    }
    Ok(nodes)
}

/// One cut of a branch at a passage through a node: the last station kept
/// before it and the first kept after it (indices into the branch, or into
/// the cycle for a closed one), and the node inserted between them.
#[derive(Clone, Copy, Debug)]
struct Cut {
    left: usize,
    right: usize,
    node: usize,
}

fn point_segment_distance(point: Vec3, start: Vec3, end: Vec3) -> f64 {
    point.distance_to_segment(start, end, crate::tolerance::FP_FLOOR)
}

/// The passages of `curve` (open, or closed WITHOUT its repeated last
/// station: `points` is then the cycle) through each node.
///
/// A passage is the station nearest the node (a local minimum of the
/// distance) whose adjacent chords pass within the node's ambiguity radius,
/// or within a quarter of the longer chord where the march stepped coarsely
/// past it (a chord's sag from the curve it samples). The stations inside
/// the radius are dropped; when none is, the nearest station stays on the
/// side of the node it lies on along the branch.
fn passages(points: &[Vec3], cyclic: bool, nodes: &[Node]) -> Vec<Cut> {
    let count = points.len() as isize;
    let at = |index: isize| -> Vec3 { points[index.rem_euclid(count) as usize] };
    // (left, right, node) in unwrapped indices, right > left.
    let mut cuts: Vec<(isize, isize, usize)> = Vec::new();
    for (node_index, node) in nodes.iter().enumerate() {
        let distance = |index: isize| at(index).sub(node.point).length();
        for index in 0..count {
            if !cyclic && (index == 0 || index + 1 == count) {
                // An open branch ending at the node needs no cut: it already
                // ends there, and the imprint's landing finishes it.
                continue;
            }
            let here = distance(index);
            if here > distance(index - 1) || here >= distance(index + 1) {
                continue;
            }
            let before = at(index - 1);
            let after = at(index + 1);
            let near_chord = point_segment_distance(node.point, before, at(index))
                .min(point_segment_distance(node.point, at(index), after));
            let longest = before.sub(at(index)).length().max(after.sub(at(index)).length());
            if near_chord > node.radius.max(0.25 * longest) {
                continue;
            }
            let (left, right) = if here <= node.radius {
                // Drop every station inside the radius, but never past the
                // branch's ends or round the whole cycle.
                let mut low = index;
                let mut high = index;
                let limit = count / 2 - 1;
                while index - low < limit && distance(low - 1) <= node.radius && (cyclic || low > 1) {
                    low -= 1;
                }
                while high - index < limit && distance(high + 1) <= node.radius && (cyclic || high + 2 < count) {
                    high += 1;
                }
                (low - 1, high + 1)
            } else if at(index).sub(node.point).dot(after.sub(before)) < 0.0 {
                (index, index + 1)
            } else {
                (index - 1, index)
            };
            if !cyclic && (left < 0 || right >= count) {
                continue;
            }
            cuts.push((left, right, node_index));
        }
    }
    cuts.sort_by_key(|cut| cut.0);
    // Two passages whose windows overlap are one cut.
    let mut merged: Vec<(isize, isize, usize)> = Vec::new();
    for cut in cuts {
        if let Some(last) = merged.last() {
            if cut.0 < last.1 {
                continue;
            }
        }
        merged.push(cut);
    }
    if cyclic && merged.len() > 1 {
        let first = merged[0];
        let last = merged[merged.len() - 1];
        if last.1 > first.0 + count {
            merged.pop();
        }
    }
    merged
        .into_iter()
        .map(|(left, right, node)| Cut {
            left: left.rem_euclid(count) as usize,
            right: right.rem_euclid(count) as usize,
            node,
        })
        .collect()
}

/// A piece of a branch from one cut (or the branch start) to the next (or
/// its end), with the nodes put on its ends.
#[allow(clippy::too_many_arguments)]
fn piece(
    curve: &SurfaceIntersectionCurve,
    indices: &[usize],
    start_node: Option<&Node>,
    end_node: Option<&Node>,
    periods_a: [f64; 2],
    periods_b: [f64; 2],
) -> Option<SurfaceIntersectionCurve> {
    if indices.is_empty() {
        return None;
    }
    let mut points: Vec<Vec3> = indices.iter().map(|&index| curve.points[index]).collect();
    let mut params_a: Vec<[f64; 2]> = indices.iter().map(|&index| curve.params_a[index]).collect();
    let mut params_b: Vec<[f64; 2]> = indices.iter().map(|&index| curve.params_b[index]).collect();
    let mut tangents: Option<Vec<Vec3>> =
        curve.tangents.as_ref().map(|tangents| indices.iter().map(|&index| tangents[index]).collect());
    if let Some(node) = start_node {
        let a = continue_params(node.params_a, params_a[0], periods_a);
        let b = continue_params(node.params_b, params_b[0], periods_b);
        let chord = points[0].sub(node.point).normalized().ok();
        points.insert(0, node.point);
        params_a.insert(0, a);
        params_b.insert(0, b);
        if let Some(tangents) = tangents.as_mut() {
            let first = tangents[0];
            tangents.insert(0, chord.unwrap_or(first));
        }
    }
    if let Some(node) = end_node {
        let last = points.len() - 1;
        let a = continue_params(node.params_a, params_a[last], periods_a);
        let b = continue_params(node.params_b, params_b[last], periods_b);
        let chord = node.point.sub(points[last]).normalized().ok();
        points.push(node.point);
        params_a.push(a);
        params_b.push(b);
        if let Some(tangents) = tangents.as_mut() {
            let previous = tangents[last];
            tangents.push(chord.unwrap_or(previous));
        }
    }
    if points.len() < 2 {
        return None;
    }
    Some(SurfaceIntersectionCurve {
        points,
        params_a,
        params_b,
        closed: false,
        tangents,
        span_deviation: curve.span_deviation,
        tangent: false,
    })
}

/// Split `curves` at every passage through a crossing node of the two
/// carriers. Returns the curves (unchanged when no node is found) and the
/// points of the nodes a branch was cut at.
pub fn split_at_crossing_nodes(
    first: &NurbsSurface,
    second: &NurbsSurface,
    curves: Vec<SurfaceIntersectionCurve>,
    tolerance: f64,
) -> Result<(Vec<SurfaceIntersectionCurve>, Vec<Vec3>), String> {
    let nodes = find_nodes(first, second, &curves, tolerance)?;
    if nodes.is_empty() {
        return Ok((curves, Vec::new()));
    }
    let periods_a = periods(first)?;
    let periods_b = periods(second)?;
    let debug = crate::debug_flag!("BREP_DEBUG_NODE_SPLIT");
    // Only a node some branch was cut at is reported: the caller records it
    // as a tangent node, which changes how the imprint lands run ends near
    // it, so a contact no branch passes through must not reach it.
    let mut used = vec![false; nodes.len()];
    let mut out = Vec::with_capacity(curves.len() + 2 * nodes.len());
    for curve in curves {
        let full = curve.points.len();
        if full < 3 || curve.tangent || curve.params_a.len() != full || curve.params_b.len() != full {
            out.push(curve);
            continue;
        }
        // A closed branch repeats its start as its last station.
        let cyclic = curve.closed && curve.points[0].sub(curve.points[full - 1]).length() <= tolerance * 10.0;
        let count = if cyclic { full - 1 } else { full };
        let cuts = passages(&curve.points[..count], cyclic, &nodes);
        if debug {
            eprintln!(
                "node split: branch of {full} stations (closed {cyclic}), nodes {:?}, cuts {:?}",
                nodes.iter().map(|node| (node.point, node.radius)).collect::<Vec<_>>(),
                cuts
            );
        }
        if cuts.is_empty() {
            out.push(curve);
            continue;
        }
        for cut in &cuts {
            used[cut.node] = true;
        }
        let mut pieces = Vec::new();
        if cyclic {
            for (index, cut) in cuts.iter().enumerate() {
                let next = cuts[(index + 1) % cuts.len()];
                let mut indices = Vec::new();
                let mut station = cut.right;
                let stop = (next.left + 1) % count;
                loop {
                    indices.push(station);
                    station = (station + 1) % count;
                    if station == stop || indices.len() >= count {
                        break;
                    }
                }
                pieces.push(piece(&curve, &indices, Some(&nodes[cut.node]), Some(&nodes[next.node]), periods_a, periods_b));
            }
        } else {
            let mut start = 0usize;
            let mut start_node: Option<&Node> = None;
            for cut in &cuts {
                let indices: Vec<usize> = (start..=cut.left).collect();
                pieces.push(piece(&curve, &indices, start_node, Some(&nodes[cut.node]), periods_a, periods_b));
                start = cut.right;
                start_node = Some(&nodes[cut.node]);
            }
            let indices: Vec<usize> = (start..count).collect();
            pieces.push(piece(&curve, &indices, start_node, None, periods_a, periods_b));
        }
        let pieces: Vec<SurfaceIntersectionCurve> = pieces.into_iter().flatten().collect();
        if debug {
            eprintln!(
                "node split: -> {} piece(s) of {:?} stations",
                pieces.len(),
                pieces.iter().map(|piece| piece.points.len()).collect::<Vec<_>>()
            );
        }
        out.extend(pieces);
    }
    Ok((
        out,
        nodes
            .iter()
            .zip(&used)
            .filter(|(_, used)| **used)
            .map(|(node, _)| node.point)
            .collect(),
    ))
}

