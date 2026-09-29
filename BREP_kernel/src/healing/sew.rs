//! Standalone heal/sew — Golovanov's "sewing": assemble a valid oriented
//! shell out of faces whose boundary representations were built independently
//! (imported shells, detached face groups, healing after hand edits).
//!
//! The boolean assembler and the offset pipeline each carry private sewing
//! passes specialised to their own invariants (`sew_coincident_one_use_edges`
//! requires already-opposed traversals; the offset welds know their rims).
//! This module is the GENERAL entry: it pairs coincident one-use boundary
//! edges by geometry alone — open chains by matched endpoints + locus
//! agreement, closed rims by mutual locus agreement — WITHOUT any orientation
//! precondition, because a face soup's components may be arbitrarily flipped.
//! Orientation is repaired after pairing: coedge-direction coherence is
//! propagated across the now-shared edges and the whole solid is flipped
//! outward by signed volume.
//!
//! Sewing is BEST-EFFORT and honest: pairs that cannot be joined within
//! tolerance stay open and are counted in the report; nothing is force-welded.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::mass_properties::solid_signed_volume;
use crate::offset_shell::{flip_all_faces, orient_open_solid_faces};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, ShellRecord, VertexRecord};
use crate::{build_pcurve_on_surface, project_point_to_curve, NurbsCurve, Vec3};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SewReport {
    pub edges_sewn: usize,
    pub shells_merged: usize,
    pub open_edges_before: usize,
    pub open_edges_after: usize,
    pub oriented_outward: bool,
    pub issues: Vec<String>,
}

fn one_use_edge_ids(solid: &BrepSolid) -> HashSet<u64> {
    let mut counts = HashMap::<u64, usize>::default();
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
        .into_iter()
        .filter(|(_, count)| *count == 1)
        .map(|(id, _)| id)
        .collect()
}

fn curve_point(edge: &EdgeRecord, fraction: f64) -> Result<Vec3, KernelRefusal> {
    edge.curve
        .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")
}

/// Worst distance from sampled points of `piece` to the locus of `carrier`.
fn locus_deviation(
    piece: &EdgeRecord,
    carrier: &EdgeRecord,
    samples: usize,
) -> Result<f64, KernelRefusal> {
    let mut worst = 0.0f64;
    for index in 0..=samples {
        let point = curve_point(piece, index as f64 / samples as f64)?;
        worst = worst.max(project_point_to_curve(&carrier.curve, point).or_refuse(KernelStage::Sew, "project_point_to_curve")?.distance);
    }
    Ok(worst)
}

/// Pointwise agreement at matched fractions — the two edges are not merely
/// the same locus but the SAME parametrization (the duplicated-edge case),
/// letting a rebind keep the existing pcurve exactly.
fn same_parametrization(
    first: &EdgeRecord,
    second: &EdgeRecord,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    for index in 0..=8 {
        let fraction = index as f64 / 8.0;
        if curve_point(first, fraction)?
            .sub(curve_point(second, fraction)?)
            .length()
            > tolerance
        {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The 3D walk this coedge's loop takes along its edge, sampled from its
/// pcurve through its face surface. Fractions 0 and 0.35: an interior second
/// sample avoids the antipodal-projection ambiguity a closed rim has at 0.5.
fn walk_points(face: &FaceRecord, coedge: &CoedgeRecord) -> Result<[Vec3; 2], KernelRefusal> {
    let [start, end] = coedge.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let sample = |fraction: f64| -> Result<Vec3, String> {
        let uv = coedge.pcurve.evaluate(start + (end - start) * fraction)?;
        face.surface.evaluate(uv.x, uv.y)
    };
    Ok([sample(0.0).or_refuse(KernelStage::Sew, "sample")?, sample(0.35).or_refuse(KernelStage::Sew, "sample")?])
}

/// One planned coedge rebind, resolved before any mutation so a failed plan
/// (an unprojectable pcurve, say) leaves the pair untouched and open.
struct RebindPatch {
    shell: usize,
    face: usize,
    loop_index: usize,
    coedge: usize,
    forward: bool,
    pcurve: Option<NurbsCurve>,
}

/// Plan the rebind of every coedge referencing `remove` onto `keep`. The
/// coedge's `forward` is derived from the 3D walk its existing pcurve
/// produces (geometry is authoritative — components may be flipped), the
/// pcurve is kept when the parametrizations agree and refit otherwise.
fn plan_rebind(
    solid: &BrepSolid,
    keep: &EdgeRecord,
    remove: &EdgeRecord,
    tolerance: f64,
) -> Result<Vec<RebindPatch>, KernelRefusal> {
    let identical = same_parametrization(keep, remove, tolerance)?;
    let closed = keep.start_vertex_id == keep.end_vertex_id;
    let period = keep.t1 - keep.t0;
    let mut patches = Vec::new();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            for (loop_index, loop_record) in face.loops.iter().enumerate() {
                for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
                    if coedge.edge_id != remove.id {
                        continue;
                    }
                    let [walk_start, walk_next] = walk_points(face, coedge)?;
                    let t_start = project_point_to_curve(&keep.curve, walk_start).or_refuse(KernelStage::Sew, "project_point_to_curve")?.u;
                    let t_next = project_point_to_curve(&keep.curve, walk_next).or_refuse(KernelStage::Sew, "project_point_to_curve")?.u;
                    let forward = if closed {
                        let mut delta = t_next - t_start;
                        while delta > period * 0.5 {
                            delta -= period;
                        }
                        while delta < -period * 0.5 {
                            delta += period;
                        }
                        delta > 0.0
                    } else {
                        t_next > t_start
                    };
                    let pcurve = if identical {
                        None
                    } else {
                        let traversal = if forward {
                            keep.curve.clone()
                        } else {
                            keep.curve.reversed().or_refuse(KernelStage::Sew, "reversed")?
                        };
                        Some(build_pcurve_on_surface(&face.surface, &traversal).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?)
                    };
                    patches.push(RebindPatch {
                        shell: shell_index,
                        face: face_index,
                        loop_index,
                        coedge: coedge_index,
                        forward,
                        pcurve,
                    });
                }
            }
        }
    }
    Ok(patches)
}

fn apply_rebind(
    solid: &mut BrepSolid,
    keep: &EdgeRecord,
    remove: &EdgeRecord,
    patches: Vec<RebindPatch>,
    tolerance: f64,
) {
    for patch in patches {
        let coedge = &mut solid.shells[patch.shell].faces[patch.face].loops[patch.loop_index]
            .coedges[patch.coedge];
        coedge.edge_id = keep.id;
        coedge.forward = patch.forward;
        if let Some(pcurve) = patch.pcurve {
            coedge.pcurve = pcurve;
        }
    }
    solid.edges.retain(|edge| edge.id != remove.id);
    // Weld the removed edge's endpoint vertices into the kept edge's, so
    // OTHER edges of the removed component (a loop mixes sewn rim edges with
    // still-duplicate interior edges) chain through the shared vertices.
    // Only coincident endpoints weld — a rotated closed rim keeps its own
    // seam vertex.
    let point_of = |vertex_id: u64| {
        solid
            .vertices
            .iter()
            .find(|vertex| vertex.id == vertex_id)
            .map(|vertex| vertex.point)
    };
    let mut welds = Vec::<(u64, u64)>::new();
    for from in [remove.start_vertex_id, remove.end_vertex_id] {
        let Some(from_point) = point_of(from) else {
            continue;
        };
        let target = [keep.start_vertex_id, keep.end_vertex_id]
            .into_iter()
            .filter_map(|candidate| {
                point_of(candidate).map(|point| (candidate, point.sub(from_point).length()))
            })
            .min_by(|first, second| first.1.total_cmp(&second.1));
        if let Some((to, distance)) = target {
            if distance <= tolerance && from != to {
                welds.push((from, to));
            }
        }
    }
    for (from, to) in welds {
        for edge in &mut solid.edges {
            if edge.start_vertex_id == from {
                edge.start_vertex_id = to;
            }
            if edge.end_vertex_id == from {
                edge.end_vertex_id = to;
            }
        }
    }
}

/// Decide which of an accepted pair is KEPT and which is REMOVED.
///
/// For an open chain, and for a closed rim whose two copies share a seam
/// point, either direction works and the scan's own order stands — `first` is
/// kept, exactly as before.
///
/// A seam-ROTATED closed rim is different, and the difference is not cosmetic.
/// The removed rim takes its seam vertex out of the solid with it, so every
/// OTHER edge that chained through that vertex is left chaining through
/// nothing. `apply_rebind` deliberately does not drag it onto the kept rim's
/// seam — a rotated closed rim keeps its own seam vertex, because moving it
/// would pull a real edge endpoint an arbitrary arc round the circle, off its
/// own curve — so the tear is not repaired, it is inherited. Sewn that way a
/// cylinder comes back with `edges_sewn: 1, open_edges_after: 0` over a wall
/// loop torn at the seam: a report that reads like success.
///
/// The direction that works is the one whose REMOVED rim's seam vertex bounds
/// no other edge. When neither qualifies — both seams anchor real geometry —
/// no rebind keeps both loops closed, and the pair is left OPEN and counted
/// rather than sewn into a solid whose own report contradicts its issues.
fn orient_pair<'a>(
    solid: &BrepSolid,
    first: &'a EdgeRecord,
    second: &'a EdgeRecord,
    points: &HashMap<u64, Vec3>,
    tolerance: f64,
) -> Option<(&'a EdgeRecord, &'a EdgeRecord)> {
    // The caller has already rejected mismatched closedness, so testing
    // `first` tests both.
    if first.start_vertex_id != first.end_vertex_id {
        return Some((first, second));
    }
    let (Some(&first_seam), Some(&second_seam)) = (
        points.get(&first.start_vertex_id),
        points.get(&second.start_vertex_id),
    ) else {
        return Some((first, second));
    };
    if first_seam.sub(second_seam).length() <= tolerance {
        // Seams coincide: `apply_rebind`'s weld joins them and every loop
        // through either one still chains. Order is free.
        return Some((first, second));
    }
    let seam_is_exclusive = |rim: &EdgeRecord| {
        !solid.edges.iter().any(|edge| {
            edge.id != rim.id
                && (edge.start_vertex_id == rim.start_vertex_id
                    || edge.end_vertex_id == rim.start_vertex_id)
        })
    };
    if seam_is_exclusive(second) {
        Some((first, second))
    } else if seam_is_exclusive(first) {
        Some((second, first))
    } else {
        None
    }
}

/// Merge shells that now share an edge into single shells (union-find).
fn merge_connected_shells(solid: &mut BrepSolid) -> usize {
    let mut owner_of_edge = HashMap::<u64, usize>::default();
    let mut parent = (0..solid.shells.len()).collect::<Vec<_>>();
    fn root(parent: &mut [usize], index: usize) -> usize {
        if parent[index] != index {
            parent[index] = root(parent, parent[index]);
        }
        parent[index]
    }
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for coedge in shell
            .faces
            .iter()
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
        {
            if let Some(&other) = owner_of_edge.get(&coedge.edge_id) {
                let first = root(&mut parent, other);
                let second = root(&mut parent, shell_index);
                if first != second {
                    parent[second] = first;
                }
            } else {
                owner_of_edge.insert(coedge.edge_id, shell_index);
            }
        }
    }
    let before = solid.shells.len();
    let original = std::mem::take(&mut solid.shells);
    let mut merged = Vec::<ShellRecord>::new();
    let mut group_of = HashMap::<usize, usize>::default();
    for (index, shell) in original.into_iter().enumerate() {
        let group = root(&mut parent, index);
        if let Some(&target) = group_of.get(&group) {
            merged[target].faces.extend(shell.faces);
        } else {
            group_of.insert(group, merged.len());
            merged.push(shell);
        }
    }
    solid.shells = merged;
    before - solid.shells.len()
}

/// Which `second` candidates the pair search offers for a given `first`.
///
/// The two strategies feed the SAME loop body in the SAME ascending order, so
/// "the prefilter is correct" reduces to one claim: the offered list is a
/// SUPERSET of the candidates that could pass. Nothing else in the search can
/// differ, which is what makes the parity test (`sew_prefilter_parity`) a real
/// proof rather than a spot check — first-match-wins makes iteration order
/// part of the RESULT, not just the cost.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PairSearch {
    /// Production: neighbourhood query for open chains, closed-rim shortlist
    /// for closed rims.
    Hashed,
    /// The reference brute-force scan the prefilter must reproduce exactly.
    #[cfg_attr(not(test), allow(dead_code))]
    Exhaustive,
}

/// The integer cell of `point`, or `None` when the coordinates cannot be
/// bucketed at all (non-finite, or so large the index overflows the i64 key).
/// A `None` anywhere disables the prefilter for the whole pass rather than
/// silently dropping a candidate — exact parity with the exhaustive scan is
/// the contract, and one lost candidate would break it.
fn grid_cell(point: Vec3, cell: f64) -> Option<[i64; 3]> {
    let mut key = [0i64; 3];
    for (slot, value) in key.iter_mut().zip([point.x, point.y, point.z]) {
        if !value.is_finite() {
            return None;
        }
        let scaled = (value / cell).floor();
        // 9e15 keeps the cast well inside i64 and inside f64's exact-integer
        // range, so `as i64` is a faithful conversion, not a saturating one.
        if !scaled.is_finite() || scaled.abs() > 9.0e15 {
            return None;
        }
        *slot = scaled as i64;
    }
    Some(key)
}

/// Uniform spatial hash over the ENDPOINTS of the open one-use candidates —
/// the prefilter that turns the pair search from a full rescan per accepted
/// pair into a neighbourhood query.
///
/// Cells are `2·tolerance` wide, not `tolerance`. A partner endpoint is at
/// most `tolerance = cell/2` away, i.e. at most HALF a cell, so it lands in
/// this cell or an immediate neighbour with a full half-cell of slack — slack
/// that absorbs the rounding of the `point/cell` division. At `cell =
/// tolerance` two points exactly `tolerance` apart can straddle two cell
/// boundaries and the 3×3×3 block would still hold them, but with zero margin
/// for the division's last bit; the factor of two costs one extra ring of
/// mostly-empty buckets and buys certainty.
struct EndpointGrid {
    cell: f64,
    buckets: HashMap<[i64; 3], Vec<usize>>,
}

impl EndpointGrid {
    /// Bucket every open candidate under BOTH of its endpoints.
    ///
    /// Closed rims are deliberately absent. They are paired by mutual locus
    /// agreement alone — `sew_solid`'s closed branch never looks at a vertex —
    /// and a seam-ROTATED duplicate rim shares its partner's locus while
    /// sharing no point position at all, so any point-keyed bucket (midpoint
    /// included) would drop exactly the pair the closed lane exists to catch.
    /// Closed rims therefore keep a linear shortlist; see `offer_seconds`.
    ///
    /// Candidates whose vertex records are missing are absent too: the
    /// exhaustive scan `continue`s such a pair outright, so dropping it here
    /// changes nothing.
    fn build(endpoints: &[Option<[Vec3; 2]>], closed: &[bool], tolerance: f64) -> Option<Self> {
        let cell = 2.0 * tolerance;
        if !(cell.is_finite() && cell > 0.0) {
            return None;
        }
        let mut buckets = HashMap::<[i64; 3], Vec<usize>>::default();
        for (position, ends) in endpoints.iter().enumerate() {
            if closed[position] {
                continue;
            }
            let Some(ends) = ends else {
                continue;
            };
            for point in ends {
                buckets
                    .entry(grid_cell(*point, cell)?)
                    .or_default()
                    .push(position);
            }
        }
        Some(EndpointGrid { cell, buckets })
    }

    /// Every open candidate with an endpoint in the 3×3×3 block around
    /// `point`, ascending and deduplicated. A superset of "has an endpoint
    /// within tolerance of `point`", which is all the pair search needs.
    /// `false` means the point itself is unbucketable and the caller must not
    /// trust the (empty) answer.
    fn near(&self, point: Vec3, out: &mut Vec<usize>) -> bool {
        out.clear();
        let Some(centre) = grid_cell(point, self.cell) else {
            return false;
        };
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(bucket) =
                        self.buckets
                            .get(&[centre[0] + dx, centre[1] + dy, centre[2] + dz])
                    {
                        out.extend_from_slice(bucket);
                    }
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        true
    }
}

/// Fill `offered` with the `second` positions to test against
/// `candidates[first_position]`, ASCENDING — the order the exhaustive scan
/// used, which first-match-wins makes load-bearing.
///
/// Three lanes, each a provable superset of what can pass:
/// - **No grid** (prefilter disabled, or `PairSearch::Exhaustive`): every
///   later position, i.e. the original scan verbatim.
/// - **Closed `first`**: the later CLOSED positions. An open `second` is
///   rejected by the `first_closed != second_closed` test before any
///   geometry is touched, so shortlisting them away changes nothing — but
///   nothing finer is available, because closed-rim pairing reads the locus
///   and not any point (see `EndpointGrid::build`). Closed rims stay
///   quadratic in their own (small) count; that is honest, not hidden.
/// - **Open `first`**: one neighbourhood query around `first`'s START point.
///   An open pair is accepted only when `first`'s start matches `second`'s
///   start (direct) or `second`'s end (reversed), so EVERY pair that can pass
///   has an endpoint within tolerance of that single point, and one query is
///   a complete superset. A `first` whose vertex records are missing offers
///   nothing, matching the exhaustive scan's `continue`.
#[allow(clippy::too_many_arguments)]
fn offer_seconds(
    grid: Option<&EndpointGrid>,
    first_position: usize,
    first_closed: bool,
    endpoints: &[Option<[Vec3; 2]>],
    closed_positions: &[usize],
    candidate_count: usize,
    neighbours: &mut Vec<usize>,
    offered: &mut Vec<usize>,
) {
    offered.clear();
    let Some(grid) = grid else {
        offered.extend(first_position + 1..candidate_count);
        return;
    };
    if first_closed {
        offered.extend(
            closed_positions
                .iter()
                .copied()
                .filter(|&position| position > first_position),
        );
        return;
    }
    let Some([start, _]) = endpoints[first_position] else {
        return;
    };
    if grid.near(start, neighbours) {
        offered.extend(
            neighbours
                .iter()
                .copied()
                .filter(|&position| position > first_position),
        );
    }
}

/// Best-effort sew of a solid's open boundary edges.
///
/// Pairs coincident one-use edges (open chains by matched endpoints + locus
/// agreement, closed rims by mutual locus agreement) and rebinds each pair to
/// one shared edge, merging the shells they join. Orientation carries NO
/// precondition: after pairing, coedge-direction coherence is propagated
/// across the shared edges and the result is flipped outward by signed
/// volume. Unsewable gaps stay open and are reported, never force-welded.
pub fn sew_solid(solid: &BrepSolid, tolerance: f64) -> Result<(BrepSolid, SewReport), KernelRefusal> {
    let (sewn, report, _examined) = sew_solid_with_search(solid, tolerance, PairSearch::Hashed)?;
    Ok((sewn, report))
}

/// [`sew_solid`] with the pair-search strategy exposed, plus the number of
/// (first, second) pairs the search actually examined.
///
/// The count is what the scaling smoke asserts on: it is a deterministic
/// function of the input, where wall time is not, so a band set from it is a
/// real bound rather than a machine-speed lottery.
fn sew_solid_with_search(
    solid: &BrepSolid,
    tolerance: f64,
    search: PairSearch,
) -> Result<(BrepSolid, SewReport, usize), KernelRefusal> {
    if !(tolerance.is_finite() && tolerance > 0.0) {
        return Err(KernelRefusal::input(KernelStage::Collect, "tolerance", "sew_solid: tolerance must be positive"));
    }
    let mut result = solid.clone();
    let open_edges_before = one_use_edge_ids(&result).len();
    let mut edges_sewn = 0usize;
    let mut pairs_examined = 0usize;
    // Pairs whose rebind plan failed (unprojectable pcurve) — left open
    // rather than retried forever.
    //
    // Keyed UNORDERED, because `orient_pair` may swap which of the two is kept
    // and which is removed. Keyed by (keep, remove) the block would be looked
    // up under (first, second), miss on a swapped pair, and the scan would
    // re-offer it, re-swap it, and fail to plan it again on every outer
    // iteration — nothing having been mutated in between, that is a hang, not
    // a retry. A pair that cannot be rebound is blocked whichever way round.
    let mut blocked = HashSet::<(u64, u64)>::default();
    let unordered = |first: u64, second: u64| (first.min(second), first.max(second));
    let mut offered = Vec::<usize>::new();
    let mut neighbours = Vec::<usize>::new();
    loop {
        let one_use = one_use_edge_ids(&result);
        // Candidate POSITIONS into `result.edges`, in edge order — the same
        // sequence the exhaustive scan walked, without cloning a NURBS curve
        // per candidate per accepted pair (that clone was the other O(k·n)
        // term hiding behind the O(k·n²) comparison count).
        let candidates = result
            .edges
            .iter()
            .enumerate()
            .filter(|(_, edge)| !edge.degenerate && one_use.contains(&edge.id))
            .map(|(index, _)| index)
            .collect::<Vec<_>>();
        let points = result
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect::<HashMap<_, _>>();
        let closed = candidates
            .iter()
            .map(|&index| {
                let edge = &result.edges[index];
                edge.start_vertex_id == edge.end_vertex_id
            })
            .collect::<Vec<_>>();
        // Both endpoint positions, or `None` when a vertex record is missing —
        // exactly the tuple the exhaustive scan destructures, hoisted out of
        // the inner loop so each candidate's two lookups happen once.
        let endpoints = candidates
            .iter()
            .map(|&index| {
                let edge = &result.edges[index];
                match (
                    points.get(&edge.start_vertex_id),
                    points.get(&edge.end_vertex_id),
                ) {
                    (Some(&start), Some(&end)) => Some([start, end]),
                    _ => None,
                }
            })
            .collect::<Vec<_>>();
        let closed_positions = (0..candidates.len())
            .filter(|&position| closed[position])
            .collect::<Vec<_>>();
        let grid = match search {
            PairSearch::Hashed => EndpointGrid::build(&endpoints, &closed, tolerance),
            PairSearch::Exhaustive => None,
        };

        let mut chosen = None;
        'pairs: for first_position in 0..candidates.len() {
            let first = &result.edges[candidates[first_position]];
            let first_closed = closed[first_position];
            offer_seconds(
                grid.as_ref(),
                first_position,
                first_closed,
                &endpoints,
                &closed_positions,
                candidates.len(),
                &mut neighbours,
                &mut offered,
            );
            for second_position in offered.iter().copied() {
                let second = &result.edges[candidates[second_position]];
                pairs_examined += 1;
                if blocked.contains(&unordered(first.id, second.id)) {
                    continue;
                }
                if first_closed != closed[second_position] {
                    continue;
                }
                if !first_closed {
                    let (Some([fs, fe]), Some([ss, se])) =
                        (endpoints[first_position], endpoints[second_position])
                    else {
                        continue;
                    };
                    let direct =
                        fs.sub(ss).length() <= tolerance && fe.sub(se).length() <= tolerance;
                    let reversed =
                        fs.sub(se).length() <= tolerance && fe.sub(ss).length() <= tolerance;
                    if !direct && !reversed {
                        continue;
                    }
                }
                if locus_deviation(first, second, 8)? > tolerance
                    || locus_deviation(second, first, 8)? > tolerance
                {
                    continue;
                }
                let Some((keep, remove)) = orient_pair(&result, first, second, &points, tolerance)
                else {
                    // A seam-rotated rim pair with no safe direction: leave it
                    // open, do not sew a torn loop shut.
                    continue;
                };
                chosen = Some((keep.clone(), remove.clone()));
                break 'pairs;
            }
        }
        let Some((keep, remove)) = chosen else {
            break;
        };
        match plan_rebind(&result, &keep, &remove, tolerance) {
            Ok(patches) => {
                apply_rebind(&mut result, &keep, &remove, patches, tolerance);
                edges_sewn += 1;
            }
            Err(_) => {
                blocked.insert(unordered(keep.id, remove.id));
            }
        }
    }
    // Drop vertices only referenced by removed duplicate edges.
    let used_vertices = result
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect::<HashSet<_>>();
    result
        .vertices
        .retain(|vertex| used_vertices.contains(&vertex.id));

    let shells_merged = merge_connected_shells(&mut result);
    let mut oriented_outward = false;
    if edges_sewn > 0 {
        // Components can arrive arbitrarily flipped; make the coedge
        // directions coherent across the shared edges, then restore outward
        // normals by signed volume.
        orient_open_solid_faces(&mut result)?;
        if let Ok(volume) = solid_signed_volume(&result) {
            if volume < 0.0 {
                flip_all_faces(&mut result)?;
            }
            oriented_outward = volume.abs() > tolerance * tolerance * tolerance;
        }
    }
    // Re-derive genus from the Euler characteristic (V − E + F − H = 2 − 2g,
    // H counting inner loops, matching validate()) so validation sees the
    // merged topology, not the input's bookkeeping. Only when sewing actually
    // changed the topology — a no-op must not touch a valid genus.
    if edges_sewn > 0 || shells_merged > 0 {
        let vertex_count = result.vertices.len() as i64;
        let edge_count = result.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
        let face_count = result
            .shells
            .iter()
            .map(|shell| shell.faces.len())
            .sum::<usize>() as i64;
        let ring_count = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .map(|face| face.loops.len().saturating_sub(1))
            .sum::<usize>() as i64;
        let numerator = 2 - (vertex_count - edge_count + face_count - ring_count);
        if numerator >= 0 && numerator % 2 == 0 {
            result.genus = numerator / 2;
        }
    }
    let open_edges_after = one_use_edge_ids(&result).len();
    let issues = result
        .validate()
        .into_iter()
        .map(|issue| issue.message)
        .collect();
    Ok((
        result,
        SewReport {
            edges_sewn,
            shells_merged,
            open_edges_before,
            open_edges_after,
            oriented_outward,
            issues,
        },
        pairs_examined,
    ))
}


/// Split PINCHED vertices — points where two (or more) umbrella fans of
/// faces meet at a single vertex record. Local manifold checks (edge use
/// counts, loop closure, orientation) cannot see a pinch; it surfaces only
/// as an odd Euler characteristic. The link of a manifold boundary vertex is
/// a single edge-connected fan: union incident edges through every loop
/// CORNER at the vertex (consecutive coedges meeting there inside one face);
/// more than one component means distinct fans sharing the record — give
/// each extra fan its own vertex at the same point and reassign that fan's
/// edge endpoints. Geometry is untouched; only identity is repaired.
pub fn split_pinched_vertices(solid: &mut BrepSolid) -> Result<usize, KernelRefusal> {
    let mut split_count = 0usize;
    let vertex_ids: Vec<u64> = solid.vertices.iter().map(|vertex| vertex.id).collect();
    let mut next_id = solid
        .vertices
        .iter()
        .map(|vertex| vertex.id)
        .chain(solid.edges.iter().map(|edge| edge.id))
        .max()
        .unwrap_or(0)
        + 1;
    // id -> index maps built once. This routine only mutates edge endpoints
    // in place and appends vertices, so indices never shift: every lookup
    // returns the identical live record the previous `.iter().find(id==)`
    // scans returned. Kills the O(V*coedges*E) nested edge find below.
    let edge_of_id: HashMap<u64, usize> = solid
        .edges
        .iter()
        .enumerate()
        .map(|(index, edge)| (edge.id, index))
        .collect();
    let vertex_of_id: HashMap<u64, usize> = solid
        .vertices
        .iter()
        .enumerate()
        .map(|(index, vertex)| (vertex.id, index))
        .collect();
    for vertex_id in vertex_ids {
        // Incident edges (either endpoint; closed and degenerate included so
        // pole/seam structures stay connected through their corners).
        let incident: Vec<u64> = solid
            .edges
            .iter()
            .filter(|edge| edge.start_vertex_id == vertex_id || edge.end_vertex_id == vertex_id)
            .map(|edge| edge.id)
            .collect();
        if incident.len() < 4 {
            // A pinch needs at least two fans of >= 2 edges each.
            continue;
        }
        let index_of: HashMap<u64, usize> = incident
            .iter()
            .enumerate()
            .map(|(index, id)| (*id, index))
            .collect();
        let mut parent: Vec<usize> = (0..incident.len()).collect();
        fn root(parent: &mut [usize], index: usize) -> usize {
            if parent[index] != index {
                parent[index] = root(parent, parent[index]);
            }
            parent[index]
        }
        let edge_end = |edge_id: u64, forward: bool| -> Option<u64> {
            edge_of_id.get(&edge_id).map(|&index| {
                let edge = &solid.edges[index];
                if forward {
                    edge.end_vertex_id
                } else {
                    edge.start_vertex_id
                }
            })
        };
        for shell in &solid.shells {
            for face in &shell.faces {
                for loop_record in &face.loops {
                    let count = loop_record.coedges.len();
                    for index in 0..count {
                        let current = &loop_record.coedges[index];
                        let next = &loop_record.coedges[(index + 1) % count];
                        // The corner between current and next sits at
                        // current's traversal END vertex.
                        let Some(junction) = edge_end(current.edge_id, current.forward) else {
                            continue;
                        };
                        if junction != vertex_id {
                            continue;
                        }
                        let (Some(&a), Some(&b)) =
                            (index_of.get(&current.edge_id), index_of.get(&next.edge_id))
                        else {
                            continue;
                        };
                        let ra = root(&mut parent, a);
                        let rb = root(&mut parent, b);
                        if ra != rb {
                            parent[rb] = ra;
                        }
                    }
                }
            }
        }
        let mut component_of: HashMap<usize, usize> = HashMap::default();
        let mut components = 0usize;
        let mut assignment: Vec<usize> = vec![0; incident.len()];
        for index in 0..incident.len() {
            let r = root(&mut parent, index);
            let component = *component_of.entry(r).or_insert_with(|| {
                components += 1;
                components - 1
            });
            assignment[index] = component;
        }
        if components < 2 {
            continue;
        }
        // Keep the original record for component 0; every further fan gets a
        // duplicate vertex at the same point.
        let point = vertex_of_id
            .get(&vertex_id)
            .map(|&index| solid.vertices[index].point)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "pinched_vertex", "split_pinched_vertices: vertex vanished"))?;
        let mut replacement_ids = vec![vertex_id];
        for _ in 1..components {
            let id = next_id;
            next_id += 1;
            solid.vertices.push(VertexRecord { id, point });
            replacement_ids.push(id);
        }
        for (offset, edge_id) in incident.iter().enumerate() {
            let replacement = replacement_ids[assignment[offset]];
            if replacement == vertex_id {
                continue;
            }
            if let Some(edge) = solid.edges.iter_mut().find(|edge| edge.id == *edge_id) {
                if edge.start_vertex_id == vertex_id {
                    edge.start_vertex_id = replacement;
                }
                if edge.end_vertex_id == vertex_id {
                    edge.end_vertex_id = replacement;
                }
            }
        }
        split_count += components - 1;
    }
    Ok(split_count)
}

