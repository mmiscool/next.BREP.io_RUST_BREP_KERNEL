//! Fuse-first operand healing pre-pass.  The tolerance-hardening work it came
//! from has two halves: size-coupled bands let the ~120 downstream predicates
//! *tolerate* near-degenerate input, while this pass *removes* that noise from
//! the operand up front so far fewer of those predicates ever matter.
//!
//! Booleans and fillets consume their operands *as-is*.  When the input carries
//! near-degenerate noise — points that should coincide but drifted by the
//! sketch solver's ~5e-5 relaxation floor, or vertices that should lie on a
//! planar face but sit a few microns off it — that noise is what tips a fixed
//! spatial band over the edge downstream (a wall skipped in the corner-round
//! guard, a boolean fragment mis-assembled).  `heal_operands` runs *before* any
//! intersection/surgery and snaps that noise out at the geometry level:
//!
//!   1. **Vertex fuse.**  Cluster the solid's vertices whose pairwise distance
//!      is within the size-coupled `heal_tol` (union-find) and collapse each
//!      cluster to a single representative point.  Two vertices that are the two
//!      ends of a genuine, non-degenerate edge are never fused (that would
//!      delete a real feature).
//!   2. **Plane snap.**  For a representative that lies (within `heal_tol`) on
//!      one or more PLANAR incident faces, project it exactly onto the common
//!      intersection of those planes.  This removes the off-plane drift that
//!      the corner-round through-plane test (`blend.rs`) and the classifier
//!      On-band are sensitive to, and it avoids the BRL-CAD "vertex off face
//!      plane" hazard.
//!   3. **Edge re-anchor.**  Re-pin every incident edge-curve's terminal
//!      control point exactly onto its (possibly moved) vertex by reusing
//!      [`crate::boolean::commit_nearby_edge_endpoints`] — the same geometry
//!      mutator the boolean/fillet output heals already use.
//!
//! Faces, loops, winding and edge identity are otherwise left untouched.
//!
//! ## Safety
//! * **No-op on clean inputs → bit-identical output.**  A clean solid has every
//!   distinct vertex separated by far more than `heal_tol`, so no cluster has
//!   more than one member, and every vertex already lies on its incident planes
//!   to floating-point precision (below [`ACTIVATION_FLOOR`]).  Nothing moves,
//!   so the boolean/fillet output is byte-identical.  `ACTIVATION_FLOOR` is the
//!   guarantee: a vertex is only ever touched once it is measurably dirty.
//! * **Validate-gate backstop.**  The whole heal is validate-gated: if it makes
//!   the solid's [`crate::topology::BrepSolid::validate`] issue count WORSE, the
//!   heal is discarded and the original operand is used unchanged (mirroring the
//!   non-worsening guard in `boolean::polish_triple_junction_vertices`).
//! * **Tiny-feature cap.**  The fuse radius for a given vertex is additionally
//!   capped at a fraction of its shortest incident non-degenerate edge, so a
//!   large part carrying a genuinely tiny feature cannot have that feature
//!   fused away.

use crate::KernelRefusal;
use crate::topology::BrepSolid;
use crate::{solid_scale, AnalyticSurface, KernelTolerances, Vec3};
use rustc_hash::FxHashMap as HashMap;

/// Dimensionless part-per-diagonal factor for the size-coupled heal band
/// (`heal_tol = policy.heal_band(diagonal, HEAL_K)`).  Calibrated so the band
/// sits comfortably above the observed ~9e-5 input noise on the ~25-30 unit
/// reported parts (diagonal·1e-5 ≈ 3e-4 on a 30-unit part) yet far below the
/// smallest real feature there (the 0.2 fillet radius, the ~10-unit extents).
const HEAL_K: f64 = 1e-5;

/// A vertex is only re-snapped when it is measurably off its planes (or joined
/// to a cluster) by more than this absolute floor.  Clean vertices lie on their
/// incident planes to full floating-point precision (well below this), so they
/// are never touched — which is exactly what keeps clean-input healing a no-op
/// and the golden/boolean-volume parity bit-identical.
const ACTIVATION_FLOOR: f64 = 1e-9;

/// Fraction of the shortest incident non-degenerate edge that caps a vertex's
/// fuse radius, protecting a genuinely tiny feature carried on a large part.
const FEATURE_CAP: f64 = 0.25;

/// Regularization weight anchoring the plane-fit least-squares to the current
/// vertex position, so 0/1/2 incident planes leave the free directions at the
/// current coordinate and only ≥3 independent planes pin all three axes.  Small
/// enough that the ≥3-plane solution is the true intersection to sub-picometre
/// accuracy; a vertex already exactly on all its planes maps to itself exactly.
const PLANE_ANCHOR: f64 = 1e-6;

/// Broad phase only: visit candidate pairs in the same (i, j) order as the
/// original all-pairs loop. Exact distances and feature caps remain downstream.
fn visit_heal_pairs(points: &[Vec3], heal_tol: f64, mut visit: impl FnMut(usize, usize)) {
    let brute_force = |visit: &mut dyn FnMut(usize, usize)| {
        for i in 0..points.len() {
            for j in i + 1..points.len() {
                visit(i, j);
            }
        }
    };
    // Tiny inputs cost less to scan. Underflow in the existing squared-distance
    // calculation can also accept pairs farther apart than a tiny radius; keep
    // its exact behavior rather than pruning those pairs with a grid.
    let width = 2.0 * heal_tol;
    if points.len() <= 32 || heal_tol < 1e-150 || !width.is_finite() || width <= 0.0 {
        brute_force(&mut visit);
        return;
    }
    // Two-radius cells leave rounding headroom: an accepted pair differs by
    // at most roughly half a cell per axis. Restrict quotients so division
    // rounding cannot consume that headroom, and integer neighbors cannot
    // overflow. Fall back for non-finite or very distant coordinates.
    let keys: Option<Vec<[i64; 3]>> = points
        .iter()
        .map(|point| {
            let q = [point.x / width, point.y / width, point.z / width];
            q.iter()
                .all(|v| v.is_finite() && v.abs() <= (1u64 << 48) as f64)
                .then(|| q.map(|v| v.floor() as i64))
        })
        .collect();
    let Some(keys) = keys else {
        brute_force(&mut visit);
        return;
    };
    let mut cells: HashMap<[i64; 3], Vec<usize>> = HashMap::default();
    for (i, key) in keys.iter().enumerate() {
        cells.entry(*key).or_default().push(i);
    }
    let mut candidates = Vec::new();
    for (i, &[x, y, z]) in keys.iter().enumerate() {
        candidates.clear();
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(indices) = cells.get(&[x + dx, y + dy, z + dz]) {
                        candidates.extend(indices.iter().copied().filter(|&j| j > i));
                    }
                }
            }
        }
        // Union-find roots affect centroid accumulation and subsequent snaps.
        // Preserve source order, independent of hash bucket traversal order.
        candidates.sort_unstable();
        for &j in &candidates {
            visit(i, j);
        }
    }
}

/// Heal a single operand in place before it is handed to a boolean or fillet.
///
/// Validate-gated: on any input this either improves (or leaves unchanged) the
/// solid or is discarded entirely, so it can never make an operand worse.
pub(crate) fn heal_operands(
    solid: &mut BrepSolid,
    policy: &KernelTolerances,
) -> Result<(), KernelRefusal> {
    if solid.vertices.len() < 2 {
        return Ok(());
    }
    let diagonal = solid_scale(solid);
    let heal_tol = policy.heal_band(diagonal, HEAL_K);
    if !(heal_tol > 0.0) || !heal_tol.is_finite() {
        return Ok(());
    }

    let original = solid.clone();
    let before = original.validate_with_tolerances(policy).len();

    let moved = heal_operands_inner(solid, heal_tol, policy.model)?;
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();
    if !moved {
        // Nothing was dirty enough to touch — output is byte-identical.
        if debug {
            eprintln!("heal: no-op (0 vertices moved, heal_tol={heal_tol:.3e})");
        }
        return Ok(());
    }
    if debug {
        let count = original
            .vertices
            .iter()
            .zip(&solid.vertices)
            .filter(|(a, b)| a.point.sub(b.point).length() > 0.0)
            .count();
        eprintln!("heal: moved {count} vertices (heal_tol={heal_tol:.3e})");
    }

    let after = solid.validate_with_tolerances(policy).len();
    if after > before {
        // The heal made the solid worse: discard it and use the original.
        if debug {
            eprintln!("heal: discarded (validate worsened {before} -> {after})");
        }
        *solid = original;
        return Ok(());
    }

    if debug {
        oracle_scan(solid, heal_tol);
    }
    Ok(())
}

/// Returns `true` iff at least one vertex position actually changed.
fn heal_operands_inner(solid: &mut BrepSolid, heal_tol: f64, model: f64) -> Result<bool, KernelRefusal> {
    let n = solid.vertices.len();

    // ----- incident planar planes, per vertex id -----
    // A plane is stored as (unit normal, signed offset d) with n·x = d.
    let planes_by_vertex = incident_planes(solid);

    // ----- shortest incident non-degenerate edge, per vertex id -----
    let shortest_edge = shortest_incident_edges(solid);

    // ----- union-find over vertices, fusing pairs within heal_tol -----
    let ids: Vec<u64> = solid.vertices.iter().map(|v| v.id).collect();
    let points: Vec<Vec3> = solid.vertices.iter().map(|v| v.point).collect();
    let index_of: HashMap<u64, usize> = ids.iter().enumerate().map(|(i, id)| (*id, i)).collect();

    // Vertex pairs that are the two ends of a genuine (non-degenerate, longer
    // than heal_tol) edge must never be fused — that would collapse a real
    // edge.  Collect them as a forbidden set keyed on unordered index pairs.
    let mut forbidden: rustc_hash::FxHashSet<(usize, usize)> = rustc_hash::FxHashSet::default();
    for edge in &solid.edges {
        if edge.start_vertex_id == edge.end_vertex_id {
            continue;
        }
        let (Some(&a), Some(&b)) = (
            index_of.get(&edge.start_vertex_id),
            index_of.get(&edge.end_vertex_id),
        ) else {
            continue;
        };
        let length = points[a].sub(points[b]).length();
        if !edge.degenerate && length > heal_tol {
            forbidden.insert((a.min(b), a.max(b)));
        }
    }

    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut Vec<usize>, mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }

    // Cap the fuse radius by a fraction of the shortest incident feature.
    let caps: Vec<f64> = ids
        .iter()
        .map(|id| {
            shortest_edge
                .get(id)
                .map(|len| len * FEATURE_CAP)
                .unwrap_or(f64::INFINITY)
        })
        .collect();
    visit_heal_pairs(&points, heal_tol, |i, j| {
        if forbidden.contains(&(i, j)) {
            return;
        }
        let radius = heal_tol.min(caps[i]).min(caps[j]);
        if points[i].sub(points[j]).length() <= radius {
            let a = find(&mut parent, i);
            let b = find(&mut parent, j);
            if a != b {
                parent[a] = b;
            }
        }
    });

    // ----- group vertices by cluster root -----
    let mut clusters: HashMap<usize, Vec<usize>> = HashMap::default();
    for i in 0..n {
        let root = find(&mut parent, i);
        clusters.entry(root).or_default().push(i);
    }

    // ----- decide each cluster's representative point -----
    let mut new_point: Vec<Vec3> = points.clone();
    let mut any_move = false;
    let mut measured = crate::EntityTolerances::with_floor(solid, model);
    for members in clusters.values() {
        // Base position: the centroid of the cluster (a single vertex keeps its
        // own position as the base).
        let base = members
            .iter()
            .fold(Vec3::default(), |acc, &m| acc.add(points[m]))
            .scale(1.0 / members.len() as f64);

        // Gather every incident planar plane across the cluster's members.
        let mut planes: Vec<(Vec3, f64)> = Vec::new();
        for &m in members {
            if let Some(list) = planes_by_vertex.get(&ids[m]) {
                for &(normal, offset) in list {
                    // Deduplicate near-parallel coincident planes so a face
                    // shared by several members does not over-weight the fit.
                    if !planes.iter().any(|(pn, pd)| {
                        pn.dot(normal).abs() > 1.0 - 1e-9 && (pd - offset).abs() <= heal_tol
                    }) {
                        planes.push((normal, offset));
                    }
                }
            }
        }

        // Project the base onto the common intersection of its incident planes
        // (regularized so under-constrained directions keep the base coord).
        let mut target = plane_snap(base, &planes);

        let is_fuse = members.len() > 1;
        if is_fuse {
            // Proximity proposes a correspondence; it does not authorize
            // moving already consistent vertices across an intentional wall.
            // Bound each movement by its measured endpoint/plane discrepancy.
            let budgets: Vec<f64> = members.iter().map(|&m| {
                let plane_error = planes_by_vertex.get(&ids[m]).into_iter()
                    .flatten().map(|(normal, offset)| (normal.dot(points[m]) - offset).abs())
                    .fold(0.0_f64, f64::max);
                measured.vertex(ids[m]).max(plane_error).max(model).min(heal_tol)
            }).collect();
            let fits = |candidate: Vec3| members.iter().zip(&budgets)
                .all(|(&m, &budget)| candidate.sub(points[m]).length() <= budget);
            if !fits(target) {
                // An accurate member can anchor a noisy one when the centroid
                // would unnecessarily displace the accurate endpoint.
                let mut anchors: Vec<usize> = (0..members.len()).collect();
                anchors.sort_by(|&a, &b| budgets[a].total_cmp(&budgets[b])
                    .then_with(|| ids[members[a]].cmp(&ids[members[b]])));
                let replacement = anchors.into_iter()
                    .map(|index| plane_snap(points[members[index]], &planes))
                    .find(|&candidate| fits(candidate));
                let Some(replacement) = replacement else { continue; };
                target = replacement;
            }
        }
        // Off-plane distance of the base from its incident planes.
        let off_plane = planes
            .iter()
            .map(|(nrm, off)| (nrm.dot(base) - off).abs())
            .fold(0.0_f64, f64::max);

        // Activation: fuse clusters always act; a lone vertex acts only if it is
        // measurably off its planes.  Below ACTIVATION_FLOOR the vertex is clean
        // and must be left byte-identical.
        if !is_fuse && off_plane <= ACTIVATION_FLOOR {
            continue;
        }

        for &m in members {
            // Never move a vertex further than heal_tol from where it started.
            if target.sub(points[m]).length() > heal_tol {
                // For a fuse cluster, fall back to the plain centroid (still
                // within heal_tol of every member by construction); for a lone
                // vertex, skip the plane snap rather than overshoot.
                if is_fuse && base.sub(points[m]).length() <= heal_tol {
                    if base.sub(points[m]).length() > 0.0 {
                        new_point[m] = base;
                        any_move = true;
                    }
                }
                continue;
            }
            if target.sub(points[m]).length() > 0.0 {
                new_point[m] = target;
                any_move = true;
            }
        }
    }

    if !any_move {
        return Ok(false);
    }

    // ----- commit moved vertex positions -----
    for (i, vertex) in solid.vertices.iter_mut().enumerate() {
        vertex.point = new_point[i];
    }

    // ----- re-anchor incident edge-curve endpoints onto the moved vertices --
    // Reuse the boolean's committed geometry mutator; a radius of heal_tol is
    // enough to pull the terminal control point onto the snapped vertex.
    crate::boolean::commit_nearby_edge_endpoints(solid, heal_tol)?;
    Ok(true)
}

/// Map each vertex id to the list of `(unit normal, offset)` planes of the
/// PLANAR faces incident to it (a coedge whose edge touches the vertex).
fn incident_planes(solid: &BrepSolid) -> HashMap<u64, Vec<(Vec3, f64)>> {
    let edge_vertices: HashMap<u64, (u64, u64)> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, (edge.start_vertex_id, edge.end_vertex_id)))
        .collect();
    let mut out: HashMap<u64, Vec<(Vec3, f64)>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let Some(AnalyticSurface::Plane {
            origin,
            u_dir,
            v_dir,
            ..
        }) = face.surface.analytic()
        else {
            continue;
        };
        let Ok(normal) = u_dir.cross(*v_dir).normalized() else {
            continue;
        };
        let offset = normal.dot(*origin);
        let mut touched: rustc_hash::FxHashSet<u64> = rustc_hash::FxHashSet::default();
        for loop_record in &face.loops {
            for coedge in &loop_record.coedges {
                if let Some(&(start, end)) = edge_vertices.get(&coedge.edge_id) {
                    touched.insert(start);
                    touched.insert(end);
                }
            }
        }
        for vertex_id in touched {
            out.entry(vertex_id).or_default().push((normal, offset));
        }
    }
    out
}

/// Map each vertex id to the length of its shortest incident non-degenerate
/// edge (chord length of the two endpoints), used to cap the fuse radius.
fn shortest_incident_edges(solid: &BrepSolid) -> HashMap<u64, f64> {
    let points: HashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    let mut out: HashMap<u64, f64> = HashMap::default();
    for edge in &solid.edges {
        if edge.degenerate || edge.start_vertex_id == edge.end_vertex_id {
            continue;
        }
        let (Some(&a), Some(&b)) = (
            points.get(&edge.start_vertex_id),
            points.get(&edge.end_vertex_id),
        ) else {
            continue;
        };
        let length = a.sub(b).length();
        for id in [edge.start_vertex_id, edge.end_vertex_id] {
            let slot = out.entry(id).or_insert(f64::INFINITY);
            if length < *slot {
                *slot = length;
            }
        }
    }
    out
}

/// Least-squares projection of `base` onto the common intersection of `planes`,
/// regularized toward `base` so under-determined directions keep the base
/// coordinate.  With no planes this returns `base`; with ≥3 independent normals
/// it returns their exact intersection.
fn plane_snap(base: Vec3, planes: &[(Vec3, f64)]) -> Vec3 {
    if planes.is_empty() {
        return base;
    }
    // Solve (Σ nnᵀ + εI) x = Σ d n + ε base.
    let mut matrix = [[0.0f64; 3]; 3];
    let mut rhs = [0.0f64; 3];
    for i in 0..3 {
        matrix[i][i] = PLANE_ANCHOR;
    }
    let base_arr = [base.x, base.y, base.z];
    for i in 0..3 {
        rhs[i] = PLANE_ANCHOR * base_arr[i];
    }
    for (normal, offset) in planes {
        let na = [normal.x, normal.y, normal.z];
        for i in 0..3 {
            for j in 0..3 {
                matrix[i][j] += na[i] * na[j];
            }
            rhs[i] += offset * na[i];
        }
    }
    match crate::fit::solve_small(matrix, rhs, 3) {
        Ok(solution) => Vec3::new(solution[0], solution[1], solution[2]),
        Err(_) => base,
    }
}

/// BRL-CAD-style oracle (diagnostic only, behind `BREP_DEBUG_BOOL`): after a
/// heal, report any vertex pair still within `heal_tol` that was not fused.
fn oracle_scan(solid: &BrepSolid, heal_tol: f64) {
    let vs = &solid.vertices;
    for i in 0..vs.len() {
        for j in (i + 1)..vs.len() {
            let gap = vs[i].point.sub(vs[j].point).length();
            if gap <= heal_tol {
                eprintln!(
                    "heal oracle: vertices {} and {} still within heal_tol ({:.3e})",
                    vs[i].id, vs[j].id, gap
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Inherited-trim refit (2026-10-04).
//
// An operand can arrive carrying a trim its own construction got wrong while
// its edge is right: `boolean_fuzz_corpus/20_helmet_merge_stage/a.json` (an
// import fitted 2026-08-07) carries two 195-control pcurves that image up to
// 4.1138e-3 off edges lying on BOTH their carriers to 3e-9, and the boolean
// carries one of them verbatim into its result. Validate at 2.5 % of the
// diagonal does not see it, so no issue-count gate can trigger a repair; the
// detection here is a band-free same-parameter reading against the existing
// pcurve construction contract ([`crate::PcurveFitReport::on_bar`]: the 1e-7
// floor plus the edge's own measured standoff from the carrier). Acceptance
// is separate and conservative: the operand's validate issues after the
// transaction must be a sub-multiset of those before it, and no closed
// shell's vector-area residual may read worse.
//
// Only pcurves change. Vertices, edge curves, ranges, carriers and senses are
// never written, and a coedge is in scope only when its unchanged edge lies on
// EVERY incident carrier within `policy.model`, read through the carriers'
// unclamped C1 extension. Fillet operands do not pass through here (the
// boolean calls it after `heal_operands`; `blending/fillet/tool.rs` does not),
// which is a stated gap, not an oversight.
// ---------------------------------------------------------------------------

/// Interior points per pcurve knot span the DETECTION read adds to validate's
/// own floor stations (the midpoint of every pcurve and edge-curve knot span).
const TRIM_REFIT_DETECT_INTERIOR: usize = 16;

/// Interior points per span the ACCEPTANCE re-read takes over the union of the
/// old pcurve's, the edge curve's and the new pcurve's knot spans. The fit's
/// own `on_bar` is quadratically blind to sideways error (its docstring), so
/// this read, not the fit's report, is what accepts a refit.
const TRIM_REFIT_REREAD_INTERIOR: usize = 64;

/// Interior points per span of the binding scope proof (the edge against
/// every incident carrier), over the union of the old pcurve's, the edge's
/// and — after the fit — the new pcurve's knot spans: as dense as the
/// acceptance re-read, so the proof is not the weaker of the two reads. (The
/// own-carrier standoff that only decides candidacy reads at the detection
/// density.)
const TRIM_REFIT_SCOPE_INTERIOR: usize = 64;

/// Why the refit left one coedge as it was.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TrimRefitDecline {
    /// A raw reading was non-finite or an evaluation failed: nothing about this
    /// coedge was measured, so nothing is claimed about it either way.
    Unreadable,
    /// The edge stands more than `policy.model` off one of its carriers: the
    /// trim is not the only thing wrong, and moving an edge is not this pass's.
    EdgeOffCarrier,
    /// A scope station's foot lies more than `policy.model` past a carrier's
    /// domain in a curved direction, where the extension is only the tangent
    /// continuation: no on-carrier claim can be made there.
    CurvedExtension,
    /// A scope station's foot lies past a domain corner (both directions):
    /// two extensions compound and neither is the carrier's own.
    CornerExtension,
    /// A scope station's foot lies on the straight line supporting a
    /// degree-1 ruling past the domain, but further than the carrier's own
    /// rational continuation of its end span reaches along that line (see
    /// [`native_reach`]): straight supporting geometry, not the carrier.
    BeyondNativeReach,
    /// The carrier is closed in u or v. No period-branch rule is implemented
    /// here, so the refit declines rather than guess a branch.
    ClosedCarrier,
    /// The old trim's image at fraction 0 or 1 is already over the bar from the
    /// edge's end: keeping the loop endpoints would keep an error.
    OldEndWrong,
    /// The construction returned an error, or a trim whose ends cannot be
    /// pinned (not clamped).
    RefitFailed,
    /// The construction did not reach its own bar.
    RefitOffItsBar,
    /// The dense re-read of the new trim is over the bar.
    RereadOverBar,
    /// The new trim's image at fraction 0 or 1 is further than the bar from
    /// the old trim's image there: the loop endpoint would move.
    EndMoved,
    /// A loop joint's uv gap reads larger after than before (strictly: no
    /// allowance is added, so a refit that moves a shared end by any amount
    /// away from its neighbour declines here and is recorded).
    UvJointWorsened,
    /// A loop joint's 3D gap reads larger after than before (strictly).
    JointWorsened,
    /// Another use of the same edge is a candidate that was not refit: an edge
    /// is repaired whole or not at all. Repairing one side of a pair of trims
    /// that are wrong together can break what the pair agreed on (measured
    /// 2026-10-04: `offset_subtract_first.json` coedge 981 refit alone, its
    /// mate 890 declined OldEndWrong, and the subtract then refused with three
    /// once-used edges).
    MateUnrepaired,
}

/// Why the whole transaction was put back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TrimRefitRollback {
    /// Validate reported an issue after the transaction that it did not report
    /// before (compared as a multiset of severity, kind and message).
    NewOrChangedIssue,
    /// A closed shell's vector-area residual read worse after than before.
    VectorAreaWorsened,
    /// The vector-area scan could not read after what it read before: a new
    /// unreadable row, a measured shell now unmeasured or absent, or a
    /// non-finite residual.
    VectorAreaUnreadable,
    /// A shell's reading changed kind (closed ↔ open), or a shell reads after
    /// that was not read before.
    VectorAreaReadingChanged,
    /// A shell's closure was not measured before or after (the scan reached
    /// its budget): nothing certifies the transaction did not worsen it, and
    /// an unchanged "unmeasured" is not evidence.
    VectorAreaUnmeasured,
}

/// What happened to one coedge the detection read as off its contract (or
/// could not read).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TrimRefitDecision {
    Refit,
    Declined(TrimRefitDecline),
    RolledBack(TrimRefitRollback),
}

/// One coedge's row in a [`TrimRefitReport`].
#[derive(Clone, Debug)]
pub(crate) struct TrimRefitEntry {
    pub face: u64,
    pub coedge: u64,
    pub edge: u64,
    /// The detection read: worst same-parameter image-to-edge distance, NaN
    /// when unreadable.
    pub miss: f64,
    /// The edge's standoff from each incident carrier `(face id, distance)`,
    /// empty when the scope read was not reached.
    pub standoff: Vec<(u64, f64)>,
    /// The acceptance re-read of the new trim, when one was built.
    pub reread: Option<f64>,
    /// Scope stations (over every carrier read) whose winning foot lay
    /// outside the carrier's domain.
    pub outside_domain: usize,
    /// Of those, the stations read through a carrier point because their
    /// extension was not proved to be the carrier's own.
    pub witnessed: usize,
    /// [`pcurve_bits`] of the trim before, and of the replacement if built.
    pub old_bits: u64,
    pub new_bits: Option<u64>,
    pub decision: TrimRefitDecision,
}

/// Every coedge [`refit_inherited_trims`] read as a candidate, and the count it
/// read in all. An operand with no entry was left byte-identical.
#[derive(Clone, Debug, Default)]
pub(crate) struct TrimRefitReport {
    pub coedges_read: usize,
    pub entries: Vec<TrimRefitEntry>,
    pub seconds: TrimRefitSeconds,
}

/// Where a [`refit_inherited_trims`] call spent its time, for the census.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct TrimRefitSeconds {
    /// The detection read over every coedge.
    pub detect: f64,
    /// Every scope read: the own-carrier candidacy standoff and the proofs.
    pub scope: f64,
    /// The construction, the end pinning, the acceptance re-read and joints.
    pub construct: f64,
    /// The whole-transaction gate.
    pub gate: f64,
}

/// PROFILING ONLY (`BREP_DEBUG_TRIM_REFIT_PROFILE`, or a control's
/// `with_scope_profile`): what the scope reads did, by phase. Off, the
/// thread-local holds `None` and every hook is a no-op; on, the hooks only
/// count and time — no seed, iteration, order or output changes (the sampled
/// hint-only comparison runs on the side and its result is never used). Its
/// own switch, apart from the `BREP_DEBUG_TRIM_REFIT` trace, so a trace-only
/// run's `TRIM-REFIT-SECONDS` carries none of the profile's cost (the key set,
/// the clocks and the side Newton). `projector_calls` counts the scope path's
/// projector calls only, not the construction's.
#[derive(Clone, Debug, Default)]
pub(crate) struct ScopeProfile {
    pub candidacy_feet: usize,
    pub old_claim_feet: usize,
    pub final_claim_feet: usize,
    pub projector_calls: usize,
    pub projector_seconds: f64,
    pub newton_iterations: usize,
    pub grid_fallbacks: usize,
    pub witness_evaluations: usize,
    /// Final claims taken whole from the old claim (identical station
    /// vectors), and final claims read again.
    pub final_claims_reused: usize,
    pub final_claims_read: usize,
    /// Feet rebuilt from a recorded foot (same carrier, fraction and incoming
    /// hint bits) instead of searched.
    pub replayed_feet: usize,
    /// Feet per phase whose key was already read earlier in the refit:
    /// [candidacy, old claim, final claim].
    pub repeated_feet: [usize; 3],
    /// One in every [`SCOPE_PROFILE_SAMPLE`] hinted feet is re-read from the
    /// hint alone: agreement within 1e-12, or which side is smaller.
    pub sampled: usize,
    pub hint_agrees: usize,
    pub hint_smaller: usize,
    pub hint_larger: usize,
    pub hint_unreadable: usize,
    /// Distinct `(carrier, edge, edge parameter bits)` keys over all feet:
    /// the exact reuse a memo could have had.
    keys: rustc_hash::FxHashSet<(usize, u64, u64)>,
    phase: ScopePhase,
    sample_clock: usize,
}

const SCOPE_PROFILE_SAMPLE: usize = 64;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ScopePhase {
    #[default]
    Candidacy,
    OldClaim,
    FinalClaim,
}

impl ScopeProfile {
    fn summary(&self) -> String {
        format!(
            "final claims reused {} read {}; replayed feet {}; repeated feet {:?}; feet candidacy {} old-claim {} final-claim {} (distinct keys {}); scope projector calls {} in {:.3}s; newton iterations {}; grid fallbacks {}; witnesses {}; hint-only sample {}: agrees {} smaller {} larger {} unreadable {}",
            self.final_claims_reused, self.final_claims_read, self.replayed_feet, self.repeated_feet,
            self.candidacy_feet, self.old_claim_feet, self.final_claim_feet, self.keys.len(),
            self.projector_calls, self.projector_seconds, self.newton_iterations, self.grid_fallbacks,
            self.witness_evaluations, self.sampled, self.hint_agrees, self.hint_smaller, self.hint_larger,
            self.hint_unreadable
        )
    }
}

thread_local! {
    static SCOPE_PROFILE: std::cell::RefCell<Option<ScopeProfile>> = const { std::cell::RefCell::new(None) };
}

/// Run `hook` on the open profile, if any. A no-op when profiling is off.
fn profiling(hook: impl FnOnce(&mut ScopeProfile)) {
    SCOPE_PROFILE.with(|cell| {
        if let Some(profile) = cell.borrow_mut().as_mut() {
            hook(profile);
        }
    });
}

fn profiling_on() -> bool {
    SCOPE_PROFILE.with(|cell| cell.borrow().is_some())
}

/// `BREP_DEBUG_TRIM_REFIT_PROFILE` set (any value): open the scope profile.
fn trim_refit_profiling() -> bool {
    std::env::var_os("BREP_DEBUG_TRIM_REFIT_PROFILE").is_some()
}

/// Bit-exact identity of a pcurve: degree, knot bits and control-point bits.
/// For the first-writer trace and the controls; never a decision input.
pub(crate) fn pcurve_bits(curve: &crate::NurbsCurve) -> u64 {
    use std::hash::Hasher;
    let mut hasher = rustc_hash::FxHasher::default();
    hasher.write_usize(curve.degree);
    for knot in &curve.knots {
        hasher.write_u64(knot.to_bits());
    }
    for point in &curve.control_points {
        for value in [point.x, point.y, point.z, point.w] {
            hasher.write_u64(value.to_bits());
        }
    }
    hasher.finish()
}

/// `BREP_DEBUG_TRIM_REFIT`: one line per candidate; `=all` adds the entry
/// identity ([`pcurve_bits`]) of every coedge read. Debug only.
fn trim_refit_tracing() -> Option<bool> {
    std::env::var("BREP_DEBUG_TRIM_REFIT").ok().map(|value| value == "all")
}

/// Refit, on the operand's own carriers, the trims an operand brings in off
/// the pcurve construction contract while their edges lie on every incident
/// carrier. Called by the boolean after [`heal_operands`], so it reads the
/// post-fuse body. A solid with no candidate is returned untouched.
pub(crate) fn refit_inherited_trims(
    solid: &mut BrepSolid,
    policy: &KernelTolerances,
) -> Result<TrimRefitReport, KernelRefusal> {
    // DIAGNOSTIC hatch, never a production setting: `BREP_DEBUG_TRIM_REFIT_OFF`
    // skips the pass entirely, so a test can be read with and without it on
    // ONE binary (attribution only).
    if std::env::var_os("BREP_DEBUG_TRIM_REFIT_OFF").is_some() {
        return Ok(TrimRefitReport::default());
    }
    refit_inherited_trims_with(solid, policy, |before, after| trim_refit_gate(before, after, policy))
}

/// [`refit_inherited_trims`] with the whole-transaction gate supplied, so a
/// control can show that a refused transaction restores every trim.
fn refit_inherited_trims_with(
    solid: &mut BrepSolid,
    policy: &KernelTolerances,
    mut gate: impl FnMut(&BrepSolid, &BrepSolid) -> Option<TrimRefitRollback>,
) -> Result<TrimRefitReport, KernelRefusal> {
    let trace = trim_refit_tracing();
    // Profiling opens here under its own switch unless a control opened it.
    let owns_profile = trim_refit_profiling() && !profiling_on();
    if owns_profile {
        SCOPE_PROFILE.with(|cell| *cell.borrow_mut() = Some(ScopeProfile::default()));
    }
    let floor = crate::PCURVE_REFINEMENT_TOLERANCE;
    let edges: HashMap<u64, &crate::topology::EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    // Every use of each edge, by index: its carriers and its old trims are
    // what the edge's scope claim reads.
    let mut uses_of: HashMap<u64, Vec<CoedgeIndex>> = HashMap::default();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            for (loop_index, lp) in face.loops.iter().enumerate() {
                for (coedge_index, coedge) in lp.coedges.iter().enumerate() {
                    uses_of.entry(coedge.edge_id).or_default().push((shell_index, face_index, loop_index, coedge_index));
                }
            }
        }
    }

    let mut report = TrimRefitReport::default();
    // One scope claim per edge, read once over the union of the edge's knots
    // and every use's OLD trim, so opposite uses share one verdict.
    let mut claims: HashMap<u64, (EdgeClaim, Option<TrimRefitDecline>)> = HashMap::default();
    // The old claims' feet, replayed by the final claims ([`edge_standoff_replay`]);
    // off under `BREP_DEBUG_TRIM_REFIT_NO_REPLAY` (a same-binary A/B).
    let replay_on = std::env::var_os("BREP_DEBUG_TRIM_REFIT_NO_REPLAY").is_none();
    let mut records: HashMap<u64, ClaimRecord> = HashMap::default();
    // (use, replacement, entry index)
    let mut replacements: Vec<(CoedgeIndex, crate::NurbsCurve, usize)> = Vec::new();
    let mut working: Option<BrepSolid> = None;
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            for (loop_index, lp) in face.loops.iter().enumerate() {
                for (coedge_index, coedge) in lp.coedges.iter().enumerate() {
                    report.coedges_read += 1;
                    let Some(&edge) = edges.get(&coedge.edge_id) else {
                        // A coedge naming no edge cannot be read; say so.
                        report.entries.push(TrimRefitEntry {
                            face: face.id,
                            coedge: coedge.id,
                            edge: coedge.edge_id,
                            miss: f64::NAN,
                            standoff: Vec::new(),
                            reread: None,
                            outside_domain: 0,
                            witnessed: 0,
                            old_bits: pcurve_bits(&coedge.pcurve),
                            new_bits: None,
                            decision: TrimRefitDecision::Declined(TrimRefitDecline::Unreadable),
                        });
                        continue;
                    };
                    if edge.degenerate {
                        continue;
                    }
                    let old_bits = pcurve_bits(&coedge.pcurve);
                    if trace == Some(true) {
                        eprintln!(
                            "TRIM-REFIT-ENTRY face {} coedge {} edge {} bits {old_bits:016x}",
                            face.id, coedge.id, edge.id
                        );
                    }
                    let mut entry = TrimRefitEntry {
                        face: face.id,
                        coedge: coedge.id,
                        edge: edge.id,
                        miss: f64::NAN,
                        standoff: Vec::new(),
                        reread: None,
                        outside_domain: 0,
                        witnessed: 0,
                        old_bits,
                        new_bits: None,
                        decision: TrimRefitDecision::Declined(TrimRefitDecline::Unreadable),
                    };
                    let started = web_time::Instant::now();
                    let miss = detection_stations(&coedge.pcurve, edge, coedge.forward)
                        .and_then(|stations| worst_same_parameter(&face.surface, &coedge.pcurve, edge, coedge.forward, &stations));
                    report.seconds.detect += started.elapsed().as_secs_f64();
                    let Some(miss) = miss else {
                        report.entries.push(entry);
                        continue;
                    };
                    entry.miss = miss;
                    if miss <= floor {
                        continue;
                    }
                    // The standoff on this face's own carrier decides
                    // candidacy only (the binding proof is the edge claim).
                    profiling(|profile| profile.phase = ScopePhase::Candidacy);
                    let started = web_time::Instant::now();
                    let own = edge_standoff(
                        &face.surface,
                        edge,
                        coedge.forward,
                        scope_stations(&[&coedge.pcurve], edge, coedge.forward, TRIM_REFIT_DETECT_INTERIOR).as_deref(),
                        policy.model,
                    );
                    report.seconds.scope += started.elapsed().as_secs_f64();
                    let Some(own) = own.map(|read| read.worst) else {
                        report.entries.push(entry);
                        continue;
                    };
                    if miss <= floor + own {
                        continue;
                    }
                    // A candidate on a CLOSED carrier declines here, before its
                    // edge's claim is read, exactly as `refit_one` declines it
                    // (an unreadable closedness is Unreadable, as there). The
                    // claim is read lazily by the edge's first candidate that
                    // is not so declined, with the same inputs, so an edge
                    // whose every candidate is closed reads no claim and every
                    // other reading is unchanged.
                    match face.surface.closed_directions() {
                        Ok((closed_u, closed_v)) if closed_u || closed_v => {
                            entry.decision = TrimRefitDecision::Declined(TrimRefitDecline::ClosedCarrier);
                            report.entries.push(entry);
                            continue;
                        }
                        Err(_) => {
                            entry.decision = TrimRefitDecision::Declined(TrimRefitDecline::Unreadable);
                            report.entries.push(entry);
                            continue;
                        }
                        Ok(_) => {}
                    }
                    // A candidate. Its edge's claim, read once per edge over
                    // every use's old trim.
                    let uses = uses_of.get(&edge.id).map(Vec::as_slice).unwrap_or(&[]);
                    if !claims.contains_key(&edge.id) {
                        profiling(|profile| profile.phase = ScopePhase::OldClaim);
                        let started = web_time::Instant::now();
                        let old_trims = trims_of(solid, uses);
                        let (claim, record) = edge_claim_with(solid, uses, edge, &old_trims, policy.model, None, replay_on);
                        report.seconds.scope += started.elapsed().as_secs_f64();
                        claims.insert(edge.id, claim);
                        if replay_on {
                            records.insert(edge.id, record);
                        }
                    }
                    let claim = &claims[&edge.id];
                    // The working copy holds the trims refit so far, so a
                    // neighbour refit earlier is the neighbour read.
                    let current = working.get_or_insert_with(|| solid.clone());
                    let at = (shell_index, face_index, loop_index, coedge_index);
                    match refit_one(current, at, edge, claim, policy, &mut entry, &mut report.seconds) {
                        Ok(pcurve) => {
                            entry.new_bits = Some(pcurve_bits(&pcurve));
                            entry.decision = TrimRefitDecision::Refit;
                            current.shells[shell_index].faces[face_index].loops[loop_index].coedges[coedge_index].pcurve = pcurve.clone();
                            replacements.push((at, pcurve, report.entries.len()));
                        }
                        Err(decline) => entry.decision = TrimRefitDecision::Declined(decline),
                    }
                    report.entries.push(entry);
                }
            }
        }
    }

    // The binding claim again, per refit EDGE, over the union of the edge's
    // knots, every use's old trim and every use's new trim: one station set
    // for the edge whichever use asks. A failing edge declines every refit on
    // it and restores those trims in the working copy.
    profiling(|profile| profile.phase = ScopePhase::FinalClaim);
    let started = web_time::Instant::now();
    let mut refit_edges: Vec<u64> = replacements.iter().map(|(_, _, index)| report.entries[*index].edge).collect();
    refit_edges.sort_unstable();
    refit_edges.dedup();
    records.retain(|edge, _| refit_edges.binary_search(edge).is_ok());
    for edge_id in refit_edges {
        let Some(&edge) = edges.get(&edge_id) else { continue };
        let uses = uses_of.get(&edge_id).map(Vec::as_slice).unwrap_or(&[]);
        let (claim, decline) = {
            let old_trims = trims_of(solid, uses);
            let mut trims = old_trims.clone();
            for ((s, f, l, c), pcurve, index) in &replacements {
                if report.entries[*index].edge == edge_id {
                    trims.push((pcurve, solid.shells[*s].faces[*f].loops[*l].coedges[*c].forward));
                }
            }
            final_claim(solid, uses, edge, &old_trims, &trims, claims.get(&edge_id), records.get(&edge_id), policy.model).0
        };
        let on_edge: Vec<(CoedgeIndex, usize)> = replacements
            .iter()
            .filter(|(_, _, index)| report.entries[*index].edge == edge_id)
            .map(|(at, _, index)| (*at, *index))
            .collect();
        for &((s, f, l, c), index) in &on_edge {
            report.entries[index].outside_domain += claim.outside;
            report.entries[index].witnessed += claim.witnessed;
            if let Some(decline) = decline {
                report.entries[index].decision = TrimRefitDecision::Declined(decline);
                if let Some(current) = working.as_mut() {
                    current.shells[s].faces[f].loops[l].coedges[c].pcurve = solid.shells[s].faces[f].loops[l].coedges[c].pcurve.clone();
                }
            }
        }
    }
    report.seconds.scope += started.elapsed().as_secs_f64();
    if let Some(current) = working.as_mut() {
        settle_refits(solid, current, &replacements, &mut report.entries);
    }
    replacements.retain(|(_, _, index)| report.entries[*index].decision == TrimRefitDecision::Refit);

    drop(edges);
    if let Some(after) = working.filter(|_| !replacements.is_empty()) {
        let started = web_time::Instant::now();
        let verdict = gate(&*solid, &after);
        report.seconds.gate += started.elapsed().as_secs_f64();
        match verdict {
            None => {
                for ((s, f, l, c), pcurve, _) in replacements {
                    solid.shells[s].faces[f].loops[l].coedges[c].pcurve = pcurve;
                }
            }
            Some(rollback) => {
                for (.., entry_index) in replacements {
                    report.entries[entry_index].decision = TrimRefitDecision::RolledBack(rollback);
                }
            }
        }
    }
    if trace.is_some() {
        for entry in &report.entries {
            eprintln!(
                "TRIM-REFIT face {} coedge {} edge {} miss {:.4e} standoff {:?} outside {} witnessed {} reread {:?} bits {:016x} -> {:?} {:?}",
                entry.face, entry.coedge, entry.edge, entry.miss, entry.standoff, entry.outside_domain, entry.witnessed, entry.reread,
                entry.old_bits, entry.new_bits.map(|bits| format!("{bits:016x}")), entry.decision
            );
        }
        eprintln!("TRIM-REFIT-SECONDS read {} {:?}", report.coedges_read, report.seconds);
    }
    if owns_profile {
        if let Some(profile) = SCOPE_PROFILE.with(|cell| cell.borrow_mut().take()) {
            eprintln!("TRIM-REFIT-SCOPE-PROFILE {}", profile.summary());
        }
    }
    Ok(report)
}

/// A coedge's position: (shell, face, loop, coedge) indices.
type CoedgeIndex = (usize, usize, usize, usize);

/// The old trims of `uses` with each use's sense.
fn trims_of<'a>(solid: &'a BrepSolid, uses: &[CoedgeIndex]) -> Vec<(&'a crate::NurbsCurve, bool)> {
    uses.iter()
        .map(|&(s, f, l, c)| {
            let coedge = &solid.shells[s].faces[f].loops[l].coedges[c];
            (&coedge.pcurve, coedge.forward)
        })
        .collect()
}

/// What an edge's scope claim read: each incident carrier's standoff (by
/// face id, once per face) and the stations whose foot lay outside a domain.
#[derive(Clone, Debug, Default)]
struct EdgeClaim {
    standoff: Vec<(u64, f64)>,
    outside: usize,
    /// Stations read through a carrier witness, over every carrier.
    witnessed: usize,
}

/// The binding scope claim for one EDGE: its unchanged curve against every
/// incident carrier over ONE station set — the edge's own knots and every
/// given trim's knots, each trim's fractions turned to the edge's forward
/// sense, at [`TRIM_REFIT_SCOPE_INTERIOR`] points per span. Every use of the
/// edge is judged by the same claim, so opposite uses cannot contradict.
/// Returns what was read, and the decline if the claim fails.
/// A fresh claim with nothing replayed or recorded (the controls' reference).
#[cfg_attr(not(test), allow(dead_code))]
fn edge_claim(
    solid: &BrepSolid,
    uses: &[CoedgeIndex],
    edge: &crate::topology::EdgeRecord,
    trims: &[(&crate::NurbsCurve, bool)],
    model: f64,
) -> (EdgeClaim, Option<TrimRefitDecline>) {
    edge_claim_with(solid, uses, edge, trims, model, None, false).0
}

/// [`edge_claim`], replaying the feet of `replay` (an earlier claim of the
/// same edge on the same solid; see [`edge_standoff_replay`]) and, when
/// `record`, returning this claim's own feet per carrier.
fn edge_claim_with(
    solid: &BrepSolid,
    uses: &[CoedgeIndex],
    edge: &crate::topology::EdgeRecord,
    trims: &[(&crate::NurbsCurve, bool)],
    model: f64,
    replay: Option<&ClaimRecord>,
    record: bool,
) -> ((EdgeClaim, Option<TrimRefitDecline>), ClaimRecord) {
    let mut records = ClaimRecord::new();
    let mut claim = EdgeClaim::default();
    let Some(stations) = edge_claim_stations(edge, trims) else {
        return ((claim, Some(TrimRefitDecline::Unreadable)), records);
    };
    let mut decline = None;
    let mut faces: Vec<(usize, usize)> = uses.iter().map(|&(s, f, _, _)| (s, f)).collect();
    faces.sort_unstable();
    faces.dedup();
    for (s, f) in faces {
        let carrier = &solid.shells[s].faces[f];
        let lookup = replay.and_then(|replay| replay.iter().find(|(at, _)| *at == (s, f))).map(|(_, feet)| station_replay(feet));
        let mut own = record.then(StationRecord::new);
        let read = edge_standoff_replay(&carrier.surface, edge, true, Some(stations.as_slice()), model, lookup.as_ref(), own.as_mut());
        if let Some(own) = own {
            records.push(((s, f), own));
        }
        let Some(read) = read else {
            return ((claim, Some(TrimRefitDecline::Unreadable)), records);
        };
        claim.standoff.push((carrier.id, read.worst));
        claim.outside += read.outside;
        claim.witnessed += read.witnessed;
        if decline.is_none() {
            decline = match read.extension {
                Some(kind) => Some(kind),
                None if read.worst > model => Some(TrimRefitDecline::EdgeOffCarrier),
                None => None,
            };
        }
    }
    ((claim, decline), records)
}

/// The edge claim's stations, as forward fractions of the edge: the union of
/// the edge's knot breaks and every trim's knot breaks (a reversed use's
/// fraction `f` is the edge's `1 − f`), at [`TRIM_REFIT_SCOPE_INTERIOR`]
/// points plus the midpoint and ends of every span. `None` if anything is
/// unreadable.
fn edge_claim_stations(edge: &crate::topology::EdgeRecord, trims: &[(&crate::NurbsCurve, bool)]) -> Option<Vec<f64>> {
    let mut sets = vec![edge_breaks(edge, true)?];
    for &(trim, forward) in trims {
        let breaks = pcurve_breaks(trim)?;
        sets.push(if forward { breaks } else { breaks.into_iter().map(|fraction| 1.0 - fraction).collect() });
    }
    Some(span_stations(&merged_breaks(&sets)?, TRIM_REFIT_SCOPE_INTERIOR))
}

/// The whole-transaction rules every refit must still meet once the set of
/// refits is known, applied until nothing changes (dropping one refit can
/// break another):
/// * an edge is repaired whole or not at all: a refit on an edge with another
///   candidate use that is not refit declines [`TrimRefitDecline::MateUnrepaired`];
/// * every loop joint of a refit trim, read in the FINAL working copy, is no
///   worse in uv and in 3D than the same joint of the original solid (the
///   per-candidate check read its neighbours as they stood then, and a
///   neighbour may since have been restored).
///
/// A declined refit's trim is restored in `working` from `solid`.
fn settle_refits(
    solid: &BrepSolid,
    working: &mut BrepSolid,
    replacements: &[(CoedgeIndex, crate::NurbsCurve, usize)],
    entries: &mut [TrimRefitEntry],
) {
    loop {
        let unrepaired: rustc_hash::FxHashSet<u64> = entries
            .iter()
            .filter(|entry| entry.decision != TrimRefitDecision::Refit)
            .map(|entry| entry.edge)
            .collect();
        let mut changed = false;
        for &((s, f, l, c), _, index) in replacements {
            if entries[index].decision != TrimRefitDecision::Refit {
                continue;
            }
            let decline = if unrepaired.contains(&entries[index].edge) {
                Some(TrimRefitDecline::MateUnrepaired)
            } else {
                joints_not_worse(solid, working, (s, f, l, c)).err()
            };
            if let Some(decline) = decline {
                entries[index].decision = TrimRefitDecision::Declined(decline);
                working.shells[s].faces[f].loops[l].coedges[c].pcurve = solid.shells[s].faces[f].loops[l].coedges[c].pcurve.clone();
                changed = true;
            }
        }
        if !changed {
            return;
        }
    }
}

/// Each loop joint of the coedge at `at`, in uv and in 3D on its carrier:
/// the gap in `working` may not exceed the gap in `solid` (both finite). A
/// one-coedge loop's joint is its own two ends.
fn joints_not_worse(solid: &BrepSolid, working: &BrepSolid, (s, f, l, c): CoedgeIndex) -> Result<(), TrimRefitDecline> {
    let surface = &solid.shells[s].faces[f].surface;
    let (old_loop, new_loop) = (&solid.shells[s].faces[f].loops[l], &working.shells[s].faces[f].loops[l]);
    let count = old_loop.coedges.len();
    // (the meeting trim's index, its fraction, this trim's fraction)
    let joints: Vec<(usize, f64, f64)> =
        if count == 1 { vec![(c, 1.0, 0.0)] } else { vec![((c + count - 1) % count, 1.0, 0.0), ((c + 1) % count, 0.0, 1.0)] };
    let gap = |a: [f64; 2], b: [f64; 2]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    let point = |uv: [f64; 2]| surface.evaluate_extended(uv[0], uv[1]).map_err(|_| TrimRefitDecline::Unreadable);
    for (theirs, their_fraction, own_fraction) in joints {
        let ends = |lp: &crate::topology::LoopRecord| -> Result<([f64; 2], [f64; 2]), TrimRefitDecline> {
            Ok((
                uv_at(&lp.coedges[theirs].pcurve, their_fraction).ok_or(TrimRefitDecline::Unreadable)?,
                uv_at(&lp.coedges[c].pcurve, own_fraction).ok_or(TrimRefitDecline::Unreadable)?,
            ))
        };
        let ((old_theirs, old_own), (new_theirs, new_own)) = (ends(old_loop)?, ends(new_loop)?);
        let (before, after) = (gap(old_theirs, old_own), gap(new_theirs, new_own));
        if !(before.is_finite() && after.is_finite()) {
            return Err(TrimRefitDecline::Unreadable);
        }
        if after > before {
            return Err(TrimRefitDecline::UvJointWorsened);
        }
        let (before, after) = (point(old_theirs)?.sub(point(old_own)?).length(), point(new_theirs)?.sub(point(new_own)?).length());
        if !(before.is_finite() && after.is_finite()) {
            return Err(TrimRefitDecline::Unreadable);
        }
        if after > before {
            return Err(TrimRefitDecline::JointWorsened);
        }
    }
    Ok(())
}

/// The final (binding) claim of a refit edge over `trims` (its old trims plus
/// the new ones). The trims enter a claim only through its station set; the
/// solid, the uses, the edge and the model are the old claim's. So when the
/// new trims add no break, the two station vectors are equal bit for bit, the
/// whole ordered read (stations, and with them every incoming hint) is the old
/// claim's, and `old` IS this claim. It is read again only when they differ,
/// replaying from `record` (the old claim's feet) every station whose fraction
/// and incoming hint are the old read's bit for bit ([`edge_standoff_replay`]).
/// Returns the claim and whether `old` was taken.
fn final_claim(
    solid: &BrepSolid,
    uses: &[CoedgeIndex],
    edge: &crate::topology::EdgeRecord,
    old_trims: &[(&crate::NurbsCurve, bool)],
    trims: &[(&crate::NurbsCurve, bool)],
    old: Option<&(EdgeClaim, Option<TrimRefitDecline>)>,
    record: Option<&ClaimRecord>,
    model: f64,
) -> ((EdgeClaim, Option<TrimRefitDecline>), bool) {
    let same_stations = match (edge_claim_stations(edge, old_trims), edge_claim_stations(edge, trims)) {
        (Some(old), Some(new)) => old.len() == new.len() && old.iter().zip(&new).all(|(a, b)| a.to_bits() == b.to_bits()),
        _ => false,
    };
    match old.filter(|_| same_stations) {
        Some(old) => {
            profiling(|profile| profile.final_claims_reused += 1);
            (old.clone(), true)
        }
        None => {
            profiling(|profile| profile.final_claims_read += 1);
            (edge_claim_with(solid, uses, edge, trims, model, record, false).0, false)
        }
    }
}

/// Construct and independently re-read one candidate on `solid` (the working
/// copy), under its edge's `claim`. Returns the replacement trim or the
/// reason it declined.
fn refit_one(
    solid: &BrepSolid,
    (shell_index, face_index, loop_index, coedge_index): CoedgeIndex,
    edge: &crate::topology::EdgeRecord,
    (claim, claim_decline): &(EdgeClaim, Option<TrimRefitDecline>),
    policy: &KernelTolerances,
    entry: &mut TrimRefitEntry,
    seconds: &mut TrimRefitSeconds,
) -> Result<crate::NurbsCurve, TrimRefitDecline> {
    let floor = crate::PCURVE_REFINEMENT_TOLERANCE;
    let face = &solid.shells[shell_index].faces[face_index];
    let lp = &face.loops[loop_index];
    let coedge = &lp.coedges[coedge_index];
    let surface = &face.surface;
    let (closed_u, closed_v) = surface.closed_directions().map_err(|_| TrimRefitDecline::Unreadable)?;
    if closed_u || closed_v {
        return Err(TrimRefitDecline::ClosedCarrier);
    }

    // Scope: the edge's claim, shared by every use of the edge.
    entry.standoff = claim.standoff.clone();
    entry.outside_domain += claim.outside;
    entry.witnessed += claim.witnessed;
    if let Some(decline) = claim_decline {
        return Err(*decline);
    }
    if claim.standoff.iter().any(|&(_, standoff)| !(standoff <= policy.model)) {
        return Err(TrimRefitDecline::EdgeOffCarrier);
    }
    let own = claim
        .standoff
        .iter()
        .find(|&&(id, _)| id == face.id)
        .map(|&(_, standoff)| standoff)
        .ok_or(TrimRefitDecline::Unreadable)?;
    let bar = floor + own;

    // The loop endpoints stay where they are only if they are right now.
    let old_start = coedge_points(surface, &coedge.pcurve, edge, coedge.forward, 0.0).ok_or(TrimRefitDecline::Unreadable)?;
    let old_end = coedge_points(surface, &coedge.pcurve, edge, coedge.forward, 1.0).ok_or(TrimRefitDecline::Unreadable)?;
    if old_start.0.sub(old_start.1).length() > bar || old_end.0.sub(old_end.1).length() > bar {
        return Err(TrimRefitDecline::OldEndWrong);
    }

    // Construction, on the existing contract.
    let started = web_time::Instant::now();
    let result = construct_refit(surface, coedge, edge, floor).and_then(|pcurve| {
        accept_refit(surface, lp, coedge, coedge_index, edge, pcurve, bar, (old_start, old_end), entry)
    });
    seconds.construct += started.elapsed().as_secs_f64();
    result
}

/// The fit on the existing contract, with its ends pinned to the old trim's.
fn construct_refit(
    surface: &crate::NurbsSurface,
    coedge: &crate::topology::CoedgeRecord,
    edge: &crate::topology::EdgeRecord,
    floor: f64,
) -> Result<crate::NurbsCurve, TrimRefitDecline> {
    let fit = crate::fit_pcurve_on_surface_range(surface, &edge.curve, edge.t0, edge.t1, coedge.forward, floor)
        .map_err(|_| TrimRefitDecline::RefitFailed)?;
    if !fit.report.on_bar() {
        return Err(TrimRefitDecline::RefitOffItsBar);
    }
    // Keep the loop endpoints exactly: a clamped trim interpolates its end
    // control points, so they take the old trim's end uv (homogeneous, the
    // weight kept). The move is the fit's own end error, and the re-read
    // judges the curve that results, not the fit.
    let mut pcurve = fit.curve;
    let degree = pcurve.degree;
    let knots = &pcurve.knots;
    let clamped = knots.len() >= 2 * (degree + 1)
        && knots[..=degree].iter().all(|&knot| knot == knots[0])
        && knots[knots.len() - 1 - degree..].iter().all(|&knot| knot == knots[knots.len() - 1]);
    if !clamped || pcurve.control_points.len() < 2 {
        return Err(TrimRefitDecline::RefitFailed);
    }
    let old_start_uv = uv_at(&coedge.pcurve, 0.0).ok_or(TrimRefitDecline::Unreadable)?;
    let old_end_uv = uv_at(&coedge.pcurve, 1.0).ok_or(TrimRefitDecline::Unreadable)?;
    let last = pcurve.control_points.len() - 1;
    for (index, uv) in [(0, old_start_uv), (last, old_end_uv)] {
        let point = &mut pcurve.control_points[index];
        point.x = uv[0] * point.w;
        point.y = uv[1] * point.w;
    }
    Ok(pcurve)
}

/// The acceptance half of [`refit_one`]: the dense re-read, the measured end
/// equality and strict no-worsening at every loop joint.
#[allow(clippy::too_many_arguments)]
fn accept_refit(
    surface: &crate::NurbsSurface,
    lp: &crate::topology::LoopRecord,
    coedge: &crate::topology::CoedgeRecord,
    coedge_index: usize,
    edge: &crate::topology::EdgeRecord,
    pcurve: crate::NurbsCurve,
    bar: f64,
    (old_start, old_end): ((Vec3, Vec3), (Vec3, Vec3)),
    entry: &mut TrimRefitEntry,
) -> Result<crate::NurbsCurve, TrimRefitDecline> {

    // The independent acceptance read: every span of the old trim, the edge
    // and the new trim, at the dense stencil, finite before any max.
    let stations = reread_stations(&coedge.pcurve, &pcurve, edge, coedge.forward).ok_or(TrimRefitDecline::Unreadable)?;
    let reread = worst_same_parameter(surface, &pcurve, edge, coedge.forward, &stations).ok_or(TrimRefitDecline::Unreadable)?;
    entry.reread = Some(reread);
    if reread > bar {
        return Err(TrimRefitDecline::RereadOverBar);
    }

    // The loop endpoints are kept by measured equality within the contract:
    // the new trim's image at each end against the old trim's image there.
    let new_start = coedge_points(surface, &pcurve, edge, coedge.forward, 0.0).ok_or(TrimRefitDecline::Unreadable)?;
    let new_end = coedge_points(surface, &pcurve, edge, coedge.forward, 1.0).ok_or(TrimRefitDecline::Unreadable)?;
    let start_moved = new_start.0.sub(old_start.0).length();
    let end_moved = new_end.0.sub(old_end.0).length();
    if !(start_moved.is_finite() && end_moved.is_finite()) {
        return Err(TrimRefitDecline::Unreadable);
    }
    if start_moved > bar || end_moved > bar {
        return Err(TrimRefitDecline::EndMoved);
    }

    // No worsening at any loop joint, in uv and in 3D: each gap to the
    // neighbour's end is read before and after, both must be finite, and the
    // after may not exceed the before. A one-coedge loop closes on itself.
    let count = lp.coedges.len();
    let joints: Vec<(&crate::NurbsCurve, f64, f64)> = if count == 1 {
        // (the trim that meets this end, its fraction, this trim's fraction)
        vec![(&coedge.pcurve, 1.0, 0.0)]
    } else {
        let previous = &lp.coedges[(coedge_index + count - 1) % count];
        let next = &lp.coedges[(coedge_index + 1) % count];
        vec![(&previous.pcurve, 1.0, 0.0), (&next.pcurve, 0.0, 1.0)]
    };
    let gap = |a: [f64; 2], b: [f64; 2]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    for (theirs, their_fraction, own_fraction) in joints {
        // For a one-coedge loop the meeting end is this trim's own other end,
        // old before and new after.
        let theirs_new: &crate::NurbsCurve = if count == 1 { &pcurve } else { theirs };
        let theirs_old_uv = uv_at(theirs, their_fraction).ok_or(TrimRefitDecline::Unreadable)?;
        let theirs_new_uv = uv_at(theirs_new, their_fraction).ok_or(TrimRefitDecline::Unreadable)?;
        let old_uv = uv_at(&coedge.pcurve, own_fraction).ok_or(TrimRefitDecline::Unreadable)?;
        let new_uv = uv_at(&pcurve, own_fraction).ok_or(TrimRefitDecline::Unreadable)?;
        let (before, after) = (gap(theirs_old_uv, old_uv), gap(theirs_new_uv, new_uv));
        if !(before.is_finite() && after.is_finite()) {
            return Err(TrimRefitDecline::Unreadable);
        }
        if after > before {
            return Err(TrimRefitDecline::UvJointWorsened);
        }
        let point = |uv: [f64; 2]| surface.evaluate_extended(uv[0], uv[1]).map_err(|_| TrimRefitDecline::Unreadable);
        let (before, after) = (
            point(theirs_old_uv)?.sub(point(old_uv)?).length(),
            point(theirs_new_uv)?.sub(point(new_uv)?).length(),
        );
        if !(before.is_finite() && after.is_finite()) {
            return Err(TrimRefitDecline::Unreadable);
        }
        if after > before {
            return Err(TrimRefitDecline::JointWorsened);
        }
    }
    Ok(pcurve)
}

/// The span budget the gate's two closure scans each read at: two of the
/// scanner's default budgets, a stated bound independent of the diagnostic
/// `BREP_VECTOR_AREA_SPANS` override. Measured 2026-10-04: the helmet operand
/// (`20_helmet_merge_stage/a.json`, 253 faces) needs 568474 spans before its
/// refit and 557748 after, over the default 400000 and under this.
const TRIM_REFIT_SPAN_BUDGET: usize = 2 * crate::soundness::VECTOR_AREA_SPAN_BUDGET;

/// The whole-transaction gate: validate's issues after must be a sub-multiset
/// of those before (an eliminated issue is allowed, a new or changed one is
/// not, multiplicity counts), and the vector-area scan must measure every
/// shell on both sides and not read worse ([`vector_areas_not_worse`]).
fn trim_refit_gate(before: &BrepSolid, after: &BrepSolid, policy: &KernelTolerances) -> Option<TrimRefitRollback> {
    trim_refit_gate_with_budget(before, after, policy, TRIM_REFIT_SPAN_BUDGET)
}

/// [`trim_refit_gate`] at an explicit scan budget, so a control can show an
/// exhausted budget rolls back.
fn trim_refit_gate_with_budget(
    before: &BrepSolid,
    after: &BrepSolid,
    policy: &KernelTolerances,
    span_budget: usize,
) -> Option<TrimRefitRollback> {
    if !issues_sub_multiset(&after.validate_with_tolerances(policy), &before.validate_with_tolerances(policy)) {
        return Some(TrimRefitRollback::NewOrChangedIssue);
    }
    let summary = |report: crate::VectorAreaReport| -> (Vec<(u64, crate::ClosureReading)>, Vec<String>) {
        (report.shells.iter().map(|shell| (shell.shell, shell.reading())).collect(), report.unreadable)
    };
    let (shells_before, unreadable_before) = summary(crate::soundness::shell_vector_areas_with_budget(before, span_budget));
    let (shells_after, unreadable_after) = summary(crate::soundness::shell_vector_areas_with_budget(after, span_budget));
    let verdict = vector_areas_not_worse((&shells_before, &unreadable_before), (&shells_after, &unreadable_after));
    if trim_refit_tracing().is_some() {
        // Debug only: the raw scans the vector-area verdict was read from, so
        // a rollback names its row. Decides nothing.
        for (label, shells, unreadable) in [("before", &shells_before, &unreadable_before), ("after", &shells_after, &unreadable_after)] {
            for (shell, reading) in shells.iter() {
                eprintln!("TRIM-REFIT-GATE {label} shell {shell} {reading:?}");
            }
            for row in unreadable.iter() {
                eprintln!("TRIM-REFIT-GATE {label} unreadable {row}");
            }
        }
        eprintln!("TRIM-REFIT-GATE verdict {verdict:?} ({})", vector_area_verdict_reason((&shells_before, &unreadable_before), (&shells_after, &unreadable_after)));
    }
    verdict
}

/// Debug only: which row [`vector_areas_not_worse`] stopped on, in words.
fn vector_area_verdict_reason(
    (shells_before, unreadable_before): (&[(u64, crate::ClosureReading)], &[String]),
    (shells_after, unreadable_after): (&[(u64, crate::ClosureReading)], &[String]),
) -> String {
    for (label, shells) in [("before", shells_before), ("after", shells_after)] {
        if let Some((shell, reading)) = shells.iter().find(|(_, reading)| matches!(reading, crate::ClosureReading::Unmeasured { .. })) {
            return format!("shell {shell} unmeasured {label}: {reading:?}");
        }
    }
    let mut available: HashMap<&str, usize> = HashMap::default();
    for row in unreadable_before {
        *available.entry(row.as_str()).or_default() += 1;
    }
    for row in unreadable_after {
        match available.get_mut(row.as_str()) {
            Some(count) if *count > 0 => *count -= 1,
            _ => return format!("new unreadable row after: {row}"),
        }
    }
    use crate::ClosureReading::{Measured, Open};
    for (shell, reading) in shells_before {
        let now = shells_after.iter().find(|(candidate, _)| candidate == shell).map(|(_, now)| now);
        let worse = match (reading, now) {
            (Measured { residual: old, .. }, Some(Measured { residual, .. })) | (Open { residual: old, .. }, Some(Open { residual, .. })) => {
                !(old.is_finite() && residual.is_finite() && residual <= old)
            }
            _ => true,
        };
        if worse {
            return format!("shell {shell}: before {reading:?}, after {now:?}");
        }
    }
    if let Some((shell, reading)) = shells_after.iter().find(|(shell, _)| !shells_before.iter().any(|(before, _)| before == shell)) {
        return format!("shell {shell} read after only: {reading:?}");
    }
    "no row".to_string()
}

/// The vector-area half of the gate, on the scans' readings:
/// * a shell UNMEASURED on either side rolls back, whatever the rows say: an
///   unread closure certifies nothing, and the same "unmeasured" before and
///   after is not a pass;
/// * every unreadable row after must have appeared before (as a multiset of
///   the rows themselves: a different row at the same count is new);
/// * EVERY shell read before, closed (Measured) or open (Open), must read
///   after as the same kind with both residuals finite and the after no
///   larger. Open is a topologically open sheet, not a failed threshold, and
///   carries no bar: its residual is its boundary's vector area, held to no
///   worsening like a closed one's. A missing shell or a non-finite residual
///   on either side rolls back;
/// * a reading of another KIND (closed ↔ open), or a shell after that was not
///   read before, rolls back. The refit writes trims only, never which edges
///   a shell uses, so its closedness cannot change; a kind change means the
///   two readings are not on one contract, and neither is evidence for the
///   other (stricter than passing an Open → Measured "improvement").
fn vector_areas_not_worse(
    (shells_before, unreadable_before): (&[(u64, crate::ClosureReading)], &[String]),
    (shells_after, unreadable_after): (&[(u64, crate::ClosureReading)], &[String]),
) -> Option<TrimRefitRollback> {
    if shells_before.iter().chain(shells_after).any(|(_, reading)| matches!(reading, crate::ClosureReading::Unmeasured { .. })) {
        return Some(TrimRefitRollback::VectorAreaUnmeasured);
    }
    let mut available: HashMap<&str, usize> = HashMap::default();
    for row in unreadable_before {
        *available.entry(row.as_str()).or_default() += 1;
    }
    for row in unreadable_after {
        match available.get_mut(row.as_str()) {
            Some(count) if *count > 0 => *count -= 1,
            _ => return Some(TrimRefitRollback::VectorAreaUnreadable),
        }
    }
    use crate::ClosureReading::{Measured, Open, Unmeasured};
    for (shell, reading) in shells_before {
        let old = match reading {
            Measured { residual, .. } | Open { residual, .. } => *residual,
            Unmeasured { .. } => return Some(TrimRefitRollback::VectorAreaUnmeasured),
        };
        if !old.is_finite() {
            return Some(TrimRefitRollback::VectorAreaUnreadable);
        }
        let Some((_, now)) = shells_after.iter().find(|(candidate, _)| candidate == shell) else {
            return Some(TrimRefitRollback::VectorAreaUnreadable);
        };
        let residual = match (reading, now) {
            (Measured { .. }, Measured { residual, .. }) | (Open { .. }, Open { residual, .. }) => *residual,
            (_, Unmeasured { .. }) => return Some(TrimRefitRollback::VectorAreaUnmeasured),
            _ => return Some(TrimRefitRollback::VectorAreaReadingChanged),
        };
        if !residual.is_finite() {
            return Some(TrimRefitRollback::VectorAreaUnreadable);
        }
        if residual > old {
            return Some(TrimRefitRollback::VectorAreaWorsened);
        }
    }
    if shells_after.iter().any(|(shell, _)| !shells_before.iter().any(|(before, _)| before == shell)) {
        return Some(TrimRefitRollback::VectorAreaReadingChanged);
    }
    None
}

/// Whether every issue in `after` (severity, kind, message; with multiplicity)
/// also appears in `before`.
fn issues_sub_multiset(after: &[crate::topology::ValidationIssue], before: &[crate::topology::ValidationIssue]) -> bool {
    let key = |issue: &crate::topology::ValidationIssue| (issue.severity, format!("{:?}", issue.kind), issue.message.clone());
    let mut available: HashMap<(&'static str, String, String), usize> = HashMap::default();
    for issue in before {
        *available.entry(key(issue)).or_default() += 1;
    }
    for issue in after {
        match available.get_mut(&key(issue)) {
            Some(count) if *count > 0 => *count -= 1,
            _ => return false,
        }
    }
    true
}

/// The trim's image and the edge point at one coedge fraction, mapped exactly
/// as validate maps them (affine in the pcurve's domain and in `[t0, t1]`,
/// reversed for a reversed use). `None` when an evaluation fails.
fn coedge_points(
    surface: &crate::NurbsSurface,
    pcurve: &crate::NurbsCurve,
    edge: &crate::topology::EdgeRecord,
    forward: bool,
    fraction: f64,
) -> Option<(Vec3, Vec3)> {
    let [q0, q1] = pcurve.domain().ok()?;
    let uv = pcurve.evaluate(q0 + (q1 - q0) * fraction).ok()?;
    let image = surface.evaluate_extended(uv.x, uv.y).ok()?;
    let t = if forward {
        edge.t0 + (edge.t1 - edge.t0) * fraction
    } else {
        edge.t1 - (edge.t1 - edge.t0) * fraction
    };
    let point = edge.curve.evaluate(t).ok()?;
    let finite = |p: Vec3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    (uv.x.is_finite() && uv.y.is_finite() && finite(image) && finite(point)).then_some((image, point))
}

fn uv_at(pcurve: &crate::NurbsCurve, fraction: f64) -> Option<[f64; 2]> {
    let [q0, q1] = pcurve.domain().ok()?;
    let uv = pcurve.evaluate(q0 + (q1 - q0) * fraction).ok()?;
    (uv.x.is_finite() && uv.y.is_finite()).then_some([uv.x, uv.y])
}

/// The largest same-parameter distance at `stations`, or `None` if any single
/// reading is non-finite or fails: `f64::max` would drop a NaN and report the
/// finite stations only.
fn worst_same_parameter(
    surface: &crate::NurbsSurface,
    pcurve: &crate::NurbsCurve,
    edge: &crate::topology::EdgeRecord,
    forward: bool,
    stations: &[f64],
) -> Option<f64> {
    let mut worst = 0.0_f64;
    for &fraction in stations {
        let (image, point) = coedge_points(surface, pcurve, edge, forward, fraction)?;
        let distance = image.sub(point).length();
        if !distance.is_finite() {
            return None;
        }
        worst = worst.max(distance);
    }
    Some(worst)
}

/// Whether every knot and control coordinate of `curve` is finite.
fn curve_is_finite(curve: &crate::NurbsCurve) -> bool {
    curve.knots.iter().all(|knot| knot.is_finite())
        && curve.control_points.iter().all(|point| [point.x, point.y, point.z, point.w].iter().all(|value| value.is_finite()))
}

/// The pcurve's distinct knots as coedge fractions, with 0 and 1. `None` if
/// the domain is empty or any knot, control coordinate or domain end is
/// non-finite: such a trim is refused as unreadable, never read on the
/// stations that happen to be finite.
fn pcurve_breaks(pcurve: &crate::NurbsCurve) -> Option<Vec<f64>> {
    if !curve_is_finite(pcurve) {
        return None;
    }
    let [q0, q1] = pcurve.domain().ok()?;
    if !(q0.is_finite() && q1.is_finite() && q1 > q0) {
        return None;
    }
    let mut breaks = vec![0.0, 1.0];
    breaks.extend(pcurve.knots.iter().filter(|&&knot| knot > q0 && knot < q1).map(|&knot| (knot - q0) / (q1 - q0)));
    Some(breaks)
}

/// The edge curve's distinct knots inside `[t0, t1]` as coedge fractions, with
/// 0 and 1.
fn edge_breaks(edge: &crate::topology::EdgeRecord, forward: bool) -> Option<Vec<f64>> {
    let span = edge.t1 - edge.t0;
    if !(edge.t0.is_finite() && edge.t1.is_finite() && span != 0.0 && curve_is_finite(&edge.curve)) {
        return None;
    }
    let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
    let mut breaks = vec![0.0, 1.0];
    breaks.extend(edge.curve.knots.iter().filter(|&&knot| knot > low && knot < high).map(|&knot| {
        if forward {
            (knot - edge.t0) / span
        } else {
            (edge.t1 - knot) / span
        }
    }));
    Some(breaks)
}

/// Sorted, exactly-deduplicated union of break sets; `None` if any break is
/// non-finite (rejected, not filtered).
fn merged_breaks(sets: &[Vec<f64>]) -> Option<Vec<f64>> {
    let mut all: Vec<f64> = sets.iter().flatten().copied().collect();
    if !all.iter().all(|value| value.is_finite()) {
        return None;
    }
    all.sort_by(f64::total_cmp);
    all.dedup();
    Some(all)
}

/// Both ends, every span's midpoint, and `interior` evenly spaced points of
/// every span between consecutive `breaks`.
fn span_stations(breaks: &[f64], interior: usize) -> Vec<f64> {
    let mut stations: Vec<f64> = breaks.to_vec();
    for pair in breaks.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if !(b > a) {
            continue;
        }
        stations.push(0.5 * (a + b));
        for index in 1..=interior {
            stations.push(a + (b - a) * index as f64 / (interior + 1) as f64);
        }
    }
    stations
}

/// Detection: validate's floor stations (every pcurve and edge-curve knot
/// span's midpoint) plus [`TRIM_REFIT_DETECT_INTERIOR`] points of every
/// pcurve span. `None` when the coedge's parameterisation cannot be read.
fn detection_stations(pcurve: &crate::NurbsCurve, edge: &crate::topology::EdgeRecord, forward: bool) -> Option<Vec<f64>> {
    let mut stations = span_stations(&merged_breaks(&[pcurve_breaks(pcurve)?])?, TRIM_REFIT_DETECT_INTERIOR);
    stations.extend(span_stations(&merged_breaks(&[edge_breaks(edge, forward)?])?, 0));
    Some(stations)
}

/// Acceptance: [`TRIM_REFIT_REREAD_INTERIOR`] points of every span of the
/// union of the old trim's, the edge's and the new trim's knots.
fn reread_stations(
    old: &crate::NurbsCurve,
    new: &crate::NurbsCurve,
    edge: &crate::topology::EdgeRecord,
    forward: bool,
) -> Option<Vec<f64>> {
    let breaks = merged_breaks(&[pcurve_breaks(old)?, pcurve_breaks(new)?, edge_breaks(edge, forward)?])?;
    Some(span_stations(&breaks, TRIM_REFIT_REREAD_INTERIOR))
}

/// Scope stations: `interior` points plus the midpoint and ends of every span
/// of the union of the given trims' knots and the edge's; `None` when any is
/// unreadable. No knot span is dropped.
fn scope_stations(trims: &[&crate::NurbsCurve], edge: &crate::topology::EdgeRecord, forward: bool, interior: usize) -> Option<Vec<f64>> {
    let mut sets = Vec::with_capacity(trims.len() + 1);
    for trim in trims {
        sets.push(pcurve_breaks(trim)?);
    }
    sets.push(edge_breaks(edge, forward)?);
    Some(span_stations(&merged_breaks(&sets)?, interior))
}

/// One scope reading of a point against a carrier: its distance to the
/// extended patch, and how far past the domain the winning foot lies, in 3D,
/// in each direction (0 inside the domain, and in a closed direction, which
/// wraps rather than extends).
#[derive(Clone, Copy, Debug)]
struct ScopeFoot {
    distance: f64,
    uv: [f64; 2],
    beyond_u: f64,
    beyond_v: f64,
}

/// The edge read against one carrier over a station set.
#[derive(Clone, Copy, Debug)]
struct ScopeRead {
    /// The largest distance to the extended patch.
    worst: f64,
    /// Stations whose winning foot lies outside the domain at all.
    outside: usize,
    /// Stations read through a carrier point ([`carrier_witness`]) because
    /// their extended foot was not proved to be on the carrier.
    witnessed: usize,
    /// The first station whose foot rests on an extension not proved to be
    /// the carrier's own ([`extension_kind`]) AND whose carrier witness is
    /// over `model`, as the decline it earns.
    extension: Option<TrimRefitDecline>,
}

/// Whether `foot`, if it lies past `surface`'s domain at ALL, rests on
/// something not proved to be the carrier, and which decline that would earn.
/// `None` inside the domain, and on a degree-1 ruling within its native
/// reach (the one extension proved to be the carrier's own point set).
///
/// What the C1 extension is, per case:
/// * a direction of degree above 1: the TANGENT continuation of a curved
///   iso-line, which the carrier does not follow ([`TrimRefitDecline::CurvedExtension`]);
/// * past both directions (a corner): two continuations compounded
///   ([`TrimRefitDecline::CornerExtension`]);
/// * a degree-1 direction: every iso-line is a straight segment, rational or
///   not (a rational linear map sends a line to a line), and the extension
///   moves along that segment's supporting line — straight supporting
///   geometry. It is the CARRIER'S continuation only where the carrier's own
///   rational end span, continued past its boundary parameter, reaches: with
///   the boundary weight above the inner one that continuation tends to a
///   finite point and stops short of the line's far part ([`native_reach`]);
///   past that reach it is [`TrimRefitDecline::BeyondNativeReach`]. A
///   distance claim there is about the point set, never a native parameter.
///
/// No overshoot threshold certifies anything: a foot a hair past a curved
/// boundary still measures the tangent extension. [`edge_standoff`] reads an
/// unproved foot through a point ON the carrier instead ([`carrier_witness`]).
fn extension_kind(surface: &crate::NurbsSurface, foot: &ScopeFoot) -> Option<TrimRefitDecline> {
    let (past_u, past_v) = (foot.beyond_u > 0.0, foot.beyond_v > 0.0);
    if past_u && past_v {
        return Some(TrimRefitDecline::CornerExtension);
    }
    for (past, degree, beyond, along_u) in [
        (past_u, surface.degree_u, foot.beyond_u, true),
        (past_v, surface.degree_v, foot.beyond_v, false),
    ] {
        if !past {
            continue;
        }
        if degree > 1 {
            return Some(TrimRefitDecline::CurvedExtension);
        }
        match native_reach(surface, along_u, foot.uv) {
            Some(reach) if beyond < reach => {}
            Some(_) => return Some(TrimRefitDecline::BeyondNativeReach),
            None => return Some(TrimRefitDecline::Unreadable),
        }
    }
    None
}

/// PROFILING ONLY: [`unclamped_foot`]'s Gauss–Newton from `seed` alone (no
/// projector, no grid, one start), returning the smallest distance seen. Never
/// used for a reading; it measures what a hint-first economy would read.
fn hint_only_distance(surface: &crate::NurbsSurface, point: Vec3, seed: [f64; 2]) -> Option<f64> {
    const ITERATIONS: usize = 50;
    let distance = |u: f64, v: f64| -> Option<f64> {
        let d = surface.evaluate_extended(u, v).ok()?.sub(point).length();
        d.is_finite().then_some(d)
    };
    let [mut u, mut v] = seed;
    let mut best = distance(u, v)?;
    for _ in 0..ITERATIONS {
        let Ok(derivatives) = surface.derivatives_extended(u, v, 1) else { break };
        let residual = derivatives[0][0].sub(point);
        let (su, sv) = (derivatives[1][0], derivatives[0][1]);
        let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let (gu, gv) = (su.dot(residual), sv.dot(residual));
        let determinant = a * c - b * b;
        if !(determinant.is_finite() && determinant.abs() > 0.0) {
            break;
        }
        let du = -(c * gu - b * gv) / determinant;
        let dv = -(a * gv - b * gu) / determinant;
        if !(du.is_finite() && dv.is_finite()) {
            break;
        }
        u += du;
        v += dv;
        if let Some(d) = distance(u, v) {
            best = best.min(d);
        }
        if du.abs() <= 1e-15 * (1.0 + u.abs()) && dv.abs() <= 1e-15 * (1.0 + v.abs()) {
            break;
        }
    }
    Some(best)
}

/// The distance from `point` to an actual point OF the carrier: `foot`'s
/// parameters clamped into the domain and evaluated there. Any carrier point
/// bounds the true standoff from above, so this is a valid witness wherever
/// the extended foot is not. `None` if it cannot be read.
fn carrier_witness(surface: &crate::NurbsSurface, foot: &ScopeFoot, point: Vec3) -> Option<f64> {
    let [u0, u1] = surface.domain_u().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    let on = surface.evaluate(foot.uv[0].clamp(u0, u1), foot.uv[1].clamp(v0, v1)).ok()?;
    let distance = on.sub(point).length();
    distance.is_finite().then_some(distance)
}

/// For a degree-1 direction (`along_u` or v): how far, in 3D along the
/// straight ruling through `uv`'s other coordinate, the carrier's own
/// rational continuation of its end span reaches past the boundary `uv` is
/// beyond. Infinite when it reaches the whole ray.
///
/// The end span from the inner knot (point `Pa`, weight `Wa`) to the boundary
/// (`Pb`, `Wb`) is `C(s) = ((1−s)·Wa·Pa + s·Wb·Pb) / ((1−s)·Wa + s·Wb)`. Its
/// midpoint parameter lands at `M = (Wa·Pa + Wb·Pb)/(Wa + Wb)`, so the weight
/// ratio is MEASURED from three evaluations: `r = Wb/Wa = |M − Pa| / |Pb − M|`.
/// For `s > 1` the continuation runs from `Pb` towards infinity when
/// `r ≤ 1` (a vanishing denominator, or a linear map at `r = 1`), and towards
/// the finite point `(Wb·Pb − Wa·Pa)/(Wb − Wa)` when `r > 1`, which lies
/// `|Pb − Pa| / (r − 1)` past `Pb`. `None` if anything is unreadable.
fn native_reach(surface: &crate::NurbsSurface, along_u: bool, uv: [f64; 2]) -> Option<f64> {
    let [u0, u1] = surface.domain_u().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    let (knots, low, high, at, other) = if along_u {
        (&surface.knots_u, u0, u1, uv[0], uv[1].clamp(v0, v1))
    } else {
        (&surface.knots_v, v0, v1, uv[1], uv[0].clamp(u0, u1))
    };
    if !(at.is_finite() && other.is_finite()) {
        return None;
    }
    let (boundary, inner) = if at > high {
        (high, knots.iter().copied().filter(|&knot| knot < high).fold(f64::NEG_INFINITY, f64::max))
    } else if at < low {
        (low, knots.iter().copied().filter(|&knot| knot > low).fold(f64::INFINITY, f64::min))
    } else {
        return Some(f64::INFINITY);
    };
    if !inner.is_finite() {
        return None;
    }
    let point = |t: f64| if along_u { surface.evaluate(t, other) } else { surface.evaluate(other, t) };
    let (pb, pa, pm) = (point(boundary).ok()?, point(inner).ok()?, point(0.5 * (boundary + inner)).ok()?);
    let (to_mid, from_mid, length) = (pm.sub(pa).length(), pb.sub(pm).length(), pb.sub(pa).length());
    if !(to_mid.is_finite() && from_mid.is_finite() && length.is_finite() && from_mid > 0.0) {
        return None;
    }
    let ratio = to_mid / from_mid;
    Some(if ratio <= 1.0 { f64::INFINITY } else { length / (ratio - 1.0) })
}

/// The edge read against `surface` over `stations` (fractions of a use running
/// `forward`), each station's foot seeded from the previous one. `None` if any
/// station or point cannot be read, or there are no stations.
fn edge_standoff(
    surface: &crate::NurbsSurface,
    edge: &crate::topology::EdgeRecord,
    forward: bool,
    stations: Option<&[f64]>,
    model: f64,
) -> Option<ScopeRead> {
    edge_standoff_replay(surface, edge, forward, stations, model, None, None)
}

/// The feet one read took on one carrier, in station order: each station's
/// fraction bits and its winning foot's uv.
type StationRecord = Vec<(u64, [f64; 2])>;

/// An edge claim's records, one per carrier `(shell, face)`.
type ClaimRecord = Vec<((usize, usize), StationRecord)>;

/// A record turned into a lookup: fraction bits -> (the incoming hint that
/// station had, its foot's uv).
type StationReplay = rustc_hash::FxHashMap<u64, (Option<[f64; 2]>, [f64; 2])>;

fn station_replay(record: &StationRecord) -> StationReplay {
    let mut replay = StationReplay::default();
    for (index, &(bits, uv)) in record.iter().enumerate() {
        let incoming = index.checked_sub(1).map(|previous| record[previous].1);
        replay.insert(bits, (incoming, uv));
    }
    replay
}

/// Bit equality of two incoming hints.
fn same_hint(a: Option<[f64; 2]>, b: Option<[f64; 2]>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => a[0].to_bits() == b[0].to_bits() && a[1].to_bits() == b[1].to_bits(),
        _ => false,
    }
}

/// [`edge_standoff`], replaying feet another read of the SAME carrier and
/// edge already took. A foot is a pure function of (carrier, point, incoming
/// hint) ([`unclamped_foot`]), and the point is a pure function of the
/// station's fraction; so a station whose fraction bits AND incoming hint
/// bits equal a recorded one has that record's foot uv, and the foot is
/// rebuilt from it ([`foot_at_uv`]) bit for bit. Nothing else is skipped: the
/// extension test, the carrier witness and every count run as before.
/// `record`, when given, receives this read's own feet.
fn edge_standoff_replay(
    surface: &crate::NurbsSurface,
    edge: &crate::topology::EdgeRecord,
    forward: bool,
    stations: Option<&[f64]>,
    model: f64,
    replay: Option<&StationReplay>,
    mut record: Option<&mut StationRecord>,
) -> Option<ScopeRead> {
    let stations = stations?;
    if stations.is_empty() {
        return None;
    }
    let mut read = ScopeRead { worst: 0.0, outside: 0, witnessed: 0, extension: None };
    let mut hint: Option<[f64; 2]> = None;
    for &fraction in stations {
        let t = if forward {
            edge.t0 + (edge.t1 - edge.t0) * fraction
        } else {
            edge.t1 - (edge.t1 - edge.t0) * fraction
        };
        let point = edge.curve.evaluate(t).ok()?;
        if !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite()) {
            return None;
        }
        profiling(|profile| {
            match profile.phase {
                ScopePhase::Candidacy => profile.candidacy_feet += 1,
                ScopePhase::OldClaim => profile.old_claim_feet += 1,
                ScopePhase::FinalClaim => profile.final_claim_feet += 1,
            }
            if !profile.keys.insert((surface as *const crate::NurbsSurface as usize, edge.id, t.to_bits())) {
                profile.repeated_feet[profile.phase as usize] += 1;
            }
        });
        let replayed = replay
            .and_then(|replay| replay.get(&fraction.to_bits()))
            .filter(|(incoming, _)| same_hint(*incoming, hint))
            .map(|&(_, uv)| uv);
        let foot = match replayed {
            Some(uv) => {
                profiling(|profile| profile.replayed_feet += 1);
                foot_at_uv(surface, point, uv)?
            }
            None => unclamped_foot(surface, point, hint)?,
        };
        if let Some(record) = record.as_mut() {
            record.push((fraction.to_bits(), foot.uv));
        }
        hint = Some(foot.uv);
        if foot.beyond_u > 0.0 || foot.beyond_v > 0.0 {
            read.outside += 1;
        }
        // Inside the domain, or on a proved ruling, the extended distance is
        // a distance to the carrier. Otherwise the standoff is read through a
        // point ON the carrier, at any overshoot, and a witness over `model`
        // declines with the reason the extension was unproved.
        let distance = match extension_kind(surface, &foot) {
            None => foot.distance,
            Some(kind) => {
                let witness = carrier_witness(surface, &foot, point)?;
                profiling(|profile| profile.witness_evaluations += 1);
                read.witnessed += 1;
                if witness > model && read.extension.is_none() {
                    read.extension = Some(kind);
                }
                witness
            }
        };
        read.worst = read.worst.max(distance);
    }
    Some(read)
}

/// Distance from `point` to the EXTENDED patch: `surface` inside its domain
/// and [`crate::NurbsSurface::derivatives_extended`]'s C1 continuation past
/// it (ruled/bilinear from the boundary), with no clamp on `(u, v)`.
/// Gauss–Newton from the global projector's foot and from `hint` (the
/// previous station's foot); a 9×9 grid's best few seeds are added only when
/// neither gives a finite start. Every evaluated point lies on the extended
/// patch, so the smallest distance seen is an upper bound on the standoff
/// from the extended patch whether or not an iteration converged. Past the
/// domain in a CURVED direction the continuation is the tangent extension,
/// not the carrier's analytic continuation; the returned `beyond_u` /
/// `beyond_v` say how far past (in 3D) the winning foot lies, so the caller
/// can read it through a carrier point instead ([`extension_kind`], [`carrier_witness`]). `None` when no finite
/// reading exists.
///
/// The public projectors clamp to the patch (and an analytic cylinder's
/// closed form clamps its axial coordinate), so a point beside the patch
/// reads its overshoot there; this one reads the extended patch.
fn unclamped_foot(surface: &crate::NurbsSurface, point: Vec3, hint: Option<[f64; 2]>) -> Option<ScopeFoot> {
    const GRID: usize = 8;
    const SEEDS: usize = 3;
    const ITERATIONS: usize = 50;
    let [u0, u1] = surface.domain_u().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    // Read here as before (an unreadable closedness is no foot); the tail
    // ([`finish_foot`]) reads it again for the overshoot.
    surface.closed_directions().ok()?;
    let distance = |u: f64, v: f64| -> Option<f64> {
        let d = surface.evaluate_extended(u, v).ok()?.sub(point).length();
        d.is_finite().then_some(d)
    };
    let mut seeds: Vec<[f64; 2]> = Vec::new();
    let projector_started = profiling_on().then(web_time::Instant::now);
    let projected = crate::project_point_to_surface(surface, point);
    if let Some(started) = projector_started {
        profiling(|profile| {
            profile.projector_calls += 1;
            profile.projector_seconds += started.elapsed().as_secs_f64();
        });
    }
    if let Ok(foot) = projected {
        seeds.push([foot.u, foot.v]);
    }
    seeds.extend(hint);
    let mut scored: Vec<(f64, [f64; 2])> = seeds
        .into_iter()
        .filter_map(|seed| distance(seed[0], seed[1]).map(|d| (d, seed)))
        .collect();
    if scored.is_empty() {
        profiling(|profile| profile.grid_fallbacks += 1);
        for i in 0..=GRID {
            for j in 0..=GRID {
                let seed = [u0 + (u1 - u0) * i as f64 / GRID as f64, v0 + (v1 - v0) * j as f64 / GRID as f64];
                if let Some(d) = distance(seed[0], seed[1]) {
                    scored.push((d, seed));
                }
            }
        }
    }
    if scored.is_empty() {
        return None;
    }
    scored.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut best = (f64::INFINITY, [f64::NAN; 2]);
    let mut iterations = 0_usize;
    for &(start, [mut u, mut v]) in scored.iter().take(SEEDS) {
        if start < best.0 {
            best = (start, [u, v]);
        }
        for _ in 0..ITERATIONS {
            iterations += 1;
            let Ok(derivatives) = surface.derivatives_extended(u, v, 1) else { break };
            let residual = derivatives[0][0].sub(point);
            let (su, sv) = (derivatives[1][0], derivatives[0][1]);
            let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
            let (gu, gv) = (su.dot(residual), sv.dot(residual));
            let determinant = a * c - b * b;
            if !(determinant.is_finite() && determinant.abs() > 0.0) {
                break;
            }
            let du = -(c * gu - b * gv) / determinant;
            let dv = -(a * gv - b * gu) / determinant;
            if !(du.is_finite() && dv.is_finite()) {
                break;
            }
            u += du;
            v += dv;
            if let Some(d) = distance(u, v) {
                if d < best.0 {
                    best = (d, [u, v]);
                }
            }
            if du.abs() <= 1e-15 * (1.0 + u.abs()) && dv.abs() <= 1e-15 * (1.0 + v.abs()) {
                break;
            }
        }
    }
    profiling(|profile| profile.newton_iterations += iterations);
    if let Some(seed) = hint.filter(|_| profiling_on()) {
        let sampled = SCOPE_PROFILE.with(|cell| {
            cell.borrow_mut().as_mut().is_some_and(|profile| {
                profile.sample_clock += 1;
                profile.sample_clock % SCOPE_PROFILE_SAMPLE == 0
            })
        });
        if sampled {
            // On the side: the hint alone, never used for the reading.
            let alone = hint_only_distance(surface, point, seed);
            profiling(|profile| {
                profile.sampled += 1;
                match alone {
                    None => profile.hint_unreadable += 1,
                    Some(value) if (value - best.0).abs() <= 1e-12 => profile.hint_agrees += 1,
                    Some(value) if value < best.0 => profile.hint_smaller += 1,
                    Some(_) => profile.hint_larger += 1,
                }
            });
        }
    }
    let (d, [u, v]) = best;
    finish_foot(surface, d, u, v)
}

/// The foot at `uv` for `point`: the distance read exactly as
/// [`unclamped_foot`] reads every candidate (`evaluate_extended`, then the
/// length), so for the uv [`unclamped_foot`] returned it is that foot's
/// distance bit for bit, and the rest is [`finish_foot`].
fn foot_at_uv(surface: &crate::NurbsSurface, point: Vec3, [u, v]: [f64; 2]) -> Option<ScopeFoot> {
    let d = surface.evaluate_extended(u, v).ok()?.sub(point).length();
    if !d.is_finite() {
        return None;
    }
    finish_foot(surface, d, u, v)
}

/// A winning `(d, u, v)` as a [`ScopeFoot`]: how far past the domain, in 3D
/// along the extension, it lies. A function of the carrier and `(u, v)` only.
fn finish_foot(surface: &crate::NurbsSurface, d: f64, u: f64, v: f64) -> Option<ScopeFoot> {
    let [u0, u1] = surface.domain_u().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    let (closed_u, closed_v) = surface.closed_directions().ok()?;
    if !(d.is_finite() && u.is_finite() && v.is_finite()) {
        return None;
    }
    // How far past the domain, in 3D along the extension, the foot lies.
    let out = |x: f64, low: f64, high: f64, closed: bool| -> f64 {
        if closed {
            0.0
        } else if x < low {
            low - x
        } else if x > high {
            x - high
        } else {
            0.0
        }
    };
    let (du_out, dv_out) = (out(u, u0, u1, closed_u), out(v, v0, v1, closed_v));
    let (beyond_u, beyond_v) = if du_out > 0.0 || dv_out > 0.0 {
        let derivatives = surface.derivatives_extended(u, v, 1).ok()?;
        let (speed_u, speed_v) = (derivatives[1][0].length(), derivatives[0][1].length());
        if !(speed_u.is_finite() && speed_v.is_finite()) {
            return None;
        }
        (du_out * speed_u, dv_out * speed_v)
    } else {
        (0.0, 0.0)
    };
    Some(ScopeFoot { distance: d, uv: [u, v], beyond_u, beyond_v })
}


