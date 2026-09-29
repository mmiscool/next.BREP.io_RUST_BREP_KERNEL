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

