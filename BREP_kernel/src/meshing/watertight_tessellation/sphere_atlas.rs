//! Watertight tessellation of a spherical face through the pole-free cube atlas.
//!
//! The generic path meshes a face in its own parameter domain.  For a sphere that
//! domain is polar, and every one of its defects lands in the mesh: the two poles
//! are whole parameter lines collapsed to a point, so the triangulation fans
//! thousands of slivers into them and the surface normal there has to be faked;
//! the seam is a periodic identification, so a trim region that straddles it is
//! not a connected parameter set at all and needs an unwrapping pass to become
//! one.  A cut placed ON either defect is where the special cases run out.
//!
//! Here the face is meshed in the six cube charts of
//! [`crate::sphere_chart::SphereAtlas`] instead.  Each chart is a regular square
//! with no pole and no periodicity, so a former pole is an ordinary interior
//! point and a former seam is an ordinary interior line.  The charts are internal
//! and must stay so:
//!
//! * **No topology.**  The face keeps its identity, its name and its loops; the
//!   atlas mints no edge and no face, and nothing about it is stored.
//! * **No crack.**  A chart boundary is subdivided ONCE, by the cube edge that
//!   carries it, and both charts read that one subdivision through a shared
//!   vertex table keyed by global index — so the two charts meeting along a cube
//!   edge emit the identical vertices, not merely coincident ones.
//! * **No T-junction against the neighbouring face.**  Where a trim polyline
//!   crosses a chart boundary, the crossing must be a point the shared
//!   edge-sample table already holds, so the face on the other side of that edge
//!   has it too.  `edge_sampling::sample_all_edges` inserts them for exactly this
//!   reason; this module only consumes what is there and never invents a boundary
//!   point of its own.
//!
//! Material is decided by [`crate::sphere_chart::SphericalRegion`] — the same
//! classifier the trim query uses — so the mesh and `parameter_point_in_face`
//! cannot disagree about which side of a trim curve is solid.
//!
//! The lane declines (returns `false`, leaving the face to the existing path
//! untouched) whenever anything is not as it expects: a surface that is not a
//! sphere, a chart whose constrained triangulation had to split a constraint, or
//! a chart mesh that fails its own manifold check.

use super::*;
use crate::sphere_chart::{
    chart_grid_divisions, chart_grid_parameters, cube_edge_divisions, cube_edge_parameters,
    ChartSide, SphereAtlas, SphericalRegion, CHART_COUNT, CUBE_CORNER_COUNT, CUBE_EDGE_COUNT,
};

/// Slack, in chart coordinates, for "this trim vertex is ON the chart square's
/// boundary".  Trim vertices land there because a boundary crossing was solved
/// for them, so the residual is rounding, not modelling error.
///
/// This is the SAME test [`chart_uv`] clamps by, and that is the point: a vertex
/// the clamp moves onto the boundary but the pinning does not file against the
/// cube edge becomes a second vertex at a position the ring already holds, and
/// the triangulator refuses the coincidence. One test, one answer.
const ON_EDGE_TOLERANCE: f64 = 1e-9;

/// Interior grid points closer than this fraction of the local grid spacing to a
/// constraint are dropped: they would only produce slivers against it.
const CONSTRAINT_CLEARANCE: f64 = 0.35;

/// Refinement passes that may add interior points to hold the chord tolerance in
/// a chart the trim left coarse.
const MAX_REFINE_PASSES: usize = 3;

/// A vertex of the atlas mesh, addressed by a GLOBAL index so that a point on a
/// cube edge or corner is one vertex, not one per chart.
struct AtlasVertices {
    positions: Vec<Vec3>,
    /// Cube corner index -> global vertex index, allocated on first use.
    corners: [Option<usize>; CUBE_CORNER_COUNT],
}

impl AtlasVertices {
    fn new() -> Self {
        Self {
            positions: Vec::new(),
            corners: [None; CUBE_CORNER_COUNT],
        }
    }

    fn push(&mut self, position: Vec3) -> usize {
        self.positions.push(position);
        self.positions.len() - 1
    }
}

/// One subdivision point of a cube edge: its edge parameter and the global
/// vertex it names.  Both charts that meet along the edge walk this same list.
#[derive(Clone, Copy)]
struct EdgePoint {
    q: f64,
    vertex: usize,
}

/// A trim polyline piece confined to one chart, in that chart's coordinates.
struct ChartChain {
    /// `(s, t)` and global vertex index, in traversal order.
    points: Vec<([f64; 2], usize)>,
}

/// Decline this face to the existing path, naming the reason when asked.
///
/// A decline is not a failure — it is the lane refusing to guess — but a SILENT
/// decline is indistinguishable from the lane never having engaged, and that
/// difference is most of what a diagnostic session needs to know.
fn decline(face_id: u32, why: &str) -> Result<bool, String> {
    if std::env::var("BREP_DEBUG_SPHERE_CHARTS").is_ok() {
        eprintln!("[sphere-atlas] face {face_id}: DECLINED — {why}");
    }
    Ok(false)
}

pub(super) fn tessellate_spherical_face_watertight(
    face: &FaceRecord,
    samples: &HashMap<u64, EdgeSamples>,
    chord_tolerance: f64,
    face_id: u32,
    mesh: &mut Mesh,
) -> Result<bool, String> {
    if std::env::var("BREP_NO_SPHERE_CHARTS").is_ok() {
        return Ok(false);
    }
    let Some(atlas) = SphereAtlas::of_surface(&face.surface) else {
        return Ok(false);
    };
    // Which way the surface's OWN parameterization faces.  The loop convention
    // (material to the left) is stated against `Su x Sv`, and a reflected sphere
    // has it pointing inward, so this cannot be assumed.
    let outward = atlas.parameterization_is_outward(&face.surface)?;

    // 1. The trim, as 3D segments built from the SHARED edge samples: the exact
    //    positions the neighbouring faces will use.
    //
    //    An edge used TWICE by this one face is a SLIT — the parametric seam of a
    //    ball the trim never cut. It bounds no material, and it is dropped here
    //    rather than left to cancel later, so the classifier and the triangulation
    //    see the same boundary. Leaving it in would also stitch a column of
    //    constraint points down the seam and hem it with slivers, which is the
    //    polar domain's artefact reappearing in a mesh that has no seam.
    let mut slit: HashMap<u64, usize> = HashMap::new();
    for coedge in face.loops.iter().flat_map(|record| record.coedges.iter()) {
        *slit.entry(coedge.edge_id).or_insert(0) += 1;
    }
    let mut boundary: Vec<(Vec3, Vec3)> = Vec::new();
    for coedge in face.loops.iter().flat_map(|record| record.coedges.iter()) {
        if slit.get(&coedge.edge_id).copied().unwrap_or(0) >= 2 {
            continue;
        }
        let edge_samples = samples
            .get(&coedge.edge_id)
            .ok_or("sphere atlas: missing edge samples")?;
        let count = edge_samples.positions.len();
        for index in 0..count.saturating_sub(1) {
            let (a, b) = if coedge.forward {
                (edge_samples.positions[index], edge_samples.positions[index + 1])
            } else {
                (
                    edge_samples.positions[count - 1 - index],
                    edge_samples.positions[count - 2 - index],
                )
            };
            boundary.push((a, b));
        }
    }
    // The material-left convention is stated against the FACE normal, which for a
    // cavity wall points INTO the sphere: `same_sense` alone and the surface
    // normal alone are each half the answer.
    let outward_face_normal = face.same_sense == outward;
    let region = SphericalRegion::from_segments(atlas.centre, &boundary, outward_face_normal);
    if !region.is_decidable() {
        // The classifier could not place a seed. Declining hands the face to the
        // existing path unchanged; answering anyway would keep every triangle and
        // mesh the whole ball twice, with nothing in the output to say so.
        return decline(face_id, "no seed: the region could not be classified");
    }

    // 2. Cut every trim segment at the chart boundaries.  Pieces come back
    //    already assigned to one chart each; a crossing point is a function of
    //    the segment alone, so the two charts sharing it agree exactly.
    let mut vertices = AtlasVertices::new();
    let mut segments = Vec::new();
    for &(a, b) in &boundary {
        segments.extend(atlas.split_segment(a, b)?);
    }

    // 3. Name every trim vertex once, and file the ones that landed on a cube
    //    edge or corner against that edge or corner so the chart boundary is
    //    subdivided at exactly the same place from both sides.
    let mut trim_vertex: HashMap<[u64; 3], usize> = HashMap::new();
    let mut edge_pinned: Vec<Vec<(f64, usize)>> = vec![Vec::new(); CUBE_EDGE_COUNT];
    let key = |p: Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
    let name = |vertices: &mut AtlasVertices,
                trim_vertex: &mut HashMap<[u64; 3], usize>,
                edge_pinned: &mut Vec<Vec<(f64, usize)>>,
                chart: usize,
                point: Vec3|
     -> usize {
        if let Some(&index) = trim_vertex.get(&key(point)) {
            return index;
        }
        let index = vertices.push(point);
        trim_vertex.insert(key(point), index);
        let sides = chart_sides(&atlas, chart, point);
        for &(side, edge, q) in &sides {
            let _ = side;
            edge_pinned[edge].push((q, index));
        }
        if sides.len() >= 2 {
            // Two sides at once is a cube CORNER. Pin it so all three charts
            // meeting there address one vertex, by name rather than by proximity.
            let (_, edge, q) = sides[0];
            let corner = crate::sphere_chart::cube_edge_corner(edge, q > 0.0);
            vertices.corners[corner] = Some(index);
        }
        index
    };

    // Stretches of a cube edge the trim itself runs along. The trim's own samples
    // are the ONLY subdivision the neighbouring face has there, so the shared
    // edge list must not carry a generated point inside one: that point would be
    // on the sphere's boundary polyline and absent from the neighbour's — a
    // T-junction, which is a crack.
    let mut edge_spans: Vec<Vec<(f64, f64)>> = vec![Vec::new(); CUBE_EDGE_COUNT];
    for segment in &segments {
        name(&mut vertices, &mut trim_vertex, &mut edge_pinned, segment.chart, segment.start);
        name(&mut vertices, &mut trim_vertex, &mut edge_pinned, segment.chart, segment.end);
        if let Some((edge, a, b)) = shared_chart_side(&atlas, segment.chart, segment.start, segment.end)
        {
            edge_spans[edge].push((a.min(b), a.max(b)));
        }
    }

    // 4. Subdivide the twelve cube edges once each, merging the pinned trim
    //    points into the shared parameter list.  A generated point that collides
    //    with a pinned one is dropped: the pinned point is the one the trim (and
    //    therefore the neighbouring face) already holds.
    let divisions = cube_edge_divisions(atlas.radius, chord_tolerance);
    let base = cube_edge_parameters(divisions);
    let spacing = 2.0 / divisions as f64;
    let mut edge_points: Vec<Vec<EdgePoint>> = Vec::with_capacity(CUBE_EDGE_COUNT);
    for edge in 0..CUBE_EDGE_COUNT {
        let mut pinned = edge_pinned[edge].clone();
        pinned.sort_by(|a, b| a.0.total_cmp(&b.0));
        pinned.dedup_by(|a, b| (a.0 - b.0).abs() <= 1e-12);
        let mut list: Vec<EdgePoint> = Vec::with_capacity(base.len() + pinned.len());
        for &q in &base {
            if q.abs() >= 1.0 {
                let corner = crate::sphere_chart::cube_edge_corner(edge, q > 0.0);
                // A trim point that landed on this corner IS the corner: it is
                // what the neighbouring face holds, and minting the canonical
                // point beside it would leave two vertices a few ulps apart —
                // one per face — which is a crack however small it looks.
                let vertex = match pinned
                    .iter()
                    .find(|(pin, _)| (pin - q).abs() <= 1e-9)
                    .map(|(_, vertex)| *vertex)
                    .or(vertices.corners[corner])
                {
                    Some(existing) => existing,
                    None => {
                        let point = corner_point(&atlas, corner)?;
                        let index = vertices.push(point);
                        vertices.corners[corner] = Some(index);
                        index
                    }
                };
                list.push(EdgePoint { q, vertex });
                continue;
            }
            if pinned
                .iter()
                .any(|(pin, _)| (pin - q).abs() <= 0.25 * spacing)
            {
                continue;
            }
            if edge_spans[edge]
                .iter()
                .any(|(low, high)| q > low + 1e-12 && q < high - 1e-12)
            {
                continue;
            }
            list.push(EdgePoint {
                q,
                vertex: vertices.push(atlas.edge_point(edge, q)?),
            });
        }
        for &(q, vertex) in &pinned {
            list.push(EdgePoint { q, vertex });
        }
        list.sort_by(|a, b| a.q.total_cmp(&b.q));
        list.dedup_by(|a, b| a.vertex == b.vertex);
        edge_points.push(list);
    }

    // 5. Trim segments, grouped per chart into chains, and the set of vertex
    //    pairs that a mesh edge may not cross (the material boundary).
    let mut blocking: HashSet<(usize, usize)> = HashSet::new();
    let mut chains: Vec<Vec<ChartChain>> = (0..CHART_COUNT).map(|_| Vec::new()).collect();
    for segment in &segments {
        let start = trim_vertex[&key(segment.start)];
        let end = trim_vertex[&key(segment.end)];
        if start == end {
            continue;
        }
        blocking.insert(ordered(start, end));
        // A segment that runs ALONG a cube edge is already in that edge's
        // subdivision; adding it again would cross the boundary constraint and
        // force the triangulator to split one of them.  Blocking the covered
        // stretch of the edge is the same instruction without the conflict.
        if shared_chart_side(&atlas, segment.chart, segment.start, segment.end)
            .is_some_and(|(edge, _, _)| {
                block_edge_span(&edge_points[edge], start, end, &mut blocking)
            })
        {
            continue;
        }
        let chart = segment.chart;
        let (Some(a), Some(b)) = (
            chart_uv(&atlas, chart, segment.start),
            chart_uv(&atlas, chart, segment.end),
        ) else {
            return decline(face_id, "a trim segment fell outside the chart it was assigned");
        };
        chains[chart].push(ChartChain {
            points: vec![(a, start), (b, end)],
        });
    }

    // 6. Mesh each chart, then weld: shared global indices mean the charts are
    //    already one mesh, with no seam to stitch and nothing to snap.
    let mut triangles: Vec<[usize; 3]> = Vec::new();
    for chart in 0..CHART_COUNT {
        let Some(chart_triangles) = mesh_chart(
            face_id,
            &atlas,
            chart,
            &edge_points,
            &chains[chart],
            &mut vertices,
            chord_tolerance,
        )?
        else {
            return decline(face_id, &format!("chart {chart} could not be triangulated"));
        };
        triangles.extend(chart_triangles);
    }
    if triangles.is_empty() {
        return decline(face_id, "no triangles");
    }

    // 7. Keep the material triangles.  Flood across mesh edges that are not the
    //    trim, so each connected region is classified once, by the same
    //    solid-angle test the trim query uses.
    let keep = material_regions(&triangles, &blocking, &vertices, &atlas, &region);

    if std::env::var("BREP_DEBUG_SPHERE_CHARTS").is_ok() {
        let kept = keep.iter().filter(|value| **value).count();
        eprintln!(
            "[sphere-atlas] face {face_id}: boundary={} segments={} whole={} triangles={} kept={} outward={outward} same_sense={}",
            boundary.len(),
            segments.len(),
            region.is_whole_sphere(),
            triangles.len(),
            kept,
            face.same_sense,
        );
    }
    // 8. Emit.  A chart is oriented so `∂p/∂s x ∂p/∂t` points out of the sphere,
    //    so a chart-counter-clockwise triangle faces outward; the face wants that
    //    winding exactly when its own sense and its parameterization agree.
    let ccw = face.same_sense == outward;
    let base_index = (mesh.positions.len() / 3) as u32;
    let mut emitted: HashMap<usize, u32> = HashMap::new();
    let mut used: Vec<u32> = Vec::new();
    for (index, triangle) in triangles.iter().enumerate() {
        if !keep[index] {
            continue;
        }
        let mut corners = [0u32; 3];
        for (slot, &vertex) in triangle.iter().enumerate() {
            corners[slot] = match emitted.get(&vertex) {
                Some(&existing) => existing,
                None => {
                    let position = vertices.positions[vertex];
                    let mut normal = position.sub(atlas.centre).normalized()?;
                    if !ccw {
                        normal = normal.scale(-1.0);
                    }
                    mesh.positions.extend([position.x, position.y, position.z]);
                    mesh.normals.extend([normal.x, normal.y, normal.z]);
                    let new = base_index + used.len() as u32;
                    used.push(new);
                    emitted.insert(vertex, new);
                    new
                }
            };
        }
        if corners[0] == corners[1] || corners[1] == corners[2] || corners[2] == corners[0] {
            continue;
        }
        if ccw {
            mesh.indices.extend(corners);
        } else {
            mesh.indices.extend([corners[0], corners[2], corners[1]]);
        }
        mesh.face_ids.push(face_id);
    }
    Ok(!mesh.indices.is_empty())
}

fn ordered(a: usize, b: usize) -> (usize, usize) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

/// The canonical 3D point of a cube corner.  Summed in a FIXED axis order so
/// every chart and every cube edge that reaches this corner computes the same
/// bits — the corner belongs to none of them and must not depend on the route.
fn corner_point(atlas: &SphereAtlas, corner: usize) -> Result<Vec3, String> {
    let sign = |bit: usize| if corner & (1 << bit) != 0 { 1.0 } else { -1.0 };
    let direction = atlas.basis[0]
        .scale(sign(0))
        .add(atlas.basis[1].scale(sign(1)))
        .add(atlas.basis[2].scale(sign(2)))
        .normalized()?;
    Ok(atlas.centre.add(direction.scale(atlas.radius)))
}

/// The sides of `chart`'s square that `point` lies on, with the cube edge each
/// carries and the point's parameter along it.
///
/// Expressed in the chart's own coordinates rather than in the sphere's axis
/// coordinates, so it answers the same question [`chart_uv`]'s clamp does. One
/// entry along a side, two at a corner, none in the interior.
fn chart_sides(atlas: &SphereAtlas, chart: usize, point: Vec3) -> Vec<(ChartSide, usize, f64)> {
    let Some((s, t)) = atlas.coordinates(chart, point) else {
        return Vec::new();
    };
    let mut sides = Vec::new();
    let mut push = |side: ChartSide, free: f64| {
        let (edge, sign) = side.edge(chart);
        sides.push((side, edge, (sign * free).clamp(-1.0, 1.0)));
    };
    if s >= 1.0 - ON_EDGE_TOLERANCE {
        push(ChartSide::SPlus, t);
    } else if s <= -1.0 + ON_EDGE_TOLERANCE {
        push(ChartSide::SMinus, t);
    }
    if t >= 1.0 - ON_EDGE_TOLERANCE {
        push(ChartSide::TPlus, s);
    } else if t <= -1.0 + ON_EDGE_TOLERANCE {
        push(ChartSide::TMinus, s);
    }
    sides
}

/// The cube edge a segment RUNS ALONG, with both endpoints' parameters on it.
///
/// Both ends and the midpoint must share one side of the chart square, so a
/// chord that merely joins two points of the same side across the interior is
/// not mistaken for one that follows it.
fn shared_chart_side(
    atlas: &SphereAtlas,
    chart: usize,
    a: Vec3,
    b: Vec3,
) -> Option<(usize, f64, f64)> {
    let start = chart_sides(atlas, chart, a);
    let end = chart_sides(atlas, chart, b);
    let middle = chart_sides(atlas, chart, a.add(b).scale(0.5));
    start.iter().find_map(|&(side, edge, qa)| {
        let (_, _, qb) = *end.iter().find(|(candidate, _, _)| *candidate == side)?;
        middle
            .iter()
            .any(|(candidate, _, _)| *candidate == side)
            .then_some((edge, qa, qb))
    })
}

/// Block every subdivision step of a cube edge between two of its own points, so
/// a trim that runs along the edge separates the two charts that share it.
fn block_edge_span(
    points: &[EdgePoint],
    start: usize,
    end: usize,
    blocking: &mut HashSet<(usize, usize)>,
) -> bool {
    let position = |vertex: usize| points.iter().position(|point| point.vertex == vertex);
    let (Some(first), Some(last)) = (position(start), position(end)) else {
        return false;
    };
    let (lo, hi) = (first.min(last), first.max(last));
    for index in lo..hi {
        blocking.insert(ordered(points[index].vertex, points[index + 1].vertex));
    }
    first != last
}

/// Chart coordinates, clamped to the closed square.  A trim vertex solved onto a
/// chart boundary can land a rounding step outside it; the 3D position is the
/// shared one either way, so only the parameter is nudged.
fn chart_uv(atlas: &SphereAtlas, chart: usize, point: Vec3) -> Option<[f64; 2]> {
    let (s, t) = atlas.coordinates(chart, point)?;
    Some([s.clamp(-1.0, 1.0), t.clamp(-1.0, 1.0)])
}

/// Triangulate one chart square: its four sides come from the shared cube-edge
/// subdivisions, its interior from a grid, and the trim pieces inside it are
/// constraints.  Returns `None` when the constrained triangulation could not
/// reproduce the input, so the caller can decline the whole face.
fn mesh_chart(
    face_id: u32,
    atlas: &SphereAtlas,
    chart: usize,
    edge_points: &[Vec<EdgePoint>],
    chains: &[ChartChain],
    vertices: &mut AtlasVertices,
    chord_tolerance: f64,
) -> Result<Option<Vec<[usize; 3]>>, String> {
    use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
    let refuse = |why: &str, points: &[([f64; 2], usize)], ring: usize| -> Option<Vec<[usize; 3]>> {
        if std::env::var("BREP_DEBUG_SPHERE_CHARTS").is_ok() {
            eprintln!("[sphere-atlas] face {face_id} chart {chart}: {why}");
            for (slot, (uv, vertex)) in points.iter().enumerate() {
                if uv[0].abs() < 1.0 - 1e-9 && uv[1].abs() < 1.0 - 1e-9 {
                    continue;
                }
                eprintln!(
                    "[sphere-atlas]     {} slot {slot} v{vertex} ({:.15}, {:.15})",
                    if slot < ring { "ring " } else { "chain" },
                    uv[0],
                    uv[1]
                );
            }
        }
        None
    };

    // The closed square ring, counter-clockwise in (s, t), walked side by side
    // out of the shared cube-edge lists.
    let mut ring: Vec<([f64; 2], usize)> = Vec::new();
    for (side, ascending) in [
        (ChartSide::TMinus, true),
        (ChartSide::SPlus, true),
        (ChartSide::TPlus, false),
        (ChartSide::SMinus, false),
    ] {
        let (edge, sign) = side.edge(chart);
        let list = &edge_points[edge];
        let mut side_points: Vec<([f64; 2], usize)> = list
            .iter()
            .map(|point| {
                let free = sign * point.q;
                let (s, t) = side.coords(free);
                ([s, t], point.vertex)
            })
            .collect();
        side_points.sort_by(|a, b| {
            let axis = usize::from(matches!(side, ChartSide::SPlus | ChartSide::SMinus));
            a.0[axis].total_cmp(&b.0[axis])
        });
        if !ascending {
            side_points.reverse();
        }
        // The first point of each side is the previous side's last.
        side_points.pop();
        ring.extend(side_points);
    }
    if ring.len() < 4 {
        return Ok(refuse("ring has fewer than four points", &[], 0));
    }

    let mut points: Vec<([f64; 2], usize)> = ring.clone();
    let mut constraints: Vec<(usize, usize)> = Vec::new();
    for index in 0..ring.len() {
        constraints.push((index, (index + 1) % ring.len()));
    }
    let mut seen: HashMap<usize, usize> = ring
        .iter()
        .enumerate()
        .map(|(slot, (_, vertex))| (*vertex, slot))
        .collect();
    for chain in chains {
        let mut previous: Option<usize> = None;
        for (uv, vertex) in &chain.points {
            let slot = *seen.entry(*vertex).or_insert_with(|| {
                points.push((*uv, *vertex));
                points.len() - 1
            });
            if let Some(previous) = previous {
                if previous != slot {
                    constraints.push((previous, slot));
                }
            }
            previous = Some(slot);
        }
    }

    // Interior grid, on the same parameter list the sides use so the density
    // matches across a chart boundary, minus anything that would only make a
    // sliver against a constraint.
    let divisions = chart_grid_divisions(atlas.radius, chord_tolerance);
    let grid = chart_grid_parameters(divisions);
    let clearance = CONSTRAINT_CLEARANCE * 2.0 / divisions as f64;
    let segments: Vec<([f64; 2], [f64; 2])> = constraints
        .iter()
        .map(|&(a, b)| (points[a].0, points[b].0))
        .collect();
    let mut interior: Vec<[f64; 2]> = Vec::new();
    for &s in &grid {
        for &t in &grid {
            let uv = [s, t];
            if segments
                .iter()
                .any(|(a, b)| distance_to_segment(uv, *a, *b) < clearance)
            {
                continue;
            }
            interior.push(uv);
        }
    }

    let mut triangulation = ConstrainedDelaunayTriangulation::<AtlasPoint>::new();
    let mut handles = Vec::with_capacity(points.len());
    for (slot, (uv, _)) in points.iter().enumerate() {
        let handle = triangulation
            .insert(AtlasPoint {
                position: Point2::new(uv[0], uv[1]),
                authored: slot,
            })
            .map_err(|error| format!("sphere atlas: chart insert failed: {error:?}"))?;
        if handle.index() != slot {
            return Ok(refuse(
                &format!(
                    "point {slot} at ({:.15}, {:.15}) coincided with an existing vertex",
                    points[slot].0[0], points[slot].0[1]
                ),
                &points,
                ring.len(),
            ));
        }
        handles.push(handle);
    }
    for &(a, b) in &constraints {
        let added = triangulation.try_add_constraint(handles[a], handles[b]);
        if added.len() > 1 || !triangulation.exists_constraint(handles[a], handles[b]) {
            return Ok(refuse(
                &format!(
                    "constraint slot {a}-{b} ({:.15},{:.15})-({:.15},{:.15}) became {} edges",
                    points[a].0[0], points[a].0[1], points[b].0[0], points[b].0[1],
                    added.len()
                ),
                &points,
                ring.len(),
            ));
        }
    }
    for uv in interior {
        insert_free_point(&mut triangulation, atlas, chart, uv, &mut points, vertices)?;
    }

    // Refine until the chord sag of every triangle is under tolerance, or the
    // pass budget runs out.  Only interior points are ever added, so no boundary
    // and no constraint is touched.
    for _ in 0..MAX_REFINE_PASSES {
        let mut additions: Vec<[f64; 2]> = Vec::new();
        for face in triangulation.inner_faces() {
            let slots = face.vertices().map(|vertex| vertex.data().authored);
            let uv = slots.map(|slot| points[slot].0);
            let corners: [Vec3; 3] = [
                vertices.positions[points[slots[0]].1],
                vertices.positions[points[slots[1]].1],
                vertices.positions[points[slots[2]].1],
            ];
            if triangle_sag(atlas, corners) <= chord_tolerance {
                continue;
            }
            let centre = [
                (uv[0][0] + uv[1][0] + uv[2][0]) / 3.0,
                (uv[0][1] + uv[1][1] + uv[2][1]) / 3.0,
            ];
            if segments
                .iter()
                .any(|(a, b)| distance_to_segment(centre, *a, *b) < 1e-9)
            {
                continue;
            }
            additions.push(centre);
        }
        if additions.is_empty() {
            break;
        }
        for uv in additions {
            insert_free_point(&mut triangulation, atlas, chart, uv, &mut points, vertices)?;
        }
    }

    let mut triangles = Vec::new();
    for face in triangulation.inner_faces() {
        let slots = face.vertices().map(|vertex| vertex.data().authored);
        let uv = slots.map(|slot| points[slot].0);
        let mut corners = [
            points[slots[0]].1,
            points[slots[1]].1,
            points[slots[2]].1,
        ];
        if corners[0] == corners[1] || corners[1] == corners[2] || corners[2] == corners[0] {
            continue;
        }
        // Counter-clockwise in (s, t), which is outward on the sphere.
        let area = (uv[1][0] - uv[0][0]) * (uv[2][1] - uv[0][1])
            - (uv[2][0] - uv[0][0]) * (uv[1][1] - uv[0][1]);
        if area < 0.0 {
            corners.swap(1, 2);
        }
        triangles.push(corners);
    }
    Ok(Some(triangles))
}


/// Insert an unconstrained interior point, keeping `points` and the
/// triangulation's vertex indexing in lockstep.
///
/// `spade::insert` on a position that already carries a vertex returns the
/// EXISTING handle and overwrites its payload — which would silently repoint an
/// already-placed vertex at the wrong chart coordinate.  Growth of the vertex
/// count is the only reliable signal that a new vertex was actually made, so the
/// insertion is rolled back whenever the count does not move.
fn insert_free_point(
    triangulation: &mut spade::ConstrainedDelaunayTriangulation<AtlasPoint>,
    atlas: &SphereAtlas,
    chart: usize,
    uv: [f64; 2],
    points: &mut Vec<([f64; 2], usize)>,
    vertices: &mut AtlasVertices,
) -> Result<(), String> {
    use spade::{Point2, Triangulation};
    let before = triangulation.num_vertices();
    let slot = points.len();
    let inserted = triangulation.insert(AtlasPoint {
        position: Point2::new(uv[0], uv[1]),
        authored: slot,
    });
    if inserted.is_err() || triangulation.num_vertices() != before + 1 {
        return Ok(());
    }
    points.push((uv, vertices.push(atlas.point(chart, uv[0], uv[1])?)));
    Ok(())
}

#[derive(Clone, Copy)]
struct AtlasPoint {
    position: spade::Point2<f64>,
    authored: usize,
}

impl spade::HasPosition for AtlasPoint {
    type Scalar = f64;
    fn position(&self) -> spade::Point2<f64> {
        self.position
    }
}

/// How far the sphere bulges from the plane of a triangle whose three corners lie
/// on it — the chord sag the tolerance is stated against.
fn triangle_sag(atlas: &SphereAtlas, corners: [Vec3; 3]) -> f64 {
    let normal = corners[1]
        .sub(corners[0])
        .cross(corners[2].sub(corners[0]));
    let Ok(unit) = normal.normalized() else {
        return 0.0;
    };
    let distance = corners[0].sub(atlas.centre).dot(unit).abs();
    (atlas.radius - distance).max(0.0)
}

fn distance_to_segment(point: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let length = dx * dx + dy * dy;
    if length <= 0.0 {
        return ((point[0] - a[0]).powi(2) + (point[1] - a[1]).powi(2)).sqrt();
    }
    let t = (((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length).clamp(0.0, 1.0);
    ((point[0] - a[0] - t * dx).powi(2) + (point[1] - a[1] - t * dy).powi(2)).sqrt()
}

/// Which triangles are material.
///
/// Flooding first and classifying second means the solid-angle test runs a few
/// times per REGION rather than once per triangle, and — more importantly — that
/// the answer is constant across a region by construction, so a triangle whose
/// centroid happens to sit close to a trim curve cannot come out different from
/// its neighbours.  A region whose probes disagree is classified triangle by
/// triangle instead of trusting a vote.
fn material_regions(
    triangles: &[[usize; 3]],
    blocking: &HashSet<(usize, usize)>,
    vertices: &AtlasVertices,
    atlas: &SphereAtlas,
    region: &SphericalRegion,
) -> Vec<bool> {
    if region.is_whole_sphere() {
        return vec![true; triangles.len()];
    }
    let mut adjacency: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (index, triangle) in triangles.iter().enumerate() {
        for (a, b) in [(0, 1), (1, 2), (2, 0)] {
            let pair = ordered(triangle[a], triangle[b]);
            if blocking.contains(&pair) {
                continue;
            }
            adjacency.entry(pair).or_default().push(index);
        }
    }
    let centroid = |triangle: &[usize; 3]| -> Vec3 {
        vertices.positions[triangle[0]]
            .add(vertices.positions[triangle[1]])
            .add(vertices.positions[triangle[2]])
            .scale(1.0 / 3.0)
    };
    let mut label = vec![usize::MAX; triangles.len()];
    let mut regions: Vec<Vec<usize>> = Vec::new();
    for seed in 0..triangles.len() {
        if label[seed] != usize::MAX {
            continue;
        }
        let id = regions.len();
        let mut members = Vec::new();
        let mut queue = vec![seed];
        label[seed] = id;
        while let Some(current) = queue.pop() {
            members.push(current);
            for (a, b) in [(0, 1), (1, 2), (2, 0)] {
                let pair = ordered(triangles[current][a], triangles[current][b]);
                let Some(neighbours) = adjacency.get(&pair) else {
                    continue;
                };
                for &neighbour in neighbours {
                    if label[neighbour] == usize::MAX {
                        label[neighbour] = id;
                        queue.push(neighbour);
                    }
                }
            }
        }
        regions.push(members);
    }
    if std::env::var("BREP_DEBUG_SPHERE_CHARTS").is_ok() {
        let mut present: HashSet<(usize, usize)> = HashSet::new();
        for triangle in triangles {
            for (a, b) in [(0, 1), (1, 2), (2, 0)] {
                present.insert(ordered(triangle[a], triangle[b]));
            }
        }
        let gaps: Vec<&(usize, usize)> =
            blocking.iter().filter(|pair| !present.contains(pair)).collect();
        eprintln!(
            "[sphere-atlas]   regions={} sizes={:?} blocking={} gaps={}",
            regions.len(),
            regions.iter().map(Vec::len).take(8).collect::<Vec<_>>(),
            blocking.len(),
            gaps.len()
        );
        for pair in gaps.iter().take(6) {
            eprintln!(
                "[sphere-atlas]   gap {:?} -> {:?}",
                vertices.positions[pair.0], vertices.positions[pair.1]
            );
        }
    }
    let debug = std::env::var("BREP_DEBUG_SPHERE_CHARTS").is_ok();
    let mut keep = vec![false; triangles.len()];
    for members in &regions {
        // Probe the members whose classification is least marginal first, and
        // require them to agree before letting them speak for the region.
        // Spread the probes across the region rather than taking a prefix of the
        // flood order, which is a contiguous patch. Do NOT rank them by
        // separation: a probe whose crossing count is wrong because its path
        // grazed a vertex has the LOWEST separation, so ranking by it selects
        // exactly the probes that cannot be trusted.
        let stride = (members.len() / 8).max(1);
        let probes: Vec<(usize, bool)> = members
            .iter()
            .step_by(stride)
            .take(16)
            .map(|&index| {
                let point = centroid(&triangles[index]);
                (
                    region.separation(atlas.centre, point),
                    region.contains(atlas.centre, point),
                )
            })
            .collect();
        let sample: Vec<bool> = probes.iter().map(|(_, inside)| *inside).collect();
        if debug {
            eprintln!(
                "[sphere-atlas]   region size={} best separation={:?} probes={:?}",
                members.len(),
                probes.first().map(|(separation, _)| *separation),
                sample
            );
        }
        let unanimous = sample.first().map(|first| sample.iter().all(|value| value == first));
        match unanimous {
            Some(true) => {
                let inside = sample[0];
                for &index in members {
                    keep[index] = inside;
                }
            }
            _ => {
                for &index in members {
                    keep[index] = region.contains(atlas.centre, centroid(&triangles[index]));
                }
            }
        }
    }
    keep
}
