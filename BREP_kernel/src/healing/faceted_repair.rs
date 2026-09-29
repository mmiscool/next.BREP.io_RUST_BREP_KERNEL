use crate::mesh_weld::{edge_key, weld_vertex};
use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{make_line, make_plane, tessellate_face, Mesh, TessellationOptions, Vec3};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

/// Convert an open exact assembly to a closed, welded triangular BREP.
///
/// This is a last-resort native recovery for self-intersection transition
/// values where carrier selection leaves an otherwise useful surface mesh
/// with boundary cycles. Exact assemblies never pass through this function.
pub(crate) fn close_as_faceted_brep(
    solid: &BrepSolid,
    tolerance: f64,
) -> Result<BrepSolid, String> {
    let options = TessellationOptions {
        slabs_per_span_u: 12,
        steps_per_span_v: 12,
    };
    let mut mesh = Mesh::default();
    let mut face_id = 0;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if let Ok(face_mesh) = tessellate_face(face, options, face_id) {
            let base = (mesh.positions.len() / 3) as u32;
            mesh.positions.extend(face_mesh.positions);
            mesh.normals.extend(face_mesh.normals);
            mesh.indices
                .extend(face_mesh.indices.into_iter().map(|index| index + base));
            mesh.face_ids.extend(face_mesh.face_ids);
        }
        face_id += 1;
    }
    if mesh.indices.is_empty() {
        return Err("faceted shell repair could not tessellate any faces".into());
    }
    triangle_soup_to_faceted_brep(&mesh.positions, Some(&mesh.indices), tolerance.max(2e-3))
}

/// Build a faceted (planar-triangle) BREP solid from a triangle soup or an
/// indexed triangle mesh — the STL/mesh import entry (§8.6 inverse: mesh →
/// body). Vertices weld within `tolerance` (pass `<= 0` to derive it from the
/// bounding-box diagonal), duplicate and degenerate triangles are dropped,
/// surplus sheets at non-manifold edges are pruned, and remaining boundary
/// cycles are capped with a fan so the result closes watertight whenever the
/// input is close to a manifold. Validation is authoritative: an input too
/// broken to close returns an error, never a silently-invalid solid.
pub fn mesh_to_faceted_brep(
    positions: &[f64],
    indices: Option<&[u32]>,
    tolerance: f64,
) -> Result<BrepSolid, String> {
    if positions.is_empty() || positions.len() % 3 != 0 {
        return Err("mesh_to_faceted_brep: positions must be xyz triples".into());
    }
    let weld_tolerance = derived_weld_tolerance(positions, tolerance);
    triangle_soup_to_faceted_brep(positions, indices, weld_tolerance)
}

/// A non-positive request derives the weld distance from the bounding-box
/// diagonal, so the mesh repair and anything rebuilt from its output share
/// one vertex identity.
fn derived_weld_tolerance(positions: &[f64], tolerance: f64) -> f64 {
    if tolerance > 0.0 && tolerance.is_finite() {
        return tolerance;
    }
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for point in positions.chunks_exact(3) {
        for axis in 0..3 {
            low[axis] = low[axis].min(point[axis]);
            high[axis] = high[axis].max(point[axis]);
        }
    }
    let diagonal = (0..3)
        .map(|axis| (high[axis] - low[axis]).powi(2))
        .sum::<f64>()
        .sqrt();
    (diagonal * 1e-6).max(1e-12)
}

/// What [`repair_triangle_soup`] had to change to close the input.
///
/// Every count is of WELDED triangles, so a mesh whose only defect is
/// duplicated vertices reports zeroes: welding alone made it a closed
/// two-manifold.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MeshRepairReport {
    /// Vertices after the weld.
    pub welded_vertices: usize,
    /// Triangles after the repair.
    pub triangles: usize,
    /// Welded triangles dropped as degenerate (two corners welded together)
    /// or as a duplicate of a triangle already kept.
    pub dropped_triangles: usize,
    /// Surplus coincident sheets pruned at edges used by three or more
    /// triangles.
    pub pruned_triangles: usize,
    /// Triangles flipped so every edge-connected patch is wound one way
    /// (an inverted end cap, say), and the patch encloses positive volume.
    pub reoriented_triangles: usize,
    /// Boundary cycles closed with a fan.
    pub capped_loops: usize,
    /// Triangles the caps added.
    pub cap_triangles: usize,
}

impl MeshRepairReport {
    /// Whether the repair changed the welded triangle set at all.
    pub fn changed(&self) -> bool {
        self.dropped_triangles != 0
            || self.pruned_triangles != 0
            || self.reoriented_triangles != 0
            || self.capped_loops != 0
    }
}

/// The mesh half of [`mesh_to_faceted_brep`]: weld, drop degenerate and
/// duplicate triangles, prune surplus sheets at non-manifold edges, and cap
/// the remaining boundary cycles with a fan — returning the repaired triangle
/// soup instead of a BREP.
///
/// This is what lets a defective mesh still reach the ANALYTIC import lanes.
/// They all require a closed two-manifold, and the defect that costs a part
/// every recognized cylinder is usually a handful of triangles far away from
/// any of them; running them on this output instead of on the raw file keeps
/// the surfaces a repairable mesh can still support. Pass `weld_tolerance`
/// `<= 0` to derive it from the bounding-box diagonal. The result is not
/// guaranteed closed — an input too broken to close comes back as whatever
/// the repair could reach, and the caller must re-test it.
pub fn repair_triangle_soup(
    positions: &[f64],
    indices: Option<&[u32]>,
    weld_tolerance: f64,
) -> Result<(Vec<f64>, Vec<u32>, MeshRepairReport), String> {
    if positions.is_empty() || !positions.len().is_multiple_of(3) {
        return Err("repair_triangle_soup: positions must be xyz triples".into());
    }
    let weld_tolerance = derived_weld_tolerance(positions, weld_tolerance);
    let (points, triangles, report) = repair_soup(positions, indices, weld_tolerance)?;
    let flat = points
        .iter()
        .flat_map(|point| [point.x, point.y, point.z])
        .collect::<Vec<_>>();
    let flat_indices = triangles
        .iter()
        .flatten()
        .map(|&index| index as u32)
        .collect::<Vec<_>>();
    Ok((flat, flat_indices, report))
}

/// Shared weld → prune → cap → build pipeline. `indices` of `None` treats
/// `positions` as a raw soup (three consecutive vertices per triangle, the
/// binary-STL layout).
fn triangle_soup_to_faceted_brep(
    positions: &[f64],
    indices: Option<&[u32]>,
    weld_tolerance: f64,
) -> Result<BrepSolid, String> {
    let (points, triangles, _) = repair_soup(positions, indices, weld_tolerance)?;
    build_faceted_brep(&points, &triangles)
}

/// The weld → dedup → prune → cap phase, shared by the BREP builder above and
/// by the public [`repair_triangle_soup`].
fn repair_soup(
    positions: &[f64],
    indices: Option<&[u32]>,
    weld_tolerance: f64,
) -> Result<(Vec<Vec3>, Vec<[usize; 3]>, MeshRepairReport), String> {
    let mut report = MeshRepairReport::default();
    let mut points = Vec::<Vec3>::new();
    let mut buckets = HashMap::<[i64; 3], Vec<usize>>::default();
    let mut mesh_to_welded = Vec::with_capacity(positions.len() / 3);
    for position in positions.chunks_exact(3) {
        mesh_to_welded.push(weld_vertex(
            Vec3::new(position[0], position[1], position[2]),
            weld_tolerance,
            &mut points,
            &mut buckets,
        ));
    }
    let sequential;
    let indices = match indices {
        Some(indices) => indices,
        None => {
            sequential = (0..mesh_to_welded.len() as u32).collect::<Vec<_>>();
            &sequential
        }
    };
    let mut triangles = Vec::<[usize; 3]>::new();
    let mut seen = HashSet::<[usize; 3]>::default();
    for triangle in indices.chunks_exact(3) {
        if triangle
            .iter()
            .any(|index| *index as usize >= mesh_to_welded.len())
        {
            return Err("mesh_to_faceted_brep: triangle index outside positions".into());
        }
        let value = [
            mesh_to_welded[triangle[0] as usize],
            mesh_to_welded[triangle[1] as usize],
            mesh_to_welded[triangle[2] as usize],
        ];
        if value[0] == value[1] || value[1] == value[2] || value[2] == value[0] {
            report.dropped_triangles += 1;
            continue;
        }
        let mut key = value;
        key.sort_unstable();
        if seen.insert(key) {
            triangles.push(value);
        } else {
            report.dropped_triangles += 1;
        }
    }
    if triangles.is_empty() {
        return Err("mesh_to_faceted_brep: no non-degenerate triangles".into());
    }

    // Remove surplus coincident sheets at non-manifold triangle edges,
    // retaining one use in each direction whenever possible.
    loop {
        let mut edge_uses = HashMap::<(usize, usize), Vec<(usize, bool)>>::default();
        for (triangle_index, triangle) in triangles.iter().enumerate() {
            for side in 0..3 {
                let first = triangle[side];
                let second = triangle[(side + 1) % 3];
                edge_uses
                    .entry(edge_key(first, second))
                    .or_default()
                    .push((triangle_index, first < second));
            }
        }
        let Some(uses) = edge_uses.values().find(|uses| uses.len() > 2) else {
            break;
        };
        let keep_forward = uses.iter().find(|(_, forward)| *forward).map(|use_| use_.0);
        let keep_reverse = uses
            .iter()
            .find(|(_, forward)| !*forward)
            .map(|use_| use_.0);
        let keep = [keep_forward, keep_reverse]
            .into_iter()
            .flatten()
            .collect::<HashSet<_>>();
        let remove = uses
            .iter()
            .map(|use_| use_.0)
            .filter(|index| !keep.contains(index))
            .collect::<HashSet<_>>();
        if remove.is_empty() {
            break;
        }
        report.pruned_triangles += remove.len();
        triangles = triangles
            .into_iter()
            .enumerate()
            .filter_map(|(index, triangle)| (!remove.contains(&index)).then_some(triangle))
            .collect();
    }

    report.reoriented_triangles = orient_consistently(&points, &mut triangles)?;

    let mut directed_boundary = Vec::<(usize, usize)>::new();
    let mut counts = HashMap::<(usize, usize), usize>::default();
    for triangle in &triangles {
        for side in 0..3 {
            *counts
                .entry(edge_key(triangle[side], triangle[(side + 1) % 3]))
                .or_default() += 1;
        }
    }
    for triangle in &triangles {
        for side in 0..3 {
            let first = triangle[side];
            let second = triangle[(side + 1) % 3];
            if counts[&edge_key(first, second)] == 1 {
                directed_boundary.push((first, second));
            }
        }
    }
    while let Some((start, mut end)) = directed_boundary.pop() {
        let mut loop_vertices = vec![start, end];
        while end != start {
            let Some(index) = directed_boundary
                .iter()
                .position(|(candidate_start, _)| *candidate_start == end)
            else {
                break;
            };
            let (_, next) = directed_boundary.swap_remove(index);
            end = next;
            if end != start {
                loop_vertices.push(end);
            }
            if loop_vertices.len() > points.len() + 1 {
                break;
            }
        }
        if end != start || loop_vertices.len() < 3 {
            continue;
        }
        let center = loop_vertices
            .iter()
            .fold(Vec3::default(), |sum, index| sum.add(points[*index]))
            .scale(1.0 / loop_vertices.len() as f64);
        let center_index = points.len();
        points.push(center);
        for index in 0..loop_vertices.len() {
            let first = loop_vertices[index];
            let second = loop_vertices[(index + 1) % loop_vertices.len()];
            // Reverse the existing boundary direction on the new cap.
            triangles.push([second, first, center_index]);
        }
        report.capped_loops += 1;
        report.cap_triangles += loop_vertices.len();
    }

    report.welded_vertices = points.len();
    report.triangles = triangles.len();
    Ok((points, triangles, report))
}

/// Make the winding agree across every two-use edge, and return how many
/// triangles were flipped.
///
/// A shell exported with a patch wound the wrong way (a pipe whose end caps
/// face inward) reaches here with edges used TWICE in the SAME direction:
/// neither the sheet prune (three or more uses) nor the cap (one use) sees
/// them, and the builder's validator refuses the result with "coedges with the
/// same sense". Each edge-connected patch is walked breadth-first from its
/// lowest triangle, flipping a neighbour whose shared edge runs the same way.
/// A patch that needed any flip then takes the sign that makes its enclosed
/// volume positive (measured about its own centroid, so an open patch reads
/// the same wherever it sits), so the repaired shell ends outward whichever
/// side the walk started on. A patch that was already consistent is left
/// exactly as it came — its winding, inward or not, is the file's.
///
/// A patch whose walk meets itself with the opposite parity (a Möbius band,
/// or a shell glued to itself) has no consistent winding; it is refused by
/// name rather than guessed.
fn orient_consistently(points: &[Vec3], triangles: &mut [[usize; 3]]) -> Result<usize, String> {
    let mut edge_uses = HashMap::<(usize, usize), Vec<usize>>::default();
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        for side in 0..3 {
            edge_uses
                .entry(edge_key(triangle[side], triangle[(side + 1) % 3]))
                .or_default()
                .push(triangle_index);
        }
    }
    let runs_forward = |triangle: &[usize; 3], first: usize, second: usize| {
        (0..3).any(|side| triangle[side] == first && triangle[(side + 1) % 3] == second)
    };

    let mut flip = vec![None::<bool>; triangles.len()];
    let mut flipped = 0;
    for seed in 0..triangles.len() {
        if flip[seed].is_some() {
            continue;
        }
        flip[seed] = Some(false);
        let mut patch = vec![seed];
        let mut queue = std::collections::VecDeque::from([seed]);
        while let Some(current) = queue.pop_front() {
            let current_flip = flip[current] == Some(true);
            for side in 0..3 {
                let first = triangles[current][side];
                let second = triangles[current][(side + 1) % 3];
                let uses = &edge_uses[&edge_key(first, second)];
                if uses.len() != 2 {
                    continue;
                }
                let neighbour = if uses[0] == current { uses[1] } else { uses[0] };
                // After its own flip, `current` runs first→second exactly when
                // it is not flipped; the neighbour must then run second→first.
                let neighbour_same_way = runs_forward(&triangles[neighbour], first, second);
                let wanted = neighbour_same_way != current_flip;
                match flip[neighbour] {
                    None => {
                        flip[neighbour] = Some(wanted);
                        patch.push(neighbour);
                        queue.push_back(neighbour);
                    }
                    Some(assigned) if assigned != wanted => {
                        return Err(format!(
                            "mesh_to_faceted_brep: the shell through triangle {seed} cannot be \
                             consistently oriented (non-orientable at the edge ({}, {}, {})–({}, {}, {}))",
                            points[first].x, points[first].y, points[first].z,
                            points[second].x, points[second].y, points[second].z
                        ));
                    }
                    Some(_) => {}
                }
            }
        }
        if patch.iter().all(|&index| flip[index] == Some(false)) {
            continue;
        }
        for &index in &patch {
            if flip[index] == Some(true) {
                triangles[index].swap(1, 2);
            }
        }
        let centroid = patch
            .iter()
            .flat_map(|&index| triangles[index])
            .fold(Vec3::default(), |sum, corner| sum.add(points[corner]))
            .scale(1.0 / (3 * patch.len()) as f64);
        let volume = patch
            .iter()
            .map(|&index| {
                let [a, b, c] = triangles[index].map(|corner| points[corner].sub(centroid));
                a.dot(b.cross(c))
            })
            .sum::<f64>();
        if volume < 0.0 {
            for &index in &patch {
                triangles[index].swap(1, 2);
            }
            flipped += patch.iter().filter(|&&index| flip[index] == Some(false)).count();
        } else {
            flipped += patch.iter().filter(|&&index| flip[index] == Some(true)).count();
        }
    }
    Ok(flipped)
}

/// One planar face per repaired triangle, validated.
fn build_faceted_brep(points: &[Vec3], triangles: &[[usize; 3]]) -> Result<BrepSolid, String> {
    let vertices = points
        .iter()
        .enumerate()
        .map(|(index, point)| VertexRecord {
            id: index as u64 + 1,
            point: *point,
        })
        .collect::<Vec<_>>();
    let mut edges = Vec::<EdgeRecord>::new();
    let mut edge_ids = HashMap::<(usize, usize), u64>::default();
    let mut faces = Vec::<FaceRecord>::new();
    let mut next_id = vertices.len() as u64 + 1;
    for triangle in triangles.iter().copied() {
        let a = points[triangle[0]];
        let b = points[triangle[1]];
        let c = points[triangle[2]];
        let u = b.sub(a);
        let u_length = u.length();
        if u_length <= 1e-10 {
            continue;
        }
        let u_direction = u.scale(1.0 / u_length);
        let c_delta = c.sub(a);
        let c_u = c_delta.dot(u_direction);
        let v = c_delta.sub(u_direction.scale(c_u));
        let v_length = v.length();
        if v_length <= 1e-10 {
            continue;
        }
        let v_direction = v.scale(1.0 / v_length);
        // An OBTUSE triangle puts the apex's u coordinate outside [0, |ab|];
        // evaluation clamps to the surface domain, so the plane must span the
        // triangle's full uv bounding box or the apex edges' pcurves map off
        // the triangle (sphere tessellations expose this; boxes never do).
        let u_low = c_u.min(0.0);
        let u_high = c_u.max(u_length);
        let surface = make_plane(
            a.add(u_direction.scale(u_low)),
            u_direction,
            v_direction,
            u_high - u_low,
            v_length,
        )?;
        let uv = [
            Vec3::new(-u_low, 0.0, 0.0),
            Vec3::new(u_length - u_low, 0.0, 0.0),
            Vec3::new(c_u - u_low, v_length, 0.0),
        ];
        let mut coedges = Vec::with_capacity(3);
        for side in 0..3 {
            let first = triangle[side];
            let second = triangle[(side + 1) % 3];
            let key = edge_key(first, second);
            let edge_id = if let Some(id) = edge_ids.get(&key) {
                *id
            } else {
                let id = next_id;
                next_id += 1;
                edges.push(EdgeRecord {
                    id,
                    curve: make_line(points[key.0], points[key.1])?,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: key.0 as u64 + 1,
                    end_vertex_id: key.1 as u64 + 1,
                    degenerate: false,
                    name: None,
                });
                edge_ids.insert(key, id);
                id
            };
            coedges.push(CoedgeRecord {
                id: next_id,
                edge_id,
                forward: first < second,
                pcurve: make_line(uv[side], uv[(side + 1) % 3])?,
            });
            next_id += 1;
        }
        let loop_record = LoopRecord {
            id: next_id,
            coedges,
        };
        next_id += 1;
        faces.push(FaceRecord {
            id: next_id,
            surface,
            same_sense: true,
            loops: vec![loop_record],
            name: None,
        });
        next_id += 1;
    }
    let mut result = BrepSolid {
        mass_properties_cache: Default::default(),
        id: next_id + 1,
        vertices,
        edges,
        shells: vec![ShellRecord { id: next_id, faces }],
        genus: 0,
    };
    let vertex_count = result.vertices.len() as i64;
    let edge_count = result.edges.len() as i64;
    let face_count = result.shells[0].faces.len() as i64;
    let numerator = 2 - (vertex_count - edge_count + face_count);
    if numerator >= 0 && numerator % 2 == 0 {
        result.genus = numerator / 2;
    }
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(format!(
            "faceted shell repair produced invalid topology: {issues:?}"
        ));
    }
    for (index, face) in result.shells[0].faces.iter().enumerate() {
        tessellate_face(face, TessellationOptions::default(), index as u32)
            .map_err(|error| format!("faceted face {index} cannot be tessellated: {error}"))?;
    }
    Ok(result)
}

