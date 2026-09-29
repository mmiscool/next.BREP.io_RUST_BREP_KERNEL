use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
use super::builder::snap_edge_curve_endpoints;
use super::edge_conform::edge_lies_on;

/// Arrangement intersections occasionally leave a microscopic one-use edge
/// between two otherwise coincident junction vertices. Collapse that edge as
/// a tolerance-level topological sliver, refitting incident edge endpoints so
/// the healed BREP remains geometrically consistent (including in STEP).
/// A vertex where exactly three faces meet is the solution of
/// r1(u,v) = r2(a,b) = r3(x,y).  Solving that 6-equation system JOINTLY
/// (Golovanov §4.5) removes the drift that composing pairwise
/// intersection-curve endpoints accumulates — the source of
/// curve-end-vs-vertex gaps.  A vertex is only moved when the joint
/// Newton converges, lands within the weld tolerance of the current
/// position, and does not worsen the worst adjacent edge-endpoint gap;
/// every adjacent edge endpoint is then re-snapped to the solved point.
pub(in crate::boolean) fn polish_triple_junction_vertices(solid: &mut BrepSolid, tolerance: f64) -> Result<usize, KernelRefusal> {
    let weld_tolerance = assembler_weld(tolerance);
    let mut vertex_faces = HashMap::<u64, Vec<usize>>::default();
    let edge_vertices: HashMap<u64, (u64, u64)> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, (edge.start_vertex_id, edge.end_vertex_id)))
        .collect();
    let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    for (face_index, face) in faces.iter().enumerate() {
        for loop_record in &face.loops {
            for coedge in &loop_record.coedges {
                let Some(&(start, end)) = edge_vertices.get(&coedge.edge_id) else {
                    continue;
                };
                for vertex_id in [start, end] {
                    let entry = vertex_faces.entry(vertex_id).or_default();
                    if !entry.contains(&face_index) {
                        entry.push(face_index);
                    }
                }
            }
        }
    }
    let mut moves = Vec::<(u64, Vec3)>::new();
    for vertex in &solid.vertices {
        let Some(adjacent) = vertex_faces.get(&vertex.id) else {
            continue;
        };
        if adjacent.len() != 3 {
            continue;
        }
        let surfaces = [
            &faces[adjacent[0]].surface,
            &faces[adjacent[1]].surface,
            &faces[adjacent[2]].surface,
        ];
        // Seed each surface's parameters by projecting the current vertex.
        let mut seeds = [[0.0f64; 2]; 3];
        let mut normals = [Vec3::default(); 3];
        let mut seed_failed = false;
        for index in 0..3 {
            match crate::project_point_to_surface(surfaces[index], vertex.point) {
                Ok(projection) if projection.distance <= weld_tolerance * 10.0 => {
                    seeds[index] = [projection.u, projection.v];
                    match surfaces[index].normal(projection.u, projection.v) {
                        Ok(normal) => normals[index] = normal,
                        Err(_) => seed_failed = true,
                    }
                }
                _ => seed_failed = true,
            }
        }
        if seed_failed {
            continue;
        }
        // Near-tangent surface pairs make the joint system singular; the
        // pairwise construction is as good as it gets there.
        if normals[0].cross(normals[1]).length() <= 1e-3
            || normals[1].cross(normals[2]).length() <= 1e-3
            || normals[0].cross(normals[2]).length() <= 1e-3
        {
            continue;
        }
        let mut parameters = seeds;
        let mut solved = None;
        for _ in 0..25 {
            let d0 = surfaces[0].derivatives(parameters[0][0], parameters[0][1], 1).or_refuse(KernelStage::Sew, "derivatives")?;
            let d1 = surfaces[1].derivatives(parameters[1][0], parameters[1][1], 1).or_refuse(KernelStage::Sew, "derivatives")?;
            let d2 = surfaces[2].derivatives(parameters[2][0], parameters[2][1], 1).or_refuse(KernelStage::Sew, "derivatives")?;
            let gap02 = d0[0][0].sub(d2[0][0]);
            let gap12 = d1[0][0].sub(d2[0][0]);
            if gap02.length() <= tolerance && gap12.length() <= tolerance {
                solved = Some(d0[0][0].add(d1[0][0]).add(d2[0][0]).scale(1.0 / 3.0));
                break;
            }
            // Rows 0-2: s0(u,v) - s2(x,y); rows 3-5: s1(a,b) - s2(x,y).
            let mut matrix = [[0.0f64; 6]; 6];
            let columns0 = [d0[1][0], d0[0][1]];
            let columns1 = [d1[1][0], d1[0][1]];
            let columns2 = [d2[1][0], d2[0][1]];
            for row in 0..3 {
                let pick = |vector: Vec3| [vector.x, vector.y, vector.z][row];
                matrix[row][0] = pick(columns0[0]);
                matrix[row][1] = pick(columns0[1]);
                matrix[row][4] = -pick(columns2[0]);
                matrix[row][5] = -pick(columns2[1]);
                matrix[row + 3][2] = pick(columns1[0]);
                matrix[row + 3][3] = pick(columns1[1]);
                matrix[row + 3][4] = -pick(columns2[0]);
                matrix[row + 3][5] = -pick(columns2[1]);
            }
            let rhs = [-gap02.x, -gap02.y, -gap02.z, -gap12.x, -gap12.y, -gap12.z];
            let Ok(step) = crate::fit::solve_small(matrix, rhs, 6) else {
                break;
            };
            let limit = weld_tolerance * 1e3;
            for (slot, delta) in [
                (0usize, [step[0], step[1]]),
                (1, [step[2], step[3]]),
                (2, [step[4], step[5]]),
            ] {
                parameters[slot][0] += delta[0].clamp(-limit, limit);
                parameters[slot][1] += delta[1].clamp(-limit, limit);
            }
        }
        let Some(solved_point) = solved else {
            continue;
        };
        if solved_point.sub(vertex.point).length() > weld_tolerance
            || solved_point.sub(vertex.point).length() <= 1e-12
        {
            continue;
        }
        // Worst adjacent edge-endpoint gap must not get worse.
        let endpoint_gap = |target: Vec3| -> f64 {
            let mut worst = 0.0f64;
            for edge in &solid.edges {
                if edge.start_vertex_id == vertex.id {
                    if let Ok(point) = edge.curve.evaluate(edge.t0) {
                        worst = worst.max(point.sub(target).length());
                    }
                }
                if edge.end_vertex_id == vertex.id {
                    if let Ok(point) = edge.curve.evaluate(edge.t1) {
                        worst = worst.max(point.sub(target).length());
                    }
                }
            }
            worst
        };
        if endpoint_gap(solved_point) >= endpoint_gap(vertex.point) {
            continue;
        }
        moves.push((vertex.id, solved_point));
    }
    let moved = moves.len();
    for (vertex_id, point) in moves {
        if let Some(vertex) = solid
            .vertices
            .iter_mut()
            .find(|vertex| vertex.id == vertex_id)
        {
            vertex.point = point;
        }
        let vertex_points: HashMap<u64, Vec3> = solid
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect();
        for edge in &mut solid.edges {
            if edge.start_vertex_id != vertex_id && edge.end_vertex_id != vertex_id {
                continue;
            }
            let start = vertex_points[&edge.start_vertex_id];
            let end = vertex_points[&edge.end_vertex_id];
            let snapped =
                snap_edge_curve_endpoints(edge.curve.clone(), edge.t0, edge.t1, start, end)?;
            [edge.t0, edge.t1] = snapped.domain().or_refuse(KernelStage::Sew, "domain")?;
            edge.curve = snapped;
        }
    }
    Ok(moved)
}

pub(super) fn collapse_short_one_use_edges(solid: &mut BrepSolid, maximum_length: f64) -> Result<(), KernelRefusal> {
    loop {
        let mut uses = HashMap::<u64, usize>::default();
        for coedge in solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
        {
            *uses.entry(coedge.edge_id).or_default() += 1;
        }
        let points = solid
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect::<HashMap<_, _>>();
        // A CLOSED edge (one seam vertex) has zero endpoint separation but
        // is not short — collapsing it deletes a cap/rim loop outright.
        // Closed edges are only ever welded (assembler), never collapsed.
        let candidate = solid.edges.iter().find(|edge| {
            !edge.degenerate
                && edge.start_vertex_id != edge.end_vertex_id
                && uses.get(&edge.id) == Some(&1)
                && points[&edge.start_vertex_id]
                    .sub(points[&edge.end_vertex_id])
                    .length()
                    <= maximum_length
        });
        let Some(candidate) = candidate.cloned() else {
            break;
        };
        let kept_vertex_id = candidate.start_vertex_id;
        let removed_vertex_id = candidate.end_vertex_id;
        let midpoint = points[&kept_vertex_id]
            .add(points[&removed_vertex_id])
            .scale(0.5);

        for face in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
            for loop_record in &mut face.loops {
                loop_record
                    .coedges
                    .retain(|coedge| coedge.edge_id != candidate.id);
            }
        }
        solid.edges.retain(|edge| edge.id != candidate.id);
        for edge in &mut solid.edges {
            let move_start =
                edge.start_vertex_id == kept_vertex_id || edge.start_vertex_id == removed_vertex_id;
            let move_end =
                edge.end_vertex_id == kept_vertex_id || edge.end_vertex_id == removed_vertex_id;
            if !move_start && !move_end {
                continue;
            }
            let sample_count = 32usize.max(edge.curve.control_points.len() * 4);
            let mut samples = (0..=sample_count)
                .map(|index| {
                    let fraction = index as f64 / sample_count as f64;
                    edge.curve
                        .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)
                })
                .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Sew, "csg.boolean.assemble.polish")?;
            if move_start {
                edge.start_vertex_id = kept_vertex_id;
                samples[0] = midpoint;
            }
            if move_end {
                edge.end_vertex_id = kept_vertex_id;
                *samples.last_mut().unwrap() = midpoint;
            }
            let parameters = (0..=sample_count)
                .map(|index| index as f64 / sample_count as f64)
                .collect::<Vec<_>>();
            edge.curve =
                interpolate_curve(&samples, edge.curve.degree.min(sample_count), &parameters).or_refuse(KernelStage::Sew, "interpolate_curve")?;
            [edge.t0, edge.t1] = edge.curve.domain().or_refuse(KernelStage::Sew, "domain")?;
        }
        if let Some(vertex) = solid
            .vertices
            .iter_mut()
            .find(|vertex| vertex.id == kept_vertex_id)
        {
            vertex.point = midpoint;
        }
        solid
            .vertices
            .retain(|vertex| vertex.id != removed_vertex_id);
    }
    Ok(())
}

/// Face completion and face merging can introduce two geometrically identical
/// boundary edges after the initial assembler sewing pass. Rebind those two
/// one-use coedges to one shared edge before evaluating manifold topology.
pub(super) fn sew_coincident_one_use_edges(solid: &mut BrepSolid, tolerance: f64) -> Result<(), KernelRefusal> {
    loop {
        let mut uses = HashMap::<u64, Vec<bool>>::default();
        for coedge in solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
        {
            uses.entry(coedge.edge_id).or_default().push(coedge.forward);
        }
        let points = solid
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect::<HashMap<_, _>>();
        // Splitting-based sewing can fragment coincident closed curves into
        // many arcs. Leave those pairs to the assembler's closed-edge welds.
        let candidates = solid
            .edges
            .iter()
            .filter(|edge| {
                !edge.degenerate
                    && edge.start_vertex_id != edge.end_vertex_id
                    && uses.get(&edge.id).is_some_and(|uses| uses.len() == 1)
            })
            .cloned()
            .collect::<Vec<_>>();
        let mut replacement = None;
        'pairs: for (index, first) in candidates.iter().enumerate() {
            for second in candidates.iter().skip(index + 1) {
                let Some(reversed) = crate::topology::edge_endpoint_alignment(
                    first, second, &points, tolerance,
                ) else {
                    continue;
                };
                if !edge_lies_on(first, second, tolerance)?
                    || !edge_lies_on(second, first, tolerance)?
                {
                    continue;
                }
                let first_forward = uses[&first.id][0];
                let mapped_second_forward = if reversed {
                    !uses[&second.id][0]
                } else {
                    uses[&second.id][0]
                };
                if first_forward == mapped_second_forward {
                    continue;
                }
                replacement = Some((first.clone(), second.id, reversed));
                break 'pairs;
            }
        }
        let Some((first_edge, second_id, reversed)) = replacement else {
            break;
        };
        for face in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
            for coedge in face
                .loops
                .iter_mut()
                .flat_map(|loop_record| &mut loop_record.coedges)
            {
                if coedge.edge_id == second_id {
                    coedge.edge_id = first_edge.id;
                    if reversed {
                        coedge.forward = !coedge.forward;
                    }
                    let traversal_curve = if coedge.forward {
                        first_edge.curve.clone()
                    } else {
                        first_edge.curve.reversed().or_refuse(KernelStage::Sew, "reversed")?
                    };
                    coedge.pcurve = build_pcurve_on_surface(&face.surface, &traversal_curve).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?;
                }
            }
        }
        solid.edges.retain(|edge| edge.id != second_id);
    }
    let used_vertices = solid
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect::<HashSet<_>>();
    solid
        .vertices
        .retain(|vertex| used_vertices.contains(&vertex.id));
    Ok(())
}

