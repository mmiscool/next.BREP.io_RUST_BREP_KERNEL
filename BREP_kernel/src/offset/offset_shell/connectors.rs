use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

#[derive(Clone)]
struct OpenBoundaryUse {
    edge: EdgeRecord,
    coedge: CoedgeRecord,
    role: OffsetFaceRole,
    start: Vec3,
    end: Vec3,
}

#[derive(Clone)]
struct ConnectorBoundaryUse {
    edge: EdgeRecord,
    coedge: CoedgeRecord,
    source_face_id: u64,
    shell_index: usize,
    start: Vec3,
    end: Vec3,
}

/// Add exact planar miter faces between separated offset images of a sharp
/// source seam. Smooth seams are synchronized earlier; at a sharp seam the
/// two offset carriers legitimately end on parallel curves with a gap that
/// must be bridged before the opening-wall and closed-cap cycles can be
/// completed.
pub(super) fn complete_sharp_offset_connectors(
    solid: &mut BrepSolid,
    source: &BrepSolid,
    face_images: &[OffsetShellFaceImageRecord],
    distance: f64,
    tolerance: f64,
) -> Result<Vec<OffsetShellFaceImageRecord>, KernelRefusal> {
    let mut use_counts = HashMap::<u64, usize>::default();
    for coedge in solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
    {
        *use_counts.entry(coedge.edge_id).or_default() += 1;
    }
    let edge_by_id = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect::<HashMap<_, _>>();
    let point_by_id = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect::<HashMap<_, _>>();
    let source_faces = source
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| (face.id, face))
        .collect::<HashMap<_, _>>();
    let mut boundary = Vec::new();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for face in &shell.faces {
            let image = face_images
                .get(face.id.saturating_sub(1) as usize)
                .ok_or_else(|| "offset_shell: connector face lost provenance".to_string()).or_refuse(KernelStage::Sew, "offset_shell_connector_face_lost_provenance")?;
            if !matches!(image.role, OffsetFaceRole::Offset) {
                continue;
            }
            for coedge in face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
            {
                let edge = &edge_by_id[&coedge.edge_id];
                if edge.degenerate || use_counts.get(&edge.id) != Some(&1) {
                    continue;
                }
                let edge_start = point_by_id[&edge.start_vertex_id];
                let edge_end = point_by_id[&edge.end_vertex_id];
                boundary.push(ConnectorBoundaryUse {
                    edge: edge.clone(),
                    coedge: coedge.clone(),
                    source_face_id: image.source_face_id,
                    shell_index,
                    start: if coedge.forward { edge_start } else { edge_end },
                    end: if coedge.forward { edge_end } else { edge_start },
                });
            }
        }
    }

    let mut candidates = Vec::<(f64, usize, usize)>::new();
    let gap_limit = distance.abs() * 2.5 + tolerance;
    for first_index in 0..boundary.len() {
        let first = &boundary[first_index];
        let first_direction = first.end.sub(first.start);
        let first_length = first_direction.length();
        if first_length <= gap_limit {
            continue;
        }
        for second_index in first_index + 1..boundary.len() {
            let second = &boundary[second_index];
            if first.source_face_id == second.source_face_id {
                continue;
            }
            let (Some(first_source), Some(second_source)) = (
                source_faces.get(&first.source_face_id),
                source_faces.get(&second.source_face_id),
            ) else {
                continue;
            };
            if !source_faces_adjacent(first_source, second_source) {
                continue;
            }
            let second_direction = second.end.sub(second.start);
            let second_length = second_direction.length();
            if (first_length - second_length).abs() > tolerance * 100.0
                || first_direction
                    .normalized().or_refuse(KernelStage::Sew, "normalized")?
                    .dot(second_direction.normalized().or_refuse(KernelStage::Sew, "normalized")?)
                    > -0.999
            {
                continue;
            }
            let first_gap = first.start.sub(second.end).length();
            let second_gap = second.start.sub(first.end).length();
            if first_gap <= tolerance
                || second_gap <= tolerance
                || first_gap > gap_limit
                || second_gap > gap_limit
                || (first_gap - second_gap).abs() > tolerance * 100.0
            {
                continue;
            }
            let u = first.start.sub(first.end);
            let v = second.start.sub(first.end);
            if u.dot(v).abs() > tolerance * 100.0 * u.length() * v.length() {
                continue;
            }
            let normal = u.cross(v).normalized().or_refuse(KernelStage::Sew, "normalized")?;
            if second.end.sub(first.end).dot(normal).abs() > tolerance * 100.0 {
                continue;
            }
            candidates.push((first_gap + second_gap, first_index, second_index));
        }
    }
    candidates.sort_by(|first, second| first.0.total_cmp(&second.0));

    let mut consumed = HashSet::default();
    let mut images = Vec::new();
    let mut next_edge_id = solid.edges.iter().map(|edge| edge.id).max().unwrap_or(0) + 1;
    let mut next_face_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| face.id)
        .max()
        .unwrap_or(0)
        + 1;
    let mut next_loop_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .map(|loop_record| loop_record.id)
        .max()
        .unwrap_or(0)
        + 1;
    let mut next_coedge_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.id)
        .max()
        .unwrap_or(0)
        + 1;
    let mut shell_parent = (0..solid.shells.len()).collect::<Vec<_>>();
    fn shell_root(parent: &mut [usize], index: usize) -> usize {
        if parent[index] != index {
            parent[index] = shell_root(parent, parent[index]);
        }
        parent[index]
    }

    for (_, first_index, second_index) in candidates {
        let first = &boundary[first_index];
        let second = &boundary[second_index];
        if consumed.contains(&first.edge.id) || consumed.contains(&second.edge.id) {
            continue;
        }
        let u_vector = first.start.sub(first.end);
        let v_vector = second.start.sub(first.end);
        let surface = crate::make_plane(
            first.end,
            u_vector.normalized().or_refuse(KernelStage::Sew, "normalized")?,
            v_vector.normalized().or_refuse(KernelStage::Sew, "normalized")?,
            u_vector.length(),
            v_vector.length(),
        ).or_refuse(KernelStage::Sew, "make_plane")?;
        let first_bridge = EdgeRecord {
            id: next_edge_id,
            curve: crate::make_line(first.start, second.end).or_refuse(KernelStage::Sew, "make_line")?,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: if first.coedge.forward {
                first.edge.start_vertex_id
            } else {
                first.edge.end_vertex_id
            },
            end_vertex_id: if second.coedge.forward {
                second.edge.end_vertex_id
            } else {
                second.edge.start_vertex_id
            },
            degenerate: false,
            name: None,
        };
        next_edge_id += 1;
        let second_bridge = EdgeRecord {
            id: next_edge_id,
            curve: crate::make_line(second.start, first.end).or_refuse(KernelStage::Sew, "make_line")?,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: if second.coedge.forward {
                second.edge.start_vertex_id
            } else {
                second.edge.end_vertex_id
            },
            end_vertex_id: if first.coedge.forward {
                first.edge.end_vertex_id
            } else {
                first.edge.start_vertex_id
            },
            degenerate: false,
            name: None,
        };
        next_edge_id += 1;
        let uses = [
            (&first.edge, !first.coedge.forward),
            (&first_bridge, true),
            (&second.edge, !second.coedge.forward),
            (&second_bridge, true),
        ];
        let mut coedges = Vec::new();
        for (edge, forward) in uses {
            let pcurve = boundary_pcurve(edge, forward, &surface, tolerance)?
                .ok_or_else(|| "offset_shell: connector edge left its plane".to_string()).or_refuse(KernelStage::Sew, "offset_shell_connector_edge_left_its_plane")?;
            coedges.push(CoedgeRecord {
                id: next_coedge_id,
                edge_id: edge.id,
                forward,
                pcurve,
            });
            next_coedge_id += 1;
        }
        let mut face = FaceRecord {
            id: next_face_id,
            surface,
            same_sense: true,
            loops: vec![LoopRecord {
                id: next_loop_id,
                coedges,
            }],
            name: None,
        };
        face.same_sense = parameter_space_area(&face).or_refuse(KernelStage::Sew, "parameter_space_area")? > 0.0;
        next_face_id += 1;
        next_loop_id += 1;
        solid.edges.push(first_bridge);
        solid.edges.push(second_bridge);

        let low_shell = first.shell_index.min(second.shell_index);
        let high_shell = first.shell_index.max(second.shell_index);
        if low_shell != high_shell {
            let low_root = shell_root(&mut shell_parent, low_shell);
            let high_root = shell_root(&mut shell_parent, high_shell);
            if low_root != high_root {
                shell_parent[high_root] = low_root;
            }
        }
        solid.shells[low_shell].faces.push(face);
        consumed.insert(first.edge.id);
        consumed.insert(second.edge.id);
        images.push(OffsetShellFaceImageRecord {
            role: OffsetFaceRole::Offset,
            source_face_id: first.source_face_id,
        });
    }
    if solid.shells.len() > 1 {
        let original_shells = std::mem::take(&mut solid.shells);
        let mut merged = Vec::<ShellRecord>::new();
        let mut group_index = HashMap::<usize, usize>::default();
        for (index, shell) in original_shells.into_iter().enumerate() {
            let root = shell_root(&mut shell_parent, index);
            if let Some(target) = group_index.get(&root).copied() {
                merged[target].faces.extend(shell.faces);
            } else {
                group_index.insert(root, merged.len());
                merged.push(shell);
            }
        }
        solid.shells = merged;
    }
    Ok(images)
}

pub(super) fn edge_on_surface(
    edge: &EdgeRecord,
    surface: &crate::NurbsSurface,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    for sample in 0..=16 {
        let fraction = sample as f64 / 16.0;
        let point = edge
            .curve
            .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
        if project_point_to_surface(surface, point).or_refuse(KernelStage::Sew, "project_point_to_surface")?.distance > tolerance {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Exact parameter-space image of a curve on an AFFINE plane patch: an affine
/// map sends a rational curve to a rational curve with the SAME weights and
/// knots, so the pcurve is the control net mapped through the plane's inverse
/// frame — no fit, no fit error. Only applies when the edge spans its whole
/// curve (the transform maps the full curve).
fn affine_plane_pcurve(
    edge: &EdgeRecord,
    forward: bool,
    surface: &crate::NurbsSurface,
) -> Result<Option<crate::NurbsCurve>, KernelRefusal> {
    if !surface.is_affine().or_refuse(KernelStage::Sew, "is_affine")? {
        return Ok(None);
    }
    let [curve_t0, curve_t1] = [
        *edge.curve.knots.first().unwrap_or(&0.0),
        *edge.curve.knots.last().unwrap_or(&0.0),
    ];
    if (edge.t0 - curve_t0).abs() > 1e-12 || (edge.t1 - curve_t1).abs() > 1e-12 {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
    let origin = surface.evaluate(u0, v0).or_refuse(KernelStage::Sew, "evaluate")?;
    let u_step = surface.evaluate(u1, v0).or_refuse(KernelStage::Sew, "evaluate")?.sub(origin).scale(1.0 / (u1 - u0));
    let v_step = surface.evaluate(u0, v1).or_refuse(KernelStage::Sew, "evaluate")?.sub(origin).scale(1.0 / (v1 - v0));
    // Solve the 2x2 Gram system for in-plane coordinates.
    let uu = u_step.dot(u_step);
    let uv = u_step.dot(v_step);
    let vv = v_step.dot(v_step);
    let determinant = uu * vv - uv * uv;
    if determinant.abs() <= 1e-18 {
        return Ok(None);
    }
    let controls = edge
        .curve
        .control_points
        .iter()
        .map(|control| {
            let point = control.point().or_refuse(KernelStage::Sew, "point")?;
            let relative = point.sub(origin);
            let pu = relative.dot(u_step);
            let pv = relative.dot(v_step);
            let u = u0 + (pu * vv - pv * uv) / determinant;
            let v = v0 + (pv * uu - pu * uv) / determinant;
            Ok(crate::Vec4::from_point(Vec3::new(u, v, 0.0), control.w))
        })
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    let pcurve = crate::NurbsCurve::new(edge.curve.degree, edge.curve.knots.clone(), controls).or_refuse(KernelStage::Sew, "new")?;
    Ok(Some(if forward { pcurve } else { pcurve.reversed().or_refuse(KernelStage::Sew, "reversed")? }))
}

pub(super) fn boundary_pcurve(
    edge: &EdgeRecord,
    forward: bool,
    surface: &crate::NurbsSurface,
    tolerance: f64,
) -> Result<Option<crate::NurbsCurve>, KernelRefusal> {
    if !edge_on_surface(edge, surface, tolerance)? {
        return Ok(None);
    }
    if let Some(exact) = affine_plane_pcurve(edge, forward, surface)? {
        return Ok(Some(exact));
    }
    Ok(Some(crate::build_pcurve_on_surface_range(
        surface,
        &edge.curve,
        edge.t0,
        edge.t1,
        forward,
        tolerance,
    ).or_refuse(KernelStage::Sew, "build_pcurve_on_surface_range")?))
}

fn cycle_center(cycle: &[OpenBoundaryUse]) -> Vec3 {
    cycle
        .iter()
        .fold(Vec3::default(), |center, usage| center.add(usage.start))
        .scale(1.0 / cycle.len() as f64)
}

/// Weld coincident one-use arcs even when their parameterizations differ,
/// such as frustum isolines paired with fitted apex-cap arcs. Doing this before
/// face completion prevents duplicate cap faces. If disconnected shells traverse
/// the arc in the same direction, flip the smaller shell; merge bridged shells.
pub(super) fn weld_duplicate_one_use_arcs(solid: &mut BrepSolid, tolerance: f64) -> Result<usize, KernelRefusal> {
    let mut welded = 0usize;
    loop {
        let mut use_addresses = HashMap::<u64, Vec<(usize, bool)>>::default();
        for (shell_index, shell) in solid.shells.iter().enumerate() {
            for coedge in shell
                .faces
                .iter()
                .flat_map(|face| &face.loops)
                .flat_map(|loop_record| &loop_record.coedges)
            {
                use_addresses
                    .entry(coedge.edge_id)
                    .or_default()
                    .push((shell_index, coedge.forward));
            }
        }
        let points = solid
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect::<HashMap<_, _>>();
        // Closed edges are excluded like in `sew_coincident_one_use_edges`:
        // this weld unifies coincident OPEN arc records only.
        let candidates = solid
            .edges
            .iter()
            .filter(|edge| {
                !edge.degenerate
                    && edge.start_vertex_id != edge.end_vertex_id
                    && use_addresses
                        .get(&edge.id)
                        .is_some_and(|uses| uses.len() == 1)
            })
            .cloned()
            .collect::<Vec<_>>();
        let mut action = None;
        'pairs: for (index, first) in candidates.iter().enumerate() {
            for second in candidates.iter().skip(index + 1) {
                let Some(reversed) = crate::topology::edge_endpoint_alignment(
                    first, second, &points, tolerance,
                ) else {
                    continue;
                };
                // Endpoint coincidence is established by the vertex check;
                // sample only the INTERIOR (an endpoint that drifts a whisker
                // past the carrier span trips the projection range guard).
                if !edge_interior_lies_on(first, second, tolerance)?
                    || !edge_interior_lies_on(second, first, tolerance)?
                {
                    continue;
                }
                let (first_shell, first_forward) = use_addresses[&first.id][0];
                let (second_shell, second_forward) = use_addresses[&second.id][0];
                let mapped_second_forward = if reversed {
                    !second_forward
                } else {
                    second_forward
                };
                let need_flip = first_forward == mapped_second_forward;
                if need_flip && first_shell == second_shell {
                    // Same component uses the arc twice in the same
                    // direction: not a weldable duplicate — leave it for the
                    // honesty gate.
                    continue;
                }
                action = Some((
                    first.clone(),
                    second.id,
                    reversed,
                    need_flip,
                    first_shell,
                    second_shell,
                ));
                break 'pairs;
            }
        }
        let Some((first_edge, second_id, reversed, need_flip, first_shell, second_shell)) = action
        else {
            break;
        };
        if need_flip {
            let flip_shell = if solid.shells[first_shell].faces.len()
                <= solid.shells[second_shell].faces.len()
            {
                first_shell
            } else {
                second_shell
            };
            os_debug!(
                "duplicate-arc weld: flipping shell {flip_shell} to align orientation for edge {} <- {second_id}",
                first_edge.id
            );
            for face in &mut solid.shells[flip_shell].faces {
                face.same_sense = !face.same_sense;
                for loop_record in &mut face.loops {
                    loop_record.coedges.reverse();
                    for coedge in &mut loop_record.coedges {
                        coedge.forward = !coedge.forward;
                        coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
                    }
                }
            }
        }
        for face in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
            for coedge in face
                .loops
                .iter_mut()
                .flat_map(|loop_record| &mut loop_record.coedges)
            {
                if coedge.edge_id != second_id {
                    continue;
                }
                coedge.edge_id = first_edge.id;
                if reversed {
                    coedge.forward = !coedge.forward;
                }
                // Keep the coedge's pcurve parameter-synchronized with the
                // adopted edge record (the kept arc may be a SUBCURVE record,
                // so project its exact t-range).
                coedge.pcurve = crate::build_pcurve_on_surface_range(
                    &face.surface,
                    &first_edge.curve,
                    first_edge.t0,
                    first_edge.t1,
                    coedge.forward,
                    tolerance,
                ).or_refuse(KernelStage::Sew, "build_pcurve_on_surface_range")?;
            }
        }
        solid.edges.retain(|edge| edge.id != second_id);
        let used_vertices = solid
            .edges
            .iter()
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .collect::<HashSet<_>>();
        solid
            .vertices
            .retain(|vertex| used_vertices.contains(&vertex.id));
        if first_shell != second_shell {
            let (keep, merge) = if first_shell < second_shell {
                (first_shell, second_shell)
            } else {
                (second_shell, first_shell)
            };
            let merged = solid.shells.remove(merge);
            solid.shells[keep].faces.extend(merged.faces);
        }
        os_debug!(
            "duplicate-arc weld: edge {} absorbed duplicate {second_id} (reversed={reversed} flipped={need_flip})",
            first_edge.id
        );
        welded += 1;
    }
    Ok(welded)
}

pub(super) fn complete_opening_boundary_cycles(
    solid: &mut BrepSolid,
    opening_carriers: &[&Carrier],
    face_images: &[OffsetShellFaceImageRecord],
    tolerance: f64,
) -> Result<Vec<OffsetShellFaceImageRecord>, KernelRefusal> {
    let mut use_counts = HashMap::<u64, usize>::default();
    for coedge in solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
    {
        *use_counts.entry(coedge.edge_id).or_default() += 1;
    }
    let edge_by_id = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect::<HashMap<_, _>>();
    let point_by_id = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect::<HashMap<_, _>>();
    let mut boundary = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let role = face_images
            .get(face.id.saturating_sub(1) as usize)
            .ok_or_else(|| "offset_shell: boundary face lost provenance".to_string()).or_refuse(KernelStage::Sew, "offset_shell_boundary_face_lost_provenance")?
            .role;
        for coedge in face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            let edge = &edge_by_id[&coedge.edge_id];
            if edge.degenerate || use_counts.get(&edge.id) != Some(&1) {
                continue;
            }
            let edge_start = point_by_id[&edge.start_vertex_id];
            let edge_end = point_by_id[&edge.end_vertex_id];
            boundary.push(OpenBoundaryUse {
                edge: edge.clone(),
                coedge: coedge.clone(),
                role,
                start: if coedge.forward { edge_start } else { edge_end },
                end: if coedge.forward { edge_end } else { edge_start },
            });
        }
    }

    if debug_enabled() {
        for usage in &boundary {
            let mid = usage
                .edge
                .curve
                .evaluate((usage.edge.t0 + usage.edge.t1) * 0.5)
                .unwrap_or_default();
            os_debug!(
                "COMPLETION boundary edge {} role={:?} start=({:.4},{:.4},{:.4}) end=({:.4},{:.4},{:.4}) mid=({:.4},{:.4},{:.4})",
                usage.edge.id,
                usage.role,
                usage.start.x,
                usage.start.y,
                usage.start.z,
                usage.end.x,
                usage.end.y,
                usage.end.z,
                mid.x,
                mid.y,
                mid.z
            );
        }
    }
    let mut consumed = HashSet::<u64>::default();
    let mut completed_images = Vec::new();
    let mut next_face_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| face.id)
        .max()
        .unwrap_or(0)
        + 1;
    let mut next_loop_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .map(|loop_record| loop_record.id)
        .max()
        .unwrap_or(0)
        + 1;
    let mut next_coedge_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.id)
        .max()
        .unwrap_or(0)
        + 1;

    for carrier in opening_carriers {
        let on_carrier = boundary
            .iter()
            .filter(|usage| {
                !consumed.contains(&usage.edge.id)
                    && edge_on_surface(
                        &usage.edge,
                        &carrier.solid.shells[0].faces[0].surface,
                        tolerance,
                    )
                    .unwrap_or(false)
            })
            .cloned()
            .collect::<Vec<_>>();
        if debug_enabled() && !on_carrier.is_empty() {
            os_debug!(
                "COMPLETION carrier kind={:?} src={} sees one-use edges {:?}",
                carrier.kind,
                carrier.source_face_id,
                on_carrier
                    .iter()
                    .map(|usage| usage.edge.id)
                    .collect::<Vec<_>>()
            );
        }
        let mut traced = HashSet::<u64>::default();
        let mut cycles = Vec::<Vec<OpenBoundaryUse>>::new();
        for (seed_index, seed) in on_carrier.iter().enumerate() {
            if traced.contains(&seed.edge.id) {
                continue;
            }
            fn search_cycle(
                uses: &[OpenBoundaryUse],
                seed_end: Vec3,
                current_index: usize,
                tolerance: f64,
                local: &mut HashSet<u64>,
                path: &mut Vec<usize>,
            ) -> bool {
                let current = &uses[current_index];
                if !local.insert(current.edge.id) {
                    return false;
                }
                path.push(current_index);
                let new_end = current.start;
                if path.len() >= 2 && seed_end.sub(new_end).length() <= tolerance {
                    return true;
                }
                let mut candidates = uses
                    .iter()
                    .enumerate()
                    .filter(|(_, candidate)| {
                        !local.contains(&candidate.edge.id)
                            && candidate.end.sub(new_end).length() <= tolerance
                    })
                    .map(|(index, candidate)| (index, candidate.end.sub(new_end).length()))
                    .collect::<Vec<_>>();
                candidates.sort_by(|first, second| first.1.total_cmp(&second.1));
                for (candidate_index, _) in candidates {
                    if search_cycle(uses, seed_end, candidate_index, tolerance, local, path) {
                        return true;
                    }
                }
                path.pop();
                local.remove(&current.edge.id);
                false
            }
            let mut local = HashSet::default();
            let mut path = Vec::new();
            if search_cycle(
                &on_carrier,
                seed.end,
                seed_index,
                tolerance,
                &mut local,
                &mut path,
            ) {
                traced.extend(local);
                let cycle: Vec<OpenBoundaryUse> = path
                    .into_iter()
                    .map(|index| on_carrier[index].clone())
                    .collect();
                if debug_enabled() {
                    os_debug!(
                        "COMPLETION carrier src={} traced cycle {:?} roles={:?}",
                        carrier.source_face_id,
                        cycle.iter().map(|usage| usage.edge.id).collect::<Vec<_>>(),
                        cycle.iter().map(|usage| usage.role).collect::<Vec<_>>()
                    );
                }
                cycles.push(cycle);
            }
        }

        let source_cycles = cycles
            .iter()
            .filter(|cycle| {
                cycle
                    .iter()
                    .all(|usage| matches!(usage.role, OffsetFaceRole::Source))
            })
            .cloned()
            .collect::<Vec<_>>();
        let offset_cycles = cycles
            .iter()
            .filter(|cycle| {
                cycle
                    .iter()
                    .all(|usage| matches!(usage.role, OffsetFaceRole::Offset))
            })
            .cloned()
            .collect::<Vec<_>>();
        let mixed_cycles = cycles
            .iter()
            .filter(|cycle| {
                let first_role = cycle[0].role;
                cycle.iter().any(|usage| usage.role != first_role)
            })
            .cloned()
            .map(|cycle| vec![cycle])
            .collect::<Vec<_>>();
        let groups = if matches!(carrier.kind, OffsetFaceRole::Offset) {
            // Retained offset carriers cannot contain the intentional part
            // opening. Every closed one-use cycle on one of these carriers is
            // therefore an omitted retained-face region.
            cycles.iter().cloned().map(|cycle| vec![cycle]).collect()
        } else {
            let mut groups = mixed_cycles;
            let mut used_offsets = HashSet::default();
            for source_cycle in source_cycles {
                let source_center = cycle_center(&source_cycle);
                if let Some((offset_index, offset_cycle)) = offset_cycles
                    .iter()
                    .enumerate()
                    .filter(|(index, _)| !used_offsets.contains(index))
                    .min_by(|(_, first), (_, second)| {
                        cycle_center(first)
                            .sub(source_center)
                            .length()
                            .total_cmp(&cycle_center(second).sub(source_center).length())
                    })
                {
                    used_offsets.insert(offset_index);
                    groups.push(vec![source_cycle, offset_cycle.clone()]);
                }
            }
            groups
        };

        for group in groups {
            let mut loops = Vec::new();
            for cycle in &group {
                let mut coedges = Vec::new();
                for usage in cycle {
                    let forward = !usage.coedge.forward;
                    let Some(pcurve) = boundary_pcurve(
                        &usage.edge,
                        forward,
                        &carrier.solid.shells[0].faces[0].surface,
                        tolerance,
                    )?
                    else {
                        coedges.clear();
                        break;
                    };
                    coedges.push(CoedgeRecord {
                        id: next_coedge_id,
                        edge_id: usage.edge.id,
                        forward,
                        pcurve,
                    });
                    next_coedge_id += 1;
                }
                if coedges.is_empty() {
                    loops.clear();
                    break;
                }
                loops.push(LoopRecord {
                    id: next_loop_id,
                    coedges,
                });
                next_loop_id += 1;
            }
            if loops.is_empty() {
                continue;
            }
            // `loops[0]` is the OUTER boundary (the kernel-wide invariant:
            // `face_split.rs`, "the winding the SOURCE face's own outer loop
            // uses"). A group arrives in PAIRING order — the source cycle
            // first, then the offset cycle it was matched with — and on an
            // outward shell's opening wall the source cycle is the cavity
            // outline, the HOLE, while the grown offset outline is the outer:
            // emitted as traced, the ring read `[hole, outer]`, and a face
            // profile taken from it called the outer loop a hole touching
            // the revolve axis. The outer is the largest |uv area|, the same
            // rule `offset_outline_wall` reads the opening's outline by.
            // (`same_sense` is not settled yet, so the SIGN cannot be used.)
            let surface = carrier.solid.shells[0].faces[0].surface.clone();
            let mut loop_areas = Vec::with_capacity(loops.len());
            for loop_record in &loops {
                let area = parameter_space_area(&FaceRecord {
                    id: 0,
                    surface: surface.clone(),
                    same_sense: true,
                    loops: vec![loop_record.clone()],
                    name: None,
                }).or_refuse(KernelStage::Sew, "parameter_space_area")?
                .abs();
                loop_areas.push(area);
            }
            if let Some(outer) = (0..loops.len()).max_by(|first, second| {
                loop_areas[*first].total_cmp(&loop_areas[*second])
            }) {
                if outer != 0 {
                    loops.swap(0, outer);
                }
            }
            let mut face = FaceRecord {
                id: next_face_id,
                surface,
                same_sense: true,
                loops,
                name: None,
            };
            let area = parameter_space_area(&face).or_refuse(KernelStage::Sew, "parameter_space_area")?;
            if area.abs() <= 1e-12 {
                continue;
            }
            face.same_sense = area > 0.0;
            let face_edge_ids = face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .map(|coedge| coedge.edge_id)
                .collect::<HashSet<_>>();
            let connected_shells = solid
                .shells
                .iter()
                .enumerate()
                .filter_map(|(shell_index, shell)| {
                    shell
                        .faces
                        .iter()
                        .any(|candidate| {
                            candidate
                                .loops
                                .iter()
                                .flat_map(|loop_record| &loop_record.coedges)
                                .any(|coedge| face_edge_ids.contains(&coedge.edge_id))
                        })
                        .then_some(shell_index)
                })
                .collect::<Vec<_>>();
            let connected_shell = connected_shells.first().copied().unwrap_or(0);
            // A recovered wall can be the first face that joins independently
            // assembled source and offset components. Merge every shell it
            // touches instead of attaching it to only the first component.
            for shell_index in connected_shells.iter().copied().skip(1).rev() {
                let merged = solid.shells.remove(shell_index);
                solid.shells[connected_shell].faces.extend(merged.faces);
            }
            if debug_enabled() {
                os_debug!(
                    "COMPLETION carrier src={} kind={:?} ADDING face {} from cycles {:?} (area {:.4})",
                    carrier.source_face_id,
                    carrier.kind,
                    face.id,
                    group
                        .iter()
                        .map(|cycle| cycle
                            .iter()
                            .map(|usage| usage.edge.id)
                            .collect::<Vec<_>>())
                        .collect::<Vec<_>>(),
                    area
                );
            }
            solid.shells[connected_shell].faces.push(face);
            for cycle in &group {
                consumed.extend(cycle.iter().map(|usage| usage.edge.id));
            }
            completed_images.push(OffsetShellFaceImageRecord {
                role: carrier.kind,
                source_face_id: carrier.source_face_id,
            });
            next_face_id += 1;
        }
    }
    Ok(completed_images)
}
