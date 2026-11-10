//! Close a selected boss at the apex of its surviving conical wall.
//!
//! Imports often split that wall at meridians. Its opening is then a rim in
//! each half's outer loop, not a hole loop that the patch cap can drop. Keep
//! those halves and their parameterizations: prolong the meridians to the
//! common apex and replace each rim by its own degenerate pole edge.

use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

pub(super) fn cap_conical_apex(
    solid: &BrepSolid,
    selection: &[u64],
) -> Option<Result<BrepSolid, KernelRefusal>> {
    let selected: HashSet<u64> = selection.iter().copied().collect();
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let edges: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut owners: HashMap<u64, Vec<u64>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
            owners.entry(coedge.edge_id).or_default().push(face.id);
        }
    }
    let removed: HashSet<u64> = owners
        .iter()
        .filter(|(_, ids)| ids.iter().all(|id| selected.contains(id)))
        .map(|(id, _)| *id)
        .collect();
    let rim: HashSet<u64> = owners
        .iter()
        .filter(|(_, ids)| {
            ids.iter().any(|id| selected.contains(id))
                && ids.iter().any(|id| !selected.contains(id))
        })
        .map(|(id, _)| *id)
        .collect();
    if rim.is_empty()
        || rim
            .iter()
            .any(|id| owners[id].len() != 2 || edges[id].degenerate)
    {
        return None;
    }
    let survivors: HashSet<u64> = rim
        .iter()
        .flat_map(|id| &owners[id])
        .filter(|id| !selected.contains(id))
        .copied()
        .collect();
    // This lane reads split walls, each with one open rim arc. A whole periodic
    // wall needs a seam-aware pole construction of its own.
    if survivors.len() < 2 || survivors.len() != rim.len() {
        return None;
    }
    let mut apex = None;
    let mut axis = None;
    let mut slope = None;
    let mut walls = Vec::new();
    for id in &survivors {
        let (shell, position) = find_face(solid, *id)?;
        let face = &solid.shells[shell].faces[position];
        let touched: Vec<_> = face
            .loops
            .iter()
            .flat_map(|l| &l.coedges)
            .filter(|coedge| rim.contains(&coedge.edge_id))
            .collect();
        if touched.len() != 1 {
            return None;
        }
        let AnalyticSurface::RuledRevolution {
            frame,
            rho0,
            rho1,
            height,
        } = face.surface.analytic()?
        else {
            return None;
        };
        if *height <= tolerance || *rho0 <= tolerance || rho1 - rho0 <= tolerance {
            return None;
        }
        let k = (rho1 - rho0) / height;
        let z = -rho0 / k;
        let pole = frame.origin.add(frame.axis.scale(z));
        if apex.is_some_and(|other: Vec3| other.sub(pole).length() > tolerance)
            || axis.is_some_and(|other: Vec3| other.sub(frame.axis).length() > 1e-9)
            || slope.is_some_and(|other: f64| (other - k).abs() > 1e-9 * k.abs().max(1.0))
        {
            return None;
        }
        apex = Some(pole);
        axis = Some(frame.axis);
        slope = Some(k);
        // The removed rim is the narrow end of this exact carrier. Its UV
        // arc stays as the pole trim; sampling also rejects a tilted opening.
        let coedge = touched[0];
        let [a, b] = coedge.pcurve.domain().ok()?;
        let edge = edges[&coedge.edge_id];
        let rim_v = coedge.pcurve.evaluate(a).ok()?.y;
        let rim_z = edge
            .curve
            .evaluate(edge.t0)
            .ok()?
            .sub(frame.origin)
            .dot(frame.axis);
        if rim_z <= z + tolerance {
            return None;
        }
        for step in 0..=16 {
            let f = step as f64 / 16.0;
            let uv = coedge.pcurve.evaluate(a + (b - a) * f).ok()?;
            let point = edge
                .curve
                .evaluate(edge.t0 + (edge.t1 - edge.t0) * f)
                .ok()?;
            if (uv.y - rim_v).abs() > 1e-9
                || (point.sub(frame.origin).dot(frame.axis) - rim_z).abs() > tolerance
            {
                return None;
            }
        }
        walls.push((*id, frame.clone(), *rho1, *height, z));
    }
    // A single connected selection in one shell, with one free circuit.
    let shell = find_face(solid, selection[0])?.0;
    if selection
        .iter()
        .chain(survivors.iter())
        .any(|id| find_face(solid, *id).map(|p| p.0) != Some(shell))
    {
        return None;
    }
    let mut reached = HashSet::default();
    let mut pending = vec![selection[0]];
    while let Some(id) = pending.pop() {
        if !reached.insert(id) {
            continue;
        }
        let (_, position) = find_face(solid, id)?;
        for coedge in solid.shells[shell].faces[position]
            .loops
            .iter()
            .flat_map(|l| &l.coedges)
        {
            pending.extend(
                owners[&coedge.edge_id]
                    .iter()
                    .filter(|other| selected.contains(other) && !reached.contains(other))
                    .copied(),
            );
        }
    }
    if reached != selected {
        return None;
    }
    let apex = apex?;
    let rim_vertices: HashSet<u64> = rim
        .iter()
        .flat_map(|id| [edges[id].start_vertex_id, edges[id].end_vertex_id])
        .collect();
    // One closed circuit, with exactly one retained meridian at each corner.
    // No other face may own a moved vertex or an extended edge.
    if rim_vertices.len() != rim.len()
        || rim
            .iter()
            .any(|id| edges[id].start_vertex_id == edges[id].end_vertex_id)
    {
        return None;
    }
    let mut circuit = HashSet::default();
    let mut pending = vec![*rim_vertices.iter().next()?];
    while let Some(vertex) = pending.pop() {
        if !circuit.insert(vertex) {
            continue;
        }
        for id in &rim {
            let edge = edges[id];
            if edge.start_vertex_id == vertex {
                pending.push(edge.end_vertex_id);
            }
            if edge.end_vertex_id == vertex {
                pending.push(edge.start_vertex_id);
            }
        }
    }
    if circuit != rim_vertices {
        return None;
    }
    let mut meridians = HashSet::default();
    for vertex in &rim_vertices {
        if rim
            .iter()
            .filter(|id| edges[id].start_vertex_id == *vertex || edges[id].end_vertex_id == *vertex)
            .count()
            != 2
        {
            return None;
        }
        let kept: Vec<_> = solid
            .edges
            .iter()
            .filter(|edge| {
                !removed.contains(&edge.id)
                    && !rim.contains(&edge.id)
                    && (edge.start_vertex_id == *vertex || edge.end_vertex_id == *vertex)
            })
            .collect();
        let [edge] = kept.as_slice() else {
            return None;
        };
        if owners[&edge.id].len() != 2 || owners[&edge.id].iter().any(|id| !survivors.contains(id))
        {
            return None;
        }
        let far_id = if edge.start_vertex_id == *vertex {
            edge.end_vertex_id
        } else {
            edge.start_vertex_id
        };
        if rim_vertices.contains(&far_id) {
            return None;
        }
        let far = solid.vertices.iter().find(|v| v.id == far_id)?.point;
        let direction = far.sub(apex).normalized().ok()?;
        for step in 0..=16 {
            let point = edge
                .curve
                .evaluate(edge.t0 + (edge.t1 - edge.t0) * step as f64 / 16.0)
                .ok()?;
            if point.sub(apex).cross(direction).length() > tolerance {
                return None;
            }
        }
        meridians.insert(edge.id);
    }
    // Meridian pcurves must be straight constant-u lines. Preserve the u
    // branch explicitly: projecting a pole cannot recover its longitude.
    for id in &survivors {
        let (s, p) = find_face(solid, *id)?;
        for coedge in solid.shells[s].faces[p]
            .loops
            .iter()
            .flat_map(|l| &l.coedges)
        {
            if !meridians.contains(&coedge.edge_id) {
                continue;
            }
            let controls = &coedge.pcurve.control_points;
            let u = controls.first()?.point().ok()?.x;
            for control in controls {
                if control.w <= 0.0 || (control.point().ok()?.x - u).abs() > 1e-9 {
                    return None;
                }
            }
        }
    }
    Some((|| {
        let mut healed = solid.clone();
        let pole_id = *rim_vertices.iter().min().unwrap();
        for vertex in &mut healed.vertices {
            if vertex.id == pole_id {
                vertex.point = apex;
            }
        }
        healed.edges.retain(|edge| !removed.contains(&edge.id));
        for edge in &mut healed.edges {
            let start_moved = rim_vertices.contains(&edge.start_vertex_id);
            let end_moved = rim_vertices.contains(&edge.end_vertex_id);
            if start_moved {
                edge.start_vertex_id = pole_id;
            }
            if end_moved {
                edge.end_vertex_id = pole_id;
            }
            if rim.contains(&edge.id) {
                edge.degenerate = true;
                edge.curve = make_line(apex, apex).or_refuse(KernelStage::Sew, "cone_pole")?;
            } else if meridians.contains(&edge.id) {
                let point = |id| healed.vertices.iter().find(|v| v.id == id).unwrap().point;
                edge.curve = make_line(point(edge.start_vertex_id), point(edge.end_vertex_id))
                    .or_refuse(KernelStage::Sew, "cone_meridian")?;
            } else {
                continue;
            }
            edge.t0 = 0.0;
            edge.t1 = 1.0;
        }
        for (id, frame, radius, height, z) in walls {
            let (s, p) = find_face(&healed, id).unwrap();
            let face = &mut healed.shells[s].faces[p];
            let far = frame
                .origin
                .add(frame.axis.scale(height))
                .add(frame.x_axis.scale(radius));
            let line = make_line(apex, far).or_refuse(KernelStage::Sew, "cone_profile")?;
            face.surface = make_revolution(apex, frame.axis, &line, std::f64::consts::TAU)
                .or_refuse(KernelStage::Sew, "cone_surface")?;
            for coedge in face.loops.iter_mut().flat_map(|l| &mut l.coedges) {
                let original = edges[&coedge.edge_id];
                for control in &mut coedge.pcurve.control_points {
                    control.y = (control.y * height - z * control.w) / (height - z);
                    if rim.contains(&coedge.edge_id) {
                        control.y = 0.0;
                    }
                }
                if meridians.contains(&coedge.edge_id) {
                    let [a, b] = coedge
                        .pcurve
                        .domain()
                        .or_refuse(KernelStage::Sew, "cone_trim")?;
                    let mut start = coedge
                        .pcurve
                        .evaluate(a)
                        .or_refuse(KernelStage::Sew, "cone_trim")?;
                    let mut end = coedge
                        .pcurve
                        .evaluate(b)
                        .or_refuse(KernelStage::Sew, "cone_trim")?;
                    let (from, to) = if coedge.forward {
                        (original.start_vertex_id, original.end_vertex_id)
                    } else {
                        (original.end_vertex_id, original.start_vertex_id)
                    };
                    if rim_vertices.contains(&from) {
                        start.y = 0.0;
                    }
                    if rim_vertices.contains(&to) {
                        end.y = 0.0;
                    }
                    coedge.pcurve =
                        make_line(start, end).or_refuse(KernelStage::Sew, "cone_trim")?;
                }
            }
        }
        for shell in &mut healed.shells {
            shell.faces.retain(|face| !selected.contains(&face.id));
        }
        let used: HashSet<u64> = healed
            .edges
            .iter()
            .flat_map(|e| [e.start_vertex_id, e.end_vertex_id])
            .collect();
        healed.vertices.retain(|v| used.contains(&v.id));
        let issues = healed.validate();
        if !issues.is_empty()
            || super::delete_faces::euler_characteristic(&healed)
                != super::delete_faces::euler_characteristic(solid)
        {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, "cone_cap_topology",
                format!("delete_faces_and_heal: conical apex cap does not preserve a closed shell: {issues:?}")));
        }
        Ok(healed)
    })())
}
