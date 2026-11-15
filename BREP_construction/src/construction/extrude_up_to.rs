//! An extrude that ends on a face of another solid, and a solid's separate
//! pieces.
//!
//! [`extrude_profile_up_to_face`] slides a profile along a direction until it
//! meets a chosen face of a target solid: the prism run on past the target,
//! less the target, and of what is left the piece that stands on the profile.
//! Its far end is then the target's own surface, trimmed by the prism's walls,
//! so the result fits the target exactly. An end that would land on any other
//! face of the target is refused by name.
use crate::topology::ShellRecord;
use crate::{boolean_operation, extrude_profile_brep, project_point_to_surface, solid_signed_volume, BooleanOperation, BooleanOptions, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3};

/// The prefix of every refusal of this operation.
pub const EXTRUDE_UP_TO_REFUSAL: &str = "extrudeUpToFace";

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{EXTRUDE_UP_TO_REFUSAL}: {message}"))
}

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Select, what, format!("{EXTRUDE_UP_TO_REFUSAL}: {message}"))
}

/// The separate pieces of `solid`: one solid per shell that bounds material,
/// each with the shells of the hollows inside it. A solid of one piece comes
/// back as itself.
pub fn solid_pieces(solid: &BrepSolid) -> Result<Vec<BrepSolid>, KernelRefusal> {
    if solid.shells.len() <= 1 {
        return Ok(vec![solid.clone()]);
    }
    let alone = |shells: Vec<ShellRecord>| -> BrepSolid {
        let mut edge_ids = std::collections::HashSet::new();
        for coedge in shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| &face.loops).flat_map(|record| &record.coedges) {
            edge_ids.insert(coedge.edge_id);
        }
        let edges: Vec<_> = solid.edges.iter().filter(|edge| edge_ids.contains(&edge.id)).cloned().collect();
        let vertex_ids: std::collections::HashSet<u64> = edges.iter().flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id]).collect();
        let vertices: Vec<_> = solid.vertices.iter().filter(|vertex| vertex_ids.contains(&vertex.id)).cloned().collect();
        // V − E + F − H = 2·shells − 2g, poles' degenerate edges not counted
        let live = edges.iter().filter(|edge| !edge.degenerate).count() as i64;
        let faces = shells.iter().map(|shell| shell.faces.len()).sum::<usize>() as i64;
        let rings = shells.iter().flat_map(|shell| &shell.faces).map(|face| face.loops.len().saturating_sub(1)).sum::<usize>() as i64;
        let twice = 2 * shells.len() as i64 - (vertices.len() as i64 - live + faces - rings);
        BrepSolid { mass_properties_cache: Default::default(), id: solid.id, vertices, edges, shells, genus: (twice / 2).max(0), kind: solid.kind, cells: Default::default() }
    };
    // a shell that bounds material has a positive volume; a hollow's is negative
    let mut outers: Vec<(Vec<ShellRecord>, BrepSolid)> = Vec::new();
    let mut hollows = Vec::new();
    for shell in &solid.shells {
        let single = alone(vec![shell.clone()]);
        let volume = solid_signed_volume(&single).or_refuse(KernelStage::Select, "solidPieces.volume")?;
        if volume > 0.0 {
            outers.push((vec![shell.clone()], single));
        } else {
            hollows.push((shell.clone(), single));
        }
    }
    for (shell, single) in hollows {
        // a hollow belongs to the piece that holds one of its vertices
        let probe = single.vertices.first().map(|vertex| vertex.point).ok_or_else(|| refuse("pieces_hollow", "a hollow shell has no vertex".into()))?;
        let mut home = None;
        for (index, (_, outer)) in outers.iter().enumerate() {
            let class = crate::classify_point(probe, outer, 1e-9).or_refuse(KernelStage::Select, "solidPieces.classify")?;
            if matches!(class.class, crate::PointClass::In) {
                home = Some(index);
                break;
            }
        }
        let index = home.ok_or_else(|| refuse("pieces_hollow", "a hollow shell lies in no piece".into()))?;
        outers[index].0.push(shell);
    }
    Ok(outers.into_iter().map(|(shells, _)| alone(shells)).collect())
}

/// The prism on `profile` (a closed loop of curves in a plane, or one closed
/// curve) slid along `direction` until it meets the face `face_id` of
/// `target`, on which it ends.
///
/// The profile must lie clear of the target's material where it starts (on
/// its boundary is clear), and the prism must reach the face all round: where
/// part of it would run past the face's edge onto another face of the target,
/// or miss the target, the operation refuses.
pub fn extrude_profile_up_to_face(profile: &[NurbsCurve], direction: Vec3, target: &BrepSolid, face_id: u64) -> Result<BrepSolid, KernelRefusal> {
    let along = direction.normalized().map_err(|_| input("direction", "direction must be non-zero".into()))?;
    let face = target
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == face_id)
        .ok_or_else(|| input("face", format!("the target has no face {face_id}")))?;
    let start = profile
        .first()
        .and_then(|curve| curve.control_points.first())
        .map(|pole| Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w))
        .ok_or_else(|| input("profile", "the profile is empty".into()))?;
    // far enough to pass every point of the target
    let poles = target.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| face.surface.control_points.iter().flatten());
    let (mut reach, mut size) = (0.0f64, 0.0f64);
    for pole in poles {
        let arm = Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w).sub(start);
        reach = reach.max(arm.dot(along));
        size = size.max(arm.length());
    }
    if reach <= 0.0 {
        return Err(refuse("misses", "the target lies behind the profile".into()));
    }
    let prism = extrude_profile_brep(profile, along, reach + 0.25 * size)?;
    let left = boolean_operation(&prism, target, BooleanOperation::Subtract, &BooleanOptions::default())?;
    let tolerance = 1e-7 * size.max(1.0);
    let at_start = |piece: &BrepSolid| piece.vertices.iter().any(|vertex| vertex.point.sub(start).dot(along).abs() <= tolerance);
    let mut standing: Vec<BrepSolid> = solid_pieces(&left)?.into_iter().filter(at_start).collect();
    let piece = match standing.len() {
        1 => standing.remove(0),
        0 => return Err(refuse("buried", "the profile starts inside the target".into())),
        _ => return Err(refuse("split", "the target cuts the prism into several pieces at the profile".into())),
    };
    // every face of the piece is the profile's own, a wall along the direction, or on the chosen
    // face's surface: a face on any other surface is another face of the target
    for end in piece.shells.iter().flat_map(|shell| &shell.faces) {
        let [u0, u1] = end.surface.domain_u().or_refuse(KernelStage::Select, "extrudeUpToFace.domain")?;
        let [v0, v1] = end.surface.domain_v().or_refuse(KernelStage::Select, "extrudeUpToFace.domain")?;
        let mut kinds = [true, true, true];
        for (a, b) in [(0.5, 0.5), (0.25, 0.3), (0.7, 0.8)] {
            let (u, v) = (u0 + (u1 - u0) * a, v0 + (v1 - v0) * b);
            let point = end.surface.evaluate(u, v).or_refuse(KernelStage::Select, "extrudeUpToFace.point")?;
            let normal = end.surface.normal(u, v).or_refuse(KernelStage::Select, "extrudeUpToFace.normal")?;
            let foot = project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Select, "extrudeUpToFace.project")?;
            kinds[0] &= point.sub(start).dot(along).abs() <= tolerance && normal.cross(along).length() <= 1e-9;
            kinds[1] &= normal.dot(along).abs() <= 1e-9;
            kinds[2] &= foot.distance <= tolerance;
        }
        if !kinds.iter().any(|kind| *kind) {
            return Err(refuse("other_face", format!("the prism meets another face of the target before face {face_id}")));
        }
    }
    let far_end = reach + 0.25 * size;
    for vertex in &piece.vertices {
        let height = vertex.point.sub(start).dot(along);
        if height.abs() <= tolerance {
            continue;
        }
        if (height - far_end).abs() <= tolerance {
            return Err(refuse("misses", "part of the prism passes the target without meeting the face".into()));
        }
        let foot = project_point_to_surface(&face.surface, vertex.point).or_refuse(KernelStage::Select, "extrudeUpToFace.project")?;
        if foot.distance > tolerance {
            return Err(refuse("other_face", format!("the prism meets another face of the target before face {face_id}")));
        }
    }
    Ok(piece)
}

