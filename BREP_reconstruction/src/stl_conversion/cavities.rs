//! Preserve inward connected shells as voids of their enclosing component.
use super::*;
use brep_kernel::BrepSolid;

pub(super) fn signed_volume(mesh: &Mesh, triangles: &[usize]) -> f64 {
    let origin = mesh.vertices[mesh.triangles[triangles[0]][0] as usize];
    triangles
        .iter()
        .map(|&t| {
            let [a, b, c] = mesh.triangles[t].map(|i| mesh.vertices[i as usize] - origin);
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

pub(super) fn outward_component(
    mesh: &Mesh,
    positions: &[f64],
    indices: Option<&[u32]>,
) -> Option<ComponentMesh> {
    let triangles = (0..mesh.triangles.len()).collect::<Vec<_>>();
    if !closed_manifold(mesh) || signed_volume(mesh, &triangles) >= 0.0 {
        return None;
    }
    let mut part = component_mesh(mesh, positions, indices, &triangles);
    for triangle in &mut part.mesh.triangles {
        triangle.swap(1, 2);
    }
    if let Some(normals) = &mut part.mesh.vertex_normals {
        for normal in normals {
            *normal = -*normal;
        }
    }
    if let Some(indices) = &mut part.indices {
        for triangle in indices.chunks_exact_mut(3) {
            triangle.swap(1, 2);
        }
    } else {
        for triangle in part.positions.chunks_exact_mut(9) {
            for coordinate in 0..3 {
                triangle.swap(3 + coordinate, 6 + coordinate);
            }
        }
        // The exact topology builder needs shared vertex indices. Preserve
        // the source coordinates while indexing repeated soup corners.
        let mut lookup = HashMap::new();
        let mut positions = Vec::new();
        let indices = part
            .positions
            .chunks_exact(3)
            .map(|point| {
                let key =
                    [point[0], point[1], point[2]].map(|v| if v == 0.0 { 0 } else { v.to_bits() });
                *lookup.entry(key).or_insert_with(|| {
                    let index = positions.len() as u32 / 3;
                    positions.extend_from_slice(point);
                    index
                })
            })
            .collect();
        part.positions = positions;
        part.indices = Some(indices);
    }
    Some(part)
}

fn inside(mesh: &Mesh, triangles: &[usize], point: Vec3) -> bool {
    // Oriented solid angle is independent of ray direction and avoids rays
    // passing through tessellation vertices. Final analytic containment is
    // certified by the BREP_WITH_VOIDS STEP round-trip gate.
    let angle: f64 = triangles
        .iter()
        .map(|&t| {
            let [a, b, c] = mesh.triangles[t].map(|i| mesh.vertices[i as usize] - point);
            let (la, lb, lc) = (a.length(), b.length(), c.length());
            2.0 * a
                .dot(b.cross(c))
                .atan2(la * lb * lc + a.dot(b) * lc + b.dot(c) * la + c.dot(a) * lb)
        })
        .sum();
    angle > std::f64::consts::TAU
}

pub(super) fn assemble(
    mesh: &Mesh,
    components: &[Vec<usize>],
    solids: Vec<BrepSolid>,
    component_ids: &[usize],
) -> Result<(Vec<BrepSolid>, Vec<String>), StlConversionError> {
    if components.len() < 2 {
        return Ok((solids, Vec::new()));
    }
    let volumes = components
        .iter()
        .map(|ts| {
            if closed_triangles(ts.iter().map(|&t| mesh.triangles[t])) {
                signed_volume(mesh, ts)
            } else {
                0.0
            }
        })
        .collect::<Vec<_>>();
    let mut parents = BTreeMap::new();
    for (inner, triangles) in components.iter().enumerate() {
        if volumes[inner] >= 0.0 {
            continue;
        }
        let vertices = triangles
            .iter()
            .flat_map(|&t| mesh.triangles[t])
            .collect::<BTreeSet<_>>();
        let mut candidates = Vec::new();
        for (outer, ts) in components.iter().enumerate() {
            if volumes[outer] <= 0.0 {
                continue;
            }
            let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut high = -low;
            for p in ts
                .iter()
                .flat_map(|&t| mesh.triangles[t])
                .map(|i| mesh.vertices[i as usize])
            {
                low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
                high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
            }
            if vertices.iter().all(|&i| {
                let p = mesh.vertices[i as usize];
                p.x > low.x
                    && p.y > low.y
                    && p.z > low.z
                    && p.x < high.x
                    && p.y < high.y
                    && p.z < high.z
            }) && vertices
                .iter()
                .all(|&i| inside(mesh, ts, mesh.vertices[i as usize]))
            {
                candidates.push(outer);
            }
        }
        if let Some(outer) = candidates
            .into_iter()
            .min_by(|&a, &b| volumes[a].total_cmp(&volumes[b]))
        {
            parents.insert(inner, outer);
        }
    }
    let mut built = component_ids
        .iter()
        .copied()
        .zip(solids)
        .collect::<BTreeMap<_, _>>();
    let mut messages = Vec::new();
    for (inner, outer) in parents {
        // Never export a cavity as a filled object when its owner failed.
        let cavity = built.remove(&inner).ok_or_else(|| {
            StlConversionError(format!(
                "cavity component {} could not be reconstructed",
                inner + 1
            ))
        })?;
        let body = built.get_mut(&outer).ok_or_else(|| {
            StlConversionError(format!(
                "enclosing component {} for cavity {} could not be reconstructed",
                outer + 1,
                inner + 1
            ))
        })?;
        append_void(body, cavity).map_err(StlConversionError)?;
        messages.push(format!(
            "component {} retained as an inward cavity of component {}",
            inner + 1,
            outer + 1
        ));
    }
    Ok((built.into_values().collect(), messages))
}

fn append_void(body: &mut BrepSolid, mut cavity: BrepSolid) -> Result<(), String> {
    // Rebase every topology identifier into the owner's namespace.
    let offset = std::iter::once(body.id)
        .chain(body.vertices.iter().map(|v| v.id))
        .chain(body.edges.iter().map(|e| e.id))
        .chain(body.shells.iter().map(|s| s.id))
        .chain(body.shells.iter().flat_map(|s| &s.faces).map(|f| f.id))
        .chain(
            body.shells
                .iter()
                .flat_map(|s| &s.faces)
                .flat_map(|f| &f.loops)
                .map(|l| l.id),
        )
        .chain(
            body.shells
                .iter()
                .flat_map(|s| &s.faces)
                .flat_map(|f| &f.loops)
                .flat_map(|l| &l.coedges)
                .map(|c| c.id),
        )
        .max()
        .unwrap_or(0)
        + 1;
    for v in &mut cavity.vertices {
        v.id += offset;
    }
    for e in &mut cavity.edges {
        e.id += offset;
        e.start_vertex_id += offset;
        e.end_vertex_id += offset;
    }
    for shell in &mut cavity.shells {
        shell.id += offset;
        for face in &mut shell.faces {
            face.id += offset;
            face.same_sense = !face.same_sense;
            for lp in &mut face.loops {
                lp.id += offset;
                lp.coedges.reverse();
                for c in &mut lp.coedges {
                    c.id += offset;
                    c.edge_id += offset;
                    c.forward = !c.forward;
                    c.pcurve = c.pcurve.reversed()?;
                }
            }
        }
    }
    body.vertices.extend(cavity.vertices);
    body.edges.extend(cavity.edges);
    body.shells.extend(cavity.shells);
    body.genus += cavity.genus;
    body.mass_properties_cache = Default::default();
    let issues = body.validate();
    if !issues.is_empty() {
        return Err(format!("cavity assembly has invalid topology: {issues:?}"));
    }
    Ok(())
}
