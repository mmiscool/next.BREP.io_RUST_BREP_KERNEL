//! Wire bodies (non-manifold plan 3.3): an `EDGE_BASED_WIREFRAME_MODEL` —
//! connected edge sets of `EDGE_CURVE`s, with their shared vertices — built as
//! one body of kind [`crate::BodyKind::Wire`]: edges and vertices, no faces.
//! Each edge is resolved exactly as a face's edge is (the same vertex welding,
//! curve resolution and endpoint healing).
use super::*;

/// Build the wire body of `EDGE_BASED_WIREFRAME_MODEL` `model_ref`.
pub(in crate::step_import) fn build_wire(resolver: &Resolver, model_ref: usize) -> Result<BrepSolid, String> {
    let entity = resolver.get(model_ref)?;
    let sets = entity
        .find("EDGE_BASED_WIREFRAME_MODEL")
        .and_then(|args| args.get(1))
        .ok_or_else(|| format!("step_import: #{model_ref} is not an EDGE_BASED_WIREFRAME_MODEL"))?
        .as_list()?;
    let mut builder = SolidBuilder {
        resolver,
        next_id: 1,
        vertices: Vec::new(),
        edges: Vec::new(),
        vertex_of_ref: HashMap::default(),
        edge_of_ref: HashMap::default(),
        edge_of_vertex_pair: HashMap::default(),
        vertex_index: HashMap::default(),
        edge_index: HashMap::default(),
        curve_ref_of_edge: HashMap::default(),
        supplied_report: SuppliedReport::default(),
        bounded: Vec::new(),
        readings: None,
        face_refs: HashMap::default(),
        reconciled_edges: HashSet::default(),
        body_faces: 0,
        open_shell: true,
    };
    for set in sets {
        let set_ref = set.as_ref_id()?;
        let edges = resolver
            .get(set_ref)?
            .find("CONNECTED_EDGE_SET")
            .and_then(|args| args.get(1))
            .ok_or_else(|| format!("step_import: #{set_ref} is not a CONNECTED_EDGE_SET"))?
            .as_list()?;
        for edge in edges {
            let edge_ref = edge.as_ref_id()?;
            builder.edge(edge_ref).map_err(|e| format!("edge #{edge_ref}: {e}"))?;
        }
    }
    // An edge shorter than the import's identity tolerance has its two ends
    // welded into one vertex. It is dropped (its vertex stays for the edges
    // that meet there) only when its whole curve is a point at that
    // tolerance: every control point there, which bounds the curve. The
    // builder's `degenerate` flag reads samples, which a long curve can pass
    // near its vertex (re-review); such an edge is a closed edge, kept.
    let points: HashMap<u64, crate::Vec3> = builder.vertices.iter().map(|v| (v.id, v.point)).collect();
    builder.edges.retain(|edge| {
        !(edge.start_vertex_id == edge.end_vertex_id
            && brep_topology::topology::control_points_within(&edge.curve, points[&edge.start_vertex_id], IMPORT_IDENTITY_TOLERANCE))
    });
    for edge in &mut builder.edges {
        edge.degenerate = false;
    }
    let used: HashSet<u64> = builder.edges.iter().flat_map(|e| [e.start_vertex_id, e.end_vertex_id]).collect();
    builder.vertices.retain(|vertex| used.contains(&vertex.id));
    if builder.edges.is_empty() {
        return Err(format!("step_import: wireframe #{model_ref} has no edges longer than the import's identity tolerance"));
    }
    let id = builder.fresh();
    let wire = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id,
        vertices: builder.vertices,
        edges: builder.edges,
        shells: Vec::new(),
        genus: 0,
        kind: crate::BodyKind::Wire,
    };
    let issues = wire.validate();
    if let Some(issue) = issues.first() {
        return Err(format!("step_import: wireframe #{model_ref} failed validation: {}", issue.message));
    }
    Ok(wire)
}

