use super::*;

// ---------------------------------------------------------------------------
// Stage 3: rebuild a BREP whose faces are the segmented regions
// ---------------------------------------------------------------------------
//
// v1 scope (anything else returns an honest `Err`):
//   * PLANE regions   → one trimmed planar face each; boundary chains between
//     two planar carriers are straight (both carriers are flat, so the shared
//     boundary lies on their intersection line) and become single line edges;
//     non-straight chains fall back to an interpolated polyline curve.
//   * CYLINDER / CONE regions bounded by exactly two axis-perpendicular
//     closed rings → one full revolve wall (seam edge + two exact circle
//     edges, the `make_cylinder_brep` layout).  Outward walls only
//     (`sense == +1`); cavity walls are refused.
//   * CYLINDER regions bounded by one four-chain loop (two axis-perpendicular
//     arcs + two rulings, the fillet-blend patch) → one exact extruded-arc
//     face with iso-parameter edges.
//   * FREEFORM regions: if EVERY region is freeform the whole mesh delegates
//     to `mesh_to_faceted_brep` (triangle-per-face, documented fallback);
//     a MIX of freeform and analytic regions is refused rather than risk an
//     inconsistent shell.
//   * SPHERE / TORUS regions are refused (their face topology needs pole /
//     seam handling not built here yet).
//
// The exact circle / arc edges are the closed-form plane×revolve
// intersections, constructed directly from the carriers (iso curves of the
// revolve / `make_arc` about the axis) so their parameterization lines up
// with the wall's uv space; `intersect_analytic_pair` yields the same loci
// but with an uncontrolled seam start, which would force fitted pcurves
// where an exact parameter line is available.

/// One maximal run of region-boundary mesh edges between corner vertices
/// (vertices where the adjacent region pair changes or more than two
/// boundary edges meet).  `vertices` are welded mesh vertex indices; for
/// closed chains the cycle is stored without repeating the first vertex.
pub(super) struct Chain {
    pub(super) vertices: Vec<usize>,
    pub(super) closed: bool,
}

/// Ring chain claimed by a full revolve wall: the wall fixed the exact
/// circle edge and its own traversal direction; the cap side must use the
/// opposite direction (checked against the cap's mesh circulation).
#[derive(Clone)]
pub(super) struct RingClaim {
    pub(super) axis_point: Vec3,
    pub(super) axis: Vec3,
    pub(super) wall_forward: bool,
}

/// Shared edge built for a chain, plus how its curve direction relates to
/// the chain's stored vertex order.
pub(super) struct ChainEdgeInfo {
    pub(super) edge_id: u64,
    pub(super) curve_along_chain: bool,
    pub(super) ring: Option<RingClaim>,
}

/// One traversal of a chain inside a region's ordered boundary walk.
pub(super) struct Traversal {
    pub(super) chain: usize,
    pub(super) forward_along_chain: bool,
}

pub(super) struct RegionBrepBuilder<'a> {
    pub(super) data: &'a MeshData,
    pub(super) tol: f64,
    pub(super) chains: Vec<Chain>,
    edge_chain: HashMap<(usize, usize), usize>,
    pub(super) chain_edges: Vec<Option<ChainEdgeInfo>>,
    pub(super) vertices: Vec<VertexRecord>,
    vertex_ids: HashMap<usize, u64>,
    edges: Vec<EdgeRecord>,
    edge_index: HashMap<u64, usize>,
    next_id: u64,
}

pub(super) fn wrap_to_pi(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= std::f64::consts::TAU;
    }
    while angle < -std::f64::consts::PI {
        angle += std::f64::consts::TAU;
    }
    angle
}

pub(super) fn max_deviation_from_segment(points: &[Vec3]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let start = points[0];
    let axis = points[points.len() - 1].sub(start);
    let length_squared = axis.length_squared();
    let mut worst = 0.0_f64;
    for &p in &points[1..points.len() - 1] {
        let d = p.sub(start);
        let t = if length_squared > 0.0 {
            (d.dot(axis) / length_squared).clamp(0.0, 1.0)
        } else {
            0.0
        };
        worst = worst.max(d.sub(axis.scale(t)).length());
    }
    worst
}

fn chord_parameters(points: &[Vec3]) -> Result<Vec<f64>, String> {
    let mut cumulative = vec![0.0_f64];
    for pair in points.windows(2) {
        let step = pair[1].sub(pair[0]).length();
        if !(step > 0.0) {
            return Err("mesh_regions_to_brep: repeated point in boundary chain".into());
        }
        cumulative.push(cumulative.last().unwrap() + step);
    }
    let total = *cumulative.last().unwrap();
    Ok(cumulative.into_iter().map(|value| value / total).collect())
}

/// Map a curve lying in a plane into the plane's parameter space (exact for
/// the affine `make_plane` patches: unit frame directions, arc-length uv).
pub(super) fn affine_pcurve(
    curve: &NurbsCurve,
    origin: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
) -> Result<NurbsCurve, String> {
    let points = curve
        .control_points
        .iter()
        .map(|control| {
            let delta = control.point()?.sub(origin);
            Ok(Vec4 {
                x: delta.dot(x_axis) * control.w,
                y: delta.dot(y_axis) * control.w,
                z: 0.0,
                w: control.w,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), points)
}

pub(super) fn parameter_segment(u0: f64, v0: f64, u1: f64, v1: f64) -> Result<NurbsCurve, String> {
    make_line(Vec3::new(u0, v0, 0.0), Vec3::new(u1, v1, 0.0))
}

impl<'a> RegionBrepBuilder<'a> {
    pub(super) fn allocate_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    pub(super) fn chain_points(&self, chain: &Chain) -> Vec<Vec3> {
        chain.vertices.iter().map(|&v| self.data.verts[v]).collect()
    }

    pub(super) fn welded_vertex_id(&mut self, welded: usize) -> u64 {
        if let Some(&id) = self.vertex_ids.get(&welded) {
            return id;
        }
        let id = self.next_id;
        self.next_id += 1;
        self.vertices.push(VertexRecord {
            id,
            point: self.data.verts[welded],
        });
        self.vertex_ids.insert(welded, id);
        id
    }

    pub(super) fn push_edge(&mut self, mut edge: EdgeRecord) -> u64 {
        edge.id = self.next_id;
        self.next_id += 1;
        self.edge_index.insert(edge.id, self.edges.len());
        let id = edge.id;
        self.edges.push(edge);
        id
    }

    pub(super) fn edge_curve(&self, edge_id: u64) -> &NurbsCurve {
        &self.edges[self.edge_index[&edge_id]].curve
    }

    /// Direction of the chain step `(a, b)` relative to the chain's stored
    /// vertex order (`Some(true)` = along the stored order).
    fn chain_step_direction(chain: &Chain, a: usize, b: usize) -> Option<bool> {
        let n = chain.vertices.len();
        let last = if chain.closed { n } else { n - 1 };
        for index in 0..last {
            let s = chain.vertices[index];
            let e = chain.vertices[(index + 1) % n];
            if s == a && e == b {
                return Some(true);
            }
            if s == b && e == a {
                return Some(false);
            }
        }
        None
    }

    /// Split a region's ordered boundary cycle into chain traversals.
    pub(super) fn cycle_traversals(&self, cycle: &[usize]) -> Result<Vec<Traversal>, String> {
        let n = cycle.len();
        if n < 2 {
            return Err("mesh_regions_to_brep: degenerate boundary cycle".into());
        }
        let step_chain = (0..n)
            .map(|i| {
                self.edge_chain
                    .get(&edge_key(cycle[i], cycle[(i + 1) % n]))
                    .copied()
                    .ok_or_else(|| {
                        "mesh_regions_to_brep: boundary step outside every chain".to_string()
                    })
            })
            .collect::<Result<Vec<_>, _>>()?;
        if step_chain.iter().all(|&c| c == step_chain[0]) {
            let chain = &self.chains[step_chain[0]];
            if !chain.closed {
                return Err("mesh_regions_to_brep: cycle traverses an open chain only".into());
            }
            let forward = Self::chain_step_direction(chain, cycle[0], cycle[1])
                .ok_or("mesh_regions_to_brep: cycle step not found in its ring chain")?;
            return Ok(vec![Traversal {
                chain: step_chain[0],
                forward_along_chain: forward,
            }]);
        }
        let start = (0..n)
            .find(|&i| step_chain[i] != step_chain[(i + n - 1) % n])
            .expect("mixed chains imply a boundary between them");
        let mut traversals: Vec<Traversal> = Vec::new();
        let mut index = 0;
        while index < n {
            let at = (start + index) % n;
            let chain_id = step_chain[at];
            let mut run = 0;
            while index + run < n && step_chain[(start + index + run) % n] == chain_id {
                run += 1;
            }
            let chain = &self.chains[chain_id];
            if chain.closed || run != chain.vertices.len() - 1 {
                return Err(
                    "mesh_regions_to_brep: partial chain traversal (pinched region boundary)"
                        .into(),
                );
            }
            let a = cycle[at];
            let b = cycle[(at + 1) % n];
            let forward = Self::chain_step_direction(chain, a, b)
                .ok_or("mesh_regions_to_brep: traversal step not found in its chain")?;
            traversals.push(Traversal {
                chain: chain_id,
                forward_along_chain: forward,
            });
            index += run;
        }
        Ok(traversals)
    }

    /// Build (or reuse) the shared edge of an open chain between planar
    /// carriers: a single line edge when the polyline is straight (the exact
    /// plane×plane case), an interpolated cubic through the polyline
    /// otherwise.
    pub(super) fn ensure_open_chain_edge(&mut self, chain_id: usize) -> Result<(), String> {
        if self.chain_edges[chain_id].is_some() {
            return Ok(());
        }
        let chain = &self.chains[chain_id];
        if chain.closed {
            return Err(
                "mesh_regions_to_brep: closed boundary ring not adjacent to any revolve wall"
                    .into(),
            );
        }
        let points = self.chain_points(chain);
        let (first, last) = (chain.vertices[0], *chain.vertices.last().unwrap());
        let curve = if max_deviation_from_segment(&points) <= self.tol {
            make_line(points[0], *points.last().unwrap())?
        } else {
            let parameters = chord_parameters(&points)?;
            interpolate_curve(&points, 3.min(points.len() - 1), &parameters)?
        };
        let start_vertex_id = self.welded_vertex_id(first);
        let end_vertex_id = self.welded_vertex_id(last);
        let edge_id = self.push_edge(EdgeRecord {
            id: 0,
            curve,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id,
            end_vertex_id,
            degenerate: false,
            name: None,
        });
        self.chain_edges[chain_id] = Some(ChainEdgeInfo {
            edge_id,
            curve_along_chain: true,
            ring: None,
        });
        Ok(())
    }

    /// Signed circulation (in radians, ±2π for a ring) of an ordered vertex
    /// cycle around an axis.
    pub(super) fn cycle_circulation(
        &self,
        cycle: &[usize],
        axis_point: Vec3,
        axis: Vec3,
    ) -> Result<f64, String> {
        let x_axis = axis.perpendicular()?;
        let y_axis = axis.cross(x_axis);
        let angle_of = |p: Vec3| -> f64 {
            let d = p.sub(axis_point);
            let radial = d.sub(axis.scale(d.dot(axis)));
            radial.dot(y_axis).atan2(radial.dot(x_axis))
        };
        let mut total = 0.0;
        for index in 0..cycle.len() {
            let a = angle_of(self.data.verts[cycle[index]]);
            let b = angle_of(self.data.verts[cycle[(index + 1) % cycle.len()]]);
            total += wrap_to_pi(b - a);
        }
        Ok(total)
    }
}

/// Rebuild a `BrepSolid` whose faces are the mesh's segmented regions.  The
/// result always passes full topology validation (watertight edge pairing,
/// Euler accounting, pcurve consistency) or the call returns `Err` — an
/// invalid solid is never returned.  See the module-level v1 scope notes.
pub fn mesh_regions_to_brep(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
) -> Result<BrepSolid, String> {
    if !(options.deflection_angle_deg > 0.0) || !options.deflection_angle_deg.is_finite() {
        return Err("mesh_regions_to_brep: deflection angle must be positive".into());
    }
    if !(options.fit_tolerance > 0.0) || !options.fit_tolerance.is_finite() {
        return Err("mesh_regions_to_brep: fit tolerance must be positive".into());
    }
    let data = build_mesh_data(positions, indices, options)?;
    let seg = segment_prepared(&data, options);

    // Documented fallback: a mesh with NO recognized carrier at all is the
    // faceted importer's business (triangle-per-face, exact for the mesh).
    if seg
        .regions
        .iter()
        .all(|region| matches!(region.carrier, RegionCarrier::Freeform))
    {
        let weld = if options.weld_tolerance > 0.0 {
            options.weld_tolerance
        } else {
            -1.0
        };
        let index_arg = (!indices.is_empty()).then_some(indices);
        // The lossy exit for this stringly lane: the class is dropped, the text kept.
        return mesh_to_faceted_brep(positions, index_arg, weld).map_err(|refusal| refusal.message);
    }
    if let Some(region) = seg
        .regions
        .iter()
        .find(|region| matches!(region.carrier, RegionCarrier::Freeform))
    {
        return Err(format!(
            "mesh_regions_to_brep: region {} is freeform; mixing freeform and analytic \
             regions in one shell is not supported (v1) — import via mesh_to_faceted_brep",
            region.id
        ));
    }
    if let Some(region) = seg.regions.iter().find(|region| {
        matches!(
            region.carrier,
            RegionCarrier::Sphere { .. } | RegionCarrier::Torus { .. }
        )
    }) {
        return Err(format!(
            "mesh_regions_to_brep: region {} is a {} — sphere/torus faces need pole/seam \
             topology not supported in v1",
            region.id,
            region.carrier.kind()
        ));
    }
    if seg
        .triangle_region_ids
        .iter()
        .any(|&id| id == UNASSIGNED_REGION)
    {
        return Err("mesh_regions_to_brep: mesh has unassignable degenerate triangles".into());
    }

    // Region-boundary graph over welded mesh edges.
    let region_of = &seg.triangle_region_ids;
    let mut boundary: HashMap<(usize, usize), (u32, u32)> = HashMap::default();
    for (&key, incident) in &data.edges {
        if incident.len() != 2 {
            return Err(format!(
                "mesh_regions_to_brep: mesh is not closed-manifold (edge with {} incident \
                 triangles)",
                incident.len()
            ));
        }
        let (ra, rb) = (
            region_of[incident[0] as usize],
            region_of[incident[1] as usize],
        );
        if ra != rb {
            boundary.insert(key, (ra.min(rb), ra.max(rb)));
        }
    }
    if boundary.is_empty() {
        return Err(
            "mesh_regions_to_brep: single boundary-less region cannot form a face loop".into(),
        );
    }

    let mut vertex_boundary: HashMap<usize, Vec<(usize, usize)>> = HashMap::default();
    for &key in boundary.keys() {
        vertex_boundary.entry(key.0).or_default().push(key);
        vertex_boundary.entry(key.1).or_default().push(key);
    }
    for list in vertex_boundary.values_mut() {
        list.sort_unstable();
    }
    let is_corner = |vertex: usize| -> bool {
        let list = &vertex_boundary[&vertex];
        list.len() != 2 || boundary[&list[0]] != boundary[&list[1]]
    };

    // Chains: walk from corners through valence-2 vertices; leftovers are
    // closed rings.
    let mut chains: Vec<Chain> = Vec::new();
    let mut edge_chain: HashMap<(usize, usize), usize> = HashMap::default();
    let other_end = |key: (usize, usize), vertex: usize| -> usize {
        if key.0 == vertex {
            key.1
        } else {
            key.0
        }
    };
    let mut corner_vertices: Vec<usize> = vertex_boundary
        .keys()
        .copied()
        .filter(|&v| is_corner(v))
        .collect();
    corner_vertices.sort_unstable();
    for &corner in &corner_vertices {
        let incident = vertex_boundary[&corner].clone();
        for start_edge in incident {
            if edge_chain.contains_key(&start_edge) {
                continue;
            }
            let chain_id = chains.len();
            let mut vertices = vec![corner];
            let mut current_edge = start_edge;
            let mut current = other_end(start_edge, corner);
            edge_chain.insert(start_edge, chain_id);
            while !is_corner(current) {
                vertices.push(current);
                let next_edge = vertex_boundary[&current]
                    .iter()
                    .copied()
                    .find(|&e| e != current_edge)
                    .expect("valence-2 vertex has a continuation");
                edge_chain.insert(next_edge, chain_id);
                current_edge = next_edge;
                current = other_end(next_edge, current);
            }
            vertices.push(current);
            chains.push(Chain {
                vertices,
                closed: false,
            });
        }
    }
    let mut remaining: Vec<(usize, usize)> = boundary
        .keys()
        .copied()
        .filter(|key| !edge_chain.contains_key(key))
        .collect();
    remaining.sort_unstable();
    for start_edge in remaining {
        if edge_chain.contains_key(&start_edge) {
            continue;
        }
        let chain_id = chains.len();
        let start = start_edge.0;
        let mut vertices = vec![start];
        let mut current_edge = start_edge;
        let mut current = start_edge.1;
        edge_chain.insert(start_edge, chain_id);
        while current != start {
            vertices.push(current);
            let next_edge = vertex_boundary[&current]
                .iter()
                .copied()
                .find(|&e| e != current_edge)
                .expect("ring vertex has a continuation");
            edge_chain.insert(next_edge, chain_id);
            current_edge = next_edge;
            current = other_end(next_edge, current);
        }
        chains.push(Chain {
            vertices,
            closed: true,
        });
    }

    // Ordered boundary cycles per region (directed walks; the triangle
    // winding makes them counterclockwise seen from outside the solid).
    let region_count = seg.regions.len();
    let mut region_tris: Vec<Vec<u32>> = vec![Vec::new(); region_count];
    for (t, &r) in region_of.iter().enumerate() {
        region_tris[r as usize].push(t as u32);
    }
    let mut region_cycles: Vec<Vec<Vec<usize>>> = Vec::with_capacity(region_count);
    for tris in &region_tris {
        let mut out: HashMap<usize, usize> = HashMap::default();
        for &t in tris {
            let tri = &data.tris[t as usize];
            for corner in 0..3 {
                let a = tri.verts[corner];
                let b = tri.verts[(corner + 1) % 3];
                if a == b || !boundary.contains_key(&edge_key(a, b)) {
                    continue;
                }
                if let Some(previous) = out.insert(a, b) {
                    if previous != b {
                        return Err(
                            "mesh_regions_to_brep: region boundary pinches at a vertex (two \
                             outgoing boundary edges)"
                                .into(),
                        );
                    }
                }
            }
        }
        let mut starts: Vec<usize> = out.keys().copied().collect();
        starts.sort_unstable();
        let mut visited: rustc_hash::FxHashSet<usize> = rustc_hash::FxHashSet::default();
        let mut cycles = Vec::new();
        for &start in &starts {
            if visited.contains(&start) {
                continue;
            }
            let mut cycle = vec![start];
            visited.insert(start);
            let mut current = out[&start];
            while current != start {
                if !visited.insert(current) {
                    return Err("mesh_regions_to_brep: region boundary walk self-crosses".into());
                }
                cycle.push(current);
                current = *out
                    .get(&current)
                    .ok_or("mesh_regions_to_brep: region boundary walk breaks")?;
            }
            cycles.push(cycle);
        }
        region_cycles.push(cycles);
    }

    let mut builder = RegionBrepBuilder {
        data: &data,
        tol: (options.fit_tolerance * data.diag).max(1e-12),
        chains,
        edge_chain,
        chain_edges: Vec::new(),
        vertices: Vec::new(),
        vertex_ids: HashMap::default(),
        edges: Vec::new(),
        edge_index: HashMap::default(),
        next_id: 1,
    };
    builder.chain_edges = (0..builder.chains.len()).map(|_| None).collect();

    // Curved regions first: walls and patches create the exact circle / arc /
    // ruling edges their planar neighbors then share.
    let mut faces: Vec<FaceRecord> = Vec::new();
    for (index, region) in seg.regions.iter().enumerate() {
        match &region.carrier {
            RegionCarrier::Cylinder { .. } | RegionCarrier::Cone { .. } => {
                let face = build_revolved_region_face(&mut builder, region, &region_cycles[index])?;
                faces.push(face);
            }
            _ => {}
        }
    }
    for (index, region) in seg.regions.iter().enumerate() {
        if let RegionCarrier::Plane { origin, normal } = region.carrier {
            let face = build_planar_region_face(
                &mut builder,
                region.id,
                origin,
                normal,
                &region_cycles[index],
            )?;
            faces.push(face);
        }
    }

    let shell_id = builder.allocate_id();
    let solid_id = builder.allocate_id();
    let mut solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: solid_id,
        vertices: builder.vertices,
        edges: builder.edges,
        shells: vec![ShellRecord {
            id: shell_id,
            faces,
        }],
        genus: 0,
    };
    let vertex_count = solid.vertices.len() as i64;
    let edge_count = solid.edges.len() as i64;
    let face_count = solid.shells[0].faces.len() as i64;
    let hole_count: i64 = solid.shells[0]
        .faces
        .iter()
        .map(|face| face.loops.len().saturating_sub(1) as i64)
        .sum();
    let euler = vertex_count - edge_count + face_count - hole_count;
    let numerator = 2 - euler;
    if numerator < 0 || numerator % 2 != 0 {
        return Err(format!(
            "mesh_regions_to_brep: rebuilt topology has non-manifold Euler count V-E+F-H={euler}"
        ));
    }
    solid.genus = numerator / 2;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!(
            "mesh_regions_to_brep: rebuilt solid failed validation: {issues:?}"
        ));
    }
    let _caller = crate::mass_caller("mesh.brep_builder");
    let volume = solid_signed_volume(&solid)?;
    if !(volume > 0.0) {
        return Err(format!(
            "mesh_regions_to_brep: rebuilt solid is inverted (signed volume {volume})"
        ));
    }
    Ok(solid)
}
