use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

pub(in crate::boolean) struct SourceEdge {
    pub(in crate::boolean) curve: crate::NurbsCurve,
    pub(in crate::boolean) t0: f64,
    pub(in crate::boolean) t1: f64,
    pub(in crate::boolean) start: Vec3,
    pub(in crate::boolean) end: Vec3,
    pub(in crate::boolean) degenerate: bool,
    pub(in crate::boolean) name: Option<String>,
    /// A CONSTRUCTED section: the source is an imprint piece the Boolean
    /// marched on two carriers, not a boundary edge of an operand (vendor
    /// geometry) nor a derived sub-span of one. Only such an edge's trims may
    /// be refreshed from the edge after vertex settling (`vertices.rs`).
    pub(in crate::boolean) constructed: bool,
    // Identity of the ORIGINAL boundary edge this source came from
    // `(operand, edge_id)`, for uncut faces passed through the arrangement.
    // `None` for imprint / derived (section) edges, which have no pre-boolean
    // identity. Two coedges of one operand that reference DIFFERENT original
    // edge ids are, by construction, distinct 1-cells of a valid input and
    // must never weld into one assembler edge — even when their geometry is
    // coincident within the weld radius (thin seam-cut bands whose rims sit a
    // single weld-tolerance apart). This keeps the weld from confusing a hole
    // wall's two rims, or a rim with its neighbouring plate rim.
    pub(in crate::boolean) boundary_key: Option<(u8, u64)>,
}

/// id-index over the immutable operands + imprint, built once before the
/// assemble loop. Every lookup returns the exact record the previous linear
/// `.iter().find(id==)` scans returned; only the O(N)-per-coedge cost changes
/// (assembly ran these scans once per coedge = O(coedges * (E + V + F))).
pub(super) struct AssembleIndex<'a> {
    operand_edges: HashMap<u8, HashMap<u64, &'a EdgeRecord>>,
    operand_vertices: HashMap<u8, HashMap<u64, Vec3>>,
    operand_face_names: HashMap<u8, HashMap<u64, Option<String>>>,
    imprint_pieces: HashMap<u64, &'a ImprintPieceRecord>,
    imprint_vertices: HashMap<u64, Vec3>,
}

impl<'a> AssembleIndex<'a> {
    pub(super) fn build(solids: &HashMap<u8, &'a BrepSolid>, imprint: &'a ImprintResultRecord) -> Self {
        let mut operand_edges: HashMap<u8, HashMap<u64, &'a EdgeRecord>> = HashMap::default();
        let mut operand_vertices: HashMap<u8, HashMap<u64, Vec3>> = HashMap::default();
        let mut operand_face_names: HashMap<u8, HashMap<u64, Option<String>>> = HashMap::default();
        for (&operand, &solid) in solids {
            operand_edges.insert(
                operand,
                solid.edges.iter().map(|edge| (edge.id, edge)).collect(),
            );
            operand_vertices.insert(
                operand,
                solid
                    .vertices
                    .iter()
                    .map(|vertex| (vertex.id, vertex.point))
                    .collect(),
            );
            operand_face_names.insert(
                operand,
                solid
                    .shells
                    .iter()
                    .flat_map(|shell| &shell.faces)
                    .map(|face| (face.id, face.name.clone()))
                    .collect(),
            );
        }
        AssembleIndex {
            operand_edges,
            operand_vertices,
            operand_face_names,
            imprint_pieces: imprint
                .pieces
                .iter()
                .map(|piece| (piece.id, piece))
                .collect(),
            imprint_vertices: imprint
                .vertices
                .iter()
                .map(|vertex| (vertex.id, vertex.point))
                .collect(),
        }
    }
}

fn vertex_in_solid(index: &AssembleIndex, operand: u8, id: u64) -> Result<Vec3, KernelRefusal> {
    index
        .operand_vertices
        .get(&operand)
        .and_then(|vertices| vertices.get(&id))
        .copied()
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("boolean assembly: missing vertex {id}")))
}

pub(super) fn face_name_in_solids(index: &AssembleIndex, operand: u8, face_id: u64) -> Option<String> {
    index
        .operand_face_names
        .get(&operand)?
        .get(&face_id)
        .cloned()
        .flatten()
}

/// New intersection edges are named after their two supporting faces using
/// the application's `FACEA|FACEB` convention (alphabetical order), so a
/// rebuilt model derives the same edge identity without geometric matching.
fn imprint_edge_name(
    index: &AssembleIndex,
    supports: &[crate::imprint::FaceKey; 2],
) -> Option<String> {
    let mut names = [
        face_name_in_solids(index, supports[0].operand, supports[0].face_id)?,
        face_name_in_solids(index, supports[1].operand, supports[1].face_id)?,
    ];
    names.sort();
    Some(format!("{}|{}", names[0], names[1]))
}

fn boundary_source_edge(
    index: &AssembleIndex,
    operand: u8,
    edge_id: u64,
) -> Result<SourceEdge, KernelRefusal> {
    let edge = *index
        .operand_edges
        .get(&operand)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("boolean assembly: missing operand {operand}")))?
        .get(&edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("boolean assembly: missing boundary edge {edge_id}")))?;
    Ok(SourceEdge {
        curve: edge.curve.clone(),
        t0: edge.t0,
        t1: edge.t1,
        start: vertex_in_solid(index, operand, edge.start_vertex_id)?,
        end: vertex_in_solid(index, operand, edge.end_vertex_id)?,
        degenerate: edge.degenerate,
        name: edge.name.clone(),
        constructed: false,
        boundary_key: Some((operand, edge_id)),
    })
}

pub(super) fn source_edge(source: &FragmentEdgeSource, index: &AssembleIndex) -> Result<SourceEdge, KernelRefusal> {
    match source {
        FragmentEdgeSource::Boundary { operand, edge_id }
        | FragmentEdgeSource::SharedBoundary { operand, edge_id } => {
            boundary_source_edge(index, *operand, *edge_id)
        }
        FragmentEdgeSource::Imprint { piece_id } => {
            let piece = *index
                .imprint_pieces
                .get(piece_id)
                .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("boolean assembly: missing imprint piece {piece_id}")))?;
            let point = |vertex_id| {
                index
                    .imprint_vertices
                    .get(&vertex_id)
                    .copied()
                    .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", "boolean assembly: missing imprint vertex"))
            };
            Ok(SourceEdge {
                curve: piece.curve.clone(),
                t0: piece.t0,
                t1: piece.t1,
                start: point(piece.start_vertex_id)?,
                end: point(piece.end_vertex_id)?,
                degenerate: false,
                name: imprint_edge_name(index, &piece.support_faces),
                constructed: true,
                boundary_key: None,
            })
        }
        FragmentEdgeSource::Derived {
            curve,
            t0,
            t1,
            start,
            end,
        } => Ok(SourceEdge {
            curve: curve.clone(),
            t0: *t0,
            t1: *t1,
            start: *start,
            end: *end,
            degenerate: start.sub(*end).length() <= 1e-8,
            name: None,
            constructed: false,
            boundary_key: None,
        }),
    }
}

pub(in crate::boolean) struct Assembler {
    pub(in crate::boolean) tolerance: f64,
    pub(in crate::boolean) vertices: Vec<VertexRecord>,
    // Source positions retain their lookup records after a witnessed edge
    // correspondence joins them. Future coedges resolve those records to the
    // same canonical vertex instead of recreating the old endpoint identity.
    pub(in crate::boolean) vertex_aliases: HashMap<u64, u64>,
    pub(in crate::boolean) edges: Vec<EdgeRecord>,
    pub(in crate::boolean) edge_use_counts: HashMap<u64, usize>,
    // Result `forward` of the FIRST coedge attached to each edge, keyed by
    // edge id. Consulted only while an edge is still one-use (a weld candidate)
    // so the weld can refuse a mate that would give the edge two same-sense
    // coedges — an unconditionally non-manifold incidence (topology.rs). The
    // entry is written once at edge creation and never resynced: a stale or
    // missing entry only makes the guard a no-op (today's behaviour), and by
    // the time later passes flip a coedge's `forward` the edge is already
    // two-use, so the map is never read for it again.
    pub(in crate::boolean) edge_first_forward: HashMap<u64, bool>,
    // Original `(operand, edge_id)` of the boundary source that CREATED each
    // assembler edge (absent for imprint/derived-sourced edges). A weld
    // candidate whose incoming source shares the operand but not the edge id
    // is a distinct 1-cell of the input and is refused — see `SourceEdge`.
    pub(in crate::boolean) edge_boundary_key: HashMap<u64, (u8, u64)>,
    // Edges created from an imprint piece (a constructed section). A weld of a
    // later source onto one of these keeps the edge; a weld of an imprint
    // source onto a boundary edge leaves the boundary edge unmarked.
    pub(in crate::boolean) imprint_edges: HashSet<u64>,
    pub(in crate::boolean) next_vertex_id: u64,
    pub(in crate::boolean) next_edge_id: u64,
    pub(in crate::boolean) next_coedge_id: u64,
    pub(in crate::boolean) next_loop_id: u64,
    pub(in crate::boolean) next_face_id: u64,
}

pub(super) fn snap_edge_curve_endpoints(
    mut curve: crate::NurbsCurve,
    t0: f64,
    t1: f64,
    start: Vec3,
    end: Vec3,
) -> Result<crate::NurbsCurve, KernelRefusal> {
    let [domain_start, domain_end] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    // NurbsCurve::split refuses parameters within its ABSOLUTE knot
    // tolerance (1e-9) of the domain ends; the skip epsilon must cover that
    // or a near-edge trim parameter slips past the guard and errors.
    let epsilon = ((domain_end - domain_start).abs().max(1.0) * 1e-10).max(2e-9);
    if t0 > domain_start + epsilon && t0 < domain_end - epsilon {
        curve = curve.split(t0).or_refuse(KernelStage::Sew, "split")?.1;
    }
    let domain = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    if t1 < domain[1] - epsilon && t1 > domain[0] + epsilon {
        curve = curve.split(t1).or_refuse(KernelStage::Sew, "split")?.0;
    } else if std::env::var("BREP_DEBUG_SUBRANGE").is_ok() && t1 < domain[1] - epsilon {
        eprintln!("abnormal subrange skip in snap_edge_curve_endpoints");
    }
    let mut controls = curve.control_points.clone();
    let first_weight = controls[0].w;
    controls[0] = crate::Vec4::from_point(start, first_weight);
    let last = controls.len() - 1;
    let last_weight = controls[last].w;
    controls[last] = crate::Vec4::from_point(end, last_weight);
    let snapped = crate::NurbsCurve::new(curve.degree, curve.knots.clone(), controls).or_refuse(KernelStage::Sew, "NurbsCurve::new")?;
    Ok(snapped)
}

impl Assembler {
    /// Vertex ids are dense (1..=len) because only `vertex()` appends with a
    /// sequential id. This lookup checks that invariant instead of assuming
    /// it, so a future change that breaks density fails loudly rather than
    /// silently welding edges to the wrong vertex position.
    fn vertex_point(&self, id: u64) -> Result<Vec3, KernelRefusal> {
        let id = self.canonical_vertex(id);
        let record = id
            .checked_sub(1)
            .and_then(|offset| usize::try_from(offset).ok())
            .and_then(|index| self.vertices.get(index))
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("assembler: vertex id {id} out of range")))?;
        if record.id != id {
            return Err(KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!(
                "assembler: vertex ids are not dense (id {id} resolved to record {})",
                record.id
            )));
        }
        Ok(record.point)
    }

    fn canonical_vertex(&self, mut id: u64) -> u64 {
        while let Some(&next) = self.vertex_aliases.get(&id) {
            id = next;
        }
        id
    }

    fn join_weld_endpoints(
        &mut self, source_start: Vec3, source_end: Vec3,
        target_start: u64, target_end: u64, reversed: bool,
    ) -> Result<bool, KernelRefusal> {
        let source_ids = [self.vertex(source_start), self.vertex(source_end)];
        let target_ids = if reversed { [target_end, target_start] }
                         else { [target_start, target_end] };
        let mut aliases = self.vertex_aliases.clone();
        fn root(aliases: &HashMap<u64, u64>, mut id: u64) -> u64 {
            while let Some(&next) = aliases.get(&id) { id = next; }
            id
        }
        for (from, to) in source_ids.into_iter().zip(target_ids) {
            let (from, to) = (root(&aliases, from), root(&aliases, to));
            if from != to { aliases.insert(from, to); }
        }
        if aliases == self.vertex_aliases { return Ok(true); }
        let band = self.tolerance.max(1e-7);
        for vertex in &self.vertices {
            let target = root(&aliases, vertex.id);
            if vertex.point.sub(self.vertex_point(target)?).length() > band {
                return Ok(false);
            }
        }
        if self.edges.iter().any(|edge| !edge.degenerate
            && edge.start_vertex_id != edge.end_vertex_id
            && root(&aliases, edge.start_vertex_id) == root(&aliases, edge.end_vertex_id))
        {
            return Ok(false);
        }
        // Validate every geometric edit before committing the correspondence.
        let mut replacements = Vec::new();
        for (index, edge) in self.edges.iter().enumerate() {
            let start = root(&aliases, edge.start_vertex_id);
            let end = root(&aliases, edge.end_vertex_id);
            if (start, end) == (edge.start_vertex_id, edge.end_vertex_id) { continue; }
            let curve = snap_edge_curve_endpoints(edge.curve.clone(), edge.t0, edge.t1,
                self.vertex_point(start)?, self.vertex_point(end)?)?;
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            replacements.push((index, start, end, curve, t0, t1));
        }
        for (index, start, end, curve, t0, t1) in replacements {
            let edge = &mut self.edges[index];
            edge.start_vertex_id = start;
            edge.end_vertex_id = end;
            edge.curve = curve;
            edge.t0 = t0;
            edge.t1 = t1;
        }
        self.vertex_aliases = aliases;
        Ok(true)
    }

    fn vertex(&mut self, point: Vec3) -> u64 {
        // Offset/intersection endpoints are independently fitted and can
        // differ by the sewing tolerance. Welding those endpoint vertices is
        // what makes consecutive coedges form a topological loop; the edge
        // geometry itself remains unchanged.
        let tolerance = self.tolerance.max(1e-7);
        if let Some(vertex) = self
            .vertices
            .iter()
            .find(|vertex| vertex.point.sub(point).length() <= tolerance)
        {
            return self.canonical_vertex(vertex.id);
        }
        let id = self.next_vertex_id;
        self.next_vertex_id += 1;
        self.vertices.push(VertexRecord { id, point });
        id
    }

    pub(in crate::boolean) fn edge(
        &mut self,
        source: SourceEdge,
        incoming_forward: bool,
    ) -> Result<(u64, bool), KernelRefusal> {
        let weld_tolerance = assembler_weld(self.tolerance);
        let identity_tolerance = self.tolerance.max(1e-7);
        let source_degenerate =
            source.degenerate || source.start.sub(source.end).length() <= identity_tolerance;
        // A degenerate pole edge is a face-local topological placeholder,
        // not a shared boundary. Geometrically coincident pole edges must
        // remain distinct so each is referenced by exactly one coedge.
        if !source_degenerate {
            let mut welded: Option<(usize, bool)> = None;
            for index in 0..self.edges.len() {
                let edge = &self.edges[index];
                if self.edge_use_counts.get(&edge.id).copied().unwrap_or(0) >= 2 {
                    continue;
                }
                if weld_refused_by_identity(
                    source.boundary_key,
                    self.edge_boundary_key.get(&edge.id).copied(),
                ) {
                    continue;
                }
                let start = self.vertex_point(edge.start_vertex_id)?;
                let end = self.vertex_point(edge.end_vertex_id)?;
                if edge.degenerate || start.sub(end).length() <= identity_tolerance {
                    continue;
                }
                let direct_gap = start.sub(source.start).length()
                    .max(end.sub(source.end).length());
                let reversed_gap = start.sub(source.end).length()
                    .max(end.sub(source.start).length());
                // A broad curve-comparison band cannot establish endpoint
                // identity. A later conformance piece may connect these two
                // nearby vertices, even if that short edge is not built yet.
                let direct = direct_gap <= identity_tolerance;
                let reversed = reversed_gap <= identity_tolerance
                    && (!direct || reversed_gap < direct_gap);
                if !direct && !reversed {
                    continue;
                }
                let same_curve = [0.2, 0.5, 0.8].into_iter().all(|fraction| {
                    let source_parameter = source.t0 + (source.t1 - source.t0) * fraction;
                    let edge_fraction = if reversed { 1.0 - fraction } else { fraction };
                    let edge_parameter = edge.t0 + (edge.t1 - edge.t0) * edge_fraction;
                    source
                        .curve
                        .evaluate(source_parameter)
                        .and_then(|source_point| {
                            edge.curve
                                .evaluate(edge_parameter)
                                .map(|edge_point| source_point.sub(edge_point).length())
                        })
                        .is_ok_and(|distance| distance <= weld_tolerance)
                });
                // Rational arcs parameterize non-uniformly, so the same
                // circular span represented as a fresh arc vs a subrange of
                // a longer arc evaluates DIFFERENT points at equal
                // fractions. When fraction sampling disagrees, accept the
                // weld if the source samples project onto the candidate's
                // trimmed curve within tolerance.
                let same_curve = same_curve
                    || [0.2, 0.5, 0.8].into_iter().all(|fraction| {
                        let source_parameter = source.t0 + (source.t1 - source.t0) * fraction;
                        source
                            .curve
                            .evaluate(source_parameter)
                            .ok()
                            .and_then(|point| project_point_to_curve(&edge.curve, point).ok())
                            .is_some_and(|projection| {
                                let low = edge.t0.min(edge.t1);
                                let high = edge.t0.max(edge.t1);
                                let margin = (high - low).abs() * 1e-6 + 1e-12;
                                projection.distance <= weld_tolerance
                                    && projection.u >= low - margin
                                    && projection.u <= high + margin
                            })
                    });
                if same_curve {
                    // Two coedges on one edge must have OPPOSITE sense; a
                    // same-sense pair is non-manifold (topology.rs). When a
                    // coincident twin edge sits within the weld radius (thin
                    // rims one weld-tolerance apart on imperfect imported
                    // geometry), the first geometric match is not necessarily
                    // the true mate. Refuse a weld that would collide senses
                    // and keep scanning for a candidate that pairs opposite —
                    // falling through to a fresh edge record if none does, so
                    // the real mate can still claim it later.
                    let would_be_forward = if reversed {
                        !incoming_forward
                    } else {
                        incoming_forward
                    };
                    if self.edge_first_forward.get(&edge.id) == Some(&would_be_forward) {
                        continue;
                    }
                    let endpoints = (edge.start_vertex_id, edge.end_vertex_id);
                    if !self.join_weld_endpoints(source.start, source.end,
                        endpoints.0, endpoints.1, reversed)? { continue; }
                    welded = Some((index, reversed));
                    break;
                }
            }
            if let Some((index, reversed)) = welded {
                let edge = &mut self.edges[index];
                // The mate arriving from the other operand may carry the
                // only name for this shared edge; keep the first name seen.
                if edge.name.is_none() {
                    edge.name = source.name;
                }
                let id = edge.id;
                *self.edge_use_counts.entry(id).or_default() += 1;
                return Ok((id, reversed));
            }
        }
        // A CLOSED edge (full circle/seam: start == end but real interior
        // geometry) is shared boundary topology exactly like an open edge —
        // a cap of a closed surface is one closed edge referenced by both
        // adjacent faces (Golovanov §5.6). Without this weld the two
        // operands' coincident copies each stay one-use and the shell splits
        // at the seam (the SweepFace cone-seam failure). Only true point
        // placeholders (poles) stay face-local.
        let interior_sweeps_away =
            |curve: &crate::NurbsCurve, t0: f64, t1: f64, anchor: Vec3| -> bool {
                [0.25, 0.5, 0.75].into_iter().any(|fraction| {
                    curve
                        .evaluate(t0 + (t1 - t0) * fraction)
                        .is_ok_and(|point| point.sub(anchor).length() > 10.0 * weld_tolerance)
                })
            };
        // Admit ANY genuinely closed source — not only degenerate-flagged
        // ones. The scan was built for the cone-seam class whose closed
        // edges arrive flagged degenerate, but a REAL closed ring (the
        // inscribed sphere's equator presented as the cylinder's own cap
        // edge via SharedBoundary) is equally shared boundary topology, and
        // without admission the identical-key copies stay one-use forever.
        // Every downstream gate (identity refusal, seam match, 7-point
        // same-curve, orientation, same-sense refusal) still applies.
        let admit_any_closed =
            std::env::var("BREP_CLOSED_WELD_ANY").as_deref() != Ok("0");
        let source_closed = (source_degenerate || admit_any_closed)
            && source.start.sub(source.end).length() <= identity_tolerance
            && interior_sweeps_away(&source.curve, source.t0, source.t1, source.start);
        if source_closed && std::env::var("BREP_DEBUG_EDGE").as_deref() == Ok("weld") {
            eprintln!(
                "closed-weld scan: source start=({:.3},{:.3},{:.3}) key={:?} candidates={}",
                source.start.x, source.start.y, source.start.z,
                source.boundary_key,
                self.edges.len()
            );
        }
        if source_closed {
            let mut welded: Option<(usize, bool)> = None;
            for index in 0..self.edges.len() {
                let edge = &self.edges[index];
                if self.edge_use_counts.get(&edge.id).copied().unwrap_or(0) >= 2 {
                    continue;
                }
                if weld_refused_by_identity(
                    source.boundary_key,
                    self.edge_boundary_key.get(&edge.id).copied(),
                ) {
                    continue;
                }
                let start = self.vertex_point(edge.start_vertex_id)?;
                let end = self.vertex_point(edge.end_vertex_id)?;
                // The candidate must be a closed edge with real interior
                // geometry sharing the seam vertex; a different seam
                // position means the mate's loop starts elsewhere and the
                // copies cannot share topology.
                if start.sub(end).length() > identity_tolerance
                    || start.sub(source.start).length() > identity_tolerance
                    || !interior_sweeps_away(&edge.curve, edge.t0, edge.t1, start)
                {
                    continue;
                }
                let same_curve = [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875]
                    .into_iter()
                    .all(|fraction| {
                        source
                            .curve
                            .evaluate(source.t0 + (source.t1 - source.t0) * fraction)
                            .ok()
                            .and_then(|point| project_point_to_curve(&edge.curve, point).ok())
                            .is_some_and(|projection| projection.distance <= weld_tolerance)
                    });
                if !same_curve {
                    continue;
                }
                // Quarter-point orientation disambiguation: a closed edge's
                // endpoints coincide, so direction must come from the
                // interior, not the endpoint pairing.
                let span = edge.t1 - edge.t0;
                let quarter_param = source.t0 + (source.t1 - source.t0) * 0.25;
                let quarter = source.curve.evaluate(quarter_param).or_refuse(KernelStage::Sew, "evaluate")?;
                let direct_gap = edge
                    .curve
                    .evaluate(edge.t0 + span * 0.25).or_refuse(KernelStage::Sew, "evaluate")?
                    .sub(quarter)
                    .length();
                let reversed_gap = edge
                    .curve
                    .evaluate(edge.t0 + span * 0.75).or_refuse(KernelStage::Sew, "evaluate")?
                    .sub(quarter)
                    .length();
                let reversed = if direct_gap <= weld_tolerance || reversed_gap <= weld_tolerance {
                    reversed_gap < direct_gap
                } else {
                    // Phase-shifted parameterizations of the same closed
                    // curve: align tangents at the geometric match instead.
                    let projection = project_point_to_curve(&edge.curve, quarter).or_refuse(KernelStage::Sew, "project_point_to_curve")?;
                    let source_tangent = source.curve.derivatives(quarter_param, 1).or_refuse(KernelStage::Sew, "derivatives")?[1];
                    let edge_tangent = edge.curve.derivatives(projection.u, 1).or_refuse(KernelStage::Sew, "derivatives")?[1];
                    source_tangent.dot(edge_tangent) < 0.0
                };
                // Same opposite-sense guard as the open-edge path: a closed
                // rim shared by more than two faces splits into coincident
                // twin edges, and the true mate is the one that pairs opposite
                // sense — not merely the first geometric match within the weld
                // radius. Refuse a same-sense weld and keep scanning.
                let would_be_forward = if reversed {
                    !incoming_forward
                } else {
                    incoming_forward
                };
                if self.edge_first_forward.get(&edge.id) == Some(&would_be_forward) {
                    if std::env::var("BREP_DEBUG_EDGE").as_deref() == Ok("weld") {
                        eprintln!(
                            "closed-weld REFUSED same-sense: edge {} first_forward={:?} incoming would_be={}",
                            edge.id,
                            self.edge_first_forward.get(&edge.id),
                            would_be_forward
                        );
                    }
                    continue;
                }
                let endpoints = (edge.start_vertex_id, edge.end_vertex_id);
                if !self.join_weld_endpoints(source.start, source.end,
                    endpoints.0, endpoints.1, reversed)? { continue; }
                welded = Some((index, reversed));
                break;
            }
            if let Some((index, reversed)) = welded {
                let edge = &mut self.edges[index];
                if edge.name.is_none() {
                    edge.name = source.name;
                }
                // A successfully welded closed edge IS shared real topology
                // (one cap circle referenced by both adjacent faces —
                // Golovanov §5.6): clear the historical degenerate marking
                // so the Euler count and the degenerate-use validation see
                // the real 1-cell.  Unwelded closed edges keep the marking
                // — the offset pipeline still builds rim pairs with
                // mismatched representations (a closed circle on one
                // side, an open fitted chain on the other), which
                // cannot weld.
                edge.degenerate = false;
                let id = edge.id;
                *self.edge_use_counts.entry(id).or_default() += 1;
                return Ok((id, reversed));
            }
        }
        let start_vertex_id = self.vertex(source.start);
        let end_vertex_id = self.vertex(source.end);
        let canonical_start = self.vertex_point(start_vertex_id)?;
        let canonical_end = self.vertex_point(end_vertex_id)?;
        // A degenerate source can be a true point placeholder (pole), a
        // CLOSED curve whose endpoints coincide (full circle/seam — real
        // geometry that must be kept), or a sub-tolerance sliver subrange of
        // a real curve (an intersection landing on a seam) whose interior
        // still sweeps measurably away from the vertex. Only the sliver is
        // rewritten: sampling confirms the whole trimmed span stays at the
        // vertex scale, then the canonical point-collapsed form (as analytic
        // pole edges use) replaces it so the curve-end-matches-vertex and
        // pcurve-consistency validations both hold.
        let collapses_to_vertex = source_degenerate
            && start_vertex_id == end_vertex_id
            && [0.25, 0.5, 0.75, 1.0].into_iter().all(|fraction| {
                let parameter = source.t0 + (source.t1 - source.t0) * fraction;
                source
                    .curve
                    .evaluate(parameter)
                    .is_ok_and(|point| point.sub(canonical_start).length() <= 10.0 * assembler_weld(self.tolerance))
            });
        // Compare the CURVE'S evaluated endpoints against the canonical
        // vertices, not the source's claimed endpoints: imprint stamps the
        // exact split positions into start/end, but the stored curve (a
        // fitted polyline, say) can evaluate measurably away from them at
        // t0/t1 — trusting the claim then skips the snap and validation
        // rejects the edge (offset-target subtract, gap 1e-4).
        let evaluated_start = source.curve.evaluate(source.t0).or_refuse(KernelStage::Sew, "evaluate")?;
        let evaluated_end = source.curve.evaluate(source.t1).or_refuse(KernelStage::Sew, "evaluate")?;
        let (curve, edge_t0, edge_t1) = if collapses_to_vertex {
            (
                crate::make_line(canonical_start, canonical_start).or_refuse(KernelStage::Sew, "make_line")?,
                0.0,
                1.0,
            )
        } else if canonical_start.sub(evaluated_start).length() > 1e-8
            || canonical_end.sub(evaluated_end).length() > 1e-8
        {
            let curve = snap_edge_curve_endpoints(
                source.curve,
                source.t0,
                source.t1,
                canonical_start,
                canonical_end,
            )?;
            // Splitting changes the represented curve to the requested
            // subrange. Keep the edge interval synchronized with that actual
            // curve domain; retaining the pre-split interval can evaluate an
            // interior point instead of the newly constrained endpoint.
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            (curve, t0, t1)
        } else {
            (source.curve, source.t0, source.t1)
        };
        let id = self.next_edge_id;
        if std::env::var("BREP_DEBUG_EDGE").is_ok_and(|v| v == id.to_string()) {
            let eval_start = curve.evaluate(edge_t0);
            let eval_end = curve.evaluate(edge_t1);
            eprintln!(
                "EDGE {id}: src start={:?} end={:?} sep={:.9} degen={} t=[{:.9},{:.9}]\n\
                 canon start={canonical_start:?} end={canonical_end:?}\n\
                 curve degree={} knots[0..4]={:?} knots[-4..]={:?} nctrl={}\n\
                 eval(t0)={eval_start:?}\n eval(t1)={eval_end:?}",
                source.start,
                source.end,
                source.start.sub(source.end).length(),
                source_degenerate,
                edge_t0,
                edge_t1,
                curve.degree,
                &curve.knots[..4.min(curve.knots.len())],
                &curve.knots[curve.knots.len().saturating_sub(4)..],
                curve.control_points.len(),
            );
        }
        self.next_edge_id += 1;
        self.edges.push(EdgeRecord {
            id,
            curve,
            t0: edge_t0,
            t1: edge_t1,
            start_vertex_id,
            end_vertex_id,
            // Historical marking: closed and sub-tolerance edges stay
            // face-local unless a closed weld later claims them (the weld
            // clears this flag on success — see above).
            degenerate: source_degenerate
                || start_vertex_id == end_vertex_id
                || canonical_start.sub(canonical_end).length() <= identity_tolerance,
            name: source.name,
        });
        self.edge_use_counts.insert(id, 1);
        // A fresh edge is created reversed=false, so its first coedge's result
        // `forward` equals the incoming `forward`. Record it so a later weld
        // can enforce the opposite-sense (manifold) pairing.
        self.edge_first_forward.insert(id, incoming_forward);
        if let Some(key) = source.boundary_key {
            self.edge_boundary_key.insert(id, key);
        }
        if source.constructed {
            self.imprint_edges.insert(id);
        }
        Ok((id, false))
    }
}

/// A weld candidate is refused when the incoming source and the existing edge
/// both descend from the SAME operand's boundary but from DIFFERENT original
/// edge ids: two distinct 1-cells of a valid input solid are never one shared
/// edge, however close their geometry runs. Anything else (cross-operand, or
/// an imprint/derived source on either side) is left to the geometric +
/// opposite-sense checks that follow.
pub(super) fn weld_refused_by_identity(
    incoming: Option<(u8, u64)>,
    existing: Option<(u8, u64)>,
) -> bool {
    match (incoming, existing) {
        (Some((op_a, id_a)), Some((op_b, id_b))) => op_a == op_b && id_a != id_b,
        _ => false,
    }
}

