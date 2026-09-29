use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

// ---------------------------------------------------------------------------
// The rebuild PLAN `offset_ruled_face`'s classification step produces and its
// apply step executes.
//
// Until this plan existed the apply step patched EXISTING ids in place: a new
// curve or trim range per edge, a new point per vertex. A push whose rebuild
// needs a vertex, an edge, a coedge or a loop the source solid does not have
// could not be expressed at all, so it refused at the corner guard even when
// the answer was known. The gear hex-bore push is the first such case: its
// mouth meets the pocket floor at TANGENT vertices, and the rebuilt rim has to
// leave each one behind on its hex flat and connect back to it.
//
// So the plan carries two parts. The GEOMETRY part is exactly what the apply
// step used to take, applied in exactly the order it used to apply it, which
// keeps every lane that adds no topology bit-identical. The TOPOLOGY part names
// what it adds by position in its own lists (`PlanVertex::Added(i)`,
// `PlanEdge::Added(i)`); ids are allocated only when the plan is applied, past
// every id the solid already uses.
// ---------------------------------------------------------------------------

/// A vertex the plan refers to: one the solid has, or the plan's `i`-th
/// addition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PlanVertex {
    Existing(u64),
    Added(usize),
}

/// An edge the plan refers to: one the solid has, or the plan's `i`-th
/// addition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PlanEdge {
    Existing(u64),
    Added(usize),
}

/// Which end of an existing edge a re-attachment moves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum EdgeEnd {
    Start,
    End,
}

/// An edge the plan adds. Its range is its curve's whole domain.
pub(super) struct PlannedEdge {
    pub(super) start: PlanVertex,
    pub(super) end: PlanVertex,
    pub(super) curve: NurbsCurve,
}

/// One use of an edge in a planned loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct PlannedCoedge {
    pub(super) edge: PlanEdge,
    pub(super) forward: bool,
}

/// A face whose loops the plan rewrites whole.
///
/// Every coedge of the rewritten loops that the face (or a face the plan
/// removes) already carried for the same edge in the same direction keeps its
/// record, id and pcurve; a new one is given a fresh id and a pcurve fitted on
/// the face's current carrier. Either way the caller re-trims the face after
/// the geometry has settled — a planned face is always one the rebuild
/// re-trims, because its boundary moved.
pub(super) struct PlannedFace {
    pub(super) face_id: u64,
    /// The face's material side flips (a floor that becomes a ceiling).
    pub(super) flip_sense: bool,
    pub(super) loops: Vec<Vec<PlannedCoedge>>,
}

/// What the classification step decided, before anything is written.
#[derive(Default)]
pub(super) struct RebuildPlan {
    /// Existing edges given a new curve; the range is the curve's domain.
    pub(super) curves: HashMap<u64, NurbsCurve>,
    /// Existing edges kept on their curve with a new trim range.
    pub(super) ranges: HashMap<u64, (f64, f64)>,
    /// Existing vertices given a new point.
    pub(super) moved: HashMap<u64, Vec3>,
    pub(super) added_vertices: Vec<Vec3>,
    pub(super) added_edges: Vec<PlannedEdge>,
    /// An existing edge's end moved onto another vertex.
    pub(super) reattached: Vec<(u64, EdgeEnd, PlanVertex)>,
    pub(super) faces: Vec<PlannedFace>,
    pub(super) removed_faces: Vec<u64>,
}

/// The ids the apply step allocated, indexed like the plan's additions.
#[derive(Debug, Default)]
pub(super) struct AppliedTopology {
    pub(super) vertex_ids: Vec<u64>,
    pub(super) edge_ids: Vec<u64>,
}

impl AppliedTopology {
    pub(super) fn vertex(&self, vertex: PlanVertex) -> u64 {
        match vertex {
            PlanVertex::Existing(id) => id,
            PlanVertex::Added(index) => self.vertex_ids[index],
        }
    }

    pub(super) fn edge(&self, edge: PlanEdge) -> u64 {
        match edge {
            PlanEdge::Existing(id) => id,
            PlanEdge::Added(index) => self.edge_ids[index],
        }
    }
}

impl RebuildPlan {
    /// TRUE when applying the plan changes the solid's topology, not only its
    /// geometry.
    pub(super) fn adds_topology(&self) -> bool {
        !(self.added_vertices.is_empty()
            && self.added_edges.is_empty()
            && self.reattached.is_empty()
            && self.faces.is_empty()
            && self.removed_faces.is_empty())
    }

    /// Every edge the rebuild touched, by curve, by trim range, or by being
    /// added — the selective pcurve re-fit's work list.
    pub(super) fn changed_edges(&self, applied: &AppliedTopology) -> HashSet<u64> {
        self.curves
            .keys()
            .chain(self.ranges.keys())
            .copied()
            .chain(applied.edge_ids.iter().copied())
            .collect()
    }

    /// Apply to a fresh clone; the input is never mutated.
    pub(super) fn apply(&self, solid: &BrepSolid) -> Result<(BrepSolid, AppliedTopology), KernelRefusal> {
        let mut result = solid.clone();
        for edge in &mut result.edges {
            if let Some(curve) = self.curves.get(&edge.id) {
                // A trimmed rim arc carries its PARENT conic's parameter range
                // (`NurbsCurve::split` preserves knot values), so the edge range
                // comes from the curve. Every whole-curve rebuild — the closed rims,
                // the seam lines — still lands on [0, 1] exactly as before.
                let [d0, d1] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                edge.curve = curve.clone();
                edge.t0 = d0;
                edge.t1 = d1;
            } else if let Some((t0, t1)) = self.ranges.get(&edge.id) {
                // A curved neighbour's own edge: same curve, new trim.
                edge.t0 = *t0;
                edge.t1 = *t1;
            }
        }
        for vertex in &mut result.vertices {
            if let Some(point) = self.moved.get(&vertex.id) {
                vertex.point = *point;
            }
        }
        let mut applied = AppliedTopology::default();
        if !self.adds_topology() {
            return Ok((result, applied));
        }

        let mut next_vertex = result.vertices.iter().map(|v| v.id).max().map_or(1, |id| id + 1);
        for point in &self.added_vertices {
            result.vertices.push(VertexRecord {
                id: next_vertex,
                point: *point,
            });
            applied.vertex_ids.push(next_vertex);
            next_vertex += 1;
        }
        let vertex_exists = |solid: &BrepSolid, id: u64| solid.vertices.iter().any(|v| v.id == id);
        let mut next_edge = result.edges.iter().map(|e| e.id).max().map_or(1, |id| id + 1);
        for planned in &self.added_edges {
            let [d0, d1] = planned.curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
            let (start, end) = (applied.vertex(planned.start), applied.vertex(planned.end));
            for id in [start, end] {
                if !vertex_exists(&result, id) {
                    return Err(KernelRefusal::internal(
                        KernelStage::Fragment,
                        "plan_edge_vertex",
                        format!("offset_ruled_face: the plan adds an edge at missing vertex {id}"),
                    ));
                }
            }
            result.edges.push(EdgeRecord {
                id: next_edge,
                curve: planned.curve.clone(),
                t0: d0,
                t1: d1,
                start_vertex_id: start,
                end_vertex_id: end,
                degenerate: false,
                name: None,
            });
            applied.edge_ids.push(next_edge);
            next_edge += 1;
        }
        for (edge_id, end, vertex) in &self.reattached {
            let id = applied.vertex(*vertex);
            if !vertex_exists(&result, id) {
                return Err(KernelRefusal::internal(
                    KernelStage::Fragment,
                    "plan_reattach_vertex",
                    format!("offset_ruled_face: the plan re-attaches edge {edge_id} to missing vertex {id}"),
                ));
            }
            let edge = result
                .edges
                .iter_mut()
                .find(|edge| edge.id == *edge_id)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Fragment,
                        "plan_reattach_edge",
                        format!("offset_ruled_face: the plan re-attaches missing edge {edge_id}"),
                    )
                })?;
            match end {
                EdgeEnd::Start => edge.start_vertex_id = id,
                EdgeEnd::End => edge.end_vertex_id = id,
            }
        }

        if !self.faces.is_empty() {
            // The coedges a rewritten loop may re-use: its own face's, then those
            // of every face the plan removes (a merge gathers them into one).
            let mut next_coedge = 1u64;
            let mut next_loop = 1u64;
            for face in result.shells.iter().flat_map(|shell| &shell.faces) {
                for loop_record in &face.loops {
                    next_loop = next_loop.max(loop_record.id + 1);
                    for coedge in &loop_record.coedges {
                        next_coedge = next_coedge.max(coedge.id + 1);
                    }
                }
            }
            let removed_pool: Vec<CoedgeRecord> = result
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .filter(|face| self.removed_faces.contains(&face.id))
                .flat_map(|face| &face.loops)
                .flat_map(|loop_record| loop_record.coedges.iter().cloned())
                .collect();
            let edges: HashMap<u64, EdgeRecord> =
                result.edges.iter().map(|edge| (edge.id, edge.clone())).collect();
            let mut taken: HashSet<u64> = HashSet::default();
            for planned in &self.faces {
                let (shell, position) = find_face(&result, planned.face_id).ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Fragment,
                        "plan_face",
                        format!("offset_ruled_face: the plan rewrites missing face {}", planned.face_id),
                    )
                })?;
                let face = &result.shells[shell].faces[position];
                let own: Vec<CoedgeRecord> = face
                    .loops
                    .iter()
                    .flat_map(|loop_record| loop_record.coedges.iter().cloned())
                    .collect();
                let old_loop_ids: Vec<u64> = face.loops.iter().map(|l| l.id).collect();
                let surface = face.surface.clone();
                let mut loops: Vec<LoopRecord> = Vec::with_capacity(planned.loops.len());
                for (index, planned_loop) in planned.loops.iter().enumerate() {
                    let mut coedges: Vec<CoedgeRecord> = Vec::with_capacity(planned_loop.len());
                    for use_ in planned_loop {
                        let edge_id = applied.edge(use_.edge);
                        let reused = own
                            .iter()
                            .chain(removed_pool.iter())
                            .find(|coedge| {
                                coedge.edge_id == edge_id
                                    && coedge.forward == use_.forward
                                    && !taken.contains(&coedge.id)
                            })
                            .cloned();
                        let coedge = match reused {
                            Some(coedge) => coedge,
                            None => {
                                let edge = edges.get(&edge_id).ok_or_else(|| {
                                    KernelRefusal::internal(
                                        KernelStage::Fragment,
                                        "plan_loop_edge",
                                        format!("offset_ruled_face: the plan's loop uses missing edge {edge_id}"),
                                    )
                                })?;
                                let mut pcurve = build_pcurve_on_surface(&surface, &edge.curve).or_refuse(KernelStage::Fragment, "build_pcurve_on_surface")?;
                                if !use_.forward {
                                    pcurve = pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")?;
                                }
                                let coedge = CoedgeRecord {
                                    id: next_coedge,
                                    edge_id,
                                    forward: use_.forward,
                                    pcurve,
                                };
                                next_coedge += 1;
                                coedge
                            }
                        };
                        taken.insert(coedge.id);
                        coedges.push(coedge);
                    }
                    let id = old_loop_ids.get(index).copied().unwrap_or_else(|| {
                        let id = next_loop;
                        next_loop += 1;
                        id
                    });
                    loops.push(LoopRecord { id, coedges });
                }
                let face = &mut result.shells[shell].faces[position];
                face.loops = loops;
                if planned.flip_sense {
                    face.same_sense = !face.same_sense;
                }
            }
        }
        if !self.removed_faces.is_empty() {
            for face_id in &self.removed_faces {
                if self.faces.iter().any(|planned| planned.face_id == *face_id) {
                    return Err(KernelRefusal::internal(
                        KernelStage::Fragment,
                        "plan_rewrite_and_remove",
                        format!("offset_ruled_face: the plan both rewrites and removes face {face_id}"),
                    ));
                }
                if find_face(&result, *face_id).is_none() {
                    return Err(KernelRefusal::internal(
                        KernelStage::Fragment,
                        "plan_remove_face",
                        format!("offset_ruled_face: the plan removes missing face {face_id}"),
                    ));
                }
            }
            for shell in &mut result.shells {
                shell.faces.retain(|face| !self.removed_faces.contains(&face.id));
            }
        }
        Ok((result, applied))
    }
}

