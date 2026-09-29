use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

#[derive(Clone, Copy)]
pub(super) struct Bounds {
    minimum: Vec3,
    maximum: Vec3,
}

impl Bounds {
    pub(super) fn from_points(points: &[Vec3]) -> Option<Self> {
        let first = *points.first()?;
        let mut bounds = Self {
            minimum: first,
            maximum: first,
        };
        for point in &points[1..] {
            bounds.minimum.x = bounds.minimum.x.min(point.x);
            bounds.minimum.y = bounds.minimum.y.min(point.y);
            bounds.minimum.z = bounds.minimum.z.min(point.z);
            bounds.maximum.x = bounds.maximum.x.max(point.x);
            bounds.maximum.y = bounds.maximum.y.max(point.y);
            bounds.maximum.z = bounds.maximum.z.max(point.z);
        }
        Some(bounds)
    }

    pub(super) fn intersects(self, other: Self, tolerance: f64) -> bool {
        self.minimum.x <= other.maximum.x + tolerance
            && self.maximum.x + tolerance >= other.minimum.x
            && self.minimum.y <= other.maximum.y + tolerance
            && self.maximum.y + tolerance >= other.minimum.y
            && self.minimum.z <= other.maximum.z + tolerance
            && self.maximum.z + tolerance >= other.minimum.z
    }
}

pub(super) fn edge_sample_points(solid: &BrepSolid) -> Result<Vec<Vec3>, KernelRefusal> {
    let mut points = Vec::new();
    for edge in &solid.edges {
        if edge.degenerate {
            continue;
        }
        for sample in 0..=8 {
            let fraction = sample as f64 / 8.0;
            points.push(
                edge.curve
                    .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?,
            );
        }
    }
    Ok(points)
}

/// Points that conservatively enclose a carrier's reach: its boundary edges,
/// a grid over each face's surface interior, AND every face's CONTROL NET. A
/// trimmed offset carrier can bulge far past its boundary edges — an offset
/// sphere trimmed near its opening samples its edges only around that opening,
/// so an edges-only bound reports x/z extents from the trim rim, not from the
/// sphere's true radius. The imprint-pair bounds pre-filter would then wrongly
/// drop a pair whose surfaces genuinely cross (the offset sphere reaching an
/// offset wall plane it clears at the source distance). Sampled points alone
/// still UNDERESTIMATE between samples — a full offset sphere whose baked pole
/// axis tilts a few degrees off +z has its true z-max between grid rows, 0.03
/// above the pole sample, which silently dropped the exact grazing pair whose
/// lens imprint the shell needed (the d=0.5 narrow-band failure). The control
/// net closes that hole: with positive weights the surface lies inside the
/// convex hull of its Cartesian control points, so net points can only ever
/// ENLARGE the bound and never hide a real crossing; the source-separation
/// gate and the SSI itself still reject non-intersecting pairs downstream.
pub(super) fn carrier_extent_points(solid: &BrepSolid) -> Result<Vec<Vec3>, KernelRefusal> {
    let mut points = edge_sample_points(solid)?;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let surface = &face.surface;
        for row in &surface.control_points {
            for control in row {
                if let Ok(point) = control.point() {
                    points.push(point);
                }
            }
        }
        let (Ok(ku), Ok(kv)) = (
            crate::KnotVector::new(surface.knots_u.clone(), surface.degree_u),
            crate::KnotVector::new(surface.knots_v.clone(), surface.degree_v),
        ) else {
            continue;
        };
        let [u0, u1] = ku.domain();
        let [v0, v1] = kv.domain();
        for iu in 0..=8 {
            for iv in 0..=8 {
                let u = u0 + (u1 - u0) * iu as f64 / 8.0;
                let v = v0 + (v1 - v0) * iv as f64 / 8.0;
                if let Ok(point) = surface.evaluate(u, v) {
                    points.push(point);
                }
            }
        }
    }
    Ok(points)
}

pub(super) fn source_faces_adjacent(first: &FaceRecord, second: &FaceRecord) -> bool {
    let first_edges = first
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect::<HashSet<_>>();
    second
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .any(|coedge| first_edges.contains(&coedge.edge_id))
}

/// The face's outward normal — the shared pointwise offset evaluator's
/// [`crate::OffsetNormal::Face`] lane, which this function used to write out by
/// hand. Its two offsetting callers (`pipeline.rs`'s carrier residual probe and
/// `carrier_rebuild.rs`'s seed) go through `offset_at` instead.
pub(super) fn face_normal(face: &FaceRecord, u: f64, v: f64) -> Result<Vec3, KernelRefusal> {
    face_offsets(face).normal(u, v).or_refuse(KernelStage::Sew, "normal")
}

/// One face's pointwise offset evaluator.
///
/// Offset-shell is the surface-type-blind pipeline (audit §0.5): it offsets
/// every retained face the same way regardless of carrier kind, so it is the
/// natural consumer of the shared evaluator. `distance` here follows the
/// evaluator's convention — signed ALONG the outward normal — so offset-shell's
/// own "positive moves opposite" reading is negated by its callers, at a named
/// place each time.
pub(super) fn face_offsets(face: &FaceRecord) -> crate::OffsetEvaluator<'_> {
    crate::OffsetEvaluator::new(
        "offset_shell",
        &face.surface,
        crate::OffsetNormal::Face {
            same_sense: face.same_sense,
        },
    )
}

/// Cosine of the largest normal deviation two faces may show along a shared
/// edge and still count as SMOOTHLY joined there (≈5.7°): the band boundary
/// synchronization pairs carriers with, and the band below which the reflex
/// miter probe sees no crossing.
pub(super) const SMOOTH_JUNCTION_COS: f64 = 0.995;

/// The coedge of `carrier_face` that images source use `(loop_index,
/// use_index)` of `source_face`.
///
/// A carrier is trimmed with the parametric image of its source trim, loop
/// for loop and use for use, and every consumer of that mirror indexes the
/// carrier's loops by the source's indices. A carrier whose source trim was
/// REBUILT over the full rectangle of its domain
/// (`cut_rim_full_rectangle_source`) has four isoline uses instead, so the
/// source index means nothing there: the 2026-09-06 fillet-rail document
/// read `coedges[4]` of a four-use loop and panicked on master. The image of
/// a source use on a rebuilt trim is the rectangle side whose isoline the
/// source use's pcurve runs along; a use that runs along no side (a curved
/// trim boundary, a cut rim) has no image and `None` says so — the rebuild
/// lane itself declines a face whose retained junctions are not isolines, so
/// a smooth junction always has one.
pub(super) fn carrier_use_imaging(
    carrier_face: &FaceRecord,
    source_face: &FaceRecord,
    loop_index: usize,
    use_index: usize,
) -> Result<Option<usize>, KernelRefusal> {
    let source_loop = source_face
        .loops
        .get(loop_index)
        .ok_or_else(|| format!("offset_shell: source face {} has no loop {loop_index}", source_face.id)).or_refuse(KernelStage::Sew, "offset_shell_source_face_has_no_loop")?;
    let source_use = source_loop
        .coedges
        .get(use_index)
        .ok_or_else(|| format!("offset_shell: source face {} has no use {use_index}", source_face.id)).or_refuse(KernelStage::Sew, "offset_shell_source_face_has_no_use")?;
    let Some(carrier_loop) = carrier_face.loops.get(loop_index) else {
        return Ok(None);
    };
    if carrier_loop.coedges.len() == source_loop.coedges.len() {
        return Ok((use_index < carrier_loop.coedges.len()).then_some(use_index));
    }
    // A rebuilt trim: which isoline does the source use run along?
    let [u0, u1] = source_face.surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
    let [v0, v1] = source_face.surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
    let band_u = 1e-9 * (u1 - u0).abs().max(1.0);
    let band_v = 1e-9 * (v1 - v0).abs().max(1.0);
    let [q0, q1] = source_use.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let mut on = [true; 4]; // u = u0, u = u1, v = v0, v = v1
    for sample in 0..=8 {
        let uv = source_use.pcurve.evaluate(q0 + (q1 - q0) * sample as f64 / 8.0).or_refuse(KernelStage::Sew, "evaluate")?;
        on[0] &= (uv.x - u0).abs() <= band_u;
        on[1] &= (uv.x - u1).abs() <= band_u;
        on[2] &= (uv.y - v0).abs() <= band_v;
        on[3] &= (uv.y - v1).abs() <= band_v;
    }
    let Some(side) = on.iter().position(|hit| *hit) else {
        return Ok(None);
    };
    for (index, coedge) in carrier_loop.coedges.iter().enumerate() {
        let [c0, c1] = coedge.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
        let (a, b) = (coedge.pcurve.evaluate(c0).or_refuse(KernelStage::Sew, "evaluate")?, coedge.pcurve.evaluate(c1).or_refuse(KernelStage::Sew, "evaluate")?);
        let matches = match side {
            0 => (a.x - u0).abs() <= band_u && (b.x - u0).abs() <= band_u,
            1 => (a.x - u1).abs() <= band_u && (b.x - u1).abs() <= band_u,
            2 => (a.y - v0).abs() <= band_v && (b.y - v0).abs() <= band_v,
            _ => (a.y - v1).abs() <= band_v && (b.y - v1).abs() <= band_v,
        };
        if matches {
            return Ok(Some(index));
        }
    }
    Ok(None)
}

pub(super) fn uses_are_tangent(
    first: &FaceRecord,
    first_use: &CoedgeRecord,
    second: &FaceRecord,
    second_use: &CoedgeRecord,
) -> Result<bool, KernelRefusal> {
    let [first_start, first_end] = first_use.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let [second_start, second_end] = second_use.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    for fraction in [0.2, 0.5, 0.8] {
        let first_uv = first_use
            .pcurve
            .evaluate(first_start + (first_end - first_start) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
        let second_fraction = if first_use.forward == second_use.forward {
            fraction
        } else {
            1.0 - fraction
        };
        let second_uv = second_use
            .pcurve
            .evaluate(second_start + (second_end - second_start) * second_fraction).or_refuse(KernelStage::Sew, "evaluate")?;
        if face_normal(first, first_uv.x, first_uv.y)?.dot(face_normal(
            second,
            second_uv.x,
            second_uv.y,
        )?) < SMOOTH_JUNCTION_COS
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn vertex_point(solid: &BrepSolid, vertex_id: u64) -> Result<Vec3, KernelRefusal> {
    solid
        .vertices
        .iter()
        .find(|vertex| vertex.id == vertex_id)
        .map(|vertex| vertex.point)
        .ok_or_else(|| format!("offset_shell: missing carrier vertex {vertex_id}"))
        .or_refuse(KernelStage::Sew, "offset_shell_missing_carrier_vertex")
}

pub(super) struct SmoothSynchronization {
    pub(super) pairs: HashSet<(usize, usize)>,
    pub(super) edge_pairs: Vec<(u8, u64, u8, u64)>,
}

pub(super) fn synchronize_smooth_offset_boundaries(
    carriers: &mut [Carrier],
    source_faces: &[&FaceRecord],
    scale: f64,
) -> Result<SmoothSynchronization, KernelRefusal> {
    let source_by_id = source_faces
        .iter()
        .map(|face| (face.id, *face))
        .collect::<HashMap<_, _>>();
    let mut smooth_pairs = HashSet::default();
    let mut edge_pairs = Vec::new();
    for first in 0..carriers.len() {
        if !matches!(carriers[first].kind, OffsetFaceRole::Offset) {
            continue;
        }
        for second in first + 1..carriers.len() {
            if !matches!(carriers[second].kind, OffsetFaceRole::Offset) {
                continue;
            }
            // Two carriers can share ONE source face (an apex-cap sphere
            // rides its cone's face). A self-pair has nothing to smooth-sync,
            // and the loop below indexes carriers by the SOURCE loop
            // structure, which a cap does not mirror.
            if carriers[first].source_face_id == carriers[second].source_face_id {
                continue;
            }
            let first_source = source_by_id[&carriers[first].source_face_id];
            let second_source = source_by_id[&carriers[second].source_face_id];
            let mut replacements = Vec::new();
            for (first_loop_index, first_loop) in first_source.loops.iter().enumerate() {
                for (first_use_index, first_use) in first_loop.coedges.iter().enumerate() {
                    let Some((second_loop_index, second_use_index, second_use)) =
                        second_source.loops.iter().enumerate().find_map(
                            |(loop_index, loop_record)| {
                                loop_record
                                    .coedges
                                    .iter()
                                    .enumerate()
                                    .find(|(_, coedge)| coedge.edge_id == first_use.edge_id)
                                    .map(|(use_index, coedge)| (loop_index, use_index, coedge))
                            },
                        )
                    else {
                        continue;
                    };
                    // Why a shared source edge did NOT become a smooth pair. A
                    // pair that bails here goes to the imprint instead, where
                    // two TANGENT offset carriers meet in no curve at all and
                    // the rim it should have welded is left one-use.
                    os_debug!(
                        "smooth pair ({first},{second}) src=({},{}) edge {}",
                        first_source.id,
                        second_source.id,
                        first_use.edge_id
                    );
                    if !uses_are_tangent(first_source, first_use, second_source, second_use)? {
                        os_debug!("  bail: uses are not tangent");
                        continue;
                    }
                    // The carrier's use that IMAGES the source use: the same
                    // index while the carrier mirrors the source loop 1:1,
                    // and the isoline side that carries the source use when
                    // the trim was rebuilt over its full rectangle
                    // (`cut_rim_full_rectangle_source`). A rebuilt trim with
                    // no image of this junction bails by name.
                    let Some(first_carrier_index) = carrier_use_imaging(
                        &carriers[first].solid.shells[0].faces[0],
                        first_source,
                        first_loop_index,
                        first_use_index,
                    )?
                    else {
                        os_debug!("  bail: carrier {first}'s rebuilt trim keeps no image of edge {}", first_use.edge_id);
                        continue;
                    };
                    let Some(second_carrier_index) = carrier_use_imaging(
                        &carriers[second].solid.shells[0].faces[0],
                        second_source,
                        second_loop_index,
                        second_use_index,
                    )?
                    else {
                        os_debug!("  bail: carrier {second}'s rebuilt trim keeps no image of edge {}", first_use.edge_id);
                        continue;
                    };
                    let first_carrier_use = &carriers[first].solid.shells[0].faces[0].loops
                        [first_loop_index]
                        .coedges[first_carrier_index];
                    let second_carrier_use = &carriers[second].solid.shells[0].faces[0].loops
                        [second_loop_index]
                        .coedges[second_carrier_index];
                    let first_edge = carriers[first]
                        .solid
                        .edges
                        .iter()
                        .find(|edge| edge.id == first_carrier_use.edge_id)
                        .cloned()
                        .ok_or_else(|| "offset_shell: missing smooth carrier edge".to_string()).or_refuse(KernelStage::Sew, "offset_shell_missing_smooth_carrier_edge")?;
                    let second_edge = carriers[second]
                        .solid
                        .edges
                        .iter()
                        .find(|edge| edge.id == second_carrier_use.edge_id)
                        .cloned()
                        .ok_or_else(|| "offset_shell: missing smooth carrier edge".to_string()).or_refuse(KernelStage::Sew, "offset_shell_missing_smooth_carrier_edge")?;
                    let first_start =
                        vertex_point(&carriers[first].solid, first_edge.start_vertex_id)?;
                    let first_end = vertex_point(&carriers[first].solid, first_edge.end_vertex_id)?;
                    let second_start =
                        vertex_point(&carriers[second].solid, second_edge.start_vertex_id)?;
                    let second_end =
                        vertex_point(&carriers[second].solid, second_edge.end_vertex_id)?;
                    let endpoint_tolerance = (scale * 2e-4).max(1e-5);
                    let endpoints_coincident = (first_start.sub(second_start).length()
                        <= endpoint_tolerance
                        && first_end.sub(second_end).length() <= endpoint_tolerance)
                        || (first_start.sub(second_end).length() <= endpoint_tolerance
                            && first_end.sub(second_start).length() <= endpoint_tolerance);
                    if !endpoints_coincident {
                        os_debug!(
                            "  bail: carrier endpoints apart by {:.6} / {:.6}",
                            first_start.sub(second_start).length(),
                            first_end.sub(second_end).length()
                        );
                        continue;
                    }
                    let [pcurve_start, pcurve_end] = second_carrier_use.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
                    let mut forward_gap = 0.0f64;
                    let mut reverse_gap = 0.0f64;
                    for sample in 0..=32 {
                        let fraction = sample as f64 / 32.0;
                        let uv = second_carrier_use
                            .pcurve
                            .evaluate(pcurve_start + (pcurve_end - pcurve_start) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
                        let on_surface = carriers[second].solid.shells[0].faces[0]
                            .surface
                            .evaluate(uv.x, uv.y).or_refuse(KernelStage::Sew, "evaluate")?;
                        let forward_point = first_edge
                            .curve
                            .evaluate(first_edge.t0 + (first_edge.t1 - first_edge.t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
                        let reverse_point = first_edge
                            .curve
                            .evaluate(first_edge.t1 - (first_edge.t1 - first_edge.t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
                        forward_gap = forward_gap.max(on_surface.sub(forward_point).length());
                        reverse_gap = reverse_gap.max(on_surface.sub(reverse_point).length());
                    }
                    if forward_gap.min(reverse_gap) > 2e-3 {
                        os_debug!(
                            "  bail: carrier edges {:.6} apart",
                            forward_gap.min(reverse_gap)
                        );
                        continue;
                    }
                    replacements.push((
                        second_loop_index,
                        second_carrier_index,
                        second_edge.id,
                        first_edge.id,
                        first_edge,
                        if forward_gap <= reverse_gap {
                            true
                        } else {
                            false
                        },
                    ));
                }
            }
            if replacements.is_empty() {
                continue;
            }
            for (loop_index, use_index, edge_id, first_edge_id, replacement, forward) in
                replacements
            {
                let start = vertex_point(&carriers[first].solid, replacement.start_vertex_id)?;
                let end = vertex_point(&carriers[first].solid, replacement.end_vertex_id)?;
                let start_vertex_id = carriers[second]
                    .solid
                    .vertices
                    .iter()
                    .find(|vertex| vertex.point.sub(start).length() <= 1e-5)
                    .map(|vertex| vertex.id)
                    .unwrap_or_else(|| {
                        let id = carriers[second]
                            .solid
                            .vertices
                            .iter()
                            .map(|vertex| vertex.id)
                            .max()
                            .unwrap_or(0)
                            + 1;
                        carriers[second]
                            .solid
                            .vertices
                            .push(crate::topology::VertexRecord { id, point: start });
                        id
                    });
                let end_vertex_id = carriers[second]
                    .solid
                    .vertices
                    .iter()
                    .find(|vertex| {
                        (start.sub(end).length() <= 1e-7 || vertex.id != start_vertex_id)
                            && vertex.point.sub(end).length() <= 1e-5
                    })
                    .map(|vertex| vertex.id)
                    .unwrap_or_else(|| {
                        let id = carriers[second]
                            .solid
                            .vertices
                            .iter()
                            .map(|vertex| vertex.id)
                            .max()
                            .unwrap_or(0)
                            + 1;
                        carriers[second]
                            .solid
                            .vertices
                            .push(crate::topology::VertexRecord { id, point: end });
                        id
                    });
                let edge = carriers[second]
                    .solid
                    .edges
                    .iter_mut()
                    .find(|edge| edge.id == edge_id)
                    .unwrap();
                edge.curve = replacement.curve;
                edge.t0 = replacement.t0;
                edge.t1 = replacement.t1;
                edge.start_vertex_id = start_vertex_id;
                edge.end_vertex_id = end_vertex_id;
                carriers[second].solid.shells[0].faces[0].loops[loop_index].coedges[use_index]
                    .forward = forward;
                edge_pairs.push((first as u8, first_edge_id, second as u8, edge_id));
            }
            smooth_pairs.insert((first, second));
        }
    }
    Ok(SmoothSynchronization {
        pairs: smooth_pairs,
        edge_pairs,
    })
}

pub(super) fn synchronize_smooth_edge_splits(
    imprint: &mut ImprintResultRecord,
    edge_pairs: &[(u8, u64, u8, u64)],
) {
    for &(first_operand, first_edge, second_operand, second_edge) in edge_pairs {
        let mut parameters = imprint
            .edge_splits
            .iter()
            .filter(|split| {
                (split.operand == first_operand && split.edge_id == first_edge)
                    || (split.operand == second_operand && split.edge_id == second_edge)
            })
            .flat_map(|split| split.parameters.iter().copied())
            .collect::<Vec<_>>();
        parameters.sort_by(f64::total_cmp);
        parameters.dedup_by(|first, second| (*first - *second).abs() <= 1e-8);
        if parameters.is_empty() {
            continue;
        }
        for (operand, edge_id) in [(first_operand, first_edge), (second_operand, second_edge)] {
            if let Some(split) = imprint
                .edge_splits
                .iter_mut()
                .find(|split| split.operand == operand && split.edge_id == edge_id)
            {
                split.parameters = parameters.clone();
            } else {
                imprint.edge_splits.push(EdgeSplitRecord {
                    operand,
                    edge_id,
                    parameters: parameters.clone(),
                });
            }
        }
    }
}
