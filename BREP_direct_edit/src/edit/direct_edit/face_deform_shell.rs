//! Deform several faces of a solid together (Golovanov 2020 §6.13): faces whose
//! shared edges are whole boundary sides of both carriers move as one shell.
//! Each shared edge moves with them — its two carriers stay coincident along it
//! (G0) and, where the edge was smooth, tangent (G1) — and is rebuilt as the
//! common deformed boundary curve; every other edge keeps its points and tangent
//! planes by [`deform_face`]'s locality rule, so the faces not selected, every
//! vertex and every pcurve are unchanged.
//!
//! v1 limits, refused by name: every selected face must be a PATCH face (one
//! loop whose edges each run along a whole side of its carrier); a selected
//! face's shared side must be one edge; a pick must lie inside a face (not on
//! a shared edge).
//!
//! [`deform_face`]: super::face_deform::deform_face

use super::face_deform::{deform_refusal, internal, span_boxes, support_may_move};
use crate::topology::FaceRecord;
use crate::{
    deform_surfaces_tied, parameter_point_in_face, project_point_to_surface, solid_self_intersections, BrepSolid,
    DeformPick, DeformSide, DeformSpec, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, PoleSupport,
    PolygonClass, SelfIntersectionOptions, SurfaceTie, TieContinuity, TiedSurface, Vec2, Vec3,
};

const OP: &str = "deformShell";

/// One pick: on face `face_id`, the surface point nearest `anchor` (inside the
/// face) moves to `target`; with `normal`, the normal there is made parallel.
#[derive(Clone, Copy, Debug)]
pub struct ShellDeformPick {
    pub face_id: u64,
    pub anchor: Vec3,
    pub target: Vec3,
    pub normal: Option<Vec3>,
}

#[derive(Clone, Debug)]
pub struct ShellDeformSpec {
    pub picks: Vec<ShellDeformPick>,
    /// Each knot span of each carrier is split into this many first.
    pub resolution: usize,
    pub min_degree: usize,
}

impl Default for ShellDeformSpec {
    fn default() -> Self {
        Self { picks: Vec::new(), resolution: 4, min_degree: 3 }
    }
}

/// What the deformation did.
#[derive(Clone, Debug)]
pub struct ShellDeformReport {
    /// Each shared edge (edge id) and what was kept across it.
    pub shared_edges: Vec<(u64, TieContinuity)>,
    /// Corner blocks held where two shared edges of one face meet.
    pub held_corners: usize,
    pub max_pick_error: f64,
}

fn side_of(pcurve: &NurbsCurve, domain: ([f64; 2], [f64; 2])) -> Option<DeformSide> {
    let ([u0, u1], [v0, v1]) = domain;
    // Each axis is read against its OWN span: a patch reparameterised to
    // `u in [0, 1]`, `v in [0, 1e12]` would otherwise test `u` against the
    // v span's tolerance and call every side `UMin`.
    let (tolerance_u, tolerance_v) = (1e-9 * (u1 - u0).abs(), 1e-9 * (v1 - v0).abs());
    let points: Vec<(f64, f64)> = pcurve.control_points.iter().map(|p| (p.x / p.w, p.y / p.w)).collect();
    let all = |f: &dyn Fn(&(f64, f64)) -> bool| points.iter().all(f);
    if all(&|p| (p.0 - u0).abs() <= tolerance_u) {
        Some(DeformSide::UMin)
    } else if all(&|p| (p.0 - u1).abs() <= tolerance_u) {
        Some(DeformSide::UMax)
    } else if all(&|p| (p.1 - v0).abs() <= tolerance_v) {
        Some(DeformSide::VMin)
    } else if all(&|p| (p.1 - v1).abs() <= tolerance_v) {
        Some(DeformSide::VMax)
    } else {
        None
    }
}

/// The 3D point at the start (along-parameter minimum) or end of `side`.
fn side_end(surface: &NurbsSurface, side: DeformSide, end: bool) -> Result<Vec3, KernelRefusal> {
    let [u0, u1] = surface.domain_u().map_err(|e| internal(&e))?;
    let [v0, v1] = surface.domain_v().map_err(|e| internal(&e))?;
    let (u, v) = match side {
        DeformSide::UMin => (u0, if end { v1 } else { v0 }),
        DeformSide::UMax => (u1, if end { v1 } else { v0 }),
        DeformSide::VMin => (if end { u1 } else { u0 }, v0),
        DeformSide::VMax => (if end { u1 } else { u0 }, v1),
    };
    surface.evaluate(u, v).map_err(|e| internal(&e))
}

/// The boundary curve of `side` (exact: the iso-curve there is the side's
/// pole row), running along the side's parameter.
fn side_curve(surface: &NurbsSurface, side: DeformSide) -> Result<NurbsCurve, KernelRefusal> {
    let [u0, u1] = surface.domain_u().map_err(|e| internal(&e))?;
    let [v0, v1] = surface.domain_v().map_err(|e| internal(&e))?;
    match side {
        DeformSide::UMin => surface.iso_curve_u(u0),
        DeformSide::UMax => surface.iso_curve_u(u1),
        DeformSide::VMin => surface.iso_curve_v(v0),
        DeformSide::VMax => surface.iso_curve_v(v1),
    }
    .map_err(|e| internal(&e))
}

/// Deform faces `face_ids` of `solid` together as `spec` asks.
pub fn deform_shell(
    solid: &BrepSolid,
    face_ids: &[u64],
    spec: &ShellDeformSpec,
) -> Result<(BrepSolid, ShellDeformReport), KernelRefusal> {
    let all: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    let mut faces: Vec<&FaceRecord> = Vec::with_capacity(face_ids.len());
    for id in face_ids {
        let face = all
            .iter()
            .find(|face| face.id == *id)
            .ok_or_else(|| KernelRefusal::input(KernelStage::Collect, "face_not_found", format!("{OP}: no face {id}")))?;
        if faces.iter().any(|f| f.id == *id) {
            return Err(KernelRefusal::input(KernelStage::Collect, "duplicate_face", format!("{OP}: face {id} given twice")));
        }
        faces.push(face);
    }
    if faces.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "no_faces", format!("{OP}: no faces")));
    }

    // Every face a patch: each coedge along one whole side; record edge -> (face, side).
    let mut sides: Vec<Vec<(u64, DeformSide)>> = Vec::with_capacity(faces.len());
    let mut tolerances = Vec::with_capacity(faces.len());
    for face in &faces {
        let domain = (face.surface.domain_u().map_err(|e| internal(&e))?, face.surface.domain_v().map_err(|e| internal(&e))?);
        // The inside tests take one uv tolerance: off the SMALLER span (see
        // `side_of` for the per-axis side reading).
        let span = (domain.0[1] - domain.0[0]).min(domain.1[1] - domain.1[0]);
        let tolerance = 1e-9 * span;
        tolerances.push(tolerance);
        if face.loops.len() != 1 {
            return Err(not_a_patch(face.id, "more than one loop"));
        }
        let mut list = Vec::new();
        for coedge in &face.loops[0].coedges {
            let side = side_of(&coedge.pcurve, domain)
                .ok_or_else(|| not_a_patch(face.id, &format!("edge {} does not run along a side", coedge.edge_id)))?;
            list.push((coedge.edge_id, side));
        }
        sides.push(list);
    }

    // Shared edges: in two selected faces, each the whole side of both.
    let mut ties: Vec<SurfaceTie> = Vec::new();
    let mut tie_edges: Vec<u64> = Vec::new();
    for a in 0..faces.len() {
        for &(edge, side_a) in &sides[a] {
            for b in a + 1..faces.len() {
                let Some(&(_, side_b)) = sides[b].iter().find(|(e, _)| *e == edge) else { continue };
                for (f, side) in [(a, side_a), (b, side_b)] {
                    if sides[f].iter().filter(|(_, s)| *s == side).count() != 1 {
                        return Err(KernelRefusal::unsupported(KernelStage::Collect, "shared_edge_not_full_side", format!(
                            "{OP}: face {}'s side {side:?} carries more than the shared edge {edge}",
                            faces[f].id
                        )));
                    }
                }
                let start_a = side_end(&faces[a].surface, side_a, false)?;
                let (start_b, end_b) = (side_end(&faces[b].surface, side_b, false)?, side_end(&faces[b].surface, side_b, true)?);
                let reversed = start_a.sub(end_b).length() < start_a.sub(start_b).length();
                ties.push(SurfaceTie { a, side_a, b, side_b, reversed });
                tie_edges.push(edge);
            }
        }
    }

    // Each face's locality: the trims it does NOT share with another selected face.
    let mut trim_boxes: Vec<Vec<[f64; 4]>> = Vec::with_capacity(faces.len());
    for face in &faces {
        let boxes = face.loops[0]
            .coedges
            .iter()
            .filter(|coedge| !tie_edges.contains(&coedge.edge_id))
            .flat_map(|coedge| span_boxes(&coedge.pcurve))
            .collect();
        trim_boxes.push(boxes);
    }

    // The picks, per face.
    let scale = crate::solid_model_scale(solid).max(f64::MIN_POSITIVE);
    let mut picks: Vec<Vec<DeformPick>> = vec![Vec::new(); faces.len()];
    let mut pick_slots = Vec::with_capacity(spec.picks.len());
    for (index, pick) in spec.picks.iter().enumerate() {
        let f = faces.iter().position(|face| face.id == pick.face_id).ok_or_else(|| {
            KernelRefusal::input(KernelStage::Collect, "pick_face_not_selected", format!(
                "{OP}: pick {index} names face {}, which is not one of the deformed faces",
                pick.face_id
            ))
        })?;
        let face = faces[f];
        let foot = project_point_to_surface(&face.surface, pick.anchor).map_err(|e| internal(&e))?;
        let inside = foot.distance <= 1e-6 * scale
            && matches!(parameter_point_in_face(face, Vec2 { x: foot.u, y: foot.v }, tolerances[f]), Ok(PolygonClass::Inside));
        if !inside {
            return Err(KernelRefusal::input(KernelStage::Collect, "anchor_off_face", format!(
                "{OP}: pick {index}'s anchor {:?} is not inside face {}",
                pick.anchor, face.id
            )));
        }
        let moved = pick.target.sub(pick.anchor).length();
        if !(moved <= scale) {
            return Err(KernelRefusal::input(KernelStage::Collect, "displacement_too_large", format!(
                "{OP}: pick {index} moves {moved:e}, more than the body's size {scale:e}"
            )));
        }
        pick_slots.push((f, picks[f].len()));
        picks[f].push(DeformPick { u: foot.u, v: foot.v, target: pick.target, normal: pick.normal });
    }

    let movers: Vec<Box<dyn Fn(PoleSupport) -> bool + '_>> = (0..faces.len())
        .map(|f| {
            let (face, boxes, tolerance) = (faces[f], &trim_boxes[f], tolerances[f]);
            Box::new(move |support: PoleSupport| support_may_move(face, boxes, tolerance, support)) as Box<dyn Fn(PoleSupport) -> bool>
        })
        .collect();
    let surfaces: Vec<TiedSurface> = (0..faces.len())
        .map(|f| TiedSurface {
            surface: &faces[f].surface,
            spec: DeformSpec {
                picks: picks[f].clone(),
                min_degree: spec.min_degree,
                split_spans: [spec.resolution.max(1), spec.resolution.max(1)],
                ..DeformSpec::default()
            },
            may_move: movers[f].as_ref(),
        })
        .collect();
    let tied = deform_surfaces_tied(&surfaces, &ties).map_err(|refusal| {
        let named = deform_refusal(faces[0].id, refusal);
        named.with_message(|m| m.replacen("deformFace", OP, 1))
    })?;

    // The new body: deformed carriers, rebuilt shared edges; nothing else changes.
    let mut result = solid.clone();
    result.mass_properties_cache = Default::default();
    for record in result.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        if let Some(f) = faces.iter().position(|face| face.id == record.id) {
            record.surface = tied.results[f].surface.clone();
        }
    }
    for (t, tie) in ties.iter().enumerate() {
        let edge_id = tie_edges[t];
        let curve = side_curve(&tied.results[tie.a].surface, tie.side_a)?;
        let edge = result.edges.iter_mut().find(|e| e.id == edge_id).ok_or_else(|| internal("shared edge record missing"))?;
        let start = result_vertex(&solid.vertices, edge.start_vertex_id)?;
        let [c0, c1] = curve.domain().map_err(|e| internal(&e))?;
        let begins_at_start = curve.evaluate(c0).map_err(|e| internal(&e))?.sub(start).length()
            <= curve.evaluate(c1).map_err(|e| internal(&e))?.sub(start).length();
        let curve = if begins_at_start { curve } else { curve.reversed().map_err(|e| internal(&e))? };
        let [c0, c1] = curve.domain().map_err(|e| internal(&e))?;
        let (t0, t1) = (edge.t0, edge.t1);
        let knots = curve.knots.iter().map(|k| t0 + (k - c0) / (c1 - c0) * (t1 - t0)).collect();
        let curve = NurbsCurve::new(curve.degree, knots, curve.control_points.clone()).map_err(|e| internal(&e))?;
        // The ends stay on the (unmoved) vertices.
        let end = result_vertex(&solid.vertices, edge.end_vertex_id)?;
        let gap = curve.evaluate(t0).map_err(|e| internal(&e))?.sub(start).length()
            .max(curve.evaluate(t1).map_err(|e| internal(&e))?.sub(end).length());
        if !(gap <= 1e-9 * scale) {
            return Err(KernelRefusal::internal(KernelStage::Validate, "deform_shell_vertex", format!(
                "{OP}: shared edge {edge_id} no longer ends on its vertices ({gap:e})"
            )));
        }
        edge.curve = curve;
    }
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::new(
            crate::RefusalClass::InvalidResultTopology { issues: issues.len() as u32 },
            KernelStage::Validate,
            format!("{OP}: the deformed body does not validate: {issues:?}"),
        ));
    }
    let report = solid_self_intersections(&result, SelfIntersectionOptions::for_solid(&result)).map_err(|e| internal(&e))?;
    if report.truncated {
        return Err(KernelRefusal::unsupported(KernelStage::Validate, "deform_scan_incomplete", format!(
            "{OP}: the self-intersection scan stopped at its size bound"
        )));
    }
    if report.is_flagged() {
        return Err(KernelRefusal::ill_posed(KernelStage::Validate, "deform_self_intersection", format!(
            "{OP}: the deformed faces pass through the body"
        )));
    }
    let volume = crate::solid_signed_volume(&result).map_err(|e| internal(&e))?;
    if !(volume > 0.0) {
        return Err(KernelRefusal::new(crate::RefusalClass::NonPositiveVolume, KernelStage::Validate, format!(
            "{OP}: the deformed body turned inside out"
        )));
    }
    let max_pick_error = tied.results.iter().map(|r| r.max_pick_error).fold(0.0, f64::max);
    let _ = pick_slots;
    Ok((
        result,
        ShellDeformReport {
            shared_edges: tie_edges.iter().copied().zip(tied.continuity.iter().copied()).collect(),
            held_corners: tied.held_corners,
            max_pick_error,
        },
    ))
}

fn result_vertex(vertices: &[crate::topology::VertexRecord], id: u64) -> Result<Vec3, KernelRefusal> {
    vertices.iter().find(|v| v.id == id).map(|v| v.point).ok_or_else(|| internal("vertex record missing"))
}

fn not_a_patch(face: u64, why: &str) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Collect, "not_a_patch_face", format!(
        "{OP}: face {face} is not a patch face ({why}); shells of trimmed faces need re-intersection (not built)"
    ))
}
