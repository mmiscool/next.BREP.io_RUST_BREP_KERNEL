//! Deform one face of a solid by its carrier's control net (Golovanov 2020
//! §3.16 deformable surface, §6.13 local deformation of a shell), keeping
//! every edge, vertex, pcurve and neighbouring face exactly as they are.
//!
//! The face's carrier is deformed by [`crate::deform_surface_where`] with a
//! locality region PROVEN to keep the face's boundary: each trim is covered by
//! the boxes of its pcurve's control polygon span by span (a NURBS curve with
//! positive weights lies in the convex hull of each span's poles), and a
//! control point may move only when no such box enters the open interior of
//! its support (the parameter box outside which its basis function is zero),
//! every box that touches the support's boundary lies on sides where the basis
//! vanishes to order 2 (its value and its first derivative across the side are
//! zero there), and the support lies inside the face (a support whose interior
//! meets no trim is wholly inside or wholly outside; one point decides). A
//! surface point and its first partials depend only on the poles whose basis
//! functions reach them to that order, so along every trim the carrier keeps
//! its points and tangent planes exactly: the edges still lie on it, G1 with
//! every neighbour is kept, and no other record of the body changes. The
//! carrier's elevation and span splits are exact, so the pcurves keep their
//! meaning.
//!
//! Before the deformed carrier is accepted the body is validated and scanned
//! for self-intersection (a face pushed through another wall), and the scan's
//! verdict is a refusal, never a repair.

use crate::topology::FaceRecord;
use crate::{
    deform_surface_where, parameter_point_in_face, PoleSupport, project_point_to_surface, solid_self_intersections,
    BrepSolid, DeformPick, DeformRefusal, DeformSpec, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface,
    PolygonClass, SelfIntersectionOptions, Vec2, Vec3,
};

const OP: &str = "deformFace";

/// One pick on the face: the surface point nearest `anchor` (which must lie
/// on the face) is moved to `target`; with `normal`, the face's normal there
/// is made parallel to it.
#[derive(Clone, Copy, Debug)]
pub struct FaceDeformPick {
    pub anchor: Vec3,
    pub target: Vec3,
    pub normal: Option<Vec3>,
}

#[derive(Clone, Debug)]
pub struct FaceDeformSpec {
    pub picks: Vec<FaceDeformPick>,
    /// Each knot span of the carrier that overlaps the face's parameter box is
    /// split into this many (1 = none). More gives the deformation more room
    /// and a tighter locality region.
    pub resolution: usize,
    /// The carrier is elevated to at least this degree first (2 ..= 9).
    pub min_degree: usize,
}

impl Default for FaceDeformSpec {
    fn default() -> Self {
        Self { picks: Vec::new(), resolution: 4, min_degree: 3 }
    }
}

/// What the deformation did, measured on the result.
#[derive(Clone, Debug)]
pub struct FaceDeformReport {
    /// The deformed carrier's control net (rows, columns) and how many of its
    /// control points could move.
    pub net: (usize, usize),
    pub free_poles: usize,
    /// Largest pick distance and normal angle (radians), read back from the
    /// deformed carrier.
    pub max_pick_error: f64,
    pub max_normal_error: f64,
    /// Each pick's carrier parameters.
    pub pick_parameters: Vec<(f64, f64)>,
}

/// Deform face `face_id` of `solid` as `spec` asks; every other record of the
/// body is returned unchanged. Refuses by name: an unknown face, a pick whose
/// anchor is off the face, a trim outside its carrier's domain, any refusal
/// of the carrier deformation (no free control point under a pick, dependent
/// picks, a fold, ...), a displacement larger than the face, and a result
/// that does not validate or passes through itself.
pub fn deform_face(
    solid: &BrepSolid,
    face_id: u64,
    spec: &FaceDeformSpec,
) -> Result<(BrepSolid, FaceDeformReport), KernelRefusal> {
    let face = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == face_id)
        .ok_or_else(|| KernelRefusal::input(KernelStage::Collect, "face_not_found", format!("{OP}: no face {face_id} in the body")))?;
    let surface = &face.surface;
    let [u0, u1] = surface.domain_u().map_err(|e| internal(&e))?;
    let [v0, v1] = surface.domain_v().map_err(|e| internal(&e))?;
    let (span_u, span_v) = (u1 - u0, v1 - v0);
    // Inside / boundary tests take ONE uv tolerance, so it comes off the
    // SMALLER span: off the larger, a patch reparameterised to a huge `v` span
    // would put every point within tolerance of a `u` side.
    let parameter_tolerance = 1e-9 * span_u.min(span_v);

    // The trims, covered span by span.
    let mut trim_boxes: Vec<[f64; 4]> = Vec::new();
    for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
        trim_boxes.extend(span_boxes(&coedge.pcurve));
    }
    if trim_boxes.is_empty() {
        return Err(KernelRefusal::unsupported(KernelStage::Collect, "face_without_trims", format!(
            "{OP}: face {face_id} has no boundary loops to keep"
        )));
    }
    let slack = 1e-6 * span_u.max(span_v);
    if let Some(outside) = trim_boxes
        .iter()
        .find(|b| b[0] < u0 - slack || b[1] > u1 + slack || b[2] < v0 - slack || b[3] > v1 + slack)
    {
        return Err(KernelRefusal::unsupported(KernelStage::Collect, "trim_outside_domain", format!(
            "{OP}: a trim of face {face_id} runs outside its carrier's domain (pcurve box {outside:?}, domain \
             [{u0}, {u1}] x [{v0}, {v1}]); the locality region cannot be proven"
        )));
    }
    let face_box = trim_boxes.iter().fold([f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY], |a, b| {
        [a[0].min(b[0]), a[1].max(b[1]), a[2].min(b[2]), a[3].max(b[3])]
    });

    // The picks, in the carrier's parameters, each on the face.
    let scale = crate::solid_model_scale(solid).max(f64::MIN_POSITIVE);
    let mut picks = Vec::with_capacity(spec.picks.len());
    let mut pick_parameters = Vec::with_capacity(spec.picks.len());
    for (index, pick) in spec.picks.iter().enumerate() {
        let foot = project_point_to_surface(surface, pick.anchor).map_err(|e| internal(&e))?;
        if !(foot.distance <= 1e-6 * scale) {
            return Err(KernelRefusal::input(KernelStage::Collect, "anchor_off_face", format!(
                "{OP}: pick {index}'s anchor {:?} is {:e} from face {face_id}'s surface",
                pick.anchor, foot.distance
            )));
        }
        let class = parameter_point_in_face(face, Vec2 { x: foot.u, y: foot.v }, parameter_tolerance).map_err(|e| internal(&e))?;
        if class != PolygonClass::Inside {
            return Err(KernelRefusal::input(KernelStage::Collect, "anchor_off_face", format!(
                "{OP}: pick {index}'s anchor {:?} lies on face {face_id}'s carrier but not inside the face ({class:?})",
                pick.anchor
            )));
        }
        picks.push(DeformPick { u: foot.u, v: foot.v, target: pick.target, normal: pick.normal });
        pick_parameters.push((foot.u, foot.v));
    }

    // A displacement in proportion to the face, not to its carrier.
    let face_size = face_extent(face, &face_box)?;
    for (index, pick) in spec.picks.iter().enumerate() {
        let moved = pick.target.sub(pick.anchor).length();
        if !(moved <= face_size) {
            return Err(KernelRefusal::input(KernelStage::Collect, "displacement_too_large", format!(
                "{OP}: pick {index} moves {moved:e}, more than face {face_id}'s size {face_size:e}"
            )));
        }
    }

    let deform_spec = DeformSpec {
        picks,
        min_degree: spec.min_degree,
        split_spans: [spec.resolution.max(1), spec.resolution.max(1)],
        split_window: Some(face_box),
        ..DeformSpec::default()
    };
    let may_move = |support: PoleSupport| support_may_move(face, &trim_boxes, parameter_tolerance, support);
    let deformed = deform_surface_where(surface, &deform_spec, &may_move).map_err(|refusal| deform_refusal(face_id, refusal))?;

    let mut result = solid.clone();
    result.mass_properties_cache = Default::default();
    for record in result.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        if record.id == face_id {
            record.surface = deformed.surface.clone();
        }
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
            "{OP}: the self-intersection scan of the deformed body stopped at its size bound, so a face passing \
             through the body cannot be ruled out"
        )));
    }
    if report.is_flagged() {
        let mut faces: Vec<u64> = Vec::new();
        for crossing in &report.confirmed {
            for id in [crossing.face_a, crossing.face_b] {
                if !faces.contains(&id) {
                    faces.push(id);
                }
            }
        }
        return Err(KernelRefusal::ill_posed(KernelStage::Validate, "deform_self_intersection", format!(
            "{OP}: the deformed face {face_id} passes through the body (faces {faces:?})"
        )));
    }
    let volume = crate::solid_signed_volume(&result).map_err(|e| internal(&e))?;
    if !(volume > 0.0) {
        return Err(KernelRefusal::new(crate::RefusalClass::NonPositiveVolume, KernelStage::Validate, format!(
            "{OP}: the deformed body turned inside out"
        )));
    }
    Ok((
        result,
        FaceDeformReport {
            net: deformed.net,
            free_poles: deformed.free_poles,
            max_pick_error: deformed.max_pick_error,
            max_normal_error: deformed.max_normal_error,
            pick_parameters,
        },
    ))
}

pub(super) fn internal(message: &str) -> KernelRefusal {
    KernelRefusal::internal(KernelStage::Collect, "deform_face", format!("{OP}: {message}"))
}

/// The carrier deformation's refusal, as the kernel's: caller errors and
/// geometry the request cannot have are `InvalidInput`/`IllPosed`, limits
/// `UnsupportedGeometry`; the slug is the deformation refusal's name.
pub(super) fn deform_refusal(face_id: u64, refusal: DeformRefusal) -> KernelRefusal {
    let message = format!("{OP}: face {face_id}: {refusal:?}");
    let slug = match &refusal {
        DeformRefusal::NoPicks => "no_picks",
        DeformRefusal::TooManyPicks { .. } => "too_many_picks",
        DeformRefusal::InvalidSpec(_) => "invalid_spec",
        DeformRefusal::InvalidPick { .. } => "invalid_pick",
        DeformRefusal::DuplicatePicks { .. } => "duplicate_picks",
        DeformRefusal::PickOnHeldRegion { .. } => "pick_on_held_region",
        DeformRefusal::DependentConstraints { .. } => "dependent_constraints",
        DeformRefusal::DisplacementTooLarge { .. } => "displacement_too_large",
        DeformRefusal::Folded { .. } => "folded",
        DeformRefusal::Amplified { .. } => "amplified",
        DeformRefusal::InvalidTie { .. } => "invalid_tie",
        DeformRefusal::EdgeRowsDisagree { .. } => "edge_rows_disagree",
        DeformRefusal::EdgeNotUniformlyTangent { .. } => "edge_not_uniformly_tangent",
        DeformRefusal::DependentTie { .. } => "dependent_tie",
        DeformRefusal::TooManyConstraints { .. } => "too_many_constraints",
        DeformRefusal::EdgeFoldedThrough { .. } => "edge_folded_through",
        DeformRefusal::InvalidWeights { .. } => "invalid_weights",
        DeformRefusal::InvalidSurface(_) => "invalid_surface",
        DeformRefusal::UnclampedKnots { .. } => "unclamped_knots",
        DeformRefusal::NetTooLarge { .. } => "net_too_large",
        DeformRefusal::IllConditioned { .. } => "ill_conditioned",
        DeformRefusal::ClosedDirectionNotHeld { .. } => "seam_not_held",
        DeformRefusal::CollapsedSideNotHeld { .. } => "pole_not_held",
        DeformRefusal::Refinement(_) => "refinement",
    };
    match &refusal {
        DeformRefusal::NoPicks
        | DeformRefusal::TooManyPicks { .. }
        | DeformRefusal::InvalidSpec(_)
        | DeformRefusal::InvalidPick { .. }
        | DeformRefusal::DuplicatePicks { .. } => KernelRefusal::input(KernelStage::Collect, slug, message),
        DeformRefusal::PickOnHeldRegion { .. }
        | DeformRefusal::DependentConstraints { .. }
        | DeformRefusal::DisplacementTooLarge { .. }
        | DeformRefusal::Folded { .. }
        | DeformRefusal::Amplified { .. }
        | DeformRefusal::DependentTie { .. }
        | DeformRefusal::EdgeFoldedThrough { .. }
        | DeformRefusal::EdgeNotUniformlyTangent { .. } => KernelRefusal::ill_posed(KernelStage::Refine, slug, message),
        _ => KernelRefusal::unsupported(KernelStage::Refine, slug, message),
    }
}

/// The locality rule: a control point with `support` may move when no trim box
/// enters its support's open interior, every box touching the support lies on
/// sides where its basis vanishes to order 2, and the support lies inside the
/// face (one point decides: its interior meets no trim).
pub(super) fn support_may_move(face: &FaceRecord, trim_boxes: &[[f64; 4]], parameter_tolerance: f64, support: PoleSupport) -> bool {
    let [a0, a1, b0, b1] = support.bounds;
    // Exact comparisons: a trim even a rounding error inside a support's
    // side is inside it (holding the pole is the safe answer).
    for b in trim_boxes {
        // Clear of the closed support: no effect at all.
        if b[1] < a0 || b[0] > a1 || b[3] < b0 || b[2] > b1 {
            continue;
        }
        // Into the open interior: the pole reaches the trim.
        if b[1] > a0 && b[0] < a1 && b[3] > b0 && b[2] < b1 {
            return false;
        }
        // Only along the support's sides: each side the box lies on must
        // have the basis vanish to order 2 there (value and the first
        // derivative across it), so the edge and its tangent plane hold.
        let on = [b[1] <= a0, b[0] >= a1, b[3] <= b0, b[2] >= b1];
        if on.iter().zip(support.vanishing).any(|(&touch, order)| touch && order < 2) {
            return false;
        }
    }
    let centre = Vec2 { x: 0.5 * (a0 + a1), y: 0.5 * (b0 + b1) };
    matches!(parameter_point_in_face(face, centre, parameter_tolerance), Ok(PolygonClass::Inside))
}

/// Boxes `[u0, u1, v0, v1]` covering a pcurve: one per knot span, the box of
/// the poles that span uses (convex hull property, weights positive).
pub(super) fn span_boxes(pcurve: &NurbsCurve) -> Vec<[f64; 4]> {
    let p = pcurve.degree;
    let count = pcurve.control_points.len();
    let mut boxes = Vec::new();
    for span in p..count {
        if pcurve.knots[span + 1] <= pcurve.knots[span] {
            continue;
        }
        let mut b = [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY];
        for pole in &pcurve.control_points[span - p..=span] {
            let (u, v) = (pole.x / pole.w, pole.y / pole.w);
            b = [b[0].min(u), b[1].max(u), b[2].min(v), b[3].max(v)];
        }
        boxes.push(b);
    }
    boxes
}

/// The face's size: the box diagonal of its carrier sampled over the face's
/// parameter box.
fn face_extent(face: &FaceRecord, face_box: &[f64; 4]) -> Result<f64, KernelRefusal> {
    let surface: &NurbsSurface = &face.surface;
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for a in 0..=8 {
        for b in 0..=8 {
            let u = face_box[0] + (face_box[1] - face_box[0]) * a as f64 / 8.0;
            let v = face_box[2] + (face_box[3] - face_box[2]) * b as f64 / 8.0;
            let p = surface.evaluate(u, v).map_err(|e| internal(&e))?;
            lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
            hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
        }
    }
    Ok(hi.sub(lo).length())
}

/// The face's deepest interior point: of a 25 x 25 grid over its parameter
/// box, the point inside the face farthest (in space) from its boundary,
/// sampled along every coedge. `None` when no grid point is inside.
pub fn face_deform_default_anchor(face: &FaceRecord) -> Option<Vec3> {
    let surface = &face.surface;
    let boxes: Vec<[f64; 4]> = face.loops.iter().flat_map(|lp| &lp.coedges).flat_map(|c| span_boxes(&c.pcurve)).collect();
    let face_box = boxes.iter().fold([f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY], |a, b| {
        [a[0].min(b[0]), a[1].max(b[1]), a[2].min(b[2]), a[3].max(b[3])]
    });
    if !face_box.iter().all(|x| x.is_finite()) {
        return None;
    }
    let mut boundary = Vec::new();
    for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
        let Ok([s0, s1]) = coedge.pcurve.domain() else { continue };
        for k in 0..=32 {
            let Ok(uv) = coedge.pcurve.evaluate(s0 + (s1 - s0) * k as f64 / 32.0) else { continue };
            if let Ok(p) = surface.evaluate(uv.x, uv.y) {
                boundary.push(p);
            }
        }
    }
    let tolerance = 1e-9 * (face_box[1] - face_box[0]).min(face_box[3] - face_box[2]);
    let mut best: Option<(f64, Vec3)> = None;
    for a in 1..25 {
        for b in 1..25 {
            let u = face_box[0] + (face_box[1] - face_box[0]) * a as f64 / 25.0;
            let v = face_box[2] + (face_box[3] - face_box[2]) * b as f64 / 25.0;
            if !matches!(parameter_point_in_face(face, Vec2 { x: u, y: v }, tolerance), Ok(PolygonClass::Inside)) {
                continue;
            }
            let Ok(p) = surface.evaluate(u, v) else { continue };
            let depth = boundary.iter().map(|q| q.sub(p).length()).fold(f64::INFINITY, f64::min);
            if best.is_none_or(|(d, _)| depth > d) {
                best = Some((depth, p));
            }
        }
    }
    best.map(|(_, p)| p)
}
