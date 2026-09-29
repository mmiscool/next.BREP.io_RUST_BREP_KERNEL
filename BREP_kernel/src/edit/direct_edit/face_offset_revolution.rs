use super::*;
use crate::{ExtendRefusal, KernelRefusal, KernelStage, OrRefuse};

/// Push a general surface of revolution with planar neighbours.
///
/// The meridian is densely offset and interpolated, then revolved through the
/// original full or partial sweep. A dense normal-distance residual check
/// bounds that approximation. Endpoint rims and start/end meridians are rebuilt,
/// and planar axial/radial neighbours are re-trimmed around the new boundary.
/// The offset meridian runs between the source parameters at which it reaches
/// each rim's cap plane, so a rim is the offset carrier's intersection with its
/// cap rather than the offset of the old rim (see
/// [`offset_general_revolution_face`]).
///
/// A PARTIAL CYLINDER (an extruded sketch arc, a fillet band between planes)
/// does not take the general lane: its meridian edges usually meet planes that
/// are parallel to the axis but do not contain it, where keeping a corner at
/// its old azimuth is wrong. [`offset_partial_cylinder_face`] re-intersects
/// those corners exactly first, and only a face outside its scope falls
/// through to the general lane below.
///
/// A boundary whose pcurve is NEITHER a constant-u nor a constant-v line is no
/// longer refused outright: its 3D image on the offset carrier comes from
/// [`crate::image_curve`]. That transfer is only valid where the boundary is
/// definitional, so a general image — or a constant-u meridian — against a
/// FIXED planar neighbour is measured against that neighbour's plane and
/// refused, with the number, when it leaves it, because the honest answer
/// there is a re-intersection this site does not do.
pub fn offset_revolution_face(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    if !distance.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "distance",
            "offset_revolution_face: distance must be finite",
        ));
    }
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let residual_tolerance = (scale * 1e-4).max(1e-6);
    // Bar for the general 3D-image ladder: a CURVE-fit accuracy target, which
    // is a different question from `residual_tolerance`'s bound on the offset
    // SURFACE, so it takes the kernel's single named fit-accuracy field.
    let fit_tolerance = crate::KernelTolerances::for_scale(scale, 1e-7).intersection_fit;
    let (shell_index, face_index) = find_face(solid, face_id).ok_or_else(|| {
        KernelRefusal::input(
            KernelStage::Collect,
            "face_id",
            format!("offset_revolution_face: no face {face_id}"),
        )
    })?;
    let face = &solid.shells[shell_index].faces[face_index];
    if !matches!(face.surface.analytic(), Some(AnalyticSurface::Revolution { .. })) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "not_revolution",
            "offset_revolution_face: the pushed face is not a general revolution",
        ));
    }
    if let Some(result) = offset_partial_cylinder_face(
        solid,
        face_id,
        distance,
        tolerance,
        residual_tolerance,
        scale,
    )? {
        return Ok(result);
    }
    let trace = BoundaryTrace::open(solid, face_id, distance);
    let outcome = offset_general_revolution_face(
        solid,
        face_id,
        distance,
        tolerance,
        residual_tolerance,
        fit_tolerance,
        scale,
        &trace,
    );
    trace.outcome(&outcome);
    outcome
}

/// The general lane of [`offset_revolution_face`]: every face the partial
/// cylinder lane does not take.
///
/// Its constant-v RIMS are solved, not transferred. A rim is the circle a
/// point of the generatrix sweeps, and against a FIXED cap the new rim is the
/// offset carrier's intersection with that cap's plane. The offset of the
/// source rim's own generatrix point is not in that plane wherever the
/// generatrix meets the cap at an angle — the offset normal there has an axial
/// component `d·n_axis` — and rebuilding the rim at that point left the lib
/// barrel's rims 8.602e-2 and 6.155e-2 off their caps at d = 0.15 and a solid
/// that validated 1.757 above the exact offset's volume. So each rim's source
/// parameter is solved where the offset meridian reaches its cap's plane
/// ([`solve_rim_span`]), the offset generatrix is sampled between those two
/// parameters — through the source carrier's natural extension where a solved
/// parameter leaves its domain — and every rebuilt rim is measured against its
/// plane and refused, with the number, past the pcurve floor.
#[allow(clippy::too_many_arguments)]
fn offset_general_revolution_face(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
    tolerance: f64,
    residual_tolerance: f64,
    fit_tolerance: f64,
    scale: f64,
    trace: &BoundaryTrace,
) -> Result<BrepSolid, KernelRefusal> {
    let (shell_index, face_index) = find_face(solid, face_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "face_lookup",
            format!("offset_revolution_face: no face {face_id}"),
        )
    })?;
    let face = &solid.shells[shell_index].faces[face_index];
    let Some(AnalyticSurface::Revolution { sweep, .. }) = face.surface.analytic() else {
        return Err(KernelRefusal::internal(
            KernelStage::Collect,
            "carrier_kind",
            "offset_revolution_face: the pushed face is not a general revolution",
        ));
    };
    let source_structure = crate::revolution_structure(&face.surface).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Classify,
            "revolution_structure",
            "offset_revolution_face: source carrier lost its revolution structure",
        )
    })?;
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Classify, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;
    if (v0.abs() + (v1 - 1.0).abs()) > 1e-9 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "generatrix_domain",
            "offset_revolution_face: a non-normalized generatrix domain is deferred",
        ));
    }
    let mut faces_of_edge: HashMap<u64, Vec<u64>> = HashMap::default();
    for shell in &solid.shells {
        for candidate in &shell.faces {
            for loop_record in &candidate.loops {
                for coedge in &loop_record.coedges {
                    faces_of_edge
                        .entry(coedge.edge_id)
                        .or_default()
                        .push(candidate.id);
                }
            }
        }
    }
    let edge_by_id: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    // Offset the meridian densely, then revolve it. A general curve offset is
    // not exactly rational; using 33 cubic interpolation stations materially
    // lowers the residual compared with fitting the whole surface back onto its
    // original sparse control net, while retaining the [0,1] pcurve contract.
    let u_probe = u0 + (u1 - u0) * 0.137;
    let span = solve_rim_span(
        solid,
        face,
        face_id,
        &faces_of_edge,
        &source_structure.frame,
        u_probe,
        distance,
        residual_tolerance,
    )?;
    let source_surface = span.surface.as_ref().unwrap_or(&face.surface);
    // The dense meridian sample is a pointwise offset — the shared evaluator's
    // `Face` lane, whose distance is signed ALONG the oriented normal exactly
    // like direct-edit's own convention (audit §4.1: this side of the kernel
    // grows the solid on a positive distance).
    let offsets = OffsetEvaluator::new(
        "push_revolution",
        source_surface,
        OffsetNormal::Face {
            same_sense: face.same_sense,
        },
    );
    // The carrier's generatrix parameter `w` runs over [0, 1] while the source
    // parameter it offsets runs over the solved span.
    let [s0, s1] = span.new;
    let source_parameter = |w: f64| s0 + (s1 - s0) * w;
    let mut meridian_points = Vec::with_capacity(33);
    let mut meridian_parameters = Vec::with_capacity(33);
    for index in 0..=32 {
        let parameter = index as f64 / 32.0;
        let v = source_parameter(v0 + (v1 - v0) * parameter);
        let target_point = offsets.at(u_probe, v, distance).or_refuse(KernelStage::Refine, "offset_at")?.point;
        let relative = target_point.sub(source_structure.frame.origin);
        let axial = relative.dot(source_structure.frame.axis);
        let radial = relative
            .sub(source_structure.frame.axis.scale(axial))
            .length();
        meridian_points.push(
            source_structure
                .frame
                .origin
                .add(source_structure.frame.x_axis.scale(radial))
                .add(source_structure.frame.axis.scale(axial)),
        );
        meridian_parameters.push(parameter);
    }
    let offset_generatrix = crate::interpolate_curve(&meridian_points, 3, &meridian_parameters).or_refuse(KernelStage::Refine, "interpolate_curve")?;
    let offset = crate::make_revolution(
        source_structure.frame.origin,
        source_structure.frame.axis,
        &offset_generatrix,
        *sweep,
    ).or_refuse(KernelStage::Refine, "make_revolution")?;
    let structure = crate::revolution_structure(&offset).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Refine,
            "offset_structure",
            "offset_revolution_face: offset carrier lost its revolution structure — refusing",
        )
    })?;
    if !matches!(offset.analytic(), Some(AnalyticSurface::Revolution { .. })) {
        return Err(KernelRefusal::internal(
            KernelStage::Refine,
            "offset_kind",
            "offset_revolution_face: offset carrier changed analytic kind — refusing",
        ));
    }

    // A general interpolated offset is approximate. Admit it only when its
    // same-parameter displacement stays normal and at the requested distance.
    let target = distance.abs();
    let (mut worst_distance, mut worst_normal) = (0.0f64, 0.0f64);
    // Stagger away from repeated knots: derivatives at an exact full-period
    // seam or a multiple internal knot are side-dependent and may be zero in
    // the generic evaluator even though the carrier is regular there.
    for iu in 0..13 {
        for iv in 0..13 {
            let u = u0 + (u1 - u0) * (iu as f64 + 0.37) / 13.0;
            let w = v0 + (v1 - v0) * (iv as f64 + 0.41) / 13.0;
            let v = source_parameter(w);
            let source_point = source_surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
            let offset_point = offset.evaluate(u, w).or_refuse(KernelStage::Refine, "evaluate")?;
            let delta = offset_point.sub(source_point);
            let normal = offsets
                .normal(u, v)
                .map_err(|error| {
                    KernelRefusal::internal(
                        KernelStage::Refine,
                        "sample_normal",
                        format!("offset_revolution_face: sample normal: {error}"),
                    )
                })?;
            let distance_error = (delta.length() - target).abs();
            let normal_error = if delta.length() > tolerance {
                1.0 - delta
                    .normalized()
                    .map_err(|error| {
                        KernelRefusal::internal(
                            KernelStage::Refine,
                            "sample_delta",
                            format!("offset_revolution_face: sample delta: {error}"),
                        )
                    })?
                    .dot(normal)
                    .abs()
            } else {
                0.0
            };
            if distance_error > residual_tolerance || normal_error > 5e-4 {
                return Err(KernelRefusal::non_convergence(
                    KernelStage::Refine,
                    "offset_fit",
                    format!(
                    "offset_revolution_face: offset fit exceeds tolerance \
                     (distance error {distance_error:.3e}, normal error {normal_error:.3e})"
                ),
                ));
            }
            worst_distance = worst_distance.max(distance_error);
            worst_normal = worst_normal.max(normal_error);
        }
    }
    trace.write(format!(
        "carrier\trims_old={:?}\trims_new={:?}\textended={}\t\
         fit_distance_error={worst_distance:.3e}\tfit_normal_error={worst_normal:.3e}",
        span.old,
        span.new,
        span.surface.is_some()
    ));

    // The offset generatrix endpoints revolve into the only two possible rims:
    // the low rim at w = 0, the high rim at w = 1.
    let [g0, g1] = structure.generatrix.domain().or_refuse(KernelStage::Refine, "domain")?;
    let mut rim_candidates = Vec::new();
    for parameter in [g0, g1] {
        let point = structure.generatrix.evaluate(parameter).or_refuse(KernelStage::Refine, "evaluate")?;
        let relative = point.sub(structure.frame.origin);
        let axial = relative.dot(structure.frame.axis);
        let radial = relative.sub(structure.frame.axis.scale(axial));
        let radius = radial.length();
        if radius <= tolerance {
            return Err(KernelRefusal::input(
                KernelStage::Refine,
                "rim_collapse",
                format!(
                "offset_revolution_face: an offset rim collapses onto the axis \
                 (point {point:?}, frame {:?}) — refusing",
                structure.frame
            ),
            ));
        }
        rim_candidates.push(crate::make_arc(
            structure
                .frame
                .origin
                .add(structure.frame.axis.scale(axial)),
            structure.frame.x_axis,
            structure.frame.y_axis,
            radius,
            0.0,
            structure.sweep,
        ).or_refuse(KernelStage::Refine, "make_arc")?);
    }

    // Each boundary's pcurve on the new carrier. A rim that is not at a
    // generatrix end — a face trimmed across its carrier — moves to w = 0 or 1,
    // so the ISO boundaries are carried by the affine map that takes the two
    // rims' old parameters there. A GENERAL boundary is a transfer of the old
    // one, so it keeps its source parameter: w = (s − s₀)/(s₁ − s₀).
    let uv_tolerance = 1e-7;
    let pcurves = face
        .loops
        .iter()
        .map(|loop_record| {
            loop_record
                .coedges
                .iter()
                .map(|coedge| match iso_kind(&coedge.pcurve, uv_tolerance)? {
                    Some(_) => span.carry_pcurve(&coedge.pcurve),
                    None => span.transfer_pcurve(&coedge.pcurve),
                })
                .collect::<Result<Vec<_>, KernelRefusal>>()
        })
        .collect::<Result<Vec<_>, KernelRefusal>>()?;

    let floor = crate::pcurve::PCURVE_REFINEMENT_TOLERANCE;
    let mut new_curves: HashMap<u64, NurbsCurve> = HashMap::default();
    let mut new_vertices: HashMap<u64, Vec3> = HashMap::default();
    let mut planar_neighbours: HashSet<u64> = HashSet::default();
    for (loop_record, loop_pcurves) in face.loops.iter().zip(&pcurves) {
        for (coedge, pcurve) in loop_record.coedges.iter().zip(loop_pcurves) {
            if new_curves.contains_key(&coedge.edge_id) {
                continue;
            }
            let edge = *edge_by_id.get(&coedge.edge_id).ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Classify,
                    "missing_edge",
                    format!("offset_revolution_face: missing edge {}", coedge.edge_id),
                )
            })?;
            let incident = faces_of_edge.get(&edge.id).cloned().unwrap_or_default();
            let neighbour = incident.iter().find(|candidate| **candidate != face_id);
            // Kept for the gates below: an edge shared with a FIXED planar
            // cap has to stay in that cap's plane, and only this loop knows
            // which plane that is.
            let mut neighbour_plane = None;
            if let Some(neighbour) = neighbour {
                let (ns, nf) = find_face(solid, *neighbour).ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Classify,
                        "missing_neighbour",
                        format!("offset_revolution_face: missing neighbour {neighbour}"),
                    )
                })?;
                match plane_of_surface(
                    &solid.shells[ns].faces[nf].surface,
                    residual_tolerance,
                    "offset_revolution_face",
                ) {
                    Err(_) => {
                        return Err(KernelRefusal::unsupported(
                            KernelStage::Classify,
                            "curved_neighbour",
                            "offset_revolution_face: only planar neighbours are supported \
                                for a general revolution — refusing",
                        ));
                    }
                    Ok(plane) => neighbour_plane = Some(plane),
                }
                planar_neighbours.insert(*neighbour);
            }

            // Constant-v pcurve → a revolved endpoint rim. Constant-u → a
            // start/end meridian (or the intrinsic full-sweep seam).
            let mut general_image = false;
            let mut meridian = false;
            let mut curve = match iso_kind(&coedge.pcurve, uv_tolerance)? {
                Some(IsoKind::ConstantV) => {
                    let [q0, _] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                    let w = pcurve.evaluate(q0).or_refuse(KernelStage::Refine, "evaluate")?.y;
                    if w.abs() <= uv_tolerance {
                        rim_candidates[0].clone()
                    } else if (w - 1.0).abs() <= uv_tolerance {
                        rim_candidates[1].clone()
                    } else {
                        return Err(KernelRefusal::unsupported(
                            KernelStage::Classify,
                            "interior_rim",
                            format!(
                            "offset_revolution_face: constant-v boundary edge {} sits at v = {w} \
                             between the face's two rims — refusing",
                            edge.id
                        ),
                        ));
                    }
                }
                Some(IsoKind::ConstantU(u)) => {
                    meridian = true;
                    offset.iso_curve_u(u).or_refuse(KernelStage::Refine, "iso_curve_u")?
                }
                None => {
                    // A GENERAL boundary pcurve — what this site used to refuse.
                    // Its image on the offset carrier is built by the shared
                    // ladder, oriented off `coedge.forward` (the authoritative
                    // pcurve↔edge relation) rather than the iso lanes' geometric
                    // nearest-endpoint match, because a general image need not
                    // share an endpoint with the old curve at all.
                    general_image = true;
                    let image = crate::image_curve(
                        &offset,
                        pcurve,
                        fit_tolerance,
                        "offset_revolution_face",
                    ).or_refuse(KernelStage::Refine, "image_curve")?;
                    if coedge.forward {
                        image.curve
                    } else {
                        image.curve.reversed().or_refuse(KernelStage::Refine, "reversed")?
                    }
                }
            };
            let kind = if general_image {
                "general"
            } else if meridian {
                "meridian"
            } else {
                "rim"
            };
            trace.edge(kind, edge, neighbour, neighbour_plane.as_ref(), &curve);
            // Every rebuilt boundary is measured against the FIXED planar
            // neighbour it must lie in; the validator's `pcurve_acceptance`
            // band (2.5% of the model diagonal) is far too loose to catch one
            // that leaves it.
            //
            // A RIM is built in its cap's plane by `solve_rim_span`, so past
            // the pcurve floor it is a cap this lane cannot meet — one not
            // normal to the axis, or two coplanar caps of one rim that
            // disagree — and it is refused with the number.
            //
            // A general image, or a constant-u MERIDIAN, is a transfer, which
            // is only legitimate where the boundary is DEFINITIONAL. Keeping a
            // meridian's azimuth is exact only when the neighbour plane contains
            // the axis (a partial revolve's radial end face); a plane beside the
            // axis — the straight sketch segment next to an extruded arc —
            // needs the corner re-intersected, and the kept azimuth left it
            // 0.33 off that plane on a reported extrude, where re-trimming the
            // neighbour around it tilted the wall and still validated.
            if let Some(plane) = &neighbour_plane {
                let [g0, g1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
                let mut worst = 0.0f64;
                for index in 0..=32 {
                    let t = g0 + (g1 - g0) * index as f64 / 32.0;
                    let point = curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
                    worst = worst.max(point.sub(plane.origin).dot(plane.normal).abs());
                }
                if !(general_image || meridian) && !(worst <= floor) {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Refine,
                        "rim_off_cap",
                        format!(
                        "offset_revolution_face: the rebuilt rim of boundary edge {} leaves the \
                         plane of neighbour face {} by {worst:.3e} (pcurve floor {floor:.0e}) — \
                         refusing",
                        edge.id,
                        neighbour.copied().unwrap_or_default()
                    ),
                    ));
                }
                if (general_image || meridian) && worst > residual_tolerance {
                    let lane = if meridian {
                        "meridian at its old azimuth"
                    } else {
                        "general image"
                    };
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Refine,
                        "transfer_off_neighbour",
                        format!(
                        "offset_revolution_face: the {lane} of boundary edge {} \
                         leaves its fixed planar neighbour by {worst:.3e} (limit \
                         {residual_tolerance:.3e}) — re-intersecting the offset carrier \
                         with a fixed neighbour is a separate capability; refusing",
                        edge.id
                    ),
                    ));
                }
            }
            if !general_image {
                let [c0, c1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
                let old_start = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?;
                if curve.evaluate(c0).or_refuse(KernelStage::Refine, "evaluate")?.sub(old_start).length()
                    > curve.evaluate(c1).or_refuse(KernelStage::Refine, "evaluate")?.sub(old_start).length()
                {
                    curve = curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
                }
            }
            // A corner is rebuilt by each boundary edge that ends on it, and
            // those must agree: a rim and a meridian meeting at one vertex, or
            // two rim arcs of one split circle. One that does not is a rim this
            // lane built across a vertex it does not end at.
            let [d0, d1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let ends = [
                (edge.start_vertex_id, curve.evaluate(d0).or_refuse(KernelStage::Refine, "evaluate")?),
                (edge.end_vertex_id, curve.evaluate(d1).or_refuse(KernelStage::Refine, "evaluate")?),
            ];
            for (vertex_id, point) in ends {
                if let Some(previous) = new_vertices.insert(vertex_id, point) {
                    let gap = previous.sub(point).length();
                    if !(gap <= floor) {
                        return Err(KernelRefusal::internal(
                            KernelStage::Fragment,
                            "vertex_agreement",
                            format!(
                            "offset_revolution_face: boundary edge {} rebuilds vertex {vertex_id} \
                             {gap:.3e} from where another boundary edge put it (pcurve floor \
                             {floor:.0e}) — refusing",
                            edge.id
                        ),
                        ));
                    }
                }
            }
            new_curves.insert(edge.id, curve);
        }
    }

    let mut result = solid.clone();
    let pushed = &mut result.shells[shell_index].faces[face_index];
    if span.old != [0.0, 1.0] || span.new != [0.0, 1.0] {
        for (loop_record, loop_pcurves) in pushed.loops.iter_mut().zip(pcurves) {
            for (coedge, pcurve) in loop_record.coedges.iter_mut().zip(loop_pcurves) {
                coedge.pcurve = pcurve;
            }
        }
    }
    pushed.surface = offset;
    retrim_planar_neighbours(
        solid,
        result,
        &edge_by_id,
        &new_curves,
        &new_vertices,
        planar_neighbours,
        residual_tolerance,
        scale,
    )
}

/// Where the offset carrier's generatrix starts and ends.
struct RimSpan {
    /// The face's low and high rim parameters on the SOURCE generatrix — the
    /// domain end where the face has no rim on that side.
    old: [f64; 2],
    /// The source parameters whose offset meridian points lie in those rims'
    /// cap planes: the new carrier's w = 0 and w = 1.
    new: [f64; 2],
    /// The source carrier extended past its domain by its own terminal span
    /// ([`NurbsSurface::extend_natural`]), when a solved parameter leaves it.
    surface: Option<NurbsSurface>,
}

impl RimSpan {
    /// An iso boundary's pcurve on the new carrier: `v ↦ (v − old₀)/(old₁ −
    /// old₀)`, which puts the two rims at w = 0 and 1.
    fn carry_pcurve(&self, pcurve: &NurbsCurve) -> Result<NurbsCurve, KernelRefusal> {
        Self::map_v(pcurve, self.old)
    }

    /// A general boundary's pcurve on the new carrier: `v ↦ (v − new₀)/(new₁ −
    /// new₀)`, the same source parameter and so the same offset point.
    fn transfer_pcurve(&self, pcurve: &NurbsCurve) -> Result<NurbsCurve, KernelRefusal> {
        Self::map_v(pcurve, self.new)
    }

    /// `pcurve` with `v ↦ (v − a₀)/(a₁ − a₀)` applied to its control net (an
    /// affine map, so exact), or the pcurve itself when that is the identity.
    fn map_v(pcurve: &NurbsCurve, [a0, a1]: [f64; 2]) -> Result<NurbsCurve, KernelRefusal> {
        if [a0, a1] == [0.0, 1.0] {
            return Ok(pcurve.clone());
        }
        let mut carried = pcurve.clone();
        for control in &mut carried.control_points {
            let point = control.point().or_refuse(KernelStage::Classify, "point")?;
            *control = crate::Vec4::from_point(
                Vec3::new(point.x, (point.y - a0) / (a1 - a0), point.z),
                control.w,
            );
        }
        Ok(carried)
    }
}

/// Solve the source generatrix parameters between which the offset carrier
/// must run so that each constant-v rim of `face` lies in the plane of the
/// fixed cap across it.
///
/// A rim at source parameter `a` against a cap whose plane the axis pierces at
/// station `h` moves to the root of `f(s) = axial(S(u, s) + d·N(u, s)) − h`
/// nearest `a`. The root is bracketed by stepping from `a` along the source
/// meridian's own axial rate (doubling the step up to four times), extending
/// the source carrier by its natural continuation where the bracket leaves the
/// domain, and bisected to exhaustion. A rim whose generatrix leaves its cap
/// tangentially (no axial rate to step along, and a crossing on either side of
/// the rim), and one whose root cannot be bracketed, are refused by name with
/// the drift.
#[allow(clippy::too_many_arguments)]
fn solve_rim_span(
    solid: &BrepSolid,
    face: &FaceRecord,
    face_id: u64,
    faces_of_edge: &HashMap<u64, Vec<u64>>,
    frame: &crate::RevolutionFrame,
    u: f64,
    distance: f64,
    residual_tolerance: f64,
) -> Result<RimSpan, KernelRefusal> {
    let op = "offset_revolution_face";
    let uv_tolerance = 1e-7;
    // (source parameter, cap station, cap face) of every constant-v boundary
    // with a planar neighbour, and the face's parameter extent.
    let mut rims: Vec<(f64, f64, u64)> = Vec::new();
    let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
        let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Classify, "domain")?;
        let uv0 = coedge.pcurve.evaluate(q0).or_refuse(KernelStage::Classify, "evaluate")?;
        let uv1 = coedge.pcurve.evaluate(q1).or_refuse(KernelStage::Classify, "evaluate")?;
        let uvm = coedge.pcurve.evaluate(0.5 * (q0 + q1)).or_refuse(KernelStage::Classify, "evaluate")?;
        for uv in [uv0, uv1, uvm] {
            v_min = v_min.min(uv.y);
            v_max = v_max.max(uv.y);
        }
        if !matches!(iso_kind(&coedge.pcurve, uv_tolerance)?, Some(IsoKind::ConstantV)) {
            continue;
        }
        let incident = faces_of_edge.get(&coedge.edge_id).cloned().unwrap_or_default();
        let Some(neighbour) = incident.iter().find(|candidate| **candidate != face_id) else {
            continue;
        };
        let (ns, nf) = find_face(solid, *neighbour).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Classify,
                "rim_neighbour",
                format!("{op}: missing neighbour {neighbour}"),
            )
        })?;
        // A curved cap is refused by the boundary loop, ahead of any rim.
        let Ok(plane) =
            plane_of_surface(&solid.shells[ns].faces[nf].surface, residual_tolerance, op)
        else {
            continue;
        };
        // Where the axis pierces the cap's plane.
        let station =
            plane.origin.sub(frame.origin).dot(plane.normal) / frame.axis.dot(plane.normal);
        rims.push((uv0.y, station, *neighbour));
    }
    if rims.is_empty() {
        return Ok(RimSpan {
            old: [0.0, 1.0],
            new: [0.0, 1.0],
            surface: None,
        });
    }
    // One rim per end: every constant-v boundary sits at the face's lowest or
    // highest parameter.
    let mut ends: [Option<(f64, f64, u64)>; 2] = [None, None];
    for rim in &rims {
        let slot = if (rim.0 - v_min).abs() <= (rim.0 - v_max).abs() { 0 } else { 1 };
        match ends[slot] {
            None => ends[slot] = Some(*rim),
            Some(first) if (first.0 - rim.0).abs() > uv_tolerance => {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "two_rims_one_side",
                    format!(
                    "{op}: face {face_id} has constant-v boundaries at v = {} and v = {} on one \
                     side of its trim — refusing",
                    first.0, rim.0
                ),
                ));
            }
            Some(_) => {}
        }
    }

    let same_sense = face.same_sense;
    let mut surface: Option<NurbsSurface> = None;
    let mut old = [0.0, 1.0];
    let mut new = [0.0, 1.0];
    for (slot, end) in ends.iter().enumerate() {
        let Some((start, station, neighbour)) = *end else {
            continue;
        };
        old[slot] = start;
        let axial_miss = |carrier: &NurbsSurface, s: f64| -> Result<f64, KernelRefusal> {
            let convention = OffsetNormal::Face { same_sense };
            let point = OffsetEvaluator::new("push_revolution", carrier, convention)
                .at(u, s, distance).or_refuse(KernelStage::Refine, "offset_at")?
                .point;
            Ok(point.sub(frame.origin).dot(frame.axis) - station)
        };
        let base = surface.clone().unwrap_or_else(|| face.surface.clone());
        let drift = axial_miss(&base, start)?;
        if drift == 0.0 {
            new[slot] = start;
            continue;
        }
        let rate = face.surface.derivatives(u, start, 1).or_refuse(KernelStage::Refine, "derivatives")?[0][1].dot(frame.axis);
        let step = -drift / rate;
        if !step.is_finite() {
            // The generatrix leaves its cap along the cap: the offset meridian
            // dips through the plane on BOTH sides of the rim, and which
            // crossing is the rim is not the solve's to pick.
            return Err(KernelRefusal::ill_posed(
                KernelStage::Refine,
                "tangent_rim",
                format!(
                "{op}: the generatrix of face {face_id} meets the plane of neighbour face \
                 {neighbour} tangentially at its rim, which the push leaves {drift:.3e} off \
                 that plane — refusing"
            ),
            ));
        }
        let mut bracket = None;
        let mut reach = 2.0 * step;
        for _ in 0..4 {
            if !reach.is_finite() {
                break;
            }
            let far = start + reach;
            let carrier = extend_to(&base, far)
                .map_err(|refusal| {
                    // The extension's own text rides along verbatim; its class is read off
                    // the variant — a lower-level failure is a defect, every other variant
                    // is a limit of the natural continuation this lane cannot cross.
                    let message = {
                        let refusal = refusal.describe();
                        format!(
                        "{op}: the rim of face {face_id} against neighbour face {neighbour} lies \
                         {drift:.3e} off that cap, and the carrier cannot be extended to reach \
                         it: {refusal} — refusing"
                    )
                    };
                    match refusal {
                        ExtendRefusal::Failed(_) => KernelRefusal::internal(
                            KernelStage::Refine,
                            "carrier_extension_failed",
                            message,
                        ),
                        _ => KernelRefusal::unsupported(
                            KernelStage::Refine,
                            "carrier_extension",
                            message,
                        ),
                    }
                })?;
            let far_miss = axial_miss(carrier.as_ref().unwrap_or(&base), far)?;
            if far_miss == 0.0 || far_miss.signum() != drift.signum() {
                bracket = Some((far, carrier));
                break;
            }
            reach *= 2.0;
        }
        let Some((far, carrier)) = bracket else {
            return Err(KernelRefusal::non_convergence(
                KernelStage::Refine,
                "rim_bracket",
                format!(
                "{op}: the offset meridian of face {face_id} does not reach the plane of \
                 neighbour face {neighbour} near its rim, which the push leaves {drift:.3e} off \
                 that plane — refusing"
            ),
            ));
        };
        let carrier_ref = carrier.as_ref().unwrap_or(&base);
        let (mut near, mut far) = (start, far);
        for _ in 0..200 {
            let middle = 0.5 * (near + far);
            if middle == near || middle == far {
                break;
            }
            if axial_miss(carrier_ref, middle)?.signum() == drift.signum() {
                near = middle;
            } else {
                far = middle;
            }
        }
        new[slot] = 0.5 * (near + far);
        if carrier.is_some() {
            surface = carrier;
        }
    }
    if !(new[1] > new[0]) {
        return Err(KernelRefusal::input(
            KernelStage::Refine,
            "rims_crossed",
            format!(
            "{op}: pushing face {face_id} by {distance} crosses its two rims over (source \
             parameters {} and {}) — refusing",
            new[0], new[1]
        ),
        ));
    }
    Ok(RimSpan { old, new, surface })
}

/// Which iso line of its carrier a boundary pcurve is, by the same three-sample
/// test the lane has always used; `None` for a general pcurve.
enum IsoKind {
    ConstantV,
    ConstantU(f64),
}

fn iso_kind(pcurve: &NurbsCurve, uv_tolerance: f64) -> Result<Option<IsoKind>, KernelRefusal> {
    let [q0, q1] = pcurve.domain().or_refuse(KernelStage::Classify, "domain")?;
    let uv0 = pcurve.evaluate(q0).or_refuse(KernelStage::Classify, "evaluate")?;
    let uv1 = pcurve.evaluate(q1).or_refuse(KernelStage::Classify, "evaluate")?;
    let uvm = pcurve.evaluate(0.5 * (q0 + q1)).or_refuse(KernelStage::Classify, "evaluate")?;
    Ok(
        if (uv0.y - uv1.y).abs() <= uv_tolerance && (uv0.y - uvm.y).abs() <= uv_tolerance {
            Some(IsoKind::ConstantV)
        } else if (uv0.x - uv1.x).abs() <= uv_tolerance && (uv0.x - uvm.x).abs() <= uv_tolerance {
            Some(IsoKind::ConstantU(uv0.x))
        } else {
            None
        },
    )
}

/// `surface` naturally extended in v so that its domain contains `parameter`,
/// or `None` when it already does.
fn extend_to(surface: &NurbsSurface, parameter: f64) -> Result<Option<NurbsSurface>, ExtendRefusal> {
    let [v0, v1] = surface.domain_v().map_err(ExtendRefusal::Failed)?;
    let (side, delta) = if parameter < v0 {
        (crate::SurfaceSide::VMin, v0 - parameter)
    } else if parameter > v1 {
        (crate::SurfaceSide::VMax, parameter - v1)
    } else {
        return Ok(None);
    };
    // The solve bisects inside the extension, so its length only has to
    // clear the knot identity bar `extend_natural` refuses an increment under.
    surface
        .extend_natural(side, delta.max(2.0 * crate::curve::KNOT_IDENTITY_TOL))
        .map(Some)
}

/// A meridian boundary of a partial cylinder and the fixed plane across it.
struct Meridian<'a> {
    edge: &'a EdgeRecord,
    neighbour: u64,
    /// Azimuth of the meridian in the carrier's frame.
    azimuth: f64,
    plane_origin: Vec3,
    plane_normal: Vec3,
}

/// Push a PARTIAL CYLINDER exactly: a straight generatrix parallel to the axis,
/// swept through less than a full turn, bounded by two meridians and two rims,
/// with a planar neighbour across each.
///
/// The offset carrier is the coaxial cylinder of radius ρ′ = ρ ± d, and every
/// boundary is re-derived as that carrier's intersection with the fixed
/// neighbour's carrier:
/// * a rim against an axis-normal cap stays at its axial station;
/// * a meridian against a plane PARALLEL to the axis moves to where the new
///   circle crosses that plane, `ρ′·A·cos(θ − φ) = D` in the carrier frame. A
///   radial end face (D = 0) keeps its azimuth — the one case the general
///   lane's kept azimuth is right for.
///
/// Of the two crossings, the corner is the one from which the new arc leaves
/// into the side of the plane the old arc left into: the old arc's tangent side
/// at a transversal corner, the centre's side at a tangent one (a fillet band,
/// a sketch arc tangent to its segment), where "nearest crossing" is a tie.
///
/// `Ok(None)` means the face is not this lane's shape and goes to the general
/// lane; an `Err` is a push this lane recognised and refuses by name.
fn offset_partial_cylinder_face(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
    tolerance: f64,
    residual_tolerance: f64,
    scale: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    use std::f64::consts::TAU;
    let op = "offset_revolution_face";
    let (shell_index, face_index) = find_face(solid, face_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "partial_face_lookup",
            format!("{op}: no face {face_id}"),
        )
    })?;
    let face = &solid.shells[shell_index].faces[face_index];
    let Some(AnalyticSurface::Revolution {
        frame, generatrix, ..
    }) = face.surface.analytic()
    else {
        return Ok(None);
    };
    if generatrix.degree != 1 || generatrix.control_points.len() != 2 {
        return Ok(None);
    }
    let (origin, axis, x_axis, y_axis) = (frame.origin, frame.axis, frame.x_axis, frame.y_axis);
    // (axial coordinate, radial vector) of a point about the carrier axis.
    let cylindrical = |point: Vec3| {
        let relative = point.sub(origin);
        let axial = relative.dot(axis);
        (axial, relative.sub(axis.scale(axial)))
    };
    let azimuth_of = |radial: Vec3| radial.dot(y_axis).atan2(radial.dot(x_axis));
    let (generatrix_z0, generatrix_radial0) = cylindrical(generatrix.control_points[0].point().or_refuse(KernelStage::Classify, "point")?);
    let (generatrix_z1, generatrix_radial1) = cylindrical(generatrix.control_points[1].point().or_refuse(KernelStage::Classify, "point")?);
    if generatrix_radial0.sub(generatrix_radial1).length() > residual_tolerance {
        return Ok(None); // a cone
    }
    let radius = generatrix_radial0.length();
    let span = (generatrix_z1 - generatrix_z0).abs();

    if face.loops.len() != 1 || face.loops[0].coedges.len() != 4 {
        return Ok(None);
    }
    let mut faces_of_edge: HashMap<u64, Vec<u64>> = HashMap::default();
    for shell in &solid.shells {
        for candidate in &shell.faces {
            for loop_record in &candidate.loops {
                for coedge in &loop_record.coedges {
                    faces_of_edge
                        .entry(coedge.edge_id)
                        .or_default()
                        .push(candidate.id);
                }
            }
        }
    }
    let edge_by_id: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();

    // Classify the four boundary edges by their 3D geometry.
    let mut meridians: Vec<Meridian> = Vec::new();
    let mut rims: Vec<(&EdgeRecord, f64)> = Vec::new();
    let mut planar_neighbours: HashSet<u64> = HashSet::default();
    for coedge in &face.loops[0].coedges {
        let edge = *edge_by_id
            .get(&coedge.edge_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Classify,
                    "partial_missing_edge",
                    format!("{op}: missing edge {}", coedge.edge_id),
                )
            })?;
        let incident = faces_of_edge.get(&edge.id).cloned().unwrap_or_default();
        if incident.len() != 2 || incident.iter().filter(|f| **f == face_id).count() != 1 {
            return Ok(None);
        }
        let neighbour = *incident.iter().find(|f| **f != face_id).expect("two incidences");
        let (ns, nf) = find_face(solid, neighbour).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Classify,
                "partial_missing_neighbour",
                format!("{op}: missing neighbour {neighbour}"),
            )
        })?;
        let Ok(plane) =
            plane_of_surface(&solid.shells[ns].faces[nf].surface, residual_tolerance, op)
        else {
            return Ok(None);
        };
        let (z_start, radial_start) = cylindrical(edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Classify, "evaluate")?);
        let (z_mid, radial_mid) = cylindrical(edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).or_refuse(KernelStage::Classify, "evaluate")?);
        let (z_end, radial_end) = cylindrical(edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Classify, "evaluate")?);
        if radial_start.sub(radial_mid).length() <= residual_tolerance
            && radial_start.sub(radial_end).length() <= residual_tolerance
        {
            if plane.normal.dot(axis).abs() * span > residual_tolerance {
                return Ok(None); // a plane that is not parallel to the axis
            }
            meridians.push(Meridian {
                edge,
                neighbour,
                azimuth: azimuth_of(radial_start),
                plane_origin: plane.origin,
                plane_normal: plane.normal,
            });
        } else if (z_start - z_mid).abs() <= residual_tolerance
            && (z_start - z_end).abs() <= residual_tolerance
        {
            if plane.normal.cross(axis).length() * radius > residual_tolerance {
                return Ok(None); // a cap that is not normal to the axis
            }
            rims.push((edge, z_start));
        } else {
            return Ok(None);
        }
        planar_neighbours.insert(neighbour);
    }
    if meridians.len() != 2 || rims.len() != 2 || (rims[0].1 - rims[1].1).abs() <= residual_tolerance
    {
        return Ok(None);
    }
    // Every corner is a meridian end at one of the two rim stations, each rim
    // runs between two corners, and every other edge ending at a corner is a
    // straight edge between two of the planes this push re-trims — so the chord
    // the re-trim rebuilds stays on both.
    let mut stations: HashMap<u64, f64> = HashMap::default();
    for meridian in &meridians {
        for vertex_id in [meridian.edge.start_vertex_id, meridian.edge.end_vertex_id] {
            let vertex = solid
                .vertices
                .iter()
                .find(|vertex| vertex.id == vertex_id)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Classify,
                        "partial_missing_vertex",
                        format!("{op}: missing vertex {vertex_id}"),
                    )
                })?;
            let (axial, _) = cylindrical(vertex.point);
            let Some(station) = rims
                .iter()
                .map(|(_, station)| *station)
                .find(|station| (axial - station).abs() <= residual_tolerance)
            else {
                return Ok(None);
            };
            stations.insert(vertex_id, station);
        }
    }
    let boundary: HashSet<u64> =
        face.loops[0].coedges.iter().map(|coedge| coedge.edge_id).collect();
    for edge in &solid.edges {
        let at_corner = stations.contains_key(&edge.start_vertex_id)
            || stations.contains_key(&edge.end_vertex_id);
        if !at_corner {
            continue;
        }
        if boundary.contains(&edge.id) {
            if !(stations.contains_key(&edge.start_vertex_id)
                && stations.contains_key(&edge.end_vertex_id))
            {
                return Ok(None);
            }
            continue;
        }
        let incident = faces_of_edge.get(&edge.id).cloned().unwrap_or_default();
        if edge.degenerate
            || edge.curve.degree != 1
            || !incident.iter().all(|f| planar_neighbours.contains(f))
        {
            return Ok(None);
        }
    }

    // The pushed radius, signed along the face's own outward normal.
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let (u_mid, v_mid) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let offsets = OffsetEvaluator::new(
        "push_revolution",
        &face.surface,
        OffsetNormal::Face {
            same_sense: face.same_sense,
        },
    );
    let sample = offsets.at(u_mid, v_mid, distance).or_refuse(KernelStage::Refine, "offset_at")?;
    let outward = cylindrical(sample.source).1.normalized().or_refuse(KernelStage::Refine, "normalized")?;
    let new_radius = cylindrical(sample.point).1.dot(outward);
    if new_radius <= tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Refine,
            "radius_collapse",
            format!(
            "{op}: pushing partial cylinder face {face_id} by {distance} collapses its radius \
             {radius:.6} onto the axis — refusing"
        ),
        ));
    }

    // Which meridian starts the sweep: the counter-clockwise run from start to
    // end is the one that passes through a rim's midpoint.
    let ccw = |from: f64, to: f64| (to - from).rem_euclid(TAU);
    let rim_mid = rims[0].0.curve.evaluate(0.5 * (rims[0].0.t0 + rims[0].0.t1)).or_refuse(KernelStage::Refine, "evaluate")?;
    let rim_azimuth = azimuth_of(cylindrical(rim_mid).1);
    let (start, end) = if ccw(meridians[0].azimuth, rim_azimuth)
        < ccw(meridians[0].azimuth, meridians[1].azimuth)
    {
        (&meridians[0], &meridians[1])
    } else {
        (&meridians[1], &meridians[0])
    };

    // Solve one corner; `sense` is +1 where the arc leaves its corner
    // counter-clockwise (the start) and -1 where it leaves clockwise (the end).
    // Returns the new azimuth and the plane's two crossings of the new circle.
    let solve = |meridian: &Meridian, sense: f64| -> Result<(f64, [f64; 2]), KernelRefusal> {
        let normal = meridian.plane_normal;
        let (nx, ny) = (normal.dot(x_axis), normal.dot(y_axis));
        let reach = nx.hypot(ny);
        let phi = ny.atan2(nx);
        let offset = meridian.plane_origin.sub(origin).dot(normal);
        let lean = (phi - meridian.azimuth).sin();
        let side = if radius * reach * lean.abs() > residual_tolerance {
            sense * lean
        } else {
            -offset
        };
        let shortfall = offset.abs() - new_radius * reach;
        if shortfall > tolerance {
            return Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "partial_separated",
                format!(
                "{op}: pushing partial cylinder face {face_id} by {distance} separates it from \
                 neighbour face {} — the radius {new_radius:.6} carrier stops {shortfall:.3e} \
                 short of that plane (refusing)",
                meridian.neighbour
            ),
            ));
        }
        let half = (offset / (new_radius * reach)).clamp(-1.0, 1.0).acos();
        let azimuth = if sense * side > 0.0 { phi - half } else { phi + half };
        Ok((azimuth, [phi - half, phi + half]))
    };
    let (start_azimuth, start_crossings) = solve(start, 1.0)?;
    let (end_azimuth, end_crossings) = solve(end, -1.0)?;
    let new_sweep = ccw(start_azimuth, end_azimuth);
    let angular_tolerance = tolerance / new_radius;
    if new_sweep <= angular_tolerance || new_sweep >= TAU - angular_tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Refine,
            "arc_closed",
            format!(
            "{op}: pushing partial cylinder face {face_id} by {distance} closes its arc between \
             neighbour faces {} and {} — refusing",
            start.neighbour, end.neighbour
        ),
        ));
    }
    // A crossing strictly inside the new arc means the arc passes through a
    // neighbour's plane before reaching its own corner — a convex round grown
    // past the two walls it was tangent to has no corner on either. Both
    // corners of an arc cut from one sketch line solve against two faces whose
    // planes agree only to the sketch solver's precision, so a crossing within
    // the residual bar of a corner is that corner.
    let crossing_margin = residual_tolerance / new_radius;
    for (meridian, crossings) in [(start, start_crossings), (end, end_crossings)] {
        for crossing in crossings {
            let along = ccw(start_azimuth, crossing);
            if along > crossing_margin && along < new_sweep - crossing_margin {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Refine,
                    "arc_through_neighbour",
                    format!(
                    "{op}: pushing partial cylinder face {face_id} by {distance} carries its arc \
                     through the plane of neighbour face {} — refusing",
                    meridian.neighbour
                ),
                ));
            }
        }
    }

    // New corner vertices, meridians and rims.
    let direction = |azimuth: f64| x_axis.scale(azimuth.cos()).add(y_axis.scale(azimuth.sin()));
    let mut new_vertices: HashMap<u64, Vec3> = HashMap::default();
    let mut new_curves: HashMap<u64, NurbsCurve> = HashMap::default();
    for (meridian, azimuth) in [(start, start_azimuth), (end, end_azimuth)] {
        let edge = meridian.edge;
        let mut ends = [Vec3::default(); 2];
        for (slot, vertex_id) in [edge.start_vertex_id, edge.end_vertex_id].into_iter().enumerate()
        {
            ends[slot] = origin
                .add(axis.scale(stations[&vertex_id]))
                .add(direction(azimuth).scale(new_radius));
            new_vertices.insert(vertex_id, ends[slot]);
        }
        new_curves.insert(edge.id, make_line(ends[0], ends[1]).or_refuse(KernelStage::Fragment, "make_line")?);
    }
    let new_x = direction(start_azimuth);
    let new_y = axis.cross(new_x);
    for (edge, station) in &rims {
        let (head, tail) = (new_vertices[&edge.start_vertex_id], new_vertices[&edge.end_vertex_id]);
        let mut arc = crate::make_arc(
            origin.add(axis.scale(*station)),
            new_x,
            new_y,
            new_radius,
            0.0,
            new_sweep,
        ).or_refuse(KernelStage::Fragment, "make_arc")?;
        let [a0, _] = arc.domain().or_refuse(KernelStage::Fragment, "domain")?;
        let first = arc.evaluate(a0).or_refuse(KernelStage::Fragment, "evaluate")?;
        if first.sub(head).length() > first.sub(tail).length() {
            arc = arc.reversed().or_refuse(KernelStage::Fragment, "reversed")?;
        }
        new_curves.insert(edge.id, arc);
    }

    // The straight neighbour edges ending at a moved corner must still run the
    // way they ran; a corner carried past an edge's far end folds that face.
    for edge in &solid.edges {
        if new_curves.contains_key(&edge.id)
            || !(new_vertices.contains_key(&edge.start_vertex_id)
                || new_vertices.contains_key(&edge.end_vertex_id))
        {
            continue;
        }
        let position = |vertex_id: u64| {
            solid
                .vertices
                .iter()
                .find(|vertex| vertex.id == vertex_id)
                .map(|vertex| vertex.point)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Fragment,
                        "corner_vertex_pos",
                        format!("{op}: missing vertex {vertex_id}"),
                    )
                })
        };
        let old_run = position(edge.end_vertex_id)?.sub(position(edge.start_vertex_id)?);
        let new_start = match new_vertices.get(&edge.start_vertex_id) {
            Some(point) => *point,
            None => position(edge.start_vertex_id)?,
        };
        let new_end = match new_vertices.get(&edge.end_vertex_id) {
            Some(point) => *point,
            None => position(edge.end_vertex_id)?,
        };
        let new_run = new_end.sub(new_start);
        if new_run.length() <= tolerance || new_run.dot(old_run) <= 0.0 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Fragment,
                "corner_past_edge",
                format!(
                "{op}: pushing partial cylinder face {face_id} by {distance} runs a corner past \
                 the far end of edge {} — refusing",
                edge.id
            ),
            ));
        }
    }

    // The offset carrier: the same axis, the new radius, the new sweep, with
    // the generatrix kept in its own direction so the normal keeps its sense.
    let carrier = make_revolution(
        origin,
        axis,
        &make_line(
            origin
                .add(axis.scale(generatrix_z0))
                .add(new_x.scale(new_radius)),
            origin
                .add(axis.scale(generatrix_z1))
                .add(new_x.scale(new_radius)),
        ).or_refuse(KernelStage::Fragment, "make_line")?,
        new_sweep,
    ).or_refuse(KernelStage::Fragment, "make_revolution")?;
    let [c0, c1] = carrier.domain_u().or_refuse(KernelStage::Fragment, "domain_u")?;
    let [w0, w1] = carrier.domain_v().or_refuse(KernelStage::Fragment, "domain_v")?;
    let old_normal = face.surface.normal(u_mid, v_mid).or_refuse(KernelStage::Fragment, "normal")?;
    let old_point = face.surface.evaluate(u_mid, v_mid).or_refuse(KernelStage::Fragment, "evaluate")?;
    let new_normal = carrier.normal(0.5 * (c0 + c1), 0.5 * (w0 + w1)).or_refuse(KernelStage::Fragment, "normal")?;
    let new_point = carrier.evaluate(0.5 * (c0 + c1), 0.5 * (w0 + w1)).or_refuse(KernelStage::Fragment, "evaluate")?;
    if old_normal.dot(cylindrical(old_point).1) * new_normal.dot(cylindrical(new_point).1) <= 0.0 {
        return Err(KernelRefusal::internal(
            KernelStage::Refine,
            "carrier_normal",
            format!(
            "{op}: the offset carrier of partial cylinder face {face_id} flipped its normal — \
             refusing"
        ),
        ));
    }

    let mut result = solid.clone();
    let pushed = &mut result.shells[shell_index].faces[face_index];
    for coedge in &mut pushed.loops[0].coedges {
        let mut pcurve = build_pcurve_on_surface(&carrier, &new_curves[&coedge.edge_id]).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?;
        if !coedge.forward {
            pcurve = pcurve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
        }
        coedge.pcurve = pcurve;
    }
    pushed.surface = carrier;
    retrim_planar_neighbours(
        solid,
        result,
        &edge_by_id,
        &new_curves,
        &new_vertices,
        planar_neighbours,
        residual_tolerance,
        scale,
    )
    .map(Some)
}

/// Land a push's rebuilt boundary on `result` (whose pushed face already
/// carries its new surface and pcurves): swap in the new edge curves and vertex
/// positions, rebuild every straight edge of a planar neighbour that ends at a
/// moved vertex, re-trim those neighbours, and accept the solid only if it
/// validates and keeps its orientation.
#[allow(clippy::too_many_arguments)]
fn retrim_planar_neighbours(
    solid: &BrepSolid,
    mut result: BrepSolid,
    edge_by_id: &HashMap<u64, &EdgeRecord>,
    new_curves: &HashMap<u64, NurbsCurve>,
    new_vertices: &HashMap<u64, Vec3>,
    planar_neighbours: HashSet<u64>,
    residual_tolerance: f64,
    scale: f64,
) -> Result<BrepSolid, KernelRefusal> {
    for edge in &mut result.edges {
        if let Some(curve) = new_curves.get(&edge.id) {
            let [d0, d1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            edge.curve = curve.clone();
            edge.t0 = d0;
            edge.t1 = d1;
        }
    }
    for vertex in &mut result.vertices {
        if let Some(point) = new_vertices.get(&vertex.id) {
            vertex.point = *point;
        }
    }
    // A full-revolve planar disk/annulus may carry its own radial seam from an
    // axis vertex to the shared circular rim. Moving the rim relocates one end
    // of that seam, so rebuild the straight edge before re-trimming the cap.
    let result_vertices: HashMap<u64, Vec3> = result
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    let mut cap_seams = HashSet::default();
    for neighbour in &planar_neighbours {
        let (ns, nf) = find_face(&result, *neighbour).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "cap_lookup",
                format!("offset_revolution_face: missing cap {neighbour}"),
            )
        })?;
        for loop_record in &result.shells[ns].faces[nf].loops {
            for coedge in &loop_record.coedges {
                let edge = *edge_by_id.get(&coedge.edge_id).ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Sew,
                        "missing_cap_edge",
                        format!(
                        "offset_revolution_face: missing cap edge {}",
                        coedge.edge_id
                    ),
                    )
                })?;
                let touches_moved = new_vertices.contains_key(&edge.start_vertex_id)
                    || new_vertices.contains_key(&edge.end_vertex_id);
                if touches_moved
                    && !new_curves.contains_key(&edge.id)
                    && !edge.degenerate
                    && edge.curve.degree == 1
                {
                    cap_seams.insert(edge.id);
                }
            }
        }
    }
    for seam_id in cap_seams {
        let source = edge_by_id[&seam_id];
        let curve = make_line(
            result_vertices[&source.start_vertex_id],
            result_vertices[&source.end_vertex_id],
        ).or_refuse(KernelStage::Sew, "make_line")?;
        let [d0, d1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
        if let Some(edge) = result.edges.iter_mut().find(|edge| edge.id == seam_id) {
            edge.curve = curve;
            edge.t0 = d0;
            edge.t1 = d1;
        }
    }
    let final_edges: HashMap<u64, EdgeRecord> = result
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    for neighbour in planar_neighbours {
        let (ns, nf) = find_face(&result, neighbour).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "retrim_cap",
                format!("offset_revolution_face: missing cap {neighbour}"),
            )
        })?;
        let plane = plane_of_surface(
            &result.shells[ns].faces[nf].surface,
            residual_tolerance,
            "offset_revolution_face",
        )?;
        retrim_planar_face(
            &mut result.shells[ns].faces[nf],
            &plane,
            &final_edges,
            scale,
            "offset_revolution_face",
        )
        .map_err(|error| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "cap_retrim",
                format!("offset_revolution_face: cap retrim: {error}"),
            )
        })?;
    }
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!(
            "offset_revolution_face: pushed solid failed validation: {issues:?}"
        ),
        ));
    }
    if let (Ok(before), Ok(after)) = (solid_signed_volume(solid), solid_signed_volume(&result)) {
        if before * after <= 0.0 {
            return Err(KernelRefusal::internal(
                KernelStage::Validate,
                INVERTED,
                "offset_revolution_face: the push inverts the solid — refusing",
            ));
        }
    }
    Ok(result)
}

/// `BREP_OFFSET_REVOLUTION_TRACE=<file>` appends one record per general-lane
/// push to `<file>`: the call, each rebuilt boundary edge against the fixed
/// planar neighbour it must lie in — before the push (the source edge) and
/// after (the rebuilt curve) — and the outcome with the volume. It writes to a
/// file, not stderr, because the case gate replays each document in a child
/// process whose stderr it discards.
struct BoundaryTrace {
    path: Option<std::path::PathBuf>,
}

impl BoundaryTrace {
    fn open(solid: &BrepSolid, face_id: u64, distance: f64) -> Self {
        let path = std::env::var_os("BREP_OFFSET_REVOLUTION_TRACE")
            .filter(|value| !value.is_empty() && value != "0")
            .map(std::path::PathBuf::from);
        let trace = Self { path };
        if trace.path.is_some() {
            let volume = solid_signed_volume(solid).map_or(f64::NAN, f64::abs);
            trace.write(format!(
                "call\tface={face_id}\tdistance={distance}\tscale={}\tvolume_before={volume:.15}",
                solid_model_scale(solid)
            ));
        }
        trace
    }

    /// One whole line per `write` call on an append-mode file, prefixed with
    /// the thread's name, so parallel test threads (named after their tests)
    /// neither interleave nor lose which test a line belongs to.
    fn write(&self, line: String) {
        use std::io::Write;
        let Some(path) = &self.path else { return };
        let thread = std::thread::current();
        let record = format!("{}\t{line}\n", thread.name().unwrap_or("?"));
        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = file.write_all(record.as_bytes());
        }
    }

    /// Largest distance from `plane` over `samples + 1` points of `curve`
    /// between `t0` and `t1`, and over its two end points.
    fn off_plane(
        curve: &NurbsCurve,
        t0: f64,
        t1: f64,
        plane: &crate::offset_retrim::Plane,
    ) -> (f64, f64) {
        let distance = |t: f64| {
            curve
                .evaluate(t)
                .map_or(f64::NAN, |point| point.sub(plane.origin).dot(plane.normal).abs())
        };
        let ends = distance(t0).max(distance(t1));
        let samples = (0..=32)
            .map(|index| distance(t0 + (t1 - t0) * index as f64 / 32.0))
            .fold(0.0f64, f64::max);
        (ends, samples)
    }

    fn edge(
        &self,
        kind: &str,
        edge: &EdgeRecord,
        neighbour: Option<&u64>,
        plane: Option<&crate::offset_retrim::Plane>,
        rebuilt: &NurbsCurve,
    ) {
        if self.path.is_none() {
            return;
        }
        let Some(plane) = plane else {
            self.write(format!("edge\t{}\t{kind}\tneighbour=none", edge.id));
            return;
        };
        let (before_ends, before_samples) = Self::off_plane(&edge.curve, edge.t0, edge.t1, plane);
        let (after_ends, after_samples) = match rebuilt.domain() {
            Ok([r0, r1]) => Self::off_plane(rebuilt, r0, r1, plane),
            Err(_) => (f64::NAN, f64::NAN),
        };
        self.write(format!(
            "edge\t{}\t{kind}\tneighbour={}\tbefore_vertices={before_ends:.3e}\t\
             before_samples={before_samples:.3e}\tafter_vertices={after_ends:.3e}\t\
             after_samples={after_samples:.3e}",
            edge.id,
            neighbour.map_or("none".to_string(), u64::to_string)
        ));
    }

    fn outcome(&self, outcome: &Result<BrepSolid, KernelRefusal>) {
        if self.path.is_none() {
            return;
        }
        match outcome {
            Ok(result) => {
                let volume = solid_signed_volume(result).map_or(f64::NAN, f64::abs);
                self.write(format!("ok\tvolume_after={volume:.15}"));
            }
            Err(error) => self.write(format!("refused\t{error}")),
        }
    }
}

