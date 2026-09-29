use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// Offset a free-form NURBS face whose boundaries have no other incident face.
/// The fitted offset must pass a dense same-parameter residual check. Intrinsic
/// iso boundaries use exact iso-curves; other self-incident trims use
/// [`crate::image_curve`]. Rebuilt topology and volume orientation are validated.
/// Neighbour-trimmed faces require general intersection and are refused.
pub fn offset_freeform_face(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    if !distance.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "distance",
            "offset_freeform_face: distance must be finite",
        ));
    }
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    // Free-form offsets are approximation-bounded, not exact. Keep the maximum
    // positional drift below 0.05% of model scale and reject rougher fits.
    let residual_tolerance = (scale * 5e-4).max(5e-6);
    // Curve-fit accuracy is separate from the surface and identity tolerances.
    let fit_tolerance = crate::KernelTolerances::for_scale(scale, 1e-7).intersection_fit;
    let (shell_index, face_index) = find_face(solid, face_id)
        .ok_or_else(|| {
            KernelRefusal::input(
                KernelStage::Collect,
                "face_id",
                format!("offset_freeform_face: no face {face_id}"),
            )
        })?;
    let face = &solid.shells[shell_index].faces[face_index];
    if face.surface.analytic().is_some() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "analytic_carrier",
            "offset_freeform_face: the pushed carrier is analytic",
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
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            let incident = faces_of_edge
                .get(&coedge.edge_id)
                .cloned()
                .unwrap_or_default();
            if incident.iter().any(|candidate| *candidate != face_id) {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "neighbour_trimmed",
                    "offset_freeform_face: neighbour-trimmed NURBS faces require general \
                            surface intersection and are deferred — refusing",
                ));
            }
        }
    }

    // A push past the face's concave curvature radius folds its offset through
    // the evolute however well that offset is FITTED, so the fit's own residual
    // gate below cannot be what refuses it: it used to, only while the fit was
    // coarse. This lane cannot carve, so ANY fold inside the face's own trim
    // refuses — the trim-scoped scan the shell's collapse probe and `thicken`'s
    // sheet gate read, at thicken's fold factor. `offset_surface(face, -d)` moves
    // along the face's outward normal, which is `same_sense · (Su × Sv)`.
    let displacement = if face.same_sense { distance } else { -distance };
    let region = crate::offset_regularity::TrimRegion::from_face(face)?;
    let scan = crate::offset_regularity::scan_offset_regularity(
        &face.surface,
        &region,
        &[displacement],
        crate::thicken::FOLD_FACTOR,
        crate::offset_regularity::ScanBudget::SHEET,
    )?;
    if let Some(worst) = scan.worst.filter(|worst| worst.factor <= crate::thicken::FOLD_FACTOR) {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "fold",
            format!(
            "offset_freeform_face: a push by {distance} folds the offset through the face's evolute \
             — inside its trim the curvature radius {:.6} at (u={:.6}, v={:.6}) is not larger \
             than the push (fold factor {:.3e}), and a push cannot carve the fold away",
            worst.radius(),
            worst.u,
            worst.v,
            worst.factor
        ),
        ));
    }
    let offset = crate::offset_surface(face, -distance, 0.0)?;
    // The residual gate asks the same question the evaluator answers: is the
    // FITTED surface still at `distance` along the pointwise offset's normal?
    let offsets = OffsetEvaluator::new(
        "push_freeform",
        &face.surface,
        OffsetNormal::Face {
            same_sense: face.same_sense,
        },
    );
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let target = distance.abs();
    for iu in 0..15 {
        for iv in 0..15 {
            let u = u0 + (u1 - u0) * (iu as f64 + 0.31) / 15.0;
            let v = v0 + (v1 - v0) * (iv as f64 + 0.43) / 15.0;
            let source_point = face.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
            let offset_point = offset.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
            let delta = offset_point.sub(source_point);
            let normal = offsets.normal(u, v).or_refuse(KernelStage::Refine, "normal")?;
            let distance_error = (delta.length() - target).abs();
            let normal_error = if delta.length() > tolerance {
                1.0 - delta.normalized().or_refuse(KernelStage::Refine, "normalized")?.dot(normal).abs()
            } else {
                0.0
            };
            if distance_error > residual_tolerance || normal_error > 2e-3 {
                return Err(KernelRefusal::non_convergence(
                    KernelStage::Refine,
                    "offset_fit",
                    format!(
                    "offset_freeform_face: offset fit exceeds tolerance \
                     (distance error {distance_error:.3e}, normal error {normal_error:.3e})"
                ),
                ));
            }
        }
    }

    let edge_by_id: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut rebuilt_edges: HashMap<u64, (NurbsCurve, f64, f64)> = HashMap::default();
    let mut rebuilt_vertices: HashMap<u64, Vec3> = HashMap::default();
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            if rebuilt_edges.contains_key(&coedge.edge_id) {
                continue;
            }
            let edge = *edge_by_id
                .get(&coedge.edge_id)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Fragment,
                        "missing_edge",
                        format!("offset_freeform_face: missing edge {}", coedge.edge_id),
                    )
                })?;
            let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
            let a = coedge.pcurve.evaluate(q0).or_refuse(KernelStage::Fragment, "evaluate")?;
            let b = coedge.pcurve.evaluate(q1).or_refuse(KernelStage::Fragment, "evaluate")?;
            let m = coedge.pcurve.evaluate(0.5 * (q0 + q1)).or_refuse(KernelStage::Fragment, "evaluate")?;
            let uv_tolerance = 1e-7;
            // The two iso lanes are untouched: an intrinsic seam or pole row is
            // an exact iso of the same-basis offset, and staying on that lane
            // keeps every solid this path already builds BIT-IDENTICAL.
            let iso = if (a.x - b.x).abs() <= uv_tolerance && (a.x - m.x).abs() <= uv_tolerance {
                Some((offset.iso_curve_u(a.x).or_refuse(KernelStage::Fragment, "iso_curve_u")?, a.y, b.y))
            } else if (a.y - b.y).abs() <= uv_tolerance && (a.y - m.y).abs() <= uv_tolerance {
                Some((offset.iso_curve_v(a.y).or_refuse(KernelStage::Fragment, "iso_curve_v")?, a.x, b.x))
            } else {
                None
            };
            // A general intrinsic trim used to be refused here. It is the image
            // of its own pcurve on the offset carrier — exactly, because this
            // path admits only edges with no OTHER incident face, so the
            // boundary is definitional rather than an intersection with a fixed
            // neighbour. `image_curve` returns `(t0, t1)` under the validator's
            // fraction contract, which the `forward` / `start <= end` handling
            // below consumes unchanged.
            let (base, mut start, mut end) = match iso {
                Some(triple) => triple,
                None => {
                    let image = crate::image_curve(
                        &offset,
                        &coedge.pcurve,
                        fit_tolerance,
                        "offset_freeform_face",
                    ).or_refuse(KernelStage::Fragment, "image_curve")?;
                    (image.curve, image.t0, image.t1)
                }
            };
            if !coedge.forward {
                std::mem::swap(&mut start, &mut end);
            }
            let [d0, d1] = base.domain().or_refuse(KernelStage::Fragment, "domain")?;
            let (curve, t0, t1) = if start <= end {
                (base, start, end)
            } else {
                (base.reversed().or_refuse(KernelStage::Fragment, "reversed")?, d0 + d1 - start, d0 + d1 - end)
            };
            let start_point = curve.evaluate(t0).or_refuse(KernelStage::Fragment, "evaluate")?;
            let end_point = curve.evaluate(t1).or_refuse(KernelStage::Fragment, "evaluate")?;
            for (vertex_id, point) in [
                (edge.start_vertex_id, start_point),
                (edge.end_vertex_id, end_point),
            ] {
                if let Some(known) = rebuilt_vertices.get(&vertex_id) {
                    if known.sub(point).length() > residual_tolerance {
                        return Err(KernelRefusal::internal(
                            KernelStage::Fragment,
                            "vertex_agreement",
                            format!(
                            "offset_freeform_face: intrinsic boundaries disagree at vertex {vertex_id}"
                        ),
                        ));
                    }
                } else {
                    rebuilt_vertices.insert(vertex_id, point);
                }
            }
            rebuilt_edges.insert(edge.id, (curve, t0, t1));
        }
    }

    finish_intrinsic_face_offset(
        solid,
        (shell_index, face_index),
        offset,
        &rebuilt_edges,
        &rebuilt_vertices,
        "offset_freeform_face",
    )
    
}

