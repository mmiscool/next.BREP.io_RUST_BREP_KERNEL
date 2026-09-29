use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// Push a toroidal face by changing its tube radius. Boundary edges must be
/// intrinsic to this face; neighbour-trimmed rims require re-intersection and
/// are refused. The carrier retains its knot vectors, weights, and UV basis,
/// keeping pcurves valid. Iso boundaries rebuild exactly; other self-incident
/// trims use [`crate::image_curve`].
pub fn offset_torus_face(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    if !distance.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "distance",
            "offset_torus_face: distance must be finite",
        ));
    }
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    // Curve-fit accuracy is separate from the surface and identity tolerances.
    let fit_tolerance = crate::KernelTolerances::for_scale(scale, 1e-7).intersection_fit;
    let (shell_index, face_index) =
        find_face(solid, face_id).ok_or_else(|| {
            KernelRefusal::input(
                KernelStage::Collect,
                "face_id",
                format!("offset_torus_face: no face {face_id}"),
            )
        })?;
    let face = &solid.shells[shell_index].faces[face_index];
    let Some(AnalyticSurface::Torus {
        frame,
        major_radius,
        minor_radius,
    }) = face.surface.analytic()
    else {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "not_torus",
            "offset_torus_face: the pushed face is not a torus",
        ));
    };

    // Positive push follows the face's outward normal. For a cavity torus the
    // topological normal opposes the tube radial, so positive distance shrinks
    // the carrier instead of growing it.
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Classify, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let point = face.surface.evaluate(um, vm).or_refuse(KernelStage::Classify, "evaluate")?;
    let mut normal = face.surface.normal(um, vm).or_refuse(KernelStage::Classify, "normal")?;
    if !face.same_sense {
        normal = normal.scale(-1.0);
    }
    let relative = point.sub(frame.origin);
    let axial = relative.dot(frame.axis);
    let radial_direction = relative.sub(frame.axis.scale(axial)).normalized().or_refuse(KernelStage::Classify, "normalized")?;
    let tube_center = frame
        .origin
        .add(frame.axis.scale(axial))
        .add(radial_direction.scale(*major_radius));
    let tube_radial = point.sub(tube_center);
    let outward_sign = if normal.dot(tube_radial) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let minor_new = *minor_radius + distance * outward_sign;
    if minor_new <= tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "tube_collapse",
            "offset_torus_face: the push collapses the tube radius — refusing",
        ));
    }
    if minor_new >= *major_radius - tolerance {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "spindle_torus",
            "offset_torus_face: the push creates a horn/spindle torus — refusing",
        ));
    }

    // A trimmed rim has another incident face. Keep that harder neighbour-heal
    // slice out of this exact full-torus path.
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
                    "offset_torus_face: a neighbour-trimmed torus is deferred; only the \
                            full torus is supported in this slice",
                ));
            }
        }
    }

    // offset_surface uses the shell convention positive=opposite outward, so
    // negate the direct-edit distance. For an analytic torus the Greville fit
    // reproduces the exact rational torus with unchanged parameterization.
    let offset = crate::offset_surface(face, -distance, 0.0)?;
    match offset.analytic() {
        Some(AnalyticSurface::Torus {
            frame: out_frame,
            major_radius: out_major,
            minor_radius: out_minor,
        }) if out_frame.origin.sub(frame.origin).length() <= tolerance
            && out_frame.axis.dot(frame.axis).abs() >= 1.0 - 1e-9
            && (*out_major - *major_radius).abs() <= tolerance
            && (*out_minor - minor_new).abs() <= tolerance => {}
        other => {
            return Err(KernelRefusal::internal(
                KernelStage::Refine,
                "offset_carrier",
                format!(
                "offset_torus_face: offset carrier was not the expected exact torus: {other:?}"
            ),
            ))
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
                        format!("offset_torus_face: missing edge {}", coedge.edge_id),
                    )
                })?;
            let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
            let a = coedge.pcurve.evaluate(q0).or_refuse(KernelStage::Fragment, "evaluate")?;
            let b = coedge.pcurve.evaluate(q1).or_refuse(KernelStage::Fragment, "evaluate")?;
            let m = coedge.pcurve.evaluate(0.5 * (q0 + q1)).or_refuse(KernelStage::Fragment, "evaluate")?;
            let uv_tolerance = 1e-8;
            // The two iso lanes are untouched: a periodic seam is an exact iso
            // of the same-basis offset torus, so every solid this path already
            // builds stays BIT-IDENTICAL.
            let iso = if (a.x - b.x).abs() <= uv_tolerance && (a.x - m.x).abs() <= uv_tolerance {
                Some((offset.iso_curve_u(a.x).or_refuse(KernelStage::Fragment, "iso_curve_u")?, a.y, b.y))
            } else if (a.y - b.y).abs() <= uv_tolerance && (a.y - m.y).abs() <= uv_tolerance {
                Some((offset.iso_curve_v(a.y).or_refuse(KernelStage::Fragment, "iso_curve_v")?, a.x, b.x))
            } else {
                None
            };
            // A general intrinsic trim used to be refused here. This path
            // admits only edges with no OTHER incident face, so the boundary is
            // definitional and its image on the offset torus IS the new edge.
            let (base, mut start, mut end) = match iso {
                Some(triple) => triple,
                None => {
                    let image = crate::image_curve(
                        &offset,
                        &coedge.pcurve,
                        fit_tolerance,
                        "offset_torus_face",
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
            for (vertex_id, vertex_point) in [
                (edge.start_vertex_id, start_point),
                (edge.end_vertex_id, end_point),
            ] {
                if let Some(known) = rebuilt_vertices.get(&vertex_id) {
                    if known.sub(vertex_point).length() > tolerance {
                        return Err(KernelRefusal::internal(
                            KernelStage::Fragment,
                            "seam_agreement",
                            format!(
                            "offset_torus_face: periodic seams disagree at vertex {vertex_id}"
                        ),
                        ));
                    }
                } else {
                    rebuilt_vertices.insert(vertex_id, vertex_point);
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
        "offset_torus_face",
    )
    
}

