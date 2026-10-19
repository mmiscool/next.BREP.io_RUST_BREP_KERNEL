//! Rebuild an outward offset with shared joints when its topology survives.
//!
//! Padding a trimmed carrier while cloning its UV wire stretches tangent
//! boundaries: a bore-mouth arc embedded in an outer planar loop scales with
//! that loop, and a partly mitered cylinder rail slides along its ruling.
//! Independently offsetting the two uses then leaves different images of the
//! same joint. Tangent SSI cannot recover the missing boundary.
//!
//! This alternative to the failed imprint construction keeps the source's
//! incidence graph, solves each vertex against all incident offset supports,
//! and constructs each edge once against both supports. Both faces then get
//! trims from that same edge. Held opening planes participate at distance zero.
//! Folded supports, excessive joint motion, unmet section fits, invalid wires
//! and self-intersections decline this construction; the caller keeps the
//! original refusal. No topology-changing offset is inferred from this graph.
use super::*;
use crate::{KernelRefusal, NurbsSurface, OrRefuse};

fn meet(
    mut point: Vec3,
    surfaces: &[&NurbsSurface],
    tolerance: f64,
    reach: f64,
) -> Result<Vec3, KernelRefusal> {
    let anchor = point;
    // A local foot is sufficient during this bounded intersection solve. The
    // finished sections are independently checked with global projections.
    let mut seeds = vec![None; surfaces.len()];
    for _ in 0..64 {
        let mut equations = Vec::new();
        let mut worst = 0.0f64;
        for (index, surface) in surfaces.iter().enumerate() {
            let foot = match seeds[index] {
                Some((u, v)) => crate::project_point_to_surface_seeded(surface, point, u, v),
                None => project_point_to_surface(surface, point),
            }
            .or_refuse(KernelStage::Refine, "joint_offset_projection")?;
            seeds[index] = Some((foot.u, foot.v));
            let normal = surface
                .normal(foot.u, foot.v)
                .or_refuse(KernelStage::Refine, "joint_offset_normal")?;
            // A support equation measures normal departure. Tangential
            // departure from a finite plane is handled by the rebuilt trim.
            let residual = point.sub(foot.point).dot(normal);
            worst = worst.max(residual.abs());
            equations.push((normal, residual));
        }
        if worst <= tolerance {
            return Ok(point);
        }
        let mut rows: Vec<(Vec3, f64)> = Vec::new();
        for (normal, residual) in equations {
            let mut direction = normal;
            let mut rhs = -residual;
            for (row, value) in &rows {
                let component = normal.dot(*row);
                direction = direction.sub(row.scale(component));
                rhs -= component * value;
            }
            let length = direction.length();
            if length > 1e-6 {
                rows.push((direction.scale(1.0 / length), rhs / length));
            }
        }
        let step = rows.iter().fold(Vec3::default(), |sum, (row, value)| {
            sum.add(row.scale(*value))
        });
        point = point.add(step);
        if point.sub(anchor).length() > reach {
            break;
        }
    }
    Err(KernelRefusal::unsupported(
        KernelStage::Refine,
        "offset_shell_joint",
        format!(
            "offset joints do not preserve the source incidence graph: {anchor:?} -> {point:?}"
        ),
    ))
}

pub(super) fn build(
    source: &BrepSolid,
    openings: &[u64],
    distance: f64,
) -> Result<OffsetShellResultRecord, KernelRefusal> {
    if distance >= 0.0 || !distance.is_finite() || openings.is_empty() {
        return Err(KernelRefusal::unsupported(
            KernelStage::Refine,
            "offset_shell_joint",
            "offset joint reconstruction requires an outward offset and planar openings",
        ));
    }
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = scale * 1e-9;
    let fit_tolerance = crate::KernelTolerances::for_solid(source, 1e-7).intersection_fit;
    let reach = distance.abs() * 8.0;
    let faces = source
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .collect::<Vec<_>>();
    if faces.iter().all(|face| openings.contains(&face.id)) {
        return Err(KernelRefusal::unsupported(
            KernelStage::Refine,
            "offset_shell_joint",
            "offset joint reconstruction needs a retained face",
        ));
    }
    // This construction repairs the two images of a curved tangent join.
    // Sharp-only arrangements also own topology-changing slot closures and
    // their numerical bands; leave those with the imprint construction.
    let mut curved_tangent_join = false;
    for face in faces.iter().filter(|face| !openings.contains(&face.id)) {
        for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
            let Some((mate, mate_use)) = junction_mate(&faces, face, coedge.edge_id) else {
                continue;
            };
            if openings.contains(&mate.id) {
                continue;
            }
            if (!face
                .surface
                .is_affine()
                .or_refuse(KernelStage::Refine, "joint_offset_support")?
                || !mate
                    .surface
                    .is_affine()
                    .or_refuse(KernelStage::Refine, "joint_offset_support")?)
                && uses_are_tangent(face, coedge, mate, mate_use)?
            {
                curved_tangent_join = true;
                break;
            }
        }
        if curved_tangent_join {
            break;
        }
    }
    if !curved_tangent_join {
        return Err(KernelRefusal::unsupported(
            KernelStage::Refine,
            "offset_shell_joint",
            "offset joint reconstruction requires a curved tangent join",
        ));
    }
    let mut body = source.clone();
    body.mass_properties_cache = Default::default();
    body.cells = Default::default();
    let mut surfaces = Vec::new();
    let mut images = Vec::new();
    for face in &faces {
        let held = openings.contains(&face.id);
        if (held
            && !face
                .surface
                .is_affine()
                .or_refuse(KernelStage::Refine, "joint_offset_opening")?)
            || (!held
                && (offset_support_collapsed(face, distance)?
                    || carve_folded_support(source, face, distance)?.is_some()))
        {
            return Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "offset_shell_joint",
                "offset joint reconstruction requires regular supports and planar openings",
            ));
        }
        let surface = crate::offset_surface(
            face,
            if held { 0.0 } else { distance },
            distance.abs() * 2.0,
        )?;
        surfaces.push(surface);
        images.push(OffsetShellFaceImageRecord {
            role: if held {
                OffsetFaceRole::Wall
            } else {
                OffsetFaceRole::Offset
            },
            source_face_id: face.id,
        });
    }
    let mut incidence = HashMap::<u64, Vec<usize>>::default();
    for (index, face) in faces.iter().enumerate() {
        for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
            let uses = incidence.entry(coedge.edge_id).or_default();
            if !uses.contains(&index) {
                uses.push(index);
            }
        }
    }
    let mut vertex_faces = HashMap::<u64, Vec<usize>>::default();
    for edge in &source.edges {
        for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
            let uses = vertex_faces.entry(vertex).or_default();
            for index in &incidence[&edge.id] {
                if !uses.contains(index) {
                    uses.push(*index);
                }
            }
        }
    }
    for vertex in &mut body.vertices {
        os_debug!("offset joint vertex {}", vertex.id);
        let adjacent = vertex_faces[&vertex.id]
            .iter()
            .map(|index| &surfaces[*index])
            .collect::<Vec<_>>();
        vertex.point = meet(vertex.point, &adjacent, tolerance, reach).map_err(|error| {
            error.with_message(|message| format!("vertex {} {message}", vertex.id))
        })?;
    }
    let points = body
        .vertices
        .iter()
        .map(|v| (v.id, v.point))
        .collect::<HashMap<_, _>>();
    for edge in &mut body.edges {
        os_debug!("offset joint edge {}", edge.id);
        let original = source.edges.iter().find(|e| e.id == edge.id).unwrap();
        let adjacent = incidence[&edge.id]
            .iter()
            .map(|index| &surfaces[*index])
            .collect::<Vec<_>>();
        let start = points[&edge.start_vertex_id];
        let end = points[&edge.end_vertex_id];
        let old_start = original
            .curve
            .evaluate(original.t0)
            .or_refuse(KernelStage::Refine, "joint_offset_edge")?;
        let old_end = original
            .curve
            .evaluate(original.t1)
            .or_refuse(KernelStage::Refine, "joint_offset_edge")?;
        let mut count = 128;
        loop {
            let mut samples = Vec::new();
            for station in 0..=count {
                let fraction = station as f64 / count as f64;
                let old = original
                    .curve
                    .evaluate(original.t0 + (original.t1 - original.t0) * fraction)
                    .or_refuse(KernelStage::Refine, "joint_offset_edge")?;
                let point = if station == 0 {
                    start
                } else if station == count {
                    end
                } else {
                    let guess = old
                        .add(start.sub(old_start).scale(1.0 - fraction))
                        .add(end.sub(old_end).scale(fraction));
                    meet(guess, &adjacent, tolerance, reach).map_err(|error| {
                        error.with_message(|message| {
                            format!("edge {} station {station} {message}", edge.id)
                        })
                    })?
                };
                samples.push(point);
            }
            // Preserve straight sections exactly, including their new end stations.
            let chord = end.sub(start);
            let straight = chord.length() > tolerance
                && samples.iter().all(|point| {
                    point.sub(start).cross(chord).length() / chord.length() <= tolerance
                });
            edge.curve = if straight {
                crate::make_line(start, end).or_refuse(KernelStage::Refine, "joint_offset_line")?
            } else {
                let parameters = (0..=count)
                    .map(|i| i as f64 / count as f64)
                    .collect::<Vec<_>>();
                if original.start_vertex_id == original.end_vertex_id {
                    // A closed section must have one tangent at its seam. Use a
                    // symmetric fourth-order derivative and commit it at both ends.
                    let tangent = samples[count - 2]
                        .sub(samples[count - 1].scale(8.0))
                        .add(samples[1].scale(8.0))
                        .sub(samples[2])
                        .scale(count as f64 / 12.0);
                    let mut curve = crate::interpolate_curve(&samples, 3, &parameters)
                        .or_refuse(KernelStage::Refine, "joint_offset_interpolate")?;
                    let last = curve.control_points.len() - 1;
                    let first_step = curve.knots[curve.degree + 1] / curve.degree as f64;
                    let last_step = (1.0 - curve.knots[last]) / curve.degree as f64;
                    // The clamped B-spline endpoint derivative is p*(P1-P0)/Δt.
                    // Moving these two controls commits the same derivative at
                    // both seam uses; the residual/refinement below gates it.
                    curve.control_points[1] =
                        crate::Vec4::from_point(start.add(tangent.scale(first_step)), 1.0);
                    curve.control_points[last - 1] =
                        crate::Vec4::from_point(end.sub(tangent.scale(last_step)), 1.0);
                    curve
                } else {
                    crate::interpolate_curve(&samples, 3, &parameters)
                        .or_refuse(KernelStage::Refine, "joint_offset_interpolate")?
                }
            };
            let [t0, t1] = edge
                .curve
                .domain()
                .or_refuse(KernelStage::Refine, "joint_offset_domain")?;
            edge.t0 = t0;
            edge.t1 = t1;
            // Read between interpolation stations, where interpolation error lives.
            // Refine to the paired surfaces' fit contract, with a bounded budget.
            let mut worst = 0.0f64;
            for span in edge.curve.knots.windows(2).filter(|span| span[1] > span[0]) {
                let point = edge
                    .curve
                    .evaluate((span[0] + span[1]) * 0.5)
                    .or_refuse(KernelStage::Refine, "joint_offset_fit")?;
                for surface in &adjacent {
                    let projection = project_point_to_surface(surface, point)
                        .or_refuse(KernelStage::Refine, "joint_offset_fit")?;
                    worst = worst.max(projection.distance);
                }
            }
            if worst <= fit_tolerance {
                break;
            }
            if count == 1024 {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_shell_joint", format!("offset edge {} misses its carrier by {worst} (fit tolerance {fit_tolerance})", edge.id)));
            }
            count *= 2;
        }
    }
    let edges = body
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect::<HashMap<_, _>>();
    for (index, face) in body
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .enumerate()
    {
        face.surface = surfaces[index].clone();
        let surface = face.surface.clone();
        crate::offset_retrim::rebuild_loop_pcurves(
            face,
            &edges,
            &surface,
            tolerance,
            "offset joints",
        )?;
        // A cavity whose opposite walls crossed can still have a closed
        // incidence graph. Its inverted wires are not a surviving offset.
        if parameter_space_area(face).or_refuse(KernelStage::Refine, "joint_offset_winding")?
            * parameter_space_area(faces[index])
                .or_refuse(KernelStage::Refine, "joint_offset_winding")?
            <= 0.0
        {
            return Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "offset_shell_joint",
                "offset joints reverse a source wire",
            ));
        }
    }
    let issues = body.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::unsupported(
            KernelStage::Refine,
            "offset_shell_joint",
            format!("offset joint validation: {issues:?}"),
        ));
    }
    // A closed, consistently wound graph can still leave a reflected cavity
    // after opposite offsets cross. Every retained outward skin must stay at
    // least |d| from ALL retained source faces. Read the actual trimmed-face
    // Euclidean distance: the planar miter metric would incorrectly shadow a
    // rounded fillet beside a tangent plane.
    let source_edges = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let retained = faces
        .iter()
        .filter(|face| !openings.contains(&face.id))
        .map(|face| {
            let edges = face
                .loops
                .iter()
                .flat_map(|lp| &lp.coedges)
                .map(|coedge| source_edges[&coedge.edge_id])
                .collect::<Vec<_>>();
            (*face, edges)
        })
        .collect::<Vec<_>>();
    for face in body
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| !openings.contains(&face.id))
    {
        for seed in source_seeds(&body, face.id)? {
            let point = face
                .surface
                .evaluate(seed.x, seed.y)
                .or_refuse(KernelStage::Refine, "joint_offset_skin")?;
            for (support, edges) in &retained {
                let measured = distance_to_trimmed_face(support, edges, point)?;
                if !measured.is_finite() || measured < distance.abs() - 2.0 * fit_tolerance {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Refine,
                        "offset_shell_joint",
                        "offset joints leave skin shadowed by another retained face",
                    ));
                }
            }
        }
    }
    if !crate::loop_self_crossings(&body).crossings.is_empty()
        || crate::solid_self_intersections(&body, crate::SelfIntersectionOptions::for_solid(&body))
            .or_refuse(KernelStage::Refine, "joint_offset_soundness")?
            .is_flagged()
    {
        return Err(KernelRefusal::unsupported(
            KernelStage::Refine,
            "offset_shell_joint",
            "offset joints change topology",
        ));
    }
    Ok(OffsetShellResultRecord {
        solid: body,
        face_images: images,
    })
}
