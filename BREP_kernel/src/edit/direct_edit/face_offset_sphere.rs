use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

// ---------------------------------------------------------------------------
// Push a SPHERICAL analytic face by OFFSETTING its carrier to a concentric
// sphere (backlog #4a Sphere × Plane through-centre + #4b off-centre / sphere).
//
// The methodology mirrors `offset_ruled_face` (a face is a TRIMMED region of an
// infinite carrier surface; to push it we replace the carrier by its offset and
// re-derive the trim against the fixed neighbour carriers):
//
//   * The exact offset of a sphere is a CONCENTRIC sphere of radius r ± δ. It is
//     realised here as the exact uniform scale of the carrier about the sphere
//     centre by k = r_new / r — an AFFINE map, so the scaled surface is byte-for-
//     byte the same rational parameterisation (same knots/weights/seam azimuth/
//     poles), only radially grown. This is the *exact* sphere offset the backlog
//     item calls for; it is NOT the `offset_surface` sample-and-fit, which was
//     tried first and does NOT re-recognise as a sphere for a full-sphere face
//     (its Greville fit degenerates at the poles → `analytic() == None`). The
//     affine scale keeps the pushed face's OWN-ONLY (seam meridian + pole) and
//     THROUGH-CENTRE-rim pcurves valid pointwise: because S′(u,v) = c + k·(S − c),
//     each such edge, scaled about c by the SAME k, lands on S′ at its OWN
//     unchanged pcurve.
//
//   * RIM classification per boundary edge (honest refusal, never a bad solid):
//       - PLANE through the centre → SCALE (4a): a uniform scale about c maps a
//         through-centre plane onto ITSELF, so the scaled great-circle rim IS the
//         exact S′ ∩ plane, no azimuth/seam mismatch.
//       - PLANE off-centre, or a SPHERE neighbour → RECOMPUTE (4b): the scaled
//         rim would leave the fixed carrier, so the rim edge is re-derived as
//         S′ ∩ neighbour. Off-centre small circles: `intersect_plane_quadric`
//         gives radius √((r±δ)²−h²); sphere × sphere: `intersect_sphere_sphere`
//         (both via the public `intersect_analytic_pair`). The rim's pushed-face
//         pcurve is rebuilt on S′; a SEAM-crossing rim (its closure vertex is
//         shared with an own-only meridian — the axis-perpendicular cap case)
//         also rebuilds that meridian as the v-subrange of the offset meridian
//         (an iso-curve preserves the v-parameter exactly, keeping the straight
//         (u,v) pcurve synchronised) and moves its closure vertex to the new
//         latitude. Seam-aligned circle construction is used there so the rim
//         starts on the u = 0 seam.
//       - coaxial cylinder / cone → RECOMPUTE: the exact latitude circle is
//         selected from the coaxial-revolution intersector and the ruled
//         neighbour is extended/retrimmed over the relocated rim.
//       - non-coaxial cylinder / cone, general revolution / torus / free-form
//         → refused.
//     Refused too: a seam-crossing rim on an OBLIQUE plane or a SPHERE neighbour,
//     a non-closed / multi-edge RECOMPUTED rim, a vanished cap (grown sphere no
//     longer reaches the plane / spheres separated or tangent), and a push that
//     collapses or inverts the solid.
//
//   * An OPEN rim on the EXACT-SCALE path is supported, and its corners are
//     carried. A spherical pocket in a box CORNER has three quarter-circle rims,
//     one per box plane, all three through the centre; the scale moves each
//     arc's endpoint radially and the box edge that ends there has its trim slid
//     to follow. That slide is a re-parameterisation, not a re-solve: both
//     planes of such a corner pass through the sphere centre (which is what put
//     the rim on this path), so their intersection line does too and the scale
//     maps the corner along it exactly. A corner that does NOT land on its own
//     fixed edge, or a slide that would collapse or invert that edge's trim,
//     refuses — the open-rim analogue of the recomputed path's closed-rim gate.
// ---------------------------------------------------------------------------

/// Push a SPHERICAL face along its outward normal by `distance` (positive grows
/// the solid) by replacing its carrier with the concentric offset sphere and
/// re-trimming its neighbours: planar caps through the centre keep their exact-
/// scale great-circle rim, while off-centre planes, spheres, and coaxial ruled
/// neighbours have their rim re-derived as S′ ∩ neighbour.
///
/// An OPEN rim (several arcs, as a pocket in a box corner has) is supported on
/// the exact-scale path, with the fixed edges that end on its corners re-trimmed
/// to follow them.
///
/// Refused cleanly (never a bad solid): non-coaxial cylinder/cone neighbours,
/// general-revolution/torus/free-form neighbours, seam-crossing oblique or
/// sphere-neighbour rims, non-closed RECOMPUTED rims, a rim corner that leaves
/// its own fixed edge or collapses that edge's trim, a vanished cap, and a push
/// that collapses/inverts the solid.
pub fn offset_sphere_face(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    if !distance.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "distance",
            "offset_sphere_face: distance must be finite",
        ));
    }
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);

    let (pshell, pface) = find_face(solid, face_id)
        .ok_or_else(|| {
            KernelRefusal::input(
                KernelStage::Collect,
                "face_id",
                format!("offset_sphere_face: no face {face_id}"),
            )
        })?;
    let pushed = &solid.shells[pshell].faces[pface];
    let Some(AnalyticSurface::Sphere { frame, radius }) = pushed.surface.analytic() else {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "not_sphere",
            "offset_sphere_face: the pushed face is not a sphere",
        ));
    };
    let center = frame.origin;
    let axis = frame.axis;
    let seam_x = frame.x_axis;
    let seam_y = frame.y_axis;
    let radius = *radius;
    if radius <= tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "degenerate_radius",
            "offset_sphere_face: degenerate sphere radius",
        ));
    }

    // Sign the push: `distance` is measured along the face's OUTWARD normal. For
    // a convex ball the outward normal is radial (s = +1) so the radius grows; a
    // spherical POCKET's outward normal points inward (s = −1) so a positive push
    // (more material) SHRINKS the cavity. Either way r_new = r + distance·s.
    let [u0, u1] = pushed.surface.domain_u().or_refuse(KernelStage::Classify, "domain_u")?;
    let [v0, v1] = pushed.surface.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let point = pushed.surface.evaluate(um, vm).or_refuse(KernelStage::Classify, "evaluate")?;
    let mut normal = pushed.surface.normal(um, vm).or_refuse(KernelStage::Classify, "normal")?;
    if !pushed.same_sense {
        normal = normal.scale(-1.0);
    }
    let radial = point.sub(center);
    if radial.length() <= tolerance {
        return Err(KernelRefusal::internal(
            KernelStage::Classify,
            "radial_direction",
            "offset_sphere_face: degenerate radial direction",
        ));
    }
    let outward_sign = if normal.dot(radial) >= 0.0 { 1.0 } else { -1.0 };
    let radius_new = radius + distance * outward_sign;
    if radius_new <= tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "collapse",
            "offset_sphere_face: the push collapses the sphere to or past its centre — refusing",
        ));
    }

    // S′ = exact concentric offset = uniform scale of the carrier about the
    // centre by k. An affine map, so the parameterisation is preserved exactly.
    let k = radius_new / radius;
    let scale_about_center = AffineTransform::new([
        k, 0.0, 0.0, (1.0 - k) * center.x,
        0.0, k, 0.0, (1.0 - k) * center.y,
        0.0, 0.0, k, (1.0 - k) * center.z,
        0.0, 0.0, 0.0, 1.0,
    ]).or_refuse(KernelStage::Refine, "affine_transform")?;
    let s_prime = transform_surface(&pushed.surface, scale_about_center).or_refuse(KernelStage::Refine, "transform_surface")?;
    // Defensive: confirm the offset is the concentric sphere we expect.
    match s_prime.analytic() {
        Some(AnalyticSurface::Sphere { frame, radius }) => {
            if frame.origin.sub(center).length() > plane_tolerance
                || (radius - radius_new).abs() > plane_tolerance.max(radius_new * 1e-9)
            {
                return Err(KernelRefusal::internal(
                    KernelStage::Refine,
                    "offset_carrier",
                    "offset_sphere_face: offset carrier is not the expected \
                            concentric sphere — refusing",
                ));
            }
        }
        _ => {
            return Err(KernelRefusal::internal(
                KernelStage::Refine,
                "offset_recognition",
                "offset_sphere_face: offset carrier did not re-recognise as a sphere — refusing",
            ))
        }
    }

    // Edge -> incident faces, to classify the pushed face's boundary edges into
    // OWN-ONLY edges (seam meridians + degenerate poles — intrinsic to the sphere)
    // and RIM edges shared with a fixed neighbour.
    let mut faces_of_edge: HashMap<u64, Vec<u64>> = HashMap::default();
    for shell in &solid.shells {
        for face in &shell.faces {
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    faces_of_edge
                        .entry(coedge.edge_id)
                        .or_default()
                        .push(face.id);
                }
            }
        }
    }
    let edge_by_id: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();

    // The v-parameter domain of the offset sphere (== the source domain; the
    // uniform scale preserves the parameterisation exactly).
    let [dv0, dv1] = s_prime.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;

    // Classify. A rim neighbour is one of:
    //   * PLANE through the centre → exact-scale path (the landed 4a behaviour):
    //     the scaled great-circle rim stays on the fixed plane.
    //   * PLANE off-centre / SPHERE / coaxial CYLINDER or CONE → RECOMPUTE:
    //     the scaled rim leaves the fixed carrier, so re-derive S′ ∩ neighbour.
    //   * non-coaxial ruled / general revolution / torus / free-form → refused.
    enum RimKind {
        OffCentrePlane(Plane),
        NeighbourSphere,
        CoaxialRuled,
    }
    let mut edges_to_scale: HashSet<u64> = HashSet::default();
    let mut through_centre_caps: HashSet<u64> = HashSet::default();
    let mut own_only_edges: Vec<u64> = Vec::new();
    let mut all_boundary_vertices: HashSet<u64> = HashSet::default();
    let mut raw_rims: Vec<(u64, u64, RimKind)> = Vec::new();

    for loop_record in &pushed.loops {
        for coedge in &loop_record.coedges {
            let edge = *edge_by_id
                .get(&coedge.edge_id)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Classify,
                        "missing_edge",
                        format!("offset_sphere_face: missing edge {}", coedge.edge_id),
                    )
                })?;
            all_boundary_vertices.insert(edge.start_vertex_id);
            all_boundary_vertices.insert(edge.end_vertex_id);

            let incident = faces_of_edge.get(&edge.id).cloned().unwrap_or_default();
            if incident.iter().all(|f| *f == face_id) {
                own_only_edges.push(edge.id); // Seam meridian or degenerate pole.
                continue;
            }
            let neighbour = *incident
                .iter()
                .find(|f| **f != face_id)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Classify,
                        "no_neighbour",
                        format!("offset_sphere_face: edge {} has no neighbour", edge.id),
                    )
                })?;
            let (nshell, nface) = find_face(solid, neighbour)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Classify,
                        "missing_neighbour",
                        format!("offset_sphere_face: missing neighbour {neighbour}"),
                    )
                })?;
            let neighbour_surface = &solid.shells[nshell].faces[nface].surface;
            match neighbour_surface.analytic() {
                Some(AnalyticSurface::Plane { .. }) => {
                    let plane =
                        plane_of_surface(neighbour_surface, plane_tolerance, "offset_sphere_face")?;
                    if center.sub(plane.origin).dot(plane.normal).abs() <= plane_tolerance {
                        // Through the centre: the exact-scale great-circle path.
                        through_centre_caps.insert(neighbour);
                        edges_to_scale.insert(edge.id);
                    } else {
                        raw_rims.push((edge.id, neighbour, RimKind::OffCentrePlane(plane)));
                    }
                }
                Some(AnalyticSurface::Sphere { .. }) => {
                    raw_rims.push((edge.id, neighbour, RimKind::NeighbourSphere));
                }
                Some(AnalyticSurface::RuledRevolution { .. }) => {
                    if !super::face_offset::ruled_neighbour_is_coaxial(
                        neighbour_surface,
                        center,
                        axis,
                        scale,
                    ) {
                        return Err(KernelRefusal::unsupported(
                            KernelStage::Classify,
                            "non_coaxial_ruled",
                            "offset_sphere_face: a cylinder/cone rim neighbour is \
                                    NON-coaxial (only coaxial sphere × ruled intersections are \
                                    supported) — refusing",
                        ));
                    }
                    raw_rims.push((edge.id, neighbour, RimKind::CoaxialRuled));
                }
                Some(AnalyticSurface::Torus { .. }) => {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "torus_neighbour",
                        "offset_sphere_face: a toroidal rim neighbour is deferred \
                                (torus is untouched in this slice)",
                    ));
                }
                _ => {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "neighbour_kind",
                        "offset_sphere_face: a rim neighbour is not a supported plane, \
                                sphere, or coaxial cylinder/cone (general revolution and \
                                free-form neighbours are deferred in this slice)",
                    ));
                }
            }
        }
    }

    // Each recomputed rim must be a single CLOSED edge (start == end vertex); its
    // closure vertex sits where the rim crosses the sphere seam.
    let mut closure_vertices: HashSet<u64> = HashSet::default();
    for (edge_id, _, _) in &raw_rims {
        let edge = *edge_by_id.get(edge_id).unwrap();
        if edge.start_vertex_id != edge.end_vertex_id {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "open_rim",
                "offset_sphere_face: a recomputed rim is not a single closed edge \
                        (multi-edge / open rims are deferred in this slice)",
            ));
        }
        closure_vertices.insert(edge.start_vertex_id);
    }

    // Own-only seam meridians whose end lands on a recomputed rim's closure vertex
    // must be rebuilt (their rim end moves to the new latitude). Uncoupled seam
    // meridians and poles just scale.
    let mut coupled_meridians: HashSet<u64> = HashSet::default();
    for own in &own_only_edges {
        let e = *edge_by_id.get(own).unwrap();
        if e.degenerate {
            edges_to_scale.insert(*own);
            continue;
        }
        if closure_vertices.contains(&e.start_vertex_id)
            || closure_vertices.contains(&e.end_vertex_id)
        {
            coupled_meridians.insert(*own);
        } else {
            edges_to_scale.insert(*own);
        }
    }
    // Every boundary vertex scales EXCEPT the relocated rim-closure vertices.
    let vertices_to_scale: HashSet<u64> = all_boundary_vertices
        .iter()
        .copied()
        .filter(|v| !closure_vertices.contains(v))
        .collect();

    // --- Build the recomputed rims (geometry only; applied below) ----------
    struct RimBuild {
        edge_id: u64,
        neighbour_id: u64,
        neighbour_is_sphere: bool,
        closure_vertex: u64,
        circle: NurbsCurve,
        closure_point: Vec3,
        v_rim: f64,
        pushed_pcurve: NurbsCurve,
    }
    let mut rim_builds: Vec<RimBuild> = Vec::new();
    for (edge_id, neighbour, kind) in &raw_rims {
        let edge = *edge_by_id.get(edge_id).unwrap();
        let closure_vertex = edge.start_vertex_id;
        let seam_coupled = coupled_meridians.iter().any(|m| {
            let e = *edge_by_id.get(m).unwrap();
            e.start_vertex_id == closure_vertex || e.end_vertex_id == closure_vertex
        });

        let mut circle = if seam_coupled {
            // A seam-crossing rim shares its closure vertex with a meridian, so
            // the new circle MUST start exactly on the seam. Build it in the
            // sphere's own frame (seam-aligned by construction). Only an
            // axis-perpendicular (latitude) plane keeps the rim a fixed-u circle;
            // oblique seam-crossing caps and sphere neighbours are refused.
            match kind {
                RimKind::OffCentrePlane(plane) => {
                    if plane.normal.dot(axis).abs() < 1.0 - 1e-6 {
                        return Err(KernelRefusal::unsupported(
                            KernelStage::Classify,
                            "oblique_seam_rim",
                            "offset_sphere_face: a seam-crossing rim on an OBLIQUE plane is \
                                    deferred in this slice (only axis-perpendicular caps)",
                        ));
                    }
                    let a = plane.origin.sub(center).dot(axis);
                    let rr2 = radius_new * radius_new - a * a;
                    if rr2 <= tolerance * tolerance {
                        return Err(KernelRefusal::unsupported(
                            KernelStage::Refine,
                            "cap_vanishes",
                            "offset_sphere_face: the push pulls the grown sphere off its cap \
                                    plane (the cap vanishes) — refusing",
                        ));
                    }
                    crate::make_arc(
                        center.add(axis.scale(a)),
                        seam_x,
                        seam_y,
                        rr2.sqrt(),
                        0.0,
                        std::f64::consts::TAU,
                    ).or_refuse(KernelStage::Refine, "make_arc")?
                }
                RimKind::NeighbourSphere => {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "sphere_seam_rim",
                        "offset_sphere_face: a sphere neighbour whose rim crosses the \
                                pushed sphere's seam is deferred in this slice",
                    ))
                }
                RimKind::CoaxialRuled => {
                    // A coaxial ruled neighbour meets the sphere in latitude
                    // circles. The analytic intersector below constructs them
                    // in the pushed sphere's revolution frame, so the selected
                    // circle starts on this same seam and can move the coupled
                    // meridian exactly.
                    let (ns, nf) = find_face(solid, *neighbour).unwrap();
                    let neighbour_surface = &solid.shells[ns].faces[nf].surface;
                    let curves = intersect_analytic_pair(&s_prime, neighbour_surface, tolerance)
                        .filter(|curves| !curves.is_empty())
                        .ok_or_else(|| {
                            KernelRefusal::unsupported(
                                KernelStage::Refine,
                                "ruled_separated",
                                "offset_sphere_face: the grown sphere no longer meets its coaxial \
                             cylinder/cone neighbour — refusing",
                            )
                        })?;
                    let old_mid = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).or_refuse(KernelStage::Refine, "evaluate")?;
                    nearest_circle(&curves, old_mid)?
                }
            }
        } else {
            let (ns, nf) = find_face(solid, *neighbour).unwrap();
            let neighbour_surface = &solid.shells[ns].faces[nf].surface;
            let curves = intersect_analytic_pair(&s_prime, neighbour_surface, tolerance)
                .filter(|curves| !curves.is_empty())
                .ok_or_else(|| {
                    KernelRefusal::unsupported(
                        KernelStage::Refine,
                        "separated",
                        "offset_sphere_face: the grown sphere no longer meets a neighbour (the cap \
                     vanishes / tangent / separated) — refusing",
                    )
                })?;
            let old_mid = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).or_refuse(KernelStage::Refine, "evaluate")?;
            nearest_circle(&curves, old_mid)?
        };

        // Match the new rim's traversal to the old edge so the preserved coedge
        // `forward` flags keep the loop winding consistent.
        let closure_old = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?;
        let old_tan = edge
            .curve
            .evaluate(edge.t0 + 0.01 * (edge.t1 - edge.t0)).or_refuse(KernelStage::Refine, "evaluate")?
            .sub(closure_old);
        let [c0, c1] = circle.domain().or_refuse(KernelStage::Refine, "domain")?;
        let new_start = circle.evaluate(c0).or_refuse(KernelStage::Refine, "evaluate")?;
        let new_tan = circle.evaluate(c0 + 0.01 * (c1 - c0)).or_refuse(KernelStage::Refine, "evaluate")?.sub(new_start);
        if old_tan.dot(new_tan) < 0.0 {
            circle = circle.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        }
        let closure_point = circle.evaluate(circle.domain().or_refuse(KernelStage::Refine, "domain")?[0]).or_refuse(KernelStage::Refine, "evaluate")?;
        let pushed_pcurve = build_pcurve_on_surface(&s_prime, &circle).or_refuse(KernelStage::Refine, "build_pcurve_on_surface")?;
        // The rim's v on the offset sphere (constant for a latitude rim; only the
        // seam-coupled meridian rebuild consumes it).
        let [q0, q1] = pushed_pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let v_rim = pushed_pcurve.evaluate(0.5 * (q0 + q1)).or_refuse(KernelStage::Refine, "evaluate")?.y;

        rim_builds.push(RimBuild {
            edge_id: *edge_id,
            neighbour_id: *neighbour,
            neighbour_is_sphere: matches!(kind, RimKind::NeighbourSphere),
            closure_vertex,
            circle,
            closure_point,
            v_rim,
            pushed_pcurve,
        });
    }

    // The offset sphere's u = 0 seam meridian (parameterised by v exactly: an
    // iso-curve preserves the surface v-parameter, so a straight (u, v) pcurve
    // stays synchronised with it — a fresh `make_arc` would not).
    let iso_meridian = s_prime.iso_curve_u(0.0).or_refuse(KernelStage::Refine, "iso_curve_u")?;

    // --- Apply to a fresh clone (the input is never mutated) ---------------
    let mut result = solid.clone();

    // Scale the intrinsic own-only edges + any through-centre great-circle rims.
    for edge in &mut result.edges {
        if edges_to_scale.contains(&edge.id) {
            edge.curve = transform_curve(&edge.curve, scale_about_center).or_refuse(KernelStage::Fragment, "transform_curve")?;
        }
    }
    // Recomputed rim edges take their new circle.
    for rb in &rim_builds {
        if let Some(edge) = result.edges.iter_mut().find(|e| e.id == rb.edge_id) {
            let [d0, d1] = rb.circle.domain().or_refuse(KernelStage::Fragment, "domain")?;
            edge.curve = rb.circle.clone();
            edge.t0 = d0;
            edge.t1 = d1;
        }
    }
    // Rebuild each seam-coupled own edge as the matching v-subrange of the
    // offset meridian. The fixed endpoint can be a pole or another trim split.
    for mer in &coupled_meridians {
        let e = *edge_by_id.get(mer).unwrap();
        let cv = if closure_vertices.contains(&e.start_vertex_id) {
            e.start_vertex_id
        } else {
            e.end_vertex_id
        };
        let v_rim = rim_builds
            .iter()
            .find(|rb| rb.closure_vertex == cv)
            .map(|rb| rb.v_rim)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "meridian_pairing",
                    "offset_sphere_face: a seam meridian is not paired with a rim — refusing",
                )
            })?;
        let closure_at_start = e.start_vertex_id == cv;
        let fixed_vertex = if closure_at_start {
            e.end_vertex_id
        } else {
            e.start_vertex_id
        };
        let fixed_old = solid
            .vertices
            .iter()
            .find(|v| v.id == fixed_vertex)
            .map(|v| v.point)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "seam_vertex",
                    format!("offset_sphere_face: missing seam vertex {fixed_vertex}"),
                )
            })?;
        let fixed_new = scale_about_center.point(fixed_old);
        let fixed_projection = project_point_to_curve(&iso_meridian, fixed_new).or_refuse(KernelStage::Fragment, "project_point_to_curve")?;
        if fixed_projection.distance > plane_tolerance {
            return Err(KernelRefusal::internal(
                KernelStage::Fragment,
                "seam_projection",
                format!(
                "offset_sphere_face: a coupled own-edge is not on the sphere seam \
                 (off by {:.3e}) — refusing",
                fixed_projection.distance
            ),
            ));
        }
        let v_start = if closure_at_start {
            v_rim
        } else {
            fixed_projection.u
        };
        let v_end = if closure_at_start {
            fixed_projection.u
        } else {
            v_rim
        };
        let (curve, t0, t1) = if v_start <= v_end {
            (iso_meridian.clone(), v_start, v_end)
        } else {
            (
                iso_meridian.reversed().or_refuse(KernelStage::Fragment, "reversed")?,
                dv0 + dv1 - v_start,
                dv0 + dv1 - v_end,
            )
        };
        if let Some(edge) = result.edges.iter_mut().find(|x| x.id == *mer) {
            edge.curve = curve;
            edge.t0 = t0;
            edge.t1 = t1;
        }
    }

    // Vertices: scale the intrinsic ones, relocate the rim-closure ones.
    for vertex in &mut result.vertices {
        if vertices_to_scale.contains(&vertex.id) {
            vertex.point = scale_about_center.point(vertex.point);
        }
    }
    for rb in &rim_builds {
        if let Some(v) = result.vertices.iter_mut().find(|v| v.id == rb.closure_vertex) {
            v.point = rb.closure_point;
        }
    }

    // The pushed face rides the offset carrier. Own-only + through-centre pcurves
    // are preserved (scale-invariant in (u, v)); recomputed rims and coupled
    // meridians get fresh pcurves.
    {
        let pface_rec = &mut result.shells[pshell].faces[pface];
        for loop_record in &mut pface_rec.loops {
            for coedge in &mut loop_record.coedges {
                if let Some(rb) = rim_builds.iter().find(|rb| rb.edge_id == coedge.edge_id) {
                    let mut pcurve = rb.pushed_pcurve.clone();
                    if !coedge.forward {
                        pcurve = pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")?;
                    }
                    coedge.pcurve = pcurve;
                } else if coupled_meridians.contains(&coedge.edge_id) {
                    let e = *edge_by_id.get(&coedge.edge_id).unwrap();
                    let cv = if closure_vertices.contains(&e.start_vertex_id) {
                        e.start_vertex_id
                    } else {
                        e.end_vertex_id
                    };
                    let v_rim = rim_builds
                        .iter()
                        .find(|rb| rb.closure_vertex == cv)
                        .map(|rb| rb.v_rim)
                        .unwrap();
                    let closure_at_pcurve_start = (e.start_vertex_id == cv) == coedge.forward;
                    coedge.pcurve =
                        patch_meridian_pcurve(&coedge.pcurve, v_rim, closure_at_pcurve_start)?;
                }
            }
        }
    }
    result.shells[pshell].faces[pface].surface = s_prime;

    // A coaxial ruled neighbour has its own straight seam meridian ending at
    // the shared rim's relocated closure vertex. Rebuild that seam between its
    // current endpoints before extending/retrimming the neighbour carrier.
    let result_vertex: HashMap<u64, Vec3> =
        result.vertices.iter().map(|v| (v.id, v.point)).collect();
    let mut ruled_seams: HashSet<u64> = HashSet::default();
    for rb in &rim_builds {
        let (ns, nf) = find_face(&result, rb.neighbour_id).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Fragment,
                "ruled_seam_neighbour",
                format!("offset_sphere_face: missing neighbour {}", rb.neighbour_id),
            )
        })?;
        if !matches!(
            result.shells[ns].faces[nf].surface.analytic(),
            Some(AnalyticSurface::RuledRevolution { .. })
        ) {
            continue;
        }
        for loop_record in &result.shells[ns].faces[nf].loops {
            for coedge in &loop_record.coedges {
                let edge = *edge_by_id.get(&coedge.edge_id).ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Fragment,
                        "ruled_seam_edge",
                        format!("offset_sphere_face: missing edge {}", coedge.edge_id),
                    )
                })?;
                let incident = faces_of_edge.get(&edge.id).cloned().unwrap_or_default();
                let is_seam = incident.iter().all(|f| *f == rb.neighbour_id);
                let touches_rim = closure_vertices.contains(&edge.start_vertex_id)
                    || closure_vertices.contains(&edge.end_vertex_id);
                if is_seam && touches_rim {
                    ruled_seams.insert(edge.id);
                }
            }
        }
    }
    let rebuilt_ruled_seams: HashSet<u64> = ruled_seams.iter().copied().collect();
    for seam_id in ruled_seams {
        let edge = *edge_by_id
            .get(&seam_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "missing_ruled_seam",
                    format!("offset_sphere_face: missing ruled seam {seam_id}"),
                )
            })?;
        let curve = make_line(
            result_vertex[&edge.start_vertex_id],
            result_vertex[&edge.end_vertex_id],
        ).or_refuse(KernelStage::Fragment, "make_line")?;
        if let Some(out) = result
            .edges
            .iter_mut()
            .find(|candidate| candidate.id == seam_id)
        {
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
            out.curve = curve;
            out.t0 = t0;
            out.t1 = t1;
        }
    }

    // --- FIXED edges that END on a relocated rim corner --------------------
    //
    // Reported 2026-09-01 ("Push face fails"): a spherical pocket in a box
    // CORNER — three quarter-circle rims, each on a box plane through the
    // sphere centre — pushed inward produced a solid the validator rejected
    // with `edge 1 curve start does not match vertex 1 (gap=1.5)`.
    //
    // Why nothing caught it. A recomputed rim must be a single CLOSED edge
    // (the guard a hundred lines up); the EXACT-SCALE path a through-centre
    // plane takes has no such guard, because a closed great circle has no
    // endpoint to leave behind. An OPEN rim arc has two, and each is also the
    // end of a FIXED edge of the two neighbour planes — the box's own axis
    // edge. The push scales the corner radially; that edge's CURVE does not
    // move (a plane × plane line the push does not touch) but its TRIM must
    // follow the corner, or the solid carries an edge whose curve no longer
    // reaches its own vertex.
    //
    // The slide is a re-parameterisation, not a re-solve, and provably so: a
    // corner of an exact-scale rim is shared by two planes that BOTH pass
    // through the sphere centre (that is what put the rim on this path), so
    // their intersection line passes through the centre and the uniform scale
    // about the centre maps the corner along it exactly. The projection below
    // is the PROOF of that, not a fit — a corner that does not land on its own
    // fixed edge (a rim whose neighbour is off-centre, or a corner against a
    // curved carrier) refuses instead of being nudged.
    let mut relocated: HashMap<u64, Vec3> = HashMap::default();
    for vertex in &result.vertices {
        let Some(old) = solid.vertices.iter().find(|o| o.id == vertex.id) else {
            continue;
        };
        if vertex.point.sub(old.point).length() > tolerance {
            relocated.insert(vertex.id, vertex.point);
        }
    }
    let already_rebuilt: HashSet<u64> = edges_to_scale
        .iter()
        .copied()
        .chain(rim_builds.iter().map(|rb| rb.edge_id))
        .chain(coupled_meridians.iter().copied())
        .chain(rebuilt_ruled_seams.iter().copied())
        .collect();
    let mut resized_edges: HashSet<u64> = HashSet::default();
    for edge in &mut result.edges {
        if edge.degenerate || already_rebuilt.contains(&edge.id) {
            continue;
        }
        for at_start in [true, false] {
            let vertex_id = if at_start {
                edge.start_vertex_id
            } else {
                edge.end_vertex_id
            };
            let Some(&target) = relocated.get(&vertex_id) else {
                continue;
            };
            let projection = project_point_to_curve(&edge.curve, target).or_refuse(KernelStage::Fragment, "project_point_to_curve")?;
            if projection.distance > 10.0 * tolerance {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Fragment,
                    "corner_off_edge",
                    format!(
                    "offset_sphere_face: the push moves rim corner (vertex {vertex_id}) OFF \
                     fixed edge {} by {:.3e} — a rim corner that leaves its own side edge \
                     needs the corner re-solved against the neighbour carriers, which is \
                     deferred — refusing",
                    edge.id, projection.distance
                ),
                ));
            }
            let [d0, d1] = edge.curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
            let span = (d1 - d0).max(1e-12);
            let fixed_t = if at_start { edge.t1 } else { edge.t0 };
            let old_t = if at_start { edge.t0 } else { edge.t1 };
            let new_t = projection.u;
            if (new_t - fixed_t) * (old_t - fixed_t) <= 0.0
                || (new_t - fixed_t).abs() <= 1e-7 * span
            {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Fragment,
                    "trim_collapse",
                    format!(
                    "offset_sphere_face: the push collapses or inverts the trim of fixed edge \
                     {} at rim corner (vertex {vertex_id}) — refusing",
                    edge.id
                ),
                ));
            }
            if at_start {
                edge.t0 = new_t;
            } else {
                edge.t1 = new_t;
            }
            resized_edges.insert(edge.id);
        }
    }

    // Re-trim the neighbours around their new rims.
    let final_edges: HashMap<u64, EdgeRecord> =
        result.edges.iter().map(|e| (e.id, e.clone())).collect();
    for rb in &rim_builds {
        let (ns, nf) = find_face(&result, rb.neighbour_id).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "retrim_neighbour",
                format!("offset_sphere_face: missing neighbour {}", rb.neighbour_id),
            )
        })?;
        if rb.neighbour_is_sphere {
            // A fixed sphere neighbour: only the shared rim coedge's pcurve moves.
            let nsurf = result.shells[ns].faces[nf].surface.clone();
            for loop_record in &mut result.shells[ns].faces[nf].loops {
                for coedge in &mut loop_record.coedges {
                    if coedge.edge_id == rb.edge_id {
                        let mut pcurve = build_pcurve_on_surface(&nsurf, &rb.circle).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?;
                        if !coedge.forward {
                            pcurve = pcurve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
                        }
                        coedge.pcurve = pcurve;
                    }
                }
            }
        } else if matches!(
            result.shells[ns].faces[nf].surface.analytic(),
            Some(AnalyticSurface::RuledRevolution { .. })
        ) {
            super::face_offset::retrim_offset_ruled_face(
                &mut result,
                rb.neighbour_id,
                &final_edges,
                tolerance,
            )?;
        } else {
            let plane = plane_of_surface(
                &result.shells[ns].faces[nf].surface,
                plane_tolerance,
                "offset_sphere_face",
            )?;
            retrim_planar_face(
                &mut result.shells[ns].faces[nf],
                &plane,
                &final_edges,
                scale,
                "offset_sphere_face",
            )?;
            refit_subrange_pcurves(
                &mut result.shells[ns].faces[nf],
                &final_edges,
                tolerance,
            )?;
        }
    }
    // Through-centre planar caps re-trim around their grown great-circle rims.
    for cap in &through_centre_caps {
        let (cshell, cface) = find_face(&result, *cap)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "missing_cap",
                    format!("offset_sphere_face: missing cap {cap}"),
                )
            })?;
        let plane = plane_of_surface(
            &result.shells[cshell].faces[cface].surface,
            plane_tolerance,
            "offset_sphere_face",
        )?;
        retrim_planar_face(
            &mut result.shells[cshell].faces[cface],
            &plane,
            &final_edges,
            scale,
            "offset_sphere_face",
        )?;
        refit_subrange_pcurves(
            &mut result.shells[cshell].faces[cface],
            &final_edges,
            tolerance,
        )?;
    }
    // Any OTHER face carrying an edge whose trim the corner slide moved keeps
    // its carrier — only the pcurve of that one coedge has to be re-fitted over
    // the new `[t0, t1]`.
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            if face
                .loops
                .iter()
                .flat_map(|l| &l.coedges)
                .any(|c| resized_edges.contains(&c.edge_id))
            {
                refit_touched_pcurves(
                    face,
                    &final_edges,
                    &resized_edges,
                    true,
                    tolerance,
                    "offset_sphere_face",
                )?;
            }
        }
    }

    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!(
            "offset_sphere_face: pushed solid failed validation: {issues:?}"
        ),
        ));
    }
    if let (Ok(before), Ok(after)) = (solid_signed_volume(solid), solid_signed_volume(&result)) {
        if before * after <= 0.0 {
            return Err(KernelRefusal::internal(
                KernelStage::Validate,
                INVERTED,
                "offset_sphere_face: the push inverts the solid — refusing",
            ));
        }
    }
    Ok(result)
}

/// `retrim_planar_face` USED TO rebuild every pcurve over the edge's WHOLE
/// curve, which is right for a full-domain edge and WRONG for an edge that is a
/// strict SUBRANGE of its own curve. It is subrange-aware itself since
/// 2026-09-16, when the same defect surfaced a third time in
/// `delete_face_and_heal`, which had no such pass; this one is kept because it
/// is what produces this caller's final pcurves at its own tolerance, and
/// removing it is a separate change. The history, for why it exists: `validate()` samples a pcurve over its whole domain against the edge
/// over `[t0, t1]`, so the pcurve of a partially-trimmed edge reads as a
/// deviation the size of the trimmed-off overhang. `move_faces` has always
/// paired its planar re-trim with this pass (`face_move.rs`, the
/// `refit_touched_pcurves` call after `retrim_planar_face`); the sphere push
/// did not, which is what made a spherical pocket in a box CORNER fail
/// validation with `max deviation 5.000000` — exactly the length of box edge
/// the original sphere had trimmed away.
///
/// `only_subrange = true`, so a face whose edges are all full-domain — every
/// shape the corpus covers — keeps byte-identical pcurves.
fn refit_subrange_pcurves(
    face: &mut FaceRecord,
    edges: &HashMap<u64, EdgeRecord>,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let face_edges: HashSet<u64> = face
        .loops
        .iter()
        .flat_map(|loop_record| loop_record.coedges.iter().map(|c| c.edge_id))
        .collect();
    refit_touched_pcurves(
        face,
        edges,
        &face_edges,
        true,
        tolerance,
        "offset_sphere_face",
    )
    
}

/// The intersection circle whose parameter-midpoint is nearest `reference`
/// (branch selection when S′ ∩ neighbour returns more than one component).
fn nearest_circle(curves: &[NurbsCurve], reference: Vec3) -> Result<NurbsCurve, KernelRefusal> {
    let mut best: Option<(f64, &NurbsCurve)> = None;
    for curve in curves {
        let [d0, d1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mid = curve.evaluate(0.5 * (d0 + d1)).or_refuse(KernelStage::Refine, "evaluate")?;
        let distance = mid.sub(reference).length();
        if best.map(|(known, _)| distance < known).unwrap_or(true) {
            best = Some((distance, curve));
        }
    }
    best.map(|(_, curve)| curve.clone())
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Refine,
                "empty_intersection",
                "offset_sphere_face: the sphere ∩ neighbour intersection is empty",
            )
        })
}

/// Move a seam-meridian pcurve's RIM endpoint to the new latitude `v_new`,
/// keeping its pole endpoint and constant u. The pcurve is a straight (u, v)
/// parameter line; the pole endpoint is the one whose v is at a domain end
/// (0 or 1); the other endpoint is the rim that the offset relocated.
fn patch_meridian_pcurve(
    pcurve: &NurbsCurve,
    v_new: f64,
    closure_at_start: bool,
) -> Result<NurbsCurve, KernelRefusal> {
    let [q0, q1] = pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    let start = pcurve.evaluate(q0).or_refuse(KernelStage::Fragment, "evaluate")?;
    let end = pcurve.evaluate(q1).or_refuse(KernelStage::Fragment, "evaluate")?;
    let (v_start, v_end) = if closure_at_start {
        (v_new, end.y)
    } else {
        (start.y, v_new)
    };
    make_line(
        Vec3::new(start.x, v_start, 0.0),
        Vec3::new(end.x, v_end, 0.0),
    )
    .or_refuse(KernelStage::Fragment, "make_line")
}

