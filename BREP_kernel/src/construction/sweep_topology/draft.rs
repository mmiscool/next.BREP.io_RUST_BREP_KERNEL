use super::*;

/// Straight extrude of a LINE/ARC profile loop with a draft (taper) angle,
/// built DIRECTLY as a BREP: every wall is the EXACT drafted surface of its
/// segment — a tilted plane for a line, a cone patch (rational ruled surface
/// between the source arc and its concentric offset, over one shared angular
/// window) for a circular arc — and every junction edge is the EXACT
/// intersection curve of the two adjacent walls.  Because all drafted walls
/// shrink linearly at the same rate, any two of them intersect in a straight
/// line (plane∧plane, or a tangent-junction ruling) or a CONIC (plane∧cone and
/// cone∧cone both reduce to a plane section of a cone — the z² terms of the
/// squared implicits cancel), so the junction edges are exact rational
/// quadratics: no lofted approximation anywhere.
///
/// Positive `draft_angle_rad` tapers the section INWARD with height (smaller
/// far cap); negative widens it; 0 reproduces a straight prism.  The far
/// section is the exact in-plane miter offset of the profile by
/// d = distance·tan(draft): line segments re-intersect at their offset
/// corners, arcs shrink/grow concentrically, and mixed junctions re-join at
/// the offset primitives' intersection.  An offset that exceeds the local
/// feature size (a collapsed arc, offsets that no longer meet) is a clear Err.
/// Face order: [walls in INPUT curve order…, START cap (profile plane), END
/// cap (offset section at +distance)] — the naming contract callers stamp on.
pub fn extrude_profile_brep_draft(
    profile: &[NurbsCurve],
    direction: Vec3,
    distance: f64,
    draft_angle_rad: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // The builder carries no face names; accept `name` for ABI symmetry with
    // the other builders (the app stamps names onto the emitted face order).
    let _ = name;
    let tolerance = 1e-6;
    if profile.len() < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_count",
            "draftExtrude: profile needs at least 2 curves forming a closed loop",
        ));
    }
    if distance.abs() <= 1e-12 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "distance",
            "draftExtrude: distance must be non-zero",
        ));
    }
    let axis = direction.normalized().map_err(|_| {
        KernelRefusal::input(
            KernelStage::Collect,
            "direction",
            "draftExtrude: direction is degenerate",
        )
    })?;

    // --- 1. Validate the profile: closed + planar; derive origin O + normal np.
    let mut samples = Vec::new();
    for (index, curve) in profile.iter().enumerate() {
        let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
        let next = &profile[(index + 1) % profile.len()];
        let next_start = next.domain().or_refuse(KernelStage::Fragment, "domain")?[0];
        if curve
            .evaluate(end).or_refuse(KernelStage::Fragment, "evaluate")?
            .sub(next.evaluate(next_start).or_refuse(KernelStage::Fragment, "evaluate")?)
            .length()
            > tolerance
        {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "profile_closed",
                format!("draftExtrude: profile is not closed at curve {index}"),
            ));
        }
        for sample in 0..16 {
            samples.push(curve.evaluate(start + (end - start) * sample as f64 / 16.0).or_refuse(KernelStage::Fragment, "evaluate")?);
        }
    }
    let normal = crate::polygon::newell_normal(&samples);
    let centroid = samples.iter().fold(Vec3::default(), |sum, &point| sum.add(point));
    let np = normal.normalized().map_err(|_| {
        KernelRefusal::input(
            KernelStage::Collect,
            "profile_area",
            "draftExtrude: profile is degenerate (zero enclosed area)",
        )
    })?;
    let origin = centroid.scale(1.0 / samples.len() as f64);
    if samples
        .iter()
        .any(|point| point.sub(origin).dot(np).abs() > tolerance * 100.0)
    {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_planar",
            "draftExtrude: profile is not planar",
        ));
    }
    // A straight draft-extrude runs along the profile normal.
    if np.dot(axis).abs() < 0.999 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "oblique_draft",
            "draftExtrude: extrude direction must be parallel to the profile normal",
        ));
    }

    // --- 2. Normalize the loop CCW about the EXTRUDE direction and derive the
    //     signed in-plane offset d = distance·tan(draft).  In the CCW frame the
    //     per-segment offset normal zh×t points INWARD, so positive d shrinks
    //     the far section — algebraically identical to the historical
    //     winding·distance·tanθ law about the Newell normal, for either profile
    //     winding and either extrude side.
    let displacement = axis.scale(distance);
    let zh = displacement.normalized().or_refuse(KernelStage::Fragment, "normalized")?;
    let height = displacement.length();
    let x_axis = zh.perpendicular().or_refuse(KernelStage::Fragment, "perpendicular")?;
    let y_axis = zh.cross(x_axis).normalized().or_refuse(KernelStage::Fragment, "normalized")?;
    let mut curves: Vec<NurbsCurve> = profile.to_vec();
    let reversed_winding = profile_area(&curves, origin, x_axis, y_axis).or_refuse(KernelStage::Fragment, "profile_area")? < 0.0;
    if reversed_winding {
        curves = curves
            .iter()
            .rev()
            .map(NurbsCurve::reversed)
            .collect::<Result<_, _>>().or_refuse(KernelStage::Fragment, "collect")?;
    }
    let signed_d = distance * draft_angle_rad.tan();

    // --- 3. Classify segments and compute the exact junction stations: the
    //     original vertices (offset 0), the mid-height offsets (d/2, the conic
    //     shoulder witnesses), and the far offsets (d, raised by the extrude
    //     vector).  Every station is an exact offset-primitive intersection.
    let segs = classify_profile_segments(&curves, zh)
        .map_err(|e| e.with_message(|e| format!("draftExtrude: {e}")))?;
    let count = segs.len();
    let mut bottom_junctions = Vec::with_capacity(count);
    let mut top_junctions = Vec::with_capacity(count);
    let mut mid_junctions = Vec::with_capacity(count);
    for index in 0..count {
        let prev = &segs[(index + count - 1) % count];
        let next = &segs[index];
        bottom_junctions.push(
            offset_junction(prev, next, zh, 0.0)
                .map_err(|e| e.with_message(|e| format!("draftExtrude: {e}")))?,
        );
        top_junctions.push(
            offset_junction(prev, next, zh, signed_d)
                .map_err(|e| e.with_message(|e| format!("draftExtrude: {e}")))?
                .add(displacement),
        );
        mid_junctions.push(
            offset_junction(prev, next, zh, signed_d * 0.5)
                .map_err(|e| e.with_message(|e| format!("draftExtrude: {e}")))?
                .add(displacement.scale(0.5)),
        );
    }

    // --- 4. Junction (side) edges: the EXACT wall∧wall intersection curve — a
    //     straight line where the mid-height witness is collinear (plane∧plane
    //     miters, tangent-junction rulings), otherwise the exact conic through
    //     both endpoints with the analytic surface-gradient end tangents.
    let mut side_curves = Vec::with_capacity(count);
    for index in 0..count {
        side_curves.push(junction_edge_curve(
            &segs[(index + count - 1) % count],
            &segs[index],
            bottom_junctions[index],
            mid_junctions[index],
            top_junctions[index],
            zh,
            height,
            signed_d,
        )?);
    }

    // --- 5. Per-segment wall geometry: bottom/top boundary curves + the exact
    //     wall surface.  Arc walls share ONE angular window between the two
    //     rows so the ruled surface is the exact cone; their boundary edges are
    //     SUBRANGES of the rows (identical parameterization → parameter-line
    //     pcurves are pointwise exact).
    enum WallSurface {
        /// Affine plane patch + its frame (pcurves = exact plane projection).
        Plane { origin: Vec3, ex: Vec3, ey: Vec3 },
        /// Exact cone patch (pcurves for side edges are fitted on-surface).
        Cone,
    }
    let mut wall_surfaces = Vec::with_capacity(count);
    let mut wall_kinds = Vec::with_capacity(count);
    let mut bottom_curves = Vec::with_capacity(count);
    let mut top_curves = Vec::with_capacity(count);
    for index in 0..count {
        let next_index = (index + 1) % count;
        let a0 = bottom_junctions[index];
        let a1 = bottom_junctions[next_index];
        let b0 = top_junctions[index];
        let b1 = top_junctions[next_index];
        match &segs[index] {
            SegGeom::Line { dir, .. } => {
                let up = b0.sub(a0);
                let ey = up
                    .sub(dir.scale(up.dot(*dir)))
                    .normalized()
                    .map_err(|_| {
                        KernelRefusal::internal(
                            KernelStage::Fragment,
                            "wall_frame",
                            "draftExtrude: wall plane frame is degenerate",
                        )
                    })?;
                // Patch extent: the four junctions plus BOTH side curves'
                // control points (a conic bulges past its chord; the control
                // polygon's convex hull bounds it).
                let mut points = vec![a0, a1, b0, b1];
                for side in [&side_curves[index], &side_curves[next_index]] {
                    for control in &side.control_points {
                        points.push(control.point().or_refuse(KernelStage::Fragment, "control_point")?);
                    }
                }
                let mut min_x = f64::INFINITY;
                let mut min_y = f64::INFINITY;
                let mut max_x = f64::NEG_INFINITY;
                let mut max_y = f64::NEG_INFINITY;
                for point in &points {
                    let delta = point.sub(a0);
                    min_x = min_x.min(delta.dot(*dir));
                    max_x = max_x.max(delta.dot(*dir));
                    min_y = min_y.min(delta.dot(ey));
                    max_y = max_y.max(delta.dot(ey));
                }
                let padding = (max_x - min_x).max(max_y - min_y) * 0.05 + 1e-6;
                let patch_origin = a0
                    .add(dir.scale(min_x - padding))
                    .add(ey.scale(min_y - padding));
                wall_surfaces.push(make_plane(
                    patch_origin,
                    *dir,
                    ey,
                    max_x - min_x + 2.0 * padding,
                    max_y - min_y + 2.0 * padding,
                ).or_refuse(KernelStage::Fragment, "make_plane")?);
                wall_kinds.push(WallSurface::Plane {
                    origin: patch_origin,
                    ex: *dir,
                    ey,
                });
                bottom_curves.push(make_line(a0, a1).or_refuse(KernelStage::Fragment, "make_line")?);
                top_curves.push(make_line(b0, b1).or_refuse(KernelStage::Fragment, "make_line")?);
            }
            SegGeom::Arc {
                center,
                radius,
                turn,
                arc_normal,
                ..
            } => {
                let in_plane = |p: Vec3| {
                    let rel = p.sub(*center);
                    rel.sub(zh.scale(rel.dot(zh)))
                };
                let ax = in_plane(a0).normalized().or_refuse(KernelStage::Fragment, "normalized")?;
                let ay = arc_normal.cross(ax).normalized().or_refuse(KernelStage::Fragment, "normalized")?;
                let angle_near = |p: Vec3, near: f64| {
                    let ve = in_plane(p);
                    let mut angle = ve.dot(ay).atan2(ve.dot(ax));
                    while angle < near - std::f64::consts::PI {
                        angle += std::f64::consts::TAU;
                    }
                    while angle > near + std::f64::consts::PI {
                        angle -= std::f64::consts::TAU;
                    }
                    angle
                };
                let mut sweep = in_plane(a1).dot(ay).atan2(in_plane(a1).dot(ax));
                if sweep <= 1e-9 {
                    sweep += std::f64::consts::TAU;
                }
                let phi0 = angle_near(b0, 0.0);
                let phi1 = angle_near(b1, sweep);
                let theta_lo = 0.0_f64.min(phi0);
                let theta_hi = sweep.max(phi1);
                if theta_hi - theta_lo > std::f64::consts::TAU {
                    return Err(KernelRefusal::internal(
                        KernelStage::Fragment,
                        "arc_window",
                        "draftExtrude: a drafted arc's trimmed window exceeds a full circle",
                    ));
                }
                let r_offset = radius - signed_d * turn;
                if r_offset <= tolerance {
                    return Err(KernelRefusal::input(
                        KernelStage::Classify,
                        "arc_collapse",
                        "draftExtrude: offset: distance is too large — a concave arc collapses",
                    ));
                }
                let row_bottom = make_arc(*center, ax, ay, *radius, theta_lo, theta_hi).or_refuse(KernelStage::Fragment, "make_arc")?;
                let row_top =
                    make_arc(center.add(displacement), ax, ay, r_offset, theta_lo, theta_hi).or_refuse(KernelStage::Fragment, "make_arc")?;
                wall_surfaces.push(ruled_between(&row_bottom, &row_top)?);
                wall_kinds.push(WallSurface::Cone);
                bottom_curves.push(arc_window_subrange(&row_bottom, a0, a1)?);
                top_curves.push(arc_window_subrange(&row_top, b0, b1)?);
            }
        }
    }

    // --- 6. Assemble the BREP: shared vertices/edges, wall faces with exact
    //     pcurves (plane projection on plane walls, parameter lines for cone
    //     row edges, on-surface fits for cone side edges), and the two planar
    //     caps — the same topology the straight extrude emits.
    let mut vertices = Vec::with_capacity(2 * count);
    for (index, point) in bottom_junctions.iter().enumerate() {
        vertices.push(VertexRecord {
            id: index as u64 + 1,
            point: *point,
        });
    }
    for (index, point) in top_junctions.iter().enumerate() {
        vertices.push(VertexRecord {
            id: (count + index) as u64 + 1,
            point: *point,
        });
    }
    let mut edges = Vec::with_capacity(3 * count);
    for index in 0..count {
        let [b_start, b_end] = bottom_curves[index].domain().or_refuse(KernelStage::Fragment, "domain")?;
        edges.push(EdgeRecord {
            id: 10 + index as u64,
            curve: bottom_curves[index].clone(),
            t0: b_start,
            t1: b_end,
            start_vertex_id: index as u64 + 1,
            end_vertex_id: ((index + 1) % count) as u64 + 1,
            degenerate: false,
            name: None,
        });
        let [t_start, t_end] = top_curves[index].domain().or_refuse(KernelStage::Fragment, "domain")?;
        edges.push(EdgeRecord {
            id: 10 + count as u64 + index as u64,
            curve: top_curves[index].clone(),
            t0: t_start,
            t1: t_end,
            start_vertex_id: (count + index) as u64 + 1,
            end_vertex_id: (count + (index + 1) % count) as u64 + 1,
            degenerate: false,
            name: None,
        });
        let [s_start, s_end] = side_curves[index].domain().or_refuse(KernelStage::Fragment, "domain")?;
        edges.push(EdgeRecord {
            id: 10 + 2 * count as u64 + index as u64,
            curve: side_curves[index].clone(),
            t0: s_start,
            t1: s_end,
            start_vertex_id: index as u64 + 1,
            end_vertex_id: (count + index) as u64 + 1,
            degenerate: false,
            name: None,
        });
    }

    let mut next_id = 1000_u64;
    let mut faces = Vec::with_capacity(count + 2);
    for index in 0..count {
        let next_index = (index + 1) % count;
        let surface = &wall_surfaces[index];
        // Traversal-order pcurves for [bottom fwd, side_next up, top rev,
        // side_this down] on this wall's surface.
        let (pc_bottom, pc_side_up, pc_top, pc_side_down) = match &wall_kinds[index] {
            WallSurface::Plane { origin, ex, ey } => (
                curve_to_plane_parameters(&bottom_curves[index], *origin, *ex, *ey).or_refuse(KernelStage::Fragment, "plane_parameters")?,
                curve_to_plane_parameters(&side_curves[next_index], *origin, *ex, *ey).or_refuse(KernelStage::Fragment, "plane_parameters")?,
                curve_to_plane_parameters(&top_curves[index], *origin, *ex, *ey).or_refuse(KernelStage::Fragment, "plane_parameters")?.reversed().or_refuse(KernelStage::Fragment, "reversed")?,
                curve_to_plane_parameters(&side_curves[index], *origin, *ex, *ey).or_refuse(KernelStage::Fragment, "plane_parameters")?.reversed().or_refuse(KernelStage::Fragment, "reversed")?,
            ),
            WallSurface::Cone => {
                let [b0, b1] = bottom_curves[index].domain().or_refuse(KernelStage::Fragment, "domain")?;
                let [t0, t1] = top_curves[index].domain().or_refuse(KernelStage::Fragment, "domain")?;
                (
                    parameter_line(b0, 0.0, b1, 0.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
                    build_pcurve_on_surface(surface, &side_curves[next_index]).or_refuse(KernelStage::Fragment, "pcurve")?,
                    parameter_line(t1, 1.0, t0, 1.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
                    build_pcurve_on_surface(surface, &side_curves[index]).or_refuse(KernelStage::Fragment, "pcurve")?.reversed().or_refuse(KernelStage::Fragment, "reversed")?,
                )
            }
        };
        let coedges = vec![
            CoedgeRecord {
                id: next_id,
                edge_id: 10 + index as u64,
                forward: true,
                pcurve: pc_bottom,
            },
            CoedgeRecord {
                id: next_id + 1,
                edge_id: 10 + 2 * count as u64 + next_index as u64,
                forward: true,
                pcurve: pc_side_up,
            },
            CoedgeRecord {
                id: next_id + 2,
                edge_id: 10 + count as u64 + index as u64,
                forward: false,
                pcurve: pc_top,
            },
            CoedgeRecord {
                id: next_id + 3,
                edge_id: 10 + 2 * count as u64 + index as u64,
                forward: false,
                pcurve: pc_side_down,
            },
        ];
        next_id += 4;
        faces.push(FaceRecord {
            id: next_id + 1,
            surface: wall_surfaces[index].clone(),
            same_sense: true,
            loops: vec![LoopRecord {
                id: next_id,
                coedges,
            }],
            name: None,
        });
        next_id += 2;
    }
    if reversed_winding {
        // The winding normalization reversed the curve loop above; callers
        // stamp wall names by INPUT-curve order — emit walls in input order.
        faces.reverse();
    }

    // Caps: planar bbox patches over each section's own footprint.
    let mut cap = |curves: &[NurbsCurve],
                   edge_base: u64,
                   forward: bool,
                   next_id: &mut u64|
     -> Result<FaceRecord, KernelRefusal> {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        let mut plane_point = None;
        for curve in curves {
            let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
            for sample in 0..=16 {
                let point = curve.evaluate(start + (end - start) * sample as f64 / 16.0).or_refuse(KernelStage::Fragment, "evaluate")?;
                let anchor = *plane_point.get_or_insert(point);
                let delta = point.sub(anchor);
                min_x = min_x.min(delta.dot(x_axis));
                max_x = max_x.max(delta.dot(x_axis));
                min_y = min_y.min(delta.dot(y_axis));
                max_y = max_y.max(delta.dot(y_axis));
            }
        }
        let anchor = plane_point.ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Fragment,
                "cap_samples",
                "draftExtrude: cap has no boundary samples",
            )
        })?;
        let padding = (max_x - min_x).max(max_y - min_y) * 0.05 + 1e-6;
        let cap_origin = anchor
            .add(x_axis.scale(min_x - padding))
            .add(y_axis.scale(min_y - padding));
        let mut coedges = Vec::with_capacity(curves.len());
        if forward {
            for (index, curve) in curves.iter().enumerate() {
                coedges.push(CoedgeRecord {
                    id: *next_id,
                    edge_id: edge_base + index as u64,
                    forward: true,
                    pcurve: curve_to_plane_parameters(curve, cap_origin, x_axis, y_axis).or_refuse(KernelStage::Fragment, "plane_parameters")?,
                });
                *next_id += 1;
            }
        } else {
            for index in (0..curves.len()).rev() {
                coedges.push(CoedgeRecord {
                    id: *next_id,
                    edge_id: edge_base + index as u64,
                    forward: false,
                    pcurve: curve_to_plane_parameters(&curves[index], cap_origin, x_axis, y_axis).or_refuse(KernelStage::Fragment, "plane_parameters")?
                        .reversed().or_refuse(KernelStage::Fragment, "reversed")?,
                });
                *next_id += 1;
            }
        }
        let loop_id = *next_id;
        let face_id = *next_id + 1;
        *next_id += 2;
        Ok(FaceRecord {
            id: face_id,
            surface: make_plane(
                cap_origin,
                x_axis,
                y_axis,
                max_x - min_x + 2.0 * padding,
                max_y - min_y + 2.0 * padding,
            ).or_refuse(KernelStage::Fragment, "make_plane")?,
            same_sense: forward,
            loops: vec![LoopRecord {
                id: loop_id,
                coedges,
            }],
            name: None,
        })
    };
    faces.push(cap(&bottom_curves, 10, false, &mut next_id)?);
    faces.push(cap(&top_curves, 10 + count as u64, true, &mut next_id)?);

    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: next_id + 1,
        vertices,
        edges,
        shells: vec![ShellRecord { id: next_id, faces }],
        genus: 0,
    };
    let issues = solid.validate();
    if issues.is_empty() {
        Ok(solid)
    } else {
        Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!("Rust draft-extrude builder produced invalid topology: {issues:?}"),
        ))
    }
}
