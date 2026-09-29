use super::*;

#[derive(Clone)]
struct SectionFrame {
    normal: Vec3,
    centroid: Vec3,
    samples: Vec<Vec3>,
    planar: bool,
}

pub(super) fn closed_points(
    curves: &[NurbsCurve],
    tolerance: f64,
) -> Result<Vec<Vec3>, KernelRefusal> {
    if curves.len() < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "section_curve_count",
            "loftSolid: a section needs at least 2 curves",
        ));
    }
    let mut points = Vec::with_capacity(curves.len());
    for (index, curve) in curves.iter().enumerate() {
        let [start, end] = curve.domain().or_refuse(KernelStage::Collect, "domain")?;
        let next = &curves[(index + 1) % curves.len()];
        let next_start = next.domain().or_refuse(KernelStage::Collect, "domain")?[0];
        if curve
            .evaluate(end)
            .or_refuse(KernelStage::Collect, "evaluate")?
            .sub(
                next.evaluate(next_start)
                    .or_refuse(KernelStage::Collect, "evaluate")?,
            )
            .length()
            > tolerance
        {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "open_section",
                format!("loftSolid: section is open at curve {index}"),
            ));
        }
        points.push(
            curve
                .evaluate(start)
                .or_refuse(KernelStage::Collect, "evaluate")?,
        );
    }
    Ok(points)
}

fn section_frame(curves: &[NurbsCurve], tolerance: f64) -> Result<SectionFrame, KernelRefusal> {
    let mut samples = Vec::new();
    for curve in curves {
        let [start, end] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
        for index in 0..16 {
            samples.push(
                curve
                    .evaluate(start + (end - start) * index as f64 / 16.0)
                    .or_refuse(KernelStage::Classify, "evaluate")?,
            );
        }
    }
    let mut normal = crate::polygon::newell_normal(&samples);
    let mut centroid = samples.iter().fold(Vec3::default(), |sum, &point| sum.add(point));
    // A zero Newell normal is a section without area — the caller's loop, not a
    // solve, so it is an input refusal carrying the vector helper's own text.
    normal = normal
        .normalized()
        .or_input(KernelStage::Classify, "section_normal")?;
    centroid = centroid.scale(1.0 / samples.len() as f64);
    let planar = samples
        .iter()
        .all(|point| point.sub(samples[0]).dot(normal).abs() <= tolerance * 100.0);
    Ok(SectionFrame {
        normal,
        centroid,
        samples,
        planar,
    })
}

/// How far off an end section's plane a step has to lean before it may say which
/// side of that section the loft's material is on. `0.1` — the same band
/// `extrude_profile_brep` refuses a sliver at, and the same one the sweep's own
/// per-station guard uses.
const CAP_ADVANCE_MIN: f64 = 0.1;

/// Find the inward advance direction at an end section, walking nearest first.
/// Skip centroids in the end plane until `|step·normal| / |step|` reaches
/// `CAP_ADVANCE_MIN`. This handles turning lofts and guides that initially
/// travel sideways; an end-to-end chord alone can misorient their caps.
/// Return `None` if the entire run stays in the end plane.
fn advance_from(
    sections: &[Vec<NurbsCurve>],
    anchor: Vec3,
    normal: Vec3,
    order: impl Iterator<Item = usize>,
    tolerance: f64,
) -> Result<Option<Vec3>, KernelRefusal> {
    for index in order {
        let step = section_frame(&sections[index], tolerance)?
            .centroid
            .sub(anchor);
        let length = step.length();
        if length > tolerance && (step.dot(normal) / length).abs() >= CAP_ADVANCE_MIN {
            return Ok(Some(step.scale(1.0 / length)));
        }
    }
    Ok(None)
}

fn reverse_section(curves: &[NurbsCurve]) -> Result<Vec<NurbsCurve>, KernelRefusal> {
    curves
        .iter()
        .rev()
        .map(NurbsCurve::reversed)
        .collect::<Result<Vec<_>, String>>()
        .or_refuse(KernelStage::Classify, "reverse_section")
}

/// One end cap. `advance` is the direction the loft RUNS THROUGH this cap —
/// measured at this end, not end to end (see `advance_from`) — and it orients
/// the cap plane: `outward` then says whether the face's own normal follows it
/// (the finish cap) or opposes it (the start cap).
fn cap_face(
    curves: &[NurbsCurve],
    edge_ids: &[u64],
    frame: &SectionFrame,
    advance: Vec3,
    outward: bool,
    next_id: &mut u64,
) -> Result<FaceRecord, KernelRefusal> {
    let normal = if frame.normal.dot(advance) >= 0.0 {
        frame.normal
    } else {
        frame.normal.scale(-1.0)
    };
    let x_axis = normal
        .perpendicular()
        .or_refuse(KernelStage::Fragment, "cap_frame")?;
    let y_axis = normal
        .cross(x_axis)
        .normalized()
        .or_refuse(KernelStage::Fragment, "cap_frame")?;
    let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
    let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for point in &frame.samples {
        let delta = point.sub(frame.centroid);
        min_x = min_x.min(delta.dot(x_axis));
        max_x = max_x.max(delta.dot(x_axis));
        min_y = min_y.min(delta.dot(y_axis));
        max_y = max_y.max(delta.dot(y_axis));
    }
    let padding = (max_x - min_x).max(max_y - min_y) * 0.05 + 1e-6;
    let origin = frame
        .centroid
        .add(x_axis.scale(min_x - padding))
        .add(y_axis.scale(min_y - padding));
    let surface = make_plane(
        origin,
        x_axis,
        y_axis,
        max_x - min_x + 2.0 * padding,
        max_y - min_y + 2.0 * padding,
    )
    .or_refuse(KernelStage::Fragment, "cap_plane")?;
    let mut coedges = Vec::with_capacity(curves.len());
    if outward {
        for (index, curve) in curves.iter().enumerate() {
            coedges.push(CoedgeRecord {
                id: *next_id,
                edge_id: edge_ids[index],
                forward: true,
                pcurve: curve_to_plane_parameters(curve, origin, x_axis, y_axis)
                    .or_refuse(KernelStage::Fragment, "cap_pcurve")?,
            });
            *next_id += 1;
        }
    } else {
        for index in (0..curves.len()).rev() {
            coedges.push(CoedgeRecord {
                id: *next_id,
                edge_id: edge_ids[index],
                forward: false,
                pcurve: curve_to_plane_parameters(&curves[index], origin, x_axis, y_axis)
                    .or_refuse(KernelStage::Fragment, "cap_pcurve")?
                    .reversed()
                    .or_refuse(KernelStage::Fragment, "cap_pcurve")?,
            });
            *next_id += 1;
        }
    }
    let loop_id = *next_id;
    *next_id += 1;
    let face_id = *next_id;
    *next_id += 1;
    Ok(FaceRecord {
        id: face_id,
        surface,
        same_sense: outward,
        loops: vec![LoopRecord {
            id: loop_id,
            coedges,
        }],
        name: None,
    })
}

pub fn loft_profile_brep(input_sections: &[Vec<NurbsCurve>]) -> Result<BrepSolid, KernelRefusal> {
    loft_profile_brep_core(input_sections, None)
}

/// §5.8 loft with END TANGENCY: the skin leaves the first section along
/// `start_direction` and arrives at the last along `end_direction` (unit
/// directions; each interpolation column scales them by its own chord length,
/// the standard magnitude that keeps the v-parametrization well conditioned).
/// Exact by construction — the column interpolant reproduces the prescribed
/// end derivatives.
pub fn loft_profile_brep_tangent(
    input_sections: &[Vec<NurbsCurve>],
    start_direction: Vec3,
    end_direction: Vec3,
) -> Result<BrepSolid, KernelRefusal> {
    let start = start_direction.normalized().map_err(|_| {
        KernelRefusal::input(
            KernelStage::Collect,
            "start_tangent",
            "loftSolid: start tangent must be a nonzero direction",
        )
    })?;
    let end = end_direction.normalized().map_err(|_| {
        KernelRefusal::input(
            KernelStage::Collect,
            "end_tangent",
            "loftSolid: end tangent must be a nonzero direction",
        )
    })?;
    loft_profile_brep_core(input_sections, Some((start, end)))
}

fn loft_profile_brep_core(
    input_sections: &[Vec<NurbsCurve>],
    end_tangents: Option<(Vec3, Vec3)>,
) -> Result<BrepSolid, KernelRefusal> {
    let tolerance = 1e-6;
    let section_count = input_sections.len();
    if section_count < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "section_count",
            "loftSolid: need at least 2 sections",
        ));
    }
    let mut sections = input_sections.to_vec();
    let curve_count = validate_sections(&sections, tolerance, "loftSolid", true)?;

    let first_frame = section_frame(&sections[0], tolerance)?;
    let last_frame = section_frame(&sections[section_count - 1], tolerance)?;
    if !first_frame.planar || !last_frame.planar {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "planar_end_sections",
            "loftSolid: end sections must be planar",
        ));
    }
    // Ends in the same place put the two caps on top of each other whatever the
    // run does in between; say that before blaming a section plane below.
    last_frame
        .centroid
        .sub(first_frame.centroid)
        .normalized()
        .map_err(|_| {
            KernelRefusal::input(
                KernelStage::Classify,
                "coincident_ends",
                "loftSolid: end sections coincide",
            )
        })?;
    // Each cap must face away from its own inward advance direction; a turning
    // loft can leave one end edge-on even when the other end is valid.
    let start_advance = advance_from(
        &sections,
        first_frame.centroid,
        first_frame.normal,
        1..section_count,
        tolerance,
    )?
    .ok_or_else(|| {
        KernelRefusal::input(
            KernelStage::Classify,
            "start_in_plane",
            "loftSolid: the loft runs inside its START section's plane — every later section \
             lies in it, so the start cap would be a sliver rather than a face",
        )
    })?;
    // Walking inward from the FAR end measures backwards, so negate it: both
    // directions point the way the loft runs, start to finish.
    let finish_advance = advance_from(
        &sections,
        last_frame.centroid,
        last_frame.normal,
        (0..section_count - 1).rev(),
        tolerance,
    )?
    .map(|step| step.scale(-1.0))
    .ok_or_else(|| {
        KernelRefusal::input(
            KernelStage::Classify,
            "end_in_plane",
            "loftSolid: the loft runs inside its END section's plane — every earlier section \
             lies in it, so the end cap would be a sliver rather than a face",
        )
    })?;
    let first_normal = if first_frame.normal.dot(start_advance) >= 0.0 {
        first_frame.normal
    } else {
        first_frame.normal.scale(-1.0)
    };
    let x_axis = first_normal
        .perpendicular()
        .or_refuse(KernelStage::Classify, "section_frame")?;
    let y_axis = first_normal
        .cross(x_axis)
        .normalized()
        .or_refuse(KernelStage::Classify, "section_frame")?;
    // Winding is normalized PER SECTION against this one frame, not decided once
    // from section 0 and applied to every section.
    //
    // The skin rails control point `k` of curve `i` of one section to the same
    // `(i, k)` of the next, so two sections traversed OPPOSITE ways rail corner
    // to opposite corner and the skin twists into a bow-tie. Deciding one flip
    // from section 0 and applying it to all of them cannot see that: it reverses
    // them together, which preserves the disagreement exactly.
    //
    // Measured before this changed (`examples/feature_refusal_census_probe.rs`,
    // `GATE loft_profile_brep.opposed_winding`): two 2x2 squares 5 apart, wound
    // opposite ways, lofted to a solid of **volume 0 that passes `validate()`**
    // — a silently-wrong result, not a refusal. The shipping LOFT feature
    // reached it with two ordinary sketches on opposed planes.
    //
    // The comparison is CHAIN-RELATIVE — each section against its PREDECESSOR,
    // not against section 0.
    //
    // Measuring every section against section 0's frame looks equivalent and is
    // not: a frame-guided loft rotates each station's plane with the guide, so
    // on a bend past 90 degrees a station's projected area goes negative purely
    // because its plane has turned (cos > 90 deg < 0), and reversing it un-does
    // the guide's own rotation. Measured on a 150-degree arc: volume 17.5685
    // against a Pappus-exact 104.7198, a 83 % error, where the predecessor rule
    // is exact.
    //
    // Consecutive stations of any sane loft turn far less than 90 degrees, so
    // the predecessor comparison leaves a bend alone, while a genuinely opposed
    // pair still flips. It also removes the knife edge at exactly 90 degrees,
    // where a fixed-frame projection is float noise.
    //
    // `section_frame(..).normal` is the Newell normal, whose ORIENTATION already
    // encodes the traversal direction; only its sign is read. Section 0 keeps
    // the original rule verbatim, so its behaviour — and the side-face
    // permutation below, which keys off section 0 alone — is byte-identical.
    let mut section_reversed = Vec::with_capacity(section_count);
    section_reversed.push(
        profile_area(&sections[0], first_frame.centroid, x_axis, y_axis)
            .or_refuse(KernelStage::Classify, "profile_area")?
            < 0.0,
    );
    let mut previous_normal = if section_reversed[0] {
        first_frame.normal.scale(-1.0)
    } else {
        first_frame.normal
    };
    for section in sections.iter().skip(1) {
        let normal = section_frame(section, tolerance)?.normal;
        let flip = normal.dot(previous_normal) < 0.0;
        section_reversed.push(flip);
        previous_normal = if flip { normal.scale(-1.0) } else { normal };
    }
    // Side faces are named by the FIRST section's input-curve order, so the
    // face permutation below still keys off section 0 alone.
    let reversed_winding = section_reversed[0];
    for (index, flip) in section_reversed.into_iter().enumerate() {
        if flip {
            sections[index] = reverse_section(&sections[index])?;
        }
    }

    let mut parameters = vec![0.0; section_count];
    let mut accumulated = vec![0.0; section_count];
    let mut columns = 0usize;
    for curve_index in 0..curve_count {
        for control_index in 0..sections[0][curve_index].control_points.len() {
            let mut total = 0.0;
            let mut chords = vec![0.0; section_count];
            for section_index in 1..section_count {
                let previous = sections[section_index - 1][curve_index].control_points
                    [control_index]
                    .point()
                    .or_refuse(KernelStage::Refine, "control_point")?;
                let current = sections[section_index][curve_index].control_points[control_index]
                    .point()
                    .or_refuse(KernelStage::Refine, "control_point")?;
                total += current.sub(previous).length();
                chords[section_index] = total;
            }
            if total <= tolerance {
                continue;
            }
            for section_index in 0..section_count {
                accumulated[section_index] += chords[section_index] / total;
            }
            columns += 1;
        }
    }
    if columns == 0 {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "coincident_sections",
            "loftSolid: sections coincide",
        ));
    }
    for index in 0..section_count {
        parameters[index] = accumulated[index] / columns as f64;
    }
    parameters[0] = 0.0;
    parameters[section_count - 1] = 1.0;
    if parameters.windows(2).any(|pair| pair[1] <= pair[0] + 1e-9) {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "section_order",
            "loftSolid: sections are not strictly ordered",
        ));
    }
    // Tangent lofts always interpolate cubically — the end-derivative rows
    // need the two extra control points even for a 2-section Hermite loft.
    let degree_v = if end_tangents.is_some() {
        3
    } else {
        3usize.min(section_count - 1)
    };
    let mut skins = Vec::with_capacity(curve_count);
    for curve_index in 0..curve_count {
        let reference = &sections[0][curve_index];
        let mut grid = Vec::with_capacity(reference.control_points.len());
        let mut knots_v = Vec::new();
        for control_index in 0..reference.control_points.len() {
            let points = sections
                .iter()
                .map(|section| section[curve_index].control_points[control_index].point())
                .collect::<Result<Vec<_>, String>>()
                .or_refuse(KernelStage::Refine, "control_point")?;
            let interpolated = match end_tangents {
                None => interpolate_curve(&points, degree_v, &parameters)
                    .or_refuse(KernelStage::Refine, "interpolate")?,
                Some((start_direction, end_direction)) => {
                    let chord: f64 = points
                        .windows(2)
                        .map(|pair| pair[1].sub(pair[0]).length())
                        .sum();
                    // A column whose stations coincide still needs a usable
                    // tangent magnitude; fall back to the section spacing.
                    let magnitude = if chord > tolerance { chord } else { 1.0 };
                    crate::interpolate_curve_with_end_tangents(
                        &points,
                        &parameters,
                        start_direction.scale(magnitude),
                        end_direction.scale(magnitude),
                    )
                    .or_refuse(KernelStage::Refine, "interpolate_tangent")?
                }
            };
            knots_v = interpolated.knots.clone();
            let weight = reference.control_points[control_index].w;
            grid.push(
                interpolated
                    .control_points
                    .iter()
                    .map(|point| Vec4::from_point(point.point().unwrap(), weight))
                    .collect(),
            );
        }
        skins.push(NurbsSurface::new(
            reference.degree,
            degree_v,
            reference.knots.clone(),
            knots_v,
            grid,
        )
        .or_refuse(KernelStage::Fragment, "skin")?);
    }

    let bottom = &sections[0];
    let top = &sections[section_count - 1];
    let bottom_points = closed_points(bottom, tolerance)?;
    let top_points = closed_points(top, tolerance)?;
    let mut vertices = Vec::with_capacity(2 * curve_count);
    for (index, point) in bottom_points.iter().chain(&top_points).enumerate() {
        vertices.push(VertexRecord {
            id: index as u64 + 1,
            point: *point,
        });
    }
    let mut edges = Vec::with_capacity(3 * curve_count);
    let mut bottom_edge_ids = Vec::new();
    let mut top_edge_ids = Vec::new();
    let mut vertical_edge_ids = Vec::new();
    for index in 0..curve_count {
        let [start, end] = bottom[index]
            .domain()
            .or_refuse(KernelStage::Fragment, "domain")?;
        // The TOP curve's own range, which is NOT the bottom's once the winding
        // normalization has reversed one section's curve ORDER: `reverse_section`
        // reverses the order and `reversed()` preserves each curve's domain, so
        // `top[i]` becomes a curve that lived on a different parameter span.
        // Stamping the bottom's span on it made `coedge_sample` evaluate the top
        // curve outside its own domain — pinned at an end while the skin swept
        // the real arc — which on a circular section split into two half-arcs is
        // a deviation of exactly the diameter.
        let [top_start, top_end] = top[index]
            .domain()
            .or_refuse(KernelStage::Fragment, "domain")?;
        let bottom_id = 10 + index as u64;
        let top_id = 10 + curve_count as u64 + index as u64;
        let vertical_id = 10 + 2 * curve_count as u64 + index as u64;
        bottom_edge_ids.push(bottom_id);
        top_edge_ids.push(top_id);
        vertical_edge_ids.push(vertical_id);
        edges.push(EdgeRecord {
            id: bottom_id,
            curve: bottom[index].clone(),
            t0: start,
            t1: end,
            start_vertex_id: index as u64 + 1,
            end_vertex_id: ((index + 1) % curve_count) as u64 + 1,
            degenerate: false,
            name: None,
        });
        edges.push(EdgeRecord {
            id: top_id,
            curve: top[index].clone(),
            t0: top_start,
            t1: top_end,
            start_vertex_id: (curve_count + index) as u64 + 1,
            end_vertex_id: (curve_count + (index + 1) % curve_count) as u64 + 1,
            degenerate: false,
            name: None,
        });
        let vertical = skins[index]
            .iso_curve_u(start)
            .or_refuse(KernelStage::Fragment, "iso_curve")?;
        let [v_start, v_end] = vertical
            .domain()
            .or_refuse(KernelStage::Fragment, "domain")?;
        edges.push(EdgeRecord {
            id: vertical_id,
            curve: vertical,
            t0: v_start,
            t1: v_end,
            start_vertex_id: index as u64 + 1,
            end_vertex_id: (curve_count + index) as u64 + 1,
            degenerate: false,
            name: None,
        });
    }

    let mut next_id = 1000u64;
    let mut faces = Vec::with_capacity(curve_count + 2);
    for index in 0..curve_count {
        let [start, end] = bottom[index]
            .domain()
            .or_refuse(KernelStage::Fragment, "domain")?;
        let coedges = vec![
            CoedgeRecord {
                id: next_id,
                edge_id: bottom_edge_ids[index],
                forward: true,
                pcurve: parameter_line(start, 0.0, end, 0.0)
                    .or_refuse(KernelStage::Fragment, "pcurve")?,
            },
            CoedgeRecord {
                id: next_id + 1,
                edge_id: vertical_edge_ids[(index + 1) % curve_count],
                forward: true,
                pcurve: parameter_line(end, 0.0, end, 1.0)
                    .or_refuse(KernelStage::Fragment, "pcurve")?,
            },
            CoedgeRecord {
                id: next_id + 2,
                edge_id: top_edge_ids[index],
                forward: false,
                pcurve: parameter_line(end, 1.0, start, 1.0)
                    .or_refuse(KernelStage::Fragment, "pcurve")?,
            },
            CoedgeRecord {
                id: next_id + 3,
                edge_id: vertical_edge_ids[index],
                forward: false,
                pcurve: parameter_line(start, 1.0, start, 0.0)
                    .or_refuse(KernelStage::Fragment, "pcurve")?,
            },
        ];
        next_id += 4;
        let loop_id = next_id;
        next_id += 1;
        let face_id = next_id;
        next_id += 1;
        faces.push(FaceRecord {
            id: face_id,
            surface: skins[index].clone(),
            same_sense: true,
            loops: vec![LoopRecord {
                id: loop_id,
                coedges,
            }],
            name: None,
        });
    }
    if reversed_winding {
        // The winding normalization reversed the section curve order above.
        // Callers stamp side-face names by INPUT-curve order (the extrude
        // and revolve wall-naming permutation, loft edition) — emit side
        // faces in input order.
        faces.reverse();
    }
    faces.push(cap_face(
        bottom,
        &bottom_edge_ids,
        &first_frame,
        start_advance,
        false,
        &mut next_id,
    )?);
    faces.push(cap_face(
        top,
        &top_edge_ids,
        &last_frame,
        finish_advance,
        true,
        &mut next_id,
    )?);
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: next_id + 1,
        vertices,
        edges,
        shells: vec![ShellRecord { id: next_id, faces }],
        genus: 0,
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!(
                "Rust loft builder produced invalid topology: {issues:?}"
            ),
        ));
    }
    // `validate()` is an INCIDENCE test: a loft whose walls pass through each
    // other — sections that cross, a run tighter than the profile's own
    // half-width — satisfies every one of its checks. This is the soundness
    // question asked at the lane's single exit, repaired where it can be and
    // refused by name where it cannot. Every path sweep ends here too.
    crate::accept_sound(solid, "loftSolid")
}
