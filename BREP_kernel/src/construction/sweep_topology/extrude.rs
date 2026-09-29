use super::*;

pub(super) fn translated_curve(curve: &NurbsCurve, delta: Vec3) -> Result<NurbsCurve, KernelRefusal> {
    NurbsCurve::new(
        curve.degree,
        curve.knots.clone(),
        curve
            .control_points
            .iter()
            .map(|point| Vec4 {
                x: point.x + point.w * delta.x,
                y: point.y + point.w * delta.y,
                z: point.z + point.w * delta.z,
                w: point.w,
            })
            .collect(),
    )
    .or_refuse(KernelStage::Fragment, "curve_new")
}

pub(crate) fn profile_area(
    curves: &[NurbsCurve],
    origin: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
) -> Result<f64, String> {
    let mut area = 0.0;
    for curve in curves {
        let [start, end] = curve.domain()?;
        let mut previous = curve.evaluate(start)?;
        for index in 1..=64 {
            let point = curve.evaluate(start + (end - start) * index as f64 / 64.0)?;
            let a = previous.sub(origin);
            let b = point.sub(origin);
            area += 0.5 * (a.dot(x_axis) * b.dot(y_axis) - b.dot(x_axis) * a.dot(y_axis));
            previous = point;
        }
    }
    Ok(area)
}

pub fn extrude_profile_brep(
    input_curves: &[NurbsCurve],
    direction: Vec3,
    distance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    if input_curves.len() < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_count",
            "profile needs at least 2 curves",
        ));
    }
    if distance.abs() <= 1e-12 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "distance",
            "extrudeSolid: distance must be non-zero",
        ));
    }
    let displacement = direction
        .normalized()
        .or_input(KernelStage::Collect, "direction")?
        .scale(distance);
    let tolerance = 1e-6;
    let mut curves: Vec<NurbsCurve> = input_curves
        .iter()
        .map(|curve| {
            NurbsCurve::new(
                curve.degree,
                curve.knots.clone(),
                curve.control_points.clone(),
            )
        })
        .collect::<Result<_, _>>().or_refuse(KernelStage::Fragment, "collect")?;

    let mut samples = Vec::new();
    for (index, curve) in curves.iter().enumerate() {
        let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
        let next = &curves[(index + 1) % curves.len()];
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
                format!("profile not closed at curve {index}"),
            ));
        }
        for sample in 0..16 {
            samples.push(curve.evaluate(start + (end - start) * sample as f64 / 16.0).or_refuse(KernelStage::Fragment, "evaluate")?);
        }
    }
    let mut normal = crate::polygon::newell_normal(&samples);
    // A zero Newell normal is a profile with no enclosed area: the caller's.
    normal = normal.normalized().or_input(KernelStage::Collect, "profile_area")?;
    if normal.dot(displacement) < 0.0 {
        normal = normal.scale(-1.0);
    }
    if normal.dot(displacement.normalized().or_refuse(KernelStage::Fragment, "normalized")?).abs() < 0.1 {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "sliver",
            "extrudeSolid: sweep direction is nearly parallel to profile plane",
        ));
    }
    let origin = samples[0];
    if samples
        .iter()
        .any(|point| point.sub(origin).dot(normal).abs() > tolerance * 100.0)
    {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "profile_planar",
            "extrudeSolid: profile is not planar",
        ));
    }
    let x_axis = normal.perpendicular().or_refuse(KernelStage::Fragment, "perpendicular")?;
    let y_axis = normal.cross(x_axis).normalized().or_refuse(KernelStage::Fragment, "normalized")?;
    let reversed_winding = profile_area(&curves, origin, x_axis, y_axis).or_refuse(KernelStage::Fragment, "profile_area")? < 0.0;
    if reversed_winding {
        curves = curves
            .iter()
            .rev()
            .map(NurbsCurve::reversed)
            .collect::<Result<_, _>>().or_refuse(KernelStage::Fragment, "collect")?;
    }

    let count = curves.len();
    let points: Vec<Vec3> = curves
        .iter()
        .map(|curve| curve.domain().and_then(|domain| curve.evaluate(domain[0])))
        .collect::<Result<_, _>>().or_refuse(KernelStage::Fragment, "collect")?;
    let mut vertices = Vec::with_capacity(count * 2);
    for (index, point) in points.iter().enumerate() {
        vertices.push(VertexRecord {
            id: index as u64 + 1,
            point: *point,
        });
    }
    for (index, point) in points.iter().enumerate() {
        vertices.push(VertexRecord {
            id: (count + index) as u64 + 1,
            point: point.add(displacement),
        });
    }

    let mut edges = Vec::with_capacity(count * 3);
    for (index, curve) in curves.iter().enumerate() {
        let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
        edges.push(EdgeRecord {
            id: 10 + index as u64,
            curve: curve.clone(),
            t0: start,
            t1: end,
            start_vertex_id: index as u64 + 1,
            end_vertex_id: ((index + 1) % count) as u64 + 1,
            degenerate: false,
            name: None,
        });
        edges.push(EdgeRecord {
            id: 10 + count as u64 + index as u64,
            curve: translated_curve(curve, displacement)?,
            t0: start,
            t1: end,
            start_vertex_id: (count + index) as u64 + 1,
            end_vertex_id: (count + (index + 1) % count) as u64 + 1,
            degenerate: false,
            name: None,
        });
        edges.push(EdgeRecord {
            id: 10 + 2 * count as u64 + index as u64,
            curve: make_line(points[index], points[index].add(displacement)).or_refuse(KernelStage::Fragment, "make_line")?,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: index as u64 + 1,
            end_vertex_id: (count + index) as u64 + 1,
            degenerate: false,
            name: None,
        });
    }

    let mut next_id = 1000_u64;
    let mut faces = Vec::with_capacity(count + 2);
    for (index, curve) in curves.iter().enumerate() {
        let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
        let coedges = vec![
            CoedgeRecord {
                id: next_id,
                edge_id: 10 + index as u64,
                forward: true,
                pcurve: parameter_line(start, 0.0, end, 0.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
            },
            CoedgeRecord {
                id: next_id + 1,
                edge_id: 10 + 2 * count as u64 + ((index + 1) % count) as u64,
                forward: true,
                pcurve: parameter_line(end, 0.0, end, 1.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
            },
            CoedgeRecord {
                id: next_id + 2,
                edge_id: 10 + count as u64 + index as u64,
                forward: false,
                pcurve: parameter_line(end, 1.0, start, 1.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
            },
            CoedgeRecord {
                id: next_id + 3,
                edge_id: 10 + 2 * count as u64 + index as u64,
                forward: false,
                pcurve: parameter_line(start, 1.0, start, 0.0).or_refuse(KernelStage::Fragment, "parameter_line")?,
            },
        ];
        next_id += 4;
        faces.push(FaceRecord {
            id: next_id + 1,
            surface: make_extrusion(curve, displacement).or_refuse(KernelStage::Fragment, "make_extrusion")?,
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
        // The winding normalization reversed the curve loop above.  Callers
        // name side faces by INPUT-curve order (the app stamps names onto
        // kernel faces in listed order), so a silently reordered face list
        // permutes every wall name — an arc's wall ends up carrying a
        // neighboring line's name.  Emit side faces in input order.
        faces.reverse();
    }

    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for point in &samples {
        let delta = point.sub(origin);
        min_x = min_x.min(delta.dot(x_axis));
        max_x = max_x.max(delta.dot(x_axis));
        min_y = min_y.min(delta.dot(y_axis));
        max_y = max_y.max(delta.dot(y_axis));
    }
    let padding = (max_x - min_x).max(max_y - min_y) * 0.05 + 1e-6;
    let width = max_x - min_x + 2.0 * padding;
    let height = max_y - min_y + 2.0 * padding;
    let bottom_origin = origin
        .add(x_axis.scale(min_x - padding))
        .add(y_axis.scale(min_y - padding));
    let mut bottom_coedges = Vec::with_capacity(count);
    for index in (0..count).rev() {
        bottom_coedges.push(CoedgeRecord {
            id: next_id,
            edge_id: 10 + index as u64,
            forward: false,
            pcurve: curve_to_plane_parameters(&curves[index], bottom_origin, x_axis, y_axis).or_refuse(KernelStage::Fragment, "plane_parameters")?
                .reversed().or_refuse(KernelStage::Fragment, "reversed")?,
        });
        next_id += 1;
    }
    faces.push(FaceRecord {
        id: next_id + 1,
        surface: make_plane(bottom_origin, x_axis, y_axis, width, height).or_refuse(KernelStage::Fragment, "make_plane")?,
        same_sense: false,
        loops: vec![LoopRecord {
            id: next_id,
            coedges: bottom_coedges,
        }],
        name: None,
    });
    next_id += 2;

    let top_origin = bottom_origin.add(displacement);
    let mut top_coedges = Vec::with_capacity(count);
    for (index, curve) in curves.iter().enumerate() {
        let translated = translated_curve(curve, displacement)?;
        top_coedges.push(CoedgeRecord {
            id: next_id,
            edge_id: 10 + count as u64 + index as u64,
            forward: true,
            pcurve: curve_to_plane_parameters(&translated, top_origin, x_axis, y_axis).or_refuse(KernelStage::Fragment, "plane_parameters")?,
        });
        next_id += 1;
    }
    faces.push(FaceRecord {
        id: next_id + 1,
        surface: make_plane(top_origin, x_axis, y_axis, width, height).or_refuse(KernelStage::Fragment, "make_plane")?,
        same_sense: true,
        loops: vec![LoopRecord {
            id: next_id,
            coedges: top_coedges,
        }],
        name: None,
    });
    next_id += 2;

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
            format!("Rust extrusion builder produced invalid topology: {issues:?}"),
        ))
    }
}
