//! Recognize a rectangular strip spanning one closed surface direction.
//! Its two meridian boundaries are chart seams, not material boundaries.
use super::*;

pub(super) fn seams(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
) -> Result<Option<[usize; 2]>, KernelRefusal> {
    let (closed_u, closed_v) = surface
        .closed_directions()
        .or_refuse(KernelStage::Collect, "closed_directions")?;
    // A doubly periodic carrier still uses the existing closed-sheet refusal.
    // This lane handles one pair of chart seams, with an open meridian.
    if !closed_u || closed_v || loops.len() != 1 || loops[0].len() != 4 {
        return Ok(None);
    }
    let [u0, u1] = surface
        .domain_u()
        .or_refuse(KernelStage::Collect, "domain_u")?;
    let [v0, v1] = surface
        .domain_v()
        .or_refuse(KernelStage::Collect, "domain_v")?;
    let eps_u = 1e-10 * (u1 - u0);
    let eps_v = 1e-10 * (v1 - v0);
    let mut points = Vec::new();
    let mut ends = Vec::new();
    for curve in &loops[0] {
        if curve.degree != 1
            || curve.control_points.len() != 2
            || (curve.control_points[0].w - curve.control_points[1].w).abs() > 1e-12
        {
            return Ok(None);
        }
        ends.push(
            curve.control_points[1]
                .point()
                .or_refuse(KernelStage::Collect, "point")?,
        );
        points.push(
            curve.control_points[0]
                .point()
                .or_refuse(KernelStage::Collect, "point")?,
        );
    }
    let low = points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let high = points.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);
    if low < v0 - eps_v || high > v1 + eps_v || high - low <= eps_v {
        return Ok(None);
    }
    let corners = [
        Vec3::new(u0, low, 0.0),
        Vec3::new(u1, low, 0.0),
        Vec3::new(u1, high, 0.0),
        Vec3::new(u0, high, 0.0),
    ];
    let matches = |p: Vec3, corner: Vec3| {
        (p.x - corner.x).abs() <= eps_u
            && (p.y - corner.y).abs() <= eps_v
            && p.z.abs() <= eps_u.max(eps_v)
    };
    let Some(shift) = (0..4).find(|shift| {
        (0..4).all(|i| {
            matches(points[i], corners[(i + shift) % 4])
                && matches(ends[i], corners[(i + shift + 1) % 4])
        })
    }) else {
        return Ok(None);
    };
    Ok(Some([(1 + 4 - shift) % 4, (3 + 4 - shift) % 4]))
}

pub(super) fn build(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    bottom_distance: f64,
    top_distance: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    if seams(surface, loops)?.is_none() {
        return Ok(None);
    }
    let [u0, u1] = surface
        .domain_u()
        .or_refuse(KernelStage::Collect, "domain_u")?;
    let [v0, v1] = surface
        .domain_v()
        .or_refuse(KernelStage::Collect, "domain_v")?;
    validate_trim_loops(
        loops,
        1e-7 * (u1 - u0).max(v1 - v0),
        1e-10 * (u1 - u0) * (v1 - v0),
    )?;
    // Keep the fold lane's named refusals; shared seams cannot repair a
    // surface offset through its centre of curvature.
    ensure_offsets_regular(surface, loops, &[bottom_distance, top_distance])?;
    if closed_poles_on() {
        if let Some(solid) = closed_poles_body(surface, loops, bottom_distance, top_distance)? {
            return Ok(Some(solid));
        }
    }
    thicken_one_region(surface, loops, bottom_distance, top_distance).map(Some)
}

/// `BREP_THICKEN_CLOSED_POLES` (default on; `=0` restores): a whole sheet
/// closed round its seam and at a pole at each end (a sphere's skin) thickens
/// to the body between its two offset sheets, each a closed shell of its own,
/// one of them a void. Without it the strip's two pole boundaries got ruled
/// walls of zero area on the axis, pinching both sheets into ONE shell whose
/// sphere faces' loops (seam, pole, seam, pole) a standard reader cannot place:
/// brep-bench's STEP measurer read thicken/sphere-skin's file as the inner
/// sphere alone, inside out (volume -523.6 against 381.2).
fn closed_poles_on() -> bool {
    switch!("BREP_THICKEN_CLOSED_POLES", true)
}

/// The body between the two offset sheets of a strip that covers the whole of
/// a sheet closed in u whose v ends are both poles, or None for any other
/// strip. Each offset sheet is one face on the whole parameter rectangle with
/// the topology of [`crate::make_sphere_brep`]'s: a seam edge at u0 and a
/// degenerate edge at each pole. The top sheet (the larger offset) faces along
/// the sheet's normal and the bottom one against it, so both face away from
/// the material between them; whichever encloses the other is the outer shell.
fn closed_poles_body(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    bottom_distance: f64,
    top_distance: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Collect, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Collect, "domain_v")?;
    // The strip must cover the whole v range: its loop's v extent is [v0, v1].
    let eps_v = 1e-10 * (v1 - v0);
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for curve in &loops[0] {
        for control in &curve.control_points {
            let point = control.point().or_refuse(KernelStage::Collect, "point")?;
            low = low.min(point.y);
            high = high.max(point.y);
        }
    }
    if (low - v0).abs() > eps_v || (high - v1).abs() > eps_v {
        return Ok(None);
    }
    // Each sheet must close at a point at both ends: nine samples along each
    // pole row within a billionth of the sheet's size of the first. The source
    // sheet is read first, so a strip with an open end (a cylinder's, a
    // cone's) costs no offset.
    let is_pole_row = |sheet: &NurbsSurface, v: f64| -> Result<bool, KernelRefusal> {
        let first = sheet.evaluate(u0, v).or_refuse(KernelStage::Collect, "evaluate")?;
        let mut size = 0.0f64;
        for control in sheet.control_points.iter().flatten() {
            size = size.max(control.point().or_refuse(KernelStage::Collect, "point")?.length());
        }
        let band = 1e-9 * size.max(1.0);
        for index in 1..=8 {
            let u = u0 + (u1 - u0) * index as f64 / 8.0;
            if sheet.evaluate(u, v).or_refuse(KernelStage::Collect, "evaluate")?.sub(first).length() > band {
                return Ok(false);
            }
        }
        Ok(true)
    };
    if !(is_pole_row(surface, v0)? && is_pole_row(surface, v1)?) {
        return Ok(None);
    }
    let bottom = offset_sheet(surface, loops, bottom_distance)?;
    let top = offset_sheet(surface, loops, top_distance)?;
    for sheet in [&bottom, &top] {
        if !(is_pole_row(sheet, v0)? && is_pole_row(sheet, v1)?) {
            return Ok(None);
        }
    }
    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let mut shells = Vec::new();
    let mut next_id = 1u64;
    for (sheet, outward) in [(top, true), (bottom, false)] {
        let mut id = || {
            next_id += 1;
            next_id - 1
        };
        let south = sheet.evaluate(u0, v0).or_refuse(KernelStage::Refine, "evaluate")?;
        let north = sheet.evaluate(u0, v1).or_refuse(KernelStage::Refine, "evaluate")?;
        let (south_id, north_id) = (id(), id());
        vertices.push(VertexRecord { id: south_id, point: south });
        vertices.push(VertexRecord { id: north_id, point: north });
        let seam = sheet.iso_curve_u(u0).or_refuse(KernelStage::Refine, "iso_curve_u")?;
        let [s0, s1] = seam.domain().or_refuse(KernelStage::Refine, "domain")?;
        let (seam_id, south_edge, north_edge) = (id(), id(), id());
        edges.push(EdgeRecord {
            id: seam_id,
            curve: seam,
            t0: s0,
            t1: s1,
            start_vertex_id: south_id,
            end_vertex_id: north_id,
            degenerate: false,
            name: None,
        });
        for (edge_id, point, vertex_id) in [(south_edge, south, south_id), (north_edge, north, north_id)] {
            edges.push(EdgeRecord {
                id: edge_id,
                curve: make_line(point, point).or_refuse(KernelStage::Refine, "make_line")?,
                t0: 0.0,
                t1: 1.0,
                start_vertex_id: vertex_id,
                end_vertex_id: vertex_id,
                degenerate: true,
                name: None,
            });
        }
        let line = |a: (f64, f64), b: (f64, f64)| parameter_line(a.0, a.1, b.0, b.1).or_refuse(KernelStage::Refine, "parameter_line");
        // Anticlockwise in (u, v) for a face along the sheet's normal (as the
        // sphere primitive's), clockwise for one against it.
        let coedges = if outward {
            vec![
                (south_edge, true, line((u0, v0), (u1, v0))?),
                (seam_id, true, line((u1, v0), (u1, v1))?),
                (north_edge, true, line((u1, v1), (u0, v1))?),
                (seam_id, false, line((u0, v1), (u0, v0))?),
            ]
        } else {
            vec![
                (seam_id, true, line((u0, v0), (u0, v1))?),
                (north_edge, true, line((u0, v1), (u1, v1))?),
                (seam_id, false, line((u1, v1), (u1, v0))?),
                (south_edge, true, line((u1, v0), (u0, v0))?),
            ]
        };
        let coedges = coedges
            .into_iter()
            .map(|(edge_id, forward, pcurve)| CoedgeRecord { id: id(), edge_id, forward, pcurve })
            .collect();
        let (loop_id, face_id, shell_id) = (id(), id(), id());
        shells.push(ShellRecord {
            id: shell_id,
            faces: vec![FaceRecord {
                id: face_id,
                surface: sheet,
                same_sense: outward,
                loops: vec![LoopRecord { id: loop_id, coedges }],
                name: None,
            }],
        });
    }
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: next_id,
        vertices,
        edges,
        shells,
        genus: 0,
        kind: crate::BodyKind::Solid,
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "thicken_validate", format!(
            "thickenSheet: assembled solid failed validation: {issues:?}"
        )));
    }
    let volume = crate::solid_signed_volume(&solid).or_refuse(KernelStage::Refine, "solid_signed_volume")?;
    if volume <= 0.0 {
        return Err(KernelRefusal::internal(KernelStage::Validate, "thicken_orientation", format!(
            "thickenSheet: internal orientation error (signed volume {volume})"
        )));
    }
    crate::accept_sound(solid, "thickenSheet").map(Some)
}
