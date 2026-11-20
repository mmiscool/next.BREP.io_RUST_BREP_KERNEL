//! The INWARD shell of a solid of revolution whose half-section is a convex
//! polygon (`BREP_OFFSET_REVOLVED_SHELL`, default on; `=0` restores): the section less its
//! cavity, revolved a full turn, built exactly in front of the pipeline.
//!
//! The cavity is bounded by each retained face moved in by d and run on to
//! where it meets the openings and the other moved faces, as the pipeline's
//! offset faces are: in the meridian half-plane (a full revolution reads every
//! distance there, see [`super::revolved_erosion`]) it is the section cut by
//! each retained run's line moved in by d, the half-plane on the section's
//! side. The openings' runs and the run along the axis bound no material the
//! shell keeps and cut nothing. With a convex section the cavity is convex,
//! and where it touches the outline (along the openings and the axis) in one
//! stretch the shell's section is one loop: the outline from where the cavity
//! leaves it round to where it comes back, then the cavity's moved runs back
//! again.
//!
//! Cause (brep-bench shell/cone-open-base): the cone of base radius 6 and
//! height 8 shelled by 1 open at its base. The pipeline fits the inward offset
//! of the cone, whose run past the moved apex folds back, and carves the fold
//! (carrier regularity 149 of 155 ms on the host). The cavity is the cone of
//! the same angle with its apex 1 / sin θ lower.
//!
//! Scope: every face an exact plane or straight revolution (its whole control
//! net recognised); what [`super::revolved_parallel`] reads as a solid of
//! revolution with every section piece a segment; each run on exactly one face of the
//! source and every face on a run; a convex section; a cavity with one
//! stretch of contact, wholly on its side of the axis. Anything else, a
//! cavity that does not reach an opening and a cavity that is empty decline to
//! the pipeline.
use super::prism_erosion::P2;
use super::revolved_parallel::half_section_polygon_reading;
use super::*;
use crate::{make_line, project_point_to_surface};

pub(super) fn enabled() -> bool {
    switch!("BREP_OFFSET_REVOLVED_SHELL", true)
}


/// The shell when the source is a convex revolved body this lane reads (see
/// the module); `None` declines to the pipeline.
pub(super) fn revolved_shell(source: &BrepSolid, opening_face_ids: &[u64], distance: f64) -> Option<OffsetShellResultRecord> {
    if !enabled() || !(distance > MINIMUM_DISTANCE) || !distance.is_finite() || opening_face_ids.is_empty() {
        return None;
    }
    match revolved_shell_impl(source, opening_face_ids, distance) {
        Ok(result) => Some(result),
        Err(reason) => {
            os_debug!("revolved shell declined: {reason}");
            None
        }
    }
}

fn revolved_shell_impl(source: &BrepSolid, opening_face_ids: &[u64], depth: f64) -> Result<OffsetShellResultRecord, String> {
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-9 * scale;
    let snap = 1e-7 * scale;
    // Every face an exact plane or straight revolution, recognised from its
    // whole control net (`analytic()` rebuilds it): the reader's sampled
    // reading of a fitted face as a straight meridian is not enough here.
    for face in source.shells.iter().flat_map(|shell| &shell.faces) {
        if !matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. } | crate::AnalyticSurface::RuledRevolution { .. })) {
            return Err("a face is not an exact plane or straight revolution".into());
        }
    }
    let section = half_section_polygon_reading(source, true).ok_or("not a solid of revolution with a polygon section")?;
    let outline: Vec<P2> = section.corners.iter().map(|&(rho, z)| P2 { x: rho, y: z }).collect();
    let n = outline.len();
    if n < 3 {
        return Err("the section has no area".into());
    }
    let place = |point: P2| section.origin.add(section.x.scale(point.x)).add(section.axis.scale(point.y));
    let faces: Vec<&FaceRecord> = source.shells.iter().flat_map(|shell| &shell.faces).collect();
    // Each run's face: the one face whose carrier holds the run's middle.
    let mut face_of: Vec<Option<usize>> = vec![None; n];
    for k in 0..n {
        if section.on_axis[k] {
            continue;
        }
        let middle = place(outline[k].add(outline[(k + 1) % n]).scale(0.5));
        let mut found = None;
        for (index, face) in faces.iter().enumerate() {
            let foot = project_point_to_surface(&face.surface, middle).map_err(|error| error.to_string())?;
            if foot.distance <= snap {
                if found.is_some() {
                    return Err("a run lies on two faces' carriers".into());
                }
                found = Some(index);
            }
        }
        face_of[k] = Some(found.ok_or("a run lies on no face")?);
    }
    if (0..faces.len()).any(|index| !face_of.contains(&Some(index))) {
        return Err("a face lies on no run".into());
    }
    if opening_face_ids.iter().any(|id| !faces.iter().any(|face| face.id == *id)) {
        return Err("an opening is not a face of the body".into());
    }
    let real: Vec<bool> = face_of.iter().map(|face| face.is_some_and(|index| !opening_face_ids.contains(&faces[index].id))).collect();
    if !real.iter().any(|&r| r) {
        return Err("no retained face".into());
    }
    // A convex section: every corner turns left (or runs straight on).
    for k in 0..n {
        let (a, b, c) = (outline[(k + n - 1) % n], outline[k], outline[(k + 1) % n]);
        if b.sub(a).cross(c.sub(b)) < -tolerance * c.sub(a).len() {
            return Err("the section has a reflex corner".into());
        }
    }
    let chain = clipped_cavity(&outline, &real, depth, tolerance)?;
    if chain.iter().any(|&(a, b, _, _)| a.x < -snap || b.x < -snap) {
        return Err("the cavity crosses the axis".into());
    }
    // The moved-in runs (on retained faces) must be one stretch of the loop,
    // the contact (on openings and the axis) the rest.
    let moved: Vec<bool> = chain.iter().map(|&(_, _, _, moved)| moved).collect();
    let m = chain.len();
    let starts: Vec<usize> = (0..m).filter(|&i| moved[i] && !moved[(i + m - 1) % m]).collect();
    let [first] = starts.as_slice() else {
        return Err(if moved.iter().all(|&x| x) { "the cavity touches no opening".into() } else { format!("{} stretches of moved runs", starts.len()) });
    };
    let mut last = *first;
    while moved[(last + 1) % m] {
        last = (last + 1) % m;
    }
    let (before, after) = ((first + m - 1) % m, (last + 1) % m);
    let (from, to) = (chain[*first].0, chain[last].1);
    let (edge_from, edge_to) = (chain[before].2, chain[after].2);
    if real[edge_from] || real[edge_to] {
        return Err("the cavity touches a retained run".into());
    }
    // Leaving and rejoining one run (a bore's cavity open only at the top),
    // the contact must be the stretch of that run from where the cavity comes
    // back to where it leaves, and the walk goes the whole way round.
    if edge_from == edge_to {
        let (start, run) = (outline[edge_from], outline[(edge_from + 1) % n].sub(outline[edge_from]));
        if !(to.sub(start).dot(run) < from.sub(start).dot(run)) {
            return Err("the cavity leaves and rejoins one run the wrong way round".into());
        }
    }
    // The shell's section: from where the cavity leaves the outline round the
    // outline to where it comes back, then the moved runs backwards. Each point
    // carries the run its segment to the next point lies on: (run, moved).
    let mut points: Vec<(P2, (usize, bool))> = vec![(from, (edge_from, false))];
    let mut k = (edge_from + 1) % n;
    loop {
        points.push((outline[k], (k, false)));
        if k == edge_to {
            break;
        }
        k = (k + 1) % n;
    }
    points.push((to, (chain[last].2, true)));
    let mut i = last;
    while i != *first {
        let previous = (i + m - 1) % m;
        points.push((chain[i].0, (chain[previous].2, true)));
        i = previous;
    }
    // Snap to the axis and drop segments with no length (a point repeating the
    // last passes its run on, since the last's segment had no length).
    let mut snapped: Vec<(P2, (usize, bool))> = Vec::with_capacity(points.len());
    for (mut point, run) in points {
        if point.x.abs() <= snap {
            point.x = 0.0;
        }
        match snapped.last_mut() {
            Some(last) if last.0.sub(point).len() <= tolerance => last.1 = run,
            _ => snapped.push((point, run)),
        }
    }
    while snapped.len() > 1 && snapped[0].0.sub(snapped[snapped.len() - 1].0).len() <= tolerance {
        snapped.pop();
    }
    if snapped.len() < 3 {
        return Err("the shell's section has no area".into());
    }
    // One sheet per run off the axis: a run square to the axis is a plane
    // trimmed by its circles (a disk or an annulus, planar like the source's
    // own), any other run its exact revolution (a cylinder or a cone). Each
    // sheet's image is its run's: a retained face's own run is that face, an
    // opening's the wall left of it, a moved run the retained face's offset.
    let y = section.axis.cross(section.x);
    let count = snapped.len();
    let mut sheets = Vec::with_capacity(count);
    let mut sheet_images = Vec::with_capacity(count);
    for k in 0..count {
        let ((a, (run, moved)), b) = (snapped[k], snapped[(k + 1) % count].0);
        if a.x == 0.0 && b.x == 0.0 {
            continue;
        }
        let Some(index) = face_of[run] else { return Err("a segment off the axis on the axis run".into()) };
        let source = faces[index].id;
        let role = match (moved, opening_face_ids.contains(&source)) {
            (true, false) => OffsetFaceRole::Offset,
            (false, false) => OffsetFaceRole::Source,
            (false, true) => OffsetFaceRole::Wall,
            (true, true) => return Err("an opening's run was moved".into()),
        };
        sheet_images.push(OffsetShellFaceImageRecord { role, source_face_id: source });
        let sheet = if (a.y - b.y).abs() <= tolerance {
            let centre = place(P2 { x: 0.0, y: a.y });
            let (outer, inner) = (a.x.max(b.x), a.x.min(b.x));
            let reach = 2.0 * outer;
            let corner = centre.sub(section.x.scale(reach)).sub(y.scale(reach));
            let plane = crate::make_plane(corner, section.x, y, 2.0 * reach, 2.0 * reach)?;
            let mut loops = vec![vec![crate::make_circle(centre, section.axis, outer)?]];
            if inner > snap {
                loops.push(vec![crate::make_circle(centre, section.axis, inner)?]);
            }
            crate::face_from_surface_trimmed(&plane, &loops)
        } else {
            let line = make_line(place(a), place(b))?;
            crate::face_from_surface(&crate::make_revolution(section.origin, section.axis, &line, std::f64::consts::TAU)?)
        };
        sheets.push(sheet.map_err(|error| error.message)?);
    }
    let solid = crate::solid_from_sheets(&sheets, snap).map_err(|error| error.message)?;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!("the shell does not validate: {:?}", issues.first()));
    }
    // The sewn body against Pappus on its own section: 2π times the section's
    // first moment about the axis.
    let moment: f64 = (0..count)
        .map(|k| {
            let (a, b) = (snapped[k].0, snapped[(k + 1) % count].0);
            (a.x * b.y - b.x * a.y) * (a.x + b.x) / 6.0
        })
        .sum();
    let expected = std::f64::consts::TAU * moment;
    let volume = crate::solid_mass_properties(&solid).map_err(|error| error.to_string())?.volume;
    if !((volume - expected).abs() <= 1e-9 * expected.abs().max(1.0)) {
        return Err(format!("the shell's volume {volume} is not its section's {expected}"));
    }
    // Each face of the sewn body is the sheet whose carrier it keeps bit for
    // bit (sewing renames and may turn a face, never its carrier).
    let mut face_images = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let matches: Vec<usize> = (0..sheets.len())
            .filter(|&i| sheets[i].shells.iter().flat_map(|shell| &shell.faces).any(|sheet_face| same_carrier(&sheet_face.surface, &face.surface)))
            .collect();
        let [index] = matches.as_slice() else { return Err(format!("a face matches {} sheets", matches.len())) };
        face_images.push(sheet_images[*index].clone());
    }
    Ok(OffsetShellResultRecord { solid, face_images })
}

/// Whether two carriers are the same surface, stored bit for bit.
fn same_carrier(a: &crate::NurbsSurface, b: &crate::NurbsSurface) -> bool {
    let bits = |values: &[f64]| values.iter().map(|value| value.to_bits()).collect::<Vec<_>>();
    a.degree_u == b.degree_u
        && a.degree_v == b.degree_v
        && bits(&a.knots_u) == bits(&b.knots_u)
        && bits(&a.knots_v) == bits(&b.knots_v)
        && a.control_points.len() == b.control_points.len()
        && a.control_points.iter().zip(&b.control_points).all(|(row_a, row_b)| {
            row_a.len() == row_b.len()
                && row_a.iter().zip(row_b).all(|(p, q)| [p.x, p.y, p.z, p.w].map(f64::to_bits) == [q.x, q.y, q.z, q.w].map(f64::to_bits))
        })
}

/// The cavity of a convex section: the section cut by each retained run's
/// line moved in by `depth` (the half-plane on the section's side). Its edges
/// counter-clockwise as (start, end, the section's run it lies on or was moved
/// from, whether it was moved). Errs when nothing is left.
fn clipped_cavity(outline: &[P2], real: &[bool], depth: f64, tolerance: f64) -> Result<Vec<(P2, P2, usize, bool)>, String> {
    let n = outline.len();
    // Vertices with the edge leaving each: (point, run, moved).
    let mut polygon: Vec<(P2, usize, bool)> = (0..n).map(|k| (outline[k], k, false)).collect();
    for k in (0..n).filter(|&k| real[k]) {
        let (a, b) = (outline[k], outline[(k + 1) % n]);
        let run = b.sub(a);
        let inward = P2 { x: -run.y, y: run.x }.scale(1.0 / run.len());
        let side = |p: P2| p.sub(a).dot(inward) - depth;
        let mut clipped = Vec::with_capacity(polygon.len() + 1);
        let m = polygon.len();
        for i in 0..m {
            let (p, tag, moved) = polygon[i];
            let q = polygon[(i + 1) % m].0;
            let (fp, fq) = (side(p), side(q));
            let crossing = || p.add(q.sub(p).scale(fp / (fp - fq)));
            match (fp >= 0.0, fq >= 0.0) {
                (true, true) => clipped.push((p, tag, moved)),
                (true, false) => {
                    clipped.push((p, tag, moved));
                    clipped.push((crossing(), k, true));
                }
                (false, true) => clipped.push((crossing(), tag, moved)),
                (false, false) => {}
            }
        }
        polygon = clipped;
        if polygon.len() < 3 {
            return Err("the cavity is empty".into());
        }
    }
    // Drop vertices that repeat the next (their edge has no length).
    let mut kept: Vec<(P2, usize, bool)> = Vec::with_capacity(polygon.len());
    for i in 0..polygon.len() {
        let next = polygon[(i + 1) % polygon.len()].0;
        if polygon[i].0.sub(next).len() > tolerance {
            kept.push(polygon[i]);
        }
    }
    let area2: f64 = (0..kept.len()).map(|i| kept[i].0.cross(kept[(i + 1) % kept.len()].0)).sum();
    if kept.len() < 3 || area2 <= tolerance * tolerance {
        return Err("the cavity is empty".into());
    }
    Ok((0..kept.len()).map(|i| (kept[i].0, kept[(i + 1) % kept.len()].0, kept[i].1, kept[i].2)).collect())
}
