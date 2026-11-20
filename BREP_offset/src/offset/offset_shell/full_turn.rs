//! A PARTIAL turn of a straight generatrix (a piece of a cylinder or a cone
//! built by `make_revolution` over less than a full turn) re-carried on the
//! FULL turn, its seam in the face's own angular gap.
//!
//! The offset reads a cylinder as a cylinder only on a full turn
//! (`AnalyticSurface::RuledRevolution`): there it is offset exactly and,
//! past a reflex junction, extended round its whole period to reach the mitre
//! with its neighbour. A partial-turn carrier, the one a seam-split cylinder's
//! two pieces are joined on (`BREP_FACE_MERGE_RESEAM`), is fitted instead and
//! cannot be extended past its own ends, so the same boss under a rim fillet
//! shelled through its bottom refused or built depending only on where the
//! cylinder's seam sat. Re-carried on the full turn seamed in the middle of
//! the gap, the face is the one the offset builds wherever the seam started.

use super::*;
use crate::{KernelRefusal, KernelStage, NurbsSurface, OrRefuse, Vec4};

/// `BREP_OFFSET_FULL_TURN_REVOLUTIONS` (default on since the 2026-11-18
/// promotion; `=0` keeps every partial turn's own carrier and never retries).
pub(super) fn full_turn_revolutions_on() -> bool {
    switch!("BREP_OFFSET_FULL_TURN_REVOLUTIONS", true)
}

/// `source` with every partial-turn ruled revolution face re-carried on its
/// full turn, or `None` when it has none (or one cannot be re-carried
/// exactly, which leaves the source as it was).
pub(super) fn full_turn_revolution_source(source: &BrepSolid) -> Result<Option<BrepSolid>, KernelRefusal> {
    let mut changed = false;
    let mut solid = source.clone();
    let edges = source.edges.iter().map(|edge| (edge.id, edge)).collect::<HashMap<_, _>>();
    let scale = crate::solid_scale(source).max(1.0);
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            let full = match full_turn_carrier(&face.surface, scale)? {
                Some(full) => full,
                None if reseat_full_turn_seams_on() => match reseated_full_turn(face, &edges, scale)? {
                    Some(full) => full,
                    None => continue,
                },
                None => continue,
            };
            let mut loops = face.loops.clone();
            let mut exact = true;
            'refit: for loop_record in &mut loops {
                for coedge in &mut loop_record.coedges {
                    let Some(edge) = edges.get(&coedge.edge_id) else {
                        exact = false;
                        break 'refit;
                    };
                    if edge.degenerate {
                        exact = false;
                        break 'refit;
                    }
                    let Ok(fit) = crate::fit_pcurve_on_surface_range(&full, &edge.curve, edge.t0, edge.t1, coedge.forward, 1e-9 * scale) else {
                        exact = false;
                        break 'refit;
                    };
                    if !(fit.report.residual <= 1e-7 * scale) {
                        exact = false;
                        break 'refit;
                    }
                    coedge.pcurve = fit.curve;
                }
            }
            if !exact {
                os_debug!("full turn: face {} keeps its partial-turn carrier (a trim did not refit exactly)", face.id);
                continue;
            }
            os_debug!("full turn: face {} re-carried on its full turn, seamed in its gap", face.id);
            face.surface = full;
            face.loops = loops;
            changed = true;
        }
    }
    if changed {
        solid.mass_properties_cache = Default::default();
        if !solid.validate().is_empty() {
            return Ok(None);
        }
    }
    Ok(changed.then_some(solid))
}

/// The full turn of a partial-turn revolution of a STRAIGHT generatrix,
/// seamed in the middle of the turn's gap; `None` for every other carrier.
fn full_turn_carrier(surface: &NurbsSurface, scale: f64) -> Result<Option<NurbsSurface>, KernelRefusal> {
    let Some(structure) = crate::revolution_structure(surface) else {
        return Ok(None);
    };
    let tau = std::f64::consts::TAU;
    // Beyond a HALF turn only: a round's turn is π less the dihedral it
    // rounds, never more than π, so a fillet's piece of a cylinder keeps its
    // own carrier; a turn past π is a boss's or a bore's.
    if !(structure.sweep > std::f64::consts::PI + 1e-6 && structure.sweep < tau - 1e-6) {
        return Ok(None);
    }
    let generatrix = &structure.generatrix;
    if generatrix.degree != 1 || generatrix.control_points.len() != 2 {
        return Ok(None);
    }
    // The gap runs from the end of the sweep round to its start; its middle is
    // as far from the face as the seam can be.
    let seam = structure.sweep + 0.5 * (tau - structure.sweep);
    turned_full_turn(surface, &structure, seam, scale)
}

/// The full turn of `structure`'s straight generatrix seamed at azimuth `seam`
/// (in its frame), when it is `surface`'s own carrier on the same side.
fn turned_full_turn(surface: &NurbsSurface, structure: &crate::RevolutionStructure, seam: f64, scale: f64) -> Result<Option<NurbsSurface>, KernelRefusal> {
    let tau = std::f64::consts::TAU;
    let generatrix = &structure.generatrix;
    let frame = &structure.frame;
    let (cos, sin) = (seam.cos(), seam.sin());
    let rotate = |point: Vec3| {
        let along = point.sub(frame.origin);
        let height = along.dot(frame.axis);
        let (x, y) = (along.dot(frame.x_axis), along.dot(frame.y_axis));
        frame
            .origin
            .add(frame.axis.scale(height))
            .add(frame.x_axis.scale(x * cos - y * sin))
            .add(frame.y_axis.scale(x * sin + y * cos))
    };
    let mut turned = generatrix.clone();
    for control in &mut turned.control_points {
        let point = control.point().or_refuse(KernelStage::Refine, "point")?;
        *control = Vec4::from_point(rotate(point), control.w);
    }
    let Ok(full) = crate::make_revolution(frame.origin, frame.axis, &turned, tau) else {
        return Ok(None);
    };
    // The new carrier must BE the old one's surface: every old point on it.
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    for i in 0..=8 {
        for j in 0..=2 {
            let point = surface.evaluate(u0 + (u1 - u0) * i as f64 / 8.0, v0 + (v1 - v0) * j as f64 / 2.0).or_refuse(KernelStage::Refine, "evaluate")?;
            let Ok(foot) = crate::project_point_to_surface(&full, point) else {
                return Ok(None);
            };
            if !(foot.distance <= 1e-9 * scale) {
                return Ok(None);
            }
        }
    }
    // And the same side: the normals agree, so `same_sense` still holds.
    let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let point = surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
    let normal = surface.normal(u, v).or_refuse(KernelStage::Refine, "normal")?;
    let Ok(foot) = crate::project_point_to_surface(&full, point) else {
        return Ok(None);
    };
    if full.normal(foot.u, foot.v).or_refuse(KernelStage::Refine, "normal")?.dot(normal) <= 0.0 {
        return Ok(None);
    }
    Ok(Some(full))
}

/// `BREP_OFFSET_RESEAT_FULL_TURN_SEAMS` (default on since the 2026-11-18
/// promotion; `=0` keeps the face's own turn): a face on a FULL turn
/// of a straight generatrix whose seam lies inside the face's angular gap near
/// either end (within its outer quarters, not on the face's own boundary) is
/// re-carried on the same turn seamed in the middle of that gap.
pub(super) fn reseat_full_turn_seams_on() -> bool {
    switch!("BREP_OFFSET_RESEAT_FULL_TURN_SEAMS", true)
}

/// The full turn `face` lies on, seamed in the middle of the face's own
/// angular gap, when its seam now lies inside that gap near either end;
/// `None` otherwise.
///
/// A boss's cylinder kept as one face (its seam buried in the wall half a
/// degree past a reflex junction) must have its offset widened past that
/// junction to reach the miter with the wall's offset; the widening runs
/// across the carrier's seam, off its domain, so the reflex-side lane
/// declined and the shell refused or built off its oracle. Seamed in the gap,
/// the face has half the gap of room on each side.
fn reseated_full_turn(face: &FaceRecord, edges: &HashMap<u64, &EdgeRecord>, scale: f64) -> Result<Option<NurbsSurface>, KernelRefusal> {
    let tau = std::f64::consts::TAU;
    let Some(structure) = crate::revolution_structure(&face.surface) else {
        return Ok(None);
    };
    if !(structure.sweep >= tau - 1e-6) || structure.generatrix.degree != 1 || structure.generatrix.control_points.len() != 2 {
        return Ok(None);
    }
    let frame = &structure.frame;
    let azimuth_at = |edge: &EdgeRecord, t: f64| -> Result<Option<f64>, KernelRefusal> {
        let point = edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
        let along = point.sub(frame.origin);
        if along.sub(frame.axis.scale(along.dot(frame.axis))).length() <= 1e-9 * scale {
            return Ok(None);
        }
        Ok(Some(along.dot(frame.y_axis).atan2(along.dot(frame.x_axis)).rem_euclid(tau)))
    };
    // Every boundary edge's azimuths, each interval bisected until it turns
    // at most AZIMUTH_STEP: a gap wider than that is seen, and no stretch of
    // the face itself reads as one (16 fixed samples of a 350-degree arc
    // leave 22-degree holes, wider than a real 10-degree gap; cross review).
    const AZIMUTH_STEP: f64 = 0.25 * std::f64::consts::PI / 180.0;
    let mut azimuths = Vec::new();
    for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
        let Some(edge) = edges.get(&coedge.edge_id) else { return Ok(None) };
        // Seeded at the curve's own knot breaks (a span squeezed into a sliver
        // of the parameter would otherwise exhaust the depth), then 16 even.
        let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
        let mut breaks = (0..=16).map(|k| low + (high - low) * k as f64 / 16.0).collect::<Vec<_>>();
        breaks.extend(edge.curve.knots.iter().copied().filter(|knot| *knot > low && *knot < high));
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        let mut intervals = breaks.windows(2).map(|pair| (pair[0], pair[1], 0usize)).collect::<Vec<_>>();
        if let Some(azimuth) = azimuth_at(edge, low)? {
            azimuths.push(azimuth);
        }
        while let Some((t0, t1, depth)) = intervals.pop() {
            let (a0, a1) = (azimuth_at(edge, t0)?, azimuth_at(edge, t1)?);
            if let Some(a1) = a1 {
                azimuths.push(a1);
            }
            let turn = match (a0, a1) {
                (Some(a0), Some(a1)) => {
                    let raw = (a1 - a0).abs();
                    raw.min(tau - raw)
                }
                _ => 0.0,
            };
            if turn > AZIMUTH_STEP {
                // Unresolved at the depth cap: the gap cannot be read, so the
                // face is not judged (it keeps its carrier).
                if depth >= 24 {
                    return Ok(None);
                }
                let middle = 0.5 * (t0 + t1);
                intervals.push((t0, middle, depth + 1));
                intervals.push((middle, t1, depth + 1));
            }
        }
    }
    azimuths.sort_by(f64::total_cmp);
    let (mut widest, mut gap_start) = (0.0f64, 0.0f64);
    for (index, &azimuth) in azimuths.iter().enumerate() {
        let next = if index + 1 < azimuths.len() { azimuths[index + 1] } else { azimuths[0] + tau };
        if next - azimuth > widest {
            widest = next - azimuth;
            gap_start = azimuth;
        }
    }
    // A gap under a degree has no room worth moving the seam into.
    if widest < std::f64::consts::PI / 180.0 {
        return Ok(None);
    }
    // Only a seam strictly INSIDE the gap, in its outer quarters: one in the
    // middle half has room already, and one ON the gap's end is the face's own
    // boundary (a cylinder split at its seam), which this does not move.
    let into = (tau - gap_start).rem_euclid(tau);
    let end_band = 1e-6;
    let inside = into > end_band && into < widest - end_band;
    if !inside || (into > 0.25 * widest && into < 0.75 * widest) {
        return Ok(None);
    }
    turned_full_turn(&face.surface, &structure, gap_start + 0.5 * widest, scale)
}

/// `source` with every pair of RULED revolution faces (cylinder or cone
/// pieces) about one axis with one generatrix that meet only along meridians
/// (a cylinder split at its own seam) joined into one face; `None` when there is no such pair (or no join validates).
///
/// Only a refusal reaches this (`offset_body`): with the seam on the exposed
/// arc, one of the two pieces can be a sliver narrower than the distance (the
/// seam a degree outside a reflex junction), and the pipeline refused what it
/// builds with the seam buried. Joined, the faces are the buried-seam ones: a
/// straight generatrix turning past half a turn on the full turn seamed in its
/// gap, any other on the one partial revolution spanning both pieces.
///
/// An OPENING piece is never joined: the selection names that piece, not the
/// union.
pub(super) fn seam_pieces_joined(source: &BrepSolid, openings: &[u64]) -> Result<Option<BrepSolid>, KernelRefusal> {
    let scale = crate::solid_scale(source).max(1.0);
    let mut solid = source.clone();
    let mut joined_any = false;
    loop {
        let faces = solid.shells.iter().flat_map(|shell| &shell.faces).cloned().collect::<Vec<_>>();
        let edges = solid.edges.iter().map(|edge| (edge.id, edge.clone())).collect::<HashMap<_, _>>();
        let edge_ids = |face: &FaceRecord| face.loops.iter().flat_map(|lp| &lp.coedges).map(|coedge| coedge.edge_id).collect::<HashSet<u64>>();
        let mut next = None;
        'pairs: for (i, first) in faces.iter().enumerate() {
            if openings.contains(&first.id) {
                continue;
            }
            let Some(a) = crate::revolution_structure(&first.surface) else { continue };
            // A cylinder or a cone only: a straight generatrix.
            if a.generatrix.degree != 1 || a.generatrix.control_points.len() != 2 {
                continue;
            }
            for second in &faces[i + 1..] {
                if openings.contains(&second.id) || first.same_sense != second.same_sense {
                    continue;
                }
                let Some(b) = crate::revolution_structure(&second.surface) else { continue };
                if !same_revolution(&a, &b, scale)? {
                    continue;
                }
                let shared = edge_ids(first).intersection(&edge_ids(second)).copied().collect::<HashSet<_>>();
                if shared.is_empty() {
                    continue;
                }
                // Every shared edge a meridian, else this partner is not one;
                // the next partner of `first` is still read.
                let mut meridians = true;
                for edge_id in &shared {
                    if !is_meridian(&edges[edge_id], &a, scale)? {
                        meridians = false;
                        break;
                    }
                }
                if !meridians {
                    continue;
                }
                // The piece that keeps its id and name is the one the boolean's
                // own rejoin keeps (`merges keep the first name`): the name the
                // other's split suffix (`X_1`) was minted from, else the older.
                let second_survives = match (&first.name, &second.name) {
                    (Some(a_name), Some(b_name)) if a_name != b_name && a_name.starts_with(b_name.as_str()) => true,
                    (Some(a_name), Some(b_name)) if a_name != b_name && b_name.starts_with(a_name.as_str()) => false,
                    _ => second.id < first.id,
                };
                next = Some(if second_survives { (second.clone(), first.clone(), shared, a) } else { (first.clone(), second.clone(), shared, a) });
                break 'pairs;
            }
        }
        let Some((first, second, shared, structure)) = next else { break };
        match join_revolution_pieces(&solid, &first, &second, &shared, &structure, scale)? {
            Some(joined) => {
                os_debug!("full turn: faces {} and {} joined across a meridian of their revolution", first.id, second.id);
                solid = joined;
                joined_any = true;
            }
            None => break,
        }
    }
    Ok(joined_any.then_some(solid))
}

/// One axis line and one generatrix profile (height and radius about the
/// axis of every control, with its weight, in order).
fn same_revolution(a: &crate::RevolutionStructure, b: &crate::RevolutionStructure, scale: f64) -> Result<bool, KernelRefusal> {
    let band = 1e-9 * scale;
    if a.frame.axis.dot(b.frame.axis) < 1.0 - 1e-12 {
        return Ok(false);
    }
    let offset = b.frame.origin.sub(a.frame.origin);
    if offset.sub(a.frame.axis.scale(offset.dot(a.frame.axis))).length() > band {
        return Ok(false);
    }
    if a.generatrix.degree != b.generatrix.degree
        || a.generatrix.control_points.len() != b.generatrix.control_points.len()
        || a.generatrix.knots.iter().zip(&b.generatrix.knots).any(|(p, q)| (p - q).abs() > 1e-12)
    {
        return Ok(false);
    }
    let profile = |point: Vec3| {
        let along = point.sub(a.frame.origin);
        let height = along.dot(a.frame.axis);
        (height, along.sub(a.frame.axis.scale(height)).length())
    };
    for (p, q) in a.generatrix.control_points.iter().zip(&b.generatrix.control_points) {
        let (hp, rp) = profile(p.point().or_refuse(KernelStage::Refine, "point")?);
        let (hq, rq) = profile(q.point().or_refuse(KernelStage::Refine, "point")?);
        if (hp - hq).abs() > band || (rp - rq).abs() > band || (p.w - q.w).abs() > 1e-12 {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Every point of `edge` at one azimuth about the revolution's axis.
fn is_meridian(edge: &EdgeRecord, structure: &crate::RevolutionStructure, scale: f64) -> Result<bool, KernelRefusal> {
    let frame = &structure.frame;
    let mut reference: Option<Vec3> = None;
    for k in 0..=8 {
        let point = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 8.0).or_refuse(KernelStage::Refine, "evaluate")?;
        let along = point.sub(frame.origin);
        let radial = along.sub(frame.axis.scale(along.dot(frame.axis)));
        if radial.length() <= 1e-9 * scale {
            continue;
        }
        let direction = radial.scale(1.0 / radial.length());
        match reference {
            None => reference = Some(direction),
            Some(first) => {
                if first.cross(direction).length() > 1e-9 || first.dot(direction) < 0.0 {
                    return Ok(false);
                }
            }
        }
    }
    Ok(reference.is_some())
}

/// `first` and `second` joined into one face (keeping `first`'s id) once the
/// `shared` meridians are dropped.
fn join_revolution_pieces(
    solid: &BrepSolid,
    first: &FaceRecord,
    second: &FaceRecord,
    shared: &HashSet<u64>,
    structure: &crate::RevolutionStructure,
    scale: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    let edges = solid.edges.iter().map(|edge| (edge.id, edge)).collect::<HashMap<_, _>>();
    let frame = &structure.frame;
    let tau = std::f64::consts::TAU;
    // The union's azimuths, read on its boundary; the widest gap between them
    // is where the union is not.
    let mut azimuths = Vec::new();
    for face in [first, second] {
        for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
            let Some(edge) = edges.get(&coedge.edge_id) else { return Ok(None) };
            for k in 0..=16 {
                let point = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 16.0).or_refuse(KernelStage::Refine, "evaluate")?;
                let along = point.sub(frame.origin);
                if along.sub(frame.axis.scale(along.dot(frame.axis))).length() <= 1e-9 * scale {
                    continue;
                }
                azimuths.push(along.dot(frame.y_axis).atan2(along.dot(frame.x_axis)).rem_euclid(tau));
            }
        }
    }
    azimuths.sort_by(f64::total_cmp);
    let (mut widest, mut gap_start) = (0.0f64, 0.0f64);
    for (index, &azimuth) in azimuths.iter().enumerate() {
        let next = if index + 1 < azimuths.len() { azimuths[index + 1] } else { azimuths[0] + tau };
        if next - azimuth > widest {
            widest = next - azimuth;
            gap_start = azimuth;
        }
    }
    if widest < 1e-3 {
        return Ok(None);
    }
    let union = tau - widest;
    let straight = structure.generatrix.degree == 1 && structure.generatrix.control_points.len() == 2;
    // A straight generatrix past half a turn goes on the full turn seamed in
    // the middle of the gap; a shorter turn on the partial turn spanning the
    // union, starting where the gap ends.
    let (start, sweep) = if straight && union > std::f64::consts::PI {
        (gap_start + 0.5 * widest, tau)
    } else {
        (gap_start + widest, union)
    };
    let (cos, sin) = (start.cos(), start.sin());
    let mut turned = structure.generatrix.clone();
    for control in &mut turned.control_points {
        let along = control.point().or_refuse(KernelStage::Refine, "point")?.sub(frame.origin);
        let height = along.dot(frame.axis);
        let (x, y) = (along.dot(frame.x_axis), along.dot(frame.y_axis));
        let point = frame.origin.add(frame.axis.scale(height)).add(frame.x_axis.scale(x * cos - y * sin)).add(frame.y_axis.scale(x * sin + y * cos));
        *control = Vec4::from_point(point, control.w);
    }
    let Ok(carrier) = crate::make_revolution(frame.origin, frame.axis, &turned, sweep) else {
        return Ok(None);
    };
    // The same side as the pieces: `same_sense` must still hold.
    {
        let [u0, u1] = first.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = first.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
        let point = first.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
        let Ok(foot) = crate::project_point_to_surface(&carrier, point) else { return Ok(None) };
        if foot.distance > 1e-9 * scale
            || carrier.normal(foot.u, foot.v).or_refuse(KernelStage::Refine, "normal")?.dot(first.surface.normal(u, v).or_refuse(KernelStage::Refine, "normal")?) <= 0.0
        {
            return Ok(None);
        }
    }
    let start_of = |coedge: &CoedgeRecord| {
        let edge = edges[&coedge.edge_id];
        if coedge.forward { edge.start_vertex_id } else { edge.end_vertex_id }
    };
    let end_of = |coedge: &CoedgeRecord| {
        let edge = edges[&coedge.edge_id];
        if coedge.forward { edge.end_vertex_id } else { edge.start_vertex_id }
    };
    let mut remaining = [first, second]
        .iter()
        .flat_map(|face| face.loops.iter().flat_map(|lp| &lp.coedges))
        .filter(|coedge| !shared.contains(&coedge.edge_id))
        .cloned()
        .collect::<Vec<_>>();
    let mut loops = Vec::new();
    // Fresh loop ids above every loop of the solid, not only these two faces'.
    let mut next_loop_id = solid.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| &face.loops).map(|lp| lp.id).max().unwrap_or(0);
    // The OUTER loop first: the one the two pieces' outer loops became.
    let outer_coedges = [first, second]
        .iter()
        .filter_map(|face| face.loops.first())
        .flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.id))
        .collect::<HashSet<u64>>();
    while let Some(head) = remaining.pop() {
        let closing = start_of(&head);
        let mut chain = vec![head];
        while end_of(chain.last().unwrap()) != closing {
            let tail = end_of(chain.last().unwrap());
            let Some(index) = remaining.iter().position(|coedge| start_of(coedge) == tail) else {
                return Ok(None);
            };
            chain.push(remaining.swap_remove(index));
        }
        for coedge in &mut chain {
            let edge = edges[&coedge.edge_id];
            let Ok(fit) = crate::fit_pcurve_on_surface_range(&carrier, &edge.curve, edge.t0, edge.t1, coedge.forward, 1e-9 * scale) else {
                return Ok(None);
            };
            if !(fit.report.residual <= 1e-7 * scale) {
                return Ok(None);
            }
            coedge.pcurve = fit.curve;
        }
        next_loop_id += 1;
        loops.push(LoopRecord { id: next_loop_id, coedges: chain });
    }
    let Some(outer) = loops.iter().position(|lp| lp.coedges.iter().any(|coedge| outer_coedges.contains(&coedge.id))) else {
        return Ok(None);
    };
    loops.swap(0, outer);
    let mut joined = solid.clone();
    for shell in &mut joined.shells {
        shell.faces.retain(|face| face.id != second.id);
        for face in &mut shell.faces {
            if face.id == first.id {
                face.surface = carrier.clone();
                face.loops = loops.clone();
            }
        }
    }
    joined.edges.retain(|edge| !shared.contains(&edge.id));
    let used = joined.edges.iter().flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id]).collect::<HashSet<_>>();
    joined.vertices.retain(|vertex| used.contains(&vertex.id));
    joined.mass_properties_cache = Default::default();
    // The meridians' ends split the rims they crossed; where both pieces of a
    // rim are one curve, they are one edge again.
    let meridian_ends = shared
        .iter()
        .filter_map(|edge_id| edges.get(edge_id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect::<HashSet<_>>();
    if let Ok(merged) = brep_boolean::coalesce::merge_curve_continuation_edges_at(&joined, 1e-7 * scale, &meridian_ends) {
        joined = merged;
    }
    if !joined.validate().is_empty() {
        os_debug!("full turn: the join of faces {} and {} does not validate: {:?}", first.id, second.id, joined.validate());
        return Ok(None);
    }
    Ok(Some(joined))
}
