//! OUTWARD SIDE WIDENING: a curved retained face whose trim is a uv rectangle
//! of isolines lying inside its surface's domain, in an OUTWARD offset, has its
//! offset carrier built over the same surface with each SHARP CONVEX side moved
//! outward in the domain, so the two offsets meet at their mitre past both
//! source rims.
//!
//! The carrier lanes clone the face's trim onto the offset surface, and an
//! analytic carrier (a sphere, a circular cylinder) takes no extension at all:
//! the half cylinder's offset stopped at the x = 0 ray, short of the offset
//! plane, and the hemisphere's offset sphere stopped at the equator, short of
//! the offset base plane. The pair imprints found nothing and the assembly
//! left the rims one-use ("connector edge left its plane", "non-integral
//! genus"). [`super::sharp_side_continued_source`] answers this for a trim on
//! the domain's own ends, by continuing the surface; here the surface already
//! goes on (the whole cylinder or sphere a boolean trimmed), so the trim's side
//! moves inside the domain and every `(u, v)` still names the same point, as
//! [`super::reflex_side_widened_source`] does for an inward reflex side.
//!
//! Each side grows by `1.5 ×` what the pipeline asked of it
//! (`outward_carrier_extension`), measured on the offset image, so the
//! crossing lands strictly inside the trim. A side on the SEAM of a closed
//! direction (the half cylinder's x = 0 ruling) first has the seam moved out
//! of the trim (`reseamed`: the surface cut outside the trim and its two
//! pieces joined the other way round, exactly, every point inside the trim
//! keeping its parameters). A side on an open domain end is left to the
//! carrier lane (a cylinder's ruled growth). A side the move would carry past
//! the domain declines, as does a degenerate side (a pole) that would have to
//! move. Sides that do not move keep their edges; a seam edge used twice stays
//! one edge, and corner vertices that do not move keep their ids.
use super::*;
use crate::{KernelRefusal, KernelStage, NurbsSurface, OrRefuse, Vec4};

/// `BREP_OFFSET_OUTWARD_SIDE_WIDEN` (default on; `=0` restores): see the module.
pub(super) fn outward_side_widen_on() -> bool {
    crate::switch!("BREP_OFFSET_OUTWARD_SIDE_WIDEN", true)
}

thread_local! {
    /// How many faces this thread has widened, read around one body's build.
    static WIDENED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

thread_local! {
    /// Concave sides held back (the tangent connector's convention) on this thread.
    static CONCAVE_HELD: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    /// Whether concave sides are continued round their arc on this thread.
    static CONCAVE_ARC: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// `BREP_OFFSET_CONCAVE_ARC_RETRY` (default on; `=0` restores the refusal): an
/// outward pass that refused after holding back a CONCAVE side (one whose
/// surface bends into the material across it) is run once more with those
/// sides continued round their arc, the OCCT convention for a concave sharp
/// join. The first pass keeps the tangent connector wherever it builds (a
/// box notched by a quarter cylinder, shelled outward: its pinned closed form
/// is that convention), so only refusals change.
///
/// Cause (probe): a box with a spherical dimple, or with a cylindrical groove,
/// grown sharp, refused at every distance (a vacated rim left open; a
/// connector edge that left its plane): the concave face's offset stopped at
/// its source rim, short of the moved plane it must meet.
pub(super) fn concave_arc_retry_on() -> bool {
    crate::switch!("BREP_OFFSET_CONCAVE_ARC_RETRY", true)
}

/// The concave sides held back on this thread so far.
pub(super) fn concave_held() -> usize {
    CONCAVE_HELD.with(std::cell::Cell::get)
}

/// Runs `action` with concave sides continued round their arc.
pub(super) fn with_concave_arc<T>(action: impl FnOnce() -> T) -> T {
    struct Restore(bool);
    impl Drop for Restore {
        fn drop(&mut self) {
            CONCAVE_ARC.with(|cell| cell.set(self.0));
        }
    }
    let _restore = Restore(CONCAVE_ARC.with(|cell| cell.replace(true)));
    action()
}

/// The number of faces widened on this thread so far.
pub(super) fn widened_count() -> usize {
    WIDENED.with(|count| count.get())
}

/// Whether `face`'s surface curves TOWARD its material across the side in
/// `slot` (0/1 a constant-u side moving in u, 2/3 a constant-v side moving in
/// v) at `at`: the second derivative along the move read against the outward
/// normal, at three stations along the side, beyond a relative band.
fn bends_into_the_material(face: &FaceRecord, slot: usize, at: f64, across: (f64, f64)) -> Result<bool, KernelRefusal> {
    for fraction in [0.25, 0.5, 0.75] {
        let other = across.0 + (across.1 - across.0) * fraction;
        let (u, v) = if slot < 2 { (at, other) } else { (other, at) };
        let derivatives = face.surface.derivatives(u, v, 2).or_refuse(KernelStage::Refine, "derivatives")?;
        let (first, second) = if slot < 2 { (derivatives[1][0], derivatives[2][0]) } else { (derivatives[0][1], derivatives[0][2]) };
        let speed = first.length();
        if !(speed > 0.0) {
            return Ok(true);
        }
        let normal = face_normal(face, u, v)?;
        // Normal curvature along the move, positive toward the outward normal:
        // a convex face (material inside its bend) reads negative.
        let curvature = second.dot(normal) / (speed * speed);
        if curvature > 1e-9 * (1.0 + 1.0 / speed) {
            return Ok(true);
        }
    }
    Ok(false)
}

pub(super) fn outward_sharp_side_widened_source(
    source: &BrepSolid,
    face: &FaceRecord,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    distance: f64,
    extension: &crate::CarrierExtension,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    if distance >= 0.0 || !outward_side_widen_on() || face.loops.len() != 1 || face.loops[0].coedges.len() != 4 {
        return Ok(None);
    }
    let surface = &face.surface;
    if surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let band_u = 1e-9 * (u1 - u0).abs().max(1.0);
    let band_v = 1e-9 * (v1 - v0).abs().max(1.0);
    // Each use's isoline: (true, u) for a constant-u use, (false, v) for a
    // constant-v one; anything else is not a rectangle.
    let mut isolines = Vec::with_capacity(4);
    for coedge in &face.loops[0].coedges {
        let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let first = coedge.pcurve.evaluate(q0).or_refuse(KernelStage::Refine, "evaluate")?;
        let (mut on_u, mut on_v) = (true, true);
        for sample in 1..=8 {
            let uv = coedge.pcurve.evaluate(q0 + (q1 - q0) * sample as f64 / 8.0).or_refuse(KernelStage::Refine, "evaluate")?;
            on_u &= (uv.x - first.x).abs() <= band_u;
            on_v &= (uv.y - first.y).abs() <= band_v;
        }
        isolines.push(match (on_u, on_v) {
            (true, false) => (true, first.x),
            (false, true) => (false, first.y),
            _ => return Ok(None),
        });
    }
    let u_sides = isolines.iter().filter(|(along_u, _)| *along_u).map(|(_, at)| *at).collect::<Vec<_>>();
    let v_sides = isolines.iter().filter(|(along_u, _)| !*along_u).map(|(_, at)| *at).collect::<Vec<_>>();
    if u_sides.len() != 2 || v_sides.len() != 2 {
        return Ok(None);
    }
    let (ua, ub) = (u_sides[0].min(u_sides[1]), u_sides[0].max(u_sides[1]));
    let (va, vb) = (v_sides[0].min(v_sides[1]), v_sides[0].max(v_sides[1]));
    if ub - ua <= band_u || vb - va <= band_v {
        return Ok(None);
    }
    let edge_by_id = source.edges.iter().map(|edge| (edge.id, edge)).collect::<HashMap<_, _>>();
    // Slot per side: 0 u low, 1 u high, 2 v low, 3 v high.
    let slot_of = |along_u: bool, at: f64| -> usize {
        if along_u {
            if (at - ua).abs() <= band_u { 0 } else { 1 }
        } else if (at - va).abs() <= band_v {
            2
        } else {
            3
        }
    };
    let mut grow = [0.0f64; 4];
    let mut degenerate = [false; 4];
    for (coedge, (along_u, at)) in face.loops[0].coedges.iter().zip(&isolines) {
        let slot = slot_of(*along_u, *at);
        let Some(edge) = edge_by_id.get(&coedge.edge_id) else {
            return Ok(None);
        };
        if edge.degenerate {
            degenerate[slot] = true;
            continue;
        }
        let asked = [extension.u_min, extension.u_max, extension.v_min, extension.v_max][slot];
        if asked <= 0.0 {
            continue;
        }
        let Some((mate, mate_use)) = junction_mate(source_faces, face, coedge.edge_id) else {
            continue;
        };
        if mate.id == face.id || opening_set.contains(&mate.id) || uses_are_tangent(face, coedge, mate, mate_use)? {
            continue;
        }
        let stations = junction_stations(face, coedge, edge, mate)?;
        if stations.is_empty() || stations.iter().any(|station| !station.is_convex()) {
            continue;
        }
        // Only where the surface runs on STRAIGHT (a ruling) or bends away
        // from the material across the side: a concave face's offset is closed
        // at such a rim by the tangent connector the pipeline builds (a box
        // notched by a quarter cylinder, shelled outward: its pinned closed
        // form is that convention), and continuing it round its arc refused.
        if bends_into_the_material(face, slot, *at, if slot < 2 { (va, vb) } else { (ua, ub) })? {
            if !CONCAVE_ARC.with(std::cell::Cell::get) {
                CONCAVE_HELD.with(|held| held.set(held.get() + 1));
                continue;
            }
            os_debug!("outward side widening: face {} side {slot} is concave, continued round its arc (retry)", face.id);
        }
        grow[slot] = asked * 1.5;
    }
    if grow == [0.0; 4] {
        return Ok(None);
    }
    const STATIONS: usize = 8;
    // The surface the widened trim lies on: the face's own, or the same
    // closed surface with its seam moved out of the trim.
    let mut grown = surface.clone();
    // A CLOSED direction whose trim ends on the seam on one side only has its
    // seam moved out of the trim first (`reseamed`): the trim's parameters,
    // and every point inside it, are unchanged.
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let mut domains = [[u0, u1], [v0, v1]];
    for (direction, closed) in [(0usize, closed_u), (1usize, closed_v)] {
        let [low, high] = domains[direction];
        let band = if direction == 0 { band_u } else { band_v };
        let (a, b) = if direction == 0 { (ua, ub) } else { (va, vb) };
        let (at_low, at_high) = ((a - low).abs() <= band, (b - high).abs() <= band);
        let wants = grow[2 * direction] > 0.0 && at_low || grow[2 * direction + 1] > 0.0 && at_high;
        if !closed || !wants || at_low == at_high {
            continue;
        }
        // Cut midway across the free part of the domain.
        let cut = if at_high { low + 0.5 * (a - low) } else { b + 0.5 * (high - b) };
        match reseamed(&grown, direction == 0, cut, at_high) {
            Ok(Some(rebased)) => {
                grown = rebased;
                domains[direction] = if direction == 0 {
                    grown.domain_u().or_refuse(KernelStage::Refine, "domain_u")?
                } else {
                    grown.domain_v().or_refuse(KernelStage::Refine, "domain_v")?
                };
                os_debug!("outward side widening: face {} reseamed at {cut:.6} ({}), domain {:?}", face.id, if direction == 0 { "u" } else { "v" }, domains[direction]);
            }
            _ => {
                os_debug!("outward side widening: face {} could not move its seam", face.id);
                return Ok(None);
            }
        }
    }
    let [[u0, u1], [v0, v1]] = domains;
    let mut reach = [ua, ub, va, vb];
    for slot in 0..4 {
        let wanted = grow[slot];
        if wanted <= 0.0 {
            continue;
        }
        let end = reach[slot];
        let sense = if slot % 2 == 0 { -1.0 } else { 1.0 };
        let (domain_low, domain_high, band) = if slot < 2 { (u0, u1, band_u) } else { (v0, v1, band_v) };
        // A side on an OPEN domain end is grown by the carrier lane itself
        // (along a ruling, or not at all); only a side inside the domain, or
        // on a closed direction's seam moved out of the way above, moves.
        let on_domain_end = if sense < 0.0 { (end - domain_low).abs() <= band } else { (end - domain_high).abs() <= band };
        if on_domain_end {
            grow[slot] = 0.0;
            continue;
        }
        let (across_a, across_b) = if slot < 2 { (reach[2], reach[3]) } else { (reach[0], reach[1]) };
        let mut speed = f64::INFINITY;
        for index in 0..=STATIONS {
            let across = across_a + (across_b - across_a) * index as f64 / STATIONS as f64;
            let (u, v) = if slot < 2 { (end, across) } else { (across, end) };
            let derivatives = grown.derivatives(u, v, 1).or_refuse(KernelStage::Refine, "derivatives")?;
            let rate = if slot < 2 { derivatives[1][0].length() } else { derivatives[0][1].length() };
            speed = speed.min(rate);
        }
        if !(speed > 0.0) {
            return Ok(None);
        }
        // The OFFSET image must travel `wanted` across the strip, the least
        // over the stations, advancing monotonically at each (a regular
        // offset, no cusp); the strip must stay inside the domain.
        let mut delta = wanted / speed;
        let mut accepted = None;
        for _ in 0..8 {
            let far = end + sense * delta;
            if far < domain_low + band || far > domain_high - band {
                return Ok(None);
            }
            let offsets =
                crate::OffsetEvaluator::new("offset_shell", &grown, crate::OffsetNormal::Face { same_sense: face.same_sense });
            let at = |along: f64, across: f64| -> Result<Vec3, KernelRefusal> {
                let (u, v) = if slot < 2 { (along, across) } else { (across, along) };
                Ok(offsets.at(u, v, -distance).or_refuse(KernelStage::Refine, "at")?.point)
            };
            let mut least = f64::INFINITY;
            for index in 0..=STATIONS {
                let across = across_a + (across_b - across_a) * index as f64 / STATIONS as f64;
                let start = at(end, across)?;
                let direction = at(far, across)?.sub(start);
                let mut previous = start;
                for step in 1..=16 {
                    let point = at(end + sense * delta * step as f64 / 16.0, across)?;
                    if point.sub(previous).dot(direction) <= 0.0 {
                        return Ok(None);
                    }
                    previous = point;
                }
                least = least.min(direction.length());
            }
            if least >= wanted * (1.0 - 1e-9) {
                accepted = Some(far);
                break;
            }
            delta *= (wanted / least.max(wanted * 1e-3)) * 1.05;
        }
        let Some(far) = accepted else {
            return Ok(None);
        };
        reach[slot] = far;
    }
    // Nothing moved (every asked side on an open domain end): the carrier
    // lanes keep the face as it is.
    if reach == [ua, ub, va, vb] {
        return Ok(None);
    }
    let [nua, nub, nva, nvb] = reach;
    // A side changes when it moves or when a side across it does; a pole
    // that would change declines.
    let changed = |slot: usize| -> bool {
        let moved = reach[slot] != [ua, ub, va, vb][slot];
        let across = if slot < 2 { nva != va || nvb != vb } else { nua != ua || nub != ub };
        moved || across
    };
    for slot in 0..4 {
        if degenerate[slot] && changed(slot) {
            return Ok(None);
        }
    }
    let mut solid = source.clone();
    let mut next_id = solid
        .vertices
        .iter()
        .map(|vertex| vertex.id)
        .chain(solid.edges.iter().map(|edge| edge.id))
        .chain(solid.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| {
            face.loops
                .iter()
                .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.id).chain([loop_record.id]))
        }))
        .max()
        .unwrap_or(0)
        + 1;
    // Corner vertices: an existing vertex of this face's loop at the same
    // point keeps its id, and corners at one point (a seam's two ends, a
    // pole) share one vertex.
    let merge = 1e-9 * crate::solid_scale(source).max(1.0);
    let loop_vertices = face.loops[0]
        .coedges
        .iter()
        .filter_map(|coedge| edge_by_id.get(&coedge.edge_id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .filter_map(|id| source.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| (id, vertex.point)))
        .collect::<Vec<_>>();
    let mut placed: Vec<(u64, Vec3)> = Vec::new();
    let mut corner = |u: f64, v: f64, solid: &mut BrepSolid, next_id: &mut u64| -> Result<u64, KernelRefusal> {
        let point = grown.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
        if let Some((id, _)) = placed.iter().chain(&loop_vertices).find(|(_, at)| at.sub(point).length() <= merge) {
            return Ok(*id);
        }
        let id = *next_id;
        *next_id += 1;
        solid.vertices.push(VertexRecord { id, point });
        placed.push((id, point));
        Ok(id)
    };
    let corner_ids = [
        corner(nua, nva, &mut solid, &mut next_id)?,
        corner(nub, nva, &mut solid, &mut next_id)?,
        corner(nub, nvb, &mut solid, &mut next_id)?,
        corner(nua, nvb, &mut solid, &mut next_id)?,
    ];
    let mut rebuilt = face.clone();
    rebuilt.surface = grown.clone();
    let mut new_edges: HashMap<u64, u64> = HashMap::default();
    for (coedge, (along_u, at)) in rebuilt.loops[0].coedges.iter_mut().zip(&isolines) {
        let slot = slot_of(*along_u, *at);
        if !changed(slot) {
            continue;
        }
        let flat = |u: f64, v: f64| Vec3::new(u, v, 0.0);
        let (curve, start, end, from, to, t0, t1) = match slot {
            0 | 1 => {
                let u = reach[slot];
                let (start, end) = if slot == 0 { (corner_ids[0], corner_ids[3]) } else { (corner_ids[1], corner_ids[2]) };
                (grown.iso_curve_u(u).or_refuse(KernelStage::Refine, "iso_curve_u")?, start, end, flat(u, nva), flat(u, nvb), nva, nvb)
            }
            _ => {
                let v = reach[slot];
                let (start, end) = if slot == 2 { (corner_ids[0], corner_ids[1]) } else { (corner_ids[3], corner_ids[2]) };
                (grown.iso_curve_v(v).or_refuse(KernelStage::Refine, "iso_curve_v")?, start, end, flat(nua, v), flat(nub, v), nua, nub)
            }
        };
        let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let (a, b) = (coedge.pcurve.evaluate(q0).or_refuse(KernelStage::Refine, "evaluate")?, coedge.pcurve.evaluate(q1).or_refuse(KernelStage::Refine, "evaluate")?);
        let increasing = if *along_u { b.y > a.y } else { b.x > a.x };
        // A seam used twice stays one edge; every use runs its isoline the
        // way the edge's curve does when its parameter increases.
        let edge_id = match new_edges.get(&coedge.edge_id) {
            Some(&edge_id) => edge_id,
            None => {
                let edge_id = next_id;
                next_id += 1;
                solid.edges.push(EdgeRecord { id: edge_id, curve, t0, t1, start_vertex_id: start, end_vertex_id: end, degenerate: false, name: None });
                new_edges.insert(coedge.edge_id, edge_id);
                edge_id
            }
        };
        coedge.edge_id = edge_id;
        coedge.forward = increasing;
        coedge.pcurve = if increasing { crate::make_line(from, to).or_refuse(KernelStage::Refine, "make_line")? } else { crate::make_line(to, from).or_refuse(KernelStage::Refine, "make_line")? };
    }
    for shell in &mut solid.shells {
        for existing in &mut shell.faces {
            if existing.id == face.id {
                *existing = rebuilt.clone();
            }
        }
    }
    WIDENED.with(|count| count.set(count.get() + 1));
    os_debug!(
        "outward side widening: source face {} widened to u = [{nua:.6}, {nub:.6}], v = [{nva:.6}, {nvb:.6}] for its sharp convex junction(s)",
        face.id
    );
    Ok(Some(solid))
}

/// `surface`, closed along `along_u` (else along v), with its seam moved to
/// the parameter line `cut`: the part past `cut` keeps its parameters and the
/// part before it follows on after the old end, one period later (`forward`),
/// or the part before `cut` keeps its parameters and the part past it comes
/// first, one period earlier. Exact (knot insertion, then the two pieces'
/// poles and knots end to end at a C0 join, the old seam). `None` when the
/// two ends are not the same poles.
fn reseamed(surface: &NurbsSurface, along_u: bool, cut: f64, forward: bool) -> Result<Option<NurbsSurface>, String> {
    let (before, after) = crate::split_surface(surface, !along_u, cut)?;
    let degree = if along_u { surface.degree_u } else { surface.degree_v };
    let knots = |part: &NurbsSurface| if along_u { part.knots_u.clone() } else { part.knots_v.clone() };
    let lines = |part: &NurbsSurface| -> Vec<Vec<Vec4>> {
        if along_u {
            part.control_points.clone()
        } else {
            let columns = part.control_points[0].len();
            (0..columns).map(|j| part.control_points.iter().map(|row| row[j]).collect()).collect()
        }
    };
    let [low, high] = if along_u { surface.domain_u()? } else { surface.domain_v()? };
    let period = high - low;
    let (first, second, shift_first, shift_second) =
        if forward { (&after, &before, 0.0, period) } else { (&after, &before, -period, 0.0) };
    let (first_lines, second_lines) = (lines(first), lines(second));
    // The join: the first piece's last line must be the second's first.
    let (last, start) = (first_lines.last().ok_or("empty net")?, second_lines.first().ok_or("empty net")?);
    let scale = last.iter().map(|pole| pole.x.abs().max(pole.y.abs()).max(pole.z.abs())).fold(1.0, f64::max);
    if last.len() != start.len()
        || last.iter().zip(start).any(|(p, q)| (p.x - q.x).abs() + (p.y - q.y).abs() + (p.z - q.z).abs() + (p.w - q.w).abs() > 1e-12 * scale)
    {
        return Ok(None);
    }
    let (first_knots, second_knots) = (knots(first), knots(second));
    let mut joined_knots: Vec<f64> = first_knots[..first_knots.len() - 1].iter().map(|k| k + shift_first).collect();
    joined_knots.extend(second_knots[degree + 1..].iter().map(|k| k + shift_second));
    let mut joined_lines = first_lines.clone();
    joined_lines.extend(second_lines[1..].iter().cloned());
    if along_u {
        Ok(Some(NurbsSurface::new(surface.degree_u, surface.degree_v, joined_knots, surface.knots_v.clone(), joined_lines)?))
    } else {
        let rows = joined_lines[0].len();
        let net = (0..rows).map(|i| joined_lines.iter().map(|line| line[i]).collect()).collect();
        Ok(Some(NurbsSurface::new(surface.degree_u, surface.degree_v, surface.knots_u.clone(), joined_knots, net)?))
    }
}

/// Whether an OUTWARD padded plane's slid trim falls short of its pad: the
/// net slides under trim loops cloned in uv, which scales the outline about
/// the slide's centre, and a half disc (a line closed by an arc) scales into
/// an ellipse lying INSIDE the offset arc between its ends (0.40 of a 0.5 pad
/// at 45 degrees), short of the neighbour's offset cylinder. Every junction
/// of the face must be sharp and convex (a smooth one keeps its edge-to-edge
/// synchronisation with the trim it has). Read on the trim's edges: each
/// sample moved back onto the source plane must lie at least the pad (less
/// 2%) from the source outline.
pub(super) fn slid_trim_falls_short(
    source: &BrepSolid,
    face: &FaceRecord,
    source_faces: &[&FaceRecord],
    carrier: &BrepSolid,
    distance: f64,
    pad: f64,
) -> Result<bool, KernelRefusal> {
    if distance >= 0.0 || !outward_side_widen_on() || !(pad > 0.0) {
        return Ok(false);
    }
    let edge_by_id = source.edges.iter().map(|edge| (edge.id, edge)).collect::<HashMap<_, _>>();
    let mut outline = Vec::new();
    for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
        let Some(edge) = edge_by_id.get(&coedge.edge_id) else { return Ok(false) };
        if edge.degenerate {
            continue;
        }
        let Some((mate, mate_use)) = junction_mate(source_faces, face, coedge.edge_id) else { return Ok(false) };
        if uses_are_tangent(face, coedge, mate, mate_use)? {
            return Ok(false);
        }
        let stations = junction_stations(face, coedge, edge, mate)?;
        if stations.is_empty() || stations.iter().any(|station| !station.is_convex()) {
            return Ok(false);
        }
        for k in 0..=128 {
            outline.push(edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 128.0).or_refuse(KernelStage::Refine, "evaluate")?);
        }
    }
    if outline.is_empty() {
        return Ok(false);
    }
    let seeds = super::source_seeds(source, face.id)?;
    let outward = face_normal(face, seeds[0].x, seeds[0].y)?;
    let back = outward.scale(distance);
    let mut shortest = f64::INFINITY;
    for edge in &carrier.edges {
        if edge.degenerate {
            continue;
        }
        for k in 0..=32 {
            let point = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 32.0).or_refuse(KernelStage::Refine, "evaluate")?.add(back);
            let near = outline.iter().map(|q| q.sub(point).length()).fold(f64::INFINITY, f64::min);
            shortest = shortest.min(near);
        }
    }
    os_debug!("slid trim of source face {}: nearest approach to the source outline {shortest:.6} for a pad of {pad:.6}", face.id);
    Ok(shortest < 0.98 * pad)
}

/// Drop the edges and vertices of a one-face carrier that its loops do not use.
pub(super) fn keep_used_edges(carrier: &mut BrepSolid) {
    let used_edges = carrier
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect::<HashSet<_>>();
    carrier.edges.retain(|edge| used_edges.contains(&edge.id));
    let used_vertices = carrier.edges.iter().flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id]).collect::<HashSet<_>>();
    carrier.vertices.retain(|vertex| used_vertices.contains(&vertex.id));
}
