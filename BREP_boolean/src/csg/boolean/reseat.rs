//! Operand seam re-seat (lane X row 11b, `BREP_SEAM_RESEAT`).
//!
//! A cylinder grazing a planar wall of the other operand from inside touches
//! it along one generatrix, and the boolean cuts the cylinder's wall there.
//! When that generatrix is the wall's own seam (`tangent_on_seam_y10`), the
//! cut costs nothing. Anywhere else (`box_minus_tangent_cylinder`) the wall
//! comes out in two pieces that touch only across the old seam, 9F/21E/12V.
//! Moving the operand's seam onto the tangent generatrix BEFORE imprint gives
//! the first body at the first cost.
//!
//! The contact is closed form on the cached analytic carriers: a plane
//! parallel to a cylinder's axis at the radius from it. The re-seat is exact
//! by construction: the wall carrier, both rim circles and the caps' rim
//! pcurves each rotate by whole spans at a knot of multiplicity `degree`
//! (no basis function crosses it, and a closed curve's clamped ends join at
//! the same multiplicity), the seam edge becomes the carrier's new u = start
//! iso line, and the two seam vertices move onto the tangent points. Every id
//! and name is kept; the wall's own pcurves (its chart rectangle) are
//! unchanged.
//!
//! It moves a named edge and two vertices geometrically, so it runs only when
//! the caller says what follows: `BooleanOptions::seam_reseat_later_references`
//! lists the later features' references (their `inputParams` text), and any
//! of them naming the wall, a rim, the seam or a cap declines it. `None`
//! (every internal boolean, the raw API) declines.
//!
//! Scope: a primitive-shaped wall (one loop of rim, seam, rim, seam; closed
//! rims each shared with a one-coedge cap) whose contact sits on a
//! full-multiplicity knot, against exactly one tangent plane. Anything else
//! declines and the operand passes through unchanged.
use crate::topology::{BrepSolid, FaceRecord};
use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};
use rustc_hash::FxHashMap as HashMap;
use std::borrow::Cow;

/// The re-seat is on by default; `BREP_SEAM_RESEAT=0` (or `off`) disables it.
/// Promoted on the 239-document census (0 documents differ) and the tangent
/// bore's paired timing (8.05 -> 7.67 ms, 9F/21E/12V -> 8F/18E/10V).
pub(super) fn seam_reseat_enabled() -> bool {
    switch!("BREP_SEAM_RESEAT", true)
}


/// Which contacts a re-seat reads: a plane of the other operand along a
/// generatrix (the original re-seat), and a cylinder of the other operand
/// tangent from outside (`BREP_SEAM_RESEAT_CYLINDERS`).
#[derive(Clone, Copy)]
pub(super) struct ContactKinds {
    pub(super) planes: bool,
    pub(super) cylinders: bool,
}

/// `BREP_SEAM_RESEAT_CYLINDERS` (default on; `off`/`0`, `check`): a
/// primitive-shaped cylinder wall that one cylinder of the other operand
/// touches from outside along a generatrix has its seam re-seated onto that
/// line before imprint, as a plane's contact does. Two holes cut one after
/// the other, tangent along a line: where the contact lay on the earlier
/// hole's own seam, the imprint left that wall unsplit and the body failed
/// its Euler check; with the later hole's seam on the contact the line pairs
/// as the plane case does. Under it the raw API (no later references given)
/// re-seats these cylinder walls only; the plane re-seat keeps its gate.
/// `check` builds both ways, prints one `boolean.seam_reseat_cylinders`
/// line, and returns the body built without it. With the retry
/// ([`seam_reseat_retry_enabled`]) a boolean that builds as given is never
/// re-seated, so on by default it acts only where the plain boolean refuses.
pub(super) fn seam_reseat_cylinders_mode() -> crate::env_read::Mode {
    crate::mode_switch!("BREP_SEAM_RESEAT_CYLINDERS", true)
}


/// Both operands, each with every tangent cylinder wall the gate admits
/// re-seated on its contact line; an operand nothing applies to is borrowed.
pub(super) fn reseat_tangent_seams<'a>(
    first: &'a BrepSolid,
    second: &'a BrepSolid,
    later: &[String],
    tolerance: f64,
) -> (Cow<'a, BrepSolid>, Cow<'a, BrepSolid>) {
    reseat_tangent_seams_of(first, second, later, tolerance, ContactKinds { planes: true, cylinders: seam_reseat_cylinders_mode() == crate::env_read::Mode::On })
}

/// [`reseat_tangent_seams`] for the given contact kinds.
pub(super) fn reseat_tangent_seams_of<'a>(
    first: &'a BrepSolid,
    second: &'a BrepSolid,
    later: &[String],
    tolerance: f64,
    kinds: ContactKinds,
) -> (Cow<'a, BrepSolid>, Cow<'a, BrepSolid>) {
    let weld = crate::tolerance::assembler_weld(tolerance);
    let before = TOPOLOGY_WORK.with(|work| work.get());
    let result = (
        reseat_operand(first, second, later, weld, kinds).map_or(Cow::Borrowed(first), Cow::Owned),
        reseat_operand(second, first, later, weld, kinds).map_or(Cow::Borrowed(second), Cow::Owned),
    );
    if crate::profile::profile_enabled() {
        let after = TOPOLOGY_WORK.with(|work| work.get());
        profile_line!("boolean.seam_topology censuses={} coedges={}", after.0 - before.0, after.1 - before.1);
    }
    result
}

fn referenced(later: &[String], name: Option<&String>) -> bool {
    match name {
        Some(name) => later.iter().any(|text| text.contains(name.as_str())),
        // An unnamed record can only be referenced by id, which a non-empty
        // reference list cannot rule out.
        None => !later.is_empty(),
    }
}

/// A closed periodic curve or row list rotated to start at knot `c` (an
/// interior knot of multiplicity `degree`): `None` unless every moved knot
/// round-trips bit for bit.
pub(crate) fn rotate_closed<T: Clone>(knots: &[f64], degree: usize, items: &[T], c: f64) -> Option<(Vec<f64>, Vec<T>)> {
    let (k0, k1) = (*knots.first()?, *knots.last()?);
    if knots.len() != items.len() + degree + 1 || knots.iter().filter(|&&knot| knot == c).count() != degree {
        return None;
    }
    let first = knots.iter().position(|&knot| knot == c)?;
    let (to_high, to_low) = (c - k0, k1 - c);
    let moved = |value: f64, shift: f64, sign: f64| {
        let out = value - sign * shift;
        (out + sign * shift == value).then_some(out)
    };
    let mut rotated = vec![k0; degree + 1];
    for &knot in &knots[first + degree..knots.len() - degree - 1] {
        rotated.push(moved(knot, to_high, 1.0)?);
    }
    let joint = moved(k1, to_high, 1.0)?;
    rotated.extend(std::iter::repeat(joint).take(degree));
    for &knot in &knots[degree + 1..first] {
        rotated.push(moved(knot, to_low, -1.0)?);
    }
    rotated.extend(std::iter::repeat(k1).take(degree + 1));
    if rotated.len() != knots.len() || rotated.windows(2).any(|pair| pair[0] > pair[1]) {
        return None;
    }
    let row = first - 1;
    Some((rotated, items[row..].iter().chain(&items[1..=row]).cloned().collect()))
}

/// The interior knots of multiplicity `degree`.
pub(crate) fn full_knots(knots: &[f64], degree: usize) -> Vec<(f64, usize)> {
    let mut found: Vec<(f64, usize)> = Vec::new();
    for (index, &knot) in knots.iter().enumerate().skip(degree + 1).take(knots.len().saturating_sub(2 * degree + 2)) {
        if found.last().is_some_and(|&(last, _)| last == knot) {
            continue;
        }
        if knots.iter().filter(|&&other| other == knot).count() == degree {
            found.push((knot, index - 1));
        }
    }
    found
}

/// `curve` (closed) rotated to start where it passes through `point`, at a
/// full-multiplicity knot whose control point maps (through `chart`) within
/// `weld` of it.
fn rotate_curve_to(curve: &NurbsCurve, point: Vec3, chart: impl Fn(Vec4) -> Option<Vec3>, weld: f64) -> Option<NurbsCurve> {
    let (first, last) = (curve.control_points.first()?, curve.control_points.last()?);
    if chart(*first)?.sub(chart(*last)?).length() > weld {
        return None;
    }
    let hits: Vec<f64> = full_knots(&curve.knots, curve.degree)
        .into_iter()
        .filter(|&(_, row)| chart(curve.control_points[row]).is_some_and(|at| at.sub(point).length() <= weld))
        .map(|(knot, _)| knot)
        .collect();
    let [c] = hits.as_slice() else { return None };
    let (knots, controls) = rotate_closed(&curve.knots, curve.degree, &curve.control_points, *c)?;
    NurbsCurve::new(curve.degree, knots, controls).ok()
}

struct Contact {
    /// The tangent generatrix: a point and the axis direction.
    point: Vec3,
    axis: Vec3,
}

/// The one planar face of `other` tangent to the cylinder `wall` along a
/// generatrix inside both faces' extents, or `None` (no plane, or two).
fn tangent_contact(wall: &FaceRecord, other: &BrepSolid, weld: f64) -> Option<Contact> {
    let Some(crate::AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) = wall.surface.analytic() else {
        return None;
    };
    if rho0 != rho1 {
        return None;
    }
    let (radius, axis, height) = (*rho0, frame.axis, *height);
    let mut contacts = Vec::new();
    for face in other.shells.iter().flat_map(|shell| &shell.faces) {
        let Some(crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) = face.surface.analytic() else {
            continue;
        };
        let Ok(normal) = u_dir.cross(*v_dir).normalized() else { continue };
        if normal.dot(axis).abs() > weld / height.max(1.0) {
            continue;
        }
        let offset = frame.origin.sub(*origin).dot(normal);
        if (offset.abs() - radius).abs() > weld {
            continue;
        }
        let toward = normal.scale(-offset.signum());
        let point = frame.origin.add(toward.scale(radius));
        // The generatrix must cross the planar face's extent, read from its
        // boundary vertices' bounding box (padded by the weld).
        let points: Vec<Vec3> = face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .filter_map(|coedge| other.edges.iter().find(|edge| edge.id == coedge.edge_id))
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .filter_map(|id| other.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point))
            .collect();
        if points.is_empty() {
            continue;
        }
        let lo = points.iter().fold(Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), |a, p| Vec3::new(a.x.min(p.x), a.y.min(p.y), a.z.min(p.z)));
        let hi = points.iter().fold(Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY), |a, p| Vec3::new(a.x.max(p.x), a.y.max(p.y), a.z.max(p.z)));
        let inside = |p: Vec3| {
            p.x >= lo.x - weld && p.x <= hi.x + weld && p.y >= lo.y - weld && p.y <= hi.y + weld && p.z >= lo.z - weld && p.z <= hi.z + weld
        };
        let samples = (0..=4).map(|index| point.add(axis.scale(height * index as f64 / 4.0)));
        if samples.filter(|&p| inside(p)).count() < 2 {
            continue;
        }
        contacts.push(Contact { point, axis });
    }
    match contacts.len() {
        1 => contacts.pop(),
        _ => None,
    }
}

/// The one cylinder face of `other` tangent to the cylinder `wall` from
/// outside along a generatrix (parallel axes, the axes the radii's sum apart
/// within `weld`), or `None` (no such face, or two). The contact line runs
/// through `wall`'s axis point toward the other's.
fn cylinder_contact(wall: &FaceRecord, other: &BrepSolid, weld: f64) -> Option<Contact> {
    let Some(crate::AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. }) = wall.surface.analytic() else {
        return None;
    };
    if rho0 != rho1 {
        return None;
    }
    let mut contacts = Vec::new();
    for face in other.shells.iter().flat_map(|shell| &shell.faces) {
        let Some(crate::AnalyticSurface::RuledRevolution { frame: theirs, rho0: r0, rho1: r1, .. }) = face.surface.analytic() else {
            continue;
        };
        if r0 != r1 || frame.axis.cross(theirs.axis).length() > 1e-9 {
            continue;
        }
        let apart = theirs.origin.sub(frame.origin);
        let apart = apart.sub(frame.axis.scale(apart.dot(frame.axis)));
        if (apart.length() - (rho0 + r0)).abs() > weld {
            continue;
        }
        let Ok(toward) = apart.normalized() else { continue };
        let point = frame.origin.add(toward.scale(*rho0));
        // Whether the other face already has an edge on the contact line (its
        // seam, or a line an earlier cut left there).
        let edged = face.loops.iter().flat_map(|loop_record| &loop_record.coedges).any(|coedge| {
            other.edges.iter().find(|edge| edge.id == coedge.edge_id).is_some_and(|edge| {
                !edge.degenerate
                    && [edge.t0, edge.t1, 0.5 * (edge.t0 + edge.t1)].iter().all(|&t| {
                        edge.curve.evaluate(t).is_ok_and(|at| {
                            let delta = at.sub(point);
                            delta.sub(frame.axis.scale(delta.dot(frame.axis))).length() <= weld
                        })
                    })
            })
        });
        contacts.push((Contact { point, axis: frame.axis }, edged));
    }
    if edged_contact_choice_enabled() {
        return ranked_seam_line(wall, frame.axis, *rho0, &contacts, other, weld);
    }
    match contacts.len() {
        1 => contacts.pop().map(|(contact, _)| contact),
        _ => None,
    }
}

/// The seam line to re-seat a cylinder wall onto, ranked over the carrier's
/// full-multiplicity u knots (the lines a re-seat can reach exactly):
///
/// 0. a contact where the other operand already has an edge on the line (its
///    seam, or a line an earlier cut left): this wall needs an edge there to
///    pair with;
/// 1. a free line: no contact, and no straight edge of the other operand
///    passing within the near band of it;
/// 2. a contact inside the other face: it builds with or without a seam on it;
/// 3. a near miss: a straight edge of the other operand passing within the
///    near band but farther than the weld. A seam there puts its end vertices
///    beside that edge, the imprint takes them as junctions on it, and the
///    web between them closes at a point; a hole cut later beside this one
///    then fails to assemble.
///
/// The wall is re-seated only when some line ranks better than its current
/// seam (u = start), onto the first best one. Holes cut one at a time in a
/// zero-web grid touch two earlier holes, one of them on that hole's seam,
/// and the outer rows sit a hair inside the plate's sides.
fn ranked_seam_line(wall: &FaceRecord, axis: Vec3, radius: f64, contacts: &[(Contact, bool)], other: &BrepSolid, weld: f64) -> Option<Contact> {
    let surface = &wall.surface;
    let near = 1e-3 * radius.max(1.0);
    let off_line = |point: Vec3, on: Vec3| {
        let delta = point.sub(on);
        delta.sub(axis.scale(delta.dot(axis))).length()
    };
    let segments: Vec<(Vec3, Vec3)> = other
        .edges
        .iter()
        .filter(|edge| !edge.degenerate && edge.curve.degree == 1 && edge.curve.control_points.len() == 2)
        .filter_map(|edge| Some((edge.curve.evaluate(edge.t0).ok()?, edge.curve.evaluate(edge.t1).ok()?)))
        .collect();
    // The least distance between the line through `on` along the axis and a
    // segment, read in the plane normal to the axis.
    let flat = |point: Vec3| point.sub(axis.scale(point.dot(axis)));
    let segment_gap = |on: Vec3, (a, b): (Vec3, Vec3)| -> f64 {
        let (p, a, b) = (flat(on), flat(a), flat(b));
        let ab = b.sub(a);
        let length = ab.dot(ab);
        let t = if length <= f64::EPSILON { 0.0 } else { (p.sub(a).dot(ab) / length).clamp(0.0, 1.0) };
        p.sub(a.add(ab.scale(t))).length()
    };
    let rank = |on: Vec3| -> u8 {
        if let Some((_, edged)) = contacts.iter().find(|(contact, _)| off_line(contact.point, on) <= weld) {
            return if *edged { 0 } else { 2 };
        }
        let gap = segments.iter().map(|&segment| segment_gap(on, segment)).fold(f64::INFINITY, f64::min);
        if gap > weld && gap <= near { 3 } else { 1 }
    };
    let row_point = |row: usize| surface.control_points[row].first().and_then(|control| control.point().ok());
    let current = rank(row_point(0)?);
    let mut best: Option<(u8, Vec3)> = None;
    for (_, row) in full_knots(&surface.knots_u, surface.degree_u) {
        let Some(point) = row_point(row) else { continue };
        let score = rank(point);
        if best.map_or(true, |(known, _)| score < known) {
            best = Some((score, point));
        }
    }
    let (score, point) = best?;
    (score < current).then_some(Contact { point, axis })
}

/// `BREP_SEAM_RESEAT_CYLINDERS_RETRY` (default on; acts only under
/// [`seam_reseat_cylinders_mode`]): the boolean runs on the operands as given
/// first, and the re-seat is computed and built (with
/// `BREP_COSURFACE_TANGENT_CYLINDERS` on) only when that refuses; `check`
/// prints what the retry gives and returns the first attempt. Measured on
/// the dense-holes grids, a re-seat applied at every cut moves seams that a
/// later cut meets, and that cut then fails where the plain body would have
/// built; every single step built from a plain operand.
pub(super) fn seam_reseat_retry_enabled() -> bool {
    switch!("BREP_SEAM_RESEAT_CYLINDERS_RETRY", true)
}

/// `BREP_SEAM_RESEAT_EDGED_CONTACT` (default on; acts only under
/// [`seam_reseat_cylinders_mode`]): the cylinder re-seat chooses its line by
/// [`ranked_seam_line`] instead of re-seating onto a lone contact. `=0`
/// restores the lone-contact rule.
fn edged_contact_choice_enabled() -> bool {
    switch!("BREP_SEAM_RESEAT_EDGED_CONTACT", true)
}

// Re-seating changes carrier/curve/pcurve coordinates, never record order,
// ids or coedge incidence. These readings therefore remain valid across all
// admitted walls of one operand. Keep the first slot for duplicate face ids,
// matching the old linear `find` even on malformed input.
struct ReseatTopology {
    face_slots: HashMap<u64, (usize, usize)>,
    uses: HashMap<u64, usize>,
}

thread_local! {
    static TOPOLOGY_WORK: std::cell::Cell<(u64, u64)> = const { std::cell::Cell::new((0, 0)) };
}

fn edge_uses(solid: &BrepSolid) -> HashMap<u64, usize> {
    let mut uses = HashMap::default();
    let mut visits = 0;
    for coedge in solid.shells.iter().flat_map(|s| &s.faces).flat_map(|f| &f.loops).flat_map(|l| &l.coedges) {
        *uses.entry(coedge.edge_id).or_default() += 1;
        visits += 1;
    }
    TOPOLOGY_WORK.with(|work| { let (calls, reads) = work.get(); work.set((calls + 1, reads + visits)); });
    uses
}

impl ReseatTopology {
    fn new(solid: &BrepSolid) -> Self {
        let mut face_slots = HashMap::default();
        for (shell, record) in solid.shells.iter().enumerate() {
            for (face, record) in record.faces.iter().enumerate() {
                face_slots.entry(record.id).or_insert((shell, face));
            }
        }
        Self { face_slots, uses: edge_uses(solid) }
    }
}

fn topology_memo_enabled() -> bool {
    switch!("BREP_RESEAT_TOPOLOGY_MEMO", true)
}

fn reseat_operand(solid: &BrepSolid, other: &BrepSolid, later: &[String], weld: f64, kinds: ContactKinds) -> Option<BrepSolid> {
    reseat_operand_indexed(solid, other, later, weld, topology_memo_enabled(), kinds)
}

fn reseat_operand_indexed(solid: &BrepSolid, other: &BrepSolid, later: &[String], weld: f64, indexed: bool, kinds: ContactKinds) -> Option<BrepSolid> {
    let topology = indexed.then(|| ReseatTopology::new(solid));
    let mut result: Option<BrepSolid> = None;
    let faces: Vec<u64> = solid.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).collect();
    for face_id in faces {
        let current = result.as_ref().unwrap_or(solid);
        if let Some(reseated) = reseat_wall(current, face_id, other, later, weld, topology.as_ref(), kinds) {
            result = Some(reseated);
        }
    }
    result
}

fn reseat_wall(solid: &BrepSolid, face_id: u64, other: &BrepSolid, later: &[String], weld: f64, topology: Option<&ReseatTopology>, kinds: ContactKinds) -> Option<BrepSolid> {
    // The diagnostic hatch retains the original collect/find and census work.
    let faces: Option<Vec<&FaceRecord>> = topology.is_none().then(|| solid.shells.iter().flat_map(|shell| &shell.faces).collect());
    let wall = if let Some(topology) = topology {
        let &(shell, face) = topology.face_slots.get(&face_id)?;
        &solid.shells[shell].faces[face]
    } else { *faces.as_ref()?.iter().find(|face| face.id == face_id)? };
    // Structure: one loop of rim, seam, rim, seam (either order), the seam
    // edge used only here, each rim closed and shared with one cap whose
    // single loop is that rim alone.
    let [only] = wall.loops.as_slice() else { return None };
    if only.coedges.len() != 4 {
        return None;
    }
    let uncached_uses = topology.is_none().then(|| edge_uses(solid));
    let uses = topology.map(|t| &t.uses).or(uncached_uses.as_ref())?;
    let edge = |id: u64| solid.edges.iter().find(|edge| edge.id == id);
    let ids: Vec<u64> = only.coedges.iter().map(|coedge| coedge.edge_id).collect();
    let seam = *ids.iter().find(|id| ids.iter().filter(|other| other == id).count() == 2)?;
    let rims: Vec<u64> = ids.iter().copied().filter(|id| *id != seam).collect();
    let [rim0, rim1] = rims.as_slice() else { return None };
    if rim0 == rim1 || uses.get(&seam) != Some(&2) {
        return None;
    }
    let seam_edge = edge(seam)?;
    let mut caps = Vec::new();
    for &rim in [rim0, rim1] {
        let record = edge(rim)?;
        if uses.get(&rim) != Some(&2) || record.start_vertex_id != record.end_vertex_id || record.degenerate {
            return None;
        }
        let cap = solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| {
            face.id != wall.id && face.loops.iter().flat_map(|loop_record| &loop_record.coedges).any(|coedge| coedge.edge_id == rim)
        })?;
        if cap.loops.len() != 1 || cap.loops[0].coedges.len() != 1 {
            return None;
        }
        caps.push(cap);
    }
    let seam_vertices = [seam_edge.start_vertex_id, seam_edge.end_vertex_id];
    let rim_vertices = [edge(*rim0)?.start_vertex_id, edge(*rim1)?.start_vertex_id];
    if !(seam_vertices == rim_vertices || seam_vertices == [rim_vertices[1], rim_vertices[0]]) {
        return None;
    }
    // References: the wall, its caps, its rims and seam, by name.
    if referenced(later, wall.name.as_ref())
        || caps.iter().any(|cap| referenced(later, cap.name.as_ref()))
        || [seam, *rim0, *rim1].iter().any(|&id| edge(id).is_some_and(|record| record.name.is_some() && referenced(later, record.name.as_ref())))
    {
        return None;
    }
    let contact = kinds
        .planes
        .then(|| tangent_contact(wall, other, weld))
        .flatten()
        .or_else(|| kinds.cylinders.then(|| cylinder_contact(wall, other, weld)).flatten())?;
    // The carrier's full-multiplicity u knot whose control row lies on the
    // contact line at both ends; the one re-seat that is exact.
    let surface = &wall.surface;
    let on_line = |point: Vec3| {
        let delta = point.sub(contact.point);
        delta.sub(contact.axis.scale(delta.dot(contact.axis))).length() <= weld
    };
    let hits: Vec<f64> = full_knots(&surface.knots_u, surface.degree_u)
        .into_iter()
        .filter(|&(_, row)| {
            let column = &surface.control_points[row];
            [column.first(), column.last()].iter().all(|control| control.and_then(|c| c.point().ok()).is_some_and(on_line))
        })
        .map(|(knot, _)| knot)
        .collect();
    let [c] = hits.as_slice() else { return None };
    let (knots_u, rows) = rotate_closed(&surface.knots_u, surface.degree_u, &surface.control_points, *c)?;
    let rotated = NurbsSurface::new(surface.degree_u, surface.degree_v, knots_u, surface.knots_v.clone(), rows).ok()?;
    let [k0, _] = rotated.domain_u().ok()?;
    let seam_curve = rotated.iso_curve_u(k0).ok()?;
    let mut out = solid.clone();
    // The two seam vertices onto the tangent points, the seam's ends in order.
    let new_points = [seam_curve.evaluate(seam_curve.domain().ok()?[0]).ok()?, seam_curve.evaluate(seam_curve.domain().ok()?[1]).ok()?];
    for vertex in &mut out.vertices {
        if vertex.id == seam_edge.start_vertex_id {
            vertex.point = new_points[0];
        } else if vertex.id == seam_edge.end_vertex_id {
            vertex.point = new_points[1];
        }
    }
    let point_of = |id: u64| out.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point);
    let rim_points: Vec<Vec3> = [rim0, rim1].iter().map(|&&rim| point_of(edge(rim).unwrap().start_vertex_id)).collect::<Option<_>>()?;
    // Rims and caps, each rotated to start at its new vertex.
    let mut rim_curves = HashMap::<u64, NurbsCurve>::default();
    for (&rim, &point) in [rim0, rim1].iter().zip(&rim_points) {
        let record = edge(*rim)?;
        let [t0, t1] = record.curve.domain().ok()?;
        if record.t0 != t0 || record.t1 != t1 {
            return None;
        }
        rim_curves.insert(*rim, rotate_curve_to(&record.curve, point, |control| control.point().ok(), weld)?);
    }
    for record in &mut out.edges {
        if record.id == seam {
            let [t0, t1] = seam_curve.domain().ok()?;
            record.curve = seam_curve.clone();
            record.t0 = t0;
            record.t1 = t1;
        } else if let Some(curve) = rim_curves.get(&record.id) {
            let [t0, t1] = curve.domain().ok()?;
            record.curve = curve.clone();
            record.t0 = t0;
            record.t1 = t1;
        }
    }
    for shell in &mut out.shells {
        for face in &mut shell.faces {
            if face.id == wall.id {
                face.surface = rotated.clone();
            } else if let Some(position) = caps.iter().position(|cap| cap.id == face.id) {
                let point = rim_points[position];
                let cap_surface = face.surface.clone();
                let coedge = &mut face.loops[0].coedges[0];
                let chart = |control: Vec4| -> Option<Vec3> {
                    if control.w.abs() <= f64::EPSILON {
                        return None;
                    }
                    cap_surface.evaluate(control.x / control.w, control.y / control.w).ok()
                };
                coedge.pcurve = rotate_curve_to(&coedge.pcurve, point, chart, weld)?;
            }
        }
    }
    out.validate().is_empty().then_some(out)
}

