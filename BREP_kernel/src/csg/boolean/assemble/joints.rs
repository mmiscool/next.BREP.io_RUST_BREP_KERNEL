//! Local trim-joint repair after carrier vertex settling and edge reconstruction.
//! Affine trims use the exact edge image. Curved trims keep their interiors:
//! only restriction, conic recutting or a bounded end-tangent extension is used.
//! No station refit is permitted. Curved-face area and soundness changes are
//! checked transactionally; failure keeps the previous geometry.

use crate::{
    fit_pcurve_on_surface_range, BrepSolid, KernelRefusal, KernelStage, KernelTolerances,
    NurbsCurve, NurbsSurface, OrRefuse, Vec3, Vec4,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
#[path = "paired.rs"]
mod paired;

/// The joint bar: how far a trim's end may sit from the image of the vertex
/// its loop passes through. The default kernel policy's MODEL tolerance,
/// which is the floor the assembler's own section trims are fitted to
/// (`finalize.rs` reads the same `for_solid(solid, 1e-7)` policy); a caller
/// whose assembly tolerance is tighter tightens it. It is a TRIGGER for a
/// local repair, never an acceptance band: nothing that met the bar before
/// this pass is changed by it.
pub(in crate::boolean) fn trim_joint_bar(solid: &BrepSolid, tolerance: f64) -> f64 {
    KernelTolerances::for_solid(solid, 1e-7)
        .model
        .min(tolerance.max(1e-12))
}

/// One trim end at one loop joint.
#[derive(Clone, Copy)]
struct End {
    face: usize,
    loop_index: usize,
    coedge: usize,
    /// Distance from the trim end's image to the shared vertex.
    miss: f64,
}

fn face_at(shells: &[crate::topology::ShellRecord], face: usize) -> &crate::FaceRecord {
    let mut index = face;
    for shell in shells {
        if index < shell.faces.len() {
            return &shell.faces[index];
        }
        index -= shell.faces.len();
    }
    unreachable!("face index {face} out of range")
}

fn face_at_mut(shells: &mut [crate::topology::ShellRecord], face: usize) -> &mut crate::FaceRecord {
    let mut index = face;
    for shell in shells {
        if index < shell.faces.len() {
            return &mut shell.faces[index];
        }
        index -= shell.faces.len();
    }
    unreachable!("face index {face} out of range")
}

fn image(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    parameter: f64,
) -> Result<Vec3, KernelRefusal> {
    let uv = pcurve
        .evaluate(parameter)
        .or_refuse(KernelStage::Sew, "evaluate")?;
    surface
        .evaluate_extended(uv.x, uv.y)
        .or_refuse(KernelStage::Sew, "evaluate_extended")
}

/// Distance to an evaluated point on the old trim image. Using a chord of
/// that image can underestimate movement or mistake chord sag for conic
/// displacement. This bounded one-dimensional refinement returns an actual
/// image witness; it is not a global minimum certificate.
fn image_distance(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    target: Vec3,
) -> Result<f64, KernelRefusal> {
    let [lo, hi] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let mut best = (f64::INFINITY, lo);
    for t in super::edges::carrier_stations(curve, lo, hi) {
        let distance = image(surface, curve, t)?.sub(target).length();
        if distance < best.0 {
            best = (distance, t);
        }
    }
    let mut t = best.1;
    for _ in 0..24 {
        let d = curve
            .derivatives(t, 1)
            .or_refuse(KernelStage::Sew, "derivatives")?;
        let sf = surface
            .derivatives_extended(d[0].x, d[0].y, 1)
            .or_refuse(KernelStage::Sew, "derivatives_extended")?;
        let tangent = sf[1][0].scale(d[1].x).add(sf[0][1].scale(d[1].y));
        let speed = tangent.dot(tangent);
        if !(speed > 1e-300) {
            break;
        }
        let step = sf[0][0].sub(target).dot(tangent) / speed;
        let mut improved = false;
        for factor in [1.0, 0.5, 0.25, 0.125] {
            let trial = (t - step * factor).clamp(lo, hi);
            let distance = image(surface, curve, trial)?.sub(target).length();
            if distance < best.0 {
                best = (distance, trial);
                t = trial;
                improved = true;
                break;
            }
        }
        if !improved {
            break;
        }
    }
    Ok(best.0)
}

/// Shift `rebuilt` by whole periods per closed direction so it sits on the
/// branch of `previous` (the mean-shift rule `imprint/junctions.rs` uses for
/// its canonicalised junction pcurves): a fit projects onto the canonical
/// branch, and a seam-adjacent trim moved a period would leave the loop.
fn align_period_branch(
    surface: &NurbsSurface,
    rebuilt: &mut NurbsCurve,
    previous: &NurbsCurve,
) -> Result<(), KernelRefusal> {
    let (closed_u, closed_v) = surface
        .closed_directions()
        .or_refuse(KernelStage::Sew, "closed_directions")?;
    if !closed_u && !closed_v {
        return Ok(());
    }
    let mean = |pcurve: &NurbsCurve| -> Option<(f64, f64)> {
        let (mut sum_u, mut sum_v, mut count) = (0.0, 0.0, 0.0);
        for point in &pcurve.control_points {
            if point.w.abs() > 1e-300 {
                sum_u += point.x / point.w;
                sum_v += point.y / point.w;
                count += 1.0;
            }
        }
        (count > 0.0).then(|| (sum_u / count, sum_v / count))
    };
    let (Some((old_u, old_v)), Some((new_u, new_v))) = (mean(previous), mean(rebuilt)) else {
        return Ok(());
    };
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
    let shift_u = if closed_u && u1 > u0 {
        ((old_u - new_u) / (u1 - u0)).round()
    } else {
        0.0
    };
    let shift_v = if closed_v && v1 > v0 {
        ((old_v - new_v) / (v1 - v0)).round()
    } else {
        0.0
    };
    if shift_u == 0.0 && shift_v == 0.0 {
        return Ok(());
    }
    for point in &mut rebuilt.control_points {
        point.x += shift_u * (u1 - u0) * point.w;
        point.y += shift_v * (v1 - v0) * point.w;
    }
    Ok(())
}

/// The parameter-space image of `point` on `surface`: its projection's (u, v).
fn uv_of(surface: &NurbsSurface, point: Vec3, near: Vec3) -> Result<Vec3, KernelRefusal> {
    let projection = crate::project_point_to_surface(surface, point)
        .or_refuse(KernelStage::Sew, "project_point_to_surface")?;
    let mut uv = Vec3::new(projection.u, projection.v, 0.0);
    let (cu, cv) = surface
        .closed_directions()
        .or_refuse(KernelStage::Sew, "closed_directions")?;
    if cu {
        let [a, b] = surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
        if b > a {
            uv.x += ((near.x - uv.x) / (b - a)).round() * (b - a);
        }
    }
    if cv {
        let [a, b] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
        if b > a {
            uv.y += ((near.y - uv.y) / (b - a)).round() * (b - a);
        }
    }
    Ok(uv)
}

/// The EXACT trim with its start and/or end re-cut to the given (u, v)
/// targets: a one-span rational quadratic is re-cut on its own conic (which
/// can also extend past the old end); any other pcurve is restricted to the
/// parameters nearest the targets when both lie inside its domain. `None`
/// when the trim cannot reach a target this way; the caller leaves it alone.
fn recut_pcurve(
    old: &NurbsCurve,
    start_target: Option<Vec3>,
    end_target: Option<Vec3>,
) -> Result<Option<NurbsCurve>, KernelRefusal> {
    if start_target.is_none() && end_target.is_none() {
        return Ok(None);
    }
    if let Some(conic) = super::edges::Conic::of(old) {
        let ua = match start_target {
            Some(target) => match conic.nearest(target, 0.0) {
                Some(u) => u,
                None => return Ok(None),
            },
            None => 0.0,
        };
        let ub = match end_target {
            Some(target) => match conic.nearest(target, 1.0) {
                Some(u) => u,
                None => return Ok(None),
            },
            None => 1.0,
        };
        return Ok(conic.arc(ua, ub));
    }
    // Any other pcurve: each requested end is either RESTRICTED to the
    // parameter nearest its target (target inside the trim) or EXTENDED to
    // the target along the end tangent (target beyond the trim's end). The
    // extension is a collinear span appended C0, so the trim's own shape is
    // untouched and the new stretch — a few 1e-6 of parameter space — leaves
    // the true curve by the curvature times that length squared.
    let mut curve = old.clone();
    if let Some(target) = end_target {
        curve = recut_end(&curve, target)?;
    }
    if let Some(target) = start_target {
        let reversed = curve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
        curve = recut_end(&reversed, target)?
            .reversed()
            .or_refuse(KernelStage::Sew, "reversed")?;
    }
    let [d0, d1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    if !(d1 > d0) {
        return Ok(None);
    }
    Ok(Some(curve))
}

/// `curve` with its END re-cut to `target`: restricted when the nearest
/// parameter lies inside the domain, extended along the end tangent when the
/// target lies beyond it.
fn recut_end(curve: &NurbsCurve, target: Vec3) -> Result<NurbsCurve, KernelRefusal> {
    let [d0, d1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let epsilon = ((d1 - d0).abs().max(1.0) * 1e-10).max(2e-9);
    let projection = crate::project_point_to_curve(curve, target)
        .or_refuse(KernelStage::Sew, "project_point_to_curve")?;
    if projection.u < d1 - epsilon {
        if projection.u <= d0 + epsilon {
            // The whole trim would go: not a re-cut.
            return Ok(curve.clone());
        }
        return Ok(curve
            .split(projection.u)
            .or_refuse(KernelStage::Sew, "split")?
            .0);
    }
    // Beyond the end: extend. Only a CLAMPED end (the last p + 1 knots equal
    // the domain end) takes a C0 span; an unclamped or periodic pcurve is
    // left alone.
    let degree = curve.degree;
    let knot_count = curve.knots.len();
    if knot_count < 2 * (degree + 1)
        || curve.knots[knot_count - degree - 1..]
            .iter()
            .any(|knot| (knot - d1).abs() > 0.0)
    {
        return Ok(curve.clone());
    }
    if curve.control_points.len() + degree + 1 != knot_count {
        return Ok(curve.clone());
    }
    let end_point = curve.evaluate(d1).or_refuse(KernelStage::Sew, "evaluate")?;
    let derivatives = curve
        .derivatives(d1, 1)
        .or_refuse(KernelStage::Sew, "derivatives")?;
    let tangent = derivatives
        .get(1)
        .copied()
        .unwrap_or(Vec3::new(0.0, 0.0, 0.0));
    let speed = tangent.length();
    if !(speed > 1e-300) {
        return Ok(curve.clone());
    }
    let direction = tangent.scale(1.0 / speed);
    let along = target.sub(end_point).dot(direction);
    if !(along > 0.0) {
        return Ok(curve.clone());
    }
    let foot = end_point.add(direction.scale(along));
    let dt = along / speed;
    if dt < 1e-6 * (d1 - d0) {
        // Too short for a span of its own: the knot-identity tolerance would
        // fold it onto the end and leave a zero-length span. Move the end
        // control point to the foot instead; the displacement bound judges
        // the (sub-1e-6-of-range) reshaping of the last span.
        let mut controls = curve.control_points.clone();
        let last = controls.len() - 1;
        controls[last] = Vec4::from_point(foot, controls[last].w);
        return NurbsCurve::new(curve.degree, curve.knots.clone(), controls)
            .or_refuse(KernelStage::Sew, "NurbsCurve::new");
    }
    let mut knots = curve.knots.clone();
    knots.truncate(knots.len() - 1);
    knots.extend(std::iter::repeat(d1 + dt).take(degree + 1));
    let mut controls = curve.control_points.clone();
    for index in 1..=degree {
        let fraction = index as f64 / degree as f64;
        let point = end_point.add(foot.sub(end_point).scale(fraction));
        controls.push(Vec4::from_point(point, 1.0));
    }
    NurbsCurve::new(degree, knots, controls).or_refuse(KernelStage::Sew, "NurbsCurve::new")
}

/// A trim this pass replaced, with the pcurve it replaced, so a caller whose
/// validation then disagrees can put it back.
pub(in crate::boolean) struct ReplacedTrim {
    face: usize,
    loop_index: usize,
    coedge: usize,
    previous: NurbsCurve,
    previous_edge: Option<super::edges::ReplacedEdge>,
}

/// A trim replacement record for another stage of this transaction (the
/// inherited-trim challenger), restored by [`restore_trims`].
pub(super) fn replaced_trim(face: usize, loop_index: usize, coedge: usize, previous: NurbsCurve) -> ReplacedTrim {
    ReplacedTrim { face, loop_index, coedge, previous, previous_edge: None }
}

/// Put back every trim in `replaced`.
pub(in crate::boolean) fn restore_trims(solid: &mut BrepSolid, replaced: Vec<ReplacedTrim>) {
    for item in replaced.into_iter().rev() {
        if let Some(edge) = item.previous_edge {
            super::edges::restore_edges(solid, vec![edge]);
        }
        face_at_mut(&mut solid.shells, item.face).loops[item.loop_index].coedges[item.coedge]
            .pcurve = item.previous;
    }
}

/// Close the trim-loop joints of `solid` that miss by more than the joint
/// bar. Returns the trims it rebuilt, with their previous pcurves. Never
/// refuses on a fit that cannot be brought on its bar: that trim is kept as
/// it was.
pub(in crate::boolean) fn close_trim_loop_joints(
    solid: &mut BrepSolid,
    tolerance: f64,
) -> Result<Vec<ReplacedTrim>, KernelRefusal> {
    let previous = solid.clone();
    let result = close_trim_loop_joints_impl(solid, tolerance);
    let Ok(replaced) = result else {
        *solid = previous;
        return Ok(Vec::new());
    };
    if replaced.is_empty() {
        return Ok(replaced);
    }
    let safe = topology_not_worse(&previous, solid);
    if !safe {
        *solid = previous;
        if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
            eprintln!("joints: restored transaction: validation or soundness worsened");
        }
        return Ok(Vec::new());
    }
    solid.mass_properties_cache = Default::default();
    Ok(replaced)
}

/// The scan is a regression guard, not a proof of absence below its chord
/// resolution. Unreadable/truncated scans cannot authorize a repair. Compare
/// stable entity identities, not totals which could hide a new crossing.
fn topology_not_worse(old: &BrepSolid, new: &BrepSolid) -> bool {
    let old_issues = old.validate();
    if new.validate().iter().any(|issue| {
        !old_issues.iter().any(|previous| {
            issue.kind == previous.kind
                && issue.severity == previous.severity
                && issue.message == previous.message
        })
    }) {
        return false;
    }
    let a = crate::loop_self_crossings(old);
    let b = crate::loop_self_crossings(new);
    if !b.unreadable.is_empty() {
        return false;
    }
    if b.crossings.iter().any(|y| {
        !a.crossings.iter().any(|x| {
            (x.face, x.loop_id, x.coedge_a, x.coedge_b)
                == (y.face, y.loop_id, y.coedge_a, y.coedge_b)
        })
    }) {
        return false;
    }
    let options = crate::SelfIntersectionOptions::for_solid(old);
    let (Ok(a), Ok(b)) = (
        crate::solid_self_intersections(old, options),
        crate::solid_self_intersections(new, options),
    ) else {
        return false;
    };
    if a.truncated
        || b.truncated
        || b.undecided > a.undecided
        || b.fold_undecided > a.fold_undecided
    {
        return false;
    }
    if b.confirmed.iter().any(|y| {
        !a.confirmed
            .iter()
            .any(|x| (x.face_a, x.face_b) == (y.face_a, y.face_b))
    }) {
        return false;
    }
    if b.folds
        .iter()
        .any(|y| !a.folds.iter().any(|x| x.face == y.face))
    {
        return false;
    }
    true
}

fn face_joints_not_worse(old: &crate::FaceRecord, new: &crate::FaceRecord, bar: f64) -> bool {
    for (a, b) in old.loops.iter().zip(&new.loops) {
        let count = a.coedges.len();
        for i in 0..count {
            let gap = |l: &crate::topology::LoopRecord| -> Option<f64> {
                let here = &l.coedges[i].pcurve;
                let next = &l.coedges[(i + 1) % count].pcurve;
                let end = image(&old.surface, here, here.domain().ok()?[1]).ok()?;
                let start = image(&old.surface, next, next.domain().ok()?[0]).ok()?;
                Some(end.sub(start).length())
            };
            let (Some(before), Some(after)) = (gap(a), gap(b)) else {
                return false;
            };
            if after > before.max(bar) + bar * 1e-4 {
                return false;
            }
        }
    }
    true
}

fn close_trim_loop_joints_impl(
    solid: &mut BrepSolid,
    tolerance: f64,
) -> Result<Vec<ReplacedTrim>, KernelRefusal> {
    // Rounds: rebuilding one trim toward its vertex can OPEN the joint at its
    // other end, where a neighbour that used to meet the old, displaced end
    // now misses the vertex by its own inherited error (the fixture's `P.CO4_S`
    // edge 6, 6.6e-6, after edge 10's trim moved). Each round applies the
    // same rule; a round that rebuilds nothing, or the fourth, ends it.
    let mut replaced = Vec::<ReplacedTrim>::new();
    // Diagnostic A/B hatch, not a policy: `BREP_TRIM_JOINTS=0` leaves every
    // trim as the fragments carried it, so one binary can read a document
    // both ways (the 2026-09-30 record's before/after columns).
    if std::env::var("BREP_TRIM_JOINTS").as_deref() == Ok("0") {
        return Ok(replaced);
    }
    let (initial, exact_curved) = paired::reconstruct(solid, trim_joint_bar(solid, tolerance));
    replaced.extend(initial);
    for _round in 0..4 {
        let previous = solid.clone();
        let mut this_round = close_trim_loop_joints_once(solid, tolerance, &exact_curved)?;
        let bar = trim_joint_bar(solid, tolerance);
        let mut rejected_faces = Vec::new();
        for face_index in this_round.iter().map(|item| item.face) {
            if rejected_faces.contains(&face_index) {
                continue;
            }
            let old = face_at(&previous.shells, face_index);
            let new = face_at(&solid.shells, face_index);
            // A closed curved loop can switch the integrator out of its band
            // lane and expose inherited interior error. Never accept that
            // material change as an endpoint repair (§13 of the diagnosis).
            if !face_joints_not_worse(old, new, bar) {
                rejected_faces.push(face_index);
                continue;
            }
            if !old.surface.is_affine().unwrap_or(false) {
                let area_ok = match (crate::face_area(old), crate::face_area(new)) {
                    (Ok(a), Ok(b)) if a.is_finite() && b.is_finite() => {
                        // Numerical integration allowance, not geometric slack.
                        let allowance = 1e-11 * a.abs().max(b.abs()) + bar * bar;
                        if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
                            eprintln!("joints: face {} curved area {a:.12} -> {b:.12}, allowance {allowance:.3e}", old.id);
                        }
                        (a - b).abs() <= allowance
                    }
                    _ => false,
                };
                if !area_ok {
                    rejected_faces.push(face_index);
                }
            }
        }
        this_round.retain(|item| !rejected_faces.contains(&item.face));
        for face_index in rejected_faces {
            *face_at_mut(&mut solid.shells, face_index) =
                face_at(&previous.shells, face_index).clone();
            if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
                eprintln!(
                    "joints: restored face {face_index}: joint or curved material area changed"
                );
            }
        }
        if this_round.is_empty() {
            break;
        }
        replaced.extend(this_round);
    }
    Ok(replaced)
}

/// Exact isoparametric boundaries retain a represented carrier construction.
/// General inherited curved trims have no such certificate: closing them can
/// expose interior bias after downstream chart normalization, even when the
/// local face-area guard reads the old seam-band route. Leave the whole curved
/// face unchanged until an exact carrier/pcurve construction is available.
fn has_exact_isoparametric_boundaries(face: &crate::FaceRecord) -> bool {
    !face.loops.is_empty()
        && face.loops.iter().flat_map(|l| &l.coedges).all(|c| {
            let Some(first) = c.pcurve.control_points.first() else {
                return false;
            };
            if !(first.w > 0.0) {
                return false;
            }
            let u = first.x / first.w;
            let v = first.y / first.w;
            c.pcurve
                .control_points
                .iter()
                .all(|p| p.w > 0.0 && p.w.is_finite())
                && (c.pcurve.control_points.iter().all(|p| p.x / p.w == u)
                    || c.pcurve.control_points.iter().all(|p| p.y / p.w == v))
        })
}

/// A full represented isocurve bounds every old/new trim image on it.
/// Periodic variable parameters may use another chart; extrapolation along
/// an open direction has no positive-rational hull certificate and declines.
fn isoparametric_image_bound(
    surface: &NurbsSurface,
    old: &NurbsCurve,
    new: &NurbsCurve,
) -> Option<NurbsCurve> {
    let first = old.control_points.first()?;
    let constant_u = old
        .control_points
        .iter()
        .all(|p| p.x / p.w == first.x / first.w)
        && new
            .control_points
            .iter()
            .all(|p| p.x / p.w == first.x / first.w);
    let constant_v = old
        .control_points
        .iter()
        .all(|p| p.y / p.w == first.y / first.w)
        && new
            .control_points
            .iter()
            .all(|p| p.y / p.w == first.y / first.w);
    let closed = surface.closed_directions().ok()?;
    let du = surface.domain_u().ok()?;
    let dv = surface.domain_v().ok()?;
    let in_domain =
        |coordinate: f64, range: [f64; 2]| coordinate >= range[0] && coordinate <= range[1];
    for p in old.control_points.iter().chain(&new.control_points) {
        if !(p.w > 0.0)
            || !p.w.is_finite()
            || (!closed.0 && !in_domain(p.x / p.w, du))
            || (!closed.1 && !in_domain(p.y / p.w, dv))
        {
            return None;
        }
    }
    let lift = |x: f64, range: [f64; 2], periodic: bool| {
        if periodic {
            range[0] + (x - range[0]).rem_euclid(range[1] - range[0])
        } else {
            x
        }
    };
    if constant_u {
        surface
            .iso_curve_u(lift(first.x / first.w, du, closed.0))
            .ok()
    } else if constant_v {
        surface
            .iso_curve_v(lift(first.y / first.w, dv, closed.1))
            .ok()
    } else {
        None
    }
}

fn close_trim_loop_joints_once(
    solid: &mut BrepSolid,
    tolerance: f64,
    exact_curved: &HashSet<usize>,
) -> Result<Vec<ReplacedTrim>, KernelRefusal> {
    let bar = trim_joint_bar(solid, tolerance);
    let debug =
        std::env::var("BREP_DEBUG_BOOL").is_ok() || std::env::var("BREP_DEBUG_JOINTS").is_ok();

    let vertices: HashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    let edges: HashMap<u64, usize> = solid
        .edges
        .iter()
        .enumerate()
        .map(|(index, edge)| (edge.id, index))
        .collect();
    // Every face using each edge: a trim is rebuilt only when its edge lies on
    // BOTH carriers within the bar (scope by construction, not by fixture:
    // a marched section standing off its carriers by the intersection
    // contract, fixture 25's, is out of scope).
    let mut edge_faces: HashMap<u64, Vec<usize>> = HashMap::default();
    {
        let mut face_index = 0usize;
        for shell in &solid.shells {
            for face in &shell.faces {
                for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
                    let entry = edge_faces.entry(coedge.edge_id).or_default();
                    if !entry.contains(&face_index) {
                        entry.push(face_index);
                    }
                }
                face_index += 1;
            }
        }
    }
    let mut out_of_scope = 0usize;

    // 1. Every joint that misses, and the ends at it that miss their vertex.
    let mut candidates: HashMap<(usize, usize, usize), End> = HashMap::default();
    let mut open_joints = 0usize;
    let mut off_carrier_vertices = 0usize;
    let face_count: usize = solid.shells.iter().map(|shell| shell.faces.len()).sum();
    for face_index in 0..face_count {
        let face = face_at(&solid.shells, face_index);
        for (loop_index, loop_record) in face.loops.iter().enumerate() {
            let count = loop_record.coedges.len();
            if count < 2 {
                continue;
            }
            for index in 0..count {
                let here = &loop_record.coedges[index];
                let next = &loop_record.coedges[(index + 1) % count];
                let (Some(&here_edge), Some(&next_edge)) =
                    (edges.get(&here.edge_id), edges.get(&next.edge_id))
                else {
                    continue;
                };
                let (here_edge, next_edge) = (&solid.edges[here_edge], &solid.edges[next_edge]);
                if here_edge.degenerate || next_edge.degenerate {
                    continue;
                }
                let vertex_id = if here.forward {
                    here_edge.end_vertex_id
                } else {
                    here_edge.start_vertex_id
                };
                let next_vertex_id = if next.forward {
                    next_edge.start_vertex_id
                } else {
                    next_edge.end_vertex_id
                };
                if vertex_id != next_vertex_id {
                    // Not a joint this pass understands (an open loop); leave it.
                    continue;
                }
                let Some(&vertex) = vertices.get(&vertex_id) else {
                    continue;
                };
                let [_, h1] = here.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
                let [n0, _] = next.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
                let end = image(&face.surface, &here.pcurve, h1)?;
                let start = image(&face.surface, &next.pcurve, n0)?;
                let gap = end.sub(start).length();
                if !(gap > bar) {
                    continue;
                }
                open_joints += 1;
                // The vertex must lie on THIS face's carrier: a trim end that
                // misses a vertex which is itself off the carrier is the
                // vertex's error (`vertices.rs` settles those first), and
                // rebuilding the trim toward it adopts the wrong corner —
                // BadBoolean +1.9e-4 against its oracle when this pass ran
                // without the rule (record 2026-09-30).
                let vertex_off_carrier = crate::project_point_to_surface(&face.surface, vertex)
                    .or_refuse(KernelStage::Sew, "project_point_to_surface")?
                    .distance;
                if vertex_off_carrier > bar * 2.0 {
                    off_carrier_vertices += 1;
                    continue;
                }
                let here_miss = end.sub(vertex).length();
                let next_miss = start.sub(vertex).length();
                let mut push = |coedge: usize, miss: f64| {
                    let entry = candidates
                        .entry((face_index, loop_index, coedge))
                        .or_insert(End {
                            face: face_index,
                            loop_index,
                            coedge,
                            miss,
                        });
                    entry.miss = entry.miss.max(miss);
                };
                let mut any = false;
                if here_miss > bar {
                    push(index, here_miss);
                    any = true;
                }
                if next_miss > bar {
                    push((index + 1) % count, next_miss);
                    any = true;
                }
                if !any {
                    // Both ends are within the bar of the vertex yet miss each
                    // other by more than it: rebuild the farther one.
                    if here_miss >= next_miss {
                        push(index, here_miss);
                    } else {
                        push((index + 1) % count, next_miss);
                    }
                }
            }
        }
    }
    if candidates.is_empty() {
        return Ok(Vec::new());
    }

    // 2. Rebuild each candidate trim once, from its edge, and keep it only
    //    inside the bounds.
    let mut replaced = Vec::<ReplacedTrim>::new();
    let mut declined = Vec::<String>::new();
    let mut accepted = Vec::<(End, NurbsCurve)>::new();
    let mut ends: Vec<End> = candidates.into_values().collect();
    ends.sort_by_key(|end| (end.face, end.loop_index, end.coedge));
    for end in ends {
        let (surface, old, forward, edge_id) = {
            let face = face_at(&solid.shells, end.face);
            let coedge = &face.loops[end.loop_index].coedges[end.coedge];
            (
                face.surface.clone(),
                coedge.pcurve.clone(),
                coedge.forward,
                coedge.edge_id,
            )
        };
        let Some(&edge) = edges.get(&edge_id) else {
            continue;
        };
        let edge = &solid.edges[edge];
        let (Some(&start_vertex), Some(&end_vertex)) = (
            vertices.get(&edge.start_vertex_id),
            vertices.get(&edge.end_vertex_id),
        ) else {
            continue;
        };
        // Scope: the edge lies on every carrier it bounds, within the bar.
        let mut edge_off = 0.0f64;
        for &other in edge_faces
            .get(&edge_id)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
        {
            let other_surface = &face_at(&solid.shells, other).surface;
            for t in super::edges::carrier_stations(&edge.curve, edge.t0, edge.t1) {
                let point = edge
                    .curve
                    .evaluate(t)
                    .or_refuse(KernelStage::Sew, "evaluate")?;
                let projection = crate::project_point_to_surface(other_surface, point)
                    .or_refuse(KernelStage::Sew, "project_point_to_surface")?;
                edge_off = edge_off.max(projection.distance);
            }
        }
        if edge_off > bar {
            out_of_scope += 1;
            declined.push(format!("edge {edge_id} on face {}: out of scope, the edge stands {edge_off:.3e} off a carrier it bounds", end.face));
            continue;
        }
        let (traversal_start, traversal_end) = if forward {
            (start_vertex, end_vertex)
        } else {
            (end_vertex, start_vertex)
        };
        let [o0, o1] = old.domain().or_refuse(KernelStage::Sew, "domain")?;
        let old_start_miss = image(&surface, &old, o0)?.sub(traversal_start).length();
        let old_end_miss = image(&surface, &old, o1)?.sub(traversal_end).length();
        let affine = surface
            .is_affine()
            .or_refuse(KernelStage::Sew, "is_affine")?;
        if affine
            && edge_faces.get(&edge_id).into_iter().flatten().any(|other| {
                let face = face_at(&solid.shells, *other);
                !face.surface.is_affine().unwrap_or(false)
                    && !has_exact_isoparametric_boundaries(face)
                    && !exact_curved.contains(other)
            })
        {
            // Closing only the affine side can cause downstream healing to
            // adopt its image on an uncertified curved partner. The partner's
            // inherited interior must be corrected first, as a coupled pair.
            declined.push(format!(
                "edge {edge_id} on face {}: curved partner lacks an exact boundary construction",
                end.face
            ));
            continue;
        }
        if !affine
            && !has_exact_isoparametric_boundaries(face_at(&solid.shells, end.face))
            && !exact_curved.contains(&end.face)
        {
            declined.push(format!("edge {edge_id} on face {}: inherited curved boundary lacks an exact isoparametric construction", end.face));
            continue;
        }
        // The candidate. On an affine carrier the exact map of the edge; on
        // a curved one the EXACT trim with only its missing end(s) re-cut to
        // the vertex's image, by restriction or extension. No interpolant can replace its
        // interior: record §13 demonstrates systematic sphere-area bias.
        let mut candidate: Option<(NurbsCurve, &'static str, f64, usize)> = None;
        if affine {
            match fit_pcurve_on_surface_range(&surface, &edge.curve, edge.t0, edge.t1, forward, bar)
            {
                Ok(fit) if fit.report.on_bar() => {
                    candidate = Some((
                        fit.curve,
                        "exact map",
                        fit.report.residual,
                        fit.report.samples,
                    ))
                }
                Ok(fit) => declined.push(format!(
                    "edge {edge_id} on face {}: exact map off its bar (residual {:.3e})",
                    end.face, fit.report.residual
                )),
                Err(message) => declined.push(format!(
                    "edge {edge_id} on face {}: exact map failed: {message}",
                    end.face
                )),
            }
        } else {
            let start_target = (old_start_miss > bar)
                .then(|| {
                    uv_of(
                        &surface,
                        traversal_start,
                        old.evaluate(o0).or_refuse(KernelStage::Sew, "evaluate")?,
                    )
                })
                .transpose()?;
            let end_target = (old_end_miss > bar)
                .then(|| {
                    uv_of(
                        &surface,
                        traversal_end,
                        old.evaluate(o1).or_refuse(KernelStage::Sew, "evaluate")?,
                    )
                })
                .transpose()?;
            // Fail-soft: a re-cut that cannot even be evaluated is declined
            // and named, never a refusal of the boolean.
            let recut: Result<Option<NurbsCurve>, KernelRefusal> = (|| {
                let Some(recut) = recut_pcurve(&old, start_target, end_target)? else {
                    return Ok(None);
                };
                let [c0, c1] = recut.domain().or_refuse(KernelStage::Sew, "domain")?;
                let start_miss = image(&surface, &recut, c0)?.sub(traversal_start).length();
                let end_miss = image(&surface, &recut, c1)?.sub(traversal_end).length();
                Ok((start_miss <= 2.0 * bar && end_miss <= 2.0 * bar).then_some(recut))
            })();
            match recut {
                Ok(Some(recut)) => candidate = Some((recut, "end re-cut", 0.0, 0)),
                Ok(None) => {}
                Err(refusal) => declined.push(format!(
                    "edge {edge_id} on face {}: end re-cut could not be evaluated ({}; old trim degree {}, {} controls, {} knots, domain [{o0:.6}, {o1:.6}], targets {:?}/{:?})",
                    end.face, refusal.message, old.degree, old.control_points.len(), old.knots.len(),
                    start_target.map(|t| (t.x, t.y)), end_target.map(|t| (t.x, t.y))
                )),
            }
        }
        let Some((mut rebuilt, how, residual, samples)) = candidate else {
            continue;
        };
        align_period_branch(&surface, &mut rebuilt, &old)?;
        let [r0, r1] = rebuilt.domain().or_refuse(KernelStage::Sew, "domain")?;
        // Each end lands within the bar of its vertex (the edge is on both
        // carriers within the bar, so nothing wider is owed to the carrier)
        // and never farther than the end it replaces.
        let reach = 2.0 * bar;
        let new_start_miss = image(&surface, &rebuilt, r0)?.sub(traversal_start).length();
        let new_end_miss = image(&surface, &rebuilt, r1)?.sub(traversal_end).length();
        if new_start_miss > old_start_miss + bar
            || new_end_miss > old_end_miss + bar
            || new_start_miss > reach
            || new_end_miss > reach
        {
            declined.push(format!(
                "edge {edge_id} on face {}: {how} ends miss their vertices by {:.3e}/{:.3e} (old trim {:.3e}/{:.3e}, reach {:.3e})",
                end.face, new_start_miss, new_end_miss, old_start_miss, old_end_miss, reach
            ));
            continue;
        }
        // Along its length the rebuilt image moves no farther from the old
        // image than the joint miss it closes (plus the bar): the trim is
        // corrected at its end, not redrawn. Geometric — the distance from
        // each new sample to the old image, sampled densely.
        let mut displacement = 0.0f64;
        let mut carrier_residual = 0.0f64;
        for t in super::edges::carrier_stations(&rebuilt, r0, r1) {
            let new_point = image(&surface, &rebuilt, t)?;
            displacement = displacement.max(image_distance(&surface, &old, new_point)?);
            for &other in edge_faces
                .get(&edge_id)
                .map(|v| v.as_slice())
                .unwrap_or(&[])
            {
                let projection = crate::project_point_to_surface(
                    &face_at(&solid.shells, other).surface,
                    new_point,
                )
                .or_refuse(KernelStage::Sew, "project_point_to_surface")?;
                carrier_residual = carrier_residual.max(projection.distance);
            }
        }
        if carrier_residual > bar {
            declined.push(format!("edge {edge_id}: candidate trim image leaves a partner carrier by {carrier_residual:.3e}"));
            continue;
        }
        let allowed = end.miss + bar;
        if displacement > allowed {
            declined.push(format!(
                "edge {edge_id} on face {}: {how} moves {:.3e} from the old trim, over the {:.3e} joint miss it closes",
                end.face, displacement, end.miss
            ));
            continue;
        }
        if affine {
            let (Some(previous_image), Some(candidate_image)) = (
                super::edges::affine_image(&surface, &old)?,
                super::edges::affine_image(&surface, &rebuilt)?,
            ) else {
                continue;
            };
            let mut previous_edge = edge.clone();
            previous_edge.curve = previous_image;
            [previous_edge.t0, previous_edge.t1] = previous_edge
                .curve
                .domain()
                .or_refuse(KernelStage::Sew, "domain")?;
            let faces: Vec<_> = solid.shells.iter().flat_map(|s| &s.faces).collect();
            if !super::clearance::motion_clear(
                &previous_edge,
                &candidate_image,
                &faces,
                &solid.edges,
            ) {
                declined.push(format!("edge {edge_id} on face {}: old-to-new planar trim motion could not exclude a nonincident carrier",end.face));
                continue;
            }
        }
        if !affine {
            let Some(bound) = isoparametric_image_bound(&surface, &old, &rebuilt) else {
                continue;
            };
            let [a, b] = bound.domain().or_refuse(KernelStage::Sew, "domain")?;
            let mut enclosing_edge = edge.clone();
            enclosing_edge.curve = bound.clone();
            enclosing_edge.t0 = a;
            enclosing_edge.t1 = b;
            let all_faces: Vec<_> = solid.shells.iter().flat_map(|s| &s.faces).collect();
            if !super::clearance::motion_clear(&enclosing_edge, &bound, &all_faces, &solid.edges) {
                declined.push(format!("edge {edge_id}: complete isoparametric trim hull cannot exclude a nonincident face"));
                continue;
            }
        }
        if debug {
            eprintln!(
                "joints: candidate trim of edge {edge_id} on face {} by {how} (miss {:.3e} -> {:.3e}/{:.3e}, residual {residual:.3e}, moved {displacement:.3e}, {samples} samples)",
                end.face, end.miss, new_start_miss, new_end_miss
            );
        }
        accepted.push((end, rebuilt));
    }

    // 3. Apply only where no joint can get worse: a rebuilt trim's ends
    //    land on the vertices, so every joint it touches must have its other
    //    side either already on the vertex or rebuilt too. Iterate to a fixed
    //    point, dropping candidates whose neighbour cannot follow.
    let mut keep: Vec<bool> = vec![true; accepted.len()];
    let key_of = |end: &End| (end.face, end.loop_index, end.coedge);
    loop {
        let mut changed = false;
        for index in 0..accepted.len() {
            if !keep[index] {
                continue;
            }
            let end = &accepted[index].0;
            let face = face_at(&solid.shells, end.face);
            let loop_record = &face.loops[end.loop_index];
            let count = loop_record.coedges.len();
            let prev_index = (end.coedge + count - 1) % count;
            let next_index = (end.coedge + 1) % count;
            let is_accepted = |coedge: usize| {
                accepted.iter().enumerate().any(|(j, (other, _))| {
                    keep[j] && key_of(other) == (end.face, end.loop_index, coedge)
                })
            };
            // The neighbour's end at each shared vertex, from the CURRENT trims.
            let mut neighbour_ok = true;
            for (neighbour, at_start) in [(prev_index, true), (next_index, false)] {
                if neighbour == end.coedge || is_accepted(neighbour) {
                    continue;
                }
                let coedge = &loop_record.coedges[neighbour];
                let Some(&edge_index) = edges.get(&coedge.edge_id) else {
                    continue;
                };
                let edge = &solid.edges[edge_index];
                let [n0, n1] = coedge
                    .pcurve
                    .domain()
                    .or_refuse(KernelStage::Sew, "domain")?;
                // At this trim's START the neighbour is the previous coedge, whose END meets it.
                let (parameter, vertex_id) = if at_start {
                    (
                        n1,
                        if coedge.forward {
                            edge.end_vertex_id
                        } else {
                            edge.start_vertex_id
                        },
                    )
                } else {
                    (
                        n0,
                        if coedge.forward {
                            edge.start_vertex_id
                        } else {
                            edge.end_vertex_id
                        },
                    )
                };
                let Some(&vertex) = vertices.get(&vertex_id) else {
                    continue;
                };
                let miss = image(&face.surface, &coedge.pcurve, parameter)?
                    .sub(vertex)
                    .length();
                if miss > bar {
                    neighbour_ok = false;
                }
            }
            if !neighbour_ok {
                keep[index] = false;
                changed = true;
                let (end, _) = &accepted[index];
                declined.push(format!("trim {} of loop {} on face {}: held back, its neighbour cannot follow to the vertex", end.coedge, end.loop_index, end.face));
            }
        }
        if !changed {
            break;
        }
    }
    for (index, (end, rebuilt)) in accepted.into_iter().enumerate() {
        if !keep[index] {
            continue;
        }
        let face = face_at_mut(&mut solid.shells, end.face);
        let previous = std::mem::replace(
            &mut face.loops[end.loop_index].coedges[end.coedge].pcurve,
            rebuilt,
        );
        replaced.push(ReplacedTrim {
            face: end.face,
            loop_index: end.loop_index,
            coedge: end.coedge,
            previous,
            previous_edge: None,
        });
    }
    if debug {
        eprintln!(
            "joints: {open_joints} joints over the {bar:.1e} bar, {off_carrier_vertices} at a vertex off this carrier (left), {out_of_scope} out of scope; {} trims rebuilt, {} declined",
            replaced.len(),
            declined.len()
        );
        for line in &declined {
            eprintln!("joints:   declined {line}");
        }
    }
    Ok(replaced)
}


