//! Concave rounds whose pad runs off a planar support's outline into air
//! (`BREP_FILLET_OUTLINE_EXTEND`, default on; `=0` restores the ordinary lanes,
//! `check` builds both and reports).
//!
//! A slab with rounded corners standing on a box (a cabin roof over its walls)
//! whose box corners come within the round of the slab's corners: the round of
//! the box's top edges reaches the slab's underside past its outline near each
//! corner, though the underside is wide enough everywhere else. The stripe
//! network refuses that pad (`blend/restrict.rs`, a concave stripe crossing
//! several boundary edges of its mate), and the cutter composition behind it
//! builds each edge's fill full length: fins standing past the slab's corner
//! round, their tops thin faces in the underside plane, and the box corners
//! ending in flat caps instead of the rolled corner. `validate()` passes it.
//!
//! The rolling ball's answer is the round of the same edges on the slab
//! extended past its outline, cut back to the outline's walls: the pad and the
//! corner sectors end on those walls carried down over them. So the support is
//! extended by a thin plate outside its outline, the selection is rounded on
//! that body (where nothing overruns, so the ordinary lanes build it), and the
//! band outside the outline is cut away again. Taken only where the outline's
//! walls are the outline's own prism (planes containing the support's normal,
//! rounds about an axis along it) and the body has no material in that band,
//! so the cut removes the plate and the overrun and nothing else.
use super::*;
use super::analyze::{analyze_edge, edge_and_faces, face_plane_normal, EdgePath};
use super::edges::resolve_edge_by_point;

/// `BREP_FILLET_OUTLINE_EXTEND` (default on; `=0` off, `=check` builds both
/// and reports, returning the ordinary answer).
pub(super) fn mode() -> crate::env_read::Mode {
    crate::mode_switch!("BREP_FILLET_OUTLINE_EXTEND", true)
}

thread_local! {
    // Set while the extended body is rounded, so that round does not ask again.
    static ACTIVE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Stations along each edge's contact rail, and around each end vertex.
const RAIL_STATIONS: usize = 64;
const END_STATIONS: usize = 48;
/// Samples per outline edge for the in-plane polygon.
const OUTLINE_SAMPLES: usize = 128;

/// The selection rounded on its support extended past the outline and cut
/// back, or `None` where it is not that shape (see the module). A failed
/// stage declines rather than refuses: the ordinary lanes answer then.
pub(super) fn build(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
) -> Option<BrepSolid> {
    STAGE.with(|stage| stage.set("selection"));
    let built = candidate(solid, edge_points, edge_names, radius, name);
    if built.is_none() && std::env::var_os("BREP_DEBUG_NETWORK").is_some() {
        eprintln!("fillet.outline_extend declined at {}", STAGE.with(|stage| stage.get()));
    }
    built
}

thread_local! {
    // The stage `candidate` reached, for the debug line of a decline.
    static STAGE: std::cell::Cell<&'static str> = const { std::cell::Cell::new("") };
}

fn at(stage: &'static str) {
    STAGE.with(|cell| cell.set(stage));
}

fn candidate(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
) -> Option<BrepSolid> {
    if ACTIVE.with(|active| active.get()) || solid.kind != crate::BodyKind::Solid || !(radius > 0.0) || !radius.is_finite() {
        return None;
    }
    let tolerances = crate::KernelTolerances::for_solid(solid, 1e-7);
    let mut overrun_face: Option<u64> = None;
    let mut deepest = 0f64;
    for point in edge_points {
        at("resolve");
        let edge_id = resolve_edge_by_point(solid, *point).ok()?;
        at("analyze_edge");
        let cross = analyze_edge(solid, edge_id, radius).ok()?;
        if cross.convex {
            return None;
        }
        let EdgePath::Straight { direction, length } = &cross.path else {
            return None;
        };
        let (direction, length) = (*direction, *length);
        let phi = cross.into_first.dot(cross.into_second).clamp(-1.0, 1.0).acos();
        if !(phi > 1e-9) || phi >= std::f64::consts::PI - 1e-9 {
            return None;
        }
        let reach = radius / (phi * 0.5).tan();
        if !reach.is_finite() {
            return None;
        }
        deepest = deepest.max(reach);
        at("edge shape");
        let (_, faces) = edge_and_faces(solid, edge_id).ok()?;
        if faces.len() != 2 {
            return None;
        }
        for (face, into) in [(faces[0], cross.into_first), (faces[1], cross.into_second)] {
            // A support whose outer loop holds the edge is not a surround
            // the pad can run off; only a face round the edge's loop is.
            let Some(outline) = Outline::of(solid, face, cross.start, &tolerances).filter(|outline| !outline.edges.contains(&edge_id)) else {
                continue;
            };
            let end = cross.start.add(direction.scale(length));
            let side = outline.normal.cross(into);
            let rail = (0..=RAIL_STATIONS)
                .map(|k| cross.start.add(direction.scale(length * k as f64 / RAIL_STATIONS as f64)).add(into.scale(reach)));
            // The zone the closure at an end vertex may touch: the ball's
            // reach plus its radius round that vertex. A bound read from the
            // square and tilted roofs (on the tilted one a zone of the reach
            // alone missed the overrun), not proved; a zone that reaches too
            // far costs only the extended build, which equals the ordinary
            // one where nothing overruns.
            let zone = reach + radius;
            let around = [cross.start, end].into_iter().flat_map(|vertex| {
                (0..END_STATIONS).map(move |k| {
                    let angle = std::f64::consts::TAU * k as f64 / END_STATIONS as f64;
                    vertex.add(into.scale(zone * angle.cos())).add(side.scale(zone * angle.sin()))
                })
            });
            if rail.chain(around).any(|point| outline.leaves(point)) {
                at("two overrun supports");
                match overrun_face {
                    Some(other) if other != face.id => return None,
                    _ => overrun_face = Some(face.id),
                }
            }
        }
    }
    at("no overrun");
    let face_id = overrun_face?;
    at("outline");
    let face = solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == face_id)?;
    let first = face.loops.first()?.coedges.first()?.edge_id;
    let anchor = solid.edges.iter().find(|edge| edge.id == first)?.curve.evaluate(0.0).ok()?;
    let outline = Outline::of(solid, face, anchor, &tolerances)?;
    built(solid, &outline, edge_points, edge_names, radius, name, deepest)
}

fn built(
    solid: &BrepSolid,
    outline: &Outline,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
    reach: f64,
) -> Option<BrepSolid> {
    // Signed depth s along the material side (−normal); the support is s = 0.
    // The plate is s ∈ [0, t]; the band cut away is s ∈ [−depth, t + margin]
    // outside the outline; the outline's prism reaches past both.
    let inward = outline.normal.scale(-1.0);
    let t = radius;
    let depth = 2.0 * reach + radius;
    let margin = 0.5 * radius;
    let span = outline.extent + 2.0 * (depth + reach) + radius;
    let square = |s0: f64, s1: f64| -> Option<BrepSolid> {
        let (u, w) = (outline.u, outline.w);
        let base = outline.origin.add(inward.scale(s0));
        let corner = |a: f64, b: f64| base.add(u.scale(outline.center.0 + a * span)).add(w.scale(outline.center.1 + b * span));
        let corners = [corner(-1.0, -1.0), corner(1.0, -1.0), corner(1.0, 1.0), corner(-1.0, 1.0)];
        let lines: Option<Vec<NurbsCurve>> = (0..4).map(|k| make_line(corners[k], corners[(k + 1) % 4]).ok()).collect();
        extrude_profile_brep(&lines?, inward, s1 - s0).ok()
    };
    let prism = |s0: f64, s1: f64| -> Option<BrepSolid> {
        let shift = inward.scale(s0);
        let curves: Vec<NurbsCurve> = outline
            .curves
            .iter()
            .map(|curve| {
                let mut moved = curve.clone();
                for cp in moved.control_points.iter_mut() {
                    cp.x += shift.x * cp.w;
                    cp.y += shift.y * cp.w;
                    cp.z += shift.z * cp.w;
                }
                moved
            })
            .collect();
        extrude_profile_brep(&curves, inward, s1 - s0).ok()
    };
    let options = BooleanOptions::default();
    at("walls prism");
    let walls = prism(-depth - margin, t + 2.0 * margin)?;
    at("plate and band");
    let plate = boolean_operation(&square(0.0, t)?, &walls, BooleanOperation::Subtract, &options).ok()?;
    let band = boolean_operation(&square(-depth, t + margin)?, &walls, BooleanOperation::Subtract, &options).ok()?;
    // The band must hold none of the body: then the plate meets it only on the
    // outline's walls, and cutting the band removes the plate and the overrun.
    let scale = crate::recentre::solid_bounds(solid).maximum.sub(crate::recentre::solid_bounds(solid).minimum).length();
    at("band guard");
    let shared = boolean_operation(solid, &band, BooleanOperation::Intersect, &options).ok()?;
    let shared_volume = if shared.shells.iter().all(|shell| shell.faces.is_empty()) {
        0.0
    } else {
        crate::solid_mass_properties(&shared).ok()?.volume.abs()
    };
    if !(shared_volume <= 1e-9 * scale.powi(3)) {
        return None;
    }
    at("extend");
    let extended = boolean_operation(solid, &plate, BooleanOperation::Union, &options).ok()?;
    ACTIVE.with(|active| active.set(true));
    let rounded = fillet_edges(&extended, edge_points, edge_names, radius, false, name);
    ACTIVE.with(|active| active.set(false));
    at("round on the extended body");
    let rounded = rounded.ok()?;
    at("cut back");
    let mut result = boolean_operation(&rounded, &band, BooleanOperation::Subtract, &options).ok()?;
    fresh_repeated_ids(&mut result);
    restore_edge_names(solid, &mut result, tolerances_of(solid));
    at("validate");
    if !result.validate().is_empty() {
        return None;
    }
    Some(result)
}

/// A planar face's outer loop, admitted as the prism of its walls.
struct Outline {
    /// Outward normal of the face.
    normal: Vec3,
    origin: Vec3,
    u: Vec3,
    w: Vec3,
    /// The outer loop as a polygon in (u, w) about `origin`, its centroid, and
    /// the largest distance of a vertex from that centroid.
    polygon: Vec<(f64, f64)>,
    center: (f64, f64),
    extent: f64,
    /// What a station must clear the polygon by to read as off the face: the
    /// polygon's chord error and the model tolerance.
    margin: f64,
    /// The outer loop's curves, in loop order and direction, and their edges.
    curves: Vec<NurbsCurve>,
    edges: Vec<u64>,
}

impl Outline {
    /// `face`'s outer loop when `face` is a plane and every wall across that
    /// loop is the loop's own prism along the normal: a plane containing the
    /// normal, or a constant-radius revolution about an axis along it. Such a
    /// wall cannot be tangent to the face, so the edge between them is sharp.
    fn of(solid: &BrepSolid, face: &FaceRecord, near: Vec3, tolerances: &crate::KernelTolerances) -> Option<Self> {
        if !matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. })) {
            return None;
        }
        let (origin, normal) = face_plane_normal(face, near).ok()?;
        let angular = tolerances.angular;
        let first = face.loops.first()?.coedges.first()?.edge_id;
        let seed = solid.edges.iter().find(|edge| edge.id == first)?;
        let along = seed.curve.evaluate(seed.t1).ok()?.sub(seed.curve.evaluate(seed.t0).ok()?);
        let u = along.sub(normal.scale(along.dot(normal))).normalized().ok()?;
        let w = normal.cross(u);
        let flat = |p: Vec3| (p.sub(origin).dot(u), p.sub(origin).dot(w));
        let mut best: Option<(f64, usize, Vec<(f64, f64)>, Vec<NurbsCurve>, f64)> = None;
        for (index, loop_record) in face.loops.iter().enumerate() {
            let mut polygon = Vec::new();
            let mut curves = Vec::new();
            let mut sag = 0f64;
            for coedge in &loop_record.coedges {
                let border = solid.edges.iter().find(|edge| edge.id == coedge.edge_id)?;
                let curve = crate::edge_subcurve::edge_subcurve(border).ok()?;
                let curve = if coedge.forward { curve } else { curve.reversed().ok()? };
                let [t0, t1] = curve.domain().ok()?;
                let point = |k: f64| curve.evaluate(t0 + (t1 - t0) * k / OUTLINE_SAMPLES as f64).ok();
                for step in 0..OUTLINE_SAMPLES {
                    let (a, b, middle) = (point(step as f64)?, point(step as f64 + 1.0)?, point(step as f64 + 0.5)?);
                    sag = sag.max(middle.sub(a.add(b).scale(0.5)).length());
                    polygon.push(flat(a));
                }
                curves.push(curve);
            }
            let area = 0.5 * (0..polygon.len()).map(|k| {
                let (a, b) = (polygon[k], polygon[(k + 1) % polygon.len()]);
                a.0 * b.1 - b.0 * a.1
            }).sum::<f64>().abs();
            if best.as_ref().is_none_or(|(best_area, ..)| area > *best_area) {
                best = Some((area, index, polygon, curves, sag));
            }
        }
        let (_, outer, polygon, curves, sag) = best?;
        // Only the outer loop's walls are the prism: the inner loops hold the
        // selected edges, whose other supports are the body's own walls.
        for coedge in &face.loops[outer].coedges {
            let (border, mates) = edge_and_faces(solid, coedge.edge_id).ok()?;
            let wall = mates.iter().find(|mate| mate.id != face.id)?;
            let at = border.curve.evaluate(0.5 * (border.t0 + border.t1)).ok()?;
            if !prism_wall(wall, normal, at, angular) {
                return None;
            }
        }
        if polygon.len() < 3 {
            return None;
        }
        let count = polygon.len() as f64;
        let center = (polygon.iter().map(|p| p.0).sum::<f64>() / count, polygon.iter().map(|p| p.1).sum::<f64>() / count);
        let extent = polygon.iter().map(|p| (p.0 - center.0).hypot(p.1 - center.1)).fold(0f64, f64::max);
        let edges = face.loops[outer].coedges.iter().map(|coedge| coedge.edge_id).collect();
        Some(Outline { normal, origin, u, w, polygon, center, extent, margin: 2.0 * sag + 4.0 * tolerances.model, curves, edges })
    }

    /// Whether `point` (in the face's plane) lies outside the outer loop by
    /// more than the polygon's error.
    fn leaves(&self, point: Vec3) -> bool {
        let rel = point.sub(self.origin);
        let (x, y) = (rel.dot(self.u), rel.dot(self.w));
        let polygon = &self.polygon;
        let mut inside = false;
        let mut nearest = f64::INFINITY;
        for k in 0..polygon.len() {
            let (a, b) = (polygon[k], polygon[(k + 1) % polygon.len()]);
            if (a.1 > y) != (b.1 > y) && x < a.0 + (y - a.1) * (b.0 - a.0) / (b.1 - a.1) {
                inside = !inside;
            }
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let along = (((x - a.0) * dx + (y - a.1) * dy) / (dx * dx + dy * dy).max(f64::MIN_POSITIVE)).clamp(0.0, 1.0);
            nearest = nearest.min((x - a.0 - along * dx).hypot(y - a.1 - along * dy));
        }
        !inside && nearest > self.margin
    }
}

/// A wall the outline's prism follows: a plane containing `normal`, or a
/// constant-radius revolution (a cylinder, or a round's wall) about an axis
/// along it.
fn prism_wall(wall: &FaceRecord, normal: Vec3, at: Vec3, angular: f64) -> bool {
    let radial = |frame: &crate::RevolutionFrame, p: Vec3| {
        let d = p.sub(frame.origin);
        d.sub(frame.axis.scale(d.dot(frame.axis))).length()
    };
    match wall.surface.analytic() {
        Some(crate::AnalyticSurface::Plane { .. }) => {
            face_plane_normal(wall, at).is_ok_and(|(_, wall_normal)| wall_normal.dot(normal).abs() <= angular)
        }
        Some(crate::AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. }) => {
            let scale = rho0.abs().max(rho1.abs()).max(1.0);
            (rho0 - rho1).abs() <= 1e-9 * scale && frame.axis.cross(normal).length() <= angular
        }
        Some(crate::AnalyticSurface::Revolution { frame, generatrix, .. }) => {
            let radii: Option<Vec<f64>> = generatrix.control_points.iter().map(|cp| cp.point().ok().map(|p| radial(&frame, p))).collect();
            radii.is_some_and(|radii| {
                let scale = radii.iter().fold(1f64, |m, r| m.max(r.abs()));
                !radii.is_empty() && radii.iter().all(|r| (r - radii[0]).abs() <= 1e-9 * scale)
            }) && frame.axis.cross(normal).length() <= angular
        }
        _ => false,
    }
}

/// [`reversed_in`] of `result` less those `solid` already had (a shelf's top
/// level with a selected underside is no fin).
pub(super) fn new_reversed_support_faces(solid: &BrepSolid, result: &BrepSolid, edge_points: &[Vec3], radius: f64) -> usize {
    reversed_in(solid, result, edge_points, radius).saturating_sub(reversed_in(solid, solid, edge_points, radius))
}

/// What the check mode reports: `None` when the candidate and the ordinary
/// answer agree (volume to 1e-9 and area to 1e-8 relative, the same face and
/// edge names), or
/// when there is no candidate.
pub(super) fn disagreement(candidate: Option<&BrepSolid>, ordinary: Result<&BrepSolid, &KernelRefusal>) -> Option<String> {
    let candidate = candidate?;
    let ordinary = match ordinary {
        Ok(body) => body,
        Err(refusal) => return Some(format!("the candidate builds, the ordinary lanes refuse: {}", refusal.message)),
    };
    let measure = |body: &BrepSolid| crate::solid_mass_properties(body).ok().map(|m| (m.volume, m.surface_area));
    let names = |body: &BrepSolid| {
        let mut faces: Vec<Option<String>> = body.shells.iter().flat_map(|s| &s.faces).map(|f| f.name.clone()).collect();
        let mut edges: Vec<Option<String>> = body.edges.iter().map(|e| e.name.clone()).collect();
        faces.sort();
        edges.sort();
        (faces, edges)
    };
    // The area quadrature reads the same body to about 1e-9 relative.
    let close = |a: f64, b: f64, bar: f64| (a - b).abs() <= bar * a.abs().max(b.abs()).max(1.0);
    let (a, b) = (measure(candidate), measure(ordinary));
    let same_measure = matches!((a, b), (Some((v1, s1)), Some((v2, s2))) if close(v1, v2, 1e-9) && close(s1, s2, 1e-8));
    let (na, nb) = (names(candidate), names(ordinary));
    if same_measure && na == nb {
        return None;
    }
    Some(format!("candidate {a:?} ordinary {b:?}; face names {:?} vs {:?}; edge names {:?} vs {:?}", na.0, nb.0, na.1, nb.1))
}

/// Planar faces of `result` lying in the plane of a planar support of one of
/// the selected concave edges of `solid` and facing the other way: a fill
/// standing past its support's outline has its top there (the fins this path
/// exists for). A material-side probe misses them, since a fin's top has its
/// fill below it; the reversed normal in the support's own plane does not.
fn reversed_in(solid: &BrepSolid, result: &BrepSolid, edge_points: &[Vec3], radius: f64) -> usize {
    let tolerances = crate::KernelTolerances::for_solid(solid, 1e-7);
    let mut planes: Vec<(Vec3, Vec3)> = Vec::new();
    for point in edge_points {
        let Ok(edge_id) = resolve_edge_by_point(solid, *point) else { continue };
        if analyze_edge(solid, edge_id, radius).map_or(true, |cross| cross.convex) {
            continue;
        }
        let Ok((_, faces)) = edge_and_faces(solid, edge_id) else { continue };
        for face in faces {
            if matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. })) {
                if let Ok(plane) = face_plane_normal(face, *point) {
                    planes.push(plane);
                }
            }
        }
    }
    result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. })))
        .filter(|face| {
            let Some(edge) = face.loops.first().and_then(|l| l.coedges.first()).and_then(|c| result.edges.iter().find(|e| e.id == c.edge_id)) else {
                return false;
            };
            let Ok(on) = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)) else { return false };
            let Ok((_, normal)) = face_plane_normal(face, on) else { return false };
            planes.iter().any(|(origin, support)| {
                normal.dot(*support) < -1.0 + tolerances.angular && on.sub(*origin).dot(*support).abs() <= 10.0 * tolerances.model
            })
        })
        .count()
}

/// A boolean can hand back two loops (or coedges) under one id, which
/// `validate()` does not read and the topology arena refuses: each repeat
/// after the first takes a fresh id past every id in use.
fn fresh_repeated_ids(solid: &mut BrepSolid) {
    let faces = || solid.shells.iter().flat_map(|shell| &shell.faces);
    let mut next_loop = faces().flat_map(|face| &face.loops).map(|l| l.id).max().unwrap_or(0) + 1;
    let mut next_coedge = faces().flat_map(|face| &face.loops).flat_map(|l| &l.coedges).map(|c| c.id).max().unwrap_or(0) + 1;
    let mut loops = std::collections::HashSet::new();
    let mut coedges = std::collections::HashSet::new();
    for face in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        for loop_record in &mut face.loops {
            if !loops.insert(loop_record.id) {
                loop_record.id = next_loop;
                loops.insert(next_loop);
                next_loop += 1;
            }
            for coedge in &mut loop_record.coedges {
                if !coedges.insert(coedge.id) {
                    coedge.id = next_coedge;
                    coedges.insert(next_coedge);
                    next_coedge += 1;
                }
            }
        }
    }
}

fn tolerances_of(solid: &BrepSolid) -> f64 {
    crate::KernelTolerances::for_solid(solid, 1e-7).model
}

/// The plate and the band rebuild the outline's edges and the walls' edges
/// between them, and a boolean's new edges carry no name: each unnamed edge
/// of `result` lying on a named edge of `input` (its ends and middle within
/// `tolerance` of that curve) takes that edge's name, as the ordinary lanes
/// leave it.
fn restore_edge_names(input: &BrepSolid, result: &mut BrepSolid, tolerance: f64) {
    let on = |edge: &crate::topology::EdgeRecord, point: Vec3| {
        crate::project_point_to_curve(&edge.curve, point).ok().is_some_and(|projection| {
            let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
            let margin = 1e-9 * (high - low).abs();
            projection.u >= low - margin && projection.u <= high + margin && projection.point.sub(point).length() <= tolerance
        })
    };
    for edge in result.edges.iter_mut().filter(|edge| edge.name.is_none() && !edge.degenerate) {
        let samples: Option<Vec<Vec3>> = [0.0, 0.5, 1.0].iter().map(|k| edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k).ok()).collect();
        let Some(samples) = samples else { continue };
        if let Some(named) = input.edges.iter().filter(|other| other.name.is_some()).find(|other| samples.iter().all(|p| on(other, *p))) {
            edge.name = named.name.clone();
        }
    }
}
