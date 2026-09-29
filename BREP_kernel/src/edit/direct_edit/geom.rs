use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

pub(super) use crate::offset_retrim::{plane_of_surface, Plane, PARALLEL_EPS};

#[derive(Clone, Copy)]
pub(super) struct Line {
    pub(super) point: Vec3,
    pub(super) dir: Vec3,
}

pub(super) fn max_topology_id(solid: &BrepSolid) -> u64 {
    let mut maximum = solid.id;
    for vertex in &solid.vertices {
        maximum = maximum.max(vertex.id);
    }
    for edge in &solid.edges {
        maximum = maximum.max(edge.id);
    }
    for shell in &solid.shells {
        maximum = maximum.max(shell.id);
        for face in &shell.faces {
            maximum = maximum.max(face.id);
            for loop_record in &face.loops {
                maximum = maximum.max(loop_record.id);
                for coedge in &loop_record.coedges {
                    maximum = maximum.max(coedge.id);
                }
            }
        }
    }
    maximum
}

/// Apply a face's rebuilt intrinsic boundaries, then validate topology and
/// reject a volume-sign reversal when both volume calculations succeed.
pub(super) fn finish_intrinsic_face_offset(
    solid: &BrepSolid,
    face_location: (usize, usize),
    offset: NurbsSurface,
    rebuilt_edges: &HashMap<u64, (NurbsCurve, f64, f64)>,
    rebuilt_vertices: &HashMap<u64, Vec3>,
    operation: &str,
) -> Result<BrepSolid, KernelRefusal> {
    let (shell_index, face_index) = face_location;
    let mut result = solid.clone();
    result.shells[shell_index].faces[face_index].surface = offset;
    for edge in &mut result.edges {
        if let Some((curve, t0, t1)) = rebuilt_edges.get(&edge.id) {
            edge.curve = curve.clone();
            edge.t0 = *t0;
            edge.t1 = *t1;
        }
    }
    for vertex in &mut result.vertices {
        if let Some(point) = rebuilt_vertices.get(&vertex.id) {
            vertex.point = *point;
        }
    }
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validation",
            format!("{operation}: pushed solid failed validation: {issues:?}"),
        ));
    }
    if let (Ok(before), Ok(after)) = (solid_signed_volume(solid), solid_signed_volume(&result)) {
        if before * after <= 0.0 {
            return Err(KernelRefusal::input(
                KernelStage::Validate,
                INVERTED,
                format!("{operation}: the push inverts the solid — refusing"),
            ));
        }
    }
    Ok(result)
}

pub(super) fn find_face(solid: &BrepSolid, face_id: u64) -> Option<(usize, usize)> {
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            if face.id == face_id {
                return Some((shell_index, face_index));
            }
        }
    }
    None
}

/// The one face (other than `exclude`) that uses `edge_id`. Errs unless the
/// edge is shared by exactly one other face (manifold interior edge).
pub(super) fn other_face_of_edge(solid: &BrepSolid, edge_id: u64, exclude: u64) -> Result<u64, KernelRefusal> {
    let mut found: Option<u64> = None;
    for shell in &solid.shells {
        for face in &shell.faces {
            if face.id == exclude {
                continue;
            }
            let uses = face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .any(|coedge| coedge.edge_id == edge_id);
            if uses {
                if found.is_some_and(|other| other != face.id) {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Collect,
                        "non_manifold_edge",
                        format!("delete_face_and_heal: edge {edge_id} is shared by more than two faces"),
                    ));
                }
                found = Some(face.id);
            }
        }
    }
    found.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Collect,
            "open_edge",
            format!("delete_face_and_heal: boundary edge {edge_id} has no opposite face"),
        )
    })
}

pub(super) fn edge_point(solid: &BrepSolid, vertex_id: u64) -> Result<Vec3, KernelRefusal> {
    solid
        .vertices
        .iter()
        .find(|vertex| vertex.id == vertex_id)
        .map(|vertex| vertex.point)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "vertex",
                format!("delete_face_and_heal: missing vertex {vertex_id}"),
            )
        })
}

pub(super) fn intersect_planes(a: &Plane, b: &Plane) -> Option<Line> {
    let dir = a.normal.cross(b.normal);
    if dir.length() <= PARALLEL_EPS {
        return None;
    }
    let ca = a.normal.dot(a.origin);
    let cb = b.normal.dot(b.origin);
    let naa = a.normal.dot(a.normal);
    let nab = a.normal.dot(b.normal);
    let nbb = b.normal.dot(b.normal);
    let determinant = naa * nbb - nab * nab;
    if determinant.abs() <= PARALLEL_EPS {
        return None;
    }
    let alpha = (ca * nbb - cb * nab) / determinant;
    let beta = (cb * naa - ca * nab) / determinant;
    let point = a.normal.scale(alpha).add(b.normal.scale(beta));
    let dir = dir.normalized().ok()?;
    Some(Line { point, dir })
}

pub(super) fn intersect_line_plane(line: &Line, plane: &Plane) -> Option<Vec3> {
    let denominator = line.dir.dot(plane.normal);
    if denominator.abs() <= PARALLEL_EPS {
        return None;
    }
    let t = (plane.normal.dot(plane.origin) - plane.normal.dot(line.point)) / denominator;
    Some(line.point.add(line.dir.scale(t)))
}

type EndpointTarget = Option<(u64, Vec3)>;

/// Collect surviving edges affected by a vertex collapse before mutating them.
pub(super) fn pending_edge_relocations(
    solid: &BrepSolid,
    excluded: &HashSet<u64>,
    collapse: &HashMap<u64, u64>,
    vertex_points: &HashMap<u64, Vec3>,
) -> Vec<(usize, EndpointTarget, EndpointTarget)> {
    let mut pending = Vec::new();
    for (index, edge) in solid.edges.iter().enumerate() {
        if excluded.contains(&edge.id) {
            continue;
        }
        let start = collapse
            .get(&edge.start_vertex_id)
            .map(|vertex| (*vertex, vertex_points[vertex]));
        let end = collapse
            .get(&edge.end_vertex_id)
            .map(|vertex| (*vertex, vertex_points[vertex]));
        if start.is_some() || end.is_some() {
            pending.push((index, start, end));
        }
    }
    pending
}

/// The point of an edge's represented range nearest `point`.
pub(super) fn nearest_on_edge(edge: &EdgeRecord, point: Vec3) -> Result<(f64, Vec3), KernelRefusal> {
    let projection = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Classify, "project")?;
    let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
    let t = projection.u.clamp(low, high);
    Ok((t, edge.curve.evaluate(t).or_refuse(KernelStage::Classify, "evaluate")?))
}

/// The shortest world length of a knot span along the four boundary curves of
/// `surface`, spans of zero length (a pole) left out.
fn shortest_boundary_span(surface: &NurbsSurface) -> Result<Option<f64>, KernelRefusal> {
    const STEPS: usize = 8;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Classify, "domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Classify, "domain")?;
    let breaks = |knots: &[f64], low: f64, high: f64| -> Vec<f64> {
        let mut values: Vec<f64> = knots.iter().copied().filter(|knot| *knot >= low && *knot <= high).collect();
        values.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * (high - low));
        values
    };
    let mut lengths: Vec<f64> = Vec::new();
    let mut span = |at: &dyn Fn(f64) -> Result<Vec3, String>, low: f64, high: f64| -> Result<(), KernelRefusal> {
        let mut length = 0.0f64;
        let mut previous = at(low).or_refuse(KernelStage::Classify, "evaluate")?;
        for step in 1..=STEPS {
            let point = at(low + (high - low) * step as f64 / STEPS as f64)
                .or_refuse(KernelStage::Classify, "evaluate")?;
            length += point.sub(previous).length();
            previous = point;
        }
        lengths.push(length);
        Ok(())
    };
    for pair in breaks(&surface.knots_v, v0, v1).windows(2) {
        span(&|v| surface.evaluate(u0, v), pair[0], pair[1])?;
        span(&|v| surface.evaluate(u1, v), pair[0], pair[1])?;
    }
    for pair in breaks(&surface.knots_u, u0, u1).windows(2) {
        span(&|u| surface.evaluate(u, v0), pair[0], pair[1])?;
        span(&|u| surface.evaluate(u, v1), pair[0], pair[1])?;
    }
    let longest = lengths.iter().copied().fold(0.0f64, f64::max);
    Ok(lengths.into_iter().filter(|length| *length > 1e-9 * longest).reduce(f64::min))
}

/// The shortest world length of a knot span of `edge`'s own curve among the spans
/// its range touches. Each span is measured WHOLE, so a range that stops just
/// past a knot does not read as a tiny span.
fn shortest_edge_span(edge: &EdgeRecord) -> Result<Option<f64>, KernelRefusal> {
    const STEPS: usize = 8;
    let [d0, d1] = edge.curve.domain().or_refuse(KernelStage::Classify, "domain")?;
    let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
    let mut knots: Vec<f64> = edge.curve.knots.iter().copied().filter(|knot| *knot >= d0 && *knot <= d1).collect();
    knots.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * (d1 - d0));
    let mut lengths: Vec<f64> = Vec::new();
    for pair in knots.windows(2) {
        if pair[1] <= low || pair[0] >= high {
            continue;
        }
        let mut length = 0.0f64;
        let mut previous = edge.curve.evaluate(pair[0]).or_refuse(KernelStage::Classify, "evaluate")?;
        for step in 1..=STEPS {
            let point = edge
                .curve
                .evaluate(pair[0] + (pair[1] - pair[0]) * step as f64 / STEPS as f64)
                .or_refuse(KernelStage::Classify, "evaluate")?;
            length += point.sub(previous).length();
            previous = point;
        }
        lengths.push(length);
    }
    let longest = lengths.iter().copied().fold(0.0f64, f64::max);
    Ok(lengths.into_iter().filter(|length| *length > 1e-9 * longest).reduce(f64::min))
}

/// How many intervals to read `edge` in: against `patch` when the face is read as
/// a fitted patch, against an analytic carrier otherwise. The incidence gate and
/// the corner lane's `coverage` check both sample here.
///
/// What can lie between two samples is a feature, and a feature has a size.
/// There are two sources of one. The EDGE can turn off its carrier and back: a
/// control point moves a degree-p curve over p + 1 of its own knot spans. A
/// PATCH has an edge of its own that can turn across the moved edge and back,
/// set by its boundary curves' knot spans. So the moved edge is read at four
/// samples to the shorter of its own shortest knot span and, on a patch, the
/// patch's shortest boundary span, in world length. Every stretch at least a
/// quarter span long then holds a sample, and one control point's excursion, p + 1
/// spans wide at its base, holds several. A stretch narrower than a quarter span
/// can still lie between two, which needs a curve to cross back inside one span.
///
/// Measured, both ways. A cap patch whose boundary notched 0.1 deep across 0.3
/// of a widened 5.9 edge lay between two of nine samples, and the heal shipped a
/// body 9.6e-2 mm³ short; the patch's spans are 0.05. A spline seam bowed 6.6e-3
/// off a cylinder inside spans of 0.2 lay between two of nine samples against the
/// infinite cylinder.
///
/// Nine is what a line or an exact arc keeps. A line is one knot span, and so is
/// every quarter turn or less of an exact arc, so up to a quarter turn reads eight
/// intervals. Neither can turn off its carrier between samples, because it has no
/// control point to turn with. Against an analytic carrier the past-the-patch arm
/// is identically zero (an infinite surface has no edge), so only the edge's spans
/// count there.
///
/// When four to a span would take more than `CAP` samples, the gate refuses by
/// name rather than read the edge more coarsely than it or its patch can turn.
pub(super) fn sample_intervals(
    patch: Option<&NurbsSurface>,
    edge: &EdgeRecord,
    face_id: u64,
    op: &str,
) -> Result<usize, KernelRefusal> {
    const FLOOR: usize = 8;
    const PER_SPAN: f64 = 4.0;
    const CAP: usize = 4096;
    let edge_span = shortest_edge_span(edge)?;
    let boundary_span = match patch {
        Some(surface) => shortest_boundary_span(surface)?,
        None => None,
    };
    let (span, owner) = match (edge_span, boundary_span) {
        (Some(own), Some(boundary)) if boundary < own => (boundary, "its face's patch has a boundary knot span"),
        (Some(own), _) => (own, "it has a knot span"),
        (None, Some(boundary)) => (boundary, "its face's patch has a boundary knot span"),
        (None, None) => return Ok(FLOOR),
    };
    let mut length = 0.0f64;
    let mut previous = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Classify, "evaluate")?;
    for step in 1..=64 {
        let point = edge
            .curve
            .evaluate(edge.t0 + (edge.t1 - edge.t0) * step as f64 / 64.0)
            .or_refuse(KernelStage::Classify, "evaluate")?;
        length += point.sub(previous).length();
        previous = point;
    }
    let intervals = ((PER_SPAN * length / span).ceil() as usize).max(FLOOR);
    if intervals > CAP {
        return Err(KernelRefusal::non_convergence(
            KernelStage::Classify,
            "sample_cap",
            format!(
                "{op}: edge {} on face {face_id} is {length:.3e} long and {owner} {span:.3e} long, so \
                 reading it at four samples to a span needs {intervals}, over the {CAP} the \
                 incidence gate reads — refusing rather than reading the edge more coarsely than \
                 it or its face can turn",
                edge.id
            ),
        ));
    }
    Ok(intervals)
}

/// Every edge a heal MOVED must lie on every face that uses it no worse than
/// it did before.
///
/// Relocation — widening a side edge along its own curve, rebuilding a straight
/// one between its resolved ends, re-deriving it from its flanking carriers — is
/// told where the edge's ends go, never what the edge has to lie on. When a
/// blend strip's end lands exactly on a vertex of a face the heal does not read
/// as that strip's neighbour, the strip's corner vertex is also that face's
/// vertex, the collapse drags it to the recovered corner, and the face's own
/// edge is rebuilt through a point off the face. Nothing downstream measured
/// that: `validate()` compares pcurves with a loose bar, and on a 20 mm cube
/// with a 0.5 mm chamfer rounded at r = 0.5 the single-strip heal returned a
/// validating body with an edge 0.5 off its face and 98 mm³ missing.
///
/// Each moved edge in `after` is sampled (`sample_intervals`: nine points, more
/// on a fitted patch), and each sample is paired with the nearest point of the
/// SAME edge in `before` (the solid's edges before relocation) over its old
/// range. Where the heal widened the edge past
/// that range, the nearest old point is the old end, so the new stretch is held
/// to the reading at the end it grew from. Both points are read against the
/// carrier of every face using the edge — the infinite surface of an analytic
/// carrier, the patch of a fitted one — as a SIGNED distance and, on a patch,
/// the distance past the patch's edge. Both readings take the healed face's
/// carrier: a lane that grows one before this runs keeps it over its old extent
/// (`extend_natural` verbatim, a ruled extension on the same infinite surface),
/// so what the pair differs by is what the heal did to the edge.
///
/// A heal may bring an edge nearer its face, never take it farther. The signed
/// reading after must lie within `bar` of the span from the face to where the
/// edge lay, `[min(0, before), max(0, before)]`, and a sample may lie past a
/// patch's edge by at most `bar` more than its pair did. An edge that lay
/// exactly on the face therefore leaves it by at most `bar`. Two weaker gates
/// were measured and set aside:
///
/// * Comparing the edge's worst sample with its old worst, `bar + 2·prior`, the
///   first form of this gate. An edge `prior` off could be carried another
///   `prior` unseen. The allowance was set for five perturbed open-heal rows
///   whose side edge read 5.29e-3 off its nudged wall and 5.444e-3 after, and
///   that was never a drift: the edge widens along its own curve, the nine
///   samples fell on different points of the two ranges, and paired point by
///   point it moves 2.3e-10.
/// * Comparing distances UNSIGNED. An edge 5e-3 outside a face and dragged 1e-2
///   through it reads 5e-3 on both sides; the chamfered cube whose face carrier
///   sits 5e-3 inside its edges healed that way to a body 1.999 mm³ short.
///
/// A face the caller RE-TRIMS on a plane after this runs (`planes`) is read as
/// that infinite plane, whatever its surface is. Every lane reads a planar
/// neighbour through `plane_of_surface` — a coplanar control net — and rebuilds
/// its carrier over the new boundary, so the patch it has now is not the one it
/// will be trimmed on. Reading it as a patch refused a correct heal: a cube's top
/// face stored as a degree-(2, 2) flat patch that stopped at its rim (the shape
/// of a STEP plane exported as a B-spline, which `analytic()` does not read as a
/// plane) had its widened edges 0.5 past the patch, and with that refusal
/// removed the heal was the cube.
///
/// Returns the worst distance seen after the heal.
pub(super) fn moved_edges_lie_on_their_faces(
    before: &[EdgeRecord],
    after: &BrepSolid,
    moved: &HashSet<u64>,
    planes: &HashMap<u64, Plane>,
    bar: f64,
    op: &str,
) -> Result<f64, KernelRefusal> {
    // How far past a patch's edge a reading lies: the part of the true distance
    // the signed one does not carry. Zero on an analytic carrier.
    let past_edge = |signed: f64, distance: f64| (distance * distance - signed * signed).max(0.0).sqrt();
    let mut worst = 0.0f64;
    for face in after.shells.iter().flat_map(|shell| &shell.faces) {
        let mut carrier: Option<TripleCarrier> = None;
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            if !moved.contains(&coedge.edge_id) {
                continue;
            }
            let Some(edge) = after.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
                continue;
            };
            if carrier.is_none() {
                carrier = Some(match planes.get(&face.id) {
                    Some(plane) => TripleCarrier::Plane {
                        origin: plane.origin,
                        normal: plane.normal,
                    },
                    None => TripleCarrier::of(&face.surface)?,
                });
            }
            let carrier = carrier.as_ref().unwrap();
            let old = before.iter().find(|old| old.id == edge.id);
            let patch = match carrier {
                TripleCarrier::Patch(surface) => Some(*surface),
                _ => None,
            };
            let intervals = sample_intervals(patch, edge, face.id, op)?;
            // One sample: how far it lies outside what the rule allows, across the
            // face or past the patch's edge, its reading after and its pair's
            // before; and its true distance after.
            let read = |point: Vec3,
                        now_foot: &mut Option<(f64, f64)>,
                        was_foot: &mut Option<(f64, f64)>|
             -> Result<((f64, bool, f64, f64), f64), KernelRefusal> {
                let (now, now_distance) = carrier.signed_and_true_distance(point, now_foot)?;
                let (was, was_distance) = match old {
                    Some(old) => carrier.signed_and_true_distance(nearest_on_edge(old, point)?.1, was_foot)?,
                    None => (0.0, 0.0),
                };
                let across = (now - was.max(0.0)).max(was.min(0.0) - now);
                let (now_past, was_past) = (past_edge(now, now_distance), past_edge(was, was_distance));
                Ok((
                    if now_past - was_past > across {
                        (now_past - was_past, true, now_past, was_past)
                    } else {
                        (across, false, now, was)
                    },
                    now_distance,
                ))
            };
            // Consecutive samples seed each other's projections on a patch, which is
            // what makes reading it densely affordable. A seeded foot can settle on a
            // local minimum, so a sample that would refuse is read again from scratch.
            let mut drift = (0.0f64, false, 0.0f64, 0.0f64);
            let (mut now_foot, mut was_foot) = (None, None);
            for sample in 0..=intervals {
                let t = edge.t0 + (edge.t1 - edge.t0) * sample as f64 / intervals as f64;
                let point = edge.curve.evaluate(t).or_refuse(KernelStage::Validate, "evaluate")?;
                let seeded = now_foot.is_some() || was_foot.is_some();
                let (mut reading, mut distance) = read(point, &mut now_foot, &mut was_foot)?;
                if seeded && reading.0 > bar {
                    (now_foot, was_foot) = (None, None);
                    (reading, distance) = read(point, &mut now_foot, &mut was_foot)?;
                }
                worst = worst.max(distance);
                if reading.0 > drift.0 {
                    drift = reading;
                }
            }
            let (by, past, now, was) = drift;
            if by > bar {
                let reading = if past {
                    format!("a point of it lies {now:.3e} past the patch's edge where the edge lay {was:.3e} past it")
                } else {
                    format!("a point of it lies {now:+.3e} from the face where the edge lay {was:+.3e} from it")
                };
                return Err(KernelRefusal::unsupported(
                    KernelStage::Validate,
                    "moved_edge_off_face",
                    format!(
                        "{op}: moved edge {} leaves face {} by {by:.3e} ({reading} before the heal) — a \
                         blend's end runs into a vertex of a face the heal does not read as its \
                         neighbour; refusing rather than re-trimming the face around an edge off it",
                        edge.id, face.id
                    ),
                ));
            }
        }
    }
    Ok(worst)
}
