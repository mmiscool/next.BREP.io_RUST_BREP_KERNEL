use super::*;
use crate::Vec4;

const HARD_SPANS: usize = 65_536;

/// Failure to certify an edge's requested chord tolerance.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeChordError {
    pub edge_id: u64,
    pub requested: f64,
    pub reason: EdgeChordFailure,
}

#[derive(Clone, Debug, PartialEq)]
pub enum EdgeChordFailure {
    InvalidTolerance,
    UnsupportedCurve,
    EndpointMismatch,
    DegenerateEdge,
    SafetyCap,
    ParameterResolution,
    Evaluation(String),
}

impl std::fmt::Display for EdgeChordError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "edge chord tolerance: edge {} requested {:e}: {:?}", self.edge_id, self.requested, self.reason)
    }
}
impl std::error::Error for EdgeChordError {}

/// A checked tessellation either meets its edge chord budget or names the
/// failing edge. Errors from face triangulation retain their original detail.
#[derive(Clone, Debug, PartialEq)]
pub enum TessellationError {
    EdgeChord(EdgeChordError),
    Geometry(String),
}
impl From<String> for TessellationError {
    fn from(value: String) -> Self { Self::Geometry(value) }
}
impl From<EdgeChordError> for TessellationError {
    fn from(value: EdgeChordError) -> Self { Self::EdgeChord(value) }
}
impl std::fmt::Display for TessellationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self { Self::EdgeChord(e) => e.fmt(f), Self::Geometry(e) => f.write_str(e) }
    }
}
impl std::error::Error for TessellationError {}

pub(super) fn enabled() -> bool {
    crate::switch!("BREP_TESS_CHORD_BUDGET", true)
}

fn certificate_spans(curve: &crate::NurbsCurve) -> Option<Vec<Vec<Vec4>>> {
    let p = curve.degree;
    let points = &curve.control_points;
    let knots = &curve.knots;
    if points.len() < p + 1 || knots.len() != points.len() + p + 1 { return None; }
    let (first, last) = (*knots.first()?, *knots.last()?);
    // Match the evaluator's first/last non-empty span on over-clamped
    // vectors. Extra end knots have corresponding inactive control points;
    // removing those pairs changes no basis function on the curve's domain.
    let head = knots.iter().take_while(|&&k| k == first).count().saturating_sub(p + 1);
    let tail = knots.iter().rev().take_while(|&&k| k == last).count().saturating_sub(p + 1);
    if head + tail > points.len() - p - 1 { return None; }
    crate::curve::bezier_decompose_raw(&knots[head..knots.len() - tail],
        &points[head..points.len() - tail], p,
        Vec4 { x: 0., y: 0., z: 0., w: 0. },
        |a, b, alpha| a.scale(alpha).add(b.scale(1. - alpha)))
        .or_else(|| separated_spans(&knots[head..knots.len() - tail],
            &points[head..points.len() - tail], p))
}

// A full-multiplicity interior knot separates independent clamped pieces.
// Preserve both endpoint poles: the bound must include both one-sided limits,
// rather than averaging away a discontinuity that the requested chord cannot meet.
#[cold]
#[inline(never)]
fn separated_spans(knots: &[f64], points: &[Vec4], p: usize) -> Option<Vec<Vec<Vec4>>> {
    if p == 0 || points.len() < p + 1 || knots.len() != points.len() + p + 1
        || knots.iter().any(|k| !k.is_finite()) || knots.windows(2).any(|k| k[1] < k[0]) {
        return None;
    }
    let decompose = |knots: &[f64], points: &[Vec4]| crate::curve::bezier_decompose_raw(
        knots, points, p, Vec4 { x: 0., y: 0., z: 0., w: 0. },
        |a, b, alpha| a.scale(alpha).add(b.scale(1. - alpha)));
    let mut spans = Vec::new();
    let (mut knot_begin, mut point_begin) = (0, 0);
    let mut i = p + 1;
    while i < knots.len() - p - 1 {
        let mut end = i + 1;
        while end < knots.len() && knots[end] == knots[i] { end += 1; }
        let multiplicity = end - i;
        if multiplicity > p + 1 { return None; }
        if multiplicity == p + 1 {
            let point_end = point_begin + end - knot_begin - p - 1;
            spans.extend(decompose(&knots[knot_begin..end], points.get(point_begin..point_end)?)?);
            knot_begin = i;
            point_begin = point_end;
        }
        i = end;
    }
    if knot_begin == 0 { return None; }
    spans.extend(decompose(&knots[knot_begin..], points.get(point_begin..)?)?);
    Some(spans)
}

fn control_bound<const N: usize>(points: &[Vec4], lo: f64, hi: f64, a: Vec3, b: Vec3, tolerance: f64) -> f64 {
    let zero = Vec4 { x: 0., y: 0., z: 0., w: 0. };
    let mut work = [zero; N];
    let mut net = [zero; N];
    let count = points.len();
    work[..count].copy_from_slice(points);
    net[0] = work[0];
    for n in (1..count).rev() {
        for i in 0..n { work[i] = work[i].scale(1. - hi).add(work[i + 1].scale(hi)); }
        net[count - n] = work[0];
    }
    if lo > 0. {
        let t = lo / hi;
        work[..count].copy_from_slice(&net[..count]);
        for n in (1..count).rev() {
            for i in 0..n { work[i] = work[i].scale(1. - t).add(work[i + 1].scale(t)); }
            net[n - 1] = work[n - 1];
        }
    }
    tight_bound::<N>(&net[..count], a, b, tolerance, 3)
}

fn tight_bound<const N: usize>(points: &[Vec4], a: Vec3, b: Vec3, tolerance: f64, depth: usize) -> f64 {
    let distance = |p: Vec4| deflection_to_chord(Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w), a, b);
    let mut bound = 0.0f64;
    for &p in points {
        let d = distance(p);
        if !d.is_finite() { return f64::INFINITY; }
        bound = bound.max(d);
    }
    if bound <= tolerance || depth == 0 { return bound; }
    // Tighten the certificate against the SAME chord before adding mesh
    // vertices. A quadratic arc's tangent pole can be twice as far from the
    // chord as the arc; refining the mesh for that loose hull wastes samples.
    let zero = Vec4 { x: 0., y: 0., z: 0., w: 0. };
    let mut work = [zero; N];
    let mut left = [zero; N];
    let mut right = [zero; N];
    let count = points.len();
    work[..count].copy_from_slice(points);
    left[0] = work[0];
    right[count - 1] = work[count - 1];
    for n in (1..count).rev() {
        for i in 0..n { work[i] = work[i].scale(0.5).add(work[i + 1].scale(0.5)); }
        left[count - n] = work[0];
        right[n - 1] = work[n - 1];
    }
    // This midpoint is on the curve, so a measured miss already proves that
    // the chord must split; further certificate work cannot rescue it.
    if distance(work[0]) > tolerance { return bound; }
    tight_bound::<N>(&left[..count], a, b, tolerance, depth - 1)
        .max(tight_bound::<N>(&right[..count], a, b, tolerance, depth - 1))
}

/// Certify the full rational curve, rather than a finite set of sag probes.
/// Positive homogeneous weights place each Bezier arc inside its control hull;
/// distance to a segment is convex, so the largest control-point distance bounds
/// the entire arc. Subdivision tightens that bound without omitting knot spans.
/// Endpoints are the shared topology vertices, including their displacement from
/// the curve. Unsupported representations fail closed.
pub fn sample_edge_polyline_checked(
    edge: &EdgeRecord,
    start: Vec3,
    end: Vec3,
    chord_tolerance: f64,
) -> Result<Vec<(f64, Vec3)>, EdgeChordError> {
    refine(edge, start, end, chord_tolerance, &[0.0, 1.0], HARD_SPANS)
}

// A positive-weight linear curve lies on its two Cartesian poles' segment.
// Pairing that segment with the topology chord bounds every point by the
// larger endpoint displacement, without allocating a Bezier decomposition.
// Other representations and additional chart stations use the general path.
fn full_line_certified(edge: &EdgeRecord, start: Vec3, end: Vec3, tolerance: f64, seeds: &[f64]) -> bool {
    let curve = &edge.curve;
    if curve.degree != 1 || curve.control_points.len() != 2 || curve.knots.len() != 4
        || !curve.is_validated() || seeds != [0., 1.] { return false; }
    let k = &curve.knots;
    if k.iter().any(|v| !v.is_finite()) || k[0] != k[1] || k[2] != k[3]
        || k[2] - k[0] <= crate::tolerance::FP_FLOOR
        || !(edge.t1 - edge.t0).is_finite() { return false; }
    let forward = edge.t0 == k[0] && edge.t1 == k[2];
    if !forward && !(edge.t0 == k[2] && edge.t1 == k[0]) { return false; }
    if curve.control_points.iter().any(|p| p.w <= crate::tolerance::FP_FLOOR
        || ![p.x, p.y, p.z, p.w].iter().all(|v| v.is_finite()))
        || ![start.x, start.y, start.z, end.x, end.y, end.z].iter().all(|v| v.is_finite()) {
        return false;
    }
    let poles = &curve.control_points;
    let a = poles[usize::from(!forward)].point().unwrap();
    let b = poles[usize::from(forward)].point().unwrap();
    let scale = [a.x.abs(), a.y.abs(), a.z.abs(), b.x.abs(), b.y.abs(), b.z.abs(),
        start.x.abs(), start.y.abs(), start.z.abs(), end.x.abs(), end.y.abs(), end.z.abs()]
        .into_iter().fold(0.0f64, f64::max);
    let roundoff = 128. * f64::EPSILON * scale;
    if !roundoff.is_finite() || tolerance <= roundoff || tolerance < 16. * f64::MIN_POSITIVE.sqrt() {
        return false;
    }
    let budget = tolerance - roundoff;
    a.sub(start).length() <= budget && b.sub(end).length() <= budget
}

pub(super) fn refine(
    edge: &EdgeRecord,
    start: Vec3,
    end: Vec3,
    tolerance: f64,
    seeds: &[f64],
    cap: usize,
) -> Result<Vec<(f64, Vec3)>, EdgeChordError> {
    let fail = |reason| EdgeChordError { edge_id: edge.id, requested: tolerance, reason };
    if !tolerance.is_finite() || tolerance <= 0.0 {
        return Err(fail(EdgeChordFailure::InvalidTolerance));
    }
    if seeds.len().saturating_sub(1) > cap {
        return Err(fail(EdgeChordFailure::SafetyCap));
    }
    if edge.curve.degree > 64 || edge.curve.control_points.len() > HARD_SPANS {
        return Err(fail(EdgeChordFailure::SafetyCap));
    }
    if full_line_certified(edge, start, end, tolerance, seeds) {
        return Ok(vec![(0., start), (1., end)]);
    }
    let spans = certificate_spans(&edge.curve)
        .ok_or_else(|| fail(EdgeChordFailure::UnsupportedCurve))?;
    let knots: Vec<_> = edge.curve.knots.windows(2).filter(|p| p[1] > p[0])
        .map(|p| (p[0], p[1])).collect();
    if spans.len() != knots.len() || spans.is_empty()
        || !edge.t0.is_finite() || !edge.t1.is_finite()
        || !(edge.t1 - edge.t0).is_finite()
        || spans.iter().flatten().any(|p| p.w <= 0.0 || ![p.x, p.y, p.z, p.w].iter().all(|v| v.is_finite()))
        || ![start.x, start.y, start.z, end.x, end.y, end.z].iter().all(|v| v.is_finite())
    {
        return Err(fail(EdgeChordFailure::UnsupportedCurve));
    }
    let parameter = |f: f64| {
        if f == 0. { edge.t0 } else if f == 1. { edge.t1 }
        else { edge.t0 + f * (edge.t1 - edge.t0) }
    };
    let evaluate = |f: f64| edge.curve.evaluate(parameter(f))
        .map_err(|e| fail(EdgeChordFailure::Evaluation(e)));
    let coordinate_scale = spans.iter().flatten()
        .flat_map(|p| [p.x.abs() / p.w, p.y.abs() / p.w, p.z.abs() / p.w])
        .chain([start.x.abs(), start.y.abs(), start.z.abs(), end.x.abs(), end.y.abs(), end.z.abs()])
        .fold(0.0f64, f64::max);
    // Reserve arithmetic slack for homogeneous subdivision and projection.
    // A request below representable coordinate accuracy must fail explicitly,
    // rather than treating rounded zero distances as a tolerance certificate.
    let roundoff = 64. * f64::EPSILON * (edge.curve.degree + 1) as f64 * coordinate_scale;
    if !roundoff.is_finite() || tolerance <= roundoff || tolerance < 16. * f64::MIN_POSITIVE.sqrt() {
        return Err(fail(EdgeChordFailure::ParameterResolution));
    }
    let budget = tolerance - roundoff;
    // A validated clamped curve interpolates its active end poles, which
    // decomposition already supplied (including over-clamped ends). Reuse
    // those values for endpoint checks without running the basis engine.
    // Keep first-use validation and the evaluator's tiny-span refusal path.
    let endpoint = |t: f64| {
        let last = knots.len() - 1;
        let pole = if !edge.curve.is_validated() { None }
            else if t == knots[0].0 && knots[0].1 - knots[0].0 > crate::tolerance::FP_FLOOR {
                spans[0].first()
            } else if t == knots[last].1 && knots[last].1 - knots[last].0 > crate::tolerance::FP_FLOOR {
                spans.last().unwrap().last()
            } else { None };
        pole.map_or_else(|| edge.curve.evaluate(t), |p| p.point())
            .map_err(|e| fail(EdgeChordFailure::Evaluation(e)))
    };
    if endpoint(edge.t0)?.sub(start).length() > tolerance || endpoint(edge.t1)?.sub(end).length() > tolerance {
        return Err(fail(EdgeChordFailure::EndpointMismatch));
    }
    let mut output = vec![(0.0, start)];
    let mut pending = Vec::new();
    let mut b = end;
    for pair in seeds.windows(2).rev() {
        let a = if pair[0] == 0.0 { start } else { evaluate(pair[0])? };
        pending.push((pair[0], pair[1], a, b));
        b = a;
    }
    while let Some((f0, f1, a, b)) = pending.pop() {
        // Match the curve evaluator's clamped domain, including trims a few
        // ulps beyond an end knot after topology operations.
        let domain = (knots[0].0, knots.last().unwrap().1);
        let t0 = parameter(f0).clamp(domain.0, domain.1);
        let t1 = parameter(f1).clamp(domain.0, domain.1);
        let mut bound = 0.0f64;
        let first = knots.partition_point(|(_, hi)| *hi < t0.min(t1));
        for ((lo, hi), points) in knots[first..].iter().zip(&spans[first..]) {
            if *lo > t0.max(t1) { break; }
            let from = t0.min(t1).max(*lo);
            let to = t0.max(t1).min(*hi);
            if to < from { continue; }
            let u = ((from - lo) / (hi - lo)).clamp(0.0, 1.0);
            let v = ((to - lo) / (hi - lo)).clamp(0.0, 1.0);
            let distance = if points.len() <= 4 {
                control_bound::<4>(points, u, v, a, b, budget)
            } else {
                control_bound::<65>(points, u, v, a, b, budget)
            };
            if !distance.is_finite() { return Err(fail(EdgeChordFailure::UnsupportedCurve)); }
            bound = bound.max(distance);
        }
        if bound <= budget {
            output.push((f1, b));
            continue;
        }
        if edge.degenerate { return Err(fail(EdgeChordFailure::DegenerateEdge)); }
        if output.len() + pending.len() >= cap {
            return Err(fail(EdgeChordFailure::SafetyCap));
        }
        let middle = f0 + (f1 - f0) * 0.5;
        if middle <= f0 || middle >= f1 || parameter(middle) == t0
            || parameter(middle) == t1 {
            return Err(fail(EdgeChordFailure::ParameterResolution));
        }
        let m = evaluate(middle)?;
        pending.push((middle, f1, m, b));
        pending.push((f0, middle, a, m));
    }
    Ok(output)
}

pub(super) fn refine_shared(edge: &EdgeRecord, start: Vec3, end: Vec3, tolerance: f64, seeds: &[f64])
    -> Result<Vec<(f64, Vec3)>, EdgeChordError>
{
    refine(edge, start, end, tolerance, seeds, HARD_SPANS)
}
