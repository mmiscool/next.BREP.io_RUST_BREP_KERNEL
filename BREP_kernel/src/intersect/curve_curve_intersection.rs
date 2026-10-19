use crate::curve::KNOT_IDENTITY_TOL as KNOT_TOLERANCE;
use crate::{NurbsCurve, Vec3};
use serde::Serialize;

const EPSILON: f64 = 1e-12;

#[derive(Clone, Copy)]
struct Bounds {
    minimum: Vec3,
    maximum: Vec3,
}

impl Bounds {
    fn from_curve(curve: &NurbsCurve) -> Result<Self, String> {
        let points = curve
            .control_points
            .iter()
            .map(|point| point.point())
            .collect::<Result<Vec<_>, _>>()?;
        let mut minimum = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut maximum = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for point in points {
            minimum.x = minimum.x.min(point.x);
            minimum.y = minimum.y.min(point.y);
            minimum.z = minimum.z.min(point.z);
            maximum.x = maximum.x.max(point.x);
            maximum.y = maximum.y.max(point.y);
            maximum.z = maximum.z.max(point.z);
        }
        Ok(Self { minimum, maximum })
    }

    fn intersects(self, other: Self, tolerance: f64) -> bool {
        self.minimum.x <= other.maximum.x + tolerance
            && self.maximum.x + tolerance >= other.minimum.x
            && self.minimum.y <= other.maximum.y + tolerance
            && self.maximum.y + tolerance >= other.minimum.y
            && self.minimum.z <= other.maximum.z + tolerance
            && self.maximum.z + tolerance >= other.minimum.z
    }
}

#[derive(Clone, Copy)]
struct Segment {
    start: f64,
    end: f64,
    bounds: Bounds,
}

fn interior_knots(curve: &NurbsCurve) -> Vec<f64> {
    let start = curve.knots[curve.degree];
    let end = curve.knots[curve.knots.len() - 1 - curve.degree];
    let mut result = Vec::new();
    for &knot in &curve.knots {
        if knot <= start + KNOT_TOLERANCE || knot >= end - KNOT_TOLERANCE {
            continue;
        }
        if result
            .last()
            .is_none_or(|previous: &f64| (*previous - knot).abs() > KNOT_TOLERANCE)
        {
            result.push(knot);
        }
    }
    result
}

fn segment_boxes(curve: &NurbsCurve, count: usize) -> Result<Vec<Segment>, String> {
    let [start, end] = curve.domain()?;
    let mut parameters: Vec<f64> = (0..=count)
        .map(|index| start + (end - start) * index as f64 / count as f64)
        .collect();
    parameters.extend(interior_knots(curve));
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|a, b| (*a - *b).abs() <= KNOT_TOLERANCE);

    let mut segments = Vec::new();
    let mut rest = curve.clone();
    let mut rest_start = start;
    for (index, &parameter) in parameters.iter().enumerate().skip(1) {
        if parameter - rest_start <= KNOT_TOLERANCE {
            continue;
        }
        if index == parameters.len() - 1 {
            segments.push(Segment {
                start: rest_start,
                end,
                bounds: Bounds::from_curve(&rest)?,
            });
        } else {
            let (piece, remainder) = rest.split(parameter)?;
            segments.push(Segment {
                start: rest_start,
                end: parameter,
                bounds: Bounds::from_curve(&piece)?,
            });
            rest = remainder;
            rest_start = parameter;
        }
    }
    Ok(segments)
}

fn newton_closest_pair(
    first: &NurbsCurve,
    second: &NurbsCurve,
    seed_s: f64,
    seed_t: f64,
) -> Result<[f64; 2], String> {
    let [s0, s1] = first.domain()?;
    let [t0, t1] = second.domain()?;
    let mut s = seed_s;
    let mut t = seed_t;
    for _ in 0..40 {
        let first_derivatives = first.derivatives_small(s, 2)?;
        let second_derivatives = second.derivatives_small(t, 2)?;
        let residual = first_derivatives[0].sub(second_derivatives[0]);
        let f = first_derivatives[1].dot(residual);
        let g = -second_derivatives[1].dot(residual);
        let j00 = first_derivatives[2].dot(residual) + first_derivatives[1].length_squared();
        let j01 = -first_derivatives[1].dot(second_derivatives[1]);
        let j10 = -second_derivatives[1].dot(first_derivatives[1]);
        let j11 = -second_derivatives[2].dot(residual) + second_derivatives[1].length_squared();
        let residual_length = residual.length();
        let converged_first =
            f.abs() <= EPSILON + 1e-10 * first_derivatives[1].length() * residual_length.max(1e-30);
        let converged_second = g.abs()
            <= EPSILON + 1e-10 * second_derivatives[1].length() * residual_length.max(1e-30);
        if (converged_first && converged_second) || residual_length <= EPSILON {
            return Ok([s, t]);
        }
        let determinant = j00 * j11 - j01 * j10;
        if determinant.abs() <= EPSILON {
            return Ok([s, t]);
        }
        let mut ds = (-f * j11 + g * j01) / determinant;
        let mut dt = (-g * j00 + f * j10) / determinant;
        ds = ds.clamp(-(s1 - s0) / 4.0, (s1 - s0) / 4.0);
        dt = dt.clamp(-(t1 - t0) / 4.0, (t1 - t0) / 4.0);
        let next_s = (s + ds).clamp(s0, s1);
        let next_t = (t + dt).clamp(t0, t1);
        if (next_s - s).abs() <= 1e-15 * (s1 - s0) && (next_t - t).abs() <= 1e-15 * (t1 - t0) {
            return Ok([next_s, next_t]);
        }
        s = next_s;
        t = next_t;
    }
    Ok([s, t])
}

fn parameter_tolerance(curve: &NurbsCurve, parameter: f64, tolerance: f64) -> Result<f64, String> {
    let speed = curve.deriv1(parameter)?.1.length();
    let [start, end] = curve.domain()?;
    if speed <= EPSILON {
        Ok((end - start) * 1e-3)
    } else {
        Ok(((tolerance / speed) * 10.0).max((end - start) * 1e-9))
    }
}

fn parameter_distance(a: f64, b: f64, start: f64, end: f64, closed: bool) -> f64 {
    let distance = (a - b).abs();
    if closed {
        distance.min(end - start - distance)
    } else {
        distance
    }
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct CurveCurveIntersection {
    pub s: f64,
    pub t: f64,
    pub point: Vec3,
    pub gap: f64,
    pub tangential: bool,
}

/// `BREP_EXACT_LINE_LINE` (lane X row 7): two straight segments that are not
/// near parallel meet where their exact closest pair says
/// ([`exact_line_line`]), instead of where Newton runs from box seeds settle.
/// On by default; `0`/`off` returns to the general search, and `census`
/// answers by the general search and appends one line per call whose answers
/// differ to `BREP_EXACT_LINE_LINE_CENSUS`.
///
/// Promoted 2026-10-16 on top of the canonical fragment order. 239 documents:
/// 6,853 calls change, every one with the same hit count; the exact pair
/// removes residuals up to 2.02e-8, and the per-hit gap goes from a 2.17e-8
/// maximum to 7.9e-9. The exact pair is exact by construction, and the 1,387
/// hits whose evaluated gap reads above the general search's, by at most
/// 1.21e-13 (p50 8.9e-16), are the rounding floor of measuring the gap, not
/// lost accuracy. 49 documents change bytes; every body is geometrically the
/// same, with no name moved.
#[derive(Clone, Copy, PartialEq, Eq)]
enum LineLineMode {
    Off,
    On,
    Census,
}

fn line_line_mode() -> LineLineMode {
    static MODE: std::sync::OnceLock<LineLineMode> = std::sync::OnceLock::new();
    let mode = *MODE.get_or_init(|| match std::env::var("BREP_EXACT_LINE_LINE").as_deref() {
        Ok("0") | Ok("off") => LineLineMode::Off,
        Ok("census") => LineLineMode::Census,
        _ => LineLineMode::On,
    });
    mode
}


/// The segment a degree-1 curve of two equal-weight control points traces
/// linearly over its clamped knots: (start point, end point, domain).
fn straight_segment(curve: &NurbsCurve) -> Option<(Vec3, Vec3, [f64; 2])> {
    if curve.degree != 1 || curve.control_points.len() != 2 || curve.knots.len() != 4 {
        return None;
    }
    let k = &curve.knots;
    if k[0] != k[1] || k[2] != k[3] || !(k[1] < k[2]) {
        return None;
    }
    if curve.control_points[0].w != curve.control_points[1].w {
        return None;
    }
    Some((curve.control_points[0].point().ok()?, curve.control_points[1].point().ok()?, [k[1], k[2]]))
}

/// The exact meeting of two straight segments that are not near parallel:
/// their unique closest pair (each parameter clamped to its segment, the
/// other then re-solved), one transverse hit when its gap is within
/// `tolerance`, none otherwise. `None` (the general search answers) for
/// anything else: a curve that is not a straight segment, a degenerate one,
/// or two directions within the tangential band of `intersect_curves`.
pub(crate) fn exact_line_line(first: &NurbsCurve, second: &NurbsCurve, tolerance: f64) -> Option<Vec<CurveCurveIntersection>> {
    let (p0, p1, [s0, s1]) = straight_segment(first)?;
    let (q0, q1, [t0, t1]) = straight_segment(second)?;
    let d1 = p1.sub(p0);
    let d2 = q1.sub(q0);
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    if a <= EPSILON || e <= EPSILON {
        return None;
    }
    if d1.cross(d2).length() <= 1e-3_f64.sin() * (a * e).sqrt() {
        return None;
    }
    let r = p0.sub(q0);
    let b = d1.dot(d2);
    let c = d1.dot(r);
    let f = d2.dot(r);
    let denominator = a * e - b * b;
    let mut s = ((b * f - c * e) / denominator).clamp(0.0, 1.0);
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }
    let first_point = p0.add(d1.scale(s));
    let second_point = q0.add(d2.scale(t));
    let gap = first_point.sub(second_point).length();
    if gap > tolerance {
        return Some(Vec::new());
    }
    Some(vec![CurveCurveIntersection {
        s: (s0 + (s1 - s0) * s).clamp(s0, s1),
        t: (t0 + (t1 - t0) * t).clamp(t0, t1),
        point: first_point.add(second_point).scale(0.5),
        gap,
        tangential: false,
    }])
}

pub fn intersect_curves(
    first: &NurbsCurve,
    second: &NurbsCurve,
    tolerance: f64,
) -> Result<Vec<CurveCurveIntersection>, String> {
    let mode = line_line_mode();
    if mode != LineLineMode::Off {
        if let Some(exact) = exact_line_line(first, second, tolerance) {
            if mode == LineLineMode::On {
                return Ok(exact);
            }
            let general = intersect_curves_general(first, second, tolerance)?;
            line_line_census(first, second, &general, &exact)?;
            return Ok(general);
        }
    }
    intersect_curves_general(first, second, tolerance)
}

/// One census line per call whose general and exact answers differ: both
/// hit lists, and for each general hit its points' distances from the exact
/// closest pair (the residual the exact answer removes).
fn line_line_census(
    first: &NurbsCurve,
    second: &NurbsCurve,
    general: &[CurveCurveIntersection],
    exact: &[CurveCurveIntersection],
) -> Result<(), String> {
    let same = general.len() == exact.len()
        && general.iter().zip(exact).all(|(g, x)| g.s.to_bits() == x.s.to_bits() && g.t.to_bits() == x.t.to_bits());
    if same {
        return Ok(());
    }
    let Ok(path) = std::env::var("BREP_EXACT_LINE_LINE_CENSUS") else {
        return Ok(());
    };
    let mut line = format!("line-line census: label {} general {} exact {}", std::env::var("BREP_CENSUS_LABEL").unwrap_or_default(), general.len(), exact.len());
    if let Some(x) = exact.first() {
        let (exact_first, exact_second) = (first.evaluate(x.s)?, second.evaluate(x.t)?);
        line.push_str(&format!(" exact s={:e} t={:e} gap={:e}", x.s, x.t, x.gap));
        for g in general {
            let residual = first.evaluate(g.s)?.sub(exact_first).length().max(second.evaluate(g.t)?.sub(exact_second).length());
            line.push_str(&format!(" | general s={:e} t={:e} gap={:e} ds={:e} dt={:e} residual={:e}", g.s, g.t, g.gap, g.s - x.s, g.t - x.t, residual));
        }
    } else {
        for g in general {
            line.push_str(&format!(" | general s={:e} t={:e} gap={:e}", g.s, g.t, g.gap));
        }
    }
    use std::io::Write;
    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
        let _ = writeln!(file, "{line}");
    }
    Ok(())
}

/// Every `(i, j)` with `first[i].bounds` and `second[j].bounds` within
/// `tolerance` of each other, in ascending `(i, j)` order — exactly the pairs
/// and the order the nested loop produced. The second list is sorted by its
/// boxes' minimum x once; for each first segment the candidates are the
/// second segments whose minimum x lies at or below the first's maximum x
/// plus the tolerance (a prefix of the sorted order), filtered by the full
/// box test. Each pair is tested by the same `Aabb::intersects` as before.
fn overlapping_segment_pairs(first: &[Segment], second: &[Segment], tolerance: f64) -> Vec<(usize, usize)> {
    let mut order: Vec<usize> = (0..second.len()).collect();
    order.sort_by(|&a, &b| second[a].bounds.minimum.x.total_cmp(&second[b].bounds.minimum.x));
    let minimum_x: Vec<f64> = order.iter().map(|&j| second[j].bounds.minimum.x).collect();
    let mut pairs = Vec::new();
    for (i, segment) in first.iter().enumerate() {
        let limit = segment.bounds.maximum.x + tolerance;
        // Candidates: the prefix with minimum x <= limit.
        let end = minimum_x.partition_point(|&x| x <= limit);
        let mut row: Vec<usize> = order[..end]
            .iter()
            .copied()
            .filter(|&j| segment.bounds.intersects(second[j].bounds, tolerance))
            .collect();
        row.sort_unstable();
        pairs.extend(row.into_iter().map(|j| (i, j)));
    }
    pairs
}

fn intersect_curves_general(
    first: &NurbsCurve,
    second: &NurbsCurve,
    tolerance: f64,
) -> Result<Vec<CurveCurveIntersection>, String> {
    let first_segment_count = 8usize.max((interior_knots(first).len() + 1) * (first.degree + 1));
    let second_segment_count = 8usize.max((interior_knots(second).len() + 1) * (second.degree + 1));
    let first_segments = segment_boxes(first, first_segment_count)?;
    let second_segments = segment_boxes(second, second_segment_count)?;
    // The overlapping segment pairs, in the order the nested loop over
    // `first_segments` then `second_segments` visits them: the seeds' order
    // decides which of two converging seeds is kept as the hit below, so the
    // pairs are gathered by a sweep over the x extents (sorted once, a binary
    // search per first segment) and then put back into that order. Two fitted
    // sections of two thousand segments each cost four million box tests
    // a call before (2026-10-17, the shelled-box report's shell: 0.57 s of
    // `lies_on` spans and 0.55 s of section crossings on three face pairs).
    let seeds = overlapping_segment_pairs(&first_segments, &second_segments, tolerance)
        .into_iter()
        .map(|(i, j)| {
            [
                (first_segments[i].start + first_segments[i].end) * 0.5,
                (second_segments[j].start + second_segments[j].end) * 0.5,
            ]
        })
        .collect::<Vec<_>>();
    let [s0, s1] = first.domain()?;
    let [t0, t1] = second.domain()?;
    let closed_first = first.evaluate(s0)?.sub(first.evaluate(s1)?).length() <= tolerance;
    let closed_second = second.evaluate(t0)?.sub(second.evaluate(t1)?).length() <= tolerance;
    let mut results: Vec<CurveCurveIntersection> = Vec::new();
    for [seed_s, seed_t] in seeds {
        let [s, t] = newton_closest_pair(first, second, seed_s, seed_t)?;
        let first_point = first.evaluate(s)?;
        let second_point = second.evaluate(t)?;
        let gap = first_point.sub(second_point).length();
        if gap > tolerance {
            continue;
        }
        let first_tangent = first.deriv1(s)?.1;
        let second_tangent = second.deriv1(t)?.1;
        let tangential = match (first_tangent.normalized(), second_tangent.normalized()) {
            (Ok(a), Ok(b)) => a.cross(b).length() <= 1e-3_f64.sin(),
            _ => false,
        };
        let point = first_point.add(second_point).scale(0.5);
        let scale = 1.0 + point.length();
        let spatial_tolerance = if tangential {
            tolerance.sqrt() * scale
        } else {
            tolerance * 10.0
        };
        let mut duplicate = false;
        for result in &results {
            if (parameter_distance(result.s, s, s0, s1, closed_first)
                <= parameter_tolerance(first, result.s, tolerance)?
                && parameter_distance(result.t, t, t0, t1, closed_second)
                    <= parameter_tolerance(second, result.t, tolerance)?)
                || result.point.sub(point).length()
                    <= if result.tangential || tangential {
                        spatial_tolerance
                    } else {
                        tolerance * 10.0
                    }
            {
                duplicate = true;
                break;
            }
        }
        if duplicate {
            continue;
        }
        results.push(CurveCurveIntersection {
            s: s.clamp(s0, s1),
            t: t.clamp(t0, t1),
            point,
            gap,
            tangential,
        });
    }
    results.sort_by(|a, b| a.s.total_cmp(&b.s));
    Ok(results)
}

