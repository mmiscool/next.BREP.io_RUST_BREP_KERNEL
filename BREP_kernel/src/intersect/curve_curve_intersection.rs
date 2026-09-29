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

pub fn intersect_curves(
    first: &NurbsCurve,
    second: &NurbsCurve,
    tolerance: f64,
) -> Result<Vec<CurveCurveIntersection>, String> {
    let first_segment_count = 8usize.max((interior_knots(first).len() + 1) * (first.degree + 1));
    let second_segment_count = 8usize.max((interior_knots(second).len() + 1) * (second.degree + 1));
    let first_segments = segment_boxes(first, first_segment_count)?;
    let second_segments = segment_boxes(second, second_segment_count)?;
    let mut seeds = Vec::new();
    for first_segment in &first_segments {
        for second_segment in &second_segments {
            if first_segment
                .bounds
                .intersects(second_segment.bounds, tolerance)
            {
                seeds.push([
                    (first_segment.start + first_segment.end) * 0.5,
                    (second_segment.start + second_segment.end) * 0.5,
                ]);
            }
        }
    }
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

