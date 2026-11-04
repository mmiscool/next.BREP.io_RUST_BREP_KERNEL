use crate::curve::KNOT_IDENTITY_TOL as KNOT_TOLERANCE;
use crate::{NurbsCurve, Vec3};
use serde::Serialize;

const EPSILON: f64 = crate::tolerance::FP_FLOOR;

use crate::spatial::Aabb as Bounds;


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

/// `segment_boxes` served from a small content-keyed memo: the same curve
/// content and count give the same boxes to the bit, and the imprint asks for
/// one section's boxes once per boundary edge it is crossed with (87 times on
/// the planter's wall, 1660 splits of a 280-control-point curve each, 107 ms
/// of a 560 ms imprint, 2026-11-02). Eight entries, most
/// recent first; the key is the curve's knot and control bits and the count.
fn segment_boxes_memoized(curve: &NurbsCurve, count: usize) -> Result<std::rc::Rc<Vec<Segment>>, String> {
    thread_local! {
        static SEGMENT_MEMO: std::cell::RefCell<Vec<([u64; 2], std::rc::Rc<Vec<Segment>>)>> = const { std::cell::RefCell::new(Vec::new()) };
    }
    let key = {
        let mut lane = crate::mass_properties::TwoLane::new();
        crate::mass_properties::hash_curve(&mut lane, curve);
        lane.usize(count);
        lane.finish()
    };
    let hit = SEGMENT_MEMO.with(|memo| {
        let mut memo = memo.borrow_mut();
        let at = memo.iter().position(|(stored, _)| *stored == key)?;
        if at != 0 {
            let entry = memo.remove(at);
            memo.insert(0, entry);
        }
        Some(memo[0].1.clone())
    });
    if let Some(boxes) = hit {
        return Ok(boxes);
    }
    let boxes = std::rc::Rc::new(segment_boxes(curve, count)?);
    SEGMENT_MEMO.with(|memo| {
        let mut memo = memo.borrow_mut();
        memo.insert(0, (key, boxes.clone()));
        memo.truncate(8);
    });
    Ok(boxes)
}

/// The control-point box of each knot span of `curve` (its Bezier pieces,
/// split at every interior knot): for positive weights each span lies inside
/// its box, so two curves whose span boxes are pairwise apart on some axis by
/// more than `d` are more than `d` apart everywhere. `None` when a weight is
/// not positive (the hull property does not hold).
pub(crate) fn span_boxes(curve: &NurbsCurve) -> Result<Option<Vec<Bounds>>, String> {
    if !curve.control_points.iter().all(|control| control.w > 0.0) {
        return Ok(None);
    }
    Ok(Some(segment_boxes(curve, 1)?.into_iter().map(|segment| segment.bounds).collect()))
}

fn segment_boxes(curve: &NurbsCurve, count: usize) -> Result<Vec<Segment>, String> {
    if crate::switch!(live "BREP_SEGMENT_BOX_WALK", true) {
        if let Some(segments) = segment_boxes_walk(curve, count)? {
            return Ok(segments);
        }
    }
    segment_boxes_split(curve, count)
}

/// SEGMENT-BOX WALK (`BREP_SEGMENT_BOX_WALK`, default on, =0 restores;
/// 2026-11-03). [`segment_boxes_split`] splits the REMAINDER of the curve at
/// every parameter, and each `NurbsCurve::split` copies and validates the whole
/// remainder: O(n) per split, about 2000 splits of a 300-control-point section
/// (8-9 ms per curve natively; the elephant's and the planter's crossing
/// searches are 60 and 73 ms of box building). Boehm insertion touches only
/// the knots and controls around the split, and the remainder's tail is always
/// the ORIGINAL curve's, bit for bit; so this keeps the remainder as the last
/// split's right half (a short head) followed by an offset into the original,
/// and splits a WINDOW of it (the head and the original up to `degree + 1`
/// knots past the parameter) with [`split_window`], `split`'s own arithmetic
/// on slices. Every box is the same bits as the split path's; anything this
/// cannot take (a parameter `split` would refuse) hands the whole curve back to
/// [`segment_boxes_split`], so errors are the split path's too.
fn segment_boxes_walk(curve: &NurbsCurve, count: usize) -> Result<Option<Vec<Segment>>, String> {
    let [start, end] = curve.domain()?;
    let mut parameters: Vec<f64> = (0..=count)
        .map(|index| start + (end - start) * index as f64 / count as f64)
        .collect();
    parameters.extend(interior_knots(curve));
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|a, b| (*a - *b).abs() <= KNOT_TOLERANCE);

    let degree = curve.degree;
    let (knots, points) = (&curve.knots, &curve.control_points);
    // The remainder: `head_*` followed by `knots[tail_knot..]`, `points[tail_point..]`.
    let mut head_knots: Vec<f64> = Vec::new();
    let mut head_points: Vec<crate::Vec4> = Vec::new();
    let (mut tail_knot, mut tail_point) = (0usize, 0usize);
    let mut segments = Vec::new();
    let mut rest_start = start;
    for (index, &parameter) in parameters.iter().enumerate().skip(1) {
        if parameter - rest_start <= KNOT_TOLERANCE {
            continue;
        }
        let knot_count = head_knots.len() + knots.len() - tail_knot;
        let point_count = head_points.len() + points.len() - tail_point;
        let rest_knot = |at: usize| if at < head_knots.len() { head_knots[at] } else { knots[tail_knot + at - head_knots.len()] };
        let rest_point = |at: usize| if at < head_points.len() { head_points[at] } else { points[tail_point + at - head_points.len()] };
        if index == parameters.len() - 1 {
            let rest = NurbsCurve::new(degree, (0..knot_count).map(rest_knot).collect(), (0..point_count).map(rest_point).collect())?;
            segments.push(Segment { start: rest_start, end, bounds: Bounds::from_curve_controls(&rest)? });
            continue;
        }
        // `split`'s own refusal: a parameter not strictly inside the remainder.
        let (rest_low, rest_high) = (rest_knot(degree), rest_knot(knot_count - 1 - degree));
        if parameter <= rest_low + KNOT_TOLERANCE || parameter >= rest_high - KNOT_TOLERANCE {
            return Ok(None);
        }
        let mut beyond = degree + 1;
        while beyond < knot_count && rest_knot(beyond) <= parameter + KNOT_TOLERANCE {
            beyond += 1;
        }
        let window_points = (beyond + degree + 1).max(head_points.len()).min(point_count);
        let window_knots = window_points + degree + 1;
        let window_knot_values: Vec<f64> = (0..window_knots).map(rest_knot).collect();
        let window_point_values: Vec<crate::Vec4> = (0..window_points).map(rest_point).collect();
        let Some((left_knots, left_points, right_knots, right_points)) =
            split_window(&window_knot_values, &window_point_values, degree, parameter)
        else {
            return Ok(None);
        };
        let piece = NurbsCurve::new(degree, left_knots, left_points)?;
        segments.push(Segment { start: rest_start, end: parameter, bounds: Bounds::from_curve_controls(&piece)? });
        tail_point += window_points - head_points.len();
        tail_knot += window_knots - head_knots.len();
        head_points = right_points;
        head_knots = right_knots;
        rest_start = parameter;
    }
    Ok(Some(segments))
}

/// `NurbsCurve::split` (and the `insert_knot` it calls) on the leading
/// WINDOW of a curve's knots and controls, with the same operations in the
/// same order: Boehm insertion reads only the knots up to `span + degree` and
/// the controls up to `span`, the span search finds the one interval holding
/// the parameter, and the window reaches `degree + 1` knots past it. Returns
/// the left piece and the window's part of the right piece; `None` where
/// `split` would refuse.
#[allow(clippy::type_complexity)]
fn split_window(
    knots: &[f64],
    points: &[crate::Vec4],
    degree: usize,
    parameter: f64,
) -> Option<(Vec<f64>, Vec<crate::Vec4>, Vec<f64>, Vec<crate::Vec4>)> {
    use crate::curve::{knot_clamp, knot_find_span};
    let parameter = knots.iter().copied().find(|knot| (knot - parameter).abs() <= KNOT_TOLERANCE).unwrap_or(parameter);
    let multiplicity = knots.iter().filter(|knot| (**knot - parameter).abs() <= KNOT_TOLERANCE).count();
    let requested = degree.saturating_sub(multiplicity);
    // insert_knot(parameter, requested)
    let inserted = {
        let at = knot_clamp(knots, degree, parameter);
        let at = knots.iter().copied().find(|knot| (knot - at).abs() <= KNOT_TOLERANCE).unwrap_or(at);
        let multiplicity = knots.iter().filter(|knot| (**knot - at).abs() <= KNOT_TOLERANCE).count();
        let insertion_count = requested.min(degree.saturating_sub(multiplicity));
        if insertion_count == 0 {
            (knots.to_vec(), points.to_vec())
        } else {
            let span = knot_find_span(knots, degree, at);
            let last_control = points.len() - 1;
            let mut refined_knots = Vec::with_capacity(knots.len() + insertion_count);
            refined_knots.extend_from_slice(&knots[..=span]);
            refined_knots.extend(std::iter::repeat_n(at, insertion_count));
            refined_knots.extend_from_slice(&knots[span + 1..]);
            let mut output = vec![crate::Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }; last_control + 1 + insertion_count];
            output[..=span - degree].copy_from_slice(&points[..=span - degree]);
            for index in span - multiplicity..=last_control {
                output[index + insertion_count] = points[index];
            }
            let mut affected = vec![crate::Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }; degree + 1];
            affected[..=degree - multiplicity].copy_from_slice(&points[span - degree..=span - multiplicity]);
            let mut left = 0;
            for insertion in 1..=insertion_count {
                left = span - degree + insertion;
                for index in 0..=degree - insertion - multiplicity {
                    let denominator = knots[index + span + 1] - knots[left + index];
                    let alpha = (at - knots[left + index]) / denominator;
                    affected[index] = affected[index + 1].scale(alpha).add(affected[index].scale(1.0 - alpha));
                }
                output[left] = affected[0];
                output[span + insertion_count - insertion - multiplicity] = affected[degree - insertion - multiplicity];
            }
            for index in left + 1..span - multiplicity {
                output[index] = affected[index - left];
            }
            (refined_knots, output)
        }
    };
    let (refined_knots, refined_points) = inserted;
    let first = refined_knots.iter().position(|knot| (*knot - parameter).abs() <= KNOT_TOLERANCE)?;
    if first == 0 {
        return None;
    }
    let mut left_knots = refined_knots[..first + degree].to_vec();
    left_knots.push(parameter);
    let left_points = refined_points[..first].to_vec();
    let mut right_knots = vec![parameter; degree + 1];
    right_knots.extend_from_slice(&refined_knots[first + degree..]);
    let right_points = refined_points[first - 1..].to_vec();
    Some((left_knots, left_points, right_knots, right_points))
}

fn segment_boxes_split(curve: &NurbsCurve, count: usize) -> Result<Vec<Segment>, String> {
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
                bounds: Bounds::from_curve_controls(&rest)?,
            });
        } else {
            let (piece, remainder) = rest.split(parameter)?;
            segments.push(Segment {
                start: rest_start,
                end: parameter,
                bounds: Bounds::from_curve_controls(&piece)?,
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
    curvature: bool,
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
        let j00 = first_derivatives[1].length_squared()
            + if curvature { first_derivatives[2].dot(residual) } else { 0.0 };
        let j01 = -first_derivatives[1].dot(second_derivatives[1]);
        let j10 = -second_derivatives[1].dot(first_derivatives[1]);
        let j11 = second_derivatives[1].length_squared()
            - if curvature { second_derivatives[2].dot(residual) } else { 0.0 };
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


/// Lane O batch 22 (tier-3 finding 1, `BREP_CURVE_TANGENT_MERGE_LOCAL`,
/// default on since batch 27; `1`/`on`, `census`): the duplicate guard's radius for
/// TANGENTIAL hits and the cluster's representative.
///
/// Today a tangential hit merges every other hit within
/// `sqrt(tolerance)·(1 + |p|)` of it — a radius that grows with the point's
/// distance from the origin, not with the contact: at |p| ≈ 76 it is 0.24
/// and swallows every tangential hit of a 3.8e-3-radius pair but one (the
/// 2026-11-02 conic-conic census, record 6), and the one it keeps is the
/// first seed's, not the closest (record 54: a 1.1e-7 graze represents a
/// 9e-14 crossing). `On` derives the radius from the contact itself: the
/// half-width of the band where the two curves stay within the tolerance,
/// `min(tolerance / sin θ, sqrt(2·tolerance·ρ))` with θ the tangent angle
/// and ρ = 1 / |κ₁n₁ − κ₂n₂| the relative curvature radius, CAPPED by
/// today's radius (never wider; a near-coincident stretch with ρ → ∞ merges
/// exactly as today) and floored at the transverse 10·tolerance; and a
/// cluster is represented by its smallest-gap hit. `Census` answers as
/// today and appends one record per call where the two rules differ in a
/// merge decision or a representative to the file
/// `BREP_CURVE_TANGENT_MERGE_CENSUS` names (one write per record).
#[derive(Clone, Copy, PartialEq, Eq)]
enum TangentMergeMode {
    Off,
    On,
    Census,
}

fn tangent_merge_mode() -> TangentMergeMode {
    static MODE: std::sync::OnceLock<TangentMergeMode> = std::sync::OnceLock::new();
    // On by default since 2026-11-03 (lane O batch 27); `0`/`off` restores
    // today's |p|-scaled radius.
    let mode = *MODE.get_or_init(|| match std::env::var("BREP_CURVE_TANGENT_MERGE_LOCAL").as_deref() {
        Ok("0") | Ok("off") => TangentMergeMode::Off,
        Ok("census") => TangentMergeMode::Census,
        _ => TangentMergeMode::On,
    });
    mode
}


/// The contact-derived merge radius at a hit `(s, t)`: the half-width of the
/// band where `first` and `second` stay within `tolerance` of each other,
/// `min(tolerance / sin θ, sqrt(2·tolerance·ρ))`, capped by `legacy` (today's
/// radius) and floored at `10·tolerance`. A hit whose tangents cannot be read
/// keeps `legacy`.
fn local_tangent_merge_radius(
    first: &NurbsCurve,
    second: &NurbsCurve,
    s: f64,
    t: f64,
    tolerance: f64,
    legacy: f64,
) -> Result<(f64, f64, f64), String> {
    let a = first.derivatives_small(s, 2)?;
    let b = second.derivatives_small(t, 2)?;
    let (speed_a, speed_b) = (a[1].length(), b[1].length());
    if speed_a <= EPSILON || speed_b <= EPSILON {
        return Ok((legacy, f64::NAN, f64::NAN));
    }
    let tangent_a = a[1].scale(1.0 / speed_a);
    let tangent_b = b[1].scale(1.0 / speed_b);
    let sin_theta = tangent_a.cross(tangent_b).length().min(1.0);
    // Curvature vectors κn = (d2 − (d2·t)t) / |d1|²: orientation-free, so the
    // two curves' traversal senses do not matter.
    let curvature_a = a[2].sub(tangent_a.scale(a[2].dot(tangent_a))).scale(1.0 / (speed_a * speed_a));
    let curvature_b = b[2].sub(tangent_b.scale(b[2].dot(tangent_b))).scale(1.0 / (speed_b * speed_b));
    let relative = curvature_a.sub(curvature_b).length();
    let mut radius = f64::INFINITY;
    if sin_theta > 0.0 {
        radius = radius.min(tolerance / sin_theta);
    }
    if relative > 0.0 {
        radius = radius.min((2.0 * tolerance / relative).sqrt());
    }
    Ok((radius.min(legacy).max(10.0 * tolerance), sin_theta, relative))
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
            line_line_census(first, second, tolerance, &general, &exact)?;
            return Ok(general);
        }
    }
    let mode = line_conic_mode();
    if mode != LineLineMode::Off {
        if let Some(seeds) = exact_line_conic_pairs(first, second, tolerance) {
            let exact = intersect_curves_from_seeds(first, second, tolerance, seeds)?;
            if mode == LineLineMode::On {
                return Ok(exact);
            }
            let general = intersect_curves_general(first, second, tolerance)?;
            crossing_census("line-conic", "BREP_EXACT_LINE_CONIC_CENSUS", first, second, tolerance, &general, &exact)?;
            return Ok(general);
        }
    }
    let mode = conic_conic_mode();
    if mode != LineLineMode::Off {
        if let Some(seeds) = exact_conic_conic_pairs(first, second, tolerance) {
            let exact = intersect_curves_from_seeds(first, second, tolerance, seeds)?;
            if mode == LineLineMode::On {
                return Ok(exact);
            }
            let general = intersect_curves_general(first, second, tolerance)?;
            // Self-check (lane O batch 22, finding 2): the 2026-11-02 census
            // recorded one general crossing for a pair every later replay
            // answers with two; a second call here, compared by bits, makes
            // the record say whether the search itself wavered in the run.
            general_rerun_census("conic-conic", "BREP_EXACT_CONIC_CONIC_CENSUS", first, second, tolerance, &general)?;
            crossing_census("conic-conic", "BREP_EXACT_CONIC_CONIC_CENSUS", first, second, tolerance, &general, &exact)?;
            return Ok(general);
        }
    }
    intersect_curves_general(first, second, tolerance)
}

/// Lane O row 40 (`BREP_EXACT_CONIC_CONIC`, default on since batch 27, `0` restores; `1`/`on` answers by
/// it, `census` answers by the general search and appends one line per call
/// whose answers differ to `BREP_EXACT_CONIC_CONIC_CENSUS`): two planar curves
/// of degree two seed the general search's own Newton, gap filter, duplicate
/// guard and tangency reading ([`intersect_curves_from_seeds`]) at closed-form
/// crossings instead of at the midpoints of every overlapping pair of segment
/// boxes. See [`exact_conic_conic_pairs`].
fn conic_conic_mode() -> LineLineMode {
    static MODE: std::sync::OnceLock<LineLineMode> = std::sync::OnceLock::new();
    // On by default since 2026-11-03 (lane O batch 27); `0`/`off` restores
    // the general search for two planar conics.
    let mode = *MODE.get_or_init(|| match std::env::var("BREP_EXACT_CONIC_CONIC").as_deref() {
        Ok("0") | Ok("off") => LineLineMode::Off,
        Ok("census") => LineLineMode::Census,
        _ => LineLineMode::On,
    });
    mode
}


/// The plane of a planar curve of degree two with positive finite weights:
/// a control point, the unit normal of the widest control triangle, and the
/// net's extent. `None` for collinear controls or a net out of one plane by
/// more than 1e-12 of its extent (row 28's conic test).
fn conic_plane(curve: &NurbsCurve) -> Option<(Vec3, Vec3, f64)> {
    if curve.degree != 2 || curve.control_points.len() < 3 {
        return None;
    }
    if curve.control_points.iter().any(|p| !(p.w > 0.0 && p.w.is_finite())) {
        return None;
    }
    let points = curve.control_points.iter().map(|p| p.point().ok()).collect::<Option<Vec<_>>>()?;
    let base = points[0];
    let extent = points.iter().fold(0.0_f64, |m, p| m.max(p.sub(base).length()));
    if !(extent > 0.0 && extent.is_finite()) {
        return None;
    }
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    let mut area = 0.0_f64;
    for i in 1..points.len() {
        for j in i + 1..points.len() {
            let cross = points[i].sub(base).cross(points[j].sub(base));
            if cross.length() > area {
                area = cross.length();
                normal = cross;
            }
        }
    }
    if !(area > 1e-9 * extent * extent) {
        return None;
    }
    let normal = normal.scale(1.0 / area);
    if points.iter().any(|p| normal.dot(p.sub(base)).abs() > 1e-12 * extent) {
        return None;
    }
    Some((base, normal, extent))
}

/// The `[s, t]` Newton seeds of row 40 for two planar curves of degree two;
/// `None` when the lane declines.
///
/// - Planes that cross (more than a milliradian apart): every crossing lies on
///   the line the planes share, so `first`'s seeds are row 28's crossings of
///   `first` with that line ([`crate::exact_line_surface::exact_line_conic_seeds`],
///   grazes within a hundred tolerances included).
/// - One plane, `second` a circle (recognized to 1e-12 of its size): a curve
///   in a circle's plane crosses it where it crosses the sphere on that
///   circle, so `first`'s seeds are row 27's sphere roots
///   ([`crate::exact_line_surface::exact_curve_sphere_seeds`], which declines
///   a span on the sphere and a double root). A sample of `first` within a
///   hundred tolerances of the circle with no seed declines too: a near
///   graze the general search would report.
/// - Anything else (parallel planes, an ellipse sharing a plane, a non-planar
///   net) declines.
///
/// Each seed is paired with its point's projection onto `second`; the general
/// search's Newton and gap filter decide what is a crossing.
fn exact_conic_conic_pairs(first: &NurbsCurve, second: &NurbsCurve, tolerance: f64) -> Option<Vec<[f64; 2]>> {
    let (base_a, normal_a, extent_a) = conic_plane(first)?;
    let (base_b, normal_b, extent_b) = conic_plane(second)?;
    let extent = extent_a.max(extent_b);
    let shared = normal_a.cross(normal_b);
    let seeds = if shared.length() > 1e-3_f64.sin() {
        // The line both planes hold, through the point of it nearest `base_a`.
        let direction = shared.scale(1.0 / shared.length());
        let (h_a, h_b) = (normal_a.dot(base_a), normal_b.dot(base_b));
        let denominator = shared.dot(shared);
        let on_line = normal_b.cross(shared).scale(h_a / denominator).add(shared.cross(normal_a).scale(h_b / denominator));
        let foot = on_line.add(direction.scale(base_a.sub(on_line).dot(direction)));
        let reach = 2.0 * (extent + base_a.sub(foot).length()).max(1.0);
        crate::exact_line_surface::exact_line_conic_seeds(foot.sub(direction.scale(reach)), foot.add(direction.scale(reach)), first, tolerance)?
    } else if normal_a.dot(base_b.sub(base_a)).abs() <= 1e-12 * extent {
        let [t0, t1] = second.domain().ok()?;
        let circle = crate::recognize_circle(second, t0, t1, 1e-12 * extent.max(1.0))?;
        let seeds = crate::exact_line_surface::exact_curve_sphere_seeds(first, circle.centre, circle.radius, tolerance)?;
        if seeds.is_empty() {
            let [s0, s1] = first.domain().ok()?;
            let band = 100.0 * tolerance;
            for i in 0..=64 {
                let point = first.evaluate(s0 + (s1 - s0) * i as f64 / 64.0).ok()?;
                if (point.sub(circle.centre).length() - circle.radius).abs() <= band {
                    return None;
                }
            }
        }
        seeds
    } else {
        return None;
    };
    seeds
        .into_iter()
        .map(|s| {
            let point = first.evaluate(s).ok()?;
            let projection = crate::project_point_to_curve(second, point).ok()?;
            Some([s, projection.u])
        })
        .collect()
}

/// Lane O row 28 (`BREP_EXACT_LINE_CONIC`, default on since batch 12; `0`/`off`
/// restores the general search, `census` answers by it and appends one line per call
/// whose answers differ to `BREP_EXACT_LINE_CONIC_CENSUS`): a straight
/// segment against a planar curve of degree two, either order, seeds the
/// general search's own Newton, gap filter, duplicate guard and tangency
/// reading ([`intersect_curves_from_seeds`]) at the closed-form crossings of
/// [`crate::exact_line_surface::exact_line_conic_seeds`] instead of at the
/// midpoints of every overlapping pair of segment boxes. Each conic seed is
/// paired with its point's projection onto the segment.
fn line_conic_mode() -> LineLineMode {
    static MODE: std::sync::OnceLock<LineLineMode> = std::sync::OnceLock::new();
    let mode = *MODE.get_or_init(|| match std::env::var("BREP_EXACT_LINE_CONIC").as_deref() {
        Ok("0") | Ok("off") => LineLineMode::Off,
        Ok("census") => LineLineMode::Census,
        _ => LineLineMode::On,
    });
    mode
}


/// The `[s, t]` Newton seeds of row 28 for `first` against `second`, when one
/// is a straight segment and the other a planar curve of degree two; `None`
/// when the lane declines.
fn exact_line_conic_pairs(first: &NurbsCurve, second: &NurbsCurve, tolerance: f64) -> Option<Vec<[f64; 2]>> {
    let (line_first, (p0, p1, [l0, l1]), conic) = match (straight_segment(first), straight_segment(second)) {
        (Some(segment), None) => (true, segment, second),
        (None, Some(segment)) => (false, segment, first),
        _ => return None,
    };
    let seeds = crate::exact_line_surface::exact_line_conic_seeds(p0, p1, conic, tolerance)?;
    let direction = p1.sub(p0);
    let length_squared = direction.dot(direction);
    seeds
        .into_iter()
        .map(|t| {
            let point = conic.evaluate(t).ok()?;
            let along = (point.sub(p0).dot(direction) / length_squared).clamp(0.0, 1.0);
            let s = l0 + along * (l1 - l0);
            Some(if line_first { [s, t] } else { [t, s] })
        })
        .collect()
}

/// One census line per call whose general and exact answers differ: both
/// hit lists, and for each general hit its points' distances from the exact
/// closest pair (the residual the exact answer removes).
fn line_line_census(
    first: &NurbsCurve,
    second: &NurbsCurve,
    tolerance: f64,
    general: &[CurveCurveIntersection],
    exact: &[CurveCurveIntersection],
) -> Result<(), String> {
    crossing_census("line-line", "BREP_EXACT_LINE_LINE_CENSUS", first, second, tolerance, general, exact)
}

/// Census self-check: the general search run a second time on the same
/// pair, appended as `<lane> rerun census: … general <n> rerun <m>` with
/// both hit lists when any hit differs by bits (one write per record).
fn general_rerun_census(
    lane: &str,
    variable: &str,
    first: &NurbsCurve,
    second: &NurbsCurve,
    tolerance: f64,
    general: &[CurveCurveIntersection],
) -> Result<(), String> {
    let Ok(path) = std::env::var(variable) else {
        return Ok(());
    };
    let rerun = intersect_curves_general(first, second, tolerance)?;
    let same = general.len() == rerun.len()
        && general.iter().zip(&rerun).all(|(g, r)| g.s.to_bits() == r.s.to_bits() && g.t.to_bits() == r.t.to_bits());
    if same {
        return Ok(());
    }
    let mut line = format!("{lane} rerun census: tolerance={tolerance:e} general {} rerun {}", general.len(), rerun.len());
    for g in general {
        line.push_str(&format!(" | general s={:e} t={:e} gap={:e}", g.s, g.t, g.gap));
    }
    for r in &rerun {
        line.push_str(&format!(" | rerun s={:e} t={:e} gap={:e}", r.s, r.t, r.gap));
    }
    if let (Ok(a), Ok(b)) = (serde_json::to_string(first), serde_json::to_string(second)) {
        line.push_str(&format!(" | curves=[{a},{b}]"));
    }
    line.push('\n');
    use std::io::Write;
    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
        let _ = file.write_all(line.as_bytes());
    }
    Ok(())
}

/// [`line_line_census`] for the lane named `lane`, appending to the file that
/// `variable` names. The record names the call's tolerance, every exact hit,
/// and every general hit with its parameter offset and spatial residual from
/// the nearest exact hit. When the two answers disagree on the hit count, or a
/// general hit sits further than the tolerance from every exact hit, the
/// record ends with both curves as JSON (`curves=`) so the pair can be
/// replayed. One record is one `write` so concurrent callers never splice.
fn crossing_census(
    lane: &str,
    variable: &str,
    first: &NurbsCurve,
    second: &NurbsCurve,
    tolerance: f64,
    general: &[CurveCurveIntersection],
    exact: &[CurveCurveIntersection],
) -> Result<(), String> {
    let same = general.len() == exact.len()
        && general.iter().zip(exact).all(|(g, x)| g.s.to_bits() == x.s.to_bits() && g.t.to_bits() == x.t.to_bits());
    if same {
        return Ok(());
    }
    let Ok(path) = std::env::var(variable) else {
        return Ok(());
    };
    let mut line = format!(
        "{lane} census: label {} tolerance={:e} general {} exact {}",
        std::env::var("BREP_CENSUS_LABEL").unwrap_or_default(),
        tolerance,
        general.len(),
        exact.len()
    );
    let mut exact_points = Vec::with_capacity(exact.len());
    for x in exact {
        exact_points.push((first.evaluate(x.s)?, second.evaluate(x.t)?));
        line.push_str(&format!(" | exact s={:e} t={:e} gap={:e}", x.s, x.t, x.gap));
    }
    let mut unmatched = false;
    for g in general {
        let (on_first, on_second) = (first.evaluate(g.s)?, second.evaluate(g.t)?);
        let nearest = exact
            .iter()
            .zip(&exact_points)
            .map(|(x, (xf, xs))| (on_first.sub(*xf).length().max(on_second.sub(*xs).length()), x))
            .min_by(|a, b| a.0.total_cmp(&b.0));
        match nearest {
            Some((residual, x)) => {
                unmatched |= residual > tolerance;
                line.push_str(&format!(
                    " | general s={:e} t={:e} gap={:e} ds={:e} dt={:e} residual={:e}",
                    g.s,
                    g.t,
                    g.gap,
                    g.s - x.s,
                    g.t - x.t,
                    residual
                ));
            }
            None => {
                unmatched = true;
                line.push_str(&format!(" | general s={:e} t={:e} gap={:e}", g.s, g.t, g.gap));
            }
        }
    }
    if general.len() != exact.len() || unmatched {
        if let (Ok(a), Ok(b)) = (serde_json::to_string(first), serde_json::to_string(second)) {
            line.push_str(&format!(" | curves=[{a},{b}]"));
        }
    }
    line.push('\n');
    use std::io::Write;
    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
        let _ = file.write_all(line.as_bytes());
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
            .filter(|&j| segment.bounds.intersects_upper(second[j].bounds, tolerance))
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
    let first_segments = segment_boxes_memoized(first, first_segment_count)?;
    let second_segments = segment_boxes_memoized(second, second_segment_count)?;
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
    intersect_curves_from_seeds(first, second, tolerance, seeds)
}

/// The general search past its seeding: a Newton run from each `[s, t]`
/// seed (a Gauss–Newton retry for those that fail), then the gap filter, the
/// duplicate guard and the tangency reading, sorted by `s`.
fn intersect_curves_from_seeds(
    first: &NurbsCurve,
    second: &NurbsCurve,
    tolerance: f64,
    seeds: Vec<[f64; 2]>,
) -> Result<Vec<CurveCurveIntersection>, String> {
    let [s0, s1] = first.domain()?;
    let [t0, t1] = second.domain()?;
    let closed_first = first.evaluate(s0)?.sub(first.evaluate(s1)?).length() <= tolerance;
    let closed_second = second.evaluate(t0)?.sub(second.evaluate(t1)?).length() <= tolerance;
    // The distance Hessian can lead Newton to a nonzero stationary point
    // instead of a crossing. For a secant of a closed circle, every box seed
    // can converge to the circle's seam this way. Retry failed seeds with
    // the positive-semidefinite Gauss–Newton system (no curvature terms).
    // Keep the successful Newton candidates first so their established
    // parameters win deduplication; the retry must meet the same gap test.
    let mut candidates = Vec::with_capacity(seeds.len());
    let mut failed = Vec::new();
    for [seed_s, seed_t] in seeds {
        let pair = newton_closest_pair(first, second, seed_s, seed_t, true)?;
        if first.evaluate(pair[0])?.sub(second.evaluate(pair[1])?).length() <= tolerance {
            candidates.push(pair);
        } else {
            failed.push([seed_s, seed_t]);
        }
    }
    for [seed_s, seed_t] in failed {
        candidates.push(newton_closest_pair(first, second, seed_s, seed_t, false)?);
    }
    let mut results: Vec<CurveCurveIntersection> = Vec::new();
    // The contact radius of each kept TANGENTIAL hit (`BREP_CURVE_TANGENT_MERGE_LOCAL`
    // on or census), so a later TRANSVERSE candidate near it is judged by that
    // hit's contact, not only by its own transverse band: the rule is the same
    // whichever of the two seeds arrives first.
    let mut kept_radii: Vec<Option<f64>> = Vec::new();
    for [s, t] in candidates {
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
        // Lane O batch 22: the contact-derived tangential radius (see
        // `TangentMergeMode`). Read only when the rule is on or counted.
        let merge_mode = tangent_merge_mode();
        let local = if tangential && merge_mode != TangentMergeMode::Off {
            Some(local_tangent_merge_radius(first, second, s, t, tolerance, spatial_tolerance)?)
        } else {
            None
        };
        let local_tolerance = local.map(|(radius, _, _)| radius).unwrap_or(spatial_tolerance);
        // The duplicate each rule finds: today's radius and the local one.
        let (mut legacy_duplicate, mut local_duplicate) = (None, None);
        for (index, result) in results.iter().enumerate() {
            let in_parameter = parameter_distance(result.s, s, s0, s1, closed_first)
                <= parameter_tolerance(first, result.s, tolerance)?
                && parameter_distance(result.t, t, t0, t1, closed_second)
                    <= parameter_tolerance(second, result.t, tolerance)?;
            let distance = result.point.sub(point).length();
            let either_tangential = result.tangential || tangential;
            if legacy_duplicate.is_none()
                && (in_parameter || distance <= if either_tangential { spatial_tolerance } else { tolerance * 10.0 })
            {
                legacy_duplicate = Some(index);
            }
            // Local: the tangential member's contact radius, the larger of the
            // two where both are tangential; a transverse pair keeps 10·tol.
            let local_radius = match (tangential, kept_radii.get(index).copied().flatten()) {
                (true, Some(kept)) => local_tolerance.max(kept),
                (true, None) => local_tolerance,
                (false, Some(kept)) => kept,
                (false, None) => tolerance * 10.0,
            };
            if local_duplicate.is_none() && (in_parameter || distance <= local_radius) {
                local_duplicate = Some(index);
            }
            if legacy_duplicate.is_some() && (merge_mode == TangentMergeMode::Off || local_duplicate.is_some()) {
                break;
            }
        }
        let candidate = CurveCurveIntersection {
            s: s.clamp(s0, s1),
            t: t.clamp(t0, t1),
            point,
            gap,
            tangential,
        };
        let candidate_radius = local.map(|(radius, _, _)| radius);
        match merge_mode {
            TangentMergeMode::Off => {
                if legacy_duplicate.is_some() {
                    continue;
                }
                results.push(candidate);
            }
            TangentMergeMode::On => match local_duplicate {
                // The cluster keeps its smallest-gap hit.
                Some(index) => {
                    if gap < results[index].gap {
                        results[index] = candidate;
                        kept_radii[index] = candidate_radius;
                    }
                }
                None => {
                    results.push(candidate);
                    kept_radii.push(candidate_radius);
                }
            },
            TangentMergeMode::Census => {
                // Answer as today; record where the local rule would differ:
                // a different merge decision, or a representative the local
                // rule would replace by this smaller-gap hit.
                let legacy_keeps = legacy_duplicate.is_none();
                let local_keeps = local_duplicate.is_none();
                let would_replace = local_duplicate.is_some_and(|index| gap < results[index].gap);
                if legacy_keeps != local_keeps || would_replace {
                    if let Ok(path) = std::env::var("BREP_CURVE_TANGENT_MERGE_CENSUS") {
                        let (local_radius, sin_theta, relative) = local.unwrap_or((spatial_tolerance, f64::NAN, f64::NAN));
                        let against = legacy_duplicate.or(local_duplicate).map(|index| &results[index]);
                        let mut line = format!(
                            "tangent-merge census: label {} tolerance={:e} p={:e} legacy_radius={:e} local_radius={:e} sin_theta={:e} relative_curvature={:e} gap={:e} tangential={} legacy_keeps={} local_keeps={} would_replace={}",
                            std::env::var("BREP_CENSUS_LABEL").unwrap_or_default(),
                            tolerance,
                            point.length(),
                            spatial_tolerance,
                            local_radius,
                            sin_theta,
                            relative,
                            gap,
                            tangential,
                            legacy_keeps,
                            local_keeps,
                            would_replace,
                        );
                        if let Some(other) = against {
                            line.push_str(&format!(
                                " | against s={:e} t={:e} gap={:e} tangential={} distance={:e}",
                                other.s,
                                other.t,
                                other.gap,
                                other.tangential,
                                other.point.sub(point).length()
                            ));
                        }
                        if let (Ok(a), Ok(b)) = (serde_json::to_string(first), serde_json::to_string(second)) {
                            line.push_str(&format!(" | curves=[{a},{b}]"));
                        }
                        line.push('\n');
                        use std::io::Write;
                        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                            let _ = file.write_all(line.as_bytes());
                        }
                    }
                }
                if legacy_duplicate.is_some() {
                    continue;
                }
                results.push(candidate);
                kept_radii.push(candidate_radius);
            }
        }
    }
    results.sort_by(|a, b| a.s.total_cmp(&b.s));
    Ok(results)
}



