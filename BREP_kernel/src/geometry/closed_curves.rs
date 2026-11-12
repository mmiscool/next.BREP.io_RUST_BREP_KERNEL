//! Closed curves beyond the circle: the ellipse and the periodic B-spline, as
//! the clamped rational curves they are exactly, a closed curve cut into the
//! two-curve loop a profile is, and the ellipsoid.
use crate::{make_arc, make_sphere_brep_framed, transform_brep, AffineTransform, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3, Vec4};

/// The unit frame of an ellipse: `major` made a unit vector, and `minor_hint`
/// made square to it in their common plane.
fn frame(name: &str, major: Vec3, minor_hint: Vec3) -> Result<(Vec3, Vec3), String> {
    let x = major.normalized().map_err(|_| format!("{name}: the major direction must be non-zero"))?;
    let y = minor_hint
        .sub(x.scale(minor_hint.dot(x)))
        .normalized()
        .map_err(|_| format!("{name}: the minor direction must not be parallel to the major one"))?;
    Ok((x, y))
}

/// The arc of the ellipse about `center` with semi-axis `semi_major` along
/// `major_direction` and `semi_minor` square to it (toward `minor_hint`), from
/// the parameter `start_angle` to `end_angle` (radians) of
/// `P(t) = center + semi_major·cos t·x + semi_minor·sin t·y`.
///
/// Exact: an ellipse is a circle's affine image, and an affine map carries a
/// rational curve's poles and keeps its weights, so this is the unit circle's
/// arc ([`make_arc`]) with each pole stretched onto the two semi-axes. Either
/// semi-axis may be the longer.
pub fn make_ellipse_arc(
    center: Vec3,
    major_direction: Vec3,
    minor_hint: Vec3,
    semi_major: f64,
    semi_minor: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<NurbsCurve, String> {
    if !(semi_major.is_finite() && semi_minor.is_finite() && semi_major > 0.0 && semi_minor > 0.0) {
        return Err("makeEllipse: both semi-axes must be positive".into());
    }
    let (x, y) = frame("makeEllipse", major_direction, minor_hint)?;
    let origin = Vec3::new(0.0, 0.0, 0.0);
    let unit = make_arc(origin, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 1.0, start_angle, end_angle)?;
    let poles = unit
        .control_points
        .iter()
        .map(|pole| {
            // the pole's own point (x/w, y/w) of the unit circle's plane, stretched and placed
            let point = center.add(x.scale(semi_major * pole.x / pole.w)).add(y.scale(semi_minor * pole.y / pole.w));
            Vec4::from_point(point, pole.w)
        })
        .collect();
    NurbsCurve::new(unit.degree, unit.knots.clone(), poles)
}

/// The whole ellipse of [`make_ellipse_arc`], closed, from its point on the
/// major direction round toward `minor_hint`.
pub fn make_ellipse(center: Vec3, major_direction: Vec3, minor_hint: Vec3, semi_major: f64, semi_minor: f64) -> Result<NurbsCurve, String> {
    make_ellipse_arc(center, major_direction, minor_hint, semi_major, semi_minor, 0.0, std::f64::consts::TAU)
}

/// The closed, periodic uniform B-spline of `degree` on the control polygon
/// `control_points` (each given once; the polygon closes from the last to the
/// first), as a clamped curve over `[0, n]` for `n` control points.
///
/// The periodic curve is the B-spline of the polygon wrapped round by
/// `degree` points on evenly spaced knots, taken over one period. Its two ends
/// are clamped there by knot insertion, which moves no point of the curve: the
/// result has the same `n` polynomial spans, joined as smoothly as the
/// periodic curve joins them (C^(degree−1)), and starts and ends at the same
/// point with the same derivatives.
pub fn make_periodic_bspline(degree: usize, control_points: &[Vec3]) -> Result<NurbsCurve, String> {
    let (p, n) = (degree, control_points.len());
    if p == 0 {
        return Err("makePeriodicBspline: the degree must be at least 1".into());
    }
    if n < p + 1 {
        return Err(format!("makePeriodicBspline: degree {p} needs at least {} control points, got {n}", p + 1));
    }
    // wrapped: n + p poles on the knots 0, 1, …, n + 2p; one period is [p, n + p]
    let mut poles: Vec<[f64; 3]> = (0..n + p).map(|index| control_points[index % n]).map(|point| [point.x, point.y, point.z]).collect();
    let mut knots: Vec<f64> = (0..=n + 2 * p).map(|index| index as f64).collect();
    for at in [p as f64, (n + p) as f64] {
        for _ in 1..p {
            insert_knot(p, &mut knots, &mut poles, at);
        }
    }
    // `at` now stands `p` times at each end: the period's poles run from the one
    // before its first copy to the one before the end's first copy
    let first = knots.iter().position(|&knot| knot == p as f64).expect("the start knot");
    let last = knots.iter().position(|&knot| knot == (n + p) as f64).expect("the end knot");
    let mut clamped = vec![0.0];
    clamped.extend(knots[first..last + p].iter().map(|knot| knot - p as f64));
    clamped.push(n as f64);
    let poles = poles[first - 1..last].iter().map(|pole| Vec4::from_point(Vec3::new(pole[0], pole[1], pole[2]), 1.0)).collect();
    NurbsCurve::new(p, clamped, poles)
}

/// Boehm's knot insertion, once, on a polynomial B-spline's raw knots and
/// poles (the knots need not be clamped).
fn insert_knot(degree: usize, knots: &mut Vec<f64>, poles: &mut Vec<[f64; 3]>, at: f64) {
    // the span k with knots[k] <= at < knots[k + 1]
    let span = knots.iter().rposition(|&knot| knot <= at).expect("a knot at or before the new one");
    let mut refined = Vec::with_capacity(poles.len() + 1);
    for index in 0..=poles.len() {
        let pole = if index + degree <= span {
            poles[index]
        } else if index > span {
            poles[index - 1]
        } else {
            // where the new knot stands on an old one the blend is all `before` (alpha = 0),
            // and at the last span there is no pole after it to read
            let alpha = (at - knots[index]) / (knots[index + degree] - knots[index]);
            let before = poles[index - 1];
            if alpha > 0.0 {
                let after = poles[index];
                [0, 1, 2].map(|axis| (1.0 - alpha) * before[axis] + alpha * after[axis])
            } else {
                before
            }
        };
        refined.push(pole);
    }
    *poles = refined;
    knots.insert(span + 1, at);
}

/// A closed curve as the loop of two curves a profile is: the curve cut at
/// the middle of its domain. `None` when the curve does not close.
pub(crate) fn closed_curve_halves(curve: &NurbsCurve) -> Result<Option<Vec<NurbsCurve>>, String> {
    let [start, end] = curve.domain()?;
    let (from, to) = (curve.evaluate(start)?, curve.evaluate(end)?);
    let size = curve
        .control_points
        .iter()
        .map(|pole| Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w).sub(from).length())
        .fold(0.0, f64::max);
    if size <= 0.0 || from.sub(to).length() > 1e-9 * size {
        return Ok(None);
    }
    let (first, second) = curve.split(0.5 * (start + end))?;
    Ok(Some(vec![first, second]))
}

/// The prism on one closed plane curve, slid by `distance` along `direction`: its
/// side ONE face closed on a seam (the curve times the slide, as the kernel's
/// cylinder is its circle times its axis), its two ends flat faces inside the
/// curve and its copy. `None` when the curve does not close.
pub(crate) fn extrude_closed_curve(curve: &NurbsCurve, direction: Vec3, distance: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    use crate::{face_from_surface, face_from_surface_trimmed, make_plane, solid_from_sheets, NurbsSurface};
    let collect = KernelStage::Collect;
    let [start, end] = curve.domain().or_refuse(collect, "extrudeClosed.domain")?;
    let points: Vec<Vec3> = curve.control_points.iter().map(|pole| Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w)).collect();
    let origin = curve.evaluate(start).or_refuse(collect, "extrudeClosed.start")?;
    let size = points.iter().map(|point| point.sub(origin).length()).fold(0.0, f64::max);
    if size <= 0.0 || origin.sub(curve.evaluate(end).or_refuse(collect, "extrudeClosed.end")?).length() > 1e-9 * size {
        return Ok(None);
    }
    let slide = direction
        .normalized()
        .map_err(|_| KernelRefusal::input(collect, "direction", "extrudeSolid: direction must be non-zero"))?
        .scale(distance);
    // the curve's plane: through its start, spanned by its two farthest-reaching poles
    let far = |from: Vec3, off: Option<Vec3>| {
        points.iter().map(|point| point.sub(from)).map(|arm| off.map_or(arm, |axis| arm.sub(axis.scale(arm.dot(axis))))).fold(Vec3::new(0.0, 0.0, 0.0), |best, arm| if arm.length() > best.length() { arm } else { best })
    };
    let planar = || KernelRefusal::input(collect, "profile_plane", "extrudeSolid: the closed profile curve does not lie in a plane");
    let e1 = far(origin, None).normalized().map_err(|_| planar())?;
    let e2 = far(origin, Some(e1)).normalized().map_err(|_| planar())?;
    let normal = e1.cross(e2);
    if points.iter().any(|point| point.sub(origin).dot(normal).abs() > 1e-9 * size) {
        return Err(planar());
    }
    if slide.dot(normal).abs() <= 1e-12 * slide.length() {
        return Err(KernelRefusal::input(collect, "direction", "extrudeSolid: direction lies in the profile's plane"));
    }
    // the side: the curve's poles and their slid copies, the curve's own weights
    let net = curve.control_points.iter().map(|pole| vec![*pole, Vec4 { x: pole.x + slide.x * pole.w, y: pole.y + slide.y * pole.w, z: pole.z + slide.z * pole.w, w: pole.w }]).collect();
    let side = NurbsSurface::new(curve.degree, 1, curve.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], net).or_refuse(collect, "extrudeClosed.side")?;
    // each end's plane, a rectangle about the curve's poles (which hold the curve)
    let reach = |axis: Vec3| points.iter().map(|point| point.sub(origin).dot(axis)).fold((0.0f64, 0.0f64), |(low, high), at| (low.min(at), high.max(at)));
    let ((low1, high1), (low2, high2)) = (reach(e1), reach(e2));
    let corner = origin.add(e1.scale(low1 - 0.5 * size)).add(e2.scale(low2 - 0.5 * size));
    let end_face = |shift: Vec3, v: f64| -> Result<BrepSolid, KernelRefusal> {
        let plane = make_plane(corner.add(shift), e1, e2, high1 - low1 + size, high2 - low2 + size).or_refuse(collect, "extrudeClosed.plane")?;
        face_from_surface_trimmed(&plane, &[vec![side.iso_curve_v(v).or_refuse(collect, "extrudeClosed.rim")?]])
    };
    let sheets = [face_from_surface(&side)?, end_face(Vec3::new(0.0, 0.0, 0.0), 0.0)?, end_face(slide, 1.0)?];
    // sewing reads each edge's place by projection, whose own error on a cubic is near 1e-8
    solid_from_sheets(&sheets, (1e-8 * size.max(slide.length())).max(1e-7)).map(Some)
}

/// The solid ellipsoid about `center` with semi-axes `semi_axes[0]` along
/// `x_direction`, `semi_axes[1]` square to it toward `y_hint`, and
/// `semi_axes[2]` along their cross product.
///
/// Exact: the unit ball under the affine map that stretches it onto the three
/// semi-axes, which carries the ball's rational poles and keeps their weights.
/// Its one face is closed on a seam, the half ellipse from pole to pole through
/// the first semi-axis, and its poles are the ends of the third.
pub fn make_ellipsoid_brep(center: Vec3, x_direction: Vec3, y_hint: Vec3, semi_axes: [f64; 3]) -> Result<BrepSolid, KernelRefusal> {
    if semi_axes.iter().any(|axis| !(axis.is_finite() && *axis > 0.0)) {
        return Err(KernelRefusal::input(KernelStage::Collect, "ellipsoid_axes", "makeEllipsoid: the three semi-axes must be positive"));
    }
    let (x, y) = frame("makeEllipsoid", x_direction, y_hint).map_err(|message| KernelRefusal::input(KernelStage::Collect, "ellipsoid_frame", message))?;
    let z = x.cross(y);
    let origin = Vec3::new(0.0, 0.0, 0.0);
    let ball = make_sphere_brep_framed(origin, 1.0, Vec3::new(0.0, 0.0, 1.0), Some(Vec3::new(1.0, 0.0, 0.0)))
        .or_refuse(KernelStage::Collect, "makeEllipsoid.ball")?;
    let [a, b, c] = semi_axes;
    let (ex, ey, ez) = (x.scale(a), y.scale(b), z.scale(c));
    let map = AffineTransform {
        elements: [ex.x, ey.x, ez.x, center.x, ex.y, ey.y, ez.y, center.y, ex.z, ey.z, ez.z, center.z, 0.0, 0.0, 0.0, 1.0],
    };
    transform_brep(&ball, map, false).or_refuse(KernelStage::Collect, "makeEllipsoid.stretch")
}

