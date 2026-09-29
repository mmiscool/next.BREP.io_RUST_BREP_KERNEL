use super::*;

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

pub(super) fn distance2(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

/// A STEP POLYLINE (open chain of straight segments) as a kernel curve: a plain
/// line for two points, otherwise a clamped degree-1 NURBS interpolating them.
pub(super) fn polyline_curve(points: &[Vec3]) -> Result<NurbsCurve, String> {
    match points.len() {
        0 | 1 => Err("step_import: POLYLINE needs at least two points".into()),
        2 => make_line(points[0], points[1]),
        n => {
            // Clamped degree-1 knot vector: [0,0,1,2,...,n-1,n-1] (length n+2).
            let mut knots = Vec::with_capacity(n + 2);
            knots.push(0.0);
            for index in 0..n {
                knots.push(index as f64);
            }
            knots.push((n - 1) as f64);
            let control_points = points.iter().map(|p| Vec4::from_point(*p, 1.0)).collect();
            NurbsCurve::new(1, knots, control_points)
        }
    }
}

pub(super) fn surface_scale(surface: &NurbsSurface) -> Result<f64, String> {
    let mut scale = 0.0f64;
    for row in &surface.control_points {
        for control in row {
            scale = scale.max(control.point()?.length());
        }
    }
    Ok(scale.max(1.0))
}

/// The pcurve fit target for every trim on `surface`: the kernel's absolute
/// refinement floor, [`crate::PCURVE_REFINEMENT_TOLERANCE`].
///
/// Until 2026-09-26 this was `min(1e-6·(1 + scale), 0.5·pcurve_consistency)`,
/// a bar relative to the part: 2.0e-3 on a building model, 1.5e-4 on a 172 mm
/// engine block. The 2026-09-18 fidelity census read 5286 imported pcurves as
/// the fitter's own error, 3633 of them UNDER that ask, and showed the ask,
/// not the fitter, was the defect: the same builder asked for 1e-9 improved
/// 312 of the worst 400 by ten times or more. The ask is now the floor every
/// other fitter in the kernel refines to, the builder measures what it
/// reached out of sample and refuses over the ask plus the stations' own
/// standoff (`crate::pcurve::accept_fit`), and a file whose curve is not on
/// its carrier is fitted to its own standoff instead of chased to the sample
/// cap.
///
/// `surface` is taken so a size-dependent floor can be argued from it later;
/// today the floor is absolute, as measured for the kernel's other fitters.
pub(super) fn loop_pcurve_tolerance(surface: &NurbsSurface) -> Result<f64, String> {
    // The REFINEMENT target, restored to the size-derived formula the importer
    // carried until 2026-09-26. For one landing it was the absolute 1e-7
    // floor: a station fitter chasing the feet of a curve that stands 1e-5 to
    // 1e-3 off its carrier down to 1e-7 fits the file's noise — the soundness
    // scan then reads more self-crossing loops on abc_00000011/12/13/23 (4, 4,
    // 4, 7 against 3, 3, 3, 6) and an offset-shell stage indexes past its
    // rail. The 1e-7 floor is the DETECTOR the fit is read against, reported
    // per trim on the import (`builder/loops.rs`), not the target it chases.
    let scale = surface_scale(surface)?;
    Ok((1e-6 * (1.0 + scale))
        .min(0.5 * crate::KernelTolerances::for_scale(scale, 1e-7).pcurve_consistency))
}

/// The bar every derived trim is READ against and reported over: the
/// kernel's absolute refinement floor, plus the stations' own standoff (no
/// pcurve can image a curve closer than it stands off its carrier).
pub(super) fn trim_detector_bar(report: &crate::PcurveFitReport) -> f64 {
    crate::PCURVE_REFINEMENT_TOLERANCE + report.off_surface
}

/// The on-surface seam ruling between two rim seam-vertices of a periodic
/// BAND face (see `stitch_seam_circle_loops`). When the two rim circles ride
/// the SAME seam meridian (constant u) at different v — a toroidal/spherical
/// band — the ruling is the surface's u-iso meridian arc between them; the
/// symmetric case (same v, different u) yields the v-iso arc. Returns `None`
/// when the two vertices do not share an iso line (a genuinely diagonal
/// pairing the caller must reject rather than forge a chord across the face).
pub(super) fn curved_seam_ruling(
    surface: &NurbsSurface,
    p1: Vec3,
    p2: Vec3,
) -> Result<Option<NurbsCurve>, String> {
    let (closed_u, closed_v) = surface.closed_directions()?;
    let proj1 = crate::project_point_to_surface(surface, p1)?;
    let proj2 = crate::project_point_to_surface(surface, p2)?;
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let u_span = u1 - u0;
    let v_span = v1 - v0;
    // Wrapped signed difference on a period (so a 0 vs 2π seam reads as ~0).
    let period_diff = |a: f64, span: f64| -> f64 {
        if span <= 0.0 {
            return a;
        }
        a - (a / span).round() * span
    };
    let scale = surface_scale(surface)?;
    let endpoint_tol = 1e-6 * (1.0 + scale);

    let try_arc = |iso: NurbsCurve, from: f64, to: f64| -> Result<Option<NurbsCurve>, String> {
        let Some(arc) = subarc_between(&iso, from, to)? else {
            return Ok(None);
        };
        let dom = arc.domain()?;
        let start = arc.evaluate(dom[0])?;
        let end = arc.evaluate(dom[1])?;
        if start.sub(p1).length() <= endpoint_tol && end.sub(p2).length() <= endpoint_tol {
            Ok(Some(arc))
        } else {
            Ok(None)
        }
    };

    // Same seam meridian (constant u), ruling varies in v: toroidal band.
    if closed_u && u_span > 0.0 {
        let same_u = period_diff(proj1.u - proj2.u, u_span).abs() <= 1e-3 * u_span;
        let distinct_v = (proj1.v - proj2.v).abs() > 1e-6 * v_span.abs().max(1.0);
        if same_u && distinct_v {
            let iso = surface.iso_curve_u(proj1.u)?;
            if let Some(arc) = try_arc(iso, proj1.v, proj2.v)? {
                return Ok(Some(arc));
            }
        }
    }

    // Same parallel (constant v), ruling varies in u.
    if closed_v && v_span > 0.0 {
        let same_v = period_diff(proj1.v - proj2.v, v_span).abs() <= 1e-3 * v_span;
        let distinct_u = (proj1.u - proj2.u).abs() > 1e-6 * u_span.abs().max(1.0);
        if same_v && distinct_u {
            let iso = surface.iso_curve_v(proj1.v)?;
            if let Some(arc) = try_arc(iso, proj1.u, proj2.u)? {
                return Ok(Some(arc));
            }
        }
    }

    Ok(None)
}

/// Extract the sub-arc of `curve` whose FORWARD domain runs from parameter
/// `from` to `to` (reversing when `from > to`), used to cut a seam meridian
/// down to the band it rules. Returns `None` for a degenerate span.
fn subarc_between(curve: &NurbsCurve, from: f64, to: f64) -> Result<Option<NurbsCurve>, String> {
    let [d0, d1] = curve.domain()?;
    let full = (d1 - d0).abs().max(1.0);
    let lo = from.min(to).clamp(d0, d1);
    let hi = from.max(to).clamp(d0, d1);
    if hi - lo <= 1e-9 * full {
        return Ok(None);
    }
    let mut arc = curve.clone();
    if lo > d0 + 1e-9 * full {
        arc = arc.split(lo)?.1;
    }
    let mid = arc.domain()?;
    if hi < mid[1] - 1e-9 * full {
        arc = arc.split(hi)?.0;
    }
    if from > to {
        arc = arc.reversed()?;
    }
    Ok(Some(arc))
}

/// Build a bounded kernel plane covering the projected span of `samples`, with
/// the placement's normal so the STEP face `same_sense` maps through directly.
pub(super) fn build_plane_over(frame: &Frame, samples: &[Vec3]) -> Result<NurbsSurface, String> {
    let (mut u_lo, mut u_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut v_lo, mut v_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for point in samples {
        let delta = point.sub(frame.origin);
        let u = delta.dot(frame.x);
        let v = delta.dot(frame.y);
        u_lo = u_lo.min(u);
        u_hi = u_hi.max(u);
        v_lo = v_lo.min(v);
        v_hi = v_hi.max(v);
    }
    if !u_lo.is_finite() {
        (u_lo, u_hi, v_lo, v_hi) = (0.0, 1.0, 0.0, 1.0);
    }
    // A plane is affine, so its trimmed mass integration is exact regardless of
    // patch size — an over-generous margin (covering any conic-sample undersizing)
    // costs nothing and guarantees the face's edges project inside the domain.
    let margin = 0.05 * (u_hi - u_lo).max(v_hi - v_lo).max(1.0) + 1e-9;
    let origin = frame
        .origin
        .add(frame.x.scale(u_lo - margin))
        .add(frame.y.scale(v_lo - margin));
    make_plane(
        origin,
        frame.x,
        frame.y,
        (u_hi - u_lo) + 2.0 * margin,
        (v_hi - v_lo) + 2.0 * margin,
    )
}

/// The base point, height, and the base's axial coordinate (relative to the
/// frame origin) needed to cover `samples` along the frame axis, with a small
/// margin so face edges land inside the domain rather than exactly on it.
pub(super) fn axial_extent(frame: &Frame, samples: &[Vec3]) -> (Vec3, f64, f64) {
    let (mut c_min, mut c_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for point in samples {
        let c = point.sub(frame.origin).dot(frame.z);
        c_min = c_min.min(c);
        c_max = c_max.max(c);
    }
    if !c_min.is_finite() {
        (c_min, c_max) = (0.0, 1.0);
    }
    let margin = 1e-4 * (c_max - c_min).max(1.0) + 1e-9;
    let c_start = c_min - margin;
    let height = (c_max - c_min) + 2.0 * margin;
    (frame.origin.add(frame.z.scale(c_start)), height, c_start)
}

/// Model scale from the vertex cloud (bounding-box half-diagonal), used to
/// scale reconciliation bands the way `surface_scale` scales surface bands.
pub(super) fn solid_like_scale(vertices: &[VertexRecord]) -> f64 {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vertex in vertices {
        low.x = low.x.min(vertex.point.x);
        low.y = low.y.min(vertex.point.y);
        low.z = low.z.min(vertex.point.z);
        high.x = high.x.max(vertex.point.x);
        high.y = high.y.max(vertex.point.y);
        high.z = high.z.max(vertex.point.z);
    }
    if !low.x.is_finite() {
        return 0.0;
    }
    0.5 * high.sub(low).length()
}

/// Parameter where an open `edge` crosses the seam of a periodic `surface`
/// direction (u when `along_u`, else v), or None. The projected
/// principal-domain coordinate jumps by ~a whole period at a crossing;
/// bisection pins the crossing parameter. An edge whose ENDPOINT sits on the
/// seam merely touches it — not a crossing.
pub(super) fn seam_crossing_parameter(
    surface: &NurbsSurface,
    edge: &EdgeRecord,
    along_u: bool,
) -> Result<Option<f64>, String> {
    let [d0, d1] = if along_u {
        surface.domain_u()?
    } else {
        surface.domain_v()?
    };
    let span = d1 - d0;
    if span.abs() <= 1e-12 {
        return Ok(None);
    }
    let coordinate = |t: f64| -> Result<f64, String> {
        let point = edge.curve.evaluate(t)?;
        let projection = crate::project_point_to_surface(surface, point)?;
        Ok(if along_u { projection.u } else { projection.v })
    };
    // Distance of a coordinate from the seam locus (the shared d0 == d1
    // boundary of the periodic direction).
    let seam_distance = |c: f64| (c - d0).abs().min((d1 - c).abs());
    const SAMPLES: usize = 32;
    let mut ts = Vec::with_capacity(SAMPLES + 1);
    let mut cs = Vec::with_capacity(SAMPLES + 1);
    for index in 0..=SAMPLES {
        let t = edge.t0 + (edge.t1 - edge.t0) * index as f64 / SAMPLES as f64;
        ts.push(t);
        cs.push(coordinate(t)?);
    }
    // Only split an edge that genuinely cannot be carried by a SINGLE continuous
    // pcurve — one whose projected parameter, unwrapped, spans MORE than a full
    // period (it wraps the seam repeatedly). An edge that kisses the seam, or
    // even straddles it once, unwraps into a run narrower than one period; on a
    // GENERAL (B-spline) carrier the pcurve fitter keeps that run continuous
    // (slightly out of domain) and the periodic evaluator/integrator wrap it, so
    // splitting would only graft a spurious seam edge that mis-trims the face
    // (ABC helmet 00000011/12 side channels wrap their side seam via such
    // straddling edges). Analytic carriers keep the exact split behaviour.
    if surface.analytic().is_none() {
        let mut unwrapped = cs.clone();
        for index in 1..unwrapped.len() {
            let mut delta = unwrapped[index] - unwrapped[index - 1];
            while delta > 0.5 * span.abs() {
                unwrapped[index] -= span.abs();
                delta = unwrapped[index] - unwrapped[index - 1];
            }
            while delta < -0.5 * span.abs() {
                unwrapped[index] += span.abs();
                delta = unwrapped[index] - unwrapped[index - 1];
            }
        }
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for &value in &unwrapped {
            lo = lo.min(value);
            hi = hi.max(value);
        }
        if hi - lo < 0.9 * span.abs() {
            return Ok(None);
        }
    }
    for index in 1..=SAMPLES {
        if (cs[index] - cs[index - 1]).abs() <= 0.5 * span.abs() {
            continue;
        }
        // Both sides of the candidate crossing must REACH away from the
        // seam: an edge that merely overshoots the seam by projection noise
        // (or a micro piece produced by a previous split, whose samples all
        // hug the seam) is a touch, not a crossing — splitting it would
        // recurse on ever-smaller slivers.
        let reach_floor = 1e-4 * span.abs();
        let before_reach = cs[..index]
            .iter()
            .fold(0.0f64, |best, c| best.max(seam_distance(*c)));
        let after_reach = cs[index..]
            .iter()
            .fold(0.0f64, |best, c| best.max(seam_distance(*c)));
        if before_reach < reach_floor || after_reach < reach_floor {
            continue;
        }
        // Bisect the jump interval down to the crossing parameter,
        // re-anchoring the reference coordinate on the no-jump side.
        let (mut lo, mut hi) = (ts[index - 1], ts[index]);
        let mut lo_c = cs[index - 1];
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            let mid_c = coordinate(mid)?;
            if (mid_c - lo_c).abs() > 0.5 * span.abs() {
                hi = mid;
            } else {
                lo = mid;
                lo_c = mid_c;
            }
        }
        let t_split = 0.5 * (lo + hi);
        // NurbsCurve::split needs a strictly interior parameter; a crossing
        // pinned numerically onto an endpoint is a touch.
        if split_parameter_is_strictly_interior(t_split, edge.t0, edge.t1) {
            return Ok(Some(t_split));
        }
    }
    Ok(None)
}

/// Match `NurbsCurve::split`'s absolute knot-identity exclusion at the ends of
/// the parameter domain. Scaling that exclusion only by the domain span makes
/// it vanishingly small on vendor micro-domains and admits a seam crossing that
/// the curve primitive must then reject.
pub(super) fn split_parameter_is_strictly_interior(parameter: f64, start: f64, end: f64) -> bool {
    let margin =
        crate::curve::KNOT_IDENTITY_TOL.max(crate::curve::KNOT_IDENTITY_TOL * (end - start).abs());
    parameter > start + margin && parameter < end - margin
}

/// Size a SURFACE_OF_LINEAR_EXTRUSION carrier to cover the face. The STEP
/// extrusion VECTOR is typically UNIT length with the surface unbounded along
/// it, so — exactly like the cylinder/cone axial sizing — the covered band
/// must come from the face's sampled boundary. Each sample's sweep coordinate
/// is measured against its closest profile point in the direction-orthogonal
/// metric (a curved profile makes a single global projection wrong), then the
/// profile is shifted to the padded minimum and extruded across the padded
/// range so every trim lands inside the surface domain.
pub(super) fn build_linear_extrusion(
    profile: &NurbsCurve,
    direction: Vec3,
    samples: &[Vec3],
) -> Result<NurbsSurface, String> {
    let sweep = direction.normalized()?;
    let [u0, u1] = profile.domain()?;
    let profile_points = (0..=64)
        .map(|index| profile.evaluate(u0 + (u1 - u0) * index as f64 / 64.0))
        .collect::<Result<Vec<_>, String>>()?;
    let (mut c_min, mut c_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for point in samples {
        let mut best_residual = f64::INFINITY;
        let mut coordinate = 0.0;
        for anchor in &profile_points {
            let delta = point.sub(*anchor);
            let along = delta.dot(sweep);
            let residual = delta.sub(sweep.scale(along)).length();
            if residual < best_residual {
                best_residual = residual;
                coordinate = along;
            }
        }
        c_min = c_min.min(coordinate);
        c_max = c_max.max(coordinate);
    }
    if !c_min.is_finite() {
        (c_min, c_max) = (0.0, 1.0);
    }
    // A PERCENT-level pad, not the analytic surfaces' 1e-4: the sweep
    // coordinate of each boundary sample is measured against a SAMPLED
    // closest profile point, and a curved boundary edge can dip below the
    // sampled extent between samples (a pocket-bottom arc's lowest point).
    // The carrier is conceptually unbounded along the sweep, so padding is
    // free; a short carrier CLAMPS projections at the domain edge and every
    // downstream pcurve inherits the error.
    let margin = 0.05 * (c_max - c_min).max(1.0) + 1e-6;
    let start = c_min - margin;
    let height = (c_max - c_min) + 2.0 * margin;
    let shift = sweep.scale(start);
    // Translate the profile homogeneously (weighted control points move by
    // w·shift) so the extrusion's v = 0 row sits at the padded minimum.
    let shifted_controls = profile
        .control_points
        .iter()
        .map(|control| Vec4 {
            x: control.x + control.w * shift.x,
            y: control.y + control.w * shift.y,
            z: control.z + control.w * shift.z,
            w: control.w,
        })
        .collect::<Vec<_>>();
    let shifted = NurbsCurve::new(profile.degree, profile.knots.clone(), shifted_controls)?;
    make_extrusion(&shifted, sweep.scale(height))
}

/// Normalize an imported B-spline surface whose PERIODIC direction is v
/// (v = revolution angle, u = open generatrix/meridian) into the kernel's
/// universal convention where the periodic direction is u.
///
/// Every import-time rim-stitch / seam-ruling path
/// (`relocate_face_rim_seams`, `revolution_seam_ruling`,
/// `stitch_seam_circle_loops`) assumes u = angle and v = generatrix — the
/// layout the kernel's own `make_revolution`/cylinder/cone/sphere builders
/// all emit. A vendor RATIONAL_B_SPLINE_SURFACE for a surface of revolution
/// can instead put the closed angle on v and the open meridian on u (ABC
/// case 00000021's dome caps). Left as-is, its pole-cap loops (a rim wrapping
/// the full angle plus a degenerate VERTEX_LOOP apex) never merge, so the face
/// integrates the wrong side of the trim (the base-side strip, not the polar
/// cap) — collapsing area/volume ~3.5x and self-intersecting the tessellation.
///
/// The remap is a pure re-parameterization: TRANSPOSE (swap u/v) then REVERSE
/// the new v direction. Transpose alone negates the surface normal du×dv; the
/// reversal negates it back, so geometry AND orientation are preserved
/// bit-for-bit. Only which direction is called "u" changes — `same_sense`,
/// the 3D edges, and every downstream projection are untouched (pcurves are
/// derived AFTER, from this surface). Skips carriers already u-periodic,
/// non-periodic, bi-periodic (tori: closed_u true), or recognized analytic.
pub(super) fn u_periodic_normalized(surface: NurbsSurface) -> Result<NurbsSurface, String> {
    let (closed_u, closed_v) = surface.closed_directions()?;
    if closed_u || !closed_v || surface.analytic().is_some() {
        return Ok(surface);
    }
    let pu = surface.degree_u;
    let pv = surface.degree_v;
    let nu = surface.control_points.len();
    let nv = surface.control_points[0].len();
    // New v knots = old u knots reflected end-for-end (a direction reversal):
    // reverse the order and mirror each value through the domain so the
    // sequence stays increasing, clamped, and spans the same interval.
    let (u_lo, u_hi) = (
        surface.knots_u[0],
        surface.knots_u[surface.knots_u.len() - 1],
    );
    let knots_v_new: Vec<f64> = surface
        .knots_u
        .iter()
        .rev()
        .map(|k| u_lo + u_hi - k)
        .collect();
    let knots_u_new = surface.knots_v.clone();
    // new_cp[a][b] = old_cp[nu-1-b][a]: a indexes new u (= old v), b indexes
    // new v (= old u, reversed).
    let control_points: Vec<Vec<Vec4>> = (0..nv)
        .map(|a| {
            (0..nu)
                .map(|b| surface.control_points[nu - 1 - b][a])
                .collect()
        })
        .collect();
    NurbsSurface::new(pv, pu, knots_u_new, knots_v_new, control_points)
}

/// Revolve a profile (generatrix) curve a full 360° about an axis into a
/// periodic NURBS wall — the general form the cylinder/cone/sphere/torus
/// builders above are all special cases of. Like those, a partial face must
/// keep its u = 0 seam OUT of the covered angular range: rotate the whole
/// generatrix about the axis so the seam falls in the gap the face's sampled
/// boundary leaves open. A full-wrap face keeps the profile where the file put
/// it, so its meridian aligns with OCC's doubled seam edge (`seam_meridian`
/// returns the frame's x direction, which the rotation then leaves untouched).
pub(super) fn build_revolution(
    profile: &NurbsCurve,
    axis_point: Vec3,
    axis_dir: Vec3,
    samples: &[Vec3],
) -> Result<NurbsSurface, String> {
    let axis = axis_dir.normalized()?;
    let x = axis.perpendicular()?;
    let y = axis.cross(x).normalized()?;
    let frame = Frame {
        origin: axis_point,
        x,
        y,
        z: axis,
    };
    let seam = seam_meridian(&frame, samples);
    let profile = if seam.sub(frame.x).length() > 1e-12 {
        // A genuine gap: put u = 0 at its middle. `seam_meridian` returned a
        // direction in the axis-perpendicular plane; its azimuth is the target,
        // and the profile's own representative azimuth is where u = 0 sits now.
        let target = seam.dot(frame.y).atan2(seam.dot(frame.x));
        let current = revolution_profile_azimuth(profile, &frame);
        rotate_curve_about_axis(profile, axis_point, axis, target - current)?
    } else {
        profile.clone()
    };
    make_revolution(axis_point, axis, &profile, TAU)
}

/// Representative azimuth of a revolution profile about the frame axis: the
/// angular position of its control point farthest from the axis (the most
/// numerically reliable — near-axis points have an ill-defined azimuth). This
/// is where `make_revolution` puts the u = 0 seam.
fn revolution_profile_azimuth(profile: &NurbsCurve, frame: &Frame) -> f64 {
    let mut best_radius = 0.0;
    let mut best_azimuth = 0.0;
    for control in &profile.control_points {
        let Ok(point) = control.point() else { continue };
        let delta = point.sub(frame.origin);
        let radial = delta.sub(frame.z.scale(delta.dot(frame.z))).length();
        if radial > best_radius {
            if let Some(a) = azimuth(frame, point) {
                best_radius = radial;
                best_azimuth = a;
            }
        }
    }
    best_azimuth
}

/// Rotate a point `angle` radians about the line (`axis_point`, unit `axis`),
/// right-hand rule (Rodrigues' rotation).
fn rotate_point_about_axis(point: Vec3, axis_point: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let v = point.sub(axis_point);
    let (sin, cos) = angle.sin_cos();
    let rotated = v
        .scale(cos)
        .add(axis.cross(v).scale(sin))
        .add(axis.scale(axis.dot(v) * (1.0 - cos)));
    axis_point.add(rotated)
}

/// Rotate every control point of a curve about the axis line, preserving
/// weights (rotation is homogeneous). Used to re-place a revolution seam.
fn rotate_curve_about_axis(
    curve: &NurbsCurve,
    axis_point: Vec3,
    axis: Vec3,
    angle: f64,
) -> Result<NurbsCurve, String> {
    let controls = curve
        .control_points
        .iter()
        .map(|control| {
            let point = control.point()?;
            Ok(Vec4::from_point(
                rotate_point_about_axis(point, axis_point, axis, angle),
                control.w,
            ))
        })
        .collect::<Result<Vec<_>, String>>()?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls)
}

/// Re-place a revolution frame's u = 0 seam so it falls in the angular GAP the
/// face's edges leave uncovered. A partial cylindrical/conical/spherical/toroidal
/// face (e.g. a 262° OCC/FreeCAD patch) then keeps all its pcurves inside [0,1]
/// and never crosses the seam; a full-wrap face (dense coverage, or a seam-only
/// meridian sample) keeps the placement's ref_direction, where its doubled seam
/// edge lives, for the existing seam-pair handling.
pub(super) fn reseam(frame: &Frame, samples: &[Vec3]) -> Frame {
    let seam = seam_meridian(frame, samples);
    let y = frame.z.cross(seam).normalized().unwrap_or(frame.y);
    Frame {
        origin: frame.origin,
        x: seam,
        y,
        z: frame.z,
    }
}

/// Pointed cones carry an explicit VERTEX_LOOP at the axis and one circular
/// rim. Put the periodic seam through that rim edge's STEP vertex. This avoids
/// forcing a shared rim circle to satisfy two different carrier-surface seams
/// (for example where the cone meets a cylindrical counterbore).
pub(super) fn reseam_cone(frame: &Frame, samples: &[Vec3]) -> Frame {
    let has_axis_sample = samples.iter().any(|point| {
        let delta = point.sub(frame.origin);
        delta.sub(frame.z.scale(delta.dot(frame.z))).length() <= 1e-7
    });
    if has_axis_sample {
        if let Some(x) = samples.iter().find_map(|point| {
            let delta = point.sub(frame.origin);
            let radial = delta.sub(frame.z.scale(delta.dot(frame.z)));
            (radial.length() > 1e-7)
                .then(|| radial.normalized().ok())
                .flatten()
        }) {
            let y = frame.z.cross(x).normalized().unwrap_or(frame.y);
            return Frame {
                origin: frame.origin,
                x,
                y,
                z: frame.z,
            };
        }
    }
    reseam(frame, samples)
}

/// Azimuthal angle of a point about the frame axis, or None if it is on the
/// axis (radius ~ 0).
fn azimuth(frame: &Frame, point: Vec3) -> Option<f64> {
    let delta = point.sub(frame.origin);
    let radial = delta.sub(frame.z.scale(delta.dot(frame.z)));
    if radial.length() < 1e-9 {
        return None;
    }
    let mut angle = radial.dot(frame.y).atan2(radial.dot(frame.x));
    if angle < 0.0 {
        angle += TAU;
    }
    Some(angle)
}

/// For a closed great-circle edge through both poles of a sphere, return the
/// center meridian of the hemisphere on the RIGHT of the authored boundary.
/// The face material is on the left, so this is the unique safe place to cut
/// the periodic carrier when the two geometric angular gaps are exactly tied.
pub(super) fn equal_gap_great_circle_seam(
    frame: &Frame,
    radius: f64,
    edge: &EdgeRecord,
    forward: bool,
    face_same_sense: bool,
) -> Result<Option<Vec3>, String> {
    let [t0, t1] = edge.curve.domain()?;
    let tolerance = 1e-5 * radius.max(1e-12);

    // Pick a reliable non-pole station. At a pole the azimuth and hence the
    // hemisphere side are undefined, while any interior great-circle station
    // away from the axis gives a stable tangent/radial frame.
    let mut station = None;
    for index in 1..16 {
        let fraction = index as f64 / 16.0;
        let t = t0 + (t1 - t0) * fraction;
        let (point, derivative) = edge.curve.deriv1(t)?;
        let radial = point.sub(frame.origin);
        let off_axis = radial.sub(frame.z.scale(radial.dot(frame.z))).length();
        if off_axis > 0.25 * radius && derivative.length() > 1e-12 * radius {
            station = Some((radial, derivative));
            break;
        }
    }
    let Some((radial, derivative)) = station else {
        return Ok(None);
    };
    if (radial.length() - radius).abs() > tolerance {
        return Ok(None);
    }
    let tangent = derivative
        .scale(if forward { 1.0 } else { -1.0 })
        .normalized()?;
    let face_normal = radial
        .normalized()?
        .scale(if face_same_sense { 1.0 } else { -1.0 });
    if tangent.dot(face_normal).abs() > 1e-4 {
        return Ok(None);
    }
    let excluded = tangent.cross(face_normal).normalized()?;

    // Prove the exact signature rather than inferring it from a closed edge:
    // every station lies on the sphere and in one center-crossing plane, that
    // plane contains the sphere axis (so the edge visits both poles), and the
    // curve makes one ordinary circumference rather than a partial/multi-cover
    // traversal. `excluded` is the oriented plane normal.
    if excluded.dot(frame.z).abs() > 1e-4 {
        return Ok(None);
    }
    let mut axial_lo = f64::INFINITY;
    let mut axial_hi = f64::NEG_INFINITY;
    let mut length = 0.0;
    let mut previous = edge.curve.evaluate(t0)?;
    const SAMPLES: usize = 128;
    for index in 0..=SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / SAMPLES as f64;
        let point = edge.curve.evaluate(t)?;
        let r = point.sub(frame.origin);
        if (r.length() - radius).abs() > tolerance || r.dot(excluded).abs() > tolerance {
            return Ok(None);
        }
        let axial = r.dot(frame.z);
        axial_lo = axial_lo.min(axial);
        axial_hi = axial_hi.max(axial);
        if index > 0 {
            length += point.sub(previous).length();
        }
        previous = point;
    }
    if axial_lo > -0.98 * radius
        || axial_hi < 0.98 * radius
        || length < 5.5 * radius
        || length > 7.0 * radius
    {
        return Ok(None);
    }
    Ok(Some(excluded))
}

/// Resolve the other spherical equal-gap ambiguity: an outer loop containing
/// an edge that crosses a carrier pole in its INTERIOR. Longitude on the pole
/// is undefined, so the two opposite-meridian limits admit two half-period UV
/// connections, representing complementary spherical regions. Put the carrier
/// seam in the authored RIGHT/excluded side so material-on-the-left becomes a
/// single ordinary in-domain trim for every downstream region consumer.
pub(super) fn interior_pole_loop_seam(
    frame: &Frame,
    radius: f64,
    edges: &[(&EdgeRecord, bool)],
    face_same_sense: bool,
) -> Result<Option<Vec3>, String> {
    if edges.len() < 2 {
        return Ok(None);
    }
    const SAMPLES: usize = 256;
    let sphere_tolerance = 1e-4 * radius.max(1e-12);
    let pole_band = 1e-2 * radius;
    let mut has_bracketed_crossing = false;
    let mut angles = Vec::new();
    let mut excluded_sum = Vec3::default();

    for &(edge, forward) in edges {
        if edge.degenerate {
            continue;
        }
        let mut points = Vec::with_capacity(SAMPLES + 1);
        for index in 0..=SAMPLES {
            let fraction = index as f64 / SAMPLES as f64;
            let fraction = if forward { fraction } else { 1.0 - fraction };
            let t = edge.t0 + (edge.t1 - edge.t0) * fraction;
            let point = edge.curve.evaluate(t)?;
            let radial = point.sub(frame.origin);
            if (radial.length() - radius).abs() > sphere_tolerance {
                return Ok(None);
            }
            let equatorial = radial.sub(frame.z.scale(radial.dot(frame.z)));
            if equatorial.length() > pole_band {
                angles.push(
                    equatorial
                        .dot(frame.y)
                        .atan2(equatorial.dot(frame.x))
                        .rem_euclid(TAU),
                );
            }
            points.push(point);
        }

        let nonsingular = points
            .iter()
            .map(|point| {
                let radial = point.sub(frame.origin);
                radial.sub(frame.z.scale(radial.dot(frame.z))).length() > pole_band
            })
            .collect::<Vec<_>>();
        if let (Some(first), Some(last)) = (
            nonsingular.iter().position(|value| *value),
            nonsingular.iter().rposition(|value| *value),
        ) {
            if let Some(singular) = (first + 1..last).find(|index| !nonsingular[*index]) {
                let before = (first..singular)
                    .rev()
                    .find(|index| nonsingular[*index]);
                let after = (singular + 1..=last).find(|index| nonsingular[*index]);
                if let (Some(before), Some(after)) = (before, after) {
                    let equatorial = |point: Vec3| {
                        let radial = point.sub(frame.origin);
                        radial.sub(frame.z.scale(radial.dot(frame.z))).normalized()
                    };
                    if equatorial(points[before])?.dot(equatorial(points[after])?) < -0.98 {
                        has_bracketed_crossing = true;
                    }
                }
            }
        }

        // Discrete line integral of the authored right-side direction. Segment
        // length is retained as the weight; pole-adjacent segments naturally
        // contribute little and never require an undefined pole normal.
        for pair in points.windows(2) {
            let chord = pair[1].sub(pair[0]);
            let midpoint_radial = pair[0].add(pair[1]).scale(0.5).sub(frame.origin);
            if chord.length() <= 1e-14 * radius || midpoint_radial.length() <= pole_band {
                continue;
            }
            let face_normal = midpoint_radial
                .normalized()?
                .scale(if face_same_sense { 1.0 } else { -1.0 });
            excluded_sum = excluded_sum.add(chord.cross(face_normal));
        }
    }
    if !has_bracketed_crossing || angles.len() < 4 {
        return Ok(None);
    }

    // Prove the geometry-only seam choice was ambiguous: the two largest empty
    // azimuth gaps are nearly tied. A clearly unique gap retains `reseam`.
    angles.sort_by(|a, b| a.total_cmp(b));
    let mut gaps = Vec::with_capacity(angles.len());
    for index in 0..angles.len() {
        let start = angles[index];
        let end = if index + 1 < angles.len() {
            angles[index + 1]
        } else {
            angles[0] + TAU
        };
        gaps.push((end - start, start, end));
    }
    gaps.sort_by(|a, b| b.0.total_cmp(&a.0));
    if gaps.len() < 2 || (gaps[0].0 - gaps[1].0).abs() > 0.2 {
        return Ok(None);
    }

    let equatorial = excluded_sum.sub(frame.z.scale(excluded_sum.dot(frame.z)));
    if equatorial.length() <= 1e-6 * radius {
        return Ok(None);
    }
    let excluded = equatorial.normalized()?;
    let excluded_angle = excluded
        .dot(frame.y)
        .atan2(excluded.dot(frame.x))
        .rem_euclid(TAU);
    // The candidate must lie safely inside one of the tied empty gaps; if the
    // oriented integral points through the boundary, there is no proven safe
    // canonical seam and the historical reseam remains in force.
    let chosen_gap = gaps.iter().take(2).find(|&&(gap, start, end)| {
        let angle = if excluded_angle < start {
            excluded_angle + TAU
        } else {
            excluded_angle
        };
        angle > start + 0.05 && angle < end - 0.05 && gap > 0.1
    });
    let Some(&(_, start, end)) = chosen_gap else {
        return Ok(None);
    };
    let seam_angle = (0.5 * (start + end)).rem_euclid(TAU);
    let seam = frame
        .x
        .scale(seam_angle.cos())
        .add(frame.y.scale(seam_angle.sin()));
    if std::env::var("BREP_DEBUG_STEP_LOOP").is_ok() {
        eprintln!(
            "sphere interior-pole seam: tied gaps {:.4}/{:.4}, excluded angle {:.4}",
            gaps[0].0, gaps[1].0, seam_angle
        );
    }
    Ok(Some(seam))
}

/// The u = 0 radial direction. For a partial-coverage face (a genuine azimuthal
/// gap, roughly 30°–330°) the seam goes in the middle of the gap so no edge
/// crosses it; for a full wrap it keeps the placement ref_direction (where
/// OCC/FreeCAD put the rim circles' seam vertices).
fn seam_meridian(frame: &Frame, samples: &[Vec3]) -> Vec3 {
    let mut angles: Vec<f64> = samples.iter().filter_map(|p| azimuth(frame, *p)).collect();
    if angles.len() >= 2 {
        angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut best_gap = 0.0;
        let mut best_mid = 0.0;
        for index in 0..angles.len() {
            let start = angles[index];
            let end = if index + 1 < angles.len() {
                angles[index + 1]
            } else {
                angles[0] + TAU
            };
            let gap = end - start;
            if gap > best_gap {
                best_gap = gap;
                best_mid = start + gap / 2.0;
            }
        }
        if best_gap > 0.52 && best_gap < TAU - 0.52 {
            return frame
                .x
                .scale(best_mid.cos())
                .add(frame.y.scale(best_mid.sin()));
        }
    }
    frame.x
}

/// Full-revolution analytic surfaces built with u = 0 on the frame's x meridian
/// (the STEP ref_direction), so the surface seam aligns with the STEP seam edge.
pub(super) fn build_cylinder(
    frame: &Frame,
    radius: f64,
    base: Vec3,
    height: f64,
) -> Result<NurbsSurface, String> {
    let start = base.add(frame.x.scale(radius));
    let generatrix = make_line(start, start.add(frame.z.scale(height)))?;
    make_revolution(base, frame.z, &generatrix, TAU)
}

pub(super) fn build_cone(
    frame: &Frame,
    base: Vec3,
    radius_bottom: f64,
    radius_top: f64,
    height: f64,
) -> Result<NurbsSurface, String> {
    let start = base.add(frame.x.scale(radius_bottom));
    let end = base
        .add(frame.z.scale(height))
        .add(frame.x.scale(radius_top));
    let generatrix = make_line(start, end)?;
    make_revolution(base, frame.z, &generatrix, TAU)
}

pub(super) fn build_sphere(frame: &Frame, radius: f64) -> Result<NurbsSurface, String> {
    let meridian = make_arc(
        frame.origin,
        frame.x,
        frame.z,
        radius,
        -std::f64::consts::FRAC_PI_2,
        std::f64::consts::FRAC_PI_2,
    )?;
    make_revolution(frame.origin, frame.z, &meridian, TAU)
}

pub(super) fn build_torus(frame: &Frame, major: f64, minor: f64) -> Result<NurbsSurface, String> {
    let tube_center = frame.origin.add(frame.x.scale(major));
    // A horn (major == minor) or degenerate (major < minor) torus has its tube
    // touching or crossing the axis of revolution at the tube's innermost point
    // (tube angle PI): the inner "equator" collapses to a single point on the
    // axis — a degenerate pole, exactly like a sphere's pole. The classic
    // parameterisation starts the tube at the OUTER equator (angle 0), which for
    // a horn puts that collapsed pole at the interior tube parameter v=0.5 and
    // the outer equator on the v=0 seam. Both are fatal: the interior pinch makes
    // point projection near the outer-equator seam snap to v=0 (an ambiguous
    // branch), which collapses the fitted pcurve of any tube-spanning edge and
    // tears the face loop open ("uv gap ... is not a collapsed pole").
    //
    // Start the tube at the innermost point instead, so the collapse lands on the
    // v-domain boundary (v=0 and v=1, both the same axis point — a sphere-style
    // pole the loop-closer already knows how to synthesise) and the smooth outer
    // equator moves to the well-conditioned interior at v=0.5. A proper ring
    // torus (major > minor) never reaches the axis, so keep its seam at the outer
    // equator as before.
    let start_angle = if major <= minor * (1.0 + 1e-6) {
        std::f64::consts::PI
    } else {
        0.0
    };
    let tube = make_arc(
        tube_center,
        frame.x,
        frame.z,
        minor,
        start_angle,
        start_angle + TAU,
    )?;
    make_revolution(frame.origin, frame.z, &tube, TAU)
}

/// How far a STEP conic edge's VERTEX_POINT may sit off the conic the edge is
/// trimmed on before the edge is refused as mismatched.
///
/// The reconstructed endpoint is the vertex PROJECTED onto the conic, so the
/// miss measured against it is purely the vertex's distance off the ideal
/// curve. Two regimes are telling it apart: a wrong radius, placement or sense
/// puts the vertex O(semi-axis) off, hence the relative term; vendor vertex
/// noise is an ABSOLUTE export-precision quantity that does not shrink with the
/// arc, hence the floor.
///
/// The floor is [`crate::VERTEX_MATCH_FLOOR`] — the band under which
/// [`crate::BrepSolid::validate`] calls a curve end and its vertex the SAME
/// point — because that is the miss the kernel accepts as-is for every other
/// curve type (`heal_imported_edge_endpoints` leaves misses inside it
/// untouched). Refusing a conic tighter than that refuses geometry the kernel
/// already validates. It used to be floored at `WELD_FLOOR` (1e-5, the boolean
/// assembler's weld radius — a different identity question), which left a
/// small arc with only its relative term: on `raspbery_pi_3.step` body #171923
/// (`MOLEX_672983090` 'Imported3', face #157815) two r = 0.15 mm fillet arcs
/// (#78939, #229506) meet a shared-corner vertex that SolidWorks placed 4.13e-4
/// mm radially off the circle, above the 1.5e-4 mm relative term and a whole
/// body was refused for a 0.4 µm miss.
///
/// A conic miss ABOVE this band is still refused: degree-2 curves are
/// deliberately excluded from `heal_imported_edge_endpoints` (translating a
/// rational conic's control points would deform the exact circle), so there is
/// no heal lane to hand it to.
fn conic_vertex_band(semi_axis: f64) -> f64 {
    (semi_axis * 1e-3).max(crate::VERTEX_MATCH_FLOOR)
}

/// Build a circle (`a == b`) or ellipse arc on `frame`, oriented so
/// `evaluate(t0) == p_start` and `evaluate(t1) == p_end`. The STEP conic is
/// parameterised CCW in the placement plane; `same_sense` picks the direction.
/// Only an EDGE_CURVE which repeats the same STEP VERTEX_POINT is `closed`.
pub(super) fn build_conic_edge(
    frame: &Frame,
    a: f64,
    b: f64,
    p_start: Vec3,
    p_end: Vec3,
    same_sense: bool,
    closed: bool,
) -> Result<NurbsCurve, String> {
    let angle = |p: Vec3| {
        let delta = p.sub(frame.origin);
        (delta.dot(frame.y) / b).atan2(delta.dot(frame.x) / a)
    };
    let theta_start = angle(p_start);
    let theta_end = angle(p_end);
    let tau = std::f64::consts::TAU;

    let curve = if closed {
        let arc = build_ellipse_arc(frame, a, b, theta_start, theta_start + tau)?;
        if same_sense {
            arc
        } else {
            arc.reversed()?
        }
    } else if same_sense {
        let mut end = theta_end;
        while end < theta_start {
            end += tau;
        }
        build_ellipse_arc(frame, a, b, theta_start, end)?
    } else {
        let mut start = theta_start;
        while start < theta_end {
            start += tau;
        }
        build_ellipse_arc(frame, a, b, theta_end, start)?.reversed()?
    };

    // Verify the trimmed conic actually connects the edge vertices. Note the
    // reconstructed arc endpoints are the vertices PROJECTED onto the conic:
    // both branches evaluate to the on-conic point at `angle(p_start)` /
    // `angle(p_end)`, and the wrap direction only changes which way the arc
    // travels BETWEEN them, never where the endpoints land. So this deviation is
    // purely how far each vertex sits off the ideal conic — genuine CAD
    // modelling noise (the source system placed the vertex, an intersection with
    // a neighbouring edge, a fraction of a micron off the nominal circle), not a
    // parse error, and OpenCASCADE imports the same edges without complaint.
    //
    // The band is [`conic_vertex_band`]: a wrong radius or placement puts a
    // vertex O(radius) off — orders of magnitude above it — so a genuinely
    // mismatched conic is still rejected, while vertex noise inside the kernel's
    // own "curve end and vertex are the same point" band is accepted exactly as
    // it is for every other curve type. The band only GATES: the returned arc is
    // the same exact conic either way, trimmed at the projected vertices.
    let [t0, t1] = curve.domain()?;
    let tolerance = conic_vertex_band(a.max(b));
    if curve.evaluate(t0)?.sub(p_start).length() > tolerance
        || curve.evaluate(t1)?.sub(p_end).length() > tolerance
    {
        return Err("step_import: conic edge endpoints do not match its vertices".into());
    }
    Ok(curve)
}

/// Build the bounded positive branch of a STEP HYPERBOLA. ISO 10303 places the
/// curve at `origin + a*cosh(t)*x + b*sinh(t)*y`; unlike an ellipse it is not
/// periodic, so `same_sense` must agree with the endpoint parameter ordering.
pub(super) fn build_hyperbola_edge(
    frame: &Frame,
    a: f64,
    b: f64,
    p_start: Vec3,
    p_end: Vec3,
    same_sense: bool,
    closed: bool,
) -> Result<NurbsCurve, String> {
    if closed {
        return Err("step_import: HYPERBOLA edge cannot be closed".into());
    }
    if !a.is_finite() || a <= 0.0 || !b.is_finite() || b <= 0.0 {
        return Err("step_import: HYPERBOLA semi-axes must be positive".into());
    }
    let t_start = (p_start.sub(frame.origin).dot(frame.y) / b).asinh();
    let t_end = (p_end.sub(frame.origin).dot(frame.y) / b).asinh();
    build_open_conic_edge(
        "HYPERBOLA",
        t_start,
        t_end,
        same_sense,
        conic_vertex_band(a.max(b)),
        p_start,
        p_end,
        |lo, hi| make_hyperbola(frame.origin, frame.x, frame.y, a, b, lo, hi),
    )
}

/// Build a bounded STEP PARABOLA, parameterized in its placement frame as
/// `origin + focal*t^2*x + 2*focal*t*y`.
pub(super) fn build_parabola_edge(
    frame: &Frame,
    focal: f64,
    p_start: Vec3,
    p_end: Vec3,
    same_sense: bool,
    closed: bool,
) -> Result<NurbsCurve, String> {
    if closed {
        return Err("step_import: PARABOLA edge cannot be closed".into());
    }
    if !focal.is_finite() || focal <= 0.0 {
        return Err("step_import: PARABOLA focal distance must be positive".into());
    }
    let t_start = p_start.sub(frame.origin).dot(frame.y) / (2.0 * focal);
    let t_end = p_end.sub(frame.origin).dot(frame.y) / (2.0 * focal);
    build_open_conic_edge(
        "PARABOLA",
        t_start,
        t_end,
        same_sense,
        conic_vertex_band(focal),
        p_start,
        p_end,
        |lo, hi| make_parabola(frame.origin, frame.x, frame.y, focal, lo, hi),
    )
}

fn build_open_conic_edge(
    label: &str,
    t_start: f64,
    t_end: f64,
    same_sense: bool,
    tolerance: f64,
    p_start: Vec3,
    p_end: Vec3,
    build: impl FnOnce(f64, f64) -> Result<NurbsCurve, String>,
) -> Result<NurbsCurve, String> {
    let span = t_end - t_start;
    let parameter_tol = 1e-12 * (1.0 + t_start.abs().max(t_end.abs()));
    if (same_sense && span < -parameter_tol) || (!same_sense && span > parameter_tol) {
        return Err(format!(
            "step_import: {label} edge orientation disagrees with same_sense"
        ));
    }
    if span.abs() <= parameter_tol {
        return Err(format!("step_import: {label} edge has coincident parameters"));
    }
    let curve = if same_sense {
        build(t_start, t_end)?
    } else {
        build(t_end, t_start)?.reversed()?
    };
    // Parameter recovery from one local coordinate also succeeds for points
    // off the carrier, so compare the exact reconstructed endpoints.
    let [u0, u1] = curve.domain()?;
    if curve.evaluate(u0)?.sub(p_start).length() > tolerance
        || curve.evaluate(u1)?.sub(p_end).length() > tolerance
    {
        return Err(format!(
            "step_import: {label} edge endpoints do not match its vertices"
        ));
    }
    Ok(curve)
}

/// A circle arc (`a == b`) directly, or an ellipse arc as the affine image of a
/// unit circle arc (an affine map preserves the exact rational-quadratic form).
pub(super) fn build_ellipse_arc(
    frame: &Frame,
    a: f64,
    b: f64,
    theta_start: f64,
    theta_end: f64,
) -> Result<NurbsCurve, String> {
    if (a - b).abs() <= 1e-12 * a.max(1.0) {
        return make_arc(frame.origin, frame.x, frame.y, a, theta_start, theta_end);
    }
    let unit = make_arc(frame.origin, frame.x, frame.y, 1.0, theta_start, theta_end)?;
    let control_points = unit
        .control_points
        .iter()
        .map(|control| {
            let point = control.point()?;
            let delta = point.sub(frame.origin);
            let mapped = frame
                .origin
                .add(frame.x.scale(a * delta.dot(frame.x)))
                .add(frame.y.scale(b * delta.dot(frame.y)))
                .add(frame.z.scale(delta.dot(frame.z)));
            Ok(Vec4::from_point(mapped, control.w))
        })
        .collect::<Result<Vec<_>, String>>()?;
    NurbsCurve::new(unit.degree, unit.knots.clone(), control_points)
}

/// (u_min, u_max, v_min, v_max) sampled across a pcurve's domain.
pub(super) fn pcurve_extent(pcurve: &NurbsCurve) -> Result<(f64, f64, f64, f64), String> {
    let [t0, t1] = pcurve.domain()?;
    let (mut u_lo, mut u_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut v_lo, mut v_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for index in 0..=8 {
        let point = pcurve.evaluate(t0 + (t1 - t0) * index as f64 / 8.0)?;
        u_lo = u_lo.min(point.x);
        u_hi = u_hi.max(point.x);
        v_lo = v_lo.min(point.y);
        v_hi = v_hi.max(point.y);
    }
    Ok((u_lo, u_hi, v_lo, v_hi))
}

pub(super) fn pcurve_tracks_edge(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    edge: &NurbsCurve,
    edge_start: f64,
    edge_end: f64,
    forward: bool,
    tolerance: f64,
) -> Result<bool, String> {
    let [p0, p1] = pcurve.domain()?;
    for index in 0..=64 {
        let fraction = index as f64 / 64.0;
        let uv = pcurve.evaluate(p0 + (p1 - p0) * fraction)?;
        let represented = surface.evaluate(uv.x, uv.y)?;
        let edge_fraction = if forward { fraction } else { 1.0 - fraction };
        let point = edge.evaluate(edge_start + (edge_end - edge_start) * edge_fraction)?;
        if represented.sub(point).length() > tolerance {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Snap a seam pcurve's constant coordinate onto the exact domain boundary.
pub(super) fn pin_pcurve(pcurve: &mut NurbsCurve, pin_u: bool, boundary: f64) -> Result<(), String> {
    let mut control_points = pcurve.control_points.clone();
    for control in &mut control_points {
        if pin_u {
            control.x = boundary * control.w;
        } else {
            control.y = boundary * control.w;
        }
    }
    *pcurve = NurbsCurve::new(pcurve.degree, pcurve.knots.clone(), control_points)?;
    Ok(())
}

/// Move a clamped curve's first/last control points so it interpolates `start`
/// and `end` exactly, but ONLY for an endpoint that currently misses its target
/// by more than `tolerance`. Leaving already-coincident endpoints untouched
/// keeps every import whose ruling already lands on its vertices byte-identical;
/// only a ruling that strayed a fit-tolerance off (near a pole) is corrected.
/// Control points are homogeneous, so scale the target by the weight.
pub(super) fn snap_curve_endpoints(
    curve: &NurbsCurve,
    start: Vec3,
    end: Vec3,
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    let [t0, t1] = curve.domain()?;
    let mut control_points = curve.control_points.clone();
    if curve.evaluate(t0)?.sub(start).length() > tolerance {
        if let Some(first) = control_points.first_mut() {
            let w = first.w;
            *first = Vec4 {
                x: start.x * w,
                y: start.y * w,
                z: start.z * w,
                w,
            };
        }
    }
    if curve.evaluate(t1)?.sub(end).length() > tolerance {
        if let Some(last) = control_points.last_mut() {
            let w = last.w;
            *last = Vec4 {
                x: end.x * w,
                y: end.y * w,
                z: end.z * w,
                w,
            };
        }
    }
    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
}

/// Vendor endpoint-imprecision ceiling (mm) for
/// [`heal_imported_edge_endpoints`]: a curve end this far from the vertex the
/// STEP file says it meets is still the same corner saved from two
/// approximations; beyond it the disagreement is real broken geometry and must
/// be refused rather than snapped shut.
///
/// 100 microns. Vendor endpoint misses observed on the ABC corpus top out
/// around 22 microns (abc_00000218 vertex #11093: two incident B-splines stop
/// 9.7 and 22.2 microns short of it on opposite sides), so this leaves several
/// times that headroom while staying orders below a genuinely disconnected
/// edge.
const VENDOR_ENDPOINT_HEAL_CEILING: f64 = 1e-1;

/// Pull an imported edge's 3D curve endpoints ONTO the topological vertices the
/// STEP file says they meet at.
///
/// Vendor exporters commit a VERTEX_POINT and its incident edges' 3D curves as
/// INDEPENDENT approximations of the same corner, so the curve ends stop short
/// of (or overshoot) the vertex by an export-precision amount. This kernel keeps
/// exact 3D edges and exact vertices — the same policy `reconcile_edges_onto_surfaces`
/// states ("vendor vertices are accurate; the drift is in the curve") — so the
/// curve is the representation that moves. Leaving the miss in place both fails
/// `BrepSolid::validate`'s edge-endpoint-vs-vertex check and tears the face's
/// parameter loop open: two neighbouring coedges that meet at ONE vertex project
/// to two different uv points, and the loop closer sees a gap it cannot honestly
/// bridge (abc_00000218 face #929: `uv gap (0.895, 0.0)->(0.933, 0.0)`, a 32
/// micron 3D jump straight out of the two curves' 9.7/22.2 micron vertex misses).
///
/// Moving only the endpoint control point can rotate the tangent dramatically
/// when its first span is short. Translate the endpoint AND its adjacent
/// control point by the same delta instead: endpoint interpolation becomes
/// exact while the endpoint tangent vector is preserved byte-for-byte. Both
/// control points influence only the curve's FIRST (resp. last) knot span, so a
/// multi-span curve keeps its interior geometry untouched.
///
/// A DEGREE-1 curve is the exception: its control points are its own polyline
/// vertices, so only the endpoint one moves — re-aiming the end segment,
/// exactly how a `LINE` edge is built in the first place (`resolve_edge_curve`
/// makes it the chord between the two STEP vertices). Vendors do write straight
/// edges as degree-1 B-splines rather than `LINE` (abc_00000023 `#39007`), and
/// those miss their vertices like any other spline.
///
/// This is deliberately not a general snap:
/// * degree 2 is left alone — a rational conic is an exact circle/ellipse that
///   a control-point translation would deform;
/// * a miss beyond [`VENDOR_ENDPOINT_HEAL_CEILING`], or past the curve's own
///   half-chord (which would fold the end past the middle), is real broken
///   geometry and is refused, so the honest import error still surfaces;
/// * a miss already inside [`crate::VERTEX_MATCH_FLOOR`] — the absolute band
///   under which the validator calls the curve end and the vertex the SAME
///   point — is left untouched, so ordinary imports keep their vendor curves
///   byte-for-byte. (On a part large enough for the validator's size-coupled
///   term to exceed that floor, a miss between the two is healed rather than
///   merely tolerated: the exact meeting is strictly the better model, and the
///   move is bounded by the vendor ceiling either way.)
pub(super) fn heal_imported_edge_endpoints(
    curve: &NurbsCurve,
    start: Vec3,
    end: Vec3,
) -> Result<NurbsCurve, String> {
    // Opt-out for A/B bisection of a STEP-import regression against this heal
    // (the same lever `BREP_NO_PCURVE_REPAIR` gives the branch-jump repair).
    let polyline = curve.degree == 1;
    if curve.degree == 2
        || curve.degree == 0
        || curve.control_points.len() < curve.degree + 1
        || std::env::var("BREP_STEP_NO_ENDPOINT_HEAL").is_ok()
    {
        return Ok(curve.clone());
    }
    let [t0, t1] = curve.domain()?;
    let curve_start = curve.evaluate(t0)?;
    let curve_end = curve.evaluate(t1)?;
    let chord = curve_end.sub(curve_start).length();
    if !(chord > 0.0) {
        return Ok(curve.clone());
    }
    let start_delta = start.sub(curve_start);
    let end_delta = end.sub(curve_end);
    let should_heal = |gap: f64| {
        gap > crate::VERTEX_MATCH_FLOOR
            && gap <= VENDOR_ENDPOINT_HEAL_CEILING
            && gap <= 0.5 * chord
    };
    let heal_start = should_heal(start_delta.length());
    let heal_end = should_heal(end_delta.length());
    if !(heal_start || heal_end) {
        return Ok(curve.clone());
    }
    let mut control_points = curve.control_points.clone();
    let translate = |control: &mut Vec4, delta: Vec3| {
        control.x += delta.x * control.w;
        control.y += delta.y * control.w;
        control.z += delta.z * control.w;
    };
    if heal_start {
        translate(&mut control_points[0], start_delta);
        if !polyline {
            translate(&mut control_points[1], start_delta);
        }
    }
    if heal_end {
        let last = control_points.len() - 1;
        translate(&mut control_points[last], end_delta);
        if !polyline {
            translate(&mut control_points[last - 1], end_delta);
        }
    }
    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
}

/// Translate a pcurve by (du, dv) in parameter space (whole-period shift on a
/// periodic surface). Control points are homogeneous, so scale by the weight.
pub(super) fn shift_pcurve(pcurve: &NurbsCurve, du: f64, dv: f64) -> Result<NurbsCurve, String> {
    let control_points = pcurve
        .control_points
        .iter()
        .map(|c| Vec4 {
            x: c.x + du * c.w,
            y: c.y + dv * c.w,
            z: c.z,
            w: c.w,
        })
        .collect();
    NurbsCurve::new(pcurve.degree, pcurve.knots.clone(), control_points)
}

/// Whether every control point of a pcurve lies inside the surface domain (with
/// a small relative slack so a boundary-touching seam ruling still qualifies).
pub(super) fn pcurve_in_domain(pcurve: &NurbsCurve, u0: f64, u1: f64, v0: f64, v1: f64) -> bool {
    let eu = 1e-3 * (u1 - u0).abs().max(1.0);
    let ev = 1e-3 * (v1 - v0).abs().max(1.0);
    pcurve.control_points.iter().all(|c| {
        if c.w.abs() <= 1e-12 {
            return false;
        }
        let u = c.x / c.w;
        let v = c.y / c.w;
        u >= u0 - eu && u <= u1 + eu && v >= v0 - ev && v <= v1 + ev
    })
}

/// Signed sense (in surface u) of `curve` as its parameter runs `t0`→`t1`,
/// measured by the change in projected u across two interior samples with the
/// periodic wrap removed. Positive means the curve advances with increasing u
/// (the direction of `surface.iso_curve_v`); negative means it runs against it.
pub(super) fn original_u_direction(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    u0: f64,
    u1: f64,
) -> Result<f64, String> {
    let span = u1 - u0;
    let sample_u = |fraction: f64| -> Result<f64, String> {
        let point = curve.evaluate(t0 + (t1 - t0) * fraction)?;
        Ok(crate::project_point_to_surface(surface, point)?.u)
    };
    let u_a = sample_u(0.25)?;
    let u_b = sample_u(0.35)?;
    let mut delta = u_b - u_a;
    if span > 1e-12 {
        if delta > span * 0.5 {
            delta -= span;
        } else if delta < -span * 0.5 {
            delta += span;
        }
    }
    Ok(delta)
}

/// Genus reproducing `validate`'s reduced-complex Euler accounting:
/// genus = shells − (V − E + F − H)/2 over the non-degenerate 1-skeleton.
pub(super) fn euler_genus(solid: &BrepSolid) -> i64 {
    use std::collections::HashSet;
    let non_degenerate_vertices: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate)
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let vertex_count = solid
        .vertices
        .iter()
        .filter(|vertex| non_degenerate_vertices.contains(&vertex.id))
        .count() as i64;
    let edge_count = solid.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    let face_count: i64 = solid
        .shells
        .iter()
        .map(|shell| shell.faces.len() as i64)
        .sum();
    let hole_count = solid.bounding_hole_count();
    let shell_count = solid.shells.len() as i64;
    // Credit OCC coincident parallel edge pairs exactly like validate() does,
    // so the derived genus matches the Euler check.
    let characteristic = vertex_count - edge_count + face_count - hole_count
        + solid.coincident_parallel_edge_pairs() as i64;
    if std::env::var("BREP_DEBUG_STEP_LOOP").is_ok() {
        let mut uses = std::collections::HashMap::<u64, usize>::new();
        for coedge in solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
        {
            *uses.entry(coedge.edge_id).or_default() += 1;
        }
        let odd: Vec<u64> = solid
            .edges
            .iter()
            .filter(|edge| !edge.degenerate && uses.get(&edge.id).copied().unwrap_or(0) != 2)
            .map(|edge| edge.id)
            .collect();
        eprintln!(
            "EULER V={vertex_count} E={edge_count} F={face_count} H={hole_count} chi={characteristic} shells={shell_count} non-two-use={odd:?}"
        );
        for (index, first) in solid.vertices.iter().enumerate() {
            for second in solid.vertices.iter().skip(index + 1) {
                let gap = first.point.sub(second.point).length();
                if gap < 1e-3 {
                    eprintln!(
                        "  NEAR-DUP vertices {} and {} gap {:.9} at ({:.4},{:.4},{:.4})",
                        first.id, second.id, gap, first.point.x, first.point.y, first.point.z
                    );
                }
            }
        }
        let mut directions = std::collections::HashMap::<u64, Vec<bool>>::new();
        for coedge in solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
        {
            directions
                .entry(coedge.edge_id)
                .or_default()
                .push(coedge.forward);
        }
        for (edge_id, uses) in &directions {
            if uses.len() == 2 && uses[0] == uses[1] {
                eprintln!("  SAME-DIRECTION uses on edge {edge_id}: {uses:?}");
            }
        }
        // Edges visited twice within the SAME loop (slits/seams) vs across loops.
        for shell in &solid.shells {
            for face in &shell.faces {
                for loop_record in &face.loops {
                    let mut counts = std::collections::HashMap::<u64, usize>::new();
                    for coedge in &loop_record.coedges {
                        *counts.entry(coedge.edge_id).or_default() += 1;
                    }
                    for (edge_id, count) in counts {
                        if count >= 2 {
                            eprintln!(
                                "  DOUBLED edge {edge_id} x{count} within loop {} of face {}",
                                loop_record.id, face.id
                            );
                        }
                    }
                }
            }
        }
    }
    shell_count - characteristic / 2
}
