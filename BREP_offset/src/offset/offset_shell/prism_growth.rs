//! The GROWN rounded parallel body of a right prism whose side is one smooth
//! convex ruled face between two planar caps (`offset_solid` by +d with
//! `OffsetCorners::Rounded`), built from its parts instead of rounding the
//! grown sharp body's rims with the general blend.
//!
//! The parallel body of a convex prism over the closed profile c(s) is bounded
//! by the side moved out, c(s) + d n(s) extruded over the height (n the
//! in-plane outward normal); the two caps moved out by d along the axis z,
//! each trimmed by the profile itself; and about each rim the quarter tube of
//! the rolling ball, c(s) + d (cos θ n(s) − sin θ z) at the bottom and
//! + sin θ z at the top, θ from 0 (on the side) to π/2 (on the cap).
//!
//! The offset of a curve that is not a line or a circle has no exact NURBS
//! form, so the profile and its offset are both interpolated, closed and
//! quintic, at the same stations (the knot vector depends only on the stations,
//! so the two fits share it), refined until both read their exact points
//! within the tolerance between stations. The stations are equal steps of the
//! tangent's turning angle, which a strictly convex profile runs through once
//! and in which both curves are smooth; the profile's own parameter is not (a
//! rational ellipse in quarter spans is only C1 in it at its knots). Each tube is
//! then exact about the fitted rails: the rational quadratic in θ with rows
//! O(s), O(s) ∓ d z and C(s) ∓ d z and weights (1, √2/2, 1) is the quarter
//! circle c + d (cos θ N − sin θ z) for N = (O − C)/d, row by row. The faces
//! are joined by the kernel's sewing (`solid_from_sheets`).
//!
//! Cause (brep-bench offset/elliptic-cylinder-grown): the elliptic cylinder
//! x²/25 + y²/9 ≤ 1, 0 ≤ z ≤ 10, grown by 1 took 2.3 s, all of it in the blend
//! marching the two elliptic rims of the grown sharp body.
//!
//! It runs before the pipeline for exactly that body (three faces: two caps
//! square to a ruled side along one translation, the profile closed and
//! strictly convex) and declines on anything else.
use super::*;
use crate::fit::interpolate_curve_closed_of_degree;
use crate::{face_from_surface, face_from_surface_trimmed, solid_from_sheets, NurbsCurve, NurbsSurface, Vec4};

/// `BREP_OFFSET_PRISM_GROWTH` (default on; `=0` restores): [`prism_growth`].
pub(super) fn prism_growth_on() -> bool {
    switch!("BREP_OFFSET_PRISM_GROWTH", true)
}


/// The fewest and most stations the fits try (doubling).
const FIRST_STATIONS: usize = 64;
const MAX_STATIONS: usize = 512;

/// `BREP_OFFSET_PRISM_GROWTH_STATIONS` (default on; `=0` restores): the
/// fits refine up to 2048 stations, not 512. Cause (brep-bench
/// offset/flat-elliptic-cylinder-grown, semi-axes 10 and 2 grown by 1): the
/// fits read 7.3e-9 off at 512 stations, over the 1e-9 bar, and settle about
/// eighty times closer per doubling; the lane declined and the blend refused
/// the rims ("wall stands 9.6e-7 off the rolling ball").
fn max_stations() -> usize {
    if switch!("BREP_OFFSET_PRISM_GROWTH_STATIONS", true) { 4 * MAX_STATIONS } else { MAX_STATIONS }
}


/// The grown rounded parallel body of `source` by `distance` (> 0), or `None`.
pub(super) fn prism_growth(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !prism_growth_on() || !(distance > 0.0) || !distance.is_finite() {
        return None;
    }
    match prism_growth_impl(source, distance) {
        Ok(result) => result,
        Err(reason) => {
            os_debug!("prism growth declined: {reason}");
            None
        }
    }
}

/// `curve` with every control point moved by `delta`.
fn shifted(curve: &NurbsCurve, delta: Vec3) -> Result<NurbsCurve, String> {
    let points = curve
        .control_points
        .iter()
        .map(|p| Vec4 { x: p.x + p.w * delta.x, y: p.y + p.w * delta.y, z: p.z + p.w * delta.z, w: p.w })
        .collect();
    NurbsCurve::new(curve.degree, curve.knots.clone(), points)
}

/// The surface over `rows` (curves on one knot vector, non-rational) along u,
/// with `weights` across them and the v knots `knots_v` of degree `degree_v`.
fn net(rows: &[&NurbsCurve], weights: &[f64], degree_v: usize, knots_v: Vec<f64>) -> Result<NurbsSurface, String> {
    let first = rows[0];
    let count = first.control_points.len();
    if rows.iter().any(|row| row.knots != first.knots || row.control_points.len() != count) {
        return Err("the rows do not share a knot vector".into());
    }
    let control_points = (0..count)
        .map(|i| {
            rows.iter()
                .zip(weights)
                .map(|(row, w)| {
                    let p = row.control_points[i];
                    Vec4 { x: p.x / p.w * w, y: p.y / p.w * w, z: p.z / p.w * w, w: *w }
                })
                .collect()
        })
        .collect();
    NurbsSurface::new(first.degree, degree_v, first.knots.clone(), knots_v, control_points)
}

fn prism_growth_impl(source: &BrepSolid, distance: f64) -> Result<Option<OffsetShellResultRecord>, String> {
    let scale = crate::solid_scale(source).max(1.0);
    let Ok(prism) = super::prism_parallel::recognize_prism(source, 1e-7 * scale) else {
        return Err("not a right prism with one ruled side".into());
    };
    let profile = &prism.profile;
    let axis = prism.axis;
    // The side is a translation sweep exactly, not only at the recognizer's
    // samples: every control row's top point is its bottom point moved by the
    // height along the axis, at one weight. The body built below reads only
    // the bottom rail, so a top rail that strays anywhere must decline.
    let run = axis.scale(prism.height);
    for row in &prism.side.surface.control_points {
        let [bottom, top] = row.as_slice() else { return Err("the side is not one ruling per row".into()) };
        let moved = Vec3::new(bottom.x + bottom.w * run.x, bottom.y + bottom.w * run.y, bottom.z + bottom.w * run.z);
        let gap = Vec3::new(top.x - moved.x, top.y - moved.y, top.z - moved.z).length();
        if (top.w - bottom.w).abs() > 1e-12 * bottom.w.abs() || gap > 1e-9 * scale * bottom.w.abs() {
            return Err("the side's top rail is not its bottom rail moved up the axis".into());
        }
    }
    let [s0, s1] = profile.domain()?;
    let period = s1 - s0;
    // The in-plane outward normal, by the profile's turn: strictly convex,
    // one sign of turn all round.
    let frame = |s: f64| -> Result<(Vec3, Vec3, f64), String> {
        let d = profile.derivatives(s, 2)?;
        let speed = d[1].length();
        if !(speed > 1e-300) {
            return Err("the profile is stationary".into());
        }
        let tangent = d[1].scale(1.0 / speed);
        Ok((d[0], tangent, d[1].cross(d[2]).dot(axis) / (speed * speed * speed)))
    };
    let mut turn_sign = 0.0f64;
    for k in 0..512 {
        let (_, _, turn) = frame(s0 + period * (k as f64 + 0.5) / 512.0)?;
        if !(turn.abs() > 1e-12) {
            return Err("the profile is not strictly convex".into());
        }
        if turn_sign == 0.0 {
            turn_sign = turn.signum();
        } else if turn.signum() != turn_sign {
            return Err("the profile is not convex".into());
        }
    }
    let point_and_offset = |s: f64| -> Result<(Vec3, Vec3), String> {
        let (point, tangent, _) = frame(s)?;
        let outward = tangent.cross(axis).scale(turn_sign);
        Ok((point, point.add(outward.scale(distance))))
    };
    let tolerance = 1e-10 * scale;
    // The turning angle, unwrapped, on a dense table of the profile's
    // parameter; a station's parameter is found by bisection inside its bracket.
    let ex = axis.perpendicular()?;
    let ey = axis.cross(ex);
    let angle = |s: f64| -> Result<f64, String> {
        let (_, tangent, _) = frame(s)?;
        Ok(turn_sign * tangent.dot(ey).atan2(tangent.dot(ex)))
    };
    const TABLE: usize = 4096;
    let mut table: Vec<(f64, f64)> = Vec::with_capacity(TABLE + 1);
    for k in 0..=TABLE {
        let s = s0 + period * k as f64 / TABLE as f64;
        let mut phi = angle(s)?;
        if let Some(&(_, before)) = table.last() {
            while phi < before - std::f64::consts::PI {
                phi += std::f64::consts::TAU;
            }
            while phi > before + std::f64::consts::PI {
                phi -= std::f64::consts::TAU;
            }
            if phi <= before {
                return Err("the turning angle does not rise".into());
            }
        }
        table.push((s, phi));
    }
    let (phi0, phi1) = (table[0].1, table[TABLE].1);
    if ((phi1 - phi0) - std::f64::consts::TAU).abs() > 1e-6 {
        return Err("the profile does not turn once".into());
    }
    let parameter_at = |phi: f64| -> Result<f64, String> {
        let k = table.partition_point(|&(_, p)| p < phi).clamp(1, TABLE);
        let (mut lo, mut hi) = (table[k - 1], table[k]);
        for _ in 0..60 {
            let mid = 0.5 * (lo.0 + hi.0);
            let mut p = angle(mid)?;
            while p < lo.1 - std::f64::consts::PI {
                p += std::f64::consts::TAU;
            }
            while p > lo.1 + std::f64::consts::PI {
                p -= std::f64::consts::TAU;
            }
            if p < phi { lo = (mid, p) } else { hi = (mid, p) }
        }
        Ok(0.5 * (lo.0 + hi.0))
    };
    // The fits, refined until both read their exact points midway between stations.
    let mut fitted = None;
    let mut stations = FIRST_STATIONS;
    let most = max_stations();
    while stations <= most {
        let parameters: Vec<f64> = (0..=stations).map(|k| std::f64::consts::TAU * k as f64 / stations as f64).collect();
        let mut points = Vec::with_capacity(stations);
        let mut offsets = Vec::with_capacity(stations);
        for &t in &parameters[..stations] {
            let (p, o) = point_and_offset(parameter_at(phi0 + t)?)?;
            points.push(p);
            offsets.push(o);
        }
        let rim = interpolate_curve_closed_of_degree(&points, &parameters, 5)?;
        let out = interpolate_curve_closed_of_degree(&offsets, &parameters, 5)?;
        let mut worst = 0.0f64;
        for k in 0..stations {
            let t = std::f64::consts::TAU * (k as f64 + 0.5) / stations as f64;
            let (p, o) = point_and_offset(parameter_at(phi0 + t)?)?;
            worst = worst.max(rim.evaluate(t)?.sub(p).length()).max(out.evaluate(t)?.sub(o).length());
        }
        if worst <= tolerance {
            fitted = Some((rim, out));
            break;
        }
        os_debug!("prism growth: {stations} stations fit within {worst:.3e}, over {tolerance:.3e}");
        stations *= 2;
    }
    let (rim, out) = fitted.ok_or("the profile's fits did not settle within the tolerance")?;
    let (h, d) = (prism.height, distance);
    let down = axis.scale(-d);
    let up = axis.scale(h + d);
    let lift = axis.scale(h);
    // The side: the offset rail swept up the height.
    let side_top = shifted(&out, lift)?;
    let side = net(&[&out, &side_top], &[1.0, 1.0], 1, vec![0.0, 0.0, 1.0, 1.0])?;
    // The tubes: side rail, the corner row, cap rail.
    let quarter = std::f64::consts::FRAC_1_SQRT_2;
    let circle_knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let (bottom_corner, bottom_rail) = (shifted(&out, down)?, shifted(&rim, down)?);
    let bottom_tube = net(&[&out, &bottom_corner, &bottom_rail], &[1.0, quarter, 1.0], 2, circle_knots.clone())?;
    let (top_corner, top_rail) = (shifted(&out, up)?, shifted(&rim, up)?);
    let top_tube = net(&[&side_top, &top_corner, &top_rail], &[1.0, quarter, 1.0], 2, circle_knots)?;
    // The caps: planes square to the axis through the cap rails, trimmed by them.
    let base = profile.evaluate(s0)?;
    let reach = 2.0 * (scale + d);
    let cap_plane = |level: f64| -> Result<NurbsSurface, String> {
        let corner = |i: f64, j: f64| {
            let p = base.add(axis.scale(level)).add(ex.scale(reach * i)).add(ey.scale(reach * j));
            Vec4::from_point(p, 1.0)
        };
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![vec![corner(-1.0, -1.0), corner(-1.0, 1.0)], vec![corner(1.0, -1.0), corner(1.0, 1.0)]],
        )
    };
    let refuse = |error: crate::KernelRefusal| error.message;
    let sheets = vec![
        face_from_surface(&side).map_err(refuse)?,
        face_from_surface(&bottom_tube).map_err(refuse)?,
        face_from_surface(&top_tube).map_err(refuse)?,
        face_from_surface_trimmed(&cap_plane(-d)?, &[vec![bottom_rail.clone()]]).map_err(refuse)?,
        face_from_surface_trimmed(&cap_plane(h + d)?, &[vec![top_rail.clone()]]).map_err(refuse)?,
    ];
    let solid = solid_from_sheets(&sheets, 1e-7 * scale).map_err(refuse)?;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!("the grown prism does not validate: {:?}", issues.first()));
    }
    if solid.shells.len() != 1 || solid.shells[0].faces.len() != 5 {
        return Err("the sewn body is not one shell of five faces".into());
    }
    // Provenance: a cap is its source cap's image, the side the side's, a tube none.
    let mut face_images = Vec::with_capacity(5);
    for face in &solid.shells[0].faces {
        let source_face_id = if face.surface.is_affine().unwrap_or(false) {
            let [a0, a1] = face.surface.domain_u()?;
            let [b0, b1] = face.surface.domain_v()?;
            let level = face.surface.evaluate(0.5 * (a0 + a1), 0.5 * (b0 + b1))?.sub(base).dot(axis);
            if level < 0.5 * h { prism.bottom.id } else { prism.top.id }
        } else if face.surface.degree_v == 1 || face.surface.degree_u == 1 {
            prism.side.id
        } else {
            0
        };
        face_images.push(OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id });
    }
    Ok(Some(OffsetShellResultRecord { solid, face_images }))
}
