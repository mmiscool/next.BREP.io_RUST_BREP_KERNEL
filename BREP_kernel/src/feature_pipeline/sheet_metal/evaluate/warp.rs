//! Carry an exact solid through a smooth point map by refitting its geometry.
//!
//! Bend reliefs are cut in the developed band, where they are exact prisms;
//! the folded band is that region under the fold map, which takes slanted
//! lines to helices and their walls to helicoids (not NURBS). [`warp_solid`]
//! rebuilds every surface and edge curve as a cubic B-spline of the mapped
//! geometry over its OWN parameter domain, so the topology and every pcurve
//! carry over unchanged. Each fit is refined until its sampled error is within
//! `tol` (owner 2026-10-28: "Fit to tolerance").
use crate::interpolate_curve;
use crate::{BrepSolid, NurbsCurve, NurbsSurface, Vec3, Vec4};

/// Cap on spans per parameter direction: past it the map is not smooth enough
/// to be fit to `tol`, and the caller gets an error rather than a coarse wall.
const MAX_SPANS: usize = 128;

pub(super) fn warp_solid(
    solid: &BrepSolid,
    map: &dyn Fn(Vec3) -> Vec3,
    tol: f64,
) -> Result<BrepSolid, String> {
    let mut out = solid.clone();
    out.mass_properties_cache = Default::default();
    for vertex in &mut out.vertices {
        vertex.point = map(vertex.point);
    }
    for edge in &mut out.edges {
        edge.curve = fit_curve(&edge.curve, map, tol)?;
    }
    for face in out.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        // Fit only the parameter box the face's trims use: a plane's carrier
        // can span far more than its face, and folded that wraps round.
        let (du, dv) = (face.surface.domain_u()?, face.surface.domain_v()?);
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
            for p in &coedge.pcurve.control_points {
                let q = p.point()?;
                lo = [lo[0].min(q.x), lo[1].min(q.y)];
                hi = [hi[0].max(q.x), hi[1].max(q.y)];
            }
        }
        let pad = |a: f64, b: f64, range: [f64; 2]| {
            let m = 1e-3 * (b - a).max(1e-9);
            [(a - m).max(range[0]), (b + m).min(range[1])]
        };
        let (bu, bv) = if lo[0] <= hi[0] && lo[1] <= hi[1] {
            (pad(lo[0], hi[0], du), pad(lo[1], hi[1], dv))
        } else {
            (du, dv)
        };
        face.surface = fit_surface(&face.surface, map, tol, bu, bv)?;
    }
    Ok(out)
}

fn uniform(range: [f64; 2], count: usize) -> Vec<f64> {
    (0..count)
        .map(|i| range[0] + (range[1] - range[0]) * i as f64 / (count - 1) as f64)
        .collect()
}

/// Cubic interpolation of `points` at `params` (degree drops for few points).
fn interpolate(points: &[Vec3], params: &[f64]) -> Result<NurbsCurve, String> {
    interpolate_curve(points, 3.min(points.len() - 1), params)
}

fn fit_curve(curve: &NurbsCurve, map: &dyn Fn(Vec3) -> Vec3, tol: f64) -> Result<NurbsCurve, String> {
    // A rational arc's parameter speed jumps at its interior knots; a smooth
    // fit across one stalls, so each knot span is fit alone and the pieces are
    // joined there (a C0 knot, as in the original).
    let mut joined: Option<NurbsCurve> = None;
    for span in breaks(&curve.knots, curve.domain()?).windows(2) {
        let piece = fit_span(curve, map, tol, [span[0], span[1]])?;
        joined = Some(match joined {
            None => piece,
            Some(head) => join(&head, &piece)?,
        });
    }
    joined.ok_or_else(|| "bend relief: an empty edge".to_string())
}

/// Join two clamped curves of one degree meeting at `head`'s end parameter.
fn join(head: &NurbsCurve, tail: &NurbsCurve) -> Result<NurbsCurve, String> {
    let p = head.degree;
    let mut knots = head.knots[..head.knots.len() - 1].to_vec();
    knots.extend_from_slice(&tail.knots[p + 1..]);
    let mut points = head.control_points.clone();
    points.extend_from_slice(&tail.control_points[1..]);
    NurbsCurve::new(p, knots, points)
}

fn fit_span(curve: &NurbsCurve, map: &dyn Fn(Vec3) -> Vec3, tol: f64, domain: [f64; 2]) -> Result<NurbsCurve, String> {
    let target = |t: f64| curve.evaluate(t).map(map);
    let mut spans = 4;
    loop {
        let params = uniform(domain, spans + 3);
        let points = params.iter().map(|&t| target(t)).collect::<Result<Vec<_>, _>>()?;
        let fitted = interpolate(&points, &params)?;
        let mut error: f64 = 0.0;
        for pair in params.windows(2) {
            let t = 0.5 * (pair[0] + pair[1]);
            error = error.max(fitted.evaluate(t)?.sub(target(t)?).length());
        }
        if error <= tol {
            return Ok(fitted);
        }
        if spans >= MAX_SPANS {
            return Err(format!("bend relief: an edge fits to {error:.3e}, not {tol:.1e}"));
        }
        spans *= 2;
    }
}

/// The distinct knots of `knots` inside `range`, with the range ends: the
/// spans a smooth fit must not cross.
fn breaks(knots: &[f64], range: [f64; 2]) -> Vec<f64> {
    let mut out = vec![range[0]];
    for &knot in knots {
        if knot > out[out.len() - 1] + 1e-12 && knot < range[1] - 1e-12 {
            out.push(knot);
        }
    }
    out.push(range[1]);
    out
}

/// Interpolate `points[k]` at `sites[k]` span by span (one site list per span,
/// sharing their end sites) and join the pieces at the span breaks.
fn interpolate_spans(points: &[Vec<Vec3>], sites: &[Vec<f64>]) -> Result<NurbsCurve, String> {
    let mut joined: Option<NurbsCurve> = None;
    for (span_points, span_sites) in points.iter().zip(sites) {
        let piece = interpolate(span_points, span_sites)?;
        joined = Some(match joined {
            None => piece,
            Some(head) => join(&head, &piece)?,
        });
    }
    joined.ok_or_else(|| "bend relief: an empty wall".to_string())
}

fn fit_surface(
    surface: &NurbsSurface,
    map: &dyn Fn(Vec3) -> Vec3,
    tol: f64,
    du: [f64; 2],
    dv: [f64; 2],
) -> Result<NurbsSurface, String> {
    let target = |u: f64, v: f64| surface.evaluate(u, v).map(map);
    let (bu, bv) = (breaks(&surface.knots_u, du), breaks(&surface.knots_v, dv));
    let (mut nu, mut nv) = (4, 4);
    loop {
        let su: Vec<Vec<f64>> = bu.windows(2).map(|w| uniform([w[0], w[1]], nu + 3)).collect();
        let sv: Vec<Vec<f64>> = bv.windows(2).map(|w| uniform([w[0], w[1]], nv + 3)).collect();
        // Rows along u at every v site (span by span), then each control column along v.
        let mut rows: Vec<Vec<NurbsCurve>> = Vec::with_capacity(sv.len());
        for span in &sv {
            let mut span_rows = Vec::with_capacity(span.len());
            for &v in span {
                let points = su
                    .iter()
                    .map(|sites| sites.iter().map(|&u| target(u, v)).collect::<Result<Vec<_>, _>>())
                    .collect::<Result<Vec<_>, _>>()?;
                span_rows.push(interpolate_spans(&points, &su)?);
            }
            rows.push(span_rows);
        }
        let count_u = rows[0][0].control_points.len();
        let mut columns = Vec::with_capacity(count_u);
        for i in 0..count_u {
            let points = rows
                .iter()
                .map(|span_rows| span_rows.iter().map(|row| row.control_points[i].point()).collect::<Result<Vec<_>, _>>())
                .collect::<Result<Vec<_>, _>>()?;
            columns.push(interpolate_spans(&points, &sv)?);
        }
        let control_points = columns
            .iter()
            .map(|column| {
                column.control_points.iter().map(|p| p.point().map(|q| Vec4::from_point(q, 1.0))).collect()
            })
            .collect::<Result<Vec<Vec<Vec4>>, String>>()?;
        let fitted = NurbsSurface::new(
            rows[0][0].degree,
            columns[0].degree,
            rows[0][0].knots.clone(),
            columns[0].knots.clone(),
            control_points,
        )?;
        // Error at cell midpoints, split by direction so only the rough one refines.
        let mids = |sites: &[Vec<f64>]| -> Vec<f64> {
            sites.iter().flat_map(|span| span.windows(2).map(|w| 0.5 * (w[0] + w[1]))).collect()
        };
        let flat = |sites: &[Vec<f64>]| -> Vec<f64> { sites.iter().flatten().copied().collect() };
        let (mu, mv, au, av) = (mids(&su), mids(&sv), flat(&su), flat(&sv));
        let (mut along_u, mut along_v, mut center): (f64, f64, f64) = (0.0, 0.0, 0.0);
        for &v in &av {
            for &u in &mu {
                along_u = along_u.max(fitted.evaluate(u, v)?.sub(target(u, v)?).length());
            }
        }
        for &v in &mv {
            for &u in &au {
                along_v = along_v.max(fitted.evaluate(u, v)?.sub(target(u, v)?).length());
            }
            for &u in &mu {
                center = center.max(fitted.evaluate(u, v)?.sub(target(u, v)?).length());
            }
        }
        // A cell centre that fails while both edge midlines pass refines both.
        if center > tol && along_u <= tol && along_v <= tol {
            along_u = center;
            along_v = center;
        }
        if along_u <= tol && along_v <= tol {
            return Ok(fitted);
        }
        if along_u > tol {
            if nu >= MAX_SPANS {
                return Err(format!("bend relief: a wall fits to {along_u:.3e}, not {tol:.1e}"));
            }
            nu *= 2;
        }
        if along_v > tol {
            if nv >= MAX_SPANS {
                return Err(format!("bend relief: a wall fits to {along_v:.3e}, not {tol:.1e}"));
            }
            nv *= 2;
        }
    }
}

