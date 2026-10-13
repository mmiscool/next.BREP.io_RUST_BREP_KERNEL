//! With a span budget the march has already measured every accepted span as the
//! chord-scaled cubic Hermite through its two stations, in 3-D and in both
//! parameter planes, against both carriers, and reports the station tangents
//! and the worst deviation. This module does the same for one clipped run: the
//! C1 piecewise cubic through the run's stations with chord-length knots (so
//! the span emitted is the span the march tested), the two parameter Hermites
//! on the same knots, and a 5-point re-measure of every span the clip altered
//! (a pierce seed inserted between two stations has no march tangent and was
//! never tested). A run whose worst deviation is over the budget, whose
//! stations have no tangents, or whose chords degenerate is left to the quintic
//! fit and the pcurve march.
//!
//! Behind `ImprintOptions::hermite_sections` / `BREP_HERMITE_SECTIONS`; off by
//! default, so the default lane is byte for byte the fit's.

use super::*;
use crate::SurfaceIntersectionCurve;

/// One run station with everything the Hermite needs.
#[derive(Clone, Copy, Debug)]
struct Station {
    point: Vec3,
    tangent: Vec3,
    uv_a: [f64; 2],
    uv_b: [f64; 2],
    /// From the march (tested there) or computed here (an inserted seed).
    marched: bool,
}

/// The emitted section: the 3-D cubic and the two parameter cubics on one
/// knot vector, and the worst span deviation (the march's, or this module's
/// re-measure where larger).
pub(super) struct HermiteSection {
    pub(super) curve: NurbsCurve,
    pub(super) pcurve_a: NurbsCurve,
    pub(super) pcurve_b: NurbsCurve,
    pub(super) deviation: f64,
    pub(super) stations: usize,
    pub(super) inserted: usize,
}

/// Why a run stayed on the fit lane, for the census.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum HermiteDecline {
    NoTangents,
    TooShort,
    DegenerateChord,
    Unprojectable,
    OverBudget,
}

fn key(point: Vec3) -> [u64; 3] {
    [point.x.to_bits(), point.y.to_bits(), point.z.to_bits()]
}

/// The parameter-plane image of a 3-D tangent at `uv`: the solution of the
/// first fundamental form `[E F; F G] d = [S_u·t, S_v·t]`.
fn parameter_velocity(surface: &NurbsSurface, uv: [f64; 2], tangent: Vec3) -> Result<[f64; 2], String> {
    let d = surface.derivatives(uv[0], uv[1], 1)?;
    let (su, sv) = (d[1][0], d[0][1]);
    let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let (bu, bv) = (su.dot(tangent), sv.dot(tangent));
    let det = e * g - f * f;
    if det.abs() <= 1e-300 * (1.0 + e * g) || !det.is_finite() {
        return Err("degenerate first fundamental form".into());
    }
    Ok([(g * bu - f * bv) / det, (e * bv - f * bu) / det])
}

/// `value` moved by whole periods to the image nearest `near` on a closed
/// direction; unchanged on an open one.
fn unwrap(value: f64, near: f64, closed: bool, domain: [f64; 2]) -> f64 {
    let period = domain[1] - domain[0];
    if closed && period > 0.0 && (value - near).abs() > 0.5 * period {
        value - period * ((value - near) / period).round()
    } else {
        value
    }
}

/// `value` brought into the domain on a closed direction, clamped on an open
/// one: where a surface is evaluated at an unwrapped parameter.
fn into_domain(value: f64, closed: bool, domain: [f64; 2]) -> f64 {
    let period = domain[1] - domain[0];
    if closed && period > 0.0 {
        domain[0] + (value - domain[0]).rem_euclid(period)
    } else {
        value.clamp(domain[0], domain[1])
    }
}

fn hermite_at<const N: usize>(p0: [f64; N], d0: [f64; N], p1: [f64; N], d1: [f64; N], s: f64) -> [f64; N] {
    let (s2, s3) = (s * s, s * s * s);
    let (h00, h10, h01, h11) = (2.0 * s3 - 3.0 * s2 + 1.0, s3 - 2.0 * s2 + s, -2.0 * s3 + 3.0 * s2, s3 - s2);
    let mut out = [0.0; N];
    for k in 0..N {
        out[k] = h00 * p0[k] + h10 * d0[k] + h01 * p1[k] + h11 * d1[k];
    }
    out
}

/// The C1 cubic B-spline through `points` with chord-length knots and the
/// given end derivatives per span (`velocity[i]` is the derivative with
/// respect to the knot parameter at station `i`; the same vector serves both
/// adjoining spans, which is what makes the joints C1). Interior knots are
/// double, so the control net is `P0, P0 + h0 v0/3, P1 - h0 v1/3, P1 + h1
/// v1/3, …, Pn-1 - hn-2 vn-1/3, Pn-1`: the Bézier form of the Hermite spans
/// with each joint point implied.
fn hermite_spline<const N: usize>(points: &[[f64; N]], velocity: &[[f64; N]], chords: &[f64]) -> Result<NurbsCurve, String> {
    let n = points.len();
    let mut knots = Vec::with_capacity(2 * n + 4);
    let mut s = 0.0;
    knots.extend([0.0; 4]);
    for (i, chord) in chords.iter().enumerate() {
        s += chord;
        if i + 1 < chords.len() {
            knots.extend([s, s]);
        }
    }
    knots.extend([s; 4]);
    let mut controls = Vec::with_capacity(2 * n);
    let lift = |value: [f64; N]| Vec4 {
        x: value[0],
        y: if N > 1 { value[1] } else { 0.0 },
        z: if N > 2 { value[2] } else { 0.0 },
        w: 1.0,
    };
    for i in 0..n - 1 {
        let h = chords[i];
        let mut a = points[i];
        let mut b = points[i + 1];
        for k in 0..N {
            a[k] += h * velocity[i][k] / 3.0;
            b[k] -= h * velocity[i + 1][k] / 3.0;
        }
        if i == 0 {
            controls.push(lift(points[0]));
        }
        controls.push(lift(a));
        controls.push(lift(b));
    }
    controls.push(lift(points[n - 1]));
    NurbsCurve::new(3, knots, controls)
}

/// Build the Hermite section of one clipped run, or say why the fit lane keeps
/// it. `run` is a sub-range of `branch.points` with pierce seeds inserted; the
/// march surfaces are the ones the branch was marched on (the faces' charts).
pub(super) fn hermite_section(
    run: &[Vec3],
    branch: &SurfaceIntersectionCurve,
    first: &NurbsSurface,
    second: &NurbsSurface,
    budget: f64,
) -> Result<Result<HermiteSection, HermiteDecline>, String> {
    let Some(tangents) = branch.tangents.as_ref() else {
        return Ok(Err(HermiteDecline::NoTangents));
    };
    if run.len() < 2 || tangents.len() != branch.points.len() {
        return Ok(Err(if run.len() < 2 { HermiteDecline::TooShort } else { HermiteDecline::NoTangents }));
    }
    let by_point: HashMap<[u64; 3], usize> = branch.points.iter().enumerate().map(|(i, p)| (key(*p), i)).collect();
    let (closed_a, closed_b) = (first.closed_directions()?, second.closed_directions()?);
    let (ua, va, ub, vb) = (first.domain_u()?, first.domain_v()?, second.domain_u()?, second.domain_v()?);
    let mut stations: Vec<Station> = Vec::with_capacity(run.len());
    let mut inserted = 0usize;
    for (k, point) in run.iter().enumerate() {
        if let Some(&i) = by_point.get(&key(*point)) {
            stations.push(Station {
                point: *point,
                tangent: tangents[i],
                uv_a: branch.params_a[i],
                uv_b: branch.params_b[i],
                marched: true,
            });
            continue;
        }
        // An inserted pierce seed: its parameters by projection, its tangent
        // from the carriers' normals, pointed along the run.
        inserted += 1;
        let (Ok(on_a), Ok(on_b)) = (project_point_to_surface(first, *point), project_point_to_surface(second, *point)) else {
            return Ok(Err(HermiteDecline::Unprojectable));
        };
        let (Ok(na), Ok(nb)) = (first.normal(on_a.u, on_a.v), second.normal(on_b.u, on_b.v)) else {
            return Ok(Err(HermiteDecline::Unprojectable));
        };
        let cross = na.cross(nb);
        if cross.length() <= 1e-9 {
            return Ok(Err(HermiteDecline::Unprojectable));
        }
        let mut tangent = cross.normalized()?;
        let along = run[(k + 1).min(run.len() - 1)].sub(run[k.saturating_sub(1)]);
        if tangent.dot(along) < 0.0 {
            tangent = tangent.scale(-1.0);
        }
        stations.push(Station { point: *point, tangent, uv_a: [on_a.u, on_a.v], uv_b: [on_b.u, on_b.v], marched: false });
    }
    // A station repeated by its neighbour (a pierce seed on a station, a
    // closed branch's returned start) spans nothing and would make the knot
    // vector non-increasing: keep one, the marched one where it matters.
    let mut deduped: Vec<Station> = Vec::with_capacity(stations.len());
    for station in stations {
        match deduped.last_mut() {
            Some(last) if last.point.sub(station.point).length() <= 1e-12 => {
                if !last.marched && station.marched {
                    *last = station;
                } else if !station.marched {
                    inserted -= 1;
                }
            }
            _ => deduped.push(station),
        }
    }
    let stations = deduped;
    if stations.len() < 2 {
        return Ok(Err(HermiteDecline::TooShort));
    }
    // Chords, and the parameter sequences unwrapped along the run.
    let mut chords = Vec::with_capacity(stations.len() - 1);
    for pair in stations.windows(2) {
        let chord = pair[1].point.sub(pair[0].point).length();
        if !(chord.is_finite() && chord > 1e-12) {
            return Ok(Err(HermiteDecline::DegenerateChord));
        }
        chords.push(chord);
    }
    let mut uv_a: Vec<[f64; 2]> = Vec::with_capacity(stations.len());
    let mut uv_b: Vec<[f64; 2]> = Vec::with_capacity(stations.len());
    for station in &stations {
        let (a, b) = match (uv_a.last(), uv_b.last()) {
            (Some(pa), Some(pb)) => (
                [unwrap(station.uv_a[0], pa[0], closed_a.0, ua), unwrap(station.uv_a[1], pa[1], closed_a.1, va)],
                [unwrap(station.uv_b[0], pb[0], closed_b.0, ub), unwrap(station.uv_b[1], pb[1], closed_b.1, vb)],
            ),
            _ => (station.uv_a, station.uv_b),
        };
        uv_a.push(a);
        uv_b.push(b);
    }
    // Velocities: unit tangents in 3-D (the knot is chord length), their
    // parameter images on each carrier, read at the in-domain parameter.
    let mut velocity_a = Vec::with_capacity(stations.len());
    let mut velocity_b = Vec::with_capacity(stations.len());
    for (i, station) in stations.iter().enumerate() {
        let at_a = [into_domain(uv_a[i][0], closed_a.0, ua), into_domain(uv_a[i][1], closed_a.1, va)];
        let at_b = [into_domain(uv_b[i][0], closed_b.0, ub), into_domain(uv_b[i][1], closed_b.1, vb)];
        let (Ok(da), Ok(db)) = (parameter_velocity(first, at_a, station.tangent), parameter_velocity(second, at_b, station.tangent)) else {
            return Ok(Err(HermiteDecline::Unprojectable));
        };
        velocity_a.push(da);
        velocity_b.push(db);
    }
    // Every span with an inserted end was never tested by the march: the
    // span test, as the march runs it, on the Hermites about to be emitted.
    let mut deviation = branch.span_deviation.unwrap_or(0.0);
    for i in 0..stations.len() - 1 {
        if stations[i].marched && stations[i + 1].marched {
            continue;
        }
        let h = chords[i];
        let (p0, p1) = (stations[i].point, stations[i + 1].point);
        let (d0, d1) = (stations[i].tangent.scale(h), stations[i + 1].tangent.scale(h));
        let scale2 = |v: [f64; 2]| [v[0] * h, v[1] * h];
        for s in [0.17, 0.33, 0.5, 0.67, 0.83] {
            let c = hermite_at([p0.x, p0.y, p0.z], [d0.x, d0.y, d0.z], [p1.x, p1.y, p1.z], [d1.x, d1.y, d1.z], s);
            let c = Vec3::new(c[0], c[1], c[2]);
            let a = hermite_at(uv_a[i], scale2(velocity_a[i]), uv_a[i + 1], scale2(velocity_a[i + 1]), s);
            let b = hermite_at(uv_b[i], scale2(velocity_b[i]), uv_b[i + 1], scale2(velocity_b[i + 1]), s);
            let a = first.evaluate(into_domain(a[0], closed_a.0, ua), into_domain(a[1], closed_a.1, va))?;
            let b = second.evaluate(into_domain(b[0], closed_b.0, ub), into_domain(b[1], closed_b.1, vb))?;
            deviation = deviation.max(a.sub(b).length()).max(a.sub(c).length()).max(b.sub(c).length());
        }
    }
    if !(deviation <= budget) {
        return Ok(Err(HermiteDecline::OverBudget));
    }
    let points3: Vec<[f64; 3]> = stations.iter().map(|s| [s.point.x, s.point.y, s.point.z]).collect();
    let velocity3: Vec<[f64; 3]> = stations.iter().map(|s| [s.tangent.x, s.tangent.y, s.tangent.z]).collect();
    let curve = hermite_spline(&points3, &velocity3, &chords)?;
    let pcurve_a = hermite_spline(&uv_a, &velocity_a, &chords)?;
    let pcurve_b = hermite_spline(&uv_b, &velocity_b, &chords)?;
    Ok(Ok(HermiteSection { curve, pcurve_a, pcurve_b, deviation, stations: stations.len(), inserted }))
}

/// A split piece of an emitted pcurve, brought onto the chart: shifted by
/// whole periods so its middle lies in the domain of each closed direction,
/// and its ends moved to the chart images of the piece's snapped 3-D ends (the
/// period image nearest the pcurve's own end), as the marched pcurve would
/// have read them.
pub(super) fn piece_pcurve_on_chart(pcurve: &NurbsCurve, chart: &NurbsSurface, start: Vec3, end: Vec3) -> Result<NurbsCurve, String> {
    let closed = chart.closed_directions()?;
    let (du, dv) = (chart.domain_u()?, chart.domain_v()?);
    let [t0, t1] = pcurve.domain()?;
    let middle = pcurve.evaluate(0.5 * (t0 + t1))?;
    let shift = |value: f64, closed: bool, domain: [f64; 2]| -> f64 {
        let period = domain[1] - domain[0];
        if !closed || period <= 0.0 {
            return 0.0;
        }
        let wrapped = domain[0] + (value - domain[0]).rem_euclid(period);
        wrapped - value
    };
    let (sx, sy) = (shift(middle.x, closed.0, du), shift(middle.y, closed.1, dv));
    let mut controls = pcurve.control_points.clone();
    if sx != 0.0 || sy != 0.0 {
        for control in &mut controls {
            control.x += sx * control.w;
            control.y += sy * control.w;
        }
    }
    let last = controls.len() - 1;
    for (index, point) in [(0usize, start), (last, end)] {
        let Ok(on) = project_point_to_surface(chart, point) else { continue };
        let w = controls[index].w;
        let (cx, cy) = (controls[index].x / w, controls[index].y / w);
        let u = unwrap(on.u, cx, closed.0, du);
        let v = unwrap(on.v, cy, closed.1, dv);
        // A projection that lands a long way from the emitted end is a
        // different sheet (a pole, a fold): keep the emitted end.
        let reach = 1e-2 * ((du[1] - du[0]).abs() + (dv[1] - dv[0]).abs());
        if ((u - cx).powi(2) + (v - cy).powi(2)).sqrt() <= reach {
            controls[index].x = u * w;
            controls[index].y = v * w;
        }
    }
    NurbsCurve::new(pcurve.degree, pcurve.knots.clone(), controls)
}
