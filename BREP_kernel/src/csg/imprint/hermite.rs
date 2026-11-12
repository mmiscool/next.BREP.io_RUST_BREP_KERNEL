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
//! The legacy `ImprintOptions::hermite_sections` / `BREP_HERMITE_SECTIONS`
//! experiment remains off by default. The carried-UV switches independently
//! select production emission, retaining fitting for ineligible sections.

use super::*;
use crate::SurfaceIntersectionCurve;

/// Trace-carried UV sections at the pcurve accuracy floor. This is independent
/// of the older, looser-budget Hermite experiment. Disable both UV switches
/// to retain fitting; reuse never changes the march budget.
pub(crate) fn carried_uv_enabled() -> bool {
    switch!("BREP_MARCH_UV_SECTIONS", true) || emission_refine_enabled()
}

/// Refine only the emission samples after clipping, never the original march.
fn emission_refine_enabled() -> bool {
    switch!("BREP_MARCH_UV_REFINE", true)
}

/// Decide eligibility before retaining metadata or entering the large emitter.
pub(crate) fn carried_uv_min_stations() -> usize {
    if emission_refine_enabled() { 32 } else { 2 }
}

/// Refinement is qualified only where at most one chart direction closes.
/// On doubly closed carriers the emitted representation can leave a downstream
/// folding-wall interval unreadable. Preserve fitting there until that chart
/// family is qualified; check original carriers before march-window cropping.
pub(super) fn refinement_carriers_eligible(first: &NurbsSurface, second: &NurbsSurface) -> bool {
    !emission_refine_enabled() || [first, second].iter().all(|surface| {
        matches!(surface.closed_directions(), Ok((u, v)) if !(u && v))
    })
}

pub(super) fn carried_uv_emission_budget(requested: Option<f64>, fit_budget: f64) -> Option<f64> {
    if carried_uv_enabled() {
        Some(requested.unwrap_or(fit_budget).min(crate::pcurve::PCURVE_REFINEMENT_TOLERANCE * 0.1))
    } else {
        requested
    }
}

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
    NotAmortized,
}

fn key(point: Vec3) -> [u64; 3] {
    [point.x.to_bits(), point.y.to_bits(), point.z.to_bits()]
}

/// The 3-D section lane for a marched run whose fit rings
/// (`BREP_SECTION_HERMITE_3D`, default on; `=0` restores the fit). A plane meeting two swept walls
/// along the G1 junction between them cuts the one wall in a section that
/// arrives square to the plane's other section and turns through a right
/// angle within a radius far below the run's own length (a drum screwed
/// across a ring sector: radius about 7e-5 on a section 2 long). The march
/// samples the turn densely and the rest coarsely; the station pool drops the
/// tip stations under its near-duplicate floor, and no global interpolant
/// through what is left can follow the turn: the quintic rings between
/// stations by hundreds of units while it reproduces every station. The
/// carried-UV lane above does not take such a run either when it crosses a
/// carrier's crease line (a knot of multiplicity at least the degree; at a
/// rational arc's double knot the wall is C1 but its second derivative jumps):
/// a parameter Hermite across the line converges as h^2, not h^4, and the
/// emission refiner's depth runs out first.
///
/// This lane keeps every march station with its march tangent, measures each
/// span's cubic Hermite against both carriers at its quarter points, and
/// bisects a span that misses onto a true intersection point (the march's own
/// Newton) until every span holds `budget`. Only the 3-D curve is emitted;
/// pcurves are built from it as for a fitted section. `None` when a span
/// cannot be brought within the budget.
pub(crate) fn hermite_3d_enabled() -> bool {
    switch!("BREP_SECTION_HERMITE_3D", true)
}

/// `BREP_HERMITE_END_WELD` (default on, `=0` off): [`hermite_section_3d`] reads an
/// interior station within the weld floor of the run's end as that end.
fn end_weld_enabled() -> bool {
    switch!("BREP_HERMITE_END_WELD", true)
}

fn decline_3d(site: usize) {
    if std::env::var("BREP_DEBUG_HERMITE_3D").is_ok() {
        eprintln!("hermite 3-D decline at site {site}");
    }
}

/// A [`hermite_section_3d`] station: point, unit tangent along the run, and
/// its parameters on both carriers (unwrapped along the run).
type Station3 = (Vec3, Vec3, [f64; 2], [f64; 2]);

/// The interior parameter lines of `surface` at a knot of multiplicity at
/// least the degree, per direction: C0 in general, and C1 at best with a jump
/// in the second derivative (a rational arc's joint).
fn crease_lines(surface: &NurbsSurface) -> [Vec<f64>; 2] {
    let lines = |knots: &[f64], degree: usize| -> Vec<f64> {
        let (low, high) = (knots[degree], knots[knots.len() - degree - 1]);
        let mut out = Vec::new();
        let mut index = 0;
        while index < knots.len() {
            let value = knots[index];
            let run = knots[index..].iter().take_while(|knot| **knot == value).count();
            if value > low && value < high && run >= degree.max(1) {
                out.push(value);
            }
            index += run;
        }
        out
    };
    [lines(&surface.knots_u, surface.degree_u), lines(&surface.knots_v, surface.degree_v)]
}

/// The C1 cubic of [`hermite_3d_enabled`] through `run` with its two pcurves
/// on the same knots, or `None`. A station is placed exactly on every crease
/// line of either carrier the run crosses, and each span's parameter Hermites
/// take their end velocities from inside the span, so no span's Hermite
/// straddles a crease and a pcurve may turn at one where its carrier's
/// parameterization does.
pub(super) fn hermite_section_3d(
    run: &[Vec3],
    branch: &SurfaceIntersectionCurve,
    first: &NurbsSurface,
    second: &NurbsSurface,
    budget: f64,
) -> Result<Option<(NurbsCurve, [NurbsCurve; 2])>, String> {
    if run.len() < 2 || !(budget > 0.0 && budget.is_finite()) {
        { decline_3d(1); return Ok(None); }
    }
    let tangents = branch.tangents.as_ref();
    let by_point: HashMap<[u64; 3], usize> = branch.points.iter().enumerate().map(|(i, p)| (key(*p), i)).collect();
    let (closed_a, closed_b) = (first.closed_directions()?, second.closed_directions()?);
    let (ua, va, ub, vb) = (first.domain_u()?, first.domain_v()?, second.domain_u()?, second.domain_v()?);
    let in_a = |uv: [f64; 2]| [into_domain(uv[0], closed_a.0, ua), into_domain(uv[1], closed_a.1, va)];
    let in_b = |uv: [f64; 2]| [into_domain(uv[0], closed_b.0, ub), into_domain(uv[1], closed_b.1, vb)];
    let near_a = |uv: [f64; 2], near: [f64; 2]| [unwrap(uv[0], near[0], closed_a.0, ua), unwrap(uv[1], near[1], closed_a.1, va)];
    let near_b = |uv: [f64; 2], near: [f64; 2]| [unwrap(uv[0], near[0], closed_b.0, ub), unwrap(uv[1], near[1], closed_b.1, vb)];
    let frame = |point: Vec3, along: Vec3| -> Result<Option<Station3>, String> {
        let (Ok(on_a), Ok(on_b)) = (project_point_to_surface(first, point), project_point_to_surface(second, point)) else {
            return Ok(None);
        };
        let cross = first.normal(on_a.u, on_a.v)?.cross(second.normal(on_b.u, on_b.v)?);
        if cross.length() <= 1e-9 {
            return Ok(None);
        }
        let mut tangent = cross.normalized()?;
        if tangent.dot(along) < 0.0 {
            tangent = tangent.scale(-1.0);
        }
        Ok(Some((point, tangent, [on_a.u, on_a.v], [on_b.u, on_b.v])))
    };
    // Stations closer than this are one station (a march station beside the
    // pierce seed that clips the run): a chord that short carries no direction.
    let reach = 1e-7 * (1.0 + run[0].length());
    let mut stations: Vec<Station3> = Vec::with_capacity(run.len());
    // An interior station within the weld floor of the run's first or last
    // point is that end: the assembler welds the two into one vertex, and the
    // end, clipped at a pierce, is itself placed no better than that when the
    // section meets the trim at a shallow angle. Kept, the pair is a span too
    // short for the corrector to place a midpoint on.
    let (head, tail) = (run[0], run[run.len() - 1]);
    let end_weld = end_weld_enabled();
    for (k, point) in run.iter().enumerate() {
        if end_weld
            && k > 0
            && k + 1 < run.len()
            && (point.sub(head).length() <= crate::tolerance::WELD_FLOOR || point.sub(tail).length() <= crate::tolerance::WELD_FLOOR)
        {
            continue;
        }
        if stations.last().is_some_and(|last: &Station3| last.0.sub(*point).length() <= reach) {
            // The run's own end is kept exactly; an interior repeat is dropped.
            if k + 1 < run.len() || stations.len() < 2 {
                continue;
            }
            stations.pop();
        }
        let along = run[(k + 1).min(run.len() - 1)].sub(run[k.saturating_sub(1)]);
        let marched = by_point.get(&key(*point)).and_then(|&i| Some((tangents?.get(i).copied()?, branch.params_a[i], branch.params_b[i])));
        match marched {
            Some((tangent, uv_a, uv_b)) if tangent.length() > 0.5 => stations.push((*point, tangent.normalized()?, uv_a, uv_b)),
            _ => match frame(*point, along)? {
                Some(station) => stations.push(station),
                None => { decline_3d(2); return Ok(None); }
            },
        }
    }
    if stations.len() < 2 {
        { decline_3d(4); return Ok(None); }
    }
    // Every tangent along the run, read against the chord to its first
    // neighbour farther than the reach.
    for i in 0..stations.len() {
        let point = stations[i].0;
        let ahead = stations[i + 1..].iter().find(|s| s.0.sub(point).length() > reach).map(|s| s.0.sub(point));
        let behind = stations[..i].iter().rev().find(|s| point.sub(s.0).length() > reach).map(|s| point.sub(s.0));
        let Some(chord) = ahead.or(behind) else {
            { decline_3d(10); return Ok(None); }
        };
        if stations[i].1.dot(chord) < 0.0 {
            stations[i].1 = stations[i].1.scale(-1.0);
        }
    }
    // Parameters unwrapped along the run.
    for i in 1..stations.len() {
        let (pa, pb) = (stations[i - 1].2, stations[i - 1].3);
        stations[i].2 = near_a(stations[i].2, pa);
        stations[i].3 = near_b(stations[i].3, pb);
    }
    let creases = [crease_lines(first), crease_lines(second)];
    let cubic = |a: &Station3, b: &Station3, s: f64| -> Vec3 {
        let h = b.0.sub(a.0).length();
        let (d0, d1) = (a.1.scale(h), b.1.scale(h));
        let c = hermite_at([a.0.x, a.0.y, a.0.z], [d0.x, d0.y, d0.z], [b.0.x, b.0.y, b.0.z], [d1.x, d1.y, d1.z], s);
        Vec3::new(c[0], c[1], c[2])
    };
    let cubic_tangent = |a: &Station3, b: &Station3, s: f64| -> Vec3 {
        let (p, q) = (cubic(a, b, (s - 1e-6).max(0.0)), cubic(a, b, (s + 1e-6).min(1.0)));
        q.sub(p)
    };
    // The span's parameter velocities, each read just inside the span.
    let velocities = |a: &Station3, b: &Station3| -> Result<Option<[[f64; 2]; 4]>, String> {
        let inside = |p: [f64; 2], q: [f64; 2]| [p[0] + 1e-9 * (q[0] - p[0]), p[1] + 1e-9 * (q[1] - p[1])];
        let (Ok(a_on_a), Ok(b_on_a), Ok(a_on_b), Ok(b_on_b)) = (
            parameter_velocity(first, in_a(inside(a.2, b.2)), a.1),
            parameter_velocity(first, in_a(inside(b.2, a.2)), b.1),
            parameter_velocity(second, in_b(inside(a.3, b.3)), a.1),
            parameter_velocity(second, in_b(inside(b.3, a.3)), b.1),
        ) else {
            return Ok(None);
        };
        Ok(Some([a_on_a, b_on_a, a_on_b, b_on_b]))
    };
    // The worst of the span's 3-D Hermite against both carriers and of each
    // parameter Hermite's image against the 3-D Hermite, at five points.
    let span_miss = |a: &Station3, b: &Station3| -> Result<f64, String> {
        let h = b.0.sub(a.0).length();
        let Some([va0, va1, vb0, vb1]) = velocities(a, b)? else {
            return Ok(f64::INFINITY);
        };
        let scale = |v: [f64; 2]| [v[0] * h, v[1] * h];
        let mut worst = 0.0f64;
        for s in [0.17, 0.33, 0.5, 0.67, 0.83] {
            let c = cubic(a, b, s);
            let pa = hermite_at(a.2, scale(va0), b.2, scale(va1), s);
            let pb = hermite_at(a.3, scale(vb0), b.3, scale(vb1), s);
            let (pa, pb) = (in_a(pa), in_b(pb));
            let on_a = first.evaluate(pa[0], pa[1])?;
            let on_b = second.evaluate(pb[0], pb[1])?;
            let foot_a = crate::project_point_to_surface_seeded(first, c, pa[0], pa[1]).map_or(f64::INFINITY, |p| p.distance);
            let foot_b = crate::project_point_to_surface_seeded(second, c, pb[0], pb[1]).map_or(f64::INFINITY, |p| p.distance);
            worst = worst.max(on_a.sub(c).length()).max(on_b.sub(c).length()).max(foot_a).max(foot_b);
        }
        Ok(worst)
    };
    // The crease a span crosses first, as (carrier, direction, value), if any.
    let crossed = |a: &Station3, b: &Station3| -> Option<(usize, usize, f64)> {
        let mut best: Option<(f64, (usize, usize, f64))> = None;
        for (carrier, (p, q)) in [(a.2, b.2), (a.3, b.3)].into_iter().enumerate() {
            for direction in 0..2 {
                let (x0, x1) = (p[direction], q[direction]);
                let band = 1e-10 * (1.0 + x0.abs().max(x1.abs()));
                for &line in &creases[carrier][direction] {
                    if (line - x0) * (line - x1) < 0.0 && (line - x0).abs() > band && (line - x1).abs() > band {
                        let fraction = (line - x0) / (x1 - x0);
                        if best.is_none_or(|(f, _)| fraction < f) {
                            best = Some((fraction, (carrier, direction, line)));
                        }
                    }
                }
            }
        }
        best.map(|(_, crease)| crease)
    };
    // The intersection point of the span on a crease line: the plane-
    // constrained corrector at the 3-D Hermite's point s, s moved by secant
    // until the carrier's parameter reads the line.
    let on_crease = |a: &Station3, b: &Station3, (carrier, direction, line): (usize, usize, f64)| -> Result<Option<Station3>, String> {
        let read = |station: &Station3| if carrier == 0 { station.2[direction] } else { station.3[direction] };
        let (x0, x1) = (read(a), read(b));
        let lerp = |p: [f64; 2], q: [f64; 2], s: f64| [p[0] + (q[0] - p[0]) * s, p[1] + (q[1] - p[1]) * s];
        let at = |s: f64| -> Result<Option<Station3>, String> {
            let predicted = cubic(a, b, s);
            let direction3 = cubic_tangent(a, b, s).normalized()?;
            let Some((point, uv_a, uv_b)) = crate::surface_surface_intersection::correct_in_plane(first, second, lerp(a.2, b.2, s), lerp(a.3, b.3, s), predicted, direction3, budget * 0.1)? else {
                return Ok(None);
            };
            let cross = first.normal(uv_a[0], uv_a[1])?.cross(second.normal(uv_b[0], uv_b[1])?);
            if cross.length() <= 1e-9 {
                return Ok(None);
            }
            let tangent = if cross.dot(direction3) < 0.0 { cross.normalized()?.scale(-1.0) } else { cross.normalized()? };
            Ok(Some((point, tangent, near_a(uv_a, a.2), near_b(uv_b, a.3))))
        };
        let (mut s0, mut f0, mut s1, mut f1) = (0.0, x0 - line, 1.0, x1 - line);
        let mut s = f0 / (f0 - f1);
        for _ in 0..40 {
            let Some(station) = at(s)? else { return Ok(None) };
            let f = read(&station) - line;
            if f.abs() <= 1e-13 * (1.0 + line.abs()) {
                let mut station = station;
                if carrier == 0 { station.2[direction] = line } else { station.3[direction] = line }
                return Ok(Some(station));
            }
            if f.signum() == f0.signum() { s0 = s; f0 = f } else { s1 = s; f1 = f }
            let secant = s0 + (s1 - s0) * f0 / (f0 - f1);
            s = if secant > s0 && secant < s1 { secant } else { 0.5 * (s0 + s1) };
        }
        Ok(None)
    };
    let mut pending: Vec<(Station3, Station3, usize)> = stations.windows(2).rev().map(|pair| (pair[0], pair[1], 0usize)).collect();
    let mut out: Vec<Station3> = vec![stations[0]];
    let mut inserted = 0usize;
    while let Some((a, b, depth)) = pending.pop() {
        if inserted >= 4096 {
            { decline_3d(5); return Ok(None); }
        }
        if let Some(crease) = crossed(&a, &b) {
            let Some(station) = on_crease(&a, &b, crease)? else {
                { decline_3d(11); return Ok(None); }
            };
            inserted += 1;
            pending.push((station, b, depth));
            pending.push((a, station, depth));
            continue;
        }
        let worst = span_miss(&a, &b)?;
        if worst <= budget {
            out.push(b);
            continue;
        }
        let chord_vector = b.0.sub(a.0);
        let chord = chord_vector.length();
        if depth >= 24 || chord <= 4.0 * crate::KNOT_IDENTITY_TOL {
            { decline_3d(5); return Ok(None); }
        }
        let predicted = cubic(&a, &b, 0.5);
        let direction = a.1.add(b.1).normalized()?;
        let lerp = |p: [f64; 2], q: [f64; 2]| [0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1])];
        let Some((mid, uv_a, uv_b)) = crate::surface_surface_intersection::correct_in_plane(first, second, lerp(a.2, b.2), lerp(a.3, b.3), predicted, direction, budget * 0.1)? else {
            { decline_3d(6); return Ok(None); }
        };
        let along = mid.sub(a.0).dot(chord_vector) / (chord * chord);
        if !(0.1..=0.9).contains(&along) || mid.sub(predicted).length() > 0.25 * chord {
            { decline_3d(7); return Ok(None); }
        }
        let cross = first.normal(uv_a[0], uv_a[1])?.cross(second.normal(uv_b[0], uv_b[1])?);
        if cross.length() <= 1e-9 {
            { decline_3d(8); return Ok(None); }
        }
        let mut tangent = cross.normalized()?;
        if tangent.dot(direction) < 0.0 {
            tangent = tangent.scale(-1.0);
        }
        let station = (mid, tangent, near_a(uv_a, a.2), near_b(uv_b, a.3));
        inserted += 1;
        pending.push((station, b, depth + 1));
        pending.push((a, station, depth + 1));
    }
    let mut chords = Vec::with_capacity(out.len() - 1);
    let mut span_velocities = Vec::with_capacity(out.len() - 1);
    for pair in out.windows(2) {
        let chord = pair[1].0.sub(pair[0].0).length();
        if !(chord.is_finite() && chord > crate::KNOT_IDENTITY_TOL) {
            { decline_3d(9); return Ok(None); }
        }
        let Some(velocity) = velocities(&pair[0], &pair[1])? else {
            { decline_3d(12); return Ok(None); }
        };
        chords.push(chord);
        span_velocities.push(velocity);
    }
    let points: Vec<[f64; 3]> = out.iter().map(|s| [s.0.x, s.0.y, s.0.z]).collect();
    let velocity: Vec<[f64; 3]> = out.iter().map(|s| [s.1.x, s.1.y, s.1.z]).collect();
    let curve = hermite_spline(&points, &velocity, &chords)?;
    let uv_a: Vec<[f64; 2]> = out.iter().map(|s| s.2).collect();
    let uv_b: Vec<[f64; 2]> = out.iter().map(|s| s.3).collect();
    let pcurve_a = hermite_spline_split(&uv_a, &span_velocities.iter().map(|v| [v[0], v[1]]).collect::<Vec<_>>(), &chords)?;
    let pcurve_b = hermite_spline_split(&uv_b, &span_velocities.iter().map(|v| [v[2], v[3]]).collect::<Vec<_>>(), &chords)?;
    Ok(Some((curve, [pcurve_a, pcurve_b])))
}

/// [`hermite_spline`] with each span's own end velocities (`[start, end]`),
/// so the curve is C0 where they differ at a station.
fn hermite_spline_split<const N: usize>(points: &[[f64; N]], velocity: &[[[f64; N]; 2]], chords: &[f64]) -> Result<NurbsCurve, String> {
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
    let lift = |value: [f64; N]| Vec4 {
        x: value[0],
        y: if N > 1 { value[1] } else { 0.0 },
        z: if N > 2 { value[2] } else { 0.0 },
        w: 1.0,
    };
    let mut controls = Vec::with_capacity(2 * n);
    for i in 0..n - 1 {
        let h = chords[i];
        let mut a = points[i];
        let mut b = points[i + 1];
        for k in 0..N {
            a[k] += h * velocity[i][0][k] / 3.0;
            b[k] -= h * velocity[i][1][k] / 3.0;
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
    // Local correction and repeated carrier checks have a setup cost. Short
    // runs already fit cheaply; leave them on that route before allocating a
    // station map or projecting inserted endpoints. This is a work threshold,
    // not an accuracy tolerance, and never alters the topology-bearing trace.
    if run.len() < carried_uv_min_stations() {
        return Ok(Err(HermiteDecline::NotAmortized));
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
    let stations = if emission_refine_enabled() {
        use crate::surface_surface_intersection::{EmissionSample, refine_emission_samples};
        let samples: Vec<_> = deduped.iter().map(|s| EmissionSample {
            point: s.point, uv_a: s.uv_a, uv_b: s.uv_b, tangent: s.tangent,
        }).collect();
        let Some(refined) = refine_emission_samples(first, second, &samples, budget).ok().flatten() else {
            return Ok(Err(HermiteDecline::OverBudget));
        };
        if profile_enabled() {
            profile_line!("imprint.emission_refine input={} output={}", samples.len(), refined.len());
        }
        // Dense input traces already fit cheaply. Reparameterizing them can
        // also perturb subsequent blends without buying the expensive pcurve
        // bypass. Require at least one added station per input span on average
        // before replacing the fit route. Keep the original accuracy floor.
        if refined.len() < samples.len().saturating_mul(2).saturating_sub(1) {
            return Ok(Err(HermiteDecline::NotAmortized));
        }
        refined.into_iter().map(|s| Station {
            point: s.point, uv_a: s.uv_a, uv_b: s.uv_b, tangent: s.tangent, marched: false,
        }).collect()
    } else {
        deduped
    };
    if stations.len() < 2 {
        return Ok(Err(HermiteDecline::TooShort));
    }
    // Chords, and the parameter sequences unwrapped along the run.
    let mut chords = Vec::with_capacity(stations.len() - 1);
    let mut knot = 0.0;
    for pair in stations.windows(2) {
        let chord = pair[1].point.sub(pair[0].point).length();
        if !(chord.is_finite() && chord > 1e-12) {
            return Ok(Err(HermiteDecline::DegenerateChord));
        }
        let next = knot + chord;
        // The curve readers snap knots within this existing identity band.
        // Do not emit distinct spans they would subsequently read as one.
        // Keep the original fit route, without changing any march station.
        if carried_uv_enabled() && next - knot <= crate::KNOT_IDENTITY_TOL {
            return Ok(Err(HermiteDecline::DegenerateChord));
        }
        knot = next;
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
    // The carried-UV lane checks every emitted span, including spans between
    // old stations that clipping may have made nonadjacent. In the older lane,
    // every span with an inserted end was never tested by the march: the
    // span test, as the march runs it, on the Hermites about to be emitted.
    let mut deviation = if emission_refine_enabled() { 0.0 } else { branch.span_deviation.unwrap_or(0.0) };
    for i in 0..stations.len() - 1 {
        if !carried_uv_enabled() && stations[i].marched && stations[i + 1].marched {
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


