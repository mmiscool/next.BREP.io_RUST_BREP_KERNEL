use super::*;

/// How far a loop travels along its own pcurves in one parameter: nothing, one
/// period forward or back, or something this path does not handle (a partial
/// or double wind, a loop broken by a jump inside a coedge).
fn winding(travel: f64, period: f64) -> Option<i32> {
    let tolerance = 1e-3 * period.abs().max(1e-300);
    if travel.abs() < tolerance {
        Some(0)
    } else if (travel - period).abs() < tolerance {
        Some(1)
    } else if (travel + period).abs() < tolerance {
        Some(-1)
    } else {
        None
    }
}

/// Integrate `kinds` over a periodic face whose loops WIND — every loop either
/// closes in the covering plane or runs one full period in the surface's
/// periodic direction (a rim, a pole line, a wavy full-wrap cut) — or return
/// None when the face is not that shape.
///
/// Such loops bound no region of the covering plane, which is why the chord
/// polygon paths treat them by rebuilding a proxy region. This path needs no
/// region: with `G(u, v) = ∫_{v0}^{v} g(u, s) ds` the integral of `g` over
/// the material between u-winding loops is `∮ −G du` along the loops' pcurves
/// alone (Green's theorem in the covering plane; the two cuts along the seam
/// carry `G` at `u0` and `u0 + period`, equal because `g` is periodic, and
/// cancel; a collapsed pole line at `v0` contributes nothing and one at `v1`
/// the whole column). A v-winding face swaps the roles (`H(u, v) = ∫_{u0}^{u}
/// g(s, v) ds`, `∮ H dv`). Every station is a pcurve Gauss point with the
/// antiderivative taken by Gauss across the surface's knot spans, so the
/// trim is read exactly as given — no chord, no proxy, no sample count.
/// Loops that do not wind (a hole inside the band) integrate the same way.
/// The overall orientation is normalised through the area, as the polygon
/// path does, and `same_sense` enters through the weighted normal.
///
/// Singly periodic surfaces only. On a torus two winding loops bound two
/// strips — between them and across the other seam — and only the loops'
/// orientation says which one is material; normalising through the area
/// would always pick the strip between them. Those faces stay with the
/// biperiodic band analysis (`biperiodic_band_range`), which reads the
/// orientation against `same_sense` for exactly that choice.
pub(super) fn winding_band_integral(
    face: &FaceRecord,
    kinds: &[Integrand],
) -> Result<Option<Vec<f64>>, String> {
    winding_band_integral_with(face, kinds, 1)
}

/// `winding_band_integral` with every pcurve span (after the surface-knot
/// cuts) subdivided `refine` times — a convergence check for the tests; the
/// integrator itself runs at 1.
pub(super) fn winding_band_integral_with(
    face: &FaceRecord,
    kinds: &[Integrand],
    refine: usize,
) -> Result<Option<Vec<f64>>, String> {
    let (closed_u, closed_v) = face.surface.closed_directions()?;
    if closed_u == closed_v {
        return Ok(None);
    }
    let [u0, u1] = face.surface.domain_u()?;
    let [v0, v1] = face.surface.domain_v()?;
    let (mut winds_u, mut winds_v) = (false, false);
    for loop_record in &face.loops {
        let (mut travel_u, mut travel_v) = (0.0, 0.0);
        for coedge in &loop_record.coedges {
            let [t0, t1] = coedge.pcurve.domain()?;
            let start = coedge.pcurve.evaluate(t0)?;
            let end = coedge.pcurve.evaluate(t1)?;
            travel_u += end.x - start.x;
            travel_v += end.y - start.y;
        }
        match (winding(travel_u, u1 - u0), winding(travel_v, v1 - v0)) {
            (Some(0), Some(0)) => {}
            (Some(_), Some(0)) if closed_u => winds_u = true,
            (Some(0), Some(_)) if closed_v => winds_v = true,
            _ => return Ok(None),
        }
    }
    let p_is_u = match (winds_u, winds_v) {
        (true, false) => true,
        (false, true) => false,
        _ => return Ok(None),
    };
    Ok(Some(winding_loops_integral(face, kinds, p_is_u, refine)?))
}

/// The boundary integral itself: `∮ −G du` (or `∮ H dv`) over every loop of
/// `face`, `G` the antiderivative across the cross parameter from the domain
/// start, every pcurve span cut at the surface's knot levels. Orientation is
/// normalised through the area, so the result is the material between the
/// winding loops read positively. `refine` subdivides every span (a
/// convergence check); the integrators call with 1.
///
/// The caller decides that the face's loops wind in `p_is_u` and that the
/// region between them is the one wanted: [`winding_band_integral_with`]
/// for a singly periodic surface (where the choice is forced), and the
/// bi-periodic band path (`biperiodic_band_integral`) for a torus strip
/// whose rims are not iso lines, where `complement` has already been read
/// off the rims' orientation and only the strip's quadrature is needed.
pub(super) fn winding_loops_integral(
    face: &FaceRecord,
    kinds: &[Integrand],
    p_is_u: bool,
    refine: usize,
) -> Result<Vec<f64>, String> {
    let [u0, u1] = face.surface.domain_u()?;
    let [v0, v1] = face.surface.domain_v()?;
    let sign = if face.same_sense { 1.0 } else { -1.0 };
    let (u_breaks, v_breaks) = surface_breaks(&face.surface)?;
    // Along a pcurve the integrand is `G(u(t), v(t))`, and `G` changes
    // polynomial piece wherever `u(t)` crosses one of the SURFACE's u-knots
    // (a rational sphere's four arcs, say); the pcurve's own knots know
    // nothing of those. A single-span pole line or rim over four surface
    // spans read by one Gauss rule was 0.3% off — so every span is cut at
    // the crossings too.
    let (level_breaks, level_period) = if p_is_u { (&u_breaks, u1 - u0) } else { (&v_breaks, v1 - v0) };
    // Every requested kind plus the area, which fixes the orientation.
    let all_kinds: Vec<Integrand> = kinds
        .iter()
        .copied()
        .chain(std::iter::once(Integrand::Area))
        .collect();
    let mut totals = vec![0.0f64; all_kinds.len()];
    let mut inner = vec![0.0f64; all_kinds.len()];
    // The antiderivative crosses the same knot spans at every station; its
    // weight roots are read once per cell here.
    let mut rules = rule::SurfaceRules::new(&face.surface)?;
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            let mut spans = split_at_surface_breaks(&coedge.pcurve, curve_breaks(&coedge.pcurve)?, level_breaks, level_period, p_is_u, true)?;
            if refine > 1 {
                let coarse = std::mem::take(&mut spans);
                for pair in coarse.windows(2) {
                    for k in 0..refine {
                        spans.push(pair[0] + (pair[1] - pair[0]) * k as f64 / refine as f64);
                    }
                }
                spans.push(*coarse.last().unwrap());
            }
            for pair in spans.windows(2) {
                // The pcurve's own weights decide this span's panels and
                // order; the antiderivative across the cross parameter asks
                // the SURFACE's weights the same question.
                for panel in rule::curve_panels(&coedge.pcurve, pair[0], pair[1])? {
                    for (parameter, weight) in panel.stations() {
                        let (uv, tangent) = coedge.pcurve.deriv1(parameter)?;
                        if p_is_u {
                            antiderivative(face, &all_kinds, uv.x, v0, uv.y, &v_breaks, sign, true, &mut inner, &mut rules)?;
                            for (total, value) in totals.iter_mut().zip(&inner) {
                                *total -= weight * tangent.x * value;
                            }
                        } else {
                            antiderivative(face, &all_kinds, uv.y, u0, uv.x, &u_breaks, sign, false, &mut inner, &mut rules)?;
                            for (total, value) in totals.iter_mut().zip(&inner) {
                                *total += weight * tangent.y * value;
                            }
                        }
                    }
                }
            }
        }
    }
    let orientation = totals[kinds.len()].signum();
    Ok(totals[..kinds.len()].iter().map(|v| v * orientation).collect())
}

/// The pcurve's spans cut further wherever its `u` (`take_u`) or `v`
/// coordinate crosses one of the surface's knot levels, taken modulo the
/// period so a pcurve carried beyond the domain (a crease crossing the seam)
/// meets the same levels. Crossings are bracketed on 32 samples per span
/// and bisected; a segment lying on a level is not a crossing.
///
/// `cut_on_level` decides what a sample sitting exactly ON a level is. For
/// this module's integrand it is a cut: the antiderivative `G` is taken
/// ACROSS the surface's knot spans, so a trim lying on a level changes piece
/// there. For a caller whose integrand is a function of the point and the
/// tangent alone — `brep/soundness.rs`'s vector-area closure — it is not:
/// nothing about the integrand changes ALONG a level, and cutting there turns
/// one iso-line trim into thirty-one spans that integrate to what one span
/// already read.
///
/// Both readings still cut where the trim CROSSES a level, which is the cut
/// that carries the accuracy: the surface's derivative jumps across a knot
/// line, so a Gauss rule read straight through one has a kink inside it.
/// Measured on the notched cap's `F7` (`vector_area_census document`, 18
/// faces), the three readings of the same shell:
///
/// ```text
/// no cut at a knot at all      residual 4.941978e-6   error bar 6.820e-7      0.72 s
/// cut at every on-level sample residual 5.106706e-6   error bar 4.196e-10   807,012 spans, 9.37 s
/// this, the lying-on skipped   residual 5.106706e-6   error bar 4.204e-10    29,428 spans, 0.34 s
/// ```
///
/// The residual is the same number to seven figures either way it is cut, and
/// the error bar under it is the same 4.2e-10; what the skip drops is 27x the
/// spans and 27x the wall clock. Without any cut the residual is only seven
/// times its own error bar, which is not a measurement.
pub(crate) fn split_at_surface_breaks(
    pcurve: &NurbsCurve,
    spans: Vec<f64>,
    levels: &[f64],
    period: f64,
    take_u: bool,
    cut_on_level: bool,
) -> Result<Vec<f64>, String> {
    let interior: Vec<f64> = levels[1..levels.len().saturating_sub(1)].to_vec();
    let coordinate = |t: f64| -> Result<f64, String> {
        let uv = pcurve.evaluate(t)?;
        Ok(if take_u { uv.x } else { uv.y })
    };
    let mut out = Vec::with_capacity(spans.len() * 2);
    for pair in spans.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        out.push(a);
        if b - a <= 0.0 {
            continue;
        }
        const SAMPLES: usize = 32;
        // Taken up front because the on-level decision below reads the NEXT
        // sample as well as the previous one.
        let mut sampled = Vec::with_capacity(SAMPLES + 1);
        for k in 0..=SAMPLES {
            sampled.push(coordinate(a + (b - a) * k as f64 / SAMPLES as f64)?);
        }
        let mut previous_t = a;
        let mut previous_c = sampled[0];
        let mut cuts = Vec::new();
        for k in 1..=SAMPLES {
            let t = a + (b - a) * k as f64 / SAMPLES as f64;
            let c = sampled[k];
            let (lo, hi) = (previous_c.min(c), previous_c.max(c));
            // Every level image `l + n·period` inside (lo, hi).
            let first = ((lo - levels[0]) / period).floor() as i64 - 1;
            let last = ((hi - levels[0]) / period).ceil() as i64 + 1;
            for n in first..=last {
                for &l in interior.iter().chain(std::iter::once(&levels[0])) {
                    let level = l + n as f64 * period;
                    if c == level {
                        // A sample sitting on the level (a pole line's quarter
                        // points on 32 samples) is the crossing itself; the
                        // span ends are cut already.
                        //
                        // Which of the two readings applies is the caller's
                        // `cut_on_level`. For the antiderivative every such
                        // sample is a piece boundary. For a boundary integral
                        // only a sample where the curve LEAVES the level is —
                        // a neighbour off the level — so a trim lying along an
                        // iso-line takes no cut at all while one that crosses
                        // at a sample still takes its one. Without that second
                        // clause the crossing would be lost entirely: this
                        // sample pushes nothing and the next one sees
                        // `previous_c == level` and skips the bisection.
                        let leaves_the_level = previous_c != level
                            || sampled.get(k + 1).map(|next| *next != level).unwrap_or(true);
                        if k < SAMPLES && (cut_on_level || leaves_the_level) {
                            cuts.push(t);
                        }
                    } else if previous_c != level && (previous_c - level) * (c - level) < 0.0 {
                        let (mut t0, mut t1, mut c0) = (previous_t, t, previous_c);
                        for _ in 0..40 {
                            let tm = 0.5 * (t0 + t1);
                            let cm = coordinate(tm)?;
                            if (c0 - level) * (cm - level) <= 0.0 {
                                t1 = tm;
                            } else {
                                t0 = tm;
                                c0 = cm;
                            }
                        }
                        cuts.push(0.5 * (t0 + t1));
                    }
                }
            }
            previous_t = t;
            previous_c = c;
        }
        cuts.sort_by(|x, y| x.partial_cmp(y).unwrap());
        for cut in cuts {
            if cut - out.last().copied().unwrap_or(a) > 1e-12 * (b - a) && b - cut > 1e-12 * (b - a) {
                out.push(cut);
            }
        }
    }
    if let Some(&last) = spans.last() {
        out.push(last);
    }
    Ok(out)
}

/// `∫_{from}^{to} g(fixed, s) ds` (or `g(s, fixed)` when `across_v` is false)
/// for every kind, Gauss–Legendre over the surface's knot spans clipped to
/// the range; a station past the domain wraps through the periodic extension.
#[allow(clippy::too_many_arguments)]
pub(super) fn antiderivative(
    face: &FaceRecord,
    kinds: &[Integrand],
    fixed: f64,
    from: f64,
    to: f64,
    breaks: &[f64],
    sign: f64,
    across_v: bool,
    out: &mut [f64],
    rules: &mut rule::SurfaceRules,
) -> Result<(), String> {
    for value in out.iter_mut() {
        *value = 0.0;
    }
    let (lo, hi) = (from.min(to), from.max(to));
    if hi - lo <= 0.0 {
        return Ok(());
    }
    let direction = if to >= from { 1.0 } else { -1.0 };
    let eps = 1e-9 * (hi - lo);
    let mut edges = vec![lo];
    for &b in breaks {
        if b > lo + eps && b < hi - eps {
            edges.push(b);
        }
    }
    edges.push(hi);
    for pair in edges.windows(2) {
        let panels = rules.direction_panels(&face.surface, !across_v, pair[0], pair[1], fixed)?;
        for panel in panels {
            for (s, span_weight) in panel.stations() {
                let (u, v) = if across_v { (fixed, s) } else { (s, fixed) };
                let (point, su, sv) = face.surface.deriv1_extended(u, v)?;
                let weighted_normal = su.cross(sv).scale(sign);
                let weight = span_weight * direction;
                for (slot, kind) in kinds.iter().enumerate() {
                    out[slot] += weight * integrand_value(*kind, point, weighted_normal);
                }
            }
        }
    }
    Ok(())
}
