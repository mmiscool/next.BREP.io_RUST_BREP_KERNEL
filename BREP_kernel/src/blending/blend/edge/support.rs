use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};

// ====================================================================
// OPEN edges (§6.9 transverse construction)
// ====================================================================

/// March an OPEN edge with a small overshoot past both ends (extended
/// curve/surface evaluation) so the support fits reach the end-face
/// crossings.
///
/// Returns the stations together with the INDICES of the two the march
/// snapped exactly onto the edge's start and end parameters — the sections
/// that sit in each vertex's normal plane.  Those indices are the march's own
/// bookkeeping and cannot be recovered from the stations afterwards (see
/// [`vertex_stations`]).
pub(in crate::blend) fn march_open_stations(
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    radius_at: &dyn Fn(f64) -> f64,
    overshoot_fraction: f64,
) -> Result<(Vec<Station>, [usize; 2]), KernelRefusal> {
    march_open_stations_ending(
        edge,
        first,
        second,
        radius_at,
        overshoot_fraction,
        [edge.t0, edge.t1],
        STATIONS,
        [false, false],
        [false, false],
        None,
    )
    .map(|(stations, vertex_indices, _)| (stations, vertex_indices))
}

/// [`march_open_stations`] with the two snapped stations placed at `ends`
/// instead of at the edge's own vertices.
///
/// The snapped stations are where the rows are broken, so they belong where
/// the blend actually ENDS and the marched data stops being the blend: a
/// flush join across a seam stops a stripe at the ball seated on the seam,
/// which is where its side contact runs off its carrier onto the ruled
/// continuation — the same curvature jump [`fit_open_rows`] breaks at the
/// vertex for, one section along.  `ends` must lie inside the overshot span.
///
/// `poles` marks an end whose section is a POINT ([`pole_end`]): the two mates
/// are tangent at that vertex from the ball's side, so the ball seated there
/// touches both at the vertex itself.  The station Newton is singular there —
/// both offset carriers are tangent, so every column of its Jacobian lies in
/// one plane — and past it the two contacts cross over, so such an end is
/// marched with NO overshoot and its station is the pole itself, set from the
/// edge's own pcurves rather than solved. Its end must be the edge's vertex.
///
/// `bounded_ends` permits stopping at the last regular station before a
/// singular carrier pole. Such rows are only usable after the caller verifies
/// that the real end-face intersections lie strictly inside the fitted span;
/// their clamped end indices are fit boundaries, not vertex sections.
#[allow(clippy::too_many_arguments)]
pub(in crate::blend) fn march_open_stations_ending(
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    radius_at: &dyn Fn(f64) -> f64,
    overshoot_fraction: f64,
    ends: [f64; 2],
    station_count: usize,
    poles: [bool; 2],
    bounded_ends: [bool; 2],
    refine_bar: Option<f64>,
) -> Result<(Vec<Station>, [usize; 2], StationRefinement), KernelRefusal> {
    let span = edge.t1 - edge.t0;
    let overshoot = span * overshoot_fraction;
    let [overshoot_before, overshoot_after] =
        poles.map(|pole| if pole { 0.0 } else { overshoot });
    // The overshot span, written `span + 2·overshoot` when neither end is a
    // pole: `(span + o) + o` can round an ulp away from it, and every station
    // parameter and the section step are read off this, so a march with no
    // pole stays the march it was to the bit.
    let marched = if poles == [false, false] {
        span + 2.0 * overshoot
    } else {
        span + overshoot_before + overshoot_after
    };
    for (slot, pole) in poles.into_iter().enumerate() {
        let vertex_t = if slot == 0 { edge.t0 } else { edge.t1 };
        if pole && ends[slot] != vertex_t {
            return Err(KernelRefusal::internal(
                KernelStage::Collect,
                "pole_end_vertex",
                format!(
                    "blend march: edge {}'s pole end is not at its vertex",
                    edge.id
                ),
            ));
        }
    }
    let radius_extent = [0.0, 0.5, 1.0]
        .into_iter()
        .map(|fraction| radius_at(edge.t0 + span * fraction).abs())
        .fold(0.0, f64::max);
    // The overshot span is deliberately NOT used here: `curve_model_scale`
    // clamps out-of-domain probes, so it would measure the same extent while
    // reading as if the overshoot mattered.
    let scale = march_model_scale(&edge.curve, edge.t0, edge.t1, radius_extent)?;
    crate::report_scale_migration("blend_march_open", scale, || {
        let a = edge.curve.evaluate(edge.t0).unwrap_or_default();
        let b = edge.curve.evaluate(edge.t0 + span * 0.5).unwrap_or_default();
        a.length().max(b.length()).max(1.0)
    });
    let surface1 = &first.face.surface;
    let surface2 = &second.face.surface;
    let signs = [first.rho.signum(), second.rho.signum()];
    let rho_at = |t: f64| -> [f64; 2] {
        let radius = radius_at(t);
        [signs[0] * radius, signs[1] * radius]
    };
    // Seed at the middle (the best-conditioned station), then march
    // outward to both ends so every station is seeded by its neighbour.
    let mid_index = station_count / 2;
    // Uniform parameters across the overshot span, with the two marched
    // parameters nearest the BLEND ENDS snapped to exactly t0 / t1: the
    // end stations are analytically knowable (the end radius's tangent
    // offsets on each wall), and making them interpolation points pins
    // the fitted rows THROUGH them — the end trims and any downstream
    // miter boolean read these row endpoints.  A half-step snap keeps the
    // parameter list strictly increasing.
    let (station_ts, vertex_indices) =
        open_station_parameters(edge, overshoot_fraction, ends, station_count, poles);
    let station_t = |index: usize| station_ts[index];
    // The section plane at `t` — the extended point and the edge's unit
    // tangent, the one-sided limit where the parameterization is stationary —
    // advancing past a stationary open end by the march's own chord
    // (`march_section`), so the overshoot stations seat the ball's contacts
    // beyond the vertex instead of on its section plane.
    let station_step = marched.abs() / station_count as f64;
    let section_frame = |t: f64| -> Result<(Vec3, Vec3), KernelRefusal> {
        march_section(edge, t, station_step, "blend march")
    };
    let solve_centered = |t: f64, seed: [f64; 4]| -> Result<([f64; 4], Vec3), KernelRefusal> {
        let (section_point, section_tangent) = section_frame(t)?;
        solve_station_centered(
            surface1,
            surface2,
            rho_at(t),
            seed,
            section_point,
            section_tangent,
            scale,
        )
    };
    let solve_at =
        |t: f64, seed: [f64; 4]| -> Result<[f64; 4], KernelRefusal> { solve_centered(t, seed).map(|(uv, _)| uv) };
    let mid_seed = {
        let t = station_t(mid_index).clamp(edge.t0, edge.t1);
        let uv1 = edge_uv_on_face(first.coedge, edge, t)?;
        let uv2 = edge_uv_on_face(second.coedge, edge, t)?;
        [uv1[0], uv1[1], uv2[0], uv2[1]]
    };
    // A pole's station is the vertex on both mates, read off the edge's own
    // pcurves: the ball there touches both carriers at the one point.
    let pole_index = |index: usize| {
        (poles[0] && index == 0) || (poles[1] && index == station_count)
    };
    let pole_solution = |t: f64| -> Result<[f64; 4], KernelRefusal> {
        let uv1 = edge_uv_on_face(first.coedge, edge, t)?;
        let uv2 = edge_uv_on_face(second.coedge, edge, t)?;
        Ok([uv1[0], uv1[1], uv2[0], uv2[1]])
    };
    // A pole's centre is read off its pcurve station where it can be; the
    // continuation guard skips its centre bar when it cannot (the tangency
    // system is singular there).
    let pole_station = |index: usize| -> Result<([f64; 4], Option<Vec3>), KernelRefusal> {
        let t = station_t(index);
        let uv = pole_solution(t)?;
        let center = section_frame(t).and_then(|(section_point, section_tangent)| {
            tangency_residual(surface1, surface2, rho_at(t), uv, section_point, section_tangent)
                .map(|(_, _, _, center)| center)
        });
        Ok((uv, center.ok()))
    };
    let mut solutions = vec![[0.0f64; 4]; station_count + 1];
    let mut centers: Vec<Option<Vec3>> = vec![None; station_count + 1];
    if pole_index(mid_index) {
        let (uv, center) = pole_station(mid_index)?;
        solutions[mid_index] = uv;
        centers[mid_index] = center;
    } else {
        let (uv, center) = solve_centered(station_t(mid_index), mid_seed)?;
        solutions[mid_index] = uv;
        centers[mid_index] = Some(center);
    }
    // Every other station is the CONTINUATION of the one it is seeded from:
    // an undamped Newton can converge on the far crossing of the section
    // plane, and the guard retries such a hop from a nearer seed
    // (`stations::continue_station`).  The notched crotch arc of a 20°
    // cylinder crossing hopped here at 64 stations (2026-09-26): support 2
    // moved 0.85 of a period and the centre 9.9 for a 2.1 chord.
    let hop = |left: &Continued, right: &Continued| {
        branch_hop([surface1, surface2], edge, &rho_at, left, right)
    };
    let continue_from = |solutions: &mut [[f64; 4]],
                             centers: &mut [Option<Vec3>],
                             index: usize,
                             from: usize|
     -> Result<(), KernelRefusal> {
        if pole_index(index) {
            let (uv, center) = pole_station(index)?;
            solutions[index] = uv;
            centers[index] = center;
            return Ok(());
        }
        let left = Continued { t: station_t(from), uv: solutions[from], center: centers[from] };
        let node = continue_station(&solve_centered, &hop, &left, station_t(index), 0)?;
        solutions[index] = node.uv;
        centers[index] = node.center;
        Ok(())
    };
    // Near a collapsed carrier boundary the offset surface need not have a
    // continuation past the pole. A free end may already have crossed its
    // end face before that singularity. Keep the solved interval for that
    // case only; the caller must prove both end-face crossings lie inside it.
    let (mut first_kept, mut last_kept) = (0, station_count);
    for index in (0..mid_index).rev() {
        if let Err(error) = continue_from(&mut solutions, &mut centers, index, index + 1) {
            if !bounded_ends[0] {
                return Err(error);
            }
            first_kept = index + 1;
            break;
        }
    }
    for index in mid_index + 1..=station_count {
        if let Err(error) = continue_from(&mut solutions, &mut centers, index, index - 1) {
            if !bounded_ends[1] {
                return Err(error);
            }
            last_kept = index - 1;
            break;
        }
    }
    if last_kept - first_kept < FIT_DEGREE {
        return Err(KernelRefusal::non_convergence(KernelStage::Refine,
            "bounded_march", "blend: too few regular stations before a carrier pole"));
    }
    let station_ts = station_ts[first_kept..=last_kept].to_vec();
    let vertex_indices = vertex_indices.map(|i| i.clamp(first_kept, last_kept) - first_kept);
    let solutions = solutions[first_kept..=last_kept].to_vec();
    let centers = centers[first_kept..=last_kept].to_vec();
    // LOCAL REFINEMENT (2026-09-27). Uniform stations under-sample a wall whose
    // contacts move fast in the edge parameter — a crotch tip, where the ball
    // centre moves ρ/w faster than the edge — and the rows fitted through them
    // bulge off the ball BETWEEN stations while every station is exact (a 15°
    // crossing's rails stood 1.35e-2 off their carriers at 256 uniform
    // stations). Inside the edge's own span, each interval's exact mid-span
    // station is solved and compared with the cubic its four neighbours
    // predict; where they differ by more than `refine_bar` the station is kept,
    // and the two halves are checked next round. Stations only where the wall
    // needs them: every downstream scan samples per knot.
    let mut refinement = StationRefinement::default();
    let (station_ts, vertex_indices, solutions, _centers) = match refine_bar {
        None => (station_ts, vertex_indices, solutions, centers),
        Some(bar) => {
            let contacts = |t: f64, uv: [f64; 4]| -> Result<(Vec3, Vec3), KernelRefusal> {
                let (section_point, section_tangent) = section_frame(t)?;
                let (_, p1, p2, _) = tangency_residual(
                    surface1,
                    surface2,
                    rho_at(t),
                    uv,
                    section_point,
                    section_tangent,
                )?;
                Ok((p1, p2))
            };
            let mut ts = station_ts.clone();
            let mut sols = solutions;
            let mut cens = centers;
            let mut points = ts
                .iter()
                .zip(&sols)
                .map(|(t, uv)| contacts(*t, *uv))
                .collect::<Result<Vec<_>, _>>()?;
            let end_ts = [ts[vertex_indices[0]], ts[vertex_indices[1]]];
            let (low, high) = (end_ts[0].min(end_ts[1]), end_ts[0].max(end_ts[1]));
            let pole_t = |t: f64| (poles[0] && t == ts_first(&station_ts)) || (poles[1] && t == ts_last(&station_ts));
            // At a STATIONARY end (the edge's derivative vanishes at its
            // vertex) the edge parameter does not measure the wall, so a cubic
            // in `t` mis-reads the end interval as under-sampled and keeps
            // halving it into near-coincident stations whose rows cross
            // themselves (zero_tangent_sites' stationary start: a loop that
            // crosses itself at 4 inserted stations). Like a pole, that end's
            // interval is left as marched.
            let stationary_end = end_ts.map(|t| {
                edge.curve
                    .derivatives(t.clamp(edge.t0.min(edge.t1), edge.t0.max(edge.t1)), 1)
                    .map_or(false, |derivatives| derivatives[1].normalized().is_err())
            });
            let at_stationary_end =
                |t: f64| (stationary_end[0] && t == end_ts[0]) || (stationary_end[1] && t == end_ts[1]);
            let ceiling = REFINE_STATION_FACTOR * (station_count + 1);
            let mut dirty = vec![true; ts.len().saturating_sub(1)];
            let mut previous_worst = f64::INFINITY;
            refinement.exit = StationRefineExit::RoundBudget;
            for _round in 0..REFINE_ROUNDS {
                let mut kept: Vec<(usize, Continued, (Vec3, Vec3))> = Vec::new();
                let mut worst = (0.0f64, 0.0f64);
                for interval in 0..ts.len() - 1 {
                    let (a, b) = (ts[interval], ts[interval + 1]);
                    if !dirty[interval] || b <= low || a >= high || pole_t(a) || pole_t(b) || at_stationary_end(a) || at_stationary_end(b) || ts.len() < 4 {
                        continue;
                    }
                    let middle = 0.5 * (a + b);
                    let left = Continued { t: a, uv: sols[interval], center: cens[interval] };
                    // A mid-span station that will not solve is left out: the
                    // refinement may only add stations, never a refusal.
                    let Ok(node) = continue_station(&solve_centered, &hop, &left, middle, 0) else {
                        continue;
                    };
                    let Ok(exact) = contacts(middle, node.uv) else {
                        continue;
                    };
                    let first = interval.saturating_sub(1).min(ts.len() - 4);
                    let predicted = |pick: fn(&(Vec3, Vec3)) -> Vec3| {
                        lagrange_cubic(&ts[first..first + 4], &points[first..first + 4], pick, middle)
                    };
                    let miss = predicted(|p| p.0)
                        .sub(exact.0)
                        .length()
                        .max(predicted(|p| p.1).sub(exact.1).length());
                    if miss > worst.0 {
                        worst = (miss, middle);
                    }
                    if miss > bar {
                        kept.push((interval, node, exact));
                    }
                }
                refinement.worst_miss = worst.0;
                refinement.worst_t = worst.1;
                if kept.is_empty() {
                    refinement.exit = StationRefineExit::Converged;
                    refinement.worst_miss = 0.0;
                    break;
                }
                if ts.len() + kept.len() > ceiling {
                    refinement.exit = StationRefineExit::StationCeiling;
                    break;
                }
                if !(worst.0 < 0.9 * previous_worst) {
                    refinement.exit = StationRefineExit::NoProgress;
                    break;
                }
                previous_worst = worst.0;
                let mut next_dirty = vec![false; dirty.len() + kept.len()];
                // Insert back to front so earlier indices stay valid.
                for (interval, node, exact) in kept.iter().rev() {
                    let at = interval + 1;
                    ts.insert(at, node.t);
                    sols.insert(at, node.uv);
                    cens.insert(at, node.center);
                    points.insert(at, *exact);
                    refinement.inserted += 1;
                }
                // The halves of every split interval are checked next round.
                let mut shift = 0usize;
                for (interval, _, _) in &kept {
                    let at = interval + shift;
                    next_dirty[at] = true;
                    next_dirty[at + 1] = true;
                    shift += 1;
                }
                dirty = next_dirty;
            }
            let vertex_indices = end_ts.map(|t| ts.iter().position(|x| *x == t).unwrap_or(0));
            (ts, vertex_indices, sols, cens)
        }
    };
    let station_count = station_ts.len() - 1;
    let station_t = |index: usize| station_ts[index];
    let pole_index = |index: usize| {
        (poles[0] && index == 0) || (poles[1] && index == station_count)
    };
    open_station_trace(edge, &station_ts, &solutions, |t, uv| {
        let (section_point, section_tangent) = section_frame(t)?;
        tangency_residual(surface1, surface2, rho_at(t), uv, section_point, section_tangent)
            .map(|(_, p1, p2, center)| (p1, p2, center, section_point))
    });
    // Does the wall this march would carry FOLD?  Same question the chain and
    // closed-edge marches ask (`blend/fold.rs`), asked here because this is the
    // third march in the kernel and the open-edge and network lanes are the two
    // that reach it.
    //
    // Only the stations INSIDE the edge's own domain are measured.  This march
    // overshoots both ends on extended evaluation, and the centre curve past a
    // carrier's real extent may turn any way at all — refusing on extrapolated
    // geometry would refuse walls that build.  `vertex_indices` are the two
    // stations snapped exactly onto `t0` and `t1`, so they bound the real span.
    {
        let probe = |t: f64,
                     seed: [f64; 4]|
         -> Result<([f64; 4], Vec3, Vec3, Vec3), KernelRefusal> {
            let uv = solve_at(t, seed)?;
            let (section_point, section_tangent) = section_frame(t)?;
            let (_, p1, p2, center) = tangency_residual(
                surface1,
                surface2,
                rho_at(t),
                uv,
                section_point,
                section_tangent,
            )?;
            Ok((uv, p1, p2, center))
        };
        let [low, high] = vertex_indices;
        let (low, high) = (low.min(high), low.max(high));
        // A pole is not probed: the ball has no seat past it to read a
        // curvature from, and its section is a point, which cannot fold.
        let probed: Vec<(f64, [f64; 4])> = (low..=high)
            .filter(|index| !pole_index(*index))
            .map(|index| (station_t(index), solutions[index]))
            .collect();
        let fold_radius = |t: f64| radius_at(t).abs();
        crate::blend::fold::check_wall_fold(
            &fold_radius,
            span,
            &probed,
            &probe,
            edge,
            [first.face, second.face],
            scale,
        )?;
    }
    let mut stations = Vec::with_capacity(station_count + 1);
    for (index, uv) in solutions.iter().enumerate() {
        let t = station_t(index);
        let (section_point, section_tangent) = section_frame(t)?;
        let (_, p1, p2, center) = tangency_residual(
            surface1,
            surface2,
            rho_at(t),
            *uv,
            section_point,
            section_tangent,
        )?;
        let n1 = raw_normal(surface1, uv[0], uv[1])?;
        let n2 = raw_normal(surface2, uv[2], uv[3])?;
        let cos_alpha = signs[0] * signs[1] * n1.dot(n2);
        let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
        if weight <= 1e-6 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "tangent_station",
                "blend: faces are tangent at a station (α = π)",
            ));
        }
        // At a pole the section is a point, so its tangent lines have no
        // meeting point of their own: the section's middle control point is
        // the pole too (a zero-sweep arc).
        let apex = if pole_index(index) {
            p1
        } else {
            apex_point(p1, n1, p2, n2, center)?
        };
        stations.push(Station {
            uv1: [uv[0], uv[1]],
            uv2: [uv[2], uv[3]],
            p1,
            p2,
            center,
            weight,
            apex,
        });
    }
    Ok((stations, vertex_indices, refinement))
}

fn ts_first(ts: &[f64]) -> f64 {
    ts[0]
}

fn ts_last(ts: &[f64]) -> f64 {
    ts[ts.len() - 1]
}

/// The cubic through four `(t, point)` samples, evaluated at `t`: the
/// prediction a mid-span station is checked against.
fn lagrange_cubic(
    ts: &[f64],
    samples: &[(Vec3, Vec3)],
    pick: fn(&(Vec3, Vec3)) -> Vec3,
    t: f64,
) -> Vec3 {
    let mut sum = Vec3::default();
    for i in 0..4 {
        let mut weight = 1.0;
        for j in 0..4 {
            if i != j {
                weight *= (t - ts[j]) / (ts[i] - ts[j]);
            }
        }
        sum = sum.add(pick(&samples[i]).scale(weight));
    }
    sum
}

/// The section parameters of an open march, as [`march_open_stations_ending`]
/// lays them out: uniform across the overshot span (no overshoot past a pole
/// end), with the two parameters nearest `ends` snapped exactly onto them.
/// Returns the parameters and the indices of the two snapped stations.  A
/// test reading a march's stations against their own section points uses
/// this rather than re-deriving the layout.
pub(in crate::blend) fn open_station_parameters(
    edge: &EdgeRecord,
    overshoot_fraction: f64,
    ends: [f64; 2],
    station_count: usize,
    poles: [bool; 2],
) -> (Vec<f64>, [usize; 2]) {
    let span = edge.t1 - edge.t0;
    let overshoot = span * overshoot_fraction;
    let [overshoot_before, overshoot_after] =
        poles.map(|pole| if pole { 0.0 } else { overshoot });
    // See `march_open_stations_ending` for why `span + 2·overshoot` is
    // written as one product.
    let marched = if poles == [false, false] {
        span + 2.0 * overshoot
    } else {
        span + overshoot_before + overshoot_after
    };
    let mut ts: Vec<f64> = (0..=station_count)
        .map(|index| edge.t0 - overshoot_before + marched * index as f64 / station_count as f64)
        .collect();
    let mut snapped = [0usize; 2];
    for (slot, target) in ends.into_iter().enumerate() {
        let mut nearest = 0usize;
        for (index, t) in ts.iter().enumerate() {
            if (t - target).abs() < (ts[nearest] - target).abs() {
                nearest = index;
            }
        }
        ts[nearest] = target;
        snapped[slot] = nearest;
    }
    (ts, snapped)
}

/// `BREP_BLEND_STATION_TRACE=1` prints every solved open-march station, the
/// same way `chain/march.rs::station_trace` prints a chain's: the two contacts,
/// the ball centre, both parameter feet and the centre's step from the previous
/// station.  The fitted rows cannot tell a station from the interpolant through
/// it, so a branch hop between two stations is only visible here.
fn open_station_trace(
    edge: &EdgeRecord,
    station_ts: &[f64],
    solutions: &[[f64; 4]],
    evaluate: impl Fn(f64, [f64; 4]) -> Result<(Vec3, Vec3, Vec3, Vec3), KernelRefusal>,
) {
    if std::env::var("BREP_BLEND_STATION_TRACE").ok().as_deref() != Some("1") {
        return;
    }
    eprintln!(
        "blend open stations: edge {} {} station(s) over t [{:.9}, {:.9}]",
        edge.id,
        solutions.len(),
        station_ts.first().copied().unwrap_or(f64::NAN),
        station_ts.last().copied().unwrap_or(f64::NAN),
    );
    let mut previous: Option<Vec3> = None;
    for (index, uv) in solutions.iter().enumerate() {
        let t = station_ts[index];
        let Ok((p1, p2, center, section)) = evaluate(t, *uv) else {
            eprintln!("  pos {index:>3} t {t:.9} (station could not be evaluated)");
            continue;
        };
        let step = previous
            .map(|point| center.sub(point).length())
            .unwrap_or(f64::NAN);
        eprintln!(
            "  pos {:>3} t {:.9} p1 ({:.9}, {:.9}, {:.9}) p2 ({:.9}, {:.9}, {:.9}) \
             c ({:.9}, {:.9}, {:.9}) s ({:.9}, {:.9}, {:.9}) uv1 ({:.9}, {:.9}) uv2 ({:.9}, {:.9}) \
             dc {:.9}",
            index, t, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, center.x, center.y, center.z,
            section.x, section.y, section.z, uv[0], uv[1], uv[2], uv[3], step,
        );
        previous = Some(center);
    }
}

/// Whether the stripe along `edge` runs out into a POLE at its `at_start` end:
/// its two mates are tangent at that vertex from the side the ball sits on, so
/// the ball seated there touches both carriers AT the vertex and the blend's
/// section shrinks to that one point.
///
/// This is the end of an edge whose dihedral closes to π along it — the seam
/// between two rounds that meet where both run tangent into a third face, the
/// crease of two tangent-meeting blends.  Read as the two offset carriers
/// meeting at the vertex: `p + ρ·n` on each mate, at the vertex, is one point
/// within `bar`.  Faces tangent from OPPOSITE sides (a knife edge) put the two
/// offsets `2ρ` apart and are not a pole.
pub(in crate::blend) fn pole_end(
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    at_start: bool,
    bar: f64,
) -> Result<bool, KernelRefusal> {
    let t = if at_start { edge.t0 } else { edge.t1 };
    let uv1 = edge_uv_on_face(first.coedge, edge, t)?;
    let uv2 = edge_uv_on_face(second.coedge, edge, t)?;
    let one = blend_offset(&first.face.surface).at(uv1[0], uv1[1], first.rho).or_refuse(KernelStage::Refine, "at")?;
    let two = blend_offset(&second.face.surface).at(uv2[0], uv2[1], second.rho).or_refuse(KernelStage::Refine, "at")?;
    Ok(one.point.sub(two.point).length() <= bar)
}

/// Clamped open interpolation of the blend rows (no wrap, no seam).
/// The row parameters of the two stations the march snapped onto the edge's
/// vertices, start then end.  `vertex_indices` comes straight from
/// [`march_open_stations`], which placed one station EXACTLY at each vertex
/// parameter; the section there is the only one that lies in the vertex's
/// normal plane, and it is what the corner/flush/cap surgery trims to.
///
/// This used to be re-derived geometrically as "the station whose ball centre
/// is nearest the vertex", on the argument that the centre path is an offset
/// of the edge and so turns its closest approach at the vertex.  That is only
/// true when the section's offset direction is constant along the edge (a
/// straight edge, or planar mates).  On a CURVED support the ball centre
/// swings as it rolls, the distance-to-vertex minimum slides off the vertex
/// station, and a NEIGHBOUR wins by a hair: on the reported box+through-hole
/// document the elliptical rim's two arcs picked stations 0.0018 apart in
/// distance but 9.0e-2 apart in space, and the two halves of one smooth rim
/// then refused each other as sitting on "different balls".
pub(in crate::blend) fn vertex_stations(
    parameters: &[f64],
    vertex_indices: [usize; 2],
) -> Option<[f64; 2]> {
    if parameters.len() < 2 || vertex_indices[0] == vertex_indices[1] {
        return None;
    }
    Some([
        *parameters.get(vertex_indices[0])?,
        *parameters.get(vertex_indices[1])?,
    ])
}

/// Fit the rows of an open march.
///
/// `vertex_indices` are the two stations [`march_open_stations`] snapped onto
/// the edge's vertices, when the caller has them.  The rows are interpolated
/// in independent pieces between them and rejoined C0 at those two stations.
///
/// # Why the fit breaks at the vertex stations
///
/// The march overshoots both ends on `derivatives_extended` and
/// `evaluate_extended`, which continue a curve along its end TANGENT and a
/// surface along its boundary TANGENT PLANE.  So on a curved carrier the
/// stations are a circle (say) up to the vertex station and a straight line
/// past it: the data is G1 there and its curvature jumps.  One cubic
/// interpolant through both rings across that jump, and the ringing lands
/// inside the edge's own span, where the corner closures read the rows.
/// Measured on the 2026-09-15 report (a sketch arc notch at r = 0.5, 64
/// stations): the notch stripe's centre row stood 1.9e-4 off tangency one
/// station inside its vertex against 2e-7 mid-span, a miter seam read against
/// it left the strip 1.4e-5 past the sharp edge, the geometrically symmetric
/// corner was built as an asymmetric one with a 1.4e-5 connector, and the
/// network's result was refused for that connector's fit.  Fitted in pieces
/// the same row reads 6.1e-7 at its worst and the seam exits on the sharp edge
/// to 8e-15.
///
/// The pieces need `FIT_DEGREE + 1` samples each; a march too short for that
/// keeps the single fit.
///
/// `extrusion` is [`extrusion_direction`] of the edge and its two mates, when
/// the caller has them: the direction both carriers are invariant along, which
/// is what makes every section of a constant-radius blend the same section.
/// How far a fitted rail stands off the carrier it is tangent to, read
/// BETWEEN the stations it was fitted through.
///
/// The rows are a cubic through the march's stations, so at a station the rail
/// is on its carrier by construction and the reading has to be taken between
/// them — at 0.25, 0.5 and 0.75 of every span, the fit record's between-station
/// sampling, the same places [`crate::blend::is_marched_fit_off_carriers`] reads a
/// marched seam.
///
/// This is the term that decides a filleted solid's volume, and it is not the
/// one any gate was watching.  Measured on the notched cap 2026-09-16, the
/// lobe's 305-degree coaxial stripe against its cylinder, by THIS function (3
/// points per span, in range): 1.419e-6 at 64 stations, where the solid reads
/// -1.618e-5 against its closed form; 3.534e-7 at 128, -1.723e-6; 3.019e-8 at
/// 256, -1.127e-6.  The five straight stripes in the same solid read 7e-15 at
/// every count, because the exact-extrusion lane makes them exact rather than
/// fitted.
///
/// `exact_extrusion_probe closure` reads the same rail as 1.360e-6 / 1.329e-7 /
/// 6.29e-8, because it samples the TRIMMED edge at 129 evenly spaced points
/// rather than three per station span.  Same rail, two instruments; the
/// ladder's numbers are the ones its bar is compared with.
///
/// The reading keeps falling as the march refines, so it has no floor worth
/// naming at these counts; what caps the climb is cost, not the reading
/// ([`crate::blend::stations::MAX_OPEN_STATIONS`]).
pub(in crate::blend) fn rails_off_carriers(
    rows: &FittedRows,
    parameters: &[f64],
    carriers: [&NurbsSurface; 2],
) -> Result<f64, KernelRefusal> {
    // Only BETWEEN the vertex stations.  The march deliberately overshoots
    // past both vertices onto the carriers' tangent continuation, so out there
    // the rail is off the carrier by construction and by a lot -- the notched
    // cap's seven stripes read 0.14 to 1.4 with the overshoot included, on
    // stripes whose in-range rails are 7e-15.  That span is not the blend.
    let [low, high] = match rows.vertex_stations {
        Some([a, b]) if a < b => [a, b],
        Some([a, b]) => [b, a],
        None => [
            *parameters.first().unwrap_or(&0.0),
            *parameters.last().unwrap_or(&1.0),
        ],
    };
    let mut worst = 0.0f64;
    for (rail, surface) in [(&rows.cr, carriers[0]), (&rows.cs, carriers[1])] {
        for pair in parameters.windows(2) {
            if pair[1] <= low || pair[0] >= high {
                continue;
            }
            for fraction in [0.25, 0.5, 0.75] {
                let u = pair[0] + (pair[1] - pair[0]) * fraction;
                if u < low || u > high {
                    continue;
                }
                let point = rail.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?;
                let off = crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?.distance;
                if !worst.is_nan() && (off.is_nan() || off > worst) {
                    worst = off;
                }
            }
        }
    }
    Ok(worst)
}

pub(in crate::blend) fn fit_open_rows(
    stations: &[Station],
    parameters: &[f64],
    chamfer: bool,
    vertex_indices: Option<[usize; 2]>,
    extrusion: Option<Vec3>,
) -> Result<FittedRows, KernelRefusal> {
    let mut samples_cr = Vec::new();
    let mut samples_mid = Vec::new();
    let mut samples_cs = Vec::new();
    let mut samples_uv1 = Vec::new();
    let mut samples_uv2 = Vec::new();
    let mut samples_center = Vec::new();
    for station in stations {
        samples_cr.push(Vec4::from_point(station.p1, 1.0));
        samples_cs.push(Vec4::from_point(station.p2, 1.0));
        samples_center.push(Vec4::from_point(station.center, 1.0));
        samples_mid.push(Vec4 {
            x: station.apex.x * station.weight,
            y: station.apex.y * station.weight,
            z: station.apex.z * station.weight,
            w: station.weight,
        });
        samples_uv1.push(Vec4::from_point(
            Vec3::new(station.uv1[0], station.uv1[1], 0.0),
            1.0,
        ));
        samples_uv2.push(Vec4::from_point(
            Vec3::new(station.uv2[0], station.uv2[1], 0.0),
            1.0,
        ));
    }
    let cr_pcurve = fit::interpolate_homogeneous(&samples_uv1, FIT_DEGREE, parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
    let cs_pcurve = fit::interpolate_homogeneous(&samples_uv2, FIT_DEGREE, parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
    // Rigidly translated sections admit exact extrusion rows, preserving an
    // analytic carrier instead of approximating it with a cubic fitted surface.
    if let Some(rows) =
        exact_extrusion_rows(stations, parameters, chamfer, extrusion, cr_pcurve.clone(), cs_pcurve.clone())?
    {
        return Ok(rows);
    }
    let breaks = row_breaks(stations.len(), vertex_indices);
    let interpolate = |samples: &[Vec4]| interpolate_in_pieces(samples, parameters, &breaks);
    let (cr_pcurve, cs_pcurve) = if breaks.is_empty() {
        (cr_pcurve, cs_pcurve)
    } else {
        (interpolate(&samples_uv1)?, interpolate(&samples_uv2)?)
    };
    let cr = interpolate(&samples_cr)?;
    let cs = interpolate(&samples_cs)?;
    let mid = if chamfer {
        None
    } else {
        Some(interpolate(&samples_mid)?)
    };
    let u_domain = cr.domain().or_refuse(KernelStage::Refine, "domain")?;
    let surface = crate::blend::rows::surface_from_rows(FIT_DEGREE, &cr, &cs, mid.as_ref(), false)?;
    let center = interpolate(&samples_center)?;
    Ok(FittedRows {
        surface,
        cr,
        cs,
        cr_pcurve,
        cs_pcurve,
        u_domain,
        center: Some(center),
        exact_extrusion: false,
        vertex_stations: None,
    })
}

/// The interior sample indices the rows break at — the vertex stations that
/// are not already an end of the march — or none when a piece between them
/// would be too short to carry the fit degree.
fn row_breaks(count: usize, vertex_indices: Option<[usize; 2]>) -> Vec<usize> {
    let Some(indices) = vertex_indices else {
        return Vec::new();
    };
    let mut breaks: Vec<usize> = indices
        .into_iter()
        .filter(|&index| index > 0 && index + 1 < count)
        .collect();
    breaks.sort_unstable();
    breaks.dedup();
    let mut bounds = Vec::with_capacity(breaks.len() + 2);
    bounds.push(0);
    bounds.extend(breaks.iter().copied());
    bounds.push(count.saturating_sub(1));
    if bounds.windows(2).all(|pair| pair[1] - pair[0] >= FIT_DEGREE) {
        breaks
    } else {
        Vec::new()
    }
}

/// Interpolate `samples` at `parameters` as one cubic per piece between the
/// sample indices in `breaks`, each clamped over its own parameter interval,
/// joined into one curve with a knot of multiplicity `FIT_DEGREE` at every
/// break.  Both neighbours interpolate the break sample, so the shared control
/// point is that sample and the curve passes through it.  No breaks is the
/// single interpolant.
fn interpolate_in_pieces(
    samples: &[Vec4],
    parameters: &[f64],
    breaks: &[usize],
) -> Result<NurbsCurve, KernelRefusal> {
    if breaks.is_empty() {
        return fit::interpolate_homogeneous(samples, FIT_DEGREE, parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous");
    }
    let mut bounds = Vec::with_capacity(breaks.len() + 2);
    bounds.push(0);
    bounds.extend(breaks.iter().copied());
    bounds.push(samples.len() - 1);
    let pieces = bounds.len() - 1;
    let mut knots: Vec<f64> = Vec::new();
    let mut control: Vec<Vec4> = Vec::new();
    for (piece, pair) in bounds.windows(2).enumerate() {
        let (from, to) = (pair[0], pair[1]);
        let (low, high) = (parameters[from], parameters[to]);
        // `interpolate_homogeneous` clamps over [0, 1], so each piece is
        // fitted on its own normalized parameters and its knots mapped back.
        let local: Vec<f64> = parameters[from..=to]
            .iter()
            .map(|parameter| (parameter - low) / (high - low))
            .collect();
        let fitted = fit::interpolate_homogeneous(&samples[from..=to], FIT_DEGREE, &local).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
        if fitted.degree != FIT_DEGREE {
            return Err(KernelRefusal::internal(
                KernelStage::Refine,
                "row_piece_short",
                "blend: a row piece is too short for the fit degree",
            ));
        }
        if piece == 0 {
            knots.extend(std::iter::repeat(low).take(FIT_DEGREE + 1));
        }
        let interior = &fitted.knots[FIT_DEGREE + 1..fitted.knots.len() - FIT_DEGREE - 1];
        knots.extend(interior.iter().map(|knot| low + knot * (high - low)));
        let multiplicity = if piece + 1 == pieces { FIT_DEGREE + 1 } else { FIT_DEGREE };
        knots.extend(std::iter::repeat(high).take(multiplicity));
        control.extend(fitted.control_points.iter().skip(usize::from(piece > 0)).copied());
    }
    NurbsCurve::new(FIT_DEGREE, knots, control).or_refuse(KernelStage::Refine, "new")
}

/// The direction a blend along `edge` between these two mates is an
/// EXTRUSION along, or `None` when the geometry does not say it is one.
///
/// It is one when the edge is a straight segment and both carriers are
/// invariant under translation along it: a plane containing it, or a cylinder
/// — a `RuledRevolution` whose two radii agree, or a general `Revolution`
/// (a partial sweep, which is what an extruded sketch arc recognizes as)
/// whose generatrix is a straight segment parallel to the axis — with its
/// axis parallel to the edge.  Every comparison is a lateral distance against
/// the kernel's `model` tolerance, the one `straight_segment` is asked with.
///
/// The question is geometric, never the representation's: an edge rejoined
/// with a redundant pole is still straight, and a quarter cylinder recognized
/// as a general revolution is still a cylinder.
pub(in crate::blend) fn extrusion_direction(
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
) -> Option<Vec3> {
    let tolerance = crate::KernelTolerances::for_scale(
        march_model_scale(&edge.curve, edge.t0, edge.t1, 0.0).ok()?,
        1e-7,
    )
    .model;
    let (start, end) = edge.curve.straight_segment(tolerance)?;
    let chord = end.sub(start);
    let direction = chord.normalized().ok()?;
    // How far `vector` leans off `axis` (a unit vector) over its own length.
    let lean = |vector: Vec3, axis: Vec3| vector.sub(axis.scale(vector.dot(axis))).length();
    let invariant = |mate: &BlendMate| -> Option<()> {
        match mate.face.surface.analytic()? {
            crate::AnalyticSurface::Plane { u_dir, v_dir, .. } => {
                let normal = u_dir.cross(*v_dir).normalized().ok()?;
                (chord.dot(normal).abs() <= tolerance).then_some(())
            }
            crate::AnalyticSurface::RuledRevolution {
                frame, rho0, rho1, ..
            } => ((rho1 - rho0).abs() <= tolerance && lean(chord, frame.axis) <= tolerance)
                .then_some(()),
            crate::AnalyticSurface::Revolution {
                frame, generatrix, ..
            } => {
                let (from, to) = generatrix.straight_segment(tolerance)?;
                (lean(to.sub(from), frame.axis) <= tolerance && lean(chord, frame.axis) <= tolerance)
                    .then_some(())
            }
            _ => None,
        }
    };
    invariant(first)?;
    invariant(second)?;
    Some(direction)
}

/// The exact rows of a translational march, or `None` when the stations are
/// not rigid translates of the first along one direction.
///
/// Exactness bar: every station's contact points, apex and centre must sit
/// within `1e-9·(1 + scale)` of the first station's translated by the
/// centre's own displacement, and the centre displacements must be collinear
/// to the same bar.  The station solver converges to `1e-11·(1 + scale)`, so
/// a genuinely straight×planar edge passes with two orders of margin, and a
/// curved carrier fails by its sagitta at the second station.
///
/// # The section weight, and the geometric door
///
/// The weight is compared too, to an absolute `1e-12`.  It is not an
/// independent quantity — it is the cosine of the section's half angle, set by
/// the contacts and the centre already held to the bar above — and on a curved
/// carrier the solve does not land it that close: the D-prism's plane ×
/// cylinder edge (`open_heal_tests::d_prism`) reads every position within
/// 1.2e-10 of its translate against a 1.2e-8 bar and the weight 6.0e-12 off, so
/// a straight edge between a plane and a cylinder took the 65-row cubic.
///
/// When the caller's `extrusion` says the carriers ARE an extrusion along the
/// edge, every section is the same section by construction, so the weight is
/// not asked of the stations: the rows are the FIRST section and that section
/// translated along the travel, and the stations remain the witness that the
/// march is one — the positions to the same bar, and the travel along the
/// extrusion.  A varying radius law fails that witness at its second station.
/// Without `extrusion`, and wherever the weights do agree, the rows are the
/// first and last stations as they always were.
fn exact_extrusion_rows(
    stations: &[Station],
    parameters: &[f64],
    chamfer: bool,
    extrusion: Option<Vec3>,
    cr_pcurve: NurbsCurve,
    cs_pcurve: NurbsCurve,
) -> Result<Option<FittedRows>, KernelRefusal> {
    let (Some(first), Some(last)) = (stations.first(), stations.last()) else {
        return Ok(None);
    };
    if stations.len() < 3 {
        return Ok(None);
    }
    let travel = last.center.sub(first.center);
    let length = travel.length();
    if !(length > 1e-12) {
        return Ok(None);
    }
    let direction = travel.scale(1.0 / length);
    let scale = stations
        .iter()
        .fold(0.0f64, |worst, station| worst.max(station.center.length()))
        .max(1.0);
    let bar = 1e-9 * (1.0 + scale);
    let mut weights_agree = true;
    for (station, parameter) in stations.iter().zip(parameters) {
        let shift = station.center.sub(first.center);
        // Collinear with the travel, at the row parameter's fraction of it.
        if shift.sub(direction.scale(shift.dot(direction))).length() > bar
            || (shift.dot(direction) - parameter * length).abs() > bar
        {
            return Ok(None);
        }
        let translate = |point: Vec3| point.add(shift);
        if station.p1.sub(translate(first.p1)).length() > bar
            || station.p2.sub(translate(first.p2)).length() > bar
            || station.apex.sub(translate(first.apex)).length() > bar
        {
            return Ok(None);
        }
        if (station.weight - first.weight).abs() > 1e-12 {
            weights_agree = false;
        }
    }
    // The last row: the last station, or the first section carried along the
    // travel when only the geometry vouches for the section.
    let (end, offset) = if weights_agree {
        (last, Vec3::default())
    } else {
        let Some(along) = extrusion else {
            return Ok(None);
        };
        if travel.sub(along.scale(travel.dot(along))).length() > bar {
            return Ok(None);
        }
        (first, travel)
    };
    // Section along v (z), spine along u: the blend's own convention.
    let section = |z_index: usize, station: &Station, offset: Vec3| -> Vec4 {
        match z_index {
            0 => Vec4::from_point(station.p1.add(offset), 1.0),
            1 if !chamfer => {
                let apex = station.apex.add(offset);
                Vec4 {
                    x: apex.x * station.weight,
                    y: apex.y * station.weight,
                    z: apex.z * station.weight,
                    w: station.weight,
                }
            }
            _ => Vec4::from_point(station.p2.add(offset), 1.0),
        }
    };
    let columns = if chamfer { 2 } else { 3 };
    let control = vec![
        (0..columns).map(|j| section(j, first, Vec3::default())).collect::<Vec<_>>(),
        (0..columns).map(|j| section(j, end, offset)).collect::<Vec<_>>(),
    ];
    let (degree_v, knots_v) = crate::blend::rows::cross_section_basis(chamfer);
    let surface = NurbsSurface::new(1, degree_v, vec![0.0, 0.0, 1.0, 1.0], knots_v, control).or_refuse(KernelStage::Refine, "new")?;
    let cr = crate::make_line(first.p1, end.p1.add(offset)).or_refuse(KernelStage::Refine, "make_line")?;
    let cs = crate::make_line(first.p2, end.p2.add(offset)).or_refuse(KernelStage::Refine, "make_line")?;
    let center = crate::make_line(first.center, end.center.add(offset)).or_refuse(KernelStage::Refine, "make_line")?;
    Ok(Some(FittedRows {
        surface,
        cr,
        cs,
        cr_pcurve,
        cs_pcurve,
        u_domain: [0.0, 1.0],
        center: Some(center),
        exact_extrusion: true,
        vertex_stations: None,
    }))
}

/// One end of an open blend: the mates' boundary edges meeting the end
/// vertex, the single end face across the corner, and where the support
/// curves cross those boundary edges.
pub(in crate::blend) struct EndSurgery {
    /// Support-curve parameters at the crossings.
    pub(in crate::blend) cr_parameter: f64,
    pub(in crate::blend) cs_parameter: f64,
    /// The boundary edges being trimmed and their crossing parameters.
    pub(in crate::blend) first_edge_id: u64,
    pub(in crate::blend) first_edge_parameter: f64,
    pub(in crate::blend) second_edge_id: u64,
    pub(in crate::blend) second_edge_parameter: f64,
    pub(in crate::blend) end_face_id: u64,
    /// New vertices at the crossings (filled during surgery).
    pub(in crate::blend) transverse_curve: NurbsCurve,
    pub(in crate::blend) transverse_blend_pcurve: NurbsCurve,
    pub(in crate::blend) transverse_end_pcurve: NurbsCurve,
    /// True when a support crossing only resolved on a looser rung of
    /// [`support_crossing_tolerances`] than the historical 1e-7 — everything
    /// built from it is NEW, so the caller gates it through
    /// [`SpokeCrossings::gate`].
    pub(in crate::blend) escalated: bool,
}

/// How one end of a stripe is closed.
///
/// A stripe that runs out on its own terminates on the single face across the
/// corner ([`EndSurgery`], the §6.9 transverse construction).  A stripe that
/// runs into a corner shared with other stripes does NOT: it stops at the
/// corner ball's station, and the closure is the corner patch sewn onto every
/// stripe that meets there.  Both occupy the same slot of the blend face's
/// loop, which is why one enum covers them.
pub(in crate::blend) enum EndPlan {
    Free(EndSurgery),
    Corner(CornerEnd),
    Miter(MiterEnd),
    Cap(CapEnd),
}

/// A stripe end closed by a CAP: the stripe stops at the plane normal to
/// the edge at its vertex. The sliver's cross-section — the blend/plane
/// intersection plus one leg on each mate back to the vertex — is filled
/// as a planar bulkhead. The edge continuing past the vertex is left
/// untouched. This is the termination a fillet gets where its edge runs smoothly into an
/// edge that is not selected (tangent chain, no propagation), and it is the
/// cutter's flush end cap built without the cutter.
///
/// The orchestrator commits the arc, the two leg edges, the leg vertices and
/// the bulkhead; the surgery references the arc (like a corner end) and
/// splices each leg into its mate's loop between the rail and the
/// continuing edge.
pub(in crate::blend) struct CapEnd {
    /// Row parameters where the cap plane cuts each rail.
    pub(in crate::blend) cr_parameter: f64,
    pub(in crate::blend) cs_parameter: f64,
    /// Rim vertices: where the cap boundary meets the first and second mate.
    pub(in crate::blend) first_vertex: u64,
    pub(in crate::blend) second_vertex: u64,
    /// The section curve, committed in the blend loop's walk direction.
    pub(in crate::blend) arc_edge_id: u64,
    pub(in crate::blend) arc_blend_pcurve: NurbsCurve,
    /// The leg on each mate, committed from the rim vertex to the sharp
    /// vertex: `(edge id, pcurve on that mate from rim to vertex)`.
    pub(in crate::blend) first_leg: (u64, NurbsCurve),
    pub(in crate::blend) second_leg: (u64, NurbsCurve),
    /// The sharp vertex the legs meet at.
    pub(in crate::blend) sharp_vertex: u64,
}

/// A stripe end that meets ONE other stripe at a vertex whose third edge
/// stays sharp (§6.9.6): the two blend surfaces run past their corner-ball
/// stations and are trimmed against each other along the seam.
///
/// Unlike a star, the two rails of this stripe stop at DIFFERENT stations —
/// the shared-face rail at the ball's tangency point, the other rail where
/// the seam leaves the strip — and the end may be closed by one edge (the
/// seam) or two (the seam and then the sibling's section on this stripe's
/// other mate, when the seam leaves the sibling first).  The orchestrator
/// commits every edge and vertex before the surgery runs; the surgery only
/// references them.
pub(in crate::blend) struct MiterEnd {
    /// Row parameters where the first and second support rows stop.
    pub(in crate::blend) cr_parameter: f64,
    pub(in crate::blend) cs_parameter: f64,
    /// Rim vertices at those stops.
    pub(in crate::blend) first_vertex: u64,
    pub(in crate::blend) second_vertex: u64,
    /// The edges closing this end, in the order the blend face's loop walks
    /// them from the first rim to the second: `(edge id, stored forward along
    /// that walk, pcurve on the blend in walk direction)`.
    pub(in crate::blend) edges: Vec<(u64, bool, NurbsCurve)>,
}

/// A stripe end that stops at a shared corner ball.
///
/// Every field is decided BEFORE any cutting, from the ball solve
/// ([`crate::blend::solve_corner_ball`]): the rows are trimmed at the station
/// where the stripe's rolling ball IS the corner ball, and the rim vertices
/// are that ball's tangency points — the same two vertices the neighbouring
/// stripes on those faces stop at, so the mate loops close with no connector
/// and nothing is left over to identify and delete.
pub(in crate::blend) struct CornerEnd {
    /// Row parameters of the corner station on the two support rows.
    pub(in crate::blend) cr_parameter: f64,
    pub(in crate::blend) cs_parameter: f64,
    /// The ball's tangency vertices on the first and second mate — created
    /// once per corner and shared by every stripe that ends there.
    pub(in crate::blend) first_vertex: u64,
    pub(in crate::blend) second_vertex: u64,
    /// The end cross-section, already committed as an `EdgeRecord` oriented
    /// the way this blend face's loop walks it (so the blend uses it
    /// `forward = true` and the corner patch uses it `forward = false`).
    pub(in crate::blend) arc_edge_id: u64,
    /// Its pcurve in the blend surface's (t, z) space, in that same walk
    /// direction.
    pub(in crate::blend) arc_blend_pcurve: NurbsCurve,
}

impl EndPlan {
    pub(in crate::blend) fn cr_parameter(&self) -> f64 {
        match self {
            EndPlan::Free(end) => end.cr_parameter,
            EndPlan::Corner(end) => end.cr_parameter,
            EndPlan::Miter(end) => end.cr_parameter,
            EndPlan::Cap(end) => end.cr_parameter,
        }
    }

    pub(in crate::blend) fn cs_parameter(&self) -> f64 {
        match self {
            EndPlan::Free(end) => end.cs_parameter,
            EndPlan::Corner(end) => end.cs_parameter,
            EndPlan::Miter(end) => end.cs_parameter,
            EndPlan::Cap(end) => end.cs_parameter,
        }
    }

    pub(in crate::blend) fn free(&self) -> Option<&EndSurgery> {
        match self {
            EndPlan::Free(end) => Some(end),
            EndPlan::Corner(_) | EndPlan::Miter(_) | EndPlan::Cap(_) => None,
        }
    }

    /// The rim vertices an already-planned (corner, miter or cap) end
    /// supplies, first mate then second.
    fn planned_rims(&self) -> Option<(u64, u64)> {
        match self {
            EndPlan::Free(_) => None,
            EndPlan::Corner(end) => Some((end.first_vertex, end.second_vertex)),
            EndPlan::Miter(end) => Some((end.first_vertex, end.second_vertex)),
            EndPlan::Cap(end) => Some((end.first_vertex, end.second_vertex)),
        }
    }

    pub(in crate::blend) fn planned_rim(&self, side_first: bool) -> Option<u64> {
        self.planned_rims()
            .map(|(first, second)| if side_first { first } else { second })
    }
}

/// What one stripe's surgery committed, for the orchestrator to splice
/// against: the ids of the two rail edges it created (a network needs them to
/// insert a miter connector between a rail and the trimmed sharp edge) and
/// the blend face.
///
/// A mate the blend CONSUMED whole is not there any more: the surgery sewed
/// the blend onto the existing edge its rail retraces and dropped the face
/// (`detect_row_coincidence`).  `consumed` says so per side, first then
/// second, and on that side the rail id is that PRE-EXISTING edge rather
/// than one this stripe created — so no later pass may look the mate up or
/// treat the rail as new.
///
/// A rail that COLLAPSED to a point (its two stops within [`consumed_band`])
/// has no edge: its id is `None`, and its two rim vertices are one vertex,
/// listed in `identified` for the orchestrator to merge once every stripe is
/// in (the other rim may be another stripe's, not yet built).
pub(in crate::blend) struct SewnStripe {
    pub(in crate::blend) cr_edge_id: Option<u64>,
    pub(in crate::blend) cs_edge_id: Option<u64>,
    pub(in crate::blend) blend_face_id: u64,
    pub(in crate::blend) consumed: [bool; 2],
    /// The farthest any rail of this stripe was snapped onto an existing
    /// vertex or edge, for [`check_snap_closure`].
    pub(in crate::blend) snap: f64,
    /// Rim vertex pairs a collapsed rail makes one vertex.
    pub(in crate::blend) identified: Vec<(u64, u64)>,
}

/// How one of the four rim points of a stripe resolves.
pub(in crate::blend) enum RimResolution {
    /// Pristine free end: a fresh vertex at the row endpoint, with the
    /// boundary edge trimmed back to it.
    Fresh,
    /// A prior fillet already consumed the whole boundary edge — reuse its far
    /// vertex and delete the boundary.
    Consumed(u64),
    /// A shared corner ball supplies the vertex.  Nothing on the mate is
    /// trimmed here: the neighbouring stripe replaces the boundary outright.
    Corner(u64),
}

impl RimResolution {
    /// The existing vertex to reuse, or `None` when a fresh one is needed.
    pub(in crate::blend) fn existing(&self) -> Option<u64> {
        match self {
            RimResolution::Fresh => None,
            RimResolution::Consumed(id) | RimResolution::Corner(id) => Some(*id),
        }
    }

    pub(in crate::blend) fn is_consumed(&self) -> bool {
        matches!(self, RimResolution::Consumed(_))
    }

    /// True when this rim trims a boundary edge of the mate face.
    pub(in crate::blend) fn trims_boundary(&self) -> bool {
        matches!(self, RimResolution::Fresh)
    }
}

/// Resolve one FREE end of a stripe: the §6.9 transverse construction on the
/// single face across the corner.
///
/// Returns the end plan together with whether both support crossings landed
/// strictly inside the fitted-row domain — the caller marches with a growing
/// overshoot until they do.
pub(in crate::blend) fn resolve_free_end(
    solid: &BrepSolid,
    edge_id: u64,
    first: &BlendMate,
    second: &BlendMate,
    rows: &FittedRows,
    vertex: u64,
    at_start: bool,
) -> Result<(EndSurgery, bool), KernelRefusal> {
    let first_face = first.face;
    let second_face = second.face;
    let first_edge_id = boundary_edge_at_vertex(solid, first_face, vertex, edge_id)?;
    let second_edge_id = boundary_edge_at_vertex(solid, second_face, vertex, edge_id)?;
    if first_edge_id == second_edge_id {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "shared_end_boundary",
            "blend: mates share their end boundary edge (unsupported)",
        ));
    }
    let end_face = end_face_id(
        solid,
        first_face.id,
        second_face.id,
        first_edge_id,
        second_edge_id,
    )?;
    let first_boundary = solid
        .edges
        .iter()
        .find(|candidate| candidate.id == first_edge_id)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "end_boundary_edge",
                "blend: end boundary edge missing",
            )
        })?;
    let second_boundary = solid
        .edges
        .iter()
        .find(|candidate| candidate.id == second_edge_id)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "end_boundary_edge",
                "blend: end boundary edge missing",
            )
        })?;
    let first_boundary_coedge = first_face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .find(|coedge| coedge.edge_id == first_edge_id)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "mate_boundary_coedge",
                "blend: mate does not use its end boundary edge",
            )
        })?;
    let second_boundary_coedge = second_face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .find(|coedge| coedge.edge_id == second_edge_id)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "mate_boundary_coedge",
                "blend: mate does not use its end boundary edge",
            )
        })?;
    let (cr_parameter, first_edge_parameter, cr_escalated) =
        match support_crossing(solid, &rows.cr, first_boundary, at_start) {
            Ok((s, t, _, escalated)) => (s, t, escalated),
            Err(_) => support_crossing_uv(
                solid,
                &rows.cr,
                &rows.cr_pcurve,
                first_boundary,
                first_boundary_coedge,
                at_start,
            )?,
        };
    let (cs_parameter, second_edge_parameter, cs_escalated) =
        match support_crossing(solid, &rows.cs, second_boundary, at_start) {
            Ok((s, t, _, escalated)) => (s, t, escalated),
            Err(_) => support_crossing_uv(
                solid,
                &rows.cs,
                &rows.cs_pcurve,
                second_boundary,
                second_boundary_coedge,
                at_start,
            )?,
        };
    // A crossing past the boundary's far vertex is not an end on this face.
    // Reject it while choosing the end construction, so a tangent continuation
    // can use its cap instead of committing to a transverse end that surgery
    // will later reject. Exact extrusions keep the existing surgery/chain
    // dispatch: a flat cap there could hide an oversized tangent-chain wall.
    // Keep the same geometric snap band as that surgery.
    for (boundary, parameter) in [
        (first_boundary, first_edge_parameter),
        (second_boundary, second_edge_parameter),
    ] {
        let (near, far) = if boundary.start_vertex_id == vertex {
            (boundary.t0, boundary.t1)
        } else {
            (boundary.t1, boundary.t0)
        };
        if !rows.exact_extrusion && (parameter - far) * (far - near) > 0.0 {
            let distance = boundary.curve.evaluate_extended(parameter)
                .or_refuse(KernelStage::Refine, "evaluate_extended")?
                .sub(boundary.curve.evaluate(far).or_refuse(KernelStage::Refine, "evaluate")?)
                .length();
            if distance > consumed_band(solid) {
                return Err(KernelRefusal::unsupported(KernelStage::Refine,
                    "end_crossing_extent", format!(
                        "blend: end crossing is {distance:.3e} past boundary edge {}'s far vertex",
                        boundary.id)));
            }
        }
    }
    // Crossings must land strictly inside the row domain [0, 1] so the
    // surgery can split the support rows.  The start end sits near 0, the
    // finish end near 1.
    let in_range = if at_start {
        cr_parameter >= RIM_MARGIN && cs_parameter >= RIM_MARGIN
    } else {
        cr_parameter <= 1.0 - RIM_MARGIN && cs_parameter <= 1.0 - RIM_MARGIN
    };
    let end_record = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == end_face)
        .ok_or_else(|| {
            KernelRefusal::internal(KernelStage::Collect, "end_face", "blend: end face missing")
        })?;
    let clamp01 = |value: f64| value.clamp(0.0, 1.0);
    let (transverse, blend_pcurve, end_pcurve) = match exact_section_transverse(
        rows,
        end_record,
        clamp01(cr_parameter),
        clamp01(cs_parameter),
    )? {
        Some(exact) => exact,
        None => transverse_curve(
            rows,
            end_record,
            clamp01(cr_parameter),
            clamp01(cs_parameter),
        )?,
    };
    Ok((
        EndSurgery {
            cr_parameter,
            cs_parameter,
            first_edge_id,
            first_edge_parameter,
            second_edge_id,
            second_edge_parameter,
            end_face_id: end_face,
            transverse_curve: transverse,
            transverse_blend_pcurve: blend_pcurve,
            transverse_end_pcurve: end_pcurve,
            escalated: cr_escalated || cs_escalated,
        },
        in_range,
    ))
}

/// The EXACT transverse curve of an exact-extrusion blend on a planar end face
/// perpendicular to the extrusion: the section itself — the surface's own
/// u-iso-curve, a rational arc — with its end-face pcurve mapped control
/// point by control point (an affine map keeps a rational curve rational).
/// `None` when the configuration is not that, and the fitted intersection
/// applies.  This is what keeps a straight×planar fillet's ends exact to the
/// mass integrator's bar instead of a cubic's chord error.
///
/// The control points go through the plane's own affine chart, UNCLAMPED.  A
/// section's middle control point is where the tangents at its two contacts
/// meet; between two planes that is the sharp corner, inside the end face, but
/// a cylinder's tangent meets the plane's past the corner, and past the end
/// face's parameter domain once the section turns far enough.  A projection
/// clamps it back onto the domain and bends the trim: the D-prism's plane ×
/// cylinder edge at r = 1.5 (a 131.8° section) put the middle control point at
/// y = 4.47 on a cap whose domain stops at y = 3, and the solid read 2.04 too
/// light while its blend face kept its exact area.
fn exact_section_transverse(
    rows: &FittedRows,
    end_face: &FaceRecord,
    cr_parameter: f64,
    cs_parameter: f64,
) -> Result<Option<(NurbsCurve, NurbsCurve, NurbsCurve)>, KernelRefusal> {
    if !rows.exact_extrusion || (cr_parameter - cs_parameter).abs() > 1e-9 {
        return Ok(None);
    }
    let Some(crate::AnalyticSurface::Plane {
        origin,
        u_dir,
        v_dir,
        u_domain,
        v_domain,
    }) = end_face.surface.analytic()
    else {
        return Ok(None);
    };
    let (a, b, c) = (u_dir.dot(*u_dir), u_dir.dot(*v_dir), v_dir.dot(*v_dir));
    let determinant = a * c - b * b;
    if determinant.abs() <= 1e-16 * (a * c).max(1.0) {
        return Ok(None);
    }
    let Ok(normal) = u_dir.cross(*v_dir).normalized() else {
        return Ok(None);
    };
    let [u0, u1] = rows.u_domain;
    let direction = rows
        .cr
        .evaluate(u1).or_refuse(KernelStage::Refine, "evaluate")?
        .sub(rows.cr.evaluate(u0).or_refuse(KernelStage::Refine, "evaluate")?)
        .normalized().or_refuse(KernelStage::Refine, "normalized")?;
    if direction.cross(normal).length() > 1e-9 {
        return Ok(None);
    }
    let section = rows.surface.iso_curve_u(cr_parameter).or_refuse(KernelStage::Refine, "iso_curve_u")?;
    let [v0, v1] = section.domain().or_refuse(KernelStage::Refine, "domain")?;
    let blend_pcurve =
        crate::sweep_topology::parameter_line(cr_parameter, v0, cr_parameter, v1).or_refuse(KernelStage::Refine, "parameter_line")?;
    let mut mapped = Vec::with_capacity(section.control_points.len());
    for control in &section.control_points {
        let offset = control.point().or_refuse(KernelStage::Refine, "point")?.sub(*origin);
        let (fu, fv) = (u_dir.dot(offset), v_dir.dot(offset));
        let u = (fu * c - fv * b) / determinant + u_domain[0];
        let v = (fv * a - fu * b) / determinant + v_domain[0];
        mapped.push(Vec4 {
            x: u * control.w,
            y: v * control.w,
            z: 0.0,
            w: control.w,
        });
    }
    let end_pcurve = NurbsCurve::new(section.degree, section.knots.clone(), mapped).or_refuse(KernelStage::Refine, "new")?;
    Ok(Some((section, blend_pcurve, end_pcurve)))
}

/// The corner of an END FACE the blend terminates on: the two boundary
/// coedges that meet at the old corner vertex, plus any DEGENERATE coedges
/// standing between them.
///
/// # Why a pole can stand between them
/// A pole is a legitimate corner.  Where the end face's apex sits ON the
/// blended edge's end vertex — revolve a profile about a line through one of
/// its own points and the cone raised on that point has its apex exactly
/// there — the face's loop reads `… first boundary, POLE, second boundary …`
/// and the pair is NOT adjacent.  The blend cuts that apex away, so the pole
/// coedges are located here and dropped by [`splice_end_corner`], and their
/// edges go with the other boundaries the surgery consumes.  With no pole the
/// walk is the plain `index + 1` this replaced.
pub(in crate::blend) struct EndCorner {
    pub(in crate::blend) loop_index: usize,
    /// The coedge the transverse curve is spliced AFTER, and the one it is
    /// spliced BEFORE, by stable id (indices shift when a sibling end shares
    /// this face).
    pub(in crate::blend) x_coedge_id: u64,
    pub(in crate::blend) y_coedge_id: u64,
    /// Whether `x` is the FIRST mate's boundary (it is `y`'s otherwise).
    pub(in crate::blend) x_is_first: bool,
    /// The pole coedges between them, in loop order, and their edges.
    pub(in crate::blend) pole_coedge_ids: Vec<u64>,
    pub(in crate::blend) pole_edge_ids: Vec<u64>,
}

/// Locate [`EndCorner`] on `face`, or `None` when the two boundary edges do
/// not meet at `corner` in any of its loops.
pub(in crate::blend) fn locate_end_corner(
    solid: &BrepSolid,
    face: &FaceRecord,
    corner: u64,
    first_edge_id: u64,
    second_edge_id: u64,
) -> Option<EndCorner> {
    let edge_of = |coedge: &CoedgeRecord| {
        solid
            .edges
            .iter()
            .find(|candidate| candidate.id == coedge.edge_id)
    };
    let traversal = |coedge: &CoedgeRecord| -> Option<(u64, u64)> {
        let edge = edge_of(coedge)?;
        Some(if coedge.forward {
            (edge.start_vertex_id, edge.end_vertex_id)
        } else {
            (edge.end_vertex_id, edge.start_vertex_id)
        })
    };
    // Only a degenerate edge AT THIS CORNER is stepped over — a pole
    // elsewhere on the loop is a boundary like any other.
    let is_pole = |coedge: &CoedgeRecord| -> bool {
        edge_of(coedge).is_some_and(|edge| {
            edge.degenerate && edge.start_vertex_id == corner && edge.end_vertex_id == corner
        })
    };
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        let count = loop_record.coedges.len();
        for index in 0..count {
            let this = &loop_record.coedges[index];
            if is_pole(this) {
                continue;
            }
            let mut pole_coedge_ids = Vec::new();
            let mut pole_edge_ids = Vec::new();
            let mut step = 1usize;
            while step < count && is_pole(&loop_record.coedges[(index + step) % count]) {
                let pole = &loop_record.coedges[(index + step) % count];
                pole_coedge_ids.push(pole.id);
                pole_edge_ids.push(pole.edge_id);
                step += 1;
            }
            if step >= count {
                continue;
            }
            let next = &loop_record.coedges[(index + step) % count];
            let this_pair = (this.edge_id, next.edge_id);
            if this_pair != (first_edge_id, second_edge_id)
                && this_pair != (second_edge_id, first_edge_id)
            {
                continue;
            }
            let (Some((_, this_end)), Some((next_start, _))) = (traversal(this), traversal(next))
            else {
                continue;
            };
            if this_end != corner || next_start != corner {
                continue;
            }
            return Some(EndCorner {
                loop_index,
                x_coedge_id: this.id,
                y_coedge_id: next.id,
                x_is_first: this.edge_id == first_edge_id,
                pole_coedge_ids,
                pole_edge_ids,
            });
        }
    }
    None
}

/// Splice the transverse coedge into a located [`EndCorner`]: drop the
/// consumed boundary coedge on either side, drop every pole coedge between
/// them, and keep the rest of the loop in order.
pub(in crate::blend) fn splice_end_corner(
    result: &mut BrepSolid,
    end_face: u64,
    corner: &EndCorner,
    transverse: CoedgeRecord,
    x_consumed: bool,
    y_consumed: bool,
) -> Result<(), KernelRefusal> {
    let face = result
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .find(|face| face.id == end_face)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "end_face_lost",
                "blend: end face lost during surgery",
            )
        })?;
    let loop_record = face
        .loops
        .get_mut(corner.loop_index)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "corner_loop_lost",
                "blend: end-face corner loop lost during surgery",
            )
        })?;
    let count = loop_record.coedges.len();
    let seg_index = loop_record
        .coedges
        .iter()
        .position(|coedge| coedge.id == corner.x_coedge_id)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "corner_coedge_lost",
                "blend: end-face corner coedge lost during surgery",
            )
        })?;
    for (offset, pole) in corner.pole_coedge_ids.iter().enumerate() {
        if loop_record.coedges[(seg_index + 1 + offset) % count].id != *pole {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "corner_pole_moved",
                "blend: end-face corner pole no longer between the boundaries",
            ));
        }
    }
    let y_index = (seg_index + 1 + corner.pole_coedge_ids.len()) % count;
    if loop_record.coedges[y_index].id != corner.y_coedge_id {
        return Err(KernelRefusal::internal(
            KernelStage::Sew,
            "corner_pair_adjacent",
            "blend: end-face corner pair no longer adjacent",
        ));
    }
    let mut transverse = Some(transverse);
    let mut rebuilt = Vec::with_capacity(count + 1);
    for index in 0..count {
        let coedge = &loop_record.coedges[index];
        if index == seg_index {
            if !x_consumed {
                rebuilt.push(coedge.clone());
            }
            rebuilt.push(transverse.take().expect("transverse spliced once"));
        } else if index == y_index {
            if !y_consumed {
                rebuilt.push(coedge.clone());
            }
        } else if !corner.pole_coedge_ids.contains(&coedge.id) {
            rebuilt.push(coedge.clone());
        }
    }
    check_loop_branches(&face.surface, &rebuilt)?;
    let loop_record = face
        .loops
        .get_mut(corner.loop_index)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "corner_loop_lost",
                "blend: end-face corner loop lost during surgery",
            )
        })?;
    loop_record.coedges = rebuilt;
    Ok(())
}

/// A rebuilt loop must walk ONE branch of a closed carrier's seam.
///
/// # The defect this is the invariant for
/// Where the blend's sliver runs past the corner into material a THIRD face
/// supplies, the §6.9 end cross-section — the genuine blend × end-carrier
/// intersection, exact to 1e-12 in 3D — meets that carrier on the far side of
/// its seam.  The captured case is a profile revolved about a line through one
/// of its own points: the end face is the cone raised on that point, its APEX
/// is the blend's own end vertex, and the arc passes BEHIND the apex, across
/// the part of the cone that lies inside the other body.  Its pcurve then
/// starts at `u = 1` where the boundary it must join ends at `u = 0` — the
/// same 3D point on the same seam, one period apart in parameter — and the
/// loop encloses the COMPLEMENT of the face: a 332-unit² cone came back as
/// 180, the part lost 158 units³ against a 15.6 closed form, and `validate`
/// saw none of it, because every endpoint still coincides in 3D.
///
/// Only a whole branch is refused (half a period or more).  The ordinary
/// junction mismatch is a fit residual — 4e-5 of the domain on that same part
/// — and the §6.9 EXTEND case, whose transverse deliberately lands past a
/// boundary's trim, keeps its endpoints on the branch it started on.
///
/// Representing what the arc actually wants needs the end face SPLIT along the
/// blend and the fragments re-classified; a NURBS carrier's parameter
/// extension does not wrap, so the loop cannot simply be unwrapped across the
/// seam. Until that lands the surgery refuses by name and the selection routes
/// to the cutter, whose boolean does split crossing faces.
fn check_loop_branches(surface: &NurbsSurface, coedges: &[CoedgeRecord]) -> Result<(), KernelRefusal> {
    let (closed_u, closed_v) = surface.closed_directions().unwrap_or((false, false));
    if !closed_u && !closed_v {
        return Ok(());
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let periods = [
        if closed_u { u1 - u0 } else { 0.0 },
        if closed_v { v1 - v0 } else { 0.0 },
    ];
    for index in 0..coedges.len() {
        let this = &coedges[index];
        let next = &coedges[(index + 1) % coedges.len()];
        let [_, this_end] = this.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let [next_start, _] = next.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let leaving = this.pcurve.evaluate(this_end).or_refuse(KernelStage::Refine, "evaluate")?;
        let arriving = next.pcurve.evaluate(next_start).or_refuse(KernelStage::Refine, "evaluate")?;
        for (axis, gap) in [
            (0usize, (leaving.x - arriving.x).abs()),
            (1, (leaving.y - arriving.y).abs()),
        ] {
            if periods[axis] > 0.0 && gap >= periods[axis] * 0.5 {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Sew,
                    "loop_crosses_seam",
                    format!(
                        "blend: the end cross-section joins the end face's loop across its SEAM \
                         (the junction jumps {gap:.6} in the closed direction, a period of \
                         {:.6}) — the blend runs past the corner into material a third face \
                         supplies, and closing it needs that face SPLIT along the blend, which \
                         the rolling-ball surgery does not do",
                        periods[axis]
                    ),
                ));
            }
        }
    }
    Ok(())
}

/// The refusal for a blend whose rail runs PAST the far edge of a face it
/// must lie on, by more than [`consumed_band`] — read at a crossing past the
/// far vertex (`build_open_surgery`) and along a row that leaves the face
/// ([`detect_row_coincidence`]).  Not terminal: the r = 10.001 pinch past an
/// earlier round reads the same way, and the composition's ridge answers it.
pub(in crate::blend) const BLEND_WIDER_THAN_FACE: &str =
    "blend: the blend is wider than the face it must lie on —";

/// The refusal for a rail that shrinks to a point where the surgery has no
/// corner to identify its two rims through.  TERMINAL in the group lane: the
/// selection is a full-width blend, and the cutter composition's answer to a
/// full-width corner the network does not build was measured wrong (a 5F/8E/5V
/// body 8.0e-4 off its closed form at 1e-5 short of the width).
pub(crate) const RAIL_COLLAPSE_UNSUPPORTED: &str =
    "blend: a full-width rail collapse is not built here —";

/// The slug of that refusal (`UnsupportedGeometry`), minted in `edge/open.rs`
/// and `network.rs`.
pub(crate) const RAIL_COLLAPSE_WHAT: &str = "rail_collapse";

/// Is this a full-width rail collapse refusal ([`RAIL_COLLAPSE_UNSUPPORTED`])?
/// Read off the class and its slug, which the mint site and this check share.
pub(crate) fn is_rail_collapse(refusal: &KernelRefusal) -> bool {
    matches!(&refusal.class, RefusalClass::UnsupportedGeometry { what } if what == RAIL_COLLAPSE_WHAT)
}

/// How far inside the fitted-row domain a support crossing must land for the
/// surgery to be able to split the rows there.
pub(in crate::blend) const RIM_MARGIN: f64 = 2.0e-3;

/// The ONE band that decides whether a blend consumes a support face whole:
/// a rail that comes within it of an existing edge is snapped onto that edge,
/// and a rail that stops further short keeps the strip between them as a face.
///
/// Both sides of that choice have a kernel number, and the band is set from
/// them rather than picked:
///
/// * NOT snapping must not build a face narrower than the solid's own
///   [`crate::KernelTolerances::sliver`] — the width below which a feature is a
///   sliver to be repaired, not a face.  So the band is at least that.
/// * Snapping must not leave the shell over its closure bar: a rail snapped
///   across δ leaves its trims δ off the edge they were sewn onto, and
///   [`crate::shell_vector_areas`] allows each trim the refinement floor.
///   That limit depends on the SHAPE (the snapped length against the whole
///   shell's edge length), so it is not read into the band; it is measured on
///   the result by [`check_snap_closure`], and a snap that breaks it refuses.
///
/// Three sites ask the question and all three read this band, because any gap
/// between them is a shape the kernel builds wrong without refusing.  Measured
/// 2026-09-17 on a 20-cube's top-front edge, when the endpoint classifier in
/// `build_open_surgery` read `1e-6·(1 + boundary length)` and
/// [`detect_row_coincidence`] read `max(1e-7, 1e-6·far-vertex separation)`:
/// at r = 20.00002 the ends were consumed (2e-5 ≤ 2.1e-5) and the row was not
/// (2e-5 > 2e-5), so both boundary edges were deleted AND a fresh rail was
/// built coincident with the far edge — a zero-width face, 6F/10E/6V, that
/// `validate()` passes.  The third site is `check_support_extent`
/// (`fillet/analyze.rs`), which refuses a blend wider than its face: a
/// refusal slack WIDER than this band admits a rail that runs past the face
/// and is neither snapped nor refused (7F/11E/6V at r = 20.0000001 with a
/// 1e-7 snap band and the old slack, and no detector flags it).
///
/// That old `1e-6·extent` band was 2.1e-5 on the 20-cube, fifty times the
/// sliver tolerance, and every snap in the upper part of it built a shell over
/// its bar: 2.8e-4 against 2.0e-5 at 1e-5 short of the width, identical on
/// the base commit through the cutter composition's lane.
pub(crate) fn consumed_band(solid: &BrepSolid) -> f64 {
    crate::KernelTolerances::for_solid(solid, 1e-7).sliver
}

/// The refusal for a snap that breaks the shell's closure — see
/// [`check_snap_closure`].
pub(crate) const CONSUMED_SNAP_UNSOUND: &str =
    "blend: the blend is within a sliver of its face's width without matching it";

/// The slug of that refusal (`IllPosed`).
pub(crate) const CONSUMED_SNAP_WHAT: &str = "consumed_snap";

/// Is this the sliver-snap refusal ([`CONSUMED_SNAP_UNSOUND`])?
/// Read off the class and its slug, which the mint site and this check share.
pub(crate) fn is_consumed_snap(refusal: &KernelRefusal) -> bool {
    matches!(&refusal.class, RefusalClass::IllPosed { what } if what == CONSUMED_SNAP_WHAT)
}

/// Accept a body whose rails were snapped across `snap` only while its shells
/// still close: a snap is a trim moved off the edge it claims, and a shell
/// whose trims no longer enclose zero vector area is a wrong body that
/// `validate()` passes.  A snap inside the refinement floor is exactly what
/// every trim is allowed, so it is not measured.
///
/// Refusing here is a statement about the WINDOW between the two rules of
/// [`consumed_band`]: this rail is too close to the far edge to keep the strip
/// as a face (narrower than the sliver tolerance) and too far to be snapped
/// onto it soundly on this shape.  The input is measured only when the result
/// fails, so a body that arrived already open is not blamed on the snap.
pub(in crate::blend) fn check_snap_closure(
    input: &BrepSolid,
    result: &BrepSolid,
    snap: f64,
) -> Result<(), KernelRefusal> {
    if !(snap > crate::pcurve::PCURVE_REFINEMENT_TOLERANCE) {
        return Ok(());
    }
    let report = crate::shell_vector_areas(result);
    if !report.is_flagged() && report.unreadable.is_empty() {
        return Ok(());
    }
    if crate::shell_vector_areas(input).is_flagged() {
        return Ok(());
    }
    let (residual, bar) = report
        .worst_shell()
        .map_or((f64::NAN, f64::NAN), |shell| (shell.residual(), shell.bar));
    Err(KernelRefusal::ill_posed(
        KernelStage::Validate,
        CONSUMED_SNAP_WHAT,
        format!(
            "{CONSUMED_SNAP_UNSOUND} — its rail ends {snap:.3e} from the face's far edge, inside \
             the {:.3e} sliver band, so the strip between them is no face; and snapped onto that \
             edge, the shell's trims close to {residual:.3e} against a bar of {bar:.3e}{}. Make \
             the size the width, or move it away from the width by more than the band",
            consumed_band(input),
            if report.unreadable.is_empty() {
                String::new()
            } else {
                format!(" ({} faces unreadable)", report.unreadable.len())
            }
        ),
    ))
}

/// A detected full-span coincidence of a blend support row with an EXISTING
/// mate-face boundary edge (OCCT PR #1449 lesson 3, the r = face-width
/// class).  The surgery sews the blend face straight onto `edge_id` instead
/// of creating a coincident fresh support edge, and drops the fully-consumed
/// mate face whose loop collapsed between the two coincident curves.
pub(in crate::blend) struct RowSew {
    /// The existing boundary edge the blend row retraces.
    pub(in crate::blend) edge_id: u64,
    /// Traversal sense the blend face's coedge on that edge must carry
    /// (equal to the dropped face's old sense, so manifold pairing with the
    /// surviving neighbour face is preserved).
    pub(in crate::blend) forward: bool,
    /// Lesson-7 leftovers of the collapsing loop: zero-span edges wholly
    /// owned by the dropped face, collapsed (deleted) with it.
    pub(in crate::blend) collapsed_edges: Vec<u64>,
    /// The largest distance from the row to the edge it was sewn onto: how
    /// far the snap moved the rail.
    pub(in crate::blend) deviation: f64,
}

/// Detect that the trimmed support `row` coincides with an existing boundary
/// edge of the mate face over that edge's FULL span, so the mate face is
/// fully consumed (OCCT PR #1449 lesson 3 — the curve-level analog of the
/// endpoint-level `classify` rule in open.rs).
///
/// Fail-safe contract: ANY ambiguity returns `Ok(None)` and the caller keeps
/// the existing surgery path (the sliver face survives, downstream validation
/// still guards it).  `Some` requires ALL of:
///
/// * both end crossings already endpoint-consumed onto DISTINCT far vertices;
/// * the mate face is single-loop (dropping a face with holes would orphan
///   them) and its loop consists of the blended edge, the two consumed
///   boundary edges, exactly ONE existing edge joining the far vertices, and
///   (lesson 7) only zero-span collapse leftovers wholly owned by this face;
/// * the row retraces that edge under the AFFINE parameter map the sewn
///   coedge will carry — sampled in 3D, the exact metric `validate`'s
///   `adaptive_coedge_error` measures (the pr1449 doc sketches this check in
///   the mate's UV space; 3D avoids the anisotropic UV-metric conversion and
///   validates the true downstream contract) — within `band`, the
///   [`consumed_band`] the caller classified both ends against, with the
///   OPPOSITE map refused;
/// * relative orientation (the parameter-delta sign of the matching map)
///   agrees with the vertex correspondence, with the dropped face's own use
///   of the edge, and with exactly one opposing use by the surviving
///   neighbour (manifold pairing).
#[allow(clippy::too_many_arguments)]
pub(in crate::blend) fn detect_row_coincidence(
    solid: &BrepSolid,
    mate: &BlendMate,
    blended_edge_id: u64,
    boundary_start_id: u64,
    boundary_finish_id: u64,
    start_consumed: bool,
    start_far_vertex: u64,
    finish_consumed: bool,
    finish_far_vertex: u64,
    row: &NurbsCurve,
    row_traversed_from_start: bool,
    band: f64,
) -> Result<Option<RowSew>, KernelRefusal> {
    if !(start_consumed && finish_consumed) {
        return Ok(None);
    }
    if start_far_vertex == finish_far_vertex {
        return Ok(None);
    }
    if mate.face.loops.len() != 1 || mate.loop_index != 0 {
        return Ok(None);
    }
    let edge_by_id = |id: u64| solid.edges.iter().find(|candidate| candidate.id == id);
    let vertex_point = |id: u64| {
        solid
            .vertices
            .iter()
            .find(|candidate| candidate.id == id)
            .map(|vertex| vertex.point)
    };
    // The ONE existing edge joining the two far vertices, plus any leftovers.
    let mut sewn_coedge: Option<&CoedgeRecord> = None;
    let mut leftovers: Vec<u64> = Vec::new();
    for coedge in &mate.face.loops[0].coedges {
        if coedge.edge_id == blended_edge_id
            || coedge.edge_id == boundary_start_id
            || coedge.edge_id == boundary_finish_id
        {
            continue;
        }
        let Some(candidate) = edge_by_id(coedge.edge_id) else {
            return Err(KernelRefusal::internal(
                KernelStage::Classify,
                "mate_loop_edge",
                "blend: mate loop references a missing edge",
            ));
        };
        let joins = (candidate.start_vertex_id == start_far_vertex
            && candidate.end_vertex_id == finish_far_vertex)
            || (candidate.start_vertex_id == finish_far_vertex
                && candidate.end_vertex_id == start_far_vertex);
        if joins {
            if sewn_coedge.is_some() {
                return Ok(None);
            }
            sewn_coedge = Some(coedge);
        } else if !leftovers.contains(&candidate.id) {
            leftovers.push(candidate.id);
        }
    }
    let Some(sewn_coedge) = sewn_coedge else {
        return Ok(None);
    };
    let Some(target) = edge_by_id(sewn_coedge.edge_id) else {
        return Ok(None);
    };
    let (Some(start_point), Some(finish_point)) = (
        vertex_point(start_far_vertex),
        vertex_point(finish_far_vertex),
    ) else {
        return Ok(None);
    };
    let extent = finish_point.sub(start_point).length();
    if extent <= band * 4.0 {
        // Far vertices too close to discriminate orientation — ambiguous.
        return Ok(None);
    }
    // Orientation by vertex correspondence; confirmed below by the matching
    // affine map (the parameter-delta sign) and the manifold senses.
    let aligned = target.start_vertex_id == start_far_vertex;
    let [row_t0, row_t1] = row.domain().or_refuse(KernelStage::Refine, "domain")?;
    // Which way is INTO the face from the far edge: toward the blended edge.
    // A row that leaves the band on the other side runs outside its face —
    // both ends read consumed only because each end's crossing is clamped to
    // the far vertex, and the pristine path would build a lens face between
    // the rail and the edge where the part has no material at all
    // (7F/11E/6V at r = 20.0000008 on a 20-cube, closure clean).
    let blended_middle = edge_by_id(blended_edge_id)
        .map(|blended| blended.curve.evaluate(0.5 * (blended.t0 + blended.t1)))
        .transpose()
        .or_refuse(KernelStage::Classify, "evaluate")?;
    const ROW_COINCIDENCE_SAMPLES: usize = 16;
    let mut deviation = 0.0f64;
    for sample in 0..=ROW_COINCIDENCE_SAMPLES {
        let fraction = sample as f64 / ROW_COINCIDENCE_SAMPLES as f64;
        let on_row = row.evaluate(row_t0 + (row_t1 - row_t0) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
        let edge_fraction = if aligned { fraction } else { 1.0 - fraction };
        let on_edge = target
            .curve
            .evaluate(target.t0 + (target.t1 - target.t0) * edge_fraction).or_refuse(KernelStage::Refine, "evaluate")?;
        let gap = on_row.sub(on_edge).length();
        if gap > band {
            let outside = blended_middle
                .is_some_and(|middle| on_row.sub(on_edge).dot(middle.sub(on_edge)) < 0.0);
            if outside {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "blend_wider_than_face",
                    format!(
                        "{BLEND_WIDER_THAN_FACE} its rail runs {gap:.3e} past face {}'s far edge \
                         {} (more than the {band:.3e} it can be snapped across), outside the face",
                        mate.face.id, target.id
                    ),
                ));
            }
            return Ok(None);
        }
        deviation = deviation.max(gap);
    }
    // The OPPOSITE map must NOT also fit — a both-ways match means the edge
    // is degenerate at this band and orientation is ambiguous.
    let quarter_row = row.evaluate(row_t0 + (row_t1 - row_t0) * 0.25).or_refuse(KernelStage::Refine, "evaluate")?;
    let opposite_fraction = if aligned { 0.75 } else { 0.25 };
    let quarter_opposite = target
        .curve
        .evaluate(target.t0 + (target.t1 - target.t0) * opposite_fraction).or_refuse(KernelStage::Refine, "evaluate")?;
    if quarter_row.sub(quarter_opposite).length() <= band {
        return Ok(None);
    }
    // Lesson 7: leftover loop edges are tolerated only as ZERO-SPAN collapse
    // edges (both ends on one vertex, no geometric extent) wholly owned by
    // the dropped face — collapsed with it, never a generic deletion.
    for id in &leftovers {
        let Some(leftover) = edge_by_id(*id) else {
            return Ok(None);
        };
        if leftover.start_vertex_id != leftover.end_vertex_id {
            return Ok(None);
        }
        let a = leftover.curve.evaluate(leftover.t0).or_refuse(KernelStage::Refine, "evaluate")?;
        let mid = leftover
            .curve
            .evaluate(leftover.t0 + (leftover.t1 - leftover.t0) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
        let b = leftover.curve.evaluate(leftover.t1).or_refuse(KernelStage::Refine, "evaluate")?;
        if a.sub(b).length() > band || a.sub(mid).length() > band {
            return Ok(None);
        }
        let used_elsewhere = solid.shells.iter().flat_map(|shell| &shell.faces).any(|face| {
            face.id != mate.face.id
                && face
                    .loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .any(|coedge| coedge.edge_id == *id)
        });
        if used_elsewhere {
            return Ok(None);
        }
    }
    // Manifold pairing: the blend's traversal of the row determines the sense
    // its coedge on the existing edge must carry; it must equal the dropped
    // face's old sense (whose partner is the surviving neighbour) and be
    // opposed by exactly one use elsewhere.
    let traversal_start = if row_traversed_from_start {
        start_far_vertex
    } else {
        finish_far_vertex
    };
    let forward = target.start_vertex_id == traversal_start;
    if sewn_coedge.forward != forward {
        return Ok(None);
    }
    let mut opposing_uses = 0usize;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if face.id == mate.face.id {
            continue;
        }
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            if coedge.edge_id != target.id {
                continue;
            }
            if coedge.forward == forward {
                return Ok(None);
            }
            opposing_uses += 1;
        }
    }
    if opposing_uses != 1 {
        return Ok(None);
    }
    Ok(Some(RowSew {
        edge_id: target.id,
        forward,
        collapsed_edges: leftovers,
        deviation,
    }))
}

/// The boundary edge of `face` that meets the blended `exclude` edge at
/// `vertex`.  Uses loop adjacency: the answer is the `exclude` coedge's
/// loop-neighbour that shares `vertex`.  This DISAMBIGUATES the case where a
/// prior fillet has left more than one edge touching `vertex` (the original
/// plus a fresh fillet/support edge) — a plain incidence scan would see
/// several and could not tell which continues this face's border.
pub(in crate::blend) fn boundary_edge_at_vertex(
    solid: &BrepSolid,
    face: &FaceRecord,
    vertex: u64,
    exclude: u64,
) -> Result<u64, KernelRefusal> {
    let edge_at = |edge_id: u64| -> Option<&EdgeRecord> {
        solid.edges.iter().find(|edge| edge.id == edge_id)
    };
    // A collapsed pole coedge carries a chart boundary but no transverse
    // curve. Skip it when locating the physical boundary of the fillet.
    // Primary: the loop-neighbour of the blended edge that shares the vertex.
    for loop_record in &face.loops {
        let count = loop_record.coedges.len();
        if count == 0 {
            continue;
        }
        for index in 0..count {
            if loop_record.coedges[index].edge_id != exclude {
                continue;
            }
            // Prefer the neighbour on the side of `vertex`, then the other.
            for step in [count - 1, 1] {
                let neighbour = &loop_record.coedges[(index + step) % count];
                if neighbour.edge_id == exclude {
                    continue;
                }
                if let Some(edge) = edge_at(neighbour.edge_id) {
                    if !edge.degenerate
                        && (edge.start_vertex_id == vertex || edge.end_vertex_id == vertex)
                    {
                        return Ok(edge.id);
                    }
                }
            }
        }
    }
    // Fallback: a plain incidence scan (used when the blended edge is not in
    // this face's loops, e.g. degenerate inputs) — accept a single unique hit.
    let mut found = None;
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            if coedge.edge_id == exclude {
                continue;
            }
            let edge = edge_at(coedge.edge_id).ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Collect,
                    "loop_edge",
                    "blend: loop references missing edge",
                )
            })?;
            if !edge.degenerate
                && (edge.start_vertex_id == vertex || edge.end_vertex_id == vertex)
            {
                if found.is_some() && found != Some(edge.id) {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Collect,
                        "boundary_edge_ambiguous",
                        "blend: multiple boundary edges at the end vertex",
                    ));
                }
                found = Some(edge.id);
            }
        }
    }
    found.ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "boundary_edge_none",
            "blend: no boundary edge at the end vertex",
        )
    })
}

pub(in crate::blend) fn end_face_id(
    solid: &BrepSolid,
    first_face: u64,
    second_face: u64,
    first_edge: u64,
    second_edge: u64,
) -> Result<u64, KernelRefusal> {
    let mut found = None;
    for shell in &solid.shells {
        for face in &shell.faces {
            if face.id == first_face || face.id == second_face {
                continue;
            }
            let uses_first = face
                .loops
                .iter()
                .flat_map(|l| &l.coedges)
                .any(|coedge| coedge.edge_id == first_edge);
            let uses_second = face
                .loops
                .iter()
                .flat_map(|l| &l.coedges)
                .any(|coedge| coedge.edge_id == second_edge);
            if uses_first && uses_second {
                if found.is_some() {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Collect,
                        "end_face_ambiguous",
                        "blend: ambiguous end face",
                    ));
                }
                found = Some(face.id);
            }
        }
    }
    found.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Collect,
            "end_face_none",
            "blend: no single end face across the corner",
        )
    })
}

/// Crossing of the fitted support pcurve with a boundary edge's pcurve in
/// the MATE FACE's parameter space — robust for free-form boundaries, and
/// capable of landing BEYOND the boundary's trim (the §6.9 "trim or
/// EXTEND" case).  Returns (support parameter, boundary EDGE parameter).
pub(in crate::blend) fn support_crossing_uv(
    solid: &BrepSolid,
    support: &NurbsCurve,
    support_pcurve: &NurbsCurve,
    boundary: &EdgeRecord,
    boundary_coedge: &CoedgeRecord,
    at_start: bool,
) -> Result<(f64, f64, bool), KernelRefusal> {
    let [s0, s1] = support_pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let [q0, q1] = boundary_coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let seed_s = if at_start { s0 } else { s1 };
    let mut solved = None;
    for seed_fraction in [0.0f64, 1.0, 0.5] {
        let mut x = [seed_s, q0 + (q1 - q0) * seed_fraction];
        let mut converged = false;
        for _ in 0..NEWTON_ITERATIONS {
            let a = support_pcurve.evaluate_extended(x[0]).or_refuse(KernelStage::Refine, "evaluate_extended")?;
            let b = boundary_coedge.pcurve.evaluate_extended(x[1]).or_refuse(KernelStage::Refine, "evaluate_extended")?;
            let residual = [a.x - b.x, a.y - b.y];
            if residual[0].abs().max(residual[1].abs()) <= 1e-12 {
                converged = true;
                break;
            }
            let step = 1e-8;
            let a_probe = support_pcurve.evaluate_extended(x[0] + step).or_refuse(KernelStage::Refine, "evaluate_extended")?;
            let b_probe = boundary_coedge.pcurve.evaluate_extended(x[1] + step).or_refuse(KernelStage::Refine, "evaluate_extended")?;
            let jacobian = [
                [(a_probe.x - a.x) / step, -(b_probe.x - b.x) / step],
                [(a_probe.y - a.y) / step, -(b_probe.y - b.y) / step],
            ];
            let determinant = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0];
            if determinant.abs() <= 1e-16 {
                break;
            }
            x[0] -= (residual[0] * jacobian[1][1] - residual[1] * jacobian[0][1]) / determinant;
            x[1] -= (residual[1] * jacobian[0][0] - residual[0] * jacobian[1][0]) / determinant;
        }
        if converged {
            // Prefer the solution nearest the support's queried end.
            let rank = (x[0] - seed_s).abs();
            if solved
                .map(|(existing_s, _): (f64, f64)| (existing_s - seed_s).abs() > rank)
                .unwrap_or(true)
            {
                solved = Some((x[0], x[1]));
            }
        }
    }
    let (s, q) = solved.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Intersect,
            "support_crossing_uv_none",
            "blend: support curve does not reach the end boundary edge",
        )
    })?;
    // The row was marched with an overshoot past both vertices, so a genuine
    // crossing always lies within its domain; a solution outside it is the
    // pcurve extrapolated off its surface image.
    let span = (s1 - s0).abs();
    if s < s0 - 1e-6 * span || s > s1 + 1e-6 * span {
        return Err(KernelRefusal::unsupported(
            KernelStage::Intersect,
            "support_crossing_uv_outside",
            format!(
                "blend: support curve does not reach the end boundary edge (the parameter-space \
                 crossing lands at {s:.4}, outside the row's [{s0:.4}, {s1:.4}])"
            ),
        ));
    }
    // Map the pcurve parameter to the boundary EDGE parameter through the
    // affine traversal contract.
    let fraction = (q - q0) / (q1 - q0);
    let fraction = if boundary_coedge.forward {
        fraction
    } else {
        1.0 - fraction
    };
    let t = boundary.t0 + (boundary.t1 - boundary.t0) * fraction;
    // A UV solution is only a crossing if the two 3D curves meet there.  A
    // rational pcurve evaluated past its domain sweeps away from its surface
    // image, so a boundary the rail never reaches — one that CONTINUES the
    // blended edge tangentially, the rail running parallel to it — can still
    // hand Newton a phantom (s, q).  The §6.9 EXTEND case, a short straight
    // boundary met just past its end, keeps its 3D point on the boundary's
    // line and passes.
    let on_support = support.evaluate_extended(s).or_refuse(KernelStage::Refine, "evaluate_extended")?;
    let on_boundary = boundary.curve.evaluate_extended(t).or_refuse(KernelStage::Refine, "evaluate_extended")?;
    // Model scale from the boundary's own extent — never from the evaluated
    // points, which an extrapolation can throw arbitrarily far and thereby
    // make any gap look small.
    let scale = boundary
        .curve
        .evaluate(boundary.t0).or_refuse(KernelStage::Refine, "evaluate")?
        .length()
        .max(boundary.curve.evaluate(boundary.t1).or_refuse(KernelStage::Refine, "evaluate")?.length())
        .max(1.0);
    let gap = on_support.sub(on_boundary).length();
    // Both sides of that subtraction are APPROXIMATIONS of the same rail: the
    // support is a cubic fit through the marched stations, and the crossing
    // parameter came out of the pcurve fit through the SAME stations — two
    // interpolants that agree at every station and drift between them by the
    // fit's own accuracy, which the surface's parameterisation amplifies (a
    // revolution's u is projective, not proportional to angle).  On a 208°
    // arc of a r≈15 revolve that drift is 2e-4, so `1e-6·scale` was demanding
    // more precision of the row than the row carries and a genuinely
    // transverse 43° corner was refused.  The kernel already names the size
    // of that disagreement — see [`support_crossing_tolerances`], which makes
    // exactly this argument for the 3D intersector — so the phantom test is
    // judged against `pcurve_consistency`, the same contract, and never
    // against a literal of its own.  A crossing accepted only past the
    // historical bar is ESCALATED: the caller must validate what it builds.
    let historical = 1e-6 * scale;
    let band = crate::KernelTolerances::for_solid(solid, 1e-7).pcurve_consistency;
    if gap > band.max(historical) {
        return Err(KernelRefusal::unsupported(
            KernelStage::Intersect,
            "support_crossing_uv_phantom",
            format!(
                "blend: support curve does not reach the end boundary edge (the parameter-space \
                 crossing is {gap:.3e} off in 3D — the boundary continues the blended edge rather \
                 than crossing its rail)"
            ),
        ));
    }
    Ok((s, t, gap > historical))
}

/// Where a fitted support curve crosses a boundary edge (nearest genuine
/// crossing to the given end of the support domain).
pub(in crate::blend) fn support_crossing(
    solid: &BrepSolid,
    support: &NurbsCurve,
    boundary: &EdgeRecord,
    at_start: bool,
) -> Result<(f64, f64, Vec3, bool), KernelRefusal> {
    for tolerance in support_crossing_tolerances(solid) {
        if let Some((s, t, point)) = support_crossing_at(support, boundary, at_start, tolerance)? {
            return Ok((s, t, point, tolerance > 1e-7));
        }
    }
    Err(KernelRefusal::unsupported(
        KernelStage::Intersect,
        "support_crossing_none",
        "blend: support curve does not reach the end boundary edge",
    ))
}

/// One rung of [`support_crossing`]'s ladder.
fn support_crossing_at(
    support: &NurbsCurve,
    boundary: &EdgeRecord,
    at_start: bool,
    tolerance: f64,
) -> Result<Option<(f64, f64, Vec3)>, KernelRefusal> {
    let hits = crate::intersect_curves(support, &boundary.curve, tolerance).or_refuse(KernelStage::Refine, "intersect_curves")?;
    let [s0, s1] = support.domain().or_refuse(KernelStage::Refine, "domain")?;
    let mut best: Option<(f64, f64, Vec3)> = None;
    for hit in hits {
        if hit.t < boundary.t0 - 1e-9 || hit.t > boundary.t1 + 1e-9 {
            continue;
        }
        // The support is a fitted row over [s0, s1]; a hit reported outside
        // it is the intersector extrapolating the row's carrier (a tangent
        // continuation lets a circle's extension meet a line far away) and
        // is not a crossing of the rail.
        let margin = 1e-9 * (1.0 + (s1 - s0).abs());
        if hit.s < s0 - margin || hit.s > s1 + margin {
            continue;
        }
        let rank = if at_start { hit.s - s0 } else { s1 - hit.s };
        if best
            .map(|(existing, ..)| {
                let existing_rank = if at_start {
                    existing - s0
                } else {
                    s1 - existing
                };
                rank < existing_rank
            })
            .unwrap_or(true)
        {
            best = Some((hit.s, hit.t, hit.point));
        }
    }
    Ok(best)
}

/// The tolerance ladder a fitted support row is intersected against, tightest
/// first.
///
/// # Why a ladder and not one number
/// A support row is a FIT through marched stations; the spoke/boundary edge it
/// crosses is the carrier's own EXACT curve.  Both lie on the same carrier by
/// construction, so the crossing exists geometrically — but the fitted row only
/// reproduces the carrier to the fit's accuracy, so in 3D the two curves pass
/// each other with a residual gap of that size instead of meeting exactly.
/// [`crate::intersect_curves`] discards any Newton answer whose 3D gap exceeds
/// the tolerance it was given, so demanding 1e-7 of a fitted row asks for more
/// precision than the row carries and the crossing is reported as ABSENT.
///
/// The kernel already names the size of that disagreement, and names it twice:
/// [`crate::KernelTolerances::intersection_fit`] ("maximum geometric error
/// accepted while fitting SSI curves") and
/// [`crate::KernelTolerances::pcurve_consistency`] ("maximum edge/pcurve/
/// carrier disagreement in a valid in-memory BREP", whose own comment records
/// that *existing fitted NURBS intersections can deviate by a few microns*).
/// Those are the contract; 1e-7 was tighter than any of them.
///
/// The ladder starts at the historical 1e-7 so every crossing that resolves
/// today resolves identically — the looser rungs are reached ONLY where the
/// tight one finds nothing, i.e. only where the caller used to hard-refuse —
/// and it stops at the FIRST rung that produces a crossing, so a crossing is
/// never accepted at a looser tolerance than it actually needs.
pub(in crate::blend) fn support_crossing_tolerances(solid: &BrepSolid) -> [f64; 3] {
    let policy = crate::KernelTolerances::for_solid(solid, 1e-7);
    [1e-7, policy.intersection_fit, policy.pcurve_consistency]
}

/// Where a fitted support row crosses a chain SPOKE edge.
///
/// Returns the crossing nearest the chain vertex as `(support parameter, spoke
/// edge parameter, escalated)`, where `escalated` is true when the crossing
/// only resolved on a looser rung than the historical 1e-7 — the caller MUST
/// then validate what the surgery built (see [`SpokeCrossings`]).  `None` means
/// the row genuinely does not cross the spoke on any rung; the caller renders
/// that refusal, because what it means differs between the closed and open
/// chain lanes.
pub(in crate::blend) fn spoke_crossing(
    solid: &BrepSolid,
    support: &NurbsCurve,
    spoke: &EdgeRecord,
    vertex_point: Vec3,
) -> Result<Option<(f64, f64, bool)>, KernelRefusal> {
    for tolerance in support_crossing_tolerances(solid) {
        let hits = crate::intersect_curves(support, &spoke.curve, tolerance).or_refuse(KernelStage::Refine, "intersect_curves")?;
        let mut best: Option<(f64, f64, f64)> = None;
        for hit in hits {
            if hit.t < spoke.t0 - 1e-9 || hit.t > spoke.t1 + 1e-9 {
                continue;
            }
            let distance = hit.point.sub(vertex_point).length();
            if best.map(|(.., known)| distance < known).unwrap_or(true) {
                best = Some((hit.s, hit.t, distance));
            }
        }
        if let Some((crossing_support, crossing_spoke, _)) = best {
            return Ok(Some((crossing_support, crossing_spoke, tolerance > 1e-7)));
        }
    }
    Ok(None)
}

/// Whether any spoke crossing in one chain surgery needed a looser rung than
/// the historical 1e-7, and the gate that fact imposes on the result.
///
/// # Why the escalation is gated and the tight rung is not
/// The tight rung reproduces master exactly, INCLUDING whatever it already
/// builds — so it is never re-judged here; re-judging it would turn an existing
/// output into a refusal, which is a regression whatever the output's quality.
/// The looser rungs, by contrast, only ever run where master hard-refused, so
/// everything they produce is NEW.  A new output is worth having only if it is
/// a valid solid: a refusal is recoverable (the caller's maximal-valid-subset
/// search, or the user's own edit), a silently invalid solid is not.  So the
/// escalated path must earn its answer, and a radius that marches past what the
/// supports can carry goes back to being refused — with a message that says so
/// — instead of shipping a torn blend.
#[derive(Clone, Copy, Default)]
pub(in crate::blend) struct SpokeCrossings {
    escalated: bool,
}

impl SpokeCrossings {
    pub(in crate::blend) fn note(&mut self, escalated: bool) {
        self.escalated |= escalated;
    }

    /// Accept `result` unless it was built on an escalated crossing AND does
    /// not validate.
    pub(in crate::blend) fn gate(&self, result: BrepSolid) -> Result<BrepSolid, KernelRefusal> {
        if !self.escalated {
            return Ok(result);
        }
        let issues = result.validate();
        if issues.is_empty() {
            return Ok(result);
        }
        Err(KernelRefusal::new(
            RefusalClass::InvalidResultTopology {
                issues: issues.len() as u32,
            },
            KernelStage::Validate,
            format!(
                "blend: chain surgery across a fit-tolerance spoke crossing did not \
                 produce a valid solid ({})",
                issues
                    .first()
                    .map(|issue| issue.message.clone())
                    .unwrap_or_default()
            ),
        ))
    }
}

/// The transverse curve where the blend surface crosses the end face:
/// per-z Newton in t on the signed distance to the end carrier, endpoints
/// pinned to the rim crossings.
pub(in crate::blend) fn transverse_curve(
    rows: &FittedRows,
    end_face: &FaceRecord,
    cr_parameter: f64,
    cs_parameter: f64,
) -> Result<(NurbsCurve, NurbsCurve, NurbsCurve), KernelRefusal> {
    const SECTIONS: usize = 16;
    let mut points3 = Vec::with_capacity(SECTIONS + 1);
    let mut blend_uv = Vec::with_capacity(SECTIONS + 1);
    let mut end_uv: Vec<[f64; 2]> = Vec::with_capacity(SECTIONS + 1);
    let mut parameters = Vec::with_capacity(SECTIONS + 1);
    // The curve is ONE branch of the end carrier: after the first section every
    // foot continues from the last one found — the previous section's, the
    // previous iterate's, and for the difference quotient the iterate's own —
    // so the level the Newton differences is the distance to that sheet, and no
    // sample of the end pcurve is the nearest point of another.
    let project = |point: Vec3, seed: Option<[f64; 2]>| match seed {
        None => crate::project_point_to_surface(&end_face.surface, point)
            .or_refuse(KernelStage::Refine, "project_point_to_surface"),
        Some(seed) => crate::projection::project_point_to_surface_from_seed(&end_face.surface, point, seed)
            .or_refuse(KernelStage::Refine, "project_point_to_surface_from_seed"),
    };
    for section in 0..=SECTIONS {
        let z = section as f64 / SECTIONS as f64;
        let mut t = cr_parameter + (cs_parameter - cr_parameter) * z;
        let mut foot = end_uv.last().copied();
        if section > 0 && section < SECTIONS {
            for _ in 0..NEWTON_ITERATIONS {
                let point = rows.surface.evaluate_extended(t, z).or_refuse(KernelStage::Refine, "evaluate_extended")?;
                let projection = project(point, foot)?;
                foot = Some([projection.u, projection.v]);
                let normal = raw_normal(&end_face.surface, projection.u, projection.v)?;
                let distance = point.sub(projection.point).dot(normal);
                if distance.abs() <= 1e-11 {
                    break;
                }
                let step = 1e-7;
                let probe = rows.surface.evaluate_extended(t + step, z).or_refuse(KernelStage::Refine, "evaluate_extended")?;
                let probe_projection = project(probe, foot)?;
                let probe_distance = probe.sub(probe_projection.point).dot(raw_normal(
                    &end_face.surface,
                    probe_projection.u,
                    probe_projection.v,
                )?);
                let derivative = (probe_distance - distance) / step;
                if derivative.abs() <= 1e-14 {
                    return Err(KernelRefusal::non_convergence(
                        KernelStage::Refine,
                        "transverse_newton",
                        "blend: transverse Newton stalled",
                    ));
                }
                t -= distance / derivative;
            }
        }
        let point = rows.surface.evaluate_extended(t, z).or_refuse(KernelStage::Refine, "evaluate_extended")?;
        let projection = project(point, foot)?;
        points3.push(Vec4::from_point(point, 1.0));
        blend_uv.push(Vec4::from_point(Vec3::new(t, z, 0.0), 1.0));
        end_uv.push([projection.u, projection.v]);
        parameters.push(z);
    }
    unwrap_seam_branch(&end_face.surface, &mut end_uv)?;
    let end_uv: Vec<Vec4> = end_uv
        .into_iter()
        .map(|uv| Vec4::from_point(Vec3::new(uv[0], uv[1], 0.0), 1.0))
        .collect();
    let curve = fit::interpolate_homogeneous(&points3, FIT_DEGREE, &parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
    let blend_pcurve = fit::interpolate_homogeneous(&blend_uv, FIT_DEGREE, &parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
    let end_pcurve = fit::interpolate_homogeneous(&end_uv, FIT_DEGREE, &parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
    Ok((curve, blend_pcurve, end_pcurve))
}

/// Make a sampled parameter sequence CONTINUOUS across a closed carrier's
/// seam, in place.
///
/// # Why a projection alone cannot decide this
/// On a surface closed in u, the parameters `u0` and `u1` name the SAME
/// meridian, so a point sitting on it projects to either, and the projector's
/// answer is arbitrary — measured on the captured revolve-about-its-own-edge
/// part, the rim point where the blend's row meets its mate came back at
/// `u = 1.000000` while every neighbouring sample answered near `u = 0`.  That
/// happens at every free end whose mate boundary IS the end face's seam, which
/// a revolve about a line through the profile produces by construction (the
/// shared generatrix is both).  One sample left on the far branch makes the
/// fitted pcurve walk the long way round the carrier.
///
/// The seam is a parameter artefact, so the fix is the parametric one: choose,
/// for each sample, the branch nearest its neighbour.  The walk starts at the
/// MIDDLE sample and runs outward in both directions — an end sample is the
/// one that lands on a seam, the middle is the one that does not.  Whether the
/// branch the WHOLE curve lands on is the one the end face's loop can join is
/// a separate question, and [`check_loop_branches`] is where it is asked.
fn unwrap_seam_branch(surface: &NurbsSurface, samples: &mut [[f64; 2]]) -> Result<(), KernelRefusal> {
    let (closed_u, closed_v) = surface.closed_directions().unwrap_or((false, false));
    if (!closed_u && !closed_v) || samples.len() < 2 {
        return Ok(());
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let periods = [
        if closed_u { u1 - u0 } else { 0.0 },
        if closed_v { v1 - v0 } else { 0.0 },
    ];
    let snap = |value: f64, reference: f64, period: f64| -> f64 {
        if !(period > 0.0) {
            return value;
        }
        let shifted = value - period * ((value - reference) / period).round();
        if (shifted - reference).abs() < (value - reference).abs() {
            shifted
        } else {
            value
        }
    };
    let middle = samples.len() / 2;
    for index in (0..middle).rev() {
        for axis in 0..2 {
            samples[index][axis] = snap(samples[index][axis], samples[index + 1][axis], periods[axis]);
        }
    }
    for index in middle + 1..samples.len() {
        for axis in 0..2 {
            samples[index][axis] = snap(samples[index][axis], samples[index - 1][axis], periods[axis]);
        }
    }
    Ok(())
}

/// Trim `edge_id` at `parameter`, moving the `old_vertex` endpoint to
/// `new_vertex`, and split every referencing coedge pcurve to the
/// matching fraction (the pcurve domain maps affinely onto [t0, t1]).
pub(in crate::blend) fn trim_edge_at(
    solid: &mut BrepSolid,
    edge_id: u64,
    parameter: f64,
    old_vertex: u64,
    new_vertex: u64,
) -> Result<(), KernelRefusal> {
    let (old_t0, old_t1, trims_start) = {
        let edge = solid
            .edges
            .iter()
            .find(|edge| edge.id == edge_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "trim_edge",
                    "blend: edge to trim is missing",
                )
            })?;
        (edge.t0, edge.t1, edge.start_vertex_id == old_vertex)
    };
    let fraction = (parameter - old_t0) / (old_t1 - old_t0);
    let interior = (1e-9..=1.0 - 1e-9).contains(&fraction);
    let extends = if trims_start {
        fraction < 1e-9
    } else {
        fraction > 1.0 - 1e-9
    };
    if !interior && !extends {
        return Err(KernelRefusal::unsupported(
            KernelStage::Sew,
            "trim_degenerate",
            "blend: end trim degenerates the boundary edge",
        ));
    }
    if extends {
        let curved = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
            .any(|coedge| coedge.edge_id == edge_id && coedge.pcurve.degree != 1);
        if curved {
            return extend_edge_on_carriers(solid, edge_id, parameter, old_vertex, new_vertex);
        }
    }
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            for loop_record in &mut face.loops {
                for coedge in &mut loop_record.coedges {
                    if coedge.edge_id != edge_id {
                        continue;
                    }
                    let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                    // Pcurve fraction runs with the traversal: forward
                    // coedges map fraction f -> q0 + (q1-q0)f, reversed
                    // ones map f -> q1 - (q1-q0)f.
                    let split_q = if coedge.forward {
                        q0 + (q1 - q0) * fraction
                    } else {
                        q1 - (q1 - q0) * fraction
                    };
                    if interior {
                        let keep_upper = trims_start == coedge.forward;
                        let (low, high) = coedge.pcurve.split(split_q).or_refuse(KernelStage::Refine, "split")?;
                        coedge.pcurve = if keep_upper { high } else { low };
                    } else {
                        // EXTEND (§6.9): the boundary continues past its old
                        // end to meet the blend.  Exact for linear pcurves;
                        // curved pcurves would need re-fitting.
                        if coedge.pcurve.degree != 1 {
                            return Err(KernelRefusal::unsupported(
                                KernelStage::Sew,
                                "extend_curved_pcurve",
                                "blend: cannot extend a curved boundary pcurve",
                            ));
                        }
                        let a = coedge.pcurve.evaluate(q0).or_refuse(KernelStage::Refine, "evaluate")?;
                        let b = coedge.pcurve.evaluate(q1).or_refuse(KernelStage::Refine, "evaluate")?;
                        let direction = b.sub(a);
                        let extended_fraction = (split_q - q0) / (q1 - q0);
                        let target = a.add(direction.scale(extended_fraction));
                        let controls = &mut coedge.pcurve.control_points;
                        let end_index = if (trims_start == coedge.forward) == false {
                            controls.len() - 1
                        } else {
                            0
                        };
                        let w = controls[end_index].w;
                        controls[end_index] = crate::Vec4::from_point(target, w);
                    }
                }
            }
        }
    }
    let edge = solid
        .edges
        .iter_mut()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| {
            KernelRefusal::internal(KernelStage::Sew, "trim_edge", "blend: edge to trim is missing")
        })?;
    if trims_start {
        edge.t0 = parameter;
        edge.start_vertex_id = new_vertex;
    } else {
        edge.t1 = parameter;
        edge.end_vertex_id = new_vertex;
    }
    Ok(())
}

/// Stations per original edge span, and per extension, of the refit in
/// [`extend_edge_on_carriers`].
const EXTENSION_REFIT_SAMPLES: usize = 96;
const EXTENSION_SAMPLES: usize = 8;

/// The §6.9 EXTEND on a CURVED boundary: the edge continues past `old_vertex`
/// to `new_vertex` along the intersection of the two faces it bounds, and its
/// curve and both pcurves are refitted over the extended range.
///
/// # Why the intersection, and not the curve's own continuation
/// A boundary is where its two carriers meet, so that is the only curve its
/// continuation can be: a cone's rim runs on round the cone, not along its end
/// tangent (whose departure is `½·d²·κ` — 4e-4 over the 0.08 a concave r = 0.1
/// blend's rail runs past a radius-7.4 rim), and a pcurve extrapolated in
/// parameter space leaves its surface image the same way. So every extension
/// point is SOLVED on both carriers, Newton on the two faces' (u, v) with the
/// point held in a plane across the chord, and the pcurves take the solved
/// parameters; the original span is resampled from the edge as it stands.
///
/// Only a boundary whose pcurves are not straight lines comes here: a straight
/// one extends exactly by moving its end control point, as before.
fn extend_edge_on_carriers(
    solid: &mut BrepSolid,
    edge_id: u64,
    parameter: f64,
    old_vertex: u64,
    new_vertex: u64,
) -> Result<(), KernelRefusal> {
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "extend_edge",
                "blend: edge to extend is missing",
            )
        })?
        .clone();
    let trims_start = edge.start_vertex_id == old_vertex;
    let target = solid
        .vertices
        .iter()
        .find(|vertex| vertex.id == new_vertex)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "extend_vertex",
                "blend: the extended boundary's new vertex is missing",
            )
        })?
        .point;
    // The two faces the boundary bounds, with their coedges' pcurves.
    let mut uses: Vec<(u64, CoedgeRecord)> = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            if coedge.edge_id == edge_id {
                uses.push((face.id, coedge.clone()));
            }
        }
    }
    let [(face_a, coedge_a), (face_b, coedge_b)] = <[(u64, CoedgeRecord); 2]>::try_from(uses)
        .map_err(|uses| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "extend_uses",
                format!(
                    "blend: cannot extend boundary edge {edge_id} along its carriers — it is used \
                     {} times, not twice",
                    uses.len()
                ),
            )
        })?;
    let surface_of = |face_id: u64| {
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == face_id)
            .map(|face| face.surface.clone())
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "extend_face",
                    "blend: a face of the extended boundary is missing",
                )
            })
    };
    let (surface_a, surface_b) = (surface_of(face_a)?, surface_of(face_b)?);
    let (t0, t1) = (edge.t0, edge.t1);
    // A pcurve read at an EDGE parameter of the original range, honouring the
    // affine traversal contract.
    let uv_at = |coedge: &CoedgeRecord, t: f64| -> Result<[f64; 2], KernelRefusal> {
        let point = edge_uv_on_face(coedge, &edge, t)?;
        Ok(point)
    };
    let end_t = if trims_start { t0 } else { t1 };
    let end_point = edge.curve.evaluate(end_t).or_refuse(KernelStage::Refine, "evaluate")?;
    let chord = target.sub(end_point);
    let reach = chord.length();
    let direction = chord
        .normalized()
        .map_err(|_| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "extend_length",
                "blend: the boundary extension has no length",
            )
        })?;
    let scale = target.length().max(end_point.length()).max(1.0);
    let tolerance = 1e-11 * (1.0 + scale);
    // March the carriers' intersection from the old end to the new vertex.
    let mut seed = {
        let a = uv_at(&coedge_a, end_t)?;
        let b = uv_at(&coedge_b, end_t)?;
        [a[0], a[1], b[0], b[1]]
    };
    let mut extension: Vec<(f64, Vec3, [f64; 4])> = Vec::with_capacity(EXTENSION_SAMPLES);
    for step in 1..=EXTENSION_SAMPLES {
        let fraction = step as f64 / EXTENSION_SAMPLES as f64;
        let level = reach * fraction;
        let mut x = seed;
        let mut solved = None;
        for _ in 0..NEWTON_ITERATIONS {
            let da = surface_a.derivatives_extended(x[0], x[1], 1).or_refuse(KernelStage::Refine, "derivatives_extended")?;
            let db = surface_b.derivatives_extended(x[2], x[3], 1).or_refuse(KernelStage::Refine, "derivatives_extended")?;
            let mismatch = da[0][0].sub(db[0][0]);
            let plane = da[0][0].sub(end_point).dot(direction) - level;
            if mismatch.length() <= tolerance && plane.abs() <= tolerance {
                solved = Some(da[0][0]);
                break;
            }
            let matrix = [
                [da[1][0].x, da[0][1].x, -db[1][0].x, -db[0][1].x],
                [da[1][0].y, da[0][1].y, -db[1][0].y, -db[0][1].y],
                [da[1][0].z, da[0][1].z, -db[1][0].z, -db[0][1].z],
                [da[1][0].dot(direction), da[0][1].dot(direction), 0.0, 0.0],
            ];
            let delta = fit::solve_small::<4>(
                matrix,
                [mismatch.x, mismatch.y, mismatch.z, plane],
                4,
            )
            .map_err(|error| {
                KernelRefusal::ill_posed(
                    KernelStage::Refine,
                    "extend_tangent_carriers",
                    format!(
                        "blend: the carriers of boundary edge {edge_id} do not cross transversally \
                         where it must extend ({error})"
                    ),
                )
            })?;
            for (value, correction) in x.iter_mut().zip(delta) {
                *value -= correction;
            }
        }
        let point = solved.ok_or_else(|| {
            KernelRefusal::non_convergence(
                KernelStage::Refine,
                "extend_march",
                format!(
                    "blend: the carriers' intersection could not be followed along boundary edge \
                     {edge_id} to the blend"
                ),
            )
        })?;
        let t = end_t + (parameter - end_t) * fraction;
        extension.push((t, point, x));
        seed = x;
    }
    // The last solved point is the carriers' own crossing of the plane through
    // the new vertex, which sits on that crossing to the rail's fit.
    let arrival = extension
        .last()
        .map(|(_, point, _)| point.sub(target).length())
        .unwrap_or(f64::INFINITY);
    let bar = crate::KernelTolerances::for_solid(solid, 1e-7).pcurve_consistency;
    if arrival > bar {
        return Err(KernelRefusal::internal(
            KernelStage::Sew,
            "extend_arrival",
            format!(
                "blend: boundary edge {edge_id} extended along its carriers arrives {arrival:.3e} \
                 from the blend's rim"
            ),
        ));
    }
    // Every sample over the new range, in increasing edge parameter.
    let mut samples: Vec<(f64, Vec3, [f64; 2], [f64; 2])> =
        Vec::with_capacity(EXTENSION_REFIT_SAMPLES + EXTENSION_SAMPLES + 1);
    for index in 0..=EXTENSION_REFIT_SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / EXTENSION_REFIT_SAMPLES as f64;
        samples.push((
            t,
            edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?,
            uv_at(&coedge_a, t)?,
            uv_at(&coedge_b, t)?,
        ));
    }
    for (t, point, x) in extension {
        samples.push((t, point, [x[0], x[1]], [x[2], x[3]]));
    }
    samples.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (new_t0, new_t1) = if trims_start {
        (parameter, t1)
    } else {
        (t0, parameter)
    };
    let parameters: Vec<f64> = samples.iter().map(|sample| sample.0).collect();
    if parameters.windows(2).any(|pair| pair[1] <= pair[0]) {
        return Err(KernelRefusal::internal(
            KernelStage::Sew,
            "extend_direction",
            format!(
                "blend: boundary edge {edge_id} extends in the wrong direction"
            ),
        ));
    }
    let curve = fit::interpolate_curve(
        &samples.iter().map(|sample| sample.1).collect::<Vec<_>>(),
        3,
        &parameters,
    ).or_refuse(KernelStage::Refine, "interpolate_curve")?;
    // Each pcurve keeps its own domain and the affine contract over the NEW
    // edge range.
    let refit = |coedge: &CoedgeRecord, side: usize| -> Result<NurbsCurve, KernelRefusal> {
        let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mut pairs: Vec<(f64, Vec3)> = samples
            .iter()
            .map(|sample| {
                let fraction = (sample.0 - new_t0) / (new_t1 - new_t0);
                let q = if coedge.forward {
                    q0 + (q1 - q0) * fraction
                } else {
                    q1 - (q1 - q0) * fraction
                };
                let uv = if side == 0 { sample.2 } else { sample.3 };
                (q, Vec3::new(uv[0], uv[1], 0.0))
            })
            .collect();
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        fit::interpolate_curve(
            &pairs.iter().map(|pair| pair.1).collect::<Vec<_>>(),
            3,
            &pairs.iter().map(|pair| pair.0).collect::<Vec<_>>(),
        )
        .or_refuse(KernelStage::Sew, "interpolate_curve")
    };
    let pcurve_a = refit(&coedge_a, 0)?;
    let pcurve_b = refit(&coedge_b, 1)?;
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            for loop_record in &mut face.loops {
                for coedge in &mut loop_record.coedges {
                    if coedge.id == coedge_a.id {
                        coedge.pcurve = pcurve_a.clone();
                    } else if coedge.id == coedge_b.id {
                        coedge.pcurve = pcurve_b.clone();
                    }
                }
            }
        }
    }
    let record = solid
        .edges
        .iter_mut()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "extend_edge",
                "blend: edge to extend is missing",
            )
        })?;
    record.curve = curve;
    record.t0 = new_t0;
    record.t1 = new_t1;
    if trims_start {
        record.start_vertex_id = new_vertex;
    } else {
        record.end_vertex_id = new_vertex;
    }
    Ok(())
}
