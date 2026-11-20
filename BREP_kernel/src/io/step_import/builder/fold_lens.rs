use super::*;

impl<'a> SolidBuilder<'a> {
    /// An inner loop made of ONE edge used twice, in opposite senses, on a
    /// carrier that folds through that edge: every point of the edge has two
    /// feet on the surface, one on each sheet of the fold, and the two
    /// coedges are the two sides of the lens the fold encloses (a swept
    /// surface whose profile turns tighter than its path, with the inner fin
    /// trimmed away). Without supplied pcurves each coedge is fitted to the
    /// nearest foot station by station, and the two feet are equally near, so
    /// both pcurves hop between sheets and retrace each other: the hole has no
    /// area, and the fin is read as part of the face (and of the solid's
    /// boundary).
    ///
    /// Each coedge is given one whole sheet instead: two chains of feet, each
    /// tracked by continuation from a station where the global foot lies on
    /// it. Which coedge takes which chain is decided by the face's own winding
    /// rule (an inner loop winds against the outer one), since both choices are
    /// locally consistent -- one reads the lens as a hole, the other as
    /// material. Applied only when the two chains are continuous, on the
    /// surface, meet at the edge's ends, and separate in between by more than
    /// tracking noise and less than half a period; any other loop is left as
    /// fitted.
    ///
    /// Hatch: `BREP_STEP_FOLD_LENS=0` keeps the fitted pcurves.
    pub(super) fn seat_fold_lens_loops(
        &mut self,
        face_ref: usize,
        surface: &NurbsSurface,
        same_sense: bool,
        outer: &[bool],
        loops: &mut [LoopRecord],
    ) -> Result<(), String> {
        if !switch!("BREP_STEP_FOLD_LENS", true) || loops.len() < 2 {
            return Ok(());
        }
        let lens = |record: &LoopRecord| {
            let coedges = &record.coedges;
            coedges.len() == 2 && coedges[0].edge_id == coedges[1].edge_id && coedges[0].forward != coedges[1].forward
        };
        // A lens is read as a hole only beside a loop that bounds the face; a
        // face of lenses alone, or a lens the file names as its outer bound, is
        // left as fitted.
        if loops.iter().all(|record| lens(record)) {
            return Ok(());
        }
        for index in 0..loops.len() {
            if outer.get(index).copied().unwrap_or(true) || !lens(&loops[index]) {
                continue;
            }
            let coedges = &loops[index].coedges;
            let edge_id = coedges[0].edge_id;
            let edge = self.edge_record(edge_id).clone();
            if edge.degenerate {
                continue;
            }
            // Stations are bisected wherever a trim misses its edge between
            // them: the sheets part steeply where they meet at the edge's ends.
            let tolerance = super::geometry::loop_pcurve_tolerance(surface)?;
            // The edge curve's own knots are stations too, so every knot span is
            // read at its quarter points however narrow it is.
            let mut fractions: Vec<f64> = (0..=FOLD_STATIONS).map(|k| k as f64 / FOLD_STATIONS as f64).collect();
            let span = edge.t1 - edge.t0;
            if span.abs() > 0.0 {
                fractions.extend(edge.curve.knots.iter().map(|k| (k - edge.t0) / span).filter(|f| *f > 0.0 && *f < 1.0));
                fractions.sort_by(f64::total_cmp);
                fractions.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
            }
            let mut seated = None;
            for _ in 0..FOLD_REFINEMENTS {
                let Some(chains) = fold_chains(surface, &edge.curve, edge.t0, edge.t1, &fractions)? else {
                    break;
                };
                let Some(candidate) = wound_lens(surface, same_sense, &loops[index], &chains, &fractions)? else {
                    break;
                };
                let mut misses = std::collections::BTreeSet::new();
                for coedge in &candidate.coedges {
                    misses.extend(missed_intervals(surface, &coedge.pcurve, &edge.curve, edge.t0, edge.t1, coedge.forward, &fractions, tolerance)?);
                }
                if misses.is_empty() {
                    seated = Some(candidate);
                    break;
                }
                for k in misses.into_iter().rev() {
                    fractions.insert(k + 1, 0.5 * (fractions[k] + fractions[k + 1]));
                }
            }
            if let Some(candidate) = seated {
                loops[index] = candidate;
                // The fitted trims' readings no longer describe this face.
                self.bounded.retain(|trim| trim.face_ref != face_ref || trim.edge_id != edge_id);
            }
        }
        Ok(())
    }
}

/// Stations along the edge at which the two sheets are first tracked, and the
/// rounds of bisection the trims may take to image the edge between them.
const FOLD_STATIONS: usize = 256;
const FOLD_REFINEMENTS: usize = 16;

/// The two continuous chains of feet of `curve` on `surface`, in edge sense,
/// when the curve lies on a fold of the surface; `None` otherwise.
fn fold_chains(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    fractions: &[f64],
) -> Result<Option<(Vec<(f64, f64)>, Vec<(f64, f64)>)>, String> {
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let (u_span, v_span) = ((u1 - u0).abs().max(1e-12), (v1 - v0).abs().max(1e-12));
    let gap = |a: (f64, f64), b: (f64, f64)| (((a.0 - b.0) / u_span).powi(2) + ((a.1 - b.1) / v_span).powi(2)).sqrt();
    let tolerance = super::geometry::loop_pcurve_tolerance(surface)?;
    let points: Vec<Vec3> =
        fractions.iter().map(|f| curve.evaluate(t0 + (t1 - t0) * f)).collect::<Result<_, _>>()?;
    let mut feet = Vec::with_capacity(points.len());
    for point in &points {
        let foot = crate::project_point_to_surface(surface, *point)?;
        if foot.distance > tolerance {
            return Ok(None);
        }
        feet.push((foot.u, foot.v));
    }
    // One chain by continuation from a seed station, both ways to the ends.
    let track = |seed: usize, foot: (f64, f64)| -> Result<Option<Vec<(f64, f64)>>, String> {
        let mut chain = vec![(0.0, 0.0); points.len()];
        chain[seed] = foot;
        for order in [(seed + 1..points.len()).collect::<Vec<_>>(), (0..seed).rev().collect()] {
            let mut previous = foot;
            for k in order {
                let foot = crate::project_point_to_surface_seeded(surface, points[k], previous.0, previous.1)?;
                if foot.distance > tolerance || gap((foot.u, foot.v), previous) > 0.05 {
                    return Ok(None);
                }
                previous = (foot.u, foot.v);
                chain[k] = previous;
            }
        }
        Ok(Some(chain))
    };
    let middle = points.len() / 2;
    let Some(first) = track(middle, feet[middle])? else {
        return Ok(None);
    };
    // A station whose global foot is off the first chain seeds the second;
    // when the global projector keeps to one sheet throughout, the middle
    // station is searched from a grid of seeds for a foot off the first chain.
    const APART: f64 = 1e-3;
    let Some(station) = (1..points.len() - 1).max_by(|&a, &b| gap(feet[a], first[a]).total_cmp(&gap(feet[b], first[b]))) else {
        return Ok(None);
    };
    let seed = if gap(feet[station], first[station]) > APART {
        Some((station, feet[station]))
    } else {
        let mut best: Option<(f64, (f64, f64))> = None;
        for i in 0..=8 {
            for j in 0..=8 {
                let (u, v) = (u0 + (u1 - u0) * i as f64 / 8.0, v0 + (v1 - v0) * j as f64 / 8.0);
                let foot = crate::project_point_to_surface_seeded(surface, points[middle], u, v)?;
                let apart = gap((foot.u, foot.v), first[middle]);
                if foot.distance <= tolerance && apart > APART && apart < 0.5 && best.is_none_or(|(d, _)| foot.distance < d) {
                    best = Some((foot.distance, (foot.u, foot.v)));
                }
            }
        }
        best.map(|(_, foot)| (middle, foot))
    };
    let Some((station, foot)) = seed else {
        return Ok(None);
    };
    let Some(second) = track(station, foot)? else {
        return Ok(None);
    };
    // The sheets meet at the edge's ends and part in between, by less than half
    // a period (a whole period apart is one sheet seen across a seam).
    let last = points.len() - 1;
    let parted = (1..last).map(|k| gap(first[k], second[k])).fold(0.0f64, f64::max);
    if gap(first[0], second[0]) > APART || gap(first[last], second[last]) > APART || parted <= APART || parted >= 0.5 {
        return Ok(None);
    }
    let mut second = second;
    second[0] = first[0];
    second[last] = first[last];
    Ok(Some((first, second)))
}

/// The lens loop with one chain on each coedge, wound as a hole: against the
/// face's outer loop, whose signed parameter area is positive exactly when the
/// face agrees with its carrier (`finish_face`). Both assignments are locally
/// consistent -- one reads the lens as a hole, the other as material -- so the
/// winding decides. `None` when neither winds as a hole.
fn wound_lens(
    surface: &NurbsSurface,
    same_sense: bool,
    record: &LoopRecord,
    (chain_a, chain_b): &(Vec<(f64, f64)>, Vec<(f64, f64)>),
    fractions: &[f64],
) -> Result<Option<LoopRecord>, String> {
    let hole_positive = !same_sense;
    for (first, second) in [(chain_a, chain_b), (chain_b, chain_a)] {
        let mut candidate = record.clone();
        for coedge in &mut candidate.coedges {
            // A chain runs in edge sense; the reversed coedge runs the other
            // chain backwards.
            let chain = if coedge.forward { first } else { second };
            let mut stations: Vec<(f64, Vec3)> =
                fractions.iter().zip(chain).map(|(&f, &(u, v))| (if coedge.forward { f } else { 1.0 - f }, Vec3::new(u, v, 0.0))).collect();
            if !coedge.forward {
                stations.reverse();
            }
            let parameters: Vec<f64> = stations.iter().map(|s| s.0).collect();
            let points: Vec<Vec3> = stations.iter().map(|s| s.1).collect();
            coedge.pcurve = crate::interpolate_curve(&points, 3, &parameters)?;
        }
        let probe = FaceRecord { id: 0, surface: surface.clone(), same_sense, loops: vec![candidate.clone()], name: None };
        let area = crate::parameter_space_area(&probe)?;
        if area != 0.0 && (area > 0.0) == hole_positive {
            return Ok(Some(candidate));
        }
    }
    Ok(None)
}

/// The station intervals (indices into `fractions`, in edge sense) over which
/// `pcurve` -- interpolated on the coedge-sense fractions -- images `curve`
/// farther than `tolerance` from the same fraction of the edge, read at each
/// interval's quarter points.
#[allow(clippy::too_many_arguments)]
fn missed_intervals(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    forward: bool,
    fractions: &[f64],
    tolerance: f64,
) -> Result<Vec<usize>, String> {
    let debug = std::env::var("BREP_DEBUG_PCURVE").is_ok();
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let (slack_u, slack_v) = (1e-12 * (u1 - u0).abs(), 1e-12 * (v1 - v0).abs());
    let mut misses = Vec::new();
    for k in 0..fractions.len() - 1 {
        for offset in [0.25, 0.5, 0.75] {
            let fraction = fractions[k] + (fractions[k + 1] - fractions[k]) * offset;
            let uv = pcurve.evaluate(if forward { fraction } else { 1.0 - fraction })?;
            let point = curve.evaluate(t0 + (t1 - t0) * fraction)?;
            // Read off the domain the surface would clamp, so a trim that
            // overshoots it counts as a miss rather than reading the boundary.
            let inside = uv.x >= u0 - slack_u && uv.x <= u1 + slack_u && uv.y >= v0 - slack_v && uv.y <= v1 + slack_v;
            let miss = if inside { surface.evaluate(uv.x, uv.y)?.sub(point).length() } else { f64::INFINITY };
            if miss > tolerance {
                if debug {
                    eprintln!("fold lens: trim misses its edge by {miss:.3e} (tolerance {tolerance:.1e}) at fraction {fraction:.3e}");
                }
                misses.push(k);
                break;
            }
        }
    }
    Ok(misses)
}

