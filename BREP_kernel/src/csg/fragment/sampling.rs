use crate::curve::interior_knot_count;
use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

pub(super) fn subcurve_by_fraction(
    curve: &NurbsCurve,
    start_fraction: f64,
    end_fraction: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    let first = start + (end - start) * start_fraction;
    let second = start + (end - start) * end_fraction;
    // NurbsCurve::split refuses parameters within its ABSOLUTE knot
    // tolerance (1e-9) of the domain ends; the skip epsilon must cover that
    // or a near-edge trim parameter slips past the guard and errors.
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut result = curve.clone();
    if first > start + epsilon && first < end - epsilon {
        result = result.split(first).or_refuse(KernelStage::Fragment, "split")?.1;
    }
    let domain = result.domain().or_refuse(KernelStage::Fragment, "domain")?;
    if second < domain[1] - epsilon && second > domain[0] + epsilon {
        result = result.split(second).or_refuse(KernelStage::Fragment, "split")?.0;
    } else if std::env::var("BREP_DEBUG_SUBRANGE").is_ok() && second < domain[1] - epsilon {
        eprintln!("abnormal subrange skip in fragment subcurve");
    }
    Ok(result)
}

pub(super) fn trimmed_curve(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<NurbsCurve, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    let span = end - start;
    if span <= 0.0 {
        return Err(KernelRefusal::internal(KernelStage::Fragment, "fragment.sampling", "fragment_face: invalid edge curve domain"));
    }
    subcurve_by_fraction(curve, (t0 - start) / span, (t1 - start) / span)
}

fn curve_is_rational(curve: &NurbsCurve) -> bool {
    curve
        .control_points
        .iter()
        .any(|control| (control.w - 1.0).abs() > 1e-12)
}


pub(super) fn sample_chain(curve: &NurbsCurve) -> Result<Vec<Vec2>, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    let straight = curve.degree == 1 && !curve_is_rational(curve);
    let spans = interior_knot_count(&curve.knots, curve.degree) + 1;
    let count = if straight {
        1
    } else {
        96usize.min(16usize.max(spans * 8))
    };
    (0..=count)
        .map(|index| {
            let point = curve.evaluate(start + (end - start) * index as f64 / count as f64).or_refuse(KernelStage::Fragment, "evaluate")?;
            Ok(Vec2 {
                x: point.x,
                y: point.y,
            })
        })
        .collect()
}

/// Deterministic midpoint-bisection refinement of a cut chain's uniform
/// sampling until every chord stays within `bound` of the true pcurve (the
/// arrangement tolerance — a polyline within it cannot invent crossings the
/// arrangement would distinguish). Probes each segment at the 1/4, 1/2 and
/// 3/4 parameters (the off-centre probes catch S-shaped folds whose midpoint
/// happens to sit on the chord) and inserts the segment midpoint while any
/// probe exceeds the bound. Purely additive sampling of the SAME curve —
/// no point is moved, merged or snapped — capped defensively (best-effort:
/// hitting the cap leaves the chain no worse than the unrefined sampling).
/// Escape hatch: BREP_CHAIN_SAG_REFINE=0 restores the raw uniform sampling.
/// Turning it off costs three known outcomes, not one: boolean `fixture_21`,
/// plus corpus rows `AnotherOffsetShellProblem2` and `BadBoolean` (both
/// `success` -> `refused` in the offset shell's watertight close), measured by
/// the 2026-09-16 coarseness census.
///
/// Returns the refined points plus a parallel flag vector marking which points
/// belong to the ORIGINAL sampling (`true`) versus refinement insertions
/// (`false`) — the boundary-crossing parity guard downstream needs to compare
/// each inserted run against its raw anchor chord.
pub(super) fn sag_refine_chain(
    curve: &NurbsCurve,
    points: Vec<Vec2>,
    bound: f64,
) -> Result<(Vec<Vec2>, Vec<bool>), KernelRefusal> {
    if points.len() < 2 || std::env::var("BREP_CHAIN_SAG_REFINE").as_deref() == Ok("0") {
        let flags = vec![true; points.len()];
        return Ok((points, flags));
    }
    // A chain whose whole sampling collapsed to ONE zero-length segment is a
    // COLLAPSED cut, and downstream machinery depends on that collapse as a
    // contract — the offset-shell pipeline deliberately leaves the fragile
    // carrier∩opening-plane graze collapsed (a closed degree-1 polyline
    // pcurve that `sample_chain`'s straight shortcut reduces to its
    // coincident endpoints) and sews the orphan rim EXACTLY afterwards
    // (`weld_coplanar_orphan_rims` + isoline upgrade). Refining it here
    // would resurrect the ring as a fitted-polyline trim and bypass that
    // exactness path. Fidelity refinement must never create topology the
    // unrefined sampling did not have, so leave collapsed chains alone.
    if points.len() == 2 && points[0].sub(points[1]).length() <= bound {
        let flags = vec![true; points.len()];
        return Ok((points, flags));
    }
    let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    let span = end - start;
    if !(span > 0.0) || !bound.is_finite() || bound <= 0.0 {
        let flags = vec![true; points.len()];
        return Ok((points, flags));
    }
    let segment_count = points.len() - 1;
    let mut refined: Vec<(f64, Vec2, bool)> = points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            (
                start + span * index as f64 / segment_count as f64,
                *point,
                true,
            )
        })
        .collect();
    const MAX_POINTS: usize = 4096;
    let minimum_step = (span * 1e-7).max(1e-12);
    let uv = |parameter: f64| -> Result<Vec2, KernelRefusal> {
        let point = curve.evaluate(parameter).or_refuse(KernelStage::Fragment, "evaluate")?;
        Ok(Vec2 {
            x: point.x,
            y: point.y,
        })
    };
    let mut index = 0;
    while index + 1 < refined.len() && refined.len() < MAX_POINTS {
        let (parameter_a, point_a, _) = refined[index];
        let (parameter_b, point_b, _) = refined[index + 1];
        if parameter_b - parameter_a <= minimum_step {
            index += 1;
            continue;
        }
        let mut sag = 0.0f64;
        for fraction in [0.25, 0.5, 0.75] {
            let probe = uv(parameter_a + (parameter_b - parameter_a) * fraction)?;
            sag = sag.max(point_segment_distance(probe, point_a, point_b));
        }
        if sag > bound {
            let middle = 0.5 * (parameter_a + parameter_b);
            refined.insert(index + 1, (middle, uv(middle)?, false));
            // Re-check the halved front segment before advancing.
        } else {
            index += 1;
        }
    }
    let flags = refined.iter().map(|(_, _, original)| *original).collect();
    Ok((refined.into_iter().map(|(_, point, _)| point).collect(), flags))
}

pub(super) fn point_segment_distance(point: Vec2, start: Vec2, end: Vec2) -> f64 {
    let direction = end.sub(start);
    let length_squared = direction.dot(direction);
    if length_squared <= 1e-30 {
        return point.sub(start).length();
    }
    let fraction = (point.sub(start).dot(direction) / length_squared).clamp(0.0, 1.0);
    point.sub(start.add(direction.scale(fraction))).length()
}

/// BOUNDARY-CROSSING PARITY GUARD for sag-refined cut chains (t164, the
/// sphere×tube rim graze). Boundary-crossing TOPOLOGY is owned by the imprint
/// layer: `process_curve` clips section pieces at the face boundary and mints
/// the junctions + edge splits for every crossing it adjudicates. Sag
/// refinement is a FIDELITY device — it may pull a chord off a boundary the
/// true pcurve clears (t70's phantom hole-corner junctions, crossings
/// REMOVED), but it must never ADD a boundary crossing the raw sampling did
/// not have: the refined polyline of a near-tangent graze (t164: the rim dips
/// 4.6e-7 mm into the sphere between two true crossings 0.105 mm apart, only
/// ONE of which the imprint minted) faithfully pokes past the trim row and
/// re-crosses it, and the arrangement then splits the boundary at a locally
/// minted position no other face ever hears about — the neighbouring cap
/// keeps the un-split imprint arc, and assembly strands both records plus the
/// poked micro-arc as one-use edges. Reverting exactly the offending runs to
/// their raw anchor chords restores the imprint-consistent sub-tolerance
/// collapse while keeping the refinement everywhere it stays inside.
/// Deterministic, purely structural (crossing parity per refined run), no
/// vertex is moved and no tolerance widened. Escape hatch:
/// BREP_SAG_CROSSING_GUARD=0.
pub(super) fn enforce_boundary_crossing_parity(
    points: Vec<Vec2>,
    original: &[bool],
    boundaries: &[(Vec2, Vec2)],
    tolerance: f64,
    debug: bool,
    face_id: u64,
) -> Vec<Vec2> {
    if boundaries.is_empty()
        || points.len() != original.len()
        || original.iter().all(|flag| *flag)
        || std::env::var("BREP_SAG_CROSSING_GUARD").as_deref() == Ok("0")
    {
        return points;
    }
    // A crossing within the junction slack of a run anchor is not a NEW
    // crossing — chain endpoints legitimately sit ON boundary junctions and
    // the arrangement cannot distinguish below its own slack anyway.
    let exclusion = tolerance * 50.0;
    let crossings = |polyline: &[Vec2], anchor_start: Vec2, anchor_end: Vec2| -> usize {
        let mut count = 0usize;
        for pair in polyline.windows(2) {
            let (p, q) = (pair[0], pair[1]);
            let low_x = p.x.min(q.x) - exclusion;
            let high_x = p.x.max(q.x) + exclusion;
            let low_y = p.y.min(q.y) - exclusion;
            let high_y = p.y.max(q.y) + exclusion;
            let direction = q.sub(p);
            for (a, b) in boundaries {
                if a.x.max(b.x) < low_x
                    || a.x.min(b.x) > high_x
                    || a.y.max(b.y) < low_y
                    || a.y.min(b.y) > high_y
                {
                    continue;
                }
                let boundary_direction = b.sub(*a);
                let denominator = direction.x * boundary_direction.y
                    - direction.y * boundary_direction.x;
                if denominator.abs() <= 1e-30 {
                    // Parallel/degenerate: never counted as a crossing (a
                    // coincident overlap is handled by the duplicate-chain
                    // drop downstream, and counting it here could only
                    // revert refinement — stay conservative).
                    continue;
                }
                let offset = a.sub(p);
                let t = (offset.x * boundary_direction.y - offset.y * boundary_direction.x)
                    / denominator;
                let s = (offset.x * direction.y - offset.y * direction.x) / denominator;
                if !(0.0..=1.0).contains(&t) || !(0.0..=1.0).contains(&s) {
                    continue;
                }
                let hit = p.add(direction.scale(t));
                if hit.sub(anchor_start).length() <= exclusion
                    || hit.sub(anchor_end).length() <= exclusion
                {
                    continue;
                }
                count += 1;
            }
        }
        count
    };
    let mut kept: Vec<Vec2> = Vec::with_capacity(points.len());
    let mut run_start = 0usize;
    kept.push(points[0]);
    for index in 1..points.len() {
        if !original[index] {
            continue;
        }
        // Run of inserted points between original anchors run_start..index.
        if index > run_start + 1 {
            let run = &points[run_start..=index];
            let anchor_start = points[run_start];
            let anchor_end = points[index];
            let chord = [anchor_start, anchor_end];
            let refined_crossings = crossings(run, anchor_start, anchor_end);
            let chord_crossings = crossings(&chord, anchor_start, anchor_end);
            if refined_crossings > 0 && chord_crossings == 0 {
                if debug {
                    eprintln!(
                        "  sag-guard: face {face_id} run {}..{} reverted to chord (refined polyline crosses boundary {refined_crossings}x, raw chord 0x)",
                        run_start, index
                    );
                }
                // Revert: drop the inserted points, keep the raw chord.
            } else {
                kept.extend(run[1..run.len() - 1].iter().copied());
            }
        }
        kept.push(points[index]);
        run_start = index;
    }
    kept
}

pub(super) fn refine_near_hints(
    points: &[Vec2],
    curve: &NurbsCurve,
    hints: &[Vec2],
) -> Result<Vec<Vec2>, KernelRefusal> {
    if hints.is_empty() || points.len() < 3 {
        return Ok(points.to_vec());
    }
    let [start, end] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    let segment_count = points.len() - 1;
    let mut result = vec![points[0]];
    for index in 0..segment_count {
        let a = points[index];
        let b = points[index + 1];
        let length = b.sub(a).length();
        if length > 0.0
            && hints
                .iter()
                .any(|hint| point_segment_distance(*hint, a, b) <= length)
        {
            let parameter_a = start + (end - start) * index as f64 / segment_count as f64;
            let parameter_b = start + (end - start) * (index + 1) as f64 / segment_count as f64;
            for sub in 1..16 {
                let point = curve
                    .evaluate(parameter_a + (parameter_b - parameter_a) * sub as f64 / 16.0).or_refuse(KernelStage::Fragment, "evaluate")?;
                result.push(Vec2 {
                    x: point.x,
                    y: point.y,
                });
            }
        }
        result.push(b);
    }
    Ok(result)
}

