use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// Piece of a support rim assigned to one mate face.
pub(super) struct RimPiece {
    pub(super) face_id: u64,
    pub(super) loop_index: usize,
    /// Global-parameter window of this piece.
    pub(super) window: [f64; 2],
    pub(super) edge_id: u64,
}

/// Blend a chain of conjugated edges with one rolling-ball blend face
/// (Golovanov §6.9.5: all conjugated edges processed together).  A CLOSED
/// chain (stadium rim, T-pipe saddle) welds into a periodic blend; an OPEN
/// chain (line->arc->line capped by end faces) rolls a clamped blend that
/// terminates in a transverse edge on each end face.
pub fn blend_smooth_chain(
    solid: &BrepSolid,
    seed_edge_id: u64,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    if !(radius > 0.0) || !radius.is_finite() {
        return Err(KernelRefusal::input(KernelStage::Collect, "radius", "blend: radius must be positive"));
    }
    let chain = collect_smooth_chain(solid, seed_edge_id)?;
    if chain.segments.len() < 2 {
        return Err(KernelRefusal::internal(KernelStage::Refine, "chain_single_segment", "blend: chain collapsed to a single segment"));
    }
    if chain.closed {
        blend_closed_smooth_chain(solid, &chain.segments, radius, chamfer, name)
    } else {
        blend_open_smooth_chain(solid, &chain, radius, chamfer, name)
    }
}

/// `blend_smooth_chain` restricted to CLOSED chains: an edge that is one arc
/// of a seam-split rim (a cyl×cyl saddle) is blended as the whole rim, but an
/// OPEN tangent chain is not walked — filleting one edge of it must not
/// fillet the unselected edges it runs into.  Tangent propagation is a
/// selection policy for the feature layer; the kernel caps an unselected
/// continuation instead (`network.rs`).
pub fn blend_smooth_chain_if_closed(
    solid: &BrepSolid,
    seed_edge_id: u64,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    if !(radius > 0.0) || !radius.is_finite() {
        return Err(KernelRefusal::input(KernelStage::Collect, "radius", "blend: radius must be positive"));
    }
    let chain = collect_smooth_chain(solid, seed_edge_id)?;
    if chain.segments.len() < 2 {
        return Err(KernelRefusal::internal(KernelStage::Refine, "chain_single_segment", "blend: chain collapsed to a single segment"));
    }
    if !chain.closed {
        return Err(
            KernelRefusal::unsupported(KernelStage::Classify, "open_tangent_chain", "blend: the edge is one arc of an OPEN tangent chain; the unselected \
             continuation is capped, not blended"),
        );
    }
    blend_closed_smooth_chain(solid, &chain.segments, radius, chamfer, name)
}

/// Blend a CLOSED chain of conjugated edges with one rolling-ball blend
/// face (Golovanov §6.9.5: all conjugated edges processed together).
///
/// A wall whose ball-centre curve turns tighter than the ball FOLDS, and the
/// envelope a plain march sweeps is then not the blend (`blend/fold.rs`).  It
/// is not refused here any more where the fold is a lens the wall closes on
/// itself: the chain is re-marched up a measured ladder of budgets and the lens
/// is carved out along the crease (`blend/carve.rs`).  The re-march is the
/// whole of the extra cost and only a folding wall pays it — the first march is
/// the ordinary one, and its refusal is what asks for the second.
fn blend_closed_smooth_chain(
    solid: &BrepSolid,
    segments: &[ChainSegment<'_>],
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let bar = crate::KernelTolerances::for_solid(solid, 1e-7).intersection_fit;
    // The plain collar climbs the same measured ladder the carve does, from the
    // same first rung: a budget picked once is a resolution request in disguise.
    let mut per_segment = CHAIN_PER_SEGMENT;
    let first = loop {
        let rung = build_closed_smooth_chain(
            solid,
            segments,
            radius,
            chamfer,
            name,
            per_segment,
            FoldPolicy::Refuse,
            bar,
        );
        match rung {
            Ok(Rung::TooCoarse { deviation }) => {
                crate::blend::carve::carve_trace(format_args!(
                    "blend chain: {per_segment} stations per segment leaves the rails \
                     {deviation:.6e} from the rolling ball, against {bar:.3e}"
                ));
                if per_segment * 2 > CHAIN_CARVE_MAX_PER_SEGMENT {
                    return Err(KernelRefusal::non_convergence(KernelStage::Refine, "chain_ladder_top", format!(
                        "blend: at {per_segment} stations per segment — the top of the chain's \
                         ladder — this collar's fitted rails are still {deviation:.6e} from the \
                         rolling ball's own contacts against a bar of {bar:.3e}, so there is no \
                         wall accurate enough to build"
                    )));
                }
                per_segment *= 2;
            }
            other => break other,
        }
    };
    if let Err(error) = &first {
        crate::blend::carve::carve_trace(format_args!(
            "blend chain: the march at {per_segment} stations per segment refused: {error}"
        ));
    }
    match first {
        Ok(Rung::Built(built)) => Ok(built),
        Ok(Rung::TooCoarse { .. }) => {
            Err(KernelRefusal::internal(KernelStage::Refine, "chain_ladder_rung", "blend: the chain ladder left an accuracy rung unhandled"))
        }
        // A fold that reaches a RAIL is not a lens: every section of that wall
        // is singular somewhere between its contacts, so there is nothing to cut
        // away and keep, and the ladder below would only re-march it to its top
        // before the carve refused the same band. Its refusal names the face it
        // cannot cross, and it is terminal as it stands (`blend/fold.rs`).
        Err(error) if crate::blend::fold::is_rail_fold(&error) => Err(error),
        // A CHAMFER section is a straight line and has no ball centre to read
        // the fold locus against, so its fold stays the refusal it was.
        Err(error) if crate::blend::is_wall_fold(&error) && !chamfer => {
            carve_ladder(solid, segments, radius, name, bar)
                // A CARVE THAT FAILS IS STILL A FOLD, and the fold is terminal.
                // If the carve's own refusal did not read as one, the ladder
                // below would answer a proven-folding centre curve with the
                // cutter's unrelated complaint — which is the exact failure
                // `fold.rs` was made terminal to stop. So every exit from this
                // branch carries the fold's own prefix, with the carve's reason
                // inside it.
                .map_err(|carve_error| {
                    if crate::blend::is_wall_fold(&carve_error) {
                        carve_error
                    } else {
                        crate::blend::fold::wall_fold_refusal(
                            Vec::new(),
                            format!(
                                "{} {} fits this edge: the wall folds, and the fold band could not \
                                 be carved — {carve_error}",
                                crate::blend::WALL_FOLDS,
                                radius.abs()
                            ),
                        )
                    }
                })
        }
        Err(error) => Err(error),
    }
}

/// Re-march a folding chain up a ladder of station budgets and carve at the
/// first rung whose wall IS the rolling ball's to `bar`.
///
/// The budget is not chosen: a larger or a tighter fillet needs a different
/// rung, and a number picked once from one fixture would be a resolution
/// request in disguise. Each rung is measured by the rolling ball itself — its
/// contacts re-solved halfway between stations, where an interpolant is worst —
/// against the fitted rails, and the ladder stops at the first rung inside
/// `intersection_fit`. A wall that still misses at the top is refused by name.
fn carve_ladder(
    solid: &BrepSolid,
    segments: &[ChainSegment<'_>],
    radius: f64,
    name: Option<&str>,
    bar: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let mut per_segment = CHAIN_PER_SEGMENT;
    loop {
        match build_closed_smooth_chain(
            solid,
            segments,
            radius,
            false,
            name,
            per_segment,
            FoldPolicy::Carve,
            bar,
        )? {
            Rung::Built(built) => {
                crate::blend::carve::carve_trace(format_args!(
                    "blend carve: {per_segment} stations per segment is the first rung inside \
                     {bar:.3e}"
                ));
                return Ok(built);
            }
            Rung::TooCoarse { deviation } => {
                crate::blend::carve::carve_trace(format_args!(
                    "blend carve: {per_segment} stations per segment leaves the rails \
                     {deviation:.6e} from the rolling ball, against {bar:.3e}"
                ));
                if per_segment * 2 > CHAIN_CARVE_MAX_PER_SEGMENT {
                    return Err(crate::blend::fold::wall_fold_refusal(
                        Vec::new(),
                        format!(
                            "{} {} fits this edge: the wall folds, and at {per_segment} stations \
                             per segment — the top of the carve's ladder — its fitted rails are \
                             still {deviation:.6e} from the rolling ball's own contacts against a \
                             bar of {bar:.3e}, so there is no wall accurate enough to carve",
                            crate::blend::WALL_FOLDS,
                            radius.abs()
                        ),
                    ));
                }
                per_segment *= 2;
            }
        }
    }
}

/// One rung of a chain build: the finished solid, or — for a carve — the
/// measured reason this budget is not enough.
enum Rung {
    Built(BrepSolid),
    TooCoarse { deviation: f64 },
}

/// The worst distance from the rolling ball's halfway contacts to the fitted
/// rails `cr` and `cs`, each found by a closest-point Newton from the
/// parameter the chord map assigns the midpoint.
///
/// THE NEWTON STAYS WHERE IT WAS SEEDED. The foot of a halfway contact is
/// inside the station interval it was solved in, so every step is bounded by
/// that interval's own length in the fit's parameter, clamped to the rail's
/// domain, and taken only when it brings the rail closer. A plain Newton stops
/// wherever the distance is stationary along the curve, which a rail that
/// turns sharply at a chain junction offers well away from the foot. Measured
/// on a 20 cube rounded at r = 3 with a bottom edge at r = 4 (the wall folds at
/// the arcs' rails, so this is the ladder's reading with that refusal set
/// aside): unguarded, every rung read the distance from the halfway contact
/// beside a junction to the junction itself — 2.927709e-1 at 24 stations per
/// segment against a half interval of 0.291667, halving per rung to
/// 9.128591e-3 at 768 — and on two of the eight mirror-image edges a foot
/// across the chain, 1.299089e1. Guarded, all eight read 6.961233e-3 at 24
/// and 6.920006e-6 to 6.920008e-6 at 768, falling fourfold per rung as a
/// cubic's does. A far foot can read short as well as long, which would pass
/// a rung that misses.
fn rail_deviation(
    samples: &[ChainSample],
    cr: &NurbsCurve,
    cs: &NurbsCurve,
    fit_low: f64,
    fit_range: f64,
) -> Result<f64, KernelRefusal> {
    let parameter_of = |segment: usize, position: isize| -> Option<f64> {
        samples
            .iter()
            .find(|sample| sample.segment == segment && sample.position == position)
            .map(|sample| sample.parameter)
    };
    let closest = |curve: &NurbsCurve, target: Vec3, seed: f64, reach: f64| -> Result<f64, KernelRefusal> {
        let [low, high] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let distance = |u: f64| -> Result<f64, KernelRefusal> {
            Ok(curve.derivatives_extended(u, 0).or_refuse(KernelStage::Refine, "derivatives_extended")?[0].sub(target).length())
        };
        let mut u = seed.clamp(low, high);
        let mut best = distance(u)?;
        for _ in 0..40 {
            let derivatives = curve.derivatives_extended(u, 2).or_refuse(KernelStage::Refine, "derivatives_extended")?;
            let offset = derivatives[0].sub(target);
            let slope = offset.dot(derivatives[1]);
            let curvature = derivatives[1].dot(derivatives[1]) + offset.dot(derivatives[2]);
            // Off a minimum's basin the Newton step points the wrong way; walk
            // downhill by the bound instead and let the test below shorten it.
            let mut step = if curvature > 0.0 {
                (slope / curvature).clamp(-reach, reach)
            } else {
                reach.copysign(slope)
            };
            let mut trial = (u - step).clamp(low, high);
            let mut trial_distance = distance(trial)?;
            for _ in 0..30 {
                if trial_distance < best {
                    break;
                }
                step *= 0.5;
                trial = (u - step).clamp(low, high);
                trial_distance = distance(trial)?;
            }
            if !(trial_distance < best) {
                break;
            }
            let moved = (trial - u).abs();
            u = trial;
            best = trial_distance;
            if moved < 1e-15 {
                break;
            }
        }
        Ok(best)
    };
    let mut worst: f64 = 0.0;
    // Each segment's worst halfway contact, for `BREP_BLEND_CARVE_TRACE=1`: where
    // along the chain a rung misses is what tells a junction from a bend.
    let mut per_segment: Vec<(f64, isize, f64, f64, Vec3, Vec3)> = Vec::new();
    for sample in samples {
        let Some([p1, p2]) = sample.midpoint else {
            continue;
        };
        let Some(next) = parameter_of(sample.segment, sample.position + 1) else {
            continue;
        };
        let global = (0.5 * (sample.parameter + next)).rem_euclid(1.0);
        let seed = (global - fit_low) / fit_range;
        let reach = (next - sample.parameter).abs() / fit_range;
        let (miss1, miss2) = (closest(cr, p1, seed, reach)?, closest(cs, p2, seed, reach)?);
        worst = worst.max(miss1).max(miss2);
        if per_segment.len() <= sample.segment {
            per_segment.resize(
                sample.segment + 1,
                (0.0, 0, 0.0, 0.0, Vec3::default(), Vec3::default()),
            );
        }
        if miss1.max(miss2) >= per_segment[sample.segment].0 {
            per_segment[sample.segment] = (miss1.max(miss2), sample.position, miss1, miss2, p1, p2);
        }
    }
    for (segment, (miss, position, miss1, miss2, p1, p2)) in per_segment.iter().enumerate() {
        crate::blend::carve::carve_trace(format_args!(
            "  rail miss segment {segment}: worst {miss:.6e} after station {position} \
             (cr {miss1:.6e} at ({:.6}, {:.6}, {:.6}), cs {miss2:.6e} at ({:.6}, {:.6}, {:.6}))",
            p1.x, p1.y, p1.z, p2.x, p2.y, p2.z
        ));
    }
    Ok(worst)
}

fn build_closed_smooth_chain(
    solid: &BrepSolid,
    segments: &[ChainSegment<'_>],
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
    per_segment: usize,
    fold: FoldPolicy,
    bar: f64,
) -> Result<Rung, KernelRefusal> {
    let samples = march_chain(segments, radius, false, per_segment, fold)?;

    // Global rows from the in-segment samples (wrapped-overlap closed fit).
    let mut global: Vec<(f64, &ChainSample)> = samples
        .iter()
        .filter(|sample| (0..per_segment as isize).contains(&sample.position))
        .map(|sample| (sample.parameter.rem_euclid(1.0), sample))
        .collect();
    global.sort_by(|a, b| a.0.total_cmp(&b.0));
    let count = global.len();
    let degree = FIT_DEGREE;
    let mut extended_params = Vec::new();
    let mut samples_cr = Vec::new();
    let mut samples_mid = Vec::new();
    let mut samples_cs = Vec::new();
    let mut push = |station: &Station, parameter: f64| {
        extended_params.push(parameter);
        samples_cr.push(Vec4::from_point(station.p1, 1.0));
        samples_cs.push(Vec4::from_point(station.p2, 1.0));
        samples_mid.push(Vec4 {
            x: station.apex.x * station.weight,
            y: station.apex.y * station.weight,
            z: station.apex.z * station.weight,
            w: station.weight,
        });
    };
    for offset in (1..=degree).rev() {
        let (parameter, sample) = &global[count - offset];
        push(&sample.station, parameter - 1.0);
    }
    for (parameter, sample) in &global {
        push(&sample.station, *parameter);
    }
    // Close the loop: repeat the first station at parameter 1, then the
    // wrap continuation.
    push(&global[0].1.station, global[0].0 + 1.0);
    for offset in 1..=degree {
        let (parameter, sample) = &global[offset];
        push(&sample.station, parameter + 1.0);
    }
    let low = extended_params[0];
    let high = *extended_params.last().unwrap();
    let range = high - low;
    let normalized: Vec<f64> = extended_params
        .iter()
        .map(|parameter| (parameter - low) / range)
        .collect();
    let seam_low = (0.0 - low) / range;
    let seam_high = (1.0 - low) / range;
    let fit_row = |row: &[Vec4]| -> Result<NurbsCurve, KernelRefusal> {
        let curve = fit::interpolate_homogeneous(row, degree, &normalized).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
        let (_, tail) = curve.split(seam_low).or_refuse(KernelStage::Refine, "split")?;
        let (middle, _) = tail.split(seam_high).or_refuse(KernelStage::Refine, "split")?;
        Ok(middle)
    };
    let cr = fit_row(&samples_cr)?;
    let cs = fit_row(&samples_cs)?;
    let mid = if chamfer {
        None
    } else {
        Some(fit_row(&samples_mid)?)
    };
    let u_domain = cr.domain().or_refuse(KernelStage::Refine, "domain")?;
    let surface = crate::blend::rows::surface_from_rows(degree, &cr, &cs, mid.as_ref(), true)?;

    // The CARVE.  A wall that folds carries a lens of parameter inside the
    // swept ball volume; the blend is that wall with the lens cut away along
    // the crease where the two sheets meet, and the crease becomes an edge of
    // the blend with two coedges on this one face.  Traced on the surface that
    // was just fitted — not on the ideal envelope — so the two kept sheets
    // meet EXACTLY on the curve the trim is built from (`blend/carve.rs`).
    // EVERY rung is measured before anything is built on it, carve or not: the
    // rails are interpolants through the stations, and between stations they
    // can leave the rolling ball's own contacts — and so the carriers they are
    // supposed to ride — by far more than the kernel's intersection-fit
    // contract. A carve pays for surgery only on an accurate rung; a plain
    // collar pays nothing more than the measurement unless it misses.
    let deviation = rail_deviation(&samples, &cr, &cs, low, range)?;
    if deviation > bar {
        return Ok(Rung::TooCoarse { deviation });
    }
    crate::blend::carve::carve_trace(format_args!(
        "blend chain: {per_segment} stations per segment leaves the rails {deviation:.6e} \
         from the rolling ball, inside {bar:.3e}"
    ));
    let crease = if fold == FoldPolicy::Carve {
        // THE BAR IS THE KERNEL'S OWN CONTRACT FOR THE QUANTITY MEASURED. What
        // the fit is held to is "does this coedge's curve-on-surface, pushed
        // back to 3-D, still trace the same locus as the edge's 3-D curve?",
        // which is `pcurve_consistency` — not `intersection_fit`, which is the
        // accuracy of a fitted intersection CURVE and is what the re-marched
        // wall itself is held to (`carve_ladder`). Measured on the
        // 2026-09-02 collar the crease's pcurves reach 5.4e-6 against that
        // 4e-3, so the margin is three decades and the bar is not what decides
        // the case either way.
        let bar = crate::KernelTolerances::for_solid(solid, 1e-7).pcurve_consistency;
        let Some(carved) = crate::blend::carve::trace_wall_crease(&surface, radius.abs())? else {
            return Err(KernelRefusal::internal(KernelStage::Refine, "carve_no_fold", format!(
                "blend carve: the wall re-marched at {per_segment} stations per segment carries                  no fold to carve, but the march at {CHAIN_PER_SEGMENT} refused one"
            )));
        };
        Some(crate::blend::carve::fit_crease(&surface, &carved, bar)?)
    } else {
        None
    };

    // Single-face sides (e.g. one cap around the whole rim) get ONE uv
    // fit in the rows' own normalized space — identical parameterization
    // to cr/cs, so the closed support piece and its pcurve agree exactly.
    let single_face = |side: usize| -> bool {
        let first_id = segments[0].mate(side).face.id;
        segments
            .iter()
            .all(|segment| segment.mate(side).face.id == first_id)
    };
    let mut whole_pcurves: [Option<NurbsCurve>; 2] = [None, None];
    for side in 0..2 {
        if !single_face(side) {
            continue;
        }
        let mut row = Vec::new();
        let select = |station: &Station| -> [f64; 2] {
            if side == 0 {
                station.uv1
            } else {
                station.uv2
            }
        };
        let mut push_uv = |station: &Station| {
            let uv = select(station);
            row.push(Vec4::from_point(Vec3::new(uv[0], uv[1], 0.0), 1.0));
        };
        for offset in (1..=degree).rev() {
            push_uv(&global[count - offset].1.station);
        }
        for (_, sample) in &global {
            push_uv(&sample.station);
        }
        push_uv(&global[0].1.station);
        for offset in 1..=degree {
            push_uv(&global[offset].1.station);
        }
        whole_pcurves[side] = Some(fit_row(&row)?);
    }

    // Per-face pcurve fits over each segment's full sample window
    // (in-segment + overshoot), in global parameters.
    let mut pcurves1 = Vec::with_capacity(segments.len());
    let mut pcurves2 = Vec::with_capacity(segments.len());
    for segment in 0..segments.len() {
        let mut window: Vec<&ChainSample> = samples
            .iter()
            .filter(|sample| sample.segment == segment)
            .collect();
        window.sort_by(|a, b| a.parameter.total_cmp(&b.parameter));
        let params: Vec<f64> = window.iter().map(|sample| sample.parameter).collect();
        let low = params[0];
        let high = *params.last().unwrap();
        let normalized: Vec<f64> = params
            .iter()
            .map(|parameter| (parameter - low) / (high - low))
            .collect();
        let fit_uv = |select: &dyn Fn(&Station) -> [f64; 2]| -> Result<NurbsCurve, KernelRefusal> {
            let points: Vec<Vec4> = window
                .iter()
                .map(|sample| {
                    let uv = select(&sample.station);
                    Vec4::from_point(Vec3::new(uv[0], uv[1], 0.0), 1.0)
                })
                .collect();
            let curve = fit::interpolate_homogeneous(&points, degree, &normalized).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
            // Re-express in GLOBAL parameters: the curve's [0,1] domain
            // corresponds to [low, high]; keep as-is and remember the
            // affine map through the window bounds.
            Ok(curve)
        };
        pcurves1.push((low, high, fit_uv(&|station| station.uv1)?));
        pcurves2.push((low, high, fit_uv(&|station| station.uv2)?));
    }

    Ok(Rung::Built(chain_surgery(
        solid,
        segments,
        ChainRows {
            surface,
            cr,
            cs,
            u_domain,
            fit_low: low,
            fit_range: range,
            pcurves1,
            pcurves2,
            whole_pcurves,
            crease,
        },
        name,
    )?))
}

pub(super) struct ChainRows {
    pub(super) surface: NurbsSurface,
    pub(super) cr: NurbsCurve,
    pub(super) cs: NurbsCurve,
    pub(super) u_domain: [f64; 2],
    /// Affine map from the rows' fit space to GLOBAL chain parameters:
    /// global = fit_low + p * fit_range (the global fit normalised its
    /// wrap-extended parameters onto [0, 1] before interpolating).
    pub(super) fit_low: f64,
    pub(super) fit_range: f64,
    /// Per-segment (window_low, window_high, uv curve over [0,1]) in
    /// GLOBAL parameters.
    pub(super) pcurves1: Vec<(f64, f64, NurbsCurve)>,
    pub(super) pcurves2: Vec<(f64, f64, NurbsCurve)>,
    /// Whole-chain uv fits (rows' fit space) for single-face sides.
    pub(super) whole_pcurves: [Option<NurbsCurve>; 2],
    /// The crease a CARVED wall is cut along: the inner loop of the blend
    /// face, one edge carrying two coedges of that same face.
    pub(super) crease: Option<crate::blend::carve::CarvedCrease>,
}

impl ChainRows {
    pub(super) fn to_global(&self, fit_parameter: f64) -> f64 {
        self.fit_low + fit_parameter * self.fit_range
    }
}

/// Extract the pcurve portion for a global window [a, b] from a
/// segment's fitted uv curve, re-parameterised so its domain maps
/// affinely onto the portion (matching the split support edge).
pub(super) fn pcurve_portion(fitted: &(f64, f64, NurbsCurve), a: f64, b: f64) -> Result<NurbsCurve, KernelRefusal> {
    let (low, high, curve) = fitted;
    // The chain parameterisation wraps with period 1; pick the period
    // copy of the window that overlaps this segment's fit range.
    let to_local = |value: f64| {
        let mut best = value;
        for candidate in [value, value + 1.0, value - 1.0] {
            if candidate >= low - 1e-9 && candidate <= high + 1e-9 {
                best = candidate;
                break;
            }
        }
        ((best - low) / (high - low)).clamp(0.0, 1.0)
    };
    let mut local_a = to_local(a);
    let mut local_b = to_local(b);
    if local_a > local_b {
        std::mem::swap(&mut local_a, &mut local_b);
    }
    let epsilon = 1e-9;
    let (_, tail) = if local_a > epsilon {
        curve.split(local_a).or_refuse(KernelStage::Refine, "split")?
    } else {
        (curve.clone(), curve.clone())
    };
    let tail = if local_a > epsilon {
        tail
    } else {
        curve.clone()
    };
    let portion = if local_b < 1.0 - epsilon {
        tail.split(local_b).or_refuse(KernelStage::Refine, "split")?.0
    } else {
        tail
    };
    Ok(portion)
}

/// The single boundary edge of a side's mate face(s) incident to a chain
/// vertex, EXCLUDING the two chain edges meeting there — i.e. the SEAM
/// (same face on both segments) or the SPOKE (face changes) that the
/// support curve must cross.  `None` when the chain simply flows through
/// (same face, no seam — e.g. a stadium cap rim vertex).
pub(super) fn cross_edge_at(
    solid: &BrepSolid,
    face_before: &FaceRecord,
    loop_before: usize,
    face_after: &FaceRecord,
    loop_after: usize,
    vertex: u64,
    before_edge: u64,
    after_edge: u64,
) -> Result<Option<u64>, KernelRefusal> {
    let mut found: Option<u64> = None;
    let mut consider = |face: &FaceRecord, loop_index: usize| -> Result<(), KernelRefusal> {
        for coedge in &face.loops[loop_index].coedges {
            if coedge.edge_id == before_edge || coedge.edge_id == after_edge {
                continue;
            }
            let edge = solid
                .edges
                .iter()
                .find(|edge| edge.id == coedge.edge_id)
                .ok_or(KernelRefusal::internal(KernelStage::Refine, "missing_edge", "blend: loop references missing edge"))?;
            if edge.start_vertex_id == vertex || edge.end_vertex_id == vertex {
                if found.is_some() && found != Some(edge.id) {
                    return Err(KernelRefusal::unsupported(KernelStage::Classify, "chain_cross_edges", "blend: multiple cross edges at a chain vertex"));
                }
                found = Some(edge.id);
            }
        }
        Ok(())
    };
    consider(face_before, loop_before)?;
    if !(face_after.id == face_before.id && loop_after == loop_before) {
        consider(face_after, loop_after)?;
    }
    Ok(found)
}

/// The nearest periodic image of a seam boundary to `value` — the boundary a
/// crossing endpoint must be pinned to, read off its ADJACENT INTERIOR sample.
fn nearest_seam_image(value: f64, low: f64, high: f64) -> f64 {
    let period = high - low;
    low + ((value - low) / period).round() * period
}

/// The mate-face pcurve of a support PIECE, built by projecting the piece's
/// 3D curve onto the (analytic) mate surface and fitting.  Because a chain
/// support rides EXACTLY on its mate carrier, the projection is exact; the
/// periodic track is unwrapped so it never jumps a period across the carrier
/// seam, and crossing endpoints are pinned ONTO the seam so the piece and the
/// trimmed seam edge close the loop in parameter space.
///
/// The carrier is closed in EITHER direction, and the crossed seam is not
/// always the u meridian.  A cylinder/cone/sphere is closed in u only, so its
/// seam is the u0 ≡ u1 meridian — but a TORUS is biperiodic, and a tube welded
/// through a plane crosses the torus' v0 ≡ v1 PARALLEL (its outer equator)
/// instead.  Pinning u there drags the endpoint a half-carrier away from its
/// own 3D vertex (the collar loop then closes on a uv corner that is nowhere
/// near the vertex, and the mate face's trim is garbage), so the pinned AXIS is
/// chosen GEOMETRICALLY: whichever closed direction's seam actually passes
/// through the endpoint.  A carrier closed in u only can only ever take the u
/// branch, exactly as before.
pub(super) fn project_piece_pcurve(
    piece: &NurbsCurve,
    surface: &NurbsSurface,
    start_is_crossing: bool,
    end_is_crossing: bool,
) -> Result<NurbsCurve, KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let (_, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let period = u1 - u0;
    let period_v = v1 - v0;
    let [d0, d1] = piece.domain().or_refuse(KernelStage::Refine, "domain")?;
    // Dense reference projection; the pcurve is fit from an adaptively
    // thinned subset kept as coarse as tolerance allows.
    //
    // The track is as dense as the PIECE is, not a fixed 96. A carved wall's
    // re-marched rail carries hundreds of control points (574 on the
    // 2026-09-02 two-torus collar), and a pcurve fitted and checked at 97
    // samples of it was accepted 1.19e-3 off the rail it claims.
    let dense = (8 * piece.control_points.len()).clamp(96, 4096);
    let mut proj_u = Vec::with_capacity(dense + 1);
    let mut proj_v = Vec::with_capacity(dense + 1);
    let mut point3 = Vec::with_capacity(dense + 1);
    let mut params = Vec::with_capacity(dense + 1);
    let mut previous_u: Option<f64> = None;
    // The foot each sample continues from: the track is ONE branch of the
    // carrier, so after the first sample each inversion starts from its
    // neighbour's foot rather than asking for the nearest point of the whole
    // surface, which on a carrier that comes back near itself is another sheet.
    let mut previous_foot: Option<[f64; 2]> = None;
    let mut feet: Vec<[f64; 2]> = Vec::with_capacity(dense + 1);
    for section in 0..=dense {
        let t = d0 + (d1 - d0) * section as f64 / dense as f64;
        let point = piece.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
        let projection = match previous_foot {
            None => crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?,
            Some(seed) => crate::projection::project_point_to_surface_from_seed(surface, point, seed).or_refuse(KernelStage::Refine, "project_point_to_surface_from_seed")?,
        };
        previous_foot = Some([projection.u, projection.v]);
        feet.push([projection.u, projection.v]);
        let mut u = projection.u;
        if let Some(previous) = previous_u {
            while u - previous > period * 0.5 {
                u -= period;
            }
            while previous - u > period * 0.5 {
                u += period;
            }
        }
        previous_u = Some(u);
        proj_u.push(u);
        proj_v.push(projection.v);
        point3.push(point);
        params.push(section as f64 / dense as f64);
    }
    // A biperiodic carrier needs the SAME unwrap in v, or a track that walks up
    // to the v seam comes back as 0 at the far end and the fit swings across
    // the whole carrier.  Anchor the chain on the first INTERIOR sample and
    // pull index 0 onto it afterwards: an endpoint sitting exactly ON the seam
    // inverts to either boundary at the solver's whim, and letting that
    // coin flip anchor the chain would shift the whole piece a period.
    if closed_v {
        for index in 2..=dense {
            while proj_v[index] - proj_v[index - 1] > period_v * 0.5 {
                proj_v[index] -= period_v;
            }
            while proj_v[index - 1] - proj_v[index] > period_v * 0.5 {
                proj_v[index] += period_v;
            }
        }
        while proj_v[0] - proj_v[1] > period_v * 0.5 {
            proj_v[0] -= period_v;
        }
        while proj_v[1] - proj_v[0] > period_v * 0.5 {
            proj_v[0] += period_v;
        }
    }
    // Interior mean decides which meridian a crossing endpoint sits on, and
    // whether the whole track needs a full-period shift into the domain.
    let mean_u: f64 = proj_u[1..dense].iter().sum::<f64>() / (dense - 1) as f64;
    let seam_u = if mean_u - u0 > u1 - mean_u { u1 } else { u0 };
    // Pin a crossing endpoint onto the seam it actually crosses.  Each
    // candidate keeps the OTHER coordinate as projected, so the losing axis
    // moves the endpoint bodily off its own 3D vertex — the residual names the
    // winner with no surface-type special-casing.  u keeps the whole-track
    // mean; a v run can legitimately cover a full period, so its boundary is
    // read off the ADJACENT INTERIOR sample instead.  Both pins land in the
    // track's own (pre-shift) period, so the shift below carries them along
    // with the rest of the track.
    let pinned = |index: usize,
                  neighbour: usize,
                  proj_u: &[f64],
                  proj_v: &[f64]|
     -> Result<(f64, f64), KernelRefusal> {
        let point = point3[index];
        let u_error = surface
            .evaluate(seam_u, proj_v[index]).or_refuse(KernelStage::Refine, "evaluate")?
            .sub(point)
            .length();
        if closed_v {
            let seam_v = nearest_seam_image(proj_v[neighbour], v0, v1);
            let v_error = surface
                .evaluate(proj_u[index], seam_v).or_refuse(KernelStage::Refine, "evaluate")?
                .sub(point)
                .length();
            if v_error < u_error {
                return Ok((proj_u[index], seam_v));
            }
        }
        Ok((seam_u, proj_v[index]))
    };
    if start_is_crossing {
        let (u, v) = pinned(0, 1, &proj_u, &proj_v)?;
        proj_u[0] = u;
        proj_v[0] = v;
    }
    if end_is_crossing {
        let (u, v) = pinned(dense, dense - 1, &proj_u, &proj_v)?;
        proj_u[dense] = u;
        proj_v[dense] = v;
    }
    let mut shift = 0.0;
    while mean_u + shift > u1 + 1e-9 {
        shift -= period;
    }
    while mean_u + shift < u0 - 1e-9 {
        shift += period;
    }
    for u in proj_u.iter_mut() {
        *u += shift;
    }
    if closed_v {
        let mean_v: f64 = proj_v[1..dense].iter().sum::<f64>() / (dense - 1) as f64;
        let mut shift_v = 0.0;
        while mean_v + shift_v > v1 + 1e-9 {
            shift_v -= period_v;
        }
        while mean_v + shift_v < v0 - 1e-9 {
            shift_v += period_v;
        }
        for v in proj_v.iter_mut() {
            *v += shift_v;
        }
    }
    // Coarsest interpolation whose fitted pcurve reproduces the projected TRACK
    // to the kernel's pcurve refinement floor — the floor every other pcurve fit
    // is held to, and the one the shell vector-area bar is built from.
    //
    // It used to stop at 0.002 absolute ("half the validator's tolerance"),
    // which is `pcurve_consistency / 2` on any model under 20 units and so four
    // decades looser than any other trim: a torus face carrying a collar's rail
    // was accepted 1.19e-3 off it, and that one coedge was the whole of a
    // 1.94e-4 shell vector-area residual, 14.5x its face's bar.
    //
    // The deviation is measured against the projected track, not the piece's
    // 3D points: a pcurve lies on its surface by construction, so a piece that
    // is itself off the carrier would hold the ladder at its top however fine
    // the fit, and that is a RAIL defect for the march to answer, not this fit.
    let tolerance = crate::pcurve::PCURVE_REFINEMENT_TOLERANCE;
    // The ladder is the one verified fitter (`track_fit`), which checks every
    // rung at the MIDPOINTS between dense samples, projected on their own —
    // checked at the dense samples, the top rung, which interpolates every one
    // of them, reads zero by construction and passes whatever lies between (on
    // the 574-control-point collar rail it read 9.6e-15) — and breaks the fit,
    // C1, where the track crosses a knot line of the carrier.
    let at = |fraction: f64| piece.evaluate(d0 + (d1 - d0) * fraction);
    let mut track = crate::blend::track_fit::Track {
        fractions: params,
        uv: proj_u.iter().zip(&proj_v).map(|(u, v)| [*u, *v]).collect(),
        feet,
        breaks: Vec::new(),
    };
    let breaks = crate::blend::track_fit::curve_breaks(piece, d0, d1, true);
    crate::blend::track_fit::polish_track(surface, &at, &mut track)?;
    crate::blend::track_fit::insert_knot_crossings(surface, &at, &mut track, &breaks)?;
    let fit = crate::blend::track_fit::fit_track(surface, &track, &at, tolerance)?;
    if std::env::var("BREP_DEBUG_TRACK_FIT").is_ok() {
        eprintln!(
            "TRACK_FIT closed-chain piece: miss {:.3e} at {} samples of {} ({} breaks), on floor {}",
            fit.miss,
            fit.samples,
            track.fractions.len(),
            track.breaks.len(),
            fit.on_floor
        );
    }
    // A fit off the floor is refused, not handed on as the best rung. Before the
    // fitter broke at knot lines this ladder returned its best rung silently,
    // and on the 2026-09-02 two-torus collar that was 1.07e-7 to 8.46e-7 off at
    // the top rung on every collar that builds (measured 2026-09-17); broken,
    // each of them reaches the floor. What still misses is a rail the march tore
    // (2.7 to 3.3 on the oversized radii, a body refused or rejected anyway).
    if !fit.on_floor {
        return Err(KernelRefusal::non_convergence(KernelStage::Refine, "chain_pcurve_floor", format!(
            "{} a closed chain's support piece misses its pcurve by {:.3e} at {} samples, \
             against a floor of {tolerance:.1e}",
            crate::blend::PCURVE_OFF_FLOOR,
            fit.miss,
            fit.samples
        )));
    }
    Ok(fit.curve)
}
